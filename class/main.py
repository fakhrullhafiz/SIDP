#!/usr/bin/env python3
"""
main.py

Orchestrator for Option A pipeline:
- Creates shared memory for latest frame
- Starts camera process (writes frames into shared memory)
- Starts YOLO process (reads frames from shared memory, outputs detections)
- Starts ultrasonic thread (I/O-bound)
- Starts Firebase uploader thread (I/O-bound)
- Starts TTS thread (I/O-bound)
- Main display loop: reads frame from shared memory + draws detections and ultrasonic info
- Graceful shutdown and cleanup

Assumptions:
- camera_process.py defines CameraWorker (as generated earlier)
- yolo_process.py defines YoloWorker (as generated earlier)
- Both modules are importable in the same directory as main.py
- Replace firebase credential path and DB URL if needed (kept same as your original file for convenience).
"""

import logging
import signal
import sys
import time
import datetime
from zoneinfo import ZoneInfo
import threading
import queue
import traceback
import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
import cv2

# Import the camera & yolo worker classes generated earlier
from camera_process import CameraWorker
from yolo_process import YoloWorker

# === Firebase / TTS imports (copied/adapted from your original integrate.py) ===
import subprocess
import firebase_admin
from firebase_admin import credentials, db

# ------------------------
# Configuration / Defaults
# ------------------------
LOG = logging.getLogger("main")
SHM_NAME = "vision_shm_latest_frame_v1"
WIDTH = 640
HEIGHT = 480
CHANNELS = 3
FRAME_BYTES = WIDTH * HEIGHT * CHANNELS

YOLO_MODEL = "yolov8n.pt"   # change if needed
YOLO_IMGSZ = 320
YOLO_CONF = 0.35
ALLOWED_CLASSES = {
    'person', 'car', 'cat', 'dog', 'stop sign',
    'toilet', 'chair', 'bed', 'tv', 'dining table'
}

FPS_DISPLAY_POS = (10, 90)

# Timezone for firebase timestamps
MALAYSIA_TZ = ZoneInfo("Asia/Kuala_Lumpur")

# Firebase credentials (copied from your integrate.py) - change path in production
FIREBASE_CRED_PATH = "/home/coe/firebase/firebase-key.json"
FIREBASE_DB_URL = "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app/"

# ------------------------
# Global queues & threads
# ------------------------
# Multiprocessing queues (create in parent so children inherit correctly)
frame_meta_queue = mp.Queue(maxsize=4)   # camera -> yolo metadata (tiny dicts)
detection_queue = mp.Queue(maxsize=8)    # yolo -> main detection dicts

# Threading queues (I/O workers)
firebase_queue = queue.Queue(maxsize=200)
tts_queue = queue.PriorityQueue()

# Ultrasonic queue (thread -> main)
ultrasonic_queue = queue.Queue(maxsize=16)

# Thread stop events
_stop_event = threading.Event()
_stop_event.clear()  # when set, threads should stop; we'll set it on shutdown

# ------------------------
# FIREBASE WORKER (thread)
# ------------------------
def init_firebase():
    try:
        cred = credentials.Certificate(FIREBASE_CRED_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
        LOG.info("Firebase initialized")
        return True
    except Exception:
        LOG.exception("Failed to initialize Firebase (check credential path)")
        return False

ultrasonic_db = None
object_db = None

def firebase_worker():
    """Background uploader thread - non-blocking via firebase_queue (thread)."""
    global ultrasonic_db, object_db
    while not _stop_event.is_set():
        try:
            item = firebase_queue.get(timeout=1)
            if item is None:
                break
            upload_type, data = item
            try:
                if upload_type == "ultrasonic" and ultrasonic_db is not None:
                    ultrasonic_db.push(data)
                elif upload_type == "objects" and object_db is not None:
                    object_db.push(data)
            except Exception:
                LOG.exception("Firebase upload error for %s", upload_type)
            finally:
                firebase_queue.task_done()
        except queue.Empty:
            continue
        except Exception:
            LOG.exception("Firebase worker unexpected error")
            continue
    LOG.info("Firebase worker exiting")


def upload_to_firebase(upload_type, data):
    try:
        firebase_queue.put_nowait((upload_type, data))
    except queue.Full:
        LOG.warning("Firebase queue full; dropping upload")


# ------------------------
# TTS WORKER (thread)
# ------------------------
use_pyttsx3 = False
engine = None

try:
    import pyttsx3
    engine = pyttsx3.init()
    engine.setProperty("rate", 150)
    engine.setProperty("volume", 1.0)
    use_pyttsx3 = True
    LOG.info("Using pyttsx3 for TTS")
except Exception:
    use_pyttsx3 = False
    LOG.info("pyttsx3 not available, using espeak via subprocess for TTS")

def tts_worker():
    while not _stop_event.is_set():
        try:
            priority, text = tts_queue.get(timeout=1)
            if text is None:
                tts_queue.task_done()
                break
            try:
                if use_pyttsx3 and engine is not None:
                    engine.say(text)
                    engine.runAndWait()
                else:
                    subprocess.run(["espeak", str(text)], check=False)
            except Exception:
                LOG.exception("TTS error")
            finally:
                tts_queue.task_done()
        except queue.Empty:
            continue
        except Exception:
            LOG.exception("tts_worker unexpected error")
            continue
    LOG.info("TTS worker exiting")


def speak(text, priority=5):
    try:
        tts_queue.put_nowait((priority, text))
    except queue.Full:
        LOG.warning("TTS queue full; dropping message")


# ------------------------
# ULTRASONIC THREAD (simple port from your original code)
# ------------------------
import RPi.GPIO as GPIO

TRIG = 11
ECHO = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)

ultrasonic_state = {
    "distance": 0.0,
    "message": "Initializing...",
    "color": (255, 255, 255),
    "last_update": time.time()
}
ultrasonic_lock = threading.Lock()

def measure_distance_hcsr04():
    try:
        # pulse trigger
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_start = time.time()
        pulse_end = time.time()
        timeout = time.time() + 0.1

        # wait for echo start
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None

        # wait for echo end
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return round(distance, 2)
    except Exception:
        LOG.exception("Ultrasonic measurement error")
        return None

def ultrasonic_thread_fn():
    last_message = None
    last_announce_time = 0
    announce_interval = 3.0

    while not _stop_event.is_set():
        d = measure_distance_hcsr04()
        current_time = time.time()
        if d is None:
            with ultrasonic_lock:
                ultrasonic_state["distance"] = 0.0
                ultrasonic_state["message"] = "Sensor Error"
                ultrasonic_state["color"] = (0, 0, 255)
            time.sleep(0.2)
            continue

        if d < 50:
            message = "Stop"
            color = (0, 0, 255)
            priority = 1
        elif d < 100:
            message = "Warning"
            color = (0, 165, 255)
            priority = 2
        elif d < 200:
            message = "Caution"
            color = (0, 255, 255)
            priority = 3
        else:
            message = "Clear"
            color = (0, 255, 0)
            priority = 5

        with ultrasonic_lock:
            ultrasonic_state["distance"] = d
            ultrasonic_state["message"] = message
            ultrasonic_state["color"] = color
            ultrasonic_state["last_update"] = current_time

        # Announce if needed
        if message != "Clear":
            if message != last_message or (current_time - last_announce_time) > announce_interval:
                speak(message, priority=priority)
                last_message = message
                last_announce_time = current_time
        else:
            last_message = None

        # Queue Firebase update (non-blocking)
        now = datetime.datetime.now(MALAYSIA_TZ)
        timestamp = now.strftime("%Y/%m/%d %H:%M:%S")
        firebase_data = {"timestamp": timestamp, "distance_cm": d, "message": message}
        upload_to_firebase("ultrasonic", firebase_data)

        time.sleep(1.0)

# ------------------------
# MAIN Orchestration
# ------------------------
def create_shared_memory(name: str, size: int):
    """Create shared memory block and return SharedMemory object."""
    try:
        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        LOG.info("Created shared memory %s (size=%d)", name, size)
        return shm
    except FileExistsError:
        LOG.info("Shared memory %s already exists; attaching", name)
        shm = shared_memory.SharedMemory(name=name, create=False)
        return shm

def cleanup_shared_memory(shm):
    try:
        if shm is not None:
            shm.close()
            try:
                shm.unlink()
            except FileNotFoundError:
                pass
            LOG.info("Cleaned up shared memory")
    except Exception:
        LOG.exception("Error cleaning up shared memory")

def main_loop(shm_name: str, width: int, height: int, channels: int):
    # Create shared memory in parent (main owns it)
    size = width * height * channels
    shm = create_shared_memory(shm_name, size)
    np_frame = np.ndarray((height, width, channels), dtype=np.uint8, buffer=shm.buf)

    # Start camera and yolo as child processes using the classes we generated
    # Important: create instances in parent and pass worker.run as target so recreate_shm behavior in camera_process handles attachment
    camera_worker = CameraWorker(
        shm_name=shm_name,
        width=width,
        height=height,
        channels=channels,
        frame_queue=frame_meta_queue,
        target_fps=20,
        recreate_shm=False,  # main created the shm; child should attach
    )
    camera_proc = mp.Process(target=camera_worker.run, name="camera_process")
    camera_proc.start()
    LOG.info("Started camera process pid=%d", camera_proc.pid)

    yolo_worker = YoloWorker(
        shm_name=shm_name,
        width=width,
        height=height,
        channels=channels,
        frame_queue=frame_meta_queue,
        result_queue=detection_queue,
        model_path=YOLO_MODEL,
        imgsz=YOLO_IMGSZ,
        conf=YOLO_CONF,
        allowed_classes=ALLOWED_CLASSES,
    )
    yolo_proc = mp.Process(target=yolo_worker.run, name="yolo_process")
    yolo_proc.start()
    LOG.info("Started yolo process pid=%d", yolo_proc.pid)

    # Start I/O worker threads
    # Firebase init (if available)
    fb_ok = init_firebase()
    if fb_ok:
        global ultrasonic_db, object_db
        ultrasonic_db = db.reference("ultrasonicDB")
        object_db = db.reference("objectDetectionDB")

    # Start firebase thread
    firebase_thread = threading.Thread(target=firebase_worker, name="firebase_thread", daemon=True)
    firebase_thread.start()

    # Start TTS thread
    tts_thread = threading.Thread(target=tts_worker, name="tts_thread", daemon=True)
    tts_thread.start()

    # Start ultrasonic thread
    ultrasonic_thread = threading.Thread(target=ultrasonic_thread_fn, name="ultrasonic_thread", daemon=True)
    ultrasonic_thread.start()

    # Main display loop variables
    last_detections = []
    last_frame_id = 0
    fps_counter = 0
    fps = 0.0
    fps_last_time = time.time()

    cv2.namedWindow("Vision Assistance System", cv2.WINDOW_NORMAL)

    try:
        while True:
            # Read latest frame directly from shared memory view (cheap/view)
            try:
                frame = np_frame.copy()
            except Exception:
                LOG.exception("Failed to read frame from shared memory; skipping frame display")
                time.sleep(0.01)
                continue

            # Drain detection_queue to get latest detection message (drop old)
            detections_msg = None
            try:
                # non-blocking get of one item then drain to get latest
                msg = detection_queue.get_nowait()
                detections_msg = msg
                # drain to latest
                while True:
                    try:
                        msg = detection_queue.get_nowait()
                        detections_msg = msg
                    except Exception:
                        break
            except Exception:
                # no message ready
                detections_msg = None

            # If we have detections, update last_detections
            if detections_msg and detections_msg.get("type") == "detection":
                last_detections = detections_msg.get("objects", [])
                # also upload to firebase
                if last_detections:
                    now = datetime.datetime.now(MALAYSIA_TZ)
                    data = {"timestamp": now.strftime("%Y/%m/%d %H:%M:%S"), "objects_detected": last_detections}
                    upload_to_firebase("objects", data)
                    # announce (non-blocking)
                    names = {o["name"] for o in last_detections}
                    if names:
                        speak(", ".join(names) + " detected", priority=5)

            # Draw bounding boxes from last_detections
            if last_detections:
                for det in last_detections:
                    try:
                        x1, y1, x2, y2 = det["bbox"]
                        label = f"{det['name']} {det['confidence']:.2f}"
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, label, (x1, max(y1 - 8, 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                    except Exception:
                        continue

            # Overlay ultrasonic info
            with ultrasonic_lock:
                distance = ultrasonic_state["distance"]
                message = ultrasonic_state["message"]
                color = ultrasonic_state["color"]

            cv2.putText(frame, f"Distance: {distance:.1f} cm", (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Status: {message}", (10, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # FPS calc
            fps_counter += 1
            now = time.time()
            if now - fps_last_time >= 1.0:
                fps = fps_counter / (now - fps_last_time)
                fps_counter = 0
                fps_last_time = now

            cv2.putText(frame, f"FPS: {fps:.1f}", FPS_DISPLAY_POS,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Show frame
            cv2.imshow("Vision Assistance System", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                LOG.info("Quit pressed")
                break

            # small sleep so loop is not a pure spin
            time.sleep(0.001)

    except KeyboardInterrupt:
        LOG.info("KeyboardInterrupt caught; shutting down")
    except Exception:
        LOG.exception("Unhandled exception in main loop")
    finally:
        LOG.info("Main shutting down...")

        # Signal threads to stop
        _stop_event.set()

        # give threads a moment to finish
        try:
            firebase_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            tts_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            ultrasonic_thread.join(timeout=2.0)
        except Exception:
            pass

        # Stop child processes
        try:
            if camera_proc.is_alive():
                camera_proc.terminate()
                camera_proc.join(timeout=2.0)
        except Exception:
            LOG.exception("Error terminating camera process")
        try:
            if yolo_proc.is_alive():
                yolo_proc.terminate()
                yolo_proc.join(timeout=2.0)
        except Exception:
            LOG.exception("Error terminating yolo process")

        # Close CV windows
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        # cleanup shared memory (main created it)
        try:
            shm.close()
            try:
                shm.unlink()
            except FileNotFoundError:
                pass
        except Exception:
            LOG.exception("Error cleaning up shared memory in main")

        # cleanup GPIO
        try:
            GPIO.cleanup()
        except Exception:
            pass

        LOG.info("Exited cleanly")


# ------------------------
# Entry point + signal handling
# ------------------------
def _sig_handler(sig, frame):
    LOG.info("Signal %s received; exiting", sig)
    raise KeyboardInterrupt()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    try:
        main_loop(SHM_NAME, WIDTH, HEIGHT, CHANNELS)
    except Exception:
        traceback.print_exc()
        sys.exit(1)
