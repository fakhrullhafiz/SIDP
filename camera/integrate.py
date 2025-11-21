#!/usr/bin/env python3

import time
import datetime
import threading
import queue
import subprocess
import sys
from zoneinfo import ZoneInfo

import cv2
import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2
import firebase_admin
from firebase_admin import credentials, db

# --------------------- CONFIG ---------------------
FIREBASE_KEY_PATH = "/home/coe/firebase/firebase-key.json"
FIREBASE_DB_URL = "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app/"
FIREBASE_ROOT = "objectDetectionDB"

MODEL_PATH = "yolov8n.pt"
PICAM_SIZE = (320, 240)         # resolution for speed
PICAM_FORMAT = "BGR888"         # camera gives frames in OpenCV-friendly BGR -> avoids full-frame color conversion
DETECTION_INTERVAL = 0.5        # seconds between detections (throttle)
DETECTION_CONF = 0.35
ALLOWED_CLASSES = {
    'person', 'car', 'cat', 'dog', 'stop sign',
    'toilet', 'chair', 'bed', 'tv', 'dining table', 'vase'
}
ANNOUNCE_COOLDOWN = 5.0         # seconds between same-class announcements
MALAYSIA_TZ = ZoneInfo("Asia/Kuala_Lumpur")

SHOW_WINDOW = True              # set False for headless
# --------------------------------------------------

# --------------------- Firebase init ---------------------
cred = credentials.Certificate(FIREBASE_KEY_PATH)
firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
root = db.reference(FIREBASE_ROOT)
print("Firebase initialized.")

# --------------------- TTS init ---------------------
tts_queue = queue.Queue(maxsize=32)
use_pyttsx3 = False
tts_engine = None
try:
    import pyttsx3
    tts_engine = pyttsx3.init()
    tts_engine.setProperty("rate", 150)
    tts_engine.setProperty("volume", 1.0)
    use_pyttsx3 = True
    print("pyttsx3 available for TTS.")
except Exception:
    use_pyttsx3 = False
    print("pyttsx3 not available, will try `espeak` for TTS.")

def tts_worker():
    while True:
        text = tts_queue.get()
        if text is None:
            break
        try:
            if use_pyttsx3 and tts_engine is not None:
                tts_engine.say(text)
                tts_engine.runAndWait()
            else:
                # fallback to espeak if pyttsx3 isn't available
                subprocess.run(["espeak", text], check=False)
        except Exception as e:
            print("TTS error:", e)
        finally:
            tts_queue.task_done()

def speak_nonblocking(text):
    try:
        tts_queue.put_nowait(text)
    except queue.Full:
        # drop TTS if overloaded
        pass

tts_thread = threading.Thread(target=tts_worker, daemon=True)
tts_thread.start()

# --------------------- Model load ---------------------
print("Loading YOLO model (may take a while)...")
model = YOLO(MODEL_PATH)
# ensure model_names accessible
if isinstance(model.names, dict):
    model_names = model.names
else:
    model_names = {i: n for i, n in enumerate(model.names)}
print("Model loaded.")

# --------------------- Shared state ---------------------
shared = {
    "frame": None,            # most recent camera frame (BGR)
    "frame_lock": threading.Lock(),
    "detections": [],         # last detections list of dicts
    "detections_lock": threading.Lock()
}
upload_queue = queue.Queue(maxsize=128)
stop_event = threading.Event()

# For announcement cooldowns
last_announced = {}

# --------------------- Camera capture thread ---------------------
def camera_capture_worker():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": PICAM_SIZE, "format": PICAM_FORMAT})
    picam2.configure(config)
    picam2.start()
    print("Camera started (capture worker).")

    try:
        while not stop_event.is_set():
            frame = picam2.capture_array()  # frame is BGR if PICAM_FORMAT=BGR888
            # store latest frame (overwrite)
            with shared["frame_lock"]:
                # Avoid copying if not necessary - but copy to be safe against concurrent use
                shared["frame"] = frame.copy()
            # small yield
            time.sleep(0) 
    except Exception as e:
        print("Camera capture error:", e)
    finally:
        try:
            picam2.stop()
        except Exception:
            pass
        print("Camera capture exiting.")

# --------------------- Detector thread ---------------------
def detector_worker():
    """
    Periodically read the latest frame, run detection, update shared['detections'], and enqueue upload payloads.
    """
    while not stop_event.is_set():
        # Read latest frame
        with shared["frame_lock"]:
            frame = shared["frame"].copy() if shared["frame"] is not None else None

        if frame is None:
            time.sleep(0.05)
            continue

        # Throttle detection
        detect_start = time.time()

        # Convert to RGB for model (model expects RGB)
        try:
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        except Exception:
            # fallback if conversion fails
            img_rgb = frame.copy()

        try:
            results = model.predict(img_rgb, imgsz=320, conf=DETECTION_CONF, verbose=False)
            r = results[0]
        except Exception as e:
            print("Inference error:", e)
            # Sleep a bit before retrying
            time.sleep(0.1)
            continue

        # Extract boxes, confs, classes robustly
        boxes_list = []
        confs_list = []
        class_list = []
        if hasattr(r, "boxes") and r.boxes is not None and len(r.boxes) > 0:
            try:
                xyxy = r.boxes.xyxy.cpu().numpy()
                scores = r.boxes.conf.cpu().numpy()
                cls_idxs = r.boxes.cls.cpu().numpy()
            except Exception:
                xyxy = np.array(r.boxes.xyxy)
                scores = np.array(r.boxes.conf)
                cls_idxs = np.array(r.boxes.cls)

            # ensure shapes ok
            if xyxy.ndim == 1 and xyxy.size == 4:
                xyxy = xyxy.reshape(1, 4)

            n = min(len(xyxy), len(scores), len(cls_idxs))
            for i in range(n):
                x1, y1, x2, y2 = xyxy[i].astype(int)
                # Convert to native Python ints
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                conf = float(scores[i])
                cls_idx = int(np.array(cls_idxs[i]).item())  # ensure python int
                name = model_names.get(cls_idx, str(cls_idx))
                boxes_list.append((x1, y1, x2, y2))
                confs_list.append(conf)
                class_list.append(name)

        # Filter allowed classes and prepare upload payload
        detections = []
        to_announce = set()
        for name, conf, bbox in zip(class_list, confs_list, boxes_list):
            if name not in ALLOWED_CLASSES:
                continue
            x1, y1, x2, y2 = bbox
            detections.append({
                "name": name,
                "confidence": float(round(conf, 2)),  # native python float
                "bbox": [int(x1), int(y1), int(x2), int(y2)]
            })

            # announcement cooldown
            last_time = last_announced.get(name, 0)
            if time.time() - last_time >= ANNOUNCE_COOLDOWN:
                to_announce.add(name)
                last_announced[name] = time.time()

        # Update shared detections for preview drawing
        with shared["detections_lock"]:
            shared["detections"] = detections  # small list of dicts

        # Trigger TTS non-blocking
        if to_announce:
            speak_nonblocking(", ".join(sorted(to_announce)) + " detected")

        # Enqueue payload for Firebase upload
        if detections:
            ts = datetime.datetime.now(MALAYSIA_TZ).strftime("%Y/%m/%d %H:%M:%S")
            payload = {"timestamp": ts, "objects_detected": detections}
            try:
                upload_queue.put_nowait(payload)
            except queue.Full:
                # drop oldest then try once
                try:
                    _ = upload_queue.get_nowait()
                except Exception:
                    pass
                try:
                    upload_queue.put_nowait(payload)
                except Exception:
                    pass

        # Respect detection interval (sleep remaining time if any)
        elapsed = time.time() - detect_start
        if elapsed < DETECTION_INTERVAL:
            time.sleep(DETECTION_INTERVAL - elapsed)

# --------------------- Uploader thread ---------------------
def uploader_worker():
    while not stop_event.is_set() or not upload_queue.empty():
        try:
            payload = upload_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        try:
            # Ensure payload contains only JSON-serializable native types (we already converted)
            root.push(payload)
            print(f"[{payload['timestamp']}] uploaded {len(payload['objects_detected'])} objects")
        except Exception as e:
            print("Firebase upload error:", e)
        finally:
            upload_queue.task_done()

# --------------------- Preview (main thread) ---------------------
def preview_loop():
    window_name = "YOLOv8 (optimized)"
    if SHOW_WINDOW:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, PICAM_SIZE[0]*2, PICAM_SIZE[1]*2)

    fps_t0 = time.time()
    frames_shown = 0

    try:
        while not stop_event.is_set():
            with shared["frame_lock"]:
                frame = shared["frame"].copy() if shared["frame"] is not None else None

            if frame is None:
                time.sleep(0.01)
                continue

            # draw last detections
            with shared["detections_lock"]:
                detections = list(shared["detections"])

            for obj in detections:
                x1, y1, x2, y2 = obj.get("bbox", [0,0,0,0])
                name = obj.get("name", "")
                conf = obj.get("confidence", 0.0)
                label = f"{name} {conf:.2f}"
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, label, (int(x1), max(int(y1)-6, 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1, cv2.LINE_AA)

            # simple FPS overlay
            frames_shown += 1
            if frames_shown % 20 == 0:
                now = time.time()
                fps = frames_shown / (now - fps_t0) if (now - fps_t0) > 0 else 0.0
                fps_t0 = now
                frames_shown = 0
                cv2.putText(frame, f"FPS: {fps:.1f}", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

            if SHOW_WINDOW:
                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_event.set()
                    break
            else:
                # headless, just sleep
                time.sleep(0.02)

    except Exception as e:
        print("Preview loop error:", e)
    finally:
        if SHOW_WINDOW:
            cv2.destroyAllWindows()

# --------------------- Graceful shutdown helpers ---------------------
def shutdown():
    print("Shutting down...")
    stop_event.set()

    # let uploader drain briefly
    try:
        uploader_thread.join(timeout=3.0)
    except Exception:
        pass

    # stop TTS thread
    try:
        tts_queue.put_nowait(None)
    except Exception:
        pass
    try:
        tts_thread.join(timeout=1.0)
    except Exception:
        pass

    # pyttsx3 engine cleanup if used
    try:
        if use_pyttsx3 and tts_engine is not None:
            tts_engine.stop()
    except Exception:
        pass

    print("Shutdown complete.")

# --------------------- Start threads ---------------------
camera_thread = threading.Thread(target=camera_capture_worker, daemon=True)
detector_thread = threading.Thread(target=detector_worker, daemon=True)
uploader_thread = threading.Thread(target=uploader_worker, daemon=True)

camera_thread.start()
detector_thread.start()
uploader_thread.start()

# Main preview loop runs in main thread to keep GUI responsive and allow keyboard interrupt
try:
    preview_loop()
except KeyboardInterrupt:
    stop_event.set()
finally:
    shutdown()
    sys.exit(0)
