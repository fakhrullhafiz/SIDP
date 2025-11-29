
import RPi.GPIO as GPIO
import time
import datetime
from zoneinfo import ZoneInfo
import cv2
from ultralytics import YOLO
import numpy as np
import queue
import threading
import subprocess
import firebase_admin
from firebase_admin import credentials, db
from http.server import BaseHTTPRequestHandler, HTTPServer
import sys
import base64
import signal

# ========================================
# SIGNAL HANDLER FOR GRACEFUL TERMINATION
# ========================================
def handle_sigterm(signum, frame):
    print("SIGTERM received, shutting down gracefully...")
    stop_event.set()

signal.signal(signal.SIGTERM, handle_sigterm)

# ========================================
# FIREBASE SETUP
# ========================================
cred = credentials.Certificate("/home/coe/firebase/firebase-key.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app/"
})
ultrasonic_db = db.reference("ultrasonicDB")
object_db = db.reference("objectDetectionDB")
live_stream_db = db.reference("liveStreamDB")
device_status_db = db.reference("deviceStatus")
print("Firebase initialized")

malaysia_tz = ZoneInfo("Asia/Kuala_Lumpur")

# ========================================
# GLOBAL STOP EVENT (for graceful shutdown)
# ========================================
stop_event = threading.Event()
mjpeg_server = None

# ========================================
# FIREBASE UPLOADER THREAD
# ========================================
device_status_db.update({
    "raspberryPi": "ON" ,
    "camera": "OFF"
})

firebase_queue = queue.Queue(maxsize=50)

def firebase_worker():
    while not stop_event.is_set():
        try:
            upload_type, data = firebase_queue.get(timeout=1)
            if upload_type is None:
                break
            if upload_type == "ultrasonic":
                # overwrite latest values
                ultrasonic_db.update({
                    "distance_cm": data["distance_cm"],
                    "message": data["message"],
                    "timestamp": data["timestamp"]
                })
                # history
                ultrasonic_db.child("history").push(data)
                
            elif upload_type == "objects":
                # overwrite latest values
                object_db.update({
                    "objects_detected": data["objects_detected"],
                    "timestamp": data["timestamp"]
                })
                # history
                object_db.child("history").push(data)
                
            elif upload_type == "frame":
                live_stream_db.set(data)
            firebase_queue.task_done()
            
        except queue.Empty:
            continue
        except:
            try:
                firebase_queue.task_done()
            except:
                pass

firebase_thread = threading.Thread(target=firebase_worker, daemon=True)
firebase_thread.start()

def upload_firebase(upload_type, data):
    try:
        firebase_queue.put_nowait((upload_type, data))
    except:
        pass

# ========================================
# SMART TTS MANAGER
# - Centralized request() API with per-source cooldowns
# - Suppresses low-priority TTS while user is interacting/listening
# - Dedupes and rate-limits repeated messages
# ========================================

use_pyttsx3 = False
try:
    import pyttsx3
    engine = pyttsx3.init()
    engine.setProperty("rate", 175)
    engine.setProperty("volume", 1.0)
    use_pyttsx3 = True
    print("Using pyttsx3")
except:
    engine = None
    print("Using espeak")


class TTSManager:
    def __init__(self, use_pyttsx3, engine):
        self.use_pyttsx3 = use_pyttsx3
        self.engine = engine
        self.q = queue.PriorityQueue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self.lock = threading.Lock()
        self.last_announced = {}  # key -> timestamp
        self.listening = False
        self.seq = 0
        # cooldowns (seconds) per source or message key prefix
        self.default_cooldown = 6.0
        self.source_cooldowns = {
            "ultrasonic_Stop": 8.0,
            "ultrasonic_Warning": 6.0,
            "ultrasonic_Caution": 5.0,
            "detection": 8.0,
            "beacon": 6.0,
        }

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        # push sentinel
        try:
            self.q.put((9999, 0, None, None, None))
        except Exception:
            pass
        self._thread.join(timeout=1.0)

    def set_listening(self, val: bool):
        with self.lock:
            self.listening = bool(val)

    def _effective_cooldown(self, key, source, message):
        # priority to specific key, then source prefix
        if key and key in self.source_cooldowns:
            return self.source_cooldowns[key]
        if source and f"{source}_{message}" in self.source_cooldowns:
            return self.source_cooldowns[f"{source}_{message}"]
        if source and source in self.source_cooldowns:
            return self.source_cooldowns[source]
        return self.default_cooldown

    def request(self, text, priority=5, source="generic", dedupe_key=None, min_interval=None):
        # If user is actively listening or initiating voice commands, suppress low-priority TTS
        with self.lock:
            if self.listening and (priority is None or priority >= 4):
                return False

        key = dedupe_key or text
        now = time.time()
        cooldown = min_interval if min_interval is not None else self._effective_cooldown(key, source, text)
        last = self.last_announced.get(key)
        if last is not None and (now - last) < cooldown:
            return False

        # enqueue with an increasing seq to preserve FIFO for same priority
        with self.lock:
            self.seq += 1
            seq = self.seq
        self.q.put((priority, seq, text, source, key))
        return True

    def _speak_text(self, text):
        try:
            if self.use_pyttsx3 and self.engine is not None:
                self.engine.say(text)
                self.engine.runAndWait()
            else:
                subprocess.run(["espeak", text], check=False)
        except Exception:
            pass

    def _worker(self):
        # simple worker; ensures dedupe/time-based cooldown enforcement
        while not self._stop.is_set():
            try:
                priority, seq, text, source, key = self.q.get(timeout=0.7)
            except queue.Empty:
                continue
            if text is None:
                break

            # re-check listening state before speaking
            with self.lock:
                if self.listening and (priority is None or priority >= 4):
                    # drop low priority while listening
                    continue

            # speak and update last_announced for dedupe key
            self._speak_text(text)
            try:
                self.last_announced[key] = time.time()
            except Exception:
                pass


# instantiate and start manager
tts_manager = TTSManager(use_pyttsx3, engine)
tts_manager.start()

def speak(text, priority=5, source="generic", dedupe_key=None, min_interval=None):
    return tts_manager.request(text, priority=priority, source=source, dedupe_key=dedupe_key, min_interval=min_interval)

# ========================================
# ULTRASONIC SENSOR SETUP
# ========================================
GPIO.setmode(GPIO.BOARD)
TRIG = 11
ECHO = 12
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)
time.sleep(0.1)

ultrasonic_data = {
    "distance": 0,
    "message": "Initializing",
    "color": (255, 255, 255),
    "last_update": time.time()
}
ultrasonic_lock = threading.Lock()

def measure_distance():
    try:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        timeout = time.time() + 0.1
        pulse_start = pulse_end = time.time()

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None

        return round((pulse_end - pulse_start) * 17150, 2)
    except:
        return None

def ultrasonic_worker():
    last_message = None
    last_announce = 0

    while not stop_event.is_set():
        distance = measure_distance()
        current_time = time.time()

        if distance is None:
            with ultrasonic_lock:
                ultrasonic_data.update({
                    "distance": 0,
                    "message": "Error",
                    "color": (0, 0, 255)
                })
            time.sleep(0.2)
            continue

        if distance < 50:
            message, color, priority = "Stop", (0, 0, 255), 1
        elif distance < 100:
            message, color, priority = "Warning", (0, 165, 255), 2
        elif distance < 200:
            message, color, priority = "Caution", (0, 255, 255), 3
        elif distance > 400:
            message, color, priority = "Clear", (0, 255, 0), 5
        else:
            message, color, priority = "Clear", (0, 255, 0), 5

        with ultrasonic_lock:
            ultrasonic_data.update({
                "distance": distance,
                "message": message,
                "color": color
            })

        if message != "Clear" and (message != last_message or (current_time - last_announce) > 2.0):
            # request TTS via manager; dedupe by ultrasonic_<message> so cooldowns apply per-message
            speak(message, priority, source="ultrasonic", dedupe_key=f"ultrasonic_{message}")
            last_message = message
            last_announce = current_time
        elif message == "Clear":
            last_message = None

        upload_firebase("ultrasonic", {
            "timestamp": datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S"),
            "distance_cm": distance,
            "message": message
        })

        time.sleep(0.3)

ultrasonic_thread = threading.Thread(target=ultrasonic_worker, daemon=True)
ultrasonic_thread.start()

# ========================================
# YOLO + WEBCAM SETUP
# ========================================
model = YOLO("yolov8n.pt")

allowed_classes = {
    'person', 'car', 'cat', 'dog', 'stop sign',
    'toilet', 'chair', 'bed', 'tv', 'dining table'
}

ANNOUNCE_COOLDOWN = 5.0
last_announced = {}

# ---------- Webcam setup ----------
CAM_DEVICE_INDEX = 0
cap = cv2.VideoCapture(CAM_DEVICE_INDEX, cv2.CAP_V4L2) if hasattr(cv2, "CAP_V4L2") else cv2.VideoCapture(CAM_DEVICE_INDEX)

# Request a small resolution for faster processing
CAM_W, CAM_H = 640, 480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
try:
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
except:
    pass
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print(f"ERROR: Cannot open webcam device index {CAM_DEVICE_INDEX}")
else:
    print("Webcam opened")
    device_status_db.update({"camera": "ON"})

# Performance tuning
fps = 0.0
fps_counter = 0
fps_start = time.time()
frame_skip = 3            # still skip frames to reduce inference frequency
frame_count = 0
cached_boxes = np.array([])
cached_names = []
cached_confs = np.array([])
cached_keep = []

FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
YELLOW = (0, 255, 255)

frame_for_stream = None

# ========================================
# MJPEG SERVER FOR LOCAL LAN
# ========================================
class VideoHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/video_feed':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            global frame_for_stream
            while not stop_event.is_set():
                if frame_for_stream is None:
                    time.sleep(0.01)
                    continue
                ret, buffer = cv2.imencode('.jpg', frame_for_stream)
                if not ret:
                    time.sleep(0.01)
                    continue
                frame_bytes = buffer.tobytes()
                try:
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                except BrokenPipeError:
                    break
        else:
            self.send_error(404)
            self.end_headers()

def start_server():
    global mjpeg_server
    mjpeg_server = HTTPServer(('0.0.0.0', 8000), VideoHandler)
    print("MJPEG stream available at http://<PI_LAN_IP>:8000/video_feed")
    try:
        mjpeg_server.serve_forever()
    except:
        pass

server_thread = threading.Thread(target=start_server, daemon=True)
server_thread.start()

# ========================================
# INPUT WATCHER
# ========================================
def input_worker():
    print("Type 'q' then Enter to quit.")
    while not stop_event.is_set():
        try:
            cmd = sys.stdin.readline()
            if not cmd:
                break
            if cmd.strip().lower() == 'q':
                print("Quit requested by user input.")
                stop_event.set()
                break
        except:
            break

input_thread = threading.Thread(target=input_worker, daemon=True)
input_thread.start()

# ========================================
# HELPER: UPLOAD FRAME TO FIREBASE
# ========================================
def upload_frame_to_firebase(frame):
    try:
        resized_frame = cv2.resize(frame, (640, 320))
        _, buffer = cv2.imencode('.jpg', resized_frame)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        upload_firebase("frame", {
            "frame": jpg_as_text,
            "timestamp": datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S"),
        })
    except Exception as e:
        print("Error uploading frame:", e)

# ========================================
# MAIN LOOP
# ========================================
try:
    while not stop_event.is_set():
        if cap is None or not cap.isOpened():
            print("Camera not available, waiting...")
            time.sleep(1.0)
            continue

        ret, frame = cap.read()
        if not ret or frame is None:
            # camera read failure - try again with a short delay
            time.sleep(0.05)
            continue

        frame_count += 1

        # read ultrasonic distance (thread-safe)
        with ultrasonic_lock:
            dist_val = ultrasonic_data.get("distance", 0)

        # Only run YOLO when ultrasonic indicates object within 200 cm and distance > 0
        run_yolo = (frame_count % frame_skip == 0) and (dist_val and dist_val > 0 and dist_val < 200)

        if run_yolo:
            # ensure contiguous memory for model
            small_frame = cv2.resize(frame, (320, 320))
            small_frame = np.ascontiguousarray(small_frame)
            try:
                results = model.predict(small_frame, imgsz=320, conf=0.4, verbose=False)
            except Exception as e:
                print("YOLO inference error:", e)
                # on error, skip this frame's detection
                results = None

            if results is not None:
                r = results[0]
                if len(r.boxes) > 0:
                    try:
                        cls_array = r.boxes.cls.cpu().numpy()
                        boxes = r.boxes.xyxy.cpu().numpy()
                        confs = r.boxes.conf.cpu().numpy()
                    except:
                        cls_array = np.array(r.boxes.cls)
                        boxes = np.array(r.boxes.xyxy)
                        confs = np.array(r.boxes.conf)

                    cls_array = np.atleast_1d(cls_array.squeeze())
                    confs = np.atleast_1d(confs.squeeze())
                    boxes = np.atleast_2d(boxes)

                    n = min(len(boxes), len(cls_array), len(confs))
                    if n > 0:
                        boxes = boxes[:n]
                        cls_array = cls_array[:n]
                        confs = confs[:n]

                        names = [model.names[int(c)] for c in cls_array]
                        keep = [i for i, name in enumerate(names) if name in allowed_classes]

                        if keep:
                            announced = set()
                            now = time.time()
                            for i in keep:
                                name = names[i]
                                if name not in announced:
                                    if now - last_announced.get(name, 0) >= ANNOUNCE_COOLDOWN:
                                        announced.add(name)
                                        last_announced[name] = now
                            if announced:
                                # combine announced objects into a single announcement and use a dedupe key
                                text = ", ".join(sorted(announced)) + " detected"
                                speak(text, 5, source="detection", dedupe_key=f"detection_{'_'.join(sorted(announced))}")

                            fb_objs = [{
                                "name": names[i],
                                "confidence": round(float(confs[i]), 2),
                                # scale box coords from 320->original frame size
                                "bbox": [
                                    int(boxes[i][0] * (frame.shape[1] / 320)),
                                    int(boxes[i][1] * (frame.shape[0] / 320)),
                                    int(boxes[i][2] * (frame.shape[1] / 320)),
                                    int(boxes[i][3] * (frame.shape[0] / 320))
                                ]
                            } for i in keep]

                            upload_firebase("objects", {
                                "timestamp": datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S"),
                                "objects_detected": fb_objs
                            })

                            cached_boxes = boxes
                            cached_names = names
                            cached_confs = confs
                            cached_keep = keep
                        else:
                            cached_boxes = np.array([])
                            cached_keep = []
                    else:
                        cached_boxes = np.array([])
                        cached_keep = []
                else:
                    cached_boxes = np.array([])
                    cached_keep = []
        else:
            # if not running YOLO, clear cached detections to avoid drawing stale boxes
            cached_boxes = np.array([])
            cached_keep = []

        '''# DRAW YOLO & ULTRASONIC INFO (fast; only draws cached boxes)
        if len(cached_keep) > 0:
            for i in cached_keep:
                x1, y1, x2, y2 = cached_boxes[i].astype(int)
                # scale box back from 320 inference -> current frame size
                sx = frame.shape[1] / 320.0
                sy = frame.shape[0] / 320.0
                x1o, y1o = int(x1 * sx), int(y1 * sy)
                x2o, y2o = int(x2 * sx), int(y2 * sy)
                cv2.rectangle(frame, (x1o, y1o), (x2o, y2o), GREEN, 2)
                label = f"{cached_names[i]} {cached_confs[i]:.2f}"
                cv2.putText(frame, label, (x1o, max(y1o - 5, 10)), FONT, 0.4, GREEN, 1)
		'''
		
        # overlay ultrasonic info
        with ultrasonic_lock:
            dist = ultrasonic_data["distance"]
            msg = ultrasonic_data["message"]
            col = ultrasonic_data["color"]
		
        #cv2.putText(frame, f"Dist:{dist:.1f}cm", (10, 25), FONT, 0.6, WHITE, 2)
        #cv2.putText(frame, msg, (10, 50), FONT, 0.7, col, 2)

        # prepare stream frame (small copy)
        try:
            frame_for_stream = cv2.resize(frame, (640, int(640 * frame.shape[0] / frame.shape[1])))
        except:
            frame_for_stream = frame.copy()

        # Upload to Firebase live stream less frequently (every 3 frames)
        if frame_count % 3 == 0:
            upload_frame_to_firebase(frame_for_stream)

        # small delay to release CPU
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nStopped by KeyboardInterrupt")
    stop_event.set()

finally:
    print("Shutting down...")
    try:
        device_status_db.update({
            "raspberryPi": "OFF",
            "camera": "OFF"
        })
    except:
        pass
    stop_event.set()
    try:
        if cap is not None:
            cap.release()
    except:
        pass
    try: cv2.destroyAllWindows()
    except: pass
    try: GPIO.cleanup()
    except: pass
    try:
        if mjpeg_server is not None:
            mjpeg_server.shutdown()
            mjpeg_server.server_close()
    except: pass

    try: firebase_queue.put((None, None)); firebase_thread.join(timeout=1)
    except: pass
    try:
        # stop the smarter TTS manager
        tts_manager.stop()
    except: pass
    if use_pyttsx3:
        try: engine.stop()
        except: pass
    print("Exit")

