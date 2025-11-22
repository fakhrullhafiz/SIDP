import RPi.GPIO as GPIO
import time
import datetime
from zoneinfo import ZoneInfo
import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
import numpy as np
import queue
import threading
import subprocess
import firebase_admin
from firebase_admin import credentials, db

# ========================================
# FIREBASE SETUP
# ========================================
cred = credentials.Certificate("/home/coe/firebase/firebase-key.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app/"
})
ultrasonic_db = db.reference("ultrasonicDB")
object_db = db.reference("objectDetectionDB")
print("Firebase initialized")

malaysia_tz = ZoneInfo("Asia/Kuala_Lumpur")

# ========================================
# THREADED FIREBASE UPLOADER
# ========================================
firebase_queue = queue.Queue(maxsize=50)

def firebase_worker():
    while True:
        try:
            upload_type, data = firebase_queue.get(timeout=1)
            if upload_type is None:
                break
            if upload_type == "ultrasonic":
                ultrasonic_db.push(data)
            elif upload_type == "objects":
                object_db.push(data)
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
# OPTIMIZED TTS SYSTEM
# ========================================
tts_queue = queue.PriorityQueue()
use_pyttsx3 = False

try:
    import pyttsx3
    engine = pyttsx3.init()
    engine.setProperty("rate", 175)
    engine.setProperty("volume", 1.0)
    use_pyttsx3 = True
    print("Using pyttsx3")
except:
    print("Using espeak")

def tts_worker():
    while True:
        try:
            priority, text = tts_queue.get(timeout=1)
            if text is None:
                tts_queue.task_done()
                break
            if use_pyttsx3 and engine is not None:
                engine.say(text)
                engine.runAndWait()
            else:
                subprocess.run(["espeak", text], check=False)
            tts_queue.task_done()
        except queue.Empty:
            continue
        except:
            try:
                tts_queue.task_done()
            except:
                pass

tts_thread = threading.Thread(target=tts_worker, daemon=True)
tts_thread.start()

def speak(text, priority=5):
    tts_queue.put((priority, text))

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
    
    while True:
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
        else:
            message, color, priority = "Clear", (0, 255, 0), 5
        
        with ultrasonic_lock:
            ultrasonic_data.update({
                "distance": distance,
                "message": message,
                "color": color
            })
        
        if message != "Clear" and (message != last_message or (current_time - last_announce) > 2.0):
            speak(message, priority)
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
# YOLO + CAMERA SETUP
# ========================================
model = YOLO("yolov8n.pt")
allowed_classes = {'person', 'car', 'stop sign', 'chair', 'dog', 'cat'}
ANNOUNCE_COOLDOWN = 5.0
last_announced = {}

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (320, 240), "format": "RGB888"})
picam2.configure(config)
picam2.start()
print("Camera started")

# Performance tracking
fps = 0.0
fps_counter = 0
fps_start = time.time()

# Optimized skip-frame
frame_skip = 3
frame_count = 0
cached_boxes = np.array([])
cached_names = []
cached_confs = np.array([])
cached_keep = []

# Pre-render constants
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
YELLOW = (0, 255, 255)

# ========================================
# MAIN LOOP
# ========================================
try:
    while True:
        frame = picam2.capture_array()
        frame_count += 1
        run_yolo = (frame_count % frame_skip == 0)
        
        # ========================================
        # YOLO INFERENCE (every 3rd frame)
        # ========================================
        if run_yolo:
            results = model.predict(frame, imgsz=320, conf=0.4, verbose=False)
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
                            speak(", ".join(announced) + " detected", 5)
                        
                        fb_objs = [{
                            "name": names[i],
                            "confidence": round(float(confs[i]), 2),
                            "bbox": boxes[i].astype(int).tolist()
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
        
        # ========================================
        # DRAW (use cached on skip frames)
        # ========================================
        if len(cached_keep) > 0:
            for i in cached_keep:
                x1, y1, x2, y2 = cached_boxes[i].astype(int)
                cv2.rectangle(frame, (x1, y1), (x2, y2), GREEN, 2)
                label = f"{cached_names[i]} {cached_confs[i]:.2f}"
                cv2.putText(frame, label, (x1, max(y1 - 5, 10)), FONT, 0.4, GREEN, 1)
        
        # ========================================
        # ULTRASONIC OVERLAY (optimized)
        # ========================================
        with ultrasonic_lock:
            dist = ultrasonic_data["distance"]
            msg = ultrasonic_data["message"]
            col = ultrasonic_data["color"]
        
        # Solid black background (no transparency)
        #cv2.rectangle(frame, (5, 5), (315, 65), BLACK, -1)
        cv2.putText(frame, f"Dist: {dist:.1f}cm", (10, 25), FONT, 0.6, WHITE, 2)
        cv2.putText(frame, msg, (10, 50), FONT, 0.7, col, 2)
        #cv2.rectangle(frame, (10, 55), (310, 63), col, -1)
        
        # FPS (update every second)
        fps_counter += 1
        if time.time() - fps_start >= 1.0:
            fps = fps_counter / (time.time() - fps_start)
            fps_counter = 0
            fps_start = time.time()
        
        cv2.putText(frame, f"FPS:{fps:.1f}", (10, 85), FONT, 0.5, YELLOW, 1)
        
        cv2.imshow("Vision System", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopped")

finally:
    print("Shutting down...")
    try:
        picam2.stop()
    except:
        pass
    try:
        cv2.destroyAllWindows()
    except:
        pass
    try:
        GPIO.cleanup()
    except:
        pass
    try:
        firebase_queue.put((None, None))
        firebase_thread.join(timeout=1)
    except:
        pass
    try:
        tts_queue.put((0, None))
        tts_thread.join(timeout=1)
    except:
        pass
    if use_pyttsx3:
        try:
            engine.stop()
        except:
            pass
    print("Exit")
