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
print("Firebase initialized successfully!")

malaysia_tz = ZoneInfo("Asia/Kuala_Lumpur")

# ========================================
# THREADED FIREBASE UPLOADER
# ========================================
firebase_queue = queue.Queue(maxsize=100)

def firebase_worker():
    """Background thread for non-blocking Firebase uploads"""
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
        except Exception as e:
            print(f"Firebase upload error: {e}")
            try:
                firebase_queue.task_done()
            except:
                pass

firebase_thread = threading.Thread(target=firebase_worker, daemon=True)
firebase_thread.start()

def upload_to_firebase(upload_type, data):
    """Queue data for background Firebase upload"""
    try:
        firebase_queue.put_nowait((upload_type, data))
    except queue.Full:
        print("Firebase queue full, skipping upload")

# ========================================
# UNIFIED TTS SYSTEM (Threaded, Non-blocking)
# ========================================
tts_queue = queue.PriorityQueue()
use_pyttsx3 = False
engine = None

try:
    import pyttsx3
    engine = pyttsx3.init()
    engine.setProperty("rate", 150)
    engine.setProperty("volume", 1.0)
    use_pyttsx3 = True
    print("Using pyttsx3 for TTS")
except Exception:
    use_pyttsx3 = False
    print("Using espeak for TTS")

def tts_worker():
    """Background thread that handles all TTS announcements"""
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
        except Exception as e:
            print(f"TTS error: {e}")
            try:
                tts_queue.task_done()
            except:
                pass

tts_thread = threading.Thread(target=tts_worker, daemon=True)
tts_thread.start()

def speak(text, priority=5):
    """Add text to TTS queue with priority"""
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
    "message": "Initializing...",
    "color": (255, 255, 255),
    "last_update": time.time()
}
ultrasonic_lock = threading.Lock()

def measure_distance():
    """Measure distance from HC-SR04 sensor with timeout protection"""
    try:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_start = time.time()
        pulse_end = time.time()
        timeout = time.time() + 0.1

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return round(distance, 2)
    except Exception as e:
        print(f"Ultrasonic error: {e}")
        return None

def ultrasonic_worker():
    """Background thread for continuous ultrasonic monitoring"""
    last_message = None
    last_announce_time = 0
    announce_interval = 3.0
    
    while True:
        distance = measure_distance()
        current_time = time.time()
        
        if distance is None:
            with ultrasonic_lock:
                ultrasonic_data["distance"] = 0
                ultrasonic_data["message"] = "Sensor Error"
                ultrasonic_data["color"] = (0, 0, 255)
            time.sleep(0.2)
            continue
        
        if distance < 50:
            message = "Stop"
            color = (0, 0, 255)
            priority = 1
        elif distance < 100:
            message = "Warning"
            color = (0, 165, 255)
            priority = 2
        elif distance < 200:
            message = "Caution"
            color = (0, 255, 255)
            priority = 3
        else:
            message = "Clear"
            color = (0, 255, 0)
            priority = 5
        
        with ultrasonic_lock:
            ultrasonic_data["distance"] = distance
            ultrasonic_data["message"] = message
            ultrasonic_data["color"] = color
            ultrasonic_data["last_update"] = current_time
        
        if message != "Clear":
            if message != last_message or (current_time - last_announce_time) > announce_interval:
                speak(message, priority=priority)
                last_message = message
                last_announce_time = current_time
        else:
            last_message = None
        
        # Non-blocking Firebase upload
        now = datetime.datetime.now(malaysia_tz)
        timestamp = now.strftime("%Y/%m/%d %H:%M:%S")
        firebase_data = {
            "timestamp": timestamp,
            "distance_cm": distance,
            "message": message
        }
        upload_to_firebase("ultrasonic", firebase_data)
        
        time.sleep(1.0)

ultrasonic_thread = threading.Thread(target=ultrasonic_worker, daemon=True)
ultrasonic_thread.start()

# ========================================
# YOLO + CAMERA SETUP
# ========================================
model = YOLO("yolov8n.pt")
allowed_classes = {
    'person', 'car', 'cat', 'dog', 'stop sign',
    'toilet', 'chair', 'bed', 'tv', 'dining table', 'vase'
}
ANNOUNCE_COOLDOWN = 5.0
last_announced = {}

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (640,480), "format": "XBGR8888"}
)
picam2.configure(config)
picam2.start()
print("Camera started. Press 'q' to quit")

# FPS tracking (updates every second)
fps = 0.0
fps_counter = 0
fps_start_time = time.time()

# Skip-frame optimization
frame_skip = 3  # Run YOLO every 2nd frame
frame_count = 0
last_boxes = []
last_detected_names = []
last_confs = []
last_keep_idx = []

# ========================================
# MAIN LOOP
# ========================================
try:
    while True:
        # Capture frame
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
        
        frame_count += 1
        run_yolo = (frame_count % frame_skip == 0)
        
        # ========================================
        # YOLO INFERENCE (every Nth frame)
        # ========================================
        if run_yolo:
            results = model.predict(frame, imgsz=320, conf=0.35, verbose=False)
            r = results[0]
            
            try:
                cls_array = r.boxes.cls.cpu().numpy()
                boxes = r.boxes.xyxy.cpu().numpy()
                confs = r.boxes.conf.cpu().numpy()
            except Exception:
                cls_array = np.array(r.boxes.cls)
                boxes = np.array(r.boxes.xyxy)
                confs = np.array(r.boxes.conf)
            
            cls_array = np.atleast_1d(np.array(cls_array).squeeze())
            confs = np.atleast_1d(np.array(confs).squeeze())
            boxes = np.array(boxes)
            if boxes.ndim == 1 and boxes.size == 4:
                boxes = boxes.reshape(1, 4)
            
            n = min(boxes.shape[0], cls_array.shape[0], confs.shape[0])
            
            detected_names = []
            firebase_objects = []
            
            if n > 0:
                boxes = boxes[:n]
                cls_array = cls_array[:n]
                confs = confs[:n]
                
                for c in cls_array:
                    idx = int(c)
                    if isinstance(model.names, dict):
                        name = model.names.get(idx, str(idx))
                    else:
                        name = model.names[idx]
                    detected_names.append(name)
                
                keep_idx = [i for i, name in enumerate(detected_names) if name in allowed_classes]
                
                # Announcement logic
                announced_this_frame = set()
                for i in keep_idx:
                    name = detected_names[i]
                    if name not in announced_this_frame:
                        last_time = last_announced.get(name, 0)
                        if (time.time() - last_time) >= ANNOUNCE_COOLDOWN:
                            announced_this_frame.add(name)
                            last_announced[name] = time.time()
                
                if announced_this_frame:
                    announcement = ", ".join(announced_this_frame) + " detected"
                    speak(announcement, priority=5)
                
                # Prepare Firebase data
                for i in keep_idx:
                    x1, y1, x2, y2 = boxes[i].astype(int)
                    conf = float(confs[i])
                    name = detected_names[i]
                    firebase_objects.append({
                        "name": name,
                        "confidence": round(conf, 2),
                        "bbox": [int(x1), int(y1), int(x2), int(y2)]
                    })
                
                # Non-blocking Firebase upload
                if firebase_objects:
                    now = datetime.datetime.now(malaysia_tz)
                    timestamp = now.strftime("%Y/%m/%d %H:%M:%S")
                    data = {
                        "timestamp": timestamp,
                        "objects_detected": firebase_objects
                    }
                    upload_to_firebase("objects", data)
                
                # Cache results for skipped frames
                last_boxes = boxes
                last_detected_names = detected_names
                last_confs = confs
                last_keep_idx = keep_idx
            else:
                # No detections
                last_boxes = []
                last_detected_names = []
                last_confs = []
                last_keep_idx = []
        
        # ========================================
        # DRAW BOUNDING BOXES (use cached results on skipped frames)
        # ========================================
        if len(last_keep_idx) > 0:
            for i in last_keep_idx:
                x1, y1, x2, y2 = last_boxes[i].astype(int)
                conf = float(last_confs[i])
                name = last_detected_names[i]
                label = f"{name} {conf:.2f}"
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, max(y1 - 8, 10)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # ========================================
        # OVERLAY ULTRASONIC DATA
        # ========================================
        with ultrasonic_lock:
            distance = ultrasonic_data["distance"]
            message = ultrasonic_data["message"]
            color = ultrasonic_data["color"]
        
        # Semi-transparent overlay
        #overlay = frame.copy()
        #cv2.rectangle(overlay, (5, 5), (315, 70), (0, 0, 0), -1)
        #cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Display ultrasonic info
        cv2.putText(frame, f"Distance: {distance:.1f} cm", (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Status: {message}", (10, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        #cv2.rectangle(frame, (10, 55), (310, 65), color, -1)
        
        # ========================================
        # FPS CALCULATION (updates every second)
        # ========================================
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= 1.0:
            fps = fps_counter / (current_time - fps_start_time)
            fps_counter = 0
            fps_start_time = current_time
        
        # Display FPS
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Show frame
        cv2.imshow("Vision Assistance System", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopped by user")

finally:
    print("Shutting down...")
    
    try:
        picam2.stop()
    except Exception:
        pass
    
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    
    try:
        GPIO.cleanup()
    except Exception:
        pass
    
    try:
        firebase_queue.put((None, None))
        firebase_thread.join(timeout=2)
    except Exception:
        pass
    
    try:
        tts_queue.put((0, None))
        tts_thread.join(timeout=2)
    except Exception:
        pass
    
    if use_pyttsx3 and engine is not None:
        try:
            engine.stop()
        except Exception:
            pass
    
    print("Exited cleanly")
