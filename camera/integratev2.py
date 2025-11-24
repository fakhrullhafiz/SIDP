import sounddevice as sd
import wavio
from vosk import Model, KaldiRecognizer
import io
import os
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
from http.server import BaseHTTPRequestHandler, HTTPServer
import sys
import base64

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
firebase_queue = queue.Queue(maxsize=50)

def firebase_worker():
    while not stop_event.is_set():
        try:
            upload_type, data = firebase_queue.get(timeout=1)
            if upload_type is None:
                break
            if upload_type == "ultrasonic":
                ultrasonic_db.push(data)
            elif upload_type == "objects":
                object_db.push(data)
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
# TEXT-TO-SPEECH (TTS) SETUP
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
    engine = None
    print("Using espeak")

def tts_worker():
    while not stop_event.is_set():
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
    # flush lower priority messages if a higher priority one arrives
    try:
        if not tts_queue.empty():
            current = tts_queue.queue[0][0]
            if priority < current:
                with tts_queue.mutex:
                    tts_queue.queue.clear()
        tts_queue.put((priority, text))
    except:
        pass

# ======= VOSK MODEL =======
VOSK_MODEL_PATH = "/home/coe/vosk-model/vosk-model-small-en-us-0.15"
if os.path.exists(VOSK_MODEL_PATH):
    vosk_model = Model(VOSK_MODEL_PATH)
else:
    print("WARNING: Vosk model not found!")

voice_recording = False
voice_buffer = []
target_name = None
target_timestamp = 0

SPEECH_TO_CLASS_MAP = {
    "chair": "chair",
    "a chair": "chair",
    "man": "person",
    "woman": "person",
    "person": "person",
    "car": "car",
    "dog": "dog",
    "cat": "cat",
    "stop sign": "stop sign",
}

allowed_classes = {'person', 'car', 'stop sign', 'chair', 'dog', 'cat'}

# ======= RECORDING AUDIO =======
def audio_callback(indata, frames, time_info, status):
    if voice_recording:
        voice_buffer.append(indata.copy())

# ======= Speech Recognition + State =======
SAMPLE_RATE = 16000
CHANNELS = 1
audio_stream = sd.InputStream(
    samplerate=SAMPLE_RATE,
    channels=CHANNELS,
    callback=audio_callback
)
audio_stream.start()

def map_transcript_to_class(transcript):
    t = transcript.lower().strip()
    for key, val in SPEECH_TO_CLASS_MAP.items():
        if key in t:
            return val
    for name in allowed_classes:
        if name in t:
            return name
    return None

def process_recording_and_set_target():
    global voice_buffer, target_name

    if not voice_buffer:
        speak("No audio captured.")
        return

    import numpy as np, wave, json
    data = np.concatenate(voice_buffer, axis=0)
    int16 = (data * 32767).astype(np.int16).flatten()

    mem = io.BytesIO()
    wf = wave.open(mem, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(2)
    wf.setframerate(SAMPLE_RATE)
    wf.writeframes(int16.tobytes())
    wf.close()
    mem.seek(0)

    rec = KaldiRecognizer(vosk_model, SAMPLE_RATE)

    while True:
        chunk = mem.read(4000)
        if len(chunk) == 0:
            break
        rec.AcceptWaveform(chunk)

    result = json.loads(rec.FinalResult())
    transcript = result.get("text", "")

    voice_buffer = []

    if not transcript:
        speak("I didn't catch that. Try again.")
        return

    dest = map_transcript_to_class(transcript)
    if not dest:
        speak(f"{transcript} is not a known target.")
        return

    target_name = dest
    speak(f"Destination set to {target_name}.")

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

# ======= VOICE BUTTON SETUP =======
VOICE_BUTTON_PIN = 16  # Using BOARD numbering since your code uses GPIO.BOARD
GPIO.setup(VOICE_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ========================================
# YOLO + CAMERA SETUP
# ========================================
model = YOLO("yolov8n.pt")

'''
allowed_classes = {
    'person', 'car', 'cat', 'dog', 'stop sign',
    'toilet', 'chair', 'bed', 'tv', 'dining table'
}
'''
ANNOUNCE_COOLDOWN = 5.0
last_announced = {}

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (240, 160), "format": "RGB888"})
picam2.configure(config)
picam2.start()
print("Camera started")

fps = 0.0
fps_counter = 0
fps_start = time.time()
frame_skip = 3
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
                    continue
                ret, buffer = cv2.imencode('.jpg', frame_for_stream)
                if not ret:
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

# ======= VOICE BUTTON WATCHER THREAD =======
def voice_button_watcher():
    global voice_recording
    last = GPIO.input(VOICE_BUTTON_PIN)

    while not stop_event.is_set():
        cur = GPIO.input(VOICE_BUTTON_PIN)

        # Detect falling edge (button press)
        if last == 1 and cur == 0:
            # HARD debounce
            time.sleep(0.15)
            if GPIO.input(VOICE_BUTTON_PIN) == 0:  # confirm still pressed
                if not voice_recording:
                    voice_recording = True
                    voice_buffer.clear()
                    speak("Recording. Say your destination and press again.")
                else:
                    voice_recording = False
                    speak("Processing destination.")
                    threading.Thread(
                        target=process_recording_and_set_target,
                        daemon=True
                    ).start()

        last = cur
        time.sleep(0.02)  # small loop delay (fine)

voice_button_thread = threading.Thread(target=voice_button_watcher, daemon=True)
voice_button_thread.start()

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
# MAIN LOOP (CLEAN VERSION)
# ========================================
GUIDANCE_COOLDOWN = 1.0
last_guidance_time = 0

ARRIVAL_COOLDOWN = 3.0
last_arrival_time = 0

try:
    while not stop_event.is_set():
        frame = picam2.capture_array()
        frame_count += 1
        run_yolo = (frame_count % frame_skip == 0)

        # ========================================
        # YOLO DETECTION
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
                            if now - last_announced.get(name, 0) >= ANNOUNCE_COOLDOWN:
                                announced.add(name)
                                last_announced[name] = now

                        if announced:
                            speak(", ".join(announced) + " detected", priority=5)

                        fb_objs = [{
                            "name": names[i],
                            "confidence": round(float(confs[i]), 2),
                            "bbox": boxes[i].astype(int).tolist()
                        } for i in keep]

                        upload_firebase("objects", {
                            "timestamp": datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S"),
                            "objects_detected": fb_objs
                        })
                    else:
                        keep = []

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

        # ========================================
        # DRAW YOLO BOXES
        # ========================================
        if len(cached_keep) > 0:
            for i in cached_keep:
                x1, y1, x2, y2 = cached_boxes[i].astype(int)
                cv2.rectangle(frame, (x1, y1), (x2, y2), GREEN, 2)
                label = f"{cached_names[i]} {cached_confs[i]:.2f}"
                cv2.putText(frame, label, (x1, max(y1 - 5, 10)),
                            FONT, 0.4, GREEN, 1)

        # ========================================
        # ULTRASONIC DISPLAY
        # ========================================
        with ultrasonic_lock:
            dist = ultrasonic_data["distance"]
            msg = ultrasonic_data["message"]
            col = ultrasonic_data["color"]

        cv2.putText(frame, msg, (10, 50), FONT, 0.7, col, 2)

        # ========================================
        # FPS COUNTER
        # ========================================
        fps_counter += 1
        if time.time() - fps_start >= 1.0:
            fps = fps_counter / (time.time() - fps_start)
            fps_counter = 0
            fps_start = time.time()

        cv2.putText(frame, f"FPS:{fps:.1f}", (10, 85), FONT, 0.5, YELLOW, 1)

        # ========================================
        # NAVIGATION LOGIC
        # ========================================
        now = time.time()

        if target_name is not None and len(cached_boxes) > 0:

            # Find object index
            possible_targets = [i for i in cached_keep if cached_names[i] == target_name]
            
            if possible_targets:
                frame_center = frame.shape[1] // 2
                target_idx = min(
                    possible_targets,
                    key=lambda i: abs(((cached_boxes[i][0] + cached_boxes[i][2]) // 2) - frame_center)
                )
            else:
                target_idx = None


            # --- Not found ---
            if target_idx is None:
                if now - last_guidance_time > GUIDANCE_COOLDOWN:
                    speak(f"{target_name} not in view. Turn slowly.", priority=3)
                    last_guidance_time = now

            else:
                # Safe access
                x1, y1, x2, y2 = cached_boxes[target_idx].astype(int)
                obj_cx = (x1 + x2) // 2
                frame_cx = frame.shape[1] // 2
                dx = obj_cx - frame_cx

                # Determine left / right / center
                horiz = "center"
                if dx < -30: horiz = "left"
                elif dx > 30: horiz = "right"

                # Guidance (cooldown protected)
                if now - last_guidance_time > GUIDANCE_COOLDOWN:
                    if horiz == "center":
                        speak("Move forward", priority=3)
                    else:
                        speak(f"Turn {horiz}", priority=3)
                    last_guidance_time = now

                # Arrival detection (cooldown protected)
                if dist < 50:
                    if now - last_arrival_time > ARRIVAL_COOLDOWN:
                        speak(f"You have arrived at the {target_name}", priority=1)
                        target_name = None
                        last_arrival_time = now

        # ========================================
        # STREAM FRAME
        # ========================================
        frame_for_stream = frame.copy()

        if frame_count % 10 == 0:
            upload_frame_to_firebase(frame_for_stream)

        time.sleep(0.03)

# ========================================
# SHUTDOWN HANDLER
# ========================================
except KeyboardInterrupt:
    print("\nStopped by KeyboardInterrupt")
    stop_event.set()

finally:
    print("Shutting down...")
    stop_event.set()
    try: picam2.stop()
    except: pass
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
    try: tts_queue.put((0, None)); tts_thread.join(timeout=1)
    except: pass
    if use_pyttsx3:
        try: engine.stop()
        except: pass

    print("Exit")
