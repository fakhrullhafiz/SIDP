import os
import io
import sys
import time
import math
import json
import queue
import wave
import threading
import subprocess
import datetime
import requests
import pynmea2

# Sound / STT / TTS
import sounddevice as sd
import numpy as np
import wavio
from vosk import Model, KaldiRecognizer

# Camera / vision
import cv2
from ultralytics import YOLO

# Firebase
import firebase_admin
from firebase_admin import credentials, db

# GPIO / hardware (Raspberry Pi)
try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None
    print("Warning: RPi.GPIO not available. Running in non-hardware mode.")

# ---------------------------
# ----- CONFIG -------------
# ---------------------------
# Edit these paths
FIREBASE_KEY_PATH = "/home/coe/firebase/firebase-key.json"
FIREBASE_DB_URL = "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app/"
VOSK_MODEL_PATH = "/home/coe/vosk-model/vosk-model-small-en-us-0.15"

# Google API key for reverse geocoding + places (from gpstest)
API_KEY = "AIzaSyC3wbCxpdu5gBjjixsDlIR1N-hFSgR2xp4"

# GPS serial port used in gpstest
GPS_SERIAL_PORT = "/dev/serial0"

# Audio / VOSK config
SAMPLE_RATE = 16000
CHANNELS = 1
VOICE_RECORDING = False
voice_buffer = []

# YOLO model path (Ultralytics)
YOLO_MODEL = "yolov8n.pt"  # or your custom model path

# Firebase nodes (will be created/used)
FIREBASE_NODES = {
    "ultrasonic": "ultrasonicDB",
    "objects": "objectDetectionDB",
    "frame": "liveStreamDB",
    "device": "deviceStatus",
    "gps": "gpsDB",
    "sos": "sos"
}

# GPIO pins (change to your wiring)
# Use BCM numbering if using GPIO.setmode(GPIO.BCM), else BOARD pins if set to BOARD.
USE_BCM = True
if USE_BCM:
    BTN_VOICE_PIN = 17   # Button A - voice / location
    BTN_SOS_PIN   = 27   # Button B - SOS
    ULTRASOUND_TRIG = 23
    ULTRASOUND_ECHO  = 24
else:
    BTN_VOICE_PIN = 16
    BTN_SOS_PIN   = 18
    ULTRASOUND_TRIG = 11
    ULTRASOUND_ECHO  = 12

# MJPEG streaming config
MJPEG_PORT = 8081

# Misc
GPS_PUSH_INTERVAL = 5  # seconds for live GPS pushes

# ---------------------------
# ----- INITIALIZATION ------
# ---------------------------
# Firebase init
try:
    cred = credentials.Certificate(FIREBASE_KEY_PATH)
    firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
    db_refs = {k: db.reference(v) for k, v in FIREBASE_NODES.items()}
    print("Firebase initialized.")
except Exception as e:
    print("Firebase init error:", e)
    db_refs = {}

# GPS serial
try:
    import serial
    gps_ser = serial.Serial(GPS_SERIAL_PORT, baudrate=9600, timeout=1)
    print("GPS serial opened:", GPS_SERIAL_PORT)
except Exception as e:
    print("GPS serial init error:", e)
    gps_ser = None

# VOSK model
vosk_model = None
if os.path.exists(VOSK_MODEL_PATH):
    try:
        vosk_model = Model(VOSK_MODEL_PATH)
        print("Loaded Vosk model.")
    except Exception as e:
        print("Vosk model load error:", e)
else:
    print("Vosk model path not found:", VOSK_MODEL_PATH)

# YOLO model
try:
    yolo = YOLO(YOLO_MODEL)
    print("YOLO model loaded:", YOLO_MODEL)
except Exception as e:
    print("YOLO model load error:", e)
    yolo = None

# Audio input stream (sounddevice)
audio_stream = None
try:
    def audio_cb(indata, frames, time_info, status):
        global VOICE_RECORDING, voice_buffer
        if VOICE_RECORDING:
            voice_buffer.append(indata.copy())

    audio_stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=CHANNELS, callback=audio_cb)
    audio_stream.start()
    print("Audio input stream started.")
except Exception as e:
    print("Audio stream init error:", e)
    audio_stream = None

# GPIO setup
if GPIO:
    if USE_BCM:
        GPIO.setmode(GPIO.BCM)
    else:
        GPIO.setmode(GPIO.BOARD)
    GPIO.setup(BTN_VOICE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_SOS_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    if ULTRASOUND_TRIG and ULTRASOUND_ECHO:
        GPIO.setup(ULTRASOUND_TRIG, GPIO.OUT)
        GPIO.setup(ULTRASOUND_ECHO, GPIO.IN)

# TTS (pyttsx3 preferred, fallback to espeak)
use_pyttsx3 = False
try:
    import pyttsx3
    # prefer espeak backend on Linux
    try:
        engine = pyttsx3.init(driverName='espeak')
    except Exception:
        engine = pyttsx3.init()
    engine.setProperty('rate', 160)
    use_pyttsx3 = True
    print("pyttsx3 TTS ready.")
except Exception:
    engine = None
    print("pyttsx3 not available; will use espeak via subprocess.")

# Short TTS wrapper
def speak(text, priority=5):
    """Non-blocking-ish TTS (queues are omitted for simplicity)."""
    try:
        if use_pyttsx3 and engine is not None:
            try:
                engine.say(text)
                engine.runAndWait()
            except Exception:
                subprocess.run(["espeak", text], check=False)
        else:
            subprocess.run(["espeak", text], check=False)
    except Exception as e:
        print("TTS error:", e)

# ---------------------------
# ----- FIREBASE QUEUE ------
# ---------------------------
fb_queue = queue.Queue(maxsize=200)

def firebase_worker():
    while True:
        try:
            item = fb_queue.get()
            if item is None:
                break
            typ, data = item
            try:
                if typ in db_refs:
                    db_refs[typ].set(data)
                    # push history entry if desired
                    try:
                        db_refs[typ].child("history").push(data)
                    except Exception:
                        pass
            except Exception as e:
                print("Firebase push error:", e)
            fb_queue.task_done()
        except Exception:
            time.sleep(0.1)

fb_thread = threading.Thread(target=firebase_worker, daemon=True)
fb_thread.start()

def upload_firebase(typ, data):
    try:
        fb_queue.put_nowait((typ, data))
    except Exception:
        pass

# ---------------------------
# ----- GPS / GEOCODE -------
# ---------------------------
def get_gps_coordinates(timeout=10):
    if gps_ser is None:
        return None, None
    start = time.time()
    while time.time() - start < timeout:
        try:
            line = gps_ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                msg = pynmea2.parse(line)
                lat = getattr(msg, 'latitude', None)
                lon = getattr(msg, 'longitude', None)
                if lat and lon:
                    return lat, lon
        except Exception:
            continue
    return None, None

def push_live_location(lat, lng):
    payload = {"latitude": lat, "longitude": lng, "timestamp": time.time()}
    upload_firebase("gps", payload)

def push_sos(lat, lng):
    payload = {"latitude": lat, "longitude": lng, "timestamp": time.time()}
    upload_firebase("sos", payload)

def reverse_geocode(lat, lng):
    try:
        url = f"https://maps.googleapis.com/maps/api/geocode/json?latlng={lat},{lng}&key={API_KEY}"
        r = requests.get(url, timeout=8)
        j = r.json()
        if j.get("status") != "OK":
            return None, None, None
        results = j.get("results", [])
        for res in results:
            if "street_address" in res.get("types", []):
                return res.get("formatted_address"), lat, lng, "reverse"
        for res in results:
            if "route" in res.get("types", []):
                geom = res.get("geometry", {}).get("location", {})
                return res.get("formatted_address"), geom.get("lat"), geom.get("lng"), "reverse"
        return None, None, None
    except Exception:
        return None, None, None

def get_nearest_place(lat, lng, radius=300):
    try:
        url = f"https://maps.googleapis.com/maps/api/place/nearbysearch/json?location={lat},{lng}&radius={radius}&key={API_KEY}"
        r = requests.get(url, timeout=8)
        j = r.json()
        if j.get("status") != "OK" or not j.get("results"):
            return None, None, None, None
        place = j["results"][0]
        name = place.get("name")
        geom = place.get("geometry", {}).get("location", {})
        return name, geom.get("lat"), geom.get("lng"), "places"
    except Exception:
        return None, None, None, None

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

def get_nearest_address(lat, lng):
    addr, a_lat, a_lng, source = reverse_geocode(lat, lng)
    if addr:
        dist = haversine(lat, lng, a_lat, a_lng)
        return addr, dist, source
    name, p_lat, p_lng, source = get_nearest_place(lat, lng)
    if name:
        dist = haversine(lat, lng, p_lat, p_lng)
        return name, dist, source
    return None, None, None

# Start background GPS live push thread
def gps_push_loop():
    while True:
        lat, lng = get_gps_coordinates(timeout=10)
        if lat is not None:
            push_live_location(lat, lng)
        time.sleep(GPS_PUSH_INTERVAL)

gps_thread = threading.Thread(target=gps_push_loop, daemon=True)
gps_thread.start()

# ---------------------------
# ----- ULTRASONIC SENSOR ---
# ---------------------------
def read_ultrasonic_distance():
    # Very simple HC-SR04 reading; ensure wiring uses 3.3V or proper level shifting
    if GPIO is None:
        return None
    try:
        GPIO.output(ULTRASOUND_TRIG, False)
        time.sleep(0.05)
        GPIO.output(ULTRASOUND_TRIG, True)
        time.sleep(0.00001)
        GPIO.output(ULTRASOUND_TRIG, False)
        pulse_start = time.time()
        timeout = pulse_start + 0.02
        while GPIO.input(ULTRASOUND_ECHO) == 0 and time.time() < timeout:
            pulse_start = time.time()
        pulse_end = time.time()
        timeout = pulse_end + 0.02
        while GPIO.input(ULTRASOUND_ECHO) == 1 and time.time() < timeout:
            pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance_cm = pulse_duration * 34300 / 2
        return distance_cm
    except Exception:
        return None

# Ultrasonic background worker (push to firebase)
def ultrasonic_worker():
    while True:
        d = read_ultrasonic_distance()
        if d is not None:
            payload = {"distance_cm": d, "timestamp": time.time()}
            upload_firebase("ultrasonic", payload)
            # possible warnings:
            if d < 80:
                speak(f"Warning {int(d)} centimeters ahead.")
        time.sleep(0.5)

ultra_thread = threading.Thread(target=ultrasonic_worker, daemon=True)
ultra_thread.start()

# ---------------------------
# ----- VOICE BUTTONS ------
# ---------------------------
def handle_get_location():
    lat, lng = get_gps_coordinates(timeout=10)
    if lat is None:
        speak("No GPS fix.")
        return
    s_lat = round(lat, 3)
    s_lng = round(lng, 3)
    speak(f"Your coordinates are latitude {s_lat} and longitude {s_lng}.")
    name, dist, source = get_nearest_address(lat, lng)
    if name is None:
        speak("I cannot find any nearby street or building.")
    else:
        meters = round(dist)
        if source == "reverse":
            speak(f"You are at or near {name}. Approximately {meters} meters from your GPS point.")
        else:
            speak(f"The nearest known place is {name}, about {meters} meters away.")

def process_recording_and_set_target():
    global voice_buffer
    if not voice_buffer:
        speak("No audio captured.")
        return
    # convert buffer to bytes and run Vosk
    audio = np.concatenate(voice_buffer, axis=0)
    int16 = (audio * 32767).astype(np.int16).flatten()
    mem = io.BytesIO()
    wf = wave.open(mem, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(2)
    wf.setframerate(SAMPLE_RATE)
    wf.writeframes(int16.tobytes())
    wf.close()
    mem.seek(0)
    if vosk_model is None:
        speak("Speech model not available.")
        voice_buffer = []
        return
    rec = KaldiRecognizer(vosk_model, SAMPLE_RATE)
    while True:
        chunk = mem.read(4000)
        if not chunk:
            break
        rec.AcceptWaveform(chunk)
    result = json.loads(rec.FinalResult())
    transcript = result.get("text", "")
    mem.close()
    voice_buffer = []
    if not transcript:
        speak("I didn't catch that. Try again.")
        return
    # map to allowed classes (simple mapping)
    mapping = {
        "chair": "chair",
        "person": "person",
        "man": "person",
        "woman": "person",
        "car": "car",
        "dog": "dog",
        "cat": "cat",
        "stop sign": "stop sign"
    }
    dest = None
    for k, v in mapping.items():
        if k in transcript.lower():
            dest = v
            break
    if not dest:
        speak(f"{transcript} is not a known destination.")
        return
    # set the target (simple global)
    global target_name
    target_name = dest
    speak(f"Destination set to {target_name}.")

# Button watcher: implements the Phase 1 and recorded double-press logic for voice
def voice_button_watcher():
    global VOICE_RECORDING, voice_buffer
    press_times = []
    last_state = GPIO.input(BTN_VOICE_PIN) if GPIO else 1
    while True:
        cur = GPIO.input(BTN_VOICE_PIN) if GPIO else 1
        if last_state == 1 and cur == 0:
            # button pressed
            time.sleep(0.12)  # debounce
            if (GPIO.input(BTN_VOICE_PIN) if GPIO else 1) == 0:
                press_times.append(time.time())
                # keep only last 2 presses within 0.6s
                press_times = [t for t in press_times if time.time() - t < 0.6]
                if VOICE_RECORDING:
                    # stop recording and process
                    VOICE_RECORDING = False
                    speak("Processing destination.")
                    threading.Thread(target=process_recording_and_set_target, daemon=True).start()
                    press_times.clear()
                else:
                    if len(press_times) >= 2:
                        # double-press -> start recording
                        VOICE_RECORDING = True
                        voice_buffer.clear()
                        speak("Recording. Say your destination and press the button again to stop.")
                        press_times.clear()
                    else:
                        # single press: defer slightly to detect double press
                        def single_action(wait=0.45):
                            time.sleep(wait)
                            # if still single (no second press), treat as get-location
                            if not VOICE_RECORDING and (len(press_times) == 1):
                                handle_get_location()
                                press_times.clear()
                        threading.Thread(target=single_action, daemon=True).start()
        last_state = cur
        time.sleep(0.02)

# SOS button watcher - single press sends SOS using GPS
def sos_button_watcher():
    last_state = GPIO.input(BTN_SOS_PIN) if GPIO else 1
    while True:
        cur = GPIO.input(BTN_SOS_PIN) if GPIO else 1
        if last_state == 1 and cur == 0:
            time.sleep(0.12)
            if (GPIO.input(BTN_SOS_PIN) if GPIO else 1) == 0:
                speak("SOS activated. Sending emergency alert now.")
                lat, lng = get_gps_coordinates(timeout=10)
                if lat is None:
                    speak("No GPS fix. SOS not sent.")
                else:
                    push_sos(lat, lng)
                    s_lat = round(lat, 3)
                    s_lng = round(lng, 3)
                    speak(f"SOS sent. Coordinates {s_lat}, {s_lng}")
        last_state = cur
        time.sleep(0.02)

# start button threads
if GPIO:
    threading.Thread(target=voice_button_watcher, daemon=True).start()
    threading.Thread(target=sos_button_watcher, daemon=True).start()
else:
    print("GPIO not available — button watchers disabled (for desktop testing).")

# ---------------------------
# ----- CAMERA / YOLO -------
# ---------------------------
cap = None
try:
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("Camera opened.")
except Exception as e:
    print("Camera open error:", e)
    cap = None

# MJPEG streaming server (simple)
class MJPEGHandler:
    def __init__(self):
        self.frame = None
        self.lock = threading.Lock()

    def set(self, f):
        with self.lock:
            self.frame = f

    def get_jpeg(self):
        with self.lock:
            if self.frame is None:
                return None
            ret, jpg = cv2.imencode('.jpg', self.frame)
            if not ret:
                return None
            return jpg.tobytes()

mjpeg = MJPEGHandler()

from http.server import BaseHTTPRequestHandler, HTTPServer

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/stream':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                frame = mjpeg.get_jpeg()
                if frame:
                    self.wfile.write(b"--frame\r\n")
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                time.sleep(0.05)
        except Exception:
            pass

def start_mjpeg_server():
    try:
        server = HTTPServer(('', MJPEG_PORT), StreamHandler)
        print("Starting MJPEG server on port", MJPEG_PORT)
        server.serve_forever()
    except Exception as e:
        print("MJPEG server error:", e)

threading.Thread(target=start_mjpeg_server, daemon=True).start()

# Detection loop: capture frame, run YOLO occasionally, push to firebase
def camera_loop():
    last_detection = 0
    detection_interval = 1.0  # seconds
    while True:
        if cap is None or not cap.isOpened():
            time.sleep(1)
            continue
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
        # send frame for MJPEG streaming
        mjpeg.set(frame)

        # run YOLO every detection_interval
        now = time.time()
        if yolo and (now - last_detection) >= detection_interval:
            last_detection = now
            try:
                results = yolo.predict(frame, imgsz=640, conf=0.35, max_det=10)
                objects = []
                for res in results:
                    for box in res.boxes:
                        cls = int(box.cls[0])
                        label = res.names.get(cls, str(cls))
                        objects.append(label)
                # upload objects to firebase
                payload = {"objects_detected": objects, "timestamp": time.time()}
                upload_firebase("objects", payload)
                # if close obstacles, speak warnings (example)
                if objects:
                    # speak the most frequent or first few
                    speak(f"Detected {', '.join(objects[:3])}")
            except Exception as e:
                print("YOLO predict error:", e)
        time.sleep(0.02)

cam_thread = threading.Thread(target=camera_loop, daemon=True)
cam_thread.start()

# ---------------------------
# ----- MAIN / SHUTDOWN -----
# ---------------------------
def graceful_shutdown():
    print("Shutting down...")
    try:
        if cap:
            cap.release()
    except:
        pass
    try:
        if audio_stream:
            audio_stream.stop()
            audio_stream.close()
    except:
        pass
    try:
        fb_queue.put_nowait(None)
    except:
        pass
    try:
        if GPIO:
            GPIO.cleanup()
    except:
        pass
    print("Shutdown complete.")

# Run until Ctrl+C
try:
    speak("Prime is ready.")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    graceful_shutdown()
    sys.exit(0)
