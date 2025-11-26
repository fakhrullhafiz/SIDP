#!/usr/bin/env python3
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

import numpy as np
import sounddevice as sd

from vosk import Model, KaldiRecognizer
import cv2
from ultralytics import YOLO

import firebase_admin
from firebase_admin import credentials, db

# GPIO (Raspberry Pi)
try:
    import RPi.GPIO as GPIO
except:
    GPIO = None
    print("Warning: GPIO unavailable")


# ======================================================
# ----------------- GLOBAL CONSTANTS -------------------
# ======================================================

SAMPLE_RATE = 16000
CHANNELS = 1
VOICE_RECORDING = False
voice_buffer = []

# Long press threshold for cancel
LONG_PRESS_DURATION = 1.5

# GPS last known cached coordinates
last_lat = None
last_lng = None

# YOLO spam reduction
last_detected_objects = set()
last_yolo_speak_time = 0
YOLO_COOLDOWN = 5  # seconds

# Ultrasonic spam reduction
last_ultra_speak_time = 0
ULTRA_COOLDOWN = 3  # seconds
ULTRA_DIFF_THRESHOLD = 8  # cm threshold for new warning
last_ultra_distance = None

# TTS globals
tts_queue = queue.PriorityQueue()
stop_tts = False

# VOX mode: when True, device is actively recording user speech — suppress chatter
VOX_MODE_ACTIVE = False

# YOLO safe distance: only speak YOLO detections if ultrasonic shows an object closer than this (cm)
SAFE_DISTANCE_YOLO = 300  # 3 meters

# Firebase URL + Key
FIREBASE_KEY_PATH = "/home/coe/firebase/firebase-key.json"
FIREBASE_DB_URL = "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app/"

# Google API Key
API_KEY = "AIzaSyC3wbCxpdu5gBjjixsDlIR1N-hFSgR2xp4"

# GPS serial port
GPS_SERIAL_PORT = "/dev/serial0"

# YOLO Model
YOLO_MODEL_PATH = "yolov8n.pt"

# Vosk model path
VOSK_MODEL_PATH = "/home/coe/vosk-model/vosk-model-small-en-us-0.15"

# GPIO Pins (BCM)
BTN_VOICE_PIN = 22
BTN_SOS_PIN   = 27
ULTRASOUND_TRIG = 23
ULTRASOUND_ECHO = 24


# ======================================================
# -------------------- GPIO INIT -----------------------
# ======================================================

if GPIO:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BTN_VOICE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_SOS_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ULTRASOUND_TRIG, GPIO.OUT)
    GPIO.setup(ULTRASOUND_ECHO, GPIO.IN)


# ======================================================
# ------------------ FIREBASE INIT ---------------------
# ======================================================

try:
    cred = credentials.Certificate(FIREBASE_KEY_PATH)
    firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
    fb_refs = {
        "ultrasonic": db.reference("ultrasonicDB"),
        "objects": db.reference("objectDetectionDB"),
        "frame": db.reference("liveStreamDB"),
        "gps": db.reference("gpsDB"),
        "sos": db.reference("sosDB"),
        "device": db.reference("deviceStatus")
    }
    print("Firebase initialized")
except Exception as e:
    print("Firebase init error:", e)
    fb_refs = {}

fb_queue = queue.Queue(maxsize=200)


# ======================================================
# ------------------ TTS ENGINE INIT -------------------
# ======================================================

use_pyttsx3 = False
try:
    import pyttsx3
    try:
        engine = pyttsx3.init(driverName='espeak')
    except:
        engine = pyttsx3.init()
    engine.setProperty("rate", 165)
    engine.setProperty("volume", 1.0)
    use_pyttsx3 = True
    print("pyttsx3 initialized")
except:
    engine = None
    print("pyttsx3 unavailable — using espeak fallback")


# ======================================================
# ------------------- TTS WORKER -----------------------
# ======================================================

def tts_worker():
    global stop_tts
    while not stop_tts:
        try:
            priority, text = tts_queue.get(timeout=1)
            if text is None:
                continue

            # Play text with pyttsx3 or fallback to espeak subprocess
            if use_pyttsx3 and engine is not None:
                try:
                    engine.say(text)
                    engine.runAndWait()
                except Exception as e:
                    print("TTS engine error, switching to espeak:", e)
                    subprocess.run(["espeak", text], check=False)
            else:
                subprocess.run(["espeak", text], check=False)

            tts_queue.task_done()

        except queue.Empty:
            continue
        except Exception as e:
            print("TTS worker error:", e)

tts_thread = threading.Thread(target=tts_worker, daemon=True)
tts_thread.start()


# ======================================================
# ---------------------- SPEAK() -----------------------
# ======================================================

def speak(text, priority=5):
    """
    Queue-based non-blocking TTS.
    Lower priority number = higher urgency (0 = SOS)
    During VOX_MODE_ACTIVE we suppress low-priority chatter to avoid interrupting recording.
    """
    try:
        # During voice recording, suppress low-priority speech
        if VOX_MODE_ACTIVE and priority > 2:
            # drop the utterance
            return

        # Preempt lower-priority messages if this has higher priority
        if not tts_queue.empty():
            current_priority = tts_queue.queue[0][0]
            if priority < current_priority:
                with tts_queue.mutex:
                    tts_queue.queue.clear()

        tts_queue.put((priority, text))
    except Exception as e:
        print("Speak failed:", e)


# ======================================================
# ------------------ FIREBASE WORKER -------------------
# ======================================================

def firebase_worker():
    while True:
        try:
            item = fb_queue.get()
            if item is None:
                break

            ref_name, data = item
            if ref_name in fb_refs:
                fb_refs[ref_name].set(data)
                try:
                    fb_refs[ref_name].child("history").push(data)
                except:
                    pass

            fb_queue.task_done()
        except:
            continue

fb_thread = threading.Thread(target=firebase_worker, daemon=True)
fb_thread.start()


def upload_firebase(ref, data):
    try:
        fb_queue.put_nowait((ref, data))
    except:
        pass


# ======================================================
# -------------------- GPS INIT ------------------------
# ======================================================

try:
    import serial
    gps_ser = serial.Serial(GPS_SERIAL_PORT, baudrate=9600, timeout=1)
    print("GPS serial opened")
except:
    gps_ser = None
    print("GPS serial unavailable")


# ======================================================
# ------------------- GPS FUNCTIONS --------------------
# ======================================================

def get_gps_coordinates(timeout=10):
    if gps_ser is None:
        return None, None

    start = time.time()
    while time.time() - start < timeout:
        try:
            line = gps_ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith("$GPGGA") or line.startswith("$GPRMC"):
                msg = pynmea2.parse(line)
                lat = getattr(msg, 'latitude', None)
                lng = getattr(msg, 'longitude', None)
                if lat and lng:
                    return lat, lng
        except:
            continue

    return None, None


def push_live_location(lat, lng):
    data = {"latitude": lat, "longitude": lng, "timestamp": time.time()}
    upload_firebase("gps", data)


def push_sos(lat, lng):
    data = {"latitude": lat, "longitude": lng, "timestamp": time.time()}
    upload_firebase("sos", data)


def reverse_geocode(lat, lng):
    try:
        url = (f"https://maps.googleapis.com/maps/api/geocode/json?"
               f"latlng={lat},{lng}&key={API_KEY}")
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
                loc = res["geometry"]["location"]
                return res["formatted_address"], loc["lat"], loc["lng"], "reverse"

        return None, None, None
    except:
        return None, None, None


def get_nearest_place(lat, lng, radius=300):
    try:
        url = (f"https://maps.googleapis.com/maps/api/place/nearbysearch/json?"
               f"location={lat},{lng}&radius={radius}&key={API_KEY}")
        r = requests.get(url, timeout=8)
        j = r.json()

        if j.get("status") != "OK" or not j.get("results"):
            return None, None, None, None

        place = j["results"][0]
        name = place.get("name")
        loc = place["geometry"]["location"]
        return name, loc["lat"], loc["lng"], "places"
    except:
        return None, None, None, None


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlamb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlamb/2)**2
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))


def get_nearest_address(lat, lng):
    addr, al, ln, source = reverse_geocode(lat, lng)
    if addr:
        dist = haversine(lat, lng, al, ln)
        return addr, dist, source

    name, pl, pn, source = get_nearest_place(lat, lng)
    if name:
        dist = haversine(lat, lng, pl, pn)
        return name, dist, source

    return None, None, None


# ======================================================
# ---------------- GPS LIVE PUSH WORKER ----------------
# ======================================================

def gps_push_loop():
    global last_lat, last_lng
    while True:
        lat, lng = get_gps_coordinates(timeout=10)
        if lat is not None:
            last_lat, last_lng = lat, lng
            push_live_location(lat, lng)
        time.sleep(5)

threading.Thread(target=gps_push_loop, daemon=True).start()


# ======================================================
# ------------------- HANDLE LOCATION ------------------
# ======================================================

def handle_get_location():
    global last_lat, last_lng

    lat, lng = get_gps_coordinates(timeout=10)

    if lat is None:
        if last_lat is not None and last_lng is not None:
            lat, lng = last_lat, last_lng
            speak("Using last known location.")
        else:
            speak("No GPS fix and no previous location available.")
            return

    last_lat, last_lng = lat, lng

    s_lat = round(lat, 3)
    s_lng = round(lng, 3)
    speak(f"Your coordinates are latitude {s_lat} and longitude {s_lng}.")

    name, dist, source = get_nearest_address(lat, lng)

    if name is None:
        speak("I cannot find any nearby street or building.")
    else:
        meters = round(dist)
        if source == "reverse":
            speak(f"You are near {name}, about {meters} meters away.")
        else:
            speak(f"Nearest place is {name}, about {meters} meters away.")


# ======================================================
# ------------------ AUDIO INPUT/VOSK ------------------
# ======================================================

def audio_callback(indata, frames, time_info, status):
    if VOICE_RECORDING:
        voice_buffer.append(indata.copy())

try:
    audio_stream = sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        callback=audio_callback
    )
    audio_stream.start()
except:
    audio_stream = None
    print("Audio stream init failed")


vosk_model = None
if os.path.exists(VOSK_MODEL_PATH):
    try:
        vosk_model = Model(VOSK_MODEL_PATH)
        print("Vosk model loaded")
    except:
        print("Vosk failed to load")


# ======================================================
# --------------- PROCESS RECORDED AUDIO ---------------
# ======================================================

object_map = {
    "person": "person",
    "man": "person",
    "woman": "person",
    "car": "car",
    "stop sign": "stop sign",
    "chair": "chair",
    "dog": "dog",
    "cat": "cat"
}

target_name = None


def process_recording_and_set_target():
    global voice_buffer, target_name, VOX_MODE_ACTIVE

    if not voice_buffer:
        speak("No audio captured.")
        VOX_MODE_ACTIVE = False
        return

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

    if vosk_model is None:
        speak("Speech model unavailable.")
        voice_buffer = []
        VOX_MODE_ACTIVE = False
        return

    rec = KaldiRecognizer(vosk_model, SAMPLE_RATE)

    while True:
        chunk = mem.read(4096)
        if not chunk:
            break
        rec.AcceptWaveform(chunk)

    result = json.loads(rec.FinalResult())
    transcript = result.get("text", "")
    voice_buffer = []

    # done recording, unlock VOX
    VOX_MODE_ACTIVE = False

    if not transcript:
        speak("I didn't catch that.")
        return

    transcript = transcript.lower()

    detected = None
    for key, val in object_map.items():
        if key in transcript:
            detected = val
            break

    if detected:
        target_name = detected
        speak(f"Destination set to {target_name}.")
    else:
        speak(f"{transcript} is not a known target.")


# ======================================================
# ------------------- ULTRASONIC WORKER ----------------
# ======================================================

def read_ultrasonic_distance():
    if GPIO is None:
        return None

    try:
        GPIO.output(ULTRASOUND_TRIG, False)
        time.sleep(0.0002)

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
        distance = pulse_duration * 34300 / 2
        return distance
    except:
        return None


def ultrasonic_worker():
    global last_ultra_speak_time, last_ultra_distance, VOX_MODE_ACTIVE

    while True:
        d = read_ultrasonic_distance()

        if d is not None:
            upload_firebase("ultrasonic", {
                "distance_cm": d,
                "timestamp": time.time()
            })

            # Always update last distance so YOLO can reference it
            # But suppress spoken warnings during VOX_MODE_ACTIVE
            now = time.time()

            if d < 80:
                if not VOX_MODE_ACTIVE:
                    if (
                        last_ultra_distance is None or
                        (now - last_ultra_speak_time) > ULTRA_COOLDOWN or
                        abs(d - last_ultra_distance) > ULTRA_DIFF_THRESHOLD
                    ):
                        speak(f"Warning {int(d)} centimeters ahead.", priority=2)
                        last_ultra_speak_time = now

                last_ultra_distance = d
            else:
                last_ultra_distance = None

        time.sleep(0.5)

threading.Thread(target=ultrasonic_worker, daemon=True).start()


# ======================================================
# ------------------ BUTTON WATCHERS -------------------
# ======================================================

def voice_button_watcher():
    global VOICE_RECORDING, voice_buffer, VOX_MODE_ACTIVE

    press_times = []
    last_state = GPIO.input(BTN_VOICE_PIN) if GPIO else 1

    while True:
        cur = GPIO.input(BTN_VOICE_PIN) if GPIO else 1

        if last_state == 1 and cur == 0:
            press_start = time.time()
            time.sleep(0.12)

            if (GPIO.input(BTN_VOICE_PIN) if GPIO else 1) == 0:

                # ------------------- DURING RECORDING --------------------
                if VOICE_RECORDING:
                    # Wait to detect long-press
                    while (GPIO.input(BTN_VOICE_PIN) == 0) and (time.time() - press_start < LONG_PRESS_DURATION):
                        time.sleep(0.01)

                    # Long press cancel
                    if (GPIO.input(BTN_VOICE_PIN) == 0) and (time.time() - press_start >= LONG_PRESS_DURATION):
                        VOICE_RECORDING = False
                        VOX_MODE_ACTIVE = False
                        voice_buffer.clear()
                        speak("Cancelled.")
                        press_times.clear()
                        while GPIO.input(BTN_VOICE_PIN) == 0:
                            time.sleep(0.01)
                        last_state = cur
                        continue

                    # Short press stop
                    VOICE_RECORDING = False
                    # Don't unlock VOX_MODE here — processing will unlock when done
                    speak("Processing destination.")
                    threading.Thread(target=process_recording_and_set_target, daemon=True).start()
                    voice_buffer = []
                    press_times.clear()
                    while GPIO.input(BTN_VOICE_PIN) == 0:
                        time.sleep(0.01)
                    last_state = cur
                    continue

                # ------------------- NOT RECORDING -----------------------
                press_times.append(time.time())
                press_times = [t for t in press_times if time.time() - t < 0.6]

                if len(press_times) >= 2:
                    VOICE_RECORDING = True
                    VOX_MODE_ACTIVE = True  # enter VOX mode, suppress chatter
                    voice_buffer.clear()
                    speak("Recording. Say your destination and press again to stop. Long press to cancel.")
                    press_times.clear()
                    while GPIO.input(BTN_VOICE_PIN) == 0:
                        time.sleep(0.01)
                    last_state = cur
                    continue

                def single_press(wait=0.45):
                    time.sleep(wait)
                    if not VOICE_RECORDING and len(press_times) == 1:
                        handle_get_location()
                        press_times.clear()

                threading.Thread(target=single_press, daemon=True).start()

        last_state = cur
        time.sleep(0.02)

threading.Thread(target=voice_button_watcher, daemon=True).start()


def sos_button_watcher():
    global last_lat, last_lng

    last_state = GPIO.input(BTN_SOS_PIN) if GPIO else 1

    while True:
        cur = GPIO.input(BTN_SOS_PIN) if GPIO else 1

        if last_state == 1 and cur == 0:
            time.sleep(0.12)

            if (GPIO.input(BTN_SOS_PIN) if GPIO else 1) == 0:
                speak("SOS activated. Sending emergency alert now.", priority=0)

                lat, lng = get_gps_coordinates(timeout=10)

                if lat is None:
                    if last_lat is not None and last_lng is not None:
                        lat, lng = last_lat, last_lng
                        speak("Using last known GPS fix for SOS.")
                    else:
                        speak("No GPS fix. SOS not sent.", priority=0)
                        last_state = cur
                        time.sleep(0.02)
                        continue

                push_sos(lat, lng)
                s_lat = round(lat, 3)
                s_lng = round(lng, 3)
                speak(f"SOS sent. Coordinates {s_lat}, {s_lng}", priority=0)

        last_state = cur
        time.sleep(0.02)

threading.Thread(target=sos_button_watcher, daemon=True).start()


# ======================================================
# ------------------ CAMERA / YOLO LOOP ----------------
# ======================================================

cap = None
try:
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
except:
    cap = None


class MJPEGStream:
    def __init__(self):
        self.frame = None
        self.lock = threading.Lock()

    def update(self, frame):
        with self.lock:
            self.frame = frame.copy()

    def get_jpeg(self):
        with self.lock:
            if self.frame is None:
                return None
            ret, jpg = cv2.imencode(".jpg", self.frame)
            return jpg.tobytes() if ret else None


mjpeg = MJPEGStream()


# MJPEG Server
from http.server import BaseHTTPRequestHandler, HTTPServer

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/stream":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while True:
                jpg = mjpeg.get_jpeg()
                if jpg:
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except:
            pass


def start_mjpeg_server():
    try:
        server = HTTPServer(("", 8081), StreamHandler)
        print("MJPEG server running at port 8081")
        server.serve_forever()
    except Exception as e:
        print("MJPEG server error:", e)

threading.Thread(target=start_mjpeg_server, daemon=True).start()


# YOLO
try:
    yolo = YOLO(YOLO_MODEL_PATH)
    print("YOLO model loaded")
except:
    yolo = None
    print("YOLO failed to load")


# CAMERA LOOP
def camera_loop():
    global last_detected_objects, last_yolo_speak_time, last_ultra_distance, VOX_MODE_ACTIVE

    last_detection = 0
    detection_interval = 1.0

    while True:
        if cap is None or not cap.isOpened():
            time.sleep(1)
            continue

        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        mjpeg.update(frame)

        # Skip YOLO entirely while user is speaking/recording
        if VOX_MODE_ACTIVE:
            time.sleep(0.02)
            continue

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

                upload_firebase("objects", {
                    "objects_detected": objects,
                    "timestamp": time.time()
                })

                # If ultrasonic reports no close object, skip vocalizing YOLO
                # Use last_ultra_distance (cm) if available; if not available, be conservative and skip speaking
                if last_ultra_distance is None:
                    # No distance info → do not speak to avoid false alarms
                    continue

                # If the nearest ultrasonic distance is greater than SAFE_DISTANCE_YOLO => skip talk
                if last_ultra_distance > SAFE_DISTANCE_YOLO:
                    continue

                current_set = set(objects)
                now2 = time.time()

                if current_set and (
                    current_set != last_detected_objects or
                    (now2 - last_yolo_speak_time) > YOLO_COOLDOWN
                ):
                    speak("Detected " + ", ".join(list(current_set)[:3]), priority=5)
                    last_detected_objects = current_set
                    last_yolo_speak_time = now2

            except Exception as e:
                print("YOLO error:", e)

        time.sleep(0.02)

threading.Thread(target=camera_loop, daemon=True).start()


# ======================================================
# -------------------- MAIN PROGRAM ---------------------
# ======================================================

try:
    speak("Prime system is online.", priority=5)
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("Shutting down...")
    stop_tts = True
    tts_queue.put((5, None))

    if GPIO:
        GPIO.cleanup()

    if cap:
        cap.release()

    if audio_stream:
        audio_stream.stop()
        audio_stream.close()

    print("Shutdown complete")
    sys.exit(0)
