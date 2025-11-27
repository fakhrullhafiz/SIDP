import serial
import pynmea2
import requests
import time
import math
import pyttsx3
import speech_recognition as sr
import threading
import firebase_admin
from firebase_admin import credentials, db
import RPi.GPIO as GPIO
from zoneinfo import ZoneInfo
import datetime

# ============================
# GOOGLE API KEY
# ============================
API_KEY = "YOUR_API_KEY"

# ============================
# GPS SERIAL PORT
# ============================
port = "/dev/serial0"
ser = serial.Serial(port, baudrate=9600, timeout=1)

# ============================
# FIREBASE INIT
# ============================
cred = credentials.Certificate("/home/coe/firebase/firebase-key.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app"
})

# ============================
# SPEECH RECOGNITION (NON-BLOCKING)
# ============================
recognizer = sr.Recognizer()
mic = sr.Microphone(sample_rate=16000)

is_listening = False
stop_listening_fn = None

print("Using microphone: HD Pro Webcam C920")

# ============================
# TTS ENGINE
# ============================
def speak(text):
    print(f"[Prime]: {text}")
    engine = pyttsx3.init()
    engine.setProperty('rate', 160)
    engine.say(text)
    engine.runAndWait()
    engine.stop()
    
# ============================
# GPS FUNCTIONS
# ============================
def get_gps_coordinates(timeout=10):
    start = time.time()
    while True:
        if time.time() - start > timeout:
            return None, None
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                msg = pynmea2.parse(line)
                lat = getattr(msg, 'latitude', None)
                lng = getattr(msg, 'longitude', None)
                if lat and lng:
                    return lat, lng
        except:
            continue

def push_live_location(lat, lng):
    ref = db.reference("gpsDB")
    ref.set({
        "latitude": lat,
        "longitude": lng,
        "timestamp": datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S")
    })

def push_sos(lat, lng):
    sos_root = db.reference("sosDB")

    # Latest SOS
    sos_root.update({
        "Active": True,
        "latitude": lat,
        "longitude": lng,
        "timestamp": datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S")
    })

    # SOS history
    sos_root.child("history").push({
        "latitude": lat,
        "longitude": lng,
        "timestamp": datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S")
    })

# ============================
# MAP LOOKUP FUNCTIONS
# ============================
def reverse_geocode(lat, lng):
    try:
        url = f"https://maps.googleapis.com/maps/api/geocode/json?latlng={lat},{lng}&key={API_KEY}"
        r = requests.get(url).json()

        if r["status"] != "OK":
            return None, None, None

        # Prefer exact street address
        for result in r["results"]:
            if "street_address" in result["types"]:
                return result["formatted_address"], lat, lng, "reverse"

        # Route is second best
        for result in r["results"]:
            if "route" in result["types"]:
                loc = result["geometry"]["location"]
                return result["formatted_address"], loc["lat"], loc["lng"], "reverse"

        return None, None, None
    except:
        return None, None, None

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

def get_nearest_place(lat, lng, radius=300):
    try:
        url = (
            f"https://maps.googleapis.com/maps/api/place/nearbysearch/json?"
            f"location={lat},{lng}&radius={radius}&key={API_KEY}"
        )
        r = requests.get(url).json()

        if r["status"] != "OK" or not r.get("results"):
            return None, None, None, None

        place = r["results"][0]
        name = place["name"]
        loc = place["geometry"]["location"]

        return name, loc["lat"], loc["lng"], "places"
    except:
        return None, None, None, None

def get_nearest_address(lat, lng):
    addr, a_lat, a_lng, src = reverse_geocode(lat, lng)
    if addr:
        dist = haversine(lat, lng, a_lat, a_lng)
        return addr, dist, src

    name, p_lat, p_lng, src = get_nearest_place(lat, lng)
    if name:
        dist = haversine(lat, lng, p_lat, p_lng)
        return name, dist, src

    return None, None, None

# ============================
# SOS
# ============================
def send_sos():
    speak("SOS detected. Sending emergency alert now.")
    lat, lng = get_gps_coordinates(timeout=10)

    if lat is None:
        speak("No GPS fix. SOS not sent.")
        return

    push_sos(lat, lng)
    speak(f"SOS sent. Coordinates {round(lat,3)}, {round(lng,3)}")

# ============================
# LIVE GPS THREAD
# ============================
def live_tracking_loop():
    while True:
        lat, lng = get_gps_coordinates(timeout=10)
        if lat:
            push_live_location(lat, lng)
        time.sleep(5)

# ============================
# COMMAND HANDLER
# ============================
def handle_command(cmd):
    if cmd is None:
        return

    if "location" in cmd:
        speak("Let me get your current location.")
        lat, lng = get_gps_coordinates(timeout=10)

        if lat is None:
            speak("No GPS fix.")
            return

        lat_s, lng_s = round(lat, 3), round(lng, 3)
        speak(f"Your coordinates are latitude {lat_s} and longitude {lng_s}.")

        name, dist, src = get_nearest_address(lat, lng)

        if not name:
            speak("I cannot find any nearby street or building.")
            return

        meters = round(dist)

        if src == "reverse":
            speak(f"You are at or near {name}. Approximately {meters} meters away.")
        else:
            speak(f"The nearest known place is {name}, about {meters} meters away.")

    elif "sos" in cmd or "help" in cmd:
        send_sos()

# ============================
# BACKGROUND STT CALLBACK
# ============================
def stt_callback(recognizer_obj, audio):
    global is_listening, stop_listening_fn

    if not is_listening:
        return  # cancelled mid-listen

    try:
        cmd = recognizer_obj.recognize_google(audio)
        print(f"[Heard]: {cmd}")
        is_listening = False
        stop_listening_fn(wait_for_stop=False)
        handle_command(cmd.lower())

    except:
        speak("I couldn't understand that.")
        is_listening = False
        stop_listening_fn(wait_for_stop=False)

# ============================
# START LISTENING
# ============================
def start_listening():
    global is_listening, stop_listening_fn

    if is_listening:
        speak("Already listening.")
        return

    is_listening = True
    speak("I'm listening.")

    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=0.8)

    stop_listening_fn = recognizer.listen_in_background(mic, stt_callback)

# ============================
# CANCEL LISTENING
# ============================
def cancel_listening():
    global is_listening, stop_listening_fn

    if not is_listening:
        speak("Not listening.")
        return

    is_listening = False
    if stop_listening_fn:
        stop_listening_fn(wait_for_stop=False)

    speak("Listening cancelled.")

# ============================
# GPIO SETUP
# ============================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

BTN_LISTEN = 24
BTN_SOS = 23

try:
    GPIO.cleanup([BTN_LISTEN, BTN_SOS])
except:
    pass

GPIO.setup(BTN_LISTEN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTN_SOS, GPIO.IN, pull_up_down=GPIO.PUD_UP)

time.sleep(0.1)

# ============================
# BUTTON POLLING THREAD
# ============================
def check_buttons():
    last_listen_state = GPIO.HIGH
    last_sos_state = GPIO.HIGH

    while True:

        # LISTEN BUTTON
        current_listen = GPIO.input(BTN_LISTEN)
        if current_listen == GPIO.LOW and last_listen_state == GPIO.HIGH:
            press_time = time.time()

            while GPIO.input(BTN_LISTEN) == GPIO.LOW:
                time.sleep(0.01)

            hold_time = time.time() - press_time

            if hold_time >= 1.5:
                print("[BUTTON] Long press detected → CANCEL")
                cancel_listening()
            else:
                print("[BUTTON] Short press detected → START LISTENING")
                start_listening()

            time.sleep(0.3)  # debounce

        last_listen_state = current_listen

        # SOS BUTTON
        current_sos = GPIO.input(BTN_SOS)
        if current_sos == GPIO.LOW and last_sos_state == GPIO.HIGH:
            print("[BUTTON] SOS button pressed")
            send_sos()
            time.sleep(0.3)

        last_sos_state = current_sos
        time.sleep(0.05)

# ============================
# MAIN PROGRAM START
# ============================
speak("Hi, I am Prime. System is ready.")

threading.Thread(target=live_tracking_loop, daemon=True).start()
threading.Thread(target=check_buttons(), daemon=True).start()

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    speak("Shutting down. Goodbye.")
    GPIO.cleanup()
