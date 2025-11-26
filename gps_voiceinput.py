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

# ============================
# GOOGLE API KEY
# ============================
API_KEY = "AIzaSyC3wbCxpdu5gBjjixsDlIR1N-hFSgR2xp4"

# ============================
# GPS SERIAL PORT
# ============================
port = "/dev/serial0"
ser = serial.Serial(port, baudrate=9600, timeout=1)

# ============================
# FIREBASE INITIALIZATION
# ============================
cred = credentials.Certificate("/home/coe/firebase/firebase-key.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app"
})

# ============================
# TTS ENGINE
# ============================
def speak(text):
    print(f"[Prime]: {text}")
    engine = pyttsx3.init(deviceName='espeak')
    engine.setProperty('rate', 160)
    engine.say(text)
    engine.runAndWait()
    engine.stop()

# ============================
# SPEECH RECOGNITION SETUP
# ============================
recognizer = sr.Recognizer()
mic = sr.Microphone(sample_rate=16000)

print("Using microphone: HD Pro Webcam C920")

# ============================
# GPS READ FUNCTION
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

# ============================
# FIREBASE PUSH
# ============================
def push_live_location(lat, lng):
    ref = db.reference("gpsDB")
    ref.set({
        "latitude": lat,
        "longitude": lng,
        "timestamp": time.time()
    })

def push_sos(lat, lng):
    ref = db.reference("sos")
    ref.set({
        "latitude": lat,
        "longitude": lng,
        "timestamp": time.time()
    })

# ============================
# LOCATION RESOLUTION FUNCTIONS
# ============================
def reverse_geocode(lat, lng):
    try:
        url = f"https://maps.googleapis.com/maps/api/geocode/json?latlng={lat},{lng}&key={API_KEY}"
        r = requests.get(url).json()

        if r["status"] != "OK":
            return None, None, None

        for result in r["results"]:
            if "street_address" in result["types"]:
                return result["formatted_address"], lat, lng, "reverse"

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
# SOS FEATURE
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
# LISTEN FOR COMMAND (ONLY ON BUTTON PRESS)
# ============================
def listen_for_command():
    try:
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=1)
            speak("I'm listening.")
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=6)

        cmd = recognizer.recognize_google(audio)
        print(f"[Heard]: {cmd}")
        return cmd.lower()

    except:
        speak("I couldn't understand that.")
        return None

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

# ============================================================
# ========================= GPIO SETUP ========================
# ============================================================
GPIO.setmode(GPIO.BCM)

BTN_LISTEN = 22   # Button to trigger listening ONCE
BTN_SOS = 27      # Immediate SOS

GPIO.setup(BTN_LISTEN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTN_SOS, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ============================
# GPIO CALLBACKS
# ============================
def button_listen_callback(channel):
    print("[BUTTON] Press detected, checking for long press...")

    press_time = time.time()

    # Wait until button is released
    while GPIO.input(BTN_LISTEN) == GPIO.LOW:
        time.sleep(0.01)

    hold_time = time.time() - press_time

    if hold_time >= 1.5:
        # LONG PRESS → CANCEL
        print("[BUTTON] Long press detected → CANCEL")
        speak("Listening cancelled.")
        return

    # SHORT PRESS → LISTEN
    print("[BUTTON] Short press detected → START LISTENING")
    cmd = listen_for_command()
    handle_command(cmd)

def button_sos_callback(channel):
    print("[BUTTON] SOS button pressed")
    send_sos()

GPIO.add_event_detect(BTN_LISTEN, GPIO.FALLING, callback=button_listen_callback, bouncetime=400)
GPIO.add_event_detect(BTN_SOS, GPIO.FALLING, callback=button_sos_callback, bouncetime=400)

# ============================
# MAIN PROGRAM START
# ============================
speak("Hi, I am Prime. System is ready.")

threading.Thread(target=live_tracking_loop, daemon=True).start()

try:
    while True:
        time.sleep(0.1)  # idle loop
except KeyboardInterrupt:
    speak("Shutting down. Goodbye.")
    GPIO.cleanup()
