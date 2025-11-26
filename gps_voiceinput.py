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

# ============================
# PUT YOUR GOOGLE API KEY HERE
# ============================
API_KEY = "AIzaSyC3wbCxpdu5gBjjixsDlIR1N-hFSgR2xp4"

# -------- GPS Serial Port --------
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
# SPEECH RECOGNITION
# ============================
recognizer = sr.Recognizer()

# FORCE using webcam mic (C920)
mic = sr.Microphone(, sample_rate=16000)

print("Using microphone: HD Pro Webcam C920 (device index 3)")

# ============================
# GPS FUNCTION
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
# FIREBASE PUSH FUNCTIONS
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
# REVERSE GEOCODING
# ============================
def reverse_geocode(lat, lng):
    try:
        url = (
            f"https://maps.googleapis.com/maps/api/geocode/json?"
            f"latlng={lat},{lng}&key={API_KEY}"
        )
        response = requests.get(url)
        data = response.json()

        if data["status"] != "OK":
            return None, None, None

        results = data["results"]

        # 1. Try exact street first
        for r in results:
            if "street_address" in r["types"]:
                return r["formatted_address"], lat, lng, "reverse"

        # 2. Try nearest road
        for r in results:
            if "route" in r["types"]:
                loc = r["geometry"]["location"]
                return r["formatted_address"], loc["lat"], loc["lng"], "reverse"

        # If nothing useful → fall back to Places
        return None, None, None

    except Exception as e:
        return None, None, None

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
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
        response = requests.get(url)
        data = response.json()

        if data["status"] != "OK" or not data.get("results"):
            return None, None, None, None

        place = data["results"][0]  # nearest
        name = place["name"]
        loc = place["geometry"]["location"]
        return name, loc["lat"], loc["lng"], "places"

    except Exception as e:
        return None, None, None, None

def get_nearest_address(lat, lng):
    # FIRST attempt reverse geocoding for streets
    addr, addr_lat, addr_lng, source = reverse_geocode(lat, lng)

    if addr is not None:
        dist = haversine(lat, lng, addr_lat, addr_lng)
        return addr, dist, source

    # If reverse geocode fails → fallback to Places
    name, p_lat, p_lng, source = get_nearest_place(lat, lng)

    if name is not None:
        dist = haversine(lat, lng, p_lat, p_lng)
        return name, dist, source

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

    # --- PUSH EXACT VALUES (NO ROUNDING) ---
    push_sos(lat, lng)

    # --- BUT SPEAK ROUNDED VALUES TO USER ---
    lat_speak = round(lat, 3)
    lng_speak = round(lng, 3)

    speak(f"SOS sent. Coordinates {lat_speak}, {lng_speak}")

# ============================
# BACKGROUND LIVE TRACKING THREAD
# ============================
def live_tracking_loop():
    while True:
        lat, lng = get_gps_coordinates(timeout=10)
        if lat is not None:
            push_live_location(lat, lng)
        time.sleep(5)  # <-- UPDATE INTERVAL (EVERY 5 SECONDS)

# ============================
# COMMAND HANDLER
# ============================
def listen_for_command():
    try:
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=1)
            speak("I'm listening.")
            print("Listening...")

            audio = recognizer.listen(source, timeout=5, phrase_time_limit=6)

        cmd = recognizer.recognize_google(audio)
        print(f"[Heard]: {cmd}")
        return cmd.lower()

    except sr.WaitTimeoutError:
        speak("I didn't hear anything.")
        return None
    except sr.UnknownValueError:
        speak("I couldn't understand that.")
        return None
    except sr.RequestError:
        speak("Speech recognition service error.")
        return None
    except Exception as e:
        print("[ERROR]:", e)
        speak("An error occurred.")
        return None

def handle_command(cmd):
    if cmd is None:
        return

    if "location" in cmd:
        speak("Let me get your current location.")

        lat, lng = get_gps_coordinates(timeout=10)

        if lat is None:
            speak("No GPS fix.")
            return
            
        full_lat = lat
        full_lng = lng

        lat_speak = round(lat, 3)
        lng_speak = round(lng, 3)

        speak(f"Your coordinates are latitude {lat_speak} and longitude {lng_speak}.")
        name, dist, source = get_nearest_address(full_lat, full_lng)

        if name is None:
            speak("I cannot find any nearby street or building.")
        else:
            meters = round(dist)
            if source == "reverse":
                speak(f"You are at or near {name}. Approximately {meters} meters from your GPS point.")
            elif source == "places":
                speak(f"The nearest known place is {name}, about {meters} meters away.")

    elif "sos" in cmd or "help" in cmd:
        send_sos()


# ============================
# MAIN PROGRAM START
# ============================
speak("Hi, I am Prime. How may I help you today?")

# Start tracking thread
threading.Thread(target=live_tracking_loop, daemon=True).start()

while True:
    try:
        cmd = listen_for_command()
        handle_command(cmd)
    except KeyboardInterrupt:
        speak("Shutting down. Goodbye.")
        break
    except Exception as e:
        print("[ERROR]", e)
        speak("An error occurred but I am still running.")
