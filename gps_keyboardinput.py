import serial
import pynmea2
import requests
import time
import math
import pyttsx3
import threading
import firebase_admin
from firebase_admin import credentials, db
from zoneinfo import ZoneInfo
import datetime

# ============================
# TIMEZONE SETUP (MALAYSIA)
# ============================
malaysia_tz = ZoneInfo("Asia/Kuala_Lumpur")

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
engine = pyttsx3.init()
engine.setProperty('rate', 160)

def speak(text):
    print(f"[Prime]: {text}")
    engine.say(text)
    engine.runAndWait()

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
        "timestamp": datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S")
    })

def push_sos(lat, lng):
    ref = db.reference("sos")
    ref.set({
        "latitude": lat,
        "longitude": lng,
        "timestamp": datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S"),
        "active": True
    })

# ============================
# SOS CLEAR FUNCTION
# ============================
def clear_sos():
    ref = db.reference("sos/active")
    ref.set(False)
    print("[Prime]: SOS cleared.")

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
        if data["status"] == "OK":
            return data["results"][0]["formatted_address"]
        return "Unable to determine address"
    except:
        return "Reverse geocoding failed"

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
    cmd = input("[Type command]: ")
    return cmd.lower() if cmd else None

def handle_command(cmd):
    if cmd is None:
        return

    if "location" in cmd:
        speak("Let me get your current location.")

        lat, lng = get_gps_coordinates(timeout=10)

        if lat is None:
            speak("No GPS fix.")
            return

        lat = round(lat, 3)
        lng = round(lng, 3)

        speak(f"Your coordinates are latitude {lat} and longitude {lng}.")
        addr = reverse_geocode(lat, lng)
        speak(f"You are currently at: {addr}")

    elif "sos" in cmd or "help" in cmd:
        send_sos()
        
    elif "clear sos" in cmd or "sos off" in cmd or "stop sos" in cmd:
        clear_sos()
        speak("SOS has been cleared.")
        
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
