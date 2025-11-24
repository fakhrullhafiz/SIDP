import serial
import pynmea2
import requests
import time
import math
import pyttsx3
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
            return "Unable to determine address"

        results = data["results"]

        # 1) Try to get exact street (best result)
        for r in results:
            if "street_address" in r["types"]:
                return r["formatted_address"]

        # 2) Try to get route (road)
        for r in results:
            if "route" in r["types"]:
                return r["formatted_address"]

        # 3) Try to get a building, shop, or place
        for r in results:
            if (
                "premise" in r["types"] or
                "establishment" in r["types"] or
                "point_of_interest" in r["types"]
            ):
                return r["formatted_address"]

        # 4) If still nothing useful → return second-best (avoid Plus Code)
        for r in results:
            if "plus_code" not in r["types"]:
                return r["formatted_address"]

        # 5) Last resort
        return results[0]["formatted_address"]

    except Exception as e:
        return f"Reverse geocoding failed: {e}"


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
            
        full_lat = lat
        full_lng = lng

        lat_speak = round(lat, 3)
        lng_speak = round(lng, 3)

        speak(f"Your coordinates are latitude {lat_speak} and longitude {lng_speak}.")
        addr = reverse_geocode(full_lat, full_lng)
        speak(f"You are currently at: {addr}")

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
