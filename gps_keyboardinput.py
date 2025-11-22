import serial
import pynmea2
import requests
import time
import math
import pyttsx3
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
cred = credentials.Certificate("sidp-5fcae-firebase-adminsdk-fbsvc-8989d44269.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app"
})

def push_sos_to_firebase(lat, lng):
    sos_ref = db.reference("/sos")
    payload = {
        "latitude": lat,
        "longitude": lng,
        "timestamp": time.time()
    }
    sos_ref.set(payload)
    print("[Prime]: SOS pushed to Firebase.")

# ============================
# TTS ENGINE SETUP
# ============================
engine = pyttsx3.init()
engine.setProperty('rate', 160)

def speak(text):
    print(f"[Prime]: {text}")
    engine.say(text)
    engine.runAndWait()

# ============================
# TEXT INPUT FUNCTION
# ============================
def listen_for_command():
    command = input("[Type command]: ")
    return command.lower() if command else None

# ============================
# GET ONE GPS FIX WITH TIMEOUT
# ============================
def get_gps_coordinates(timeout=10, round_coords=False):
    """Return latitude and longitude, or None if GPS fix fails after timeout."""
    start = time.time()
    while True:
        if time.time() - start > timeout:
            return None, None  # GPS failed

        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                msg = pynmea2.parse(line)
                lat = getattr(msg, 'latitude', None)
                lng = getattr(msg, 'longitude', None)
                if lat and lng:
                    return lat, lng
        except Exception as e:
            print(f"[GPS ERROR] {e}")
            continue

# ============================
# GOOGLE REVERSE GEOCODING
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
        else:
            print(f"[Geocode ERROR] Status: {data['status']}, Message: {data.get('error_message')}")
            return "[ERROR] Reverse geocoding failed"
    except Exception as e:
        print(f"[Geocode EXCEPTION] {e}")
        return "[ERROR] Reverse geocoding failed"

# ============================
# SOS FEATURE
# ============================
def send_sos():
    speak("SOS detected. Sending emergency alert now.")
    
    lat, lng = get_gps_coordinates(timeout=10, round_coords=False)
    if lat is None:
        speak("Could not get a GPS fix. SOS not sent.")
        return
        
    # Round coordiantes to 3 decimals before sending
    lat = round(lat,3)
    lng = round(lng,3)
    
    try:
        push_sos_to_firebase(lat, lng)
        speak(f"Your SOS has been sent. Help is on the way. Coordinates: {lat}, {lng}")
    except Exception as e:
        print(f"[Firebase ERROR] {e}")
        speak("Failed to send SOS to Firebase. Please try again.")

# ============================
# COMMAND HANDLER
# ============================
def handle_command(command):
    if command is None:
        return

    # ------------------- LOCATION -------------------
    if "location" in command:
        speak("Let me get your current coordinates.")
        lat, lng = get_gps_coordinates(timeout=10, round_coords=True)
        if lat is None:
            speak("Could not get GPS fix. Try moving to a location with better signal.")
            return
        
        # Round for display
        lat = round(lat,3)
        lng = round(lng,3)
        
        speak(f"Your coordinates are latitude {lat} and longitude {lng}.")
        address = reverse_geocode(lat, lng)
        speak(f"You are currently at: {address}")

    # ------------------- SOS FEATURE -------------------
    elif "sos" in command or "help" in command:
        send_sos()

# ============================
# MAIN LOOP
# ============================
speak("Hi, I am Prime. How may I help you today?")

while True:
    try:
        command = listen_for_command()
        handle_command(command)
    except KeyboardInterrupt:
        speak("Shutting down. Goodbye.")
        break
    except Exception as e:
        # Catch all unexpected exceptions so Prime never crashes
        print(f"[ERROR] {e}")
        speak("An error occurred, but I am still running.")
