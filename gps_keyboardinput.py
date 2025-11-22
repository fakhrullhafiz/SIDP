import serial
import pynmea2
import requests
import time
import math
import pyttsx3

# ============================
# PUT YOUR GOOGLE API KEY HERE
# ============================
API_KEY = "AIzaSyC3wbCxpdu5gBjjixsDlIR1N-hFSgR2xp4"

# -------- GPS Serial Port --------
port = "/dev/serial0"
ser = serial.Serial(port, baudrate=9600, timeout=1)

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
# TEXT INPUT FUNCTION (FOR TESTING)
# ============================
def listen_for_command():
    command = input("[Type command]: ")
    return command.lower() if command else None

# ============================
# GET ONE GPS FIX
# ============================
def get_gps_coordinates():
    while True:
        line = ser.readline().decode('ascii', errors='replace').strip()

        if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
            try:
                msg = pynmea2.parse(line)
                lat = getattr(msg, 'latitude', None)
                lng = getattr(msg, 'longitude', None)

                if lat and lng:
                    return lat, lng
            except:
                continue

# ============================
# GOOGLE REVERSE GEOCODING
# ============================
def reverse_geocode(lat, lng):
    url = (
        f"https://maps.googleapis.com/maps/api/geocode/json?"
        f"latlng={lat},{lng}&key={API_KEY}"
    )
    response = requests.get(url)
    data = response.json()

    if data["status"] == "OK":
        return data["results"][0]["formatted_address"]
    return "[ERROR] Reverse geocoding failed"

# ============================
# COMMAND HANDLER
# ============================
def handle_command(command):
    if command is None:
        return

    if "location" in command:
        speak("Let me get your current coordinates.")
        lat, lng = get_gps_coordinates()

        speak(f"Your coordinates are latitude {lat} and longitude {lng}.")

        # optional: reverse geocode
        address = reverse_geocode(lat, lng)
        speak(f"You are currently at: {address}")

    elif "sos" in command:
        speak("SOS feature is not implemented yet.")

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
