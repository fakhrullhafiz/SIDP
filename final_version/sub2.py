#!/usr/bin/env python3
"""
gpstest_with_beacon.py (modified)
- Adds terminal scan/status prints
- Prints beacon found (UUID/Major/Minor/Tx)
- Prints distance + RSSI regularly
- Adds TTS state machine: detected -> approaching/away -> almost there -> reached
- Keeps Firebase/GPS/TTS functionality intact
"""

import os
import time
import queue
import threading
from collections import deque
from datetime import datetime, timezone
from zoneinfo import ZoneInfo
import datetime

# BLE
import asyncio
from bleak import BleakScanner

# GPS / Serial / NMEA
import serial
import pynmea2

# TTS
import pyttsx3

# Speech recognition (online)
import speech_recognition as sr

# Networking / Firebase
import requests
import firebase_admin
from firebase_admin import credentials, db

# GPIO
import RPi.GPIO as GPIO

# ============================
# TIMEZONE SETUP (MALAYSIA)
# ============================
malaysia_tz = ZoneInfo("Asia/Kuala_Lumpur")

# -------------------------
# === Credentials (exact as provided) ===
# ============================
# GOOGLE API KEY
# ============================
API_KEY = "AIzaSyC3wbCxpdu5gBjjixsDlIR1N-hFSgR2xp4"

# ============================
# GPS SERIAL PORT
# ============================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUDRATE = 9600

# ============================
# FIREBASE INITIALIZATION
# ============================
FIREBASE_CRED_PATH = "/home/coe/firebase/firebase-key.json"
FIREBASE_DB_URL = "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app"

# ==============================
# CONFIG FOR YOUR BEACONS
# ==============================
TARGET_UUID = "fda50693-a4e2-4fb1-afcf-c6eb07647825".lower()

TX_POWER_DEFAULT = -59      # Your beacons' power setting
RSSI_WINDOW = 8             # Number of readings to average (smoother)
ENV_FACTOR = 2.5            # Path-loss exponent (2.5=indoor)

# -------------------------
# Other config
# -------------------------
# TTS
TTS_RATE = 160

# GPIO pins (BCM)
BTN_LISTEN = 24
BTN_SOS = 23

# Push interval for live GPS (seconds)
GPS_PUSH_INTERVAL = 5

# Beacon mapping (major, minor) -> label (update to match your beacons)
BEACON_LOCATIONS = {
    (1, 1): "Demo End",
}

# Output rate limiting (seconds)
UPDATE_INTERVAL = 3.0       # Seconds between updates for same beacon
SUMMARY_INTERVAL = 10.0     # Seconds between summary prints

# SOS strict phrases
STRICT_SOS_PHRASES = [
    "sos", "help me", "send help", "send sos", "i need help",
    "emergency", "please help me"
]

# Speech keywords for location queries
LOCATION_KEYWORDS = ["location", "where", "where am i", "where are we"]

# timezone
zone = ZoneInfo("Asia/Kuala_Lumpur")

# -------------------------
# Globals and shared state
# -------------------------
latest_gps_lock = threading.Lock()
latest_gps = {"lat": None, "lng": None, "timestamp": None}

# beacon event queue
event_queue = queue.Queue()

# TTS engine and lock
tts_engine = pyttsx3.init()
tts_engine.setProperty("rate", TTS_RATE)
tts_lock = threading.Lock()

# Firebase flag
firebase_enabled = False

# Beacon control
beacon_stop_event = threading.Event()

# Speech recognizer
recognizer = sr.Recognizer()
mic = sr.Microphone()  # may raise if no mic; wrap when used

# -------------------------
# State machine for TTS and terminal output
# -------------------------
# Distance categories (meters)
REACHED_THRESHOLD = 1.0       # <1.0 m => reached
NEAR_THRESHOLD = 3.0          # <3.0 m => almost there
MID_THRESHOLD = 9.0           # <9.0 m => mid-range
# Far = >= MID_THRESHOLD

# Cooldowns (seconds)
COOLDOWN_GENERAL = 6.0        # for approach/away notifications
COOLDOWN_NEAR = 10.0          # when in NEAR
COOLDOWN_REACHED = 30.0       # once reached, longer cooldown to prevent repetition

device_state_lock = threading.Lock()
device_state = {
    "last_distance": None,
    "last_rssi": None,
    "last_state": None,   # one of: None, "detected", "far", "mid", "near", "reached"
    "last_state_ts": 0.0,
    "first_seen": False
}

# -------------------------
# Utility functions
# -------------------------
def now_iso():
    # Return timestamp in the same format as gpstest.py using malaysia_tz
    return datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S")

def speak(text):
    """Thread-safe TTS using a single pyttsx3 engine."""
    with tts_lock:
        try:
            print("[TTS]", text)
            tts_engine.say(text)
            tts_engine.runAndWait()
        except Exception as e:
            print("TTS error:", e)

# -------------------------
# Firebase init & helpers
# -------------------------
def init_firebase():
    global firebase_enabled
    try:
        if not os.path.exists(FIREBASE_CRED_PATH):
            print("Firebase credential file not found:", FIREBASE_CRED_PATH)
            firebase_enabled = False
            return
        cred = credentials.Certificate(FIREBASE_CRED_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
        firebase_enabled = True
        print("Firebase initialized.")
    except Exception as e:
        print("Firebase init failed:", e)
        firebase_enabled = False

def push_live_location(lat, lng, ts_iso):
    if not firebase_enabled:
        return
    try:
        ref = db.reference("gpsDB")
        ref.set({
            "latitude": lat,
            "longitude": lng,
            "timestamp": ts_iso
        })
    except Exception as e:
        print("push_live_location error:", e)

def push_sos_record(lat, lng, ts_iso, reason=None):
    if not firebase_enabled:
        return
    try:
        sos_root = db.reference("sosDB")
        ts_local = datetime.datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S")
        sos_root.update({
            "Active": True,
            "latitude": lat,
            "longitude": lng,
            "timestamp": ts_local
        })
        sos_root.child("history").push({
            "latitude": lat,
            "longitude": lng,
            "timestamp": ts_local
        })
    except Exception as e:
        print("push_sos_record error:", e)

def push_beacon_sighting(beacon_payload):
    if not firebase_enabled:
        return
    try:
        ref = db.reference("/beacon_sightings")
        ref.push(beacon_payload)
    except Exception as e:
        print("push_beacon_sighting error:", e)

# -------------------------
# GPS reading & handling
# -------------------------
def open_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print("Opened serial:", SERIAL_PORT)
        return ser
    except Exception as e:
        print("Failed to open serial:", e)
        return None

def parse_nmea_line(line):
    try:
        msg = pynmea2.parse(line)
        if hasattr(msg, "latitude") and hasattr(msg, "longitude"):
            if msg.latitude and msg.longitude:
                return msg.latitude, msg.longitude
    except pynmea2.nmea.ChecksumError:
        pass
    except Exception:
        pass
    return None, None

def gps_thread_fn():
    global latest_gps
    ser = open_serial()
    if ser is None:
        print("GPS serial unavailable; exiting GPS thread.")
        return
    last_push = 0
    while True:
        try:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                time.sleep(0.05)
                continue
            lat, lng = parse_nmea_line(raw)
            if lat is not None and lng is not None:
                ts_iso = now_iso()
                with latest_gps_lock:
                    latest_gps["lat"] = lat
                    latest_gps["lng"] = lng
                    latest_gps["timestamp"] = ts_iso
                if time.time() - last_push >= GPS_PUSH_INTERVAL:
                    push_live_location(lat, lng, ts_iso)
                    last_push = time.time()
        except Exception as e:
            print("GPS thread exception:", e)
            time.sleep(0.5)

# -------------------------
# Reverse geocode & places (Google)
# -------------------------
def reverse_geocode(lat, lng):
    """Return (formatted_address or None)."""
    try:
        url = f"https://maps.googleapis.com/maps/api/geocode/json?latlng={lat},{lng}&key={API_KEY}"
        r = requests.get(url, timeout=4)
        data = r.json()
        if data.get("status") == "OK" and data.get("results"):
            return data["results"][0].get("formatted_address")
    except Exception as e:
        print("reverse_geocode error:", e)
    return None

def get_nearest_place(lat, lng):
    """Return (name, vicinity) or (None, None)"""
    try:
        url = f"https://maps.googleapis.com/maps/api/place/nearbysearch/json?location={lat},{lng}&rankby=distance&key={API_KEY}"
        r = requests.get(url, timeout=4)
        data = r.json()
        if data.get("status") == "OK" and data.get("results"):
            top = data["results"][0]
            name = top.get("name")
            vicinity = top.get("vicinity") or top.get("formatted_address")
            return name, vicinity
    except Exception as e:
        print("get_nearest_place error:", e)
    return None, None

# -------------------------
# SOS & voice handling
# -------------------------
def send_sos():
    """Get latest GPS and push SOS record + speak."""
    lat = None; lng = None; ts = None
    with latest_gps_lock:
        lat = latest_gps.get("lat")
        lng = latest_gps.get("lng")
        ts = latest_gps.get("timestamp")
    if lat is None or lng is None:
        speak("GPS fix unavailable. SOS cannot be sent.")
        return
    push_sos_record(lat, lng, ts, reason="manual_sos")
    speak(f"S O S sent. Coordinates: {round(lat,5)} latitude, {round(lng,5)} longitude.")

def listen_for_command(timeout=6):
    """Listen using speech_recognition and return recognised lowercase text, or None."""
    try:
        with sr.Microphone() as source:
            recognizer.adjust_for_ambient_noise(source, duration=0.5)
            speak("Listening.")
            audio = recognizer.listen(source, timeout=timeout, phrase_time_limit=6)
        text = recognizer.recognize_google(audio)
        if text:
            text = text.lower()
            print("Recognized:", text)
            return text
    except sr.WaitTimeoutError:
        print("Listen timeout.")
    except sr.UnknownValueError:
        print("Could not understand audio.")
    except sr.RequestError as e:
        print("Speech recognition request error:", e)
    except Exception as e:
        print("listen_for_command error:", e)
    return None

def handle_command(cmd):
    """Handle voice command text."""
    if not cmd:
        return
    # SOS phrases
    for phrase in STRICT_SOS_PHRASES:
        if phrase in cmd:
            speak("SOS voice command detected.")
            send_sos()
            return
    # Location queries
    for kw in LOCATION_KEYWORDS:
        if kw in cmd:
            with latest_gps_lock:
                lat = latest_gps.get("lat")
                lng = latest_gps.get("lng")
                ts = latest_gps.get("timestamp")
            if lat is None or lng is None:
                speak("I do not have a GPS fix right now.")
                return
            speak(f"Current coordinates: latitude {round(lat,5)}, longitude {round(lng,5)}.")
            addr = reverse_geocode(lat, lng)
            if addr:
                speak(f"My best guess: {addr}")
                return
            name, vicinity = get_nearest_place(lat, lng)
            if name:
                speak(f"Nearest place: {name}. {vicinity or ''}")
            else:
                speak("No nearby place found.")
            return

# -------------------------
# GPIO buttons thread
# -------------------------
def btn_poll_thread():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BTN_LISTEN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_SOS, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("Button poll thread started.")
    try:
        while True:
            if GPIO.input(BTN_LISTEN) == GPIO.LOW:
                time.sleep(0.05)
                if GPIO.input(BTN_LISTEN) == GPIO.LOW:
                    print("Listen pressed")
                    try:
                        cmd = listen_for_command()
                        handle_command(cmd)
                    except Exception as e:
                        print("Listen handler error:", e)
                    while GPIO.input(BTN_LISTEN) == GPIO.LOW:
                        time.sleep(0.05)
            if GPIO.input(BTN_SOS) == GPIO.LOW:
                time.sleep(0.05)
                if GPIO.input(BTN_SOS) == GPIO.LOW:
                    print("SOS pressed")
                    try:
                        send_sos()
                    except Exception as e:
                        print("SOS handler error:", e)
                    while GPIO.input(BTN_SOS) == GPIO.LOW:
                        time.sleep(0.05)
            time.sleep(0.05)
    except Exception as e:
        print("Button thread exception:", e)
    finally:
        GPIO.cleanup()

# -------------------------
# BLE / iBeacon parsing & scanner
# -------------------------
def parse_ibeacon(manufacturer_data_bytes):
    """Return (uuid, major, minor, tx) or None."""
    try:
        # iBeacon payload is 23 bytes (0x02,0x15 + 16-byte UUID + 2-byte major + 2-byte minor + 1-byte tx)
        if not manufacturer_data_bytes or len(manufacturer_data_bytes) < 23:
            return None
        if manufacturer_data_bytes[0] != 0x02 or manufacturer_data_bytes[1] != 0x15:
            return None
        uuid_bytes = manufacturer_data_bytes[2:18]
        uuid = (
            uuid_bytes[0:4].hex() + "-" +
            uuid_bytes[4:6].hex() + "-" +
            uuid_bytes[6:8].hex() + "-" +
            uuid_bytes[8:10].hex() + "-" +
            uuid_bytes[10:16].hex()
        )
        major = int.from_bytes(manufacturer_data_bytes[18:20], "big")
        minor = int.from_bytes(manufacturer_data_bytes[20:22], "big")
        tx = int.from_bytes(manufacturer_data_bytes[22:23], "big", signed=True)
        return uuid.lower(), major, minor, tx
    except Exception:
        return None

def rssi_to_distance(rssi, tx_power=TX_POWER_DEFAULT, n=ENV_FACTOR):
    try:
        if rssi is None:
            return None
        distance = 10 ** ((tx_power - rssi) / (10 * n))
        return round(distance, 2)
    except Exception:
        return None

async def _ble_scanner_main(queue_out, stop_event):
    print("Beacon scanner async starting...")
    print(f"[SCAN] Searching for beacon (UUID {TARGET_UUID.upper()})...")
    rssi_buffers = {}
    while not stop_event.is_set():
        # Bleak detection via callback; create local callback
        def detection_callback(device, advertisement_data):
            try:
                mfd = advertisement_data.manufacturer_data
                if not mfd:
                    return
                # Prefer Apple company id 76 if present
                chosen_key = None
                for k in mfd.keys():
                    if isinstance(k, int) and k == 76:
                        chosen_key = k
                        break
                if chosen_key is None:
                    chosen_key = next(iter(mfd.keys()))
                md = mfd[chosen_key]
                parsed = parse_ibeacon(md)
                if not parsed:
                    return
                uuid, major, minor, tx = parsed
                uuid = uuid.lower()
                if uuid != TARGET_UUID.lower():
                    return
                # get rssi
                rssi_val = advertisement_data.rssi if advertisement_data.rssi is not None else device.rssi
                if rssi_val is None:
                    return
                # Buffer per (major, minor)
                key = (major, minor)
                if key not in rssi_buffers:
                    rssi_buffers[key] = deque(maxlen=RSSI_WINDOW)
                rssi_buffers[key].append(rssi_val)
                avg_rssi = sum(rssi_buffers[key]) / len(rssi_buffers[key])
                distance = rssi_to_distance(avg_rssi, tx_power=tx if tx is not None else TX_POWER_DEFAULT)
                event = {
                    "type": "beacon_seen",
                    "uuid": uuid,
                    "major": major,
                    "minor": minor,
                    "tx": tx,
                    "rssi": avg_rssi,
                    "distance_m": distance,
                    "timestamp": time.time()
                }
                try:
                    queue_out.put_nowait(event)
                except Exception:
                    pass
            except Exception as e:
                print("Beacon detection callback error:", e)

        scanner = BleakScanner(detection_callback)
        try:
            async with scanner:
                # run until stop_event set
                while not stop_event.is_set():
                    await asyncio.sleep(0.5)
        except Exception as e:
            print("Bleak scanner main loop error:", e)
            await asyncio.sleep(1)

def beacon_thread_fn(queue_out, stop_event):
    try:
        asyncio.run(_ble_scanner_main(queue_out, stop_event))
    except Exception as e:
        print("Beacon thread exception:", e)

# -------------------------
# Beacon event processor
# -------------------------
def classify_distance(distance):
    """Return state string based on distance (None -> 'unknown')"""
    if distance is None:
        return "unknown"
    try:
        d = float(distance)
    except Exception:
        return "unknown"
    if d < REACHED_THRESHOLD:
        return "reached"
    elif d < NEAR_THRESHOLD:
        return "near"
    elif d < MID_THRESHOLD:
        return "mid"
    else:
        return "far"

def beacon_event_processor():
    print("Beacon event processor started.")
    seen_cooldown = {}
    COOLDOWN_S = 3.0
    # We'll print scanning status periodically if no beacon seen
    last_scan_print = 0
    SCAN_PRINT_INTERVAL = 5.0
    # Rate limiting / summary state (per-beacon)
    last_print_time = {}
    last_status = {}
    detected_beacons = {}
    last_summary_time = time.time()

    while not beacon_stop_event.is_set():
        # if no events, periodically show scanning
        try:
            event = event_queue.get(timeout=0.5)
        except queue.Empty:
            # no event — occasionally show that scan is active
            if time.time() - last_scan_print >= SCAN_PRINT_INTERVAL:
                print("[SCAN] scanning for beacon...")
                last_scan_print = time.time()
            continue

        try:
            if event.get("type") == "beacon_seen":
                major = event.get("major")
                minor = event.get("minor")
                distance = event.get("distance_m")
                rssi = event.get("rssi")
                ts = event.get("timestamp")
                label = BEACON_LOCATIONS.get((major, minor), f"Beacon {major}.{minor}")
                key = (major, minor)

                # Only proceed for your single beacon (major=1,minor=1) - but keep mapping if others added.
                # If you want to completely ignore other beacons, you could `continue` here if key != (1,1)
                # For now we will show only the matched beacon(s) that are in BEACON_LOCATIONS

                # On first valid detection, print full details and announce detected
                with device_state_lock:
                    first_seen_local = device_state["first_seen"]
                if not first_seen_local:
                    print("[BEACON FOUND]")
                    print("UUID:", event.get("uuid").upper())
                    print("Major:", major)
                    print("Minor:", minor)
                    print("TX Power:", event.get("tx") if event.get("tx") is not None else TX_POWER_DEFAULT)
                    print("Initial RSSI (avg):", round(rssi, 2) if rssi is not None else "N/A")
                    # announce once
                    speak(f"{label} detected. Navigating toward the destination.")
                    with device_state_lock:
                        device_state["first_seen"] = True
                        device_state["last_state"] = "detected"
                        device_state["last_state_ts"] = time.time()
                        device_state["last_distance"] = distance
                        device_state["last_rssi"] = rssi

                # Update detected_beacons for summary and for closest-beacon logic
                detected_beacons[key] = {
                    'label': label,
                    'distance': distance,
                    'avg_rssi': rssi,
                    'timestamp': ts,
                }

                # Rate-limited printing logic (per-beacon)
                current_time = time.time()
                should_print = False
                reason = ""
                if key not in last_print_time:
                    should_print = True
                    reason = "NEW BEACON"
                elif key in last_status and last_status[key] != classify_distance(distance):
                    should_print = True
                    reason = f"STATUS CHANGE: {last_status[key]} -> {classify_distance(distance)}"
                elif current_time - last_print_time.get(key, 0) >= UPDATE_INTERVAL:
                    should_print = True
                    reason = "PERIODIC UPDATE"

                # If it's time to print, format and show
                if should_print:
                    dist_text = "unknown" if distance is None else f"{round(distance,2)} m"
                    print(f"[{time.strftime('%H:%M:%S')}] 📍 {label} ({reason})\n"
                          f"  └─ Beacon: Major {major}, Minor {minor}\n"
                          f"  └─ RSSI (avg): {round(rssi,1) if rssi is not None else 'N/A'} dBm\n"
                          f"  └─ Distance: {dist_text}\n")
                    last_print_time[key] = current_time

                # Update last_status for this beacon
                last_status[key] = classify_distance(distance)
                # Periodic summary print
                if time.time() - last_summary_time >= SUMMARY_INTERVAL:
                    print('\n' + '=' * 70)
                    print(f"BEACON SUMMARY [{time.strftime('%H:%M:%S')}]")
                    print('=' * 70)
                    if not detected_beacons:
                        print("No beacons detected")
                    else:
                        # find closest
                        closest = None
                        for k, v in detected_beacons.items():
                            if v['distance'] is None:
                                continue
                            if closest is None or (v['distance'] < detected_beacons[closest]['distance']):
                                closest = k
                        for k, v in sorted(detected_beacons.items()):
                            marker = '⭐' if k == closest else ' '
                            name = v.get('label', f"{k[0]}.{k[1]}")
                            d = v['distance'] if v['distance'] is not None else 'N/A'
                            print(f"{marker} {name:<20} | {d:>6} m | RSSI: {v['avg_rssi']:>5} dBm")
                    print('=' * 70 + '\n')
                    last_summary_time = time.time()

                # Decide whether to speak based on state transitions and cooldowns
                with device_state_lock:
                    prev_distance = device_state["last_distance"]
                    prev_state = device_state["last_state"]
                    last_state_ts = device_state["last_state_ts"]

                curr_state = classify_distance(distance)
                nowt = time.time()

                # Trend detection: approaching or moving away
                trend = None
                if prev_distance is not None and distance is not None:
                    # if distance decreased significantly
                    try:
                        if distance < prev_distance - 0.15:
                            trend = "approaching"
                        elif distance > prev_distance + 0.15:
                            trend = "moving_away"
                    except Exception:
                        trend = None

                # Choose announcements based on state and trend
                do_speak = False
                speak_text = None
                cooldown_ok = (nowt - last_state_ts) >= COOLDOWN_GENERAL

                if curr_state == "reached":
                    # Always announce reached when first entering this state (with longer cooldown)
                    if prev_state != "reached" or (nowt - last_state_ts) >= COOLDOWN_REACHED:
                        do_speak = True
                        speak_text = "Destination reached."
                        with device_state_lock:
                            device_state["last_state_ts"] = nowt
                elif curr_state == "near":
                    # Announce "almost there" once when entering near
                    if prev_state != "near" and (nowt - last_state_ts) >= COOLDOWN_NEAR:
                        do_speak = True
                        speak_text = "Almost there."
                        with device_state_lock:
                            device_state["last_state_ts"] = nowt
                elif curr_state == "mid":
                    # Use trend to say "getting closer" or "moving away"
                    if trend == "approaching" and cooldown_ok:
                        do_speak = True
                        speak_text = "You are getting closer."
                        with device_state_lock:
                            device_state["last_state_ts"] = nowt
                    elif trend == "moving_away" and cooldown_ok:
                        do_speak = True
                        speak_text = "You are moving away from the destination."
                        with device_state_lock:
                            device_state["last_state_ts"] = nowt
                elif curr_state == "far":
                    # Announce initial guidance for far state once
                    if prev_state not in ("far", "detected") and cooldown_ok:
                        do_speak = True
                        speak_text = "Beacon detected. Move forward toward the destination."
                        with device_state_lock:
                            device_state["last_state_ts"] = nowt
                # else unknown: do nothing

                # Update stored last_distance/state
                with device_state_lock:
                    device_state["last_distance"] = distance
                    device_state["last_rssi"] = rssi
                    device_state["last_state"] = curr_state

                if do_speak and speak_text:
                    speak(speak_text)

                # Push sighting to firebase (non-blocking)
                try:
                    with latest_gps_lock:
                        lat = latest_gps.get("lat")
                        lng = latest_gps.get("lng")
                        gps_ts = latest_gps.get("timestamp")
                    payload = {
                        "label": label,
                        "major": major,
                        "minor": minor,
                        "rssi": rssi,
                        "distance_m": distance,
                        "detected_at": datetime.datetime.fromtimestamp(ts, tz=timezone.utc).astimezone(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S"),
                        "gps_lat": lat,
                        "gps_lng": lng,
                        "gps_ts": gps_ts
                    }
                    threading.Thread(target=push_beacon_sighting, args=(payload,), daemon=True).start()
                except Exception as e:
                    print("Error pushing beacon sighting:", e)

        except Exception as e:
            print("Beacon event processor exception:", e)

    print("Beacon event processor exiting.")

# -------------------------
# Main
# -------------------------
def main():
    print("Starting gpstest_with_beacon.py")
    init_firebase()

    # Start GPS thread
    gps_t = threading.Thread(target=gps_thread_fn, daemon=True)
    gps_t.start()

    # Start button poll thread
    btn_t = threading.Thread(target=btn_poll_thread, daemon=True)
    btn_t.start()

    # Start beacon scanner thread
    scanner_t = threading.Thread(target=beacon_thread_fn, args=(event_queue, beacon_stop_event), daemon=True)
    scanner_t.start()

    # Start beacon event processor
    processor_t = threading.Thread(target=beacon_event_processor, daemon=True)
    processor_t.start()

    speak("Prime is ready. Beacon scanner active.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt — shutting down.")
    finally:
        beacon_stop_event.set()
        time.sleep(0.5)
        speak("Shutting down.")
        # Threads are daemon; process will exit

if __name__ == "__main__":
    main()
