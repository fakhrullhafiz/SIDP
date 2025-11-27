#!/usr/bin/env python3
"""
gpstest_with_beacon.py

Integrated GPS + Beacon (iBeacon scanner) script for Raspberry Pi 4B.

Key behaviour:
 - Reads GPS from /dev/serial0 (pynmea2)
 - Pushes live GPS to Firebase Realtime DB (optional)
 - Handles two GPIO buttons: Listen and SOS (keeps existing behaviours)
 - Runs a Bleak-based iBeacon scanner in a separate thread/asyncio loop
 - Scanner posts events to a queue; main thread speaks beacon detections and logs to Firebase.

Adapt / tune constants below to your setup (GPIO pins, Beacon UUIDs, Firebase paths, etc).
"""

import os
import sys
import time
import json
import threading
import queue
import math
from collections import deque
from datetime import datetime, timezone
from zoneinfo import ZoneInfo
import traceback

# Bluetooth scanning
import asyncio
from bleak import BleakScanner

# GPS parsing
import serial
import pynmea2

# TTS
import pyttsx3

# Networking / Firebase (optional)
import requests
try:
    import firebase_admin
    from firebase_admin import credentials, db
    FIREBASE_AVAILABLE = True
except Exception:
    FIREBASE_AVAILABLE = False

# GPIO
try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except Exception:
    RPI_AVAILABLE = False

# -------------------------
# Configuration (edit these)
# -------------------------
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUDRATE = 9600

# Firebase
FIREBASE_CRED_PATH = os.getenv("FIREBASE_CRED", "/home/coe/firebase/firebase-key.json")
FIREBASE_DB_URL = os.getenv("FIREBASE_DB_URL", None)  # e.g., "https://project-id.firebaseio.com/"

# TTS
TTS_RATE = 160

# GPIO pins
BTN_LISTEN = 24
BTN_SOS = 23

# Beacon scanning config (iBeacon)
TARGET_UUID = "e2c56db5-dffb-48d2-b060-d0f5a71096e0"  # replace with your beacon UUID (lowercase, no braces)
TX_POWER_DEFAULT = -59
RSSI_WINDOW = 6
ENV_FACTOR = 2.0  # path-loss exponent, adjust for environment

# Beacon locations mapping (major, minor) -> label
BEACON_LOCATIONS = {
    (1, 1): "Demo Start",
    (1, 2): "Demo End",
    # add more (major, minor): "Label"
}

# How often to push live GPS to Firebase (seconds)
GPS_PUSH_INTERVAL = 5

# -------------------------
# Globals & shared state
# -------------------------
zone = ZoneInfo("Asia/Kuala_Lumpur")

# Latest GPS (thread-shared) — updated by GPS thread and read by beacon processor for logging
latest_gps_lock = threading.Lock()
latest_gps = {"lat": None, "lng": None, "timestamp": None}

# Event queue from beacon scanner -> main
event_queue = queue.Queue()

# TTS engine (single instance)
tts_engine = pyttsx3.init()
tts_engine.setProperty("rate", TTS_RATE)
tts_lock = threading.Lock()  # ensure only one thread uses pyttsx3 at once

# Firebase initialization flag
firebase_enabled = False

# Beacon scanner control
beacon_stop_event = threading.Event()

# -------------------------
# Helper utilities
# -------------------------
def speak(text):
    """Speak text using single shared pyttsx3 engine (thread-safe)."""
    with tts_lock:
        try:
            print("[TTS] " + text)
            tts_engine.say(text)
            tts_engine.runAndWait()
        except Exception as e:
            print("TTS error:", e)

def now_ts():
    return datetime.now(zone).isoformat()

# -------------------------
# Firebase helpers
# -------------------------
def init_firebase():
    global firebase_enabled
    if not FIREBASE_AVAILABLE:
        print("firebase_admin not available; Firebase features disabled.")
        firebase_enabled = False
        return

    if not FIREBASE_DB_URL or not os.path.exists(FIREBASE_CRED_PATH):
        print("Firebase DB URL or credential file missing. Firebase disabled.")
        firebase_enabled = False
        return

    try:
        cred = credentials.Certificate(FIREBASE_CRED_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
        firebase_enabled = True
        print("Firebase initialized.")
    except Exception as e:
        print("Failed to initialize Firebase:", e)
        firebase_enabled = False

def push_live_location_to_firebase(lat, lng, ts_iso):
    if not firebase_enabled:
        return
    try:
        ref = db.reference("/gpsDB/Live")
        ref.set({
            "latitude": lat,
            "longitude": lng,
            "timestamp": ts_iso
        })
    except Exception as e:
        print("Firebase push_live_location error:", e)

def push_beacon_sighting_to_firebase(beacon_event):
    if not firebase_enabled:
        return
    try:
        ref = db.reference("/beacon_sightings")
        # push a new child
        ref.push(beacon_event)
    except Exception as e:
        print("Firebase push_beacon_sighting error:", e)

# -------------------------
# GPS reading
# -------------------------
def get_gps_coordinates_from_serial(ser):
    """
    Read lines from serial and parse NMEA. Return (lat, lng) or (None, None).
    Non-blocking read: read a line and try parse.
    """
    try:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            return None, None
        if line.startswith("$"):
            try:
                msg = pynmea2.parse(line)
            except pynmea2.nmea.ChecksumError:
                return None, None
            except Exception:
                return None, None
            # Prefer GGA or RMC
            if hasattr(msg, "latitude") and hasattr(msg, "longitude"):
                if msg.latitude and msg.longitude:
                    return msg.latitude, msg.longitude
        return None, None
    except Exception as e:
        # sometimes serial I/O errors happen; swallow and continue
        # print("GPS read error:", e)
        return None, None

def gps_live_thread():
    """
    Thread that continuously reads GPS and pushes live locations to Firebase.
    """
    global latest_gps
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print("GPS serial opened:", SERIAL_PORT)
    except Exception as e:
        print("Failed to open GPS serial:", e)
        return

    last_push = 0
    try:
        while True:
            lat, lng = get_gps_coordinates_from_serial(ser)
            if lat is not None and lng is not None:
                ts_iso = now_ts()
                with latest_gps_lock:
                    latest_gps["lat"] = lat
                    latest_gps["lng"] = lng
                    latest_gps["timestamp"] = ts_iso
                # push periodically
                if time.time() - last_push >= GPS_PUSH_INTERVAL:
                    push_live_location_to_firebase(lat, lng, ts_iso)
                    last_push = time.time()
            time.sleep(0.1)
    except Exception as e:
        print("GPS thread exception:", e)
        traceback.print_exc()

# -------------------------
# Existing button & speech handling (simplified)
# -------------------------
def btn_poll_thread(listen_callback, sos_callback):
    """
    Poll two GPIO buttons. When listen button pressed, call listen_callback().
    When SOS button pressed, call sos_callback().
    """
    if not RPI_AVAILABLE:
        print("RPi GPIO not available; button functionality disabled.")
        return

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BTN_LISTEN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_SOS, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print("Button poll thread started.")
    try:
        while True:
            if GPIO.input(BTN_LISTEN) == GPIO.LOW:
                # Debounce short press
                time.sleep(0.05)
                if GPIO.input(BTN_LISTEN) == GPIO.LOW:
                    print("Listen button pressed")
                    try:
                        listen_callback()
                    except Exception as e:
                        print("listen_callback error:", e)
                    # wait for release
                    while GPIO.input(BTN_LISTEN) == GPIO.LOW:
                        time.sleep(0.05)

            if GPIO.input(BTN_SOS) == GPIO.LOW:
                time.sleep(0.05)
                if GPIO.input(BTN_SOS) == GPIO.LOW:
                    print("SOS button pressed")
                    try:
                        sos_callback()
                    except Exception as e:
                        print("sos_callback error:", e)
                    while GPIO.input(BTN_SOS) == GPIO.LOW:
                        time.sleep(0.05)

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Button poll thread exiting (KeyboardInterrupt).")
    except Exception as e:
        print("Button poll thread exception:", e)
    finally:
        GPIO.cleanup()

# Placeholder listen & sos callbacks (reuse your project's actual implementations)
def listen_for_command():
    # Minimal placeholder - implement your existing speech_recognition logic here
    speak("Listening (placeholder).")
    # In your real code you'd capture audio and return recognized text or None
    return None

def send_sos():
    # Minimal placeholder - implement your existing SOS sending logic here
    speak("SOS triggered (placeholder).")
    # Reuse your existing push_sos(...) if present in your original script.

# -------------------------
# Beacon parsing & scanner (iBeacon)
# -------------------------
def parse_ibeacon(manufacturer_data: bytes):
    """
    Parse iBeacon manufacturer data bytes and return (uuid_str, major, minor, tx_power)
    Expect Apple Manufacturer data format: [0x02, 0x15] prefix after company ID.
    manufacturer_data: raw bytes
    """
    try:
        # manufacturer_data should be bytes beginning with 0x02 0x15
        if len(manufacturer_data) < 25:
            return None
        if manufacturer_data[0] != 0x02 or manufacturer_data[1] != 0x15:
            return None
        uuid_bytes = manufacturer_data[2:18]
        uuid = (
            uuid_bytes[0:4].hex() + "-" +
            uuid_bytes[4:6].hex() + "-" +
            uuid_bytes[6:8].hex() + "-" +
            uuid_bytes[8:10].hex() + "-" +
            uuid_bytes[10:16].hex()
        )
        major = int.from_bytes(manufacturer_data[18:20], byteorder="big")
        minor = int.from_bytes(manufacturer_data[20:22], byteorder="big")
        tx = int.from_bytes(manufacturer_data[22:23], byteorder="big", signed=True)
        return uuid, major, minor, tx
    except Exception:
        return None

def rssi_to_distance(rssi, tx_power=TX_POWER_DEFAULT, n=ENV_FACTOR):
    """
    Estimate distance (m) from RSSI using the log-distance path loss model.
    d = 10 ^ ((tx - rssi) / (10 * n))
    """
    try:
        return 10 ** ((tx_power - rssi) / (10 * n))
    except Exception:
        return None

async def _ble_scanner_main(queue_out: queue.Queue, stop_event: threading.Event):
    """
    Async Bleak scanner main. Puts events into queue_out dicts:
       {"type": "beacon_seen", "uuid":..., "major":..., "minor":..., "rssi":..., "distance":..., "timestamp":...}
    """
    print("Beacon scanner (async) starting...")
    rssi_buffers = {}  # key -> deque for smoothing
    last_reported_status = {}  # key -> (status, timestamp)
    UPDATE_INTERVAL = 2.0  # seconds between spoken updates per-beacon by scanner logic (we still push events each time)
    last_update_time = {}

    def detection_callback(device, advertisement_data):
        # advertisement_data.manufacturer_data is a dict keyed by company id (int) -> bytes
        try:
            mfd = advertisement_data.manufacturer_data
            if not mfd:
                return
            # Apple company id is 0x004C (76) - but bleak returns keys as ints
            apple_key = None
            for k in mfd.keys():
                # manufacturer_data entry is bytes-like
                if isinstance(k, int):
                    # 76 indicates Apple company id; but some beacons may use other ids — we attempt apple first
                    if k == 76:
                        apple_key = k
                        break
            if apple_key is None:
                # pick first manufacturer_data entry and try parse
                apple_key = next(iter(mfd.keys()))
            md_bytes = mfd[apple_key]
            parsed = parse_ibeacon(md_bytes)
            if not parsed:
                return
            uuid, major, minor, tx = parsed
            uuid = uuid.lower()
            if uuid != TARGET_UUID.lower():
                # ignore beacons not matching target UUID
                return

            key = (major, minor)
            rssi = advertisement_data.rssi if advertisement_data.rssi is not None else device.rssi
            if rssi is None:
                return

            # smoothing
            if key not in rssi_buffers:
                rssi_buffers[key] = deque(maxlen=RSSI_WINDOW)
            rssi_buffers[key].append(rssi)
            avg_rssi = sum(rssi_buffers[key]) / len(rssi_buffers[key])
            distance = rssi_to_distance(avg_rssi, tx_power=tx if tx is not None else TX_POWER_DEFAULT)
            ts = time.time()
            # prepare event
            event = {
                "type": "beacon_seen",
                "uuid": uuid,
                "major": major,
                "minor": minor,
                "tx_power": tx,
                "rssi": avg_rssi,
                "distance_m": distance,
                "timestamp": ts
            }
            # Put event to queue for main thread to handle
            try:
                queue_out.put_nowait(event)
            except Exception:
                # if queue full or other issue, ignore
                pass
        except Exception as e:
            # Avoid raising inside callback
            print("Beacon detection callback error:", e)

    scanner = BleakScanner(detection_callback)
    try:
        async with scanner:
            # run until stop_event set
            while not stop_event.is_set():
                await asyncio.sleep(0.5)
    except Exception as e:
        print("Bleak scanner error:", e)
    finally:
        print("Beacon scanner stopping.")

def beacon_thread_fn(queue_out: queue.Queue, stop_event: threading.Event):
    """Thread target that runs the asyncio-based Bleak scanner."""
    # Each thread must create/run its own asyncio loop via asyncio.run
    try:
        asyncio.run(_ble_scanner_main(queue_out, stop_event))
    except Exception as e:
        print("Beacon thread exception:", e)

# -------------------------
# Beacon event processor (main-thread safe)
# -------------------------
def beacon_event_processor_thread():
    """
    Consume events from event_queue and handle them in main-thread context.
    This function will:
     - Speak a short phrase (e.g., 'Arrived at Demo Start') for a detection.
     - Optionally push sightings to Firebase including latest GPS coords.
    """
    print("Beacon event processor started.")
    seen_cooldown = {}  # (major, minor) -> last_spoken_ts to avoid repeating too often
    COOLDOWN = 3.0  # seconds between spoken notifications for same beacon

    while not beacon_stop_event.is_set():
        try:
            event = event_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        try:
            if event.get("type") == "beacon_seen":
                major = event.get("major")
                minor = event.get("minor")
                distance = event.get("distance_m")
                avg_rssi = event.get("rssi")
                ts_iso = datetime.fromtimestamp(event.get("timestamp"), tz=timezone.utc).astimezone(zone).isoformat()

                label = BEACON_LOCATIONS.get((major, minor), f"Beacon {major}.{minor}")
                # round distance sensibly
                dist_text = ""
                if distance is None:
                    dist_text = ""
                else:
                    if distance < 0.5:
                        dist_text = "very close"
                    elif distance < 2:
                        dist_text = f"about {round(distance, 1)} meters"
                    else:
                        dist_text = f"about {round(distance, 0)} meters"

                # Avoid repeating too often
                key = (major, minor)
                nowt = time.time()
                last = seen_cooldown.get(key, 0)
                if nowt - last >= COOLDOWN:
                    spoken = f"{label}. {dist_text}."
                    speak(spoken)
                    seen_cooldown[key] = nowt

                # push to firebase asynchronously (fire-and-forget)
                try:
                    lat = None
                    lng = None
                    gps_ts = None
                    with latest_gps_lock:
                        lat = latest_gps.get("lat")
                        lng = latest_gps.get("lng")
                        gps_ts = latest_gps.get("timestamp")
                    payload = {
                        "label": label,
                        "major": major,
                        "minor": minor,
                        "rssi": avg_rssi,
                        "distance_m": distance,
                        "detected_at": ts_iso,
                        "gps_lat": lat,
                        "gps_lng": lng,
                        "gps_ts": gps_ts
                    }
                    # non-blocking push: use a thread
                    threading.Thread(target=push_beacon_sighting_to_firebase, args=(payload,), daemon=True).start()
                except Exception as e:
                    print("Error pushing beacon sighting to firebase:", e)

        except Exception as e:
            print("Beacon processor exception:", e)
            traceback.print_exc()

    print("Beacon event processor exiting.")

# -------------------------
# Main program
# -------------------------
def main():
    print("Starting gpstest_with_beacon.py")
    init_firebase()

    # Start GPS thread
    gps_thread = threading.Thread(target=gps_live_thread, daemon=True)
    gps_thread.start()

    # Start button poll thread (if you want buttons active)
    btn_thread = threading.Thread(target=btn_poll_thread, args=(listen_for_command, send_sos), daemon=True)
    btn_thread.start()

    # Start beacon scanner thread
    scanner_thread = threading.Thread(target=beacon_thread_fn, args=(event_queue, beacon_stop_event), daemon=True)
    scanner_thread.start()

    # Start beacon event processor (runs in its own thread but uses main-thread-safe TTS)
    processor_thread = threading.Thread(target=beacon_event_processor_thread, daemon=True)
    processor_thread.start()

    # Announce ready
    speak("Prime is ready. Beacon scanner active.")

    try:
        # Main thread can idle, but keep process alive and do lightweight tasks.
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down...")
    finally:
        # signal beacon scanner & processor to stop
        beacon_stop_event.set()
        # give threads a moment to finish
        time.sleep(1)
        speak("Shutting down.")
        # No explicit join since threads are daemonic; allow process to exit.

if __name__ == "__main__":
    main()
