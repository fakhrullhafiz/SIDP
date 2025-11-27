#!/usr/bin/env python3
"""
gpstest_with_beacon_fixed.py
Fixed version with better beacon detection and debugging
"""

import os
import time
import json
import math
import queue
import threading
import traceback
from collections import deque
from datetime import datetime, timezone
from zoneinfo import ZoneInfo

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

# ============================
# CREDENTIALS
# ============================
API_KEY = "AIzaSyC3wbCxpdu5gBjjixsDlIR1N-hFSgR2xp4"
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUDRATE = 9600
FIREBASE_CRED_PATH = "/home/coe/firebase/firebase-key.json"
FIREBASE_DB_URL = "https://sidp-5fcae-default-rtdb.asia-southeast1.firebasedatabase.app"

# ==============================
# BEACON CONFIG
# ==============================
TARGET_UUID = "fda50693-a4e2-4fb1-afcf-c6eb07647825".lower()
TX_POWER_DEFAULT = -59
RSSI_WINDOW = 8
ENV_FACTOR = 2.5

# Other config
TTS_RATE = 160
BTN_LISTEN = 24
BTN_SOS = 23
GPS_PUSH_INTERVAL = 5

BEACON_LOCATIONS = {
    (1, 1): "Demo Start",
    (1, 2): "Demo End",
}

STRICT_SOS_PHRASES = [
    "sos", "help me", "send help", "send sos", "i need help",
    "emergency", "please help me"
]

LOCATION_KEYWORDS = ["location", "where", "where am i", "where are we"]

# -------------------------
# Globals
# -------------------------
latest_gps_lock = threading.Lock()
latest_gps = {"lat": None, "lng": None, "timestamp": None}
event_queue = queue.Queue()
tts_engine = pyttsx3.init()
tts_engine.setProperty("rate", TTS_RATE)
tts_lock = threading.Lock()
firebase_enabled = False
beacon_stop_event = threading.Event()
recognizer = sr.Recognizer()

# Debug flag - set to False to reduce console spam
DEBUG_BEACONS = True

# -------------------------
# Utility functions
# -------------------------
def now_iso():
    return datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S")

def speak(text):
    """Thread-safe TTS"""
    with tts_lock:
        try:
            print(f"[TTS] {text}")
            tts_engine.say(text)
            tts_engine.runAndWait()
        except Exception as e:
            print(f"TTS error: {e}")

# -------------------------
# Firebase
# -------------------------
def init_firebase():
    global firebase_enabled
    try:
        if not os.path.exists(FIREBASE_CRED_PATH):
            print(f"Firebase credential file not found: {FIREBASE_CRED_PATH}")
            firebase_enabled = False
            return
        cred = credentials.Certificate(FIREBASE_CRED_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
        firebase_enabled = True
        print("✓ Firebase initialized.")
    except Exception as e:
        print(f"Firebase init failed: {e}")
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
        print(f"push_live_location error: {e}")

def push_sos_record(lat, lng, ts_iso, reason=None):
    if not firebase_enabled:
        return
    try:
        sos_root = db.reference("sosDB")
        ts_local = datetime.now(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S")
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
        print(f"push_sos_record error: {e}")

def push_beacon_sighting(beacon_payload):
    if not firebase_enabled:
        return
    try:
        ref = db.reference("/beacon_sightings")
        ref.push(beacon_payload)
    except Exception as e:
        print(f"push_beacon_sighting error: {e}")

# -------------------------
# GPS
# -------------------------
def open_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print(f"✓ Opened serial: {SERIAL_PORT}")
        return ser
    except Exception as e:
        print(f"Failed to open serial: {e}")
        return None

def parse_nmea_line(line):
    try:
        msg = pynmea2.parse(line)
        if hasattr(msg, "latitude") and hasattr(msg, "longitude"):
            if msg.latitude and msg.longitude:
                return msg.latitude, msg.longitude
    except:
        pass
    return None, None

def gps_thread_fn():
    global latest_gps
    ser = open_serial()
    if ser is None:
        print("GPS serial unavailable; exiting GPS thread.")
        return
    last_push = 0
    print("✓ GPS thread running")
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
                    print(f"GPS: {lat:.5f}, {lng:.5f}")
        except Exception as e:
            print(f"GPS thread exception: {e}")
            time.sleep(0.5)

# -------------------------
# Geocoding
# -------------------------
def reverse_geocode(lat, lng):
    try:
        url = f"https://maps.googleapis.com/maps/api/geocode/json?latlng={lat},{lng}&key={API_KEY}"
        r = requests.get(url, timeout=4)
        data = r.json()
        if data.get("status") == "OK" and data.get("results"):
            return data["results"][0].get("formatted_address")
    except Exception as e:
        print(f"reverse_geocode error: {e}")
    return None

def get_nearest_place(lat, lng):
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
        print(f"get_nearest_place error: {e}")
    return None, None

# -------------------------
# SOS & Voice
# -------------------------
def send_sos():
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
    try:
        with sr.Microphone() as source:
            recognizer.adjust_for_ambient_noise(source, duration=0.5)
            speak("Listening.")
            audio = recognizer.listen(source, timeout=timeout, phrase_time_limit=6)
        text = recognizer.recognize_google(audio)
        if text:
            text = text.lower()
            print(f"Recognized: {text}")
            return text
    except sr.WaitTimeoutError:
        print("Listen timeout.")
    except sr.UnknownValueError:
        print("Could not understand audio.")
    except sr.RequestError as e:
        print(f"Speech recognition request error: {e}")
    except Exception as e:
        print(f"listen_for_command error: {e}")
    return None

def handle_command(cmd):
    if not cmd:
        return
    for phrase in STRICT_SOS_PHRASES:
        if phrase in cmd:
            speak("SOS voice command detected.")
            send_sos()
            return
    for kw in LOCATION_KEYWORDS:
        if kw in cmd:
            with latest_gps_lock:
                lat = latest_gps.get("lat")
                lng = latest_gps.get("lng")
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
# GPIO Buttons
# -------------------------
def btn_poll_thread():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BTN_LISTEN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_SOS, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("✓ Button poll thread started")
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
                        print(f"Listen handler error: {e}")
                    while GPIO.input(BTN_LISTEN) == GPIO.LOW:
                        time.sleep(0.05)
            if GPIO.input(BTN_SOS) == GPIO.LOW:
                time.sleep(0.05)
                if GPIO.input(BTN_SOS) == GPIO.LOW:
                    print("SOS pressed")
                    try:
                        send_sos()
                    except Exception as e:
                        print(f"SOS handler error: {e}")
                    while GPIO.input(BTN_SOS) == GPIO.LOW:
                        time.sleep(0.05)
            time.sleep(0.05)
    except Exception as e:
        print(f"Button thread exception: {e}")
    finally:
        GPIO.cleanup()

# -------------------------
# BLE iBeacon
# -------------------------
def parse_ibeacon(manufacturer_data_bytes):
    """Parse iBeacon data. Returns (uuid, major, minor, tx) or None."""
    try:
        if len(manufacturer_data_bytes) < 25:
            return None
        # iBeacon starts with 0x02 0x15
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
        return uuid, major, minor, tx
    except Exception as e:
        if DEBUG_BEACONS:
            print(f"iBeacon parse error: {e}")
        return None

def rssi_to_distance(rssi, tx_power=TX_POWER_DEFAULT, n=ENV_FACTOR):
    try:
        return 10 ** ((tx_power - rssi) / (10 * n))
    except Exception:
        return None

# New: Wrapper to run async scanner in thread with proper event loop
def beacon_scanner_thread_fn(queue_out, stop_event):
    """Run BLE scanner in dedicated thread with its own event loop."""
    print("✓ Beacon scanner thread starting...")
    
    # Create a new event loop for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(ble_scanner_async(queue_out, stop_event))
    except Exception as e:
        print(f"Beacon scanner thread error: {e}")
        traceback.print_exc()
    finally:
        loop.close()
        print("Beacon scanner thread exited")

async def ble_scanner_async(queue_out, stop_event):
    """Async BLE scanner with enhanced detection."""
    print("✓ Starting BLE scan...")
    rssi_buffers = {}
    scan_count = 0
    
    def detection_callback(device, advertisement_data):
        nonlocal scan_count
        scan_count += 1
        
        try:
            # Debug: Show all BLE devices detected (first 10 only)
            if DEBUG_BEACONS and scan_count <= 10:
                print(f"BLE Device detected: {device.name or 'Unknown'} [{device.address}]")
            
            mfd = advertisement_data.manufacturer_data
            if not mfd:
                return
            
            # Debug: Show manufacturer data
            if DEBUG_BEACONS and scan_count <= 10:
                for key in mfd.keys():
                    print(f"  Manufacturer ID: {key}, Data length: {len(mfd[key])}")
            
            # Try to find Apple (76) or any manufacturer data
            chosen_key = None
            for k in mfd.keys():
                if isinstance(k, int) and k == 76:  # Apple company ID
                    chosen_key = k
                    break
            
            if chosen_key is None:
                # Try first available key
                chosen_key = next(iter(mfd.keys()))
            
            md = mfd[chosen_key]
            
            # Try to parse as iBeacon
            parsed = parse_ibeacon(md)
            if not parsed:
                if DEBUG_BEACONS and scan_count <= 10:
                    print(f"  Not an iBeacon (data: {md[:6].hex() if len(md) >= 6 else md.hex()})")
                return
            
            uuid, major, minor, tx = parsed
            uuid = uuid.lower()
            
            # Check UUID match
            if uuid != TARGET_UUID:
                if DEBUG_BEACONS:
                    print(f"  iBeacon found but wrong UUID: {uuid} (want {TARGET_UUID})")
                return
            
            # Get RSSI
            rssi_val = advertisement_data.rssi if advertisement_data.rssi is not None else device.rssi
            if rssi_val is None:
                return
            
            # SUCCESS! Found matching beacon
            print(f"✓ BEACON DETECTED: UUID={uuid}, Major={major}, Minor={minor}, RSSI={rssi_val}, TX={tx}")
            
            # Buffer RSSI for smoothing
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
            except:
                pass
                
        except Exception as e:
            print(f"Detection callback error: {e}")
            if DEBUG_BEACONS:
                traceback.print_exc()
    
    # Continuous scanning
    scanner = BleakScanner(detection_callback)
    
    try:
        print("✓ BLE Scanner active, waiting for beacons...")
        await scanner.start()
        
        # Keep scanning until stop event
        while not stop_event.is_set():
            await asyncio.sleep(1)
            if scan_count % 10 == 0 and scan_count > 0:
                print(f"BLE scan active... ({scan_count} devices seen)")
        
        await scanner.stop()
        print("BLE Scanner stopped")
        
    except Exception as e:
        print(f"BLE scanner error: {e}")
        traceback.print_exc()

# -------------------------
# Beacon Event Processor
# -------------------------
def beacon_event_processor():
    print("✓ Beacon event processor started")
    seen_cooldown = {}
    COOLDOWN_S = 3.0
    
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
                rssi = event.get("rssi")
                ts = event.get("timestamp")
                
                label = BEACON_LOCATIONS.get((major, minor), f"Beacon {major}.{minor}")
                
                if distance is None:
                    dist_text = ""
                elif distance < 0.5:
                    dist_text = "very close"
                elif distance < 2:
                    dist_text = f"about {round(distance, 1)} meters"
                else:
                    dist_text = f"about {round(distance, 0)} meters"
                
                # Cooldown check
                key = (major, minor)
                nowt = time.time()
                last = seen_cooldown.get(key, 0)
                
                if nowt - last >= COOLDOWN_S:
                    speak(f"{label}. {dist_text}")
                    seen_cooldown[key] = nowt
                
                # Push to Firebase
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
                        "detected_at": datetime.fromtimestamp(ts, tz=timezone.utc).astimezone(malaysia_tz).strftime("%Y/%m/%d %H:%M:%S"),
                        "gps_lat": lat,
                        "gps_lng": lng,
                        "gps_ts": gps_ts
                    }
                    threading.Thread(target=push_beacon_sighting, args=(payload,), daemon=True).start()
                except Exception as e:
                    print(f"Error pushing beacon sighting: {e}")
                    
        except Exception as e:
            print(f"Beacon event processor exception: {e}")
            traceback.print_exc()
    
    print("Beacon event processor exiting")

# -------------------------
# Main
# -------------------------
def main():
    print("=" * 60)
    print("Starting GPS + Beacon System")
    print("=" * 60)
    
    init_firebase()
    
    # Start GPS thread
    gps_t = threading.Thread(target=gps_thread_fn, daemon=True, name="GPS-Thread")
    gps_t.start()
    
    # Start button thread
    btn_t = threading.Thread(target=btn_poll_thread, daemon=True, name="Button-Thread")
    btn_t.start()
    
    # Start beacon scanner thread (NEW VERSION)
    scanner_t = threading.Thread(
        target=beacon_scanner_thread_fn, 
        args=(event_queue, beacon_stop_event), 
        daemon=True,
        name="Beacon-Scanner-Thread"
    )
    scanner_t.start()
    
    # Start beacon event processor
    processor_t = threading.Thread(target=beacon_event_processor, daemon=True, name="Beacon-Processor-Thread")
    processor_t.start()
    
    # Wait a moment for threads to initialize
    time.sleep(2)
    
    speak("Prime is ready. GPS and Beacon scanner active.")
    print("\n✓ All systems operational")
    print("=" * 60)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nKeyboardInterrupt – shutting down...")
    finally:
        beacon_stop_event.set()
        time.sleep(1)
        speak("Shutting down.")
        print("Shutdown complete")

if __name__ == "__main__":
    main()
