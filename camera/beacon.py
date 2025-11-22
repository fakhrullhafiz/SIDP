#!/usr/bin/env python3
"""
BLE iBeacon Scanner using Bleak (Async)
Vision-Impaired Navigation System - 2 Beacon Setup
OPTIMIZED: Rate-limited output for easier monitoring
"""

import asyncio
from bleak import BleakScanner
from collections import deque
import time
import math

# ==============================
# CONFIG FOR YOUR BEACONS
# ==============================
TARGET_UUID = "fda50693-a4e2-4fb1-afcf-c6eb07647825".lower()

TX_POWER_DEFAULT = -59      # Your beacons' power setting
RSSI_WINDOW = 8             # Number of readings to average (smoother)
ENV_FACTOR = 2.5            # Path-loss exponent (2.0=free space, 2.5=indoor, 3.0=complex indoor)

# OUTPUT RATE LIMITING
UPDATE_INTERVAL = 3.0       # Seconds between updates for same beacon (adjust this: 2, 3, 5, 10)
SUMMARY_INTERVAL = 10.0     # Seconds between summary prints

rssi_buffers = {}           # Per-device RSSI smoothing buffer
last_status = {}            # Track last status to detect changes
last_print_time = {}        # Track when we last printed each beacon
last_summary_time = 0       # Track summary prints

# ==============================
# BEACON LOCATION MAPPING
# ==============================
BEACON_LOCATIONS = {
    (1, 1): {
        "name": "Main Entrance",
        "description": "Building entrance, elevator ahead"
    },
    (1, 2): {
        "name": "Elevator Area",
        "description": "Elevator lobby, press call button"
    },
    # Ready for Beacon 3 when you add it:
    # (1, 3): {
    #     "name": "Exit Door",
    #     "description": "Exit to parking area"
    # },
}


# ---------------------------------------------------------
# Distance Estimation (Indoor Optimized)
# ---------------------------------------------------------
def estimate_distance(rssi, tx_power, n=ENV_FACTOR):
    """
    Calculate distance using RSSI and path-loss formula
    Distance (m) = 10 ^ ((TX_power - RSSI) / (10 * n))
    """
    if rssi is None:
        return None
    
    ratio_db = tx_power - rssi
    distance = 10 ** (ratio_db / (10 * n))
    return round(distance, 2)


# ---------------------------------------------------------
# Get human-readable distance status
# ---------------------------------------------------------
def get_status(distance):
    """Convert distance to proximity status"""
    if distance is None:
        return "Unknown"
    if distance < 1.5:
        return "Arrived"         # Very close, user has reached location
    if distance < 3:
        return "Very Close"      # Within 1-3 meters
    if distance < 5:
        return "Near"            # Within 3-5 meters
    if distance < 8:
        return "Moderate"        # Within 5-8 meters
    return "Far"                 # Beyond 8 meters


# ---------------------------------------------------------
# Get location info from major/minor
# ---------------------------------------------------------
def get_location_info(major, minor):
    """Look up beacon location from major/minor values"""
    key = (major, minor)
    if key in BEACON_LOCATIONS:
        return BEACON_LOCATIONS[key]
    else:
        return {
            "name": f"Unknown Location",
            "description": f"Beacon {major}/{minor} not configured"
        }


# ---------------------------------------------------------
# Parse iBeacon advertisement data
# ---------------------------------------------------------
def parse_ibeacon(manufacturer_data):
    """
    Extract iBeacon data from BLE advertisement
    Returns: (uuid, major, minor, tx_power) or None
    """
    if not manufacturer_data:
        return None

    for company_id, data in manufacturer_data.items():
        # Apple company ID for iBeacon = 0x004C (76 decimal)
        if company_id == 76 and len(data) >= 23:
            # Check iBeacon prefix (0x02 0x15)
            if data[0] == 0x02 and data[1] == 0x15:
                
                # Extract UUID (bytes 2-17)
                uuid_bytes = data[2:18]
                uuid = (
                    uuid_bytes[0:4].hex() + "-" +
                    uuid_bytes[4:6].hex() + "-" +
                    uuid_bytes[6:8].hex() + "-" +
                    uuid_bytes[8:10].hex() + "-" +
                    uuid_bytes[10:16].hex()
                ).lower()

                # Extract Major (bytes 18-19)
                major = int.from_bytes(data[18:20], "big")
                
                # Extract Minor (bytes 20-21)
                minor = int.from_bytes(data[20:22], "big")
                
                # Extract TX Power (byte 22, signed)
                tx = int.from_bytes(data[22:23], "big", signed=True)

                return uuid, major, minor, tx

    return None


# ---------------------------------------------------------
# Find closest beacon
# ---------------------------------------------------------
def get_closest_beacon(all_beacons):
    """
    Given dictionary of beacons with distances, return the closest one
    Returns: (beacon_key, beacon_data) or None
    """
    if not all_beacons:
        return None
    
    closest = min(all_beacons.items(), key=lambda x: x[1]['distance'] if x[1]['distance'] else 999)
    return closest if closest[1]['distance'] is not None else None


# ---------------------------------------------------------
# Print summary of all beacons
# ---------------------------------------------------------
def print_summary(detected_beacons):
    """Print a clean summary table of all detected beacons"""
    timestamp = time.strftime('%H:%M:%S')
    print("\n" + "=" * 70)
    print(f"📊 BEACON SUMMARY [{timestamp}]")
    print("=" * 70)
    
    if not detected_beacons:
        print("No beacons detected")
        print("=" * 70 + "\n")
        return
    
    # Find closest
    closest = get_closest_beacon(detected_beacons)
    closest_key = closest[0] if closest else None
    
    # Print each beacon
    for beacon_key, data in sorted(detected_beacons.items()):
        marker = "⭐" if beacon_key == closest_key else "  "
        status_emoji = "✅" if data['status'] == "Arrived" else \
                      "🟢" if data['status'] == "Very Close" else \
                      "🟡" if data['status'] == "Near" else \
                      "🟠" if data['status'] == "Moderate" else "⚪"
        
        print(f"{marker} {status_emoji} {data['location']['name']:<20} | "
              f"{data['distance']:>5} m | "
              f"RSSI: {data['avg_rssi']:>5.1f} dBm | "
              f"{data['status']}")
    
    if closest:
        print(f"\n🎯 Closest: {detected_beacons[closest_key]['location']['name']} "
              f"({detected_beacons[closest_key]['distance']} m)")
    
    print("=" * 70 + "\n")


# ---------------------------------------------------------
# MAIN BLE SCANNER (Async)
# ---------------------------------------------------------
async def scan_beacons(duration=3600):
    """
    Scan for BLE iBeacons continuously with rate-limited output
    Args:
        duration: How long to scan in seconds (default 1 hour)
    """
    global last_summary_time
    
    print("=" * 70)
    print("VISION-IMPAIRED NAVIGATION - MULTI-BEACON SCANNER")
    print("=" * 70)
    print(f"Target UUID: {TARGET_UUID}")
    print(f"Configured Beacons: {len(BEACON_LOCATIONS)}")
    for key, info in BEACON_LOCATIONS.items():
        print(f"  • Major {key[0]}, Minor {key[1]}: {info['name']}")
    print(f"\nOutput Settings:")
    print(f"  • Update interval: {UPDATE_INTERVAL} seconds per beacon")
    print(f"  • Summary interval: {SUMMARY_INTERVAL} seconds")
    print(f"  • RSSI smoothing: {RSSI_WINDOW} readings")
    print(f"  • Environment factor: {ENV_FACTOR}")
    print("\nMake sure all beacons are broadcasting!")
    print("Press Ctrl+C to stop\n")
    print("-" * 70)
    
    # Store all detected beacons
    detected_beacons = {}  # key: (major, minor), value: beacon_info
    last_summary_time = time.time()

    def detection_callback(device, advertisement_data):
        """Called whenever a BLE device is detected"""
        global last_summary_time
        
        # Try to parse as iBeacon
        result = parse_ibeacon(advertisement_data.manufacturer_data)
        if not result:
            return  # Not an iBeacon, ignore

        uuid, major, minor, tx_measured = result

        # Filter: Only process beacons with our target UUID
        if uuid != TARGET_UUID:
            return

        # Get device info
        address = device.address
        rssi = advertisement_data.rssi
        
        if rssi is None:
            return  # Invalid RSSI, skip

        # Create unique key for this beacon
        beacon_key = (major, minor)
        current_time = time.time()

        # Initialize RSSI buffer for this beacon if needed
        if beacon_key not in rssi_buffers:
            rssi_buffers[beacon_key] = deque(maxlen=RSSI_WINDOW)

        # Add RSSI to buffer and calculate average (smoothing)
        rssi_buffers[beacon_key].append(rssi)
        avg_rssi = sum(rssi_buffers[beacon_key]) / len(rssi_buffers[beacon_key])

        # Use TX power from beacon, fallback to default
        tx_power = tx_measured if tx_measured is not None else TX_POWER_DEFAULT

        # Calculate distance and status
        distance = estimate_distance(avg_rssi, tx_power)
        status = get_status(distance)
        location_info = get_location_info(major, minor)

        # Store beacon data
        detected_beacons[beacon_key] = {
            'address': address,
            'major': major,
            'minor': minor,
            'rssi': rssi,
            'avg_rssi': avg_rssi,
            'tx_power': tx_power,
            'distance': distance,
            'status': status,
            'location': location_info,
            'timestamp': current_time
        }

        # ===== DECIDE IF WE SHOULD PRINT =====
        should_print = False
        reason = ""
        
        # Reason 1: First time detecting this beacon
        if beacon_key not in last_print_time:
            should_print = True
            reason = "NEW BEACON"
        
        # Reason 2: Status changed (important!)
        elif beacon_key in last_status and last_status[beacon_key] != status:
            should_print = True
            reason = f"STATUS CHANGE: {last_status[beacon_key]} → {status}"
        
        # Reason 3: Enough time passed since last print
        elif current_time - last_print_time.get(beacon_key, 0) >= UPDATE_INTERVAL:
            should_print = True
            reason = "PERIODIC UPDATE"
        
        # ===== PRINT IF NEEDED =====
        if should_print:
            timestamp = time.strftime('%H:%M:%S')
            print(
                f"[{timestamp}] 📍 {location_info['name']} ({reason})\n"
                f"  └─ Beacon: Major {major}, Minor {minor}\n"
                f"  └─ RSSI: {rssi} dBm (avg: {round(avg_rssi, 1)} dBm)\n"
                f"  └─ Distance: {distance} m\n"
                f"  └─ Status: {status}\n"
            )
            last_print_time[beacon_key] = current_time
        
        # Update status tracking
        last_status[beacon_key] = status
        
        # ===== PRINT SUMMARY PERIODICALLY =====
        if current_time - last_summary_time >= SUMMARY_INTERVAL:
            print_summary(detected_beacons)
            last_summary_time = current_time

    # Create BLE scanner with callback
    scanner = BleakScanner(detection_callback)

    # Start scanning
    async with scanner:
        await asyncio.sleep(duration)


# ---------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------
if __name__ == "__main__":
    try:
        # Run async scanner
        asyncio.run(scan_beacons(duration=3600))  # Scan for 1 hour
    except KeyboardInterrupt:
        print("\n\n🛑 Scanner stopped by user")
        print("✅ Program terminated")
