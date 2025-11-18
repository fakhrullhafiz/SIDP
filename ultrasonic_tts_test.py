import RPi.GPIO as GPIO
import time
import os

# === GPIO SETUP ===
GPIO.setmode(GPIO.BOARD)

# Pins for the ultrasonic sensor
TRIG = 11  # GPIO 17
ECHO = 12  # GPIO 18

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, False)
print("Waiting for sensor to settle...")
time.sleep(2)

# === FUNCTION TO MEASURE DISTANCE ===
def measure_distance():
    # Send a 10μs pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()

    # Wait for echo to go high
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # Wait for echo to go low
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Convert to cm
    distance = round(distance, 2)
    return distance


# === MAIN LOOP ===
try:
    last_message = None
    last_announce_time = 0
    announce_interval = 2  # seconds between TTS messages

    while True:
        distance = measure_distance()
        print(f"Distance: {distance} cm")

        current_time = time.time()

        # === Threshold logic ===
        if distance < 50:
            message = "Stop"
        elif distance < 100:
            message = "Warning, object ahead"
        elif distance < 200:
            message = "Caution"
        else:
            message = None

        # === Speak only if message changes or interval passes ===
        if message and (message != last_message or current_time - last_announce_time > announce_interval):
            os.system(f"espeak '{message}'")
            last_message = message
            last_announce_time = current_time

        time.sleep(1)

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    GPIO.cleanup()
