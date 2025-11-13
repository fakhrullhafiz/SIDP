import cv2
import time
import threading
import queue
import subprocess
from ultralytics import YOLO
from picamera2 import Picamera2
import numpy as np

# --------- Text-to-Speech setup ----------
tts_queue = queue.Queue()
use_pyttsx3 = False
engine = None

try:
    import pyttsx3
    engine = pyttsx3.init()
    engine.setProperty("rate", 150)
    engine.setProperty("volume", 1.0)
    use_pyttsx3 = True
except Exception:
    use_pyttsx3 = False


def tts_worker():
    while True:
        text = tts_queue.get()
        if text is None:
            break
        try:
            if use_pyttsx3 and engine is not None:
                engine.say(text)
                engine.runAndWait()
            else:
                subprocess.run(["espeak", text], check=False)
        except Exception as e:
            print("TTS error:", e)
        tts_queue.task_done()


tts_thread = threading.Thread(target=tts_worker, daemon=True)
tts_thread.start()


def speak(text):
    tts_queue.put(text)


# --------- YOLO + PiCamera2 ----------
model = YOLO("yolov8n.pt")

allowed_classes = {
    'person', 'car', 'cat', 'dog', 'stop sign',
    'toilet', 'chair', 'bed', 'tv', 'dining table', 'vase'
}

ANNOUNCE_COOLDOWN = 5.0
last_announced = {}

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (320, 240), "format": "XBGR8888"}
)
picam2.configure(config)
picam2.start()

print("Camera running... Press 'q' to quit")

try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

        results = model.predict(frame, imgsz=320, conf=0.35, verbose=False)
        r = results[0]

        # if no detections at all
        if len(r.boxes) == 0:
            cv2.imshow("YOLOv8", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # --- Robust extraction and shaping ---
        try:
            cls_array = r.boxes.cls.cpu().numpy()
            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
        except Exception:
            # fallback: some `boxes` objects might already be numpy/ lists
            cls_array = np.array(r.boxes.cls)
            boxes = np.array(r.boxes.xyxy)
            confs = np.array(r.boxes.conf)

        # Squeeze safely but keep dims for single detection
        cls_array = np.array(cls_array).squeeze()
        confs = np.array(confs).squeeze()
        boxes = np.array(boxes)

        # If boxes is 1D with 4 elements, reshape to (1,4)
        if boxes.ndim == 1 and boxes.size == 4:
            boxes = boxes.reshape(1, 4)

        # Ensure cls_array and confs are at least 1-D arrays
        cls_array = np.atleast_1d(cls_array)
        confs = np.atleast_1d(confs)

        # Make sure lengths align: sometimes one of them may be mismatched
        n_boxes = boxes.shape[0]
        n_cls = cls_array.shape[0]
        n_confs = confs.shape[0]
        n = min(n_boxes, n_cls, n_confs)

        if n == 0:
            # weird edge case where shapes don't match, skip frame
            cv2.imshow("YOLOv8", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # Truncate arrays to common length n
        boxes = boxes[:n]
        cls_array = cls_array[:n]
        confs = confs[:n]

        # Build detected names list
        detected_names = []
        for c in cls_array:
            idx = int(c)
            if isinstance(model.names, dict):
                name = model.names.get(idx, str(idx))
            else:
                name = model.names[idx]
            detected_names.append(name)

        # Filter to allowed classes
        keep_idx = [i for i, name in enumerate(detected_names) if name in allowed_classes]

        now = time.time()
        announced_this_frame = set()
        for i in keep_idx:
            name = detected_names[i]
            if name in announced_this_frame:
                continue
            announced_this_frame.add(name)

            last_time = last_announced.get(name, 0)
            if now - last_time >= ANNOUNCE_COOLDOWN:
                speak(f"{name} detected")
                last_announced[name] = now

        # Draw boxes for kept detections
        for i in keep_idx:
            x1, y1, x2, y2 = boxes[i].astype(int)
            conf = float(confs[i])  # safe now because confs is at least 1D
            name = detected_names[i]
            label = f"{name} {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame, label, (x1, max(y1 - 8, 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1
            )

        cv2.imshow("YOLOv8", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    tts_queue.put(None)
    tts_thread.join(timeout=1)
    print("Exited cleanly.")
