#!/usr/bin/env python3
"""
yolo_process.py

YOLO worker process for Option A pipeline (shared memory reader + detection queue).

Responsibilities:
- Attach to an existing shared memory block created by camera_process.py
- Wait for small frame metadata notifications on `frame_queue`
- Drain the metadata queue to get the latest frame_id (drop stale notifications)
- Read the latest frame via a numpy view into shared memory
- Run YOLO inference (ultralytics.YOLO) on that frame
- Send small detection dicts to `result_queue` for main.py to consume
- Support graceful shutdown and safe resource cleanup

Usage (example):
    from multiprocessing import Process, Queue
    frame_queue = Queue(maxsize=4)
    result_queue = Queue(maxsize=8)
    p = Process(target=YoloWorker(...).run, args=...)
    p.start()
"""

import logging
import multiprocessing as mp
import os
import signal
import sys
import time
import traceback
from multiprocessing import shared_memory
from typing import Optional

import numpy as np

# Limit OpenMP / MKL threads to avoid oversubscription on CPU
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

LOG = logging.getLogger("yolo_process")

# Try importing ultralytics YOLO (model)
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception:
    YOLO_AVAILABLE = False

# cv2 is used to convert colors if necessary
import cv2


class YoloWorker:
    def __init__(
        self,
        shm_name: str,
        width: int = 640,
        height: int = 480,
        channels: int = 3,
        frame_queue: Optional[mp.Queue] = None,
        result_queue: Optional[mp.Queue] = None,
        model_path: str = "yolov8n.pt",
        imgsz: int = 320,
        conf: float = 0.35,
        device: str = None,
        allowed_classes: Optional[set] = None,
        heartbeat_interval: float = 2.0,
    ):
        """
        Parameters:
            shm_name: name of shared memory created by camera process
            width, height, channels: frame shape
            frame_queue: small queue used by camera process to notify new frames
            result_queue: queue to send detection dicts to main process
            model_path: path to YOLO model file
            imgsz: inference image size
            conf: confidence threshold
            device: model device (None -> default)
            allowed_classes: set of class names to keep (None -> keep all)
        """
        self.shm_name = shm_name
        self.width = width
        self.height = height
        self.channels = channels
        self.frame_shape = (height, width, channels)
        self.frame_bytes = width * height * channels

        self.frame_queue = frame_queue
        self.result_queue = result_queue
        self.model_path = model_path
        self.imgsz = imgsz
        self.conf = conf
        self.device = device
        self.allowed_classes = allowed_classes
        self.heartbeat_interval = heartbeat_interval

        self._running = mp.Event()
        self._running.set()

        self._shm = None
        self._np_view = None
        self._model = None
        self._last_heartbeat = time.time()

    # ---------------------------
    # Shared memory attach / view
    # ---------------------------
    def attach_shm(self):
        try:
            self._shm = shared_memory.SharedMemory(name=self.shm_name)
        except FileNotFoundError:
            LOG.exception("Shared memory '%s' not found. Did you start camera_process?", self.shm_name)
            raise
        except Exception:
            LOG.exception("Error attaching shared memory '%s'", self.shm_name)
            raise

        # Create a numpy view on the shared memory
        try:
            arr = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=self._shm.buf)
            self._np_view = arr
            LOG.info("Attached to shared memory '%s' shape=%s", self.shm_name, self.frame_shape)
        except Exception:
            LOG.exception("Failed to create numpy view over shared memory")
            raise

    # ---------------------------
    # Model loading
    # ---------------------------
    def load_model(self):
        if not YOLO_AVAILABLE:
            raise RuntimeError("ultralytics YOLO package not available in this Python environment")

        # Limit torch/OpenCV threads at runtime as extra safety
        try:
            import torch
            torch.set_num_threads(1)
        except Exception:
            pass

        try:
            # Create model; device argument optional
            if self.device:
                self._model = YOLO(self.model_path, device=self.device)
            else:
                self._model = YOLO(self.model_path)
            LOG.info("Loaded YOLO model '%s'", self.model_path)
        except Exception:
            LOG.exception("Failed to load YOLO model")
            raise

    # ---------------------------
    # Drain frame_queue to get latest metadata
    # ---------------------------
    def _drain_latest_meta(self, timeout: float = 1.0):
        """
        Read frame metadata queue until empty and return last item.
        If queue is None, returns None.
        If nothing received within timeout, returns None.
        """
        if self.frame_queue is None:
            return None

        meta = None
        start = time.time()
        # Block until at least one item or timeout
        try:
            m = self.frame_queue.get(timeout=timeout)
            meta = m
        except Exception:
            return None

        # Drain any additional items (keep the latest)
        try:
            while True:
                m = self.frame_queue.get_nowait()
                meta = m
        except Exception:
            pass

        return meta

    # ---------------------------
    # Run loop
    # ---------------------------
    def run(self):
        try:
            self.attach_shm()
        except Exception:
            LOG.error("Cannot attach to shared memory; exiting")
            return

        try:
            self.load_model()
        except Exception:
            LOG.error("Cannot load model; exiting")
            return

        LOG.info("YOLO worker entering main loop")
        try:
            while self._running.is_set():
                # Drain notifications quickly; prefer latest frame
                meta = self._drain_latest_meta(timeout=0.5)

                if meta is None:
                    # No new frame notification; optionally run a low-rate heartbeat or sleep
                    now = time.time()
                    if now - self._last_heartbeat >= self.heartbeat_interval:
                        # send lightweight heartbeat to result_queue to indicate alive
                        try:
                            if self.result_queue:
                                hb = {"type": "yolo_heartbeat", "timestamp": now}
                                self.result_queue.put_nowait(hb)
                        except Exception:
                            pass
                        self._last_heartbeat = now
                    time.sleep(0.01)
                    continue

                # Read the latest frame directly from shared memory view (it's a live view)
                try:
                    frame = self._np_view.copy()  # copy local to avoid race if camera writes concurrently
                except Exception:
                    LOG.exception("Failed to read frame from shared memory; skipping")
                    continue

                # Convert color if needed (shared memory is BGR as produced by camera_process)
                if frame.ndim == 3 and frame.shape[2] == 3:
                    input_frame = frame
                else:
                    # fallback: ensure 3-channel BGR
                    try:
                        input_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                    except Exception:
                        try:
                            input_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                        except Exception:
                            LOG.warning("Unexpected frame shape %s; skipping", frame.shape)
                            continue

                # Run inference (wrap in try/except)
                try:
                    # ultralytics YOLO predict: pass numpy array directly
                    results = self._model.predict(input_frame, imgsz=self.imgsz, device=self.device, conf=self.conf, verbose=False)
                    r = results[0]  # first result for single image
                except Exception:
                    LOG.exception("YOLO inference failed on frame_id=%s", meta.get("frame_id", "N/A"))
                    continue

                # Parse results into small dict
                detections = []
                try:
                    # Access boxes, classes, confs in a robust way
                    # r.boxes.xyxy  (N x 4), r.boxes.conf (N), r.boxes.cls (N)
                    if hasattr(r, "boxes") and r.boxes is not None and len(r.boxes) > 0:
                        try:
                            cls_array = r.boxes.cls.cpu().numpy()
                            boxes = r.boxes.xyxy.cpu().numpy()
                            confs = r.boxes.conf.cpu().numpy()
                        except Exception:
                            # fallback (some ultralytics versions have different types)
                            cls_array = np.array(r.boxes.cls)
                            boxes = np.array(r.boxes.xyxy)
                            confs = np.array(r.boxes.conf)

                        cls_array = np.atleast_1d(np.array(cls_array).squeeze())
                        confs = np.atleast_1d(np.array(confs).squeeze())
                        boxes = np.array(boxes)

                        if boxes.ndim == 1 and boxes.size == 4:
                            boxes = boxes.reshape(1, 4)

                        n = min(boxes.shape[0], cls_array.shape[0], confs.shape[0])
                        # build detections list
                        for i in range(n):
                            idx = int(cls_array[i])
                            # Resolve class name
                            try:
                                if isinstance(self._model.names, dict):
                                    name = self._model.names.get(idx, str(idx))
                                else:
                                    name = self._model.names[idx]
                            except Exception:
                                name = str(idx)

                            # Optionally filter classes
                            if self.allowed_classes and name not in self.allowed_classes:
                                continue

                            x1, y1, x2, y2 = boxes[i].astype(int).tolist()
                            conf_val = float(confs[i])
                            detections.append({"name": name, "confidence": round(conf_val, 3), "bbox": [x1, y1, x2, y2]})
                except Exception:
                    LOG.exception("Failed to parse YOLO results")

                # Build message
                message = {
                    "type": "detection",
                    "timestamp": time.time(),
                    "frame_meta": meta,
                    "objects": detections,
                }

                # Send to result queue (non-blocking if possible)
                if self.result_queue is not None:
                    try:
                        self.result_queue.put_nowait(message)
                    except Exception:
                        # If full, try a blocking put with short timeout, else drop
                        try:
                            self.result_queue.put(message, timeout=0.1)
                        except Exception:
                            LOG.debug("Result queue full; dropping detection message")

                # Small yield so other processes can run
                time.sleep(0.001)

        except Exception:
            LOG.exception("Unhandled exception in YOLO worker")
        finally:
            # cleanup
            LOG.info("YOLO worker shutting down")
            try:
                if self._shm:
                    self._shm.close()
            except Exception:
                LOG.exception("Error closing shared memory in YOLO worker")

    def stop(self):
        self._running.clear()


# ---------------------------
# CLI / Standalone test entry
# ---------------------------
def parse_args():
    import argparse
    p = argparse.ArgumentParser(prog="yolo_process.py", description="YOLO worker (shared memory reader)")
    p.add_argument("--shm-name", default="vision_shm_latest_frame", help="Shared memory name")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--channels", type=int, default=3)
    p.add_argument("--model", default="yolov8n.pt", help="YOLO model path")
    p.add_argument("--imgsz", type=int, default=320)
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--device", default=None, help="device (e.g., 0, 'cpu', 'cuda:0')")
    return p.parse_args()


def run_standalone(args):
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    frame_queue = mp.Queue(maxsize=4)
    result_queue = mp.Queue(maxsize=8)

    worker = YoloWorker(
        shm_name=args.shm_name,
        width=args.width,
        height=args.height,
        channels=args.channels,
        frame_queue=frame_queue,
        result_queue=result_queue,
        model_path=args.model,
        imgsz=args.imgsz,
        conf=args.conf,
        device=args.device,
    )

    # For standalone test we can't attach unless camera process created the shared memory.
    # This script assumes shared memory exists. If not, the attach_shm will fail.
    try:
        worker.run()
    except KeyboardInterrupt:
        worker.stop()
    except Exception:
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    args = parse_args()
    run_standalone(args)