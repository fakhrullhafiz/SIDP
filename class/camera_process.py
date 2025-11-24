#!/usr/bin/env python3
"""
camera_process.py

Camera process for Option A pipeline (shared memory + notification queue).

Responsibilities:
- Create / attach to a shared memory block sized for (height, width, channels)
- Capture frames from Picamera2 as quickly as possible
- Convert frame to BGR uint8 and write directly into shared memory buffer (zero-copy)
- Put a small metadata message (frame_id, timestamp) onto a multiprocessing.Queue to notify consumer(s)
- Handle graceful shutdown and cleanup of shared memory

Usage:
- As part of a multiprocessing system:
    main creates:
      - shm_name (string)
      - a multiprocessing.Queue instance `frame_queue`
    then spawns this module as a Process and passes those objects/args.

- Standalone (for testing):
    python3 camera_process.py
    (it will create its own shared memory and a queue and print frame notifications)
"""

import argparse
import logging
import signal
import sys
import time
from multiprocessing import shared_memory, Queue
import multiprocessing as mp
import traceback
from typing import Optional

import numpy as np

# Picamera2 is Raspberry Pi specific
try:
    from picamera2 import Picamera2
    from libcamera import Transform
    PICAMERA2_AVAILABLE = True
except Exception:
    PICAMERA2_AVAILABLE = False

import cv2


# -----------------------------------------------------------------------------
# Helpers / Defaults
# -----------------------------------------------------------------------------
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_CHANNELS = 3  # we will store BGR (3 channels)
DEFAULT_SHM_NAME = "vision_shm_latest_frame"
DEFAULT_FPS = 30
LOG = logging.getLogger("camera_process")


# -----------------------------------------------------------------------------
# Camera Worker
# -----------------------------------------------------------------------------
class CameraWorker:
    def __init__(
        self,
        shm_name: str = DEFAULT_SHM_NAME,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        channels: int = DEFAULT_CHANNELS,
        frame_queue: Optional[Queue] = None,
        target_fps: int = DEFAULT_FPS,
        recreate_shm: bool = True,
    ):
        self.shm_name = shm_name
        self.width = width
        self.height = height
        self.channels = channels
        self.frame_size = width * height * channels
        self.frame_queue = frame_queue
        self.target_fps = target_fps
        self.recreate_shm = recreate_shm

        self._running = mp.Event()
        self._running.set()

        self._picam2 = None
        self._shm = None
        self._np_view = None
        self._frame_id = 0

    def create_or_attach_shm(self):
        """
        Create a new shared memory block, or attach if it already exists.
        We create with create=True and let the owner be this process. The main process
        can also create it and pass the name.
        """
        try:
            # Try to unlink old shared memory if recreate_shm is True and exists
            if self.recreate_shm:
                try:
                    existing = shared_memory.SharedMemory(name=self.shm_name)
                    LOG.info("Found existing shared memory block; unlinking as recreate_shm=True")
                    existing.unlink()
                    existing.close()
                except FileNotFoundError:
                    pass
                except Exception:
                    # if exists but can't unlink, continue trying to create new
                    LOG.warning("Could not unlink existing shared memory (non-critical)")

            self._shm = shared_memory.SharedMemory(name=self.shm_name, create=True, size=self.frame_size)
            LOG.info("Created shared memory '%s' size=%d bytes", self.shm_name, self.frame_size)
        except FileExistsError:
            LOG.info("Shared memory '%s' already exists. Attaching.", self.shm_name)
            self._shm = shared_memory.SharedMemory(name=self.shm_name, create=False)
        except Exception as e:
            LOG.exception("Failed to create or attach shared memory: %s", e)
            raise

        # Build numpy view over the shared buffer
        arr = np.ndarray((self.height, self.width, self.channels), dtype=np.uint8, buffer=self._shm.buf)
        self._np_view = arr

    def configure_camera(self):
        if not PICAMERA2_AVAILABLE:
            raise RuntimeError("Picamera2 not available in this environment")

        self._picam2 = Picamera2()
        # Preview configuration that produces 4-channel XBGR8888 in many setups.
        config = self._picam2.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "XBGR8888"}
        )
        self._picam2.configure(config)
        self._picam2.start()
        LOG.info("Picamera2 started (width=%d height=%d)", self.width, self.height)

    def stop_camera(self):
        try:
            if self._picam2:
                self._picam2.stop()
                LOG.info("Picamera2 stopped")
        except Exception:
            LOG.exception("Error stopping Picamera2")

    def cleanup_shm(self):
        try:
            if self._shm is not None:
                LOG.info("Closing shared memory '%s'", self.shm_name)
                self._shm.close()
                # Unlink only if we created it (recreate_shm True). If main is owner,
                # main should handle unlinking.
                if self.recreate_shm:
                    try:
                        self._shm.unlink()
                        LOG.info("Unlinked shared memory '%s'", self.shm_name)
                    except FileNotFoundError:
                        pass
                    except Exception:
                        LOG.exception("Error unlinking shared memory")
        except Exception:
            LOG.exception("Error cleaning up shared memory")

    def run(self):
        """
        Main capture loop. Writes frames into shared memory and push notification messages
        into frame_queue.
        """
        try:
            self.create_or_attach_shm()
        except Exception:
            LOG.error("Cannot create/attach shared memory. Exiting.")
            return

        try:
            self.configure_camera()
        except Exception:
            LOG.exception("Picamera2 configuration failed. Exiting.")
            self.cleanup_shm()
            return

        LOG.info("Camera worker entering capture loop")
        period = 1.0 / max(1, self.target_fps)
        last_time = time.time()

        try:
            while self._running.is_set():
                # Capture frame as numpy array (likely XBGR8888 -> BGRA order)
                frame = self._picam2.capture_array()
                # Some builds give 4 channels (BGRA); we want BGR (3 channels)
                if frame.ndim == 3 and frame.shape[2] == 4:
                    # Convert BGRA -> BGR
                    try:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                    except Exception:
                        # If conversion fails, just drop alpha channel
                        frame = frame[:, :, :3].copy()
                elif frame.ndim == 2:
                    # Grayscale -> convert to BGR by stacking
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

                # Resize if camera returns unexpected shape
                if frame.shape[0] != self.height or frame.shape[1] != self.width:
                    frame = cv2.resize(frame, (self.width, self.height))

                # Write directly into shared memory view (in-place)
                try:
                    # Copying the small NumPy view into shared memory buffer (this is native memory copy)
                    np.copyto(self._np_view, frame)
                except Exception:
                    LOG.exception("Failed to write frame into shared memory; continuing")

                # Notify via queue (non-blocking)
                self._frame_id += 1
                meta = {"frame_id": self._frame_id, "timestamp": time.time()}
                if self.frame_queue is not None:
                    try:
                        # Try a non-blocking put; if full, drop the notification (consumer reads latest)
                        self.frame_queue.put_nowait(meta)
                    except Exception:
                        # If queue fails or full, ignore — consumer should read shared memory directly.
                        pass

                # Throttle to target_fps to avoid wasting CPU
                elapsed = time.time() - last_time
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                last_time = time.time()

        except Exception:
            LOG.exception("Exception in camera capture loop")
        finally:
            LOG.info("Camera worker shutting down")
            self.stop_camera()
            self.cleanup_shm()

    def stop(self):
        self._running.clear()


# -----------------------------------------------------------------------------
# Signal handling & CLI entrypoint for standalone testing
# -----------------------------------------------------------------------------
def run_standalone(args):
    # configure logging
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    frame_queue = Queue(maxsize=8)

    worker = CameraWorker(
        shm_name=args.shm_name,
        width=args.width,
        height=args.height,
        channels=args.channels,
        frame_queue=frame_queue,
        target_fps=args.fps,
        recreate_shm=True,
    )

    # run in a separate process to emulate how main will spawn it
    proc = mp.Process(target=worker.run, name="camera_process")
    proc.start()

    LOG.info("Standalone camera_process started (pid=%d). Press Ctrl+C to stop.", proc.pid)

    try:
        # In standalone test mode: consume a few notifications and print them
        while proc.is_alive():
            try:
                meta = frame_queue.get(timeout=2.0)
                print(f"[camera_process] frame_id={meta['frame_id']} ts={meta['timestamp']:.3f}")
            except Exception:
                # If no frames in timeout, just loop to check process alive
                pass
    except KeyboardInterrupt:
        LOG.info("KeyboardInterrupt received; terminating camera process...")
    finally:
        if proc.is_alive():
            proc.terminate()
            proc.join(timeout=2.0)
        LOG.info("Standalone camera_process exiting.")


def install_signal_handlers(stop_event: mp.Event, proc: Optional[mp.Process] = None):
    def handler(signum, frame):
        LOG.info("Signal %s received; setting stop event", signum)
        stop_event.clear()
        if proc is not None and proc.is_alive():
            try:
                proc.terminate()
            except Exception:
                pass

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


def parse_args():
    p = argparse.ArgumentParser(prog="camera_process.py", description="Camera process (shared memory writer)")
    p.add_argument("--shm-name", default=DEFAULT_SHM_NAME, help="Shared memory name")
    p.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Frame width")
    p.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Frame height")
    p.add_argument("--channels", type=int, default=DEFAULT_CHANNELS, help="Number of channels (3 = BGR)")
    p.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Target FPS for capture loop")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        run_standalone(args)
    except Exception:
        traceback.print_exc()
        sys.exit(1)