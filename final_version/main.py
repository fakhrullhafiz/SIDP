#!/usr/bin/env python3
"""
Simple supervisor to run `sub1.py` and `sub2.py` as separate processes.

Features:
- Launches each script in its own process (using the same Python interpreter).
- Streams their stdout/stderr with a prefix so you can tell which script produced the line.
- Restarts a script with exponential backoff if it exits unexpectedly.
- Gracefully shuts down children on KeyboardInterrupt (Ctrl+C).

Place this file in the same folder as `sub1.py` and `sub2.py` and run with Python 3 on the target device.
"""
import subprocess
import sys
import os
import time
import threading

SCRIPT_DIR = os.path.dirname(__file__)
SUBS = {
    "sub1": os.path.join(SCRIPT_DIR, "sub1.py"),
    "sub2": os.path.join(SCRIPT_DIR, "sub2.py"),
}

procs = {}
stop_event = threading.Event()


def start_process(path):
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    cmd = [sys.executable, path]
    # start with line-buffered text mode
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=SCRIPT_DIR, text=True, bufsize=1)


def stream_output(name, proc):
    try:
        if proc.stdout is None:
            return
        for line in proc.stdout:
            print(f"[{name}] {line.rstrip()}")
    except Exception:
        pass


def monitor(name, path):
    backoff = 1.0
    while not stop_event.is_set():
        try:
            proc = start_process(path)
        except Exception as e:
            print(f"[{name}] failed to start: {e}")
            time.sleep(backoff)
            backoff = min(backoff * 2, 60.0)
            continue

        procs[name] = proc
        t = threading.Thread(target=stream_output, args=(name, proc), daemon=True)
        t.start()

        # wait for process to end or stop_event
        while proc.poll() is None and not stop_event.is_set():
            time.sleep(0.5)

        # if stopping, terminate child then break
        if stop_event.is_set():
            try:
                proc.terminate()
            except Exception:
                pass
            break

        ret = proc.returncode
        print(f"[{name}] exited with code {ret}. restarting in {backoff:.1f}s")
        # cleanup
        try:
            proc.stdout.close()
        except Exception:
            pass

        time.sleep(backoff)
        backoff = min(backoff * 2, 60.0)


def terminate_all():
    stop_event.set()
    print("Supervisor shutting down children...")
    # request terminate
    for name, p in list(procs.items()):
        try:
            if p.poll() is None:
                print(f"Terminating {name} (pid {p.pid})")
                p.terminate()
        except Exception:
            pass

    # give time to exit
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if all((p.poll() is not None) for p in procs.values()):
            break
        time.sleep(0.2)

    # force kill remaining
    for name, p in list(procs.items()):
        try:
            if p.poll() is None:
                print(f"Killing {name} (pid {p.pid})")
                p.kill()
        except Exception:
            pass


def main():
    print("Starting supervisor for sub1.py and sub2.py")

    threads = []
    for name, path in SUBS.items():
        t = threading.Thread(target=monitor, args=(name, path), daemon=True)
        t.start()
        threads.append(t)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received — shutting down")
        terminate_all()

    # small join
    for t in threads:
        t.join(timeout=1.0)

    print("Supervisor exited")


if __name__ == "__main__":
    main()
