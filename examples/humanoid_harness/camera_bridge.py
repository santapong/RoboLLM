"""Own one local camera subprocess without sharing incompatible Python packages."""

from __future__ import annotations

import base64
import json
import math
import os
import subprocess
import threading
import time
from pathlib import Path


class CameraBridge:
    def __init__(self, python: str, device: str, model: str | None = None):
        self.python, self.device, self.model = python, device, model
        self.process: subprocess.Popen | None = None
        self.lock = threading.Lock()
        self._lifecycle = threading.RLock()
        self._reader: threading.Thread | None = None
        self.packet = {"running": False, "tracked": False, "message": "Camera off"}
        self.image: bytes | None = None
        self.received_at: float | None = None
        self.captured_at: float | None = None

    def start(self):
        with self._lifecycle:
            with self.lock:
                process, running = self.process, self.packet.get("running")
            if process is not None and process.poll() is None and running:
                return
            if not Path(self.python).is_file():
                raise RuntimeError("Camera Python was not found; set --camera-python to an environment with OpenCV/MediaPipe")
            self.stop()
            argv = [self.python, str(Path(__file__).with_name("camera_worker.py")), "--device", self.device]
            if self.model:
                argv += ["--model", self.model]
            env = {**os.environ, "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "TF_CPP_MIN_LOG_LEVEL": "2"}
            # Publish startup status only after Popen succeeds. No reader can
            # race this assignment, and an old reader has already been detached.
            process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=subprocess.DEVNULL, env=env)
            with self.lock:
                self.process = process
                self.packet = {"running": True, "tracked": False, "message": "Starting camera and hand tracker…"}
            self._reader = threading.Thread(target=self._read, args=(process,), daemon=True)
            self._reader.start()

    def _accept(self, process, packet, image):
        """Accept a packet only from the current process; freeze time validity."""
        received = time.monotonic()
        stamp = packet.get("timestamp")
        try:
            valid = (isinstance(stamp, (int, float)) and not isinstance(stamp, bool)
                     and math.isfinite(stamp) and 0 <= stamp <= received)
        except OverflowError:
            valid = False
        with self.lock:
            if self.process is not process:
                return False
            self.packet, self.image, self.received_at = packet, image, received
            self.captured_at = float(stamp) if valid else None
        return True

    def _read(self, process):
        try:
            for line in iter(process.stdout.readline, b""):
                try:
                    packet = json.loads(line)
                    if not isinstance(packet, dict):
                        continue
                    encoded = packet.pop("jpeg_b64", None)
                    image = base64.b64decode(encoded, validate=True) if encoded else None
                    if not self._accept(process, packet, image):
                        return
                except (ValueError, TypeError):
                    continue
        except (OSError, ValueError):
            pass  # A concurrent stop may close the owned pipe.
        finally:
            with self.lock:
                if self.process is process:
                    message = self.packet.get("message")
                    if (self.packet.get("running") is not False
                            or not isinstance(message, str) or not message.strip()):
                        self.packet["message"] = "Camera worker stopped; check device or camera environment"
                    self.packet.update(running=False, tracked=False, confidence=0.0)
                    self.image = None
                    self.received_at = self.captured_at = None

    def snapshot(self):
        with self.lock:
            packet, image = dict(self.packet), self.image
            received, captured = self.received_at, self.captured_at
            if isinstance(packet.get("target"), list):
                packet["target"] = list(packet["target"])
        now = time.monotonic()
        # Keep age finite for operational JSON. The worker repeats snapshots,
        # so receipt time alone cannot establish that a captured image is fresh.
        valid = received is not None and captured is not None and captured <= now and received <= now
        age = max(now - received, now - captured) if valid else 1_000_000.0
        if not packet.get("running") or not valid or age > 0.5:
            was_tracked = packet.get("tracked")
            packet.update(tracked=False, confidence=0.0)
            if packet.get("running") and received is not None:
                if not valid and was_tracked:
                    packet["message"] = "Invalid camera capture timestamp; robot holds"
                elif valid and age > 0.5:
                    packet["message"] = "Camera data is stale — robot holds"
        return packet, image, age

    def stop(self):
        with self._lifecycle:
            with self.lock:
                process, self.process = self.process, None
                self.packet = {"running": False, "tracked": False, "message": "Camera off"}
                self.image = None
                self.received_at = self.captured_at = None
            reader, self._reader = self._reader, None
            if process is not None:
                if process.stdin:
                    try:
                        process.stdin.close()
                    except (OSError, ValueError):
                        pass  # Still reap our child if it has already closed stdin.
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        process.terminate()
                    except ProcessLookupError:
                        pass  # Child exited between wait and terminate.
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        try:
                            process.kill()
                        except ProcessLookupError:
                            pass
                        process.wait(timeout=2)
                if reader is not None:
                    reader.join(timeout=1)
                if process.stdout:
                    process.stdout.close()
            with self.lock:
                self.packet["message"] = "Camera off; device released"
