"""Opt-in local webcam targets for the humanoid simulation.

Nothing opens or imports vision libraries until ``start()``. Frames stay in
memory; this module has no disk-write or network path. ``snapshot()`` is JSON
safe; ``jpeg()`` returns the latest mirrored preview separately.

Target coordinates are [-1, 1]: x is mirrored image horizontal, y is always
zero (depth is uncalibrated), z is image up. The hand target is the palm center,
not full-body retargeting, a VLM, or a learned robot policy. ``confidence`` is
binary acceptance after detector/geometry gates, not a calibrated probability.
Consumers MUST gate on ``tracked``; the last target is retained on loss.
"""

from __future__ import annotations

import math
import os
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any


def image_target(x: float, y: float) -> list[float]:
    """Map raw image fractions to mirrored, depth-neutral target signals."""
    if not all(math.isfinite(v) for v in (x, y)):
        raise ValueError("Image coordinates must be finite")
    return [max(-1.0, min(1.0, 1.0 - 2.0 * x)), 0.0,
            max(-1.0, min(1.0, 1.0 - 2.0 * y))]


class _HandTracker:
    name = "mediapipe_hand"
    label = "LOCAL HAND TARGET"

    def __init__(self, cv2, model_path: str):
        import mediapipe as mp
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision

        self.cv2, self.mp = cv2, mp
        self.detector = vision.HandLandmarker.create_from_options(
            vision.HandLandmarkerOptions(
                base_options=python.BaseOptions(model_asset_path=model_path),
                running_mode=vision.RunningMode.VIDEO,
                num_hands=1,
                min_hand_detection_confidence=0.65,
                min_hand_presence_confidence=0.65,
                min_tracking_confidence=0.65,
            )
        )
        self.last_ms = -1

    def process(self, frame, timestamp: float) -> dict[str, Any]:
        # Inference always sees raw frames; only the final preview is mirrored.
        rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        self.last_ms = max(int(timestamp * 1000), self.last_ms + 1)
        result = self.detector.detect_for_video(image, self.last_ms)
        if not result.hand_landmarks:
            return {"tracked": False, "message": "Raise one open hand into frame; hold"}
        palm = [result.hand_landmarks[0][i] for i in (0, 5, 9, 13, 17)]
        coords = [(float(p.x), float(p.y)) for p in palm]
        if not all(math.isfinite(v) and 0 <= v <= 1 for p in coords for v in p):
            return {"tracked": False, "message": "Hand is outside the image; hold"}
        x = sum(p[0] for p in coords) / len(coords)
        y = sum(p[1] for p in coords) / len(coords)
        return {"tracked": True, "target": image_target(x, y), "confidence": 1.0,
                "point": (x, y), "message": "Palm target; simulated arm only; fixed depth"}

    def close(self):
        self.detector.close()


class _RedMarkerTracker:
    name = "red_marker"
    label = "RED MARKER TARGET (NO HAND MODEL)"

    def __init__(self, cv2):
        self.cv2 = cv2

    def process(self, frame, timestamp: float) -> dict[str, Any]:
        import numpy as np

        cv2 = self.cv2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low = cv2.inRange(hsv, np.array([0, 130, 90]), np.array([9, 255, 255]))
        high = cv2.inRange(hsv, np.array([171, 130, 90]), np.array([179, 255, 255]))
        mask = cv2.morphologyEx(low | high, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return {"tracked": False, "message": "Show a bright red marker; hold"}
        contour = max(contours, key=cv2.contourArea)
        h, w = frame.shape[:2]
        area = cv2.contourArea(contour)
        if not 0.0015 * w * h <= area <= 0.25 * w * h:
            return {"tracked": False, "message": "Red region too small or large; hold"}
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            return {"tracked": False, "message": "No usable red marker; hold"}
        x = moments["m10"] / moments["m00"] / w
        y = moments["m01"] / moments["m00"] / h
        return {"tracked": True, "target": image_target(x, y), "confidence": 1.0,
                "point": (x, y), "message": "Red-color heuristic; fixed depth; not hand tracking"}

    def close(self):
        pass


class CameraInput:
    """Latest-frame reader; start/stop are explicit and repeatable.

    ``mode`` is ``auto``, ``hand`` or ``red_marker``. Auto uses the supplied
    hand model (or ROBOLLM_HAND_MODEL) when usable, otherwise explicitly labels
    the red-marker fallback. Factories are an in-memory test seam, never a
    device opened by construction. Run this in the existing vision interpreter
    when its NumPy ABI differs from the UI; do not merge site-packages.
    """

    def __init__(
        self, device: str | int = "/dev/video0", mode: str = "auto", fps: float = 12,
        model_path: str | None = None, *, stale_after: float = 0.6,
        capture_factory: Callable | None = None, tracker_factory: Callable | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        if mode not in ("auto", "hand", "red_marker"):
            raise ValueError("mode must be auto, hand, or red_marker")
        if not math.isfinite(fps) or not 1 <= fps <= 30:
            raise ValueError("fps must be within [1, 30]")
        if not math.isfinite(stale_after) or stale_after <= 0:
            raise ValueError("stale_after must be positive and finite")
        self.device, self.mode, self.fps = device, mode, float(fps)
        self.model_path = model_path or os.environ.get("ROBOLLM_HAND_MODEL")
        self.stale_after, self._clock = stale_after, clock
        self._capture_factory, self._tracker_factory = capture_factory, tracker_factory
        self._lock, self._lifecycle = threading.Lock(), threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._preview: bytes | None = None
        self._status = {"running": False, "tracked": False, "target": [0.0, 0.0, 0.0],
                        "confidence": 0.0, "timestamp": None, "backend": "off",
                        "message": "Camera off", "frames": 0, "fallback_reason": ""}

    def start(self) -> dict[str, Any]:
        with self._lifecycle:
            if self._thread is not None and self._thread.is_alive():
                return self.snapshot()
            import cv2

            tracker, capture = None, None
            try:
                fallback = ""
                if self._tracker_factory is not None:
                    tracker = self._tracker_factory(cv2)
                elif self.mode == "red_marker":
                    tracker = _RedMarkerTracker(cv2)
                else:
                    try:
                        if not self.model_path or not Path(self.model_path).is_file():
                            raise RuntimeError("No local hand model configured")
                        tracker = _HandTracker(cv2, str(self.model_path))
                    except (ImportError, RuntimeError, ValueError, OSError) as exc:
                        if self.mode == "hand":
                            raise
                        fallback = f"Hand backend unavailable: {type(exc).__name__}: {exc}"
                        tracker = _RedMarkerTracker(cv2)
                capture = (self._capture_factory(self.device) if self._capture_factory
                           else cv2.VideoCapture(self.device, cv2.CAP_V4L2))
                if not capture.isOpened():
                    raise RuntimeError(f"Camera {self.device} could not open (missing or busy)")
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                capture.set(cv2.CAP_PROP_FPS, self.fps)
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self._stop.clear()
                with self._lock:
                    self._preview = None
                    self._status.update(running=True, tracked=False, confidence=0.0,
                                        timestamp=None, backend=tracker.name, frames=0,
                                        message="Camera starting; hold", fallback_reason=fallback)
                self._thread = threading.Thread(target=self._loop,
                                                args=(capture, tracker, cv2), daemon=True,
                                                name="humanoid-local-camera")
                self._thread.start()
            except Exception as exc:
                if capture is not None:
                    capture.release()
                if tracker is not None:
                    tracker.close()
                with self._lock:
                    self._status.update(running=False, tracked=False, confidence=0.0,
                                        message=f"Camera start failed: {type(exc).__name__}: {exc}")
                raise
        return self.snapshot()

    def _loop(self, capture, tracker, cv2):
        try:
            while not self._stop.is_set():
                begun = self._clock()
                ok, frame = capture.read()
                captured = self._clock()
                if self._stop.is_set():
                    break
                if not ok or frame is None:
                    with self._lock:
                        self._status.update(tracked=False, confidence=0.0,
                                            message="Camera frame unavailable; hold")
                    self._stop.wait(0.1)
                    continue
                result = tracker.process(frame, captured)
                tracked = bool(result.get("tracked", False))
                target = result.get("target")
                if tracked and (not isinstance(target, (list, tuple)) or len(target) != 3
                                or not all(math.isfinite(v) and -1 <= v <= 1 for v in target)):
                    tracked = False
                    result["message"] = "Invalid target rejected; hold"
                canvas = frame.copy()
                if tracked and result.get("point") is not None:
                    x, y = result["point"]
                    h, w = canvas.shape[:2]
                    cv2.circle(canvas, (int(x * w), int(y * h)), 12, (80, 240, 100), 2)
                canvas = cv2.flip(canvas, 1)
                cv2.putText(canvas, tracker.label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                            0.48, (80, 240, 100) if tracked else (0, 190, 255), 1)
                cv2.putText(canvas, "TRACKED" if tracked else "HOLD - no target", (10, 47),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
                encoded, jpg = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 75])
                with self._lock:
                    if self._stop.is_set():
                        break
                    self._preview = jpg.tobytes() if encoded else None
                    if tracked:
                        self._status["target"] = [float(v) for v in target]
                    self._status.update(tracked=tracked, confidence=1.0 if tracked else 0.0,
                                        timestamp=captured, message=result.get("message", ""),
                                        frames=self._status["frames"] + 1)
                self._stop.wait(max(0.0, 1.0 / self.fps - (self._clock() - begun)))
        except Exception as exc:  # noqa: BLE001 -- report worker errors and fail closed.
            with self._lock:
                self._status.update(tracked=False, confidence=0.0,
                                    message=f"Camera processing failed: {type(exc).__name__}: {exc}")
        finally:
            try:
                capture.release()
            finally:
                try:
                    tracker.close()
                finally:
                    with self._lock:
                        self._status.update(running=False, tracked=False, confidence=0.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._status)
            result["target"] = list(result["target"])
        stamp = result["timestamp"]
        age = None if stamp is None else max(0.0, self._clock() - stamp)
        result["age_seconds"] = age
        if self._stop.is_set() or not result["running"]:
            result.update(tracked=False, confidence=0.0)
        elif age is not None and age > self.stale_after:
            result.update(tracked=False, confidence=0.0, message="Camera target stale; hold")
        return result

    def jpeg(self) -> bytes | None:
        with self._lock:
            return self._preview

    def stop(self) -> dict[str, Any]:
        with self._lifecycle:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            stopping = self._thread is not None and self._thread.is_alive()
            with self._lock:
                self._preview = None
                self._status.update(running=stopping, tracked=False, confidence=0.0,
                                    message="Camera closing; hold" if stopping else "Camera off")
        return self.snapshot()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()
