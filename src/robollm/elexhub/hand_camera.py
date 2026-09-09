"""Explicitly started, local LEFT-hand input for relative simulation mirroring.

The worker mirrors the camera image once before MediaPipe inference and preview.
Apparent palm size and pinch are image-space controls, never measured 3D depth.
Frames remain in memory; this service neither records images nor drives an arm.
"""

from __future__ import annotations

import hashlib
import math
import threading
import time
from collections import deque
from numbers import Real
from pathlib import Path

import cv2
import numpy as np

from .camera_ownership import CameraOwnership
from .vision import CalibrationError, HandTracker

HAND_MODEL_SHA256 = "fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1"
HAND_CONFIDENCE = 0.65
SAMPLE_MAX_AGE_S = 0.35
FRAME_MAX_AGE_S = 1.0
MIN_PALM_SCALE = 0.025
PINCH_CLOSED_RATIO = 0.15
PINCH_OPEN_RATIO = 1.0
FRAME_PERIOD_S = 0.05
HAND_EDGES = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    (5, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    (9, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    (13, 17),
    (0, 17),
    (17, 18),
    (18, 19),
    (19, 20),
)
STATE_LABELS = {
    "left_hand": "LEFT HAND detected",
    "right_hand": "RIGHT HAND ignored - show your LEFT hand",
    "no_hand": "Show your LEFT hand",
    "ambiguous": "Multiple LEFT hands - tracking paused",
    "low_confidence": "Hand detection uncertain - tracking paused",
    "unavailable": "Cannot detect camera frame",
    "stopped": "Camera stopped",
}


def _finite(value):
    return (
        isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)
    )


def _confidence(hand):
    value = hand.get("confidence")
    return float(value) if _finite(value) and 0 <= value <= 1 else None


def _landmarks(hand):
    values = hand.get("landmarks_norm")
    if not isinstance(values, (list, tuple)) or len(values) != 21:
        return None
    if any(
        not isinstance(point, (list, tuple))
        or len(point) != 2
        or any(not _finite(v) or not 0 <= v <= 1 for v in point)
        for point in values
    ):
        return None
    return np.asarray(values, dtype=float)


def hand_sample(hands, size, timestamp):
    """Select one confident Left hand from an already mirrored image.

    Returns (state, sample, handedness, confidence). Two Left classifications
    are ambiguous even if one has a low score; their identity is not guessed.
    """
    if (
        not isinstance(hands, (list, tuple))
        or any(not isinstance(hand, dict) for hand in hands)
        or not _finite(timestamp)
        or timestamp < 0
        or not isinstance(size, (list, tuple))
        or len(size) != 2
        or any(type(value) is not int or value <= 0 for value in size)
    ):
        return "low_confidence", None, None, None
    left = [hand for hand in hands if hand.get("handedness") == "Left"]
    if len(left) > 1:
        scores = [_confidence(hand) for hand in left]
        return (
            "ambiguous",
            None,
            "Left",
            max((v for v in scores if v is not None), default=None),
        )
    if not left:
        right = [hand for hand in hands if hand.get("handedness") == "Right"]
        if right:
            scores = [_confidence(hand) for hand in right]
            return (
                "right_hand",
                None,
                "Right",
                max((v for v in scores if v is not None), default=None),
            )
        return ("low_confidence" if hands else "no_hand"), None, None, None
    hand = left[0]
    confidence = _confidence(hand)
    points = _landmarks(hand)
    if confidence is None or confidence < HAND_CONFIDENCE or points is None:
        return "low_confidence", None, "Left", confidence
    # Use the same pixel aspect correction for palm length and finger separation.
    aspect = np.array([size[0] / size[1], 1.0])
    palm = float(np.linalg.norm((points[9] - points[0]) * aspect))
    if not math.isfinite(palm) or not MIN_PALM_SCALE <= palm <= math.hypot(*aspect):
        return "low_confidence", None, "Left", confidence
    ratio = float(np.linalg.norm((points[4] - points[8]) * aspect)) / palm
    pinch = float(
        np.clip(
            (ratio - PINCH_CLOSED_RATIO) / (PINCH_OPEN_RATIO - PINCH_CLOSED_RATIO), 0, 1
        )
    )
    sample = {
        "timestamp": float(timestamp),
        "wrist": points[0].tolist(),
        "palm_scale": palm,
        "pinch": pinch,
        "handedness": "Left",
        "confidence": confidence,
    }
    return "left_hand", sample, "Left", confidence


class HandMirrorCamera:
    """One worker owns its camera, landmarker and shared camera reservation."""

    def __init__(
        self,
        root,
        *,
        capture_factory=None,
        tracker_factory=None,
        ownership=None,
        model_path=None,
    ):
        self.root = Path(root)
        self.model_path = (
            Path(model_path)
            if model_path is not None
            else self.root / ".cache/perception/hand_landmarker.task"
        )
        self.capture_factory = capture_factory or cv2.VideoCapture
        self.tracker_factory = tracker_factory or HandTracker
        self.ownership = ownership or CameraOwnership()
        self.lock = threading.RLock()
        self.stopped = threading.Event()
        self.thread = None
        self.tracking_state = "stopped"
        self.error = None
        self.jpeg = self.current_sample = None
        self.stamp = None
        self.resolution = None
        self.handedness = self.confidence = None
        self.model_verified = False
        self.frame_times = deque(maxlen=30)

    def _clear_frame(self):
        self.jpeg = self.current_sample = None
        self.stamp = None
        self.handedness = self.confidence = None
        self.frame_times.clear()

    def _age(self):
        return None if self.stamp is None else max(0.0, time.monotonic() - self.stamp)

    def _expire(self, age):
        if age is not None and age > SAMPLE_MAX_AGE_S:
            self.current_sample = None
        if age is not None and age > FRAME_MAX_AGE_S:
            self.jpeg = None
            self.frame_times.clear()

    def _fresh_sample(self, age):
        return (
            not self.stopped.is_set()
            and self.current_sample is not None
            and age is not None
            and age <= SAMPLE_MAX_AGE_S
        )

    def _fresh_frame(self, age):
        return (
            not self.stopped.is_set()
            and self.jpeg is not None
            and age is not None
            and age <= FRAME_MAX_AGE_S
        )

    def status(self):
        with self.lock:
            age = self._age()
            self._expire(age)
            state, error = self.tracking_state, self.error
            if not self.stopped.is_set() and age is not None and age > SAMPLE_MAX_AGE_S:
                state, error = "unavailable", "Camera frame is stale. Tracking paused."
            fps = 0.0
            if len(self.frame_times) >= 2 and self._fresh_frame(age):
                span = self.frame_times[-1] - self.frame_times[0]
                fps = (len(self.frame_times) - 1) / span if span > 0 else 0.0
            return {
                "running": self.thread is not None and self.thread.is_alive(),
                "frame_available": self._fresh_frame(age),
                "sample_available": self._fresh_sample(age),
                "tracking_state": state,
                "handedness": self.handedness if state != "unavailable" else None,
                "confidence": self.confidence if state != "unavailable" else None,
                "error": error,
                "frame_age_s": age,
                "resolution": list(self.resolution) if self.resolution else None,
                "fps": fps,
                "mirrored": True,
                "model_verified": self.model_verified,
                "simulation_only": True,
            }

    def frame(self):
        with self.lock:
            age = self._age()
            self._expire(age)
            return self.jpeg if self._fresh_frame(age) else None

    def sample(self):
        with self.lock:
            age = self._age()
            self._expire(age)
            if not self._fresh_sample(age):
                return None
            return {**self.current_sample, "wrist": list(self.current_sample["wrist"])}

    def start(self, device=0, width=640, height=480):
        if type(device) is not int or not 0 <= device <= 63:
            raise CalibrationError("Choose a local camera index from 0 to 63.")
        if (
            type(width) is not int
            or type(height) is not int
            or (width, height) not in {(640, 480), (1280, 720)}
        ):
            raise CalibrationError("Choose 640 × 480 or 1280 × 720.")
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                raise CalibrationError(
                    "Stop the current hand camera and wait for it to release first."
                )
            self.ownership.claim(self)
            try:
                self._clear_frame()
                self.resolution, self.error = None, None
                self.model_verified = False
                self.stopped = threading.Event()
                self.tracking_state = "unavailable"
                self.thread = threading.Thread(
                    target=self._run,
                    args=(device, width, height),
                    daemon=True,
                    name="elexhub-hand-camera",
                )
                self.thread.start()
            except Exception:
                self.ownership.release(self)
                raise
        return self.status()

    def _verify_model(self):
        try:
            digest = hashlib.sha256(self.model_path.read_bytes()).hexdigest()
        except OSError as exc:
            raise CalibrationError(
                "Local hand tracking model is missing or unreadable. Prepare the perception model first."
            ) from exc
        if digest != HAND_MODEL_SHA256:
            raise CalibrationError(
                "Hand tracking model checksum does not match the pinned model."
            )
        with self.lock:
            self.model_verified = True

    @staticmethod
    def _annotate(frame, hands, state):
        preview = frame.copy()
        h, w = preview.shape[:2]
        for hand in hands:
            points = _landmarks(hand) if isinstance(hand, dict) else None
            if points is None:
                continue
            confidence = _confidence(hand)
            accepted = (
                hand.get("handedness") == "Left"
                and confidence is not None
                and confidence >= HAND_CONFIDENCE
                and state == "left_hand"
            )
            color = (100, 230, 90) if accepted else (50, 170, 250)
            pixels = np.rint(points * [w - 1, h - 1]).astype(int)
            for a, b in HAND_EDGES:
                cv2.line(preview, tuple(pixels[a]), tuple(pixels[b]), color, 2)
            for point in pixels:
                cv2.circle(preview, tuple(point), 3, color, -1)
        cv2.rectangle(preview, (0, 0), (w, 33), (24, 24, 24), -1)
        cv2.putText(
            preview,
            STATE_LABELS[state],
            (10, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (240, 240, 240),
            1,
            cv2.LINE_AA,
        )
        return preview

    def _run(self, device, width, height):
        capture = tracker = None
        try:
            self._verify_model()
            if self.stopped.is_set():
                return
            tracker = self.tracker_factory(self.model_path)
            if self.stopped.is_set():
                return
            capture = self.capture_factory(device)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                raise CalibrationError(
                    "Cannot detect camera. Check its connection and index."
                )
            while not self.stopped.is_set():
                cycle = time.monotonic()
                ok, raw = capture.read()
                stamp = time.monotonic()
                if self.stopped.is_set():
                    break
                if not ok or raw is None:
                    with self.lock:
                        self._clear_frame()
                        self.tracking_state = "unavailable"
                        self.error = "Cannot detect camera frame. Reconnect or restart the camera."
                    self.stopped.wait(0.05)
                    continue
                if (
                    not isinstance(raw, np.ndarray)
                    or raw.dtype != np.uint8
                    or raw.ndim != 3
                    or raw.shape[2] != 3
                    or min(raw.shape[:2]) < 1
                ):
                    raise CalibrationError("Camera returned an invalid BGR frame.")
                frame = cv2.flip(raw, 1)
                size = (frame.shape[1], frame.shape[0])
                hands = tracker.detect(frame, stamp * 1000)
                state, sample, handedness, confidence = hand_sample(hands, size, stamp)
                preview = self._annotate(
                    frame, hands if isinstance(hands, (list, tuple)) else [], state
                )
                encoded, jpeg = cv2.imencode(
                    ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                if not encoded:
                    raise CalibrationError("Could not prepare the camera preview.")
                with self.lock:
                    if self.stopped.is_set():
                        break
                    self.jpeg = jpeg.tobytes()
                    self.stamp, self.resolution = stamp, size
                    self.current_sample = sample
                    self.handedness, self.confidence = handedness, confidence
                    self.tracking_state, self.error = state, None
                    self.frame_times.append(stamp)
                self.stopped.wait(max(0.0, FRAME_PERIOD_S - (time.monotonic() - cycle)))
        except Exception as exc:  # noqa: BLE001 - camera/inference failure must clear control input
            with self.lock:
                self._clear_frame()
                self.tracking_state, self.error = "unavailable", str(exc)
        finally:
            cleanup_errors = []
            try:
                if capture is not None:
                    try:
                        capture.release()
                    except Exception as exc:  # noqa: BLE001 - release remaining resources too
                        cleanup_errors.append(str(exc))
            finally:
                try:
                    if tracker is not None:
                        try:
                            tracker.close()
                        except Exception as exc:  # noqa: BLE001 - release camera ownership too
                            cleanup_errors.append(str(exc))
                finally:
                    with self.lock:
                        self._clear_frame()
                        if self.stopped.is_set():
                            self.tracking_state = "stopped"
                        if cleanup_errors:
                            self.error = "Camera resource cleanup failed: " + "; ".join(
                                cleanup_errors
                            )
                    self.ownership.release(self)

    def stop(self):
        with self.lock:
            self.stopped.set()
            self._clear_frame()
            self.tracking_state = "stopped"
            thread = self.thread
        if thread is not None:
            thread.join(timeout=2.0)
        return self.status()

    def close(self):
        return self.stop()
