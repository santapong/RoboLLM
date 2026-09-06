"""Optional local webcam capture; frames stay on this machine."""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np

from .vision import CalibrationError, HandTracker, TableCamera


class Webcam:
    def __init__(self, device, calibration, hand_model):
        self.camera = TableCamera(calibration)
        self.hands = HandTracker(hand_model)
        self.capture = cv2.VideoCapture(device)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera.size[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera.size[1])
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.capture.isOpened():
            self.hands.close()
            raise CalibrationError("webcam could not be opened")
        self.lock = threading.Lock()
        self.latest = {"reference_visible": False, "hand_in_workspace": True}
        self.jpeg = None
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stopped.is_set():
            ok, frame = self.capture.read()
            stamp = time.monotonic()
            if not ok:
                with self.lock:
                    self.latest = {
                        "error": "camera disconnected",
                        "reference_visible": False,
                        "captured_at_monotonic": stamp,
                        "hand_in_workspace": True,
                    }
                self.stopped.wait(0.05)
                continue
            try:
                observation = self.camera.observe(frame, captured_at=stamp)
                hands = self.hands.detect(frame, stamp * 1000)
                # Conservative assist: any detected hand blocks motion. This is
                # not a safety-rated detector or replacement for the cutoff.
                observation["hand_in_workspace"] = bool(hands)
                observation["pointing_mat_m"] = None
                if len(hands) == 1:
                    point = self.camera.pixel_to_mat(
                        hands[0]["tip_px"], observation["mat_pose"]
                    )
                    if 0 <= point[0] <= 0.18 and 0 <= point[1] <= 0.12:
                        observation["pointing_mat_m"] = point[:2].tolist()
                for hand in hands:
                    for x, y in hand["landmarks_px"]:
                        cv2.circle(frame, (int(x), int(y)), 3, (80, 220, 250), -1)
                observation["pointing_method"] = (
                    "fingertip pixel projected onto tabletop; touch/hover selection"
                )
            except Exception as exc:  # noqa: BLE001 - any perception failure blocks motion
                observation = {
                    "captured_at_monotonic": stamp,
                    "calibration_id": self.camera.id,
                    "reference_visible": False,
                    "hand_in_workspace": True,
                    "error": str(exc),
                }
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self.lock:
                self.latest = observation
                self.jpeg = encoded.tobytes() if ok else None

    def observe(self):
        with self.lock:
            return dict(self.latest)

    def public_observation(self):
        value = self.observe()
        value.pop("mat_pose", None)
        return value

    def pixel(self, xy):
        observation = self.observe()
        if (
            time.monotonic() - observation.get("captured_at_monotonic", 0) > 0.30
            or "mat_pose" not in observation
        ):
            raise CalibrationError("fresh workspace reference required for selection")
        point = self.camera.pixel_to_mat(np.asarray(xy, float), observation["mat_pose"])
        if not 0 <= point[0] <= 0.18 or not 0 <= point[1] <= 0.12:
            raise CalibrationError("selected pixel is outside the grid")
        return point[:2].tolist()

    def close(self):
        self.stopped.set()
        self.capture.release()
        self.thread.join(timeout=2)
        if not self.thread.is_alive():
            self.hands.close()
