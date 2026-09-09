"""Operator-started webcam intrinsics capture, separate from arm control."""

from __future__ import annotations

import hashlib
import json
import threading
import time
import uuid
from pathlib import Path

import cv2
from fastapi import APIRouter
from fastapi.responses import FileResponse, HTMLResponse, Response
from pydantic import BaseModel, ConfigDict, Field

from .camera_ownership import CameraOwnership
from .printables import board
from .vision import CalibrationError, calibrate_images, save_calibration


class CameraRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    device: int = Field(default=0, ge=0, le=63)
    width: int = 640
    height: int = 480


class CameraSetup:
    """One camera owner; no camera is opened by construction or status reads."""

    def __init__(self, root, *, enabled=True, capture_factory=None, ownership=None):
        self.root = Path(root)
        self.enabled = enabled
        self.ownership = ownership or CameraOwnership()
        self.capture_factory = capture_factory or cv2.VideoCapture
        self.lock = threading.RLock()
        self.stopped = threading.Event()
        self.thread = None
        self.session = None
        self.settings = None
        self.state = "idle"
        self.error = None
        self.jpeg = self.raw = None
        self.stamp = 0.0
        self.corners = 0
        self.size = None
        self.views = []
        self.result = self.result_path = None
        self.calibrating = False

    def _clear_frame(self):
        self.jpeg = self.raw = None
        self.stamp = 0.0
        self.corners = 0

    def _fresh(self):
        return (
            self.raw is not None
            and not self.stopped.is_set()
            and time.monotonic() - self.stamp <= 1.0
        )

    def status(self):
        with self.lock:
            fresh = self._fresh()
            running = self.thread is not None and self.thread.is_alive()
            return {
                "enabled": self.enabled,
                "state": self.state if fresh or not running else "waiting_for_frame",
                "running": running,
                "frame_available": fresh and self.jpeg is not None,
                "corners": self.corners if fresh else 0,
                "resolution": self.size,
                "views": len(self.views),
                "minimum_views": 12,
                "recommended_views": 20,
                "maximum_views": 40,
                "can_capture": fresh
                and self.corners >= 10
                and len(self.views) < 40
                and not self.calibrating,
                "calibrating": self.calibrating,
                "error": self.error,
                "calibration": self.result,
                "session_directory": str(self.session.relative_to(self.root))
                if self.session
                else None,
                "robot_control": False,
            }

    def start(self, device=0, width=640, height=480):
        if not self.enabled:
            raise CalibrationError(
                "Camera setup requires simulation mode without another webcam owner."
            )
        if type(device) is not int or not 0 <= device <= 63:
            raise CalibrationError("Choose a local camera index from 0 to 63.")
        if (width, height) not in {(640, 480), (1280, 720)}:
            raise CalibrationError("Choose 640 × 480 or 1280 × 720.")
        with self.lock:
            if self.calibrating or (self.thread and self.thread.is_alive()):
                raise CalibrationError("Stop the current camera session first.")
            self.ownership.claim(self)
            try:
                self.session = (
                    self.root / "captures/camera-setup" / f"session-{uuid.uuid4().hex}"
                )
                self.session.mkdir(parents=True, mode=0o700)
                self.settings = {"device": device, "requested_size": [width, height]}
                self.views = []
                self.size = self.result = self.result_path = self.error = None
                self._clear_frame()
                self.stopped = threading.Event()
                self.state = "starting"
                self._save_session()
                self.thread = threading.Thread(
                    target=self._run, args=(device, width, height), daemon=True
                )
                self.thread.start()
            except Exception:
                self.ownership.release(self)
                raise
        return self.status()

    def _run(self, device, width, height):
        capture = None
        try:
            capture = self.capture_factory(device)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                raise CalibrationError(
                    "Cannot detect camera. Check its connection and index."
                )
            detector = cv2.aruco.CharucoDetector(board())
            while not self.stopped.is_set():
                ok, frame = capture.read()
                if self.stopped.is_set():
                    break
                if not ok or frame is None:
                    with self.lock:
                        self._clear_frame()
                        self.state = "unavailable"
                        self.error = "Cannot detect camera frame. Reconnect or restart the camera."
                    self.stopped.wait(0.1)
                    continue
                stamp = time.monotonic()
                size = [frame.shape[1], frame.shape[0]]
                found, labels, _, _ = detector.detectBoard(frame)
                count = 0 if labels is None else len(labels)
                preview = frame.copy()
                if count:
                    cv2.aruco.drawDetectedCornersCharuco(preview, found, labels)
                encoded, jpeg = cv2.imencode(
                    ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                with self.lock:
                    if self.size and size != self.size:
                        raise CalibrationError(
                            "Camera resolution changed. Start a new capture session."
                        )
                    self.size = size
                    self.raw = frame.copy()
                    self.jpeg = jpeg.tobytes() if encoded else None
                    self.stamp, self.corners = stamp, count
                    self.state, self.error = "preview", None
                self.stopped.wait(0.08)
        except Exception as exc:  # noqa: BLE001 - isolate device failures from the console
            with self.lock:
                self._clear_frame()
                self.state, self.error = "unavailable", str(exc)
        finally:
            # Only the owning worker releases the device, including failed opens.
            try:
                if capture is not None:
                    capture.release()
            finally:
                with self.lock:
                    self._clear_frame()
                    if self.stopped.is_set():
                        self.state = "stopped"
                self.ownership.release(self)

    def stop(self):
        with self.lock:
            self.stopped.set()
            self._clear_frame()
            thread = self.thread
            self.state = "stopping" if thread and thread.is_alive() else "stopped"
        if thread:
            thread.join(timeout=2)
        with self.lock:
            if not thread or not thread.is_alive():
                self.state = "stopped"
        return self.status()

    def frame(self):
        with self.lock:
            return self.jpeg if self._fresh() else None

    def _save_session(self):
        data = {
            "schema_version": 1,
            "purpose": "camera intrinsics; robot/base registration not performed",
            "source": "operator-started local webcam",
            "settings": self.settings,
            "actual_size": self.size,
            "views": self.views,
        }
        (self.session / "SESSION.json").write_text(json.dumps(data, indent=2) + "\n")

    def capture_view(self):
        with self.lock:
            if self.calibrating:
                raise CalibrationError(
                    "Calibration is running; wait before capturing another view."
                )
            if not self._fresh():
                raise CalibrationError("Cannot detect a fresh camera frame.")
            if self.corners < 10:
                raise CalibrationError(
                    "Show at least 10 ChArUco corners before capturing."
                )
            if len(self.views) >= 40:
                raise CalibrationError("This session already has 40 views.")
            ok, encoded = cv2.imencode(".png", self.raw)
            if not ok:
                raise CalibrationError("Could not save this camera view.")
            image = encoded.tobytes()
            digest = hashlib.sha256(image).hexdigest()
            if any(v["sha256"] == digest for v in self.views):
                raise CalibrationError(
                    "This frame was already saved. Move or tilt the board."
                )
            name = f"view-{len(self.views) + 1:03d}.png"
            (self.session / name).write_bytes(image)
            self.views.append(
                {
                    "file": name,
                    "sha256": digest,
                    "corners": self.corners,
                    "captured_at_unix": time.time(),
                }
            )
            self.result = self.result_path = None
            self._save_session()
        return self.status()

    def calibrate(self):
        with self.lock:
            if self.calibrating:
                raise CalibrationError("Calibration is already running.")
            if len(self.views) < 12:
                raise CalibrationError(
                    "Capture at least 12 usable views; 20 is recommended."
                )
            self.calibrating = True
            session, views = self.session, list(self.views)
            self.result = self.result_path = self.error = None
        output = session / f"intrinsics-{uuid.uuid4().hex[:12]}.json"
        try:
            for view in views:
                if (
                    hashlib.sha256((session / view["file"]).read_bytes()).hexdigest()
                    != view["sha256"]
                ):
                    raise CalibrationError(
                        "A saved view changed. Start a new capture session."
                    )
            data = calibrate_images([session / v["file"] for v in views], output)
            data["capture_manifest"] = {
                "settings": self.settings,
                "image_sha256": {v["file"]: v["sha256"] for v in views},
            }
            data = save_calibration(data, output)
            with self.lock:
                self.result_path = output
                self.result = {
                    "calibration_id": data["calibration_id"],
                    "rms_px": data["rms_px"],
                    "views": len(data["views"]),
                    "resolution": [data["width"], data["height"]],
                    "registered_for_robot": False,
                    "file": str(output.relative_to(self.root)),
                }
        except (OSError, cv2.error) as exc:
            with self.lock:
                self.error = (
                    "Could not read or calibrate the saved views. Start a new session."
                )
            raise CalibrationError(self.error) from exc
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            raise
        finally:
            with self.lock:
                self.calibrating = False
        return self.status()

    def calibration_file(self):
        with self.lock:
            if self.result_path is None:
                raise CalibrationError(
                    "No completed intrinsics calibration is available."
                )
            return self.result_path

    def close(self):
        self.stop()


def camera_router(setup):
    router = APIRouter()

    @router.get("/camera")
    def page():
        return HTMLResponse(Path(__file__).with_name("camera_setup.html").read_text())

    @router.get("/api/camera-setup/state")
    def status():
        return setup.status()

    @router.get("/api/camera-setup/frame.jpg")
    def frame():
        jpeg = setup.frame()
        return (
            Response(jpeg, media_type="image/jpeg")
            if jpeg
            else Response(status_code=204)
        )

    @router.post("/api/camera-setup/start")
    def start(body: CameraRequest):
        return setup.start(body.device, body.width, body.height)

    @router.post("/api/camera-setup/stop")
    def stop():
        return setup.stop()

    @router.post("/api/camera-setup/capture")
    def capture():
        return setup.capture_view()

    @router.post("/api/camera-setup/calibrate")
    def calibrate():
        return setup.calibrate()

    @router.get("/api/camera-setup/intrinsics.json")
    def download():
        return FileResponse(
            setup.calibration_file(),
            media_type="application/json",
            filename="elexhub-camera-intrinsics.json",
        )

    return router
