"""Camera preparation with fake devices and synthetic boards; no hardware access."""

from __future__ import annotations

import hashlib
import json
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient

from robollm.elexhub.camera_setup import CameraSetup
from robollm.elexhub.printables import board
from robollm.elexhub.vision import (
    CalibrationError,
    TableCamera,
    calibrate_images,
    calibration_id,
)
from robollm.elexhub.web import create_app


def wait_for(predicate):
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("Fake camera did not reach the expected state")


class FakeCapture:
    def __init__(self):
        self.lock = threading.Lock()
        self.opened = True
        self.image = np.full((480, 640, 3), 235, np.uint8)
        self.released = []

    def set(self, *args):
        return True

    def isOpened(self):
        return self.opened

    def read(self):
        with self.lock:
            return (False, None) if self.image is None else (True, self.image.copy())

    def release(self):
        self.released.append(threading.get_ident())


@pytest.fixture
def rig(tmp_path):
    fake, calls = FakeCapture(), []

    def factory(index):
        calls.append(index)
        return fake

    setup = CameraSetup(tmp_path, capture_factory=factory)
    # No renderer or transport: these routes need only the camera service.
    app = create_app(SimpleNamespace(), tmp_path, camera_setup=setup)
    client = TestClient(app)
    yield SimpleNamespace(setup=setup, fake=fake, calls=calls, client=client, app=app)
    setup.close()
    client.close()


def synthetic_view(index, *, tilted=True):
    """Render the metric board through known intrinsics and a varied plane pose."""
    K = np.array([[700.0, 0, 320], [0, 700.0, 240], [0, 0, 1]])
    source = board().generateImage((1400, 1000), marginSize=0, borderBits=1)
    plane = np.array([[0.0, 0, 0], [0.21, 0, 0], [0.21, 0.15, 0], [0, 0.15, 0]])
    rx = [-0.35, 0, 0.35][index // 4] if tilted else 0.0
    ry = [-0.35, -0.12, 0.12, 0.35][index % 4] if tilted else 0.0
    rvec = np.array([rx, ry, 0.06 * ((index % 3) - 1)])
    rotation = cv2.Rodrigues(rvec)[0]
    center = np.array(
        [
            0.025 * ((index % 3) - 1),
            0.015 * ((index % 4) - 1.5),
            0.44 + 0.025 * (index % 3),
        ]
    )
    translation = center - rotation @ np.array([0.105, 0.075, 0])
    target = cv2.projectPoints(plane, rvec, translation, K, np.zeros(5))[0]
    transform = cv2.getPerspectiveTransform(
        np.array([[0, 0], [1399, 0], [1399, 999], [0, 999]], np.float32),
        target.reshape(4, 2).astype(np.float32),
    )
    gray = cv2.warpPerspective(source, transform, (640, 480), borderValue=235)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def show(rig, image):
    with rig.fake.lock:
        rig.fake.image = image

    def published():
        with rig.setup.lock:
            return rig.setup.raw is not None and np.array_equal(rig.setup.raw, image)

    wait_for(published)


def test_reads_and_rejected_requests_never_open_camera(rig, tmp_path):
    client = rig.client
    assert client.get("/camera").status_code == 200
    assert client.get("/api/camera-setup/state").json()["state"] == "idle"
    assert client.get("/api/camera-setup/frame.jpg").status_code == 204
    assert client.get("/api/camera-setup/intrinsics.json").status_code == 400
    assert client.post("/api/camera-setup/capture").status_code == 400
    assert client.post("/api/camera-setup/calibrate").status_code == 400
    assert (
        client.post(
            "/api/camera-setup/start",
            json={},
            headers={"Origin": "https://external.example"},
        ).status_code
        == 403
    )
    for device in ["0", True, -1, 64]:
        assert (
            client.post("/api/camera-setup/start", json={"device": device}).status_code
            == 422
        )
    assert (
        client.post("/api/camera-setup/start", json={"width": 999}).status_code == 400
    )
    assert not rig.calls
    assert not (tmp_path / "captures").exists()


def test_failed_open_returns_cannot_detect_and_releases_on_owner_thread(rig):
    rig.fake.opened = False
    assert rig.client.post("/api/camera-setup/start", json={}).status_code == 200
    wait_for(lambda: not rig.setup.status()["running"])
    state = rig.client.get("/api/camera-setup/state").json()
    assert state["state"] == "unavailable" and "Cannot detect" in state["error"]
    assert rig.client.get("/api/camera-setup/frame.jpg").status_code == 204
    assert len(rig.fake.released) == 1
    assert rig.fake.released[0] != threading.get_ident()
    rig.fake.opened = True
    rig.setup.start()
    wait_for(lambda: rig.setup.status()["frame_available"])
    rig.setup.stop()
    assert len(rig.fake.released) == 2


def test_capture_uses_raw_image_and_clears_disconnected_or_stale_preview(rig):
    rig.setup.start()
    wait_for(lambda: rig.setup.status()["frame_available"])
    assert not rig.setup.status()["can_capture"]
    assert rig.client.post("/api/camera-setup/capture").status_code == 400
    expected = synthetic_view(0)
    show(rig, expected)
    assert rig.client.post("/api/camera-setup/capture").json()["views"] == 1
    stored = cv2.imread(str(rig.setup.session / "view-001.png"))
    np.testing.assert_array_equal(stored, expected)
    assert rig.client.post("/api/camera-setup/capture").status_code == 400
    assert rig.client.get("/api/camera-setup/frame.jpg").status_code == 200
    # Raw calibration frames must never enter the robot's calibrated observations.
    assert rig.client.get("/api/frame.jpg?camera=true").status_code == 204
    with rig.setup.lock:
        rig.setup.stamp = time.monotonic() - 5
        assert rig.setup.frame() is None
        assert not rig.setup.status()["can_capture"]
    with rig.fake.lock:
        rig.fake.image = None
    wait_for(lambda: rig.setup.status()["error"] is not None)
    assert rig.client.get("/api/camera-setup/frame.jpg").status_code == 204
    assert rig.client.post("/api/camera-setup/capture").status_code == 400
    show(rig, synthetic_view(1))
    assert rig.setup.status()["can_capture"]


def test_full_capture_calibration_download_and_integrity(rig):
    rig.setup.start()
    for index in range(12):
        show(rig, synthetic_view(index))
        response = rig.client.post("/api/camera-setup/capture")
        assert response.status_code == 200, response.text
    rig.setup.stop()
    assert rig.client.get("/api/camera-setup/frame.jpg").status_code == 204
    response = rig.client.post("/api/camera-setup/calibrate")
    assert response.status_code == 200, response.text
    result = response.json()["calibration"]
    assert result["views"] == 12 and result["rms_px"] < 0.6
    assert result["registered_for_robot"] is False
    response = rig.client.get("/api/camera-setup/intrinsics.json")
    assert response.status_code == 200
    data = response.json()
    assert data["calibration_id"] == calibration_id(data) == result["calibration_id"]
    assert data["width"] == 640 and data["height"] == 480
    np.testing.assert_allclose(np.diag(data["K"])[:2], [700, 700], atol=12)
    assert data["T_base_mat"] is data["T_toolmarker_tcp"] is None
    with pytest.raises(CalibrationError):
        TableCamera(data).require_hardware()
    manifest = json.loads((rig.setup.session / "SESSION.json").read_text())
    assert manifest["actual_size"] == [640, 480]
    for view in manifest["views"]:
        actual = hashlib.sha256(
            (rig.setup.session / view["file"]).read_bytes()
        ).hexdigest()
        assert (
            actual
            == view["sha256"]
            == data["capture_manifest"]["image_sha256"][view["file"]]
        )
    changed = rig.setup.session / manifest["views"][-1]["file"]
    changed.write_bytes(b"changed after capture")
    assert rig.client.post("/api/camera-setup/calibrate").status_code == 400
    assert rig.client.get("/api/camera-setup/intrinsics.json").status_code == 400
    changed.unlink()
    assert rig.client.post("/api/camera-setup/calibrate").status_code == 400
    previous = rig.setup.session
    rig.setup.start()
    assert rig.setup.session != previous and previous.is_dir()
    assert rig.setup.status()["views"] == 0


def test_parallel_frontal_views_do_not_pass_intrinsics_check(tmp_path):
    paths = []
    for index in range(12):
        path = tmp_path / f"front-{index}.png"
        cv2.imwrite(str(path), synthetic_view(index, tilted=False))
        paths.append(path)
    with pytest.raises(CalibrationError):
        calibrate_images(paths, tmp_path / "invalid.json")
    assert not (tmp_path / "invalid.json").exists()


@pytest.mark.parametrize(
    "mode,has_webcam", [("shadow", False), ("hardware", False), ("sim", True)]
)
def test_setup_cannot_compete_with_experiment_camera(tmp_path, mode, has_webcam):
    calls = []
    setup = CameraSetup(tmp_path, capture_factory=lambda index: calls.append(index))
    webcam = SimpleNamespace(observe=dict) if has_webcam else None
    app = create_app(
        SimpleNamespace(), tmp_path, mode=mode, webcam=webcam, camera_setup=setup
    )
    client = TestClient(app)
    assert not client.get("/api/camera-setup/state").json()["enabled"]
    assert client.post("/api/camera-setup/start", json={}).status_code == 400
    assert not calls
