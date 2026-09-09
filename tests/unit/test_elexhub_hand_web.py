"""Mirror routes with a real simulator and an injected, camera-free source."""

from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import time

import pytest
from fastapi.testclient import TestClient
from robo_arm_driver.config import load_arm_config

from robollm.elexhub.camera_ownership import CameraOwnership
from robollm.elexhub.simulation import SimArm
from robollm.elexhub.web import create_app


ROOT = Path(__file__).resolve().parents[2]


class FakeHandCamera:
    def __init__(self):
        self.ownership = CameraOwnership()
        self.running = False
        self.calls = []
        self.jpeg = None

    def status(self):
        return {
            "running": self.running,
            "frame_available": self.jpeg is not None,
            "sample_available": False,
            "tracking_state": "no_hand",
        }

    def sample(self):
        return None

    def frame(self):
        return self.jpeg

    def start(self, device=0, width=640, height=480):
        self.ownership.claim(self)
        self.calls.append(device)
        self.running = True

    def stop(self):
        self.running = False
        self.jpeg = None
        self.ownership.release(self)

    close = stop


@pytest.fixture
def rig(tmp_path):
    config = load_arm_config(ROOT / "ros2/robo_arm_driver/config/elexhub.sim.yaml")
    arm = SimArm(config, ROOT / "assets/elexhub/provisional", rebuild=False)
    hand = FakeHandCamera()
    app = create_app(arm, tmp_path, hand_camera=hand)
    client = TestClient(app)
    yield SimpleNamespace(arm=arm, hand=hand, app=app, client=client)
    app.state.hand_control.close()
    hand.close()
    arm.close()
    client.close()


def test_page_state_and_missing_feed_never_open_camera(rig):
    assert rig.client.get("/mirror").status_code == 200
    s = rig.client.get("/api/hand-mirror/state").json()
    assert s["enabled"] and not s["physical_robot_control"]
    assert not s["controller"]["running"] and not s["camera"]["running"]
    assert s["model"]["dimensions"] == "estimated_from_photos"
    rig.app.state.runtime.update(jpeg=b"simulation", jpeg_at=time.monotonic())
    assert rig.client.get("/api/frame.jpg").content == b"simulation"
    assert rig.client.get("/api/hand-mirror/frame.jpg").status_code == 204
    assert rig.client.get("/api/frame.jpg?camera=true").status_code == 204
    assert not rig.hand.calls


def test_origin_schema_and_start_order(rig):
    c = rig.client
    assert c.post("/api/hand-mirror/start", json={}).status_code == 400
    for device in [-1, 64, "0", True]:
        assert (
            c.post("/api/hand-mirror/camera/start", json={"device": device}).status_code
            == 422
        )
    assert (
        c.post(
            "/api/hand-mirror/camera/start",
            json={},
            headers={"Origin": "https://other.example"},
        ).status_code
        == 403
    )
    assert not rig.hand.calls


def test_mirror_and_task_exclude_each_other_and_invalidate_preview(rig):
    c, task = rig.client, rig.app.state.task
    assert c.post("/api/hand-mirror/camera/start", json={}).status_code == 200
    pending = Future()
    rig.app.state.runtime["future"] = pending
    assert c.post("/api/hand-mirror/start", json={}).status_code == 400
    pending.set_result(None)
    task.previewed = ("old-preview", {})
    assert c.post("/api/hand-mirror/start", json={}).status_code == 200
    assert task.previewed is None
    assert c.get("/api/state").json()["hand_mirror_active"]
    assert (
        c.post(
            "/api/preview",
            json={"instruction": "reach here", "destination": [0.09, 0.08]},
        ).status_code
        == 400
    )
    assert (
        c.post(
            "/api/execute", json={"preview_id": "old-preview", "workspace_clear": True}
        ).status_code
        == 400
    )
    assert c.post("/api/hand-mirror/heartbeat", json={}).status_code == 200
    assert c.post("/api/hand-mirror/stop", json={}).status_code == 200
    assert not rig.app.state.hand_control.status()["running"]
    assert rig.hand.running


@pytest.mark.parametrize(
    "endpoint", ["/api/reset", "/api/abort", "/api/hand-mirror/camera/stop"]
)
def test_reset_abort_and_camera_off_stop_motion(rig, endpoint):
    c = rig.client
    c.post("/api/hand-mirror/camera/start", json={})
    c.post("/api/hand-mirror/start", json={})
    assert c.post(endpoint, json={}).status_code == 200
    assert not rig.app.state.hand_control.status()["running"]
    if endpoint != "/api/reset":
        assert not rig.hand.running
        assert c.get("/api/hand-mirror/frame.jpg").status_code == 204


def test_camera_pages_share_exclusive_ownership(rig):
    owner = object()
    setup = rig.app.state.camera_setup
    setup.ownership.claim(owner)
    assert rig.client.post("/api/hand-mirror/camera/start", json={}).status_code == 400
    assert not rig.hand.calls
    setup.ownership.release(owner)
    assert rig.client.post("/api/hand-mirror/camera/start", json={}).status_code == 200
    # Calibration startup must refuse before opening another device or creating a session.
    setup.capture_factory = lambda device: pytest.fail("competing camera opened")
    assert rig.client.post("/api/camera-setup/start", json={}).status_code == 400
    assert setup.session is None


@pytest.mark.parametrize(
    "mode,webcam",
    [("hardware", None), ("shadow", None), ("sim", SimpleNamespace(observe=dict))],
)
def test_non_simulation_and_registered_camera_cannot_start_mirror(
    rig, tmp_path, mode, webcam
):
    hand = FakeHandCamera()
    app = create_app(rig.arm, tmp_path, mode=mode, webcam=webcam, hand_camera=hand)
    client = TestClient(app)
    assert not client.get("/api/hand-mirror/state").json()["enabled"]
    for endpoint in ["camera/start", "start", "recenter", "heartbeat"]:
        assert client.post("/api/hand-mirror/" + endpoint, json={}).status_code == 400
    assert not hand.calls
    assert client.get("/api/hand-mirror/frame.jpg").status_code == 204
    client.close()
