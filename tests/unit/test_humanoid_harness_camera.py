"""Camera privacy/lifecycle and observation gates using generated pixels only."""

import importlib.util
import threading
import time
from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[2] / "examples/humanoid_harness/camera.py"
spec = importlib.util.spec_from_file_location("humanoid_demo_camera", SOURCE)
camera_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(camera_module)
CameraInput = camera_module.CameraInput


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for camera worker")


class FakeCapture:
    def __init__(self, frame, opened=True):
        self.frame, self.opened = frame, opened
        self.released = False

    def isOpened(self):
        return self.opened

    def set(self, *_):
        return True

    def read(self):
        return self.frame is not None, self.frame

    def release(self):
        self.released = True


class FakeTracker:
    name, label = "scripted_test", "SCRIPTED TEST INPUT"

    def __init__(self):
        self.result = {"tracked": True, "target": [0.4, 0.0, 0.6], "message": "test"}
        self.closed = False

    def process(self, *_):
        return dict(self.result)

    def close(self):
        self.closed = True


@pytest.fixture
def pixels():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    return np.zeros((120, 160, 3), dtype=np.uint8)


def test_construction_never_opens_camera():
    calls = []
    camera = CameraInput(capture_factory=lambda device: calls.append(device))
    assert calls == []
    assert camera.snapshot()["running"] is False
    assert camera.snapshot()["tracked"] is False
    assert camera.jpeg() is None
    assert camera.stop()["running"] is False
    assert calls == []


def test_normalized_image_mapping_has_fixed_depth_and_mirrored_x():
    assert camera_module.image_target(0, 0) == [1.0, 0.0, 1.0]
    assert camera_module.image_target(1, 1) == [-1.0, 0.0, -1.0]
    assert camera_module.image_target(0.5, 0.5) == [0.0, 0.0, 0.0]
    with pytest.raises(ValueError):
        camera_module.image_target(float("nan"), 0)


def test_loss_holds_last_target_and_stop_releases_preview_and_resources(pixels):
    capture, tracker = FakeCapture(pixels), FakeTracker()
    camera = CameraInput(fps=30, capture_factory=lambda _: capture,
                         tracker_factory=lambda _: tracker)
    try:
        camera.start()
        wait_for(lambda: camera.snapshot()["tracked"])
        target = camera.snapshot()["target"]
        assert camera.jpeg().startswith(b"\xff\xd8")
        tracker.result = {"tracked": False, "message": "missing hand"}
        wait_for(lambda: not camera.snapshot()["tracked"])
        assert camera.snapshot()["target"] == target
        assert camera.snapshot()["confidence"] == 0.0
    finally:
        camera.stop()
    assert capture.released and tracker.closed
    assert camera.jpeg() is None
    assert not camera.snapshot()["running"]


def test_failed_open_closes_detector_and_capture(pixels):
    capture, tracker = FakeCapture(pixels, opened=False), FakeTracker()
    camera = CameraInput(capture_factory=lambda _: capture, tracker_factory=lambda _: tracker)
    with pytest.raises(RuntimeError, match="could not open"):
        camera.start()
    assert capture.released and tracker.closed
    assert not camera.snapshot()["running"]
    assert camera.jpeg() is None


def test_nonfinite_detector_signal_is_rejected(pixels):
    capture, tracker = FakeCapture(pixels), FakeTracker()
    tracker.result["target"] = [float("nan"), 0, 0]
    camera = CameraInput(capture_factory=lambda _: capture, tracker_factory=lambda _: tracker)
    try:
        camera.start()
        wait_for(lambda: camera.snapshot()["frames"] > 0)
        assert not camera.snapshot()["tracked"]
        assert camera.snapshot()["target"] == [0, 0, 0]
        assert "Invalid target" in camera.snapshot()["message"]
    finally:
        camera.stop()


def test_stale_frame_is_not_reported_as_live(pixels):
    capture, tracker = FakeCapture(pixels), FakeTracker()
    gate = threading.Event()
    original_process = tracker.process

    def process(*args):
        if gate.is_set():
            time.sleep(0.2)
        return original_process(*args)

    tracker.process = process
    camera = CameraInput(fps=30, stale_after=0.06, capture_factory=lambda _: capture,
                         tracker_factory=lambda _: tracker)
    try:
        camera.start()
        wait_for(lambda: camera.snapshot()["tracked"])
        gate.set()
        wait_for(lambda: "stale" in camera.snapshot()["message"])
        assert not camera.snapshot()["tracked"]
        assert camera.snapshot()["confidence"] == 0
    finally:
        camera.stop()


def test_red_marker_is_explicit_and_tracks_generated_pixels(pixels):
    import cv2

    tracker = camera_module._RedMarkerTracker(cv2)
    assert tracker.name == "red_marker"
    assert not tracker.process(pixels, 0)["tracked"]
    cv2.circle(pixels, (40, 30), 10, (0, 0, 255), -1)
    result = tracker.process(pixels, 1)
    assert result["tracked"]
    assert result["target"] == pytest.approx([0.5, 0.0, 0.5])
    assert "not hand tracking" in result["message"]


def test_start_stop_can_repeat_without_reusing_released_capture(pixels):
    captures, trackers = [], []

    def make_capture(_):
        captures.append(FakeCapture(pixels))
        return captures[-1]

    def make_tracker(_):
        trackers.append(FakeTracker())
        return trackers[-1]

    camera = CameraInput(capture_factory=make_capture, tracker_factory=make_tracker)
    for _ in range(2):
        try:
            camera.start()
            camera.start()  # Idempotent while running.
            wait_for(lambda: camera.snapshot()["tracked"])
        finally:
            camera.stop()
    assert len(captures) == len(trackers) == 2
    assert all(c.released for c in captures)
    assert all(t.closed for t in trackers)
