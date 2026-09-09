"""LEFT-hand camera behavior with fake devices; no physical camera access."""

from __future__ import annotations

import copy
import hashlib
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from robollm.elexhub import hand_camera
from robollm.elexhub.hand_camera import HandMirrorCamera, hand_sample
from robollm.elexhub.vision import CalibrationError, HandTracker


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Fake hand camera did not reach the expected state")


def hand(label="Left", confidence=0.95, *, ratio=0.5, aspect=4 / 3):
    points = [[0.5, 0.5] for _ in range(21)]
    points[0], points[9] = [0.4, 0.7], [0.4, 0.5]
    points[4], points[8] = [0.3, 0.4], [0.3 + ratio * 0.2 / aspect, 0.4]
    return {"handedness": label, "confidence": confidence, "landmarks_norm": points}


class FakeCapture:
    def __init__(self):
        self.lock = threading.Lock()
        self.image = np.zeros((480, 640, 3), np.uint8)
        self.image[:, :320] = [255, 0, 0]
        self.image[:, 320:] = [0, 0, 255]
        self.opened = True
        self.block = False
        self.blocked = threading.Event()
        self.unblock = threading.Event()
        self.released = []

    def set(self, *_):
        return True

    def isOpened(self):
        return self.opened

    def read(self):
        if self.block:
            self.blocked.set()
            self.unblock.wait(timeout=5)
        with self.lock:
            return (False, None) if self.image is None else (True, self.image.copy())

    def release(self):
        self.released.append(threading.get_ident())


class FakeTracker:
    def __init__(self):
        self.lock = threading.Lock()
        self.hands = [hand()]
        self.frames = []
        self.timestamps = []
        self.error = None
        self.closed = []

    def detect(self, frame, timestamp):
        with self.lock:
            if self.error:
                raise self.error
            self.frames.append(frame.copy())
            self.timestamps.append(timestamp)
            return copy.deepcopy(self.hands)

    def close(self):
        self.closed.append(threading.get_ident())


@pytest.fixture
def rig(tmp_path, monkeypatch):
    model = tmp_path / ".cache/perception/hand_landmarker.task"
    model.parent.mkdir(parents=True)
    model.write_bytes(b"unit-test hand model")
    monkeypatch.setattr(
        hand_camera, "HAND_MODEL_SHA256", hashlib.sha256(model.read_bytes()).hexdigest()
    )
    capture, tracker = FakeCapture(), FakeTracker()
    calls, tracker_calls = [], []

    def camera_factory(device):
        calls.append((device, threading.get_ident()))
        return capture

    def tracker_factory(path):
        tracker_calls.append((path, threading.get_ident()))
        return tracker

    camera = HandMirrorCamera(
        tmp_path, capture_factory=camera_factory, tracker_factory=tracker_factory
    )
    yield SimpleNamespace(
        camera=camera,
        capture=capture,
        tracker=tracker,
        calls=calls,
        tracker_calls=tracker_calls,
        model=model,
        root=tmp_path,
    )
    capture.unblock.set()
    camera.close()


def test_selection_rejects_right_unknown_weak_and_multiple_left():
    now = time.monotonic()
    cases = [
        ([], "no_hand"),
        ([hand("Right")], "right_hand"),
        ([hand("Unknown")], "low_confidence"),
        ([hand(confidence=0.64)], "low_confidence"),
        ([hand(), hand(confidence=0.1)], "ambiguous"),
        ([hand(), hand()], "ambiguous"),
    ]
    for hands, expected in cases:
        state, sample, _, _ = hand_sample(hands, (640, 480), now)
        assert state == expected
        assert sample is None
    state, sample, label, confidence = hand_sample(
        [hand("Right"), hand()], (640, 480), now
    )
    assert state == "left_hand" and label == "Left" and confidence == 0.95
    assert sample["wrist"] == [0.4, 0.7]
    assert sample["timestamp"] == now


@pytest.mark.parametrize("aspect,size", [(4 / 3, (640, 480)), (16 / 9, (1280, 720))])
def test_palm_scale_and_pinch_are_aspect_correct(aspect, size):
    for ratio, expected in [(0.05, 0), (0.15, 0), (0.575, 0.5), (1.0, 1), (1.2, 1)]:
        sample = hand_sample([hand(ratio=ratio, aspect=aspect)], size, 1.0)[1]
        assert sample["palm_scale"] == pytest.approx(0.2)
        assert sample["pinch"] == pytest.approx(expected)
    horizontal = hand()
    horizontal["landmarks_norm"][9] = [0.6, 0.7]
    sample = hand_sample([horizontal], size, 1.0)[1]
    assert sample["palm_scale"] == pytest.approx(0.2 * aspect)


@pytest.mark.parametrize(
    "field,value",
    [
        ("confidence", float("nan")),
        ("confidence", float("inf")),
        ("confidence", 1.1),
        ("confidence", True),
        ("landmarks_norm", []),
        ("landmarks_norm", None),
    ],
)
def test_invalid_detection_cannot_produce_input(field, value):
    invalid = hand()
    invalid[field] = value
    assert hand_sample([invalid], (640, 480), 1.0)[1] is None


@pytest.mark.parametrize(
    "point",
    [
        [float("nan"), 0.5],
        [0.5, float("inf")],
        [-0.1, 0.5],
        [1.1, 0.5],
        [True, 0.5],
        [0.5],
    ],
)
def test_invalid_landmark_cannot_produce_input(point):
    invalid = hand()
    invalid["landmarks_norm"][7] = point
    assert hand_sample([invalid], (640, 480), 1.0)[1] is None


def test_degenerate_palm_and_invalid_timestamp_rejected():
    invalid = hand()
    invalid["landmarks_norm"][9] = invalid["landmarks_norm"][0]
    assert hand_sample([invalid], (640, 480), 1.0)[1] is None
    for stamp in [float("nan"), float("inf"), -1, True]:
        assert hand_sample([hand()], (640, 480), stamp)[1] is None
    for size in [None, [], [640], [640, 0], [True, 480]]:
        assert hand_sample([hand()], size, 1.0)[1] is None


def test_constructor_status_and_stop_do_not_open_camera(rig):
    assert rig.camera.status()["tracking_state"] == "stopped"
    assert rig.camera.frame() is None and rig.camera.sample() is None
    rig.camera.stop()
    assert not rig.calls and not rig.tracker_calls


@pytest.mark.parametrize(
    "settings",
    [
        {"device": True},
        {"device": -1},
        {"device": 64},
        {"width": 320},
        {"height": True},
    ],
)
def test_invalid_start_settings_leave_camera_closed(rig, settings):
    with pytest.raises(CalibrationError):
        rig.camera.start(**settings)
    assert not rig.calls and not rig.tracker_calls


def test_worker_mirrors_once_and_emits_private_in_memory_preview(rig):
    rig.camera.start()
    wait_for(lambda: rig.camera.sample() is not None)
    with rig.tracker.lock:
        np.testing.assert_array_equal(rig.tracker.frames[0], rig.capture.image[:, ::-1])
    image = cv2.imdecode(
        np.frombuffer(rig.camera.frame(), dtype=np.uint8), cv2.IMREAD_COLOR
    )
    np.testing.assert_allclose(image[450, 100], [0, 0, 255], atol=5)
    np.testing.assert_allclose(image[450, 540], [255, 0, 0], atol=5)
    status = rig.camera.status()
    assert status["tracking_state"] == "left_hand" and status["model_verified"]
    assert status["mirrored"] and status["simulation_only"]
    assert status["resolution"] == [640, 480]
    assert status["confidence"] == 0.95
    sample = rig.camera.sample()
    sample["wrist"][0] = 0
    assert rig.camera.sample()["wrist"][0] == 0.4
    assert not [p for p in rig.root.rglob("*") if p.suffix in {".jpg", ".png", ".mp4"}]
    with pytest.raises(CalibrationError):
        rig.camera.start()
    rig.camera.stop()
    assert rig.capture.released == [rig.calls[0][1]]
    assert rig.tracker.closed == [rig.tracker_calls[0][1]]
    assert rig.calls[0][1] != threading.get_ident()


def test_tracking_clears_on_rejection_disconnect_and_recovers(rig):
    rig.camera.start()
    wait_for(lambda: rig.camera.sample() is not None)
    for hands, state in [
        ([hand("Right")], "right_hand"),
        ([], "no_hand"),
        ([hand(), hand()], "ambiguous"),
        ([hand(confidence=0.3)], "low_confidence"),
    ]:
        with rig.tracker.lock:
            rig.tracker.hands = hands
        wait_for(lambda state=state: rig.camera.status()["tracking_state"] == state)
        assert rig.camera.sample() is None
        assert rig.camera.frame() is not None
    with rig.capture.lock:
        saved = rig.capture.image
        rig.capture.image = None
    wait_for(lambda: rig.camera.status()["tracking_state"] == "unavailable")
    assert rig.camera.sample() is None and rig.camera.frame() is None
    with rig.tracker.lock:
        rig.tracker.hands = [hand()]
    with rig.capture.lock:
        rig.capture.image = saved
    wait_for(lambda: rig.camera.sample() is not None)


def test_stale_capture_has_stricter_control_than_preview_cutoff(rig):
    rig.camera.start()
    wait_for(lambda: rig.camera.sample() is not None)
    rig.capture.block = True
    assert rig.capture.blocked.wait(2)
    with rig.camera.lock:
        rig.camera.stamp = time.monotonic() - 0.4
    assert rig.camera.sample() is None
    assert rig.camera.frame() is not None
    assert rig.camera.status()["tracking_state"] == "unavailable"
    with rig.camera.lock:
        rig.camera.stamp = time.monotonic() - 1.1
    assert rig.camera.frame() is None
    assert rig.camera.jpeg is None


def test_stop_clears_immediately_and_holds_ownership_until_worker_exits(rig):
    ownership = rig.camera.ownership
    rig.camera.start()
    wait_for(lambda: rig.camera.sample() is not None)
    rig.capture.block = True
    assert rig.capture.blocked.wait(2)
    stopping = threading.Thread(target=rig.camera.stop)
    stopping.start()
    wait_for(rig.camera.stopped.is_set)
    assert rig.camera.sample() is None and rig.camera.frame() is None
    with pytest.raises(CalibrationError):
        rig.camera.start()
    with pytest.raises(ValueError):
        ownership.claim(object())
    rig.capture.unblock.set()
    stopping.join(timeout=3)
    assert not stopping.is_alive()
    next_owner = object()
    ownership.claim(next_owner)
    ownership.release(next_owner)
    assert len(rig.capture.released) == 1 and len(rig.tracker.closed) == 1


def test_shared_ownership_refuses_second_camera_before_open(rig):
    owner = object()
    rig.camera.ownership.claim(owner)
    with pytest.raises(ValueError):
        rig.camera.start()
    assert not rig.calls and not rig.tracker_calls
    rig.camera.ownership.release(owner)
    rig.camera.start()
    wait_for(lambda: rig.camera.sample() is not None)


def test_model_missing_or_modified_fails_before_camera_open(rig):
    rig.model.unlink()
    rig.camera.start()
    wait_for(lambda: not rig.camera.status()["running"])
    assert "missing" in rig.camera.status()["error"]
    assert not rig.calls and not rig.tracker_calls
    rig.model.write_bytes(b"modified")
    rig.camera.start()
    wait_for(lambda: not rig.camera.status()["running"])
    assert "checksum" in rig.camera.status()["error"]
    assert not rig.calls and not rig.tracker_calls


def test_inference_failure_clears_input_releases_worker_resources(rig):
    rig.camera.start()
    wait_for(lambda: rig.camera.sample() is not None)
    with rig.tracker.lock:
        rig.tracker.error = RuntimeError("inference unavailable")
    wait_for(lambda: not rig.camera.status()["running"])
    assert rig.camera.status()["tracking_state"] == "unavailable"
    assert rig.camera.sample() is None and rig.camera.frame() is None
    assert len(rig.capture.released) == 1 and len(rig.tracker.closed) == 1


def test_real_cached_model_blank_inference_has_no_hand():
    pytest.importorskip("mediapipe")
    model = (
        Path(__file__).resolve().parents[2] / ".cache/perception/hand_landmarker.task"
    )
    if not model.is_file():
        pytest.skip("Local pinned perception model is not prepared")
    assert (
        hashlib.sha256(model.read_bytes()).hexdigest() == hand_camera.HAND_MODEL_SHA256
    )
    tracker = HandTracker(model)
    try:
        assert tracker.detect(np.zeros((480, 640, 3), np.uint8), 1) == []
        assert tracker.detect(np.zeros((480, 640, 3), np.uint8), 1) == []
    finally:
        tracker.close()
