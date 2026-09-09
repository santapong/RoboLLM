"""Freshness and owned-process lifecycle checks; never open a camera."""

import importlib.util
import json
import math
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

SOURCE = Path(__file__).resolve().parents[2] / "examples/humanoid_harness/camera_bridge.py"
spec = importlib.util.spec_from_file_location("humanoid_camera_bridge", SOURCE)
bridge_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge_module)
CameraBridge = bridge_module.CameraBridge


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Camera pipe condition did not arrive")


class FakeProcess:
    def __init__(self, *, resistant=False):
        self.lines = queue.Queue()
        self.exited = threading.Event()
        self.resistant = resistant
        self.terminated = self.killed = False
        self.stdout_closed = False
        self.stdout = SimpleNamespace(readline=self.lines.get, close=self.close_stdout)
        self.stdin = SimpleNamespace(close=self.close_stdin)

    def close_stdout(self):
        self.stdout_closed = True
        self.lines.put(b"")

    def close_stdin(self):
        if not self.resistant:
            self.finish()

    def finish(self):
        self.exited.set()
        self.lines.put(b"")

    def poll(self):
        return 0 if self.exited.is_set() else None

    def wait(self, timeout):
        if not self.exited.wait(min(timeout, 0.02)):
            raise subprocess.TimeoutExpired("fake-camera", timeout)
        return 0

    def terminate(self):
        self.terminated = True
        if not self.resistant:
            self.finish()

    def kill(self):
        self.killed = True
        self.finish()


@pytest.fixture
def controlled(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(bridge_module, "time", SimpleNamespace(monotonic=lambda: now[0]))
    bridge = CameraBridge(sys.executable, "/not/a/camera")
    process = object()
    bridge.process = process
    return bridge, process, now


def good_packet(stamp=100.0):
    return {"running": True, "tracked": True, "target": [0.2, 0, 0.4],
            "timestamp": stamp, "confidence": 1.0}


def test_repeated_snapshot_does_not_refresh_capture_age(controlled):
    bridge, process, now = controlled
    bridge._accept(process, good_packet(), b"preview")
    now[0] = 100.4
    bridge._accept(process, good_packet(), b"preview")
    packet, _, age = bridge.snapshot()
    assert packet["tracked"] and age == pytest.approx(0.4)
    now[0] = 100.6
    bridge._accept(process, good_packet(), b"preview")
    packet, _, age = bridge.snapshot()
    assert not packet["tracked"]
    assert packet["confidence"] == 0
    assert age == pytest.approx(0.6)
    assert "stale" in packet["message"]


@pytest.mark.parametrize("stamp", [None, "100", True, -1, float("nan"), float("inf"), 101.0, 10 ** 400])
def test_invalid_or_future_capture_times_fail_closed(controlled, stamp):
    bridge, process, now = controlled
    bridge._accept(process, good_packet(stamp), None)
    now[0] = 102.0  # A future timestamp must not become valid as time catches up.
    packet, _, age = bridge.snapshot()
    assert not packet["tracked"]
    assert age > 0.5 and math.isfinite(age)
    assert "Invalid" in packet["message"]
    json.dumps({"age": age}, allow_nan=False)


def test_missing_timestamp_and_idle_startup_hold_with_finite_age(controlled):
    bridge, process, _ = controlled
    bridge.packet = {"running": True, "tracked": False, "message": "Starting tracker"}
    packet, _, age = bridge.snapshot()
    assert packet["running"] and packet["message"] == "Starting tracker"
    assert not packet["tracked"] and math.isfinite(age)
    incoming = good_packet()
    del incoming["timestamp"]
    bridge._accept(process, incoming, None)
    assert not bridge.snapshot()[0]["tracked"]


def test_old_reader_and_old_eof_cannot_overwrite_new_process(controlled):
    bridge, current, _ = controlled
    old = FakeProcess()
    bridge._accept(current, good_packet(), b"current preview")
    old.lines.put(json.dumps(good_packet()).encode())
    old.lines.put(b"")
    bridge._read(old)
    assert bridge.snapshot()[0]["tracked"]
    assert bridge.snapshot()[1] == b"current preview"


def test_start_is_idempotent_and_stop_restart_discards_frames(monkeypatch):
    created = []

    def launch(*_, **__):
        created.append(FakeProcess())
        return created[-1]

    monkeypatch.setattr(bridge_module.subprocess, "Popen", launch)
    bridge = CameraBridge(sys.executable, "/not/a/camera")
    try:
        bridge.start()
        bridge.start()
        assert len(created) == 1
        assert bridge.snapshot()[0]["running"]
        created[0].lines.put(json.dumps(good_packet(time.monotonic())).encode())
        wait_for(lambda: bridge.snapshot()[0]["tracked"])
        bridge.stop()
        assert not bridge.snapshot()[0]["running"]
        assert bridge.snapshot()[1] is None
        assert created[0].exited.is_set() and created[0].stdout_closed
        bridge.start()
        assert len(created) == 2
        assert bridge.snapshot()[0]["running"]
        assert not bridge.snapshot()[0]["tracked"]
        assert bridge.snapshot()[1] is None
    finally:
        bridge.stop()


def test_eof_clears_preview_and_allows_replacing_alive_broken_worker(monkeypatch):
    created = []

    def launch(*_, **__):
        created.append(FakeProcess())
        return created[-1]

    monkeypatch.setattr(bridge_module.subprocess, "Popen", launch)
    bridge = CameraBridge(sys.executable, "/not/a/camera")
    try:
        bridge.start()
        old = created[0]
        old.lines.put(b"[]\n")  # Non-dictionary packets are ignored safely.
        old.lines.put(b"not json\n")
        old.lines.put(b"")  # stdout EOF before the process itself exits.
        wait_for(lambda: not bridge.snapshot()[0]["running"])
        assert old.poll() is None
        bridge.start()
        assert len(created) == 2 and old.exited.is_set()
        assert bridge.snapshot()[0]["running"]
        assert not bridge.snapshot()[0]["tracked"]
    finally:
        bridge.stop()


def test_stop_escalates_only_its_owned_stuck_process(monkeypatch):
    process = FakeProcess(resistant=True)
    monkeypatch.setattr(bridge_module.subprocess, "Popen", lambda *_, **__: process)
    bridge = CameraBridge(sys.executable, "/not/a/camera")
    bridge.start()
    bridge.stop()
    assert process.terminated and process.killed and process.exited.is_set()
    assert bridge.process is None
    assert not bridge.snapshot()[0]["tracked"]


def test_failed_process_launch_leaves_off_state(monkeypatch):
    def fail(*_, **__):
        raise OSError("Cannot spawn camera")

    monkeypatch.setattr(bridge_module.subprocess, "Popen", fail)
    bridge = CameraBridge(sys.executable, "/not/a/camera")
    with pytest.raises(OSError, match="Cannot spawn"):
        bridge.start()
    assert not bridge.snapshot()[0]["running"]
    assert bridge.process is None


def test_closed_stdin_does_not_prevent_child_reaping(monkeypatch):
    process = FakeProcess()

    def broken_stdin():
        raise BrokenPipeError("Child already closed input")

    process.stdin.close = broken_stdin
    monkeypatch.setattr(bridge_module.subprocess, "Popen", lambda *_, **__: process)
    bridge = CameraBridge(sys.executable, "/not/a/camera")
    bridge.start()
    bridge.stop()
    assert process.terminated and process.exited.is_set()
    assert not bridge.snapshot()[0]["running"]


@pytest.mark.parametrize("running,message,preserve", [
    (False, "Camera /dev/video0 could not open (missing or busy)", True),
    (False, "  ", False),
    (True, "Palm target visible", False),
])
def test_eof_preserves_explicit_worker_error_but_clears_live_data(running, message, preserve):
    bridge = CameraBridge(sys.executable, "/not/a/camera")
    process = FakeProcess()
    bridge.process = process
    packet = {**good_packet(time.monotonic()), "running": running, "message": message,
              "backend": "mediapipe_hand", "jpeg_b64": "cHJldmlldw=="}
    process.lines.put(json.dumps(packet).encode())
    process.lines.put(b"")
    bridge._read(process)
    final, image, age = bridge.snapshot()
    assert not final["running"] and not final["tracked"]
    assert final["confidence"] == 0 and image is None
    assert final["backend"] == "mediapipe_hand"
    assert final["message"] == (message if preserve else
                                "Camera worker stopped; check device or camera environment")
    assert age > 0.5
