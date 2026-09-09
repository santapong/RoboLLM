"""Left-hand simulation control using synthetic samples; no camera or hardware."""

from __future__ import annotations

import hashlib
import json
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest
from robo_arm_driver.config import ConfigError, load_arm_config

from robollm.elexhub import hand_control, simulation
from robollm.elexhub.hand_control import (
    CONTROL_INTERVAL_S,
    MAX_TARGET_SPEED_M_S,
    HandMirrorController,
    accepted_sample,
    relative_target,
)
from robollm.elexhub.simulation import MotionRejected, SimArm

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "assets/elexhub/provisional"


@pytest.fixture
def config():
    return load_arm_config(ROOT / "ros2/robo_arm_driver/config/elexhub.sim.yaml")


@pytest.fixture
def arm(config):
    value = SimArm(config, ASSETS, rebuild=False)
    yield value
    value.close()


def sample(stamp=100.0, wrist=(0.5, 0.5), scale=0.2, pinch=1.0):
    return {
        "timestamp": stamp,
        "wrist": list(wrist),
        "palm_scale": scale,
        "pinch": pinch,
        "handedness": "Left",
        "confidence": 0.9,
    }


class ManualWorker:
    """Drive real controller ticks explicitly with a deterministic clock."""

    def __init__(self, *, target, **kwargs):
        self.owner = target.__self__
        self.started = False

    def start(self):
        self.started = True

    def is_alive(self):
        return self.started and not self.owner.stopped.is_set()

    def join(self, **kwargs):
        pass


@pytest.fixture
def rig(arm, monkeypatch):
    monkeypatch.setattr(hand_control.threading, "Thread", ManualWorker)
    state = SimpleNamespace(now=100.0, sample=sample(), calls=[])

    def step(target, closure, dt, *, guard):
        guard()
        state.calls.append((np.asarray(target).copy(), closure, dt))
        return arm.tcp()

    monkeypatch.setattr(arm, "teleop_step", step)
    controller = HandMirrorController(
        arm, lambda: state.sample, clock=lambda: state.now
    )
    state.controller, state.arm = controller, arm
    yield state
    controller.close()


def test_existing_model_load_never_regenerates_and_stale_profile_is_rejected(
    config, monkeypatch, tmp_path
):
    def no_build(*args, **kwargs):
        raise AssertionError("read-only startup must not regenerate assets")

    monkeypatch.setattr(simulation, "build_assets", no_build)
    names = ["cad-description.json", "elexhub.xml", "elexhub.glb", "model.html"]
    before = {
        name: hashlib.sha256((ASSETS / name).read_bytes()).hexdigest() for name in names
    }
    loaded = SimArm(config, ASSETS, rebuild=False)
    loaded.close()
    assert before == {
        name: hashlib.sha256((ASSETS / name).read_bytes()).hexdigest() for name in names
    }
    with pytest.raises(ConfigError, match="missing or invalid"):
        SimArm(config, tmp_path, rebuild=False)
    descriptor = json.loads((ASSETS / "cad-description.json").read_text())
    descriptor["profile_hash"] = "stale"
    (tmp_path / "cad-description.json").write_text(json.dumps(descriptor))
    with pytest.raises(ConfigError, match="stale"):
        SimArm(config, tmp_path, rebuild=False)


@pytest.mark.parametrize(
    "change",
    [
        {"handedness": "Right"},
        {"confidence": 0.64},
        {"timestamp": 99.0},
        {"timestamp": 101.0},
        {"wrist": [np.nan, 0.5]},
        {"wrist": [0.2]},
        {"wrist": [1.1, 0.2]},
        {"palm_scale": 0},
        {"pinch": 1.1},
    ],
)
def test_control_boundary_rejects_nonleft_stale_and_malformed_samples(change):
    assert accepted_sample({**sample(), **change}, 100.0) is None
    assert accepted_sample(sample(), 100.0) is not None


def test_relative_mapping_matches_mirrored_axes_and_preserves_neutral(arm):
    neutral = accepted_sample(sample(), 100.0)
    origin = arm.tcp()
    np.testing.assert_allclose(
        relative_target(neutral, neutral, origin, arm.geometry), origin, atol=1e-12
    )
    moved = accepted_sample(sample(wrist=(0.7, 0.3), scale=0.24), 100.0)
    target = relative_target(moved, neutral, origin, arm.geometry)
    assert np.all(target > origin)  # closer -> +X, mirrored right -> +Y, up -> +Z
    extreme = accepted_sample(sample(wrist=(1, 0), scale=2.0), 100.0)
    limited = relative_target(extreme, neutral, origin, arm.geometry)
    assert np.linalg.norm(limited - origin) < 0.11


def test_constructor_is_idle_and_rejects_hardware_objects(arm):
    calls = []
    control = HandMirrorController(arm, lambda: calls.append(True))
    assert control.thread is None and not control.status()["running"]
    assert calls == []
    with pytest.raises(MotionRejected, match="local SimArm"):
        HandMirrorController(SimpleNamespace(config=arm.config), lambda: None)
    physical = load_arm_config(ROOT / "ros2/robo_arm_driver/config/elexhub.yaml")
    previous = arm.config
    arm.config = physical
    try:
        with pytest.raises(MotionRejected, match="local SimArm"):
            HandMirrorController(arm, lambda: None)
    finally:
        arm.config = previous


def test_loss_reacquisition_and_recenter_do_not_replay_old_targets(rig):
    control = rig.controller
    control.start()
    control._tick()
    assert control.status()["state"] == "tracking" and not rig.calls
    rig.now += CONTROL_INTERVAL_S
    rig.sample = sample(rig.now, wrist=(0.9, 0.1), pinch=0.0)
    control._tick()
    target, closure, dt = rig.calls[-1]
    assert closure == 1.0 and dt == CONTROL_INTERVAL_S
    assert np.linalg.norm(target - rig.arm.tcp()) <= MAX_TARGET_SPEED_M_S * dt + 1e-12
    assert target[1] > rig.arm.tcp()[1] and target[2] > rig.arm.tcp()[2]
    rig.arm.queue.append(np.full(rig.arm.n, 0.8))
    rig.sample = None
    rig.now += CONTROL_INTERVAL_S
    control._tick()
    assert control.status()["state"] == "holding"
    for queued in rig.arm.queue:
        np.testing.assert_array_equal(queued, rig.arm.last_command)
    calls = len(rig.calls)
    rig.sample = sample(rig.now, wrist=(0.1, 0.9))
    control._tick()
    assert len(rig.calls) == calls
    np.testing.assert_allclose(control.status()["target_m"], rig.arm.tcp())
    control.recenter()
    assert control.status()["state"] == "waiting_for_left"
    control._tick()
    assert len(rig.calls) == calls
    json.dumps(control.status(), allow_nan=False)


def test_heartbeat_expiry_requires_explicit_restart(rig):
    control = rig.controller
    control.start()
    control._tick()
    rig.now += 1.6
    control._tick()
    assert not control.status()["running"]
    assert "connection lost" in control.status()["error"]
    control.heartbeat()
    rig.sample = sample(rig.now)
    control._tick()
    assert not rig.calls and not control.status()["running"]
    control.start()
    control._tick()
    assert control.status()["running"] and control.status()["state"] == "tracking"


def test_rejected_target_holds_the_existing_simulation(rig, monkeypatch):
    control = rig.controller
    control.start()
    control._tick()
    before = rig.arm.data.qpos.copy()

    def rejected(*args, **kwargs):
        raise MotionRejected("collision test")

    monkeypatch.setattr(rig.arm, "teleop_step", rejected)
    rig.now += CONTROL_INTERVAL_S
    rig.sample = sample(rig.now, wrist=(0.8, 0.5))
    control._tick()
    assert control.status()["state"] == "rejected"
    assert "collision" in control.status()["error"]
    np.testing.assert_array_equal(rig.arm.data.qpos, before)


def test_incremental_physics_tracks_without_batch_motion_or_rate_violations(
    arm, monkeypatch
):
    def no_batch(*args, **kwargs):
        raise AssertionError("streaming must not call batch motion methods")

    monkeypatch.setattr(arm, "reach", no_batch)
    monkeypatch.setattr(arm, "move_joints", no_batch)
    monkeypatch.setattr(arm, "set_gripper", no_batch)
    arm.hold()
    target = arm.tcp() + [0.003, 0.008, 0.005]
    rate = np.array([j.max_velocity_rad_s for j in arm.config.joints])
    for _ in range(50):
        previous, grip = arm.last_command.copy(), arm.gripper
        arm.teleop_step(target, 0.65, CONTROL_INTERVAL_S)
        assert np.all(
            np.abs(arm.last_command - previous) <= rate * CONTROL_INTERVAL_S + 1e-10
        )
        assert (
            abs(arm.gripper - grip)
            <= arm.config.gripper.max_velocity_percent_s / 100 * CONTROL_INTERVAL_S
            + 1e-10
        )
        assert not arm.collisions()
    assert np.linalg.norm(arm.tcp() - target) < 0.005
    assert arm.gripper == pytest.approx(0.65)


def test_synthetic_hand_sweep_moves_actual_tcp_and_gripper_without_rejections(
    arm, monkeypatch
):
    monkeypatch.setattr(hand_control.threading, "Thread", ManualWorker)
    state = SimpleNamespace(now=100.0, sample=sample())
    control = HandMirrorController(arm, lambda: state.sample, clock=lambda: state.now)
    control.start()
    control._tick()
    positions, closures = [], []
    try:
        for index in range(1, 121):
            elapsed = index * CONTROL_INTERVAL_S
            state.now = 100 + elapsed
            state.sample = sample(
                state.now,
                wrist=(
                    0.5 + 0.07 * np.sin(2 * np.pi * elapsed / 6),
                    0.5 - 0.07 * np.sin(2 * np.pi * elapsed / 5),
                ),
                scale=0.2 * (1 + 0.15 * np.sin(2 * np.pi * elapsed / 7)),
                pinch=0.5 + 0.5 * np.cos(2 * np.pi * elapsed / 4),
            )
            control.heartbeat()
            control._tick()
            status = control.status()
            assert status["state"] == "tracking", status["error"]
            positions.append(status["tcp_m"])
            closures.append(status["gripper"])
        spans = np.ptp(np.asarray(positions), axis=0)
        assert np.all(spans > 0.002)
        assert max(closures) > 0.9 and min(closures) < 0.1
        print(
            {
                "target_m": status["target_m"],
                "tcp_m": status["tcp_m"],
                "error_m": float(
                    np.linalg.norm(np.asarray(status["tcp_m"]) - status["target_m"])
                ),
                "tcp_span_m": spans.tolist(),
                "closure_range": [min(closures), max(closures)],
            }
        )
    finally:
        control.close()


def test_expired_guard_and_real_object_collision_cannot_commit_a_control_step(arm):
    before, started = arm.data.qpos.copy(), arm.data.time
    calls = 0

    def expired():
        nonlocal calls
        calls += 1
        if calls >= 4:
            raise MotionRejected("lease expired")

    with pytest.raises(MotionRejected, match="lease expired"):
        arm.teleop_step(
            arm.tcp() + [0, 0.002, 0], 0.5, CONTROL_INTERVAL_S, guard=expired
        )
    np.testing.assert_array_equal(arm.data.qpos, before)
    assert arm.data.time == started
    address = arm.model.joint("object_free").qposadr[0]
    # Use a partial overlap: the checked MuJoCo box collider produces no
    # contacts for this pair when their centers coincide exactly.
    arm.data.qpos[address : address + 3] = arm.data.geom("finger_left_pad").xpos + [
        0,
        0.005,
        0,
    ]
    mujoco.mj_forward(arm.model, arm.data)
    assert arm.collisions()
    before = arm.data.qpos.copy()
    with pytest.raises(MotionRejected, match="intersects|contact|collision"):
        arm.teleop_step(arm.tcp(), 0.5, CONTROL_INTERVAL_S)
    np.testing.assert_array_equal(arm.data.qpos, before)


def test_stop_returns_while_sample_is_blocked_and_prevents_late_motion(
    arm, monkeypatch
):
    entered, release = threading.Event(), threading.Event()
    commands = []

    def blocked_sample():
        entered.set()
        release.wait(2)
        return sample(time.monotonic())

    monkeypatch.setattr(
        arm, "teleop_step", lambda *args, **kwargs: commands.append(args)
    )
    control = HandMirrorController(arm, blocked_sample)
    control.start()
    try:
        assert entered.wait(1)
        started = time.monotonic()
        control.stop()
        assert time.monotonic() - started < 0.6
        release.set()
        control.thread.join(timeout=1)
        assert not control.thread.is_alive() and not commands
        assert control.status()["state"] == "stopped"
    finally:
        release.set()
        control.close()
