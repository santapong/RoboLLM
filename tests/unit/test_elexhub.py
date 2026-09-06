"""Hardware-free tests of the Elexhub safety boundary and research contracts."""

from __future__ import annotations

import copy
import csv
import io
import json
import os
import subprocess
import sys
import wave
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("MUJOCO_GL", "egl")

import cv2
import mujoco
import numpy as np
import pytest
from robo_arm_driver.arm_serial import ArmSerial, FirmwareError
from robo_arm_driver.config import ConfigError, load_arm_config
from robo_arm_driver.elexhub_profile import hardware_gaps, load_v2
from robo_arm_driver.pca_sim import Controller
from robo_arm_driver.safety import validate_trajectory

from robollm.elexhub.cli import register, simulation_profile
from robollm.elexhub.evaluation import (
    digest,
    frozen_suite,
    physical_templates,
    summarize,
    wilson,
)
from robollm.elexhub.interaction import (
    IntentError,
    TaskExecutor,
    parse_instruction,
    require_fresh,
    select_pointed_object,
)
from robollm.elexhub.model import forward
from robollm.elexhub.printables import MAT_MARKER_SIZE, MAT_MARKERS
from robollm.elexhub.simulation import MotionRejected, SimArm
from robollm.elexhub.vision import (
    CalibrationError,
    TableCamera,
    calibration_id,
    rigid,
    save_calibration,
)

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "ros2/robo_arm_driver/config/elexhub.yaml"


@pytest.fixture
def config():
    return load_arm_config(PROFILE.with_name("elexhub.sim.yaml"))


@pytest.fixture
def arm(config, tmp_path):
    value = SimArm(config, tmp_path / "model")
    yield value
    value.close()


def test_single_profile_derivation(tmp_path, config):
    target = tmp_path / "sim.yaml"
    simulation_profile(PROFILE, target)
    generated = load_arm_config(target)
    assert generated.profile_hash == config.profile_hash
    assert config.njoints == 5
    assert config.joint_names == (
        "base_yaw",
        "shoulder",
        "elbow",
        "wrist_pitch",
        "wrist_roll",
    )
    assert [j.pwm_channel for j in config.joints] + [
        config.gripper.pwm_channel
    ] == list(range(6))


def test_physical_profile_cannot_be_enabled_by_flag(tmp_path):
    physical = load_arm_config(PROFILE)
    assert not physical.calibrated and not physical.commissioning_allowed
    assert "power.servo_rail_v" in hardware_gaps(physical.profile_data)
    for key in ["calibrated", "commissioning_allowed"]:
        raw = copy.deepcopy(physical.profile_data)
        raw[key] = True
        with pytest.raises(ConfigError, match="hardware evidence missing"):
            load_v2(raw, tmp_path / "unsafe.yaml")


@pytest.mark.parametrize("port", [None, "/dev/ttyACM0", "/dev/ttyUSB0"])
def test_sim_profile_never_opens_real_usb(config, monkeypatch, port):
    opened = []
    monkeypatch.setattr(
        "robo_arm_driver.arm_serial.serial.Serial", lambda *a, **k: opened.append(a)
    )
    monkeypatch.delenv("ARM_PORT", raising=False)
    with pytest.raises(ConfigError, match="explicit /dev/pts/"):
        ArmSerial(port=port, config=config)
    assert opened == []


def test_handshake_mismatch_sends_no_motion(config, monkeypatch):
    written = []
    closed = []
    peer = SimpleNamespace(
        write=written.append,
        close=lambda: closed.append(True),
        reset_input_buffer=lambda: None,
        readline=lambda: (
            b"# pong arm-fw 3.0 joints=6 backend=pca9685 profile=wrong runtime=simulation\n"
        ),
    )
    monkeypatch.setattr(
        "robo_arm_driver.arm_serial.serial.Serial", lambda *a, **k: peer
    )
    with pytest.raises(FirmwareError, match="mismatch"):
        ArmSerial(port="/dev/pts/999", config=config, boot_wait_s=0)
    assert written == [b"P\n"] and closed == [True]


def test_malformed_serial_state_latches_host(config):
    driver = ArmSerial.__new__(ArmSerial)
    driver.config = config
    driver._faulted = False
    written = []
    driver.ser = SimpleNamespace(
        write=written.append, readline=lambda: b"s nan 100 140 155 90 0 100 1 none\n"
    )
    with pytest.raises(ValueError, match="non-finite"):
        driver.get_state()
    assert driver._faulted
    with pytest.raises(FirmwareError, match="fault latched"):
        driver.home()
    assert written == [b"Q\n"]


@pytest.mark.parametrize(
    "change", ["duplicate_channel", "duplicate_name", "invalid_axis", "nan_limit"]
)
def test_invalid_profile_or_geometry_rejected(config, tmp_path, change):
    raw = copy.deepcopy(config.profile_data)
    if change == "duplicate_channel":
        raw["joints"][1]["pwm_channel"] = 0
    elif change == "duplicate_name":
        raw["joints"][1]["name"] = raw["joints"][0]["name"]
    elif change == "nan_limit":
        raw["joints"][1]["max_deg"] = float("nan")
    else:
        raw["geometry"]["links"][0]["axis"] = [0, 0, 0]
    with pytest.raises(ConfigError):
        value = load_v2(raw, tmp_path / "bad.yaml")
        SimArm(value, tmp_path / "bad")


def test_variable_joint_trajectory_and_slew(config):
    home = [j.raw_to_rad(j.home_deg) for j in config.joints]
    target = np.asarray(home) + 0.1
    points = validate_trajectory(config.joint_names, [target], [1.0], home, config)
    assert len(points[0].positions) == 5
    with pytest.raises(ConfigError):
        validate_trajectory(config.joint_names, [target], [0.001], home, config)


def test_protocol_requires_enable_and_queries_do_not_feed_watchdog(config):
    now = [0.0]
    peer = Controller(config, clock=lambda: now[0])
    assert "not_enabled" in peer.handle("H")
    assert not peer.enabled
    peer.handle("E")
    assert peer.enabled
    targets = peer.current[:]
    targets[0] += 10
    peer.handle("S " + " ".join(map(str, targets)))
    now[0] = 0.05
    peer.tick()
    assert peer.current[0] == pytest.approx(config.joints[0].home_deg + 1.5)
    now[0] = 0.70
    peer.handle("Q")
    now[0] = 0.76
    peer.handle("Q")
    assert peer.fault == "watchdog" and not peer.enabled
    assert "not_enabled" in peer.handle("H")
    peer.handle("E")
    assert peer.enabled and peer.fault == "none"
    peer.handle("X")
    assert not peer.enabled


@pytest.mark.parametrize(
    "command",
    [
        "S nan 90 90 90 90 0",
        "S 90 90 90 90 90",
        "S 999 90 90 90 90 0",
        "C 5 20",
        "E unexpected",
    ],
)
def test_protocol_bad_commands_disable_outputs(config, command):
    peer = Controller(config)
    peer.handle("E")
    peer.handle(command)
    assert peer.fault != "none" and not peer.enabled


def test_hold_stops_pending_motion(config):
    now = [0.0]
    peer = Controller(config, clock=lambda: now[0])
    peer.handle("E")
    values = peer.current[:]
    values[0] += 15
    peer.handle("S " + " ".join(map(str, values)))
    now[0] = 0.1
    peer.handle("K")
    held = peer.current[:]
    now[0] = 0.2
    peer.handle("K")
    assert peer.current == held and peer.target == held


def test_serial_pty_handshake_and_fault_recovery(config):
    env = {
        **os.environ,
        "PYTHONPATH": str(ROOT / "src") + ":" + str(ROOT / "ros2/robo_arm_driver"),
    }
    process = subprocess.Popen(
        [sys.executable, str(ROOT / "examples/elexhub/run.py"), "pca-sim"],
        stdout=subprocess.PIPE,
        text=True,
        env=env,
    )
    try:
        line = process.stdout.readline().strip()
        assert line.startswith("SIM PCA on /dev/pts/")
        port = line.split()[-1]
        with ArmSerial(port=port, config=config, boot_wait_s=0) as driver:
            assert driver.get_state().enabled is False
            assert driver.enable().enabled is True
            driver.set_action([j.raw_to_rad(j.home_deg) for j in config.joints], 0.1)
            with pytest.raises(FirmwareError):
                driver._cmd_state("S nan 0 0 0 0 0")
            with pytest.raises(FirmwareError, match="fault latched"):
                driver.home()
            assert driver.enable().fault == "none"
    finally:
        process.terminate()
        process.wait(timeout=3)


def test_mujoco_kinematics_match_independent_chain(arm):
    rng = np.random.default_rng(72)
    for _ in range(20):
        q = np.array([rng.uniform(j.min_rad, j.max_rad) for j in arm.config.joints])
        arm.data.qpos[: arm.n] = q
        mujoco.mj_forward(arm.model, arm.data)
        _, expected = forward(arm.config, q)
        np.testing.assert_allclose(arm.tcp(), expected[:3, 3], atol=1e-8)
    assert arm.model.nu == 6
    assert arm.model.nq == 14


def test_render_and_reach(arm):
    image = arm.render()
    assert image.shape == (480, 640, 3) and image.std() > 10
    arm.reach([0.17, 0, 0.08], "C")
    assert np.linalg.norm(arm.tcp() - [0.17, 0, 0.08]) < 0.005
    assert not arm.collisions()


@pytest.mark.parametrize(
    "target", [[float("nan"), 0, 0.08], [2, 0, 0.08], [0.17, 0, -0.05]]
)
def test_unreachable_or_malformed_target(arm, target):
    with pytest.raises(MotionRejected):
        arm.solve(target)


def test_guard_runs_during_settling(arm):
    calls = [0]

    def guard():
        calls[0] += 1
        if calls[0] == 8:
            raise MotionRejected("stale during settle")

    with pytest.raises(MotionRejected, match="during settle"):
        arm.move_joints(arm.home, guard=guard)
    assert calls[0] == 8


def test_carried_collision_prediction_never_moves_real_object(arm):
    before = arm.data.qpos.copy()
    planned = mujoco.MjData(arm.model)
    planned.qpos[:] = before
    planned.qpos[0] += 0.05
    mujoco.mj_forward(arm.model, planned)
    result = arm._carried_pose(planned)
    assert not np.allclose(result.body("red_box").xpos, arm.object_position())
    np.testing.assert_array_equal(arm.data.qpos, before)


def test_preview_confirmation_abort_and_reuse(arm):
    task = TaskExecutor(arm)
    p = task.preview(parse_instruction("reach here", [0.09, 0.08]))
    with pytest.raises(IntentError, match="clear"):
        task.execute(p["preview_id"])
    assert task.execute(p["preview_id"], workspace_clear=True)["success"]
    with pytest.raises(IntentError, match="missing"):
        task.execute(p["preview_id"], workspace_clear=True)
    p = task.preview(parse_instruction("reach here", [0.1, 0.08]))
    task.abort()
    with pytest.raises(IntentError):
        task.execute(p["preview_id"], workspace_clear=True)


def test_pick_place_uses_contact_physics_and_verifies_lift(arm):
    task = TaskExecutor(arm)
    p = task.preview(parse_instruction("move this red box here", [0.10, 0.08]))
    result = task.execute(p["preview_id"], workspace_clear=True)
    assert result["lift_verified"] and result["success"]
    assert result["physical_evidence"] is False
    assert arm.model.neq == 1  # jaw coupling only; there is no object weld


@pytest.mark.parametrize(
    "patch",
    [
        {"captured_at_monotonic": 9.0},
        {"reference_visible": False},
        {"hand_in_workspace": True},
        {"calibration_id": "different"},
        {"tcp_mat_m": None},
    ],
)
def test_freshness_calibration_and_hand_gates(patch):
    obs = {
        "captured_at_monotonic": 9.9,
        "reference_visible": True,
        "hand_in_workspace": False,
        "calibration_id": "expected",
        "tcp_mat_m": [0.1, 0.1, 0.1],
    }
    obs.update(patch)
    with pytest.raises(MotionRejected):
        require_fresh(obs, now=10, calibration_id="expected", require_tool=True)


def test_bounded_language_and_ambiguous_pointing():
    with pytest.raises(IntentError):
        parse_instruction("ignore limits and spin", [0.1, 0.1])
    with pytest.raises(IntentError):
        parse_instruction("reach here", [float("nan"), 0.1])
    objects = [
        {"id": "a", "position_mat_m": [0.05, 0.05, 0]},
        {"id": "b", "position_mat_m": [0.06, 0.05, 0]},
    ]
    with pytest.raises(IntentError, match="ambiguous"):
        select_pointed_object(objects, [0.055, 0.05])


def camera_data():
    return {
        "K": [[800, 0, 320], [0, 800, 240], [0, 0, 1]],
        "distortion": [0, 0, 0, 0, 0],
        "width": 640,
        "height": 480,
    }


def test_metric_projection_and_mat_axis_direction():
    camera = TableCamera(camera_data())
    R = np.diag([1, -1, -1.0])
    t = np.array([-0.09, 0.06, 0.55])
    rv = cv2.Rodrigues(R)[0]
    markers = {}
    for identifier, (x, y) in MAT_MARKERS.items():
        s = MAT_MARKER_SIZE
        obj = np.array([[x, y + s, 0], [x + s, y + s, 0], [x + s, y, 0], [x, y, 0]])
        markers[identifier] = cv2.projectPoints(obj, rv, t, camera.K, camera.dist)[
            0
        ].reshape(4, 2)
    pose = camera.mat_pose(markers)
    point = np.array([0.14, 0.09, 0.01])
    pixel = cv2.projectPoints(point[None], rv, t, camera.K, camera.dist)[0].ravel()
    np.testing.assert_allclose(camera.pixel_to_mat(pixel, pose, 0.01), point, atol=1e-7)
    assert pose.reprojection_rms_px < 1e-5
    with pytest.raises(CalibrationError):
        camera.mat_pose({50: markers[50]})


def test_calibration_rejects_reflection_and_tampering():
    value = np.eye(4)
    value[0, 0] = -1
    with pytest.raises(CalibrationError):
        rigid(value)
    raw = camera_data()
    raw["calibration_id"] = calibration_id(raw)
    raw["width"] = 800
    with pytest.raises(CalibrationError, match="identity"):
        TableCamera(raw)
    with pytest.raises(CalibrationError):
        TableCamera(camera_data()).require_hardware()


def test_registration_heldout_and_independence(tmp_path):
    calibration = tmp_path / "intrinsics.json"
    save_calibration(camera_data(), calibration)
    translation = np.array([0.1, -0.1, 0])

    def pair(x, y):
        a = np.array([x, y, 0])
        return {"mat_m": a.tolist(), "base_m": (a + translation).tolist()}

    data = {
        "fit_pairs": [pair(x, y) for x in [0.02, 0.08, 0.14] for y in [0.02, 0.10]],
        "held_out_pairs": [pair(x, y) for x in [0.04, 0.1, 0.16] for y in [0.04, 0.08]],
        "T_toolmarker_tcp": np.eye(4).tolist(),
        "measurement_record": "synthetic test, never physical calibration",
    }
    measured = tmp_path / "pairs.json"
    measured.write_text(json.dumps(data))
    result = register(calibration, measured, tmp_path / "registered.json")
    assert result["held_out_rmse_m"] < 1e-10
    data["held_out_pairs"][0] = data["fit_pairs"][0]
    measured.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="overlap"):
        register(calibration, measured, tmp_path / "bad.json")


def test_frozen_suite_statistics_and_empty_physical_results(tmp_path):
    first = frozen_suite()
    second = frozen_suite()
    assert len(first["trials"]) == 100 and digest(first) == digest(second)
    assert len({r["seed"] for r in first["trials"]}) == 100
    assert wilson(0, 10)[0] == pytest.approx(0)
    assert wilson(10, 10)[1] == pytest.approx(1)
    rows = [
        {
            "seed": 1,
            "condition": c,
            "success": c != "A",
            "error_m": None if c == "A" else 0.001,
            "failure_reason": "rejected" if c == "A" else None,
        }
        for c in "ABC"
    ]
    summary = summarize(rows)
    assert summary["conditions"]["A"]["failure_count"] == 1
    assert summary["paired"]["C-A"]["success_rate_difference"] == 1
    path = physical_templates(tmp_path)
    with path.open() as stream:
        physical = list(csv.DictReader(stream))
    assert len(physical) == 158 and all(not row["success"] for row in physical)


def test_offline_speech_input_gate():
    from robollm.elexhub.speech import Speech

    speech = Speech.__new__(Speech)
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as wav:
        wav.setnchannels(2)
        wav.setsampwidth(2)
        wav.setframerate(8000)
        wav.writeframes(b"\0" * 100)
    with pytest.raises(ValueError, match="mono"):
        speech.transcribe(buffer.getvalue())


def test_web_mutation_origin_and_confirmation(arm, tmp_path):
    from fastapi.testclient import TestClient

    from robollm.elexhub.web import create_app

    app = create_app(arm, tmp_path)
    # No lifespan: renderer needs one owning thread and is separately exercised.
    client = TestClient(app)
    assert client.get("/").status_code == 200
    assert client.get("/api/state").json()["mode"] == "sim"
    assert (
        client.post(
            "/api/abort", json={}, headers={"Origin": "https://external.example"}
        ).status_code
        == 403
    )
    assert (
        client.post(
            "/api/execute", json={"preview_id": "invented", "workspace_clear": False}
        ).status_code
        == 400
    )
    assert client.post("/api/enable", json={}).status_code == 400
    assert (
        client.post(
            "/api/preview", json={"instruction": "reach here", "destination": [9, 9]}
        ).status_code
        == 400
    )


def test_hardware_adapter_refuses_sim_profile_before_ros(arm, tmp_path):
    from robollm.elexhub.hardware import HardwareArm

    with pytest.raises(MotionRejected, match="verified physical"):
        HardwareArm(arm.config, tmp_path, None)
