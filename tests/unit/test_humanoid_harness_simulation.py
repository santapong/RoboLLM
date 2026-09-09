"""Headless dynamics checks for the explicitly downloaded, supported G1 demo."""
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from examples.humanoid_harness.fetch_model import DEFAULT_ASSETS
from examples.humanoid_harness.simulation import (
    WORKSPACE,
    HumanoidSimulation,
)

pytestmark = pytest.mark.skipif(not (DEFAULT_ASSETS / "provenance.json").exists(), reason="Run fetch_model.py to download pinned G1 assets")


@pytest.fixture
def sim():
    simulator = HumanoidSimulation(width=320, height=240)
    yield simulator
    simulator.close()


@pytest.mark.parametrize("arm,target", [("right", [0.30, -0.26, 1.10]), ("right", [0.34, -0.32, 1.22]), ("left", [0.30, 0.26, 1.10])])
def test_physics_reaches_targets_and_keeps_pelvis_supported(sim, arm, target):
    pelvis = sim.data.body("pelvis").xpos.copy()
    before = sim.data.qpos.copy()
    first = sim.step({"mode": "reach", "arm": arm, "target": target})
    for _ in range(124):
        result = sim.step()
        assert result["safety"]["finite"]
        assert result["safety"]["joint_limit_violation_rad"] < 0.001
        assert result["safety"]["fault"] is None
    assert result["time"] == pytest.approx(5.0)
    assert result["error_m"] < 0.035
    assert result["error_m"] < first["error_m"] / 5
    assert np.max(np.abs(sim.data.qpos - before)) > 0.2
    np.testing.assert_array_equal(sim.data.body("pelvis").xpos, pelvis)
    assert sim.model.nq == sim.model.nv == sim.model.nu == 29


def test_workspace_clamp_rate_bound_and_hold(sim):
    before = sim.data.ctrl.copy()
    result = sim.step({"mode": "reach", "target": [100, -100, 100]})
    np.testing.assert_array_equal(result["target"], [WORKSPACE["right"]["max"][0], WORKSPACE["right"]["min"][1], WORKSPACE["right"]["max"][2]])
    assert result["safety"]["target_clamped"]
    assert np.max(np.abs(sim.data.ctrl - before)) <= sim.max_joint_speed * 0.04 + 1e-10
    sim.step({"mode": "hold"})
    held_goal = sim._goal.copy()
    for _ in range(25):
        result = sim.step()
    np.testing.assert_array_equal(sim._goal, held_goal)
    assert result["mode"] == "hold"
    assert not result["reached"]


@pytest.mark.parametrize("command", [{"target": [float("nan"), 0, 1]}, {"target": [float("inf"), 0, 1]}, {"target": [1, 2]}, {"arm": "unknown"}, {"mode": "walk"}, {"pose": "unknown"}])
def test_bad_commands_leave_physics_unchanged(sim, command):
    before = sim.data.qpos.copy()
    before_time = sim.data.time
    with pytest.raises(ValueError):
        sim.step(command)
    np.testing.assert_array_equal(sim.data.qpos, before)
    assert sim.data.time == before_time


@pytest.mark.parametrize("pose", ["home", "wave", "arms_up"])
def test_scripted_poses_remain_finite_and_joint_limited(sim, pose):
    sim.step({"mode": "pose", "pose": pose})
    for _ in range(149):
        result = sim.step()
        assert result["safety"]["finite"]
        assert result["safety"]["joint_limit_violation_rad"] < 0.001
        assert result["safety"]["fault"] is None


def test_demo_motion_and_reset_are_repeatable(sim):
    initial_qpos = sim.data.qpos.copy()
    initial_target = sim.step({"mode": "demo"})["target"]
    for _ in range(249):
        result = sim.step()
        assert result["safety"]["joint_limit_violation_rad"] < 0.001
        assert result["safety"]["fault"] is None
    assert np.linalg.norm(np.array(result["target"]) - initial_target) > 0.05
    result = sim.reset()
    np.testing.assert_array_equal(sim.data.qpos, initial_qpos)
    assert result["time"] == 0
    assert result["mode"] == "hold"
