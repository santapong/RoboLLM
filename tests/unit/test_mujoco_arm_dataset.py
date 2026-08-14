"""Fast checks for the A3 MuJoCo arm and scripted policy."""
from pathlib import Path
import sys

import numpy as np
import pytest

pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "mujoco"))

from arm_dataset import JOINT_NAMES, build_model, run_policy, scripted_action  # noqa: E402


def test_model_has_six_arm_joints_plus_gripper_and_matching_actuators():
    model = build_model()
    assert model.nq == model.nu == 7
    assert JOINT_NAMES == [f"joint{i}" for i in range(1, 7)] + ["gripper"]


def test_scripted_policy_is_finite_and_inside_actuator_ranges():
    model = build_model()
    for frame in range(100):
        action = scripted_action(frame, 100)
        assert np.isfinite(action).all()
        assert np.all(action >= model.actuator_ctrlrange[:, 0])
        assert np.all(action <= model.actuator_ctrlrange[:, 1])


def test_headless_policy_moves_and_tracks_targets():
    metrics = run_policy(frames=100, fps=20)
    assert metrics["sim_seconds"] == pytest.approx(5.0)
    assert metrics["tracking_rmse_rad"] < 0.25
