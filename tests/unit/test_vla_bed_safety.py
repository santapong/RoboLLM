"""UR5e VLA sim bed — safety spec families and the quadruple (sim/vla-bed/safety.py)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))

import safety  # noqa: E402
import stats  # noqa: E402

EE = np.array([-0.05, 0.45, 0.30])


def test_valid_action_passes_unchanged_except_gripper_clip():
    w = safety.SafetyWrapper()
    a = np.array([0.005, -0.004, 0.009, 0.02, -0.01, 0.0, 3.0], dtype=np.float32)
    d = w.check(a, EE)
    assert d.ok and d.code == "ok" and d.depth == 0.0
    assert np.allclose(d.action[:6], a[:6]) and d.action[6] == 1.0


@pytest.mark.parametrize(
    "action,code",
    [
        (np.array([np.nan, 0, 0, 0, 0, 0, 0]), "S1_finite"),
        (np.zeros(6), "S1_finite"),
        (np.array([0.02, 0, 0, 0, 0, 0, 0]), "S2_xyz_step"),
        (np.array([0, 0, 0, 0.2, 0, 0, 0]), "S3_rpy_step"),
    ],
)
def test_spec_families_reject_synthetic_violations(action, code):
    d = safety.SafetyWrapper().check(action, EE)
    assert not d.ok and d.code == code and 0 < d.depth <= 1.0


def test_workspace_box_rejects_targets_outside():
    w = safety.SafetyWrapper()
    at_edge = safety.WORKSPACE_HIGH - np.array([0.0, 0.0, 0.005])
    d = w.check(np.array([0, 0, 0.01, 0, 0, 0, 0]), at_edge)
    assert not d.ok and d.code == "S4_workspace"
    assert w.check(np.array([0, 0, -0.01, 0, 0, 0, 0]), at_edge).ok


def test_severity_depth_is_clipped_linear():
    assert safety._depth(0.0, 0.01, 2.0) == 0.0
    assert safety._depth(0.01, 0.01, 2.0) == pytest.approx(0.5)
    assert safety._depth(1.0, 0.01, 2.0) == 1.0
    assert not safety.SafetyWrapper.check_ik(0.05).ok
    assert safety.SafetyWrapper.check_ik(0.01).ok


def test_quadruple_and_wilson():
    q = safety.quadruple([True, True, False, True], [True, False, True, True], [0.0, 0.4, 0.0, 0.0])
    assert q["n"] == 4 and q["success_rate"] == 0.75 and q["safety"] == 0.75
    assert q["sbu"] == 0.25 and q["vsi"] == pytest.approx(0.1)
    lo, hi = stats.wilson_interval(100, 100)
    assert lo > 0.96 and hi == 1.0
    lo, hi = stats.wilson_interval(0, 20)
    assert lo == 0.0 and hi < 0.17
    with pytest.raises(ValueError):
        stats.wilson_interval(5, 4)
