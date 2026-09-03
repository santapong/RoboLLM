"""UR5e VLA sim bed — mink controller and scripted experts (sim/vla-bed/expert.py).

Needs mink and the pinned Menagerie checkout; skipped in the fast-CI venv.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))

mink = pytest.importorskip("mink")
if not (BED / "assets" / "mujoco_menagerie" / "universal_robots_ur5e" / "scene.xml").exists():
    pytest.skip("Menagerie checkout missing (run scripts/pi_setup.sh)", allow_module_level=True)

import expert  # noqa: E402
import families  # noqa: E402
from scene import build_scene  # noqa: E402


@pytest.fixture(scope="module")
def controller():
    return expert.MinkController(build_scene.load_model())


def test_clip_action_respects_step_limits():
    a = expert.clip_action(np.array([1, -1, 1, 1, -1, 1, 5.0]))
    assert np.allclose(a[:3], [0.01, -0.01, 0.01]) and np.allclose(a[3:6], [0.05, -0.05, 0.05]) and a[6] == 1.0


def test_settle_reaches_cell_centres(controller):
    for cell in families.cells()[::5]:
        assert controller.settle_residual(cell.centre) < 0.005


def test_solve_delta_moves_commanded_pose_by_the_delta(controller):
    q = controller.configuration.q.copy()
    q[controller.arm_qpos] = build_scene.HOME_QPOS
    controller.configuration.update(q)
    controller.posture_task.set_target(controller.configuration.q)
    before, _ = controller.ee_pose()
    delta = np.array([0.008, -0.005, 0.006, 0.0, 0.0, 0.0, 0.0])
    _, residual = controller.solve_delta(delta)
    after, _ = controller.ee_pose()
    assert residual < 0.002
    assert np.allclose(after - before, delta[:3], atol=0.002)


def test_noisy_expert_is_seeded_labels_clean_and_within_limits(controller):
    spec = families.episode_specs(1, 5, "train")[0]
    ee = np.array([-0.134, 0.492, 0.332])
    target = np.asarray(spec.target)
    oracle = expert.make_expert("oracle", controller.home_rot)
    noisy_a = expert.make_expert("noisy", controller.home_rot, 0.5)
    noisy_b = expert.make_expert("noisy", controller.home_rot, 0.5)
    noisy_a.reset(spec)
    noisy_b.reset(spec)
    oa = oracle.act(ee, controller.home_rot, target)
    na = noisy_a.act(ee, controller.home_rot, target)
    nb = noisy_b.act(ee, controller.home_rot, target)
    assert np.allclose(na.clean, oa.clean)  # the label is the clean expert action
    assert np.allclose(na.executed, nb.executed)  # seeded
    assert not np.allclose(na.executed, na.clean)  # noise was applied
    assert np.all(np.abs(na.executed[:3]) <= expert.XYZ_STEP_LIMIT_M + 1e-7)
    assert np.all(np.abs(na.executed[3:6]) <= expert.RPY_STEP_LIMIT_RAD + 1e-7)


def test_make_expert_rejects_bad_inputs(controller):
    with pytest.raises(ValueError):
        expert.make_expert("dagger", controller.home_rot)
    with pytest.raises(ValueError):
        expert.make_expert("noisy", controller.home_rot, -1.0)
