"""UR5e VLA sim bed — environment contract and expert rollouts (sim/vla-bed/env.py).

Headless (render=False); needs mink and the Menagerie checkout, skipped otherwise.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))

pytest.importorskip("mink")
if not (BED / "assets" / "mujoco_menagerie" / "universal_robots_ur5e" / "scene.xml").exists():
    pytest.skip("Menagerie checkout missing (run scripts/pi_setup.sh)", allow_module_level=True)

import env as bed_env  # noqa: E402
import families  # noqa: E402
from expert import make_expert  # noqa: E402


@pytest.fixture(scope="module")
def env():
    e = bed_env.BedEnv(render=False)
    yield e
    e.close()


def test_observation_contract_matches_b1_keys_and_new_shapes(env):
    spec = families.episode_specs(1, 70_000, "evaluation")[0]
    obs = env.reset(spec)
    assert set(obs) == {"observation.images.front", "observation.state", "observation.camera_lag_ms"}
    assert obs["observation.images.front"].shape == (224, 224, 3)
    assert obs["observation.state"].shape == (14,) and obs["observation.state"].dtype == np.float32
    assert obs["observation.camera_lag_ms"].shape == (1,)
    assert env.substeps == 25
    feats = bed_env.dataset_features()
    assert feats["action"]["shape"] == (7,) and feats["action.executed"]["shape"] == (7,)
    assert feats["observation.state"]["names"][:3] == ["ee_x", "ee_y", "ee_z"]


def test_rejected_action_holds_and_is_counted(env):
    spec = families.episode_specs(1, 70_000, "evaluation")[0]
    env.reset(spec)
    before = env.end_effector.copy()
    result = env.step(np.array([0.5, 0, 0, 0, 0, 0, 0]))
    assert not result.decision.ok and result.decision.code == "S2_xyz_step"
    assert np.linalg.norm(env.end_effector - before) < 1e-3
    assert env.safety.rejections == {"S2_xyz_step": 1} and not env.safety.safe


def test_success_needs_five_consecutive_frames():
    d = bed_env.SuccessDetector()
    assert [d.update(0.01) for _ in range(4)] == [False] * 4
    assert d.update(0.01) is True
    d.update(0.1)
    assert d.streak == 0


@pytest.mark.parametrize("variation", bed_env.VARIATIONS)
def test_oracle_succeeds_on_three_cells_under_every_variation(env, variation):
    specs = families.episode_specs(20, 70_000, "evaluation")[::7][:3]
    oracle = make_expert("oracle", env.controller.home_rot)
    for spec in specs:
        row = bed_env.run_episode(env, oracle, spec, variation)
        assert row["success"], row
        assert row["safe"], row
        assert row["frames"] <= 60


def test_unknown_variation_rejected(env):
    with pytest.raises(ValueError):
        env.reset(families.episode_specs(1, 1, "train")[0], "occlusion")


def test_camera_azimuth_keeps_distance_and_aims_at_the_lookat(env):
    import mujoco

    spec = families.episode_specs(1, 10_000, "evaluation")[0]
    env.reset(spec)
    pos0, quat0 = env.model.cam_pos[env.cam_id].copy(), env.model.cam_quat[env.cam_id].copy()
    env.reset(spec, camera_azimuth_deg=20.0)
    pos = env.model.cam_pos[env.cam_id].copy()
    assert not np.allclose(pos, pos0)
    assert np.isclose(np.linalg.norm(pos - env.cam_lookat), np.linalg.norm(pos0 - env.cam_lookat))
    assert np.isclose(pos[2], pos0[2])  # elevation kept
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, env.model.cam_quat[env.cam_id])
    forward = -mat.reshape(3, 3)[:, 2]  # MuJoCo cameras look down their -z axis
    aim = env.cam_lookat - pos
    assert np.isclose(np.dot(forward, aim) / np.linalg.norm(aim), 1.0, atol=1e-6)
    env.reset(spec)
    assert np.allclose(env.model.cam_pos[env.cam_id], pos0) and np.allclose(env.model.cam_quat[env.cam_id], quat0)


def test_oracle_with_headroom_succeeds_under_camera_azimuth(env):
    from expert import make_expert

    spec = families.episode_specs(1, 10_000, "evaluation")[0]
    expert = make_expert("oracle", env.controller.home_rot, headroom=0.7)
    env.reset(spec, camera_azimuth_deg=-15.0)
    expert.reset(spec)
    success = False
    for _ in range(100):
        pos, rot = env.commanded_ee
        result = env.step(expert.act(pos, rot, env.target).executed)
        if result.success:
            success = True
            break
    assert success and env.safety.safe


def test_camera_translation_keeps_orientation_and_far_shift_variation(env):
    spec = families.episode_specs(1, 10_000, "evaluation")[0]
    env.reset(spec)
    pos0, quat0 = env.model.cam_pos[env.cam_id].copy(), env.model.cam_quat[env.cam_id].copy()
    env.reset(spec, camera_translation_m=(0.1, -0.05, 0.02))
    assert np.allclose(env.model.cam_pos[env.cam_id], pos0 + np.array([0.1, -0.05, 0.02]))
    assert np.allclose(env.model.cam_quat[env.cam_id], quat0)  # translation only: no re-aim
    env.reset(spec, variation="camera_shift_far")
    assert np.allclose(env.model.cam_pos[env.cam_id], pos0 + bed_env.CAMERA_SHIFT_FAR_M)
    env.reset(spec)
    assert np.allclose(env.model.cam_pos[env.cam_id], pos0)
