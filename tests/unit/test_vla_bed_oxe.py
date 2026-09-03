"""UR5e VLA sim bed — OXE mapping and replay geometry (sim/vla-bed/oxe/). Pure numpy, no network."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))

from oxe import geom, map as oxe_map  # noqa: E402


def test_quaternion_and_rotvec_helpers():
    assert np.allclose(oxe_map.quat_xyzw_to_matrix(np.array([0, 0, 0, 1.0])), np.eye(3))
    R = geom.rot_z(np.deg2rad(10))
    assert np.isclose(np.degrees(oxe_map.rotation_error_rad(R, np.eye(3))), 10.0, atol=1e-6)
    v = np.array([0.01, -0.02, 0.03])
    assert np.allclose(oxe_map.matrix_to_rotvec(oxe_map.rotvec_to_matrix(v)), v)
    q = geom.mat_to_quat_wxyz(R)
    assert np.isclose(np.linalg.norm(q), 1.0) and q[0] > 0.99


def test_fit_recovers_signed_permutation_and_gain():
    rng = np.random.default_rng(0)
    P = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
    frames, episodes = 60, 20
    S, A, E = [], [], []
    for ep in range(episodes):
        a = rng.uniform(-0.02, 0.02, size=(frames, 7))
        a[:, 3:6] = rng.uniform(-1 / 15, 1 / 15, size=(frames, 3))
        a[:, 6] = 1.0
        pos = np.zeros((frames, 3))
        R = np.eye(3)
        rots = []
        for k in range(frames):
            rots.append(R)
            if k + 1 < frames:
                pos[k + 1] = pos[k] + 0.6 * P @ a[k, :3] + rng.normal(0, 1e-4, 3)
                R = oxe_map.rotvec_to_matrix(0.6 * P @ a[k, 3:6]) @ R
        quats = []
        for Rk in rots:
            w, x, y, z = geom.mat_to_quat_wxyz(Rk)
            quats.append([x, y, z, w])
        S.append(np.hstack([pos, np.array(quats), np.zeros((frames, 1))]))
        A.append(a)
        E.append(np.full(frames, ep))
    fit = oxe_map.fit_action_frame(np.vstack(S), np.vstack(A), np.concatenate(E))
    assert np.array_equal(np.asarray(fit["translation"]["P_action_to_state"]), P.astype(int))
    assert abs(fit["translation"]["gain"] - 0.6) < 0.02 and fit["translation"]["r2_lag0"] > 0.95
    assert np.array_equal(np.asarray(fit["rotation"]["P_action_to_state"]), P.astype(int))
    assert abs(fit["rotation"]["gain"] - 0.6) < 0.05


def test_substep_targets_split_the_motion_in_four():
    pa, pb = np.zeros(3), np.array([0.02, 0.0, -0.01])
    Ra, Rb = np.eye(3), geom.rot_z(0.06)
    targets = geom.substep_targets(pa, Ra, pb, Rb, 4)
    assert len(targets) == 4
    assert np.allclose(targets[0][0], pb / 4) and np.allclose(targets[-1][0], pb)
    assert np.allclose(targets[-1][1], Rb)
    assert np.isclose(np.degrees(oxe_map.rotation_error_rad(targets[1][1], geom.rot_z(0.03))), 0.0, atol=1e-6)


def test_rot_z_alignment_keeps_tool_down():
    tool_down = np.diag([1.0, -1.0, -1.0])  # z axis → −z
    for deg in (0, 90, 180, 270):
        R = geom.rot_z(np.deg2rad(deg)) @ tool_down
        assert np.allclose(R[:, 2], [0, 0, -1])


def test_map_file_if_present_is_verified():
    path = BED / "configs" / "oxe_ur5_map.yaml"
    if not path.exists():
        pytest.skip("map not generated yet")
    yaml = pytest.importorskip("yaml")
    doc = yaml.safe_load(path.read_text())
    assert doc["verified"] is True
    assert doc["state"]["quat_order"] == "xyzw"
    assert doc["action"]["gripper_coding"].startswith("1=open")
    P = np.asarray(doc["action_frame_vs_state_frame"]["translation"]["P_action_to_state"])
    assert P.shape == (3, 3) and np.allclose(np.abs(P).sum(1), 1)
