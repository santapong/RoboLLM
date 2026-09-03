"""Pure-numpy pose helpers for the OXE replay (importable without MuJoCo)."""

from __future__ import annotations

import numpy as np

from oxe.map import matrix_to_rotvec, quat_xyzw_to_matrix, rotation_error_rad, rotvec_to_matrix  # noqa: F401


def rot_z(theta_rad: float) -> np.ndarray:
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def slerp_matrix(Ra: np.ndarray, Rb: np.ndarray, u: float) -> np.ndarray:
    """Geodesic interpolation between rotation matrices, u ∈ [0, 1]."""
    rel = matrix_to_rotvec(Rb @ Ra.T)
    return rotvec_to_matrix(u * rel) @ Ra


def interpolate_pose(pa: np.ndarray, Ra: np.ndarray, pb: np.ndarray, Rb: np.ndarray, u: float) -> tuple[np.ndarray, np.ndarray]:
    return pa + u * (pb - pa), slerp_matrix(Ra, Rb, u)


def substep_targets(pa, Ra, pb, Rb, n: int) -> list[tuple[np.ndarray, np.ndarray]]:
    """The n intermediate targets (u = 1/n … 1) between two 5 Hz samples for a 20 Hz bed (n = 4)."""
    return [interpolate_pose(pa, Ra, pb, Rb, (i + 1) / n) for i in range(n)]


def mat_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    if i == 1:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])
