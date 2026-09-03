"""Train-time action-label representations for the bed (SDD §14 R1/R3, Phase 4).

The datasets store per-step deltas in the base frame (P1 decision). Two other
representations from the literature depend on the *chunk's start frame*, so they
cannot be stored per frame and are applied to the sampled ``[chunk, 7]`` window
at training time, then inverted at inference:

- ``gripper_frame`` (Feng 2602.23408, ZETA 2609.02546): every translation and
  rotation delta of the chunk is expressed in the end-effector frame at the
  chunk's first observation, R_t. Exact, since a fixed frame change maps
  rotation vectors as ordinary vectors: a' = R_tᵀ a, a = R_t a'.
- ``chunk_delta`` (Feng 2602.23408 "chunk-wise"): each entry is the *cumulative*
  displacement from the chunk's start pose — translation by summation, rotation
  by exact composition of the per-step rotation vectors (log of the product,
  in the base frame, the same order the bed's controller applies them).
  The gripper channel (a command, integrated by the env) stays per-step.

``identity`` is the stored representation. All functions are numpy, torch-free,
and their inverses round-trip to 1e-9 (tests).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from oxe.map import matrix_to_rotvec, rotvec_to_matrix  # noqa: E402

REPRESENTATIONS = ("identity", "gripper_frame", "chunk_delta")


def quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q)  # states are float32; renormalise so R is orthonormal to 1e-15
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def ee_rotation_from_state(state: np.ndarray) -> np.ndarray:
    """R_t from the bed's state vector (indices 3:7 are the EE quaternion, wxyz)."""
    return quat_wxyz_to_matrix(np.asarray(state, dtype=np.float64)[3:7])


# ----- gripper frame -----


def to_gripper_frame(actions: np.ndarray, R_t: np.ndarray) -> np.ndarray:
    a = np.asarray(actions, dtype=np.float64)
    out = a.copy()
    out[:, :3] = a[:, :3] @ R_t  # (R_tᵀ a)ᵀ row-wise
    out[:, 3:6] = a[:, 3:6] @ R_t
    return out


def from_gripper_frame(actions: np.ndarray, R_t: np.ndarray) -> np.ndarray:
    a = np.asarray(actions, dtype=np.float64)
    out = a.copy()
    out[:, :3] = a[:, :3] @ R_t.T
    out[:, 3:6] = a[:, 3:6] @ R_t.T
    return out


# ----- chunk-wise cumulative deltas -----


def to_chunk_delta(actions: np.ndarray) -> np.ndarray:
    a = np.asarray(actions, dtype=np.float64)
    out = a.copy()
    out[:, :3] = np.cumsum(a[:, :3], axis=0)
    R = np.eye(3)
    for k in range(len(a)):
        R = rotvec_to_matrix(a[k, 3:6]) @ R  # base-frame pre-multiplication, as the controller applies it
        out[k, 3:6] = matrix_to_rotvec(R)
    return out


def from_chunk_delta(actions: np.ndarray) -> np.ndarray:
    a = np.asarray(actions, dtype=np.float64)
    out = a.copy()
    out[0, :3] = a[0, :3]
    out[1:, :3] = np.diff(a[:, :3], axis=0)
    prev = np.eye(3)
    for k in range(len(a)):
        R = rotvec_to_matrix(a[k, 3:6])
        out[k, 3:6] = matrix_to_rotvec(R @ prev.T)
        prev = R
    return out


# ----- dispatch -----


def transform(actions: np.ndarray, representation: str, state_t: np.ndarray | None = None) -> np.ndarray:
    """Stored per-step base-frame deltas → the training representation."""
    if representation == "identity":
        return np.asarray(actions, dtype=np.float64).copy()
    if representation == "gripper_frame":
        return to_gripper_frame(actions, ee_rotation_from_state(state_t))
    if representation == "chunk_delta":
        return to_chunk_delta(actions)
    raise ValueError(f"unknown representation {representation!r}; choose from {REPRESENTATIONS}")


def invert(actions: np.ndarray, representation: str, state_t: np.ndarray | None = None) -> np.ndarray:
    """Training representation → per-step base-frame deltas the env executes."""
    if representation == "identity":
        return np.asarray(actions, dtype=np.float64).copy()
    if representation == "gripper_frame":
        return from_gripper_frame(actions, ee_rotation_from_state(state_t))
    if representation == "chunk_delta":
        return from_chunk_delta(actions)
    raise ValueError(f"unknown representation {representation!r}; choose from {REPRESENTATIONS}")


def window_stats(windows: np.ndarray) -> dict[str, np.ndarray]:
    """LeRobot-style stats (mean/std/min/max/q01/q99/count) over transformed windows [N, K, 7]."""
    flat = np.asarray(windows, dtype=np.float64).reshape(-1, windows.shape[-1])
    return {
        "mean": flat.mean(0).astype(np.float32),
        "std": flat.std(0).astype(np.float32),
        "min": flat.min(0).astype(np.float32),
        "max": flat.max(0).astype(np.float32),
        "q01": np.quantile(flat, 0.01, axis=0).astype(np.float32),
        "q99": np.quantile(flat, 0.99, axis=0).astype(np.float32),
        "count": np.array([len(flat)]),
    }
