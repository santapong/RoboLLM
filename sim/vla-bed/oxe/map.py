"""Data-verified mapping between lerobot/berkeley_autolab_ur5 and the bed (SDD §6.3).

Every field below is checked against the parquet before the YAML is written:
state slices, quaternion order, action ranges, gripper coding, and — the finding
that makes this file necessary — the rotation between the action frame and the
state frame, fitted by least squares on Δstate ≈ gain · P · action.

    .venv-lerobot/bin/python sim/vla-bed/oxe/map.py --verify      # prints and writes configs/oxe_ur5_map.yaml
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

BED_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = BED_DIR.parents[1] / "datasets" / "oxe" / "berkeley_autolab_ur5"
MAP_FILE = BED_DIR / "configs" / "oxe_ur5_map.yaml"

DOC_XYZ_LIMIT = 0.02  # m per 5 Hz step (dataset website)
DOC_RPY_LIMIT = 1.0 / 15.0  # rad per step


def quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotvec_to_matrix(v: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    th = float(np.arccos(c))
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def rotation_error_rad(Ra: np.ndarray, Rb: np.ndarray) -> float:
    """Geodesic angle between two rotations, via the Frobenius norm.

    ‖Ra − Rb‖_F = 2√2·sin(θ/2), so SimplerEnv's eq. 4, arcsin(‖·‖_F / 2√2), equals θ/2.
    The bed reports the full angle θ (twice SimplerEnv's number) and says so in the SDD.
    """
    return float(2.0 * np.arcsin(np.clip(np.linalg.norm(Ra - Rb) / (2 * np.sqrt(2)), 0.0, 1.0)))


def load_arrays(root: Path = DATA_DIR):
    import pandas as pd

    df = pd.read_parquet(root / "data" / "chunk-000" / "file-000.parquet")
    S = np.stack(df["observation.state"].to_numpy()).astype(np.float64)
    A = np.stack(df["action"].to_numpy()).astype(np.float64)
    return df, S, A, df["episode_index"].to_numpy()


def fit_action_frame(S: np.ndarray, A: np.ndarray, E: np.ndarray, max_episodes: int = 300) -> dict:
    """Least-squares Δstate_xyz ≈ M · action_xyz within episodes; nearest rotation + gain."""
    X, Y, RX, RY = [], [], [], []
    for ep in np.unique(E)[:max_episodes]:
        idx = np.where(E == ep)[0]
        s, a = S[idx], A[idx]
        X.append(a[:-1, :3])
        Y.append(s[1:, :3] - s[:-1, :3])
        # rotation: relative rotation between consecutive states as a rotvec, vs action rpy
        for k in range(len(idx) - 1):
            Rk, Rk1 = quat_xyzw_to_matrix(s[k, 3:7]), quat_xyzw_to_matrix(s[k + 1, 3:7])
            RY.append(matrix_to_rotvec(Rk1 @ Rk.T))
            RX.append(a[k, 3:6])
    X, Y = np.vstack(X), np.vstack(Y)
    M, *_ = np.linalg.lstsq(X, Y, rcond=None)
    Mt = M.T  # Δstate = Mt @ action
    U, sv, Vt = np.linalg.svd(Mt)
    P = U @ Vt
    if np.linalg.det(P) < 0:  # keep a proper rotation
        U[:, -1] *= -1
        P = U @ Vt
    P = np.round(P)  # the fitted rotation is a signed permutation to 2 decimals
    gain = float(np.mean(sv))
    resid = Y - X @ M
    r2 = float(1 - (resid**2).sum() / ((Y - Y.mean(0)) ** 2).sum())
    RX, RY = np.vstack(RX), np.vstack(RY)
    Mr, *_ = np.linalg.lstsq(RX, RY, rcond=None)
    Ur, svr, Vtr = np.linalg.svd(Mr.T)
    Pr = np.round(Ur @ Vtr)
    r2r = float(1 - ((RY - RX @ Mr) ** 2).sum() / ((RY - RY.mean(0)) ** 2).sum())
    return {
        "translation": {"P_action_to_state": P.astype(int).tolist(), "gain": round(gain, 3), "singular_values": np.round(sv, 3).tolist(), "r2_lag0": round(r2, 3)},
        "rotation": {"P_action_to_state": Pr.astype(int).tolist(), "gain": round(float(np.mean(svr)), 3), "singular_values": np.round(svr, 3).tolist(), "r2": round(r2r, 3)},
    }


def verify(root: Path = DATA_DIR) -> dict:
    df, S, A, E = load_arrays(root)
    info = json.loads((root / "meta" / "info.json").read_text())
    q = S[:, 3:7]
    norms = np.linalg.norm(q, axis=1)
    tool_z_xyzw = np.mean([quat_xyzw_to_matrix(qq)[:, 2] for qq in q[::50]], axis=0)
    tool_z_wxyz = np.mean([quat_xyzw_to_matrix(np.r_[qq[1:], qq[0]])[:, 2] for qq in q[::50]], axis=0)
    quat_order = "xyzw" if tool_z_xyzw[2] < tool_z_wxyz[2] else "wxyz"
    in_xyz = np.mean(np.all(np.abs(A[:, :3]) <= DOC_XYZ_LIMIT + 1e-6, axis=1))
    in_rpy = np.mean(np.all(np.abs(A[:, 3:6]) <= DOC_RPY_LIMIT + 1e-6, axis=1))
    g_vals, g_cnt = np.unique(A[:, 6], return_counts=True)
    nxt, act = [], []
    for ep in np.unique(E)[:300]:
        idx = np.where(E == ep)[0]
        act.append(A[idx[:-1], 6])
        nxt.append(S[idx[1:], 7])
    act, nxt = np.concatenate(act), np.concatenate(nxt)
    p_closed_given_1 = float(nxt[act == 1].mean()) if np.any(act == 1) else float("nan")
    p_closed_given_0 = float(nxt[act == 0].mean()) if np.any(act == 0) else float("nan")
    gripper_coding = "1=open,0=close (absolute)" if p_closed_given_1 < p_closed_given_0 else "1=close,0=open (absolute)"
    frame = fit_action_frame(S, A, E)
    doc = {
        "schema": "robollm.vla-bed.oxe-ur5-map.v1",
        "repo_id": "lerobot/berkeley_autolab_ur5",
        "fps": int(info.get("fps", 5)),
        "verified": bool(in_xyz > 0.99 and in_rpy > 0.99 and abs(norms.mean() - 1) < 1e-3 and frame["translation"]["r2_lag0"] > 0.9),
        "state": {
            "layout": "[x, y, z, qx, qy, qz, qw, gripper_is_closed] = robot_state[6:14] (Octo transform); base frame",
            "xyz": [0, 3],
            "quat": [3, 7],
            "quat_order": quat_order,
            "quat_norm_mean": round(float(norms.mean()), 5),
            "tool_z_axis_mean_under_xyzw": np.round(tool_z_xyzw, 3).tolist(),
            "gripper": 7,
            "gripper_values": sorted(np.unique(S[:, 7]).tolist()),
            "xyz_min": np.round(S[:, :3].min(0), 3).tolist(),
            "xyz_max": np.round(S[:, :3].max(0), 3).tolist(),
        },
        "action": {
            "layout": "[dx, dy, dz, droll, dpitch, dyaw, gripper]",
            "xyz": [0, 3],
            "rpy": [3, 6],
            "gripper": 6,
            "doc_xyz_limit_m": DOC_XYZ_LIMIT,
            "doc_rpy_limit_rad": round(DOC_RPY_LIMIT, 5),
            "fraction_within_doc_xyz": round(float(in_xyz), 4),
            "fraction_within_doc_rpy": round(float(in_rpy), 4),
            "q01": np.round(np.quantile(A, 0.01, axis=0), 4).tolist(),
            "q99": np.round(np.quantile(A, 0.99, axis=0), 4).tolist(),
            "gripper_values": {str(round(float(v), 3)): int(c) for v, c in zip(g_vals, g_cnt)},
            "gripper_coding": gripper_coding,
            "p_next_closed_given_action_1": round(p_closed_given_1, 3),
            "p_next_closed_given_action_0": round(p_closed_given_0, 3),
        },
        "action_frame_vs_state_frame": {
            **frame,
            "reading": "Δstate ≈ gain · P · action with P a signed permutation: the actions are commands in a rotated teleop frame and the real controller realised only `gain` of each command per 5 Hz step. For the bed: apply P (and the rotation P to rotation vectors) to express real actions in the state/base frame; the gain is the real controller's tracking, not part of the convention.",
        },
        "bed_alignment": {
            "note": "chosen by measurement in oxe_replay.py (state mode): rotation about z by k·90° and a z offset so the lowest real pose sits 0.10 m above the bed floor; filled in below",
            "chosen": None,
        },
        "sample_count": int(len(S)),
    }
    return doc


def write_yaml(doc: dict, path: Path = MAP_FILE) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import yaml

        path.write_text(yaml.safe_dump(doc, sort_keys=False, allow_unicode=True))
    except ImportError:
        path.with_suffix(".json").write_text(json.dumps(doc, indent=2) + "\n")


def load_map(path: Path = MAP_FILE) -> dict:
    if path.exists():
        import yaml

        return yaml.safe_load(path.read_text())
    return json.loads(path.with_suffix(".json").read_text())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    if not args.verify:
        parser.print_help()
        return 0
    doc = verify()
    write_yaml(doc)
    print(json.dumps({k: doc[k] for k in ("verified", "state", "action", "action_frame_vs_state_frame")}, indent=1))
    print(f"→ {MAP_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
