"""Magnitude probe: what a checkpoint predicts versus the labels it was trained on (SDD §8 P4).

The closed-loop suites showed 30–45 % of the policy's steps rejected at the S2 cap
(max |Δxyz| > 0.010 m per step). The training labels sit exactly on that cap (84 % of
steps saturate, audit of 4 Sep 2026), so two explanations compete: (a) unbiased
regression noise around a boundary label is rejected one-sidedly, or (b) the
trainer/normalisation biases the magnitude upward. This probe separates them
open-loop: for N training frames it predicts a chunk from the recorded image and
state, inverts the run's label representation, and compares the first
`n_action_steps` rows with the recorded `action` window, per axis.

Reports bias (pred − label), spread, the L∞ magnitudes of both, the fraction of
predicted steps over the cap versus the fraction of labels at the cap, and the
same for rotation. Sampling noise is seeded per frame.

    python sim/vla-bed/gpu/magnitude_probe.py --run baseline --checkpoint <dir>/pretrained_model --frames 500
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

import numpy as np

BED_DIR = Path(__file__).resolve().parents[1]
ROOT = BED_DIR.parents[1]
sys.path.insert(0, str(BED_DIR))
import labels  # noqa: E402
from safety import RPY_STEP_LIMIT_RAD, XYZ_STEP_LIMIT_M  # noqa: E402

GPU_CONFIG = BED_DIR / "gpu" / "config.json"
EPS = 1e-9


def frame_indices(num_frames: int, n: int, seed: int = 0) -> np.ndarray:
    """n frame indices spread over the dataset (every k-th, offset by a seeded start)."""
    n = min(n, num_frames)
    stride = max(1, num_frames // n)
    start = int(np.random.default_rng(seed).integers(0, stride))
    return np.arange(start, num_frames, stride)[:n]


def summarize(pred: np.ndarray, label: np.ndarray) -> dict:
    """pred, label: [M, 7] rows of base-frame per-step deltas (the first n_action_steps of every probed chunk)."""
    err = pred - label
    p_xyz = np.max(np.abs(pred[:, :3]), axis=1)
    l_xyz = np.max(np.abs(label[:, :3]), axis=1)
    p_rpy = np.max(np.abs(pred[:, 3:6]), axis=1)
    l_rpy = np.max(np.abs(label[:, 3:6]), axis=1)
    sat = l_xyz > XYZ_STEP_LIMIT_M - 1e-6  # labels sitting on the cap
    corr = [float(np.corrcoef(pred[:, i], label[:, i])[0, 1]) if np.std(label[:, i]) > 0 and np.std(pred[:, i]) > 0 else None for i in range(6)]
    return {
        "rows": int(len(pred)),
        "bias_per_axis": [round(float(x), 6) for x in err.mean(axis=0)],
        "spread_per_axis": [round(float(x), 6) for x in err.std(axis=0)],
        "label_abs_mean_per_axis": [round(float(x), 6) for x in np.abs(label).mean(axis=0)],
        "pred_abs_mean_per_axis": [round(float(x), 6) for x in np.abs(pred).mean(axis=0)],
        "corr_per_axis": corr,
        "xyz_linf": {"label_mean": round(float(l_xyz.mean()), 6), "pred_mean": round(float(p_xyz.mean()), 6), "ratio_pred_over_label": round(float(p_xyz.mean() / l_xyz.mean()), 4) if l_xyz.mean() > 0 else None,
                     "label_at_cap_fraction": round(float(sat.mean()), 4), "pred_over_cap_fraction": round(float(np.mean(p_xyz > XYZ_STEP_LIMIT_M + EPS)), 4),
                     "pred_over_cap_fraction_given_label_at_cap": round(float(np.mean(p_xyz[sat] > XYZ_STEP_LIMIT_M + EPS)), 4) if sat.any() else None,
                     "pred_over_cap_fraction_given_label_below_cap": round(float(np.mean(p_xyz[~sat] > XYZ_STEP_LIMIT_M + EPS)), 4) if (~sat).any() else None,
                     "pred_p50": round(float(np.median(p_xyz)), 6), "pred_p90": round(float(np.quantile(p_xyz, 0.9)), 6), "pred_max": round(float(p_xyz.max()), 6)},
        "rpy_linf": {"label_mean": round(float(l_rpy.mean()), 6), "pred_mean": round(float(p_rpy.mean()), 6), "label_at_cap_fraction": round(float(np.mean(l_rpy > RPY_STEP_LIMIT_RAD - 1e-6)), 4), "pred_over_cap_fraction": round(float(np.mean(p_rpy > RPY_STEP_LIMIT_RAD + EPS)), 4)},
        "direction_cosine_xyz_mean": round(float(np.mean([np.dot(p, l) / (np.linalg.norm(p) * np.linalg.norm(l)) for p, l in zip(pred[:, :3], label[:, :3]) if np.linalg.norm(p) > 0 and np.linalg.norm(l) > 0])), 4),
        "reading": None,
    }


def reading(s: dict) -> str:
    x = s["xyz_linf"]
    bias = max(abs(b) for b in s["bias_per_axis"][:3])
    rel_bias = bias / x["label_mean"] if x["label_mean"] else 0.0
    if rel_bias > 0.10:
        return f"magnitude BIAS: mean per-axis bias up to {bias:.4f} m ({100*rel_bias:.0f} % of the label L∞ mean) — look at the trainer/normalisation before recording new data"
    return f"no magnitude bias (≤ {100*rel_bias:.0f} % of the label mean); {100*x['pred_over_cap_fraction']:.0f} % of predicted steps exceed the cap while {100*x['label_at_cap_fraction']:.0f} % of labels sit on it — regression spread around a boundary label, i.e. headroom in the data is the fix"


def probe(checkpoint: str, dataset_root: Path, repo_id: str, representation: str, n_action_steps: int, chunk_size: int, frames: int, device: str | None, seed: int, instruction: str) -> dict:
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

    dev = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    policy = SmolVLAPolicy.from_pretrained(checkpoint).to(dev).eval()
    override = {"device_processor": {"device": str(dev)}}
    pre, post = make_pre_post_processors(policy.config, pretrained_path=checkpoint, preprocessor_overrides=override, postprocessor_overrides=override)
    fps = 20
    ds = LeRobotDataset(repo_id, root=str(dataset_root), delta_timestamps={"action": [i / fps for i in range(chunk_size)]})
    idx = frame_indices(ds.num_frames, frames, seed)
    preds, labs, per_frame = [], [], []
    t0 = time.perf_counter()
    for k, i in enumerate(idx):
        item = ds[int(i)]
        img = item["observation.images.front"]
        img = torch.as_tensor(np.asarray(img)) if not hasattr(img, "dim") else img
        if img.dim() == 3 and img.shape[-1] == 3:  # HWC uint8 → CHW float
            img = img.permute(2, 0, 1).float().div(255.0)
        img = img.float()
        state = np.asarray(item["observation.state"], dtype=np.float32).reshape(-1)[-14:]
        label = np.asarray(item["action"], dtype=np.float64).reshape(-1, 7)
        pad = np.asarray(item.get("action_is_pad", np.zeros(len(label), dtype=bool))).reshape(-1)
        batch = {"observation.images.front": img.to(dev), "observation.state": torch.from_numpy(state).to(dev), "task": instruction}
        if "observation.images.wrist" in item:  # recipe v6
            w = item["observation.images.wrist"]
            w = torch.as_tensor(np.asarray(w)) if not hasattr(w, "dim") else w
            if w.dim() == 3 and w.shape[-1] == 3:
                w = w.permute(2, 0, 1).float().div(255.0)
            batch["observation.images.wrist"] = w.float().to(dev)
        policy.reset()
        torch.manual_seed(seed + int(i))
        with torch.inference_mode():
            chunk = post(policy.predict_action_chunk(pre(batch)))
        chunk = np.asarray(chunk.detach().cpu().numpy() if hasattr(chunk, "detach") else chunk, dtype=np.float64)
        chunk = chunk[0] if chunk.ndim == 3 else chunk
        chunk = labels.invert(chunk, representation, state)
        keep = np.arange(min(n_action_steps, len(label)))
        keep = keep[~pad[keep]] if pad.any() else keep
        preds.append(chunk[keep])
        labs.append(label[keep])
        per_frame.append({"frame": int(i), "episode": int(item.get("episode_index", -1)), "pred_xyz_linf_first": round(float(np.max(np.abs(chunk[0, :3]))), 6), "label_xyz_linf_first": round(float(np.max(np.abs(label[0, :3]))), 6)})
        if k % 50 == 0:
            print(f"  {k}/{len(idx)} frames, {time.perf_counter() - t0:.0f} s", flush=True)
    pred = np.concatenate(preds)
    lab = np.concatenate(labs)
    s = summarize(pred, lab)
    s["reading"] = reading(s)
    return {
        "schema": "robollm.vla-bed.magnitude-probe.v1",
        "host": socket.gethostname(),
        "checkpoint": checkpoint,
        "representation": representation,
        "n_action_steps": n_action_steps,
        "frames": int(len(idx)),
        "dataset_root": str(dataset_root),
        "limits": {"xyz_step_m": XYZ_STEP_LIMIT_M, "rpy_step_rad": RPY_STEP_LIMIT_RAD},
        "summary": s,
        "per_frame": per_frame,
        "wall_s": round(time.perf_counter() - t0, 1),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--run", default="baseline")
    parser.add_argument("--frames", type=int, default=500)
    parser.add_argument("--dataset-root", type=Path, default=None)
    parser.add_argument("--device", default=None)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--label", default=None, help="results label, default <run>/<checkpoint step>")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    config = json.loads(GPU_CONFIG.read_text())
    run = next((r for r in config["runs"] if r["name"] == args.run), None)
    if run is None:
        raise SystemExit(f"unknown run {args.run!r}")
    root = args.dataset_root or (ROOT / config["dataset"]["train_root"])
    repo_id = config["dataset"]["train_repo_id"]
    instruction = config.get("instruction", "touch the red target")
    result = probe(args.checkpoint, root, repo_id, run["representation"], int(config["n_action_steps"]), int(config["chunk_size"]), args.frames, args.device, args.seed, instruction)
    step = Path(args.checkpoint).parent.name if Path(args.checkpoint).name == "pretrained_model" else Path(args.checkpoint).name
    label = args.label or f"{args.run}/{step}"
    out = args.out or (BED_DIR / "results" / "p5" / socket.gethostname() / label / "magnitude.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result["summary"].items() if k != "corr_per_axis"}, indent=1))
    print(f"→ {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
