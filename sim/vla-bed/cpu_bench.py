"""SmolVLA-base CPU inference cost on this machine (SDD §4, §8/P2).

Builds the batch from the checkpoint's own ``input_features`` (image keys, state
width) so the timed path is the real one: preprocessor (resize with padding to
the config size, normalisation, tokenisation) → ``predict_action_chunk`` → post.
Reports seconds per chunk and the implied maximum control rate for
n_action_steps ∈ {1, 10, 50} against the bed's 20 Hz. Nothing is trained and
nothing leaves the machine; the checkpoint is used for measurement only.

    .venv-lerobot/bin/python sim/vla-bed/cpu_bench.py [--checkpoint lerobot/smolvla_base] [--runs 10]
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import socket
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import resources  # noqa: E402
from env import INSTRUCTION  # noqa: E402

DEFAULT_CHECKPOINT = "lerobot/smolvla_base"


def cpu_model() -> str:
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--threads", type=int, default=0, help="torch threads (0 = torch default)")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--frame", type=Path, default=None, help="PNG to use as the camera frame (default: P1 oracle frame)")
    args = parser.parse_args()

    import torch
    from PIL import Image
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

    if args.threads:
        torch.set_num_threads(args.threads)
    host = socket.gethostname()
    out = args.out or (Path(__file__).resolve().parent / "results" / "p2" / host)
    out.mkdir(parents=True, exist_ok=True)

    t_load = time.perf_counter()
    policy = SmolVLAPolicy.from_pretrained(args.checkpoint)
    policy.eval()
    # The checkpoint's saved processor pipelines pin device='cuda'; override to CPU
    # (B1's SmolVLAPolicyAdapter in examples/mujoco/evaluate_reaching.py lacks this).
    cpu_override = {"device_processor": {"device": "cpu"}}
    pre, post = make_pre_post_processors(
        policy.config, pretrained_path=args.checkpoint, preprocessor_overrides=cpu_override, postprocessor_overrides=cpu_override
    )
    load_s = time.perf_counter() - t_load

    cfg = policy.config
    features = {k: (v.type.name if hasattr(v.type, "name") else str(v.type), list(v.shape)) for k, v in cfg.input_features.items()}
    image_keys = [k for k, (t, _) in features.items() if t.upper().startswith("VISUAL")]
    state_key = next((k for k, (t, _) in features.items() if t.upper().startswith("STATE")), None)
    state_dim = int(features[state_key][1][0]) if state_key else 0

    frame_path = args.frame or (Path(__file__).resolve().parent / "results" / "p1" / host / "oracle_front_high_0_final.png")
    if frame_path.exists():
        frame = np.asarray(Image.open(frame_path).convert("RGB"))
    else:
        frame = np.zeros((224, 224, 3), dtype=np.uint8)
    image = torch.from_numpy(frame).permute(2, 0, 1).float().div(255.0)

    bed_state = np.zeros(14, dtype=np.float32)
    state = np.zeros(max(state_dim, 1), dtype=np.float32)
    state[: min(14, state.shape[0])] = bed_state[: state.shape[0]]

    def make_obs() -> dict:
        obs = {k: image.clone() for k in image_keys}
        if state_key:
            obs[state_key] = torch.from_numpy(state.copy())
        obs["task"] = INSTRUCTION
        return obs

    pre_times, predict_times, post_times = [], [], []
    with torch.inference_mode():
        for i in range(args.runs + 2):
            policy.reset()
            t0 = time.perf_counter()
            batch = pre(make_obs())
            t1 = time.perf_counter()
            action = policy.predict_action_chunk(batch)
            t2 = time.perf_counter()
            action = post(action)
            t3 = time.perf_counter()
            if i >= 2:  # two warm-ups
                pre_times.append(t1 - t0)
                predict_times.append(t2 - t1)
                post_times.append(t3 - t2)
    chunk = np.asarray(action.detach().cpu().numpy() if hasattr(action, "detach") else action)
    total = np.array(pre_times) + np.array(predict_times) + np.array(post_times)
    med = float(np.median(total))
    result = {
        "schema": "robollm.vla-bed.cpu-bench.v1",
        "checkpoint": args.checkpoint,
        "host": {"hostname": host, "machine": platform.machine(), "cpu": cpu_model(), "python": platform.python_version(), "torch": torch.__version__, "threads": torch.get_num_threads()},
        "config": {"chunk_size": cfg.chunk_size, "n_action_steps": cfg.n_action_steps, "max_state_dim": cfg.max_state_dim, "max_action_dim": cfg.max_action_dim, "resize_imgs_with_padding": list(cfg.resize_imgs_with_padding) if cfg.resize_imgs_with_padding else None, "num_steps": getattr(cfg, "num_steps", None), "vlm": getattr(cfg, "vlm_model_name", None)},
        "input_features": features,
        "batch_built_from": {"image_keys": image_keys, "state_key": state_key, "state_dim_used": state_dim, "frame": str(frame_path) if frame_path.exists() else "zeros", "frame_hw": list(frame.shape[:2])},
        "action_chunk_shape": list(chunk.shape),
        "timing_s": {
            "load": round(load_s, 2),
            "runs": args.runs,
            "preprocess_median": round(float(np.median(pre_times)), 4),
            "predict_median": round(float(np.median(predict_times)), 4),
            "postprocess_median": round(float(np.median(post_times)), 4),
            "total_median": round(med, 4),
            "total_p95": round(float(np.percentile(total, 95)), 4),
            "total_min": round(float(np.min(total)), 4),
        },
        "implied_control_hz": {str(n): round(n / med, 3) for n in (1, 10, 50)},
        "bed_control_hz": 20,
        "lockstep_note": "In the bed the simulator only advances on request, so this latency slows wall-clock, never the policy's view (SDD §0).",
        "resources": resources.snapshot(),
        "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    (out / "cpu_bench.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in ("host", "config", "batch_built_from", "action_chunk_shape", "timing_s", "implied_control_hz")}, indent=2))
    print(f"cpu_bench → {out / 'cpu_bench.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
