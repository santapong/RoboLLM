"""Bed evaluator (SDD §8 P4/P5): the frozen suite, scripted baselines and SmolVLA checkpoints.

Frozen suite = the v2 recipe's *evaluation* split (100 seeds, 20 cells, never
trained on), read from the dataset manifest so every policy sees the same
targets and initial joints. Every step goes through the bed's own controller
and SafetyWrapper (env.step), so S1–S7 are measured exactly as in recording.

Policies:
  oracle            the P1 scripted expert (control: must be 100 % on nominal)
  hold              zero deltas (control: must be 0 %)
  smolvla           a LeRobot checkpoint; --checkpoint DIR, --representation from
                    labels.py (or --run NAME to read it from gpu/config.json)

Probes (SDD §14 R6/R11): --blank-image feeds an all-black frame to the policy
(the env still renders; success must collapse if the policy uses vision);
--gain 0.61 executes 0.61 × every commanded delta before re-observing, the real
UR5's measured per-step realisation (P3).

    .venv-lerobot/bin/python sim/vla-bed/evaluate.py --policy oracle
    .venv-lerobot/bin/python sim/vla-bed/evaluate.py --policy smolvla --run baseline \
        --checkpoint artifacts/vla-bed/baseline/full/checkpoints/020000/pretrained_model --label baseline/020000
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

BED_DIR = Path(__file__).resolve().parent
ROOT = BED_DIR.parents[1]
sys.path.insert(0, str(BED_DIR))
import labels  # noqa: E402
import resources  # noqa: E402
from families import EpisodeSpec  # noqa: E402
from safety import quadruple  # noqa: E402
from stats import bootstrap_mean, wilson_interval  # noqa: E402

DEFAULT_MANIFEST = ROOT / "datasets" / "vla-bed" / "v2" / "manifest.json"
GPU_CONFIG = BED_DIR / "gpu" / "config.json"
POLICIES = ("oracle", "hold", "smolvla")
DEFAULT_N_ACTION_STEPS = 10


# ----- suite -----


def load_suite(manifest_path: Path = DEFAULT_MANIFEST, split: str = "evaluation", episodes: int | None = None) -> tuple[dict, list[EpisodeSpec]]:
    manifest = json.loads(Path(manifest_path).read_text())
    entries = manifest["splits"][split]["episodes"]
    if episodes is not None:
        entries = entries[:episodes]
    specs = [EpisodeSpec(int(e["seed"]), e["split"], e["family"], int(e["cell"]), tuple(e["target"]), tuple(e["initial_q"])) for e in entries]
    return manifest, specs


# ----- policies: reset(spec) + act(env, observation) -> chunk [K, 7] of base-frame per-step deltas -----


class OraclePolicy:
    name = "oracle"

    def __init__(self, env) -> None:
        from expert import make_expert

        self.expert = make_expert("oracle", env.controller.home_rot)

    def reset(self, spec) -> None:
        self.expert.reset(spec)

    def act(self, env, observation) -> np.ndarray:  # noqa: ARG002 — the oracle reads the env, not the image
        cmd_pos, cmd_rot = env.commanded_ee
        return self.expert.act(cmd_pos, cmd_rot, env.target).executed.reshape(1, 7)


class HoldPolicy:
    name = "hold"

    def reset(self, spec) -> None:  # noqa: ARG002
        return None

    def act(self, env, observation) -> np.ndarray:  # noqa: ARG002
        return np.zeros((1, 7))


class SmolVLAPolicy:
    """LeRobot checkpoint adapter (B1's pattern) with the cpu/cuda device override and the label inverse."""

    name = "smolvla"

    def __init__(self, checkpoint: str, representation: str = "identity", instruction: str = "touch the red target", blank_image: bool = False, device: str | None = None) -> None:
        import torch
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy as _Policy

        self.torch = torch
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.policy = _Policy.from_pretrained(checkpoint)
        self.policy.to(self.device).eval()
        override = {"device_processor": {"device": str(self.device)}}
        self.pre, self.post = make_pre_post_processors(self.policy.config, pretrained_path=checkpoint, preprocessor_overrides=override, postprocessor_overrides=override)
        self.representation = representation
        self.instruction = instruction
        self.blank_image = blank_image
        self.chunk_size = int(self.policy.config.chunk_size)

    def reset(self, spec) -> None:  # noqa: ARG002
        self.policy.reset()

    def act(self, env, observation) -> np.ndarray:  # noqa: ARG002
        torch = self.torch
        image = np.asarray(observation["observation.images.front"])
        if self.blank_image:
            image = np.zeros_like(image)
        state = np.asarray(observation["observation.state"], dtype=np.float32)
        batch = {
            "observation.images.front": torch.from_numpy(image.copy()).permute(2, 0, 1).float().div(255.0).to(self.device),
            "observation.state": torch.from_numpy(state).to(self.device),
            "task": self.instruction,
        }
        with torch.inference_mode():
            chunk = self.post(self.policy.predict_action_chunk(self.pre(batch)))
        chunk = np.asarray(chunk.detach().cpu().numpy() if hasattr(chunk, "detach") else chunk, dtype=np.float64)
        chunk = chunk[0] if chunk.ndim == 3 else chunk
        return labels.invert(chunk, self.representation, state)


def make_policy(name: str, env, args) -> object:
    if name == "oracle":
        return OraclePolicy(env)
    if name == "hold":
        return HoldPolicy()
    if name == "smolvla":
        if not args.checkpoint:
            raise SystemExit("--checkpoint is required for smolvla")
        return SmolVLAPolicy(str(args.checkpoint), args.representation, blank_image=args.blank_image, device=args.device)
    raise SystemExit(f"unknown policy {name!r}")


# ----- rollout -----


def run_policy_episode(env, policy, spec: EpisodeSpec, variation: str, n_action_steps: int, gain: float, max_frames: int) -> dict:
    from env import PROGRESS_HALF_DISTANCE_M

    observation = env.reset(spec, variation)
    policy.reset(spec)
    success, frames, chunks, latencies = False, 0, 0, []
    while frames < max_frames and not success:
        t0 = time.perf_counter()
        chunk = np.asarray(policy.act(env, observation), dtype=np.float64)
        latencies.append(time.perf_counter() - t0)
        chunks += 1
        for action in chunk[:n_action_steps]:
            a = action.copy()
            if gain != 1.0:
                a[:6] *= gain
            result = env.step(a)
            frames += 1
            observation = result.observation
            if result.success:
                success = True
                break
            if frames >= max_frames:
                break
    return {
        "seed": spec.seed,
        "family": spec.family,
        "cell": spec.cell,
        "variation": variation,
        "success": success,
        "progress": 1.0 if success else (0.5 if env.min_error_m <= PROGRESS_HALF_DISTANCE_M else 0.0),
        "frames": frames,
        "chunks": chunks,
        "final_error_m": round(env.error_m, 5),
        "min_error_m": round(float(env.min_error_m), 5),
        "safe": env.safety.safe,
        "rejections": dict(env.safety.rejections),
        "measured": dict(env.safety.measured),
        "worst_depth": round(env.safety.worst_depth, 4),
        "latency_s_mean": round(float(np.mean(latencies)), 4) if latencies else 0.0,
    }


def aggregate(rows: list[dict]) -> dict:
    """The §9 per-policy block: quadruple, Wilson CIs, progress, lengths, per-family success, faults."""
    q = quadruple([r["success"] for r in rows], [r["safe"] for r in rows], [r["worst_depth"] for r in rows])
    n = q["n"]
    if n == 0:
        return q
    successes = sum(r["success"] for r in rows)
    safes = sum(r["safe"] for r in rows)
    lengths = [r["frames"] for r in rows]
    progress = [r["progress"] for r in rows]
    families: dict[str, list[bool]] = {}
    for r in rows:
        families.setdefault(r["family"], []).append(bool(r["success"]))
    rejected: dict[str, int] = {}
    measured: dict[str, int] = {}
    for r in rows:
        for k, v in r["rejections"].items():
            rejected[k] = rejected.get(k, 0) + v
        for k, v in r["measured"].items():
            measured[k] = measured.get(k, 0) + v
    return {
        **q,
        "ci95_wilson_success": list(wilson_interval(successes, n)),
        "ci95_wilson_safety": list(wilson_interval(safes, n)),
        "progress_mean": float(np.mean(progress)),
        "progress_ci95_bootstrap": list(bootstrap_mean(progress)),
        "episode_len_mean": float(np.mean(lengths)),
        "episode_len_ci95_bootstrap": list(bootstrap_mean(lengths)),
        "chunks_mean": float(np.mean([r.get("chunks", 0) for r in rows])),
        "latency_s_mean": float(np.mean([r.get("latency_s_mean", 0.0) for r in rows])),
        "final_error_m_mean": float(np.mean([r["final_error_m"] for r in rows])),
        "per_family_success": {f: sum(v) / len(v) for f, v in sorted(families.items())},
        "per_family_n": {f: len(v) for f, v in sorted(families.items())},
        "faults": {"rejected": rejected, "measured": measured},
    }


def evaluate(args) -> dict:
    from env import FPS, INSTRUCTION, MAX_FRAMES, BedEnv

    manifest, specs = load_suite(args.manifest, "evaluation", args.episodes)
    env = BedEnv(render=True)
    try:
        policy = make_policy(args.policy, env, args)
        t0 = time.perf_counter()
        rows = [run_policy_episode(env, policy, spec, args.variation, args.n_action_steps, args.gain, MAX_FRAMES) for spec in specs]
        wall = time.perf_counter() - t0
    finally:
        env.close()
    label = args.label or args.policy
    summary = {
        "schema": "robollm.vla-bed.phase-summary.v1",
        "phase": "P5" if args.policy == "smolvla" else "P4-pre",
        "route": "M",
        "host": {"hostname": socket.gethostname(), "machine": platform.machine(), "python": platform.python_version(), "MUJOCO_GL": os.environ.get("MUJOCO_GL", "")},
        "task": INSTRUCTION,
        "embodiment": "ur5e+2f85",
        "schedule": {"manifest": str(Path(args.manifest).relative_to(ROOT)) if str(args.manifest).startswith(str(ROOT)) else str(args.manifest), "recipe": manifest["recipe"], "split": "evaluation", "base_seed": manifest["splits"]["evaluation"]["base_seed"], "episodes": len(specs), "cells": manifest["cells"], "variation": args.variation, "fps": FPS, "max_frames": MAX_FRAMES},
        "limits": {"xyz_step_m": 0.01, "rpy_step_rad": 0.05},
        "policy": {"name": args.policy, "label": label, "checkpoint": str(args.checkpoint) if args.checkpoint else None, "representation": args.representation, "n_action_steps": args.n_action_steps if args.policy == "smolvla" else 1, "gain": args.gain, "blank_image": bool(args.blank_image)},
        label: aggregate(rows),
        "episodes": {label: rows},
        "wall_s": round(wall, 1),
        "resources": resources.snapshot(),
        "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    return summary


def default_out(args) -> Path:
    suffix = args.variation + ("_blank" if args.blank_image else "") + (f"_gain{args.gain:g}" if args.gain != 1.0 else "")
    return BED_DIR / "results" / "p5" / socket.gethostname() / (args.label or args.policy) / f"{suffix}.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", choices=POLICIES, required=True)
    parser.add_argument("--checkpoint", type=Path, default=None)
    parser.add_argument("--run", default=None, help="run name in gpu/config.json; sets --representation and --n-action-steps")
    parser.add_argument("--representation", default=None, choices=labels.REPRESENTATIONS)
    parser.add_argument("--n-action-steps", type=int, default=None)
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--variation", default="nominal")
    parser.add_argument("--gain", type=float, default=1.0)
    parser.add_argument("--blank-image", action="store_true")
    parser.add_argument("--device", default=None)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--label", default=None)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    if args.run:
        config = json.loads(GPU_CONFIG.read_text())
        run = next((r for r in config["runs"] if r["name"] == args.run), None)
        if run is None:
            raise SystemExit(f"unknown run {args.run!r}")
        args.representation = args.representation or run["representation"]
        args.n_action_steps = args.n_action_steps or config["n_action_steps"]
    args.representation = args.representation or "identity"
    args.n_action_steps = args.n_action_steps or DEFAULT_N_ACTION_STEPS
    summary = evaluate(args)
    out = args.out or default_out(args)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2) + "\n")
    label = summary["policy"]["label"]
    block = summary[label]
    print(json.dumps({"policy": label, "variation": args.variation, "n": block["n"], "success_rate": block["success_rate"], "ci95": block.get("ci95_wilson_success"), "safety": block["safety"], "sbu": block["sbu"], "vsi": round(block["vsi"], 4), "progress_mean": round(block.get("progress_mean", 0.0), 3), "episode_len_mean": round(block.get("episode_len_mean", 0.0), 1), "per_family_success": block.get("per_family_success"), "faults": block.get("faults"), "wall_s": summary["wall_s"], "peak_rss_mb": summary["resources"]["peak_rss_mb"]}, indent=1))
    print(f"→ {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
