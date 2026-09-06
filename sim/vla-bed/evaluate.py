"""Bed evaluator (SDD §8 P4/P5): the frozen suite, scripted baselines and SmolVLA checkpoints.

Frozen suite = the v2 recipe's *evaluation* split (100 seeds, 20 cells, never
trained on), read from the dataset manifest so every policy sees the same
targets and initial joints. Every step goes through the bed's own controller
and SafetyWrapper (env.step), so S1–S7 are measured exactly as in recording.
The camera is rendered only when the policy is about to be queried (success and
error are state-based), which is what makes 100 episodes affordable on a slow CPU.

Policies:
  oracle            the P1 scripted expert (control: must be 100 % on nominal)
  hold              zero deltas (control: must be 0 %)
  smolvla           a LeRobot checkpoint; --checkpoint DIR, --representation from
                    labels.py (or --run NAME to read it from gpu/config.json)

Probes (SDD §14 R6/R11): --blank-image feeds an all-black frame to the policy
(the env still renders; success must collapse if the policy uses vision);
--gain 0.61 executes 0.61 × every commanded delta before re-observing, the real
UR5's measured per-step realisation (P3). Policy-side post-processors (`--post`)
sit before the safety wrapper, like --gain, and are reported as such:
  clip              clip every pose delta to the S2/S3 limits (how much of the gap is the cap alone)
  ensemble          linear temporal ensemble over overlapping chunk predictions
                    (Lazzati et al. 2608.02547): re-plan every --replan-every steps and
                    execute the mean of every prediction made for the step

Every episode row records the commanded and executed step magnitudes and the
rejected-step fraction, so "the policy's steps exceed the cap" is a measurement.
--shard i/n evaluates every n-th episode (parallel workers); --merge joins shards.

    .venv-lerobot/bin/python sim/vla-bed/evaluate.py --policy oracle
    .venv-lerobot/bin/python sim/vla-bed/evaluate.py --policy smolvla --run baseline \\
        --checkpoint artifacts/vla-bed/baseline/full/checkpoints/020000/pretrained_model --label baseline/020000
    .venv-lerobot/bin/python sim/vla-bed/evaluate.py --merge a_shard0of2.json a_shard1of2.json --out a.json
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
from safety import RPY_STEP_LIMIT_RAD, XYZ_STEP_LIMIT_M, quadruple  # noqa: E402
from stats import bootstrap_mean, wilson_interval  # noqa: E402

DEFAULT_MANIFEST = ROOT / "datasets" / "vla-bed" / "v2" / "manifest.json"
GPU_CONFIG = BED_DIR / "gpu" / "config.json"
POLICIES = ("oracle", "hold", "smolvla")
POSTS = ("none", "clip", "ensemble")
DEFAULT_N_ACTION_STEPS = 10
SCHEMA = "robollm.vla-bed.phase-summary.v2"
EPS = 1e-9


# ----- suite -----


def load_suite(manifest_path: Path = DEFAULT_MANIFEST, split: str = "evaluation", episodes: int | None = None, shard: tuple[int, int] | None = None) -> tuple[dict, list[EpisodeSpec]]:
    manifest = json.loads(Path(manifest_path).read_text())
    entries = manifest["splits"][split]["episodes"]
    if episodes is not None:
        entries = entries[:episodes]
    if shard is not None:
        i, n = shard
        entries = entries[i::n]
    specs = [EpisodeSpec(int(e["seed"]), e["split"], e["family"], int(e["cell"]), tuple(e["target"]), tuple(e["initial_q"])) for e in entries]
    return manifest, specs


def parse_shard(text: str | None) -> tuple[int, int] | None:
    if not text:
        return None
    i, n = (int(x) for x in text.split("/"))
    if n < 1 or not 0 <= i < n:
        raise SystemExit(f"--shard must be i/n with 0 <= i < n, got {text!r}")
    return (i, n)


# ----- policies: reset(spec) + act(env, observation) -> chunk [K, 7] of base-frame per-step deltas -----


class OraclePolicy:
    name = "oracle"

    def __init__(self, env, headroom: float = 1.0) -> None:
        from expert import make_expert

        self.expert = make_expert("oracle", env.controller.home_rot, headroom=headroom)

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

    def __init__(self, checkpoint: str, representation: str = "identity", instruction: str = "touch the red target", blank_image: bool = False, device: str | None = None, seed: int | None = 0, vlm_dtype: str | None = None) -> None:
        import torch
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy as _Policy

        self.torch = torch
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.policy = _Policy.from_pretrained(checkpoint)
        if vlm_dtype and vlm_dtype != "float32":  # train/eval dtype consistency probe: the frozen VLM as it ran in training
            self.policy.model.vlm_with_expert.vlm.to(getattr(torch, vlm_dtype))
        self.policy.to(self.device).eval()
        override = {"device_processor": {"device": str(self.device)}}
        self.pre, self.post = make_pre_post_processors(self.policy.config, pretrained_path=checkpoint, preprocessor_overrides=override, postprocessor_overrides=override)
        self.representation = representation
        self.instruction = instruction
        self.blank_image = blank_image
        self.chunk_size = int(self.policy.config.chunk_size)
        self.seed = seed
        self.vlm_dtype = vlm_dtype

    def reset(self, spec) -> None:
        self.policy.reset()
        if self.seed is not None:  # flow-matching sampling is stochastic: seed per episode so repeated suites are comparable (SDD §13)
            self.torch.manual_seed(int(self.seed) + int(spec.seed))

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
        wrist = observation.get("observation.images.wrist")
        if wrist is not None:  # recipe v6: the checkpoint's own rename map routes it to camera2
            wrist = np.zeros_like(np.asarray(wrist)) if self.blank_image else np.asarray(wrist)
            batch["observation.images.wrist"] = torch.from_numpy(wrist.copy()).permute(2, 0, 1).float().div(255.0).to(self.device)
        with torch.inference_mode():
            chunk = self.post(self.policy.predict_action_chunk(self.pre(batch)))
        chunk = np.asarray(chunk.detach().cpu().numpy() if hasattr(chunk, "detach") else chunk, dtype=np.float64)
        chunk = chunk[0] if chunk.ndim == 3 else chunk
        return labels.invert(chunk, self.representation, state)


def make_policy(name: str, env, args) -> object:
    if name == "oracle":
        return OraclePolicy(env, getattr(args, "oracle_headroom", 1.0))
    if name == "hold":
        return HoldPolicy()
    if name == "smolvla":
        if not args.checkpoint:
            raise SystemExit("--checkpoint is required for smolvla")
        return SmolVLAPolicy(str(args.checkpoint), args.representation, blank_image=args.blank_image, device=args.device, seed=args.sampling_seed, vlm_dtype=args.vlm_dtype)
    raise SystemExit(f"unknown policy {name!r}")


# ----- policy-side post-processing (before the safety wrapper; reported, never silent) -----


def postprocess(action: np.ndarray, gain: float = 1.0, post: str = "none") -> np.ndarray:
    """One base-frame delta as the wrapper will see it: the gain probe, then the optional clip to the S2/S3 limits."""
    a = np.array(action, dtype=np.float64, copy=True)
    if gain != 1.0:
        a[:6] *= gain
    if post == "clip":
        a[:3] = np.clip(a[:3], -XYZ_STEP_LIMIT_M, XYZ_STEP_LIMIT_M)
        a[3:6] = np.clip(a[3:6], -RPY_STEP_LIMIT_RAD, RPY_STEP_LIMIT_RAD)
    return a


class TemporalEnsemble:
    """Linear temporal ensemble over overlapping chunk predictions (Lazzati et al. 2608.02547, AC(n)-TE).

    `add(t0, chunk)` registers a prediction for steps t0, t0+1, …; `pop(t)` returns the plain
    mean of every prediction made for step t (exponential weighting, as in ACT, is deliberately
    not used: the paper shows it weakens the ensembling effect)."""

    def __init__(self) -> None:
        self.pending: dict[int, list[np.ndarray]] = {}

    def add(self, t0: int, chunk: np.ndarray) -> None:
        for i, a in enumerate(np.asarray(chunk, dtype=np.float64)):
            self.pending.setdefault(t0 + i, []).append(a)

    def has(self, t: int) -> bool:
        return t in self.pending

    def pop(self, t: int) -> tuple[np.ndarray, int]:
        preds = self.pending.pop(t)
        return np.mean(preds, axis=0), len(preds)


# ----- rollout -----


def run_policy_episode(env, policy, spec: EpisodeSpec, variation: str, n_action_steps: int, gain: float, max_frames: int, post: str = "none", replan_every: int | None = None, ensemble_horizon: int | None = None) -> dict:
    from env import PROGRESS_HALF_DISTANCE_M

    env.reset(spec, variation)
    policy.reset(spec)
    ensemble = TemporalEnsemble() if post == "ensemble" else None
    replan = int(replan_every or n_action_steps)
    horizon = int(ensemble_horizon or n_action_steps)
    success, frames, chunks, latencies = False, 0, 0, []
    cmd_xyz: list[float] = []
    cmd_rpy: list[float] = []
    exec_xyz: list[float] = []
    members: list[int] = []
    rejected = 0
    queue: list[np.ndarray] = []
    while frames < max_frames and not success:
        need_plan = (frames % replan == 0 or not ensemble.has(frames)) if ensemble is not None else not queue
        if need_plan:
            observation = env.observation()  # the only render of the loop: when the policy needs a frame
            t0 = time.perf_counter()
            chunk = np.asarray(policy.act(env, observation), dtype=np.float64)
            latencies.append(time.perf_counter() - t0)
            chunks += 1
            if ensemble is None:
                queue = [row for row in chunk[:n_action_steps]]
            else:
                ensemble.add(frames, chunk[:horizon])
        if ensemble is None:
            raw = queue.pop(0)
        else:
            raw, k = ensemble.pop(frames)
            members.append(k)
        a = postprocess(raw, gain, post)
        cmd_xyz.append(float(np.max(np.abs(a[:3]))))
        cmd_rpy.append(float(np.max(np.abs(a[3:6]))))
        result = env.step(a, render=False)
        frames += 1
        if result.decision.ok:
            exec_xyz.append(float(np.max(np.abs(np.asarray(result.executed, dtype=np.float64)[:3]))))
        else:
            rejected += 1
        if result.success:
            success = True
    cx, cr = np.asarray(cmd_xyz), np.asarray(cmd_rpy)
    row = {
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
        "rejected_steps": rejected,
        "rejected_fraction": round(rejected / frames, 4) if frames else 0.0,
        "cmd_xyz_linf_mean": round(float(cx.mean()), 5) if frames else 0.0,
        "cmd_xyz_linf_max": round(float(cx.max()), 5) if frames else 0.0,
        "cmd_xyz_over_cap_fraction": round(float(np.mean(cx > XYZ_STEP_LIMIT_M + EPS)), 4) if frames else 0.0,
        "cmd_rpy_linf_mean": round(float(cr.mean()), 5) if frames else 0.0,
        "cmd_rpy_linf_max": round(float(cr.max()), 5) if frames else 0.0,
        "cmd_rpy_over_cap_fraction": round(float(np.mean(cr > RPY_STEP_LIMIT_RAD + EPS)), 4) if frames else 0.0,
        "exec_xyz_linf_mean": round(float(np.mean(exec_xyz)), 5) if exec_xyz else 0.0,
    }
    if ensemble is not None:
        row["ensemble_members_mean"] = round(float(np.mean(members)), 3) if members else 0.0
    return row


def _mean_of(rows: list[dict], key: str) -> float | None:
    vals = [r[key] for r in rows if key in r]
    return float(np.mean(vals)) if vals else None


def _max_of(rows: list[dict], key: str) -> float | None:
    vals = [r[key] for r in rows if key in r]
    return float(np.max(vals)) if vals else None


def aggregate(rows: list[dict]) -> dict:
    """The §9 per-policy block: quadruple, Wilson CIs, progress, lengths, per-family success, faults, magnitudes."""
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
    # rejected fraction is derivable for rows written before schema v2 (rejections are per step)
    rejected_fraction = [r["rejected_fraction"] if "rejected_fraction" in r else (sum(r["rejections"].values()) / r["frames"] if r["frames"] else 0.0) for r in rows]
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
        "rejected_fraction_mean": float(np.mean(rejected_fraction)),
        "rejected_steps_total": int(sum(sum(r["rejections"].values()) for r in rows)),
        "frames_total": int(sum(lengths)),
        "cmd_xyz_linf_mean": _mean_of(rows, "cmd_xyz_linf_mean"),
        "cmd_xyz_linf_max": _max_of(rows, "cmd_xyz_linf_max"),
        "cmd_xyz_over_cap_fraction_mean": _mean_of(rows, "cmd_xyz_over_cap_fraction"),
        "cmd_rpy_linf_mean": _mean_of(rows, "cmd_rpy_linf_mean"),
        "cmd_rpy_over_cap_fraction_mean": _mean_of(rows, "cmd_rpy_over_cap_fraction"),
        "exec_xyz_linf_mean": _mean_of(rows, "exec_xyz_linf_mean"),
        "ensemble_members_mean": _mean_of(rows, "ensemble_members_mean"),
        "per_family_success": {f: sum(v) / len(v) for f, v in sorted(families.items())},
        "per_family_n": {f: len(v) for f, v in sorted(families.items())},
        "faults": {"rejected": rejected, "measured": measured},
    }


def evaluate(args) -> dict:
    from env import FPS, INSTRUCTION, MAX_FRAMES, BedEnv

    shard = parse_shard(args.shard)
    manifest, specs = load_suite(args.manifest, "evaluation", args.episodes, shard)
    if args.env and args.env != "local":  # P5: the bed served by sim_server.py on another machine (SDD §6.4)
        from remote_env import RemoteEnv, ZmqTransport
        env = RemoteEnv(ZmqTransport(args.env))
    else:
        import dataset as ds
        recipe = ds.RECIPES.get(str(manifest.get("recipe", "")))
        env = BedEnv(render=True, wrist_camera=bool(recipe and recipe.wrist_camera))
    try:
        policy = make_policy(args.policy, env, args)
        t0 = time.perf_counter()
        rows = [run_policy_episode(env, policy, spec, args.variation, args.n_action_steps, args.gain, MAX_FRAMES, args.post, args.replan_every, args.ensemble_horizon) for spec in specs]
        wall = time.perf_counter() - t0
    finally:
        env.close()
    label = args.label or args.policy
    summary = {
        "schema": SCHEMA,
        "phase": "P5" if args.policy == "smolvla" else "P4-pre",
        "route": "M",
        "host": {"hostname": socket.gethostname(), "machine": platform.machine(), "python": platform.python_version(), "MUJOCO_GL": os.environ.get("MUJOCO_GL", "")},
        "env": ({"transport": args.env, "server": getattr(env, "info", None), "wire_s_total": round(getattr(env, "wire_s", 0.0), 2), "requests": getattr(env, "requests", 0)} if args.env and args.env != "local" else {"transport": "local"}),
        "task": INSTRUCTION,
        "embodiment": "ur5e+2f85",
        "schedule": {"manifest": str(Path(args.manifest).relative_to(ROOT)) if str(args.manifest).startswith(str(ROOT)) else str(args.manifest), "recipe": manifest["recipe"], "split": "evaluation", "base_seed": manifest["splits"]["evaluation"]["base_seed"], "episodes": len(specs), "cells": manifest["cells"], "variation": args.variation, "fps": FPS, "max_frames": MAX_FRAMES, "shard": args.shard},
        "limits": {"xyz_step_m": XYZ_STEP_LIMIT_M, "rpy_step_rad": RPY_STEP_LIMIT_RAD},
        "policy": {"name": args.policy, "label": label, "checkpoint": str(args.checkpoint) if args.checkpoint else None, "representation": args.representation, "n_action_steps": args.n_action_steps if args.policy == "smolvla" else 1, "gain": args.gain, "blank_image": bool(args.blank_image), "sampling_seed": args.sampling_seed, "oracle_headroom": args.oracle_headroom if args.policy == "oracle" else None, "post": args.post, "replan_every": args.replan_every if args.post == "ensemble" else None, "ensemble_horizon": args.ensemble_horizon if args.post == "ensemble" else None, "vlm_dtype": args.vlm_dtype},
        label: aggregate(rows),
        "episodes": {label: rows},
        "wall_s": round(wall, 1),
        "resources": resources.snapshot(),
        "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    return summary


def merge_summaries(parts: list[dict]) -> dict:
    """Join shard results of one suite into the summary a single run would have written."""
    if not parts:
        raise ValueError("nothing to merge")
    first = parts[0]
    label = first["policy"]["label"]
    for p in parts[1:]:
        if p["policy"]["label"] != label or {k: v for k, v in p["policy"].items()} != {k: v for k, v in first["policy"].items()} or p["schedule"]["variation"] != first["schedule"]["variation"]:
            raise ValueError("shards disagree on policy or variation")
    rows = sorted((r for p in parts for r in p["episodes"][label]), key=lambda r: r["seed"])
    seeds = [r["seed"] for r in rows]
    if len(set(seeds)) != len(seeds):
        raise ValueError("duplicate seeds across shards")
    merged = json.loads(json.dumps(first))
    merged["schedule"]["episodes"] = len(rows)
    merged["schedule"]["shard"] = None
    merged["schedule"]["merged_from"] = [p["schedule"].get("shard") for p in parts]
    merged[label] = aggregate(rows)
    merged["episodes"] = {label: rows}
    merged["wall_s"] = round(max(p["wall_s"] for p in parts), 1)
    merged["wall_s_total"] = round(sum(p["wall_s"] for p in parts), 1)
    merged["measured_at"] = max(p["measured_at"] for p in parts)
    return merged


def default_out(args) -> Path:
    suffix = args.variation + ("_blank" if args.blank_image else "") + (f"_gain{args.gain:g}" if args.gain != 1.0 else "")
    if args.post == "clip":
        suffix += "_clip"
    elif args.post == "ensemble":
        suffix += f"_ens{args.replan_every or args.n_action_steps}"
    if args.vlm_dtype and args.vlm_dtype != "float32":
        suffix += f"_{args.vlm_dtype}"
    if getattr(args, "env", "local") not in (None, "local"):
        suffix += "_zmq"
    if args.shard:
        i, n = parse_shard(args.shard)
        suffix += f"_shard{i}of{n}"
    return BED_DIR / "results" / "p5" / socket.gethostname() / (args.label or args.policy) / f"{suffix}.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", choices=POLICIES, default=None)
    parser.add_argument("--checkpoint", type=Path, default=None)
    parser.add_argument("--run", default=None, help="run name in gpu/config.json; sets --representation and --n-action-steps")
    parser.add_argument("--representation", default=None, choices=labels.REPRESENTATIONS)
    parser.add_argument("--n-action-steps", type=int, default=None)
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--env", default="local", help='"local" (in-process BedEnv) or zmq://host:5555 (sim_server.py on the Pi, SDD §6.4)')
    parser.add_argument("--variation", default="nominal")
    parser.add_argument("--gain", type=float, default=1.0)
    parser.add_argument("--blank-image", action="store_true")
    parser.add_argument("--post", choices=POSTS, default="none", help="policy-side post-processor before the safety wrapper (a reported probe)")
    parser.add_argument("--replan-every", type=int, default=None, help="ensemble: re-query the policy every k steps (default n_action_steps)")
    parser.add_argument("--ensemble-horizon", type=int, default=None, help="ensemble: use only the first H rows of each chunk (default n_action_steps)")
    parser.add_argument("--vlm-dtype", default=None, choices=("float32", "float16", "bfloat16"), help="cast the frozen VLM at eval (train/eval dtype consistency probe)")
    parser.add_argument("--oracle-headroom", type=float, default=1.0, help="oracle policy: cap the expert at this fraction of the S2/S3 limits (recipe v3 = 0.7)")
    parser.add_argument("--device", default=None)
    parser.add_argument("--sampling-seed", type=int, default=0, help="seed for the policy's flow-matching noise (added to the episode seed); -1 = unseeded")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--shard", default=None, help="i/n: evaluate episodes i, i+n, i+2n, … (parallel workers); merge with --merge")
    parser.add_argument("--merge", nargs="+", type=Path, default=None, help="shard result files to merge into --out")
    parser.add_argument("--label", default=None)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    if args.merge:
        if not args.out:
            raise SystemExit("--merge needs --out")
        summary = merge_summaries([json.loads(p.read_text()) for p in args.merge])
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(summary, indent=2) + "\n")
        label = summary["policy"]["label"]
        print(json.dumps({"merged": [str(p) for p in args.merge], "n": summary[label]["n"], "success_rate": summary[label]["success_rate"], "ci95": summary[label]["ci95_wilson_success"]}, indent=1))
        print(f"→ {args.out}")
        return 0
    if not args.policy:
        raise SystemExit("--policy is required (or --merge)")
    if args.run:
        config = json.loads(GPU_CONFIG.read_text())
        run = next((r for r in config["runs"] if r["name"] == args.run), None)
        if run is None:
            raise SystemExit(f"unknown run {args.run!r}")
        args.representation = args.representation or run["representation"]
        args.n_action_steps = args.n_action_steps or config["n_action_steps"]
    args.representation = args.representation or "identity"
    args.n_action_steps = args.n_action_steps or DEFAULT_N_ACTION_STEPS
    if args.policy == "oracle" and args.oracle_headroom != 1.0 and not args.label:
        args.label = f"oracle_h{args.oracle_headroom:g}"
    if args.sampling_seed is not None and args.sampling_seed < 0:
        args.sampling_seed = None
    summary = evaluate(args)
    out = args.out or default_out(args)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2) + "\n")
    label = summary["policy"]["label"]
    block = summary[label]
    print(json.dumps({"policy": label, "variation": args.variation, "post": args.post, "n": block["n"], "success_rate": block["success_rate"], "ci95": block.get("ci95_wilson_success"), "safety": block["safety"], "sbu": block["sbu"], "vsi": round(block["vsi"], 4), "progress_mean": round(block.get("progress_mean", 0.0), 3), "episode_len_mean": round(block.get("episode_len_mean", 0.0), 1), "rejected_fraction_mean": round(block.get("rejected_fraction_mean", 0.0), 3), "cmd_xyz_linf_mean": block.get("cmd_xyz_linf_mean"), "cmd_xyz_over_cap_fraction_mean": block.get("cmd_xyz_over_cap_fraction_mean"), "per_family_success": block.get("per_family_success"), "faults": block.get("faults"), "wall_s": summary["wall_s"], "peak_rss_mb": summary["resources"]["peak_rss_mb"]}, indent=1))
    print(f"→ {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
