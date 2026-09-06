"""One DAgger round (recipe v7): the trained policy drives, the scripted oracle labels (SDD §7.2; plan step 5).

The policy runs closed-loop on N fresh train-split seeds (a seed block disjoint from the base recipe's train and from
the frozen suite), every visited frame is recorded with the oracle's capped clean action as ``action`` and the
policy's executed action as ``action.executed`` (the DART convention the recipes already use; a rejected step
stores zeros as executed, exactly what moved the arm). Camera jitter follows the base recipe's train rules so the
rollouts match its viewpoint distribution. The result is a LeRobot dataset under ``<output_root>/v7/train`` plus a
manifest; ``merge`` concatenates it with the base recipe's train split (LeRobot ``aggregate_datasets``) for training.

    python sim/vla-bed/dagger.py relabel --base v5a --run baseline --checkpoint <ckpt>/pretrained_model --episodes 400
    python sim/vla-bed/dagger.py merge   --base v5a            # → datasets/vla-bed/v7/train-merged (base ∪ rollouts)
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

BED_DIR = Path(__file__).resolve().parent
ROOT = BED_DIR.parents[1]
sys.path.insert(0, str(BED_DIR))
import dataset as ds  # noqa: E402
import families  # noqa: E402
from expert import make_expert  # noqa: E402

DAGGER_RECIPE = "v7"
DAGGER_BASE_SEED = 20_000  # disjoint from the recipes' 10_000 block (train and the frozen evaluation suite)
GPU_CONFIG = BED_DIR / "gpu" / "config.json"


def relabel(base_name: str, run: str, checkpoint: str, episodes: int, output_root: Path = ds.DEFAULT_OUTPUT_ROOT, base_seed: int = DAGGER_BASE_SEED, device: str | None = None,
            sampling_seed: int = 0, dataset_class: Any | None = None, policy: Any | None = None, env: Any | None = None, height: int = 224, width: int = 224) -> dict:
    from env import FPS, INSTRUCTION, MAX_FRAMES, BedEnv, dataset_features
    from evaluate import postprocess

    base = ds.RECIPES[base_name]
    config = json.loads(GPU_CONFIG.read_text())
    run_cfg = next(r for r in config["runs"] if r["name"] == run)
    n_action_steps = int(config["n_action_steps"])
    specs = families.episode_specs(episodes, base_seed, "train")
    frozen = {(c["family"], c["cell"]) for c in families.load_frozen()["verified"]}
    missing = sorted({(s.family, s.cell) for s in specs} - frozen)
    if missing:
        raise ds.RecordingFault(f"cells not IK-verified: {missing}")
    out = Path(output_root) / DAGGER_RECIPE
    root = out / "train"
    repo_id = f"local/robollm-vla-bed-{DAGGER_RECIPE}-train"
    features = dataset_features(height, width, base.wrist_camera)
    dataset = ds._dataset_type(dataset_class).create(repo_id=repo_id, root=root, fps=FPS, robot_type=ds.ROBOT_TYPE, features=features, use_videos=True)
    own_env = env is None
    env = env or BedEnv(render=True, height=height, width=width, wrist_camera=base.wrist_camera)
    if policy is None:
        from evaluate import SmolVLAPolicy
        policy = SmolVLAPolicy(checkpoint, run_cfg["representation"], device=device, seed=sampling_seed)
    oracle = make_expert("oracle", env.controller.home_rot, 0.0, base.headroom)
    rows: list[dict] = []
    successes, t0 = 0, time.perf_counter()
    try:
        for index, spec in enumerate(specs):
            azimuth = ds.camera_azimuth(base, spec, "train")
            translation = ds.camera_translation(base, spec, "train")
            observation = env.reset(spec, camera_azimuth_deg=azimuth, camera_translation_m=translation)
            policy.reset(spec)
            oracle.reset(spec)
            queue: list[np.ndarray] = []
            frames, rejected, success = 0, 0, False
            while frames < MAX_FRAMES and not success:
                if not queue:
                    chunk = np.asarray(policy.act(env, observation), dtype=np.float64)
                    queue = [row for row in chunk[:n_action_steps]]
                a = postprocess(queue.pop(0))
                cmd_pos, cmd_rot = env.commanded_ee
                label = oracle.act(cmd_pos, cmd_rot, env.target).clean
                result = env.step(a, render=True)
                dataset.add_frame({**observation, "action": np.asarray(label, dtype=np.float32), "action.executed": np.asarray(result.executed, dtype=np.float32),
                                   "observation.noise_sigma": np.asarray([0.0], dtype=np.float32), "task": INSTRUCTION})
                observation = result.observation
                frames += 1
                rejected += int(not result.decision.ok)
                success = bool(result.success)
            dataset.save_episode()
            successes += int(success)
            rows.append({**spec.to_dict(), "frame_count": frames, "success": success, "final_error_m": round(env.error_m, 5), "min_error_m": round(float(env.min_error_m), 5),
                         "rejected_steps": rejected, "safe": env.safety.safe, "camera_azimuth_deg": round(azimuth, 4), "camera_translation_m": [round(v, 4) for v in translation]})
            if index % 25 == 0:
                print(f"  {index}/{len(specs)} episodes, policy success so far {successes}/{index + 1}, {time.perf_counter() - t0:.0f} s", flush=True)
    except Exception:
        dataset.clear_episode_buffer()
        raise
    finally:
        if own_env:
            env.close()
        dataset.finalize()
    manifest = {
        "schema": ds.MANIFEST_SCHEMA, "recipe": DAGGER_RECIPE, "base_recipe": base_name, "host": socket.gethostname(), "instruction": INSTRUCTION, "fps": FPS, "max_frames": MAX_FRAMES,
        "relabel": {"run": run, "checkpoint": str(checkpoint), "representation": run_cfg["representation"], "n_action_steps": n_action_steps, "sampling_seed": sampling_seed,
                    "labeller": "oracle", "headroom": base.headroom, "wrist_camera": base.wrist_camera},
        "features": {k: {**v, "shape": list(v["shape"])} for k, v in features.items()},
        "splits": {"train": {"repo_id": repo_id, "root": str(root), "base_seed": base_seed, "expert": "dagger", "noise_fraction": 0.0, "clean_every": None, "headroom": base.headroom,
                             "camera_jitter_deg": base.camera_jitter_deg, "camera_translate_m": base.camera_translate_m, "wrist_camera": base.wrist_camera,
                             "episode_count": len(specs), "frame_count": sum(r["frame_count"] for r in rows), "success_count": successes,
                             "policy_success_rate": successes / len(specs) if specs else 0.0, "wall_s": round(time.perf_counter() - t0, 1), "episodes": rows}},
    }
    ds._write_manifest(out / "manifest.json", manifest)
    return {"recipe": DAGGER_RECIPE, "base": base_name, "episodes": len(specs), "policy_successes": successes, "frames": manifest["splits"]["train"]["frame_count"], "manifest": str(out / "manifest.json")}


def merge(base_name: str, output_root: Path = ds.DEFAULT_OUTPUT_ROOT) -> Path:
    """base train split ∪ DAgger rollouts → <output_root>/v7/train-merged (LeRobot aggregate_datasets, episode indices renumbered)."""
    from lerobot.datasets.aggregate import aggregate_datasets

    base_root = Path(output_root) / base_name / "train"
    v7_root = Path(output_root) / DAGGER_RECIPE / "train"
    merged = Path(output_root) / DAGGER_RECIPE / "train-merged"
    for p in (base_root, v7_root):
        if not (p / "meta" / "info.json").exists():
            raise FileNotFoundError(f"missing dataset at {p}")
    aggregate_datasets([f"local/robollm-vla-bed-{base_name}-train", f"local/robollm-vla-bed-{DAGGER_RECIPE}-train"], f"local/robollm-vla-bed-{DAGGER_RECIPE}m-train",
                       roots=[base_root, v7_root], aggr_root=merged)
    info = json.loads((merged / "meta" / "info.json").read_text())
    print(f"merged → {merged}: {info.get('total_episodes')} episodes, {info.get('total_frames')} frames")
    return merged


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("relabel")
    r.add_argument("--base", required=True, choices=sorted(ds.RECIPES))
    r.add_argument("--run", default="baseline")
    r.add_argument("--checkpoint", required=True)
    r.add_argument("--episodes", type=int, default=400)
    r.add_argument("--base-seed", type=int, default=DAGGER_BASE_SEED)
    r.add_argument("--output-root", type=Path, default=ds.DEFAULT_OUTPUT_ROOT)
    r.add_argument("--device", default=None)
    r.add_argument("--sampling-seed", type=int, default=0)
    m = sub.add_parser("merge")
    m.add_argument("--base", required=True, choices=sorted(ds.RECIPES))
    m.add_argument("--output-root", type=Path, default=ds.DEFAULT_OUTPUT_ROOT)
    args = ap.parse_args()
    if args.cmd == "relabel":
        print(json.dumps(relabel(args.base, args.run, args.checkpoint, args.episodes, args.output_root, args.base_seed, args.device, args.sampling_seed), indent=1))
    else:
        merge(args.base, args.output_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
