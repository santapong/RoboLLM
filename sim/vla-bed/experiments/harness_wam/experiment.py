"""CPU mechanism pilot: behavior cloning, auxiliary dynamics, visibility harness.

This is a feature-based visuomotor policy, not a VLM or a paper reproduction.
Only recorded images and proprioception enter the learner. Targets in the
manifest belong exclusively to the unchanged simulator/evaluator.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import platform
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch
from torch import nn

BED = Path(__file__).resolve().parents[2]
ROOT = BED.parents[1]
sys.path.insert(0, str(BED))
from safety import RPY_STEP_LIMIT_RAD, XYZ_STEP_LIMIT_M

CAMERAS = ("front", "wrist")
VARIANTS = ("bc", "aux_future", "shuffled_future")
ACTION_SCALE = np.array([XYZ_STEP_LIMIT_M] * 3 + [RPY_STEP_LIMIT_RAD] * 3 + [1.0], dtype=np.float32)
FEATURE_NAMES = [f"proprio_{i}" for i in range(14)] + [f"{c}_{f}" for c in CAMERAS for f in ("visible", "u", "v", "area_sqrt")]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def color_features(image: np.ndarray) -> np.ndarray:
    """Fixed RGB detector: visible, normalized centroid (u, v), sqrt(area).

    Identical extraction on recorded 224x224 RGB videos and live RGB renders.
    Absence is explicit; a missing target never becomes a valid centre pixel.
    """
    a = np.asarray(image)
    if a.ndim != 3 or a.shape[2] != 3 or not np.isfinite(a).all():
        raise ValueError("expected finite HxWx3 RGB image")
    r, g, b = np.moveaxis(a.astype(np.float32), -1, 0)
    mask = (r > 80) & (r > 1.4 * g + 20) & (r > 1.4 * b + 20)
    y, x = np.nonzero(mask)
    if not len(x):
        return np.zeros(4, dtype=np.float32)
    return np.array([1, 2 * x.mean() / max(1, a.shape[1] - 1) - 1,
                     2 * y.mean() / max(1, a.shape[0] - 1) - 1,
                     np.sqrt(mask.mean())], dtype=np.float32)


def observation_features(observation: dict) -> np.ndarray:
    state = np.asarray(observation["observation.state"], dtype=np.float32)
    if state.shape != (14,) or not np.isfinite(state).all():
        raise ValueError("expected 14 finite proprioceptive values")
    return np.concatenate([state] + [color_features(observation[f"observation.images.{c}"]) for c in CAMERAS])


def transition_indices(episodes: np.ndarray, frames: np.ndarray) -> np.ndarray:
    """Never pair across resets or bridge a missing recorded frame."""
    if len(episodes) != len(frames):
        raise ValueError("episode/frame lengths differ")
    same = episodes[:-1] == episodes[1:]
    if np.any(same & (frames[1:] != frames[:-1] + 1)):
        raise ValueError("nonconsecutive frames inside an episode")
    return np.flatnonzero(same)


@dataclass
class Transitions:
    x: np.ndarray
    clean: np.ndarray
    executed: np.ndarray
    delta: np.ndarray
    episode: np.ndarray

    def subset(self, mask: np.ndarray) -> Transitions:
        return Transitions(*(getattr(self, k)[mask] for k in self.__dataclass_fields__))


def load_split(dataset: Path, split: str, manifest: dict) -> tuple[Transitions, dict]:
    import av
    import pyarrow.parquet as pq

    entry = manifest["splits"][split]
    tables = sorted((dataset / split / "data").glob("chunk-*/*.parquet"))
    # v6 has one synchronized video per camera. Refuse to guess concatenation
    # across shards; a future dataset needs an episode-metadata-aware reader.
    if len(tables) != 1:
        raise ValueError("pilot supports exactly one v6 table per split")
    table = pq.read_table(tables[0]).to_pandas()
    expected = np.concatenate([np.full(e["frame_count"], i) for i, e in enumerate(entry["episodes"])])
    if len(table) != entry["frame_count"] or not np.array_equal(table.episode_index, expected):
        raise ValueError("manifest/table episode alignment mismatch")
    expected_frames = np.concatenate([np.arange(e["frame_count"]) for e in entry["episodes"]])
    if not np.array_equal(table.frame_index, expected_frames):
        raise ValueError("manifest/table frame alignment mismatch")
    hashes = {str(tables[0].relative_to(dataset)): sha256(tables[0])}
    visuals = []
    for camera in CAMERAS:
        videos = sorted((dataset / split / "videos" / f"observation.images.{camera}").glob("chunk-*/*.mp4"))
        if len(videos) != 1:
            raise ValueError("pilot requires one synchronized video per camera")
        with av.open(str(videos[0])) as container:
            stream = container.streams.video[0]
            stream.thread_count = 1
            features = []
            for i, frame in enumerate(container.decode(video=0)):
                if frame.width != 224 or frame.height != 224:
                    raise ValueError("expected v6 camera size 224x224")
                if abs(float(frame.time) - i / manifest["fps"]) > 0.5 / manifest["fps"]:
                    raise ValueError("video timestamps are not aligned to the table")
                features.append(color_features(frame.to_ndarray(format="rgb24")))
        if len(features) != len(table):
            raise ValueError("video/table frame count mismatch")
        visuals.append(np.stack(features))
        hashes[str(videos[0].relative_to(dataset))] = sha256(videos[0])
    x = np.concatenate([np.stack(table["observation.state"]), *visuals], axis=1).astype(np.float32)
    clean = np.stack(table["action"]).astype(np.float32)
    executed = np.stack(table["action.executed"]).astype(np.float32)
    if not all(np.isfinite(v).all() for v in (x, clean, executed)):
        raise ValueError("nonfinite recorded data")
    if np.any(np.abs(clean) > ACTION_SCALE + 1e-7) or np.any(np.abs(executed) > ACTION_SCALE + 1e-7):
        raise ValueError("recorded commands exceed the existing action limits")
    ix = transition_indices(table.episode_index.to_numpy(), table.frame_index.to_numpy())
    return Transitions(x[ix], clean[ix], executed[ix], x[ix + 1] - x[ix], expected[ix]), hashes


@dataclass
class Normalizer:
    mean: np.ndarray
    std: np.ndarray
    delta_std: np.ndarray

    @classmethod
    def fit(cls, training: Transitions) -> Normalizer:
        return cls(training.x.mean(0), np.maximum(training.x.std(0), 0.01),
                   np.maximum(training.delta.std(0), 0.001))

    def transform(self, x: np.ndarray) -> np.ndarray:
        return (x - self.mean) / self.std


class TinyPolicy(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.encoder = nn.Sequential(nn.Linear(22, 128), nn.SiLU(), nn.Linear(128, 128), nn.SiLU())
        self.action = nn.Sequential(nn.Linear(128, 7), nn.Tanh())
        self.future = nn.Sequential(nn.Linear(128 + 7, 128), nn.SiLU(), nn.Linear(128, 22))

    def forward(self, x: torch.Tensor, executed: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        latent = self.encoder(x)
        return self.action(latent), self.future(torch.cat([latent, executed], dim=-1))


def fit_model(data: Transitions, norm: Normalizer, variant: str, seed: int, steps: int,
              batch_size: int, auxiliary_weight: float) -> tuple[TinyPolicy, dict]:
    torch.manual_seed(seed)
    model = TinyPolicy()
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.001, weight_decay=0.0001)
    x = torch.from_numpy(norm.transform(data.x))
    clean = torch.from_numpy(data.clean / ACTION_SCALE)
    executed = torch.from_numpy(data.executed / ACTION_SCALE)
    future = torch.from_numpy(data.delta / norm.delta_std)
    if variant == "shuffled_future":
        # Shuffle targets once; identical marginal target distribution, destroyed
        # input/next-state association. It is a negative control, not world data.
        permutation = np.random.default_rng(seed + 901).permutation(len(future))
        future = future[permutation]
    rng = np.random.default_rng(seed)  # same minibatches for all three arms
    losses = []
    start = time.perf_counter()
    for step in range(steps):
        ix = rng.integers(0, len(x), size=batch_size)
        predicted_action, predicted_future = model(x[ix], executed[ix])
        action_loss = nn.functional.mse_loss(predicted_action, clean[ix])
        future_loss = nn.functional.mse_loss(predicted_future, future[ix])
        loss = action_loss + (auxiliary_weight if variant != "bc" else 0.0) * future_loss
        if not torch.isfinite(loss):
            raise ValueError("nonfinite training loss")
        optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        if step == 0 or (step + 1) % 250 == 0 or step + 1 == steps:
            losses.append({"step": step + 1, "action_mse": action_loss.item(), "future_mse": future_loss.item()})
    model.eval()
    return model, {"seed": seed, "variant": variant, "steps": steps, "batch_size": batch_size,
                   "samples_seen": steps * batch_size, "wall_s": time.perf_counter() - start,
                   "parameters": sum(p.numel() for p in model.parameters()), "losses": losses}


def offline_metrics(model: TinyPolicy, data: Transitions, norm: Normalizer, world_trained: bool) -> dict:
    with torch.no_grad():
        action, future = model(torch.from_numpy(norm.transform(data.x)), torch.from_numpy(data.executed / ACTION_SCALE))
    errors = {
        "action_xyz_mae_mm": np.abs(action.numpy()[:, :3] * ACTION_SCALE[:3] - data.clean[:, :3]).mean(1) * 1000,
        "action_rotation_mae_rad": np.abs(action.numpy()[:, 3:6] * ACTION_SCALE[3:6] - data.clean[:, 3:6]).mean(1),
        "persistence_xyz_mae_mm": np.abs(data.delta[:, :3]).mean(1) * 1000,
    }
    if world_trained:
        errors["future_xyz_mae_mm"] = np.abs(future.numpy()[:, :3] * norm.delta_std[:3] - data.delta[:, :3]).mean(1) * 1000
    rows = [{"episode_index": int(e), **{k: float(v[data.episode == e].mean()) for k, v in errors.items()}}
            for e in np.unique(data.episode)]
    return {"episodes": len(rows), "transitions": len(data.x), "per_episode": rows,
            "episode_mean": {k: float(np.mean([r[k] for r in rows])) for k in errors}}


class LearnedAdapter:
    """Evaluator adapter with observation-only action inference and optional hold.

    The environment/spec are deliberately ignored. The harness cannot see the
    simulator target, success, reward, privileged error or future observations.
    """
    def __init__(self, model: TinyPolicy, norm: Normalizer, harness: bool, blank: bool = False):
        self.model, self.norm, self.harness, self.blank = model, norm, harness, blank
        self.holds = 0

    def reset(self, spec) -> None:
        self.holds = 0

    def act(self, env, observation: dict) -> np.ndarray:
        features = observation_features(observation)
        if self.blank:
            features[14:] = 0
        if self.harness and features[14] == 0 and features[18] == 0:
            self.holds += 1
            return np.zeros((1, 7), dtype=np.float32)
        with torch.no_grad():
            latent = self.model.encoder(torch.from_numpy(self.norm.transform(features)[None]))
            action = self.model.action(latent).numpy() * ACTION_SCALE
        return action


def paired_success(a: list[dict], b: list[dict]) -> dict:
    from stats import bootstrap_mean

    left, right = {r["seed"]: r for r in a}, {r["seed"]: r for r in b}
    if len(left) != len(a) or len(right) != len(b) or set(left) != set(right):
        raise ValueError("paired comparison requires equal unique seed sets")
    delta = [int(right[s]["success"]) - int(left[s]["success"]) for s in sorted(left)]
    return {"pairs": len(delta), "success_delta": float(np.mean(delta)),
            "improved": delta.count(1), "regressed": delta.count(-1),
            "ci95_episode_bootstrap": list(bootstrap_mean(delta))}


def closed_loop(models: dict, norm: Normalizer, manifest: dict, count: int,
                output: Path, variation: str) -> dict:
    from env import MAX_FRAMES, BedEnv
    from evaluate import HoldPolicy, OraclePolicy, aggregate, run_policy_episode
    from families import EpisodeSpec

    entries = manifest["splits"]["evaluation"]["episodes"][:count]
    specs = [EpisodeSpec(e["seed"], e["split"], e["family"], e["cell"], tuple(e["target"]), tuple(e["initial_q"])) for e in entries]
    summaries, episode_rows = {}, {}
    env = BedEnv(render=True, wrist_camera=True)
    start = time.perf_counter()
    try:
        arms = {"oracle": OraclePolicy(env), "hold": HoldPolicy()}
        for variant, model in models.items():
            arms[variant] = LearnedAdapter(model, norm, harness=False)
            arms[variant + "+visibility"] = LearnedAdapter(model, norm, harness=True)
        arms["bc+blank"] = LearnedAdapter(models["bc"], norm, harness=False, blank=True)
        for label, policy in arms.items():
            t0 = time.perf_counter()
            rows = []
            for spec in specs:
                row = run_policy_episode(env, policy, spec, variation, n_action_steps=1, gain=1.0, max_frames=MAX_FRAMES)
                row["harness_holds"] = getattr(policy, "holds", 0)
                rows.append(row)
            episode_rows[label] = rows
            summaries[label] = {**aggregate(rows), "joint_success_and_safe": sum(r["success"] and r["safe"] for r in rows),
                                "harness_holds": sum(r["harness_holds"] for r in rows), "wall_s": time.perf_counter() - t0}
            write_json(output / (label.replace("+", "_") + ".json"), {"label": label, "aggregate": summaries[label], "episodes": rows})
            print(f"rollout {label}: success {sum(r['success'] for r in rows)}/{len(rows)}, safe {sum(r['safe'] for r in rows)}/{len(rows)}", flush=True)
    finally:
        env.close()
    comparisons = {f"bc_vs_{v}": paired_success(episode_rows["bc"], episode_rows[v]) for v in VARIANTS[1:]}
    comparisons.update({f"{v}_vs_visibility": paired_success(episode_rows[v], episode_rows[v + "+visibility"]) for v in VARIANTS})
    return {"status": "complete", "evidence_scope": "development pilot; one trained initialization per arm",
            "seeds": [s.seed for s in specs], "variation": variation, "max_frames": MAX_FRAMES,
            "controls_pass": all(r["success"] and r["safe"] for r in episode_rows["oracle"]) and not any(r["success"] for r in episode_rows["hold"]),
            "aggregate": summaries, "paired": comparisons, "wall_s": time.perf_counter() - start}


def run(args) -> dict:
    from scene.build_scene import MENAGERIE_COMMIT, menagerie_dir

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True)
    dataset = args.dataset.resolve()
    manifest_path = dataset / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest["recipe"] != "v6" or manifest["fps"] != 20:
        raise ValueError("pilot requires the v6 20 Hz front+wrist dataset")
    train_entries = manifest["splits"]["train"]["episodes"]
    eval_entries = manifest["splits"]["evaluation"]["episodes"]
    train_seeds = [e["seed"] for e in train_entries]
    eval_seeds = [e["seed"] for e in eval_entries]
    if len(set(train_seeds)) != len(train_seeds) or len(set(eval_seeds)) != len(eval_seeds) or set(train_seeds) & set(eval_seeds):
        raise ValueError("duplicate/overlapping train and evaluation seeds")
    if len(train_entries) != 400 or len(eval_entries) != 100:
        raise ValueError("expected 400 training and 100 evaluation episodes")
    if args.steps < 1 or args.batch_size < 1 or args.aux_weight <= 0 or not np.isfinite(args.aux_weight):
        raise ValueError("steps, batch and auxiliary weight must be positive")
    if len(args.train_seeds) != len(set(args.train_seeds)) or args.rollout_seed not in args.train_seeds:
        raise ValueError("unique training seeds required; rollout seed must be trained")
    if args.rollout_episodes not in (0, 20, 40, 60, 80, 100):
        raise ValueError("rollout count must be zero or complete 20-cell blocks")
    asset_commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=menagerie_dir(), text=True).strip()
    if asset_commit != MENAGERIE_COMMIT:
        raise ValueError("Menagerie asset commit differs from the bed's pin")
    args.out.mkdir(parents=True, exist_ok=False)  # never overwrite a completed or partial run
    start = time.perf_counter()
    record = {"schema": "robollm.harness-wam.cpu-pilot.v1", "status": "started",
              "created_at": datetime.now(timezone.utc).isoformat(),
              "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "source_dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip()),
              "source_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in [Path(__file__), BED / "evaluate.py", BED / "env.py", BED / "safety.py", BED / "families.py", BED / "scene/build_scene.py", BED / "expert.py", BED / "stats.py"]},
              "menagerie_commit": asset_commit,
              "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
              "environment": {"python": platform.python_version(), "machine": platform.machine(), "device": "cpu", "torch_threads": 1, "MUJOCO_GL": os.environ.get("MUJOCO_GL", ""),
                              "packages": {k: importlib.metadata.version(k) for k in ("numpy", "torch", "av", "pyarrow", "mujoco", "mink")}},
              "feature_names": FEATURE_NAMES, "manifest_sha256": sha256(manifest_path),
              "data_roles": {"fit_seeds": train_seeds[:320], "calibration_seeds": train_seeds[320:],
                             "development_seeds": eval_seeds, "final_test": "not run; reserve a fresh suite before further selection"},
              "claim_ceiling": "handcrafted RGB features + MLP on one simulated reaching task; no language model, video generator, RL, real hardware or SmolVLA fine-tuning"}
    write_json(args.out / "record.json", record)
    print("decoding v6 RGB streams and checking temporal alignment", flush=True)
    train, hashes_train = load_split(dataset, "train", manifest)
    development, hashes_eval = load_split(dataset, "evaluation", manifest)
    fit, calibration = train.subset(train.episode < 320), train.subset(train.episode >= 320)
    norm = Normalizer.fit(fit)
    np.savez_compressed(args.out / "normalizer.npz", mean=norm.mean, std=norm.std, delta_std=norm.delta_std)
    record.update({"input_sha256": {**hashes_train, **hashes_eval}, "data_counts": {"fit_transitions": len(fit.x), "calibration_transitions": len(calibration.x), "development_transitions": len(development.x)},
                   "normalizer_sha256": sha256(args.out / "normalizer.npz"), "offline": []})
    rollout_models = {}
    for seed in args.train_seeds:
        for variant in VARIANTS:
            model, info = fit_model(fit, norm, variant, seed, args.steps, args.batch_size, args.aux_weight)
            checkpoint = args.out / f"{variant}_{seed}.pt"
            torch.save(model.state_dict(), checkpoint)
            info.update({"checkpoint": checkpoint.name, "checkpoint_sha256": sha256(checkpoint),
                         "calibration": offline_metrics(model, calibration, norm, variant != "bc"),
                         "development": offline_metrics(model, development, norm, variant != "bc")})
            record["offline"].append(info)
            write_json(args.out / "record.json", record)
            if seed == args.rollout_seed:
                rollout_models[variant] = model
            print(f"trained {variant} seed={seed}: {info['development']['episode_mean']}", flush=True)
    record["closed_loop"] = {"status": "not requested"}
    if args.rollout_episodes:
        record["closed_loop"] = closed_loop(rollout_models, norm, manifest, args.rollout_episodes, args.out, args.variation)
    record.update({"status": "complete", "wall_s": time.perf_counter() - start})
    write_json(args.out / "record.json", record)
    return record


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", type=Path, required=True, help="existing datasets/vla-bed/v6 directory (read only)")
    p.add_argument("--out", type=Path, required=True, help="new run directory; existing paths are refused")
    p.add_argument("--steps", type=int, default=1000)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--aux-weight", type=float, default=0.1)
    p.add_argument("--train-seeds", type=int, nargs="+", default=[7, 17, 27])
    p.add_argument("--rollout-seed", type=int, default=7)
    p.add_argument("--rollout-episodes", type=int, default=20)
    p.add_argument("--variation", choices=("nominal", "camera_shift_far", "lighting"), default="nominal")
    result = run(p.parse_args())
    print(json.dumps({"status": result["status"], "wall_s": result["wall_s"]}))


if __name__ == "__main__":
    main()
