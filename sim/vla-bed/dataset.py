"""LeRobot v3 recording, validation and summary for the bed (SDD §6.1, §7.2, §8/P2).

Recipes (SDD §7): v1 oracle 40/10 (reproduction anchor); v2 noisy σ = 0.5 × limit
400/100 with one clean episode in five; v2b the same at σ = 0.25 ×; v3 = v2 with the
expert capped at 0.7 × the safety limits (headroom: the v2 labels sat exactly on the S2
cap, audit 4 Sep 2026) and a per-episode camera azimuth jitter of ±20° on the train
split (Cai et al. 2603.26757; the evaluation split — the frozen suite — stays at the
nominal camera). Every frame
stores the clean expert label as ``action`` and the applied action as
``action.executed`` (DART / Zhang et al.), plus the episode's σ.

Each frame pairs the observation the expert acted on with the action it chose
(the LeRobot convention). Note that B1's recorder pairs the action with the
post-step observation instead (`examples/mujoco/reaching_dataset.py`); the two
beds are trained separately, so this does not affect comparability, but it is
stated here so nobody copies frames across.

The validator re-checks the safety contract offline: S1–S4 on both action
vectors using the recorded end-effector position (SDD §6.2), σ constant per
episode, contiguous frame indices and timestamps, decodable video.
"""

from __future__ import annotations

import importlib.metadata
import json
import os
import platform
import subprocess
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np  # noqa: E402

BED_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BED_DIR))
import families  # noqa: E402
from safety import SafetyWrapper  # noqa: E402

MANIFEST_SCHEMA = "robollm.vla-bed.dataset-manifest.v1"
ROBOT_TYPE = "ur5e_2f85_sim"
DEFAULT_OUTPUT_ROOT = BED_DIR.parents[1] / "datasets" / "vla-bed"
FEATURE_KEYS = (
    "observation.images.front",
    "observation.state",
    "action",
    "action.executed",
    "observation.noise_sigma",
    "observation.camera_lag_ms",
)


@dataclass(frozen=True)
class Recipe:
    name: str
    expert: str
    noise_fraction: float
    train: int
    evaluation: int
    base_seed: int
    clean_every: int | None  # every k-th episode is recorded clean (σ = 0)
    headroom: float = 1.0  # clean-label cap as a fraction of the safety limits (S2/S3)
    camera_jitter_deg: float = 0.0  # train-split camera azimuth ~ U(−j, +j) about the look-at point, seeded per episode

    def episodes(self, split: str) -> int:
        return self.train if split == "train" else self.evaluation

    def sigma(self, index: int) -> float:
        if self.expert == "oracle":
            return 0.0
        if self.clean_every and index % self.clean_every == 0:
            return 0.0
        return self.noise_fraction


RECIPES: dict[str, Recipe] = {
    "v1": Recipe("v1", "oracle", 0.0, 40, 10, 10_000, None),
    "v2": Recipe("v2", "noisy", 0.5, 400, 100, 10_000, 5),
    "v2b": Recipe("v2b", "noisy", 0.25, 400, 100, 10_000, 5),
    "v3": Recipe("v3", "noisy", 0.5, 400, 100, 10_000, 5, headroom=0.7, camera_jitter_deg=20.0),
}
CAMERA_JITTER_SEED_OFFSET = 777_777


def camera_azimuth(recipe: Recipe, spec, split: str) -> float:
    """Per-episode camera azimuth for recording: 0 unless the recipe jitters the train split."""
    if split != "train" or recipe.camera_jitter_deg <= 0:
        return 0.0
    rng = np.random.default_rng(int(spec.seed) + CAMERA_JITTER_SEED_OFFSET)
    return float(rng.uniform(-recipe.camera_jitter_deg, recipe.camera_jitter_deg))


class RecordingFault(RuntimeError):
    """The expert tripped the safety contract while recording; the dataset is not trustworthy."""


def _dataset_type(dataset_class: Any | None = None):
    if dataset_class is not None:
        return dataset_class
    try:
        from lerobot.datasets import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError("Install sim/vla-bed/requirements-record.txt into .venv-lerobot before recording") from exc
    return LeRobotDataset


def dependency_versions() -> dict[str, str]:
    versions = {"python": platform.python_version()}
    for package in ("numpy", "mujoco", "mink", "lerobot", "torch", "torchcodec", "transformers"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = "not-installed"
    return versions


def git_state() -> tuple[str, bool]:
    try:
        commit = subprocess.run(["git", "rev-parse", "HEAD"], check=True, capture_output=True, text=True, cwd=BED_DIR).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain"], check=True, capture_output=True, text=True, cwd=BED_DIR).stdout.strip())
        return commit, dirty
    except (OSError, subprocess.CalledProcessError):
        return "unknown", True


def _read_manifest(path: Path, recipe: Recipe) -> dict:
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    from env import FPS, INSTRUCTION, MAX_FRAMES  # noqa: WPS433

    commit, dirty = git_state()
    return {
        "schema": MANIFEST_SCHEMA,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "recipe": recipe.name,
        "expert": recipe.expert,
        "noise_fraction": recipe.noise_fraction,
        "clean_every": recipe.clean_every,
        "headroom": recipe.headroom,
        "camera_jitter_deg": recipe.camera_jitter_deg,
        "instruction": INSTRUCTION,
        "fps": FPS,
        "max_frames_per_episode": MAX_FRAMES,
        "goal_families": list(families.FAMILY_NAMES),
        "cells": len(families.cells()),
        "dependencies": dependency_versions(),
        "git_commit": commit,
        "git_dirty": dirty,
        "features": {},
        "splits": {},
    }


def _write_manifest(path: Path, manifest: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def record_split(
    recipe_name: str,
    split: str,
    output_root: str | Path = DEFAULT_OUTPUT_ROOT,
    manifest_path: str | Path | None = None,
    episodes: int | None = None,
    repo_id: str | None = None,
    height: int = 224,
    width: int = 224,
    dataset_class: Any | None = None,
    env: Any | None = None,
) -> dict[str, Any]:
    from env import FPS, INSTRUCTION, MAX_FRAMES, BedEnv, dataset_features  # noqa: WPS433
    from expert import make_expert  # noqa: WPS433

    recipe = RECIPES[recipe_name]
    if split not in ("train", "evaluation"):
        raise ValueError("split must be train or evaluation")
    count = episodes if episodes is not None else recipe.episodes(split)
    repo_id = repo_id or f"local/robollm-vla-bed-{recipe.name}"
    root = Path(output_root) / recipe.name / split
    manifest_file = Path(manifest_path) if manifest_path else Path(output_root) / recipe.name / "manifest.json"
    specs = families.episode_specs(count, recipe.base_seed, split)
    frozen = {(c["family"], c["cell"]) for c in families.load_frozen()["verified"]}
    missing = sorted({(s.family, s.cell) for s in specs} - frozen)
    if missing:
        raise RecordingFault(f"cells not IK-verified: {missing}")

    features = dataset_features(height, width)
    dataset = _dataset_type(dataset_class).create(
        repo_id=f"{repo_id}-{split}", root=root, fps=FPS, robot_type=ROBOT_TYPE, features=features, use_videos=True
    )
    own_env = env is None
    env = env or BedEnv(render=True, height=height, width=width)
    wrapper = SafetyWrapper()
    experts: dict[float, Any] = {}
    rows: list[dict[str, Any]] = []
    successes = 0
    t0 = time.perf_counter()
    try:
        for index, spec in enumerate(specs):
            sigma = recipe.sigma(index)
            expert = experts.get(sigma)
            if expert is None:
                expert = experts[sigma] = make_expert(recipe.expert, env.controller.home_rot, sigma, recipe.headroom)
            azimuth = camera_azimuth(recipe, spec, split)
            observation = env.reset(spec, camera_azimuth_deg=azimuth)
            expert.reset(spec)
            succeeded = False
            recorded = 0
            for _ in range(MAX_FRAMES):
                cmd_pos, cmd_rot = env.commanded_ee
                out = expert.act(cmd_pos, cmd_rot, env.target)
                if not wrapper.check(out.clean, cmd_pos).ok:
                    raise RecordingFault(f"clean label failed S1–S4 at seed {spec.seed} frame {recorded}")
                result = env.step(out.executed)
                if not result.decision.ok:
                    raise RecordingFault(f"executed action rejected ({result.decision.code}) at seed {spec.seed} frame {recorded}")
                dataset.add_frame(
                    {
                        **observation,
                        "action": np.asarray(out.clean, dtype=np.float32),
                        "action.executed": np.asarray(out.executed, dtype=np.float32),
                        "observation.noise_sigma": np.asarray([sigma], dtype=np.float32),
                        "task": INSTRUCTION,
                    }
                )
                observation = result.observation
                recorded += 1
                if result.success:
                    succeeded = True
                    break
            if not env.safety.safe:
                raise RecordingFault(f"unsafe expert episode at seed {spec.seed}: {env.safety.rejections} {env.safety.measured}")
            dataset.save_episode()
            successes += int(succeeded)
            rows.append(
                {
                    **spec.to_dict(),
                    "frame_count": recorded,
                    "success": succeeded,
                    "progress": 1.0 if succeeded else (0.5 if env.min_error_m <= 0.06 else 0.0),
                    "final_error_m": round(env.error_m, 5),
                    "noise_sigma": sigma,
                    "safe": env.safety.safe,
                    "camera_azimuth_deg": round(azimuth, 4),
                }
            )
    except Exception:
        dataset.clear_episode_buffer()
        raise
    finally:
        if own_env:
            env.close()
        dataset.finalize()

    manifest = _read_manifest(manifest_file, recipe)
    manifest["features"] = {k: {**v, "shape": list(v["shape"])} for k, v in features.items()}
    manifest["splits"][split] = {
        "repo_id": f"{repo_id}-{split}",
        "root": str(root),
        "base_seed": recipe.base_seed,
        "expert": recipe.expert,
        "noise_fraction": recipe.noise_fraction,
        "clean_every": recipe.clean_every,
        "headroom": recipe.headroom,
        "camera_jitter_deg": recipe.camera_jitter_deg if split == "train" else 0.0,
        "episode_count": count,
        "frame_count": sum(r["frame_count"] for r in rows),
        "success_count": successes,
        "wall_s": round(time.perf_counter() - t0, 1),
        "episodes": rows,
    }
    _write_manifest(manifest_file, manifest)
    return {
        "recipe": recipe.name,
        "split": split,
        "episodes": count,
        "frames": manifest["splits"][split]["frame_count"],
        "successes": successes,
        "success_rate": successes / count,
        "family_counts": dict(Counter(s.family for s in specs)),
        "manifest": str(manifest_file),
    }


# ----------------------------------------------------------------------------- validation


def _to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def validate_manifest(path: str | Path) -> dict:
    manifest = json.loads(Path(path).read_text(encoding="utf-8"))
    errors: list[str] = []
    if manifest.get("schema") != MANIFEST_SCHEMA:
        errors.append("bad schema")
    recipe = RECIPES.get(manifest.get("recipe", ""))
    if recipe is None:
        errors.append("unknown recipe")
    if set(manifest.get("features", {})) != set(FEATURE_KEYS):
        errors.append(f"feature set mismatch: {sorted(manifest.get('features', {}))}")
    if any("target" in k for k in manifest.get("features", {})):
        errors.append("target leaked into features")
    seeds: dict[str, set[int]] = {}
    for split, entry in manifest.get("splits", {}).items():
        rows = entry.get("episodes", [])
        seeds[split] = {r["seed"] for r in rows}
        if entry.get("episode_count") != len(rows):
            errors.append(f"{split}: episode_count != rows")
        if entry.get("frame_count") != sum(r["frame_count"] for r in rows):
            errors.append(f"{split}: frame_count != sum of rows")
        for i, r in enumerate(rows):
            if not (1 <= r["frame_count"] <= manifest.get("max_frames_per_episode", 100)):
                errors.append(f"{split}: episode {i} frame_count out of range")
            if recipe is not None and abs(r["noise_sigma"] - recipe.sigma(i)) > 1e-9:
                errors.append(f"{split}: episode {i} sigma {r['noise_sigma']} != schedule {recipe.sigma(i)}")
            if not r.get("safe", False):
                errors.append(f"{split}: episode {i} recorded unsafe")
            if not all(np.isfinite(v) for v in r["target"]):
                errors.append(f"{split}: episode {i} non-finite target")
    if "train" in seeds and "evaluation" in seeds and seeds["train"] & seeds["evaluation"]:
        errors.append("train/evaluation seeds overlap")
    return {"valid": not errors, "errors": errors, "manifest": manifest}


def _default_loader(repo_id: str, root: str):
    from lerobot.datasets import LeRobotDataset  # noqa: WPS433

    return LeRobotDataset(repo_id, root=root)


def validate_dataset(manifest_path: str | Path, decode_video: bool = True, dataset_loader=None, max_frames: int | None = None) -> dict:
    report = validate_manifest(manifest_path)
    manifest = report["manifest"]
    errors = list(report["errors"])
    loader = dataset_loader or _default_loader
    wrapper = SafetyWrapper()
    total = 0
    decoded = 0
    label_faults = 0
    executed_faults = 0
    per_split: dict[str, dict] = {}
    for split, entry in manifest.get("splits", {}).items():
        dataset = loader(entry["repo_id"], entry["root"])
        n = len(dataset)
        if n != entry["frame_count"]:
            errors.append(f"{split}: dataset has {n} frames, manifest says {entry['frame_count']}")
        expected_next: dict[int, int] = {}
        counts: dict[int, int] = {}
        sigma_by_episode: dict[int, float] = {}
        limit = n if max_frames is None else min(n, max_frames)
        for i in range(limit):
            frame = dataset[i]
            state = _to_numpy(frame["observation.state"]).reshape(-1)
            action = _to_numpy(frame["action"]).reshape(-1)
            executed = _to_numpy(frame["action.executed"]).reshape(-1)
            sigma = _to_numpy(frame["observation.noise_sigma"]).reshape(-1)
            if state.shape != (14,) or action.shape != (7,) or executed.shape != (7,) or sigma.shape != (1,):
                errors.append(f"{split}: bad vector shape at frame {i}")
                continue
            if not (np.all(np.isfinite(state)) and np.all(np.isfinite(action)) and np.all(np.isfinite(executed))):
                errors.append(f"{split}: non-finite value at frame {i}")
            ee = state[:3]
            if not wrapper.check(action, ee).ok:
                label_faults += 1
            if not wrapper.check(executed, ee).ok:
                executed_faults += 1
            episode = int(_to_numpy(frame.get("episode_index", 0)).item())
            frame_index = int(_to_numpy(frame.get("frame_index", 0)).item())
            if frame_index != expected_next.get(episode, 0):
                errors.append(f"{split}: non-contiguous frame_index at frame {i}")
            expected_next[episode] = frame_index + 1
            counts[episode] = counts.get(episode, 0) + 1
            prev_sigma = sigma_by_episode.setdefault(episode, float(sigma[0]))
            if abs(prev_sigma - float(sigma[0])) > 1e-9:
                errors.append(f"{split}: sigma changed inside episode {episode}")
            if "timestamp" in frame:
                ts = float(_to_numpy(frame["timestamp"]).item())
                if abs(ts - frame_index / manifest["fps"]) > 1e-4:
                    errors.append(f"{split}: timestamp mismatch at frame {i}")
            if decode_video:
                image = _to_numpy(frame["observation.images.front"])
                if image.ndim != 3 or 3 not in (image.shape[0], image.shape[-1]):
                    errors.append(f"{split}: bad decoded image shape {image.shape}")
                elif not np.all(np.isfinite(image)):
                    errors.append(f"{split}: non-finite decoded image at frame {i}")
                decoded += 1
            total += 1
        if max_frames is None:
            for ep_index, row in enumerate(entry["episodes"]):
                if counts.get(ep_index, 0) != row["frame_count"]:
                    errors.append(f"{split}: episode {ep_index} has {counts.get(ep_index, 0)} frames, manifest says {row['frame_count']}")
        per_split[split] = {"frames_checked": limit, "episodes": len(entry["episodes"])}
    if label_faults:
        errors.append(f"{label_faults} clean labels fail S1–S4")
    if executed_faults:
        errors.append(f"{executed_faults} executed actions fail S1–S4")
    return {
        "valid": not errors,
        "errors": errors,
        "frames_checked": total,
        "decoded_video_frames": decoded,
        "clean_label_faults": label_faults,
        "executed_action_faults": executed_faults,
        "splits": per_split,
    }


# ----------------------------------------------------------------------------- summary

DISTANCE_BINS_M = [0.0, 0.03, 0.06, 0.10, 0.20, 0.40, 1.00]


def chunk_padding_percent(lengths: list[int], chunk: int) -> float:
    """Share of chunk slots that are padding when each episode is cut into chunks of `chunk`."""
    slots = sum(int(np.ceil(n / chunk)) * chunk for n in lengths)
    frames = sum(lengths)
    return 100.0 * (slots - frames) / slots if slots else 0.0


def summarize(manifest_path: str | Path, dataset_loader=None) -> dict:
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    loader = dataset_loader or _default_loader
    out: dict[str, Any] = {"recipe": manifest["recipe"], "splits": {}}
    for split, entry in manifest["splits"].items():
        rows = entry["episodes"]
        lengths = [r["frame_count"] for r in rows]
        by_sigma: dict[str, dict] = {}
        for sigma in sorted({r["noise_sigma"] for r in rows}):
            sub = [r for r in rows if r["noise_sigma"] == sigma]
            by_sigma[str(sigma)] = {
                "episodes": len(sub),
                "frames": sum(r["frame_count"] for r in sub),
                "success_rate": sum(r["success"] for r in sub) / len(sub),
                "episode_len_mean": float(np.mean([r["frame_count"] for r in sub])),
            }
        # distance-to-target coverage from the recorded EE position and the manifest target
        dataset = loader(entry["repo_id"], entry["root"])
        distances: dict[str, list[float]] = {k: [] for k in by_sigma}
        offsets = np.cumsum([0] + lengths[:-1])
        for ep_index, row in enumerate(rows):
            target = np.asarray(row["target"])
            for j in range(row["frame_count"]):
                frame = dataset[int(offsets[ep_index]) + j]
                ee = _to_numpy(frame["observation.state"]).reshape(-1)[:3]
                distances[str(row["noise_sigma"])].append(float(np.linalg.norm(ee - target)))
        for k, d in distances.items():
            hist, _ = np.histogram(d, bins=DISTANCE_BINS_M)
            by_sigma[k]["distance_to_target_hist"] = {"bins_m": DISTANCE_BINS_M, "counts": hist.tolist()}
            by_sigma[k]["distance_to_target_mean_m"] = float(np.mean(d)) if d else 0.0
        out["splits"][split] = {
            "episodes": len(rows),
            "frames": sum(lengths),
            "success_rate": sum(r["success"] for r in rows) / len(rows),
            "episode_len_mean": float(np.mean(lengths)),
            "episode_len_max": int(max(lengths)),
            "chunk_padding_percent": {"20": round(chunk_padding_percent(lengths, 20), 1), "50": round(chunk_padding_percent(lengths, 50), 1)},
            "by_sigma": by_sigma,
            "wall_s": entry.get("wall_s"),
        }
    return out
