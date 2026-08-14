#!/usr/bin/env python3
"""Generate and validate reproducible LeRobot datasets for B1 preparation."""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import platform
import subprocess
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
from arm_dataset import dataset_features, record_dataset
from reaching import (
    ACTION_SLEW,
    FAMILY_NAMES,
    FPS,
    INSTRUCTION,
    MAX_FRAMES,
    OracleExpert,
    ReachingEnv,
    actuator_bounds,
    episode_specs,
)

MANIFEST_SCHEMA = "robollm.b1.dataset-manifest.v1"


def _dataset_type(dataset_class: Any | None = None):
    if dataset_class is not None:
        return dataset_class
    try:
        from lerobot.datasets import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError(
            "Install requirements/lerobot.txt in .venv-lerobot before recording"
        ) from exc
    return LeRobotDataset


def dependency_versions() -> dict[str, str]:
    versions = {"python": platform.python_version()}
    for package in ("numpy", "mujoco", "lerobot", "torch", "torchcodec"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = "not-installed"
    return versions


def git_state() -> tuple[str, bool]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
        return commit, dirty
    except (OSError, subprocess.CalledProcessError):
        return "unknown", True


def _read_manifest(path: Path) -> dict[str, Any]:
    if not path.exists():
        commit, dirty = git_state()
        return {
            "schema": MANIFEST_SCHEMA,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "instruction": INSTRUCTION,
            "fps": FPS,
            "max_frames_per_episode": MAX_FRAMES,
            "goal_families": list(FAMILY_NAMES),
            "dependencies": dependency_versions(),
            "git_commit": commit,
            "git_dirty": dirty,
            "features": dataset_features(240, 320),
            "splits": {},
        }
    with path.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    if manifest.get("schema") != MANIFEST_SCHEMA:
        raise ValueError(f"unsupported manifest schema in {path}")
    return manifest


def _write_manifest(path: Path, manifest: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    temporary.replace(path)


def record_reaching_split(
    output_root: str | Path,
    manifest_path: str | Path,
    split: str,
    count: int,
    seed: int,
    repo_id: str = "local/robollm-red-target",
    frames: int = MAX_FRAMES,
    height: int = 240,
    width: int = 320,
    dataset_class: Any | None = None,
) -> dict[str, Any]:
    if frames <= 0 or frames > MAX_FRAMES:
        raise ValueError(f"frames must be between 1 and {MAX_FRAMES}")
    specs = episode_specs(count, seed, split)
    root = Path(output_root) / split
    split_repo_id = f"{repo_id}-{split}"
    dataset = _dataset_type(dataset_class).create(
        repo_id=split_repo_id,
        root=root,
        fps=FPS,
        robot_type="robollm_diy_arm_sim",
        features=dataset_features(height, width),
        use_videos=True,
    )
    env = ReachingEnv(height=height, width=width, render=True)
    expert = OracleExpert()
    episode_rows: list[dict[str, Any]] = []
    successes = 0

    try:
        for spec in specs:
            observation = env.reset(spec)
            succeeded = False
            error = env.error_m
            recorded = 0
            for frame in range(frames):
                action = expert.action(env)
                result = env.step(action)
                observation = result.observation
                error = result.error_m
                dataset.add_frame(
                    {
                        **observation,
                        "action": np.asarray(action, dtype=np.float32),
                        "task": INSTRUCTION,
                    }
                )
                recorded = frame + 1
                if result.success:
                    succeeded = True
                    break
            dataset.save_episode()
            successes += int(succeeded)
            episode_rows.append(
                {
                    **spec.to_dict(),
                    "frame_count": recorded,
                    "success": succeeded,
                    "final_error_m": error,
                }
            )
    except Exception:
        dataset.clear_episode_buffer()
        raise
    finally:
        env.close()
        dataset.finalize()

    manifest_file = Path(manifest_path)
    manifest = _read_manifest(manifest_file)
    manifest["features"] = dataset_features(height, width)
    manifest["splits"][split] = {
        "repo_id": split_repo_id,
        "root": str(root),
        "base_seed": seed,
        "episode_count": count,
        "frame_count": sum(row["frame_count"] for row in episode_rows),
        "episodes": episode_rows,
    }
    _write_manifest(manifest_file, manifest)
    return {
        "split": split,
        "episodes": count,
        "frames": sum(row["frame_count"] for row in episode_rows),
        "successes": successes,
        "success_rate": successes / count,
        "family_counts": dict(Counter(spec.family for spec in specs)),
    }


def _to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _open_dataset(dataset_type: Any, repo_id: str, root: Path):
    try:
        return dataset_type(repo_id=repo_id, root=root)
    except TypeError:
        return dataset_type(repo_id, root=root)


def validate_manifest(path: str | Path) -> dict[str, Any]:
    manifest_path = Path(path)
    manifest = _read_manifest(manifest_path)
    errors: list[str] = []
    required_splits = {"train", "evaluation"}
    missing = required_splits - set(manifest["splits"])
    if missing:
        errors.append(f"missing splits: {sorted(missing)}")

    feature_names = set(manifest.get("features", {}))
    if any("target" in name.lower() for name in feature_names):
        errors.append("target coordinates leaked into model features")
    expected = {
        "observation.images.front",
        "observation.state",
        "observation.camera_lag_ms",
        "action",
    }
    if feature_names != expected:
        errors.append(f"unexpected feature set: {sorted(feature_names)}")

    seeds_by_split: dict[str, set[int]] = {}
    targets_by_split: dict[str, set[tuple[float, ...]]] = {}
    total_episodes = 0
    for split, split_data in manifest["splits"].items():
        rows = split_data.get("episodes", [])
        total_episodes += len(rows)
        if split_data.get("episode_count") != len(rows):
            errors.append(f"{split}: episode count does not match rows")
        seeds_by_split[split] = {int(row["seed"]) for row in rows}
        targets_by_split[split] = {tuple(row["target"]) for row in rows}
        for row in rows:
            if (
                len(row.get("initial_state", [])) != 7
                or len(row.get("goal_state", [])) != 7
                or len(row.get("target", [])) != 3
            ):
                errors.append(f"{split}: malformed episode seed {row.get('seed')}")
            values = np.asarray(
                row.get("initial_state", [])
                + row.get("goal_state", [])
                + row.get("target", [])
            )
            if not np.isfinite(values).all():
                errors.append(f"{split}: non-finite manifest values")
            if not (
                1 <= row.get("frame_count", 0) <= manifest["max_frames_per_episode"]
            ):
                errors.append(f"{split}: invalid frame count")
    if seeds_by_split.get("train", set()) & seeds_by_split.get("evaluation", set()):
        errors.append("train/evaluation seed overlap")
    if targets_by_split.get("train", set()) & targets_by_split.get("evaluation", set()):
        errors.append("train/evaluation target overlap")
    return {
        "valid": not errors,
        "errors": errors,
        "episodes": total_episodes,
        "splits": sorted(manifest["splits"]),
    }


def validate_dataset(
    path: str | Path,
    dataset_class: Any | None = None,
    decode_video: bool = True,
) -> dict[str, Any]:
    manifest_path = Path(path)
    manifest = _read_manifest(manifest_path)
    result = validate_manifest(manifest_path)
    errors = list(result["errors"])
    dataset_type = _dataset_type(dataset_class)
    model = ReachingEnv(render=False).model
    low, high = actuator_bounds(model)
    decoded = 0
    checked_frames = 0

    for split, split_data in manifest["splits"].items():
        dataset = _open_dataset(
            dataset_type, split_data["repo_id"], Path(split_data["root"])
        )
        expected_frames = int(split_data["frame_count"])
        if len(dataset) != expected_frames:
            errors.append(
                f"{split}: dataset has {len(dataset)} frames, expected {expected_frames}"
            )
        episode_rows = split_data["episodes"]
        previous_by_episode: dict[int, np.ndarray] = {
            index: np.asarray(row["initial_state"], dtype=np.float64)
            for index, row in enumerate(episode_rows)
        }
        expected_frame_by_episode: dict[int, int] = {
            index: 0 for index in range(len(episode_rows))
        }
        seen_by_episode: Counter[int] = Counter()
        for index in range(len(dataset)):
            frame = dataset[index]
            state = _to_numpy(frame["observation.state"]).reshape(-1)
            action = _to_numpy(frame["action"]).reshape(-1)
            if state.shape != (7,) or action.shape != (7,):
                errors.append(f"{split}: bad vector shape at frame {index}")
                continue
            if not np.isfinite(state).all() or not np.isfinite(action).all():
                errors.append(f"{split}: non-finite vector at frame {index}")
            if np.any(action < low - 1e-6) or np.any(action > high + 1e-6):
                errors.append(f"{split}: action bounds violation at frame {index}")
            episode_index = int(_to_numpy(frame.get("episode_index", 0)).item())
            frame_index = int(_to_numpy(frame.get("frame_index", 0)).item())
            seen_by_episode[episode_index] += 1
            if frame_index != expected_frame_by_episode.get(episode_index, -1):
                errors.append(f"{split}: non-contiguous frame index at frame {index}")
            expected_frame_by_episode[episode_index] = frame_index + 1
            previous = previous_by_episode.get(episode_index)
            if previous is None:
                errors.append(f"{split}: unexpected episode index {episode_index}")
            elif np.any(np.abs(action - previous) > ACTION_SLEW + 1e-6):
                errors.append(f"{split}: action slew violation at frame {index}")
            previous_by_episode[episode_index] = action
            if "timestamp" in frame:
                timestamp = float(_to_numpy(frame["timestamp"]).item())
                expected_timestamp = frame_index / manifest["fps"]
                if abs(timestamp - expected_timestamp) > 1e-4:
                    errors.append(f"{split}: timing violation at frame {index}")
            if decode_video:
                image = _to_numpy(frame["observation.images.front"])
                if image.ndim != 3 or 3 not in (image.shape[0], image.shape[-1]):
                    errors.append(f"{split}: bad decoded image shape {image.shape}")
                elif not np.isfinite(image).all():
                    errors.append(f"{split}: non-finite decoded image at frame {index}")
                decoded += 1
            checked_frames += 1
        for episode_index, row in enumerate(episode_rows):
            if seen_by_episode[episode_index] != row["frame_count"]:
                errors.append(f"{split}: episode {episode_index} frame count mismatch")

    return {
        "valid": not errors,
        "errors": errors,
        "episodes": result["episodes"],
        "frames": checked_frames,
        "decoded_video_frames": decoded,
        "splits": result["splits"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario", choices=("red-target", "joint-wave"), default="red-target"
    )
    parser.add_argument("--seed", type=int, default=10_000)
    parser.add_argument("--split", choices=("train", "evaluation"), default="train")
    parser.add_argument("--episodes", type=int, default=40)
    parser.add_argument("--output-root", default="datasets/b1-red-target")
    parser.add_argument("--manifest", default="datasets/b1-red-target/manifest.json")
    parser.add_argument("--repo-id", default="local/robollm-red-target")
    parser.add_argument("--frames", type=int, default=MAX_FRAMES)
    parser.add_argument("--validate", action="store_true")
    args = parser.parse_args()

    if args.validate:
        metrics = validate_dataset(args.manifest)
    elif args.scenario == "joint-wave":
        metrics = record_dataset(
            root=Path(args.output_root) / args.split,
            repo_id=f"{args.repo_id}-joint-wave-{args.split}",
            task="move every arm joint smoothly",
            episodes=args.episodes,
            frames=args.frames,
        )
    else:
        metrics = record_reaching_split(
            output_root=args.output_root,
            manifest_path=args.manifest,
            split=args.split,
            count=args.episodes,
            seed=args.seed,
            repo_id=args.repo_id,
            frames=args.frames,
        )
    print("RESULT:" + json.dumps(metrics, sort_keys=True))
    return 0 if metrics.get("valid", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
