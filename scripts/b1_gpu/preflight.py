#!/usr/bin/env python3
"""Fail-closed preflight for the deferred B1 GPU fine-tune."""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import shutil
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "mujoco"))

EXPECTED_SCHEMA = "robollm.b1.smolvla-training.v1"
REQUIRED_CHECKPOINTS = [5000, 10000, 20000]


def load_config(path: Path) -> dict[str, Any]:
    config = json.loads(path.read_text(encoding="utf-8"))
    expected = {
        "schema": EXPECTED_SCHEMA,
        "model": "lerobot/smolvla_base",
        "dataset_repo_id": "local/robollm-red-target-train",
        "dataset_root": "datasets/b1-red-target/train",
        "manifest": "datasets/b1-red-target/manifest.json",
        "output_dir": "artifacts/b1-smolvla",
        "steps": 20_000,
        "checkpoint_steps": REQUIRED_CHECKPOINTS,
        "device": "cuda",
        "wandb": False,
        "push_to_hub": False,
        "save_checkpoint_to_hub": False,
        "local_artifacts_only": True,
        "fps": 20,
        "action_shape": [7],
        "instruction": "touch the red target",
    }
    mismatches = {
        key: {"expected": value, "actual": config.get(key)}
        for key, value in expected.items()
        if config.get(key) != value
    }
    if config.get("save_freq") != 5_000:
        mismatches["save_freq"] = {"expected": 5_000, "actual": config.get("save_freq")}
    if mismatches:
        raise ValueError(f"training configuration mismatch: {mismatches}")
    return config


def check_environment(
    config: dict[str, Any],
    execute: bool,
    min_disk_gb: float,
    output_override: Path | None = None,
) -> dict[str, Any]:
    output_dir = output_override or ROOT / config["output_dir"]
    disk_gb = (
        shutil.disk_usage(
            output_dir.parent if output_dir.parent.exists() else ROOT
        ).free
        / 1e9
    )
    errors: list[str] = []
    if disk_gb < min_disk_gb:
        errors.append(f"only {disk_gb:.1f} GB free; require {min_disk_gb:.1f} GB")
    if output_dir.exists() and any(output_dir.iterdir()):
        errors.append(f"output path is not empty: {output_dir}")

    cuda = False
    gpu_name = None
    torch_version = "not-installed"
    try:
        import torch

        torch_version = torch.__version__
        cuda = torch.cuda.is_available()
        if cuda:
            gpu_name = torch.cuda.get_device_name(0)
    except ImportError:
        if execute:
            errors.append("torch is not installed")
    if execute and not cuda:
        errors.append("no CUDA device is available")

    try:
        lerobot_version = importlib.metadata.version("lerobot")
    except importlib.metadata.PackageNotFoundError:
        lerobot_version = "not-installed"
    if execute and lerobot_version != "0.6.0":
        errors.append(f"LeRobot 0.6.0 required, found {lerobot_version}")

    return {
        "valid": not errors,
        "errors": errors,
        "cuda_available": cuda,
        "gpu": gpu_name,
        "torch": torch_version,
        "lerobot": lerobot_version,
        "disk_free_gb": round(disk_gb, 2),
        "output_dir": str(output_dir),
    }


def check_dataset(config: dict[str, Any], execute: bool) -> dict[str, Any]:
    manifest = ROOT / config["manifest"]
    if not manifest.exists():
        return {"valid": False, "errors": [f"missing dataset manifest: {manifest}"]}
    from reaching_dataset import validate_dataset, validate_manifest

    result = (
        validate_dataset(manifest, decode_video=True)
        if execute
        else validate_manifest(manifest)
    )
    manifest_data = json.loads(manifest.read_text(encoding="utf-8"))
    train = manifest_data.get("splits", {}).get("train", {})
    if train.get("repo_id") != config["dataset_repo_id"]:
        result["errors"].append("training repo_id does not match dataset manifest")
    if Path(train.get("root", "")) != Path(config["dataset_root"]):
        result["errors"].append("training root does not match dataset manifest")
    result["valid"] = not result["errors"]
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=ROOT / "training" / "b1_smolvla.json"
    )
    parser.add_argument("--min-disk-gb", type=float, default=30.0)
    parser.add_argument(
        "--output-dir", type=Path, help="Exact smoke/full output path to guard"
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Require CUDA, pinned LeRobot, and full decoded-dataset validation",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    environment = check_environment(
        config, args.execute, args.min_disk_gb, args.output_dir
    )
    dataset = check_dataset(config, args.execute)
    result = {
        "schema": "robollm.b1.gpu-preflight.v1",
        "mode": "execute" if args.execute else "dry-run",
        "config": str(args.config),
        "environment": environment,
        "dataset": dataset,
        "valid": environment["valid"] and dataset["valid"],
    }
    print("RESULT:" + json.dumps(result, sort_keys=True))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
