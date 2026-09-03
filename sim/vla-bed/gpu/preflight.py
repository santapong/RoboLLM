#!/usr/bin/env python3
"""Bed GPU preflight (SDD §8 P4): everything that must be true before a priced command runs.

Dry run (default) reports; --execute additionally requires CUDA, LeRobot 0.6.0,
the pinned smolvla_base revision, decoded-dataset validation of the v2 split,
and free disk for checkpoints. Writes the resolved model revision into the
report so the SDD cost ledger can pin it.

    python sim/vla-bed/gpu/preflight.py [--execute] [--min-disk-gb 30]
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

BED_DIR = Path(__file__).resolve().parents[1]
ROOT = BED_DIR.parents[1]
sys.path.insert(0, str(BED_DIR))
import dataset as ds  # noqa: E402
import resources  # noqa: E402

CONFIG_FILE = BED_DIR / "gpu" / "config.json"


def check_environment(config: dict, execute: bool, min_disk_gb: float) -> dict:
    out_root = ROOT / config["output_dir"]
    probe = out_root if out_root.exists() else ROOT
    disk_gb = shutil.disk_usage(probe).free / 1e9
    errors: list[str] = []
    if disk_gb < min_disk_gb:
        errors.append(f"only {disk_gb:.1f} GB free; require {min_disk_gb:.1f} GB")
    cuda, gpu, vram_gb, torch_version = False, None, None, "not-installed"
    try:
        import torch

        torch_version = torch.__version__
        cuda = torch.cuda.is_available()
        if cuda:
            gpu = torch.cuda.get_device_name(0)
            vram_gb = round(torch.cuda.get_device_properties(0).total_memory / 1e9, 1)
    except ImportError:
        if execute:
            errors.append("torch is not installed")
    if execute and not cuda:
        errors.append("no CUDA device is available")
    try:
        lerobot_version = importlib.metadata.version("lerobot")
    except importlib.metadata.PackageNotFoundError:
        lerobot_version = "not-installed"
    if execute and lerobot_version != config["lerobot_version"]:
        errors.append(f"LeRobot {config['lerobot_version']} required, found {lerobot_version}")
    revision = None
    try:
        from huggingface_hub import snapshot_download

        path = snapshot_download(config["model"], allow_patterns=["config.json"], local_files_only=not execute)
        revision = Path(path).name
    except Exception as exc:  # noqa: BLE001 — offline dry runs may have no cache
        if execute:
            errors.append(f"could not resolve {config['model']}: {exc}")
    pinned = config.get("model_revision")
    if execute and pinned and pinned != "SET_BY_PREFLIGHT" and revision and revision != pinned:
        errors.append(f"model revision {revision} differs from the pinned {pinned}")
    return {
        "valid": not errors,
        "errors": errors,
        "cuda_available": cuda,
        "gpu": gpu,
        "vram_gb": vram_gb,
        "torch": torch_version,
        "lerobot": lerobot_version,
        "model_revision": revision,
        "disk_free_gb": round(disk_gb, 2),
        "cpus_online": resources.snapshot()["cpus_online"],
    }


def check_datasets(config: dict, execute: bool, dataset_root: Path | None = None) -> dict:
    base = Path(dataset_root) if dataset_root else None  # a copy of datasets/vla-bed/v2 elsewhere (Kaggle input)
    manifest = (base / "manifest.json") if base else ROOT / config["dataset"]["manifest"]
    if not manifest.exists():
        return {"valid": False, "errors": [f"missing dataset manifest: {manifest}"]}
    data = json.loads(manifest.read_text())
    if base:  # the manifest's roots are the recording machine's; point them at the copy
        for split, entry in data["splits"].items():
            entry["root"] = str(base / split)
        patched = manifest.parent / "manifest.local.json" if os.access(manifest.parent, os.W_OK) else Path(tempfile.gettempdir()) / "vla-bed-manifest.local.json"
        patched.write_text(json.dumps(data))
        manifest = patched
    result = ds.validate_dataset(manifest, decode_video=execute) if execute else ds.validate_manifest(manifest)
    errors = list(result.get("errors", []))
    for split in ("train", "evaluation"):
        entry = data["splits"].get(split, {})
        if entry.get("repo_id") != config["dataset"][f"{split}_repo_id"]:
            errors.append(f"{split} repo_id does not match the manifest")
        expected = (base / split) if base else ROOT / config["dataset"][f"{split}_root"]
        if Path(entry.get("root", "")).resolve() != expected.resolve():
            errors.append(f"{split} root does not match the manifest")
    if result.get("clean_label_faults", 0):
        errors.append(f"{result['clean_label_faults']} clean-label faults")
    result["errors"] = errors
    result["valid"] = not errors
    return result


def check_runs(config: dict) -> dict:
    errors = []
    names = [r["name"] for r in config["runs"]]
    if len(set(names)) != len(names):
        errors.append("duplicate run names")
    for r in config["runs"]:
        out = ROOT / config["output_dir"] / r["name"] / "full"
        if r.get("enabled") and out.exists() and any(out.iterdir()):
            errors.append(f"output path not empty for enabled run {r['name']}: {out}")
    return {"valid": not errors, "errors": errors, "enabled": [r["name"] for r in config["runs"] if r.get("enabled")]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--min-disk-gb", type=float, default=30.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dataset-root", type=Path, default=None, help="a copy of datasets/vla-bed/v2 (e.g. /kaggle/input/vla-bed-v2/v2)")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    env = check_environment(config, args.execute, args.min_disk_gb)
    data = check_datasets(config, args.execute, args.dataset_root)
    runs = check_runs(config)
    result = {
        "schema": "robollm.vla-bed.gpu-preflight.v1",
        "mode": "execute" if args.execute else "dry-run",
        "environment": env,
        "datasets": {k: v for k, v in data.items() if k != "splits"},
        "runs": runs,
        "valid": env["valid"] and data["valid"] and runs["valid"],
    }
    print("RESULT:" + json.dumps(result, sort_keys=True, default=str))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
