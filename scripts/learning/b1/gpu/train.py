#!/usr/bin/env python3
"""Render or explicitly execute the pinned local-only LeRobot training command."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("smoke", "full"))
    parser.add_argument(
        "--config", type=Path, default=ROOT / "configs" / "training" / "b1_smolvla.json"
    )
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    steps = config["smoke_steps"] if args.mode == "smoke" else config["steps"]
    output = ROOT / config["output_dir"] / args.mode
    command = [
        "lerobot-train",
        f"--policy.path={config['model']}",
        f"--dataset.repo_id={config['dataset_repo_id']}",
        f"--dataset.root={ROOT / config['dataset_root']}",
        f"--batch_size={2 if args.mode == 'smoke' else config['batch_size']}",
        f"--num_workers={config['num_workers']}",
        f"--steps={steps}",
        f"--save_freq={steps if args.mode == 'smoke' else config['save_freq']}",
        "--save_checkpoint=true",
        f"--output_dir={output}",
        f"--job_name=robollm_b1_{args.mode}",
        f"--seed={config['seed']}",
        "--policy.device=cuda",
        "--policy.push_to_hub=false",
        "--save_checkpoint_to_hub=false",
        "--wandb.enable=false",
    ]
    rendered = shlex.join(command)
    if not args.execute:
        print("DRY-RUN: " + rendered)
        return 0

    preflight = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "learning" / "b1" / "gpu" / "preflight.py"),
            "--execute",
            "--output-dir",
            str(output),
        ],
        check=False,
    )
    if preflight.returncode:
        return preflight.returncode
    output.parent.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.update(
        {
            "WANDB_MODE": "disabled",
            "HF_HUB_DISABLE_TELEMETRY": "1",
            "HF_HUB_OFFLINE": "0",
        }
    )
    return subprocess.run(command, cwd=ROOT, env=environment, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
