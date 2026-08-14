#!/usr/bin/env python3
"""Validate a B1 LeRobot dataset and persist only a compact evidence summary."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "examples" / "mujoco"))

from reaching_dataset import validate_dataset


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "examples" / "mujoco" / "results" / "b1_dataset_acceptance.json",
    )
    args = parser.parse_args()
    validation = validate_dataset(args.manifest, decode_video=True)
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    families: Counter[str] = Counter()
    successes = 0
    max_frames = 0
    for split in manifest["splits"].values():
        for row in split["episodes"]:
            families[row["family"]] += 1
            successes += int(row["success"])
            max_frames = max(max_frames, int(row["frame_count"]))
    result = {
        "schema": "robollm.b1.dataset-acceptance.v1",
        "valid": validation["valid"],
        "episodes": validation["episodes"],
        "frames": validation["frames"],
        "decoded_video_frames": validation["decoded_video_frames"],
        "splits": validation["splits"],
        "family_counts": dict(sorted(families.items())),
        "successful_expert_episodes": successes,
        "maximum_episode_frames_observed": max_frames,
        "maximum_episode_frames_allowed": manifest["max_frames_per_episode"],
        "instruction": manifest["instruction"],
        "fps": manifest["fps"],
        "dependencies": manifest["dependencies"],
        "git_commit": manifest["git_commit"],
        "git_dirty": manifest.get("git_dirty", True),
        "errors": validation["errors"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print("RESULT:" + json.dumps(result, sort_keys=True))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
