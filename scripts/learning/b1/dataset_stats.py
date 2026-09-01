#!/usr/bin/env python3
"""Report B1 training-shape statistics for a dataset manifest.

This is a diagnostic, not acceptance evidence: it never decodes video and never
writes the frozen ``robollm.b1.dataset-acceptance.v1`` artifact.  It answers the
two questions that decide whether a fine-tune is worth GPU hours — how much of
each action chunk is padding, and how many times the trainer will revisit every
frame.
"""

from __future__ import annotations

import argparse
import json
import statistics
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def chunk_padding_ratio(episode_frames: list[int], chunk_size: int) -> float:
    """Mean fraction of a predicted action chunk that is end-of-episode padding.

    LeRobot builds each sample's target from actions ``[t, t + chunk_size)`` and
    pads whatever runs past the final frame, so a chunk longer than the episode
    is mostly padding no matter which frame it starts from.
    """
    padded = 0
    total = 0
    for length in episode_frames:
        for start in range(length):
            padded += max(0, start + chunk_size - length)
            total += chunk_size
    return padded / total if total else 0.0


def summarize(manifest_path: Path, chunk_size: int, steps: int, batch: int) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    splits: dict[str, dict] = {}
    for name, split in manifest["splits"].items():
        lengths = [int(row["frame_count"]) for row in split["episodes"]]
        splits[name] = {
            "episodes": len(lengths),
            "frames": sum(lengths),
            "episode_frames": {
                "min": min(lengths),
                "max": max(lengths),
                "mean": round(statistics.fmean(lengths), 2),
                "median": statistics.median(lengths),
            },
            "family_counts": dict(
                sorted(Counter(row["family"] for row in split["episodes"]).items())
            ),
            "expert_success_rate": round(
                sum(int(row["success"]) for row in split["episodes"]) / len(lengths), 4
            ),
            "chunk_padding_ratio": round(chunk_padding_ratio(lengths, chunk_size), 4),
            # The recipe is recorded per split, not at the manifest root.
            "expert": split.get("expert", "oracle"),
            "expert_noise_scale": split.get("expert_noise_scale"),
        }

    train = splits.get("train", {})
    train_frames = train.get("frames", 0)
    return {
        "schema": "robollm.b1.dataset-stats.v1",
        "manifest": str(manifest_path),
        "chunk_size": chunk_size,
        "splits": splits,
        "training_shape": {
            "steps": steps,
            "batch_size": batch,
            "train_frames": train_frames,
            "sample_draws": steps * batch,
            "effective_epochs": (
                round(steps * batch / train_frames, 1) if train_frames else None
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=ROOT / "datasets" / "b1-red-target" / "manifest.json",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=50,
        help="Action chunk length to score padding against (SmolVLA default: 50)",
    )
    parser.add_argument("--steps", type=int, default=20_000)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--output", type=Path, help="Optional JSON path to write")
    args = parser.parse_args()

    result = summarize(args.manifest, args.chunk_size, args.steps, args.batch_size)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
