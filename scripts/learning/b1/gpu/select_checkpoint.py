#!/usr/bin/env python3
"""Select the best frozen-suite checkpoint from compact evaluation JSON files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

EXPECTED_STEPS = (5000, 10000, 20000)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument(
        "--output", type=Path, default=Path("artifacts/b1-results/selected.json")
    )
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    candidates = []
    for step in EXPECTED_STEPS:
        files = sorted((args.results_root / f"{step:06d}").glob("*.json"))
        if not files:
            continue
        rows = [json.loads(path.read_text(encoding="utf-8")) for path in files]
        candidates.append(
            {
                "step": step,
                "mean_success_rate": sum(row["success_rate"] for row in rows)
                / len(rows),
                "mean_final_error_m": sum(
                    row["end_effector_error_m"]["final_mean"] for row in rows
                )
                / len(rows),
                "suite_count": len(rows),
            }
        )
    if not candidates:
        raise SystemExit("no results found for checkpoints 5000, 10000, or 20000")
    selected = max(
        candidates,
        key=lambda row: (row["mean_success_rate"], -row["mean_final_error_m"]),
    )
    result = {
        "schema": "robollm.b1.checkpoint-selection.v1",
        "selected": selected,
        "candidates": candidates,
    }
    if not args.execute:
        print("DRY-RUN:" + json.dumps(result, sort_keys=True))
        return 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print("RESULT:" + json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
