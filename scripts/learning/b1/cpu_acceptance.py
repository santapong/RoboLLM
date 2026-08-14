#!/usr/bin/env python3
"""Run B1 CPU policy/fault acceptance and save compact reproducible metrics."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "examples" / "mujoco"))

from evaluate_reaching import SUITES, compact_summary, evaluate
from reaching import episode_specs, run_oracle_episode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "examples" / "mujoco" / "results" / "b1_cpu_acceptance.json",
    )
    args = parser.parse_args()

    fixed_oracle = [
        run_oracle_episode(spec) for spec in episode_specs(100, 70_000, "evaluation")
    ]
    suites = {
        suite: compact_summary(
            evaluate("oracle", suite=suite, episodes=20, seed=70_000)
        )
        for suite in SUITES
    }
    baselines = {
        adapter: compact_summary(evaluate(adapter, episodes=20, seed=70_000))
        for adapter in ("hold", "noise")
    }
    faults = {
        fault: compact_summary(evaluate("oracle", episodes=1, seed=70_000, fault=fault))
        for fault in ("nan", "out_of_range", "overspeed", "camera_dropout")
    }
    fixed_successes = sum(bool(row["success"]) for row in fixed_oracle)
    result = {
        "schema": "robollm.b1.cpu-acceptance.v1",
        "status": "B1 preparation: complete",
        "fixed_seed_oracle": {
            "episodes": 100,
            "successes": fixed_successes,
            "success_rate": fixed_successes / 100,
        },
        "suites": suites,
        "baselines": baselines,
        "faults": faults,
        "acceptance": {
            "oracle_at_least_95_of_100": fixed_successes >= 95,
            "all_invalid_actions_rejected": all(
                faults[name]["rejections"] == 1
                and faults[name]["invalid_commands_reaching_env"] == 0
                for name in ("nan", "out_of_range", "overspeed")
            ),
            "camera_dropout_aborts_within_one_tick": faults["camera_dropout"]["aborts"]
            == 1,
            "baselines_materially_below_oracle": max(
                row["success_rate"] for row in baselines.values()
            )
            <= suites["nominal"]["success_rate"] - 0.5,
        },
    }
    result["valid"] = all(result["acceptance"].values())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print("RESULT:" + json.dumps(result["acceptance"], sort_keys=True))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
