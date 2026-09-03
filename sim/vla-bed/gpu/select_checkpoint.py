#!/usr/bin/env python3
"""Pick a run's checkpoint by closed-loop success on the held-out suite (SDD §14 R11: never by loss).

Reads results/p5/<host>/<run>/<step>/nominal.json written by evaluate.py; ranks by
success rate, then progress, then lower final error; writes selected.json with the
Wilson intervals so a tie inside the CI is visible.

    python sim/vla-bed/gpu/select_checkpoint.py --run baseline [--host NAME] [--execute]
"""

from __future__ import annotations

import argparse
import json
import socket
from pathlib import Path

BED_DIR = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", required=True)
    parser.add_argument("--host", default=socket.gethostname())
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    root = BED_DIR / "results" / "p5" / args.host / args.run
    candidates = []
    for path in sorted(root.glob("*/nominal.json")):
        doc = json.loads(path.read_text())
        block = doc[doc["policy"]["label"]]
        candidates.append({"step": path.parent.name, "success_rate": block["success_rate"], "ci95_wilson_success": block["ci95_wilson_success"], "progress_mean": block["progress_mean"], "final_error_m_mean": block["final_error_m_mean"], "safety": block["safety"], "n": block["n"], "file": str(path.relative_to(BED_DIR))})
    if not candidates:
        raise SystemExit(f"no nominal.json under {root}")
    selected = max(candidates, key=lambda c: (c["success_rate"], c["progress_mean"], -c["final_error_m_mean"]))
    within_ci = [c["step"] for c in candidates if c["success_rate"] >= selected["ci95_wilson_success"][0]]
    result = {"schema": "robollm.vla-bed.checkpoint-selection.v1", "run": args.run, "selected": selected, "steps_within_selected_ci": within_ci, "candidates": candidates}
    if not args.execute:
        print("DRY-RUN:" + json.dumps(result, sort_keys=True))
        return 0
    out = root / "selected.json"
    out.write_text(json.dumps(result, indent=2) + "\n")
    print("RESULT:" + json.dumps({"run": args.run, "selected": selected["step"], "success_rate": selected["success_rate"], "within_ci": within_ci}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
