"""Phase 2 gate G2 (SDD §8): every recipe valid, noisy recipes ≥ 95 % success,
zero clean-label faults, CPU bench present. Writes
``results/p2/<host>/dataset_acceptance.json``; exit 1 on FAIL.

    .venv-lerobot/bin/python sim/vla-bed/p2_gate.py [--output-root datasets/vla-bed] [--no-decode]
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dataset as ds  # noqa: E402
import resources  # noqa: E402

REQUIRED_SUCCESS = {"v1": 1.0, "v2": 0.95, "v2b": 0.95, "v3": 0.95}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output-root", type=Path, default=ds.DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--no-decode", action="store_true")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    host = socket.gethostname()
    out = args.out or (Path(__file__).resolve().parent / "results" / "p2" / host)
    out.mkdir(parents=True, exist_ok=True)

    recipes: dict[str, dict] = {}
    checks: dict[str, bool] = {}
    t0 = time.perf_counter()
    for name in sorted(ds.RECIPES):
        manifest = args.output_root / name / "manifest.json"
        if not manifest.exists():
            recipes[name] = {"present": False}
            checks[f"{name}_present"] = False
            continue
        validation = ds.validate_dataset(manifest, decode_video=not args.no_decode)
        summary = ds.summarize(manifest) if validation["valid"] else {}
        recipes[name] = {"present": True, "validation": validation, "summary": summary}
        checks[f"{name}_present"] = True
        checks[f"{name}_valid"] = bool(validation["valid"])
        checks[f"{name}_clean_labels_pass"] = validation["clean_label_faults"] == 0
        for split, s in summary.get("splits", {}).items():
            checks[f"{name}_{split}_success_ge_{REQUIRED_SUCCESS[name]}"] = s["success_rate"] >= REQUIRED_SUCCESS[name]
    bench = out / "cpu_bench.json"
    checks["cpu_bench_present"] = bench.exists()
    verdict = "PASS" if checks and all(checks.values()) else "FAIL"
    acceptance = {
        "schema": "robollm.vla-bed.dataset-acceptance.v1",
        "phase": "P2",
        "verdict": verdict,
        "checks": checks,
        "host": host,
        "dependencies": ds.dependency_versions(),
        "recipes": {k: {kk: vv for kk, vv in v.items()} for k, v in recipes.items()},
        "cpu_bench": json.loads(bench.read_text()) if bench.exists() else None,
        "wall_s": round(time.perf_counter() - t0, 1),
        "resources": resources.snapshot(),
        "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    (out / "dataset_acceptance.json").write_text(json.dumps(acceptance, indent=2) + "\n")
    brief = {"verdict": verdict, "checks": checks}
    for name, r in recipes.items():
        if r.get("summary"):
            brief[name] = {split: {k: s[k] for k in ("episodes", "frames", "success_rate", "episode_len_mean", "chunk_padding_percent")} for split, s in r["summary"]["splits"].items()}
    print(json.dumps(brief, indent=2))
    print(f"G2 {verdict} on {host} → {out / 'dataset_acceptance.json'}")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
