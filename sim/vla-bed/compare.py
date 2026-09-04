"""Paired comparison of two evaluation results on the same seeds (SDD §9, rule R7).

Two suites of the frozen 100 seeds are paired episode by episode, so the question
"is B better than A?" is answered from the discordant pairs (A failed, B succeeded
and vice versa) instead of from two overlapping Wilson intervals: an exact McNemar
test and a paired bootstrap of the success difference. At n = 100 this resolves
differences that the marginal intervals (±8 points) cannot.

    python sim/vla-bed/compare.py A.json B.json [--out C.json]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def rows_of(summary: dict) -> tuple[str, list[dict]]:
    label = summary["policy"]["label"]
    return label, summary["episodes"][label]


def pair_rows(a_rows: list[dict], b_rows: list[dict]) -> list[tuple[dict, dict]]:
    b_by_seed = {r["seed"]: r for r in b_rows}
    return [(r, b_by_seed[r["seed"]]) for r in sorted(a_rows, key=lambda r: r["seed"]) if r["seed"] in b_by_seed]


def mcnemar_exact(b01: int, b10: int) -> float:
    """Two-sided exact McNemar p-value: under H0 the discordant pairs split 50/50."""
    n = b01 + b10
    if n == 0:
        return 1.0
    k = min(b01, b10)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2**n
    return float(min(1.0, 2 * tail))


def paired_bootstrap(diffs, n_boot: int = 10_000, seed: int = 0, alpha: float = 0.05) -> tuple[float, float]:
    arr = np.asarray(list(diffs), dtype=float)
    if arr.size == 0:
        return (0.0, 0.0)
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, arr.size, size=(n_boot, arr.size))
    means = arr[idx].mean(axis=1)
    return (float(np.quantile(means, alpha / 2)), float(np.quantile(means, 1 - alpha / 2)))


def compare(a: dict, b: dict) -> dict:
    la, ra = rows_of(a)
    lb, rb = rows_of(b)
    pairs = pair_rows(ra, rb)
    if not pairs:
        raise ValueError("no shared seeds")
    sa = np.array([bool(x["success"]) for x, _ in pairs])
    sb = np.array([bool(y["success"]) for _, y in pairs])
    b01 = int(np.sum(~sa & sb))  # A failed, B succeeded
    b10 = int(np.sum(sa & ~sb))
    diffs = sb.astype(float) - sa.astype(float)
    pa = np.array([x["progress"] for x, _ in pairs])
    pb = np.array([y["progress"] for _, y in pairs])
    families: dict[str, list[float]] = {}
    for x, y in pairs:
        families.setdefault(x["family"], []).append(float(y["success"]) - float(x["success"]))

    def rej(rows):
        vals = [r["rejected_fraction"] if "rejected_fraction" in r else (sum(r["rejections"].values()) / r["frames"] if r["frames"] else 0.0) for r in rows]
        return float(np.mean(vals)) if vals else None

    p = mcnemar_exact(b01, b10)
    lo, hi = paired_bootstrap(diffs)
    d = float(diffs.mean())
    verdict = "B better" if (hi < 0 or lo > 0) and d > 0 else ("A better" if (hi < 0 or lo > 0) else "not separable at n = %d" % len(pairs))
    return {
        "schema": "robollm.vla-bed.paired-comparison.v1",
        "a": {"label": la, "variation": a["schedule"].get("variation"), "post": a["policy"].get("post"), "gain": a["policy"].get("gain"), "checkpoint": a["policy"].get("checkpoint")},
        "b": {"label": lb, "variation": b["schedule"].get("variation"), "post": b["policy"].get("post"), "gain": b["policy"].get("gain"), "checkpoint": b["policy"].get("checkpoint")},
        "n_pairs": len(pairs),
        "success_a": float(sa.mean()),
        "success_b": float(sb.mean()),
        "success_diff_b_minus_a": d,
        "success_diff_ci95_paired_bootstrap": [lo, hi],
        "discordant": {"a_fail_b_success": b01, "a_success_b_fail": b10, "both_success": int(np.sum(sa & sb)), "both_fail": int(np.sum(~sa & ~sb))},
        "mcnemar_exact_p": p,
        "progress_a": float(pa.mean()),
        "progress_b": float(pb.mean()),
        "progress_diff_ci95_paired_bootstrap": list(paired_bootstrap(pb - pa)),
        "rejected_fraction_a": rej([x for x, _ in pairs]),
        "rejected_fraction_b": rej([y for _, y in pairs]),
        "per_family_success_diff": {f: float(np.mean(v)) for f, v in sorted(families.items())},
        "verdict": verdict,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("a", type=Path)
    parser.add_argument("b", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    result = compare(json.loads(args.a.read_text()), json.loads(args.b.read_text()))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2) + "\n")
    d = result["discordant"]
    print(f"{result['a']['label']} ({result['a']['variation']}, post {result['a']['post']}, gain {result['a']['gain']}) vs {result['b']['label']} ({result['b']['variation']}, post {result['b']['post']}, gain {result['b']['gain']}): "
          f"n = {result['n_pairs']}, success {result['success_a']:.2f} → {result['success_b']:.2f} (diff {result['success_diff_b_minus_a']:+.2f} [{result['success_diff_ci95_paired_bootstrap'][0]:+.2f}, {result['success_diff_ci95_paired_bootstrap'][1]:+.2f}]), "
          f"discordant {d['a_fail_b_success']} vs {d['a_success_b_fail']}, McNemar p = {result['mcnemar_exact_p']:.3g} → {result['verdict']}")
    if args.out:
        print(f"→ {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
