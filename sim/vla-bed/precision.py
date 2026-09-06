"""Success-versus-tolerance curves from the per-episode rows already on disk (SDD §9; Curse of Precision, 2607.23108).

Every evaluation row records ``min_error_m``, the smallest end-effector-to-target distance reached in the
episode. Success in the suite means ≤ 0.03 m for five consecutive frames; re-thresholding ``min_error_m`` at a
larger tolerance P answers "how much would success rise if the acceptance radius were P", per recipe and
checkpoint, with Wilson intervals, and pairs two suites at every tolerance with ``compare.compare``.

Caveats (stored in every output): the curve counts an episode that ever reached P, while the suite's success
needs five consecutive frames within 0.03 m, so the P = 0.03 m point is ≥ the suite's success rate (the gap
counts episodes that reached the radius and did not hold it); and an episode stops once it succeeds, so
``min_error_m`` is right-censored at 0.03 m — the curve is valid only for P ≥ 0.03 m.
Fits of log(1 − SR) against log P and against 1/(P − c) (grid over c) are reported with their R²; with 8 points
they describe the curve, they do not establish a law.

    python sim/vla-bed/precision.py                      # every schema-v2 suite under results/p5/<host>/<run>/<ckpt>/
    python sim/vla-bed/precision.py --pair A.json B.json # one paired comparison at every tolerance
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

BED_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BED_DIR))
import compare  # noqa: E402
from report import esc, wilson  # noqa: E402

RESULTS = BED_DIR / "results"
SCHEMA_ROWS = "robollm.vla-bed.phase-summary.v2"
SCHEMA_OUT = "robollm.vla-bed.precision-curve.v1"
SUCCESS_TOLERANCE_M = 0.03
TOLERANCES_M = [round(0.03 + 0.01 * i, 3) for i in range(8)]  # 0.03 … 0.10
CAVEAT = ("Success at tolerance P counts an episode whose min_error_m ever dipped to P; the suite's own success needs five consecutive frames "
          "within 0.03 m, so the P = 0.03 m point is >= the suite's success rate (the gap = episodes that reached the radius and did not hold it). "
          "Episodes stop once they succeed, so min_error_m is right-censored at 0.03 m and the curve is valid for P >= 0.03 m only; larger P "
          "answers how much a wider acceptance radius would help.")
HOST_RECIPE = {"8ac6124fd05b": "v2", "7a90b7940018": "v2", "a85e64a183e5": "v2", "9f5dbf4dd492": "v3", "06f90d52039b": "v3", "145880075d6f": "v4", "8af08da6f6a2": "v5a", "bd883c780fba": "v6", "53f731baded6": "v5a"}  # Kaggle hosts → recipe of the checkpoint


def success_at_tolerance(rows: list[dict], tol_m: float) -> int:
    return int(sum(1 for r in rows if float(r["min_error_m"]) <= tol_m + 1e-9))


def curve(rows: list[dict], tolerances=TOLERANCES_M) -> list[dict]:
    n = len(rows)
    out = []
    for p in tolerances:
        k = success_at_tolerance(rows, p)
        lo, hi = wilson(k, n) if n else (0.0, 0.0)
        out.append({"tolerance_m": p, "k": k, "n": n, "success_rate": k / n if n else 0.0, "ci95_wilson": [round(lo, 4), round(hi, 4)]})
    return out


def fit_curse_of_precision(points: list[dict]) -> dict:
    """log(1 − SR) against log P (power law) and against 1/(P − c) (Curse of Precision), c on a grid below the smallest P."""
    p = np.array([q["tolerance_m"] for q in points], dtype=float)
    sr = np.array([q["success_rate"] for q in points], dtype=float)
    keep = sr < 1.0
    if keep.sum() < 3:
        return {"note": "fewer than 3 tolerances with failures; no fit"}
    y = np.log(1.0 - sr[keep]); p = p[keep]

    def r2(x, y):
        a, b = np.polyfit(x, y, 1)
        pred = a * x + b
        ss = float(np.sum((y - y.mean()) ** 2))
        return float(a), float(b), (1.0 - float(np.sum((y - pred) ** 2)) / ss if ss > 0 else 0.0)

    a_pow, b_pow, r2_pow = r2(np.log(p), y)
    best = None
    for c in np.linspace(0.0, p.min() - 1e-3, 60):
        a, b, r = r2(1.0 / (p - c), y)
        if best is None or r > best[3]:
            best = (float(c), a, b, r)
    return {"power_law": {"slope": round(a_pow, 4), "intercept": round(b_pow, 4), "r2": round(r2_pow, 4)},
            "curse_of_precision": {"c_m": round(best[0], 4), "slope": round(best[1], 5), "intercept": round(best[2], 4), "r2": round(best[3], 4)},
            "points_used": int(keep.sum())}


def load_suites(root: Path = RESULTS) -> list[tuple[Path, dict]]:
    found = []
    for f in sorted(root.glob("p5/*/*/*/*.json")):
        if f.name.startswith("compare_") or f.name == "magnitude.json":
            continue
        try:
            d = json.loads(f.read_text())
        except json.JSONDecodeError:
            continue
        if d.get("schema") == SCHEMA_ROWS and d.get("policy", {}).get("label") in d.get("episodes", {}):
            found.append((f, d))
    return found


def suite_curve(path: Path, d: dict) -> dict:
    label, rows = compare.rows_of(d)
    host = path.parts[-4]
    recipe = d.get("schedule", {}).get("recipe") or HOST_RECIPE.get(host)
    points = curve(rows)
    source = str(path.relative_to(RESULTS)) if RESULTS in path.parents else str(path)
    return {"schema": SCHEMA_OUT, "source": source, "host": host, "recipe": recipe, "label": label,
            "variation": d.get("schedule", {}).get("variation"), "post": d.get("policy", {}).get("post"), "gain": d.get("policy", {}).get("gain"),
            "blank_image": d.get("policy", {}).get("blank_image"), "n": len(rows), "success_tolerance_m": SUCCESS_TOLERANCE_M,
            "suite_success_rate": round(float(np.mean([bool(r["success"]) for r in rows])), 4) if rows else None,
            "reached_but_not_held": int(points[0]["k"] - sum(bool(r["success"]) for r in rows)),
            "curve": points, "fit": fit_curse_of_precision(points), "caveat": CAVEAT}


def pair_at_tolerances(a: dict, b: dict, tolerances=TOLERANCES_M) -> dict:
    """compare.compare at every tolerance, with success re-thresholded on min_error_m in both suites."""
    out = []
    for p in tolerances:
        def rethreshold(d):
            d2 = json.loads(json.dumps(d))
            label, rows = compare.rows_of(d2)
            for r in rows:
                r["success"] = bool(float(r["min_error_m"]) <= p + 1e-9)
            return d2
        c = compare.compare(rethreshold(a), rethreshold(b))
        out.append({"tolerance_m": p, "success_a": c["success_a"], "success_b": c["success_b"], "diff": c["success_diff_b_minus_a"],
                    "ci95": c["success_diff_ci95_paired_bootstrap"], "discordant": c["discordant"], "mcnemar_exact_p": c["mcnemar_exact_p"], "verdict": c["verdict"]})
    la, _ = compare.rows_of(a); lb, _ = compare.rows_of(b)
    return {"schema": "robollm.vla-bed.precision-paired.v1", "a": la, "b": lb, "caveat": CAVEAT, "at_tolerance": out}


def line_chart(title: str, series: list[tuple[str, list[dict], str]], width: int = 720, note: str = "") -> str:
    """Success vs tolerance, one polyline per series with Wilson whiskers; same palette and typography as report.hbar_chart."""
    left, right, top, bottom = 56, 24, 46, 52
    height = 320
    pw, ph = width - left - right, height - top - bottom
    xs = TOLERANCES_M
    x_of = lambda p: left + pw * (p - xs[0]) / (xs[-1] - xs[0])
    y_of = lambda v: top + ph * (1.0 - v)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" font-family="IBM Plex Sans, Helvetica, Arial, sans-serif" font-size="13">',
           f'<rect width="{width}" height="{height}" fill="#FFFFFF"/>', f'<text x="16" y="24" font-size="15" font-weight="600" fill="#1A232C">{esc(title)}</text>']
    for t in range(0, 6):
        y = y_of(t / 5)
        svg.append(f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" stroke="#E3E8EC" stroke-width="1"/>')
        svg.append(f'<text x="{left-8}" y="{y+4:.1f}" text-anchor="end" fill="#5A6874" font-size="11">{t/5:g}</text>')
    for p in xs:
        svg.append(f'<text x="{x_of(p):.1f}" y="{height-bottom+16}" text-anchor="middle" fill="#5A6874" font-size="11">{p:g}</text>')
    svg.append(f'<text x="{left+pw/2:.1f}" y="{height-bottom+32}" text-anchor="middle" fill="#5A6874" font-size="11">acceptance radius P (m), valid for P ≥ 0.03 m (episodes stop at success)</text>')
    for i, (label, pts, colour) in enumerate(series):
        poly = " ".join(f"{x_of(q['tolerance_m']):.1f},{y_of(q['success_rate']):.1f}" for q in pts)
        svg.append(f'<polyline points="{poly}" fill="none" stroke="{colour}" stroke-width="2"/>')
        for q in pts:
            x, lo, hi = x_of(q["tolerance_m"]), q["ci95_wilson"][0], q["ci95_wilson"][1]
            svg.append(f'<line x1="{x:.1f}" y1="{y_of(lo):.1f}" x2="{x:.1f}" y2="{y_of(hi):.1f}" stroke="{colour}" stroke-width="1" opacity="0.6"/>')
            svg.append(f'<circle cx="{x:.1f}" cy="{y_of(q["success_rate"]):.1f}" r="3" fill="{colour}"/>')
        svg.append(f'<rect x="{left+8}" y="{top+6+i*16}" width="10" height="10" fill="{colour}"/><text x="{left+24}" y="{top+15+i*16}" fill="#1A232C" font-size="11">{esc(label)}</text>')
    if note:
        svg.append(f'<text x="16" y="{height-4}" fill="#5A6874" font-size="11">{esc(note)}</text>')
    svg.append("</svg>")
    return "\n".join(svg)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", type=Path, default=RESULTS)
    ap.add_argument("--pair", nargs=2, type=Path, metavar=("A", "B"))
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    if args.pair:
        a, b = (json.loads(p.read_text()) for p in args.pair)
        res = pair_at_tolerances(a, b)
        out = args.out or args.pair[1].with_name(f"precision_paired_{args.pair[0].stem}_vs_{args.pair[1].stem}.json")
        out.write_text(json.dumps(res, indent=2) + "\n")
        for q in res["at_tolerance"]:
            print(f"P = {q['tolerance_m']:.2f} m: {q['success_a']:.2f} → {q['success_b']:.2f} (diff {q['diff']:+.2f} [{q['ci95'][0]:+.2f}, {q['ci95'][1]:+.2f}], p = {q['mcnemar_exact_p']:.3g}) {q['verdict']}")
        print(f"→ {out}")
        return 0
    out_root = args.out or (args.results / "p5" / "precision")
    written = []
    for path, d in load_suites(args.results):
        res = suite_curve(path, d)
        rel = path.relative_to(args.results / "p5")
        out = out_root / rel.parts[0] / rel.parts[1] / rel.parts[2] / (path.stem + ".json")
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(res, indent=2) + "\n")
        written.append(res)
        sr = " ".join(f"{q['success_rate']:.2f}" for q in res["curve"])
        print(f"{res['recipe'] or res['host']} {res['label']} {path.stem} n={res['n']}: suite {res['suite_success_rate']:.2f}, SR(P) {sr} | power-law r² {res['fit'].get('power_law', {}).get('r2')}, 1/(P-c) r² {res['fit'].get('curse_of_precision', {}).get('r2')} (c = {res['fit'].get('curse_of_precision', {}).get('c_m')})")
    print(f"{len(written)} curves → {out_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
