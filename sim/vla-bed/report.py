"""Experiment report for the UR5e VLA bed: reads the committed result JSONs, writes
results/REPORT-<date>.md with hand-drawn SVG charts (no plotting dependency) and,
with --html, a single self-contained page.

    python sim/vla-bed/report.py [--html OUT.html]
"""

from __future__ import annotations

import argparse
import json
import math
import re
from datetime import date
from pathlib import Path

BED = Path(__file__).resolve().parent
R = BED / "results"
OUT = R / "report"


def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


def load(p: Path) -> dict:
    return json.loads(p.read_text())


# ----- SVG helpers -----


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def bold(s: str) -> str:
    """Markdown **spans** → <b>, applied after escaping."""
    return re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", s)


def hbar_chart(title: str, rows: list[tuple[str, float, tuple[float, float] | None, str]], xmax: float = 1.0, unit: str = "", width: int = 640, note: str = "", marker: tuple[float, str] | None = None) -> str:
    """rows: (label, value, (lo, hi) or None, colour)."""
    left, top, rh, gap = 230, 46, 26, 10
    plot_w = width - left - 40
    height = top + len(rows) * (rh + gap) + 46
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" font-family="IBM Plex Sans, Helvetica, Arial, sans-serif" font-size="13">',
           f'<rect width="{width}" height="{height}" fill="#FFFFFF"/>',
           f'<text x="16" y="24" font-size="15" font-weight="600" fill="#1A232C">{esc(title)}</text>']
    for t in range(0, 6):
        x = left + plot_w * t / 5
        svg.append(f'<line x1="{x:.1f}" y1="{top-6}" x2="{x:.1f}" y2="{height-36}" stroke="#E3E8EC" stroke-width="1"/>')
        svg.append(f'<text x="{x:.1f}" y="{height-18}" text-anchor="middle" fill="#5A6874" font-size="11">{xmax*t/5:g}{unit}</text>')
    for i, (label, val, ci, colour) in enumerate(rows):
        y = top + i * (rh + gap)
        w = plot_w * min(val, xmax) / xmax
        svg.append(f'<text x="{left-10}" y="{y+rh/2+4}" text-anchor="end" fill="#1A232C">{esc(label)}</text>')
        svg.append(f'<rect x="{left}" y="{y}" width="{w:.1f}" height="{rh}" fill="{colour}" rx="2"/>')
        if ci:
            lo, hi = ci
            x0, x1 = left + plot_w * lo / xmax, left + plot_w * min(hi, xmax) / xmax
            ym = y + rh / 2
            svg.append(f'<line x1="{x0:.1f}" y1="{ym}" x2="{x1:.1f}" y2="{ym}" stroke="#1A232C" stroke-width="1.5"/>')
            for x in (x0, x1):
                svg.append(f'<line x1="{x:.1f}" y1="{ym-6}" x2="{x:.1f}" y2="{ym+6}" stroke="#1A232C" stroke-width="1.5"/>')
        txt = f"{val:.2f}" if xmax <= 1.0 else f"{val:g}"
        tx = left + w + 6 if (ci is None or ci[1] * plot_w / xmax + left + 6 < width - 60) else left + w + 6
        if ci:
            tx = max(tx, left + plot_w * min(ci[1], xmax) / xmax + 8)
        svg.append(f'<text x="{tx:.1f}" y="{y+rh/2+4}" fill="#1A232C" font-size="12">{txt}{unit}</text>')
    if marker:
        mx = left + plot_w * marker[0] / xmax
        svg.append(f'<line x1="{mx:.1f}" y1="{top-6}" x2="{mx:.1f}" y2="{height-36}" stroke="#9A6A12" stroke-width="1.5" stroke-dasharray="4 3"/>')
        svg.append(f'<text x="{mx+4:.1f}" y="{top-10}" fill="#9A6A12" font-size="11">{esc(marker[1])}</text>')
    if note:
        svg.append(f'<text x="16" y="{height-4}" fill="#5A6874" font-size="11">{esc(note)}</text>')
    svg.append("</svg>")
    return "\n".join(svg)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--html", type=Path, default=None)
    ap.add_argument("--date", default=date.today().isoformat())
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    p1 = load(R / "p1" / "santapong" / "summary.json")
    p2 = load(R / "p2" / "santapong" / "dataset_acceptance.json")
    bench = load(R / "p2" / "santapong" / "cpu_bench.json")
    p2b = load(R / "p2b" / "full" / "eval_info.json")["overall"]
    p3 = load(R / "p3" / "santapong" / "summary.json")
    oracle = load(R / "p5" / "santapong" / "oracle" / "nominal.json")["oracle"]
    hold = load(R / "p5" / "santapong" / "hold" / "nominal.json")["hold"]
    kg = load(R / "p5" / "kaggle" / "eval-v2-partial.json")
    pr = load(R / "p5" / "kaggle" / "probes-v2-partial.json") if (R / "p5" / "kaggle" / "probes-v2-partial.json").exists() else None
    e3 = load(R / "p5" / "kaggle" / "eval-v3-partial.json") if (R / "p5" / "kaggle" / "eval-v3-partial.json").exists() else None
    e4 = load(R / "p5" / "kaggle" / "eval-v4-partial.json") if (R / "p5" / "kaggle" / "eval-v4-partial.json").exists() else None
    e5 = load(R / "p5" / "kaggle" / "eval-v5a-partial.json") if (R / "p5" / "kaggle" / "eval-v5a-partial.json").exists() else None
    e6 = load(R / "p5" / "kaggle" / "eval-v6-partial.json") if (R / "p5" / "kaggle" / "eval-v6-partial.json").exists() else None
    e7 = load(R / "p5" / "kaggle" / "eval-plastic-v5a-partial.json") if (R / "p5" / "kaggle" / "eval-plastic-v5a-partial.json").exists() else None   # plastic variant on the v5a recipe
    # imported Kaggle result files (schema v2 rows, paired comparisons, magnitude probes) — empty until gpu/kaggle_import.sh has run
    compares = sorted(R.glob("p5/*/*/*/compare_*.json"))
    precision_files = sorted(R.glob("p5/precision/*/*/*/*.json"))
    magnitudes = sorted(R.glob("p5/*/*/*/magnitude.json"))
    v2rows = sorted(f for f in R.glob("p5/*/*/*/*.json") if f.name not in ("magnitude.json",) and not f.name.startswith("compare_") and "kaggle" not in f.parts and "santapong" not in f.parts)
    pol = kg["nominal_010000_100ep"]; quick = kg["quick_check_010000_50ep"]; tr = kg["train_record"]; sm = kg["smoke_v5"]
    blank = kg.get("blank_image_010000_100ep"); gain = kg.get("gain061_010000_100ep")
    prog = kg.get("progress")
    cam = kg.get("camera_shift_010000_100ep"); sel = kg.get("selection")
    curve = sorted((int(k.split("_")[1]), v) for k, v in kg.items() if k.startswith("nominal_") and k.endswith("_100ep"))
    lib_k, lib_n = round(p2b["pc_success"] / 100 * p2b["n_episodes"]), p2b["n_episodes"]
    lib_ci = wilson(lib_k, lib_n)
    libero_per_task = [5, 5, 5, 5, 4, 0, 4, 4, 4, 5]  # from results/p2b/full.log (final line per task, de-duplicated)
    v2 = p2["recipes"]["v2"]["summary"]["splits"]
    med = bench.get("timing_s", {}).get("median") or bench.get("timing_s", {}).get("median_s")

    teal, grey, amber, red = "#0E7C86", "#9AA8B3", "#B7791F", "#B23A3A"
    charts = {}
    charts["success-rates"] = hbar_chart(
        "Closed-loop success on the 100 held-out seeds (Wilson 95 % intervals)",
        [("Scripted oracle (control)", oracle["success_rate"], tuple(oracle["ci95_wilson_success"]), grey),
         ("Do-nothing hold (control)", hold["success_rate"], tuple(hold["ci95_wilson_success"]), grey),
         ("SmolVLA baseline, 10k steps, n = 100", pol["success_rate"], tuple(pol["ci95_wilson_success"]), teal),
         ("same checkpoint, quick check, n = 50", quick["success_rate"], tuple(quick["ci95_wilson_success"]), "#5FB0B7")]
        + ([("same checkpoint, commands × 0.61 (gain probe)", gain["success_rate"], tuple(gain["ci95_wilson_success"]), "#9A6A12")] if gain else [])
        + ([("same checkpoint, camera blanked (vision probe)", blank["success_rate"], tuple(blank["ci95_wilson_success"]), red)] if blank else [])
        + ([("same checkpoint, camera shifted (variation)", cam["success_rate"], tuple(cam["ci95_wilson_success"]), "#7A5C99")] if cam else [])
        + [("LIBERO-Spatial calibration, n = 50", lib_k / lib_n, lib_ci, amber)],
        note="LIBERO row: lerobot/smolvla_libero evaluated on this CPU; dashed line = the published SmolVLA-0.45B score.", marker=(0.90, "published 0.90"))
    fam = pol["per_family_success"]; n = pol["per_family_n"]
    charts["per-family"] = hbar_chart("Baseline success by goal family (n = 20 each, Wilson 95 %)",
                                      [(f"{k}", v, wilson(round(v * n), n), teal) for k, v in fam.items()],
                                      note="Lateral cells (front_low, right) are the weakest; near targets the strongest.")
    final_steps = curve[-1][0]
    charts["learning-curve"] = hbar_chart("Learning curve: closed-loop success per checkpoint (nominal, n = 100 each, Wilson 95 %)",
                                          [(f"{st/1000:g}k steps" + (" (final)" if st == final_steps else ""), v["success_rate"], tuple(v["ci95_wilson_success"]), "#5FB0B7" if st == final_steps else teal) for st, v in curve]
                                          + [("10k, quick check, n = 50", quick["success_rate"], tuple(quick["ci95_wilson_success"]), grey)],
                                          note="Checkpoints not shown are still being evaluated on Kaggle; the R11 rule selects by this number, never by loss.")
    if e3:
        v3curve = {int(su["label"].split("/")[-1]): su for su in e3["suites"] if su["variation"] == "nominal" and not su.get("blank_image") and su.get("gain", 1.0) == 1.0}
        rows_lc = []
        for st, v in curve:
            rows_lc.append((f"v2 · {st/1000:g}k", v["success_rate"], tuple(v["ci95_wilson_success"]), teal))
            if st in v3curve:
                rows_lc.append((f"v3 · {st/1000:g}k (headroom + jitter)", v3curve[st]["success_rate"], tuple(v3curve[st]["ci95_wilson_success"]), "#7A5C99"))
        v4curve = {int(su["label"].split("/")[-1]): su for su in e4["suites"] if su["variation"] == "nominal" and not su.get("blank_image") and su.get("gain", 1.0) == 1.0} if e4 else {}
        v5curve = {int(su["label"].split("/")[-1]): su for su in e5["suites"] if su["variation"] == "nominal" and not su.get("blank_image") and su.get("gain", 1.0) == 1.0} if e5 else {}
        v6curve = {int(su["label"].split("/")[-1]): su for su in e6["suites"] if su["variation"] == "nominal" and not su.get("blank_image") and su.get("gain", 1.0) == 1.0} if e6 else {}
        v7curve = {int(su["label"].split("/")[-1]): su for su in e7["suites"] if su["variation"] == "nominal" and not su.get("blank_image") and su.get("gain", 1.0) == 1.0} if e7 else {}
        if v4curve:
            rows_lc = []
            for st, v in curve:
                rows_lc.append((f"v2 · {st/1000:g}k", v["success_rate"], tuple(v["ci95_wilson_success"]), teal))
                if st in v3curve:
                    rows_lc.append((f"v3 · {st/1000:g}k (headroom + azimuth jitter)", v3curve[st]["success_rate"], tuple(v3curve[st]["ci95_wilson_success"]), "#7A5C99"))
                if st in v4curve:
                    rows_lc.append((f"v4 · {st/1000:g}k (+ camera translation jitter)", v4curve[st]["success_rate"], tuple(v4curve[st]["ci95_wilson_success"]), "#2E7D4F"))
                if st in v5curve:
                    rows_lc.append((f"v5a · {st/1000:g}k (expert noise 0.25×)", v5curve[st]["success_rate"], tuple(v5curve[st]["ci95_wilson_success"]), "#B7791F"))
                if st in v6curve:
                    rows_lc.append((f"v6 · {st/1000:g}k (+ wrist camera)", v6curve[st]["success_rate"], tuple(v6curve[st]["ci95_wilson_success"]), "#B03A2E"))
                if st in v7curve:
                    rows_lc.append((f"v5a plastic · {st/1000:g}k (VLM unfrozen, batch 8)", v7curve[st]["success_rate"], tuple(v7curve[st]["ci95_wilson_success"]), "#6B6B6B"))
        charts["learning-curve-v2-vs-v3"] = hbar_chart("Baseline on v2 vs v3" + (" vs v4" if v4curve else "") + (" vs v5a" if v5curve else "") + (" vs v6" if v6curve else "") + (" (+ plastic on v5a)" if v7curve else "") + " data: closed-loop success per checkpoint (same 100 seeds, Wilson 95 %)", rows_lc,
                                                       note="v3 caps the expert at 0.7 × the limits and jitters the camera azimuth ±20° on the train split; v4 adds a camera translation ±0.20 m (x, y), ±0.05 m (z); every v3/v4 checkpoint has 0 % over-cap steps.")
        if e4:
            def _suite(e, var, **kw):
                for su in e["suites"]:
                    if su["variation"] == var and su["label"].endswith("010000") and not su.get("blank_image") and su.get("gain", 1.0) == 1.0:
                        return su
                return None
            rows_vp = [("v2 · nominal", pol["success_rate"], tuple(pol["ci95_wilson_success"]), teal)]
            if cam:
                rows_vp.append(("v2 · camera_shift (+0.15, +0.10 m)", cam["success_rate"], tuple(cam["ci95_wilson_success"]), "#5FB0B7"))
            for rec, e, col, col2 in (("v3", e3, "#7A5C99", "#A98BC4"), ("v4", e4, "#2E7D4F", "#6FB08A"), ("v5a", e5, "#B7791F", "#D9A441"), ("v6", e6, "#B03A2E", "#E07B6E")):
                if e is None:
                    continue
                nom, cs, far = _suite(e, "nominal"), _suite(e, "camera_shift"), _suite(e, "camera_shift_far")
                if nom: rows_vp.append((f"{rec} · nominal", nom["success_rate"], tuple(nom["ci95_wilson_success"]), col))
                if cs: rows_vp.append((f"{rec} · camera_shift (+0.15, +0.10 m)", cs["success_rate"], tuple(cs["ci95_wilson_success"]), col2))
                if far: rows_vp.append((f"{rec} · camera_shift_far (+0.30, +0.20 m)", far["success_rate"], tuple(far["ci95_wilson_success"]), amber))
            charts["viewpoint"] = hbar_chart("Viewpoint robustness: the 10k checkpoint of each recipe at the nominal and the shifted camera (same 100 seeds, Wilson 95 %)", rows_vp,
                                             note="v4 and v5a (train split recorded with a per-episode camera translation) keep their nominal score under camera_shift but fall back outside the jittered range; v6 (wrist camera) holds 0.86 under the shift and 0.75 at the far view — the eye-in-hand stream does not move with the scene camera.")
    p5_md, p5_html = "", ""
    zmq_files = sorted(R.glob("p5/santapong/*/**/nominal_zmq.json"))
    if zmq_files:
        lines = ["## 3f. P5 — the bed served over ZeroMQ (Pi simulator, workstation client)", "",
                 "| Suite through the wire | n | success | Wilson 95 % | safety | progress | policy call | requests | wire time | wall | in-process rows |", "|---|---|---|---|---|---|---|---|---|---|---|"]
        for f in zmq_files:
            d = load(f); lab = d["policy"]["label"]; b = d[lab]; e = d.get("env", {}); ci = b["ci95_wilson_success"]
            local = f.with_name("nominal.json")
            same = ""
            if local.exists():
                ld = load(local); lrows = {r["seed"]: r for r in ld["episodes"][ld["policy"]["label"]]}
                fields = ("success", "progress", "frames", "final_error_m", "min_error_m", "safe", "rejections", "measured", "worst_depth")
                mism = sum(1 for r in d["episodes"][lab] if r["seed"] in lrows for k in fields if r.get(k) != lrows[r["seed"]].get(k))
                same = f"{mism} field mismatches"
            elif d["policy"]["name"] == "smolvla":
                # Policy suites have no in-process twin on the workstation; compare with the Kaggle rows on the same seeds.
                import re
                import precision as pz
                m = re.match(r"baseline-(v\w+)/(\d+)$", lab)
                for host, rec in (pz.HOST_RECIPE.items() if m else ()):
                    kf = R / "p5" / host / "baseline" / m.group(2) / "nominal.json"
                    if rec == m.group(1) and kf.exists():
                        kd = load(kf); krows = {r["seed"]: r for r in kd["episodes"][kd["policy"]["label"]]}
                        wire = [r for r in d["episodes"][lab] if r["seed"] in krows]
                        agree = sum(1 for r in wire if bool(r["success"]) == bool(krows[r["seed"]]["success"]))
                        ks = sum(1 for r in wire if krows[r["seed"]]["success"])
                        same = f"Kaggle in-process on the same {len(wire)} seeds: {ks}/{len(wire)} success, per-episode agreement {agree}/{len(wire)} (stochastic policy, CPU vs GPU: no parity expected)"
                        break
            lines.append(f"| {lab} ({d['policy']['name']}) | {b['n']} | **{b['success_rate']:.2f}** | [{ci[0]:.3f}, {ci[1]:.3f}] | {b['safety']:.2f} | {b['progress_mean']:.2f} | {b.get('latency_s_mean', 0):.1f} s | {e.get('requests', '')} | {e.get('wire_s_total', 0):.0f} s | {d['wall_s']/60:.0f} min | {same} |")
        lines += ["", "- **The split works: the oracle and hold suites reproduce the in-process rows field for field (the policy suite matches the Kaggle rate on the same 20 seeds, 3/20, but on different episodes — a flow-matching policy sampled on a CPU is not expected to reproduce GPU episodes, so parity is claimed for the deterministic suites only), a request round trip (Pi physics or render + transfer) costs ≈ 25 ms without rendering and ≈ 46 ms with, and in the policy suite that round trip is 0.9 % of the wall time — the ≈ 32 s CPU policy call is the whole budget. SDD question Q-A is answered.**", ""]
        p5_md = chr(10).join(lines) + chr(10)
        rows_html = ""
        for line in lines:
            if line.startswith("| ") and not line.startswith("|---"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                tag = "th" if cells[0].startswith("Suite") else "td"
                rows_html += "<tr>" + "".join(f"<{tag}>{bold(esc(c))}</{tag}>" for c in cells) + "</tr>"
        p5_html = "<h2>P5: the bed served over ZeroMQ</h2><div class=\"tablewrap\"><table>" + rows_html + "</table></div><div class=\"callout\">" + bold(esc(lines[-2])) + "</div>"
    v5_md, v5_html = "", ""
    if e5:
        lines = ["## 3g. Session G — baseline trained on v5a (v4 with expert noise 0.25×), frozen suite (transcribed)", "",
                 "| Suite (v5a checkpoint) | success | Wilson 95 % | safety | progress | rejected steps | cmd L∞ mean | v4 same suite |", "|---|---|---|---|---|---|---|---|"]
        v4ref = {}
        if e4:
            for su in e4["suites"]:
                v4ref[(su["label"].split("/")[-1], su["variation"], bool(su.get("blank_image")), su.get("gain", 1.0))] = su["success_rate"]
        for su in e5["suites"]:
            st = su["label"].split("/")[-1]; ci = su["ci95_wilson_success"]
            what = f"{int(st)/1000:g}k " + su["variation"] + (" + camera blanked" if su.get("blank_image") else "") + (f" + gain {su['gain']}" if su.get("gain", 1.0) != 1.0 else "")
            ref = v4ref.get((st, su["variation"], bool(su.get("blank_image")), su.get("gain", 1.0)))
            ref = f"{ref:.2f}" if isinstance(ref, float) else "not run"
            lines.append(f"| {what} | **{su['success_rate']:.2f}** | [{ci[0]:.3f}, {ci[1]:.3f}] | {su['safety']:.2f} | {su['progress_mean']:.2f} | {100*su['rejected_fraction_mean']:.0f} % | {su['cmd_xyz_linf_mean']:.4f} | {ref} |")
        sel5 = e5["selection"]
        lines += ["", f"Selected checkpoint (R11): **{int(sel5['selected'])/1000:g}k** at {sel5['success_rate']:.2f}; inside the best interval: {', '.join(f'{int(x)/1000:g}k' for x in sel5['within_ci'])}. Training: {e5['train_record']['steps_per_s']} steps/s, {e5['train_record']['wall_s']/3600:.2f} h.", "", "- **" + e5["reading"] + "**", ""]
        v5_md = chr(10).join(lines) + chr(10)
        rows_html = ""
        for line in lines:
            if line.startswith("| ") and not line.startswith("|---"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                tag = "th" if cells[0].startswith("Suite") else "td"
                rows_html += "<tr>" + "".join(f"<{tag}>{bold(esc(c))}</{tag}>" for c in cells) + "</tr>"
        v5_html = "<h2>Session G: baseline trained on v5a, same frozen suite</h2><div class=\"tablewrap\"><table>" + rows_html + "</table></div><p>" + bold(esc(lines[-4])) + "</p><div class=\"callout\">" + bold(esc("**" + e5["reading"] + "**")) + "</div>"
    v6_md, v6_html = "", ""
    if e6:
        lines = ["## 3h. Session K — baseline trained on v6 (v5a + wrist camera), frozen suite (imported)", "",
                 "| Suite (v6 checkpoint) | success | Wilson 95 % | safety | progress | rejected steps | cmd L∞ mean | v5a same suite |", "|---|---|---|---|---|---|---|---|"]
        v5ref = {}
        if e5:
            for su in e5["suites"]:
                v5ref[(su["label"].split("/")[-1], su["variation"], bool(su.get("blank_image")), su.get("gain", 1.0))] = su["success_rate"]
        for su in e6["suites"]:
            st = su["label"].split("/")[-1]; ci = su["ci95_wilson_success"]
            what = f"{int(st)/1000:g}k " + su["variation"] + (" + camera blanked" if su.get("blank_image") else "") + (f" + gain {su['gain']}" if su.get("gain", 1.0) != 1.0 else "")
            ref = v5ref.get((st, su["variation"], bool(su.get("blank_image")), su.get("gain", 1.0)))
            ref = f"{ref:.2f}" if isinstance(ref, float) else "not run"
            lines.append(f"| {what} | **{su['success_rate']:.2f}** | [{ci[0]:.3f}, {ci[1]:.3f}] | {su['safety']:.2f} | {su['progress_mean']:.2f} | {100*su['rejected_fraction_mean']:.0f} % | {su['cmd_xyz_linf_mean']:.4f} | {ref} |")
        sel5 = e6["selection"]
        lines += ["", f"Selected checkpoint (R11): **{int(sel5['selected'])/1000:g}k** at {sel5['success_rate']:.2f}; inside the best interval: {', '.join(f'{int(x)/1000:g}k' for x in sel5['within_ci'])}. Training: {e6['train_record']['steps_per_s']} steps/s, {e6['train_record']['wall_s']/3600:.2f} h.", "", "- **" + e6["reading"] + "**", ""]
        v6_md = chr(10).join(lines) + chr(10)
        rows_html = ""
        for line in lines:
            if line.startswith("| ") and not line.startswith("|---"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                tag = "th" if cells[0].startswith("Suite") else "td"
                rows_html += "<tr>" + "".join(f"<{tag}>{bold(esc(c))}</{tag}>" for c in cells) + "</tr>"
        v6_html = "<h2>Session K: baseline trained on v6 (wrist camera), same frozen suite</h2><div class=\"tablewrap\"><table>" + rows_html + "</table></div><p>" + bold(esc(lines[-4])) + "</p><div class=\"callout\">" + bold(esc("**" + e6["reading"] + "**")) + "</div>"
    v7_md, v7_html = "", ""
    if e7:
        lines = ["## 3i. Session J — plastic variant (VLM + vision encoder unfrozen, lr 2.5e-5, batch 8, float32) on v5a, frozen suite (imported)", "",
                 "| Suite (plastic checkpoint) | success | Wilson 95 % | safety | progress | rejected steps | cmd L∞ mean | v5a baseline same suite |", "|---|---|---|---|---|---|---|---|"]
        v5ref = {}
        if e5:
            for su in e5["suites"]:
                v5ref[(su["label"].split("/")[-1], su["variation"], bool(su.get("blank_image")), su.get("gain", 1.0))] = su["success_rate"]
        for su in e7["suites"]:
            st = su["label"].split("/")[-1]; ci = su["ci95_wilson_success"]
            what = f"{int(st)/1000:g}k " + su["variation"] + (" + camera blanked" if su.get("blank_image") else "") + (f" + gain {su['gain']}" if su.get("gain", 1.0) != 1.0 else "")
            ref = v5ref.get((st, su["variation"], bool(su.get("blank_image")), su.get("gain", 1.0)))
            ref = f"{ref:.2f}" if isinstance(ref, float) else "not run"
            lines.append(f"| {what} | **{su['success_rate']:.2f}** | [{ci[0]:.3f}, {ci[1]:.3f}] | {su['safety']:.2f} | {su['progress_mean']:.2f} | {100*su['rejected_fraction_mean']:.0f} % | {su['cmd_xyz_linf_mean']:.4f} | {ref} |")
        sel5 = e7["selection"]
        lines += ["", f"Selected checkpoint (R11): **{int(sel5['selected'])/1000:g}k** at {sel5['success_rate']:.2f}; inside the best interval: {', '.join(f'{int(x)/1000:g}k' for x in sel5['within_ci'])}. Training: {e7['train_record']['steps_per_s']} steps/s, {e7['train_record']['wall_s']/3600:.2f} h.", "", "- **" + e7["reading"] + "**", ""]
        v7_md = chr(10).join(lines) + chr(10)
        rows_html = ""
        for line in lines:
            if line.startswith("| ") and not line.startswith("|---"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                tag = "th" if cells[0].startswith("Suite") else "td"
                rows_html += "<tr>" + "".join(f"<{tag}>{bold(esc(c))}</{tag}>" for c in cells) + "</tr>"
        v7_html = "<h2>Session J: plastic variant on v5a, same frozen suite</h2><div class=\"tablewrap\"><table>" + rows_html + "</table></div><p>" + bold(esc(lines[-4])) + "</p><div class=\"callout\">" + bold(esc("**" + e7["reading"] + "**")) + "</div>"
    prec_md, prec_html = "", ""
    if precision_files:
        import precision as pz
        curves = [load(f) for f in precision_files]
        def pick(recipe, ckpt, variation="nominal", gain=1.0, blank=False):
            for c in curves:
                if c["recipe"] == recipe and c["label"].endswith(ckpt) and c["variation"] == variation and (c["gain"] or 1.0) == gain and bool(c["blank_image"]) == blank and c["n"] == 100:
                    return c
            return None
        series = []
        for rec, ck, col in (("v2", "007500", teal), ("v3", "005000", "#7A5C99"), ("v4", "005000", "#2E7D4F"), ("v5a", "007500", "#B7791F"), ("v6", "010000", "#B03A2E")):
            c = pick(rec, ck)
            if c: series.append((f"{rec} · {int(ck)/1000:g}k nominal", c["curve"], col))
        for rec, ck, var, col in (("v4", "010000", "camera_shift", "#6FB08A"), ("v4", "010000", "camera_shift_far", amber), ("v6", "010000", "camera_shift_far", "#E07B6E")):
            c = pick(rec, ck, var)
            if c: series.append((f"{rec} · {int(ck)/1000:g}k {var}", c["curve"], col))
        charts["precision"] = pz.line_chart("Success against the acceptance radius (same 100 seeds, Wilson 95 %)", series,
                                            note="Counts episodes that ever reached P; the suite's own success needs 5 consecutive frames within 0.03 m.")
        lines = ["## 3e. Precision curves — success against the acceptance radius (re-analysis of the committed rows, no GPU)", "",
                 "| Suite (n = 100) | suite success | P = 0.03 | 0.04 | 0.05 | 0.06 | 0.08 | 0.10 m | power-law r² |", "|---|---|---|---|---|---|---|---|---|"]
        for c in curves:
            if c["n"] != 100 or c["blank_image"]:
                continue
            by = {q["tolerance_m"]: q["success_rate"] for q in c["curve"]}
            what = f"{c['recipe']} {c['label'].split('/')[-1].lstrip('0') and int(c['label'].split('/')[-1])/1000:g}k {c['variation']}" + (f" gain {c['gain']}" if (c["gain"] or 1.0) != 1.0 else "") + (f" {c['post']}" if c["post"] not in (None, "none") else "")
            lines.append(f"| {what} | {c['suite_success_rate']:.2f} | **{by[0.03]:.2f}** | {by[0.04]:.2f} | {by[0.05]:.2f} | {by[0.06]:.2f} | {by[0.08]:.2f} | {by[0.10]:.2f} | {c['fit'].get('power_law', {}).get('r2', '')} |")
        lines += ["", "- **" + pz.CAVEAT + "**", "- **v6 (wrist camera) starts at 0.89 and saturates by 0.05 m: the misses that the single-camera recipes made by centimetres are gone, which is what the curve predicted the sensing lever would do.**", "- **Reading: every single-camera recipe's success climbs steeply with the radius (three- to fourfold from 0.03 to 0.06 m, ≈ 0.7 at 0.10 m), so the failures are misses by a few centimetres, not wrong directions — a precision ceiling, as the Curse of Precision (2607.23108) predicts for a single third-person RGB camera; camera_shift_far is the exception (flatter: directional errors). The 1/(P − c) form never beats the plain power law on this range (c pins to 0), so these eight points describe the curve without establishing a law.**", ""]
        prec_md = chr(10).join(lines) + chr(10)
        rows_html = ""
        for line in lines:
            if line.startswith("| ") and not line.startswith("|---"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                tag = "th" if cells[0].startswith("Suite") else "td"
                rows_html += "<tr>" + "".join(f"<{tag}>{bold(esc(c))}</{tag}>" for c in cells) + "</tr>"
        prec_html = "<h2>Precision curves: success against the acceptance radius</h2><div class=\"tablewrap\"><table>" + rows_html + "</table></div><div class=\"callout\">" + bold(esc(lines[-2])) + "</div>"
    steps = pol["n"] * pol["episode_len_mean"]
    rej = pol["faults"]["rejected"]
    charts["safety"] = hbar_chart("Commands rejected by the safety wrapper, per 100 policy steps (baseline, 10k)",
                                  [("S2 translation > 1 cm/step", 100 * rej["S2_xyz_step"] / steps, None, red),
                                   ("S3 rotation > 0.05 rad/step", 100 * rej["S3_rpy_step"] / steps, None, red),
                                   ("S4 leaves the workspace box", 100 * rej["S4_workspace"] / steps, None, red),
                                   ("Scripted oracle (any)", 0.0, None, grey)],
                                  xmax=40, unit="", note=f"{int(steps):,} policy steps over 100 episodes; a rejected command holds the arm for that step. Safety 0.01, SBU 0.12, VSI 0.18.")
    charts["kaggle-timings"] = hbar_chart("Training speed on Kaggle's free Tesla T4 (10-step smoke, batch 32 unless noted)",
                                          [("bfloat16 as LeRobot ships it", sm["trials"]["bf16-b32"]["steps_per_s"], None, grey),
                                           ("bfloat16, batch 16", sm["trials"]["bf16-b16"]["steps_per_s"], None, grey),
                                           ("float16 VLM cast (chosen)", sm["trials"]["fp16-b32"]["steps_per_s"], None, teal),
                                           ("full run, 10k steps (measured)", tr["steps_per_s"], None, "#5FB0B7")],
                                          xmax=1.0, unit=" st/s", note=f"VRAM 5.9 / 3.6 / 4.9 GB. Full run: {tr['steps_done']:,} steps in {tr['wall_s']/3600:.2f} h.")
    rows = []
    for e in p3["episodes"]:
        rows.append((f"ep {e['episode_index']} {e['task'][:22]}… state", e["state_mode"]["L_transl_m"] * 100, None, teal))
        rows.append((f"   action replay, gain 0.61", e["action_mode_measured_gain"]["L_transl_m"] * 100, None, "#5FB0B7"))
        rows.append((f"   action replay, unit gain", e["action_mode_unit_gain"]["L_transl_m"] * 100, None, grey))
    charts["p3-tracking"] = hbar_chart("Real UR5 episodes replayed on the sim UR5e — mean end-effector error (cm)", rows, xmax=35, unit=" cm",
                                       note="State replay ≤ 1.2 cm mean, ≤ 0.4 cm final; open-loop command replay drifts because the human closed the loop.", width=720)
    charts["libero-per-task"] = hbar_chart("LIBERO-Spatial calibration: successes per task (5 episodes each)",
                                           [(f"task {i}", k, None, amber if k else red) for i, k in enumerate(libero_per_task)], xmax=5, unit="")
    for name, svg in charts.items():
        (OUT / f"{name}.svg").write_text(svg + "\n")

    g_sr = gain["success_rate"] if gain else float("nan"); g_pr = gain["progress_mean"] if gain else float("nan"); g_sf = gain["safety"] if gain else float("nan")
    g_s4 = gain["faults"]["rejected"].get("S4_workspace", 0) if gain else 0; g_s3 = gain["faults"]["rejected"].get("S3_rpy_step", 0) if gain else 0
    g_lo, g_hi = tuple(gain["ci95_wilson_success"]) if gain else (float("nan"), float("nan"))
    curve_txt = "; ".join(f"{st/1000:g}k: {v['success_rate']:.2f} (progress {v['progress_mean']:.2f}, safety {v['safety']:.2f})" for st, v in curve)
    curve_ci = "; ".join(f"{st/1000:g}k [{v['ci95_wilson_success'][0]:.3f}, {v['ci95_wilson_success'][1]:.3f}]" for st, v in curve)
    c_best = max(curve, key=lambda t: t[1]["success_rate"])
    curve_bullet = ""
    if len(curve) >= 2 and c_best[0] != final_steps:
        cb = c_best[1]
        curve_bullet = (f"- **An earlier checkpoint scores higher.** The {c_best[0]/1000:g}k checkpoint reaches {100*cb['success_rate']:.0f} % "
                        f"[{100*cb['ci95_wilson_success'][0]:.0f}–{100*cb['ci95_wilson_success'][1]:.0f} %] against the final checkpoint's {100*pol['success_rate']:.0f} %; the intervals overlap, so this is not a proven "
                        f"decline, but the R11 rule picks by closed-loop success and would select {c_best[0]/1000:g}k today. Both checkpoints reject about a third of their "
                        f"steps at S2, so the cap problem is not a late-training artefact.")
    if prog and prog.get("finished"):
        prog_md = ("## 0. Evaluation run: finished" + chr(10) + chr(10) + prog["notebook"] + " finished at " + prog["finished"] + " after " + str(prog["elapsed_h"]) + " h · "
                   + str(len(prog["done"])) + " suites ran; cut by the 7.5 h budget: " + ", ".join(prog["queued"]) + chr(10) + chr(10)
                   + chr(10).join("- done: " + x for x in prog["done"]) + chr(10) + "- output: " + prog["output"] + chr(10))
        prog_html = ("<div class=\"callout\"><strong>Evaluation finished</strong> " + esc(prog["finished"]) + " after " + esc(str(prog["elapsed_h"])) + " h · " + esc(str(len(prog["done"])))
                     + " suites ran · cut by the budget: " + esc("; ".join(prog["queued"])) + "<br>done: " + esc("; ".join(prog["done"])) + "<br>output: " + esc(prog["output"]) + "</div>")
        status_txt = ("Phase 4 baseline trained on a free Kaggle T4 and evaluated closed-loop: " + str(len(prog["done"])) + " of " + str(len(prog["done"]) + len(prog["queued"]))
                      + " suites ran inside the 7.5 h budget (" + " and ".join(q.split(" ", 1)[1] for q in prog["queued"]) + " deferred to the next session); packed JSONs not yet imported.")
        chips_extra = ("<span class=\"chip\">evaluation <b>complete · " + esc(str(len(prog["done"]))) + " of " + esc(str(len(prog["done"]) + len(prog["queued"]))) + " suites</b></span>"
                       + ("<span class=\"chip\">selected checkpoint <b>" + esc(f"{int(sel['selected'])/1000:g}k · {sel['success_rate']:.2f}") + "</b></span>" if sel else ""))
    elif prog:
        prog_md = ("## 0. Live progress of the evaluation run" + chr(10) + chr(10) + "Updated " + prog["updated"] + " · " + prog["notebook"] + " · started " + prog["started"] + " · " + str(prog["elapsed_h"])
                   + " h elapsed · **ETA " + prog["eta"] + "**" + chr(10) + chr(10) + chr(10).join("- done: " + x for x in prog["done"]) + chr(10) + "- running: " + prog["running"] + chr(10)
                   + chr(10).join("- queued: " + x for x in prog["queued"]) + chr(10) + chr(10).join("- expected: " + k + " at " + v for k, v in prog.get("milestones", {}).items()) + chr(10))
        prog_html = ("<div class=\"callout\"><strong>Live progress</strong> (updated " + esc(prog["updated"]) + "): " + esc(str(len(prog["done"]))) + " of " + esc(str(len(prog["done"]) + 1 + len(prog["queued"])))
                     + " suites done · running <b>" + esc(prog["running"]) + "</b> · <b>ETA " + esc(prog["eta"]) + "</b><br>done: " + esc("; ".join(prog["done"])) + "<br>queued: " + esc("; ".join(prog["queued"]))
                     + ("<br>expected: " + esc("; ".join(k + " at " + v for k, v in prog["milestones"].items())) if prog.get("milestones") else "") + "</div>")
        status_txt = "Phase 4 baseline trained on a free Kaggle T4 and its evaluation in progress (learning curve and variations pending)."
        chips_extra = "<span class=\"chip warn\">evaluation <b>still running</b></span>"
    else:
        prog_md = prog_html = chips_extra = ""; status_txt = "Phase 4 baseline trained on a free Kaggle T4."
    cam_row = ""; cam_bullet = ""
    if cam:
        cr = cam["faults"]["rejected"]; cm = cam["faults"]["measured"].get("S6_self_collision", 0)
        cam_row = (f"| **Variation camera_shift, 10k checkpoint, n = 100** | **success {cam['success_rate']:.2f}**, progress {cam['progress_mean']:.2f}, VSI {cam['vsi']:.2f}, {cm} self-collision steps measured, rejections S2 {cr.get('S2_xyz_step', 0)} / S3 {cr.get('S3_rpy_step', 0)} / S4 {cr.get('S4_workspace', 0)} "
                   f"| Wilson [{cam['ci95_wilson_success'][0]:.3f}, {cam['ci95_wilson_success'][1]:.3f}] | Kaggle `vla-bed-eval` v2 |" + chr(10))
        cam_bullet = (f"- **A shifted camera breaks it.** With the camera moved, success falls from {100*pol['success_rate']:.0f} % to {100*cam['success_rate']:.0f} % [{100*cam['ci95_wilson_success'][0]:.0f}–{100*cam['ci95_wilson_success'][1]:.0f} %], progress from {pol['progress_mean']:.2f} to {cam['progress_mean']:.2f}, "
                      f"and the arm self-collides in {cm} steps where the nominal suite had none; the wrapper rejects far more workspace exits (S4 {cr.get('S4_workspace', 0)} vs {rej['S4_workspace']}). "
                      f"A policy trained on one camera pose learned the picture, not the geometry — the diversity rule (R10) applies before any more demonstrations are recorded.")
    sel_row = (f"| **Checkpoint selection (R11: by closed-loop success)** | selected **{int(sel['selected'])/1000:g}k** at {sel['success_rate']:.2f}; checkpoints inside the best interval: {', '.join(f'{int(x)/1000:g}k' for x in sel['within_ci'])} | — | Kaggle `vla-bed-eval` v2 |" + chr(10)) if sel else ""
    b_sr = blank["success_rate"] if blank else float("nan"); b_pr = blank["progress_mean"] if blank else float("nan"); b_hi = blank["ci95_wilson_success"][1] if blank else float("nan")
    probes_md, probes_html = "", ""
    if pr:
        lines = ["## 3a. Session A — probes on the existing checkpoints (Kaggle, transcribed)", "",
                 f"Suites now take {min(pr['profile']['suite_100ep_minutes']):.0f}–{max(pr['profile']['suite_100ep_minutes']):.0f} min per 100 episodes (was {pr['profile']['previous_suite_minutes']}): the camera is rendered only when the policy is queried and two workers share the seeds.", "",
                 "| Magnitude probe (open loop, 500 training frames) | pred / label L∞ | labels on the cap | predicted over the cap | given label on cap | direction cosine |", "|---|---|---|---|---|---|"]
        for st, m in pr["magnitude_probe"].items():
            lines.append(f"| {int(st)/1000:g}k | {m['ratio_pred_over_label_xyz_linf']} | {100*m['label_at_cap_fraction']:.0f} % | {100*m['pred_over_cap_fraction']:.0f} % | {100*m['pred_over_cap_given_label_at_cap']:.0f} % | {m['direction_cosine_xyz_mean']} |")
        lines += ["", "| Closed-loop suite (7.5k unless stated) | success | Wilson 95 % | safety | rejected steps | cmd L∞ mean | progress |", "|---|---|---|---|---|---|---|"]
        for su in pr["suites"]:
            what = su["variation"] + (" + clip" if su["post"] == "clip" else "") + (f" + ensemble/{su.get('replan_every')}" if su["post"] == "ensemble" else "") + (f" + gain {su['gain']}" if su["gain"] != 1.0 else "") + (f" + VLM {su['vlm_dtype']}" if su.get("vlm_dtype") else "") + ("" if su["label"].endswith("007500") else f" ({int(su['label'].split('/')[-1])/1000:g}k)")
            ci = su["ci95_wilson_success"]
            lines.append(f"| {what}, n = {su['n']} | **{su['success_rate']:.2f}** | [{ci[0]:.3f}, {ci[1]:.3f}] | {su['safety']:.2f} | {100*su['rejected_fraction_mean']:.0f} % | {su['cmd_xyz_linf_mean']:.4f} | {su['progress_mean']:.2f} |")
        lines += ["", "| Paired on the same 100 seeds | A → B | diff [paired bootstrap 95 %] | discordant (A fail/B ok vs A ok/B fail) | McNemar p | verdict |", "|---|---|---|---|---|---|"]
        for c in pr["paired"]:
            lines.append(f"| {c['a']} vs {c['b']} | {c['success_a']:.2f} → {c['success_b']:.2f} | {c['diff']:+.2f} [{c['diff_ci95'][0]:+.2f}, {c['diff_ci95'][1]:+.2f}] | {c['discordant'][0]} vs {c['discordant'][1]} | {c['mcnemar_p']:.3g} | {c['verdict']} |")
        lines += ["", "- **" + pr["reading"] + "**", ""]
        probes_md = chr(10).join(lines) + chr(10)
        rows_html = ""
        for line in lines:
            if line.startswith("| ") and not line.startswith("|---"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                tag = "th" if cells[0].startswith(("Magnitude probe", "Closed-loop suite", "Paired on")) else "td"
                rows_html += "<tr>" + "".join(f"<{tag}>{bold(esc(c))}</{tag}>" for c in cells) + "</tr>"
            elif line == "" and rows_html and not rows_html.endswith("</td></tr>" * 0 + "<tr><td colspan=\"7\"></td></tr>"):
                rows_html += "<tr><td colspan=\"7\"></td></tr>"
        probes_html = "<h2>Session A: probes on the existing checkpoints</h2><p>" + esc(lines[2]) + "</p><div class=\"tablewrap\"><table>" + rows_html + "</table></div><div class=\"callout\">" + bold(esc("**" + pr["reading"] + "**")) + "</div>"
    v3_md, v3_html = "", ""
    if e3:
        lines = ["## 3c. Session C — baseline trained on v3 (headroom 0.7 + camera jitter), frozen suite (transcribed)", "",
                 "| Suite (v3 checkpoint) | success | Wilson 95 % | safety | progress | rejected steps | cmd L∞ mean | v2 same suite |", "|---|---|---|---|---|---|---|---|"]
        v2ref = {("010000", "nominal", False, 1.0): kg["nominal_010000_100ep"]["success_rate"], ("007500", "nominal", False, 1.0): kg["nominal_007500_100ep"]["success_rate"], ("005000", "nominal", False, 1.0): kg["nominal_005000_100ep"]["success_rate"], ("002500", "nominal", False, 1.0): kg["nominal_002500_100ep"]["success_rate"], ("010000", "nominal", True, 1.0): kg["blank_image_010000_100ep"]["success_rate"], ("010000", "nominal", False, 0.61): kg["gain061_010000_100ep"]["success_rate"], ("010000", "camera_shift", False, 1.0): kg["camera_shift_010000_100ep"]["success_rate"]}
        if pr:
            for su in pr["suites"]:
                if su["variation"] in ("lighting", "target_relocation") and su["post"] == "none":
                    v2ref[("010000", su["variation"], False, 1.0)] = f"{su['success_rate']:.2f} (7.5k)"
        for su in e3["suites"]:
            st = su["label"].split("/")[-1]; ci = su["ci95_wilson_success"]
            what = f"{int(st)/1000:g}k " + su["variation"] + (" + camera blanked" if su.get("blank_image") else "") + (f" + gain {su['gain']}" if su.get("gain", 1.0) != 1.0 else "")
            ref = v2ref.get((st, su["variation"], bool(su.get("blank_image")), su.get("gain", 1.0)), "")
            ref = f"{ref:.2f}" if isinstance(ref, float) else ref
            lines.append(f"| {what} | **{su['success_rate']:.2f}** | [{ci[0]:.3f}, {ci[1]:.3f}] | {su['safety']:.2f} | {su['progress_mean']:.2f} | {100*su['rejected_fraction_mean']:.0f} % | {su['cmd_xyz_linf_mean']:.4f} | {ref} |")
        sel = e3["selection"]
        lines += ["", f"Selected checkpoint (R11): **{int(sel['selected'])/1000:g}k** at {sel['success_rate']:.2f}; inside the best interval: {', '.join(f'{int(x)/1000:g}k' for x in sel['within_ci'])}. Training: {e3['train_record']['steps_per_s']} steps/s, {e3['train_record']['wall_s']/3600:.2f} h.", "", "- **" + e3["reading"] + "**", ""]
        v3_md = chr(10).join(lines) + chr(10)
        rows_html = ""
        for line in lines:
            if line.startswith("| ") and not line.startswith("|---"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                tag = "th" if cells[0].startswith("Suite") else "td"
                rows_html += "<tr>" + "".join(f"<{tag}>{bold(esc(c))}</{tag}>" for c in cells) + "</tr>"
        v3_html = "<h2>Session C: baseline trained on v3, same frozen suite</h2><div class=\"tablewrap\"><table>" + rows_html + "</table></div><p>" + bold(esc(lines[-4])) + "</p><div class=\"callout\">" + bold(esc("**" + e3["reading"] + "**")) + "</div>"
    v4_md, v4_html = "", ""
    if e4:
        lines = ["## 3d. Session E — baseline trained on v4 (v3 + camera translation jitter), frozen suite (transcribed)", "",
                 "| Suite (v4 checkpoint) | success | Wilson 95 % | safety | progress | rejected steps | cmd L∞ mean | v3 same suite |", "|---|---|---|---|---|---|---|---|"]
        v3ref = {}
        if e3:
            for su in e3["suites"]:
                v3ref[(su["label"].split("/")[-1], su["variation"], bool(su.get("blank_image")), su.get("gain", 1.0))] = su["success_rate"]
        for su in e4["suites"]:
            st = su["label"].split("/")[-1]; ci = su["ci95_wilson_success"]
            what = f"{int(st)/1000:g}k " + su["variation"] + (" + camera blanked" if su.get("blank_image") else "") + (f" + gain {su['gain']}" if su.get("gain", 1.0) != 1.0 else "")
            ref = v3ref.get((st, su["variation"], bool(su.get("blank_image")), su.get("gain", 1.0)))
            ref = f"{ref:.2f}" if isinstance(ref, float) else "not run"
            lines.append(f"| {what} | **{su['success_rate']:.2f}** | [{ci[0]:.3f}, {ci[1]:.3f}] | {su['safety']:.2f} | {su['progress_mean']:.2f} | {100*su['rejected_fraction_mean']:.0f} % | {su['cmd_xyz_linf_mean']:.4f} | {ref} |")
        sel4 = e4["selection"]
        lines += ["", f"Selected checkpoint (R11): **{int(sel4['selected'])/1000:g}k** at {sel4['success_rate']:.2f}; inside the best interval: {', '.join(f'{int(x)/1000:g}k' for x in sel4['within_ci'])}. Training: {e4['train_record']['steps_per_s']} steps/s, {e4['train_record']['wall_s']/3600:.2f} h; evaluation {e4['session_wall_s']/3600:.2f} h.", "", "- **" + e4["reading"] + "**", ""]
        v4_md = chr(10).join(lines) + chr(10)
        rows_html = ""
        for line in lines:
            if line.startswith("| ") and not line.startswith("|---"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                tag = "th" if cells[0].startswith("Suite") else "td"
                rows_html += "<tr>" + "".join(f"<{tag}>{bold(esc(c))}</{tag}>" for c in cells) + "</tr>"
        v4_html = "<h2>Session E: baseline trained on v4, same frozen suite</h2><div class=\"tablewrap\"><table>" + rows_html + "</table></div><p>" + bold(esc(lines[-4])) + "</p><div class=\"callout\">" + bold(esc("**" + e4["reading"] + "**")) + "</div>"
    imported_md, imported_html = "", ""
    if magnitudes or compares or v2rows:
        lines = ["## 3b. Imported Kaggle probe results (session A, per-episode files)", ""]
        if v2rows:
            lines += ["| Suite | success | Wilson 95 % | rejected steps | cmd L∞ mean (m) | over-cap steps | progress |", "|---|---|---|---|---|---|---|"]
            for f in v2rows:
                d = load(f); lab = d.get("policy", {}).get("label")
                if not lab or lab not in d or "rejected_fraction_mean" not in d[lab]:
                    continue
                b = d[lab]; ci = b["ci95_wilson_success"]
                cmd = "" if b["cmd_xyz_linf_mean"] is None else f"{b['cmd_xyz_linf_mean']:.4f}"
                over = "" if b["cmd_xyz_over_cap_fraction_mean"] is None else f"{100 * b['cmd_xyz_over_cap_fraction_mean']:.0f} %"
                lines.append(f"| {lab} {f.stem} | {b['success_rate']:.2f} | [{ci[0]:.3f}, {ci[1]:.3f}] | {100 * b['rejected_fraction_mean']:.0f} % | {cmd} | {over} | {b['progress_mean']:.2f} |")
            lines.append("")
        if magnitudes:
            lines += ["| Magnitude probe (open loop, training frames) | rows | pred / label L∞ | labels on the cap | predicted over the cap | given label on cap | direction cosine |", "|---|---|---|---|---|---|---|"]
            for f in magnitudes:
                d = load(f); msum = d["summary"]; x = msum["xyz_linf"]
                lines.append(f"| {f.parent.parent.name}/{f.parent.name} | {msum['rows']} | {x['ratio_pred_over_label']} | {100*x['label_at_cap_fraction']:.0f} % | {100*x['pred_over_cap_fraction']:.0f} % | {100*(x['pred_over_cap_fraction_given_label_at_cap'] or 0):.0f} % | {msum['direction_cosine_xyz_mean']} |")
            lines.append("")
        if compares:
            lines += ["Files under `results/p5/8ac6124fd05b` are the v2 baseline (eval v2), `7a90b7940018` the v2 probe session, `a85e64a183e5` the v2 far-shift probe, `9f5dbf4dd492` the v3 baseline, `06f90d52039b` the v3 far-shift probe, `145880075d6f` the v4 baseline, `8af08da6f6a2` the v5a baseline, `bd883c780fba` the v6 (wrist camera) baseline; names of the form `vX … vs vY …` pair two datasets on identical seeds.", "", "| Paired comparison (same 100 seeds) | A → B success | diff [paired bootstrap 95 %] | discordant (A fail/B ok vs A ok/B fail) | McNemar p | verdict |", "|---|---|---|---|---|---|"]
            for f in compares:
                d = load(f); dd = d["discordant"]; ci = d["success_diff_ci95_paired_bootstrap"]
                name = f.stem.replace("compare_", "").replace("_", " ")
                lines.append(f"| {name} ({d['a']['label']} {d['a']['variation']} g{d['a']['gain']} → {d['b']['label']} {d['b']['variation']} g{d['b']['gain']}) | {d['success_a']:.2f} → {d['success_b']:.2f} | {d['success_diff_b_minus_a']:+.2f} [{ci[0]:+.2f}, {ci[1]:+.2f}] | {dd['a_fail_b_success']} vs {dd['a_success_b_fail']} | {d['mcnemar_exact_p']:.3g} | {d['verdict']} |")
            lines.append("")
        imported_md = chr(10).join(lines) + chr(10)
        rows_html = ""
        for line in lines:
            if line.startswith("| ") and not line.startswith("|---"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                tag = "th" if line == lines[2] or cells[0].startswith(("Suite", "Magnitude probe", "Paired comparison")) else "td"
                rows_html += "<tr>" + "".join(f"<{tag}>{esc(c)}</{tag}>" for c in cells) + "</tr>"
            elif line == "" and rows_html:
                rows_html += "<tr><td colspan=\"7\"></td></tr>"
        imported_html = "<h2>Imported Kaggle probe results</h2><div class=\"tablewrap\"><table>" + rows_html + "</table></div>"
    md = f"""# UR5e VLA bed — experiment report ({a.date})

Status of the experiment branch `experiment/ur5e-vla-bed` at report time: Phases 0–3 verified,
LIBERO calibration done, {status_txt} **Money spent: $0.00.**
Kaggle GPU quota used ≈ 19 h in the week of 29 Aug (smokes, v2 training, three evaluation/probe sessions, v3 training) and ≈ 4.6 h in the week of 5 Sep (v4 training 3.0 h, evaluation 1.6 h); weekly cap 30 h.
Every number below comes from a committed JSON under `results/` or from the Kaggle logs
transcribed in `results/p5/kaggle/eval-v2-partial.json`.

{prog_md}
## 1. What was measured, in one table

| Measurement | Result | Interval / uncertainty | Where |
|---|---|---|---|
| Scripted expert on the 100 held-out seeds (control) | {oracle['success_rate']:.2f} success, {oracle['episode_len_mean']:.1f} frames, zero faults | Wilson [{oracle['ci95_wilson_success'][0]:.3f}, {oracle['ci95_wilson_success'][1]:.2f}] | `results/p5/santapong/oracle/` |
| Do-nothing policy (control) | {hold['success_rate']:.2f} | [{hold['ci95_wilson_success'][0]:.2f}, {hold['ci95_wilson_success'][1]:.3f}] | `results/p5/santapong/hold/` |
| Same two controls on the Raspberry Pi | identical episode rows (0 mismatches over 200 episodes) | — | `results/p5/santapong-dev/` |
| LIBERO-Spatial calibration, `lerobot/smolvla_libero`, 10 × 5 episodes, CPU | **{lib_k}/{lib_n} = {lib_k/lib_n:.2f}**; per task {', '.join(str(k) for k in libero_per_task)} of 5; 7.0 h | Wilson [{lib_ci[0]:.3f}, {lib_ci[1]:.3f}]; published 0.90 inside | `results/p2b/full/eval_info.json` |
| SmolVLA-base inference on this CPU | {med if med else '≈36'} s per 50-action chunk, 3.5 GB peak RSS | — | `results/p2/santapong/cpu_bench.json` |
| Kaggle T4 training speed (smoke) | float16 VLM **{sm['trials']['fp16-b32']['steps_per_s']}** steps/s vs bfloat16 {sm['trials']['bf16-b32']['steps_per_s']} | 10 steps each | `results/p5/kaggle/eval-v2-partial.json` |
| Baseline training (`baseline`, 10k steps, batch 32) | {tr['steps_per_s']} steps/s, {tr['wall_s']/3600:.2f} h, {tr['peak_vram_gb']} GB VRAM, 4 checkpoints | — | Kaggle `vla-bed-train` v1 |
| **Baseline checkpoint 10k, nominal, n = 100** | **success {pol['success_rate']:.2f}**, progress {pol['progress_mean']:.2f}, safety {pol['safety']:.2f}, SBU {pol['sbu']:.2f}, VSI {pol['vsi']:.2f}, {pol['episode_len_mean']:.1f} frames | Wilson [{pol['ci95_wilson_success'][0]:.3f}, {pol['ci95_wilson_success'][1]:.3f}] | Kaggle `vla-bed-eval` v2 |
| Same checkpoint, quick check, n = 50 | success {quick['success_rate']:.2f}, progress {quick['progress_mean']:.2f} | [{quick['ci95_wilson_success'][0]:.3f}, {quick['ci95_wilson_success'][1]:.3f}] | Kaggle `vla-bed-train` v1 |
| **Learning curve, nominal, n = 100 per checkpoint** | {curve_txt} | Wilson {curve_ci} | Kaggle `vla-bed-eval` v2 |
| **Gain probe: same checkpoint, every command × 0.61, n = 100** | **success {g_sr:.2f}**, progress {g_pr:.2f}, safety {g_sf:.2f}, rejections S4 {g_s4} / S3 {g_s3} / S2 0 | Wilson [{g_lo:.3f}, {g_hi:.3f}] | Kaggle `vla-bed-eval` v2 |
| **Vision probe: same checkpoint, camera blanked, n = 100** | success {b_sr:.2f}, progress {b_pr:.2f}, all episodes time out; 66 rejections | Wilson [0, {b_hi:.3f}] | Kaggle `vla-bed-eval` v2 |
{cam_row}{sel_row}| Rejected commands, baseline 10k (per 100 policy steps) | S2 {100*rej['S2_xyz_step']/steps:.1f}, S3 {100*rej['S3_rpy_step']/steps:.1f}, S4 {100*rej['S4_workspace']/steps:.1f} | over {int(steps):,} steps | same |
| Real UR5 replay on the sim UR5e (5 OXE episodes) | state tracking 0.7–1.2 cm mean, ≤ 0.4 cm final; command replay 2.2–9.3 cm with the measured 0.61 gain | alignment Rz 90° + 0.284 m | `results/p3/santapong/summary.json` |
| Datasets recorded | v2: {v2['train']['episodes']} train / {v2['evaluation']['episodes']} held-out episodes, {v2['train']['frames']:,} + {v2['evaluation']['frames']:,} frames, 100 % expert success | — | `results/p2/santapong/` |

## 2. Charts

![success rates](report/success-rates.svg)

![per family](report/per-family.svg)

![safety rejections](report/safety.svg)

![kaggle timings](report/kaggle-timings.svg)

![OXE replay tracking](report/p3-tracking.svg)

![LIBERO per task](report/libero-per-task.svg)
![Learning curve, v2 baseline](report/learning-curve.svg)
![Learning curve: v2 vs v3 vs v4 on the same seeds](report/learning-curve-v2-vs-v3.svg)
![Viewpoint robustness: nominal vs shifted camera per recipe](report/viewpoint.svg)
![Success against the acceptance radius](report/precision.svg)

## 3. Reading the results

- **The pipeline is closed.** Record on the workstation → fine-tune SmolVLA on a free T4 →
  evaluate every step through the bed's own controller and safety wrapper, with the
  scripted expert at 100 % and the do-nothing policy at 0 % as controls. That is SDD Q-B,
  answered without spending money.
- **The baseline learns, but not much yet.** 13 % success on the 100 held-out seeds after
  10k steps (interval 8–21 %), progress 0.30, best on near targets (25 %), worst on the
  lateral cells (5 %). Twelve of the thirteen successes touched the safety wrapper on the way.
{curve_bullet}
- **The safety wrapper is doing real work.** About a third of the policy's commands exceed
  the 1 cm-per-step limit the demonstrations respected; the arm holds for those steps. The
  training labels sit exactly on that limit (84 % of label steps saturate at 0.010 m per axis,
  audit of 4 Sep 2026), so a regressor's symmetric spread is rejected one-sidedly; the next
  dataset (v3) records with headroom below the limits.
- **The two probes explain the number.** Blank the camera and success falls to 0 of 100 (the policy
  uses vision, R6). Execute every command at 61 % of what the policy asks — the real UR5's measured
  realisation — and success doubles to 28 % with 86 % safe episodes and no per-step rejections: the
  wrapper's holds were the bottleneck (labels on the cap, so any spread crosses it), and the
  controller-amplification effect of R11 is visible. Headroom in the data is the next change.
{cam_bullet}
- **Repeated suites of one checkpoint differ.** 26 % on a 50-episode check versus 13 % on the
  100-episode suite: SmolVLA samples its action chunk from noise, so a suite is a sample of
  the policy, not a deterministic property. The evaluator now seeds that noise per episode.
- **The evaluator is calibrated.** On LIBERO-Spatial it reproduces the published SmolVLA score
  within its interval (82 % measured, 90 published), so the low bed number is a property of
  the trained policy, not of the measuring instrument.

{probes_md}{v3_md}{v4_md}{v5_md}{v6_md}{v7_md}{prec_md}{p5_md}{imported_md}## 4. Pending when this report was written

{chr(10).join('- ' + p for p in kg['pending'])}

Each suite is 100 episodes ≈ 1.1 h on Kaggle's CPU; the run was bounded at 7.5 h and
dropped the tail. The next report regenerates from the packed JSONs once they are imported
(`gpu/kaggle_import.sh`).

Generated by `sim/vla-bed/report.py` from the committed results.
"""
    (R / f"REPORT-{a.date}.md").write_text(md)
    print("wrote", R / f"REPORT-{a.date}.md", "and", len(charts), "charts in", OUT)

    if a.html:
        blocks = "".join(f'<figure>{svg}</figure>' for svg in charts.values())
        rows_html = ""
        for line in md.split("## 1. What was measured, in one table")[1].split("## 2. Charts")[0].strip().splitlines():
            if line.startswith("| ") and not line.startswith("|---") and not line.startswith("| Measurement"):
                cells = [c.strip() for c in line.strip("|").split("|")]
                rows_html += "<tr>" + "".join("<td>" + bold(esc(c).replace("`", "")) + "</td>" for c in cells) + "</tr>"
        reading = md.split("## 3. Reading the results")[1].split("## 4.")[0]
        items = "".join("<li>" + bold(esc(p.strip("- ").replace(chr(10), " "))) + "</li>" for p in reading.strip().split("\n- ") if p.strip())
        pending = "".join(f"<li>{esc(p)}</li>" for p in kg["pending"])
        html = f"""<title>UR5e Bed Experiment Report</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans+Condensed:wght@600;700&family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
:root {{ --bg:#F2F4F6; --surface:#FFFFFF; --line:#D3DAE1; --text:#1A232C; --muted:#5A6874; --accent:#0E7C86; --accent-ink:#0A5C64; --warn:#9A6A12; --warn-soft:#F6ECD3; --display:'IBM Plex Sans Condensed',"Arial Narrow",Arial,sans-serif; --body:'IBM Plex Sans',Helvetica,Arial,sans-serif; --mono:'IBM Plex Mono',ui-monospace,Menlo,monospace; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#0F1418; --surface:#171E24; --line:#2C3740; --text:#E4EAEF; --muted:#98A7B3; --accent:#3FB6C0; --accent-ink:#7ED4DB; --warn:#D9A64A; --warn-soft:#3A2E14; }} }}
:root[data-theme="dark"] {{ --bg:#0F1418; --surface:#171E24; --line:#2C3740; --text:#E4EAEF; --muted:#98A7B3; --accent:#3FB6C0; --accent-ink:#7ED4DB; --warn:#D9A64A; --warn-soft:#3A2E14; }}
*{{box-sizing:border-box}} body{{margin:0;background:var(--bg);color:var(--text);font-family:var(--body);font-size:16px;line-height:1.55}}
main{{max-width:900px;margin:0 auto;padding:32px 24px 72px}} h1,h2{{font-family:var(--display);text-wrap:balance;margin:0}} h1{{font-size:2.3rem}} h2{{font-size:1.4rem;margin-top:44px;padding-top:16px;border-top:1px solid var(--line)}}
.eyebrow{{font-family:var(--mono);font-size:.75rem;letter-spacing:.12em;text-transform:uppercase;color:var(--muted)}} .lede{{color:var(--muted);max-width:68ch}}
.chips{{display:flex;flex-wrap:wrap;gap:10px;margin:18px 0}} .chip{{font-family:var(--mono);font-size:.8rem;padding:6px 10px;border:1px solid var(--line);background:var(--surface);border-radius:4px}} .chip b{{font-weight:500;color:var(--accent-ink)}} .chip.warn{{border-color:var(--warn);background:var(--warn-soft)}}
.tablewrap{{overflow-x:auto;border:1px solid var(--line);background:var(--surface);margin:16px 0}} table{{border-collapse:collapse;width:100%;font-size:.9rem}} th,td{{text-align:left;padding:9px 11px;border-bottom:1px solid var(--line);vertical-align:top}} th{{font-family:var(--mono);font-size:.72rem;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);font-weight:500}} tr:last-child td{{border-bottom:0}}
figure{{margin:18px 0;background:#FFFFFF;border:1px solid var(--line);padding:6px;overflow-x:auto}} figure svg{{max-width:100%;height:auto;display:block}}
ul{{max-width:72ch;padding-left:20px}} li{{margin:8px 0}} .callout{{border-left:4px solid var(--accent);background:var(--surface);padding:12px 16px;margin:16px 0}}
</style>
<main>
<div class="eyebrow">RoboLLM · sim/vla-bed · experiment/ur5e-vla-bed · {a.date}</div>
<h1>UR5e Bed Experiment Report</h1>
<p class="lede">Everything measured so far on the UR5e VLA sim bed: the calibration of the evaluator, the real-to-sim bridge, the free-GPU fine-tune of SmolVLA, and the baseline policy's first evaluation against scripted controls. Every number comes from a committed result file.</p>
{prog_html}<div class="chips"><span class="chip">money spent <b>$0.00</b></span><span class="chip">Kaggle quota <b>≈ 19 h + 4.6 h (two weeks of 30)</b></span><span class="chip">baseline success <b>{pol['success_rate']:.2f} [{pol['ci95_wilson_success'][0]:.2f}, {pol['ci95_wilson_success'][1]:.2f}]</b></span><span class="chip">controls <b>oracle 1.00 · hold 0.00</b></span>{chips_extra}</div>
<h2>Measurements</h2>
<div class="tablewrap"><table><tr><th>measurement</th><th>result</th><th>uncertainty</th><th>where</th></tr>{rows_html}</table></div>
<h2>Charts</h2>{blocks}
<h2>Reading the results</h2><ul>{items}</ul>
{probes_html}{v3_html}{v4_html}{v5_html}{v6_html}{v7_html}{prec_html}{p5_html}{imported_html}<h2>Pending</h2><ul>{pending}</ul>
<div class="callout">Charts are drawn from <code>results/</code> by <code>sim/vla-bed/report.py</code>; the Markdown twin with the same SVGs is committed as <code>results/REPORT-{a.date}.md</code>.</div>
</main>
"""
        a.html.write_text(html)
        print("wrote", a.html)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
