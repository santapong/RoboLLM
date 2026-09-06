"""Unit tests for sim/vla-bed/precision.py — tolerance re-thresholding, Wilson agreement, fits and pairing need no MuJoCo."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))
pr = pytest.importorskip("precision")
from report import wilson  # noqa: E402


def _rows(errors, seed0=10):
    return [{"seed": seed0 + i, "family": "near", "cell": 0, "variation": "nominal", "success": e <= 0.03, "progress": 1.0 if e <= 0.03 else 0.0,
             "frames": 20, "final_error_m": e, "min_error_m": e, "rejected_fraction": 0.0} for i, e in enumerate(errors)]


def _suite(errors, label="baseline/010000"):
    return {"schema": pr.SCHEMA_ROWS, "policy": {"label": label, "post": "none", "gain": 1.0}, "schedule": {"variation": "nominal", "recipe": "vX"}, "episodes": {label: _rows(errors)}}


def test_success_at_tolerance_is_a_step_in_min_error():
    rows = _rows([0.05] * 10)
    assert pr.success_at_tolerance(rows, 0.04) == 0 and pr.success_at_tolerance(rows, 0.05) == 10 and pr.success_at_tolerance(rows, 0.10) == 10


def test_curve_counts_reached_at_0_03_and_wilson():
    errors = [0.01, 0.02, 0.03, 0.045, 0.055, 0.07, 0.09, 0.2, 0.3, 0.4]
    c = pr.curve(_rows(errors))
    assert c[0]["tolerance_m"] == 0.03 and c[0]["k"] == 3 and c[0]["success_rate"] == 0.3
    assert c[0]["ci95_wilson"] == [round(x, 4) for x in wilson(3, 10)]
    assert [q["k"] for q in c] == [3, 3, 4, 5, 6, 6, 7, 7]


def test_fit_reports_both_forms_with_r2():
    pts = [{"tolerance_m": p, "success_rate": 1 - 0.5 * (0.03 / p) ** 1.5} for p in pr.TOLERANCES_M]
    f = pr.fit_curse_of_precision(pts)
    assert 0.9 < f["power_law"]["r2"] <= 1.0 and 0 <= f["curse_of_precision"]["c_m"] < 0.03 and f["points_used"] == 8
    assert "note" in pr.fit_curse_of_precision([{"tolerance_m": 0.03, "success_rate": 1.0}] * 3)


def test_pair_at_tolerances_rethresholds_both_suites():
    a = _suite([0.05] * 10 + [0.2] * 10)
    b = _suite([0.035] * 10 + [0.2] * 10)
    res = pr.pair_at_tolerances(a, b, tolerances=[0.03, 0.04, 0.06])
    by = {q["tolerance_m"]: q for q in res["at_tolerance"]}
    assert by[0.03]["success_a"] == 0.0 and by[0.03]["success_b"] == 0.0
    assert by[0.04]["success_a"] == 0.0 and by[0.04]["success_b"] == 0.5 and by[0.04]["discordant"]["a_fail_b_success"] == 10
    assert by[0.06]["success_a"] == 0.5 and by[0.06]["success_b"] == 0.5 and by[0.06]["mcnemar_exact_p"] == 1.0
    assert "censored" in res["caveat"]


def test_suite_curve_and_chart(tmp_path):
    d = _suite([0.02] * 5 + [0.06] * 5)
    p = tmp_path / "p5" / "hostx" / "baseline" / "010000" / "nominal.json"
    p.parent.mkdir(parents=True)
    p.write_text(__import__("json").dumps(d))
    res = pr.suite_curve(p, d)
    assert res["recipe"] == "vX" and res["host"] == "hostx" and res["curve"][0]["success_rate"] == 0.5 and res["curve"][-1]["success_rate"] == 1.0
    assert res["suite_success_rate"] == 0.5 and res["reached_but_not_held"] == 0
    d["episodes"]["baseline/010000"][0]["success"] = False  # reached 0.02 m but did not hold it
    res = pr.suite_curve(p, d)
    assert res["suite_success_rate"] == 0.4 and res["reached_but_not_held"] == 1
    svg = pr.line_chart("t", [("vX", res["curve"], "#0E7C86")], note=res["caveat"][:40])
    assert svg.startswith("<svg") and "polyline" in svg and svg.count("<circle") == 8
