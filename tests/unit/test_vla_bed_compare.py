"""Unit tests for sim/vla-bed/compare.py — paired comparison needs no MuJoCo."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))
cmp = pytest.importorskip("compare")


def _summary(label, successes, variation="nominal", post="none", gain=1.0):
    rows = [{"seed": 1000 + i, "family": "near" if i % 2 else "left", "success": bool(s), "progress": 1.0 if s else 0.0, "frames": 20, "rejections": {} if s else {"S2_xyz_step": 5}, "rejected_fraction": 0.0 if s else 0.25} for i, s in enumerate(successes)]
    return {"policy": {"label": label, "post": post, "gain": gain, "checkpoint": None}, "schedule": {"variation": variation}, "episodes": {label: rows}}


def test_mcnemar_exact_symmetry_and_extremes():
    assert cmp.mcnemar_exact(0, 0) == 1.0
    assert cmp.mcnemar_exact(5, 5) == 1.0
    assert cmp.mcnemar_exact(10, 0) == pytest.approx(2 / 1024)
    assert cmp.mcnemar_exact(0, 10) == cmp.mcnemar_exact(10, 0)


def test_compare_pairs_by_seed_and_counts_discordant():
    a = _summary("A", [1, 0, 0, 0, 1, 0, 0, 0, 0, 0])
    b = _summary("B", [1, 1, 1, 0, 1, 1, 0, 0, 1, 0])
    r = cmp.compare(a, b)
    assert r["n_pairs"] == 10
    assert r["discordant"] == {"a_fail_b_success": 4, "a_success_b_fail": 0, "both_success": 2, "both_fail": 4}
    assert r["success_diff_b_minus_a"] == pytest.approx(0.4)
    assert r["mcnemar_exact_p"] == pytest.approx(2 / 16)
    assert r["rejected_fraction_a"] == pytest.approx(0.2) and r["rejected_fraction_b"] == pytest.approx(0.1)
    assert set(r["per_family_success_diff"]) == {"left", "near"}


def test_compare_oracle_vs_hold_is_separable():
    a = _summary("hold", [0] * 100)
    b = _summary("oracle", [1] * 100)
    r = cmp.compare(a, b)
    assert r["discordant"]["a_fail_b_success"] == 100
    assert r["mcnemar_exact_p"] < 1e-29
    lo, hi = r["success_diff_ci95_paired_bootstrap"]
    assert lo == hi == 1.0
    assert r["verdict"] == "B better"


def test_compare_requires_shared_seeds():
    a = _summary("A", [1, 0])
    b = _summary("B", [1, 0])
    for row in b["episodes"]["B"]:
        row["seed"] += 999
    with pytest.raises(ValueError):
        cmp.compare(a, b)
