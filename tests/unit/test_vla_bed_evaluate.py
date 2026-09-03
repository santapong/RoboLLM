"""Unit tests for sim/vla-bed/evaluate.py — suite loading and aggregation need no MuJoCo."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))
ev = pytest.importorskip("evaluate")


def _row(seed, family, success, safe=True, frames=20, rejections=None, depth=0.0):
    return {"seed": seed, "family": family, "cell": 0, "variation": "nominal", "success": success, "progress": 1.0 if success else 0.0, "frames": frames, "chunks": 2, "final_error_m": 0.01 if success else 0.2, "min_error_m": 0.01, "safe": safe, "rejections": rejections or {}, "measured": {}, "worst_depth": depth, "latency_s_mean": 0.1}


def test_aggregate_quadruple_and_intervals():
    rows = [_row(i, "left" if i % 2 else "near", i < 7, safe=i != 8, rejections={"S2_xyz_step": 3} if i == 8 else None, depth=0.5 if i == 8 else 0.0) for i in range(10)]
    a = ev.aggregate(rows)
    assert a["n"] == 10 and a["success_rate"] == 0.7 and a["safety"] == 0.9
    assert a["sbu"] == 0.0 and np.isclose(a["vsi"], 0.05)
    lo, hi = a["ci95_wilson_success"]
    assert lo < 0.7 < hi
    assert a["per_family_success"] == {"left": 0.6, "near": 0.8}
    assert a["faults"]["rejected"] == {"S2_xyz_step": 3}
    assert a["episode_len_mean"] == 20.0


def test_aggregate_empty():
    assert ev.aggregate([]) == {"n": 0}


def test_load_suite_from_manifest(tmp_path):
    entries = [{"seed": 5, "split": "evaluation", "family": "near", "cell": 1, "target": [0.0, 0.4, 0.3], "initial_q": [0.0] * 6}, {"seed": 6, "split": "evaluation", "family": "left", "cell": 2, "target": [-0.2, 0.4, 0.3], "initial_q": [0.1] * 6}]
    m = tmp_path / "manifest.json"
    m.write_text(json.dumps({"recipe": "v2", "cells": 20, "splits": {"evaluation": {"base_seed": 10000, "episodes": entries}}}))
    manifest, specs = ev.load_suite(m, "evaluation", episodes=1)
    assert manifest["recipe"] == "v2" and len(specs) == 1
    assert specs[0].seed == 5 and specs[0].family == "near" and specs[0].target == (0.0, 0.4, 0.3)


def test_hold_policy_chunk_shape():
    chunk = ev.HoldPolicy().act(None, {})
    assert chunk.shape == (1, 7) and not chunk.any()
