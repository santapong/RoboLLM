"""Unit tests for sim/vla-bed/evaluate.py — suite loading, post-processing, ensembling, aggregation and
shard merging need no MuJoCo."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))
ev = pytest.importorskip("evaluate")


def _row(seed, family, success, safe=True, frames=20, rejections=None, depth=0.0, legacy=False):
    row = {"seed": seed, "family": family, "cell": 0, "variation": "nominal", "success": success, "progress": 1.0 if success else 0.0, "frames": frames, "chunks": 2, "final_error_m": 0.01 if success else 0.2, "min_error_m": 0.01, "safe": safe, "rejections": rejections or {}, "measured": {}, "worst_depth": depth, "latency_s_mean": 0.1}
    if not legacy:
        rej = sum((rejections or {}).values())
        row.update({"rejected_steps": rej, "rejected_fraction": rej / frames, "cmd_xyz_linf_mean": 0.009, "cmd_xyz_linf_max": 0.012 if rej else 0.01, "cmd_xyz_over_cap_fraction": rej / frames, "cmd_rpy_linf_mean": 0.02, "cmd_rpy_linf_max": 0.05, "cmd_rpy_over_cap_fraction": 0.0, "exec_xyz_linf_mean": 0.009})
    return row


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
    assert a["rejected_steps_total"] == 3 and a["frames_total"] == 200
    assert np.isclose(a["rejected_fraction_mean"], 0.015)
    assert np.isclose(a["cmd_xyz_over_cap_fraction_mean"], 0.015)
    assert a["cmd_xyz_linf_max"] == 0.012
    assert a["ensemble_members_mean"] is None


def test_aggregate_legacy_rows_derive_rejected_fraction():
    rows = [_row(i, "near", False, frames=10, rejections={"S2_xyz_step": 5}, legacy=True) for i in range(4)]
    a = ev.aggregate(rows)
    assert np.isclose(a["rejected_fraction_mean"], 0.5)
    assert a["cmd_xyz_linf_mean"] is None


def test_aggregate_empty():
    assert ev.aggregate([]) == {"n": 0}


def test_load_suite_from_manifest_and_shards(tmp_path):
    entries = [{"seed": 5 + i, "split": "evaluation", "family": "near", "cell": 1, "target": [0.0, 0.4, 0.3], "initial_q": [0.0] * 6} for i in range(5)]
    m = tmp_path / "manifest.json"
    m.write_text(json.dumps({"recipe": "v2", "cells": 20, "splits": {"evaluation": {"base_seed": 10000, "episodes": entries}}}))
    manifest, specs = ev.load_suite(m, "evaluation", episodes=1)
    assert manifest["recipe"] == "v2" and len(specs) == 1 and specs[0].seed == 5
    _, even = ev.load_suite(m, "evaluation", shard=(0, 2))
    _, odd = ev.load_suite(m, "evaluation", shard=(1, 2))
    assert [s.seed for s in even] == [5, 7, 9] and [s.seed for s in odd] == [6, 8]
    assert ev.parse_shard("1/2") == (1, 2) and ev.parse_shard(None) is None
    with pytest.raises(SystemExit):
        ev.parse_shard("2/2")


def test_hold_policy_chunk_shape():
    chunk = ev.HoldPolicy().act(None, {})
    assert chunk.shape == (1, 7) and not chunk.any()


def test_postprocess_gain_then_clip():
    a = np.array([0.02, -0.005, 0.0, 0.1, 0.0, -0.06, 1.0])
    g = ev.postprocess(a, gain=0.5)
    assert np.allclose(g, [0.01, -0.0025, 0.0, 0.05, 0.0, -0.03, 1.0])
    c = ev.postprocess(a, post="clip")
    assert np.allclose(c, [0.01, -0.005, 0.0, 0.05, 0.0, -0.05, 1.0])
    assert ev.postprocess(a).tolist() == a.tolist() and ev.postprocess(a) is not a


def test_temporal_ensemble_is_a_plain_mean_of_overlaps():
    te = ev.TemporalEnsemble()
    te.add(0, np.ones((4, 7)))
    te.add(2, np.full((4, 7), 3.0))
    a, k = te.pop(0)
    assert k == 1 and np.allclose(a, 1.0)
    a, k = te.pop(2)
    assert k == 2 and np.allclose(a, 2.0)
    assert te.has(5) and not te.has(0)


def test_merge_summaries_joins_shards_and_reaggregates():
    def part(shard, seeds, wall):
        rows = [_row(s, "near", s % 2 == 0) for s in seeds]
        return {"policy": {"label": "L", "post": "none"}, "schedule": {"variation": "nominal", "shard": shard, "episodes": len(rows)}, "L": ev.aggregate(rows), "episodes": {"L": rows}, "wall_s": wall, "measured_at": f"2026-09-04T1{len(seeds)}:00:00+0700"}

    merged = ev.merge_summaries([part("0/2", [10, 12, 14], 30.0), part("1/2", [11, 13], 20.0)])
    assert [r["seed"] for r in merged["episodes"]["L"]] == [10, 11, 12, 13, 14]
    assert merged["L"]["n"] == 5 and merged["L"]["success_rate"] == 0.6
    assert merged["wall_s"] == 30.0 and merged["wall_s_total"] == 50.0
    assert merged["schedule"]["shard"] is None and merged["schedule"]["episodes"] == 5
    with pytest.raises(ValueError):
        ev.merge_summaries([part("0/2", [10], 1.0), part("1/2", [10], 1.0)])


def test_default_out_suffixes(tmp_path):
    class A:
        variation, blank_image, gain, post, replan_every, n_action_steps, vlm_dtype, shard, label, policy = "nominal", False, 1.0, "none", None, 10, None, None, "x", "smolvla"

    assert ev.default_out(A()).name == "nominal.json"
    A.post, A.gain = "clip", 0.61
    assert ev.default_out(A()).name == "nominal_gain0.61_clip.json"
    A.post, A.gain, A.replan_every, A.shard = "ensemble", 1.0, 5, "1/2"
    assert ev.default_out(A()).name == "nominal_ens5_shard1of2.json"
