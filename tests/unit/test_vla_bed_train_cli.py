"""Unit tests for sim/vla-bed/gpu/train.py — argv building, overrides, the step clock, the VLM cast (no torch needed)."""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED / "gpu"))
sys.path.insert(0, str(BED))
train = pytest.importorskip("train")


def _config():
    return json.loads((BED / "gpu" / "config.json").read_text())


def test_build_argv_defaults_and_modes():
    cfg = _config(); run = train.get_run(cfg, "baseline")
    argv = train.build_argv(cfg, run, "cpu-smoke", Path("/tmp/out"))
    assert "--policy.device=cpu" in argv and "--batch_size=2" in argv and f"--steps={cfg['cpu_smoke_steps']}" in argv
    full = train.build_argv(cfg, run, "full", Path("/tmp/out"))
    assert "--policy.device=cuda" in full and f"--batch_size={cfg['batch_size']}" in full and f"--steps={run['steps']}" in full and f"--save_freq={run['save_freq']}" in full
    assert any(a.startswith("--rename_map=") and "camera1" in a for a in full)


def test_build_argv_overrides():
    cfg = _config(); run = train.get_run(cfg, "plastic")
    argv = train.build_argv(cfg, run, "full", Path("/k/out"), {"dataset_root": "/kaggle/input/v2/train", "steps": 10000, "batch_size": 32, "save_freq": 2500})
    assert "--dataset.root=/kaggle/input/v2/train" in argv and "--steps=10000" in argv and "--batch_size=32" in argv and "--save_freq=2500" in argv
    assert "--policy.train_expert_only=false" in argv and "--policy.freeze_vision_encoder=false" in argv and "--policy.optimizer_lr=2.5e-05" in argv


def test_output_dir_override():
    cfg = _config(); run = train.get_run(cfg, "baseline")
    assert train.output_dir(cfg, run, "smoke", Path("/kaggle/working/a")) == Path("/kaggle/working/a/baseline/smoke")


def test_step_clock_counts_and_budget():
    calls = []
    clock = train.StepClock(lambda *a, **k: calls.append(1) or ("m", {}), max_hours=None, warmup_steps=1)
    for _ in range(4):
        clock(None)
    assert clock.steps == 4 and clock.steps_per_s is not None and clock.steps_per_s > 0
    tight = train.StepClock(lambda *a, **k: None, max_hours=0.0)
    with pytest.raises(train.TimeBudgetExceeded):
        tight(None)


def test_cast_vlm_touches_only_the_vlm(monkeypatch):
    seen = {}
    class VLM:
        def to(self, dtype): seen["dtype"] = dtype
    fake_torch = types.SimpleNamespace(float16="F16", bfloat16="BF16")
    monkeypatch.setitem(sys.modules, "torch", fake_torch)
    policy = types.SimpleNamespace(model=types.SimpleNamespace(vlm_with_expert=types.SimpleNamespace(vlm=VLM())))
    assert train.cast_vlm(policy, "float16") is policy and seen["dtype"] == "F16"
