"""UR5e VLA sim bed — goal families and episode schedules (sim/vla-bed/families.py)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))

import families  # noqa: E402

HOME = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])


def test_twenty_cells_family_major_and_inside_regions():
    grid = families.cells()
    assert len(grid) == 20
    assert [c.family for c in grid[:4]] == ["front_high"] * 4
    for c in grid:
        (x0, x1), (y0, y1), (z0, z1) = families.FAMILY_REGIONS[c.family]
        assert x0 - 1e-9 <= c.low[0] < c.high[0] <= x1 + 1e-9
        assert y0 - 1e-9 <= c.low[1] < c.high[1] <= y1 + 1e-9
        assert (c.low[2], c.high[2]) == (z0, z1)


def test_specs_are_deterministic_balanced_and_split_isolated():
    a = families.episode_specs(40, 123, "train", home_q=HOME)
    b = families.episode_specs(40, 123, "train", home_q=HOME)
    assert [s.to_dict() for s in a] == [s.to_dict() for s in b]
    counts = {}
    for s in a:
        counts[(s.family, s.cell)] = counts.get((s.family, s.cell), 0) + 1
    assert set(counts.values()) == {2}  # 40 episodes over 20 cells
    train_seeds = {s.seed for s in a}
    eval_seeds = {s.seed for s in families.episode_specs(40, 123, "evaluation", home_q=HOME)}
    assert train_seeds.isdisjoint(eval_seeds)


def test_target_inside_cell_and_initial_state_near_home():
    spec = families.make_episode_spec(7, "left", 3, "evaluation", home_q=HOME)
    cell = [c for c in families.cells() if (c.family, c.cell) == ("left", 3)][0]
    assert np.all(np.asarray(spec.target) >= np.asarray(cell.low))
    assert np.all(np.asarray(spec.target) <= np.asarray(cell.high))
    assert np.max(np.abs(np.asarray(spec.initial_q) - HOME)) <= families.INITIAL_JOINT_JITTER + 1e-12
    relocated = families.relocated_target(spec)
    assert np.all(relocated >= np.asarray(cell.low)) and np.all(relocated <= np.asarray(cell.high))
    assert not np.allclose(relocated, spec.target)


def test_unknown_split_rejected():
    with pytest.raises(ValueError):
        families.make_episode_spec(1, "near", 0, "test", home_q=HOME)


def test_frozen_families_file_covers_every_cell():
    doc = families.load_frozen()
    verified = {(c["family"], c["cell"]) for c in doc["verified"]}
    assert verified == {(c.family, c.cell) for c in families.cells()}
    assert all(c["worst_residual_m"] <= doc["max_residual_m"] for c in doc["verified"])
