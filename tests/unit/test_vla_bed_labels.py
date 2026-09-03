"""Unit tests for sim/vla-bed/labels.py — the train-time action representations (P4)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))
labels = pytest.importorskip("labels")


def _random_actions(rng, n=20):
    a = np.zeros((n, 7))
    a[:, :3] = rng.uniform(-0.01, 0.01, size=(n, 3))
    a[:, 3:6] = rng.uniform(-0.05, 0.05, size=(n, 3))
    a[:, 6] = rng.choice([-1.0, 0.0, 1.0], size=n)
    return a


def _random_state(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    return np.concatenate([rng.uniform(-0.3, 0.5, 3), q, [0.2], rng.uniform(-1, 1, 6)]).astype(np.float32)


@pytest.mark.parametrize("representation", labels.REPRESENTATIONS)
def test_round_trip(representation):
    rng = np.random.default_rng(0)
    a, s = _random_actions(rng), _random_state(rng)
    back = labels.invert(labels.transform(a, representation, s), representation, s)
    assert np.allclose(back, a, atol=1e-9)


def test_gripper_frame_is_a_rotation_and_keeps_gripper_channel():
    rng = np.random.default_rng(1)
    a, s = _random_actions(rng), _random_state(rng)
    g = labels.transform(a, "gripper_frame", s)
    assert np.allclose(np.linalg.norm(g[:, :3], axis=1), np.linalg.norm(a[:, :3], axis=1))
    assert np.allclose(g[:, 6], a[:, 6])
    R = labels.ee_rotation_from_state(s)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(R), 1.0)


def test_chunk_delta_of_a_straight_line_is_linear_and_rotation_composes():
    a = np.zeros((10, 7))
    a[:, 0] = 0.005
    a[:, 5] = 0.02  # constant yaw rate
    c = labels.transform(a, "chunk_delta")
    assert np.allclose(c[:, 0], 0.005 * np.arange(1, 11))
    assert np.allclose(c[:, 5], 0.02 * np.arange(1, 11), atol=1e-9)  # same axis → angles add exactly
    assert np.allclose(c[:, 6], a[:, 6])


def test_window_stats_shapes():
    rng = np.random.default_rng(2)
    w = np.stack([_random_actions(rng) for _ in range(5)])
    st = labels.window_stats(w)
    assert st["mean"].shape == (7,) and st["q99"].shape == (7,) and int(st["count"][0]) == 100
    assert np.all(st["max"] >= st["q99"]) and np.all(st["min"] <= st["q01"])


def test_unknown_representation_raises():
    with pytest.raises(ValueError):
        labels.transform(np.zeros((2, 7)), "nope")
