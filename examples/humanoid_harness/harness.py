"""Bounded observation-to-target adapter; no privileged target or robot command API."""

from __future__ import annotations

import numpy as np


def camera_target(packet: dict, age: float, low, high) -> np.ndarray | None:
    """Map mirrored image horizontal/vertical to right-arm lateral/height.

    Keep forward depth fixed: one uncalibrated camera cannot measure it.
    Missing/stale/invalid observations request a hold, not a guessed target.
    """
    if not packet.get("running") or not packet.get("tracked") or not np.isfinite(age) or not 0 <= age <= 0.5:
        return None
    try:
        value = np.asarray(packet.get("target"), dtype=float)
    except (ValueError, TypeError):
        return None
    if value.shape != (3,) or not np.isfinite(value).all():
        return None
    value = np.clip(value, -1.0, 1.0)
    low, high = np.asarray(low), np.asarray(high)
    target = (low + high) / 2
    target[1] = low[1] + (1 - value[0]) * 0.5 * (high[1] - low[1])
    target[2] = low[2] + (value[2] + 1) * 0.5 * (high[2] - low[2])
    return np.clip(target, low, high)


def slew_target(previous, requested, max_step=0.012):
    """Bound the Euclidean target displacement per update, not just each axis."""
    previous, requested = np.asarray(previous, dtype=float), np.asarray(requested, dtype=float)
    if previous.shape != (3,) or requested.shape != (3,) or not np.isfinite([previous, requested]).all():
        raise ValueError("expected finite XYZ targets")
    if not np.isfinite(max_step) or max_step <= 0:
        raise ValueError("max_step must be positive")
    delta = requested - previous
    return previous + delta * min(1.0, max_step / max(float(np.linalg.norm(delta)), 1e-12))
