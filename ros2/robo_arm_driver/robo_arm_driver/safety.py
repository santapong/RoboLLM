"""Pure-Python trajectory validation and sampling for the arm bridge."""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Sequence

from .config import ArmConfig, ConfigError, NJOINTS


@dataclass(frozen=True)
class SafePoint:
    positions: tuple[float, ...]
    time_from_start_s: float


def validate_trajectory(
    joint_names: Sequence[str],
    positions: Sequence[Sequence[float]],
    times_s: Sequence[float],
    initial_positions: Sequence[float],
    config: ArmConfig,
) -> tuple[SafePoint, ...]:
    """Normalize a named trajectory into config channel order or reject it.

    Limits are checked twice in the full stack: here in logical radians and in
    firmware in raw servo degrees. This function never clamps an invalid plan.
    """
    if len(joint_names) != NJOINTS or len(set(joint_names)) != NJOINTS:
        raise ConfigError(f"trajectory must name each of {config.joint_names} exactly once")
    if set(joint_names) != set(config.joint_names):
        raise ConfigError(
            f"trajectory joints {tuple(joint_names)} do not match {config.joint_names}")
    if not positions or len(positions) != len(times_s):
        raise ConfigError("trajectory needs equally sized, non-empty point and time lists")
    if len(initial_positions) != NJOINTS:
        raise ConfigError(f"initial state must contain {NJOINTS} joints")

    source_index = {name: index for index, name in enumerate(joint_names)}
    order = [source_index[name] for name in config.joint_names]
    points: list[SafePoint] = []
    previous_q = tuple(float(q) for q in initial_positions)
    for joint, value in zip(config.joints, previous_q):
        joint.require_rad(value)
    previous_t = 0.0

    for point_index, (raw_positions, raw_time) in enumerate(zip(positions, times_s)):
        if len(raw_positions) != NJOINTS:
            raise ConfigError(f"trajectory point {point_index} needs {NJOINTS} positions")
        time_s = float(raw_time)
        if not math.isfinite(time_s) or time_s <= previous_t:
            raise ConfigError("trajectory times must be finite, positive, and strictly increasing")
        q = tuple(float(raw_positions[i]) for i in order)
        for joint, value in zip(config.joints, q):
            joint.require_rad(value)

        dt = time_s - previous_t
        for joint, old, new in zip(config.joints, previous_q, q):
            velocity = abs(new - old) / dt
            if velocity > joint.max_velocity_rad_s + 1e-9:
                raise ConfigError(
                    f"{joint.name}: segment {point_index} requests "
                    f"{math.degrees(velocity):.2f} deg/s, above configured "
                    f"{joint.max_velocity_deg_s:.2f} deg/s")
        points.append(SafePoint(q, time_s))
        previous_q = q
        previous_t = time_s

    return tuple(points)


class TrajectorySampler:
    """Linearly sample a previously validated trajectory at the control rate."""

    def __init__(self, initial: Sequence[float], points: Sequence[SafePoint], start_s: float):
        if not points:
            raise ValueError("at least one trajectory point is required")
        self.initial = tuple(float(q) for q in initial)
        self.points = tuple(points)
        self.start_s = float(start_s)
        self._times = [point.time_from_start_s for point in self.points]

    def sample(self, now_s: float) -> tuple[tuple[float, ...], bool]:
        elapsed = max(0.0, float(now_s) - self.start_s)
        if elapsed >= self.points[-1].time_from_start_s:
            return self.points[-1].positions, True

        right = bisect.bisect_right(self._times, elapsed)
        if right == 0:
            q0, t0 = self.initial, 0.0
            q1, t1 = self.points[0].positions, self.points[0].time_from_start_s
        else:
            q0 = self.points[right - 1].positions
            t0 = self.points[right - 1].time_from_start_s
            q1 = self.points[right].positions
            t1 = self.points[right].time_from_start_s
        alpha = (elapsed - t0) / (t1 - t0)
        return tuple(a + alpha * (b - a) for a, b in zip(q0, q1)), False
