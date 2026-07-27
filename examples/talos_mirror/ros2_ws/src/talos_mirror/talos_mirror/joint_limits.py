"""Joint limits for TALOS's 30 group joints, and the safety clamp built on
them.

MEASURED is a byte-for-byte copy of docs/joint-inventory.md / docs/joints.json
(position limits, verified 2026-07-27 from the expanded URDF) plus the
velocity figures from talos_moveit_config/config/joint_limits.yaml (also
measured from the URDF's own <limit velocity="..."/> values). It is hardcoded
rather than read from docs/joints.json at runtime because that file lives
under examples/talos_mirror/docs/, a sibling of ros2_ws/ that docker/ros2-arm
does NOT mount into the container (only ros2_ws/ is) — the acceptance tool
cross-checks this table against the LIVE URDF instead, which is the
"every joint in joints.json exists with matching limits" check in practice
and catches the same class of drift ffw_check.py's MEASURED-vs-URDF
cross-check does for FFW.

gripper_left_joint / gripper_right_joint are in docs/joints.json but are OUT
OF SCOPE here (no gripper group, no gripper macros in
talos_moveit_config/config/talos.urdf.xacro) — deliberately absent from this
table, not an oversight.
"""

import math

# name -> (lower, upper, velocity) in rad, rad/s.
MEASURED = {
    # torso
    "torso_1_joint": (-1.2566370614359172, 1.2566370614359172, 5.4),
    "torso_2_joint": (-0.22689280275926285, 0.7330382858376184, 5.4),
    # arm_l
    "arm_left_1_joint": (-1.5707963267948966, 0.7853981633974483, 2.7),
    "arm_left_2_joint": (0.008726646259971648, 2.871066619530672, 3.66),
    "arm_left_3_joint": (-2.426007660272118, 2.426007660272118, 4.58),
    "arm_left_4_joint": (-2.234021442552742, -0.003490658503988659, 4.58),
    "arm_left_5_joint": (-2.5132741228718345, 2.5132741228718345, 1.95),
    "arm_left_6_joint": (-1.3700834628155487, 1.3700834628155487, 1.76),
    "arm_left_7_joint": (-0.6806784082777885, 0.6806784082777885, 1.76),
    # arm_r — joint2 MIRRORS arm_left_2_joint (one-sided the other way)
    "arm_right_1_joint": (-0.7853981633974483, 1.5707963267948966, 2.7),
    "arm_right_2_joint": (-2.871066619530672, -0.008726646259971648, 3.66),
    "arm_right_3_joint": (-2.426007660272118, 2.426007660272118, 4.58),
    "arm_right_4_joint": (-2.234021442552742, -0.003490658503988659, 4.58),
    "arm_right_5_joint": (-2.5132741228718345, 2.5132741228718345, 1.95),
    "arm_right_6_joint": (-1.3700834628155487, 1.3700834628155487, 1.76),
    "arm_right_7_joint": (-0.6806784082777885, 0.6806784082777885, 1.76),
    # head
    "head_1_joint": (-0.20943951023931956, 0.7853981633974483, 3.0),
    "head_2_joint": (-1.3089969389957472, 1.3089969389957472, 3.0),
    # leg_l
    "leg_left_1_joint": (-0.3490658503988659, 1.5707963267948966, 3.87),
    "leg_left_2_joint": (-0.5236, 0.5236, 5.8),
    "leg_left_3_joint": (-2.095, 0.7, 5.8),
    "leg_left_4_joint": (0.0, 2.618, 7.0),
    "leg_left_5_joint": (-1.27, 0.68, 5.8),
    "leg_left_6_joint": (-0.5236, 0.5236, 4.8),
    # leg_r
    "leg_right_1_joint": (-1.5707963267948966, 0.3490658503988659, 3.87),
    "leg_right_2_joint": (-0.5236, 0.5236, 5.8),
    "leg_right_3_joint": (-2.095, 0.7, 5.8),
    "leg_right_4_joint": (0.0, 2.618, 7.0),
    "leg_right_5_joint": (-1.27, 0.68, 5.8),
    "leg_right_6_joint": (-0.5236, 0.5236, 4.8),
}
assert len(MEASURED) == 30

# Stay this far off every limit. Commanding a joint exactly to its stop makes
# the controller fight the limit and makes MoveIt report states as invalid.
DEFAULT_MARGIN = 0.05


def parse_urdf_limits(urdf_xml):
    """{joint: (lower, upper, velocity)} for every bounded joint in a URDF."""
    import xml.etree.ElementTree as ET

    out = {}
    root = ET.fromstring(urdf_xml)
    for joint in root.findall("joint"):
        if joint.get("type") not in ("revolute", "prismatic"):
            continue  # continuous joints have no lower/upper; nothing to clamp
        limit = joint.find("limit")
        if limit is None:
            continue
        try:
            out[joint.get("name")] = (
                float(limit.get("lower")),
                float(limit.get("upper")),
                float(limit.get("velocity")),
            )
        except (TypeError, ValueError):
            continue
    return out


def clamp(value, name, limits=None, margin=DEFAULT_MARGIN):
    """Clamp one joint command into [lower+margin, upper-margin]."""
    lo, up, _vel = (limits or MEASURED)[name]
    lo, up = lo + margin, up - margin
    if lo > up:  # margin wider than the range itself — degenerate, use midpoint
        return 0.5 * (lo + up)
    return min(max(value, lo), up)


def excursion(value, name, limits=None, margin=DEFAULT_MARGIN):
    """How far `value` sits OUTSIDE [lower+margin, upper-margin]; 0 if inside.

    Always >= 0. This is the quantity the M2 sweep verification reports as
    "measured max excursion per group" — it must stay at 0.0 the whole run,
    because the sweep node clamps every command with `clamp()` above using
    the SAME margin before publishing.
    """
    lo, up, _vel = (limits or MEASURED)[name]
    lo, up = lo + margin, up - margin
    if value < lo:
        return lo - value
    if value > up:
        return value - up
    return 0.0


def slew(target, current, max_step):
    """Move `current` toward `target` by at most `max_step` this tick."""
    delta = target - current
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    return current + delta


def max_step_for(rate_hz, rad_per_sec):
    """Per-tick slew budget for a desired angular speed at a given rate."""
    return rad_per_sec / float(rate_hz)


def safe_amplitude(name, home, limits=None, margin=DEFAULT_MARGIN, frac=0.85):
    """Sine-sweep amplitude around `home` that stays inside the clamped
    envelope by construction (frac < 1 leaves headroom so the hard clamp in
    `clamp()` is a backstop, not the thing doing the work every tick)."""
    lo, up, _vel = (limits or MEASURED)[name]
    lo, up = lo + margin, up - margin
    return frac * min(home - lo, up - home)


class OneEuro:
    """One-Euro filter, Casiez et al. CHI 2012. Timestamps in seconds.

    Copied from humanoid_mirror's joint_limits.py (itself copied from
    hand_follow.py), proven on that hardware at min_cutoff=1.0, beta=0.5. Kept
    as a copy rather than an import -- this example is self-contained, no
    runtime imports from humanoid_mirror (see docs/joints.json header / the
    house rule this repeats).

    Filter ANGLES (mirror_node's OUTPUT joint targets), not landmarks:
    filtering landmarks and then computing angles lets noise through the
    nonlinearity of the acos/atan2 in retarget.py's solvers.
    """

    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x_prev = self.dx_prev = self.t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self):
        self.x_prev = self.dx_prev = self.t_prev = None

    def __call__(self, x, t):
        if self.t_prev is None:
            self.x_prev, self.dx_prev, self.t_prev = x, 0.0, t
            return x
        dt = max(t - self.t_prev, 1e-4)
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev
        a = self._alpha(self.min_cutoff + self.beta * abs(dx_hat), dt)
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat


# math is imported for callers (sweep_node) that want math.sin/pi alongside
# these helpers without a second import line; keeps this module the single
# "joint safety" import like humanoid_mirror's joint_limits.py.
__all__ = [
    "MEASURED",
    "DEFAULT_MARGIN",
    "parse_urdf_limits",
    "clamp",
    "excursion",
    "slew",
    "max_step_for",
    "safe_amplitude",
    "OneEuro",
    "math",
]
