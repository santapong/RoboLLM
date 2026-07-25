"""Joint limits for the FFW semi-humanoid, and the safety clamps built on them.

Limits are read from the live URDF (`/robot_description`) when one is available,
because a variant swap or an upstream package bump would otherwise silently
invalidate a hardcoded table. MEASURED is the fallback AND the cross-check: if
the URDF disagrees with it, the node warns loudly rather than trusting either
blindly.

MEASURED was taken from the expanded ffw_bg2_rev4_follower xacro. Two entries
in it are the ones that break naive code, so they are worth reading twice:

  * arm_l_joint2 is ONE-SIDED  0.0 .. 3.14  (cannot adduct across the chest)
    arm_r_joint2 MIRRORS it   -3.14 .. 0.0
    A symmetric left/right pose is therefore out of range on one side.
  * head_joint1 is axis Y = PITCH and head_joint2 is axis Z = YAW — the
    opposite of the "pan/tilt" reading ROBOTIS's own docs suggest. Positive
    head_joint1 means looking DOWN.
"""

import math

# name -> (lower, upper, velocity) in rad or m, rad/s or m/s.
MEASURED = {
    "lift_joint": (-0.5, 0.0, 4.8),
    "head_joint1": (-0.2317, 0.6951, 4.8),  # axis Y = PITCH, + = looking down
    "head_joint2": (-0.35, 0.35, 4.8),      # axis Z = YAW, +/- 20 deg only
    "arm_l_joint1": (-3.14, 3.14, 4.8),     # axis Y  shoulder pitch
    "arm_l_joint2": (0.0, 3.14, 4.8),       # axis X  shoulder roll — ONE-SIDED
    "arm_l_joint3": (-3.14, 3.14, 4.8),     # axis Z  humeral yaw
    "arm_l_joint4": (-2.9361, 1.0786, 4.8),  # axis Y  elbow — flexes NEGATIVE
    "arm_l_joint5": (-3.14, 3.14, 4.8),
    "arm_l_joint6": (-1.57, 1.57, 4.8),
    "arm_l_joint7": (-1.8201, 1.5804, 4.8),
    "arm_r_joint1": (-3.14, 3.14, 4.8),
    "arm_r_joint2": (-3.14, 0.0, 4.8),      # MIRRORS arm_l_joint2
    "arm_r_joint3": (-3.14, 3.14, 4.8),
    "arm_r_joint4": (-2.9361, 1.0786, 4.8),
    "arm_r_joint5": (-3.14, 3.14, 4.8),
    "arm_r_joint6": (-1.57, 1.57, 4.8),
    "arm_r_joint7": (-1.5804, 1.8201, 4.8),  # MIRRORS arm_l_joint7
}

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


def resolve_limits(urdf_xml=None, logger=None):
    """Limits to use, preferring the live URDF and cross-checking MEASURED.

    Returns (limits, source) where source is "urdf" or "measured".
    """
    if not urdf_xml:
        if logger:
            logger.warn(
                "no robot_description available — falling back to the MEASURED "
                "limit table for ffw_bg2_rev4_follower. If you changed variant, "
                "the clamps are now wrong."
            )
        return dict(MEASURED), "measured"

    parsed = parse_urdf_limits(urdf_xml)
    missing = [j for j in MEASURED if j not in parsed]
    if missing:
        if logger:
            logger.warn(
                f"robot_description is missing {len(missing)} expected joints "
                f"({missing[:3]}...) — falling back to MEASURED."
            )
        return dict(MEASURED), "measured"

    # Cross-check. A mismatch means the robot is not the variant this example
    # was written against, and every retargeting constant below is suspect.
    for name, (lo, up, _vel) in MEASURED.items():
        p_lo, p_up, _ = parsed[name]
        if abs(p_lo - lo) > 1e-3 or abs(p_up - up) > 1e-3:
            if logger:
                logger.warn(
                    f"{name}: URDF limits ({p_lo:.4f}, {p_up:.4f}) disagree with "
                    f"MEASURED ({lo:.4f}, {up:.4f}). Using the URDF. If this "
                    f"fires you are probably not on ffw_bg2_rev4_follower."
                )
    return {j: parsed[j] for j in MEASURED}, "urdf"


def clamp(value, name, limits, margin=DEFAULT_MARGIN):
    """Clamp one joint command into [lower+margin, upper-margin]."""
    lo, up, _vel = limits[name]
    lo, up = lo + margin, up - margin
    if lo > up:  # margin wider than the range itself — degenerate, use midpoint
        return 0.5 * (lo + up)
    return min(max(value, lo), up)


def slew(target, current, max_step):
    """Move `current` toward `target` by at most `max_step` this tick.

    This is the velocity limit, and it is the difference between a smooth
    demo and the robot snapping across its workspace on the first frame.

    NOTE the sizing trap inherited from hand_follow: its max_joint_step_rad of
    0.10 at 20 Hz is EXACTLY 2.0 rad/s, which its docstring calls "under" the
    weld arm's limit but is in fact AT it. Copy that number into a 50 Hz loop
    and you silently get 5.0 rad/s. Size this from rate, never by habit.
    """
    delta = target - current
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    return current + delta


def max_step_for(rate_hz, rad_per_sec):
    """Per-tick slew budget for a desired angular speed at a given rate."""
    return rad_per_sec / float(rate_hz)


class OneEuro:
    """One-Euro filter, Casiez et al. CHI 2012. Timestamps in seconds.

    Copied verbatim from hand_follow.py — proven on this hardware at
    min_cutoff=1.0, beta=0.5. Kept as a copy rather than an import because the
    two examples vendor independent workspaces; the byte-identity of shared
    math is guarded in CI for arm_ik.py and the same applies here.

    Filter ANGLES, not landmarks: filtering landmarks and then computing angles
    lets noise through the nonlinearity of the arccos/atan2.
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
