"""Sources of joint-angle targets for the mirror node.

A source answers one question: *what should the robot's joints be right now?*
It returns `{joint_name: angle}` when it has an opinion, or `None` when it has
lost tracking (which the node turns into hold-then-decay).

M2 ships SyntheticPoseSource only — a scripted sweep with no camera and no
MediaPipe import anywhere in the process. That is deliberate: it proves the
whole publish path (four JTCs, disjoint joint sets, controller names, limits,
slew, RViz) before any vision risk enters the picture. When M3/M4 add
BodyPoseSource it plugs in behind this same interface, and the node does not
change.

Pure math, no ROS and no numpy, so the acceptance tools can import it directly.
"""

import math

from humanoid_mirror.ffw_config import ARM_L_JOINTS, ARM_R_JOINTS

# Sweep timing. Slow on purpose — this is a demo you watch, and a fast sweep
# would hide slew-limit bugs by never asking the robot to move faster than the
# clamp allows anyway.
PERIOD_S = 14.0


def _ease(phase):
    """0..1 triangle-ish wave with continuous derivative, from a phase in 0..1.

    A raw triangle wave has a velocity discontinuity at the turn, which would
    make the slew clamp fire once per cycle and mask whether the clamp is
    correctly sized. A raised cosine turns smoothly.
    """
    return 0.5 - 0.5 * math.cos(2.0 * math.pi * phase)


class SyntheticPoseSource:
    """Scripted whole-body sweep: both arms raise and fold, head nods and
    glances, lift travels. Exercises every joint the node drives.

    Respects the robot's real asymmetry rather than papering over it:
    arm_l_joint2 is one-sided 0..3.14 and arm_r_joint2 mirrors it at -3.14..0,
    so the two arms take opposite-signed roll for the same physical gesture.
    That asymmetry is exactly what a symmetric-looking test would fail to
    catch, so the sweep drives it directly.
    """

    name = "synthetic"

    def __init__(self, period_s=PERIOD_S, use_lift=True):
        self.period_s = float(period_s)
        self.use_lift = use_lift

    def start(self):
        return None

    def stop(self):
        return None

    def read(self, t):
        """Joint targets at time `t` seconds. Never returns None — the
        synthetic source cannot lose tracking, which is the point: any dropout
        seen in synthetic mode is a bug in the node, not in the input."""
        phase = (t % self.period_s) / self.period_s

        # Four overlapping motions at different phases, so joints are rarely
        # all at a turning point together.
        raise_p = _ease(phase)                      # arms out to the sides
        fold_p = _ease((phase + 0.25) % 1.0)        # elbows fold
        pitch_p = _ease((phase + 0.5) % 1.0)        # shoulders swing fwd/back
        head_p = _ease((phase + 0.15) % 1.0)
        glance_p = math.sin(2.0 * math.pi * phase)  # signed: look left/right

        # Shoulder roll: arms out to ~100 deg. Left positive, right negative.
        roll = 0.20 + 1.55 * raise_p
        # Shoulder pitch: gentle forward/back swing about zero.
        pitch = -0.60 + 1.20 * pitch_p
        # Humeral yaw: small, keeps the forearms from looking locked.
        yaw = -0.40 + 0.80 * raise_p
        # Elbow flexes NEGATIVE on FFW.
        elbow = -0.10 - 2.30 * fold_p

        targets = {}
        for joints, sign in ((ARM_L_JOINTS, +1.0), (ARM_R_JOINTS, -1.0)):
            targets[joints[0]] = pitch                 # joint1  shoulder pitch
            targets[joints[1]] = sign * roll           # joint2  roll, mirrored
            targets[joints[2]] = sign * yaw            # joint3  humeral yaw
            targets[joints[3]] = elbow                 # joint4  elbow
            targets[joints[4]] = 0.0                   # joint5..7 wrist parked
            targets[joints[5]] = 0.0
            targets[joints[6]] = 0.0

        # Head. joint1 is PITCH (+ = down, range -0.2317..0.6951), joint2 is
        # YAW (+/-0.35). Driven to ~80% of travel so the clamp is exercised
        # without living on the stops.
        targets["head_joint1"] = -0.15 + 0.70 * head_p
        targets["head_joint2"] = 0.28 * glance_p

        if self.use_lift:
            # Prismatic, -0.5 .. 0.0 m. Travel the middle half.
            targets["lift_joint"] = -0.35 + 0.25 * _ease((phase + 0.6) % 1.0)

        return targets
