"""Synthetic full-body pose source for talos_mirror's tracking node.

Ported from examples/humanoid_mirror/ros2_ws/src/humanoid_mirror/humanoid_mirror/
pose_source.py and extended. Upstream's SyntheticPoseSource produced ROBOT
JOINT ANGLES for FFW's arms+head+lift (there is no camera-tracking-only
concept in humanoid_mirror's M2). talos_mirror's `track` verb has no robot in
the loop at all (M3-equivalent: tracking only) -- so what needs to be
synthesised here is a full-body BODY OBSERVATION, in the same shape
BodyTracker.read() would hand back from a real camera, covering everything
this milestone tracks: both arms, both legs (hip/knee/ankle/heel/toe), the
head, AND a genuinely leaning torso.

Pure math (no ROS, no numpy, no mediapipe, no cv2), so `tools/body_accept.py`
can drive it directly with no camera and no launch file.

All landmarks are always fully visible (visibility=1.0): the point of
synthetic mode is to prove the whole publish path (TF, /body/markers,
/body/observation) works end-to-end, the same reasoning humanoid_mirror's M2
sweep used for the robot side. Per-limb GATING is exercised in
tools/body_accept.py by constructing observations directly with
body_track.torso_frame()/BodyObservation on hand-picked visibility
dictionaries -- that is where "hide one leg -> only that leg drops" is
actually proven, independent of this generator.
"""

import math

from talos_mirror.body_track import (
    LEFT_ANKLE,
    LEFT_EAR,
    LEFT_ELBOW,
    LEFT_FOOT_INDEX,
    LEFT_HEEL,
    LEFT_HIP,
    LEFT_KNEE,
    LEFT_SHOULDER,
    LEFT_WRIST,
    NOSE,
    RIGHT_ANKLE,
    RIGHT_EAR,
    RIGHT_ELBOW,
    RIGHT_FOOT_INDEX,
    RIGHT_HEEL,
    RIGHT_HIP,
    RIGHT_KNEE,
    RIGHT_SHOULDER,
    RIGHT_WRIST,
    BodyObservation,
    torso_frame,
)

# Slow on purpose -- this is a demo you watch in RViz, and a fast cycle would
# be hard to read as "arms raise, legs step, head glances, torso leans".
PERIOD_S = 16.0

# Segment lengths (metres), ordinary adult proportions -- these only need to
# look plausible in RViz, nothing downstream asserts exact numbers.
UPPER_ARM_LEN = 0.28
FOREARM_LEN = 0.26
THIGH_LEN = 0.42
SHIN_LEN = 0.40

HIP_HALF_WIDTH = 0.10
SHOULDER_HALF_WIDTH = 0.20
SHOULDER_HEIGHT = 0.50  # above the hip midpoint, standing


def _ease(phase):
    """0..1 raised-cosine wave from a phase in 0..1 -- continuous derivative,
    so nothing in the generated motion has a velocity discontinuity to mask a
    downstream clamp/slew bug behind (same reasoning as FFW's _ease)."""
    return 0.5 - 0.5 * math.cos(2.0 * math.pi * phase)


def _dir(roll, pitch):
    """Unit direction in BODY axes (x fwd, y left, z up) for a limb hanging
    from a ball joint: roll=0,pitch=0 is straight down. roll lifts the limb
    OUT TO THE SIDE (positive roll -> +y, the subject's left); pitch swings
    it FORWARD (positive pitch -> +x). Shared by arms and legs so "raise" and
    "swing forward" mean the same thing for both."""
    return (
        math.sin(pitch),
        math.cos(pitch) * math.sin(roll),
        -math.cos(pitch) * math.cos(roll),
    )


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _along(origin, direction, length):
    return _add(origin, (direction[0] * length, direction[1] * length, direction[2] * length))


def _arm(shoulder, side_sign, raise_p, fold_p, swing_p):
    """(elbow, wrist) for one arm. side_sign is +1 (left) or -1 (right) --
    only the roll sign differs between arms, matching real shoulder anatomy
    (mirrored, not identical)."""
    roll = side_sign * (0.15 + 1.35 * raise_p)
    pitch = -0.35 + 0.70 * swing_p
    elbow_flex = 0.15 + 1.55 * fold_p

    upper_dir = _dir(roll, pitch)
    elbow = _along(shoulder, upper_dir, UPPER_ARM_LEN)
    # Forearm curls forward relative to the upper arm as the elbow flexes.
    fore_dir = _dir(roll * 0.6, pitch + elbow_flex)
    wrist = _along(elbow, fore_dir, FOREARM_LEN)
    return elbow, wrist


def _leg(hip, lift_p):
    """(knee, ankle, heel, toe) for one leg. lift_p in 0..1: 0 = standing
    straight, 1 = knee lifted and swung forward (a marching-in-place step)."""
    hip_pitch = 0.55 * lift_p
    knee_flex = 1.15 * lift_p

    thigh_dir = _dir(0.0, hip_pitch)
    knee = _along(hip, thigh_dir, THIGH_LEN)
    shin_dir = _dir(0.0, hip_pitch + knee_flex)
    ankle = _along(knee, shin_dir, SHIN_LEN)
    heel = _add(ankle, (-0.05, 0.0, -0.02))
    toe = _add(ankle, (0.12, 0.0, -0.03))
    return knee, ankle, heel, toe


def body_points(t, period_s=PERIOD_S):
    """All 18 tracked landmarks (BODY axes, x fwd/y left/z up, hip-midpoint
    origin) at time `t`. Pure function of time -- exactly reproducible, which
    matters for tools/body_accept.py to assert on specific phases."""
    phase = (t % period_s) / period_s

    raise_p = _ease(phase)                      # arms out to the sides
    fold_p = _ease((phase + 0.25) % 1.0)         # elbows fold
    swing_p = _ease((phase + 0.5) % 1.0)         # shoulders swing fwd/back
    lean_x = 0.06 * math.sin(2.0 * math.pi * phase)   # torso leans fwd/back
    glance_y = 0.05 * math.sin(4.0 * math.pi * phase)  # head glances L/R
    nod_x = 0.02 * _ease((phase + 0.15) % 1.0)          # head nods

    # Legs alternate: left and right are 180 degrees out of phase, like
    # marching in place.
    left_lift = _ease(phase)
    right_lift = _ease((phase + 0.5) % 1.0)

    left_hip = (0.0, HIP_HALF_WIDTH, 0.0)
    right_hip = (0.0, -HIP_HALF_WIDTH, 0.0)

    lean = (lean_x, 0.0, 0.0)
    left_shoulder = _add((0.0, SHOULDER_HALF_WIDTH, SHOULDER_HEIGHT), lean)
    right_shoulder = _add((0.0, -SHOULDER_HALF_WIDTH, SHOULDER_HEIGHT), lean)
    shoulder_mid = (
        (left_shoulder[0] + right_shoulder[0]) * 0.5,
        0.0,
        (left_shoulder[2] + right_shoulder[2]) * 0.5,
    )
    nose = _add(shoulder_mid, (0.22 + nod_x, glance_y, 0.28))
    left_ear = _add(shoulder_mid, (0.08, 0.08 + glance_y, 0.25))
    right_ear = _add(shoulder_mid, (0.08, -0.08 + glance_y, 0.25))

    left_elbow, left_wrist = _arm(left_shoulder, +1.0, raise_p, fold_p, swing_p)
    right_elbow, right_wrist = _arm(right_shoulder, -1.0, raise_p, fold_p, swing_p)

    left_knee, left_ankle, left_heel, left_toe = _leg(left_hip, left_lift)
    right_knee, right_ankle, right_heel, right_toe = _leg(right_hip, right_lift)

    return {
        NOSE: nose,
        LEFT_EAR: left_ear,
        RIGHT_EAR: right_ear,
        LEFT_SHOULDER: left_shoulder,
        RIGHT_SHOULDER: right_shoulder,
        LEFT_ELBOW: left_elbow,
        RIGHT_ELBOW: right_elbow,
        LEFT_WRIST: left_wrist,
        RIGHT_WRIST: right_wrist,
        LEFT_HIP: left_hip,
        RIGHT_HIP: right_hip,
        LEFT_KNEE: left_knee,
        RIGHT_KNEE: right_knee,
        LEFT_ANKLE: left_ankle,
        RIGHT_ANKLE: right_ankle,
        LEFT_HEEL: left_heel,
        RIGHT_HEEL: right_heel,
        LEFT_FOOT_INDEX: left_toe,
        RIGHT_FOOT_INDEX: right_toe,
    }


class SyntheticBodyPoseSource:
    """Deterministic whole-body motion generator: arms raise and fold, legs
    march in place, head nods and glances, torso leans fwd/back. Exercises
    every landmark this milestone tracks, with no camera and no mediapipe
    import anywhere in the process.

    Always fully visible and always tracked -- any dropout seen while this
    source drives the track node is a bug in the node, not in the input
    (same contract FFW's SyntheticPoseSource documents for the sweep node).
    """

    name = "synthetic"

    def __init__(self, period_s=PERIOD_S):
        self.period_s = float(period_s)

    def start(self):
        return None

    def stop(self):
        return None

    def read(self, t):
        points = body_points(t, self.period_s)
        visibility = {i: 1.0 for i in points}
        basis, source = torso_frame(points, visibility)
        tracked = basis is not None
        return BodyObservation(
            t, points, visibility, basis, source if tracked else "",
            tracked, "" if tracked else source,
        )
