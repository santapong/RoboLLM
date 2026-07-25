"""Human body observation -> FFW joint targets.

Pure math (no ROS, no numpy, no mediapipe) so it is unit-testable offline.

MIRROR MODE (the default, and what the user asked for): the robot behaves like
a mirror facing you — you raise your LEFT arm, the robot raises its RIGHT, and
the raised arm stays on the same side of the room.

Derivation, because sign errors here are invisible until someone waves:
put the robot at the origin facing +x with up +z, and the human in front of it
facing back. The human's forward is -x_world and, for a right-handed
(x fwd, y left, z up) body frame, the human's LEFT is -y_world. So a vector
(hx, hy, hz) in the human's torso frame is (-hx, -hy, hz) in the robot's.
Feed that to the OPPOSITE arm and the result is a true mirror.

DIRECT MODE is "the robot is you, seen from behind": same-side arm, and the
torso-frame components pass through untouched.

PER-ARM GATING. At a desk the hips are invisible and the elbows drop to ~0.02
visibility whenever the arms are at rest (both measured — see body_track.py).
So each arm is gated INDEPENDENTLY on its own shoulder/elbow/wrist, and an arm
that is not visible is simply left out of the returned targets. The node holds
those joints instead of commanding them, which is the difference between "the
robot waits for you to raise your arm" and "the robot chases a hand MediaPipe
invented".
"""

from humanoid_mirror.body_track import (
    LEFT_ELBOW,
    LEFT_SHOULDER,
    LEFT_WRIST,
    RIGHT_ELBOW,
    RIGHT_SHOULDER,
    RIGHT_WRIST,
)
from humanoid_mirror.ffw_arm import solve_arm

ARM_LANDMARKS = {
    "left": (LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST),
    "right": (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST),
}

# FFW head limits (measured; axes are NOT what the docs imply).
#   head_joint1  axis Y = PITCH, -0.2317 .. 0.6951, POSITIVE = looking DOWN
#   head_joint2  axis Z = YAW,   -0.35 .. 0.35  (+-20 degrees only)
HEAD_PITCH_LIMITS = (-0.2317, 0.6951)
HEAD_YAW_LIMITS = (-0.35, 0.35)


def _clip(v, lo, hi):
    return min(max(v, lo), hi)


def retarget(
    obs,
    mirror=True,
    min_visibility=0.5,
    head_gain_yaw=0.33,
    head_gain_pitch=0.6,
    prev=None,
    margin=0.05,
):
    """Joint targets for whatever of the body is currently visible.

    Returns (targets, info). `targets` maps joint name -> radians and contains
    ONLY the joints backed by visible landmarks; everything else is the
    caller's to hold. `info` reports which arms were used and the residual
    retargeting error per arm, so a caller can reject a bad frame.
    """
    prev = prev or {}
    targets, info = {}, {"arms": [], "err_deg": {}, "degenerate": {}, "skipped": {}}

    if obs is None or not obs.tracked or obs.basis is None:
        info["skipped"]["body"] = "not tracked"
        return targets, info

    for human_side in ("left", "right"):
        sh, el, wr = ARM_LANDMARKS[human_side]
        weak = [i for i in (sh, el, wr)
                if obs.visibility.get(i, 0.0) < min_visibility]
        if weak:
            # Named so the runbook's "raise your arms into frame" advice has
            # something concrete behind it.
            info["skipped"][human_side] = f"landmarks {weak} below {min_visibility}"
            continue

        a, b = obs.limb_dirs(human_side)
        if mirror:
            a = (-a[0], -a[1], a[2])
            b = (-b[0], -b[1], b[2])
            robot_side = "right" if human_side == "left" else "left"
        else:
            robot_side = human_side

        prefix = "arm_l" if robot_side == "left" else "arm_r"
        seed = [prev.get(f"{prefix}_joint{k}") for k in range(1, 5)]
        seed = seed if all(v is not None for v in seed) else None

        q, qinfo = solve_arm(a, b, robot_side, q_seed=seed, margin=margin)
        for k, v in enumerate(q, start=1):
            targets[f"{prefix}_joint{k}"] = v
        # Wrist joints are parked in this milestone: MediaPipe Pose has no
        # hand orientation, and inventing one would be a lie the robot acts on.
        for k in (5, 6, 7):
            targets[f"{prefix}_joint{k}"] = 0.0

        info["arms"].append(f"{human_side}->{robot_side}")
        info["err_deg"][robot_side] = round(qinfo["err_deg"], 3)
        info["degenerate"][robot_side] = qinfo["degenerate"]

    head = _retarget_head(obs, mirror, head_gain_yaw, head_gain_pitch, margin)
    targets.update(head)
    return targets, info


def _retarget_head(obs, mirror, gain_yaw, gain_pitch, margin):
    """Neck from NOSE + both EARs. Direct angle mapping, not IK.

    The gains MUST be well below 1: human yaw comfortably reaches +-60 degrees
    against this neck's +-20, so a 1:1 map spends most of its time pinned to a
    stop and reads as twitchy. This is a nod and a glance, not a look-around.
    """
    import math

    h = obs.head_dir()
    if h == (0.0, 0.0, 0.0):
        return {}
    yaw = math.atan2(h[1], h[0])                       # + = looking to own left
    pitch_down = -math.asin(max(-1.0, min(1.0, h[2])))  # + = looking down
    if mirror:
        yaw = -yaw
    lo_p, hi_p = HEAD_PITCH_LIMITS
    lo_y, hi_y = HEAD_YAW_LIMITS
    return {
        "head_joint1": _clip(gain_pitch * pitch_down, lo_p + margin, hi_p - margin),
        "head_joint2": _clip(gain_yaw * yaw, lo_y + margin, hi_y - margin),
    }


class MirrorPoseSource:
    """Adapts a BodyTracker's latest observation into joint targets.

    The node's vision thread owns the tracker and refreshes `observation`;
    this reads whatever is current. Deliberately does NOT block on a new
    frame: the control tick runs at 50 Hz while vision manages ~30 Hz, and
    re-solving the latest observation is what keeps motion smooth between
    frames rather than stepping at the camera rate.
    """

    name = "camera"

    def __init__(self, node):
        self.node = node

    def start(self):
        return None

    def stop(self):
        return None

    def read(self, t):
        with self.node._lock:
            obs = self.node.observation
        targets, info = retarget(
            obs,
            mirror=self.node.mirror_mode,
            min_visibility=self.node.min_visibility,
            head_gain_yaw=self.node.head_gain_yaw,
            head_gain_pitch=self.node.head_gain_pitch,
            prev=self.node.q_cmd,
            margin=self.node.margin,
        )
        self.node.retarget_info = info
        return targets or None
