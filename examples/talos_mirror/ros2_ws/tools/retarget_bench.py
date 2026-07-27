#!/usr/bin/env python3
"""M4 verification: is TALOS's full-body retargeting geometry actually right?

Ported from examples/humanoid_mirror/ros2_ws/tools/retarget_bench.py and
substantially extended: TALOS adds legs (with the pelvis pinned) and the
DISCRIMINATING-CASE + MUTATION-KILL requirement this task adds is new --
upstream never ran its own tests against a deliberately-wrong formula, it
only asserted the correct one. Here every discriminating case is run THREE
times: once with THE MIRROR LAW (mirror_vec, correct), once with a
deliberately negate-x-too formula, and once with a deliberately
left/right-swap-only formula -- and is expected to FAIL under both wrong
formulas (one documented exception: see "sideways raise" below).

Tiers, hardest question first:

  --fk    GROUND TRUTH. Compares this repo's arm_dirs()/leg_dirs() forward
          kinematics against MoveIt's /compute_fk over random joint
          configurations. If these disagree, every angle downstream is
          wrong and no amount of round-trip self-consistency reveals it.
          Needs `ros2-arm talos` (or `mirror`) running.

  (default) PURE MATH. Round-trip + known answers + the discriminating-case
          mutation-kill matrix + continuity, no ROS, no camera.

  --ros   LIVE. With `ros2-arm mirror synthetic` running, checks all six
          controllers are actually commanded and stay inside limits.

Emits RESULT:{json}.
"""

import argparse
import json
import math
import random
import sys

_ROOT = __file__.rsplit("/tools/", 1)[0]
sys.path.insert(0, _ROOT + "/src/talos_mirror")

from talos_mirror.body_track import (  # noqa: E402
    LEFT_ANKLE, LEFT_EAR, LEFT_ELBOW, LEFT_FOOT_INDEX, LEFT_HEEL, LEFT_HIP,
    LEFT_KNEE, LEFT_SHOULDER, LEFT_WRIST, NOSE, RIGHT_ANKLE, RIGHT_EAR,
    RIGHT_ELBOW, RIGHT_FOOT_INDEX, RIGHT_HEEL, RIGHT_HIP, RIGHT_KNEE,
    RIGHT_SHOULDER, RIGHT_WRIST,
)
from talos_mirror.joint_limits import MEASURED, max_step_for, slew  # noqa: E402
from talos_mirror.pose_source import SyntheticBodyPoseSource  # noqa: E402
from talos_mirror.retarget import (  # noqa: E402
    ARM_LIMITS,
    LEG_LIMITS,
    _mirror_bad_negate_xy,
    _mirror_bad_swap_only,
    ang_between,
    arm_dirs,
    leg_dirs,
    mirror_vec,
    retarget,
    solve_arm,
    solve_leg_hip_knee,
)

FAILURES = []


def check(ok, label, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}{(' -- ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(label)
    return ok


def ang(u, v):
    return math.degrees(ang_between(u, v))


IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def obs(points, gates=None, basis=None):
    """A minimal /body/observation-shaped dict for retarget() with an
    IDENTITY torso basis by default -- lets every test case name landmarks
    directly in torso-frame coordinates (x fwd, y left, z up) without
    routing through body_track's world-axis conversion, which M3's
    body_accept.py already covers independently. gates defaults to
    everything visible.

    `basis` overrides the identity -- needed for the torso-lean case, whose
    signal IS the basis itself (retarget()'s torso angles read obs["basis"]
    [0] directly, not a landmark pair), and pointless for every other case
    (arms/legs/head all read landmark DIFFERENCES, which an identity basis
    passes through unchanged, exactly like humanoid_mirror's own tests never
    needed a non-identity basis either)."""
    gates = gates if gates is not None else {
        g: True for g in ("arm_l", "arm_r", "leg_l", "leg_r", "head")
    }
    basis = basis if basis is not None else IDENTITY
    return {
        "t": 0.0, "tracked": True, "frame_source": "bench",
        "gates": dict(gates),
        "landmarks": {str(i): list(v) for i, v in points.items()},
        "basis": [list(row) for row in basis],
    }


def _basis_from_forward(f):
    """An orthonormal right-handed (f, l, u) frame with `f` (approximately)
    as forward, keeping `l`/`u` close to canonical left/up -- i.e. a small
    lean/turn away from upright, which is what torso_frame() itself always
    produces (see body_track.py). Used only by case_torso_lean(), which is
    the one case whose signal genuinely lives in obs["basis"] rather than a
    landmark pair.

    Derivation: given f, project the canonical up (0,0,1) perpendicular to
    it to get u (Gram-Schmidt), then l = u x f -- the cyclic identity for a
    right-handed frame where f = l x u (same convention body_track.py's
    torso_frame() and basis_to_quaternion() use throughout)."""
    from talos_mirror.retarget import _dot, _sub, _unit

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0])

    fu = _unit(f)
    u0 = (0.0, 0.0, 1.0)
    u = _unit(_sub(u0, tuple(c * _dot(fu, u0) for c in fu)))
    l = cross(u, fu)
    return (fu, l, u)


# A relaxed standing pose: arms down, legs straight, head level. Every case
# below copies this and perturbs exactly the landmarks it needs.
NEUTRAL = {
    LEFT_SHOULDER: (0.0, 0.20, 0.50), RIGHT_SHOULDER: (0.0, -0.20, 0.50),
    LEFT_ELBOW: (0.0, 0.20, 0.20), LEFT_WRIST: (0.0, 0.20, -0.10),
    RIGHT_ELBOW: (0.0, -0.20, 0.20), RIGHT_WRIST: (0.0, -0.20, -0.10),
    LEFT_HIP: (0.0, 0.10, 0.0), RIGHT_HIP: (0.0, -0.10, 0.0),
    LEFT_KNEE: (0.0, 0.10, -0.40), LEFT_ANKLE: (0.0, 0.10, -0.80),
    LEFT_HEEL: (-0.05, 0.10, -0.83), LEFT_FOOT_INDEX: (0.12, 0.10, -0.83),
    RIGHT_KNEE: (0.0, -0.10, -0.40), RIGHT_ANKLE: (0.0, -0.10, -0.80),
    RIGHT_HEEL: (-0.05, -0.10, -0.83), RIGHT_FOOT_INDEX: (0.12, -0.10, -0.83),
    NOSE: (0.15, 0.0, 0.65), LEFT_EAR: (0.05, 0.08, 0.63), RIGHT_EAR: (0.05, -0.08, 0.63),
}


def _pose(overrides):
    """NEUTRAL with `overrides` (a dict keyed by the INTEGER landmark index
    constants, e.g. {LEFT_ELBOW: (...)}) applied on top.

    Deliberately a positional dict, not **kwargs: landmark indices are
    ints, and Python keyword arguments can only ever be string names, so
    `_pose(LEFT_ELBOW=...)` would silently insert the STRING key
    "LEFT_ELBOW" into the pose dict rather than overriding the real
    LEFT_ELBOW=13 entry -- the case would keep NEUTRAL's straight-down arm
    and every "discriminating" case would silently degrade into asserting
    on the same at-rest pose. That shipped once in this file's own history
    (caught by the round-trip/known-answer checks going green while every
    discriminating case failed outright, which is what non-signal looks
    like) -- kept here as the reason this helper's signature looks
    unnecessarily strict.
    """
    p = dict(NEUTRAL)
    p.update(overrides)
    return p


MUTATIONS = {
    "negate_x_too": _mirror_bad_negate_xy,
    "swap_only": _mirror_bad_swap_only,
}


def _run(points, mirror_fn=None, prev=None, basis=None):
    return retarget(obs(points, basis=basis), mirror=True, prev=prev, mirror_fn=mirror_fn)


# =========================================================================
# THE DISCRIMINATING CASES. Each returns (points, assert_fn, basis) where
# assert_fn(targets, info) -> (ok, detail) must be TRUE under THE MIRROR
# LAW, and basis is None (identity -- everything except torso lean) or a
# custom (f, l, u) frame (torso lean only). Every case is then re-run under
# both MUTATIONS and (with one documented exception) must FLIP to False.
# =========================================================================
def case_forward_reach_left():
    """Human's LEFT arm reaches diagonally FORWARD and toward their own
    LEFT. Correct mirror: robot's RIGHT arm reaches forward (+x stays +x)
    and toward the robot's OWN right (mirrored y). Has BOTH a nonzero
    forward and a nonzero lateral component on purpose -- unlike a "pure"
    forward reach (y=0), this discriminates BOTH wrong formulas at once."""
    p = _pose({LEFT_ELBOW: (0.35, 0.40, 0.35), LEFT_WRIST: (0.75, 0.60, 0.20)})

    def assert_fn(targets, info):
        q = [targets[f"arm_right_{k}_joint"] for k in range(1, 5)]
        ua, _fa = arm_dirs(q, "right")
        ok = ua[0] > 0.3 and ua[1] < -0.15
        return ok, f"robot right upper-arm dir={tuple(round(v,3) for v in ua)} (want x>0.3, y<-0.15)"

    return p, assert_fn, None


def case_forward_reach_right():
    """Mirror of the above: human's RIGHT arm reaches forward+own-right ->
    robot's LEFT arm forward+robot's-own-left."""
    p = _pose({RIGHT_ELBOW: (0.35, -0.40, 0.35), RIGHT_WRIST: (0.75, -0.60, 0.20)})

    def assert_fn(targets, info):
        q = [targets[f"arm_left_{k}_joint"] for k in range(1, 5)]
        ua, _fa = arm_dirs(q, "left")
        ok = ua[0] > 0.3 and ua[1] > 0.15
        return ok, f"robot left upper-arm dir={tuple(round(v,3) for v in ua)} (want x>0.3, y>0.15)"

    return p, assert_fn, None


def case_forward_step_left():
    """Human's LEFT leg steps diagonally FORWARD and toward their own LEFT
    (a real march never stays on a perfect sagittal line). Correct mirror:
    robot's RIGHT leg steps forward, on the robot's own right, knee bending
    forward (not backward) -- the exact scenario THE MIRROR LAW's docstring
    calls out for legs."""
    p = _pose({LEFT_KNEE: (0.28, 0.20, -0.28), LEFT_ANKLE: (0.55, 0.30, -0.50),
               LEFT_HEEL: (0.50, 0.32, -0.46), LEFT_FOOT_INDEX: (0.70, 0.32, -0.60)})

    def assert_fn(targets, info):
        q = [targets[f"leg_right_{k}_joint"] for k in range(1, 5)]
        ta, sa = leg_dirs(q, "right")
        ok = ta[0] > 0.3 and ta[1] < -0.1 and sa[0] > 0.0
        return ok, (f"robot right thigh dir={tuple(round(v,3) for v in ta)} "
                    f"shin dir={tuple(round(v,3) for v in sa)} "
                    f"(want thigh x>0.3,y<-0.1; shin x>0 = knee bends FORWARD)")

    return p, assert_fn, None


def case_forward_step_right():
    p = _pose({RIGHT_KNEE: (0.28, -0.20, -0.28), RIGHT_ANKLE: (0.55, -0.30, -0.50),
               RIGHT_HEEL: (0.50, -0.32, -0.46), RIGHT_FOOT_INDEX: (0.70, -0.32, -0.60)})

    def assert_fn(targets, info):
        q = [targets[f"leg_left_{k}_joint"] for k in range(1, 5)]
        ta, sa = leg_dirs(q, "left")
        ok = ta[0] > 0.3 and ta[1] > 0.1 and sa[0] > 0.0
        return ok, (f"robot left thigh dir={tuple(round(v,3) for v in ta)} "
                    f"shin dir={tuple(round(v,3) for v in sa)} "
                    f"(want thigh x>0.3,y>0.1; shin x>0 = knee bends FORWARD)")

    return p, assert_fn, None


def case_sideways_raise():
    """THE DEGENERATE CASE, on purpose. Human's LEFT arm raised straight OUT
    TO THE SIDE: (0, 1, 0) in the human frame maps to (0, -1, 0) whether or
    not x is ALSO (wrongly) negated -- negating zero changes nothing. This
    case is included specifically so retarget-bench can DEMONSTRATE that,
    and is NOT expected to catch the negate_x_too mutation; it IS expected
    to catch swap_only (which fails to negate y at all)."""
    p = _pose({LEFT_ELBOW: (0.0, 0.48, 0.50), LEFT_WRIST: (0.0, 0.78, 0.50)})

    def assert_fn(targets, info):
        q = [targets[f"arm_right_{k}_joint"] for k in range(1, 5)]
        ua, _fa = arm_dirs(q, "right")
        ok = ua[1] < -0.85 and abs(ua[0]) < 0.2
        return ok, f"robot right upper-arm dir={tuple(round(v,3) for v in ua)} (want y<-0.85)"

    return p, assert_fn, None


def case_head_turn():
    """Human turns their head to look toward their own LEFT (and slightly
    down). Correct mirror: robot's head yaw flips sign (looks toward the
    robot's own right, since the robot faces you); pitch is unaffected.

    The assertion is a TIGHT window around the value THE MIRROR LAW itself
    produces, not a loose one-sided threshold: both mutations here still
    land on a NEGATIVE yaw (just the wrong magnitude -- negate_x_too drives
    it hard into the joint's clamp, -1.259 vs the correct -0.772), so a bare
    "yaw < -0.05" check would not flip to False under either mutation and
    the case would silently fail to discriminate anything. Computing the
    reference value from THE MIRROR LAW itself (rather than hand-deriving
    and hardcoding it) keeps this test honest if the gains/limits ever
    change."""
    p = _pose({NOSE: (0.35, 0.35, 0.55), LEFT_EAR: (0.05, 0.10, 0.60),
               RIGHT_EAR: (0.20, -0.05, 0.62)})
    ref_targets, _ref_info = retarget(obs(p), mirror=True, mirror_fn=mirror_vec)
    ref_yaw = ref_targets["head_2_joint"]
    assert ref_yaw < -0.05, "case_head_turn's own pose must yield a clearly negative yaw"

    def assert_fn(targets, info):
        yaw = targets["head_2_joint"]
        ok = abs(yaw - ref_yaw) < 0.03
        return ok, f"head_2_joint(yaw)={yaw:+.4f} (want within 0.03 rad of {ref_yaw:+.4f})"

    return p, assert_fn, None


def case_torso_lean():
    """Human leans forward and twists toward their own left. Correct
    mirror: torso_2 (pitch) stays a FORWARD lean either way (never
    negated); torso_1 (yaw) flips sign.

    Torso lean is the ONE signal that lives in obs["basis"] itself, not a
    landmark pair (see _basis_from_forward's docstring and retarget.py's
    _torso_angles) -- so unlike every other case here, this one must set
    "basis" directly rather than relying on shoulder landmarks (which this
    bench's identity-basis obs() never turns into a lean at all)."""
    forward = (0.85, 0.35, -0.30)  # forward + toward own left + leaning fwd
    basis = _basis_from_forward(forward)
    p = dict(NEUTRAL)
    ref_targets, _ref_info = retarget(obs(p, basis=basis), mirror=True, mirror_fn=mirror_vec)
    ref_yaw = ref_targets["torso_1_joint"]
    assert ref_yaw < -0.03, "case_torso_lean's own pose must yield a clearly negative yaw"

    def assert_fn(targets, info):
        # Same "tight window around THE MIRROR LAW's own answer" reasoning
        # as case_head_turn: negate_x_too still lands on a NEGATIVE yaw here
        # (just driven to a different magnitude, same as the head), so a
        # bare "yaw < -0.03" threshold would not discriminate it. Pitch
        # keeps the loose one-sided check -- THE MIRROR LAW never touches
        # pitch, under EITHER mutation, so there is nothing to discriminate
        # there and no reason to tighten it.
        yaw, pitch = targets["torso_1_joint"], targets["torso_2_joint"]
        ok = abs(yaw - ref_yaw) < 0.03 and pitch > 0.03
        return ok, (f"torso yaw={yaw:+.4f} pitch={pitch:+.4f} "
                    f"(want yaw within 0.03 of {ref_yaw:+.4f}, pitch>0.03)")

    return p, assert_fn, basis


CASES = [
    ("forward reach LEFT arm (diagonal)", case_forward_reach_left, True),
    ("forward reach RIGHT arm (diagonal)", case_forward_reach_right, True),
    ("forward step LEFT leg (diagonal)", case_forward_step_left, True),
    ("forward step RIGHT leg (diagonal)", case_forward_step_right, True),
    ("sideways raise (DEGENERATE vs negate_x_too, documented)", case_sideways_raise, False),
    ("head turn", case_head_turn, True),
    ("torso lean", case_torso_lean, True),
]


def run_discriminating_cases():
    print("\nDISCRIMINATING CASES vs THE MIRROR LAW and its two named mutations")
    tally = {name: {"escaped": 0, "killed": 0} for name in MUTATIONS}
    for label, builder, expect_kills_negate_x in CASES:
        points, assert_fn, basis = builder()

        targets, info = _run(points, mirror_fn=mirror_vec, basis=basis)
        ok, detail = assert_fn(targets, info)
        check(ok, f"{label}: correct under THE MIRROR LAW", detail)

        for mut_name, mut_fn in MUTATIONS.items():
            mtargets, minfo = _run(points, mirror_fn=mut_fn, basis=basis)
            mok, mdetail = assert_fn(mtargets, minfo)
            killed = ok and not mok  # the correct answer passed, the mutant's failed
            if mut_name == "negate_x_too" and not expect_kills_negate_x:
                # THE documented exception -- report, do not fail the bench.
                print(f"  [INFO] {label}: negate_x_too mutation "
                      f"{'ESCAPED (expected -- this is the degenerate case)' if not killed else 'unexpectedly killed too'} "
                      f"-- {mdetail}")
                if killed:
                    tally[mut_name]["killed"] += 1
                else:
                    tally[mut_name]["escaped"] += 1
                continue
            check(killed, f"{label}: {mut_name} mutation is CAUGHT (assertion flips to FAIL)",
                  mdetail)
            tally[mut_name]["killed" if killed else "escaped"] += 1

    print("\nmutation-kill tally (of {} cases, excluding the documented sideways-raise "
          "exception for negate_x_too)".format(len(CASES)))
    for mut_name, counts in tally.items():
        total = counts["killed"] + counts["escaped"]
        print(f"  {mut_name:14s} killed={counts['killed']}/{total}  escaped={counts['escaped']}/{total}")
    return tally


# =========================================================================
# ROUND-TRIP + KNOWN ANSWERS, pure math
# =========================================================================
def run_math(samples):
    rng = random.Random(7)

    print("\nFK sanity (zero pose)")
    ua, fa = arm_dirs([0, 0, 0, 0])
    check(abs(ua[1]) < 1e-9 and ua[2] < -0.9, "arm q=0: hangs mostly straight down",
          f"{tuple(round(v,4) for v in ua)}")
    zig = ang(ua, fa)
    check(5.0 < zig < 12.0, "the wrist-roll offset makes the joint centres zigzag at q=0",
          f"{zig:.2f} deg -- small but real, from the -0.02 m x offset in D5")

    ta, sa = leg_dirs([0, 0, 0, 0])
    check(ta == (0.0, 0.0, -1.0) and sa == (0.0, 0.0, -1.0),
          "leg q=0: hangs EXACTLY straight down (no offset baggage in the hip/knee chain)",
          f"thigh={ta} shin={sa}")

    print("\nROUND-TRIP: solve_arm()/solve_leg_hip_knee() reproduce the direction asked for")
    worst_arm, worst_leg, tested = {"left": 0.0, "right": 0.0}, {"left": 0.0, "right": 0.0}, 0
    for side in ("left", "right"):
        for _ in range(samples):
            q = [rng.uniform(lo + 0.15, hi - 0.15) for lo, hi in ARM_LIMITS[side]]
            a, b = arm_dirs(q, side)
            _qs, info = solve_arm(a, b, side)
            worst_arm[side] = max(worst_arm[side], info["err_deg"])
        check(worst_arm[side] < 1.0, f"arm {side}: round-trip error under 1 deg",
              f"worst {worst_arm[side]:.4f} deg over {samples} poses")
        for _ in range(samples):
            q = [rng.uniform(lo + 0.15, hi - 0.15) for lo, hi in LEG_LIMITS[side]]
            a, b = leg_dirs(q, side)
            _qs, info = solve_leg_hip_knee(a, b, side)
            worst_leg[side] = max(worst_leg[side], info["err_deg"])
        check(worst_leg[side] < 1.0, f"leg {side}: round-trip error under 1 deg",
              f"worst {worst_leg[side]:.4f} deg over {samples} poses")
        tested += 2 * samples

    print("\nsign convention: arm shoulder roll is opposite-signed for a symmetric raise")
    ql, _ = solve_arm((0.0, 0.9, -0.44), (0.0, 0.9, -0.44), "left")
    qr, _ = solve_arm((0.0, -0.9, -0.44), (0.0, -0.9, -0.44), "right")
    check(ql[1] > 0.3 and qr[1] < -0.3,
          "left and right shoulder-roll are OPPOSITE-SIGNED for a symmetric raise",
          f"q2_left={ql[1]:+.3f} q2_right={qr[1]:+.3f}")

    return {"roundtrip_worst_arm_deg": {k: round(v, 4) for k, v in worst_arm.items()},
            "roundtrip_worst_leg_deg": {k: round(v, 4) for k, v in worst_leg.items()},
            "roundtrip_samples": tested}


# =========================================================================
# CONTINUITY -- judged by TICK INDEX, never by arrival/wall time.
# =========================================================================
def run_continuity():
    print("\nCONTINUITY across gating transitions (by TICK INDEX, not wall-clock arrival)")
    rate_hz = 50.0
    max_step = max_step_for(rate_hz, 1.5)  # mirror_node's own default
    src = SyntheticBodyPoseSource(period_s=16.0)
    n_ticks = 400
    # Force leg_l/leg_r OUT of gate for a window in the middle -- the
    # transition into and out of that window is exactly the "gating
    # transition" the task asks about: does the held joint jump when the
    # limb reappears?
    drop_start, drop_end = 150, 220

    q_cmd = None
    biggest = {}
    from talos_mirror.body_track import observation_dict
    from talos_mirror.talos_config import ALL_JOINTS, HOME_POSE, GROUP_JOINTS

    home = {}
    for group, joints in GROUP_JOINTS.items():
        for name, value in zip(joints, HOME_POSE[group]):
            home[name] = value

    for i in range(n_ticks):
        t = i / rate_hz  # tick INDEX drives synthetic time deterministically;
        # nothing here ever reads a wall clock.
        raw_obs = src.read(t)
        d = observation_dict(raw_obs, min_visibility=0.5)
        if drop_start <= i < drop_end:
            d["gates"]["leg_l"] = False
            d["gates"]["leg_r"] = False

        targets, _info = retarget(d, mirror=True, prev=q_cmd)
        if q_cmd is None:
            q_cmd = dict(home)
        for j in ALL_JOINTS:
            if j not in targets:
                continue  # held -- must not move at all this tick
            target = targets[j]
            before = q_cmd[j]
            q_cmd[j] = slew(target, before, max_step)
            delta = abs(q_cmd[j] - before)
            biggest[j] = max(biggest.get(j, 0.0), delta)

    worst_joint = max(biggest, key=biggest.get)
    worst = biggest[worst_joint]
    check(worst <= 0.04 + 1e-9,
          f"no per-tick jump exceeds 0.04 rad across {n_ticks} ticks "
          f"(incl. a leg gate-out/gate-in transition at ticks {drop_start}/{drop_end})",
          f"worst = {worst:.5f} rad on {worst_joint} (slew cap {max_step:.5f} rad/tick)")
    return {"continuity_max_rad": round(worst, 5), "continuity_ticks": n_ticks,
            "continuity_max_step_rad": round(max_step, 5)}


# ----------------------------------------------------------------------- fk
def run_fk(samples):
    """Ground truth: compare arm_dirs()/leg_dirs() against MoveIt's own
    /compute_fk. Needs `ros2-arm mirror` or `ros2-arm talos` running."""
    import rclpy
    from moveit_msgs.srv import GetPositionFK
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header

    from talos_mirror.retarget import _sub, _unit

    rclpy.init()
    node = Node("retarget_bench_fk")
    cli = node.create_client(GetPositionFK, "/compute_fk")
    if not cli.wait_for_service(timeout_sec=20.0):
        check(False, "/compute_fk available", "is `ros2-arm mirror` or `talos` running?")
        node.destroy_node()
        rclpy.shutdown()
        return {}

    rng = random.Random(20260727)
    worst = {"arm_left": 0.0, "arm_right": 0.0, "leg_left": 0.0, "leg_right": 0.0}
    n_ok = 0
    # First link in each triple is the anchor arm_dirs()/leg_dirs() actually
    # use for the FIRST direction (upper-arm / thigh), which is the SHOULDER
    # (joint2 origin, "arm_*_2_link") for arms but joint1's own origin for
    # legs -- see retarget.py's geometry docstring: the arm has a real
    # ~0.137 m offset between joint1 and joint2 (D2), so anchoring at link1
    # instead of link2 measures a DIFFERENT segment entirely (this shipped
    # once as a bench bug: it produced a spurious 26-27 deg "FK mismatch"
    # that was actually a mismatched anchor, not a wrong model -- caught
    # only because leg_left/leg_right's zero-offset hip agreed with MoveIt
    # to 0.0000 deg while arm_left/arm_right did not, which is what made it
    # obviously a comparison-setup bug and not a geometry bug). The leg's
    # hip joints 1/2/3 all share ONE physical location (d2=d3=0), so link1
    # is correct there.
    chains = {
        "arm_left": (ARM_LIMITS["left"], ["arm_left_2_link", "arm_left_4_link", "arm_left_7_link"],
                     [f"arm_left_{i}_joint" for i in range(1, 8)]),
        "arm_right": (ARM_LIMITS["right"], ["arm_right_2_link", "arm_right_4_link", "arm_right_7_link"],
                      [f"arm_right_{i}_joint" for i in range(1, 8)]),
        "leg_left": (LEG_LIMITS["left"], ["leg_left_1_link", "leg_left_4_link", "leg_left_6_link"],
                     [f"leg_left_{i}_joint" for i in range(1, 7)]),
        "leg_right": (LEG_LIMITS["right"], ["leg_right_1_link", "leg_right_4_link", "leg_right_6_link"],
                      [f"leg_right_{i}_joint" for i in range(1, 7)]),
    }
    # PIN THE TORSO. /compute_fk fills any joint NOT named in the request
    # from the CURRENT planning-scene state, not from zero -- and when this
    # tier is run against a live `mirror` session (as the task asks: verify
    # the synthetic session, then cross-check FK against it), the torso is
    # actively moving. Two arm links both inherit that same live torso
    # rotation, so their DIFFERENCE vector is NOT torso-invariant (a rigid
    # rotation applied to both endpoints rotates the difference too; only a
    # rigid TRANSLATION would cancel) -- while arm_dirs()/leg_dirs() compute
    # everything relative to a torso pinned at zero. Omitting this shipped
    # once as a spurious ~2.9 deg "FK mismatch" that reproduced at ANY joint
    # sample (a real bug would not), and vanished to EXACTLY 0.0000 deg the
    # moment torso_1_joint/torso_2_joint were added to the request -- the
    # tell that it was a comparison-setup bug, not a geometry bug.
    torso_joint_names = ["torso_1_joint", "torso_2_joint"]
    for key, (limits4, links, joint_names) in chains.items():
        side = "left" if key.endswith("left") else "right"
        for _ in range(samples):
            q4 = [rng.uniform(lo + 0.1, hi - 0.1) for lo, hi in limits4]
            q_full = list(q4) + [0.0] * (len(joint_names) - 4)
            req = GetPositionFK.Request()
            req.header = Header(frame_id="base_link")
            req.fk_link_names = links
            req.robot_state.joint_state = JointState(
                name=joint_names + torso_joint_names,
                position=q_full + [0.0, 0.0],
            )
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            res = fut.result()
            if res is None or res.error_code.val != 1 or len(res.pose_stamped) != 3:
                continue
            P = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                 for p in res.pose_stamped]
            moveit_a = _unit(_sub(P[1], P[0]))
            moveit_b = _unit(_sub(P[2], P[1]))
            ours_a, ours_b = (arm_dirs(q4, side) if key.startswith("arm")
                               else leg_dirs(q4, side))
            worst[key] = max(worst[key], ang(moveit_a, ours_a), ang(moveit_b, ours_b))
            n_ok += 1

    node.destroy_node()
    rclpy.shutdown()
    print()
    check(n_ok >= samples, "compute_fk answered", f"{n_ok}/{samples * 4} samples")
    for key in chains:
        check(worst[key] < 1.0, f"{key}: our FK direction matches MoveIt /compute_fk",
              f"worst {worst[key]:.4f} deg")
    return {"fk_worst_deg": {k: round(v, 4) for k, v in worst.items()}, "fk_samples": n_ok}


# ----------------------------------------------------------------------- ros
def run_ros(seconds):
    import time

    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory

    from talos_mirror.talos_config import CONTROLLER_TOPICS

    rclpy.init()
    node = Node("retarget_bench_ros")
    got = {g: 0 for g in CONTROLLER_TOPICS}
    bad = []

    def cb(group, msg):
        got[group] += 1
        for n, p in zip(msg.joint_names, msg.points[0].positions):
            lo, hi, _v = MEASURED[n]
            if not (lo - 1e-6 <= p <= hi + 1e-6):
                bad.append(f"{n}={p:.4f} outside [{lo}, {hi}]")

    for group, topic in CONTROLLER_TOPICS.items():
        node.create_subscription(JointTrajectory, topic, lambda m, g=group: cb(g, m), 20)

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    print()
    for group, count in got.items():
        check(count > 5, f"{group} is being commanded", f"{count} msgs")
    check(not bad, "every commanded angle is inside its URDF limit",
          f"{len(bad)} violations" + (f" e.g. {bad[0]}" if bad else ""))
    return {"ros_msgs": got, "ros_limit_violations": len(bad)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fk", action="store_true", help="compare against MoveIt /compute_fk")
    ap.add_argument("--ros", action="store_true", help="check live commands (needs `mirror`)")
    ap.add_argument("--samples", type=int, default=150)
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()

    print("=" * 72)
    print("  talos_mirror M4 verification -- full-body retargeting")
    print("=" * 72)

    extra = run_math(args.samples)
    extra.update(run_continuity())
    tally = run_discriminating_cases()
    extra["mutation_tally"] = tally
    if args.fk:
        extra.update(run_fk(min(args.samples, 40)) or {})
    if args.ros:
        extra.update(run_ros(args.seconds) or {})

    print("\n" + "=" * 72)
    if FAILURES:
        print(f"  FAILED -- {len(FAILURES)} check(s):")
        for f in FAILURES:
            print(f"    - {f}")
    else:
        print("  ALL CHECKS PASSED")
    print("=" * 72)
    print("RESULT:" + json.dumps({"passed": not FAILURES, "failures": FAILURES[:20], **extra}))
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
