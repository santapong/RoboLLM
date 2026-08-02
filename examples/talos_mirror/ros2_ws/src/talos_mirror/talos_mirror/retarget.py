"""Human body observation -> TALOS joint targets. FULL-BODY DIRECT joint
retargeting: shoulder(3)+elbow for each arm (wrists parked), pan/tilt head,
lean yaw/pitch torso, and hip(3)+knee+ankle-pitch for each leg (ankle roll
parked, pelvis PINNED).

Pure math (no ROS, no numpy, no mediapipe) so it is unit-testable offline.
Ported from examples/humanoid_mirror's retarget.py + ffw_arm.py solve_arm and
re-derived against TALOS's OWN geometry (talos_description/urdf/{arm,leg}/
*.urdf.xacro, calibration offsets all zero -- confirmed by reading
talos_description_calibration/urdf/calibration/{arm,leg}_{left,right}.xacro).
This module makes NO runtime import from humanoid_mirror.

=============================================================================
THE MIRROR LAW (identical derivation to humanoid_mirror/retarget.py -- read
it there for the full geometric proof; restated here because it governs
LEGS too now, which upstream never had)
=============================================================================
Put the robot at the origin facing +x with up +z, and the human in front of
it facing back. The human's forward is -x_world; for a right-handed body
frame (x fwd, y left, z up) the human's LEFT is -y_world. A human-frame
vector (hx, hy, hz) is therefore (-hx, -hy, hz) in WORLD.

A mirror is a REFLECTION through the vertical plane between the two of you,
which negates the world axis joining you (x) and leaves y, z alone:
    world (-hx, -hy, hz)  ->  reflected (hx, -hy, hz)
The robot's own axes coincide with world, so the robot-frame target is

    (hx, hy, hz)  ->  (hx, -hy, hz)      NEGATE Y ONLY

fed to the OPPOSITE limb (arm or leg).

Negating x too is the historic bug (it shipped once in humanoid_mirror) and
is INVISIBLE on a sideways raise/step -- (0, 1, 0) maps to (0, -1, 0) either
way. It only shows on FORWARD motions: reaching or STEPPING forward, (1, 0,
0), must stay (1, 0, 0); an extra x flip sends the robot's limb BACKWARD
while the human moves forward. retarget-bench checks this for arms (as
upstream did) AND for legs (new): a forward step with the human's LEFT leg
must produce a forward step of the robot's RIGHT leg, on the SAME side of
the room, with the knee bending the same physical way (forward), not just
"some limb moved". retarget-bench also runs a "swap-only" mutation (sides
swap but nothing is ever negated) to prove the discriminating cases would
catch that failure mode too, not just the x-flip one.

DIRECT MODE ("the robot is you, seen from behind") is mirror_fn=identity with
the side swap turned off; see retarget()'s `mirror` parameter.

ONE function (mirror_vec, or whatever is passed as mirror_fn) is applied
EVERYWHERE a human-frame direction becomes a robot-frame direction -- arms,
legs, head, torso -- so none of them can disagree about the convention. This
is also what makes the mutation tests in retarget-bench possible: swapping
mirror_fn for a deliberately-wrong one exercises every limb through the same
single seam.

=============================================================================
GEOMETRY (from talos_description/urdf/arm/{arm,wrist}.urdf.xacro and
urdf/leg/leg.urdf.xacro; every calibration offset is 0.0)
=============================================================================
ARM (torso_2_link -> arm_{side}_1..7_link), reflect = +1 left, -1 right:
  joint1 (shoulder yaw,  axis z) origin (0,       0.1575*r, 0.232 )
  joint2 (shoulder roll, axis x) origin (0.00493, 0.1365*r, 0.04673), rel link1
  joint3 (humeral yaw,   axis z) origin (0, 0, 0),                    rel link2
  joint4 (ELBOW,         axis y) origin (0.02, 0, -0.2782),           rel link3
  joint5 (wrist roll,    axis z) origin (-0.02, 0, -0.2643),          rel link4
  joint6 (wrist pitch,   axis x) origin (0, 0, 0),                    rel link5
  joint7 (wrist yaw,     axis y) origin (0, 0, 0),                    rel link6
Only joint1/joint2's Y origin and the joint LIMITS mirror by side -- joints
3-7's origins carry NO reflect factor anywhere in the xacro, so (exactly the
humanoid_mirror finding for FFW) the two arms are geometrically IDENTICAL
below the shoulder; only the base offset and the clamp differ.

THE STRUCTURE THAT MATTERS: outer=Rz(q1), inner=Rx(q2) is a "shoulder gimbal"
that reproduces ANY direction EXACTLY (2 DOF onto a 2-DOF target); q3 (also
axis z, sitting BELOW the gimbal) rotates the elbow offset onto a cone and is
the free/searched variable, exactly as ffw_arm.py's q3 was -- the same
"exact-2 then free-1" structure, just with the outer axis relabelled Z
instead of Y (TALOS's shoulder yaws about vertical where FFW's pitched about
horizontal; the closed-form math is isomorphic, only which vector component
is rotation-invariant changes: Z-outer leaves a_z alone instead of a_y).

LEG (base_link -> leg_{side}_1..6_link, PELVIS PINNED -- base_link never
moves), reflect = +1 left, -1 right:
  joint1 (hip yaw,   axis z) origin (-0.02, 0.085*r, -0.27105)
  joint2 (hip roll,  axis x) origin (0, 0, 0),      rel link1
  joint3 (hip pitch, axis y) origin (0, 0, 0),      rel link2
  joint4 (KNEE,       axis y) origin (0, 0, -0.38),  rel link3
  joint5 (ankle pitch,axis y) origin (0, 0, -0.325), rel link4
  joint6 (ankle roll, axis x) origin (0, 0, 0),      rel link5
Joints 2-6 origins carry no reflect factor either -- only joint1's Y origin
and the joint1 LIMIT range mirror. The hip is a TRUE 3-DOF ball joint at a
single FIXED pivot (d2=d3=0, unlike the arm's non-zero D2): joint1/2/3 all
share one physical location, they only differ in which axis they spin about.

NOT the same decomposition as the arm, despite the superficial resemblance
(both are "outer Z / inner X / a third rotation / then a Y flex"). The arm's
free joint (q3, humeral yaw, axis Z) spins an elbow offset that points
mostly ALONG that same Z axis -- a small wobble, never near-singular. The
leg's equivalent-looking free-axis candidate (hip pitch, axis Y) would spin
the knee offset D4=(0,0,-0.38), which points PERPENDICULAR to that axis, all
the way through the exactly-level thigh pose -- a real, frequently-reached
teleop pose (a high march step), not an edge case -- where that gimbal
genuinely loses a degree of freedom. See solve_leg_hip_knee()'s own
docstring for the closed form actually used (hip YAW is the swept/free
variable instead; roll+pitch are solved as an exact pair per candidate yaw).
The flex joint (knee) is still axis Y like the arm's elbow, and the
knee->ankle offset (0,0,-0.325) has NO x-offset at all (unlike the arm's
wrist-roll offset, which has -0.02 m), so the knee's "F0" is exactly
(0,0,-1) -- reusing _flex_angle() as a strictly simpler special case, not a
different formula.

ANKLE: only PITCH is solved from landmarks (heel/toe give exactly one
observable direction -- the foot's own forward axis -- and a foot-forward
target is PROVABLY invariant to rotation about that same axis: Rx(roll) *
(1,0,0) = (1,0,0) for ANY roll, a geometric fact, not a solver gap). Ankle
ROLL is parked at 0.0, on the same footing as the arm's wrist joints ("to
neutral for now -- the QP task upgrades wrists later"): there is no landmark
pair this milestone tracks that can see it.

HEAD/ARMS attach to torso_2_link (MoveIt SRDF base links), so their targets
are TORSO-RELATIVE -- obs.basis rotates the raw landmark offsets into the
torso frame before matching, exactly as humanoid_mirror did. LEGS attach to
base_link (the PINNED pelvis), which never rotates with the torso, so leg
targets are computed in the RAW body/camera axes (no torso-frame rotation)
-- using the torso-relative direction for a pinned-base limb would silently
smuggle the human's torso lean into the stepping motion, double-counting it
against the separate torso_1/torso_2 lean targets below. This is a direct
consequence of talos_config.BASE_LINKS, not a free design choice.
"""

import math

from talos_mirror.body_track import (
    LEFT_ANKLE,
    LEFT_ELBOW,
    LEFT_FOOT_INDEX,
    LEFT_HEEL,
    LEFT_HIP,
    LEFT_KNEE,
    LEFT_SHOULDER,
    LEFT_WRIST,
    RIGHT_ANKLE,
    RIGHT_ELBOW,
    RIGHT_FOOT_INDEX,
    RIGHT_HEEL,
    RIGHT_HIP,
    RIGHT_KNEE,
    RIGHT_SHOULDER,
    RIGHT_WRIST,
    BodyObservation,
)
from talos_mirror.joint_limits import MEASURED

# --------------------------------------------------------------- vector math
# Deliberately re-implemented here (not imported from body_track) for the
# handful of ops retarget.py needs on plain tuples that are NOT already body
# frame landmarks -- rotation-by-angle helpers in particular have no home in
# body_track.py, which only ever composes a fixed basis, never a joint angle.


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _unit(v):
    n = math.sqrt(_dot(v, v))
    return (0.0, 0.0, 0.0) if n < 1e-12 else (v[0] / n, v[1] / n, v[2] / n)


def _rx(v, t):
    c, s = math.cos(t), math.sin(t)
    return (v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c)


def _ry(v, t):
    c, s = math.cos(t), math.sin(t)
    return (v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c)


def _rz(v, t):
    c, s = math.cos(t), math.sin(t)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2])


def _wrap(t):
    return (t + math.pi) % (2.0 * math.pi) - math.pi


def ang_between(u, v):
    """Angle in radians between two (near-)unit vectors, numerically safe."""
    return math.acos(max(-1.0, min(1.0, _dot(_unit(u), _unit(v)))))


def _clip(v, lo, hi):
    return min(max(v, lo), hi)


# ------------------------------------------------------------- THE MIRROR LAW
def mirror_vec(v):
    """Human-frame direction -> robot-frame direction, mirrored.

    Negates Y ONLY. See the module docstring for the derivation. Used for
    EVERY limb (arms, legs, head, torso) so none of them can disagree about
    the convention -- and so retarget-bench can swap this one function for a
    deliberately wrong one and watch every discriminating case fail.
    """
    return (v[0], -v[1], v[2])


def _mirror_bad_negate_xy(v):
    """MUTATION #1 for retarget-bench: the historic bug, negating x too."""
    return (-v[0], -v[1], v[2])


def _mirror_bad_swap_only(v):
    """MUTATION #2 for retarget-bench: sides still swap, nothing negates."""
    return (v[0], v[1], v[2])


# ============================================================== ARM (4-DOF)
# reflect-independent below the shoulder (see module docstring).
ARM_D4 = (0.02, 0.0, -0.2782)                       # joint3 -> joint4 (elbow)
ARM_F0 = _unit((-0.02, 0.0, -0.2643))                # joint4 -> joint5, wrist parked
ARM_LIMITS = {
    side: [MEASURED[f"arm_{'left' if side == 'left' else 'right'}_{k}_joint"][:2]
           for k in range(1, 5)]
    for side in ("left", "right")
}

# ============================================================= LEG (4+1 DOF)
LEG_D4 = (0.0, 0.0, -0.38)                           # hip3 -> knee
LEG_F0 = _unit((0.0, 0.0, -0.325))                   # knee -> ankle5, i.e. (0,0,-1)
LEG_LIMITS = {
    side: [MEASURED[f"leg_{'left' if side == 'left' else 'right'}_{k}_joint"][:2]
           for k in range(1, 5)]
    for side in ("left", "right")
}
ANKLE_PITCH_LIMITS = {
    side: MEASURED[f"leg_{'left' if side == 'left' else 'right'}_5_joint"][:2]
    for side in ("left", "right")
}

# Below this angle between the two target directions, the "free" axis
# (humeral yaw / hip pitch) is unobservable -- see solve_arm/solve_leg's
# docstrings and ffw_arm.py's identical STRAIGHT_EPS for the full argument.
STRAIGHT_EPS = 0.06


def _pick_gimbal(a, w):
    """Both (q1, q2) solutions for outer=Rz(q1), inner=Rx(q2) reproducing
    direction `a` from the FIXED vector `w`. Returns [branch0, branch1],
    either entry None if `a` is outside this gimbal's reachable cone.

    Derivation: v = Rx(w, q2), a = Rz(v, q1). Rz leaves the z component
    alone, so a_z = v_z = w_y sin q2 + w_z cos q2 = R cos(q2 - psi), with
    R = hypot(w_y, w_z), psi = atan2(w_y, w_z). That isolates q2 (the usual
    two-branch acos ambiguity, q2 = psi +- acos(a_z / R)) independent of q1.
    With q2 fixed, v is fully known and a_x, a_y are v_x, v_y rotated by q1
    in the x-y plane, so q1 = atan2(a_y, a_x) - atan2(v_y, v_x) -- no
    further ambiguity.

    ACOS, NOT ASIN. An earlier version of this function used the equally
    valid asin form (q2 = asin(c) - phi, pi - asin(c) - phi): mathematically
    correct pointwise, but WRONG as a branch LABEL, because asin's second
    branch (pi - x) wraps around +-pi as q2 sweeps near there, so "branch 1"
    silently jumps onto the physically different acos-branch solution
    between adjacent q3 grid samples. _solve4's search treats branch 0/1 as
    stable identities to bisect a residual against a FIXED branch (see its
    own docstring and ffw_arm.py's shoulder_branches: "keeping the branches
    SEPARATE and in a stable order matters... a helper silently switched
    branches midway [makes] the residual jump, manufacturing fake roots and
    hiding real ones"). Measured: this cost 24 deg of round-trip error on the
    leg solver (whose hip roll range, +-30 deg, sits close enough to the
    asin wrap that the swap happened inside a normal sample bracket) before
    the fix -- caught by retarget-bench's own round-trip check, not by
    inspection.
    """
    r = math.hypot(w[1], w[2])
    if r < 1e-9:
        return [None, None]
    psi = math.atan2(w[1], w[2])
    c = max(-1.0, min(1.0, a[2] / r))
    half = math.acos(c)
    out = []
    for q2 in (psi + half, psi - half):
        v = _rx(w, q2)
        q1 = math.atan2(a[1], a[0]) - math.atan2(v[1], v[0])
        out.append((_wrap(q1), _wrap(q2)))
    return out


def _flex_angle(f0, local):
    """q such that Ry(q) * f0 matches local's (x, z) footprint; local[1] (the
    y component) is the out-of-plane residual a q3 search should zero out.

    Shared by the arm's elbow (f0 has a small x offset, a real ~4.3 deg
    zigzag) and the leg's knee (f0 = (0,0,-1) exactly, x offset zero) -- the
    same formula, the leg is simply the case where f0's x term vanishes.
    """
    a0 = math.atan2(f0[0], -f0[2])
    at = math.atan2(local[0], -local[2])
    return _wrap(a0 - at)


def _solve4(a, b, limits4, free_rotate, d4, f0, q_seed=None, margin=0.05):
    """Shared 4-DOF solve: outer=Rz(q1), inner=Rx(q2), free=free_rotate(q3)
    applied to d4 gives w(q3) which the gimbal matches to `a` exactly for any
    q3; flex=Ry(q4) applied to f0 (in the frame q1,q2,q3 undo) matches `b`.
    q3 is swept/bisected to zero the out-of-plane residual -- see ffw_arm.py's
    solve_arm docstring for why this has to be a 1-D search rather than
    alternating closed-form blocks (the coupling through the elbow/knee
    offset is not weak).

    `free_rotate` is _rz for the arm (humeral yaw, axis z) or _ry for the leg
    (hip pitch, axis y) -- the ONLY structural difference between the two
    limbs; everything else in this function is shared.

    Returns (q1..q4, info) with info["err_deg"], ["clamped"], ["degenerate"].
    """
    prev = list(q_seed[:4]) if q_seed is not None and len(q_seed) >= 4 \
        else [0.0, 0.0, 0.0, 0.0]
    degenerate = ang_between(a, b) < STRAIGHT_EPS

    def evaluate(q3, branch):
        w = _unit(free_rotate(d4, q3))
        sol = _pick_gimbal(a, w)[branch]
        if sol is None:
            return None, None
        q1, q2 = sol
        b1 = _rz(b, -q1)
        b2 = _rx(b1, -q2)
        b3 = free_rotate(b2, -q3)
        q4 = _flex_angle(f0, b3)
        return b3[1], [q1, q2, _wrap(q3), q4]

    lo3, hi3 = limits4[2]
    steps = 24
    grid = [lo3 + (hi3 - lo3) * i / steps for i in range(steps + 1)]
    if degenerate:
        grid = grid + [prev[2]]

    solutions = []
    for branch in (0, 1):
        samples = [(g,) + evaluate(g, branch) for g in grid]
        for _g, _r, qv in samples:
            if qv is not None:
                solutions.append(qv)
        if degenerate:
            continue
        for (g0, r0, _q0), (g1, r1, _q1) in zip(samples, samples[1:]):
            if r0 is None or r1 is None:
                continue
            if r0 == 0.0 or r0 * r1 < 0.0:
                lo_g, hi_g, ra = g0, g1, r0
                # T1 (loop-algo A2, round 3 -- corrects a wrong number round
                # 2 shipped in this same comment): 24 -> 12 bisection
                # halvings, ARM SOLVER ONLY. Round-1's identical cut to the
                # LEG solver (solve_leg_hip_knee below) was REFUTED twice: it
                # turns `retarget-bench --samples 2000` from a passing gate
                # into a failing one (leg_right round-trip 0.0520 -> 1.4367
                # deg, > the tool's own 1 deg bar; leg_left 0.0573 -> 0.9625
                # deg, within 4% of it) at the leg's documented hip-pitch
                # singularity (see solve_leg_hip_knee()'s docstring), so the
                # leg loop below is back to 24 halvings and this 12-halving
                # cut applies to the arm only. The arm has NO such
                # singularity (see _solve4's/solve_arm's own docstrings) and
                # is empirically free here: re-derived on the A1 golden
                # corpus (601-tick fixture, tools/diff_replay.py) with T2's
                # round-3 cache-key fix in place (so this is now the
                # ISOLATED T1 signal, not conflated with a T2 defect -- see
                # MirrorPoseSource's own comments), the worst per-joint
                # deviation this halving introduces is 8.0214e-4 deg on
                # arm_left_3_joint (tick 96) and 8.0214e-4 deg on
                # arm_right_3_joint (tick 398) -- NOT tick 234 as a prior
                # round of this comment claimed, an unverified number caught
                # by the round-3 gate; every other group (torso, head,
                # leg_l, leg_r) exactly 0.0 deg -- MEASURED, diff-bench
                # compare against tools/golden_commanded.json, not the
                # catalog's refuted "0.0014 deg" figure (that number is this
                # same measurement's p99 over a wider paired 24-vs-12
                # sample, not a bound). retarget-bench --samples 2000 arm
                # round-trip is unchanged to 4 decimal places (arm_l 0.2079,
                # arm_r 0.3400 deg, bit-identical to the 24-halving
                # baseline).
                for _ in range(12):
                    m = 0.5 * (lo_g + hi_g)
                    rm, _q = evaluate(m, branch)
                    if rm is None:
                        break
                    if ra * rm <= 0.0:
                        hi_g = m
                    else:
                        lo_g, ra = m, rm
                _r, qv = evaluate(0.5 * (lo_g + hi_g), branch)
                if qv is not None:
                    solutions.append(qv)

    if not solutions:
        solutions = [list(prev[:4])]

    def _clamp4(qv):
        out = []
        for i, v in enumerate(qv):
            lo, hi = limits4[i]
            lo, hi = lo + margin, hi - margin
            out.append(min(max(v, lo), hi) if lo <= hi else 0.5 * (lo + hi))
        return out

    def _fk4(qv):
        w = _unit(free_rotate(d4, qv[2]))
        v = _rx(w, qv[1])
        dir_a = _unit(_rz(v, qv[0]))
        fb = _ry(f0, qv[3])
        fb = free_rotate(fb, qv[2])
        fb = _rx(fb, qv[1])
        dir_b = _unit(_rz(fb, qv[0]))
        return dir_a, dir_b

    def rank(qv):
        qc = _clamp4(qv)
        ua, fa = _fk4(qc)
        err = max(ang_between(ua, a), ang_between(fa, b))
        drift = sum(abs(_wrap(qc[k] - prev[k])) for k in range(4))
        return err, drift

    ranked = [(rank(qv), qv) for qv in solutions]
    best_err = min(r[0][0] for r in ranked)
    near = [r for r in ranked if r[0][0] <= best_err + 1e-3]
    q = _clamp4(min(near, key=lambda r: r[0][1])[1])

    ua, fa = _fk4(q)
    err = math.degrees(max(ang_between(ua, a), ang_between(fa, b)))
    clamped = any(
        abs(q[i] - (limits4[i][0] + margin)) < 1e-9
        or abs(q[i] - (limits4[i][1] - margin)) < 1e-9
        for i in range(4))
    return q, {"err_deg": err, "clamped": clamped, "degenerate": degenerate}


def arm_dirs(q, side="left"):
    """(upper_arm_unit, forearm_unit) an arm's q1..q4 produce -- for
    retarget-bench's round-trip check. side is accepted for API symmetry
    with the (side-independent) geometry; unused."""
    w = _unit(_rz(ARM_D4, q[2]))
    v = _rx(w, q[1])
    ua = _unit(_rz(v, q[0]))
    fb = _ry(ARM_F0, q[3])
    fb = _rz(fb, q[2])
    fb = _rx(fb, q[1])
    fa = _unit(_rz(fb, q[0]))
    return ua, fa


def leg_dirs(q, side="left"):
    """(thigh_unit, shin_unit) a leg's q1..q4 produce. side unused (see
    arm_dirs)."""
    ta = _unit(_rz(_rx(_ry((0.0, 0.0, -1.0), q[2]), q[1]), q[0]))
    fb = _ry((0.0, 0.0, -1.0), q[3])
    fb = _ry(fb, q[2])
    fb = _rx(fb, q[1])
    sa = _unit(_rz(fb, q[0]))
    return ta, sa


def solve_arm(a, b, side="left", q_seed=None, margin=0.05):
    """Joint angles q1..q4 (shoulder x3 + elbow) whose direction reproduces
    upper-arm `a` and forearm `b`. Direct joint-angle retargeting, matching
    DIRECTIONS -- invariant to the human's limb length, never handed an
    unreachable point. Wrist joints 5-7 are parked at 0.0 by the caller."""
    return _solve4(a, b, ARM_LIMITS[side], _rz, ARM_D4, ARM_F0, q_seed, margin)


def _hip_exact_pair(a_local):
    """Given `a_local` = thigh direction with hip YAW already undone (i.e.
    Rz(-q1) applied), the EXACT (roll, pitch) = (q2, q3) pair reproducing it
    from straight-down (0,0,-1). Two branches (asin ambiguity in q3).

    a_local = Rx(q2) * Ry(q3) * (0,0,-1)
            = (-sin q3, cos q3 * sin q2, -cos q3 * cos q2)
    so a_local_x = -sin q3 fixes q3 up to the usual reflection, and then
    sin q2 = a_local_y / cos q3, cos q2 = -a_local_z / cos q3 fixes q2 --
    q2 = atan2(a_local_y, -a_local_z), corrected by +pi when cos q3 < 0
    (dividing an atan2 pair by a negative constant rotates the angle by pi,
    not just its sign).
    """
    x, y, z = a_local
    x = max(-1.0, min(1.0, x))
    p = math.asin(-x)
    out = []
    for q3 in (p, math.pi - p):
        q3 = _wrap(q3)
        cq3 = math.cos(q3)
        if abs(cq3) < 1e-9:
            q2 = 0.0  # singular: roll unobservable when the thigh is exactly
                      # level in the (post-yaw) sagittal plane -- hold at 0
        else:
            q2 = math.atan2(y, -z)
            if cq3 < 0.0:
                q2 = _wrap(q2 + math.pi)
        out.append((_wrap(q2), q3))
    return out


def solve_leg_hip_knee(a, b, side="left", q_seed=None, margin=0.05):
    """Joint angles q1..q4 (hip x3 + knee) whose direction reproduces thigh
    `a` and shin `b`.

    NOT the same decomposition as solve_arm, despite the hip and shoulder
    sharing an outer=Rz/inner=Rx/free=?/flex=Ry axis LAYOUT. The arm's free
    joint (q3, humeral yaw, axis Z) rotates an elbow offset that is mostly
    ALONG that same Z axis (D4=(0.02,0,-0.2782), z dominant) -- rotating a
    vector about an axis it is nearly parallel to only wobbles it a little,
    so the "outer two solve `a` exactly, for ANY free q3" gimbal never gets
    near singular. The leg's free-axis candidate (hip pitch, axis Y) rotates
    the knee offset D4=(0,0,-0.38), which points STRAIGHT ALONG Z -- i.e.
    perpendicular to that rotation axis -- so sweeping it swings the thigh
    through the WHOLE sagittal plane, including exactly level (thigh
    horizontal), where the outer Rz/Rx gimbal genuinely loses a degree of
    freedom (its resolvent r = hypot(w_y, w_z) passes through 0). A "raise
    the knee to march" pose sits close enough to that region that the
    round-trip test measured 24-30 deg of error before this was caught.

    So here the SWEPT/free variable is q1 (hip YAW, axis Z, the outermost
    joint) instead, and q2, q3 (roll, pitch) are solved EXACTLY from the
    thigh direction for each candidate q1 via _hip_exact_pair -- a true
    closed form (the hip has no offset baggage at all: d2 = d3 = 0, a real
    3-DOF ball joint), not another search. The q1 sweep + bisection is only
    there to zero the SHIN's out-of-plane residual, exactly as ffw_arm.py's
    q3 sweep does for the elbow -- same overall shape, different joint
    plays the "free" role, because the geometry genuinely differs.
    """
    limits4 = LEG_LIMITS[side]
    prev = list(q_seed[:4]) if q_seed is not None and len(q_seed) >= 4 \
        else [0.0, 0.0, 0.0, 0.0]
    degenerate = ang_between(a, b) < STRAIGHT_EPS

    def evaluate(q1, branch):
        a_local = _rz(a, -q1)
        pair = _hip_exact_pair(a_local)[branch]
        q2, q3 = pair
        b1 = _rz(b, -q1)
        b2 = _rx(b1, -q2)
        b3 = _ry(b2, -q3)
        q4 = _flex_angle(LEG_F0, b3)
        return b3[1], [q1, q2, q3, q4]

    lo1, hi1 = limits4[0]
    steps = 24
    grid = [lo1 + (hi1 - lo1) * i / steps for i in range(steps + 1)]
    if degenerate:
        grid = grid + [prev[0]]

    solutions = []
    for branch in (0, 1):
        samples = [(g,) + evaluate(g, branch) for g in grid]
        for _g, _r, qv in samples:
            solutions.append(qv)
        if degenerate:
            continue
        for (g0, r0, _q0), (g1, r1, _q1) in zip(samples, samples[1:]):
            if r0 == 0.0 or r0 * r1 < 0.0:
                lo_g, hi_g, ra = g0, g1, r0
                # T1 (loop-algo A2, round 2): REVERTED to 24 bisection
                # halvings (round 1 cut this to 12; see git history / task
                # A2). Two independent re-gates demonstrated the cut is NOT
                # free here, reproducibly: `retarget-bench --samples 2000`
                # round-trip worst case rose from 0.0520 deg (24 halvings,
                # this baseline) to 1.4367 deg at 12 halvings on leg_right
                # (leg_left 0.0573 -> 0.9625 deg), breaching the tool's own
                # <1 deg gate; at 40k samples/side the tail reaches 14.0554
                # deg (leg_right) / 3.2533 deg (leg_left). Root cause is the
                # documented hip-pitch-near-+-90deg singularity above (a
                # "high march step", a realistic teleop pose, not an edge
                # case): near-zero cos(q3) in _hip_exact_pair's hip-roll
                # inversion amplifies a coarser q1 estimate there. The ARM
                # solver's identical-looking cut (_solve4 above) has no such
                # singularity and stays at 12 halvings -- see its comment
                # for the re-derived arm-only bound. This loop stays at the
                # pre-T1 halving count until/unless a leg-specific mechanism
                # (e.g. adaptively finer steps only near the singular
                # region) is proposed and re-gated.
                for _ in range(24):
                    m = 0.5 * (lo_g + hi_g)
                    rm, _q = evaluate(m, branch)
                    if ra * rm <= 0.0:
                        hi_g = m
                    else:
                        lo_g, ra = m, rm
                _r, qv = evaluate(0.5 * (lo_g + hi_g), branch)
                solutions.append(qv)

    def _clamp4(qv):
        out = []
        for i, v in enumerate(qv):
            lo, hi = limits4[i]
            lo, hi = lo + margin, hi - margin
            out.append(min(max(v, lo), hi) if lo <= hi else 0.5 * (lo + hi))
        return out

    def rank(qv):
        qc = _clamp4(qv)
        ta, sa = leg_dirs(qc, side)
        err = max(ang_between(ta, a), ang_between(sa, b))
        drift = sum(abs(_wrap(qc[k] - prev[k])) for k in range(4))
        return err, drift

    ranked = [(rank(qv), qv) for qv in solutions]
    best_err = min(r[0][0] for r in ranked)
    near = [r for r in ranked if r[0][0] <= best_err + 1e-3]
    q = _clamp4(min(near, key=lambda r: r[0][1])[1])

    ta, sa = leg_dirs(q, side)
    err = math.degrees(max(ang_between(ta, a), ang_between(sa, b)))
    clamped = any(
        abs(q[i] - (limits4[i][0] + margin)) < 1e-9
        or abs(q[i] - (limits4[i][1] - margin)) < 1e-9
        for i in range(4))
    return q, {"err_deg": err, "clamped": clamped, "degenerate": degenerate}


def solve_ankle_pitch(foot_dir, hip_knee_q, side="left", margin=0.05):
    """Ankle PITCH (q5) from the heel->toe direction, given the already-
    solved hip+knee angles. Ankle ROLL is not returned -- see the module
    docstring: Rx(roll) * (1,0,0) = (1,0,0) for any roll, so a foot-forward
    target can never observe it; the caller parks roll at 0.0.

    `foot_dir` must be in the SAME frame `a`/`b` were passed to
    solve_leg_hip_knee in (raw body axes for TALOS's pinned-pelvis legs)."""
    q1, q2, q3, q4 = hip_knee_q
    c1 = _rz(foot_dir, -q1)
    c2 = _rx(c1, -q2)
    c3 = _ry(c2, -q3)
    c4 = _ry(c3, -q4)
    q5 = math.atan2(-c4[2], c4[0])
    lo, hi = ANKLE_PITCH_LIMITS[side]
    return _clip(q5, lo + margin, hi - margin)


# =================================================================== HEAD
HEAD_PITCH_LIMITS = MEASURED["head_1_joint"][:2]   # axis y, + assumed = down
HEAD_YAW_LIMITS = MEASURED["head_2_joint"][:2]      # axis z


def _head_angles(head_dir, gain_yaw, gain_pitch, margin):
    yaw = math.atan2(head_dir[1], head_dir[0])
    pitch_down = -math.asin(max(-1.0, min(1.0, head_dir[2])))
    lo_p, hi_p = HEAD_PITCH_LIMITS
    lo_y, hi_y = HEAD_YAW_LIMITS
    return {
        "head_1_joint": _clip(gain_pitch * pitch_down, lo_p + margin, hi_p - margin),
        "head_2_joint": _clip(gain_yaw * yaw, lo_y + margin, hi_y - margin),
    }


# ================================================================== TORSO
TORSO_YAW_LIMITS = MEASURED["torso_1_joint"][:2]     # axis z
TORSO_PITCH_LIMITS = MEASURED["torso_2_joint"][:2]   # axis y, + assumed = fwd lean


def _torso_angles(forward_dir, gain_yaw, gain_pitch, margin):
    """torso_1 (yaw) / torso_2 (pitch) from the torso basis's OWN forward
    vector -- exactly the head's direct-angle-mapping pattern, applied to the
    basis itself instead of a landmark pair. At rest (facing the camera,
    upright) forward_dir == (1,0,0) and both angles are 0.

    yaw = atan2(f_y, f_x) is exact for any pitch (Rz(yaw) leaves the pitch
    contribution's ratio f_y/f_x untouched); pitch = -asin(f_z) mirrors the
    head's pitch formula, reusing the SAME sign convention (+ = leaning/
    looking forward-down) for both, on the same "one convention everywhere"
    principle THE MIRROR LAW already commits to.
    """
    yaw = math.atan2(forward_dir[1], forward_dir[0])
    pitch = -math.asin(max(-1.0, min(1.0, forward_dir[2])))
    lo_y, hi_y = TORSO_YAW_LIMITS
    lo_p, hi_p = TORSO_PITCH_LIMITS
    return {
        "torso_1_joint": _clip(gain_yaw * yaw, lo_y + margin, hi_y - margin),
        "torso_2_joint": _clip(gain_pitch * pitch, lo_p + margin, hi_p - margin),
    }


# ============================================================ ARM/LEG maps
ARM_LANDMARKS = {
    "left": (LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST),
    "right": (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST),
}
LEG_LANDMARKS = {
    "left": (LEFT_HIP, LEFT_KNEE, LEFT_ANKLE, LEFT_HEEL, LEFT_FOOT_INDEX),
    "right": (RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE, RIGHT_HEEL, RIGHT_FOOT_INDEX),
}
ARM_GATE = {"left": "arm_l", "right": "arm_r"}
LEG_GATE = {"left": "leg_l", "right": "leg_r"}


def _pts(landmarks, indices):
    """[(x,y,z), ...] for `indices` out of the JSON landmarks dict (string
    keys, as /body/observation serialises them)."""
    return [tuple(landmarks[str(i)]) for i in indices]


def retarget(
    obs,
    mirror=True,
    prev=None,
    margin=0.05,
    head_gain_yaw=0.8,
    head_gain_pitch=0.6,
    torso_gain_yaw=0.5,
    torso_gain_pitch=0.5,
    mirror_fn=None,
):
    """Joint targets for whatever of the body is currently visible.

    `obs` is EITHER a body_track.BodyObservation (used directly, e.g. from
    tests/retarget-bench) OR the plain dict body_track.observation_dict()
    produces (used by mirror_node, which only has the /body/observation JSON
    payload, not a live BodyObservation -- track_node and mirror_node are
    separate processes/nodes). Both shapes carry the same information; this
    function normalises to a BodyObservation-compatible view internally.

    Returns (targets, info). `targets` maps joint name -> radians and
    contains ONLY the joints backed by a currently-gated limb; everything
    else is the caller's (mirror_node's) to hold. `info` reports which
    limbs were used, the residual retargeting error, and what was skipped.

    `mirror_fn` overrides THE MIRROR LAW's vector transform (default
    mirror_vec) -- retarget-bench injects the two wrong mutations through
    this hook so every discriminating case runs through the real code path,
    not a reimplementation of it.
    """
    mirror_fn = mirror_fn or mirror_vec
    prev = prev or {}
    targets, info = {}, {"arms": [], "legs": [], "err_deg": {}, "degenerate": {},
                          "skipped": {}}

    obs = _as_observation(obs)
    if obs is None:
        info["skipped"]["body"] = "not tracked"
        return targets, info

    # c2-qp (M5): mirrored forearm direction per ROBOT arm side, in the
    # TORSO frame -- exactly the `b` this function already feeds
    # solve_arm's forearm-direction match, exposed here so a QP layer
    # (qp_retarget.WristQPSolver) can build a Cartesian ORIENTATION target
    # from it without recomputing the mirror law itself. None for a side
    # whose arm gate is currently off (held, not target-chased, same
    # contract as `targets` itself omitting that side's joints).
    info["forearm_dir"] = {"left": None, "right": None}

    gates = obs["gates"]
    landmarks = obs["landmarks"]
    basis = obs["basis"]

    # ------------------------------------------------------------------ arms
    for human_side in ("left", "right"):
        gate = ARM_GATE[human_side]
        if not gates.get(gate):
            info["skipped"][gate] = "limb not gated (landmarks not visible)"
            continue
        sh, el, wr = _pts(landmarks, ARM_LANDMARKS[human_side])
        a = _unit(_rot_into_torso(basis, _sub(el, sh)))
        b = _unit(_rot_into_torso(basis, _sub(wr, el)))
        if mirror:
            a, b = mirror_fn(a), mirror_fn(b)
            robot_side = "right" if human_side == "left" else "left"
        else:
            robot_side = human_side

        prefix = f"arm_{robot_side}"
        seed = [prev.get(f"{prefix}_{k}_joint") for k in range(1, 5)]
        seed = seed if all(v is not None for v in seed) else None

        q, qinfo = solve_arm(a, b, robot_side, q_seed=seed, margin=margin)
        for k, v in enumerate(q, start=1):
            targets[f"{prefix}_{k}_joint"] = v
        # Wrist parked -- MediaPipe Pose has no hand/wrist orientation.
        for k in (5, 6, 7):
            targets[f"{prefix}_{k}_joint"] = 0.0

        info["forearm_dir"][robot_side] = b
        info["arms"].append(f"{human_side}->{robot_side}")
        info["err_deg"][f"arm_{robot_side}"] = round(qinfo["err_deg"], 3)
        info["degenerate"][f"arm_{robot_side}"] = qinfo["degenerate"]

    # ------------------------------------------------------------------ legs
    for human_side in ("left", "right"):
        gate = LEG_GATE[human_side]
        if not gates.get(gate):
            info["skipped"][gate] = "limb not gated (landmarks not visible)"
            continue
        hip, knee, ankle, heel, toe = _pts(landmarks, LEG_LANDMARKS[human_side])
        # RAW body axes -- legs attach to the PINNED base_link, not the
        # torso, so no basis rotation here (see module docstring).
        a = _unit(_sub(knee, hip))
        b = _unit(_sub(ankle, knee))
        c = _unit(_sub(toe, heel))
        if mirror:
            a, b, c = mirror_fn(a), mirror_fn(b), mirror_fn(c)
            robot_side = "right" if human_side == "left" else "left"
        else:
            robot_side = human_side

        prefix = f"leg_{robot_side}"
        seed = [prev.get(f"{prefix}_{k}_joint") for k in range(1, 5)]
        seed = seed if all(v is not None for v in seed) else None

        q, qinfo = solve_leg_hip_knee(a, b, robot_side, q_seed=seed, margin=margin)
        for k, v in enumerate(q, start=1):
            targets[f"{prefix}_{k}_joint"] = v
        targets[f"{prefix}_5_joint"] = solve_ankle_pitch(c, q, robot_side, margin=margin)
        targets[f"{prefix}_6_joint"] = 0.0  # ankle roll parked -- see module docstring

        info["legs"].append(f"{human_side}->{robot_side}")
        info["err_deg"][f"leg_{robot_side}"] = round(qinfo["err_deg"], 3)
        info["degenerate"][f"leg_{robot_side}"] = qinfo["degenerate"]

    # ------------------------------------------------------------------ head
    if gates.get("head"):
        head_dir = _head_dir(landmarks, basis)
        if mirror:
            head_dir = mirror_fn(head_dir)
        targets.update(_head_angles(head_dir, head_gain_yaw, head_gain_pitch, margin))
    else:
        info["skipped"]["head"] = "limb not gated (landmarks not visible)"

    # ----------------------------------------------------------------- torso
    # Always available whenever ANYTHING is tracked -- obs.basis is a
    # prerequisite for tracked=True at all (torso_frame() needs the
    # shoulders), so there is no separate torso gate to check.
    forward = basis[0]
    if mirror:
        forward = mirror_fn(forward)
    targets.update(_torso_angles(forward, torso_gain_yaw, torso_gain_pitch, margin))

    return targets, info


def _rot_into_torso(basis, v):
    """v (raw body axes) -> torso frame, i.e. dot with each (f, l, u) row.
    Same operation as body_track.rot_transpose_apply, re-derived here so
    retarget.py never depends on which internal representation basis takes
    (list-of-lists from JSON vs tuple-of-tuples from a live BodyObservation
    both work)."""
    f, l, u = basis
    return (_dot(f, v), _dot(l, v), _dot(u, v))


def _head_dir(landmarks, basis):
    """NOSE - mid(EAR_L, EAR_R), rotated into the torso frame -- identical
    definition to body_track.BodyObservation.head_dir()."""
    from talos_mirror.body_track import LEFT_EAR, NOSE, RIGHT_EAR

    nose, ear_l, ear_r = _pts(landmarks, (NOSE, LEFT_EAR, RIGHT_EAR))
    ear_mid = ((ear_l[0] + ear_r[0]) * 0.5, (ear_l[1] + ear_r[1]) * 0.5,
               (ear_l[2] + ear_r[2]) * 0.5)
    return _unit(_rot_into_torso(basis, _sub(nose, ear_mid)))


def _as_observation(obs):
    """Normalise a body_track.BodyObservation OR a plain observation dict
    (JSON-decoded /body/observation payload) into
    {"gates": {...}, "landmarks": {str(idx): [x,y,z]}, "basis": 3x3}, or
    None if untracked. Landmark values stay whatever numeric type they came
    in as (tuple from a live object, list from JSON) -- every consumer above
    only ever indexes [0]/[1]/[2], never relies on the container type."""
    if isinstance(obs, BodyObservation):
        if not obs.tracked or obs.basis is None:
            return None
        return {
            "gates": obs.gates(),
            "landmarks": {str(i): v for i, v in obs.points_body.items()},
            "basis": obs.basis,
        }
    if isinstance(obs, dict):
        if not obs.get("tracked") or obs.get("basis") is None:
            return None
        return {
            "gates": obs.get("gates", {}),
            "landmarks": obs.get("landmarks", {}),
            "basis": obs["basis"],
        }
    return None


class MirrorPoseSource:
    """Adapts mirror_node's latest /body/observation payload into joint
    targets. mirror_node's subscription callback refreshes `node.observation`
    (the parsed JSON dict) on ITS OWN schedule (whatever rate track_node
    publishes at); this reads whatever is current without blocking, so the
    50 Hz control tick stays smooth between observation messages exactly as
    humanoid_mirror's MirrorPoseSource does between camera frames -- the
    input-rate/command-rate decoupling is the same pattern, just moved
    across a topic boundary instead of a thread boundary."""

    name = "body_observation"

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
            prev=self.node.q_cmd,
            margin=self.node.margin,
            head_gain_yaw=self.node.head_gain_yaw,
            head_gain_pitch=self.node.head_gain_pitch,
            torso_gain_yaw=self.node.torso_gain_yaw,
            torso_gain_pitch=self.node.torso_gain_pitch,
        )
        self.node.retarget_info = info
        return targets or None
