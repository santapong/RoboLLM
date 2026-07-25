"""Exact forward kinematics for one FFW arm, and the retargeting solve.

Pure math — no ROS, no numpy, no mediapipe — so it is unit-testable and cheap
(a solve costs microseconds). `tools/retarget_bench.py` verifies fk_arm()
against MoveIt's own /compute_fk, so these constants are not taken on trust.

=========================================================================
MEASURED GEOMETRY (from the expanded ffw_bg2_rev4_follower xacro)
=========================================================================
Every joint origin has rpy="0 0 0", so the whole arm is axis-aligned at zero.

  joint1  xyz=(0, +-0.1045, 0)        axis Y    shoulder pitch
  joint2  xyz=(0, +-0.123,  0)        axis X    shoulder roll
  joint3  xyz=(0, 0, -0.165)          axis Z    humeral yaw
  joint4  xyz=(+0.041004, 0, -0.135)  axis Y    ELBOW
  joint5  xyz=(-0.041004, 0, -0.1489) axis Z    wrist roll
  joint6  xyz=(0, 0, -0.1041)         axis Y    wrist pitch
  joint7  xyz=(0, 0, -0.0885)         axis X    wrist yaw

THE FINDING THAT MATTERS: the LEFT and RIGHT arms are GEOMETRICALLY
IDENTICAL. Same axes, same link offsets — only the base y offset (+-0.1045,
+-0.123) and the joint2 / joint7 LIMIT RANGES mirror:

    arm_l_joint2  [ 0.0,  3.14]        arm_r_joint2  [-3.14, 0.0]

So ONE formula serves both arms; only the clamp differs. The researched design
guessed the right arm needed `roll_r = asin(-a_y)` because its axis was
"almost certainly mirrored". It is not. That guess would have driven the right
arm's roll positive, straight into a limit it can never satisfy, clamping it
to ~0 — a right arm that simply never lifts, looking like a tracking problem.

Two more geometric facts that a naive "the arm points down at zero" model gets
wrong, both consequences of the +-0.041 m elbow offset:

  * shoulder(joint2) -> elbow(joint4) is (0.041, 0, -0.300): a 7.8 degree
    forward tilt, NOT straight down.
  * elbow -> wrist(joint7) is (-0.041, 0, -0.3415): 6.9 degrees the other way.
    So at q=0 the joint centres ZIGZAG by ~16.9 degrees at the elbow, even
    though the net shoulder->wrist is exactly (0, 0, -0.6415) — dead straight.

Ignoring those offsets biases every pose by ~8-17 degrees. solve_arm() seeds
from the closed form and then refines against the exact FK, so the residual is
numerically zero instead of "close enough".
"""

import math

# Link offsets shared by both arms (metres). Only BASE_Y differs by side.
BASE_Y = {"left": 0.1045, "right": -0.1045}
D2_Y = {"left": 0.123, "right": -0.123}
D3 = (0.0, 0.0, -0.165)
D4 = (0.041004, 0.0, -0.135)
D5 = (-0.041004, 0.0, -0.1489)
D6 = (0.0, 0.0, -0.1041)
D7 = (0.0, 0.0, -0.0885)

# Joint limits, per side (rad). Identical except joint2 and joint7.
LIMITS = {
    "left": [(-3.14, 3.14), (0.0, 3.14), (-3.14, 3.14), (-2.9361, 1.0786),
             (-3.14, 3.14), (-1.57, 1.57), (-1.8201, 1.5804)],
    "right": [(-3.14, 3.14), (-3.14, 0.0), (-3.14, 3.14), (-2.9361, 1.0786),
              (-3.14, 3.14), (-1.57, 1.57), (-1.5804, 1.8201)],
}


def _rx(v, t):
    c, s = math.cos(t), math.sin(t)
    return (v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c)


def _ry(v, t):
    c, s = math.cos(t), math.sin(t)
    return (v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c)


def _rz(v, t):
    c, s = math.cos(t), math.sin(t)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _unit(v):
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    return (0.0, 0.0, 0.0) if n < 1e-12 else (v[0] / n, v[1] / n, v[2] / n)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def ang_between(u, v):
    """Angle in radians between two (near-)unit vectors, numerically safe."""
    return math.acos(max(-1.0, min(1.0, _dot(u, v))))


def fk_arm(q, side="left"):
    """Joint-centre positions in arm_base_link, for q = [q1..q7] (short ok).

    Returns {"shoulder": p2, "humerus": p3, "elbow": p4, "wrist": p7, ...}.
    Rotations compose down the chain, so a vector fixed in link k maps to the
    base through R1*R2*...*Rk — implemented by rotating each accumulated offset
    by the joints ABOVE it, outermost last.
    """
    qq = list(q) + [0.0] * (7 - len(q))
    q1, q2, q3, q4, q5, q6, q7 = qq[:7]

    def up(v):
        """Map a vector from the post-joint2 frame out to arm_base_link."""
        return _ry(_rx(v, q2), q1)

    p1 = (0.0, BASE_Y[side], 0.0)                     # joint1 origin
    p2 = _add(p1, (0.0, D2_Y[side] , 0.0))            # joint2 origin = shoulder
    # Below joint2 everything is expressed in the post-joint2 frame, then
    # lifted out by up().
    v3 = D3                                            # joint3 origin
    v4 = _add(v3, _rz(D4, q3))                         # joint4 origin = elbow
    v5 = _add(v4, _rz(_ry(D5, q4), q3))
    v6 = _add(v5, _rz(_ry(_rz(D6, q5), q4), q3))
    v7 = _add(v6, _rz(_ry(_rz(_ry(D7, q6), q5), q4), q3))

    # Link frames: in URDF a child link's frame IS its joint's origin frame,
    # so link_k <-> joint_k origin. Named explicitly because getting this
    # correspondence wrong makes an FK cross-check compare link3->link4
    # against link1->link4 and report a 31 degree "disagreement" that is
    # entirely the test's fault.
    return {
        "j1": p1,          # arm_*_link1
        "shoulder": p2,    # arm_*_link2  (joint2 origin)
        "humerus": _add(p2, up(v3)),   # arm_*_link3
        "elbow": _add(p2, up(v4)),     # arm_*_link4
        "wrist5": _add(p2, up(v5)),    # arm_*_link5
        "wrist6": _add(p2, up(v6)),    # arm_*_link6
        "wrist": _add(p2, up(v7)),     # arm_*_link7
    }


def arm_dirs(q, side="left"):
    """(upper_arm_unit, forearm_unit) in arm_base_link — what we match to the
    human's shoulder->elbow and elbow->wrist directions."""
    p = fk_arm(q, side)
    return (
        _unit(_sub(p["elbow"], p["shoulder"])),
        _unit(_sub(p["wrist"], p["elbow"])),
    )


# Below this elbow angle the arm counts as straight and humeral yaw becomes
# unobservable from two direction vectors. 0.06 rad ~ 3.4 deg: tight on
# purpose. At 14 deg of bend the yaw is still perfectly observable, and
# holding it there costs 12 deg of round-trip error — measured.
STRAIGHT_EPS = 0.06

# Shoulder->elbow offset at q3 = 0, and elbow->wrist with the wrist parked.
_S0 = _add(D3, D4)                                  # (0.041, 0, -0.300)
_F0 = _unit(_add(_add(D5, D6), D7))                 # (-0.1195, 0, -0.9928)


def _wrap(t):
    return (t + math.pi) % (2.0 * math.pi) - math.pi


def _pick(cands, lo, hi, prev):
    """Choose the branch that is in range and nearest the previous solution.

    WRAP FIRST. `acos` branches come back as `psi +- acos(x)`, which is only
    correct modulo 2*pi: the true elbow solution for a straight arm arrives as
    +6.028 rad, whose wrapped value -0.255 is the one inside [-2.94, 1.08].
    Range-checking before wrapping silently discards it and returns the far
    branch instead — worth 15-80 degrees of round-trip error that still looks
    like a plausible pose. Every joint on this arm has a range inside +-pi, so
    wrapping is always safe here.

    Branch selection is the whole ballgame for teleop continuity: an
    in-range-but-far solution is what makes a robot snap across its workspace
    between two visually similar frames.
    """
    wrapped = [_wrap(c) for c in cands]
    inside = [c for c in wrapped if lo - 1e-9 <= c <= hi + 1e-9]
    pool = inside or wrapped
    return min(pool, key=lambda c: abs(_wrap(c - prev)))


def shoulder_branches(a, q3=0.0):
    """Both (q1, q2) solutions reproducing upper-arm direction `a` at this q3.

    Returned as a fixed-length list [branch0, branch1] (either may be None
    when `a` lies outside the shoulder's cone). Keeping the branches SEPARATE
    and in a stable order matters: solve_arm() sweeps q3 looking for sign
    changes in a residual, and if a helper silently switched branches midway
    the residual would jump, manufacturing fake roots and hiding real ones.
    """
    w = _unit(_add(D3, _rz(D4, q3)))
    r = math.hypot(w[1], w[2])
    if r < 1e-9:
        return [None, None]
    # THE SHOULDER CANNOT REACH EVERY DIRECTION. Rx leaves w_x alone and Ry
    # preserves y, so a_y is bounded by r = sqrt(1 - w_x^2) ~ 0.9908: the arm
    # can never point exactly sideways (0, +-1, 0). A textbook T-pose is
    # 7.78 degrees out of reach — a real property of the 41 mm elbow offset,
    # not a solver defect. Clamp to the nearest REACHABLE direction rather
    # than returning nothing, or an out-of-cone request silently collapses
    # the arm to its seed pose (measured: 87 degrees of error).
    delta = math.atan2(w[2], w[1])
    base = math.acos(max(-1.0, min(1.0, a[1] / r)))
    out = []
    for q2 in (_wrap(base - delta), _wrap(-base - delta)):
        v = _rx(w, q2)
        q1 = _wrap(math.atan2(-a[2], a[0]) - math.atan2(-v[2], v[0]))
        out.append((q1, q2))
    return out


def solve_shoulder(a, side, q3=0.0, prev=(0.0, 0.0)):
    """Closed-form (q1, q2) reproducing upper-arm direction `a`, given q3.

    With q3 fixed, the shoulder->elbow vector in the post-joint2 frame is
        w = unit(D3 + Rz(q3) * D4)
    and the arm direction is a = Ry(q1) * Rx(q2) * w.

    Rx leaves x alone and Ry preserves y, so the y component isolates q2:
        a_y = w_y cos q2 - w_z sin q2 = R cos(q2 + d),
        R = hypot(w_y, w_z),  d = atan2(w_z, w_y)
    Then with v = Rx(q2) w, the pair (x, -z) is a plane rotation by q1:
        (a_x, -a_z) = Rot(q1) * (v_x, -v_z)
    Exact — no iteration, no singularity except |a_y| > R, which means the
    target is simply outside this shoulder's cone and gets clamped.
    """
    w = _unit(_add(D3, _rz(D4, q3)))
    r = math.hypot(w[1], w[2])
    if r < 1e-9:
        return prev[0], prev[1]
    delta = math.atan2(w[2], w[1])
    c = max(-1.0, min(1.0, a[1] / r))
    lo, hi = LIMITS[side][1]
    q2 = _pick([math.acos(c) - delta, -math.acos(c) - delta], lo, hi, prev[1])
    v = _rx(w, q2)
    q1 = _wrap(math.atan2(-a[2], a[0]) - math.atan2(-v[2], v[0]))
    return q1, q2


def _elbow_for(b_local):
    """q4 that swings the forearm onto b_local's x-z projection.

    Ry(q4) F0 always has y = 0, so the reachable forearm set (for a given
    q1,q2,q3) is one great circle. Measuring both vectors as azimuth from -z,
    Ry(q4) shifts the azimuth by exactly -q4, so the answer is a subtraction.
    Whatever y-component b_local retains is the out-of-plane residual, and
    that is precisely what the q3 search below drives to zero.
    """
    a0 = math.atan2(_F0[0], -_F0[2])
    at = math.atan2(b_local[0], -b_local[2])
    return _wrap(a0 - at)


def solve_forearm(b, q1, q2, side, prev=(0.0, 0.0)):
    """Closed-form (q3, q4) reproducing forearm direction `b`, given q1, q2.

    Rotate the target back into the post-joint2 frame, where the forearm is
        b_local = Rz(q3) * Ry(q4) * F0.
    Ry keeps y at zero, so p = Ry(q4) F0 = (px, 0, pz) and
        Rz(q3) p = (px cos q3, px sin q3, pz).
    The z component therefore fixes q4 outright:
        pz = b_local_z = cos(q4 - psi),  psi = atan2(-F0_x, F0_z)
    and the x-y part gives q3 = atan2(b_y, b_x) (flipped when px < 0).
    Exact, and it explains why humeral yaw is nearly free when the arm is
    straight: px -> 0 there, so q3's axis and the forearm become collinear.
    """
    b_local = _rx(_ry(b, -q1), -q2)
    psi = math.atan2(-_F0[0], _F0[2])
    pz = max(-1.0, min(1.0, b_local[2]))
    lo4, hi4 = LIMITS[side][3]
    q4 = _pick([psi + math.acos(pz), psi - math.acos(pz)], lo4, hi4, prev[1])
    p = _ry(_F0, q4)
    px = p[0]
    lo3, hi3 = LIMITS[side][2]
    if abs(px) < 1e-6:
        q3 = prev[0]  # degenerate: straight arm, humeral yaw is unobservable
    else:
        base = math.atan2(b_local[1], b_local[0])
        q3 = _pick([_wrap(base if px > 0 else base + math.pi)], lo3, hi3, prev[0])
    return q3, q4


def clamp_to_limits(q, side, margin=0.05):
    out = []
    for i, v in enumerate(q):
        lo, hi = LIMITS[side][i]
        lo, hi = lo + margin, hi - margin
        out.append(min(max(v, lo), hi) if lo <= hi else 0.5 * (lo + hi))
    return out


def solve_arm(a, b, side="left", q_seed=None, margin=0.05):
    """Joint angles q1..q4 whose FK reproduces upper-arm `a` and forearm `b`.

    Direct joint-angle retargeting, NOT Cartesian IK: it matches DIRECTIONS,
    so it is invariant to the human's limb lengths and can never be handed an
    unreachable point.

    Structure of the problem, which took two wrong solvers to see clearly:
    q3 (humeral yaw) sits BELOW the shoulder gimbal, so it swings the 41 mm
    elbow offset around a cone of half-angle 7.78 deg. Changing q3 therefore
    moves the upper-arm direction by up to 15.6 degrees — the coupling is NOT
    weak, and alternating closed-form blocks settles onto spurious fixed
    points worth exactly that much error. (An earlier damped-gradient version
    was worse still: 20 deg error and 3 rad discontinuities.)

    So the solve is a 1-D search over q3 alone:
      * for ANY q3, shoulder_branches() reproduces `a` EXACTLY, so the upper
        arm is never approximated — it is the visually dominant segment;
      * _elbow_for() then places the forearm on its reachable great circle,
        leaving one scalar residual: the out-of-plane component b_local_y;
      * that residual is continuous in q3 WITHIN a branch, so each branch is
        swept separately and its sign changes bisected. Letting a helper pick
        branches mid-sweep makes the residual jump, which manufactures fake
        roots and hides real ones.

    Result: upper arm exact by construction, forearm exact wherever the arm
    can physically reach it, and no convergence to babysit.

    Returns (q1..q4, info) with info["err_deg"], ["clamped"], ["degenerate"].
    """
    prev = list(q_seed[:4]) if q_seed is not None and len(q_seed) >= 4 \
        else [0.0, 0.0, 0.0, 0.0]

    # STRAIGHT-ARM DEGENERACY. With the forearm nearly collinear with the
    # upper arm, humeral yaw spins the limb about its own axis and is
    # UNOBSERVABLE from two direction vectors: the residual is ~0 for every
    # q3, root-finding returns arbitrary answers, and q3 wanders (measured:
    # 0.79 rad steps between adjacent frames, every one at 0.0000 deg error).
    # Physically that is the humerus spinning on a straight arm — correct by
    # the metric, wrong to look at. Hold the previous yaw instead.
    degenerate = ang_between(a, b) < STRAIGHT_EPS

    def evaluate(q3, branch):
        """(residual, q) for a humeral yaw on ONE shoulder branch."""
        sol = shoulder_branches(a, q3)[branch]
        if sol is None:
            return None, None
        q1, q2 = sol
        b_local = _rz(_rx(_ry(b, -q1), -q2), -q3)
        return b_local[1], [q1, q2, _wrap(q3), _elbow_for(b_local)]

    # ONE candidate pool, whatever the regime. Roots of the out-of-plane
    # residual give exact solutions; the grid points themselves are kept too,
    # because when the arm is straight the residual is ~0 everywhere and there
    # are no clean sign changes to bisect. The shared ranking below then picks
    # on accuracy first and proximity second, so the degenerate case needs no
    # special-cased answer — only a candidate set that covers q3.
    lo3, hi3 = LIMITS[side][2]
    steps = 24
    grid = [lo3 + (hi3 - lo3) * i / steps for i in range(steps + 1)]
    if degenerate:
        # Humeral yaw is unobservable here, so bias the pool toward where the
        # arm already is; the previous value is always a candidate.
        grid = grid + [prev[2]]

    solutions = []
    for branch in (0, 1):
        samples = [(g,) + evaluate(g, branch) for g in grid]
        for _g, _r, qv in samples:
            if qv is not None:
                solutions.append(qv)
        if degenerate:
            continue                      # no usable sign changes to bisect
        for (g0, r0, _q0), (g1, r1, _q1) in zip(samples, samples[1:]):
            if r0 is None or r1 is None:
                continue
            if r0 == 0.0 or r0 * r1 < 0.0:
                a3, b3, ra = g0, g1, r0
                for _ in range(24):              # 24 halvings ~ 1e-7 rad
                    m = 0.5 * (a3 + b3)
                    rm, _q = evaluate(m, branch)
                    if rm is None:
                        break
                    if ra * rm <= 0.0:
                        b3 = m
                    else:
                        a3, ra = m, rm
                _r, qv = evaluate(0.5 * (a3 + b3), branch)
                if qv is not None:
                    solutions.append(qv)

    if not solutions:
        # `a` is outside the shoulder cone for every q3 on both branches.
        # Hold rather than fail; the caller sees it in info["err_deg"].
        solutions = [list(prev[:4])]

    def rank(qv):
        qc = clamp_to_limits(qv, side, margin)
        ua, fa = arm_dirs(qc, side)
        err = max(ang_between(ua, a), ang_between(fa, b))
        drift = sum(abs(_wrap(qc[k] - prev[k])) for k in range(4))
        return err, drift

    ranked = [(rank(qv), qv) for qv in solutions]
    best_err = min(r[0][0] for r in ranked)
    # ACCURACY FIRST, continuity only as a tie-break among equally good
    # solutions. Folding drift into one weighted score (an earlier attempt)
    # lets a 3-degree-worse pose win on proximity, because near a solution the
    # error term is tiny and the drift term is not.
    near = [r for r in ranked if r[0][0] <= best_err + 1e-3]
    q = clamp_to_limits(min(near, key=lambda r: r[0][1])[1], side, margin)

    ua, fa = arm_dirs(q, side)
    err = math.degrees(max(ang_between(ua, a), ang_between(fa, b)))
    clamped = any(
        abs(q[i] - (LIMITS[side][i][0] + margin)) < 1e-9
        or abs(q[i] - (LIMITS[side][i][1] - margin)) < 1e-9
        for i in range(4))
    return q, {"err_deg": err, "clamped": clamped, "degenerate": degenerate}
