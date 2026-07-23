"""ik_client.py — oriented IK / FK client for the Kinova Gen3 lite (U3).

Wraps move_group's /compute_ik and /compute_fk services for the hand-guided
pick-and-place demo.  Verified names (U2, mock-hardware bring-up):

  * MoveIt group  : "arm"  (KDL solver, tip = tool_frame; /compute_ik answers
                    with an explicit ik_link_name="tool_frame")
  * Arm joints    : joint_1 .. joint_6 (exact /joint_states names; extra
                    finger joints are tolerated and ignored — subset
                    reconcile, gap 8)
  * Base frame    : base_link
  * FK            : /compute_fk IS served by move_group (gap 12) — used for
                    reseed/err; no TF fallback is needed on this bring-up.

PALM-FRAME ORIENTATION SPEC (binding).  Inputs come from the SAME
GestureRecognizer result index i as the accepted LEFT hand (parallel arrays —
gestures / handedness / hand_landmarks / hand_world_landmarks are all indexed
by i and must never be indexed independently).  From hand_world_landmarks[i]
(metric, hand-centered, axes = flipped-camera frame: x right, y down, z away
from camera) take the palm-rigid points only (never fingertips — they move
with grip state):

    P0 = WRIST(0)   P5 = INDEX_MCP(5)   P9 = MIDDLE_MCP(9)   P17 = PINKY_MCP(17)

Per vision frame:
  (1) v1 = P9 - P0  (palm forward),  v2 = P5 - P17  (across palm, pinky->index)
  (2) validity gate: |v1| >= 0.05 m, |v2| >= 0.04 m and conditioning
      c = |v1 x v2| / (|v1|·|v2|) >= 0.35 (~20 deg); else the frame is
      orientation-INVALID (feeds the degradation counters; position tracking
      is unaffected).
  (3) palm-facing normal n = v2 x v1, n_hat = n/|n|.  SIGN (binding for this
      flipped/mirror pipeline where the physical left hand is labeled
      'Left'): left palm facing the camera in the flipped view has INDEX_MCP
      image-right of PINKY_MCP, so v2 ~ (+1,0,0), v1 ~ (0,-1,0) and
      v2 x v1 = (0,0,-1) = toward camera = palm-facing.  The build must
      verify this once visually (preview-overlay the projected normal;
      palm-toward-camera must give n_hat . z_cam < 0); if the world-landmark
      axis convention differs, flip the CROSS-PRODUCT ORDER, not downstream
      signs.
  (4) camera->base rotation R_cb is FIXED and consistent with the validated
      position map (flipped image: u/right -> base +Y, up -> base +Z,
      toward-camera -> base +X), i.e. for a camera-frame vector (xc, yc, zc):

          base = (-zc, +xc, -yc)

Gripper frame built from the palm: tool z (approach axis, out of the gripper
between the fingers) = R_cb·n_hat, tool x = MINUS R_cb·v1_hat (exactly
orthogonal — n is perpendicular to v1 by construction), tool y = z x x.
The pi flip about the approach axis is a deliberate branch choice, NOT a
sign error: a parallel-jaw grasp is invariant under a pi rotation about the
approach axis, and the gen3-lite wrist joints only span +-2.6 rad (< pi), so
the un-flipped branch (tool x = +X_base for a flat palm-down hand) needs a
joint_6 wrap that is OUT OF LIMITS across most of the workspace (verified
empirically: fixed-down yaw-0 fails almost everywhere, yaw-180 solves).
A flat palm-down hand with fingers toward the camera therefore reproduces
the fixed top-down grasp (tool z = -Z_base, tool x = -X_base) exactly.

DEGRADATION CHAIN (gap 1): oriented -> fixed-downward -> report failure so
the caller holds.  ``solve()`` walks the chain and reports which stage fired;
on total failure the caller must hold the last q_cmd, warn (throttled) and
freeze target advance after N consecutive failures (gap 6) — the client keeps
the ``consecutive_failures`` counter for that.

WORKSPACE BOX + SPAWN (gaps 10/16): tuned constants below; validated
non-circularly by gen3_pick_place.workspace_sweep (IK grid + 200-step
oriented sweep + explicit grasp-pose solves + >= 3 cm geometric margins).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState

# ---------------------------------------------------------------------------
# Verified names (U2)

GROUP_NAME = "arm"
IK_LINK = "tool_frame"
BASE_FRAME = "base_link"
ARM_JOINTS: Tuple[str, ...] = (
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
)
GRIPPER_JOINT = "right_finger_bottom_joint"

# A mid-workspace elbow-bent seed used before the first /joint_states arrives
# (all-zero is a singular stretched-up pose for the gen3-lite).
DEFAULT_SEED: Tuple[float, ...] = (0.0, 0.4, 1.2, 0.0, -0.6, 0.0)

# ---------------------------------------------------------------------------
# Tuned workspace box for the HAND-GUIDED target, meters in base_link.
# Tuned/validated by gen3_pick_place.workspace_sweep (gap 16): 5x5x5 IK grid
# (fixed-down + tilted) must fully solve, and the 200-step oriented sweep must
# reach >= 99 %.  Gen3-lite reach is 0.76 m; the box keeps a top-down wrist
# (~13 cm above tool_frame) comfortably inside it.
#
# RECONCILED with gen3_pick_place.scene_box (WS_BOX / SPAWN_XYZ / D_ATTACH —
# scene_box's header says "U3 must reconcile WS_BOX when it lands"): the
# numbers below are the single tuned set; workspace_sweep cross-checks the
# two modules stay equal.
#
# TUNED numbers (U3, empirical): a 5x5x5 fixed-down IK grid over this box
# solves 125/125 with the warm chain; pushing to x<=0.42 / z<=0.32 starts
# failing at the far-high edge (x=0.42, z=0.32), x<=0.24 is marginal near
# the base (tightly folded top-down configs at small |y|), and the pre-tune
# box (x<=0.45, z<=0.40) fails 25/125 — the z=0.40 plane and far corners
# are genuinely out of the gen3-lite's top-down envelope.
WORKSPACE_BOX: Dict[str, Tuple[float, float]] = {
    "x": (0.26, 0.40),
    "y": (-0.16, 0.16),
    "z": (0.10, 0.30),
}
HOME_TARGET: Tuple[float, float, float] = (0.32, 0.0, 0.20)

# 4 cm cube spawn (gap 10): center MUST lie inside WORKSPACE_BOX at a
# reachable grasp height with >= 3 cm margin on every face (gap 16, checked
# non-circularly by the sweep tool), tied to the attach gate distance
# (spawn margin to every box face >= D_ATTACH so the hand target can come
# within the attach gate without leaving the box).
BOX_SIZE = 0.04                    # m, cube edge
BOX_SPAWN: Tuple[float, float, float] = (0.34, 0.0, 0.16)   # cube CENTER
GRASP_HEIGHT = BOX_SPAWN[2]        # tool_frame z at grasp = cube center
D_ATTACH = 0.06                    # m, proximity gate radius (gap 5): FIST
                                   # attaches only when |tool - box| < D_ATTACH

# Fixed top-down fallback orientation: tool z = -Z_base, tool x = -X_base
# == rotation of pi about base Y -> quaternion (x, y, z, w).  The -X branch
# (not +X) keeps joint_6 inside its +-2.6 rad (< pi) limit — see the
# palm-frame spec note above; verified empirically over the box.
DOWN_QUAT: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 0.0)

# URDF joint limits (processed gen3_lite_gen3_lite_2f.xacro, U2 container):
# all wrists span < pi — the reason for the -X tool-x branch above and for
# the recovery-seed retry ladder (KDL on this non-redundant 6-DOF chain is
# effectively single-shot from its seed; measured per-random-seed
# convergence ~15-30 %).
JOINT_LIMITS: Tuple[Tuple[float, float], ...] = (
    (-2.68, 2.68), (-2.61, 2.61), (-2.61, 2.61),
    (-2.60, 2.60), (-2.53, 2.53), (-2.60, 2.60),
)

# Orientation-validity gates (binding spec numbers)
_V1_MIN = 0.05
_V2_MIN = 0.04
_COND_MIN = 0.35

# Landmark indices (MediaPipe 21-point hand)
_WRIST, _INDEX_MCP, _MIDDLE_MCP, _PINKY_MCP = 0, 5, 9, 17


# ---------------------------------------------------------------------------
# Small vector helpers (no numpy — keep the mpvenv/system-python split happy)

def _xyz(p: Any) -> Tuple[float, float, float]:
    if hasattr(p, "x"):
        return (float(p.x), float(p.y), float(p.z))
    return (float(p[0]), float(p[1]), float(p[2]))


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return (a[0] / n, a[1] / n, a[2] / n)


def cam_to_base(v: Sequence[float]) -> Tuple[float, float, float]:
    """Fixed R_cb: flipped-camera vector (x right, y down, z away) -> base.

    Consistent with the validated position map: image-right -> +Y,
    image-up -> +Z, toward-camera -> +X.
    """
    xc, yc, zc = v
    return (-zc, xc, -yc)


def _mat_to_quat(x, y, z) -> Tuple[float, float, float, float]:
    """Rotation matrix with COLUMNS x,y,z -> quaternion (qx, qy, qz, qw)."""
    m00, m01, m02 = x[0], y[0], z[0]
    m10, m11, m12 = x[1], y[1], z[1]
    m20, m21, m22 = x[2], y[2], z[2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


# ---------------------------------------------------------------------------
# Palm-frame orientation (spec above)

@dataclass
class PalmOrientation:
    valid: bool
    quat: Optional[Tuple[float, float, float, float]] = None  # (x, y, z, w)
    reason: str = ""
    # diagnostics (camera frame) for the one-time visual sign verification
    n_cam: Optional[Tuple[float, float, float]] = None
    cond: float = 0.0


def palm_orientation(world_landmarks: Sequence[Any]) -> PalmOrientation:
    """Build the gripper orientation from hand_world_landmarks[i].

    ``world_landmarks`` MUST be the world-landmark list taken at the SAME
    result index i as the accepted LEFT hand (parallel-arrays invariant).
    Returns valid=False (orientation-INVALID frame) on any gate failure —
    position tracking is unaffected; the degradation chain handles it.
    """
    if world_landmarks is None or len(world_landmarks) < 21:
        return PalmOrientation(False, reason="no/short landmark list")
    p0 = _xyz(world_landmarks[_WRIST])
    p5 = _xyz(world_landmarks[_INDEX_MCP])
    p9 = _xyz(world_landmarks[_MIDDLE_MCP])
    p17 = _xyz(world_landmarks[_PINKY_MCP])

    v1 = _sub(p9, p0)      # palm forward
    v2 = _sub(p5, p17)     # across palm, pinky -> index
    n1, n2 = _norm(v1), _norm(v2)
    if n1 < _V1_MIN:
        return PalmOrientation(False, reason=f"|v1|={n1:.3f} < {_V1_MIN}")
    if n2 < _V2_MIN:
        return PalmOrientation(False, reason=f"|v2|={n2:.3f} < {_V2_MIN}")
    n = _cross(v2, v1)     # palm-facing normal (binding cross order)
    cond = _norm(n) / (n1 * n2)
    if cond < _COND_MIN:
        return PalmOrientation(False, cond=cond,
                               reason=f"conditioning {cond:.2f} < {_COND_MIN}")
    n_hat = _unit(n)
    f_hat = _unit(v1)

    z_t = _unit(cam_to_base(n_hat))         # approach axis
    fb = cam_to_base(f_hat)                 # palm-forward in base; exactly ⟂ z_t
    x_t = _unit((-fb[0], -fb[1], -fb[2]))   # pi grasp-branch flip (see spec
                                            # note: keeps joint_6 in limits)
    # numeric re-orthogonalization (landmark noise): project + renormalize
    d = _dot(x_t, z_t)
    x_t = _unit((x_t[0] - d * z_t[0], x_t[1] - d * z_t[1], x_t[2] - d * z_t[2]))
    y_t = _cross(z_t, x_t)
    return PalmOrientation(True, quat=_mat_to_quat(x_t, y_t, z_t),
                           n_cam=n_hat, cond=cond)


# ---------------------------------------------------------------------------
# Results

MODE_ORIENTED = "oriented"
MODE_FIXED_DOWN = "fixed_down"
MODE_FAILED = "failed"          # caller HOLDS last q_cmd (gap 6)


@dataclass
class IKResult:
    ok: bool
    joints: Optional[Tuple[float, ...]] = None   # 6 arm joints, ARM_JOINTS order
    error_code: int = 0
    rt_ms: float = 0.0           # service round-trip, wall clock
    max_jump: float = 0.0        # max |q - seed| over the 6 joints, rad


@dataclass
class OrientedIKResult:
    mode: str                    # MODE_ORIENTED | MODE_FIXED_DOWN | MODE_FAILED
    joints: Optional[Tuple[float, ...]] = None
    orientation_valid: bool = False
    quat: Optional[Tuple[float, float, float, float]] = None  # quat actually used
    rt_ms_total: float = 0.0
    attempts: List[Tuple[str, IKResult]] = field(default_factory=list)
    invalid_reason: str = ""
    # True when the winning solution came from a RECOVERY seed (not the warm
    # seed): the solution may be far from the last command — the caller MUST
    # treat it like hand_follow's branch-guard case (pause target slew,
    # clamp per-tick joint deltas while the joints close the gap).
    recovered: bool = False
    winning_attempt: int = -1


class IKClient:
    """Warm-seeded /compute_ik + /compute_fk client for the gen3-lite 'arm'.

    NOT thread-safe; call from one control thread.  The owning node must not
    be spun concurrently by another executor while ``solve``/``fk`` block
    (they use rclpy.spin_until_future_complete on the owning node), matching
    the hand_follow.py manual-spin control-loop pattern.
    """

    def __init__(self, node, service_timeout: float = 0.15,
                 ik_timeout: float = 0.012,
                 oriented_attempts: int = 10, fixed_attempts: int = 30,
                 oriented_budget_s: float = 0.10,
                 fixed_budget_s: float = 0.30) -> None:
        self.node = node
        self.service_timeout = float(service_timeout)   # future wait, s
        self.ik_timeout = float(ik_timeout)             # solver budget, s
        # RECOVERY-SEED RETRY LADDER: MoveIt's KDL plugin on this
        # non-redundant 6-DOF chain converges (or not) deterministically from
        # its seed — no useful internal restarts (measured: per-random-seed
        # success 15-30 % on reachable poses).  Warm tracking almost never
        # needs retries (sub-ms); after a failure the chain retries each
        # stage from recent distinct successes, DEFAULT_SEED, then bounded
        # random seeds within JOINT_LIMITS, capped per stage by attempt
        # count and wall clock.  ik_timeout is deliberately small (12 ms):
        # a converging KDL solve returns in 1-7 ms, a diverging one only
        # stops at the timeout — short attempts buy more retries per budget.
        self.oriented_attempts = int(oriented_attempts)
        self.fixed_attempts = int(fixed_attempts)
        self.oriented_budget_s = float(oriented_budget_s)
        self.fixed_budget_s = float(fixed_budget_s)
        self._ik = node.create_client(GetPositionIK, "/compute_ik")
        self._fk = node.create_client(GetPositionFK, "/compute_fk")
        self.seed: List[float] = list(DEFAULT_SEED)
        self._last_good: List[Tuple[float, ...]] = []   # diverse successes
        self._rng = __import__("random").Random(0xC0FFEE)
        self.consecutive_failures = 0                   # gap 6 counter
        self.stats = {MODE_ORIENTED: 0, MODE_FIXED_DOWN: 0, MODE_FAILED: 0,
                      "orientation_invalid": 0, "recovery_retries": 0}

    # -- plumbing -----------------------------------------------------------

    def wait_for_services(self, timeout: float = 20.0) -> bool:
        t0 = time.monotonic()
        for cli, name in ((self._ik, "/compute_ik"), (self._fk, "/compute_fk")):
            remain = timeout - (time.monotonic() - t0)
            if remain <= 0 or not cli.wait_for_service(timeout_sec=remain):
                self.node.get_logger().error(f"{name} not available")
                return False
        return True

    def _call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut,
                                         timeout_sec=self.service_timeout)
        if not fut.done():
            fut.cancel()
            return None
        return fut.result()

    def seed_from_joint_state(self, msg: JointState) -> bool:
        """Subset-reconcile seed from /joint_states (gap 8): the 6 arm joints
        must all be present; extra finger/mimic joints are tolerated."""
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in ARM_JOINTS):
            return False
        self.seed = [float(msg.position[idx[j]]) for j in ARM_JOINTS]
        return True

    # -- IK -----------------------------------------------------------------

    def solve_pose(self, xyz: Sequence[float],
                   quat: Sequence[float],
                   avoid_collisions: bool = False,
                   seed: Optional[Sequence[float]] = None) -> IKResult:
        """One /compute_ik round-trip for tool_frame at (xyz, quat) seeded
        with ``seed`` (default: the warm seed).  ``max_jump`` is ALWAYS
        measured against the warm seed (self.seed = last accepted command),
        even when a recovery seed was used for the solve.  Does NOT walk the
        degradation chain and does NOT update the seed — ``solve`` owns
        both."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP_NAME
        r.ik_link_name = IK_LINK
        r.avoid_collisions = bool(avoid_collisions)
        r.robot_state.is_diff = False
        r.robot_state.joint_state.name = list(ARM_JOINTS)
        r.robot_state.joint_state.position = list(seed if seed is not None
                                                  else self.seed)
        sec = int(self.ik_timeout)
        r.timeout = DurationMsg(sec=sec,
                                nanosec=int((self.ik_timeout - sec) * 1e9))
        ps = PoseStamped()
        ps.header.frame_id = BASE_FRAME
        ps.pose.position.x = float(xyz[0])
        ps.pose.position.y = float(xyz[1])
        ps.pose.position.z = float(xyz[2])
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])
        r.pose_stamped = ps

        t0 = time.perf_counter()
        res = self._call(self._ik, req)
        rt = (time.perf_counter() - t0) * 1e3
        if res is None:
            return IKResult(False, error_code=0, rt_ms=rt)
        if res.error_code.val != 1:  # SUCCESS
            return IKResult(False, error_code=res.error_code.val, rt_ms=rt)
        idx = {n: i for i, n in enumerate(res.solution.joint_state.name)}
        if not all(j in idx for j in ARM_JOINTS):
            return IKResult(False, error_code=-99, rt_ms=rt)
        q = tuple(float(res.solution.joint_state.position[idx[j]])
                  for j in ARM_JOINTS)
        jump = max(abs(q[i] - self.seed[i]) for i in range(6))
        return IKResult(True, joints=q, error_code=1, rt_ms=rt, max_jump=jump)

    def solve(self, xyz: Sequence[float],
              world_landmarks: Optional[Sequence[Any]] = None,
              quat: Optional[Sequence[float]] = None,
              max_jump_rad: Optional[float] = None) -> OrientedIKResult:
        """Degradation chain (gap 1): oriented -> fixed-downward -> failed.

        Pass EITHER ``world_landmarks`` (hand_world_landmarks[i] of the
        accepted LEFT hand — same index i as its gesture, parallel arrays) or
        an explicit ``quat``.  An orientation-INVALID frame skips straight to
        the fixed-downward stage.  ``max_jump_rad`` (optional) is the
        branch-flip guard: it rejects a WARM-SEED solution that jumps more
        than that from the last accepted command (an elbow flip on a smooth
        path).  RECOVERY-seed solutions are exempt — after a failure there
        is legitimately no continuity — but come back flagged
        ``recovered=True`` and the caller must slew (freeze target advance
        while per-tick joint clamps close the gap, hand_follow pattern).

        On success the seed is updated (warm start).  On MODE_FAILED the
        caller must hold the last q_cmd and, after N consecutive failures,
        freeze target advance (gap 6) — see ``consecutive_failures``.
        """
        out = OrientedIKResult(mode=MODE_FAILED)
        oriented_quat: Optional[Tuple[float, float, float, float]] = None
        if quat is not None:
            oriented_quat = tuple(float(v) for v in quat)  # type: ignore
            out.orientation_valid = True
        elif world_landmarks is not None:
            po = palm_orientation(world_landmarks)
            out.orientation_valid = po.valid
            if po.valid:
                oriented_quat = po.quat
            else:
                out.invalid_reason = po.reason
                self.stats["orientation_invalid"] += 1

        chain: List[Tuple[str, Tuple[float, float, float, float], int, float]] = []
        if oriented_quat is not None:
            chain.append((MODE_ORIENTED, oriented_quat,
                          self.oriented_attempts, self.oriented_budget_s))
        chain.append((MODE_FIXED_DOWN, DOWN_QUAT,
                      self.fixed_attempts, self.fixed_budget_s))

        for mode, q_quat, max_att, budget_s in chain:
            t0 = time.monotonic()
            for att, s in enumerate(self._seed_ladder(max_att)):
                if att > 0 and time.monotonic() - t0 > budget_s:
                    break
                res = self.solve_pose(xyz, q_quat, seed=s)
                out.attempts.append((mode, res))
                out.rt_ms_total += res.rt_ms
                if att > 0:
                    self.stats["recovery_retries"] += 1
                # branch-flip guard on the warm attempt only (att == 0);
                # recovery solutions are accepted but flagged (see docstring)
                if res.ok and (att > 0 or max_jump_rad is None
                               or res.max_jump <= max_jump_rad):
                    out.mode = mode
                    out.joints = res.joints
                    out.quat = q_quat
                    out.recovered = att > 0
                    out.winning_attempt = att
                    self.seed = list(res.joints)      # warm start
                    self._remember_good(res.joints)
                    self.consecutive_failures = 0
                    self.stats[mode] += 1
                    return out
        self.consecutive_failures += 1
        self.stats[MODE_FAILED] += 1
        return out

    def _seed_ladder(self, max_attempts: int):
        """Warm seed first, then recent distinct successes, DEFAULT_SEED,
        then bounded-random seeds within JOINT_LIMITS."""
        yielded = 0
        for s in ([list(self.seed)] + [list(g) for g in self._last_good]
                  + [list(DEFAULT_SEED)]):
            if yielded >= max_attempts:
                return
            yielded += 1
            yield s
        while yielded < max_attempts:
            yielded += 1
            yield [self._rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS]

    def _remember_good(self, q: Tuple[float, ...], keep: int = 8) -> None:
        for g in self._last_good:
            if max(abs(a - b) for a, b in zip(q, g)) < 0.3:
                return                                # not distinct enough
        self._last_good.insert(0, tuple(q))
        del self._last_good[keep:]

    # -- FK (gap 12: /compute_fk is served — reseed / tracking error) -------

    def fk(self, joints: Sequence[float],
           links: Sequence[str] = (IK_LINK,)) -> Optional[Dict[str, Tuple[
               Tuple[float, float, float], Tuple[float, float, float, float]]]]:
        """FK via /compute_fk: link -> ((x,y,z), (qx,qy,qz,qw)) in base_link."""
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = list(links)
        req.robot_state.joint_state.name = list(ARM_JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in joints]
        res = self._call(self._fk, req)
        if res is None or res.error_code.val != 1:
            return None
        out = {}
        for name, ps in zip(res.fk_link_names, res.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            out[name] = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        return out


# ---------------------------------------------------------------------------
# Workspace-box helpers (shared with workspace_sweep + later units)

def in_box(xyz: Sequence[float], margin: float = 0.0) -> bool:
    (x0, x1), (y0, y1), (z0, z1) = (WORKSPACE_BOX["x"], WORKSPACE_BOX["y"],
                                    WORKSPACE_BOX["z"])
    x, y, z = xyz
    return (x0 + margin <= x <= x1 - margin
            and y0 + margin <= y <= y1 - margin
            and z0 + margin <= z <= z1 - margin)


def box_margins(xyz: Sequence[float]) -> Tuple[float, ...]:
    """Distance to each of the 6 box faces (all must be >= 0.03 for the box
    spawn / grasp pose per gap 16)."""
    (x0, x1), (y0, y1), (z0, z1) = (WORKSPACE_BOX["x"], WORKSPACE_BOX["y"],
                                    WORKSPACE_BOX["z"])
    x, y, z = xyz
    return (x - x0, x1 - x, y - y0, y1 - y, z - z0, z1 - z)


def clamp_to_box(xyz: Sequence[float]) -> Tuple[float, float, float]:
    (x0, x1), (y0, y1), (z0, z1) = (WORKSPACE_BOX["x"], WORKSPACE_BOX["y"],
                                    WORKSPACE_BOX["z"])
    return (min(max(xyz[0], x0), x1),
            min(max(xyz[1], y0), y1),
            min(max(xyz[2], z0), z1))
