#!/opt/mpvenv/bin/python
"""wall_weld.py — closed-fist-triggered AUTONOMOUS serpentine wall welding.

Runs under /opt/mpvenv/bin/python inside ros2-arm:jazzy (mediapipe venv,
--system-site-packages so rclpy resolves). Ported from the proven
hand_follow.py skeleton (ThreadedCapture / preflight / preview / synthetic /
20 Hz JointTrajectory streaming via arm_ik.solve_track).

WHAT IT DOES
  * Spawns a WALL (box CollisionObject `weld_wall`, default 0.35 w x 0.25 h x
    0.02 t) into the MoveIt planning scene via /apply_planning_scene.
  * The user's CLOSED FIST (held, N consecutive frames) triggers an
    AUTONOMOUS weld of the whole wall: a serpentine raster over the wall
    face (horizontal passes, row_step apart, boundary margin, torch tip at
    `standoff` from the face, approach/retract moves), streamed at rate_hz
    as single-point JointTrajectory commands through arm_ik.solve_track.
    This is automation, NOT hand-following — the hand only starts/aborts.
  * OPEN PALM (or hand loss > hand_loss_abort_sec, camera mode) mid-weld
    ABORTS: motion freezes, then the arm returns home via slewed joint
    motion; the partial bead stays. After a completed weld, re-fisting
    re-welds (bead resets). While idle the arm holds HOME.
  * Weld progress is visible: a growing LINE_STRIP bead on the wall +
    short-lived spark POINTS at the torch tip (welding_effects.py style) on
    /wall_weld_markers; progress fraction on /weld_progress, state on
    /weld_state, machine-readable events on /weld_event.

TORCH GEOMETRY (why tool0 is NOT at the wall face)
  tool0's collision box (URDF gripper, 0.07x0.04x0.08) has a ~0.057 m
  half-diagonal, so the FLANGE cannot come 5 mm from the wall. The torch is
  a virtual stick: tip = wall face point + normal*standoff; tool0 target =
  tip + normal*tool_back + z*tool_up (a real welder's tilted-torch stance).
  tool0 therefore stays ~(standoff+tool_back) off the face — collision-free
  by construction, verified by /check_state_validity in acceptance. tool0
  positions come from position-only warm-start IK (arm_ik.solve_track,
  posture-regularized), same as hand_follow.

WALL PLACEMENT
  wall_mode:=fixed   parameterized default pose (wall_x/y/z/yaw).
  wall_mode:=marker  an ArUco DICT_4X4_50 marker seen by the webcam MAPS to
    the wall pose. HONESTY NOTE: this is a MAPPED placement, not metric
    SLAM/extrinsics — marker image center (u,v) linearly maps into a clamped
    reachable window for wall center (y,z); apparent marker size (side px /
    frame height) maps to wall distance x within [wall_x_min, wall_x_max];
    marker in-plane roll (TL->TR edge angle) maps to wall yaw clamped to
    +-yaw_clamp_deg. Signs use the same mirror convention as the gesture
    preview. Detection runs on the RAW (unflipped) frame — flipping would
    mirror the marker bits; coordinates are mirrored afterwards. cv2 5.0
    ArucoDetector class API (no module-level detectMarkers in this venv).
    Wall pose is re-captured ONLY while idle; /wall_reset (std_srvs/Trigger)
    respawns the wall (idle only) and clears the bead.

RELIABILITY (mandatory design constraints)
  * Reachability pre-check before ACCEPTING any wall pose: 4 corners + a
    grid_ny x grid_nz grid of tool0 targets (+ approach/retract/home) must
    each solve to <= ik_check_err_m with no workspace clamp; on failure the
    wall shrinks by shrink_factor (up to shrink_max times, floor
    wall_min_w/h) with a clear log, else the pose is rejected.
  * IK-failure tick policy: a failed tick (solve err > ik_err_max_m or
    branch-jump flag) holds the arm + warns (throttled); ik_fail_abort
    consecutive failures abort the weld.
  * Per-tick joint delta is always clamped to max_joint_step_rad; per-tick
    task-space advance is weld_speed/rate (weld) or travel_speed/rate.
  * Fist/palm commits need n_fist/n_palm CONSECUTIVE frames (asymmetric
    hysteresis, gen3_pick_place/gesture_sm.py pattern) and the gesture is
    read at the SAME GestureRecognizer result index as the selected hand
    (parallel-arrays invariant).

MODES
  synthetic:=true   scripted wall at the default pose + auto-injected fist
                    (and optionally palm at synthetic_abort_frac progress)
                    through the IDENTICAL SM -> planner -> executor ->
                    effects path; no camera/mediapipe.
  preview:=true     annotated webcam window: per-hand gesture labels, wall
                    capture state, weld progress % (camera mode only).
"""
import math
import os
import signal
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

# arm_ik lives at <ws>/tools in the dev workspace; after a colcon build it is
# also installed next to this script (see CMakeLists). Same dance as
# hand_follow.py.
_here = os.path.dirname(os.path.abspath(__file__))
for _cand in (_here,
              os.path.normpath(os.path.join(_here, "..", "..", "..", "tools")),
              "/ros2_ws/tools"):
    if os.path.isfile(os.path.join(_cand, "arm_ik.py")):
        sys.path.insert(0, _cand)
        break
import arm_ik

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
JLIM = 2.85                      # URDF limit is +-2.9 — keep a margin
TRAJ_TOPIC = "/arm_controller/joint_trajectory"
BASE_FRAME = "base_link"
WALL_ID = "weld_wall"
LABEL_FIST = "Closed_Fist"
LABEL_PALM = "Open_Palm"
WRIST, MID_MCP = 0, 9            # mediapipe hand landmark indices


def check_joint_names(names):
    """None if `names` is exactly joint1..joint6 (any order), else an error
    string (same guard as hand_follow.py — abort on the wrong robot)."""
    if sorted(names) != sorted(JOINTS):
        return ("/joint_states reports joints {} but this node requires "
                "exactly {} — wrong robot or wrong controller running? "
                "Expected the robot_arm_moveit_config demo.launch.py arm."
                .format(sorted(names), JOINTS))
    return None


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Rng:
    """Tiny deterministic LCG for spark scatter (welding_effects.py style)."""

    def __init__(self, seed=12345):
        self.s = seed

    def f(self, lo, hi):
        self.s = (1103515245 * self.s + 12345) & 0x7fffffff
        return lo + (self.s / 0x7fffffff) * (hi - lo)


# ---------------------------------------------------------------------------
# Gesture commit state machine — the asymmetric N-consecutive-frame
# hysteresis pattern proven in gen3_pick_place/gesture_sm.py, reduced to the
# start/abort semantics this node needs. A held fist fires ONCE (latched
# until the label breaks), so "after completion re-fisting re-welds" needs a
# real release + re-fist, never a refire from the same held fist.

class FistPalmSM:
    def __init__(self, n_fist, n_palm, score_min):
        self.n_fist, self.n_palm, self.score_min = n_fist, n_palm, score_min
        self.fist_run = 0
        self.palm_run = 0
        self.latched = None            # 'fist' | 'palm' | None

    def update(self, label, score):
        """One vision frame -> 'fist' | 'palm' | None (commit events)."""
        fist = label == LABEL_FIST and score >= self.score_min
        palm = label == LABEL_PALM and score >= self.score_min
        self.fist_run = self.fist_run + 1 if fist else 0
        self.palm_run = self.palm_run + 1 if palm else 0
        if not fist and self.latched == "fist":
            self.latched = None
        if not palm and self.latched == "palm":
            self.latched = None
        if self.fist_run >= self.n_fist and self.latched != "fist":
            self.latched = "fist"
            return "fist"
        if self.palm_run >= self.n_palm and self.latched != "palm":
            self.latched = "palm"
            return "palm"
        return None


def select_hand(res, want, conf, aspect):
    """Pick the accepted hand from a GestureRecognizer result and bundle ALL
    its per-index data. This is the ONLY place the result's parallel arrays
    (handedness / gestures / hand_landmarks) are indexed — every field comes
    from the SAME index i, so the gesture always belongs to the selected
    hand. Two accepted hands -> the larger (closer) wins. `want` is 'Any',
    'Left' or 'Right' (labels are correct-as-seen because the frame was
    cv2.flip'd first). Returns (i, scale, handed_label, handed_score,
    gesture_label, gesture_score) or None."""
    best = None
    for i in range(len(res.handedness)):
        handed = res.handedness[i]
        if not handed:
            continue
        cat = handed[0]
        if cat.score < conf:
            continue
        if want != "Any" and cat.category_name != want:
            continue
        lms = res.hand_landmarks[i]
        du = (lms[MID_MCP].x - lms[WRIST].x) * aspect
        dv = lms[MID_MCP].y - lms[WRIST].y
        s = math.hypot(du, dv)
        if best is not None and s <= best[1]:
            continue
        glist = res.gestures[i] if i < len(res.gestures) else []
        top = glist[0] if glist else None
        best = (i, s, cat.category_name, cat.score,
                top.category_name if top else None,
                top.score if top else 0.0)
    return best


# ---------------------------------------------------------------------------
# Wall + raster planning

class WallPose:
    """Wall board pose/size in base_link. yaw about +Z; the welded FACE is
    the -x' side (normal pointing back toward the robot base)."""

    def __init__(self, x, y, z, yaw, w, h, t):
        self.x, self.y, self.z, self.yaw = x, y, z, yaw
        self.w, self.h, self.t = w, h, t

    @property
    def normal(self):
        """Unit normal of the welded face (toward the robot)."""
        return (-math.cos(self.yaw), -math.sin(self.yaw), 0.0)

    @property
    def tangent(self):
        """In-plane horizontal unit vector (wall-frame +y')."""
        return (-math.sin(self.yaw), math.cos(self.yaw), 0.0)

    def face_point(self, yp, zoff):
        """Point ON the welded face at wall-frame (y', z-offset)."""
        n, t = self.normal, self.tangent
        return (self.x + n[0] * (self.t / 2) + t[0] * yp,
                self.y + n[1] * (self.t / 2) + t[1] * yp,
                self.z + zoff)

    def desc(self):
        return ("({:.3f},{:.3f},{:.3f}) yaw {:.1f}deg {:.2f}x{:.2f}x{:.3f}"
                .format(self.x, self.y, self.z, math.degrees(self.yaw),
                        self.w, self.h, self.t))


class RasterPlan:
    """Planned serpentine: parallel lists tool0[i] (IK targets), tips[i]
    (torch tip = face + n*standoff), bead[i] (drawn on the face)."""

    def __init__(self, tool0, tips, bead, n_rows, approach, retract, offset):
        self.tool0, self.tips, self.bead = tool0, tips, bead
        self.n_rows = n_rows
        self.approach, self.retract = approach, retract
        self.offset = offset           # tool0 target = tip + offset (3-vec)
        self.path_len = sum(math.dist(a, b)
                            for a, b in zip(tool0, tool0[1:]))


class RasterPlanner:
    """Serpentine waypoints in the wall frame -> base frame, plus the
    corner+grid reachability pre-check with shrink-to-fit."""

    def __init__(self, node):
        self.n = node

    def _offsets(self, wall):
        n = wall.normal
        return (n[0] * self.n.tool_back, n[1] * self.n.tool_back,
                self.n.tool_up)

    def _targets(self, wall, yp, zoff):
        """(tip, tool0, bead) for one wall-frame raster point."""
        n = wall.normal
        f = wall.face_point(yp, zoff)
        tip = (f[0] + n[0] * self.n.standoff, f[1] + n[1] * self.n.standoff,
               f[2])
        off = self._offsets(wall)
        tool0 = (tip[0] + off[0], tip[1] + off[1], tip[2] + off[2])
        bead = (f[0] + n[0] * self.n.bead_proud,
                f[1] + n[1] * self.n.bead_proud, f[2])
        return tip, tool0, bead

    def plan(self, wall):
        hw = wall.w / 2 - self.n.margin
        hh = wall.h / 2 - self.n.margin
        if hw <= 0.0 or hh <= 0.0:
            raise ValueError(
                "margin {:.3f} leaves no weldable area on a {:.2f}x{:.2f} "
                "wall — need margin < min(w, h)/2".format(
                    self.n.margin, wall.w, wall.h))
        n_rows = int(2 * hh / self.n.row_step + 1e-9) + 1
        n_pass = int(2 * hw / self.n.pass_step + 1e-9) + 1
        tool0, tips, bead = [], [], []
        for r in range(n_rows):
            z = -hh + r * self.n.row_step
            ys = [-hw + i * self.n.pass_step for i in range(n_pass)]
            if r % 2:
                ys.reverse()
            for y in ys:
                tp, t0, bd = self._targets(wall, y, z)
                tips.append(tp)
                tool0.append(t0)
                bead.append(bd)
        nrm = wall.normal
        ap = tuple(tool0[0][k] + nrm[k] * self.n.approach_dist
                   for k in range(3))
        rt = tuple(tool0[-1][k] + nrm[k] * self.n.approach_dist
                   for k in range(3))
        return RasterPlan(tool0, tips, bead, n_rows, ap, rt,
                          self._offsets(wall))

    def _bad_points(self, wall):
        """Reachability check: 4 corners + grid of tool0 targets, plus
        approach/retract/home. Each point solved fresh from a waist-aimed
        NOMINAL seed; fails on workspace clamp or err > ik_check_err_m (the
        solve_track jump flag is a streaming-continuity signal, meaningless
        for isolated far-apart probes, so it is ignored here)."""
        hw = wall.w / 2 - self.n.margin
        hh = wall.h / 2 - self.n.margin
        pts = [self._targets(wall, sy * hw, sz * hh)[1]
               for sy in (-1, 1) for sz in (-1, 1)]
        ny, nz = self.n.grid_ny, self.n.grid_nz
        for i in range(ny):
            for j in range(nz):
                yp = -hw + i * (2 * hw) / max(ny - 1, 1)
                zo = -hh + j * (2 * hh) / max(nz - 1, 1)
                pts.append(self._targets(wall, yp, zo)[1])
        nrm = wall.normal
        pts.append(tuple(pts[0][k] + nrm[k] * self.n.approach_dist
                         for k in range(3)))
        pts.append((self.n.home_x, self.n.home_y, self.n.home_z))
        bad = []
        for p in pts:
            seed = list(arm_ik.NOMINAL)
            seed[0] = math.atan2(p[1], p[0])
            _, info = arm_ik.solve_track(p, seed)
            if info["clamped"] or info["err_m"] > self.n.ik_check_err_m:
                bad.append((p, info["clamped"], info["err_m"]))
        return bad, len(pts)

    def precheck(self, wall):
        """(ok, wall_possibly_shrunk, report). Shrinks w/h by shrink_factor
        up to shrink_max times before rejecting; every step is logged."""
        w = wall
        for attempt in range(self.n.shrink_max + 1):
            bad, n_pts = self._bad_points(w)
            if not bad:
                note = ("" if attempt == 0 else
                        " — SHRUNK from {:.2f}x{:.2f} to fit reach"
                        .format(wall.w, wall.h))
                return True, w, ("reach precheck OK: {} pts (corners+{}x{} "
                                 "grid+approach+home) all <= {:.1f} mm{}"
                                 .format(n_pts, self.n.grid_ny,
                                         self.n.grid_nz,
                                         self.n.ik_check_err_m * 1e3, note))
            self.n.get_logger().warning(
                "wall precheck: {}/{} unreachable pts at {:.2f}x{:.2f} "
                "(worst {}) — shrinking by {:.2f}"
                .format(len(bad), n_pts, w.w, w.h,
                        "clamped" if bad[0][1] else
                        "{:.1f} mm".format(bad[0][2] * 1e3),
                        self.n.shrink_factor))
            w = WallPose(w.x, w.y, w.z, w.yaw, w.w * self.n.shrink_factor,
                         w.h * self.n.shrink_factor, w.t)
            if w.w < self.n.wall_min_w or w.h < self.n.wall_min_h:
                break
        return False, wall, ("reach precheck REJECTED wall {}: unreachable "
                             "even after shrinking to {:.2f}x{:.2f} (floor "
                             "{:.2f}x{:.2f})".format(
                                 wall.desc(), w.w, w.h, self.n.wall_min_w,
                                 self.n.wall_min_h))


class WallManager:
    """Owns the weld_wall CollisionObject lifecycle via /apply_planning_scene
    (verified ADD form from gen3_pick_place/scene_box.py — ADD with the same
    id also moves an existing object)."""

    def __init__(self, node):
        self.node = node
        self.aps = node.create_client(ApplyPlanningScene,
                                      "/apply_planning_scene",
                                      callback_group=node._cbg)

    def spawn(self, wall, timeout=8.0):
        scene = PlanningScene()
        scene.is_diff = True
        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.id = WALL_ID
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(wall.t), float(wall.w), float(wall.h)]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(wall.x), float(wall.y), float(wall.z))
        pose.orientation.z = math.sin(wall.yaw / 2)
        pose.orientation.w = math.cos(wall.yaw / 2)
        co.primitives = [prim]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
        scene.world.collision_objects = [co]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        if not self.aps.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().error("/apply_planning_scene unavailable")
            return False
        res = self.node._wait_future(self.aps.call_async(req), timeout)
        return res is not None and res.success


# ---------------------------------------------------------------------------

class ThreadedCapture:
    """Camera reader on its own thread (latest-frame only) — verbatim
    pattern from hand_follow.py."""

    def __init__(self, device):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                "camera {} failed to open — is the container started by the "
                "ros2-arm launcher (needs --device /dev/video0)?".format(device))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, _ = self.cap.read()
        if not ok:
            self.cap.release()
            raise RuntimeError("camera {} opened but read() failed — device "
                               "busy or wrong node?".format(device))
        self.cond = threading.Condition()
        self.frame, self.t, self.seq = None, 0.0, 0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True,
                                       name="capture")
        self.thread.start()

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self.cond:
                self.frame, self.t, self.seq = f, time.monotonic(), self.seq + 1
                self.cond.notify_all()

    def get(self, last_seq, timeout=0.5):
        with self.cond:
            if self.seq == last_seq:
                self.cond.wait(timeout)
            if self.seq == last_seq or self.frame is None:
                return None
            return self.seq, self.frame, self.t

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)
        self.cap.release()


# ---------------------------------------------------------------------------

class WallWeld(Node):
    """States: INIT -> HOMING -> IDLE <-> (APPROACH -> WELD -> RETRACT ->
    HOMING); ABORT_FREEZE (from APPROACH/WELD) -> HOMING. The 20 Hz control
    tick owns all state transitions; vision/synthetic threads only post
    start/abort requests and (idle-gated) wall captures."""

    def __init__(self):
        super().__init__("wall_weld")
        p = self.declare_parameter
        # -- mode / io
        self.synthetic = p("synthetic", False).value
        self.preview = p("preview", False).value
        self.camera = p("camera", "/dev/video0").value
        self.model_path = p("model_path",
                            "/opt/models/gesture_recognizer.task").value
        self.rate_hz = p("rate_hz", 20.0).value
        self.tfs = p("time_from_start", 0.12).value
        # -- wall (fixed-mode pose; marker mode replaces pose, keeps size)
        self.wall_mode = p("wall_mode", "fixed").value
        self.wall_x0 = p("wall_x", 0.30).value
        self.wall_y0 = p("wall_y", 0.0).value
        self.wall_z0 = p("wall_z", 0.25).value
        self.wall_yaw0 = p("wall_yaw", 0.0).value
        self.wall_w0 = p("wall_width", 0.35).value
        self.wall_h0 = p("wall_height", 0.25).value
        self.wall_t0 = p("wall_thick", 0.02).value
        # -- raster / torch geometry
        self.row_step = p("row_step", 0.02).value
        self.pass_step = p("pass_step", 0.01).value
        self.margin = p("margin", 0.02).value
        self.standoff = p("standoff", 0.015).value
        self.tool_back = p("tool_back", 0.07).value
        self.tool_up = p("tool_up", 0.09).value
        self.approach_dist = p("approach_dist", 0.06).value
        self.bead_proud = p("bead_proud", 0.003).value
        self.weld_speed = p("weld_speed", 0.05).value
        self.travel_speed = p("travel_speed", 0.08).value
        # -- control / safety
        self.max_dq = p("max_joint_step_rad", 0.10).value
        self.ik_err_max = p("ik_err_max_m", 0.004).value
        self.ik_fail_abort = p("ik_fail_abort", 5).value
        self.home_x = p("home_x", 0.20).value
        self.home_y = p("home_y", 0.0).value
        self.home_z = p("home_z", 0.30).value
        self.freeze_sec = p("freeze_sec", 0.3).value
        self.settle_rad = p("settle_rad", 0.01).value
        # -- gesture
        self.hand = p("hand", "Any").value
        self.hand_conf = p("hand_conf", 0.6).value
        self.gesture_score = p("gesture_score", 0.6).value
        self.n_fist = p("n_fist", 5).value
        self.n_palm = p("n_palm", 3).value
        self.hand_loss_abort = p("hand_loss_abort_sec", 3.0).value
        # -- marker capture (mapped placement — see header)
        self.marker_id = p("marker_id", -1).value
        self.aruco_every = p("aruco_every", 3).value
        self.capture_frames = p("capture_frames", 10).value
        self.recapture_min_move = p("recapture_min_move_m", 0.02).value
        self.recapture_min_yaw = p("recapture_min_yaw_rad", 0.09).value
        # realtime wall tracking: the wall FOLLOWS the marker continuously
        # while IDLE (EMA-smoothed, throttled scene re-poses, bead cleared on
        # move); the raster is planned lazily when the fist commits.
        self.wall_track = p("wall_track", False).value
        self.track_hz = p("track_hz", 4.0).value
        self.track_alpha = p("track_alpha", 0.35).value
        self.track_eps_m = p("track_eps_m", 0.008).value
        self.track_eps_yaw = p("track_eps_yaw", 0.03).value
        self.wx_min = p("wall_x_min", 0.26).value
        self.wx_max = p("wall_x_max", 0.34).value
        self.wy_min = p("wall_y_min", -0.10).value
        self.wy_max = p("wall_y_max", 0.10).value
        self.wz_min = p("wall_z_min", 0.20).value
        self.wz_max = p("wall_z_max", 0.32).value
        self.yaw_lim = math.radians(p("yaw_clamp_deg", 20.0).value)
        self.m_near = p("marker_s_near", 0.30).value
        self.m_far = p("marker_s_far", 0.08).value
        # -- precheck / shrink
        self.grid_ny = p("grid_ny", 7).value
        self.grid_nz = p("grid_nz", 5).value
        self.ik_check_err_m = p("ik_check_err_m", 0.003).value
        self.shrink_factor = p("shrink_factor", 0.85).value
        self.shrink_max = p("shrink_max", 4).value
        self.wall_min_w = p("wall_min_w", 0.10).value
        self.wall_min_h = p("wall_min_h", 0.06).value
        # -- synthetic script
        self.synth_start = p("synth_start_sec", 2.0).value
        self.synth_abort_frac = p("synthetic_abort_frac", -1.0).value

        self._cbg = ReentrantCallbackGroup()
        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self.pub_markers = self.create_publisher(MarkerArray,
                                                 "/wall_weld_markers", 10)
        self.pub_progress = self.create_publisher(Float32, "/weld_progress", 10)
        self.pub_state = self.create_publisher(String, "/weld_state", 10)
        self.pub_event = self.create_publisher(String, "/weld_event", 10)
        self.sub_js = self.create_subscription(JointState, "/joint_states",
                                               self._on_js, 10)
        self.srv_reset = self.create_service(Trigger, "/wall_reset",
                                             self._on_wall_reset,
                                             callback_group=self._cbg)
        self.planner = RasterPlanner(self)
        self.wallmgr = WallManager(self)
        self.sm = FistPalmSM(self.n_fist, self.n_palm, self.gesture_score)
        self.rng = Rng()

        self.lock = threading.RLock()  # reentrant: tick starts welds under it
        self.js_msg, self.js_time = None, None
        self.running = True
        self._spinning = False         # executor started (service-call mode)
        self.state = "INIT"
        self.q_cmd = None
        self.q_home = None
        self.wall = None               # accepted WallPose
        self.plan = None               # RasterPlan for self.wall
        self.respawn_hold = None       # why-string while a respawn is in
        #   flight; set ONLY via _acquire_idle_hold (atomically with a
        #   state==IDLE check) and blocks the tick's IDLE->APPROACH start,
        #   closing the capture/reset vs fist TOCTOU window
        self.bead = []                 # deposited bead pts (base frame)
        self.progress = 0.0
        self.req_start = None
        self.req_abort = None
        self.last_seen = None          # monotonic t of last accepted hand
        self.cmd_pos = None            # task-space slewed position (3-list)
        self.move_seq, self.move_idx = None, 0
        self.path_idx = 0              # last raster waypoint reached
        self.ik_fail_run = 0
        self.t_freeze = 0.0
        self.advanced = False          # last tick moved (sparks gate)
        self.last_gesture = (None, 0.0)
        self.marker_state = "no marker"
        self._marker_window = deque(maxlen=max(int(self.capture_frames), 1))
        self._track_f = None               # EMA state (x, y, z, yaw)
        self._last_repose_t = 0.0
        self._last_track_event_t = 0.0
        self._marker_seen = None
        self._capture_cooldown_t = 0.0
        self.capture = None
        self.recognizer = None
        self._aruco = None
        self.t0 = time.monotonic()
        self.n_pub = 0

    # ------------------------------------------------------------- plumbing
    def _on_js(self, msg):
        with self.lock:
            self.js_msg = msg
            self.js_time = time.monotonic()

    def _wait_future(self, fut, timeout):
        """Bounded wait usable both before the executor exists (preflight:
        spin_once) and while it spins (poll — the reentrant group processes
        the response on another executor thread)."""
        deadline = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < deadline:
            if self._spinning:
                time.sleep(0.02)
            else:
                rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def _event(self, text):
        self.pub_event.publish(String(data=text))
        self.get_logger().info("EVENT " + text)

    def _set_state(self, st):
        if st != self.state:
            self.get_logger().info("state {} -> {}".format(self.state, st))
            self.state = st
            self.pub_state.publish(String(data=st))

    # ------------------------------------------------------------- wall ops
    def _acquire_idle_hold(self, why):
        """Atomic (state == IDLE and no hold) -> set respawn_hold. While the
        hold is set the 20 Hz tick refuses IDLE->APPROACH (its start check
        runs under the same lock), so a fist committed in the same window as
        a wall capture / /wall_reset can no longer start a weld whose plan
        is then swapped mid-flight. Callers MUST _release_idle_hold()."""
        with self.lock:
            if self.state != "IDLE" or self.respawn_hold is not None:
                return False
            self.respawn_hold = why
            return True

    def _release_idle_hold(self):
        with self.lock:
            self.respawn_hold = None

    def _respawn_wall(self, wall, why):
        """Plan FIRST (a degenerate/failed plan leaves the planning scene
        untouched), then move the collision object, then commit wall/plan +
        bead clear atomically. Any caller that can run concurrently with
        the control tick (/wall_reset, marker capture) must be inside
        _acquire_idle_hold; preflight runs before the tick exists."""
        try:
            plan = self.planner.plan(wall)
        except ValueError as exc:
            self._event("WALL_PLAN_FAILED why={} err='{}'".format(why, exc))
            return False
        ok = self.wallmgr.spawn(wall)
        if not ok:
            self._event("WALL_SPAWN_FAILED why=" + why)
            return False
        with self.lock:
            self.wall = wall
            self.plan = plan
            self.bead = []
            self.progress = 0.0
        self._event("WALL_SPAWNED {} why={}".format(wall.desc(), why))
        self._event("PLAN waypoints={} rows={} path_m={:.2f} standoff={} "
                    "row_step={}".format(len(plan.tool0), plan.n_rows,
                                         plan.path_len, self.standoff,
                                         self.row_step))
        return True

    def _track_repose(self, wall):
        """Lightweight LIVE re-pose for wall_track mode: move the collision
        object + commit the wall with plan=None (raster is planned lazily at
        fist time by _tick). Runs on the vision thread, only while IDLE, and
        under the same idle-hold that makes captures atomic vs the tick."""
        if not self._acquire_idle_hold("wall track"):
            return False
        try:
            if not self.wallmgr.spawn(wall):
                return False
            with self.lock:
                self.wall = wall
                self.plan = None           # stale by definition — lazy plan
                self.bead = []
                self.progress = 0.0
            return True
        finally:
            self._release_idle_hold()

    def _on_wall_reset(self, req, resp):
        if not self._acquire_idle_hold("/wall_reset"):
            resp.success = False
            resp.message = ("busy: state {} — reset only while idle"
                            .format(self.state))
            return resp
        try:
            with self.lock:
                wall = self.wall
            if wall is None:
                resp.success = False
                resp.message = "no wall accepted yet"
                return resp
            ok = self._respawn_wall(wall, "/wall_reset")
            resp.success = ok
            resp.message = ("wall respawned at " + wall.desc()) if ok else \
                "wall respawn failed (see /weld_event)"
            return resp
        finally:
            self._release_idle_hold()

    # ------------------------------------------------------------- preflight
    def preflight(self):
        log = self.get_logger()
        yaw = clamp(self.wall_yaw0, -self.yaw_lim, self.yaw_lim)
        if yaw != self.wall_yaw0:
            log.warning("wall_yaw {:.2f} clamped to +-{:.2f} rad"
                        .format(self.wall_yaw0, self.yaw_lim))
        # The precheck shrink-to-fit can accept any wall down to
        # min(configured, floor) per axis; margin must leave weldable area
        # on the SMALLEST of those or RasterPlanner.plan() degenerates.
        small_w = min(self.wall_w0, self.wall_min_w)
        small_h = min(self.wall_h0, self.wall_min_h)
        if self.margin >= min(small_w, small_h) / 2:
            log.fatal(
                "margin {:.3f} >= half of the smallest acceptable wall "
                "dimension ({:.2f}x{:.2f}, shrink floor {:.2f}x{:.2f}) — "
                "the raster would be empty after shrink-to-fit; lower "
                "margin or raise wall_min_w/wall_min_h".format(
                    self.margin, small_w, small_h,
                    self.wall_min_w, self.wall_min_h))
            return False
        if not self.synthetic:
            if not os.path.isfile(self.model_path):
                log.fatal("model file missing: {} — the ros2-arm:jazzy image "
                          "bakes it at /opt/models/gesture_recognizer.task"
                          .format(self.model_path))
                return False
            try:
                self.capture = ThreadedCapture(self.camera)
            except RuntimeError as e:
                log.fatal(str(e))
                return False
            log.info("camera {} OK, model {} OK".format(self.camera,
                                                        self.model_path))
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and self.js_msg is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.js_msg is None:
            log.fatal("no /joint_states within 12 s — start the arm demo "
                      "first (ros2-arm wallweld includes it)")
            return False
        err = check_joint_names(list(self.js_msg.name))
        if err:
            log.fatal(err)
            return False
        pos = dict(zip(self.js_msg.name, self.js_msg.position))
        self.q_cmd = [pos[j] for j in JOINTS]
        home = (self.home_x, self.home_y, self.home_z)
        self.q_home, info = arm_ik.solve_track(home, list(arm_ik.NOMINAL))
        if info["err_m"] > self.ik_check_err_m:
            log.fatal("home target {} unreachable (err {:.1f} mm)"
                      .format(home, info["err_m"] * 1e3))
            return False
        if not self.wallmgr.aps.wait_for_service(timeout_sec=20.0):
            log.fatal("/apply_planning_scene unavailable — move_group up?")
            return False
        wall = WallPose(self.wall_x0, self.wall_y0, self.wall_z0, yaw,
                        self.wall_w0, self.wall_h0, self.wall_t0)
        ok, wall, report = self.planner.precheck(wall)
        log.info(report)
        if not ok:
            log.fatal("default wall pose failed the reachability precheck — "
                      "fix wall_* params")
            return False
        if not self._respawn_wall(wall, "preflight default ({} mode)"
                                  .format(self.wall_mode)):
            log.fatal("initial wall spawn failed")
            return False
        if self.pub.get_subscription_count() == 0:
            log.warning("{} has no subscriber yet — arm_controller not up? "
                        "streaming anyway".format(TRAJ_TOPIC))
        self._set_state("HOMING")
        log.info("preflight OK: seed q={} home q={} tool0 home={}"
                 .format([round(v, 3) for v in self.q_cmd],
                         [round(v, 3) for v in self.q_home], home))
        return True

    # ------------------------------------------------------------- threads
    def start(self):
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.hb_timer = self.create_timer(1.0, self._heartbeat)
        if self.synthetic:
            self.vision_thread = threading.Thread(target=self._synth_loop,
                                                  daemon=True, name="synth")
            self.get_logger().info(
                "SYNTHETIC mode: scripted wall + auto fist at t>={}s{}"
                .format(self.synth_start,
                        ", palm inject at progress>={:.2f}".format(
                            self.synth_abort_frac)
                        if self.synth_abort_frac >= 0 else ""))
        else:
            base = mp_python.BaseOptions(model_asset_path=self.model_path)
            self.recognizer = mp_vision.GestureRecognizer.create_from_options(
                mp_vision.GestureRecognizerOptions(
                    base_options=base,
                    running_mode=mp_vision.RunningMode.VIDEO,
                    num_hands=2,
                    min_hand_detection_confidence=0.5,
                    min_tracking_confidence=0.5))
            if self.wall_mode == "marker":
                d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                self._aruco = cv2.aruco.ArucoDetector(
                    d, cv2.aruco.DetectorParameters())
            self.vision_thread = threading.Thread(target=self._vision_loop,
                                                  daemon=True, name="vision")
            self.get_logger().info(
                "camera mode: hand={} conf>={}, fist x{} = WELD, palm x{} = "
                "ABORT, wall_mode={}".format(self.hand, self.hand_conf,
                                             self.n_fist, self.n_palm,
                                             self.wall_mode))
        self.vision_thread.start()

    def _request_start(self, why):
        with self.lock:
            self.req_start = why

    def _request_abort(self, why):
        with self.lock:
            self.req_abort = why

    # ------------------------------------------------------------- synthetic
    def _synth_loop(self):
        """30 Hz scripted fake-vision frames through the IDENTICAL SM ->
        request -> control path as the camera pipeline (acceptance a/b)."""
        t0 = time.monotonic()
        fist_done = False
        injected = False
        while self.running and rclpy.ok():
            now = time.monotonic()
            t = now - t0
            with self.lock:
                st, prog = self.state, self.progress
            label, score = None, 0.0
            if st in ("APPROACH", "WELD"):
                fist_done = True
            if not fist_done and st == "IDLE" and t >= self.synth_start:
                label, score = LABEL_FIST, 0.9
            if (self.synth_abort_frac >= 0.0 and not injected
                    and st == "WELD" and prog >= self.synth_abort_frac):
                injected = True
                self._event("SYNTH_PALM_INJECT t={:.2f} progress={:.3f}"
                            .format(t, prog))
            if injected and st in ("APPROACH", "WELD"):
                label, score = LABEL_PALM, 0.9
            ev = self.sm.update(label, score)
            if ev == "fist":
                self._request_start("synthetic fist")
            elif ev == "palm":
                self._request_abort("synthetic palm")
            time.sleep(1.0 / 30.0)

    # ------------------------------------------------------------- vision
    def _vision_loop(self):
        seq, ts_prev, frame_i = 0, -1, 0
        log = self.get_logger()
        while self.running and rclpy.ok():
            got = self.capture.get(seq)
            if got is None:
                continue
            seq, raw, t_frame = got
            frame_i += 1
            try:
                frame = cv2.flip(raw, 1)       # mirror view — labels correct
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w = rgb.shape[:2]
                ts = max(int((t_frame - self.t0) * 1000), ts_prev + 1)
                ts_prev = ts
                res = self.recognizer.recognize_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)
                obs = select_hand(res, self.hand, self.hand_conf, w / h)
                now = time.monotonic()
                if obs is not None:
                    with self.lock:
                        self.last_seen = now
                        self.last_gesture = (obs[4], obs[5])
                    ev = self.sm.update(obs[4], obs[5])
                else:
                    with self.lock:
                        self.last_gesture = (None, 0.0)
                    ev = self.sm.update(None, 0.0)
                    if res.handedness:
                        log.info("hand(s) seen but none accepted (need {} "
                                 ">= {:.2f}): {}".format(
                                     self.hand, self.hand_conf, ", ".join(
                                         "{} {:.2f}".format(
                                             hd[0].category_name, hd[0].score)
                                         for hd in res.handedness)),
                                 throttle_duration_sec=2.0)
                if ev == "fist":
                    self._request_start("fist (held x{})".format(self.n_fist))
                elif ev == "palm":
                    self._request_abort("open palm")
                if (self._aruco is not None and self.state == "IDLE"
                        and frame_i % max(int(self.aruco_every), 1) == 0):
                    self._aruco_tick(raw, now)
                if self.preview:
                    self._show_preview(frame, res, obs)
            except Exception as exc:  # noqa: BLE001 — vision must not die
                log.error("vision frame error: {}".format(exc),
                          throttle_duration_sec=2.0)

    # ------------------------------------------------------------- aruco
    def _aruco_tick(self, raw, now):
        """MAPPED wall placement from the largest (or id-matched) marker —
        see the header for the honesty note. Only ever called while IDLE."""
        corners, ids, _ = self._aruco.detectMarkers(raw)
        sel = None
        if ids is not None:
            for c, mid in zip(corners, ids.flatten()):
                if self.marker_id >= 0 and int(mid) != self.marker_id:
                    continue
                pts = c[0]                     # 4x2 float: TL,TR,BR,BL
                side = sum(
                    math.dist(pts[i], pts[(i + 1) % 4]) for i in range(4)) / 4
                if sel is None or side > sel[1]:
                    sel = (pts, side, int(mid))
        if sel is None:
            self._marker_window.clear()
            self._marker_seen = None
            self.marker_state = "no marker"
            return
        pts, side, mid = sel
        h, w = raw.shape[:2]
        u = 1.0 - float(pts[:, 0].mean()) / w      # mirror-view u
        v = float(pts[:, 1].mean()) / h
        s = side / h
        roll = math.atan2(float(pts[1][1] - pts[0][1]),
                          float(pts[1][0] - pts[0][0]))
        wy = self.wy_min + clamp(u, 0.0, 1.0) * (self.wy_max - self.wy_min)
        wz = self.wz_max - clamp(v, 0.0, 1.0) * (self.wz_max - self.wz_min)
        k = clamp((s - self.m_far) / (self.m_near - self.m_far), 0.0, 1.0)
        wx = self.wx_max - k * (self.wx_max - self.wx_min)
        yaw = clamp(-roll, -self.yaw_lim, self.yaw_lim)
        if self.wall_track:
            # REALTIME tracking: EMA-smooth the mapped pose and re-pose the
            # scene wall live (throttled). No stability window, no cooldown —
            # the wall follows the marker; fist plans lazily wherever it is.
            a = clamp(self.track_alpha, 0.05, 1.0)
            if self._track_f is None:
                self._track_f = (wx, wy, wz, yaw)
            else:
                f = self._track_f
                self._track_f = tuple(a * n + (1.0 - a) * o
                                      for n, o in zip((wx, wy, wz, yaw), f))
            sx, sy, sz, syaw = self._track_f
            self.marker_state = "id {} TRACKING".format(mid)
            if now - self._last_repose_t < 1.0 / max(self.track_hz, 0.1):
                return
            cur = self.wall
            if cur is not None and (
                    math.dist((sx, sy, sz), (cur.x, cur.y, cur.z))
                    < self.track_eps_m
                    and abs(syaw - cur.yaw) < self.track_eps_yaw):
                return                         # holding still — no churn
            wall = WallPose(sx, sy, sz, syaw,
                            self.wall_w0, self.wall_h0, self.wall_t0)
            if self._track_repose(wall):
                self._last_repose_t = now
                if now - self._last_track_event_t > 1.5:
                    self._last_track_event_t = now
                    self._event("WALL_TRACKED " + wall.desc())
            return
        self._marker_window.append((wx, wy, wz, yaw))
        self._marker_seen = (pts, mid, now)
        n_seen = len(self._marker_window)
        self.marker_state = "id {} seen {}/{}".format(mid, n_seen,
                                                      self.capture_frames)
        if n_seen < self.capture_frames or now < self._capture_cooldown_t:
            return
        cols = list(zip(*self._marker_window))
        spread = max(max(c) - min(c) for c in cols[:3])
        yaw_spread = max(cols[3]) - min(cols[3])
        if spread > 0.02 or yaw_spread > 0.09:     # marker still moving
            return
        cand = [sum(c) / n_seen for c in cols]
        cur = self.wall
        if cur is not None and (
                math.dist(cand[:3], (cur.x, cur.y, cur.z))
                < self.recapture_min_move
                and abs(cand[3] - cur.yaw) < self.recapture_min_yaw):
            self.marker_state = "id {} captured (stable)".format(mid)
            return
        wall = WallPose(cand[0], cand[1], cand[2], cand[3],
                        self.wall_w0, self.wall_h0, self.wall_t0)
        ok, wall, report = self.planner.precheck(wall)
        self.get_logger().info(report)
        if not ok:
            self._event("WALL_CAPTURE_REJECTED marker={} {}".format(mid,
                                                                    report))
            self._marker_window.clear()
            self._capture_cooldown_t = now + 2.0
            return
        if not self._acquire_idle_hold("marker capture"):
            # a weld started while we were validating — drop this capture;
            # the IDLE gate in _vision_loop resumes detection after homing
            return
        try:
            if self._respawn_wall(wall, "marker capture id={}".format(mid)):
                self.marker_state = "id {} CAPTURED".format(mid)
                self._capture_cooldown_t = now + 1.0
        finally:
            self._release_idle_hold()

    # ------------------------------------------------------------- preview
    def _show_preview(self, frame, res, obs):
        h, w = frame.shape[:2]
        for i in range(len(res.handedness)):
            cat = res.handedness[i][0]
            lms = res.hand_landmarks[i]
            sel = obs is not None and i == obs[0]
            color = ((0, 220, 0) if sel else
                     (0, 200, 255) if cat.score >= self.hand_conf
                     else (80, 80, 255))
            for lm in lms:
                cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 3, color, -1)
            glist = res.gestures[i] if i < len(res.gestures) else []
            tag = "{} {:.2f}".format(cat.category_name, cat.score)
            if glist:
                tag += " | {} {:.2f}".format(glist[0].category_name,
                                             glist[0].score)
            wx, wy = int(lms[WRIST].x * w), int(lms[WRIST].y * h)
            cv2.putText(frame, tag, (max(wx - 70, 2), max(wy - 12, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        if self._marker_seen is not None and \
                time.monotonic() - self._marker_seen[2] < 0.5:
            pts = self._marker_seen[0]
            for i in range(4):
                p1 = (int(w - 1 - pts[i][0]), int(pts[i][1]))
                p2 = (int(w - 1 - pts[(i + 1) % 4][0]),
                      int(pts[(i + 1) % 4][1]))
                cv2.line(frame, p1, p2, (255, 120, 0), 2)
        with self.lock:
            st, prog, wall = self.state, self.progress, self.wall
            glabel, gscore = self.last_gesture
        cv2.rectangle(frame, (0, 0), (w, 44), (30, 30, 30), -1)
        line1 = "[{}] weld {:3.0f}%  gesture: {}".format(
            st, prog * 100,
            "{} {:.2f}".format(glabel, gscore) if glabel else "-")
        line2 = "wall({}): {}  {}".format(
            self.wall_mode, wall.desc() if wall else "none",
            self.marker_state if self.wall_mode == "marker" else "")
        cv2.putText(frame, line1, (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 220, 0) if st == "WELD" else (0, 200, 255), 1)
        cv2.putText(frame, line2, (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (200, 200, 200), 1)
        cv2.imshow("wall_weld — webcam (press q to close)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.preview = False
            cv2.destroyAllWindows()

    # ------------------------------------------------------------- control
    def _publish_traj(self, q):
        msg = JointTrajectory()
        msg.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start.sec = int(self.tfs)
        pt.time_from_start.nanosec = int((self.tfs - int(self.tfs)) * 1e9)
        msg.points.append(pt)
        self.pub.publish(msg)
        self.n_pub += 1

    def _slew_joints(self, q_tgt):
        """One tick of joint-space slew toward q_tgt. True when settled."""
        q = [max(-JLIM, min(JLIM,
             c + max(-self.max_dq, min(self.max_dq, t - c))))
             for c, t in zip(self.q_cmd, q_tgt)]
        self.q_cmd = q
        return max(abs(t - c) for t, c in zip(q_tgt, q)) < self.settle_rad

    def _solve_step(self, pos, allow_abort):
        """IK for one task-space position with the failure policy: on
        failure hold + throttled warn; ik_fail_abort consecutive -> abort
        (when allow_abort). True = q_cmd advanced toward pos."""
        q, info = arm_ik.solve_track(tuple(pos), self.q_cmd)
        if info["jump"] or info["err_m"] > self.ik_err_max:
            self.ik_fail_run += 1
            self.get_logger().warning(
                "IK failure #{} at ({:.3f},{:.3f},{:.3f}): err {:.1f} mm "
                "jump={} — holding".format(self.ik_fail_run, *pos,
                                           info["err_m"] * 1e3, info["jump"]),
                throttle_duration_sec=1.0)
            if allow_abort and self.ik_fail_run >= self.ik_fail_abort:
                self._abort("IK failed {} consecutive ticks"
                            .format(self.ik_fail_run))
            return False
        self.ik_fail_run = 0
        self.q_cmd = [max(-JLIM, min(JLIM,
                      c + max(-self.max_dq, min(self.max_dq, v - c))))
                      for c, v in zip(self.q_cmd, q)]
        return True

    @staticmethod
    def _advance(pos, idx, dist, pts):
        """Move `dist` m along polyline pts from (pos, idx-last-reached)."""
        pos = list(pos)
        while dist > 1e-12 and idx < len(pts) - 1:
            tgt = pts[idx + 1]
            d = math.dist(pos, tgt)
            if d <= dist:
                pos, idx, dist = list(tgt), idx + 1, dist - d
            else:
                pos = [p + (t - p) * dist / d for p, t in zip(pos, tgt)]
                dist = 0.0
        return pos, idx

    def _start_weld(self, why):
        plan = self.plan
        with self.lock:
            self.bead = []
            self.progress = 0.0
        self.ik_fail_run = 0
        self.cmd_pos = list(arm_ik.fk_tool0(self.q_cmd))
        self.move_seq = [tuple(self.cmd_pos), plan.approach, plan.tool0[0]]
        self.move_idx = 0
        self.path_idx = 0
        self._event("START {} wall={} waypoints={}".format(
            why, self.wall.desc(), len(plan.tool0)))
        self._set_state("APPROACH")

    def _abort(self, why):
        with self.lock:
            prog = self.progress
        self._event("ABORT reason='{}' progress={:.3f} bead_pts={}".format(
            why, prog, len(self.bead)))
        self.t_freeze = time.monotonic()
        self._set_state("ABORT_FREEZE")

    def _finish_weld(self):
        with self.lock:
            self.progress = 1.0
        self._event("DONE waypoints={} bead_pts={}".format(
            len(self.plan.tool0), len(self.bead)))
        self.cmd_pos = list(self.plan.tool0[-1])
        self.move_seq = [tuple(self.cmd_pos), self.plan.retract]
        self.move_idx = 0
        self._set_state("RETRACT")

    def _weld_tick(self):
        plan = self.plan
        new_pos, new_idx = self._advance(self.cmd_pos, self.path_idx,
                                         self.weld_speed / self.rate_hz,
                                         plan.tool0)
        if not self._solve_step(new_pos, allow_abort=True):
            self.advanced = False
            return
        with self.lock:
            for k in range(self.path_idx + 1, new_idx + 1):
                self.bead.append(plan.bead[k])
            self.progress = new_idx / max(len(plan.tool0) - 1, 1)
        self.cmd_pos, self.path_idx = new_pos, new_idx
        self.advanced = True
        if new_idx >= len(plan.tool0) - 1:
            self._finish_weld()

    def _move_tick(self, next_state, allow_abort):
        new_pos, new_idx = self._advance(self.cmd_pos, self.move_idx,
                                         self.travel_speed / self.rate_hz,
                                         self.move_seq)
        ok = self._solve_step(new_pos, allow_abort=allow_abort)
        if not ok:
            if not allow_abort and self.ik_fail_run >= self.ik_fail_abort:
                self.get_logger().warning("retract IK stuck — homing in "
                                          "joint space")
                self._set_state("HOMING")
            return
        self.cmd_pos, self.move_idx = new_pos, new_idx
        if (self.move_idx >= len(self.move_seq) - 1
                and math.dist(self.cmd_pos, self.move_seq[-1]) < 1e-4):
            if next_state == "WELD":
                with self.lock:
                    self.bead.append(self.plan.bead[0])
            self._set_state(next_state)

    def _tick(self):
        now = time.monotonic()
        with self.lock:
            req_start, req_abort = self.req_start, self.req_abort
            self.req_start = self.req_abort = None
            last_seen = self.last_seen
        st = self.state
        if (not self.synthetic and st in ("APPROACH", "WELD")
                and last_seen is not None
                and now - last_seen > self.hand_loss_abort):
            req_abort = req_abort or ("hand lost > {:.0f} s mid-weld"
                                      .format(self.hand_loss_abort))
        if req_abort and st in ("APPROACH", "WELD"):
            self._abort(req_abort)
        elif req_start:
            # wall_track mode leaves plan=None on every live re-pose — plan
            # + precheck lazily HERE, at fist time, for wherever the wall is
            # right now. Slow work (~150 ms) runs outside the lock; the
            # commit re-checks identity so a concurrent re-pose drops it.
            with self.lock:
                lazy_wall = (self.wall if self.plan is None
                             and self.state == "IDLE"
                             and self.respawn_hold is None else None)
            if lazy_wall is not None:
                ok, wall2, report = self.planner.precheck(lazy_wall)
                self.get_logger().info(report)
                new_plan = None
                if ok:
                    try:
                        new_plan = self.planner.plan(wall2)
                    except ValueError as exc:
                        self._event("WALL_PLAN_FAILED why=fist-lazy "
                                    "err='{}'".format(exc))
                else:
                    self._event("WALL_CAPTURE_REJECTED at-fist " + report)
                if new_plan is not None:
                    if wall2 is not lazy_wall:      # precheck shrunk it
                        self.wallmgr.spawn(wall2)
                    with self.lock:
                        if (self.wall is lazy_wall and self.state == "IDLE"
                                and self.respawn_hold is None):
                            self.wall, self.plan = wall2, new_plan
                        else:
                            new_plan = None        # wall moved meanwhile
                    if new_plan is not None:
                        self._event(
                            "PLAN waypoints={} rows={} path_m={:.2f} "
                            "why=fist-lazy".format(len(new_plan.tool0),
                                                   new_plan.n_rows,
                                                   new_plan.path_len))
            # decide AND transition under the lock so it is atomic against
            # _acquire_idle_hold (capture/reset): once a hold is set no weld
            # starts; once we start here, the hold acquisition fails.
            with self.lock:
                can_start = (self.state == "IDLE" and self.plan is not None
                             and self.respawn_hold is None)
                if can_start:
                    self._start_weld(req_start)
            if not can_start:
                self.get_logger().info(
                    "fist ignored (state {}, plan {}{})".format(
                        st, "ok" if self.plan else "missing",
                        ", wall respawn in flight" if self.respawn_hold
                        else ""),
                    throttle_duration_sec=2.0)
        st = self.state
        self.advanced = False
        if st == "INIT":
            return
        if st == "HOMING":
            if self._slew_joints(self.q_home):
                self._set_state("IDLE")
        elif st == "IDLE":
            pass                               # hold home
        elif st == "ABORT_FREEZE":
            if now - self.t_freeze >= self.freeze_sec:
                self._set_state("HOMING")
        elif st == "APPROACH":
            self._move_tick("WELD", allow_abort=True)
        elif st == "WELD":
            self._weld_tick()
        elif st == "RETRACT":
            self._move_tick("HOMING", allow_abort=False)
        self._publish_traj(self.q_cmd)
        with self.lock:
            prog = self.progress
        self.pub_progress.publish(Float32(data=float(prog)))
        self._publish_markers(now)

    # ------------------------------------------------------------- markers
    def _publish_markers(self, now):
        with self.lock:
            bead = list(self.bead)
            wall, prog = self.wall, self.progress
        st = self.state
        arr = MarkerArray()
        if wall is not None:
            outline = Marker()
            outline.header.frame_id = BASE_FRAME
            outline.ns, outline.id = "wall", 5
            outline.type, outline.action = Marker.LINE_STRIP, Marker.ADD
            outline.scale.x = 0.004
            outline.color.r = outline.color.g = 0.75
            outline.color.b = 0.8
            outline.color.a = 1.0
            for sy, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1)):
                fp = wall.face_point(sy * wall.w / 2, sz * wall.h / 2)
                outline.points.append(Point(x=fp[0], y=fp[1], z=fp[2]))
            arr.markers.append(outline)
            hud = Marker()
            hud.header.frame_id = BASE_FRAME
            hud.ns, hud.id = "hud", 4
            hud.type, hud.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            top = wall.face_point(0.0, wall.h / 2 + 0.05)
            hud.pose.position.x, hud.pose.position.y, hud.pose.position.z = top
            hud.pose.orientation.w = 1.0
            hud.scale.z = 0.03
            hud.color.r = hud.color.g = hud.color.b = hud.color.a = 1.0
            hud.text = "{} {:.0f}%".format(st, prog * 100)
            arr.markers.append(hud)
        if len(bead) >= 2:
            m = Marker()
            m.header.frame_id = BASE_FRAME
            m.ns, m.id = "bead", 1
            m.type, m.action = Marker.LINE_STRIP, Marker.ADD
            m.scale.x = 0.008
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.45, 0.1, 1.0
            m.points = [Point(x=b[0], y=b[1], z=b[2]) for b in bead]
            arr.markers.append(m)
        else:
            m = Marker()
            m.header.frame_id = BASE_FRAME
            m.ns, m.id = "bead", 1
            m.action = Marker.DELETE
            arr.markers.append(m)
        tool0 = arm_ik.fk_tool0(self.q_cmd) if self.q_cmd else None
        if tool0 is not None:
            if (st in ("APPROACH", "WELD", "RETRACT", "ABORT_FREEZE")
                    and self.plan is not None and self.cmd_pos is not None):
                off = self.plan.offset
                tip = (self.cmd_pos[0] - off[0], self.cmd_pos[1] - off[1],
                       self.cmd_pos[2] - off[2])
            else:
                tip = (tool0[0], tool0[1], tool0[2] - 0.12)
            torch = Marker()
            torch.header.frame_id = BASE_FRAME
            torch.ns, torch.id = "torch", 2
            torch.type, torch.action = Marker.LINE_STRIP, Marker.ADD
            torch.scale.x = 0.012
            torch.color.r, torch.color.g, torch.color.b = 0.15, 0.15, 0.18
            torch.color.a = 1.0
            torch.points = [Point(x=tool0[0], y=tool0[1], z=tool0[2]),
                            Point(x=tip[0], y=tip[1], z=tip[2])]
            arr.markers.append(torch)
            glow = Marker()
            glow.header.frame_id = BASE_FRAME
            glow.ns, glow.id = "torch", 3
            glow.type, glow.action = Marker.SPHERE, Marker.ADD
            glow.pose.position.x, glow.pose.position.y, \
                glow.pose.position.z = tip
            glow.pose.orientation.w = 1.0
            s = 0.028 if st == "WELD" else 0.012
            glow.scale.x = glow.scale.y = glow.scale.z = s
            glow.color.r = 1.0
            glow.color.g = 0.9 if st == "WELD" else 0.5
            glow.color.b = 0.7 if st == "WELD" else 0.15
            glow.color.a = 1.0
            arr.markers.append(glow)
            if st == "WELD" and self.advanced:
                sparks = Marker()
                sparks.header.frame_id = BASE_FRAME
                sparks.ns, sparks.id = "sparks", 0
                sparks.type, sparks.action = Marker.POINTS, Marker.ADD
                sparks.scale.x = sparks.scale.y = 0.005
                sparks.color.r, sparks.color.g, sparks.color.b = 1.0, 0.85, 0.2
                sparks.color.a = 1.0
                sparks.lifetime = Duration(sec=0, nanosec=120_000_000)
                for _ in range(10):
                    sparks.points.append(Point(
                        x=tip[0] + self.rng.f(-0.025, 0.025),
                        y=tip[1] + self.rng.f(-0.025, 0.025),
                        z=tip[2] + self.rng.f(-0.015, 0.04)))
                arr.markers.append(sparks)
        self.pub_markers.publish(arr)

    # ------------------------------------------------------------- health
    def _heartbeat(self):
        self.pub_state.publish(String(data=self.state))
        with self.lock:
            t_js, prog = self.js_time, self.progress
        if t_js is not None and time.monotonic() - t_js > 1.0:
            self.get_logger().warning(
                "/joint_states stale for {:.1f} s — controller down?"
                .format(time.monotonic() - t_js), throttle_duration_sec=5.0)
        self.get_logger().info(
            "[{}] progress {:.0f}% bead {} pts sent #{}".format(
                self.state, prog * 100, len(self.bead), self.n_pub),
            throttle_duration_sec=5.0)

    # ------------------------------------------------------------- shutdown
    def shutdown(self):
        self.running = False
        if getattr(self, "timer", None) is not None:
            self.timer.cancel()
        if self.q_cmd is not None and rclpy.ok():
            self._publish_traj(self.q_cmd)     # final hold
            self.get_logger().info("published final hold, shutting down")
        if self.recognizer is not None:
            self.recognizer.close()
        if self.capture is not None:
            self.capture.stop()
            cv2.destroyAllWindows()


def main():
    global cv2, mp, mp_python, mp_vision
    rclpy.init()
    node = WallWeld()
    if not node.synthetic:                     # lazy imports: synthetic mode
        import cv2                             # must run without camera stack
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision

    def _sigterm(*_):
        raise KeyboardInterrupt          # docker stop -> clean shutdown path
    signal.signal(signal.SIGTERM, _sigterm)
    try:
        if not node.preflight():
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        node.start()
        node._spinning = True
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
