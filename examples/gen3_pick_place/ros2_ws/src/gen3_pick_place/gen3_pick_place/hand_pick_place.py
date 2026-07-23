#!/opt/mpvenv/bin/python
"""hand_pick_place.py — hand-guided pick-and-place for the Kinova Gen3 lite (U6).

Ports the proven hand_follow.py skeleton (ThreadedCapture, One-Euro filter,
LEFT-hand filter, workspace map, slew/freeze/reseed, /follow_enable,
preflight, synthetic mode, preview window, latency probe) onto the gen3-lite
mock-hardware stack (pickplace_demo.launch.py), adding:

  * GestureRecognizer replacing HandLandmarker — ONE recognize_for_video call
    per frame returns gestures + handedness + 21 landmarks + world landmarks
    as PARALLEL ARRAYS.  The gesture is ALWAYS read at the SAME result index
    i as the accepted LEFT hand (gap 2) — enforced by select_left_hand(),
    the only place result arrays are indexed.
  * HAND-DERIVED gripper orientation (user decision) via ik_client's palm
    spec, with the gap-1 degradation chain oriented -> fixed-downward ->
    position-hold (IKClient.solve owns the chain; an orientation-INVALID
    frame or a stale palm quat degrades to DOWN_QUAT).
  * Gesture state machine (gesture_sm, U5): Closed_Fist = grip, Open_Palm =
    release with the landmark-openness + None-dwell release fallback (gap 4)
    and the transition position-latch (gap 3) — while the SM reports
    position_hold, the control loop tracks its back-dated frozen_target.
  * Proximity-gated attach (gap 5): a FIST far from the box only closes the
    gripper; the gate (within_attach on the commanded tool target vs the box
    world pose) is re-evaluated EVERY tick while fisted, so carrying the fist
    to the box attaches on arrival.  Attach FIRST, then pinch to GRIP_PINCH
    (gap 9) — an empty-fist close uses GRIP_CLOSE_EMPTY and re-opens to the
    pinch width when the attach later succeeds.
  * IK-failure policy (gap 6): MODE_FAILED holds the last q_cmd (published as
    a hold), warns throttled, and after ik_fail_freeze consecutive failures
    freezes target advance until a solve succeeds.  recovered=True solutions
    (recovery-seed, branch flip) freeze the NEXT tick's target advance while
    the per-tick joint clamp closes the gap (hand_follow branch-guard
    pattern, per U3's jump-guard semantics).
  * FK reseed / tracking err via /compute_fk (gap 12 — served by move_group
    on this bring-up; scene_box keeps a TF fallback for the box pose).
  * In-process SceneBoxManager (U4): spawns the 4 cm box at BOX_SPAWN, serves
    /box_reset (gap 11), attach/detach with world-pose bookkeeping.

TOPICS / SERVICES
  pub  /joint_trajectory_controller/joint_trajectory  single-point stream, 20 Hz
  pub  /gripper_controller/joint_trajectory           right_finger_bottom_joint
  sub  /joint_states                                  seed + preflight (gap 8:
       subset reconcile — 6 arm joints required, finger/mimic extras tolerated)
  srv  /follow_enable (std_srvs/SetBool)              pause/resume (resume
       re-seeds from /joint_states + /compute_fk — never teleports)
  srv  /box_reset (std_srvs/Trigger)                  via SceneBoxManager

EXECUTOR MODEL (matches ik_client's contract)
  The control loop is a MANUAL spin loop in the main thread: rclpy.spin_once
  for callbacks, then a 20 Hz tick that may block in IKClient.solve/fk
  (rclpy.spin_until_future_complete on this node — legal only because the
  node is never concurrently spun by another executor).  The /follow_enable
  callback therefore only sets a flag; the reseed happens in the loop.
  SceneBoxManager is a SECOND node on its own background MultiThreadedExecutor;
  all its blocking methods are called from the control thread only.

AXIS MAPPING (identical convention to hand_follow.py, remapped to the tuned
gen3-lite workspace box from ik_client):  frames are cv2.flip'd to a mirror
view; in the flipped image u -> base +Y, v(up) -> +Z, scale s (hand size,
wrist->middle-MCP over image height: depth proxy) -> +X.
  WORKSPACE_BOX: x 0.26..0.40  y -0.16..0.16  z 0.10..0.30 (m, base_link)
  HOME_TARGET (0.32, 0, 0.20); BOX_SPAWN (0.34, 0, 0.16); D_ATTACH 0.06.

RUN
  synthetic (no camera, scripted full pick-and-place — acceptance a):
    ros2 run gen3_pick_place hand_pick_place --ros-args -p synthetic:=true
  camera (mediapipe venv python so cv2/mediapipe resolve):
    /opt/mpvenv/bin/python -m gen3_pick_place.hand_pick_place [--ros-args ...]
"""
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gen3_pick_place.ik_client import (
    ARM_JOINTS, DOWN_QUAT, GRIPPER_JOINT, HOME_TARGET, IK_LINK,
    MODE_FAILED, WORKSPACE_BOX, IKClient, clamp_to_box, palm_orientation,
)
from gen3_pick_place.scene_box import (
    D_ATTACH, GRIP_OPEN, GRIP_PINCH, SPAWN_XYZ, SceneBoxManager, within_attach,
)
from gen3_pick_place.gesture_sm import (
    LABEL_FIST, LABEL_PALM, STATE_GRIPPING, STATE_RELEASED, GestureSM,
    extension_ratios_from_world_landmarks,
)

ARM_TRAJ_TOPIC = "/joint_trajectory_controller/joint_trajectory"
GRIP_TRAJ_TOPIC = "/gripper_controller/joint_trajectory"

# Empty-fist close (no box between the fingers): near full close of the
# 0.96 rad range.  With the box attached the target is GRIP_PINCH (0.50,
# gap 9 — visually pinches the 4 cm cube without passing through it).
GRIP_CLOSE_EMPTY = 0.80

WRIST, MID_MCP = 0, 9            # mediapipe hand landmark indices
PALM_QUAT_FRESH_S = 0.5          # stale palm quat -> degrade to fixed-down


# ---------------------------------------------------------------------------
# Pure logic (unit-checked by test/test_hand_pick_place.py — acceptance c)

def reconcile_joint_names(names) -> Optional[str]:
    """Gap 8 preflight reconcile: the 6 arm joints must ALL be present in
    /joint_states; extra finger/mimic joints are tolerated (the gen3-lite
    mock hardware publishes right_finger_bottom_joint and its mimics).
    Returns None when OK, else an error string."""
    missing = [j for j in ARM_JOINTS if j not in names]
    if missing:
        return ("/joint_states lacks arm joints {} (got {}) — wrong robot or "
                "controllers not up? Expected the gen3_pick_place "
                "pickplace_demo.launch.py stack.".format(missing, sorted(names)))
    return None


@dataclass
class FrameObs:
    """Everything read from ONE GestureRecognizer result index i (gap 2)."""
    index: int
    scale: float                 # hand-size depth proxy (aspect-corrected)
    u: float                     # wrist x in [0,1], flipped image
    v: float                     # wrist y in [0,1]
    handed_score: float
    gesture_label: Optional[str]
    gesture_score: float
    world_landmarks: Sequence[Any]   # hand_world_landmarks[i] (metric)


def select_left_hand(res, hand_conf: float, aspect: float = 4.0 / 3.0
                     ) -> Optional[FrameObs]:
    """Select the accepted LEFT hand and bundle ALL of its per-index data.

    This is the ONLY place the GestureRecognizer result arrays are indexed:
    gestures / handedness / hand_landmarks / hand_world_landmarks are
    parallel arrays and every field of the returned FrameObs comes from the
    SAME index i (gap 2).  Two accepted Lefts -> the larger (closer) wins,
    and the gesture follows the WINNER's index.  Right hands and
    low-confidence Lefts are ignored."""
    best: Optional[FrameObs] = None
    for i in range(len(res.handedness)):
        handed = res.handedness[i]
        if not handed:
            continue
        cat = handed[0]
        if cat.category_name != "Left" or cat.score < hand_conf:
            continue
        lms = res.hand_landmarks[i]
        du = (lms[MID_MCP].x - lms[WRIST].x) * aspect
        dv = (lms[MID_MCP].y - lms[WRIST].y)
        s = math.hypot(du, dv)
        if best is not None and s <= best.scale:
            continue
        glist = res.gestures[i] if i < len(res.gestures) else []
        top = glist[0] if glist else None
        wlm = (res.hand_world_landmarks[i]
               if i < len(res.hand_world_landmarks) else [])
        best = FrameObs(
            index=i, scale=s, u=lms[WRIST].x, v=lms[WRIST].y,
            handed_score=cat.score,
            gesture_label=top.category_name if top else None,
            gesture_score=top.score if top else 0.0,
            world_landmarks=wlm)
    return best


def map_to_workspace(u: float, v: float, s: float,
                     s_near: float, s_far: float) -> Tuple[float, float, float]:
    """Flipped-image (u, v, scale) -> base_link target inside WORKSPACE_BOX.
    u right -> +Y, v up -> +Z, s bigger (closer) -> +X (mirror behavior)."""
    u = min(max(u, 0.0), 1.0)
    v = min(max(v, 0.0), 1.0)
    k = min(max((s - s_far) / (s_near - s_far), 0.0), 1.0)
    (x0, x1), (y0, y1), (z0, z1) = (WORKSPACE_BOX["x"], WORKSPACE_BOX["y"],
                                    WORKSPACE_BOX["z"])
    return (x0 + k * (x1 - x0), y0 + u * (y1 - y0), z1 - v * (z1 - z0))


def gripper_plan(sm_state: str, attached: bool, near_box: bool
                 ) -> Tuple[str, float]:
    """Tick-level scene/gripper reconciliation -> (action, gripper_target).

    action in {'none', 'attach', 'detach'}.  Encodes gaps 5 + 9:
      * GRIPPING + not attached + near  -> ('attach', GRIP_PINCH): attach
        FIRST, then close to the pinch width (executor orders it that way).
      * GRIPPING + not attached + far   -> ('none', GRIP_CLOSE_EMPTY): a far
        fist closes the gripper but does NOT attach; because this runs every
        tick, carrying the fist to the box flips near_box and attaches then.
      * GRIPPING + attached             -> hold the pinch.
      * RELEASED + attached             -> ('detach', GRIP_OPEN): detach
        (box re-enters the world at its carried pose), then open.
      * RELEASED + not attached         -> stay open."""
    if sm_state == STATE_GRIPPING:
        if attached:
            return ("none", GRIP_PINCH)
        if near_box:
            return ("attach", GRIP_PINCH)
        return ("none", GRIP_CLOSE_EMPTY)
    if attached:
        return ("detach", GRIP_OPEN)
    return ("none", GRIP_OPEN)


# ---------------------------------------------------------------------------
# Synthetic scripted pick-and-place (acceptance a) — 30 Hz fake vision frames.
# Each row: (t_end, xyz_at_end, label, score, extension_ratios); position
# lerps from the previous row's xyz across the segment.  All waypoints are
# inside WORKSPACE_BOX (unit-checked); the drop pose is >= 0.15 m from
# BOX_SPAWN so the recorded detach satisfies the acceptance distance.

NEUTRAL_RATIOS = (1.4, 1.4, 1.4, 1.4)   # dead band — keeps prior finger state
CURLED_RATIOS = (1.0, 1.0, 1.0, 1.0)
OPEN_RATIOS = (1.8, 1.8, 1.8, 1.8)
SYNTH_DROP = (0.29, -0.14, 0.26)         # 0.179 m from BOX_SPAWN

SYNTH_SCRIPT = [
    # t_end  xyz               label       score  ratios
    (4.0,  SPAWN_XYZ,          None,       0.0,   NEUTRAL_RATIOS),  # approach
    (5.0,  SPAWN_XYZ,          None,       0.0,   NEUTRAL_RATIOS),  # dwell
    (6.0,  SPAWN_XYZ,          LABEL_FIST, 0.9,   CURLED_RATIOS),   # grip+attach
    (9.0,  (0.36, 0.12, 0.24), LABEL_FIST, 0.9,   CURLED_RATIOS),   # carry 1
    (13.0, SYNTH_DROP,         LABEL_FIST, 0.9,   CURLED_RATIOS),   # carry 2
    (14.0, SYNTH_DROP,         LABEL_FIST, 0.9,   CURLED_RATIOS),   # settle
    (15.5, SYNTH_DROP,         LABEL_PALM, 0.9,   OPEN_RATIOS),     # release
    (18.0, SYNTH_DROP,         None,       0.0,   NEUTRAL_RATIOS),  # hold
]


def synth_frame(t: float, script=SYNTH_SCRIPT):
    """Scripted (xyz, label, score, ratios) at time t since script start."""
    prev_t, prev_xyz = 0.0, HOME_TARGET
    for (t_end, xyz, label, score, ratios) in script:
        if t <= t_end:
            f = 1.0 if t_end <= prev_t else (t - prev_t) / (t_end - prev_t)
            cur = tuple(a + f * (b - a) for a, b in zip(prev_xyz, xyz))
            return cur, label, score, ratios
        prev_t, prev_xyz = t_end, xyz
    return script[-1][1], None, 0.0, NEUTRAL_RATIOS


# ---------------------------------------------------------------------------
# Ported infrastructure (hand_follow.py)

class OneEuro:
    """One-Euro filter, Casiez et al. CHI 2012. Timestamps in seconds."""

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


class ThreadedCapture:
    """Camera reader on its own thread (latest-frame only) so the ~40 ms
    v4l2 read overlaps mediapipe inference."""

    def __init__(self, device):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                "camera {} failed to open — container started without "
                "--device /dev/video0?".format(device))
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

class HandPickPlace(Node):
    def __init__(self):
        super().__init__("hand_pick_place")
        p = self.declare_parameter
        self.synthetic = p("synthetic", False).value
        self.camera = p("camera", "/dev/video0").value
        self.model_path = p("model_path",
                            "/opt/models/gesture_recognizer.task").value
        self.rate_hz = p("rate_hz", 20.0).value
        self.tfs = p("time_from_start", 0.12).value
        self.min_cutoff = p("min_cutoff", 1.0).value
        self.beta = p("beta", 0.5).value
        self.max_step = p("max_step_m", 0.03).value          # slew, m/tick
        self.decay_step = p("decay_step_m", 0.005).value     # home decay
        self.loss_hold = p("loss_hold_sec", 2.0).value
        self.max_dq = p("max_joint_step_rad", 0.10).value    # 2 rad/s at 20 Hz
        self.hand_conf = p("hand_conf", 0.6).value
        self.acq_frames = p("acq_frames", 3).value   # acquisition debounce:
        # N consecutive accepted frames before tracking (re)commits — a
        # single-frame ghost detection must never command the arm (verified
        # need: camera smoke saw a 1-frame scale-0.035 'Left' in a dark
        # scene that would otherwise start a track->loss->homing cycle).
        self.s_near = p("s_near", 0.30).value
        self.s_far = p("s_far", 0.10).value
        self.max_jump = p("max_jump_rad", 0.5).value         # branch-flip guard
        self.n_fail_freeze = p("ik_fail_freeze", 5).value    # gap 6 N
        self.latency_probe = p("latency_probe", False).value
        self.preview = p("preview", False).value

        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TRAJ_TOPIC, 10)
        self.grip_pub = self.create_publisher(JointTrajectory, GRIP_TRAJ_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_service(SetBool, "/follow_enable", self._on_enable)

        self.ik = IKClient(self)
        self.sm = GestureSM()                  # vision/synth thread only
        self.mgr: Optional[SceneBoxManager] = None   # set by main()

        self.lock = threading.Lock()
        # shared with vision/synth thread (under self.lock)
        self.js_msg = None
        self.js_time = None
        self.enabled = True
        self.reseed_pending = False            # /follow_enable resume flag
        self.hand_target = None                # filtered mapped target
        self.last_seen = None
        self.palm_quat = None                  # latest VALID palm quat
        self.palm_quat_t = -1e9
        self.sm_state = STATE_RELEASED
        self.position_hold = False
        self.frozen_target = None

        # control-thread-only state
        self.running = True
        self.cmd_target = None                 # slewed target sent to IK
        self.q_cmd = None                      # last commanded arm joints
        self.grip_cmd = None                   # last commanded gripper target
        self.attached = False
        self.box_xyz = tuple(SPAWN_XYZ)
        self.q_home = None                     # startup joint-space homing
        self.startup_homing = False
        self.track_state = "init"
        self.ik_frozen = False                 # recovered-solution guard
        self.ik_mode_last = None
        self.n_pub = 0
        self.tick_count = 0
        self.last_attach_try = -1e9
        self.capture = None
        self.recognizer = None
        self.t0 = time.monotonic()
        self._t_status = 0.0
        self._t_health = 0.0
        self._t_latrep = 0.0

        # latency probe: rolling windows of per-stage timings in ms
        self.lat_capture = deque(maxlen=150)
        self.lat_infer = deque(maxlen=150)
        self.lat_ik = deque(maxlen=150)
        self.lat_publish = deque(maxlen=150)

    # ------------------------------------------------------------- callbacks
    def _on_js(self, msg):
        with self.lock:
            self.js_msg = msg
            self.js_time = time.monotonic()

    def _on_enable(self, req, resp):
        """Only flips flags — the reseed itself runs in the control loop
        (a service callback must never nest-spin this node)."""
        with self.lock:
            was, self.enabled = self.enabled, req.data
            if req.data and not was:
                self.reseed_pending = True
        if req.data and not was:
            self.get_logger().info("follow ENABLED (reseed queued)")
        elif not req.data and was:
            self.get_logger().info("follow DISABLED — arm holds, stream paused")
        resp.success = True
        resp.message = "following enabled" if req.data else "following disabled"
        return resp

    # ------------------------------------------------------------- reseed
    def _reseed(self):
        """Re-seed q_cmd from /joint_states and cmd_target from /compute_fk
        (gap 12) so tracking resumes without a teleport."""
        with self.lock:
            msg = self.js_msg
        if msg is None or not self.ik.seed_from_joint_state(msg):
            return False
        q = list(self.ik.seed)
        fk = self.ik.fk(q)
        if not fk or IK_LINK not in fk:
            return False
        self.q_cmd = q
        self.cmd_target = list(fk[IK_LINK][0])
        return True

    # ------------------------------------------------------------- preflight
    def preflight(self):
        log = self.get_logger()
        if not self.synthetic:
            if not os.path.isfile(self.model_path):
                log.fatal("model file missing: {} — the ros2-arm:jazzy image "
                          "bakes /opt/models/gesture_recognizer.task"
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
            log.fatal("no /joint_states within 12 s — start the stack first: "
                      "ros2 launch gen3_pick_place pickplace_demo.launch.py")
            return False
        err = reconcile_joint_names(list(self.js_msg.name))
        if err:
            log.fatal(err)
            return False
        extras = [n for n in self.js_msg.name if n not in ARM_JOINTS]
        if GRIPPER_JOINT not in self.js_msg.name:
            log.warning("{} not in /joint_states — gripper feedback missing "
                        "(mock hardware should publish it)".format(GRIPPER_JOINT))
        log.info("joint reconcile OK: 6 arm joints present, extras tolerated: "
                 "{}".format(extras))
        if not self.ik.wait_for_services(20.0):
            log.fatal("/compute_ik + /compute_fk unavailable — move_group up?")
            return False
        for attempt in range(5):
            if self._reseed():
                break
            rclpy.spin_once(self, timeout_sec=0.3)
        else:
            log.fatal("FK reseed failed 5x — /compute_fk unhealthy")
            return False
        # STARTUP JOINT-SPACE HOMING: the arm may start OUTSIDE the tuned
        # workspace/top-down envelope (mock hardware boots at all-zeros =
        # straight up, tool z ~1.0 m), where task-space fixed-down IK
        # legitimately fails and the gap-6 freeze would pin the target
        # there.  Solve IK ONCE for the proven HOME_TARGET (recovery ladder
        # reaches DEFAULT_SEED) and slew joints there BEFORE task-space
        # tracking; cmd_target then starts inside the validated box.
        res = self.ik.solve(HOME_TARGET, quat=DOWN_QUAT)
        if res.mode == MODE_FAILED or res.joints is None:
            log.fatal("startup homing IK for HOME_TARGET {} failed"
                      .format(HOME_TARGET))
            return False
        self.q_home = list(res.joints)
        self.startup_homing = True
        log.info("startup homing: slewing joints to home config {} "
                 "(HOME_TARGET {})".format(
                     [round(v, 3) for v in self.q_home], HOME_TARGET))
        # scene: wait for the manager's box spawn; recover a stale ACO from a
        # previous run (reset re-homes it), then cache the box world pose.
        if self.mgr is not None:
            if not self.mgr._aps.wait_for_service(timeout_sec=20.0):
                log.fatal("/apply_planning_scene unavailable")
                return False
            box_deadline = time.monotonic() + 8.0
            world = att = None
            while time.monotonic() < box_deadline:
                world, att = self.mgr.scene_state(timeout=3.0)
                if att is not None:
                    log.warning("stale attached box from a previous run — "
                                "resetting")
                    self.mgr.reset_box()
                    continue
                if world is not None:
                    break
                time.sleep(0.3)
            if world is None:
                if not self.mgr.spawn_box():
                    log.fatal("box spawn failed")
                    return False
                world, _ = self.mgr.scene_state()
            if world is not None:
                self.box_xyz = self.mgr.world_xyz(world)
        if self.arm_pub.get_subscription_count() == 0:
            log.warning("{} has no subscriber yet — controller not up? "
                        "streaming anyway".format(ARM_TRAJ_TOPIC))
        self._grip_publish(GRIP_OPEN)          # known gripper state
        log.info("preflight OK: seed q={} tool=({:.3f},{:.3f},{:.3f}) box={} "
                 "D_ATTACH={}".format([round(v, 3) for v in self.q_cmd],
                                      *self.cmd_target, self.box_xyz, D_ATTACH))
        return True

    # ------------------------------------------------------------- vision
    def start_vision(self):
        if self.synthetic:
            self.get_logger().info(
                "SYNTHETIC mode: scripted pick-and-place, box x{} y{} z{}, "
                "spawn {}".format(WORKSPACE_BOX["x"], WORKSPACE_BOX["y"],
                                  WORKSPACE_BOX["z"], SPAWN_XYZ))
            self.vision_thread = threading.Thread(target=self._synth_loop,
                                                  daemon=True, name="synth")
        else:
            base = mp_python.BaseOptions(model_asset_path=self.model_path)
            self.recognizer = mp_vision.GestureRecognizer.create_from_options(
                mp_vision.GestureRecognizerOptions(
                    base_options=base,
                    running_mode=mp_vision.RunningMode.VIDEO,
                    num_hands=2,
                    min_hand_detection_confidence=0.5,
                    min_tracking_confidence=0.5))
            self.vision_thread = threading.Thread(target=self._vision_loop,
                                                  daemon=True, name="vision")
            self.get_logger().info(
                "LEFT-hand pick-place: conf>={}, fist=grip palm=release, "
                "box x{} y{} z{}".format(self.hand_conf, WORKSPACE_BOX["x"],
                                         WORKSPACE_BOX["y"], WORKSPACE_BOX["z"]))
        self.filters = [OneEuro(self.min_cutoff, self.beta) for _ in range(3)]
        self.vision_thread.start()

    def _sm_share(self, out, now):
        """Publish the SM snapshot to the control thread (caller holds lock)."""
        self.sm_state = out.state
        self.position_hold = out.position_hold
        self.frozen_target = out.frozen_target
        return out

    def _synth_loop(self):
        """30 Hz scripted fake-vision frames through the IDENTICAL
        SM -> shared-state -> control path as the camera pipeline."""
        t0 = time.monotonic()
        log = self.get_logger()
        while self.running and rclpy.ok():
            now = time.monotonic()
            xyz, label, score, ratios = synth_frame(now - t0)
            out = self.sm.update(now, label, score, ratios, tuple(xyz))
            with self.lock:
                self.hand_target = tuple(xyz)
                self.last_seen = now
                self.palm_quat = DOWN_QUAT     # scripted valid orientation
                self.palm_quat_t = now
                self._sm_share(out, now)
            if out.event:
                log.info("GESTURE EVENT: {} (t={:.2f}s script)"
                         .format(out.event, now - t0))
            time.sleep(1.0 / 30.0)

    def _vision_loop(self):
        seq, ts_prev = 0, -1
        acq_run = 0                    # consecutive accepted-obs frames
        log = self.get_logger()
        while self.running and rclpy.ok():
            got = self.capture.get(seq)
            if got is None:
                continue
            seq, frame, t_frame = got
            try:
                if self.latency_probe:
                    self.lat_capture.append((time.monotonic() - t_frame) * 1e3)
                frame = cv2.flip(frame, 1)      # mirror view
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w = rgb.shape[:2]
                ts = max(int((t_frame - self.t0) * 1000), ts_prev + 1)
                ts_prev = ts
                t_inf = time.perf_counter()
                res = self.recognizer.recognize_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)
                if self.latency_probe:
                    self.lat_infer.append((time.perf_counter() - t_inf) * 1e3)
                obs = select_left_hand(res, self.hand_conf, aspect=w / h)
                now = time.monotonic()
                if obs is None:
                    acq_run = 0
                    out = self.sm.update(now, None, 0.0, None, None)
                    with self.lock:
                        self._sm_share(out, now)
                    if res.handedness:
                        log.info(
                            "hand(s) seen but none accepted (need Left >= "
                            "{:.2f}): {}".format(self.hand_conf, ", ".join(
                                "{} {:.2f}".format(hd[0].category_name,
                                                   hd[0].score)
                                for hd in res.handedness)),
                            throttle_duration_sec=2.0)
                    if self.preview:
                        self._show_preview(frame, res, None, None)
                    continue
                acq_run += 1
                with self.lock:
                    gap = (None if self.last_seen is None
                           else now - self.last_seen)
                if (gap is None or gap > 0.5) and acq_run < self.acq_frames:
                    # acquisition debounce: not yet committed — this frame
                    # is treated as untracked (ghost detections never move
                    # the arm; a real hand commits ~100 ms later)
                    out = self.sm.update(now, None, 0.0, None, None)
                    with self.lock:
                        self._sm_share(out, now)
                    if self.preview:
                        self._show_preview(frame, res, None, out)
                    continue
                raw = map_to_workspace(obs.u, obs.v, obs.scale,
                                       self.s_near, self.s_far)
                if gap is None or gap > 0.5:
                    for f in self.filters:
                        f.reset()
                    log.info("hand ACQUIRED (scale {:.3f}) -> target "
                             "({:.3f},{:.3f},{:.3f}) — slewing"
                             .format(obs.scale, *raw))
                filt = tuple(f(x, t_frame)
                             for f, x in zip(self.filters, raw))
                ratios = None
                if len(obs.world_landmarks) >= 21:
                    ratios = extension_ratios_from_world_landmarks(
                        obs.world_landmarks)
                # palm orientation from the SAME index's world landmarks
                po = palm_orientation(obs.world_landmarks)
                out = self.sm.update(now, obs.gesture_label,
                                     obs.gesture_score, ratios, filt)
                with self.lock:
                    self.hand_target = filt
                    self.last_seen = now
                    if po.valid:
                        self.palm_quat = po.quat
                        self.palm_quat_t = now
                    self._sm_share(out, now)
                if out.event:
                    log.info("GESTURE EVENT: {} (label {} {:.2f}, openness "
                             "{})".format(out.event, obs.gesture_label,
                                          obs.gesture_score, out.openness))
                if self.preview:
                    self._show_preview(frame, res, obs, out, po.valid)
            except Exception as exc:  # noqa: BLE001 — vision must not die
                log.error("vision frame error: {}".format(exc),
                          throttle_duration_sec=2.0)

    # ------------------------------------------------------------- preview
    def _show_preview(self, frame, res, obs, out, palm_ok=False):
        h, w = frame.shape[:2]
        for i in range(len(res.handedness)):
            cat = res.handedness[i][0]
            lms = res.hand_landmarks[i]
            sel = obs is not None and i == obs.index
            color = ((0, 220, 0) if sel else
                     (0, 200, 255) if cat.category_name == "Left"
                     else (80, 80, 255))
            for lm in lms:
                cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 3, color, -1)
            wx, wy = int(lms[WRIST].x * w), int(lms[WRIST].y * h)
            glist = res.gestures[i] if i < len(res.gestures) else []
            gtag = ("{} {:.2f}".format(glist[0].category_name, glist[0].score)
                    if glist else "-")
            cv2.putText(frame, "{} {:.2f} | {}".format(
                cat.category_name, cat.score, gtag),
                (max(wx - 60, 2), max(wy - 12, 14)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        state = out.state if out else self.sm_state
        status = ("TRACKING scale {:.2f}".format(obs.scale) if obs else
                  "{} hand(s), none accepted".format(len(res.handedness))
                  if res.handedness else "no hand — raise your LEFT hand")
        bar = "[{}] {} | {}{} | orient:{}".format(
            self.track_state, status, state,
            "+ATTACHED" if self.attached else "",
            "palm" if palm_ok else "down")
        cv2.rectangle(frame, (0, 0), (w, 26), (30, 30, 30), -1)
        cv2.putText(frame, bar, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 220, 0) if obs else (0, 200, 255), 1)
        cv2.imshow("hand_pick_place — webcam (q to close)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.preview = False
            cv2.destroyAllWindows()

    # ------------------------------------------------------------- control
    def _desired(self, now):
        """(target, step_limit, state) for this tick.  Loss during a grip
        NEVER releases or homes (hold); homing only when RELEASED."""
        with self.lock:
            hold = self.position_hold
            frozen = self.frozen_target
            tgt = frozen if (hold and frozen is not None) else self.hand_target
            seen = self.last_seen
            smst = self.sm_state
        if seen is None:
            return None, 0.0, "init"           # never saw a hand: do nothing
        gripping = smst == STATE_GRIPPING or self.attached
        lost_for = now - seen
        if tgt is not None and (lost_for <= self.loss_hold or gripping):
            state = ("latched" if hold else
                     "carrying" if gripping else "tracking")
            return tgt, self.max_step, state
        if gripping:
            return None, 0.0, "carry-hold"     # tracked-loss during carry
        if lost_for <= self.loss_hold:
            return None, 0.0, "hold"
        return HOME_TARGET, self.decay_step, "homing"

    def tick(self, now):
        with self.lock:
            enabled = self.enabled
            reseed = self.reseed_pending
            self.reseed_pending = False
        if reseed:
            ok = self._reseed()
            self.get_logger().info("reseed from /joint_states+FK: {}".format(ok))
        if not enabled:
            return                             # frozen — arm truly holds
        self.tick_count += 1
        if self.startup_homing:
            q = [p + max(-self.max_dq, min(self.max_dq, h - p))
                 for p, h in zip(self.q_cmd, self.q_home)]
            self.q_cmd = q
            self._publish_arm(q, self.tfs)
            if max(abs(p - h) for p, h in zip(q, self.q_home)) < 0.02:
                self.startup_homing = False
                self.cmd_target = list(HOME_TARGET)
                self.ik.seed = list(self.q_home)
                self.get_logger().info(
                    "startup homing done — task-space tracking from "
                    "HOME_TARGET {}".format(HOME_TARGET))
            self._housekeeping(now)
            return
        des, step_lim, state = self._desired(now)
        if state != self.track_state:
            self.get_logger().info("state {} -> {}".format(self.track_state,
                                                           state))
            self.track_state = state
        if state == "init":
            self._housekeeping(now)
            return                             # no target yet: no IK, no cmd
        if self.cmd_target is None or self.q_cmd is None:
            return

        # target slew (freeze on recovery catch-up or persistent IK failure)
        freeze = self.ik_frozen or (self.ik.consecutive_failures
                                    >= self.n_fail_freeze)
        if des is not None and not freeze:
            des = clamp_to_box(des)
            dx = [d - c for d, c in zip(des, self.cmd_target)]
            n = math.sqrt(sum(v * v for v in dx))
            if n > step_lim:
                dx = [v * step_lim / n for v in dx]
            self.cmd_target = [c + v for c, v in zip(self.cmd_target, dx)]

        # orientation: fresh palm quat -> oriented stage; stale/invalid ->
        # solve() runs fixed-down only (gap 1 chain, graceful degradation)
        with self.lock:
            pq, pt = self.palm_quat, self.palm_quat_t
        quat = pq if (pq is not None and now - pt < PALM_QUAT_FRESH_S) else None

        res = self.ik.solve(self.cmd_target, quat=quat,
                            max_jump_rad=self.max_jump)
        if self.latency_probe:
            self.lat_ik.append(res.rt_ms_total)
        if res.mode == MODE_FAILED:
            self.get_logger().warning(
                "IK FAILED at ({:.3f},{:.3f},{:.3f}) — holding q_cmd "
                "({} consecutive{})".format(
                    *self.cmd_target, self.ik.consecutive_failures,
                    ", target advance FROZEN"
                    if self.ik.consecutive_failures >= self.n_fail_freeze
                    else ""),
                throttle_duration_sec=2.0)
            self._publish_arm(self.q_cmd, self.tfs)   # explicit hold
        else:
            if res.recovered:
                self.get_logger().warning(
                    "IK recovered via retry ladder (attempt {}) — pausing "
                    "target slew while joints catch up".format(
                        res.winning_attempt), throttle_duration_sec=2.0)
            self.ik_frozen = res.recovered
            if res.mode != self.ik_mode_last:
                self.get_logger().info("IK mode -> {}{}".format(
                    res.mode, "" if res.orientation_valid
                    else " (orientation invalid/stale)"))
                self.ik_mode_last = res.mode
            q = [p + max(-self.max_dq, min(self.max_dq, v - p))
                 for p, v in zip(self.q_cmd, res.joints)]
            self.q_cmd = q
            self._publish_arm(q, self.tfs)
            self.n_pub += 1

        self._scene_reconcile(now)
        self._housekeeping(now)

    # ------------------------------------------------------------- scene
    def _scene_reconcile(self, now):
        """Gap 5/9/11 executor: evaluate the proximity gate EVERY tick while
        fisted; attach-then-pinch; detach-then-open; slow-poll the scene."""
        with self.lock:
            smst = self.sm_state
        near = within_attach(self.cmd_target, self.box_xyz)
        action, gtar = gripper_plan(smst, self.attached, near)
        # F2: attach/detach run IN the 20 Hz control thread, so every scene
        # call gets a small TOTAL budget (2 s worst-case stall, vs the old
        # 10 s-per-call defaults that could freeze streaming 10-35 s if
        # move_group wedged).  A timed-out attach is retried by the gate on
        # a later tick; a timed-out detach is resynced by the slow poll.
        if action == "attach" and self.mgr is not None:
            if now - self.last_attach_try >= 0.5:
                self.last_attach_try = now
                ok, detail = self.mgr.attach_box(timeout=2.0)
                if ok:
                    self.attached = True
                    self.get_logger().info(
                        "ATTACH at d={:.3f} m (gate {:.2f}): {}".format(
                            math.dist(self.cmd_target, self.box_xyz),
                            D_ATTACH, detail))
                else:
                    self.get_logger().warning(
                        "attach failed: {} — retrying".format(detail),
                        throttle_duration_sec=2.0)
                    gtar = GRIP_CLOSE_EMPTY     # keep fist; no pinch w/o attach
            else:
                gtar = self.grip_cmd if self.grip_cmd is not None \
                    else GRIP_CLOSE_EMPTY
        elif action == "detach" and self.mgr is not None:
            ok, xyz, detail = self.mgr.detach_box(timeout=2.0)
            self.attached = False
            if ok and xyz is not None:
                self.box_xyz = tuple(xyz)
                self.get_logger().info("DETACH -> box back in world at "
                                       "({:.3f},{:.3f},{:.3f})".format(*xyz))
            else:
                self.get_logger().warning("detach: {}".format(detail))
        self._grip_publish(gtar)
        # slow poll (0.75 s): box world pose + attach flag (also resyncs
        # after an external /box_reset during a carry)
        if self.tick_count % 15 == 0 and self.mgr is not None:
            world, att = self.mgr.scene_state(timeout=1.0)
            if att is None and world is not None:
                self.box_xyz = tuple(self.mgr.world_xyz(world))
                if self.attached:
                    self.get_logger().warning("scene says box no longer "
                                              "attached — resyncing")
                self.attached = False
            elif att is not None:
                self.attached = True

    def _grip_publish(self, target):
        if target is None:
            return
        if self.grip_cmd is not None and abs(target - self.grip_cmd) < 1e-6:
            return
        msg = JointTrajectory()
        msg.joint_names = [GRIPPER_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions = [float(target)]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(0.8 * 1e9)
        msg.points.append(pt)
        self.grip_pub.publish(msg)
        self.grip_cmd = target
        self.get_logger().info("gripper -> {:.2f} rad".format(target))

    def _publish_arm(self, q, tfs):
        t0 = time.perf_counter() if self.latency_probe else None
        msg = JointTrajectory()
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start.sec = int(tfs)
        pt.time_from_start.nanosec = int((tfs - int(tfs)) * 1e9)
        msg.points.append(pt)
        self.arm_pub.publish(msg)
        if self.latency_probe:
            self.lat_publish.append((time.perf_counter() - t0) * 1e3)

    # ------------------------------------------------------------- periodic
    @staticmethod
    def _median(dq):
        if not dq:
            return None
        s = sorted(dq)
        return s[len(s) // 2]

    def _housekeeping(self, now):
        if now - self._t_health >= 1.0:
            self._t_health = now
            with self.lock:
                t_js = self.js_time
            if t_js is not None and now - t_js > 1.0:
                self.get_logger().warning(
                    "/joint_states stale for {:.1f} s — controller down?"
                    .format(now - t_js), throttle_duration_sec=5.0)
        if now - self._t_status >= 5.0:
            self._t_status = now
            err_mm = float("nan")
            if self.q_cmd is not None and self.cmd_target is not None:
                fk = self.ik.fk(self.q_cmd)          # gap 12: err via FK
                if fk and IK_LINK in fk:
                    err_mm = math.dist(fk[IK_LINK][0], self.cmd_target) * 1e3
            with self.lock:
                smst = self.sm_state
            st = self.ik.stats
            self.get_logger().info(
                "[{}] sm={} attached={} box=({:.2f},{:.2f},{:.2f}) "
                "tgt={} err {:.1f} mm ik(o/f/x/inv)={}/{}/{}/{} sent #{}"
                .format(self.track_state, smst, self.attached, *self.box_xyz,
                        ["{:.3f}".format(v) for v in self.cmd_target]
                        if self.cmd_target else None,
                        err_mm, st["oriented"], st["fixed_down"], st["failed"],
                        st["orientation_invalid"], self.n_pub))
        if self.latency_probe and now - self._t_latrep >= 3.0:
            self._t_latrep = now
            c, i = self._median(self.lat_capture), self._median(self.lat_infer)
            k, u = self._median(self.lat_ik), self._median(self.lat_publish)
            fmt = lambda v: "{:.1f}ms".format(v) if v is not None else "n/a"
            pipe = ("{:.1f}ms".format(c + i + k) if None not in (c, i, k)
                    else "n/a")
            self.get_logger().info(
                "LATENCY (median, n_ik={}) capture={} infer={} ik={} "
                "publish={} -- pipeline={}".format(
                    len(self.lat_ik), fmt(c), fmt(i), fmt(k), fmt(u), pipe))

    # ------------------------------------------------------------- main loop
    def control_loop(self):
        """Manual spin loop (see EXECUTOR MODEL in the module docstring)."""
        period = 1.0 / self.rate_hz
        next_t = time.monotonic()
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.monotonic()
            if now >= next_t:
                self.tick(now)
                next_t += period
                if next_t < now:               # fell behind (slow IK tick)
                    next_t = now + period

    # ------------------------------------------------------------- shutdown
    def shutdown(self):
        self.running = False
        if self.q_cmd is not None and rclpy.ok():
            try:                       # SIGINT may invalidate the context
                self._publish_arm(self.q_cmd, 0.3)   # between ok() and here
                self.get_logger().info("published final hold, shutting down")
            except Exception:  # noqa: BLE001 — best-effort final hold
                pass
        if self.recognizer is not None:
            self.recognizer.close()
        if self.capture is not None:
            self.capture.stop()
            cv2.destroyAllWindows()


def main(args=None):
    global cv2, mp, mp_python, mp_vision
    rclpy.init(args=args)
    node = HandPickPlace()
    if not node.synthetic:                     # lazy: synthetic runs without
        import cv2                             # the camera stack
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision
    mgr = SceneBoxManager(node_name="pickplace_scene", spawn_on_start=True)
    node.mgr = mgr
    mgr_exec = MultiThreadedExecutor(num_threads=2)
    mgr_exec.add_node(mgr)
    mgr_thread = threading.Thread(target=mgr_exec.spin, daemon=True,
                                  name="scene_exec")
    mgr_thread.start()

    def _sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)
    try:
        if not node.preflight():
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        node.start_vision()
        node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        mgr_exec.shutdown(timeout_sec=2.0)
        node.destroy_node()
        mgr.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
