"""mirror_node — drives the FFW semi-humanoid's arms, head and lift.

Three modes, selected by parameter:
  synthetic:=true   scripted whole-body sweep, no camera, mediapipe never
                    imported (the vision imports are lazy, in _make_tracker).
  track_only:=true  vision runs and publishes TF/markers; the robot is PARKED.
  (default)         LIVE MIRRORING: your arms and head drive the robot.

ARCHITECTURE NOTE — input rate and command rate are DECOUPLED. hand_follow
ticks at 20 Hz because that is what MediaPipe sustains. Here vision runs on its
own thread (~30 Hz measured) while the control timer runs at 50 Hz and
One-Euro-interpolates toward the latest observation, so motion stays smooth
between frames instead of stepping at the camera rate.

RUNNING UNDER THE RIGHT PYTHON MATTERS: mediapipe lives in /opt/mpvenv, not in
the system python that ament console-scripts are shebanged to. mirror.launch.py
sets prefix=/opt/mpvenv/bin/python. Without it, tracking dies with
ModuleNotFoundError *while synthetic mode keeps working* — which reads as a
camera fault. _make_tracker() catches that and says so.
"""

import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from humanoid_mirror.ffw_config import (
    ARM_L_JOINTS,
    ARM_R_JOINTS,
    HEAD_JOINTS,
    LIFT_JOINTS,
)
from humanoid_mirror.joint_limits import (
    OneEuro,
    clamp,
    max_step_for,
    resolve_limits,
    slew,
)

# Controller topic <- joint list. Disjoint by construction; ffw_check asserts it.
GROUPS = {
    "/arm_l_controller/joint_trajectory": ARM_L_JOINTS,
    "/arm_r_controller/joint_trajectory": ARM_R_JOINTS,
    "/head_controller/joint_trajectory": HEAD_JOINTS,
    "/lift_controller/joint_trajectory": LIFT_JOINTS,
}

# Pose the robot decays to when tracking is lost. All zeros is a safe,
# collision-free "arms down, looking straight ahead" for FFW; lift stays where
# it is rather than dropping.
HOME = {j: 0.0 for j in ARM_L_JOINTS + ARM_R_JOINTS + HEAD_JOINTS}

# TF frames published per tracked body landmark: index -> child frame id.
TF_LANDMARKS = {
    11: "human/l_shoulder",
    13: "human/l_elbow",
    15: "human/l_wrist",
    12: "human/r_shoulder",
    14: "human/r_elbow",
    16: "human/r_wrist",
    0: "human/head",
}


class MirrorNode(Node):
    def __init__(self):
        super().__init__("mirror_node")

        p = self.declare_parameter
        p("synthetic", True)
        p("rate_hz", 50.0)
        p("time_from_start", 0.06)
        # 2.0 rad/s at 50 Hz, comfortably under FFW's 4.8 rad/s URDF limit.
        # Sized from rate, NOT copied from hand_follow — see joint_limits.slew.
        p("max_joint_speed", 2.0)
        p("limit_margin", 0.05)
        p("min_cutoff", 1.0)
        p("beta", 0.5)
        p("head_min_cutoff", 0.5)
        p("use_lift", True)
        p("loss_hold_sec", 2.0)
        p("decay_speed", 0.5)
        p("sweep_period_s", 14.0)
        p("latency_probe", False)
        # --- M3: body tracking ---
        p("track_only", False)      # run vision + TF/markers, robot stays PARKED
        p("camera", "/dev/video0")
        p("pose_model_path", "/opt/models/pose_landmarker_full.task")
        p("min_visibility", 0.5)
        p("preview", False)
        p("body_frame_id", "camera_link")
        # --- M4: live mirroring ---
        p("mirror_mode", "mirror")   # "mirror" (facing you) or "direct"
        p("head_gain_yaw", 0.33)     # human yaw +-60 deg -> robot +-20 deg
        p("head_gain_pitch", 0.6)
        p("max_retarget_err_deg", 25.0)

        g = self.get_parameter
        self.synthetic = g("synthetic").value
        self.rate_hz = float(g("rate_hz").value)
        self.time_from_start = float(g("time_from_start").value)
        self.margin = float(g("limit_margin").value)
        self.use_lift = bool(g("use_lift").value)
        self.loss_hold_sec = float(g("loss_hold_sec").value)
        self.latency_probe = bool(g("latency_probe").value)
        self.track_only = bool(g("track_only").value)
        self.body_frame_id = g("body_frame_id").value
        self.preview = bool(g("preview").value)
        self.mirror_mode = str(g("mirror_mode").value).lower() != "direct"
        self.min_visibility = float(g("min_visibility").value)
        self.head_gain_yaw = float(g("head_gain_yaw").value)
        self.head_gain_pitch = float(g("head_gain_pitch").value)
        self.max_retarget_err_deg = float(g("max_retarget_err_deg").value)
        self.retarget_info = {}

        self.max_step = max_step_for(self.rate_hz, float(g("max_joint_speed").value))
        self.decay_step = max_step_for(self.rate_hz, float(g("decay_speed").value))

        # Limits: prefer the live URDF, cross-check the measured table.
        self.limits, limit_src = resolve_limits(
            self._fetch_robot_description(), logger=self.get_logger()
        )

        self.driven = list(ARM_L_JOINTS) + list(ARM_R_JOINTS) + list(HEAD_JOINTS)
        if self.use_lift:
            self.driven += list(LIFT_JOINTS)

        # One filter per OUTPUT JOINT ANGLE, not per landmark — filtering
        # landmarks and then computing angles lets noise through the
        # nonlinearity. The head gets a slower filter: its travel is tiny
        # (+/-20 deg yaw), so the same responsiveness reads as twitchy.
        mc, beta, hmc = (
            float(g("min_cutoff").value),
            float(g("beta").value),
            float(g("head_min_cutoff").value),
        )
        self.filters = {
            j: OneEuro(min_cutoff=(hmc if j in HEAD_JOINTS else mc), beta=beta)
            for j in self.driven
        }

        self.pubs = {
            topic: self.create_publisher(JointTrajectory, topic, 10)
            for topic, joints in GROUPS.items()
            if self.use_lift or joints is not LIFT_JOINTS
        }

        self.enabled = True
        self.q_cmd = None            # last commanded pose; None until seeded
        self.last_seen_t = None      # wall time of the last successful read
        self.joint_state = None
        self.joint_state_t = None
        self._lock = threading.Lock()
        self._ticks = 0
        self._clamp_hits = 0
        self._slew_hits = 0
        self._t0 = time.monotonic()

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_service(SetBool, "/mirror_enable", self._on_enable)

        # --- M3 tracking outputs ---
        self.marker_pub = self.create_publisher(MarkerArray, "/body/markers", 10)
        self.tracked_pub = self.create_publisher(Bool, "/body/tracked", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.observation = None
        self._vision_thread = None
        self._vision_stop = threading.Event()
        self._vision_ms = []
        self._vision_frames = 0
        self._vision_tracked = 0

        self.source = self._make_source()
        self.tracker = self._make_tracker()
        if self.source is not None:
            self.source.start()

        self.create_timer(1.0 / self.rate_hz, self._tick)
        if self.latency_probe:
            self.create_timer(3.0, self._report)

        if self.tracker is not None:
            # Vision runs on its OWN thread at its own rate (~32 Hz measured
            # for pose_landmarker_full on this box) so the 50 Hz control tick
            # is never blocked by a 31 ms inference.
            self._vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
            self._vision_thread.start()

        mode = (
            "TRACK-ONLY (robot parked)"
            if self.track_only
            else ("synthetic sweep" if self.synthetic else "mirroring")
        )
        self.get_logger().info(
            f"mirror_node up: mode={mode} "
            f"source={self.source.name if self.source else self.tracker.name} "
            f"rate={self.rate_hz:.0f} Hz max_step={self.max_step:.4f} rad/tick "
            f"({self.max_step * self.rate_hz:.2f} rad/s) limits={limit_src} "
            f"joints={len(self.driven)}"
        )

    # ------------------------------------------------------------------ setup
    def _make_source(self):
        """Build the JOINT-TARGET source, or None when nothing drives the robot.

        Vision imports are LAZY and live only in _make_tracker, so a synthetic
        run never imports mediapipe or cv2 — a missing camera or a drifted venv
        cannot make the synthetic demo fail.
        """
        if self.synthetic:
            from humanoid_mirror.pose_source import SyntheticPoseSource

            return SyntheticPoseSource(
                period_s=float(self.get_parameter("sweep_period_s").value),
                use_lift=self.use_lift,
            )
        if self.track_only:
            return None  # M3: tracking is published, but nothing commands joints
        from humanoid_mirror.retarget import MirrorPoseSource

        return MirrorPoseSource(self)

    def _make_tracker(self):
        """Build the camera tracker, or None in synthetic mode."""
        if self.synthetic:
            return None
        try:
            from humanoid_mirror.body_track import BodyTracker
        except ModuleNotFoundError as exc:
            # Almost always: the node is running under the SYSTEM python, but
            # mediapipe lives in /opt/mpvenv. Say so, rather than letting a
            # bare ModuleNotFoundError read as a camera problem.
            import sys

            raise RuntimeError(
                f"{exc}. mirror_node is running under {sys.executable}. In the "
                f"Docker image mediapipe lives in /opt/mpvenv — launch via "
                f"mirror.launch.py (it sets prefix=/opt/mpvenv/bin/python), or "
                f"run: /opt/mpvenv/bin/python $(which mirror_node). Synthetic "
                f"mode needs none of this, which is why it still works."
            ) from exc

        g = self.get_parameter
        return BodyTracker(
            device=g("camera").value,
            model_path=g("pose_model_path").value,
            min_visibility=float(g("min_visibility").value),
            preview=self.preview,
        )

    # ---------------------------------------------------------- vision thread
    def _vision_loop(self):
        """Run inference as fast as it will go, publish tracking, hand the
        latest observation to the control tick. Never touches q_cmd."""
        t0 = self._t0
        while not self._vision_stop.is_set():
            t = time.monotonic() - t0
            start = time.monotonic()
            try:
                obs = self.tracker.read(t)
            except Exception as exc:  # noqa: BLE001 — vision must not kill the node
                self.get_logger().warn(
                    f"tracker raised: {exc}", throttle_duration_sec=5.0
                )
                time.sleep(0.1)
                continue
            elapsed_ms = (time.monotonic() - start) * 1e3

            if obs is None:
                time.sleep(0.005)
                continue

            with self._lock:
                self.observation = obs
                self._vision_ms.append(elapsed_ms)
                if len(self._vision_ms) > 200:
                    self._vision_ms.pop(0)
                self._vision_frames += 1
                if obs.tracked:
                    self._vision_tracked += 1

            self._publish_tracking(obs)
            if self.preview:
                self._show_preview(obs)

    def _show_preview(self, obs):
        try:
            import cv2

            canvas = self.tracker.annotate(obs)
            if canvas is not None:
                cv2.imshow("humanoid_mirror — body (mirrored preview)", canvas)
                cv2.waitKey(1)
        except Exception as exc:  # noqa: BLE001 — no X11 must not kill tracking
            self.get_logger().warn(
                f"preview unavailable: {exc}", throttle_duration_sec=10.0
            )
            self.preview = False

    def _publish_tracking(self, obs):
        """TF frames + skeleton markers + the tracked flag."""
        stamp = self.get_clock().now().to_msg()
        self.tracked_pub.publish(Bool(data=bool(obs.tracked)))

        if not obs.tracked or obs.basis is None:
            # Clear the skeleton so a stale one cannot look like live tracking.
            clear = Marker()
            clear.header.frame_id = self.body_frame_id
            clear.header.stamp = stamp
            clear.action = Marker.DELETEALL
            self.marker_pub.publish(MarkerArray(markers=[clear]))
            return

        from humanoid_mirror.body_track import basis_to_quaternion

        transforms = []
        for index, child in TF_LANDMARKS.items():
            if index not in obs.points_body:
                continue
            if obs.visibility.get(index, 0.0) < float(
                self.get_parameter("min_visibility").value
            ):
                continue  # never publish TF for a landmark MediaPipe hallucinated
            x, y, z = obs.points_body[index]
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.body_frame_id
            tf.child_frame_id = child
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = float(z)
            tf.transform.rotation.w = 1.0
            transforms.append(tf)

        # The torso frame itself, as a real TF frame with orientation. This is
        # the debugging aid that matters for M4: if retargeted angles look
        # wrong, check that human/torso's axes point where the body does before
        # suspecting the trigonometry.
        qx, qy, qz, qw = basis_to_quaternion(obs.basis)
        torso = TransformStamped()
        torso.header.stamp = stamp
        torso.header.frame_id = self.body_frame_id
        torso.child_frame_id = "human/torso"
        sx, sy, sz = obs.points_body[11]
        rx, ry, rz = obs.points_body[12]
        torso.transform.translation.x = float((sx + rx) * 0.5)
        torso.transform.translation.y = float((sy + ry) * 0.5)
        torso.transform.translation.z = float((sz + rz) * 0.5)
        torso.transform.rotation.x = float(qx)
        torso.transform.rotation.y = float(qy)
        torso.transform.rotation.z = float(qz)
        torso.transform.rotation.w = float(qw)
        transforms.append(torso)
        self.tf_broadcaster.sendTransform(transforms)

        self.marker_pub.publish(self._skeleton_markers(obs, stamp))

    def _skeleton_markers(self, obs, stamp):
        """Skeleton as a LINE_LIST plus a SPHERE_LIST, gated on visibility."""
        min_vis = float(self.get_parameter("min_visibility").value)
        try:
            from mediapipe.tasks.python import vision as mpv

            connections = [
                (c.start, c.end)
                for c in mpv.PoseLandmarksConnections.POSE_LANDMARKS
            ]
        except Exception:  # noqa: BLE001 — markers are cosmetic, never fatal
            connections = []

        lines = Marker()
        lines.header.frame_id = self.body_frame_id
        lines.header.stamp = stamp
        lines.ns = "body"
        lines.id = 0
        lines.type = Marker.LINE_LIST
        lines.action = Marker.ADD
        lines.scale.x = 0.02
        lines.color.g = 0.9
        lines.color.b = 0.4
        lines.color.a = 1.0
        lines.pose.orientation.w = 1.0
        for a, b in connections:
            if a not in obs.points_body or b not in obs.points_body:
                continue
            if min(obs.visibility.get(a, 0.0), obs.visibility.get(b, 0.0)) < min_vis:
                continue
            for idx in (a, b):
                x, y, z = obs.points_body[idx]
                lines.points.append(Point(x=float(x), y=float(y), z=float(z)))

        dots = Marker()
        dots.header.frame_id = self.body_frame_id
        dots.header.stamp = stamp
        dots.ns = "body"
        dots.id = 1
        dots.type = Marker.SPHERE_LIST
        dots.action = Marker.ADD
        dots.scale.x = dots.scale.y = dots.scale.z = 0.035
        dots.color.r = 1.0
        dots.color.g = 0.8
        dots.color.a = 1.0
        dots.pose.orientation.w = 1.0
        for idx, (x, y, z) in obs.points_body.items():
            if obs.visibility.get(idx, 0.0) < min_vis:
                continue
            dots.points.append(Point(x=float(x), y=float(y), z=float(z)))

        return MarkerArray(markers=[lines, dots])

    def _fetch_robot_description(self):
        """URDF from robot_state_publisher, if it is up. Best-effort: the node
        must still start (on the measured table) when it is not."""
        from rcl_interfaces.srv import GetParameters

        cli = self.create_client(
            GetParameters, "/robot_state_publisher/get_parameters"
        )
        if not cli.wait_for_service(timeout_sec=10.0):
            return None
        fut = cli.call_async(GetParameters.Request(names=["robot_description"]))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None or not res.values:
            return None
        return res.values[0].string_value or None

    # --------------------------------------------------------------- callbacks
    def _on_joint_state(self, msg):
        with self._lock:
            self.joint_state = dict(zip(msg.name, msg.position))
            self.joint_state_t = time.monotonic()

    def _on_enable(self, request, response):
        """Deadman. false freezes the robot where it is; true re-seeds from
        /joint_states so resuming cannot produce a jump."""
        self.enabled = bool(request.data)
        if self.enabled:
            self.q_cmd = None  # force a re-seed from the live state
            for f in self.filters.values():
                f.reset()
            response.message = "mirroring ENABLED (re-seeded from /joint_states)"
        else:
            response.message = "mirroring DISABLED (holding current pose)"
        response.success = True
        self.get_logger().info(response.message)
        return response

    # --------------------------------------------------------------- the tick
    def _seed(self):
        """Seed q_cmd from the live robot so the first command is a no-op."""
        with self._lock:
            state = dict(self.joint_state) if self.joint_state else None
        if state is None:
            return False
        self.q_cmd = {j: state.get(j, 0.0) for j in self.driven}
        return True

    def _tick(self):
        if self.source is None:
            return  # M3 track_only: vision publishes, nothing commands the robot
        if not self.enabled:
            return
        if self.q_cmd is None and not self._seed():
            return  # no /joint_states yet — publishing now would be a guess

        t = time.monotonic() - self._t0
        t_read = time.monotonic()
        try:
            targets = self.source.read(t)
        except Exception as exc:  # noqa: BLE001 — a source must never kill the loop
            self.get_logger().warn(f"pose source raised: {exc}", throttle_duration_sec=5)
            targets = None
        read_ms = (time.monotonic() - t_read) * 1e3

        if targets:
            self.last_seen_t = t
            step = self.max_step
        else:
            # Lost tracking: hold, then decay home at a gentler speed.
            held = self.last_seen_t is not None and (t - self.last_seen_t) < self.loss_hold_sec
            if held:
                return
            targets, step = HOME, self.decay_step

        for j in self.driven:
            if j not in targets:
                continue
            filtered = self.filters[j](float(targets[j]), t)
            limited = clamp(filtered, j, self.limits, self.margin)
            if abs(limited - filtered) > 1e-9:
                self._clamp_hits += 1
            before = self.q_cmd[j]
            self.q_cmd[j] = slew(limited, before, step)
            if abs(limited - before) > step + 1e-9:
                self._slew_hits += 1

        self._publish()
        self._ticks += 1
        if self.latency_probe:
            self._last_read_ms = read_ms

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        sec = int(self.time_from_start)
        nanosec = int((self.time_from_start - sec) * 1e9)
        for topic, joints in GROUPS.items():
            pub = self.pubs.get(topic)
            if pub is None:
                continue
            msg = JointTrajectory()
            msg.header.stamp = stamp
            msg.joint_names = list(joints)
            point = JointTrajectoryPoint()
            point.positions = [self.q_cmd[j] for j in joints]
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points = [point]
            pub.publish(msg)

    def _report(self):
        elapsed = max(time.monotonic() - self._t0, 1e-6)
        line = (
            f"ticks={self._ticks} ({self._ticks / elapsed:.1f} Hz effective) "
            f"clamp_hits={self._clamp_hits} slew_hits={self._slew_hits}"
        )
        with self._lock:
            samples = list(self._vision_ms)
            frames, tracked = self._vision_frames, self._vision_tracked
            obs = self.observation
        if samples:
            median = sorted(samples)[len(samples) // 2]
            line += (
                f" | vision {frames} frames ({frames / elapsed:.1f} Hz), "
                f"infer median {median:.1f} ms, tracked {tracked}/{frames}"
            )
            if obs is not None and obs.tracked:
                line += f", frame={obs.frame_source}"
            elif obs is not None:
                line += f", lost({obs.reason})"
        ri = self.retarget_info
        if ri:
            arms = ",".join(ri.get("arms", [])) or "none"
            line += f" | arms={arms} err={ri.get('err_deg', {})}"
            if ri.get("skipped"):
                line += f" skipped={ri['skipped']}"
        self.get_logger().info(line)

    def shutdown(self):
        """Publish one final hold so the robot does not keep chasing a stale
        trajectory point after the node dies."""
        self._vision_stop.set()
        if self._vision_thread is not None:
            self._vision_thread.join(timeout=2.0)
        try:
            if self.q_cmd is not None and self.source is not None:
                self._publish()
            if self.source is not None:
                self.source.stop()
            if self.tracker is not None:
                self.tracker.stop()
        except Exception:  # noqa: BLE001 — best effort during teardown
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MirrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
