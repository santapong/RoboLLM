"""mirror_node — drives the FFW semi-humanoid's arms, head and lift.

M2 scope: SYNTHETIC ONLY. `synthetic:=true` runs a scripted whole-body sweep
with no camera and, importantly, **no MediaPipe imported anywhere in the
process** (the vision imports are lazy, inside _make_source, exactly as
hand_follow does it). This is the milestone that proves the entire publish
path — four JTCs over disjoint joint sets, controller names, limit clamps,
slew limits, RViz — before any vision risk exists.

Camera mode arrives in M4 and plugs in behind the same PoseSource interface;
the control tick below does not change.

ARCHITECTURE NOTE — the one real upgrade over hand_follow: the input rate and
the command rate are DECOUPLED. hand_follow ticks at 20 Hz because that is
what MediaPipe can sustain. Here the control timer runs at 50 Hz and
One-Euro-interpolates toward whatever the source last said, so when M4 adds
PoseLandmarker (24.8 ms, ~13 Hz) the robot still moves at 50 Hz. That buys
visibly smoother motion for zero extra vision CPU.
"""

import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

        g = self.get_parameter
        self.synthetic = g("synthetic").value
        self.rate_hz = float(g("rate_hz").value)
        self.time_from_start = float(g("time_from_start").value)
        self.margin = float(g("limit_margin").value)
        self.use_lift = bool(g("use_lift").value)
        self.loss_hold_sec = float(g("loss_hold_sec").value)
        self.latency_probe = bool(g("latency_probe").value)

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

        self.source = self._make_source()
        self.source.start()

        self.create_timer(1.0 / self.rate_hz, self._tick)
        if self.latency_probe:
            self.create_timer(3.0, self._report)

        self.get_logger().info(
            f"mirror_node up: source={self.source.name} rate={self.rate_hz:.0f} Hz "
            f"max_step={self.max_step:.4f} rad/tick "
            f"({self.max_step * self.rate_hz:.2f} rad/s) limits={limit_src} "
            f"joints={len(self.driven)}"
        )

    # ------------------------------------------------------------------ setup
    def _make_source(self):
        """Build the pose source. Vision imports are LAZY and live only in the
        camera branch, so synthetic mode never imports mediapipe or cv2 — a
        missing camera or a broken venv cannot make the synthetic demo fail."""
        if self.synthetic:
            from humanoid_mirror.pose_source import SyntheticPoseSource

            return SyntheticPoseSource(
                period_s=float(self.get_parameter("sweep_period_s").value),
                use_lift=self.use_lift,
            )
        raise NotImplementedError(
            "camera mode lands in M4. Run with synthetic:=true — "
            "see examples/humanoid_mirror/README.md for the build plan."
        )

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
        self.get_logger().info(
            f"ticks={self._ticks} ({self._ticks / elapsed:.1f} Hz effective) "
            f"clamp_hits={self._clamp_hits} slew_hits={self._slew_hits}"
        )

    def shutdown(self):
        """Publish one final hold so the robot does not keep chasing a stale
        trajectory point after the node dies."""
        try:
            if self.q_cmd is not None:
                self._publish()
            self.source.stop()
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
