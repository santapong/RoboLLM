"""mirror_node -- TALOS M4: drives all SIX groups (torso, arm_l, arm_r,
head, leg_l, leg_r) from /body/observation, mirror mode by default.

c2-qp (M5) adds `mode` ("direct" or "qp", default "direct" -- see below):
direct is the unmodified M4 pose source (retarget.MirrorPoseSource, pure
closed-form direction matching, wrists parked at 0); qp additionally runs
both arms through qp_retarget.WristQPSolver (pink task-space differential
IK, warm-started, joint/velocity-limited from the pinocchio model) via
qp_pose_source.QPMirrorPoseSource, which ALSO One-Euro-filters the arm
landmark inputs before retargeting (see that module's docstring for why
that is a distinct filter from the per-output-joint-angle one below, which
still runs in BOTH modes). Legs/head/torso are byte-identical to direct
mode regardless of `mode` -- WristQPSolver.solve only ever touches the two
7-DOF arm chains, see its own docstring. mode=="qp" imports pink/pinocchio,
which live in /opt/mpvenv only -- this node must run under
prefix=/opt/mpvenv/bin/python (mirror.launch.py already sets this
unconditionally, so switching `mode` needs no launch change); mode=="direct"
never imports pink at all, so it still runs under a plain system python if
launched that way by hand.

DEFAULT IS "direct", NOT "qp" -- the A/B judge REFUTED qp's own "tracks
better" claim (see tools/qp_ab_harness.py's session output): wrist POSITION
regresses from 0 to 5.31 mm versus the M4 baseline (qp only ever had an
orientation target to gain from, and pin_model.build_wrist_target's
docstring already documents the ~4-5 deg residual that convention carries
even at its best), the orientation metric this milestone can report has no
independent ground truth to confirm the gain is real, and the worst-case
solve tick on this CPU (31-58 ms, see qp_info.solve_time_s) can overrun this
node's own 50 Hz / 20 ms control budget. `mode` stays a launch arg (qp is
still fully selectable, mode:=qp) for exactly the experimentation that
measured those numbers -- it is just no longer what a bare `ros2 launch`
gives you.

DEADMAN: /mirror_enable (std_srvs/SetBool), ported from humanoid_mirror's
mirror_node -- request.data=false HOLDS every group exactly where q_cmd
already has it (the _tick() guard returns before publishing, same mechanism
a lost observation already uses); true re-seeds q_cmd from the live
/joint_states and resets every OneEuro filter, so resuming can never jump.
`enabled` (parameter + mirror.launch.py launch arg) sets the STARTING state;
it defaults to true so `ros2 launch talos_mirror mirror.launch.py` keeps
auto-starting the mock demo the same way it always has -- /mirror_enable is
for toggling it live, not for changing what a bare launch does.

Ported from examples/humanoid_mirror's mirror_node.py, adapted to a
DECOUPLED architecture: humanoid_mirror ran its own vision thread inside the
same process; here vision is track_node (M3, unchanged) running as a
SEPARATE node, and this node only SUBSCRIBES to the /body/observation JSON
topic it publishes. That is what "mirror_node.py (subscribes body obs...)"
in the task means literally -- mirror.launch.py brings up mock_bringup +
track_node + this node together (mirror [synthetic] verb), but they are
three independent ROS nodes, not one process sharing a lock like upstream's
tracker+solver.

GATING: each of the six groups is held (its q_cmd left untouched) unless
its own gate/prerequisite is currently satisfied -- torso is always driven
whenever ANYTHING is tracked (torso_frame() needs the shoulders for tracked
to be True at all), the other five follow retarget()'s per-limb "targets
only contains what is currently gated in" contract exactly like
humanoid_mirror's per-arm hold. This is the "holds gated-out limbs"
requirement: a joint absent from retarget()'s returned targets this tick is
left exactly where q_cmd already has it, filtered/clamped/slewed or not.

RATE DECOUPLING: track_node publishes at its own rate (30 Hz synthetic, or
whatever the camera sustains); this node's control timer runs at a fixed
50 Hz and re-solves the LATEST observation every tick (never blocks waiting
for a new one), exactly as humanoid_mirror's vision-thread/control-timer
split does -- just with a topic in place of a shared attribute + lock.
"""

import json
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from talos_mirror.joint_limits import MEASURED, OneEuro, clamp, max_step_for, slew
from talos_mirror.retarget import MirrorPoseSource
from talos_mirror.talos_config import ALL_JOINTS, CONTROLLER_TOPICS, GROUP_JOINTS, home_joint_state

HOME = home_joint_state()


class MirrorNode(Node):
    def __init__(self):
        super().__init__("talos_mirror_node")

        p = self.declare_parameter
        p("rate_hz", 50.0)
        p("time_from_start", 0.06)
        # 1.5 rad/s at 50 Hz -> 0.03 rad/tick, comfortably under every
        # group's slowest URDF velocity limit (leg pitch/roll 5.8 rad/s,
        # torso 5.4 rad/s, wrist yaw 1.76 rad/s -- the tightest -- 1.5 rad/s
        # clears all of them with margin). Sized from rate, never copied
        # from another example -- see joint_limits.slew's own warning about
        # exactly this trap.
        p("max_joint_speed", 1.5)
        p("limit_margin", 0.05)
        p("min_cutoff", 1.0)
        p("beta", 0.5)
        p("head_min_cutoff", 0.5)
        p("loss_hold_sec", 2.0)
        p("decay_speed", 0.4)
        p("mirror_mode", "mirror")     # "mirror" (facing you) or "direct"
        p("head_gain_yaw", 0.8)
        p("head_gain_pitch", 0.6)
        p("torso_gain_yaw", 0.5)
        p("torso_gain_pitch", 0.5)
        p("latency_probe", False)
        # c2-qp REFUTED: default "direct" (M4 only) -- qp (pink diff-IK on
        # wrists) is selectable but no longer the default, see module
        # docstring's DEFAULT IS "direct" section for the A/B numbers.
        p("mode", "direct")
        p("enabled", True)             # deadman starting state -- see /mirror_enable
        p("landmark_min_cutoff", 1.0)  # One-Euro on arm landmarks, BEFORE retargeting
        p("landmark_beta", 0.5)
        p("qp_position_cost", 1.0)
        p("qp_orientation_cost", 0.5)
        p("qp_posture_cost", 1e-2)

        g = self.get_parameter
        self.rate_hz = float(g("rate_hz").value)
        self.time_from_start = float(g("time_from_start").value)
        self.margin = float(g("limit_margin").value)
        self.loss_hold_sec = float(g("loss_hold_sec").value)
        self.mirror_mode = str(g("mirror_mode").value).lower() != "direct"
        self.head_gain_yaw = float(g("head_gain_yaw").value)
        self.head_gain_pitch = float(g("head_gain_pitch").value)
        self.torso_gain_yaw = float(g("torso_gain_yaw").value)
        self.torso_gain_pitch = float(g("torso_gain_pitch").value)
        self.latency_probe = bool(g("latency_probe").value)
        self.mode = str(g("mode").value).lower()
        if self.mode not in ("qp", "direct"):
            self.get_logger().warn(f"unknown mode={self.mode!r}, falling back to direct")
            self.mode = "direct"
        self.enabled = bool(g("enabled").value)
        self.landmark_min_cutoff = float(g("landmark_min_cutoff").value)
        self.landmark_beta = float(g("landmark_beta").value)
        self.qp_position_cost = float(g("qp_position_cost").value)
        self.qp_orientation_cost = float(g("qp_orientation_cost").value)
        self.qp_posture_cost = float(g("qp_posture_cost").value)
        self.retarget_info = {}
        self.qp_info = {}

        self.max_step = max_step_for(self.rate_hz, float(g("max_joint_speed").value))
        self.decay_step = max_step_for(self.rate_hz, float(g("decay_speed").value))

        self.driven = list(ALL_JOINTS)
        mc, beta, hmc = (
            float(g("min_cutoff").value), float(g("beta").value),
            float(g("head_min_cutoff").value),
        )
        head_torso = set(GROUP_JOINTS["head"]) | set(GROUP_JOINTS["torso"])
        # One filter per OUTPUT JOINT ANGLE, not per landmark -- see
        # humanoid_mirror's mirror_node for why (filtering landmarks then
        # computing angles lets noise through the acos/atan2 nonlinearity).
        # Head+torso get the slower filter: both have small travel (head
        # +-75 deg yaw at most, torso +-13/-13..42 deg), so full
        # responsiveness reads as twitchy the same way FFW's head did.
        self.filters = {
            j: OneEuro(min_cutoff=(hmc if j in head_torso else mc), beta=beta)
            for j in self.driven
        }

        self.pubs = {
            group: self.create_publisher(JointTrajectory, topic, 10)
            for group, topic in CONTROLLER_TOPICS.items()
        }

        # T3 (loop-algo A2): preallocate the six JointTrajectory/
        # JointTrajectoryPoint message objects ONCE, here, instead of
        # instantiating six of each every _publish() tick (50 Hz x 6 groups
        # = 300 message-object allocations/sec previously). joint_names is
        # also built once per group here -- it never changes across the
        # node's lifetime (GROUP_JOINTS is a module-level constant) -- and
        # msg.points is set to [point] once, wiring each preallocated
        # JointTrajectoryPoint into its parent message permanently. Every
        # subsequent _publish() only MUTATES msg.header.stamp and this same
        # point's .positions/.time_from_start in place; no message wrapper
        # object nor joint_names list is ever recreated after this loop.
        # See _publish()'s own comment for why reusing these objects across
        # publish() calls carries no aliasing hazard.
        self._traj_msgs = {}
        for group, joints in GROUP_JOINTS.items():
            msg = JointTrajectory()
            msg.joint_names = list(joints)
            point = JointTrajectoryPoint()
            msg.points = [point]
            self._traj_msgs[group] = (msg, point)

        self.q_cmd = None
        self.last_seen_t = None
        self.joint_state = None
        self._lock = threading.Lock()
        self._ticks = 0
        self._clamp_hits = 0
        self._slew_hits = 0
        self._t0 = time.monotonic()

        self.observation = None
        self.create_subscription(String, "/body/observation", self._on_observation, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_service(SetBool, "/mirror_enable", self._on_enable)

        if self.mode == "qp":
            # Imports pink/pinocchio -- /opt/mpvenv only, see module
            # docstring. Deferred to here (not a top-of-file import) so
            # mode=="direct" never needs pink installed at all.
            from talos_mirror.qp_pose_source import QPMirrorPoseSource
            from talos_mirror.qp_retarget import WristQPSolver

            self.qp_solver = WristQPSolver(
                position_cost=self.qp_position_cost,
                orientation_cost=self.qp_orientation_cost,
                posture_cost=self.qp_posture_cost,
            )
            self.source = QPMirrorPoseSource(self)
        else:
            self.source = MirrorPoseSource(self)

        self.create_timer(1.0 / self.rate_hz, self._tick)
        if self.latency_probe:
            self.create_timer(3.0, self._report)

        self.get_logger().info(
            f"talos_mirror_node up: retarget_mode={self.mode} "
            f"mirror={'mirror' if self.mirror_mode else 'direct'} "
            f"enabled={self.enabled} "
            f"rate={self.rate_hz:.0f} Hz max_step={self.max_step:.4f} rad/tick "
            f"({self.max_step * self.rate_hz:.2f} rad/s) joints={len(self.driven)} -- "
            f"subscribing /body/observation (published by talos_track, run "
            f"separately -- see launch/mirror.launch.py); deadman on "
            f"/mirror_enable (std_srvs/SetBool)"
        )

    # --------------------------------------------------------------- callbacks
    def _on_observation(self, msg):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self.observation = d

    def _on_joint_state(self, msg):
        with self._lock:
            self.joint_state = dict(zip(msg.name, msg.position))

    def _on_enable(self, request, response):
        """Deadman, ported from humanoid_mirror's mirror_node. false HOLDS
        every group exactly where q_cmd already has it (the _tick() guard
        below returns before publishing, same mechanism a lost observation
        already uses -- no new hold path to get wrong); true re-seeds q_cmd
        from the live /joint_states and resets every OneEuro filter so
        resuming after a hold can never jump.

        REPAIR (gate round 2, T2 defect): also invalidate the source's
        observation-dedup cache (if it has one -- QPMirrorPoseSource does
        not) so the first post-resume tick is forced to re-solve. Without
        this, an unchanged /body/observation across the freeze (operator
        still, or the upstream tracker paused) made the dedup cache replay
        the pre-freeze solution -- computed against the STALE prev=q_cmd
        seed -- instead of solving fresh against the seed re-seeded above.
        See MirrorPoseSource.invalidate()'s own docstring for the measured
        worst case this closes."""
        self.enabled = bool(request.data)
        if self.enabled:
            self.q_cmd = None  # force a re-seed from the live state
            for f in self.filters.values():
                f.reset()
            invalidate = getattr(self.source, "invalidate", None)
            if invalidate is not None:
                invalidate()
            response.message = "mirroring ENABLED (re-seeded from /joint_states)"
        else:
            response.message = "mirroring DISABLED (holding current pose)"
        response.success = True
        self.get_logger().info(response.message)
        return response

    # --------------------------------------------------------------- the tick
    def _seed(self):
        """Seed q_cmd from the live robot (mock hardware echoes clamped
        commands back as /joint_states) so the first command is a no-op."""
        with self._lock:
            state = dict(self.joint_state) if self.joint_state else None
        if state is None:
            return False
        self.q_cmd = {j: state.get(j, HOME.get(j, 0.0)) for j in self.driven}
        return True

    def _tick(self):
        if not self.enabled:
            return  # deadman: hold exactly where q_cmd already is, publish nothing
        if self.q_cmd is None and not self._seed():
            return  # no /joint_states yet -- publishing now would be a guess

        t = time.monotonic() - self._t0
        try:
            targets = self.source.read(t)
        except Exception as exc:  # noqa: BLE001 -- a source must never kill the loop
            self.get_logger().warn(f"pose source raised: {exc}", throttle_duration_sec=5)
            targets = None

        if targets:
            self.last_seen_t = t
            step = self.max_step
        else:
            held = self.last_seen_t is not None and (t - self.last_seen_t) < self.loss_hold_sec
            if held:
                return  # hold everything -- no observation yet, or fully lost
            targets, step = HOME, self.decay_step

        for j in self.driven:
            if j not in targets:
                continue  # HELD: gated-out limb, joint stays exactly where it is
            filtered = self.filters[j](float(targets[j]), t)
            limited = clamp(filtered, j, MEASURED, self.margin)
            if abs(limited - filtered) > 1e-9:
                self._clamp_hits += 1
            before = self.q_cmd[j]
            self.q_cmd[j] = slew(limited, before, step)
            if abs(limited - before) > step + 1e-9:
                self._slew_hits += 1

        self._publish()
        self._ticks += 1

    def _publish(self):
        # T3: reuse the six preallocated (msg, point) pairs from __init__ --
        # mutate header.stamp and this tick's positions/time_from_start in
        # place, then publish the SAME message object again. Safe to reuse
        # immediately after publish() returns: rclpy's Publisher.publish()
        # converts the Python message and calls straight through to the
        # rmw/DDS layer (rmw_fastrtps_cpp's rmw_publish) SYNCHRONOUSLY --
        # it CDR-serializes the message into the RTPS writer's own buffer
        # (or, depending on QoS, copies it into the writer history cache)
        # before rmw_publish returns control to rclpy, and rclpy's publish()
        # itself does not queue the message for later (there is no
        # background serialization thread in the middleware's send path,
        # and this node's publishers use create_publisher's own vanilla
        # depth-10 QoS, no zero-copy/loaned-message API that would extend a
        # message's lifetime past the call). So by the time pub.publish(msg)
        # returns here, the wire bytes for THIS tick are already committed
        # and the next tick's in-place mutation cannot race or alias a
        # publish still in flight. No new message wrapper objects, and no
        # new joint_names list, are created on this path any more.
        stamp = self.get_clock().now().to_msg()
        sec = int(self.time_from_start)
        nanosec = int((self.time_from_start - sec) * 1e9)
        for group, joints in GROUP_JOINTS.items():
            pub = self.pubs.get(group)
            if pub is None:
                continue
            msg, point = self._traj_msgs[group]
            msg.header.stamp = stamp
            point.positions = [self.q_cmd[j] for j in joints]
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            pub.publish(msg)

    def _report(self):
        elapsed = max(time.monotonic() - self._t0, 1e-6)
        ri = self.retarget_info
        arms = ",".join(ri.get("arms", [])) or "none"
        legs = ",".join(ri.get("legs", [])) or "none"
        line = (
            f"ticks={self._ticks} ({self._ticks / elapsed:.1f} Hz effective) "
            f"clamp_hits={self._clamp_hits} slew_hits={self._slew_hits} "
            f"arms={arms} legs={legs} err={ri.get('err_deg', {})}"
        )
        if ri.get("skipped"):
            line += f" skipped={ri['skipped']}"
        if self.mode == "qp" and self.qp_info:
            st = self.qp_info.get("solve_time_s")
            if st is not None:
                line += f" qp_solve_ms={st * 1000.0:.2f}"
        self.get_logger().info(line)

    def shutdown(self):
        """Publish one final hold so the robot does not keep chasing a stale
        trajectory point after the node dies."""
        try:
            if self.q_cmd is not None:
                self._publish()
        except Exception:  # noqa: BLE001 -- best effort during teardown
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
