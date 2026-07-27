"""M2 synthetic exercise for TALOS: a 50 Hz sine sweep across all 30 group
joints (arm_l, arm_r, head, torso, leg_l, leg_r), hard-clamped at the
measured limits, publishing single-point JointTrajectory messages to the six
mock JointTrajectoryControllers talos_moveit_config's mock_bringup.launch.py
brings up.

Same shape as humanoid_mirror's SyntheticPoseSource + mirror_node control
loop, collapsed into one node since TALOS's M2 has no camera/filter/mirror
concerns to separate out — this is the whole-body no-camera proof that the
publish path (six JTCs, disjoint joint sets, controller names, limits) works,
same as FFW's M2 proved for the four-controller stack it inherited.

TRAPS avoided (see ../../TECHNICAL.md and humanoid_mirror/TECHNICAL.md):
  * amplitude is sized from MEASURED limits with headroom (safe_amplitude,
    frac<1), AND every computed position is still passed through
    joint_limits.clamp() before publishing — belt and suspenders, because a
    hand-picked amplitude is exactly the kind of number that silently drifts
    wrong when a joint's home value or limits change.
  * per-joint phase offsets keep joints from all turning around at once,
    which would otherwise mask whether the clamp is correctly sized (see
    humanoid_mirror's _ease() docstring for the same reasoning applied to a
    raised-cosine wave here we use a pure sine, which is already C1
    continuous — no discontinuity to paper over).
  * NEVER compute a rate or a speed from subscriber/publish ARRIVAL times —
    not applicable to this node (it only publishes), but the limit_monitor
    node in this same package that watches this sweep's output follows the
    same rule: it reads header stamps / the invariant directly, never wall
    time between DDS deliveries.
"""

import math

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from talos_mirror.joint_limits import DEFAULT_MARGIN, clamp, safe_amplitude
from talos_mirror.talos_config import (
    CONTROLLER_TOPICS,
    GROUP_JOINTS,
    GROUPS,
    home_joint_state,
)

HOME = home_joint_state()

# Per-joint phase offset (fraction of the period), spread out so the 30
# joints do not all reach a turning point together. Deterministic, index-
# based — no randomness, so a run is exactly reproducible.
_PHASE = {name: (i / 30.0) for i, name in enumerate(sorted(HOME))}


class SweepNode(Node):
    def __init__(self):
        super().__init__("talos_sweep")

        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("period_s", 10.0)
        self.declare_parameter("margin", DEFAULT_MARGIN)
        self.declare_parameter("amplitude_frac", 0.85)
        self.declare_parameter("time_from_start", 0.06)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.period_s = float(self.get_parameter("period_s").value)
        self.margin = float(self.get_parameter("margin").value)
        self.amplitude_frac = float(self.get_parameter("amplitude_frac").value)
        self.time_from_start = float(self.get_parameter("time_from_start").value)

        # amplitude per joint, computed once at startup from MEASURED limits.
        self._amplitude = {
            name: safe_amplitude(name, home, margin=self.margin, frac=self.amplitude_frac)
            for name, home in HOME.items()
        }

        self._pubs = {
            group: self.create_publisher(JointTrajectory, topic, 10)
            for group, topic in CONTROLLER_TOPICS.items()
        }

        self._t0 = self.get_clock().now()
        period = 1.0 / self.rate_hz
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"talos_sweep: {len(HOME)} joints across {len(GROUPS)} groups, "
            f"{self.rate_hz:.0f} Hz, period {self.period_s:.1f} s"
        )

    def _target(self, name, t):
        home = HOME[name]
        amp = self._amplitude[name]
        phase = _PHASE[name]
        raw = home + amp * math.sin(2.0 * math.pi * (t / self.period_s + phase))
        # Hard clamp — the backstop, independent of whether the amplitude
        # sizing above was ever wrong.
        return clamp(raw, name, margin=self.margin)

    def _tick(self):
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds / 1e9
        stamp = now.to_msg()

        for group, joints in GROUP_JOINTS.items():
            positions = [self._target(name, t) for name in joints]
            msg = JointTrajectory()
            msg.header.stamp = stamp
            msg.joint_names = list(joints)
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(self.time_from_start)
            point.time_from_start.nanosec = int(
                (self.time_from_start - int(self.time_from_start)) * 1e9
            )
            msg.points = [point]
            self._pubs[group].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
