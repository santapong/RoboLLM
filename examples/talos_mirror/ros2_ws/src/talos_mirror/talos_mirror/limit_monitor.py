"""M2 acceptance, live: watches /joint_states for `duration_s` seconds and
asserts every joint stays inside its measured limit envelope, then reports
the measured max excursion per group and exits.

Runs as a SECOND NODE IN THE SAME LAUNCH as sweep_node (see
launch/sweep.launch.py) — not a separate acceptance script run afterward
against a live process, unlike humanoid_mirror's tools/mirror_accept.py.
Both approaches are valid; this one is what the M2 task for TALOS asked for.

⚠️ Excursion is measured directly from /joint_states POSITION values against
joint_limits.excursion() (distance outside [lower+margin, upper-margin]; 0 if
inside) — never from subscriber arrival-time deltas. That trap is documented
at length in ../../TECHNICAL.md and humanoid_mirror/TECHNICAL.md: DDS
delivers in bursts, so a rate or a "how fast did this move" number computed
from wall-clock arrival time is fiction. Excursion has no time dimension at
all, which sidesteps the whole class of bug — it only ever needs the
position value and the limit table.
"""

import json
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from talos_mirror.joint_limits import DEFAULT_MARGIN, excursion
from talos_mirror.talos_config import ALL_JOINTS, GROUP_JOINTS


class LimitMonitor(Node):
    def __init__(self):
        super().__init__("talos_limit_monitor")

        self.declare_parameter("duration_s", 10.0)
        self.declare_parameter("margin", DEFAULT_MARGIN)

        self.duration_s = float(self.get_parameter("duration_s").value)
        self.margin = float(self.get_parameter("margin").value)

        self._max_excursion = {name: 0.0 for name in ALL_JOINTS}
        self._span = {}  # joint -> (min, max) seen, to also report travel
        self._samples = 0
        self._joints_seen = set()

        self.create_subscription(JointState, "/joint_states", self._on_state, 50)
        self._deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.duration_s
        )
        self._timer = self.create_timer(0.2, self._check_deadline)
        self.get_logger().info(
            f"talos_limit_monitor: watching /joint_states for {self.duration_s:.1f} s"
        )

    def _on_state(self, msg):
        self._samples += 1
        for name, pos in zip(msg.name, msg.position):
            if name not in self._max_excursion:
                continue  # not one of our 30 group joints (e.g. a mimic)
            self._joints_seen.add(name)
            exc = excursion(pos, name, margin=self.margin)
            if exc > self._max_excursion[name]:
                self._max_excursion[name] = exc
            lo, hi = self._span.get(name, (pos, pos))
            self._span[name] = (min(lo, pos), max(hi, pos))

    def _check_deadline(self):
        if self.get_clock().now() >= self._deadline:
            self._report()
            self._timer.cancel()
            rclpy.shutdown()

    def _report(self):
        print("=" * 66)
        print("  talos_mirror M2 acceptance — limit monitor")
        print("=" * 66)
        print(f"\n/joint_states samples: {self._samples}")

        missing = set(ALL_JOINTS) - self._joints_seen
        if missing:
            print(f"  [WARN] {len(missing)} joint(s) never seen: {sorted(missing)}")

        by_group = {}
        overall_pass = True
        print(f"\nmax excursion per group (margin {self.margin:.3f}, 0.0 = never exceeded)")
        for group, joints in GROUP_JOINTS.items():
            group_max = max((self._max_excursion[j] for j in joints), default=0.0)
            group_travel = max(
                ((self._span[j][1] - self._span[j][0]) if j in self._span else 0.0)
                for j in joints
            )
            ok = group_max <= 1e-9
            overall_pass &= ok
            by_group[group] = {
                "max_excursion_rad": round(group_max, 6),
                "max_travel_rad": round(group_travel, 4),
            }
            print(
                f"  [{'PASS' if ok else 'FAIL'}] {group:8s} "
                f"max_excursion={group_max:.6f} rad   max_travel={group_travel:.4f} rad"
            )

        print("\n" + "=" * 66)
        verdict = "PASS" if (overall_pass and not missing) else "FAIL"
        print(f"  {verdict} — /joint_states {'never' if overall_pass else 'DID'} "
              f"exceed limits over {self.duration_s:.1f} s")
        print("=" * 66)

        result = {
            "passed": bool(overall_pass and not missing),
            "duration_s": self.duration_s,
            "samples": self._samples,
            "missing_joints": sorted(missing),
            "by_group": by_group,
        }
        print("RESULT:" + json.dumps(result))
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = LimitMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
