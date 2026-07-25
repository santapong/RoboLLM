#!/usr/bin/env python3
"""M2 acceptance: is the humanoid actually being driven, correctly?

Subscribes to the four controller topics for a window and asserts the things
that would make a demo silently wrong rather than visibly broken:

  1. all four topics publish at the configured rate (default 50 Hz)
  2. every joint stays inside [lower+margin, upper-margin] — never on a stop
  3. no per-tick jump exceeds the slew budget (the velocity limit)
  4. the robot actually MOVED — a node that publishes a frozen pose at a
     perfect 50 Hz would pass 1-3 and show nothing in RViz
  5. /joint_states follows the commands (mock hardware is really executing)

Run it against a live `ros2-arm mirror synthetic rviz:=false`:

    ros2 run humanoid_mirror mirror_accept       # if installed
    python3 tools/mirror_accept.py --seconds 8

Emits a machine-readable RESULT:{json} line, matching wallweld_accept.py and
hand_accept.py, so CI can parse it without scraping the table.
"""

import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, __file__.rsplit("/tools/", 1)[0] + "/src/humanoid_mirror")

from humanoid_mirror.ffw_config import (  # noqa: E402
    ARM_L_JOINTS,
    ARM_R_JOINTS,
    HEAD_JOINTS,
    LIFT_JOINTS,
)
from humanoid_mirror.joint_limits import MEASURED  # noqa: E402

TOPICS = {
    "/arm_l_controller/joint_trajectory": ARM_L_JOINTS,
    "/arm_r_controller/joint_trajectory": ARM_R_JOINTS,
    "/head_controller/joint_trajectory": HEAD_JOINTS,
    "/lift_controller/joint_trajectory": LIFT_JOINTS,
}

# A joint that never moves more than this over the whole window is stuck.
MOVED_EPS = 0.02


class Accept(Node):
    def __init__(self, expect_hz, margin, max_speed):
        super().__init__("mirror_accept")
        self.expect_hz = expect_hz
        self.margin = margin
        self.max_speed = max_speed
        # The per-tick budget the node enforces. Checking this directly is
        # stronger than checking a derived speed, because it needs no clock.
        self.max_step = max_speed / expect_hz
        self.step_violations = []

        self.counts = {t: 0 for t in TOPICS}
        self.first_t = {}
        self.last_t = {}
        self.prev = {}                       # joint -> last position
        self.prev_t = {}                     # joint -> last stamp
        self.span = {}                       # joint -> (min, max)
        self.limit_violations = []
        self.speed_violations = []
        self.joint_state = None

        for topic in TOPICS:
            self.create_subscription(
                JointTrajectory, topic, self._make_cb(topic), 50
            )
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_state(self, msg):
        self.joint_state = dict(zip(msg.name, msg.position))

    def _make_cb(self, topic):
        def cb(msg):
            # Arrival time is only used for the RATE check. It must NOT be used
            # to compute joint speed: DDS delivers in bursts, so two messages
            # the node published 20 ms apart can arrive 6 ms apart, and
            # delta/arrival_dt then reports a phantom 6+ rad/s. Speed is
            # measured against the publisher's own header stamp below.
            t_arrive = self.get_clock().now().nanoseconds / 1e9
            self.counts[topic] += 1
            self.first_t.setdefault(topic, t_arrive)
            self.last_t[topic] = t_arrive

            if not msg.points:
                self.limit_violations.append(f"{topic}: empty trajectory point")
                return

            t_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            for name, pos in zip(msg.joint_names, msg.points[0].positions):
                lo, up, _vel = MEASURED[name]
                if not (lo + self.margin - 1e-6 <= pos <= up - self.margin + 1e-6):
                    self.limit_violations.append(
                        f"{name}={pos:.4f} outside [{lo + self.margin:.4f}, "
                        f"{up - self.margin:.4f}]"
                    )
                if name in self.prev:
                    delta = abs(pos - self.prev[name])
                    # THE REAL INVARIANT: the node clamps every tick to
                    # max_step, so consecutive commands can never differ by
                    # more than that — regardless of any timing measurement.
                    if delta > self.max_step * 1.05 + 1e-9:
                        self.step_violations.append(
                            f"{name}: step {delta:.4f} > {self.max_step:.4f} rad/tick"
                        )
                    # Speed from the publisher's stamps, which are immune to
                    # burst delivery. Skip degenerate/duplicate stamps.
                    dt = t_stamp - self.prev_t[name]
                    if dt > 1e-4:
                        speed = delta / dt
                        if speed > self.max_speed * 1.5:
                            self.speed_violations.append(
                                f"{name}: {speed:.2f} rad/s > {self.max_speed:.2f}"
                            )
                self.prev[name] = pos
                self.prev_t[name] = t_stamp
                lo_s, hi_s = self.span.get(name, (pos, pos))
                self.span[name] = (min(lo_s, pos), max(hi_s, pos))

        return cb

    def rates(self):
        out = {}
        for topic, n in self.counts.items():
            span = self.last_t.get(topic, 0) - self.first_t.get(topic, 0)
            out[topic] = (n - 1) / span if span > 0 and n > 1 else 0.0
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--margin", type=float, default=0.05)
    ap.add_argument("--max-speed", type=float, default=2.0)
    ap.add_argument("--no-lift", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Accept(args.rate, args.margin, args.max_speed)
    deadline = node.get_clock().now().nanoseconds + args.seconds * 1e9
    while node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    expected_topics = [
        t for t in TOPICS if not (args.no_lift and TOPICS[t] is LIFT_JOINTS)
    ]
    rates = node.rates()
    failures = []

    print("=" * 66)
    print("  humanoid_mirror M2 acceptance — synthetic whole-body sweep")
    print("=" * 66)

    print("\npublish rates")
    for topic in expected_topics:
        hz = rates[topic]
        ok = abs(hz - args.rate) <= max(5.0, args.rate * 0.2)
        print(f"  [{'PASS' if ok else 'FAIL'}] {topic:42s} {hz:6.1f} Hz "
              f"({node.counts[topic]} msgs)")
        if not ok:
            failures.append(f"{topic} at {hz:.1f} Hz, expected ~{args.rate:.0f}")

    print("\njoint limits (margin %.3f)" % args.margin)
    uniq = sorted(set(node.limit_violations))
    print(f"  [{'PASS' if not uniq else 'FAIL'}] "
          f"{len(node.limit_violations)} violation(s)")
    for v in uniq[:6]:
        print(f"        {v}")
    failures += uniq[:6]

    print("\nper-tick slew budget (<= %.4f rad/tick)" % node.max_step)
    uniq_p = sorted(set(node.step_violations))
    print(f"  [{'PASS' if not uniq_p else 'FAIL'}] "
          f"{len(node.step_violations)} violation(s)")
    for v in uniq_p[:6]:
        print(f"        {v}")
    failures += uniq_p[:6]

    print("\nvelocity from publisher stamps (<= %.2f rad/s)" % args.max_speed)
    uniq_s = sorted(set(node.speed_violations))
    print(f"  [{'PASS' if not uniq_s else 'FAIL'}] "
          f"{len(node.speed_violations)} violation(s)")
    for v in uniq_s[:6]:
        print(f"        {v}")
    failures += uniq_s[:6]

    print("\nmotion (did the robot actually move?)")
    expected_joints = [j for t in expected_topics for j in TOPICS[t]]
    # Wrists are parked at 0 by design in M2, so only require motion on the
    # joints the sweep actually drives.
    driven = [j for j in expected_joints if not j.endswith(("joint5", "joint6", "joint7"))]
    still = [
        j for j in driven
        if j in node.span and (node.span[j][1] - node.span[j][0]) < MOVED_EPS
    ]
    missing = [j for j in driven if j not in node.span]
    print(f"  [{'PASS' if not still and not missing else 'FAIL'}] "
          f"{len(driven) - len(still) - len(missing)}/{len(driven)} swept joints moved")
    for j in (still + missing)[:6]:
        rng = node.span.get(j)
        print(f"        {j}: {'never published' if rng is None else f'range {rng[1] - rng[0]:.4f}'}")
    failures += [f"{j} did not move" for j in (still + missing)[:6]]

    print("\nmock hardware follows commands")
    if node.joint_state is None:
        print("  [FAIL] /joint_states silent")
        failures.append("/joint_states silent")
    else:
        tracked = [
            j for j in driven
            if j in node.joint_state and j in node.prev
            and abs(node.joint_state[j] - node.prev[j]) < 0.30
        ]
        ok = len(tracked) >= len(driven) - 2
        print(f"  [{'PASS' if ok else 'FAIL'}] {len(tracked)}/{len(driven)} "
              f"joints within 0.30 rad of the last command")
        if not ok:
            failures.append("mock hardware is not following commands")

    print("\n" + "=" * 66)
    if failures:
        print(f"  FAILED — {len(failures)} problem(s)")
    else:
        print("  ALL CHECKS PASSED — the humanoid is moving, safely.")
    print("=" * 66)

    result = {
        "passed": not failures,
        "rates_hz": {t: round(rates[t], 2) for t in expected_topics},
        "limit_violations": len(node.limit_violations),
        "step_violations": len(node.step_violations),
        "speed_violations": len(node.speed_violations),
        "joints_moved": len(driven) - len(still) - len(missing),
        "joints_expected": len(driven),
        "failures": failures[:12],
    }
    print("RESULT:" + json.dumps(result))

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
