#!/usr/bin/env python3
"""pickplace_accept.py — topic-level acceptance meter for gen3_pick_place.

The vendored gen3_pick_place tree ships no dedicated acceptance observer, so
this meter asserts topic-level evidence that the synthetic pick-and-place
actually ran, mirroring the handfollow acceptance shape:

  * attach event   -- the box is grasped: a GetPlanningScene query reports a
                      non-empty robot_state.attached_collision_objects at some
                      point during the run (proximity-gated attach succeeded).
  * command rate   -- the arm command stream on
                      /joint_trajectory_controller/joint_trajectory holds
                      >= 15 Hz (hand_pick_place publishes every 20 Hz tick,
                      including explicit holds).

Run INSIDE the running stack container (plain python3, no mediapipe):
    python3 pickplace_accept.py [duration_s]
Prints a single RESULT:{json} line (plus human-readable progress).
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
ARM_CMD_TOPIC = "/joint_trajectory_controller/joint_trajectory"
RATE_ACCEPT = 15.0


class Meter(Node):
    def __init__(self):
        super().__init__("pickplace_accept")
        self.arm_t = []       # command-message arrival times
        self.js_t = []
        self.attach_seen = False
        self.attach_at = None
        self.create_subscription(JointTrajectory, ARM_CMD_TOPIC,
                                 self._arm, 50)
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.gps = self.create_client(GetPlanningScene, "/get_planning_scene")

    def _arm(self, _m):
        self.arm_t.append(time.monotonic())

    def _js(self, _m):
        self.js_t.append(time.monotonic())

    def poll_attach(self):
        if self.attach_seen or not self.gps.service_is_ready():
            return
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        fut = self.gps.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if res is not None and \
                res.scene.robot_state.attached_collision_objects:
            self.attach_seen = True
            self.attach_at = time.monotonic()
            print("  attach observed (box attached to gripper)", flush=True)


def _rate(times):
    if len(times) < 2:
        return 0.0
    span = times[-1] - times[0]
    return (len(times) - 1) / span if span > 0 else 0.0


def main():
    rclpy.init()
    n = Meter()
    n.gps.wait_for_service(timeout_sec=15.0)
    t_end = time.monotonic() + DUR
    next_poll = 0.0
    while time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
        now = time.monotonic()
        if now >= next_poll:
            n.poll_attach()
            next_poll = now + 1.0
        if n.attach_seen and now - (n.attach_at or now) > 8.0:
            # attach confirmed and enough command samples gathered; stop early
            if len(n.arm_t) > 50:
                break
    arm_rate = _rate(n.arm_t)
    js_rate = _rate(n.js_t)
    print("samples: arm_cmd={} joint_states={} dur={:.0f}s".format(
        len(n.arm_t), len(n.js_t), DUR))
    print("arm command rate: {:.2f} Hz  [accept: >= {:.0f} Hz]".format(
        arm_rate, RATE_ACCEPT))
    print("attach event: {}".format("SEEN" if n.attach_seen else "MISSING"))
    ok = n.attach_seen and arm_rate >= RATE_ACCEPT
    print("ACCEPT: {}".format("PASS" if ok else "FAIL"))
    out = {
        "mode": "pickplace",
        "duration_s": DUR,
        "attach_seen": bool(n.attach_seen),
        "n_arm_cmd": len(n.arm_t),
        "n_js": len(n.js_t),
        "arm_rate_hz": round(arm_rate, 2),
        "js_rate_hz": round(js_rate, 2),
        "rate_accept_hz": RATE_ACCEPT,
        "pass": bool(ok),
    }
    print("RESULT:" + json.dumps(out), flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
