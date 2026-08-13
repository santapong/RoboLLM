#!/usr/bin/env python3
"""Black-box probe for the arm driver's standard trajectory action.

Run while ``simulation.launch.py`` is connected to ``hardware/sim_uno.py``.
The probe never opens a serial device and never selects the physical profile.
"""
from __future__ import annotations

import sys

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

ACTION = "/arm_controller/follow_joint_trajectory"
JOINTS = [f"joint{index}" for index in range(1, 7)]


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("robo_arm_action_probe")
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.feedback_count = 0

    def goal(self, positions: list[float], seconds: int = 2):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=positions,
            time_from_start=Duration(sec=seconds),
        )]
        return goal

    def feedback(self, _message) -> None:
        self.feedback_count += 1

    def send(self, goal):
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def result(self, handle):
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main() -> int:
    rclpy.init()
    probe = Probe()
    try:
        if not probe.client.wait_for_server(timeout_sec=5.0):
            raise AssertionError(f"action server {ACTION} was not available")

        completed = probe.send(probe.goal([0.15, 0.10, 0.05, 0.0, -0.05, -0.10]))
        assert completed.accepted, "valid trajectory was rejected"
        completed_result = probe.result(completed)
        assert completed_result.status == GoalStatus.STATUS_SUCCEEDED
        assert completed_result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        assert probe.feedback_count > 0, "accepted goal produced no feedback"

        canceling = probe.send(probe.goal([0.0] * 6, seconds=4))
        assert canceling.accepted, "cancel test trajectory was rejected"
        cancel_future = canceling.cancel_goal_async()
        rclpy.spin_until_future_complete(probe, cancel_future)
        assert cancel_future.result().goals_canceling, "cancel request was rejected"
        canceled_result = probe.result(canceling)
        assert canceled_result.status == GoalStatus.STATUS_CANCELED

        invalid = probe.goal([0.0] * 6)
        invalid.trajectory.joint_names[-1] = "not_a_joint"
        rejected = probe.send(invalid)
        assert not rejected.accepted, "invalid joint set was accepted"

        print(
            "arm action probe: PASS "
            f"(feedback={probe.feedback_count}, success, cancel, rejection)"
        )
        return 0
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"arm action probe: FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
