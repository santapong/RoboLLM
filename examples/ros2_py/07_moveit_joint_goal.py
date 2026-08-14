#!/usr/bin/env python3
"""07 · Move a robot arm with MoveIt (the MoveGroup action, pure rclpy).

Plans AND executes a joint-space goal for the Franka Panda arm by sending a
moveit_msgs/action/MoveGroup goal to move_group. No moveit_py bindings needed —
this shows what those wrappers do underneath: build a MotionPlanRequest with
joint constraints, send it, let OMPL plan, execute the trajectory.

Prereqs (two terminals):
  1) scripts/launch/simulation/moveit_panda.sh # brings up move_group + RViz (Panda)
  2) .venv/bin/python examples/ros2_py/07_moveit_joint_goal.py --pose ready

Watch the arm move in the RViz window.
Concepts: actions, MotionPlanRequest, JointConstraint, PlanningOptions.
"""
import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
POSES = {
    "ready":    [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    "home":     [0.0,  0.0,   0.0,  0.0,   0.0, 0.0,   0.0],
    "extended": [0.0, -0.3,   0.0, -1.0,   0.0, 1.9,   0.785],
}


class ArmGoal(Node):
    def __init__(self):
        super().__init__("moveit_joint_goal")
        self.client = ActionClient(self, MoveGroup, "move_action")

    def move(self, group: str, targets: list[float]):
        self.get_logger().info("waiting for move_group…")
        if not self.client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error("move_group unavailable — run scripts/launch/simulation/moveit_panda.sh")
            return

        constraints = Constraints()
        for name, pos in zip(JOINTS, targets):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.goal_constraints.append(constraints)
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.planning_options.plan_only = False        # plan AND execute

        self.get_logger().info(f"planning to {targets} …")
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error("goal REJECTED")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val
        self.get_logger().info(f"done (MoveItErrorCode {code}; 1 = SUCCESS)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", choices=sorted(POSES), default="ready")
    ap.add_argument("--group", default="panda_arm")
    args = ap.parse_args()
    rclpy.init()
    node = ArmGoal()
    try:
        node.move(args.group, POSES[args.pose])
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
