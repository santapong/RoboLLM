#!/usr/bin/env python3
"""06 · Send a Nav2 goal yourself (the action-client pattern).

This is the low-level version of the MCP `navigate_to` tool — learn how a ROS 2
*action* works: send a goal, get accepted/rejected, stream feedback, await result.

Prereqs (three terminals):
  1) sim/launch_turtlebot.sh
  2) sim/launch_nav2.sh   (Nav2 + a map; set the initial pose in RViz first!)
  3) this script:  .venv/bin/python examples/ros2_py/06_send_nav_goal.py --x 1.5 --y 0.5

Concepts: ActionClient, NavigateToPose, goal handles, feedback callbacks.
"""
import argparse
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class NavGoal(Node):
    def __init__(self):
        super().__init__("send_nav_goal")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def go(self, x, y, yaw_deg):
        self.get_logger().info("waiting for Nav2 action server…")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 not available — is sim/launch_nav2.sh running?")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        goal.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)

        self.get_logger().info(f"sending goal → x={x} y={y} yaw={yaw_deg}°")
        send = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error("goal REJECTED")
            return
        self.get_logger().info("goal accepted — navigating…")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"done (status code {result_future.result().status})")

    def on_feedback(self, fb):
        d = fb.feedback.distance_remaining
        self.get_logger().info(f"  {d:.2f} m remaining", throttle_duration_sec=1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=1.5)
    ap.add_argument("--y", type=float, default=0.5)
    ap.add_argument("--yaw", type=float, default=0.0)
    args = ap.parse_args()
    rclpy.init()
    node = NavGoal()
    try:
        node.go(args.x, args.y, args.yaw)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
