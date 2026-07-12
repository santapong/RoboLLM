#!/usr/bin/env python3
"""
ros2_mcp_server.py — a tiny, readable ROS 2 <-> MCP bridge.

It runs a single ROS 2 node in a background thread and exposes a handful of
tools over MCP (stdio). An LLM client (Claude Code) can then introspect and
drive the robot by calling these tools.

Extend it by adding more @mcp.tool() functions — that's the whole point:
you own this file and grow it as you learn.

Requires ROS 2 (Jazzy) to be sourced before launch — see run-server.sh.
"""
from __future__ import annotations

import os
import subprocess
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose

from mcp.server.fastmcp import FastMCP

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")
os.makedirs(ASSETS, exist_ok=True)

mcp = FastMCP("ros2-bridge")


class Bridge(Node):
    """One long-lived ROS 2 node the MCP tools talk to."""

    def __init__(self) -> None:
        super().__init__("claude_ros2_bridge")
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._last_odom: Odometry | None = None
        self._last_scan: LaserScan | None = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    # ---- helpers used by tools -------------------------------------------
    def publish_twist(self, linear: float, angular: float, seconds: float) -> None:
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        end = time.time() + max(0.0, seconds)
        rate_hz = 10.0
        while time.time() < end:
            self.cmd_vel.publish(t)
            time.sleep(1.0 / rate_hz)
        self.cmd_vel.publish(Twist())  # stop

    def pose(self) -> dict[str, Any]:
        if self._last_odom is None:
            return {"error": "no /odom yet — is the simulation running?"}
        p = self._last_odom.pose.pose.position
        q = self._last_odom.pose.pose.orientation
        return {"x": round(p.x, 3), "y": round(p.y, 3),
                "qz": round(q.z, 3), "qw": round(q.w, 3)}


# --- start ROS in a background thread so MCP stays responsive --------------
rclpy.init()
_node = Bridge()
_executor = SingleThreadedExecutor()
_executor.add_node(_node)
threading.Thread(target=_executor.spin, daemon=True).start()


# ================= MCP TOOLS ===============================================
@mcp.tool()
def list_topics() -> list[dict[str, str]]:
    """List all active ROS 2 topics and their message types."""
    return [{"topic": n, "type": ",".join(t)}
            for n, t in _node.get_topic_names_and_types()]


@mcp.tool()
def list_nodes() -> list[str]:
    """List all running ROS 2 nodes."""
    return [f"{ns.rstrip('/')}/{name}" for name, ns in _node.get_node_names_and_namespaces()]


@mcp.tool()
def get_robot_pose() -> dict[str, Any]:
    """Get the robot's current pose from /odom (x, y, orientation)."""
    return _node.pose()


@mcp.tool()
def drive(linear: float = 0.2, angular: float = 0.0, seconds: float = 2.0) -> str:
    """Drive the robot by publishing /cmd_vel.

    linear:  forward m/s (negative = backward)
    angular: turn rad/s (positive = left)
    seconds: how long to drive, then auto-stop.
    """
    seconds = min(float(seconds), 10.0)  # safety cap
    _node.publish_twist(linear, angular, seconds)
    return f"drove linear={linear} angular={angular} for {seconds}s, then stopped"


@mcp.tool()
def stop() -> str:
    """Immediately stop the robot."""
    _node.cmd_vel.publish(Twist())
    return "stopped"


@mcp.tool()
def navigate_to(x: float, y: float, yaw_deg: float = 0.0, timeout_s: float = 120.0) -> str:
    """Send a Nav2 goal to (x, y) with heading yaw_deg. Requires Nav2 running."""
    import math
    if not _node.nav.wait_for_server(timeout_sec=3.0):
        return "error: Nav2 action server 'navigate_to_pose' not available — is Nav2 running?"
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = _node.get_clock().now().to_msg()
    goal.pose.pose.position.x = float(x)
    goal.pose.pose.position.y = float(y)
    goal.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
    goal.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
    send = _node.nav.send_goal_async(goal)
    deadline = time.time() + timeout_s
    while not send.done() and time.time() < deadline:
        time.sleep(0.1)
    handle = send.result()
    if handle is None or not handle.accepted:
        return "error: goal rejected"
    result_future = handle.get_result_async()
    while not result_future.done() and time.time() < deadline:
        time.sleep(0.2)
    if not result_future.done():
        return "error: navigation timed out"
    return f"navigation finished (status={result_future.result().status})"


@mcp.tool()
def get_laser_scan() -> dict[str, Any]:
    """Summarize the latest /scan: nearest obstacle distance (meters) in each
    direction — front, left, right, back. Useful for obstacle awareness."""
    import math
    s = _node._last_scan
    if s is None:
        return {"error": "no /scan yet — is the sim running (and does the robot have a lidar)?"}

    def nearest(center_deg: float, width_deg: float = 30.0) -> float | None:
        vals = []
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r <= s.range_min:
                continue
            ang = math.degrees(s.angle_min + i * s.angle_increment) % 360.0
            d = min((ang - center_deg) % 360.0, (center_deg - ang) % 360.0)
            if d <= width_deg / 2.0:
                vals.append(r)
        return round(min(vals), 3) if vals else None

    return {"front": nearest(0), "left": nearest(90),
            "back": nearest(180), "right": nearest(270),
            "num_points": len(s.ranges)}


@mcp.tool()
def run_ros2(command: str, timeout_s: float = 15.0) -> str:
    """Run a `ros2` CLI command and return its output — the fast way to
    introspect and learn. Examples:
      'topic list'
      'topic echo /odom --once'
      'node info /claude_ros2_bridge'
      'interface show geometry_msgs/msg/Twist'
      'param list'
    ROS 2 is already sourced in this process. Prefer read-only commands.
    """
    args = ["ros2"] + command.split()
    try:
        out = subprocess.run(args, capture_output=True, text=True,
                             timeout=min(float(timeout_s), 60.0))
    except subprocess.TimeoutExpired:
        return f"(timed out after {timeout_s}s — for streaming echo use '--once')"
    result = (out.stdout or "") + (("\n[stderr] " + out.stderr) if out.stderr else "")
    return result.strip() or "(no output)"


if __name__ == "__main__":
    mcp.run(transport="stdio")
