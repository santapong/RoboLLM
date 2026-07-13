#!/usr/bin/env python3
"""
robot_bridge.py — one shared ROS 2 node used by BOTH the MCP server and the web UI.

Why a shared module? So there is a single, readable place that talks to the robot.
- ros2_mcp_server.py imports it  -> Claude can drive the robot via MCP tools.
- web/server.py imports it       -> you drive the robot from a browser dashboard.

The node runs in a background thread so the host program (MCP stdio loop or the
web server's event loop) stays responsive.

Teleop note: the web UI needs *continuous* motion while a key is held. Publishing
a fixed-duration Twist doesn't fit that. So the bridge keeps a "target velocity"
and a 20 Hz timer republishes it, with a DEADMAN watchdog: if no fresh command
arrives within `deadman_s`, the robot is automatically stopped. That means a
dropped websocket or a released key can never leave the robot driving away.

Requires ROS 2 (Jazzy) sourced before import — see run-server.sh / web/run-web.sh.
"""
from __future__ import annotations

import math
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


class Bridge(Node):
    """One long-lived ROS 2 node. Publishers/subscribers + small helper methods."""

    def __init__(self, node_name: str = "claude_ros2_bridge", deadman_s: float = 0.6) -> None:
        super().__init__(node_name)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._last_odom: Odometry | None = None
        self._last_scan: LaserScan | None = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)

        # continuous-teleop state, guarded by the deadman watchdog
        self._deadman_s = deadman_s
        self._target = Twist()
        self._target_stamp = 0.0
        self._teleop_active = False
        self.create_timer(1.0 / 20.0, self._teleop_tick)  # 20 Hz republish

    # ---- subscriptions ----------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    # ---- continuous teleop (used by the web UI) ---------------------------
    def set_velocity(self, linear: float, angular: float) -> None:
        """Set the target velocity. A 20 Hz timer keeps publishing it until a
        newer command arrives or the deadman timeout stops the robot."""
        self._target.linear.x = float(linear)
        self._target.angular.z = float(angular)
        self._target_stamp = time.monotonic()
        self._teleop_active = True

    def _teleop_tick(self) -> None:
        if not self._teleop_active:
            return
        if time.monotonic() - self._target_stamp > self._deadman_s:
            self.cmd_vel.publish(Twist())  # deadman: no fresh command -> stop
            self._teleop_active = False
            return
        self.cmd_vel.publish(self._target)

    # ---- timed drive (used by the MCP `drive` tool) -----------------------
    def drive_for(self, linear: float, angular: float, seconds: float) -> None:
        """Blocking: drive for `seconds`, then stop. Bypasses teleop state."""
        self._teleop_active = False
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        end = time.time() + max(0.0, seconds)
        while time.time() < end:
            self.cmd_vel.publish(t)
            time.sleep(0.1)
        self.cmd_vel.publish(Twist())

    def stop(self) -> None:
        self._teleop_active = False
        self.cmd_vel.publish(Twist())

    # ---- read helpers -----------------------------------------------------
    def pose(self) -> dict[str, Any]:
        if self._last_odom is None:
            return {"error": "no /odom yet — is the simulation running?"}
        p = self._last_odom.pose.pose.position
        q = self._last_odom.pose.pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        v = self._last_odom.twist.twist
        return {"x": round(p.x, 3), "y": round(p.y, 3), "yaw_deg": round(yaw, 1),
                "vx": round(v.linear.x, 3), "wz": round(v.angular.z, 3)}

    def laser_summary(self) -> dict[str, Any]:
        s = self._last_scan
        if s is None:
            return {"error": "no /scan yet — is the sim running (with a lidar)?"}

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

    def laser_radar(self, sectors: int = 36) -> list[float | None]:
        """Nearest distance per angular sector (for a mini radar plot in the UI)."""
        s = self._last_scan
        if s is None:
            return []
        buckets: list[list[float]] = [[] for _ in range(sectors)]
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r <= s.range_min:
                continue
            ang = (s.angle_min + i * s.angle_increment) % (2 * math.pi)
            buckets[int(ang / (2 * math.pi) * sectors) % sectors].append(r)
        return [round(min(b), 3) if b else None for b in buckets]

    def navigate_to(self, x: float, y: float, yaw_deg: float = 0.0,
                    timeout_s: float = 120.0) -> str:
        if not self.nav.wait_for_server(timeout_sec=3.0):
            return "error: Nav2 action server not available — is Nav2 running?"
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
        goal.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
        send = self.nav.send_goal_async(goal)
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


# --- module-level singleton: init ROS once, spin in a background thread -----
_node: Bridge | None = None
_lock = threading.Lock()


def get_bridge(node_name: str = "claude_ros2_bridge") -> Bridge:
    """Return the shared Bridge, starting rclpy + a spin thread on first call."""
    global _node
    with _lock:
        if _node is None:
            if not rclpy.ok():
                rclpy.init()
            _node = Bridge(node_name=node_name)
            executor = SingleThreadedExecutor()
            executor.add_node(_node)
            threading.Thread(target=executor.spin, daemon=True).start()
        return _node
