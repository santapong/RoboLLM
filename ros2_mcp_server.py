#!/usr/bin/env python3
"""
ros2_mcp_server.py — a tiny, readable ROS 2 <-> MCP bridge.

Exposes a handful of tools over MCP (stdio) so an LLM client (Claude Code) can
introspect and drive the robot. The actual ROS 2 node lives in robot_bridge.py,
which is shared with the web dashboard (web/server.py) — one place talks to ROS.

Extend it by adding more @mcp.tool() functions — that's the whole point:
you own this file and grow it as you learn. Restart Claude Code to reload tools.

Requires ROS 2 (Jazzy) sourced before launch — see run-server.sh.
"""
from __future__ import annotations

import os
import subprocess
from typing import Any

from mcp.server.fastmcp import FastMCP

from robot_bridge import get_bridge

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")
os.makedirs(ASSETS, exist_ok=True)

mcp = FastMCP("ros2-bridge")
_node = get_bridge("claude_ros2_bridge")


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
    """Get the robot's current pose from /odom (x, y, yaw, velocity)."""
    return _node.pose()


@mcp.tool()
def drive(linear: float = 0.2, angular: float = 0.0, seconds: float = 2.0) -> str:
    """Drive the robot by publishing /cmd_vel.

    linear:  forward m/s (negative = backward)
    angular: turn rad/s (positive = left)
    seconds: how long to drive, then auto-stop.
    """
    seconds = min(float(seconds), 10.0)  # safety cap
    _node.drive_for(linear, angular, seconds)
    return f"drove linear={linear} angular={angular} for {seconds}s, then stopped"


@mcp.tool()
def stop() -> str:
    """Immediately stop the robot."""
    _node.stop()
    return "stopped"


@mcp.tool()
def navigate_to(x: float, y: float, yaw_deg: float = 0.0, timeout_s: float = 120.0) -> str:
    """Send a Nav2 goal to (x, y) with heading yaw_deg. Requires Nav2 running."""
    return _node.navigate_to(x, y, yaw_deg, timeout_s)


@mcp.tool()
def get_laser_scan() -> dict[str, Any]:
    """Summarize the latest /scan: nearest obstacle distance (meters) in each
    direction — front, left, right, back. Useful for obstacle awareness."""
    return _node.laser_summary()


@mcp.tool()
def get_camera_image() -> dict[str, Any]:
    """Capture the robot's current camera view to a JPEG in assets/screenshots
    and return its path + size. Needs a camera-equipped model (waffle/waffle_pi;
    the burger has no camera). This is how Claude 'sees' what it's driving."""
    shots = os.path.join(ASSETS, "screenshots")
    os.makedirs(shots, exist_ok=True)
    # stable-ish name without wall-clock (frame count would need state); overwrite latest
    path = os.path.join(shots, "camera_latest.jpg")
    return _node.save_camera_jpeg(path)


@mcp.tool()
def set_safe_mode(enabled: bool = True, min_obstacle_m: float = 0.35) -> str:
    """Toggle obstacle-safe teleop. When on, forward driving is blocked if an
    obstacle is closer than min_obstacle_m (turning is still allowed)."""
    _node.safe_mode = bool(enabled)
    _node.min_obstacle_m = float(min_obstacle_m)
    return f"safe_mode={_node.safe_mode}, min_obstacle_m={_node.min_obstacle_m}"


_bag_proc: subprocess.Popen | None = None


@mcp.tool()
def start_recording(topics: str = "/odom /scan /cmd_vel /camera/image_raw",
                    name: str = "run") -> str:
    """Start recording topics to a rosbag under assets/bags/<name> — build a
    dataset while you drive/experiment. Pass space-separated topic names."""
    global _bag_proc
    if _bag_proc and _bag_proc.poll() is None:
        return "already recording — stop_recording first"
    bags = os.path.join(ASSETS, "bags")
    os.makedirs(bags, exist_ok=True)
    out = os.path.join(bags, name)
    args = ["ros2", "bag", "record", "-o", out] + topics.split()
    _bag_proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return f"recording {topics} -> {out} (pid {_bag_proc.pid}); call stop_recording when done"


@mcp.tool()
def stop_recording() -> str:
    """Stop the active rosbag recording."""
    global _bag_proc
    if not _bag_proc or _bag_proc.poll() is not None:
        return "not recording"
    _bag_proc.terminate()
    try:
        _bag_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        _bag_proc.kill()
    return "recording stopped"


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
