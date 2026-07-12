#!/usr/bin/env bash
# Launch wrapper for the ROS 2 <-> MCP bridge.
# Sources ROS 2, activates the venv (which sees system rclpy), runs the server.
# This is the command Claude Code invokes as the "ros2" MCP server.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

exec "$HERE/.venv/bin/python" "$HERE/ros2_mcp_server.py"
