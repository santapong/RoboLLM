#!/usr/bin/env bash
# Launch the browser dashboard for the ROS 2 robot.
# Sources ROS 2 + the project venv (which sees system rclpy), then runs uvicorn.
# Open http://localhost:8080 after it starts.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"

source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

cd "$ROOT"
# Localhost-only by default (safe). To expose on your LAN:
#   HOST=0.0.0.0 ROBOT_TOKEN=somesecret web/run-web.sh
HOST="${HOST:-127.0.0.1}"
echo "Dashboard → http://localhost:8080   (Ctrl-C to stop)"
[[ -n "$ROBOT_TOKEN" ]] && echo "auth token required (?token=…)" || echo "no auth (localhost only)"
exec "$ROOT/.venv/bin/python" -m uvicorn web.server:app --host "$HOST" --port 8080
