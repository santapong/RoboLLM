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
echo "Dashboard → http://localhost:8080   (Ctrl-C to stop)"
exec "$ROOT/.venv/bin/python" -m uvicorn web.server:app --host 0.0.0.0 --port 8080
