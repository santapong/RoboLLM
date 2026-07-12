#!/usr/bin/env bash
# Launch TurtleBot3 in Gazebo so the LLM has a robot to drive.
# Run this in its OWN terminal (it needs a display and stays running).
set -e
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

echo "Launching TurtleBot3 ($TURTLEBOT3_MODEL) in Gazebo..."
echo "Leave this running, then talk to Claude Code in another terminal."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
