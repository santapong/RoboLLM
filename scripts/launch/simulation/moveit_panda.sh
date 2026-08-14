#!/usr/bin/env bash
# MoveIt: bring up the Franka Panda arm with move_group + RViz MotionPlanning.
# This is a self-contained demo (its own fake controllers) — no Gazebo needed.
# Run in its own terminal, then:
#   - drag the arm's goal marker in RViz and click "Plan & Execute", OR
#   - .venv/bin/python examples/ros2_py/07_moveit_joint_goal.py --pose ready
set -e
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"

echo "Launching Panda MoveIt demo (move_group + RViz)…"
echo "Then run: .venv/bin/python examples/ros2_py/07_moveit_joint_goal.py --pose ready"
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
