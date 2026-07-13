#!/usr/bin/env bash
# SLAM: build a map of the world while you drive the robot around.
# Run this AFTER sim/launch_turtlebot.sh, in its own terminal.
#
# Then drive the robot (web dashboard at :8080, or teleop, or ask Claude) to
# explore every wall. Watch the map fill in RViz. When it looks complete, save it:
#
#     ros2 run nav2_map_server map_saver_cli -f ~/map
#
# That writes ~/map.yaml + ~/map.pgm — feed it to sim/launch_nav2.sh.
set -e
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

echo "SLAM (cartographer) — drive around to build the map, then:"
echo "  ros2 run nav2_map_server map_saver_cli -f ~/map"
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
