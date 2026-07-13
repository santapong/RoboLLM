#!/usr/bin/env bash
# Navigation: load a saved map and let the robot plan paths to goals.
# Run AFTER sim/launch_turtlebot.sh, in its own terminal.
#
#   sim/launch_nav2.sh [path/to/map.yaml]      # default: ~/map.yaml
#
# In the RViz window that opens:
#   1) click "2D Pose Estimate" and click+drag where the robot actually is
#      (this is REQUIRED — Nav2 needs to localize before it can navigate).
#   2) then either click "Nav2 Goal" to pick a target, OR ask Claude
#      ("navigate to x=1.5, y=0.5"), OR run examples/ros2_py/06_send_nav_goal.py.
set -e
MAP="${1:-$HOME/map.yaml}"
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

if [[ ! -f "$MAP" ]]; then
  echo "No map at '$MAP'. Build one first with sim/launch_slam.sh, then:"
  echo "  ros2 run nav2_map_server map_saver_cli -f ~/map"
  exit 1
fi
echo "Nav2 with map: $MAP   (set the 2D Pose Estimate in RViz first!)"
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:="$MAP"
