#!/usr/bin/env bash
# One command to bring up the core loop: TurtleBot3 sim + browser dashboard,
# each in its own terminal window. Needs a desktop session (a display).
#
#   ./launch_all.sh                 # sim + web dashboard
#   ./launch_all.sh --nav ~/map.yaml  # also start Nav2 with a map
#
# Then open http://localhost:8080 and/or talk to Claude.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

open_term() {  # $1 = title, $2 = command to run
  if command -v gnome-terminal >/dev/null; then
    gnome-terminal --title="$1" -- bash -c "$2; exec bash"
  elif command -v konsole >/dev/null; then
    konsole -p tabtitle="$1" -e bash -c "$2; exec bash" &
  elif command -v xfce4-terminal >/dev/null; then
    xfce4-terminal --title="$1" -x bash -c "$2; exec bash" &
  elif command -v xterm >/dev/null; then
    xterm -T "$1" -e bash -c "$2; exec bash" &
  else
    echo "!! No known terminal emulator found. Run this yourself:"
    echo "   $2"
  fi
}

NAV_MAP=""
[[ "$1" == "--nav" ]] && NAV_MAP="${2:-$HOME/map.yaml}"

echo "Launching TurtleBot3 sim…"
open_term "TB3 sim" "$ROOT/sim/launch_turtlebot.sh"
sleep 4   # give Gazebo a head start so the dashboard sees topics immediately

echo "Launching web dashboard → http://localhost:8080"
open_term "dashboard" "$ROOT/web/run-web.sh"

if [[ -n "$NAV_MAP" ]]; then
  sleep 2
  echo "Launching Nav2 with map: $NAV_MAP"
  open_term "nav2" "$ROOT/sim/launch_nav2.sh '$NAV_MAP'"
fi

cat <<EOF

Up and running (in separate terminals):
  • Gazebo sim
  • Dashboard  → http://localhost:8080
$( [[ -n "$NAV_MAP" ]] && echo "  • Nav2 (set the 2D Pose Estimate in RViz)" )

Not started here (run when you want them):
  • SLAM / build a map :  $ROOT/sim/launch_slam.sh
  • Robot arm (MoveIt) :  $ROOT/sim/launch_moveit_panda.sh
EOF
