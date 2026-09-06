#!/usr/bin/env bash
# Start / stop / status of the bed's ZeroMQ sim server on this machine (SDD §6.4, P5). Run on the Pi:
#   sim/vla-bed/scripts/sim-server.sh start [tcp://100.74.8.82:5555]   # detached (setsid), log in /tmp/sim_server.log
#   sim/vla-bed/scripts/sim-server.sh status | stop
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PY="$HERE/.venv/bin/python"; [[ -x "$PY" ]] || PY="$HERE/../../.venv-vla-bed/bin/python"
LOG=/tmp/sim_server.log
PAT="sim_server.py --bind"
case "${1:-status}" in
  start)
    if pgrep -f "$PAT" >/dev/null; then echo "already running: $(pgrep -af "$PAT" | head -1)"; exit 0; fi
    cd "$HERE" && MUJOCO_GL="${MUJOCO_GL:-egl}" setsid nohup "$PY" sim_server.py --bind "${2:-tcp://*:5555}" --persistent > "$LOG" 2>&1 < /dev/null &
    sleep 4; tail -3 "$LOG"; pgrep -af "$PAT" | head -1 ;;
  stop)   pkill -f "$PAT" && echo stopped || echo "not running" ;;
  status) pgrep -af "$PAT" | head -1 || echo "not running"; tail -3 "$LOG" 2>/dev/null ;;
  *) echo "usage: $0 start|stop|status [bind]"; exit 2 ;;
esac
