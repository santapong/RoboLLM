#!/usr/bin/env bash
# run_scenario.sh — drive one container-CI acceptance scenario end to end.
#
#   scripts/ci/run_scenario.sh <scenario>
#
# Starts the matching example stack detached in-container using the example's
# own docker/ros2-arm launcher (rviz:=false, --net/--ipc host already provided
# by the launcher), pins ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1 (both are
# forwarded into the container; on this host Fast DDS silently drops traffic
# unless BOTH the stack and the meter share ROS_LOCALHOST_ONLY=1), polls
# readiness via `ros2 topic list`, runs the matching acceptance meter inside
# the container under `timeout`, parses the meter's RESULT:{json} verdict,
# tears the stack down, and exits 0 (pass) / 1 (fail) / 2 (bad usage).
#
# Scenarios:
#   wallweld-full          synthetic full weld           -> wallweld_accept full
#   wallweld-abort         synthetic mid-weld palm abort  -> wallweld_accept abort
#   wallweld-idle          35 s camera-less idle hold     -> wallweld_accept idle 35
#   pickplace-synthetic    scripted pick-and-place        -> pickplace_accept (attach+rate)
#   handfollow-synthetic   sine-sweep hand follow         -> hand_accept (RESULT line)
set -u

SCENARIO="${1:-}"
SELF="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$SELF/../.." && pwd)"

# Pinned DDS environment (overridable, defaults chosen for this box).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export ROS_LOCALHOST_ONLY=1

# Image used only for the post-run ownership fix-up below (the local tag the
# example launchers build/run). Overridable for CI, where the GHCR pull is
# retagged to this same name.
IMAGE="${IMAGE:-ros2-arm:jazzy}"

RESULTS_DIR="${RESULTS_DIR:-$REPO/artifacts/ci-results}"
mkdir -p "$RESULTS_DIR"

# ------------------------------------------------------------------ config
# Per scenario: example launcher dir, verb+args, container name, ready
# topics (all must appear), meter command (run in-container), and timeouts.
case "$SCENARIO" in
  wallweld-full)
    EXDIR="examples/wall_weld/docker"; CNAME="armwallweld"
    STACK="wallweld synthetic rviz:=false"
    READY="/weld_state /joint_states"; APPPUB="/weld_state"
    METER="python3 /ros2_ws/tools/wallweld_accept.py full"
    # wall_weld emits the (non-latched) PLAN event at preflight, before the
    # control tick exists; a meter that waits for readiness misses it and
    # cannot verify planned==bead waypoints.  Start the meter early so it is
    # subscribed to /weld_event before preflight (DDS late-join to the weld
    # node's publisher then delivers PLAN/START/DONE in order).
    EARLY_METER=1
    MTIMEOUT=320 ;;
  wallweld-abort)
    EXDIR="examples/wall_weld/docker"; CNAME="armwallweld"
    STACK="wallweld synthetic rviz:=false synthetic_abort_frac:=0.4"
    READY="/weld_state /joint_states"; APPPUB="/weld_state"
    METER="python3 /ros2_ws/tools/wallweld_accept.py abort"
    MTIMEOUT=260 ;;
  wallweld-idle)
    EXDIR="examples/wall_weld/docker"; CNAME="armwallweld"
    STACK="wallweld synthetic rviz:=false synth_start_sec:=999.0"
    READY="/weld_state /joint_states"; APPPUB="/weld_state"
    METER="python3 /ros2_ws/tools/wallweld_accept.py idle 35"
    MTIMEOUT=90 ;;
  pickplace-synthetic)
    EXDIR="examples/gen3_pick_place/docker"; CNAME="armpickplace"
    STACK="pickplace synthetic rviz:=false"
    READY="/joint_trajectory_controller/joint_trajectory /joint_states"
    APPPUB="/joint_trajectory_controller/joint_trajectory"
    METER="python3 /tmp/pickplace_accept.py 120"
    MTIMEOUT=150 ;;
  handfollow-synthetic)
    EXDIR="examples/hand_follow/docker"; CNAME="armhandfollow"
    STACK="handfollow synthetic rviz:=false"
    READY="/arm_controller/joint_trajectory /joint_states"
    APPPUB="/arm_controller/joint_trajectory"
    METER="python3 /ros2_ws/tools/hand_accept.py 20"
    MTIMEOUT=45 ;;
  *)
    echo "usage: $0 <scenario>" >&2
    echo "scenarios: wallweld-full wallweld-abort wallweld-idle" \
         "pickplace-synthetic handfollow-synthetic" >&2
    echo "unknown scenario: '${SCENARIO}'" >&2
    exit 2 ;;
esac

LAUNCHER="$REPO/$EXDIR/ros2-arm"
STACK_LOG="$RESULTS_DIR/${SCENARIO}.stack.log"
METER_LOG="$RESULTS_DIR/${SCENARIO}.meter.log"
RESULT_JSON="$RESULTS_DIR/${SCENARIO}.json"
RTIMEOUT="${RTIMEOUT:-300}"   # readiness budget (covers first-run colcon build)

if [ ! -x "$LAUNCHER" ]; then
  echo "launcher not found/executable: $LAUNCHER" >&2
  exit 2
fi

cleanup() {
  docker rm -f "$CNAME" >/dev/null 2>&1 || true
  # The in-container colcon build (and any core dumps) land in the mounted
  # checkout owned by root, because the containers run as root.  Left behind,
  # the runner's next `actions/checkout` does `git clean -ffdx` as the runner
  # user and fails EPERM on those dirs, so ci-container goes permanently red
  # from the second run on.  Restore ownership to the invoking user — run as
  # root inside the image, over the whole repo (idempotent; chowns nothing the
  # user does not already own back to itself).
  docker run --rm -e _U="$(id -u)" -e _G="$(id -g)" \
    -v "$REPO:/repo" "$IMAGE" \
    bash -lc 'chown -R "$_U":"$_G" /repo 2>/dev/null; true' >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "=== scenario: $SCENARIO (domain $ROS_DOMAIN_ID, localhost_only=$ROS_LOCALHOST_ONLY) ==="
docker rm -f "$CNAME" >/dev/null 2>&1 || true

# ------------------------------------------------------------- start stack
echo "-> launching stack: ros2-arm $STACK"
# shellcheck disable=SC2086
( cd "$REPO/$EXDIR" && exec ./ros2-arm $STACK ) >"$STACK_LOG" 2>&1 &

# wait for the named container to appear
for _ in $(seq 1 30); do
  docker ps --format '{{.Names}}' | grep -qx "$CNAME" && break
  sleep 1
done
if ! docker ps --format '{{.Names}}' | grep -qx "$CNAME"; then
  echo "FAIL: stack container '$CNAME' never started" >&2
  tail -20 "$STACK_LOG" 2>/dev/null >&2
  exit 1
fi

run_meter() {   # runs the in-container meter, streams to stdout + $METER_LOG
  docker exec "$CNAME" bash -lc \
    "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
     source install/setup.bash 2>/dev/null; \
     timeout ${MTIMEOUT} ${METER}" 2>&1 | tee "$METER_LOG"
  return "${PIPESTATUS[0]}"
}

if [ "${EARLY_METER:-0}" = 1 ]; then
  # ---------------------------------------------------- early-meter path
  # Subscribe before the app node preflights; the meter itself blocks on the
  # events it needs, so we do NOT gate on readiness here.
  echo "-> early meter (subscribes before app preflight): $METER"
  run_meter &
  mpid=$!
  # guard: if the stack container dies while the meter waits, bail out.
  while kill -0 "$mpid" 2>/dev/null; do
    if ! docker ps --format '{{.Names}}' | grep -qx "$CNAME"; then
      echo "FAIL: stack container exited while metering" >&2
      tail -25 "$STACK_LOG" 2>/dev/null >&2
      kill "$mpid" 2>/dev/null; wait "$mpid" 2>/dev/null
      exit 1
    fi
    sleep 2
  done
  wait "$mpid"; mstatus=$?
else
  # ------------------------------------------------------- readiness gate
  # Existence of $READY confirms controllers/graph are up, but the app node
  # (weld/handfollow/pickplace) is often a delayed launch action, so its
  # command topic can already be listed via the controller's *subscription*
  # before the node publishes anything.  Require the app node's own publisher
  # to be live on $APPPUB (Publisher count >= 1) before metering.
  echo "-> waiting for topics: $READY + publisher on $APPPUB (up to ${RTIMEOUT}s)"
  ready=0
  deadline=$(( $(date +%s) + RTIMEOUT ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if ! docker ps --format '{{.Names}}' | grep -qx "$CNAME"; then
      echo "FAIL: stack container exited during bring-up" >&2
      tail -25 "$STACK_LOG" 2>/dev/null >&2
      exit 1
    fi
    probe=$(docker exec "$CNAME" bash -lc \
      'source /opt/ros/jazzy/setup.bash 2>/dev/null;
       ros2 topic list 2>/dev/null;
       echo "___PUBINFO___";
       ros2 topic info '"$APPPUB"' 2>/dev/null')
    topics=${probe%%___PUBINFO___*}
    pubinfo=${probe#*___PUBINFO___}
    ok=1
    for t in $READY; do
      echo "$topics" | grep -qx "$t" || ok=0
    done
    pubn=$(echo "$pubinfo" | sed -n 's/.*[Pp]ublisher count: *\([0-9]*\).*/\1/p')
    [ -n "$pubn" ] && [ "$pubn" -ge 1 ] 2>/dev/null || ok=0
    if [ "$ok" = 1 ]; then ready=1; echo "   ready (publishers on $APPPUB: $pubn)."; break; fi
    sleep 3
  done
  if [ "$ready" != 1 ]; then
    echo "FAIL: readiness timeout (${RTIMEOUT}s) waiting for: $READY" >&2
    tail -25 "$STACK_LOG" 2>/dev/null >&2
    exit 1
  fi

  # inject the pickplace meter (no vendored copy in that tree)
  if [ "$SCENARIO" = "pickplace-synthetic" ]; then
    docker cp "$SELF/pickplace_accept.py" "$CNAME:/tmp/pickplace_accept.py" \
      >/dev/null 2>&1 || { echo "FAIL: could not copy pickplace meter" >&2; exit 1; }
  fi

  echo "-> running meter (timeout ${MTIMEOUT}s): $METER"
  run_meter; mstatus=$?
fi

# ------------------------------------------------------------- parse verdict
line=$(grep '^RESULT:' "$METER_LOG" | tail -1)
if [ -z "$line" ]; then
  echo "FAIL: meter emitted no RESULT line (exit $mstatus)" >&2
  echo '{"scenario":"'"$SCENARIO"'","pass":false,"error":"no RESULT line","meter_exit":'"$mstatus"'}' \
    > "$RESULT_JSON"
  echo "VERDICT $SCENARIO: FAIL"
  exit 1
fi
echo "${line#RESULT:}" > "$RESULT_JSON"

verdict=$(python3 - "$RESULT_JSON" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    print("pass" if d.get("pass") is True else "fail")
except Exception:
    print("fail")
PY
)

echo "----------------------------------------------------------------"
echo "RESULT JSON -> $RESULT_JSON"
cat "$RESULT_JSON"; echo
if [ "$verdict" = "pass" ] && [ "$mstatus" = 0 ]; then
  echo "VERDICT $SCENARIO: PASS"
  exit 0
fi
echo "VERDICT $SCENARIO: FAIL (verdict=$verdict meter_exit=$mstatus)"
exit 1
