#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
CHECKPOINT=""
EXECUTE=0
while (($#)); do
  case "$1" in
    --checkpoint) CHECKPOINT=${2:?}; shift 2 ;;
    --execute) EXECUTE=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$CHECKPOINT" ]] || { echo "--checkpoint is required" >&2; exit 2; }
for SUITE in nominal camera_shift lighting occlusion target_relocation; do
  COMMAND=(python3 "$ROOT/examples/mujoco/evaluate_reaching.py" --policy-adapter smolvla --suite "$SUITE" --episodes 20 --seed 70000 --checkpoint "$CHECKPOINT" --compact --json-output "$ROOT/artifacts/b1-results/$SUITE.json")
  if ((EXECUTE == 0)); then
    printf 'DRY-RUN:'; printf ' %q' "${COMMAND[@]}"; printf '\n'
  else
    "${COMMAND[@]}"
  fi
done
