#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
CHECKPOINT=""
STEP=""
EXECUTE=0
while (($#)); do
  case "$1" in
    --checkpoint) CHECKPOINT=${2:?}; shift 2 ;;
    --step) STEP=${2:?}; shift 2 ;;
    --execute) EXECUTE=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$CHECKPOINT" ]] || { echo "--checkpoint is required" >&2; exit 2; }

# select_checkpoint.py reads <results-root>/<step:06d>/<suite>.json, so each
# candidate must land in its own directory. lerobot-train writes checkpoints as
# .../checkpoints/<step:06d>/pretrained_model, so derive the step from the path
# unless it was given explicitly.
if [[ -z "$STEP" ]]; then
  for PART in $(tr '/' ' ' <<<"$CHECKPOINT"); do
    [[ "$PART" =~ ^[0-9]{6}$ ]] && STEP=$PART
  done
fi
[[ "$STEP" =~ ^[0-9]+$ ]] || {
  echo "could not derive a training step from --checkpoint; pass --step N" >&2
  exit 2
}
# 10# is required: checkpoint directories are zero-padded, and printf would
# otherwise read "010000" as octal and write results into 004096.
OUT_DIR=$(printf '%s/artifacts/b1-results/by-checkpoint/%06d' "$ROOT" "$((10#$STEP))")

for SUITE in nominal camera_shift lighting occlusion target_relocation; do
  COMMAND=(python3 "$ROOT/examples/mujoco/evaluate_reaching.py" --policy-adapter smolvla --suite "$SUITE" --episodes 20 --seed 70000 --checkpoint "$CHECKPOINT" --compact --json-output "$OUT_DIR/$SUITE.json")
  if ((EXECUTE == 0)); then
    printf 'DRY-RUN:'; printf ' %q' "${COMMAND[@]}"; printf '\n'
  else
    mkdir -p "$OUT_DIR"
    "${COMMAND[@]}"
  fi
done
