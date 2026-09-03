#!/usr/bin/env bash
# Evaluate every checkpoint of one run on the frozen suite (nominal), then the
# variations and probes on the run's final checkpoint; then select by success.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd); BED=$(cd "$HERE/.." && pwd); ROOT=$(cd "$BED/../.." && pwd)
source "$HERE/_common.sh"; parse_args "$@"
[[ -n "$RUN" ]] || { echo "--run NAME is required" >&2; exit 2; }
CKPTS=("$ROOT"/artifacts/vla-bed/"$RUN"/full/checkpoints/*/pretrained_model)
[[ -d "${CKPTS[0]}" ]] || { echo "no checkpoints under artifacts/vla-bed/$RUN/full/checkpoints" >&2; exit 1; }
PY=python3
for CK in "${CKPTS[@]}"; do
  STEP=$(basename "$(dirname "$CK")")
  run_or_print "$PY" "$BED/evaluate.py" --policy smolvla --run "$RUN" --checkpoint "$CK" --label "$RUN/$STEP" --variation nominal
done
LAST=${CKPTS[-1]}; STEP=$(basename "$(dirname "$LAST")")
for VAR in camera_shift lighting target_relocation; do
  run_or_print "$PY" "$BED/evaluate.py" --policy smolvla --run "$RUN" --checkpoint "$LAST" --label "$RUN/$STEP" --variation "$VAR"
done
run_or_print "$PY" "$BED/evaluate.py" --policy smolvla --run "$RUN" --checkpoint "$LAST" --label "$RUN/$STEP" --variation nominal --blank-image
run_or_print "$PY" "$BED/evaluate.py" --policy smolvla --run "$RUN" --checkpoint "$LAST" --label "$RUN/$STEP" --variation nominal --gain 0.61
SEL=("$PY" "$HERE/select_checkpoint.py" --run "$RUN"); ((EXECUTE)) && SEL+=(--execute)
"${SEL[@]}"
