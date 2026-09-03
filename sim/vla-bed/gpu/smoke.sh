#!/usr/bin/env bash
# 10 GPU steps at batch 2 for one run: proves the stack, measures it/s and VRAM before anything priced.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$HERE/_common.sh"; parse_args "$@"
RUN=${RUN:-baseline}
CMD=(python3 "$HERE/train.py" --run "$RUN" --mode smoke)
((EXECUTE)) && CMD+=(--execute)
"${CMD[@]}"
