#!/usr/bin/env bash
# The priced run for one entry of config.json (preflight first).
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$HERE/_common.sh"; parse_args "$@"
[[ -n "$RUN" ]] || { echo "--run NAME is required" >&2; exit 2; }
if ((EXECUTE)); then python3 "$HERE/preflight.py" --execute; fi
CMD=(python3 "$HERE/train.py" --run "$RUN" --mode full)
((EXECUTE)) && CMD+=(--execute)
"${CMD[@]}"
