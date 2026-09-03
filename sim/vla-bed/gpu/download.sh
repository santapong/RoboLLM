#!/usr/bin/env bash
# Pull evaluation results and the checkpoints' pretrained_model directories back to the workstation.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd); ROOT=$(cd "$HERE/../../.." && pwd)
source "$HERE/_common.sh"; parse_args "$@"; require_host
SSH="ssh $(ssh_opts)"
run_or_print rsync -av -e "$SSH" "$HOST:$REMOTE_ROOT/sim/vla-bed/results/p5/" "$ROOT/sim/vla-bed/results/p5/"
run_or_print rsync -av -e "$SSH" --include '*/' --include 'pretrained_model/**' --include 'run_record.json' --include 'train_config.json' --exclude '*' "$HOST:$REMOTE_ROOT/artifacts/vla-bed/" "$ROOT/artifacts/vla-bed/"
