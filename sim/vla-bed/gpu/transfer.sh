#!/usr/bin/env bash
# Sync the repo (no .git, venvs, artifacts) and the v2 dataset to the rented host.
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd); ROOT=$(cd "$HERE/../../.." && pwd)
source "$HERE/_common.sh"; parse_args "$@"; require_host
SSH="ssh $(ssh_opts)"
run_or_print rsync -av --delete-delay -e "$SSH" --exclude .git --exclude '.venv*' --exclude artifacts --exclude datasets --exclude '__pycache__' "$ROOT/" "$HOST:$REMOTE_ROOT/"
run_or_print rsync -av -e "$SSH" "$ROOT/datasets/vla-bed/v2/" "$HOST:$REMOTE_ROOT/datasets/vla-bed/v2/"
