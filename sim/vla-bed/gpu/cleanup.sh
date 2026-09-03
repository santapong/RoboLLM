#!/usr/bin/env bash
# Recoverable cleanup: move the remote project into a sibling trash directory (nothing is deleted).
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$HERE/_common.sh"; parse_args "$@"; require_host
STAMP=$(date -u +%Y%m%dT%H%M%SZ); PARENT=${REMOTE_ROOT%/*}; NAME=${REMOTE_ROOT##*/}
# shellcheck disable=SC2046
run_or_print ssh $(ssh_opts) "$HOST" -- mkdir -p "$PARENT/.robollm-trash" '&&' mv "$REMOTE_ROOT" "$PARENT/.robollm-trash/$NAME-$STAMP"
