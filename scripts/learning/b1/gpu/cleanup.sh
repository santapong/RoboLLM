#!/usr/bin/env bash
set -euo pipefail
HOST=""
REMOTE_ROOT=""
EXECUTE=0
while (($#)); do
  case "$1" in
    --host) HOST=${2:?}; shift 2 ;;
    --remote-root) REMOTE_ROOT=${2:?}; shift 2 ;;
    --execute) EXECUTE=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$HOST" =~ ^[A-Za-z0-9._@-]+$ ]] || { echo "invalid host" >&2; exit 2; }
[[ "$REMOTE_ROOT" =~ ^/[A-Za-z0-9._/-]+$ && "$REMOTE_ROOT" != "/" ]] || {
  echo "usage: $0 --host USER@HOST --remote-root ABSOLUTE_PATH [--execute]" >&2; exit 2;
}
# Recoverable cleanup: move the scoped remote project into its sibling trash.
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
PARENT=${REMOTE_ROOT%/*}
NAME=${REMOTE_ROOT##*/}
COMMAND=(ssh "$HOST" -- mkdir -p "$PARENT/.robollm-trash" '&&' mv "$REMOTE_ROOT" "$PARENT/.robollm-trash/$NAME-$STAMP")
if ((EXECUTE == 0)); then
  printf 'DRY-RUN:'; printf ' %q' "${COMMAND[@]}"; printf '\n'; exit 0
fi
"${COMMAND[@]}"
