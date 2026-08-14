#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
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
[[ -n "$HOST" && -n "$REMOTE_ROOT" ]] || {
  echo "usage: $0 --host USER@HOST --remote-root ABSOLUTE_PATH [--execute]" >&2
  exit 2
}
[[ "$HOST" =~ ^[A-Za-z0-9._@-]+$ ]] || { echo "invalid host" >&2; exit 2; }
[[ "$REMOTE_ROOT" =~ ^/[A-Za-z0-9._/-]+$ && "$REMOTE_ROOT" != "/" ]] || {
  echo "remote root must be an absolute, non-root path" >&2; exit 2;
}
COMMAND=(rsync -av --delete-delay --exclude .git --exclude '.venv*' --exclude artifacts "$ROOT/" "$HOST:$REMOTE_ROOT/")
if ((EXECUTE == 0)); then
  printf 'DRY-RUN:'; printf ' %q' "${COMMAND[@]}"; printf '\n'
  exit 0
fi
"${COMMAND[@]}"
