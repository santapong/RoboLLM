# shared argument parsing for the bed's GPU session scripts (sourced, not executed)
HOST=""; PORT=""; REMOTE_ROOT=""; EXECUTE=0; RUN=""
parse_args() {
  while (($#)); do
    case "$1" in
      --host) HOST=${2:?}; shift 2 ;;
      --port) PORT=${2:?}; shift 2 ;;
      --remote-root) REMOTE_ROOT=${2:?}; shift 2 ;;
      --run) RUN=${2:?}; shift 2 ;;
      --execute) EXECUTE=1; shift ;;
      *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
  done
}
require_host() {
  [[ -n "$HOST" && -n "$REMOTE_ROOT" ]] || { echo "usage: $0 --host USER@HOST [--port N] --remote-root ABSOLUTE_PATH [--execute]" >&2; exit 2; }
  [[ "$HOST" =~ ^[A-Za-z0-9._@-]+$ ]] || { echo "invalid host" >&2; exit 2; }
  [[ -z "$PORT" || "$PORT" =~ ^[0-9]{1,5}$ ]] || { echo "invalid port" >&2; exit 2; }
  [[ "$REMOTE_ROOT" =~ ^/[A-Za-z0-9._/-]+$ && "$REMOTE_ROOT" != "/" ]] || { echo "remote root must be an absolute, non-root path" >&2; exit 2; }
}
ssh_opts() { if [[ -n "$PORT" ]]; then echo "-p $PORT"; fi; }
run_or_print() {  # run_or_print CMD...
  if ((EXECUTE == 0)); then printf 'DRY-RUN:'; printf ' %q' "$@"; printf '\n'; else "$@"; fi
}
