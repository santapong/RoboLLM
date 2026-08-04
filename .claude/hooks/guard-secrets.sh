#!/usr/bin/env bash
# PreToolUse guard — backstop to the deny permission rules for a PUBLIC repo.
# Blocks (exit 2) tool calls touching secrets or running destructive commands.
# Test: echo '{"tool_name":"Bash","tool_input":{"command":"rm -rf /"}}' | .claude/hooks/guard-secrets.sh; echo $?
set -euo pipefail
INPUT=$(cat)

tool=$(jq -r '.tool_name // ""' <<<"$INPUT")
cmd=$(jq -r '.tool_input.command // ""' <<<"$INPUT")
path=$(jq -r '.tool_input.file_path // ""' <<<"$INPUT")

block() { echo "Blocked by guard-secrets hook: $1" >&2; exit 2; }

SECRET_RE='(^|/)\.env|(^|/)\.ssh/|(^|/)\.aws/|(^|/)secrets/|id_rsa|\.pem$|github-recovery'
DANGER_RE='rm[[:space:]]+-rf[[:space:]]+[/~]|sudo[[:space:]]|:\(\)\{|mkfs|dd[[:space:]]+if='

if [[ -n "$path" && "$path" =~ $SECRET_RE ]]; then
  block "path '$path' looks like a secret/credential file (this repo is PUBLIC)"
fi

if [[ "$tool" == "Bash" && -n "$cmd" ]]; then
  [[ "$cmd" =~ $DANGER_RE ]] && block "command matches a destructive pattern"
  [[ "$cmd" =~ $SECRET_RE ]] && block "command references a secret/credential path"
fi

exit 0
