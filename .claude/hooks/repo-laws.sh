#!/usr/bin/env bash
# PreToolUse guard for RoboLLM's two hard-won repo laws (see CLAUDE.md):
#   1. NEVER commit or push on main — work happens on develop / experiment/*.
#   2. numpy MUST stay 1.26.4 (ROS Jazzy ABI): any pip install into the repo
#      venv must carry -c constraints.txt, and numpy/opencv must not be
#      upgraded past the pins. (Scratch venvs outside the repo are exempt.)
# Also blocks gh — this machine's gh CLI is unauthenticated; use plain git.
# Test: echo '{"tool_name":"Bash","tool_input":{"command":"git commit -m x"}}' | .claude/hooks/repo-laws.sh
set -euo pipefail
INPUT=$(cat)

cmd=$(jq -r '.tool_input.command // ""' <<<"$INPUT")
[[ -z "$cmd" ]] && exit 0

block() { echo "Blocked by repo-laws hook: $1" >&2; exit 2; }

# Law 3: gh is unauthenticated on this machine
if [[ "$cmd" =~ (^|[[:space:];&|])gh[[:space:]] ]]; then
  block "gh CLI is unauthenticated here — use plain git (see memory: git-not-gh)"
fi

# Law 1: no commits/pushes while on main
if [[ "$cmd" =~ git[[:space:]]+(commit|push) ]]; then
  branch=$(git -C "${CLAUDE_PROJECT_DIR:-.}" branch --show-current 2>/dev/null || echo "")
  if [[ "$branch" == "main" ]]; then
    block "on 'main' — never commit/push to main; switch to develop or experiment/<topic> (docs/branching.md)"
  fi
fi

# Law 2: pip installs into the repo venv must pin via constraints.txt
if [[ "$cmd" =~ pip[[:space:]]+install ]] && [[ ! "$cmd" =~ constraints\.txt ]]; then
  # scratch venvs elsewhere are fine; repo venv or bare pip inside the repo is not
  if [[ "$cmd" =~ \.venv/bin/pip ]] || [[ ! "$cmd" =~ /tmp/ ]]; then
    block "pip install without -c constraints.txt — numpy must stay 1.26.4 (ROS Jazzy ABI; see CLAUDE.md). Add: -c \${CLAUDE_PROJECT_DIR}/constraints.txt"
  fi
fi

exit 0
