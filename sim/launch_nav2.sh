#!/usr/bin/env bash
# Compatibility launcher. Prefer scripts/launch/simulation/nav2.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/scripts/launch/simulation/nav2.sh" "$@"
