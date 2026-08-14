#!/usr/bin/env bash
# Compatibility launcher. Prefer scripts/launch/simulation/slam.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/scripts/launch/simulation/slam.sh" "$@"
