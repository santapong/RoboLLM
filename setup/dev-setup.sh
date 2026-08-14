#!/usr/bin/env bash
# Compatibility launcher. Prefer scripts/setup/dev-environment.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/scripts/setup/dev-environment.sh" "$@"
