#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
exec python3 "$ROOT/scripts/learning/b1/gpu/train.py" full "$@"
