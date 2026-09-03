#!/usr/bin/env bash
# Import a Kaggle notebook's output zip (results/p5 + the selected checkpoint) into the repo layout.
#   sim/vla-bed/gpu/kaggle_import.sh ~/Downloads/vla-bed-output.zip [EXPECTED_SHA256]
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
ZIP=${1:?usage: kaggle_import.sh OUTPUT.zip [SHA256]}; WANT=${2:-}
GOT=$(sha256sum "$ZIP" | cut -d' ' -f1)
if [[ -n "$WANT" && "$GOT" != "$WANT" ]]; then echo "sha256 mismatch: got $GOT want $WANT" >&2; exit 1; fi
echo "sha256 $GOT"
TMP=$(mktemp -d); unzip -q "$ZIP" -d "$TMP"
[[ -d "$TMP/results/p5" ]] && { mkdir -p "$ROOT/sim/vla-bed/results/p5"; cp -r "$TMP/results/p5/." "$ROOT/sim/vla-bed/results/p5/"; echo "results → sim/vla-bed/results/p5/$(ls "$TMP/results/p5")"; }
[[ -d "$TMP/artifacts" ]] && { mkdir -p "$ROOT/artifacts/vla-bed"; cp -r "$TMP/artifacts/." "$ROOT/artifacts/vla-bed/"; echo "checkpoints → artifacts/vla-bed/ (git-ignored)"; }
[[ -f "$TMP/run_record.json" ]] && { cat "$TMP/run_record.json" | head -30; }
rm -rf "$TMP"
