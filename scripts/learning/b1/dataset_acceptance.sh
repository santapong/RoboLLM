#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
OUTPUT_ROOT=${1:-datasets/b1-red-target}
MANIFEST="$OUTPUT_ROOT/manifest.json"
cd "$ROOT"
python3 examples/mujoco/reaching_dataset.py \
  --scenario red-target --split train --seed 10000 --episodes 40 \
  --output-root "$OUTPUT_ROOT" --manifest "$MANIFEST"
python3 examples/mujoco/reaching_dataset.py \
  --scenario red-target --split evaluation --seed 10000 --episodes 10 \
  --output-root "$OUTPUT_ROOT" --manifest "$MANIFEST"
python3 examples/mujoco/reaching_dataset.py --manifest "$MANIFEST" --validate
