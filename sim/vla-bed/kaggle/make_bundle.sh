#!/usr/bin/env bash
# Zip the v2 dataset (train + evaluation, no leftover images/) for a one-time upload
# as a private Kaggle dataset named vla-bed-v2. Prints size and sha256.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
SRC="$ROOT/datasets/vla-bed/v2"
OUT="$ROOT/artifacts/vla-bed/vla-bed-v2.zip"
[[ -f "$SRC/manifest.json" ]] || { echo "missing $SRC/manifest.json" >&2; exit 1; }
mkdir -p "$(dirname "$OUT")"; rm -f "$OUT"
( cd "$ROOT/datasets/vla-bed" && zip -qr "$OUT" v2/manifest.json v2/train/data v2/train/meta v2/train/videos v2/evaluation/data v2/evaluation/meta v2/evaluation/videos )
ls -l "$OUT" | awk '{print $5" bytes"}'
sha256sum "$OUT"
echo "upload $OUT on kaggle.com → Datasets → New Dataset (private), title: vla-bed-v2"
