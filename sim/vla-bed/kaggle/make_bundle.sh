#!/usr/bin/env bash
# Zip one dataset recipe (train + evaluation, no leftover images/) for a one-time upload; a browser upload capped at 10 MB per file can use `split -b 9M -d -a 2 <zip> <dir>/<name>.zip.part` + `sha256sum > SHA256SUMS` (the notebooks re-join the parts)
# as a private Kaggle dataset named vla-bed-<recipe>. Prints size and sha256.
#   sim/vla-bed/kaggle/make_bundle.sh [v2|v3|v4|v5a|v5b]
set -euo pipefail
RECIPE="${1:-v2}"
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
SRC="$ROOT/datasets/vla-bed/$RECIPE"
OUT="$ROOT/artifacts/vla-bed/vla-bed-$RECIPE.zip"
[[ -f "$SRC/manifest.json" ]] || { echo "missing $SRC/manifest.json" >&2; exit 1; }
mkdir -p "$(dirname "$OUT")"; rm -f "$OUT"
( cd "$ROOT/datasets/vla-bed" && zip -qr "$OUT" "$RECIPE/manifest.json" "$RECIPE/train/data" "$RECIPE/train/meta" "$RECIPE/train/videos" "$RECIPE/evaluation/data" "$RECIPE/evaluation/meta" "$RECIPE/evaluation/videos" )
ls -l "$OUT" | awk '{print $5" bytes"}'
sha256sum "$OUT"
echo "upload $OUT on kaggle.com → Datasets → New Dataset (private), title: vla-bed-$RECIPE"
