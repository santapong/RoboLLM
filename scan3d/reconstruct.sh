#!/usr/bin/env bash
# reconstruct.sh — photogrammetry path (higher fidelity than the visual hull).
#
# Uses COLMAP to recover camera poses + a 3D point cloud from your captured images.
# Sparse reconstruction (Structure-from-Motion) runs on the CPU. The DENSE step
# (a real surface) needs an NVIDIA GPU / CUDA — so on this laptop run sparse here,
# and do dense + meshing on your GCP/AWS GPU box. This script does what your
# hardware allows and tells you the next step.
#
#   sudo apt install colmap        # one-time (CPU build is fine for sparse)
#   ./reconstruct.sh <session>     # session = folder under ../assets/scan/
set -e
SESSION="${1:-object}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESS="$HERE/../assets/scan/$SESSION"
IMAGES="$SESS/images"
DB="$SESS/colmap.db"
SPARSE="$SESS/sparse"

[[ -d "$IMAGES" ]] || { echo "no images at $IMAGES — run capture.py first"; exit 1; }
command -v colmap >/dev/null || { echo "colmap not installed → sudo apt install colmap"; exit 1; }

mkdir -p "$SPARSE"
echo "[1/3] feature extraction"
colmap feature_extractor --database_path "$DB" --image_path "$IMAGES" \
  --ImageReader.single_camera 1

echo "[2/3] matching"
colmap exhaustive_matcher --database_path "$DB"

echo "[3/3] sparse reconstruction (Structure-from-Motion)"
colmap mapper --database_path "$DB" --image_path "$IMAGES" --output_path "$SPARSE"

echo
echo "Sparse model → $SPARSE   (view: colmap gui --import_path $SPARSE/0 --database_path $DB --image_path $IMAGES)"
cat <<EOF

DENSE mesh needs CUDA. On your GPU cloud instance, continue with:
  colmap image_undistorter --image_path "$IMAGES" --input_path "$SPARSE/0" --output_path "$SESS/dense"
  colmap patch_match_stereo --workspace_path "$SESS/dense"
  colmap stereo_fusion --workspace_path "$SESS/dense" --output_path "$SESS/dense/fused.ply"
  colmap poisson_mesher --input_path "$SESS/dense/fused.ply" --output_path "$SESS/${SESSION}_photo.ply"
Then back here:  python3 mesh_to_urdf.py ../assets/scan/$SESSION/${SESSION}_photo.ply --name $SESSION
EOF
