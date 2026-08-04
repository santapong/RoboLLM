#!/usr/bin/env bash
# reconstruct_cpu.sh — Route C: full photogrammetry ON THIS LAPTOP, no CUDA.
#
# Same sparse SfM as reconstruct.sh (COLMAP), but the dense surface comes from
# OpenMVS, which is CPU-capable — so no cloud GPU box needed. Runs entirely in
# Docker; nothing to apt-install. Output feeds mesh_to_urdf.py like Route A/B.
#
#   ./reconstruct_cpu.sh <session>                 # images from capture.py
#   ./reconstruct_cpu.sh <session> <photos-dir>    # phone photos (ingested)
#   ./reconstruct_cpu.sh <session> <video.mp4>     # phone video -> 2 fps frames
#
# Phone tip: orbit a STATIC object (2-3 height rings, ~10° steps, >=60%
# overlap) — do NOT turntable-rotate it, moving backgrounds break matching.
# Expect ~10-40 min for 50 photos; DensifyPointCloud is the slow step.
set -euo pipefail

COLMAP_IMG=colmap/colmap:latest
OPENMVS_IMG=openmvs/openmvs-ubuntu:latest

SESSION="${1:-object}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESS="$HERE/../assets/scan/$SESSION"
IMAGES="$SESS/images"
mkdir -p "$IMAGES"

step() { echo -e "\n=== [$(date +%H:%M:%S)] $* ==="; }

# --- 0. Optional ingest of phone photos / video ----------------------------
if [[ $# -ge 2 ]]; then
  SRC=$2
  if [[ -f "$SRC" ]]; then
    step "extracting frames from video (2 fps)"
    ffmpeg -y -i "$SRC" -vf fps=2 -q:v 2 "$IMAGES/frame_%04d.jpg" -loglevel error
  elif [[ -d "$SRC" ]]; then
    step "copying photos"
    find "$SRC" -maxdepth 1 -type f \( -iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.png' \) -exec cp {} "$IMAGES/" \;
  else
    echo "error: $SRC is neither a file nor a directory" >&2; exit 1
  fi
fi

N=$(find "$IMAGES" -type f | wc -l)
[[ $N -gt 0 ]] || { echo "no images at $IMAGES — run capture.py or pass a photos dir"; exit 1; }
echo "$N images"
[[ $N -ge 20 ]] || echo "WARNING: <20 images — reconstruction may fail. Aim for 40-80."

CR() { docker run --rm -v "$SESS:/work" -w /work "$COLMAP_IMG" "$@"; }
MR() { docker run --rm -v "$SESS:/work" -w /work --entrypoint "" "$OPENMVS_IMG" "$@"; }

# --- 1. Sparse SfM (COLMAP, CPU SIFT) --------------------------------------
step "[1/7] COLMAP feature extraction"
CR colmap feature_extractor --database_path /work/colmap.db --image_path /work/images \
   --ImageReader.camera_model OPENCV --ImageReader.single_camera 1 \
   --SiftExtraction.use_gpu 0

step "[2/7] COLMAP matching"
CR colmap exhaustive_matcher --database_path /work/colmap.db --SiftMatching.use_gpu 0

step "[3/7] COLMAP mapping (sparse SfM)"
mkdir -p "$SESS/sparse"
CR colmap mapper --database_path /work/colmap.db --image_path /work/images --output_path /work/sparse

step "[4/7] COLMAP undistortion"
CR colmap image_undistorter --image_path /work/images --input_path /work/sparse/0 \
   --output_path /work/dense --output_type COLMAP

# --- 2. Dense + mesh (OpenMVS, CPU) ----------------------------------------
step "[5/7] OpenMVS densify (the slow one)"
MR /usr/local/bin/OpenMVS/InterfaceCOLMAP -w /work -i /work/dense -o /work/scene.mvs --image-folder /work/dense/images
MR /usr/local/bin/OpenMVS/DensifyPointCloud -w /work /work/scene.mvs -o /work/scene_dense.mvs

step "[6/7] OpenMVS mesh"
MR /usr/local/bin/OpenMVS/ReconstructMesh -w /work /work/scene_dense.mvs -o /work/scene_mesh.mvs

step "[7/7] OpenMVS texture"
MR /usr/local/bin/OpenMVS/TextureMesh -w /work /work/scene_dense.mvs -m /work/scene_mesh.ply -o "/work/${SESSION}_photo.mvs"

step "DONE"
ls -lh "$SESS"/*.ply
cat <<EOF

Final mesh: assets/scan/$SESSION/${SESSION}_photo.ply
Next:       ../.venv/bin/python mesh_to_urdf.py ../assets/scan/$SESSION/${SESSION}_photo.ply --name $SESSION
EOF
