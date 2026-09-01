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

# --init is NOT optional: without it the tool runs as PID 1, where the kernel ignores
# SIGABRT's default action, so a COLMAP/OpenMVS abort() prints its diagnostic, dumps a
# stack trace and then HANGS FOREVER instead of exiting. Verified: bad --path wedges >5 min
# without --init, exits 134 immediately with it. set -e cannot save you from a process that
# never returns.
CR() { docker run --rm --init -v "$SESS:/work" -w /work "$COLMAP_IMG" "$@"; }
MR() { docker run --rm --init -v "$SESS:/work" -w /work --entrypoint "" "$OPENMVS_IMG" "$@"; }

# --- 1. Sparse SfM (COLMAP, CPU SIFT) --------------------------------------
step "[1/7] COLMAP feature extraction"
# Route E (turntable, fixed camera): if masks/ exists, tell COLMAP to ignore the
# static room. Without this a fixed camera reconstructs the room and discards the
# rotating object as a moving outlier. Absent masks/, the phone-orbit path below
# is byte-identical to what it always was.
MASK_ARGS=()
if [[ -d "$SESS/masks" ]] && compgen -G "$SESS/masks/*.png" >/dev/null; then
  NM=$(find "$SESS/masks" -name '*.png' | wc -l)
  echo "Route E: $NM masks found → --ImageReader.mask_path"
  [[ $NM -eq $N ]] || echo "WARNING: $NM masks for $N images — COLMAP silently ignores an image whose mask is missing"
  MASK_ARGS=(--ImageReader.mask_path /work/masks)
fi
CR colmap feature_extractor --database_path /work/colmap.db --image_path /work/images \
   --ImageReader.camera_model OPENCV --ImageReader.single_camera 1 \
   "${MASK_ARGS[@]}" \
   --SiftExtraction.use_gpu 0

step "[2/7] COLMAP matching"
CR colmap exhaustive_matcher --database_path /work/colmap.db --SiftMatching.use_gpu 0

step "[3/7] COLMAP mapping (sparse SfM)"
mkdir -p "$SESS/sparse"
CR colmap mapper --database_path /work/colmap.db --image_path /work/images --output_path /work/sparse

step "[3b/7] export text model + solve metric scale (ChArUco mat, best-effort)"
mkdir -p "$SESS/sparse_txt"
CR colmap model_converter --input_path /work/sparse/0 --output_path /work/sparse_txt --output_type TXT
# System python3 is not guaranteed to carry cv2 (it did not, after the 3.14
# rolling upgrade), and silently skipping this step costs the ChArUco metric
# scale that the S1 accuracy gate depends on. Search instead of assuming.
SCALE_PY=""
for CAND in "${SCAN3D_PYTHON:-}" "$HERE/../.venv/bin/python" "$HERE/../.venv-lerobot/bin/python" python3; do
  [[ -n "$CAND" ]] || continue
  if command -v "$CAND" >/dev/null 2>&1 && "$CAND" -c 'import cv2, numpy' 2>/dev/null; then
    SCALE_PY="$CAND"; break
  fi
done
if [[ -n "$SCALE_PY" ]]; then
  echo "scale solve using: $SCALE_PY"
  "$SCALE_PY" "$HERE/scale_mat.py" solve --session "$SESS" \
    || echo "no scale mat found — pass --height-mm to mesh_to_print.py instead"
else
  echo "WARNING: no interpreter with cv2 found (tried \$SCAN3D_PYTHON, ../.venv, ../.venv-lerobot, python3)."
  echo "         Skipping metric scale — you must pass --height-mm to mesh_to_print.py."
fi

step "[4/7] COLMAP undistortion"
CR colmap image_undistorter --image_path /work/images --input_path /work/sparse/0 \
   --output_path /work/dense --output_type COLMAP

# --- 2. Dense + mesh (OpenMVS, CPU) ----------------------------------------
step "[5/7] OpenMVS densify (the slow one)"
MR /usr/local/bin/OpenMVS/InterfaceCOLMAP -w /work -i /work/dense -o /work/scene.mvs --image-folder /work/dense/images
MR /usr/local/bin/OpenMVS/DensifyPointCloud -w /work /work/scene.mvs -o /work/scene_dense.mvs

if [[ "${MESHER:-openmvs}" == "poisson" ]]; then
  step "[6/7] Screened Poisson mesh (MESHER=poisson, Open3D)"
  docker build -q -t scan3d/poisson -f "$HERE/poisson.Dockerfile" "$HERE" >/dev/null
  docker run --rm --init -v "$SESS:/work" scan3d/poisson \
      /work/scene_dense.ply -o /work/scene_mesh.ply ${POISSON_ARGS:-}
else
  step "[6/7] OpenMVS mesh (MESHER=poisson for the Screened Poisson alternative)"
  MR /usr/local/bin/OpenMVS/ReconstructMesh -w /work /work/scene_dense.mvs -o /work/scene_mesh.mvs
fi

step "[7/7] OpenMVS texture"
MR /usr/local/bin/OpenMVS/TextureMesh -w /work /work/scene_dense.mvs -m /work/scene_mesh.ply -o "/work/${SESSION}_photo.mvs"

step "DONE"
ls -lh "$SESS"/*.ply
cat <<EOF

Final mesh: assets/scan/$SESSION/${SESSION}_photo.ply
Next:       ../.venv/bin/python mesh_to_urdf.py ../assets/scan/$SESSION/${SESSION}_photo.ply --name $SESSION
EOF
