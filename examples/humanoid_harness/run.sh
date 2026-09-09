#!/usr/bin/env bash
set -euo pipefail
demo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_python="${ROBOLLM_SIM_PYTHON:-/home/santapong/RoboLLM/.venv-vla-bed/bin/python}"
if [[ ! -x "$sim_python" ]]; then
  echo "Set ROBOLLM_SIM_PYTHON to a Python environment with mujoco, numpy, Pillow, viser and mjviser." >&2
  exit 1
fi
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MUJOCO_GL="${MUJOCO_GL:-egl}"
if [[ ! -f "$demo_dir/assets/unitree_g1/provenance.json" ]]; then
  "$sim_python" "$demo_dir/fetch_model.py"
fi
exec "$sim_python" "$demo_dir/app.py" "$@"
