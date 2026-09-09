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
selected_model="g1"
expect_model=0
for argument in "$@"; do
  if [[ "$expect_model" == 1 ]]; then
    selected_model="$argument"
    expect_model=0
  elif [[ "$argument" == --model ]]; then
    expect_model=1
  elif [[ "$argument" == --model=* ]]; then
    selected_model="${argument#--model=}"
  fi
done
case "$selected_model" in
  g1|h1)
    if [[ ! -f "$demo_dir/assets/unitree_$selected_model/provenance.json" ]]; then
      "$sim_python" "$demo_dir/fetch_model.py" --model "$selected_model"
    fi
    ;;
esac
exec "$sim_python" "$demo_dir/app.py" "$@"
