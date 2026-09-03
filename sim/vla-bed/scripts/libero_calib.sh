#!/usr/bin/env bash
# P2b — LIBERO-Spatial calibration of the SmolVLA evaluation plumbing on CPU (SDD §8).
#
# Uses the official lerobot/smolvla_libero checkpoint (Apache-2.0) in the separate
# .venv-libero (hf-libero pins mujoco<3.10, so it must not share the bed's venv).
# The paper (2506.01844, Table 2) reports Spatial 90 % at 10 trials per task with a
# new chunk predicted after every executed action on GPU; here n_action_steps=10
# (Table 13 shows 10 ≥ 1 in its ablation) because a chunk costs ~36 s on this CPU.
#
#   bash sim/vla-bed/scripts/libero_calib.sh probe            # 1 episode, task 0 — time it first
#   bash sim/vla-bed/scripts/libero_calib.sh full [EPISODES]  # all 10 tasks, EPISODES per task (default 10)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
VENV="${LIBERO_VENV:-$ROOT/.venv-libero}"
OUT="${LIBERO_OUT:-$ROOT/sim/vla-bed/results/p2b}"
CKPT="${LIBERO_CKPT:-lerobot/smolvla_libero}"
SUITE="${LIBERO_SUITE:-libero_spatial}"
N_ACTION_STEPS="${LIBERO_N_ACTION_STEPS:-10}"
MODE="${1:-probe}"
EPISODES="${2:-10}"

mkdir -p "$OUT"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export TOKENIZERS_PARALLELISM=false

common=(
  --env.type=libero
  --env.task="$SUITE"
  --policy.path="$CKPT"
  --policy.device=cpu
  --policy.n_action_steps="$N_ACTION_STEPS"
  --eval.batch_size=1
  --eval.use_async_envs=false
  # The LeRobot LIBERO checkpoints expect camera1..3; the env exposes agentview + wrist.
  # Mapping them onto camera1/camera2 satisfies the subset rule and SmolVLA masks camera3.
  --env.camera_name_mapping="${LIBERO_CAMERA_MAP:-{\"agentview_image\": \"camera1\", \"robot0_eye_in_hand_image\": \"camera2\"}}"
)

case "$MODE" in
  probe)
    echo "== probe: 1 episode of $SUITE task 0, n_action_steps=$N_ACTION_STEPS"
    start=$(date +%s)
    "$VENV/bin/lerobot-eval" "${common[@]}" --env.task_ids='[0]' --eval.n_episodes=1 \
      --output_dir="$OUT/probe" 2>&1 | tee "$OUT/probe.log" | grep -v "WARNING\|SyntaxWarning" | tail -40
    echo "probe wall time: $(( $(date +%s) - start )) s (includes checkpoint/asset downloads on first run)" | tee -a "$OUT/probe.log"
    ;;
  full)
    echo "== full: $SUITE, $EPISODES episodes per task, n_action_steps=$N_ACTION_STEPS (started $(date -Is))"
    nohup "$VENV/bin/lerobot-eval" "${common[@]}" --eval.n_episodes="$EPISODES" \
      --output_dir="$OUT/full" > "$OUT/full.log" 2>&1 < /dev/null &
    echo "pid $! → $OUT/full.log ; results in $OUT/full/eval_info.json when done"
    ;;
  *)
    echo "usage: $0 {probe|full} [episodes-per-task]" >&2; exit 2 ;;
esac
