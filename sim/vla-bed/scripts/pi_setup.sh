#!/usr/bin/env bash
# Phase 0 setup on the Raspberry Pi (or any host): Python 3.13 venv with the
# pinned sim stack, and MuJoCo Menagerie at the pinned commit (sparse: only the
# UR5e and 2F-85 model directories). Nothing upstream is modified.
#
#   bash scripts/pi_setup.sh            # from the sim/vla-bed directory
#   VLA_BED_PYTHON=/usr/bin/python3.13 bash scripts/pi_setup.sh
set -euo pipefail

BED_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${VLA_BED_PYTHON:-python3}"
VENV="${VLA_BED_VENV:-$BED_DIR/.venv}"
MENAGERIE_DIR="$BED_DIR/assets/mujoco_menagerie"
MENAGERIE_COMMIT="e4049d0a3bfd58d2a3081614e6777d4007e3f86a"

echo "== python: $("$PY" --version) at $(command -v "$PY")"
case "$("$PY" -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")')" in
  3.12|3.13) ;;
  *) echo "need Python 3.12 or 3.13 (mujoco has no 3.14 wheel yet); set VLA_BED_PYTHON" >&2; exit 2 ;;
esac

if [ ! -x "$VENV/bin/python" ]; then
  echo "== creating venv $VENV"
  "$PY" -m venv "$VENV"
fi
"$VENV/bin/python" -m pip install --quiet --upgrade pip
echo "== installing pins from requirements.txt"
"$VENV/bin/python" -m pip install --quiet -r "$BED_DIR/requirements.txt"

if [ ! -d "$MENAGERIE_DIR/.git" ]; then
  echo "== cloning MuJoCo Menagerie (sparse) into $MENAGERIE_DIR"
  mkdir -p "$(dirname "$MENAGERIE_DIR")"
  git clone --quiet --filter=blob:none --no-checkout \
    https://github.com/google-deepmind/mujoco_menagerie.git "$MENAGERIE_DIR"
  git -C "$MENAGERIE_DIR" sparse-checkout set universal_robots_ur5e robotiq_2f85
fi
git -C "$MENAGERIE_DIR" checkout --quiet "$MENAGERIE_COMMIT"
echo "== menagerie at $(git -C "$MENAGERIE_DIR" log --oneline -1)"

echo "== GL libraries present:"
ls /usr/lib/*/libEGL.so.1 /usr/lib/*/libOSMesa.so.8 2>/dev/null || true
echo "   (EGL is the default; if the gate fails, apt install libosmesa6 and rerun with MUJOCO_GL=osmesa)"

"$VENV/bin/python" - <<'EOF'
import mujoco, mjviser, viser, mink, zmq
print(f"== ok: mujoco {mujoco.__version__}, viser {viser.__version__}, pyzmq {zmq.__version__}")
EOF
echo "== next: MUJOCO_GL=egl $VENV/bin/python $BED_DIR/p0_gate.py"
