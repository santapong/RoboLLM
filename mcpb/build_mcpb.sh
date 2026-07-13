#!/usr/bin/env bash
# Build ros2-bridge.mcpb — a self-contained MCP bundle you can install in
# Claude Desktop (drag-and-drop). It stages the server code + vendors the `mcp`
# Python SDK (rclpy comes from your ROS install at runtime), then zips it.
#
#   mcpb/build_mcpb.sh
# Output: mcpb/dist/ros2-bridge.mcpb
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
PY="$ROOT/.venv/bin/python"
STAGE="$HERE/build"
DIST="$HERE/dist"

echo "[1/4] stage server code"
rm -rf "$STAGE"; mkdir -p "$STAGE/server" "$DIST"
cp "$HERE/manifest.json" "$STAGE/manifest.json"
cp "$ROOT/ros2_mcp_server.py" "$ROOT/robot_bridge.py" "$ROOT/gazebo_world.py" "$STAGE/server/"

echo "[2/4] vendor the mcp SDK into server/lib (rclpy stays from ROS)"
"$PY" -m pip install --quiet --target "$STAGE/server/lib" "mcp==1.28.1"
# trim caches only — KEEP *.dist-info (mcp reads its own version via importlib.metadata)
find "$STAGE/server/lib" -type d -name "__pycache__" -prune -exec rm -rf {} + 2>/dev/null || true

echo "[3/4] zip -> dist/ros2-bridge.mcpb"
"$PY" - "$STAGE" "$DIST/ros2-bridge.mcpb" <<'PYZIP'
import os, sys, zipfile
stage, out = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for root, _, files in os.walk(stage):
        for f in files:
            full = os.path.join(root, f)
            z.write(full, os.path.relpath(full, stage))   # manifest.json at zip root
print("wrote", out)
PYZIP

echo "[4/4] done"
ls -lh "$DIST/ros2-bridge.mcpb"
echo "Install: open Claude Desktop -> Settings -> Extensions -> install this .mcpb"
