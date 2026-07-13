# robot-llm-loop

Your LLM ↔ robotics workbench. Claude (via MCP) can introspect and drive a ROS 2
robot and design CAD/URDF in FreeCAD — **and** you get a browser dashboard to
drive it yourself, a set of runnable examples to learn robot software, and a
webcam 3D scanner that turns real objects into robot parts.

```
        you ── natural language ──▶ Claude Code ──┬─ ros2 MCP ──▶ robot_bridge.py ──▶ TurtleBot3 / Gazebo
        │                                         └─ freecad MCP ─▶ FreeCAD (CAD → URDF)
        │
        └── browser ──▶ web/ dashboard ──▶ robot_bridge.py ──▶ same robot
                                              ▲
                        webcam ─▶ scan3d/ ─▶ URDF part ─┘  (drop scanned objects into the world)
```

`robot_bridge.py` is the single ROS 2 node; **both** Claude (MCP) and the web
dashboard use it, so you and the LLM drive the exact same robot.

## What's here
| Path | Purpose |
|------|---------|
| `robot_bridge.py` | The shared ROS 2 node (pubs/subs + helpers, deadman teleop). |
| `ros2_mcp_server.py` | MCP server — 8 tools Claude calls. **Edit to add robot skills.** |
| `run-server.sh` | Launches the `ros2` MCP server. |
| `web/` | Browser dashboard: live telemetry, lidar radar, WASD teleop, Nav2 goals. |
| `examples/` | Runnable ROS 2 + PyBullet + MuJoCo learning path. |
| `scan3d/` | Webcam → 3D mesh → URDF (CPU visual hull + COLMAP photogrammetry). |
| `sim/launch_turtlebot.sh` | Starts TurtleBot3 in Gazebo (own terminal). |
| `assets/` | Outputs: `urdf/`, `cad/`, `screenshots/`, `scan/` (git-ignored). |
| `setup/dev-setup.sh` | Full machine provisioner. |
| `requirements-extra.txt` | Extra pip deps (numpy pinned to 1.26 for ROS). |

## 1 · Browser dashboard (easy teleop)  🕹️
```bash
web/run-web.sh          # then open http://localhost:8080
```
Hold **WASD** or the on-screen arrows to drive (auto-stops on release via a
deadman watchdog), watch live pose + a lidar radar, list topics, send Nav2 goals.
Runs alongside the sim and the MCP server — all three share the robot.

## 2 · Talk to Claude (the MCP loop)
Start the sim (`sim/launch_turtlebot.sh`), then just ask:
- "List the ROS 2 topics." · "Where is the robot?" · "Drive forward 3 s, then turn left."
- "What's the nearest obstacle?" · "Navigate to x=1.5, y=0.5." (Nav2 running)

### MCP tools (`ros2_mcp_server.py`)
| Tool | Does |
|------|------|
| `list_topics` / `list_nodes` | Introspect the ROS graph |
| `get_robot_pose` | x / y / yaw / velocity from `/odom` |
| `get_laser_scan` | Nearest obstacle front/left/right/back from `/scan` |
| `drive` / `stop` | Move via `/cmd_vel` (auto-stop, safety cap) |
| `navigate_to` | Nav2 goal to (x, y, yaw) |
| `run_ros2` | Run any `ros2` CLI command — the fast learn/introspect tool |

Add a tool = add an `@mcp.tool()` function, then restart Claude Code.

## 3 · Learn robot software (`examples/`)
A guided path: ROS 2 nodes → pub/sub → open-loop drive → closed-loop obstacle
avoidance → Nav2 goal (actions) → MoveIt arm, plus CPU physics sims (PyBullet,
MuJoCo). Launch helpers for **SLAM** (map building), **Nav2** (navigation), and a
**MoveIt Panda** arm live in `sim/`. See `examples/README.md`.

## 4 · Scan a real object with your webcam (`scan3d/`)
```bash
cd scan3d
../.venv/bin/python capture.py --background
../.venv/bin/python capture.py --turntable 36 --session mug
../.venv/bin/python visual_hull.py --session mug --height-mm 95
../.venv/bin/python mesh_to_urdf.py ../assets/scan/mug/mug_hull.obj --name mug
```
CPU-only visual hull runs on the laptop; a COLMAP photogrammetry path (dense step
on your GPU cloud) is included. See `scan3d/README.md`.

## 5 · CAD → URDF (the `freecad` MCP)
Open FreeCAD → **MCP Addon** workbench → **Start RPC Server**, then ask:
- "In FreeCAD, create a 2-link robot arm and export it as URDF."
- "Make a 100×60×20 mm mounting bracket with four M4 holes."

## Setup
Extra Python deps live in the venv (created with `--system-site-packages` so it
sees ROS `rclpy`):
```bash
.venv/bin/python -m pip install -r requirements-extra.txt
```

## Install / distribute the MCP server
| Client | Format | Where |
|--------|--------|-------|
| Claude Code (yours) | user scope (done) + `.mcp.json` (project scope, in repo) | `claude mcp add --scope user ros2 -- ~/Desktop/robot-llm-loop/run-server.sh` |
| Claude Desktop | `.mcpb` bundle (drag-and-drop) | build with `mcpb/build_mcpb.sh` → `mcpb/dist/ros2-bridge.mcpb` |

`.mcpb` is a **Claude Desktop** format; **Claude Code uses `.mcp.json`** (shipped
in this repo — opening the folder registers `ros2` after one approval). Details
and the bundle build in `mcpb/README.md`.

## Version control
This folder is a git repo; `.venv/`, scan sessions, and large binaries are
git-ignored. Normal flow: `git add -A && git commit -m "…" && git push`.
