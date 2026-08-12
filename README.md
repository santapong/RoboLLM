# RoboLLM · Hybrid robotics workbench

![RoboLLM — Build, Observe, Measure, Learn](docs/brand/robollm-banner.svg)

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-1168bd)](https://docs.ros.org/en/jazzy/)
[![Python 3.12](https://img.shields.io/badge/Python-3.12-3776ab)](https://www.python.org/)
[![Arduino Uno](https://img.shields.io/badge/Arduino-Uno-00979d)](hardware/README.md)
[![Branch](https://img.shields.io/badge/branch-develop-f59e0b)](docs/branching.md)

> **Build → Observe → Measure → Learn.** RoboLLM is a learning-first robotics
> platform where classical control, perception, robot learning, and language
> planning meet behind one non-negotiable safety boundary.

RoboLLM combines an MCP-controlled ROS 2 workbench, browser teleoperation,
runnable manipulation examples, CAD and scanning pipelines, and a safe DIY
6-DOF arm foundation. Humans and language-model clients can propose actions;
validated ROS 2 and firmware layers decide what the robot may execute.

![System context — human and LLM clients share the same robot interfaces](docs/architecture/c4-context.svg)

`robot_bridge.py` is the shared ROS 2 node for both the MCP server and browser
dashboard. The physical arm uses the stricter `robo_arm_driver` → serial →
arm-fw 2.1 path. See the [C4 workbench architecture](docs/ARCHITECTURE.md) and
the [physical-arm C4 + Kruchten 4+1 model](docs/physical-arm/ARCHITECTURE.md).

## Start here

| Goal | First stop | Delivery state |
|---|---|---|
| Understand the whole platform | [Architecture tour](docs/ARCHITECTURE.md) | **Current** |
| Run a simulation learning path | [Examples guide](examples/README.md) | **Verified examples** |
| Commission the DIY arm | [Hardware worksheet](docs/physical-arm/HARDWARE_WORKSHEET.md) | **Bench-gated** |
| Use the ROS 2 arm driver | [ROS 2 workspace](ros2/README.md) | **Code-ready** |
| Track Phase 0–5 delivery | [Physical-arm roadmap](docs/physical-arm/ROADMAP.md) | **Authoritative** |
| Find any document | [Documentation hub](docs/README.md) | **Current** |

## Delivery status

The full Phase 0–5 plan is **not complete**. RoboLLM currently has the Phase 0
software/commissioning foundation and the v0.2 ROS driver core. Physical
calibration, measured URDF/MoveIt execution, webcam control on this arm,
autonomous pick-and-place, VLA training, and the arm-specific LLM planner remain
gated work.

| Area | Status | Evidence boundary |
|---|---|---|
| Shared ROS 2 bridge + 22 MCP tools | **Verified** | Simulation demos exist; live use requires a running ROS graph. |
| Browser dashboard | **Verified** | FastAPI/WebSocket surface over the shared bridge. |
| Simulation examples | **Verified** | Each example names its own acceptance environment. |
| Physical arm v0.2 foundation | **Code-ready** | 30 native tests; ROS/Arduino/bench evidence remains open. |
| Physical arm Phases 2–5 | **Planned / gated** | Not promoted from simulation examples. |

## Repository map

| Path | Purpose |
|------|---------|
| `robot_bridge.py` | The shared ROS 2 node (pubs/subs + helpers, deadman teleop). |
| `ros2_mcp_server.py` | MCP server — 22 tools exposed to language-model clients. **Edit to add robot skills.** |
| `run-server.sh` | Launches the `ros2` MCP server. |
| `web/` | Browser dashboard: live telemetry, lidar radar, WASD teleop, Nav2 goals. |
| `examples/` | Runnable ROS 2 + PyBullet + MuJoCo learning path. |
| `examples/hand_follow/` | **Webcam hand teleop**: a 6-DOF arm follows your LEFT hand in RViz (MediaPipe → IK, Docker or native). |
| `examples/gen3_pick_place/` | **Gesture pick-and-place** on a Kinova Gen3 lite: fist = grip, open palm = release. |
| `scan3d/` | Webcam → 3D mesh → URDF (CPU visual hull + COLMAP photogrammetry). |
| `hardware/` | **Real DIY arm**: fail-closed Uno firmware, serial simulator, bench tools, and Pi 5 setup — start with the [Phase 0 worksheet](docs/physical-arm/HARDWARE_WORKSHEET.md). |
| `ros2/` | Installable physical-arm packages; v0.2 starts with the validated `robo_arm_driver`. |
| `sim/launch_turtlebot.sh` | Starts TurtleBot3 in Gazebo (own terminal). |
| `docs/` | Architecture (C4), live-session playbook, branching — index in `docs/README.md`. |
| `assets/` | Outputs: `urdf/`, `cad/`, `screenshots/`, `scan/` (git-ignored). |
| `setup/dev-setup.sh` | Full machine provisioner. |
| `requirements-extra.txt` | Extra pip deps (numpy pinned to 1.26 for ROS). |

## Explore the platform

### Browser dashboard · human control

```bash
web/run-web.sh          # then open http://localhost:8080
```
Hold **WASD** or the on-screen arrows to drive (auto-stops on release via a
deadman watchdog), watch live pose + a lidar radar, list topics, send Nav2 goals.
Runs alongside the sim and the MCP server — all three share the robot.

Technical: [`web/TECHNICAL.md`](web/TECHNICAL.md) — server internals, HTTP/WS endpoint tables, diagram.

### MCP loop · language-model control

Start the sim (`sim/launch_turtlebot.sh`), then ask your MCP client:

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
| `get_camera_image` | Capture the robot's camera to a JPEG — how Claude *sees* (waffle model) |
| `set_safe_mode` | Toggle obstacle-safe teleop (block forward into close obstacles) |
| `start_recording` / `stop_recording` | Record topics to a rosbag (build learning datasets) |
| `list_bags` / `play_bag` | Browse and replay recorded datasets |
| `get_transform` | TF2 lookup between any two frames (map→base_link→laser…) |
| `get_joint_states` | Every joint's position — the layer under MoveIt/ros2_control |
| `move_arm` | Plan + execute a MoveIt joint-space arm motion |
| `get_map_status` | SLAM/map progress: size, resolution, % explored |
| `spawn_object` / `delete_object` | Put boxes/spheres/cylinders into the Gazebo world ("red box in front of the robot") |
| `reset_world` / `list_world_objects` | Reset or inspect the simulation world |
| `run_ros2` | Run any `ros2` CLI command — the fast learn/introspect tool |

Add a tool by adding an `@mcp.tool()` function, then restart the MCP client.
The current registry contains 22 tools.

**Ready for the full demo?** Follow `docs/live-session.md` — a guided ~30 min
session where an MCP client sees, spawns objects, drives, maps, navigates, and moves an arm.

### Learning path · robot software

A guided path: ROS 2 nodes → pub/sub → open-loop drive → closed-loop obstacle
avoidance → Nav2 goal (actions) → MoveIt arm, plus CPU physics sims (PyBullet,
MuJoCo) and two webcam **hand-teleop** arm examples (below). Launch helpers for
**SLAM** (map building), **Nav2** (navigation), and a **MoveIt Panda** arm live
in `sim/`. See `examples/README.md`.

### Drive an arm with your hand (webcam, CPU-only)

Two containerized examples turn the webcam into a robot controller — no GPU, no
Gazebo, no hardware. Build the shared image once
(`docker build -t ros2-arm:jazzy examples/hand_follow/docker/`), then:
```bash
examples/hand_follow/docker/ros2-arm handfollow preview:=true  # 6-DOF arm follows your LEFT hand in RViz
examples/gen3_pick_place/docker/ros2-arm pickplace synthetic   # Kinova Gen3 lite pick-and-place (fist=grip, palm=release), no camera needed
```
Details, native (non-Docker) routes, and verified numbers:
`examples/hand_follow/README.md` and `examples/gen3_pick_place/README.md`; the
pipeline walkthrough is in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

### Scan pipeline · webcam to mesh and URDF

```bash
cd scan3d
../.venv/bin/python capture.py --background
../.venv/bin/python capture.py --turntable 36 --session mug
../.venv/bin/python visual_hull.py --session mug --height-mm 95
../.venv/bin/python mesh_to_urdf.py ../assets/scan/mug/mug_hull.obj --name mug
```
CPU-only visual hull runs on the laptop; a COLMAP photogrammetry path (dense step
on your GPU cloud) is included. See `scan3d/README.md`.

Technical: [`scan3d/TECHNICAL.md`](scan3d/TECHNICAL.md) — both reconstruction routes, script internals, diagram.

### CAD pipeline · FreeCAD to URDF and simulation

A worked **2-link arm** proves the pipeline, and it runs headless:
```bash
freecadcmd cad/build_two_link_arm.py        # FreeCAD builds + exports meshes
.venv/bin/python cad/make_arm_urdf.py       # -> assets/urdf/two_link_arm/*.urdf
.venv/bin/python cad/verify_arm_pybullet.py # PASS: 2 revolute joints, tip moves
```
See `cad/README.md`. Or interactively via the `freecad` MCP: open FreeCAD →
**MCP Addon** workbench → **Start RPC Server**, then ask the CAD client:
- "In FreeCAD, create a 2-link robot arm and export it as URDF."
- "Make a 100×60×20 mm mounting bracket with four M4 holes."

Technical: [`cad/TECHNICAL.md`](cad/TECHNICAL.md) — headless build, joint-frame discipline, PyBullet verify, diagram.

## Documentation map

| Doc | What |
|-----|------|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | The C4 tour: context → containers → components diagrams + narrative. |
| [`docs/physical-arm/ARCHITECTURE.md`](docs/physical-arm/ARCHITECTURE.md) | Physical-arm C4 levels 1–3 plus the 4+1 view model. |
| [`docs/physical-arm/ROADMAP.md`](docs/physical-arm/ROADMAP.md) | Phase 0–5 delivery truth and acceptance gates. |
| [`docs/STYLE_GUIDE.md`](docs/STYLE_GUIDE.md) | Shared theme, status vocabulary, and documentation conventions. |
| [`CHANGELOG.md`](CHANGELOG.md) | What changed, when — grouped by date on `develop`. |
| [`docs/live-session.md`](docs/live-session.md) | Guided ~30 min LLM ↔ robot demo playbook. |
| [`docs/branching.md`](docs/branching.md) | Branch tiers: `main` / `develop` / `experiment/*`. |
| [`docs/README.md`](docs/README.md) | Index of every doc in the repo. |
| `examples/*/README.md` | Per-example run instructions (`examples/README.md` = the learning path). |
| `examples/hand_follow/docs/` · `examples/gen3_pick_place/docs/` | Researched theory (ROS 2 × LLM, teleop, grasping) + runbooks. |

## Environment setup

Extra Python deps live in the venv (created with `--system-site-packages` so it
sees ROS `rclpy`):
```bash
.venv/bin/python -m pip install -r requirements-extra.txt
```

## MCP client packaging

| Client | Format | Where |
|--------|--------|-------|
| Claude Code | project `.mcp.json` or user scope | `claude mcp add --scope user ros2 -- /path/to/RoboLLM/run-server.sh` |
| Claude Desktop | `.mcpb` bundle (drag-and-drop) | build with `mcpb/build_mcpb.sh` → `mcpb/dist/ros2-bridge.mcpb` |

`.mcpb` is a **Claude Desktop** format; **Claude Code uses `.mcp.json`** (shipped
in this repo — opening the folder registers `ros2` after one approval). Details
and the bundle build in `mcpb/README.md`.

## Development workflow

This folder is a git repo; `.venv/`, scan sessions, and large binaries are
git-ignored. Day-to-day verified work is committed and pushed to `develop`;
`main` receives a later `develop` merge only after the touched physical demos
actually run. See [`docs/branching.md`](docs/branching.md).
