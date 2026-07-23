# Changelog

Notable changes to **robot-llm-loop**. Format follows
[Keep a Changelog](https://keepachangelog.com/); this project is not yet
versioned — entries are grouped by date on `develop` (merged to `main`
after the touched demos verifiably run).

## 2026-07-23 — wall_weld: gesture-triggered automation

### Added
- `examples/wall_weld/`: show the webcam an ArUco marker to place — or
  **live-track** (`wall_track:=true`) — a wall in the MoveIt planning scene;
  a held **fist** triggers an autonomous serpentine weld of the entire wall
  face (growing bead + spark markers), an **open palm** aborts mid-weld.
  Collision-checked raster (101/101 sampled states valid at the 15 mm
  standoff), reachability precheck with shrink-to-fit, `/wall_reset`,
  synthetic no-camera acceptance mode, `ros2-arm wallweld` launcher verb.
- `CHANGELOG.md` (this file).

### Fixed (found by adversarial review before release)
- TOCTOU race between marker capture / `/wall_reset` and the 20 Hz control
  tick — a wall plan can no longer be swapped under an in-flight weld.
- Degenerate-raster crash when margins exceed the (possibly shrunk) wall;
  plans now happen before the scene moves, with clean failure events.
- 5 mm torch standoff shipped 88 % collision-valid — the tool's collision
  body is thicker than its tip; the verified default is 15 mm.

## 2026-07-23 — documentation: the C4 pattern

### Added
- `docs/ARCHITECTURE.md` + `docs/architecture/`: hand-crafted C4 SVG
  diagrams (L1 context, L2 containers, L3 hand-teleop pipeline).
- Per-module `TECHNICAL.md` + pipeline diagram for every example
  (`ros2_py`, `patrol_bot`, `pybullet`, `mujoco`, `panda_arm`,
  `hand_follow`, `gen3_pick_place`) and subsystem (`hardware`, `web`,
  `scan3d`, `cad`); `docs/README.md` doc index.

### Fixed
- README's stale "8 MCP tools" → 22; several doc claims corrected against
  sources (patrol_bot's `/scan` is published but not consumed; pybullet IK
  is closed-form, not the PyBullet solver; MCP `spawn_object` is
  primitives-only).

## 2026-07-23 — examples/gen3_pick_place

### Added
- Gesture-driven pick-and-place on a **Kinova Gen3 lite** (6-DOF +
  integrated gripper, official Jazzy packages): LEFT hand guides the arm
  with palm-derived gripper orientation, **fist = grip**, **palm =
  release**; a box in the planning scene is picked and placed via
  attach/detach. One MediaPipe GestureRecognizer inference per frame,
  warm-seeded `/compute_ik` streaming at 20 Hz.
- Shared `docker/` image bakes the Kinova packages plus an SHA-pinned fix
  for the broken upstream `gen3_lite` xacro macro (0.2.6).

## 2026-07-22 — examples/hand_follow

### Added
- Webcam **LEFT-hand teleoperation** of a vendored 6-DOF arm: MediaPipe
  HandLandmarker → One-Euro smoothing → warm-start IK (~0.4 ms) → 20 Hz
  JointTrajectory streaming; live preview window, synthetic test mode,
  latency probe. Runs CPU-only in RViz; verified Docker route with
  auto-building launcher.

### Fixed
- Made the example runnable from a fresh clone (installed-share script
  resolution, `arm_ik` packaging, workspace auto-detection, first-run
  colcon build); scrubbed machine paths and personal email from the
  public tree.

## 2026-07-14 and earlier — the workbench

### Added
- Core loop: `robot_bridge.py` (single shared rclpy node),
  `ros2_mcp_server.py` (22 MCP tools: drive, navigate_to, camera, rosbag,
  TF2, MoveIt arm, Gazebo world control), FastAPI web dashboard with safe
  deadman teleop, TurtleBot3/SLAM/Nav2/MoveIt launch helpers,
  `launch_all.sh`, `.mcpb` bundle for Claude Desktop.
- Learning path `examples/` 01–10 + `patrol_bot` colcon package +
  `panda_arm` manipulation series; `cad/` FreeCAD→URDF pipeline verified
  in PyBullet; `scan3d/` webcam→mesh→URDF scanner.
- `hardware/`: the real DIY arm — Uno R3 firmware (text serial protocol,
  115200), rootless arduino-cli toolchain, `sim_uno.py` pty emulator,
  Pi 5 setup, 6-step health check.
- Project conventions: `CLAUDE.md`, branching workflow
  (`main ← develop ← experiment/*`), public-repo hygiene, numpy 1.26.4 law.
