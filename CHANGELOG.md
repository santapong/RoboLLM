# Changelog

Notable changes to **robot-llm-loop**. Format follows
[Keep a Changelog](https://keepachangelog.com/); this project is not yet
versioned — entries are grouped by date on `develop` (merged to `main`
after the touched demos verifiably run).

## 2026-07-25 — examples/humanoid_mirror: a humanoid in MoveIt (M0 + M1)

### Added
- **`examples/humanoid_mirror/`** — the start of webcam whole-upper-body
  teleop (left arm + right arm + head) of a humanoid. **MoveIt ships no
  humanoid**: `moveit_resources` is Panda + Fanuc + a PR2 that is
  description-only, whose SRDF is a 75-line stub with one
  `disable_collisions` pair, no head group, and `<test_depend>` status.
  We use **ROBOTIS FFW "AI Worker"** (`ffw_bg2_rev4_follower`,
  Apache-2.0) — the only apt-installable ROS 2 Jazzy robot whose MoveIt
  config already defines `arm_l` / `arm_r` / **`head`** (+ `lift`), with
  418 real `disable_collisions` pairs. 2×7-DOF arms, 2-DOF neck,
  prismatic lift; 25 meshes, 26.8 MB. It is a *semi-humanoid* — torso +
  arms + head on a lift column, **no legs**.
- `humanoid_mirror/ffw_config.py` — corrected `MoveItConfigsBuilder`
  chain. `ffw_moveit_config`'s own `moveit.launch.py` **crashes**: it
  calls `.robot_description_semantic()` but never `.robot_description()`
  and declares no dependency on `ffw_description`, so it dies
  `XML_ERROR_EMPTY_DOCUMENT` → `[FATAL] Unable to configure planning
  scene monitor` → SIGABRT. Bug is in jazzy-branch HEAD too.
- `launch/mock_bringup.launch.py` (`ros2-arm humanoid`) — RSP +
  `mock_components/GenericSystem` + `move_group` + RViz, with
  `joint_state_broadcaster` and four JTCs chained on `OnProcessExit`.
- `ffw_check.py` (`ros2-arm humanoid-check`) — M1 acceptance, no camera:
  19 checks covering descriptions, all four SRDF groups, 19 mock joints,
  four active controllers over disjoint joint sets, and `/compute_ik`
  success for **both** 7-DOF arms. All green.
- `pose_landmarker_full.task` baked into the image (sha256-pinned,
  versioned URL) for M3+, plus `ros-jazzy-pick-ik`.

### Fixed
- **The numpy law was being violated in the image.** `/opt/mpvenv` held
  numpy **2.5.1** and opencv-contrib-python **5.0.0**, shadowing the
  system 1.26.4 — and since `handfollow.launch.py` runs its node under
  `/opt/mpvenv/bin/python`, `hand_follow` and `gen3_pick_place` were
  already running on numpy 2.x. Root cause: mediapipe 0.10.35 declares
  *both* numpy and opencv-contrib-python unpinned, and pip resolves the
  latter to 5.x, which hard-requires numpy≥2. Measured symptom:
  `cv_bridge`'s numpy-1.x C extension raises `KeyError: 16`, so no node
  could publish an annotated `sensor_msgs/Image`. Both are now pinned in
  all four Dockerfile copies **and verified at build time** (version
  assert + a real `cv_bridge` roundtrip); `constraints.txt` gained
  `opencv-contrib-python<5` so the native route can't regress.

### Notes
- FFW's head axes are the **opposite** of the "pan/tilt" reading its own
  docs suggest: `head_joint1` is axis Y = **pitch** (−13°…+40°, positive
  = looking down), `head_joint2` is axis Z = **yaw** (**±20° only**).
  Head mirroring will be a nod and a glance, not a look-around.
- `arm_l_joint2` is one-sided `0…3.14` and `arm_r_joint2` mirrors it at
  `−3.14…0` — a symmetric seed pose is out of range on one side.
- `ffw-bringup` and `realsense2-description` are mandatory but **not
  declared** as dependencies; without them xacro dies `PackageNotFoundError`.
- Use `bg2_rev4`, not `sg2_rev1`: the latter's `<robot name>` mismatches
  the SRDF and it has 3 broken `${swerve_meshes_dir}` meshes.

## 2026-07-23 — tests + CI: the testing pyramid

### Added
- **First CI**: `.github/workflows/ci-fast.yml` — native no-ROS gate on
  every push/PR (<3 min, blocking): the 14 gesture-SM tests (collected
  from the vendored source via a re-export shim, never copied), a new
  hypothesis property suite over the shared `arm_ik.py` (FK agreement,
  clamp invariants, sub-mm solve_track accuracy in its operating regime,
  jump-flag consistency), a byte-identity guard for the duplicated
  `arm_ik` copies, an executable numpy==1.26.4 law check, and
  errors-only ruff over the root glue.
- `.github/workflows/build-image.yml` — builds `ros2-arm:jazzy` and
  publishes to `ghcr.io/santapong/robollm/ros2-arm:jazzy` on develop
  pushes touching a Dockerfile (registry-cache; 90 min timeout).
- `.github/workflows/ci-container.yml` + `ci/run_scenario.sh` — container
  tier: wallweld selftest (30 checks) + the 16 rclpy gen3 tests + a
  5-scenario acceptance matrix (wallweld full/abort/idle, pickplace +
  handfollow synthetic), all verified green locally. **Manual dispatch
  only for now** — it targets a self-hosted runner that is not yet
  registered (see Security below).
- `tests_ros/test_robot_bridge.py` — deadman via injected clock (wall
  sleeps not required), 20 Hz teleop tick, safe-mode forward block,
  singleton identity; `robot_bridge.py` gained an injectable time source
  (behavior-preserving).
- `hand_accept.py` now emits a machine-readable `RESULT:{json}` line
  (parity with `wallweld_accept.py`).

### Fixed
- Supply chain: `hand_landmarker.task` was fetched from a mutable
  `/latest/` URL and its sha256 recorded but never checked — now pinned
  to the versioned URL and `sha256sum -c`-verified at build time, in all
  four Dockerfile copies.
- One real lint error (unused import in `web/server.py`).

### Security
- The Fable audit caught `ci-container.yml` triggering on pull_request
  against a self-hosted runner in a public repo (arbitrary code
  execution on the runner box) and hanging forever with no runner
  registered — switched to `workflow_dispatch` until a runner strategy
  (self-hosted vs GHCR-pull on hosted runners) is decided.

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
