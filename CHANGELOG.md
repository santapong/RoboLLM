# RoboLLM · Changelog

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](README.md) · [Documentation](docs/README.md) · [Roadmap](ROADMAP.md) · [Architecture](docs/ARCHITECTURE.md)

Notable changes to **RoboLLM**. Format follows
[Keep a Changelog](https://keepachangelog.com/); this project is not yet
versioned — entries are grouped by date on `develop` (merged to `main`
after the touched demos verifiably run).

## 2026-08-13 — unified documentation theme

### Changed

- Adopted the RoboLLM identity and **Build → Observe → Measure → Learn** theme
  across first-party documentation.
- Added consistent navigation, status vocabulary, and evidence boundaries to
  root, module, runbook, architecture, and research pages.
- Added a repository banner SVG and documentation style guide; vendored
  upstream documents remain untouched.
- Rebuilt the root README as a portfolio-quality landing page with architecture,
  capability maturity, quick starts, safety boundaries, and validation guidance.
- Restyled all 20 repository SVGs with shared typography, semantic colors,
  accessible titles/descriptions, consistent naming, and a RoboLLM accent rule.
- Replaced machine-specific `robot-llm-loop` paths with repository-relative or
  `/path/to/RoboLLM` examples.

## 2026-08-12 — physical arm v0.2 foundation and architecture baseline

### Added

- Installable `robo_arm_driver` package with named `JointTrajectory` input,
  exact name/limit/time/velocity validation, trajectory sampling,
  `/joint_states`, and honest `/arm/status` provenance.
- arm-fw 2.1 commissioning lock, generated per-arm limits, strict command
  rejection, single-joint commissioning path, and communication watchdog.
- Canonical physical/simulation YAML profiles, firmware config generator,
  pseudo-terminal integration tests, and Phase 0 hardware worksheet.
- Physical-arm phase matrix with explicit evidence gates for Phases 0–5.
- C4 context, container, and driver-component SVGs plus a 4+1 architectural
  view SVG. Planned capabilities are visually distinct from delivered code.

### Verified

- Native suite: 30 passed, 1 environment-dependent skip.
- Serial driver ↔ simulated Uno contract passes in calibrated and commissioning
  modes; host and firmware reject unsafe/bypassing commands.
- Ruff, Python syntax, YAML, shell syntax, package metadata, XML, and generated
  firmware-config synchronization pass.

### Pending hardware/environment evidence

- ROS 2 Jazzy/colcon build, Arduino CLI compilation/flash, electrical and
  mechanical calibration, measured URDF/MoveIt, and all later physical phases.

## 2026-07-25 — humanoid_mirror: mirror direction + preview window (both user-reported)

### Fixed
- **The mirror map negated X as well as Y, so reaching FORWARD drove the
  robot's arm BACKWARD.** A mirror reflects through the plane BETWEEN you
  and the robot, which negates only the world axis joining you; in the
  robot's own frame that leaves forward alone. Correct map is
  `(hx, hy, hz) -> (hx, -hy, hz)` — negate Y only.
  **The existing test could not have caught this**: a sideways raise
  `(0,1,0)` maps to `(0,-1,0)` whether or not x is flipped, so the one
  case I tested was the one case that does not discriminate. Found by the
  user watching the robot, not by the suite. `retarget-bench` now checks
  a forward reach and an overhead raise, and those checks were themselves
  verified to FAIL against the old formula before being accepted.
  Head retargeting now applies the same `mirror_vec()` and reads angles
  off the result instead of negating yaw separately, so the arms and the
  head cannot drift apart on this convention again.
- **The webcam preview was a black rectangle with a working toolbar.**
  `cv2.imshow` was being called from the vision thread; OpenCV HighGUI is
  main-thread only, and on the Qt backend it creates the window but never
  paints. The vision thread now only renders the annotated frame and a
  20 Hz rclpy timer (which runs on the executor = the spin thread) does
  the imshow/waitKey. Also `namedWindow` + `resizeWindow` on first draw:
  left alone the Qt backend opens at ~370x127 and squashes a 640x480
  frame into an unreadable thumbnail.

## 2026-07-25 — humanoid_mirror M4: it mirrors you

### Added
- `ffw_arm.py` — exact FK for one FFW arm plus the retargeting solve, pure
  math. **Verified against MoveIt's own `/compute_fk` to 0.0000 deg** on
  both arms, directions AND link lengths.
- `retarget.py` — body observation -> joint targets, mirror/direct modes,
  per-arm visibility gating, head yaw/pitch with sub-unity gains.
- `tools/retarget_bench.py` (`ros2-arm retarget-bench`) — FK-vs-MoveIt
  tier (`--fk`), pure-math tier (round-trip, mirror semantics, gating,
  continuity, speed, and comparison against an INDEPENDENT brute-force
  optimum), and a live tier (`--ros`).
- `ros2-arm mirror` now does live mirroring; `mirror synthetic` unchanged.

### Measured
- FK vs MoveIt `/compute_fk`: **0.0000 deg**, both arms.
- Retarget round-trip: worst **0.27 deg** over 240 reachable poses.
- Continuity: largest frame-to-frame joint step **0.019 rad**.
- Speed: **0.87 ms** per arm (2 arms = 8.7% of a 20 ms tick).
- Live: both arms commanded, **0** limit violations.

### Geometry findings that contradicted the researched design
- **Both arms are GEOMETRICALLY IDENTICAL** — same axes, same offsets;
  only the base y-offset and the joint2/joint7 LIMITS mirror. One formula
  serves both. The design guessed the right arm needed `asin(-a_y)`; that
  would have driven right-arm roll positive into a limit it can never
  satisfy, clamping to ~0 — a right arm that never lifts, looking like a
  tracking fault.
- The **+-0.041 m elbow offset** is not ignorable: shoulder->elbow tilts
  7.8 deg forward, elbow->wrist 6.9 deg back, and the joint centres
  zigzag **14.6 deg** at q=0 despite a dead-straight net arm.
- `q3` sits BELOW the shoulder gimbal, so it swings that offset around a
  7.78 deg cone and moves the upper-arm direction by up to **15.6 deg**.
  The coupling is NOT weak. Two solvers were written and discarded:
  damped gradient descent (20 deg error, 3 rad jumps) and alternating
  closed-form blocks (spurious fixed points worth exactly 15.6 deg). The
  shipped solve is a 1-D search over q3 with the shoulder exact for any
  q3, each branch swept separately.

### Traps found by testing, not by reading
- **`acos` branches are only correct modulo 2*pi.** The straight-arm elbow
  solution arrives as +6.028 rad, whose wrapped value -0.255 is the one in
  range. Range-checking before wrapping discards it and returns the far
  branch — 15-80 deg of error in a pose that still looks plausible.
- **The shoulder cannot reach every direction at a given q3** (|a_y| <=
  0.9908), but the bound is q3-DEPENDENT, so a T-pose is reachable after
  all. Bailing out instead of clamping collapsed the arm to its seed
  pose: 87 deg of error.
- **Straight-arm degeneracy**: humeral yaw is unobservable when the
  forearm is collinear with the upper arm — measured 0.79 rad steps
  between adjacent frames at 0.0000 deg error (the humerus spinning on a
  straight arm). Hold the previous yaw below 0.06 rad of bend; setting
  that threshold at 14 deg instead cost 12 deg of round-trip error.
- **A self-consistent solver cannot detect a wrong model.** The solver was
  internally exact while the first FK-vs-MoveIt harness reported 31 deg
  disagreement — the harness was comparing link3->link4 against
  link1->link4. In URDF a child link's frame IS its joint's origin.

### Notes
- **Raise your arms into frame to mirror.** Measured elbow visibility at a
  desk is 0.02-0.09 with arms at rest, so gating is PER-ARM: an unseen arm
  is held, never guessed. Whole-body gating would mean constant dropout or
  chasing invented limbs.
- Wrist joints 5-7 are parked at 0 — MediaPipe Pose carries no hand
  orientation, and inventing one would be a lie the robot acts on.
- Mirror derivation: human forward is -x_world and human LEFT is -y_world,
  so `(hx, hy, hz)` in the human torso frame is `(-hx, -hy, hz)` in the
  robot's; feed it to the OPPOSITE arm. Head mirroring flips YAW only.

## 2026-07-25 — humanoid_mirror M3: body tracking (robot parked)

### Added
- `body_track.py` — MediaPipe PoseLandmarker on the **RAW** frame ->
  torso-relative body frame. Pure-math top half (no cv2/mediapipe/ROS/
  numpy) so the geometry is unit-testable with no camera; camera classes
  import vision libs lazily.
- `mirror_node track_only:=true` (`ros2-arm track`) — vision on its own
  thread publishing `/body/tracked`, `/body/markers` (visibility-gated
  skeleton) and TF `camera_link -> human/{l,r}_{shoulder,elbow,wrist}`,
  `human/head`, plus **`human/torso` with the torso frame's full
  orientation** — the debugging aid that matters for M4. Robot parked:
  verified 0 messages on all four controller topics.
- `tools/body_accept.py` (`ros2-arm body-accept`) — three tiers:
  synthetic (26 known-answer geometry checks, no camera, CI-able),
  `--live` (camera, incl. the flip regression guard), `--ros` (topics).

### Measured — and two findings CONTRADICT the researched design
- **Axis convention, measured not read**: `body_x=-world_z`,
  `body_y=+world_x`, `body_z=-world_y` (from
  LEFT-RIGHT_SHOULDER x +0.304, SHOULDER-HIP y -0.485, NOSE-EAR z -0.112).
- **HIPS ARE INVISIBLE at a desk** — measured visibility 0.00-0.01 vs
  1.00 for shoulders. The designed shoulder-to-hip torso "up" vector does
  not exist in practice, so the camera-up fallback is the PRIMARY path.
  Live runs log `frame=camera_up`; that is not a warning state. For M4:
  arms must be RAISED INTO FRAME to mirror (elbow visibility drops to
  0.09 at rest), so gating must be per-arm, not whole-body.
- **The flip trap is real**: `|flip.LEFT-(1-raw.RIGHT)| = 0.018-0.022`
  vs `|flip.LEFT-(1-raw.LEFT)| = 0.445-0.670`, a 20-38x separation.
  POSE labels follow ANATOMY, so cv2.flip swaps them — the OPPOSITE of
  the hand API, where handedness assumes a mirrored image. Pose runs on
  the RAW frame; only the preview is flipped. Permanently guarded by
  `body-accept --live`.
- `pose_landmarker_full` on this box: **median 28-31 ms (~28-32 Hz),
  p95 47 ms, 100% detection** — inside the 70 ms U5 gate. (The design's
  24.8 ms was the i3-9100 laptop.)
- Tracking loss publishes `DELETEALL`, never a stale skeleton: verified
  173/173 `tracked=false`, 171/171 `DELETEALL`.

### Fixed
- **The node must run under `/opt/mpvenv/bin/python`.** mediapipe is not
  in the system python that ament console-scripts are shebanged to, so
  tracking died with `ModuleNotFoundError: No module named 'mediapipe'`
  *while synthetic mode kept working* — which reads as a camera fault.
  `mirror.launch.py` now sets `prefix=/opt/mpvenv/bin/python` (the same
  fix hand_follow uses), and `_make_tracker()` catches the error and
  explains it. Caught by the M3 ROS-tier check, not by inspection.

## 2026-07-25 — humanoid_mirror M2: the humanoid moves

### Added
- `mirror_node` + `mirror.launch.py` (`ros2-arm mirror synthetic`) — a
  scripted whole-body sweep drives both 7-DOF arms, the 2-DOF head and
  the lift in RViz at 50 Hz. **No camera, and MediaPipe is never
  imported** (vision imports are lazy, inside the camera branch), so the
  demo cannot be broken by a missing webcam or a drifted venv. Camera
  mode raises `NotImplementedError` with a pointer to the build plan
  rather than failing obscurely.
- `pose_source.py` — pose sources behind one interface
  (`read(t) -> {joint: angle} | None`); M4's camera source plugs in
  without touching the node. Pure math, no ROS/numpy, so tools import it.
- `joint_limits.py` — `MEASURED` limit table + a URDF parser. Limits are
  read from the **live** URDF and cross-checked against the table; a
  mismatch warns loudly, since it means the robot is not the variant the
  retargeting constants were written for.
- `tools/mirror_accept.py` (`ros2-arm mirror-accept`) — M2 acceptance.
  Measured over 10 s: **50.8 Hz on all four controller topics, 0
  joint-limit violations, 0 per-tick slew violations, 11/11 swept joints
  moved, mock hardware tracking every command.** Emits `RESULT:{json}`.
- `/mirror_enable` (`std_srvs/SetBool`) landed early from M5 — the
  control loop needed a freeze path anyway. Verified: frozen publishes
  **nothing**, resume re-seeds from `/joint_states` (max step on resume
  0.0164 rad, under the 0.0400 budget — no jump).

### Notes
- Input rate and command rate are **decoupled**: the timer runs at 50 Hz
  and interpolates toward the latest observation, so when M4 adds
  PoseLandmarker (24.8 ms, ~13 Hz) the robot still moves at 50 Hz.
- `max_joint_speed` is sized from the rate (2.0 rad/s → 0.04 rad/tick at
  50 Hz), never copied. hand_follow's 0.10 at 20 Hz is *exactly* 2.0
  rad/s despite its docstring claiming "under" the limit; copied into a
  50 Hz loop that silently becomes 5.0 rad/s.
- **Measurement trap, found the hard way:** never compute joint speed
  from subscriber *arrival* times. DDS delivers in bursts, so messages
  published 20 ms apart can arrive 6 ms apart — the first version of
  mirror_accept reported phantom 6.68 rad/s violations against a node
  that provably clamps to 0.04 rad/tick. Use the publisher's header
  stamp, and prefer asserting the timing-free per-tick invariant.

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
  18 checks covering descriptions, all four SRDF groups, 19 mock joints,
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
