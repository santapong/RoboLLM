# robot-llm-loop — project overview for Claude

LLM ↔ robotics learning workbench (learn/research first, product later).
Owner: santapong. GitHub: `santapong/RoboLLM` (**PUBLIC** — everything
pushed here is visible to the world; keep secrets/tokens out). Laptop:
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, **no NVIDIA GPU** (heavy sim/RL
belongs on cloud). Real hardware: DIY arm = Raspberry Pi 5 + Arduino Uno R3.

## Architecture
- `robot_bridge.py` — THE single shared rclpy node (`get_bridge()` singleton,
  background spin thread). Both the MCP server and the web dashboard use it.
- `ros2_mcp_server.py` — FastMCP stdio server, 22 tools (drive, navigate_to,
  camera, rosbag, TF2, MoveIt move_arm, Gazebo spawn/delete/reset via
  `gazebo_world.py`, run_ros2, …). Registered in `.mcp.json` → `run-server.sh`.
  **Editing MCP/bridge code requires restarting Claude Code to reload.**
- `web/` — FastAPI dashboard, `web/run-web.sh` → http://localhost:8080
  (127.0.0.1 by default; LAN: `HOST=0.0.0.0 ROBOT_TOKEN=secret`).
- `examples/` — learning path: ros2_py 01–10, pybullet (IK on the CAD arm),
  mujoco, colcon_pkg/patrol_bot (real ament_python package), panda_arm
  (7-DOF pipeline: FK/IK math GUI → serial→virtual Arduino → camera
  pixel-solving pick & place; RViz-based, no Gazebo needed), hand_follow
  (webcam LEFT-hand teleop of a vendored 6-DOF arm: MediaPipe → One-Euro →
  warm-start IK → 20 Hz JointTrajectory; RViz-based, CPU-only; NOTE its
  mediapipe install must keep the numpy 1.26.4 pin — verified variant in
  its README; Docker route ships its own image), gen3_pick_place (gesture
  pick-and-place on a Kinova Gen3 lite: fist=grip, palm=release, box
  attach/detach in the MoveIt scene; GestureRecognizer + /compute_ik;
  shares the hand_follow Docker image/launcher — Kinova pkgs + SHA-pinned
  xacro fix baked in; same numpy law applies natively), wall_weld
  (gesture-triggered automation: ArUco marker places/live-tracks a wall in
  the MoveIt scene, fist = autonomous serpentine weld of the whole wall
  with bead+spark markers, palm = abort; same weld-arm package + Docker
  image; 15 mm standoff is collision-verified, 5 mm is not),
  humanoid_mirror (webcam whole-upper-body teleop — left arm + right arm
  + head — of a HUMANOID. MoveIt ships none: moveit_resources is
  Panda/Fanuc + a description-only PR2 whose SRDF is a 75-line stub with
  no head group. We use ROBOTIS FFW `ffw_bg2_rev4_follower`, apt-installed,
  the only Jazzy robot with arm_l/arm_r/HEAD groups ready. M0+M1 done:
  `ros2-arm humanoid` + `humanoid-check` (18 checks green, dual-arm IK).
  M2: `ros2-arm mirror synthetic` sweeps arms+head+lift at 50 Hz, no
  camera. M3: `ros2-arm track` = webcam body tracking (TF human/* +
  /body/markers) with the robot parked; `body-accept` has synthetic/
  --live/--ros tiers. M4: `ros2-arm mirror` = LIVE mirroring (your LEFT arm -> its RIGHT), verified vs MoveIt /compute_fk to 0.0000 deg. GOTCHAS: (a)
  ffw_moveit_config's own launch file CRASHES — use ours; (b) the node
  needs prefix=/opt/mpvenv/bin/python or mediapipe is missing; (c)
  cv2.flip SWAPS mediapipe POSE left/right labels (opposite of hands!)
  so Pose runs on the RAW frame; (d) hips are INVISIBLE at a desk
  (vis 0.01) so the camera-up torso fallback is the primary path, and
  elbows are too at rest (0.02) so YOU MUST RAISE YOUR ARMS to mirror;
  (e) both FFW arms are geometrically IDENTICAL — only limits mirror).
- `cad/` — FreeCAD→URDF pipeline (runs headless via `freecadcmd`).
- `scan3d/` — webcam → visual hull mesh → URDF (CPU-only; COLMAP dense = cloud).
- `sim/` — launch scripts (TurtleBot3 Gazebo, SLAM, Nav2, MoveIt Panda);
  root `launch_all.sh`. `docs/live-session.md` = guided 30-min demo playbook;
  `docs/ARCHITECTURE.md` = C4 diagrams + narrative; `docs/README.md` = doc index.
- `hardware/` — the REAL arm: Uno firmware (serial protocol @115200),
  `arm_serial.py` driver, `arm_bridge_node.py` ROS 2 node, `sim_uno.py` fake
  Uno on a pty (develop with no hardware), `check_arduino.sh` 6-step health
  check, `pi5_setup.sh` for the Pi. Uno R3 can't run micro-ROS (2 KB RAM) —
  text serial is intentional.

## Environment gotchas (learned the hard way — do not rediscover)
- **numpy MUST stay 1.26.4** (ROS Jazzy ABI; 2.x breaks rclpy). Always
  `pip install -c constraints.txt`. Venv is `--system-site-packages`.
  The sneaky path in: mediapipe declares BOTH numpy and
  opencv-contrib-python unpinned, and pip resolves opencv to 5.x, which
  hard-requires numpy>=2 — so pinning numpy alone is NOT enough. Pin
  `opencv-contrib-python<5` too (it's in constraints.txt). Symptom when it
  slips: `cv_bridge` raises `KeyError: 16`. Must be fixed in the Docker
  BUILD LAYER — downgrading numpy inside a populated venv corrupts it
  ("numpy._core.multiarray failed to import").
- User shell is zsh but ROS `setup.bash` is bash-only → run verification as
  `bash -c 'source /opt/ros/jazzy/setup.bash && …'`.
- `TURTLEBOT3_MODEL=waffle_pi` for a camera (default burger = lidar only).
- Opening the Uno's serial port DTR-resets it → wait ~2.5 s (ArmSerial does).
- Servo power = external 5–6 V, common GND, NEVER the Uno's 5V pin.
- Never name an rclpy Node method `handle` (shadows Node.handle).
- `pkill -f <pattern>` in a compound command can match your own shell — use
  `pkill -f '[p]attern'` in a separate command.
- /map topic needs TRANSIENT_LOCAL QoS (latched); MoveIt demo runs headless
  with `demo.launch.py use_rviz:=false`.
- Arduino toolchain is ROOTLESS: `~/.local/bin/arduino-cli`, core in
  `~/.arduino15`; Arduino IDE 2.x at `~/.local/opt/arduino-ide` (launcher adds
  `--no-sandbox` for Ubuntu 24.04 AppArmor). Do NOT `apt install arduino`.
- `.mcpb` bundle (mcpb/) is for Claude DESKTOP; Claude CODE uses `.mcp.json`.
  build_mcpb.sh must copy gazebo_world.py and keep `*.dist-info`.

## Verify / run
- Quick MCP smoke test: ask Claude `list_topics` (needs a sim running).
- Sim: `sim/launch_turtlebot.sh` (own terminal, display needed).
- Arm with no hardware: `python3 hardware/sim_uno.py` then
  `ARM_PORT=/dev/pts/N python3 hardware/arm_serial.py ping`.
- Real Uno health check: `hardware/check_arduino.sh` (needs user in `dialout`).
- Examples self-test with no sim: 08, 09, 10 (`--test`).

## Status (2026-08-04)
Done & verified headless: dashboard, 22 MCP tools, examples 01–10 +
patrol_bot, CAD arm, scan3d, .mcpb, hardware stack vs sim_uno. scan3d
print/CAD stack merged to develop (Route C Docker CPU photogrammetry,
mesh_to_print.py, scale_mat.py ChArUco metric scale, MESHER=poisson;
all verified on synthetic data only). PENDING user:
`sudo usermod -aG dialout santapong` + plug the real Uno in → run
check_arduino.sh (now flashes arm-fw 2.0 — the measured-state protocol from
the Phase A convergence, hardware/docs/phaseA-convergence.md; camera_logger
+ acceptance_test are ported and sim-verified; encoders still stubbed).

scan3d BACKLOG (in order — first item gates develop→main for scan3d):
1. PHYSICAL VALIDATION: `python3 scan3d/scale_mat.py make -o mat.png`,
   print at 100% + measure a square, put a calliper-measurable object on
   it, orbit 40–80 phone photos, `./reconstruct_cpu.sh test ~/photos/`,
   `mesh_to_print.py` → compare STL dims vs callipers (target: within
   ~1–2%). Also solves Scene 3 if the same object is scanned with the
   KIRI Engine app for comparison.
2. Scene 4 VGGT rescue path (needs cloud GPU + an object COLMAP fails
   on): VGGT-1B-Commercial checkpoint → COLMAP-format poses → existing
   OpenMVS tail. Do not use MASt3R/DUSt3R (CC-BY-NC).
3. Optional: swap scale_mat.py's hand-rolled COLMAP text parser for
   pycolmap (BSD) if the format ever breaks.

Open gaps: cloud-GPU workflow, tests/CI, RL example,
live desktop demo (docs/live-session.md).

## Conventions
- Commit style: short imperative subject.
- Branching (`docs/branching.md`): day-to-day work on `develop`;
  learning/testing on `experiment/<topic>` (branch off develop, delete when
  done); merge `develop` → `main` only after the touched demos actually run.
  Never commit directly to `main`.
- Keep examples runnable on this laptop (CPU-only, GPU-free).
