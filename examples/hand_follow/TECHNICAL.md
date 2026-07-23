# hand_follow — technical notes

`examples/hand_follow/` is webcam **LEFT-hand** teleop of a vendored 6-DOF arm,
CPU-only and RViz-based (no Gazebo, no GPU, no hardware). A single node,
`scripts/hand_follow.py` (node name `hand_follow`), runs MediaPipe
HandLandmarker on `/dev/video0`, maps the wrist through a mirror-corrected
axis convention (hand right → base_link +Y, up → +Z, closer/bigger → +X via a
hand-size depth proxy), One-Euro-filters the target, clamps it to a validated
workspace box (x 0.20–0.32, y ±0.15, z 0.16–0.36 m), solves warm-start IK with
`tools/arm_ik.solve_track`, and streams single-point `JointTrajectory`
commands at 20 Hz to the mock-`ros2_control` arm from
`robot_arm_moveit_config`'s `demo.launch.py`. The full theory (ROS2×LLM tiers
+ teleop pipeline) is in [docs/handfollow-inception.md](docs/handfollow-inception.md);
the operational runbook is [docs/handfollow-run.md](docs/handfollow-run.md).

![Hand-teleop component diagram](../../docs/architecture/c4-component-handteleop.svg)

## Component walkthrough

- **`scripts/hand_follow.py`** — the follower. Vision thread: `ThreadedCapture`
  (latest-frame-only camera reader, `CAP_PROP_BUFFERSIZE=1`) → `cv2.flip(1)`
  mirror → `detect_for_video` → accept only `Left` hands with score ≥
  `hand_conf` (larger of two wins) → map + One-Euro filter. 20 Hz control
  tick: slew `cmd_target` by ≤ `max_step_m`, `solve_track` IK, clamp per-tick
  joint deltas to `max_joint_step_rad` (2 rad/s at 20 Hz, under the URDF
  limit), publish. On hand loss: hold `loss_hold_sec`, then decay to HOME
  (0.26, 0, 0.26) at `decay_step_m`/tick. `preflight()` verifies camera,
  model file, `/joint_states` names == `joint1..6` (abort on mismatch), and a
  live controller; shutdown publishes a final hold.
- **`tools/arm_ik.py`** — pure-math FK/IK (no ROS, no numpy). `fk`/`fk_tool0`,
  weld-pose `solve` and warm-start `solve_track(target_xyz, q_prev)` returning
  `(q, info)` with `err_m`, `time_ms`, `clamped`, `reseeded`, `jump`; `jump`
  (per-joint delta > 0.5 rad = branch flip) makes the node freeze target slew
  while joints catch up.
- **`launch/handfollow.launch.py`** — one-command bring-up: includes
  `demo.launch.py` (move_group + RViz + mock controllers), forwards every node
  parameter as a launch argument, then starts `hand_follow.py` after a 22 s
  `TimerAction`. Picks `/opt/mpvenv/bin/python` (Docker mediapipe venv) when
  present, else `python3`.
- **`docker/`** — `Dockerfile` bakes ROS Jazzy + mediapipe venv + the
  `hand_landmarker.task` model into `ros2-arm:jazzy`; the `ros2-arm` launcher
  script auto-detects and colcon-builds the vendored `ros2_ws/` on first run.

## Node parameters (all also launch args of `handfollow.launch.py`)

| Parameter | Default | Meaning |
|---|---|---|
| `synthetic` | `false` | replace camera+mediapipe with a 3D sine sweep in the box |
| `camera` | `/dev/video0` | video capture device |
| `model_path` | `/opt/models/hand_landmarker.task` | MediaPipe hand model file |
| `rate_hz` | `20.0` | control tick / command rate |
| `time_from_start` | `0.12` | seconds-ahead stamp on each trajectory point |
| `min_cutoff` / `beta` | `1.0` / `0.5` | One-Euro filter (smoothness vs lag) |
| `max_step_m` | `0.03` | per-tick target slew clamp while tracking |
| `decay_step_m` | `0.005` | per-tick slew while decaying to HOME |
| `loss_hold_sec` | `2.0` | hold time after hand loss before homing |
| `max_joint_step_rad` | `0.10` | per-tick joint-space delta clamp |
| `hand_conf` | `0.6` | min handedness confidence for a Left hand |
| `s_near` / `s_far` | `0.30` / `0.10` | hand-size scale mapped to near/far X (depth) |
| `latency_probe` | `false` | log per-stage timing medians every 3 s |
| `preview` | `false` | annotated webcam window (X11): green=tracked, yellow=low conf, red=Right |

## Topics and services

| Name | Type | Direction | Notes |
|---|---|---|---|
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | publish | single point, `joint1..6`, 20 Hz |
| `/joint_states` | `sensor_msgs/JointState` | subscribe | seed/re-seed pose; staleness > 1 s warned |
| `/follow_enable` | `std_srvs/SetBool` | service server | `false` pauses (arm holds, state frozen); `true` re-seeds from `/joint_states` — no jump |

## Key files

| File | Role |
|---|---|
| `ros2_ws/src/robot_arm_moveit_config/scripts/hand_follow.py` | the teleop node (`hand_follow`) |
| `ros2_ws/src/robot_arm_moveit_config/launch/handfollow.launch.py` | demo + follower bring-up |
| `ros2_ws/src/robot_arm_description/` | 6-DOF arm URDF |
| `ros2_ws/tools/arm_ik.py` | FK/IK library incl. `solve_track` warm-start mode |
| `ros2_ws/tools/track_bench.py` | U3 acceptance: `fk_tool0` vs 4×4 FK + 200-target sweep (timing, error, jumps) |
| `ros2_ws/tools/track_fk_tf_check.py` | U3 acceptance: `fk_tool0(/joint_states)` vs live TF `base_link→tool0` at 4 poses |
| `ros2_ws/tools/hand_accept.py` | U4 acceptance meter: records both topics, prints amplitudes, smoothness, rates |
| `docker/Dockerfile`, `docker/ros2-arm` | verified container route |

## Run and verify

```bash
cd examples/hand_follow
# Docker (as originally verified):
docker build -t ros2-arm:jazzy docker/
./docker/ros2-arm handfollow synthetic       # no-camera smoke test
./docker/ros2-arm handfollow preview:=true   # live: demo + follower + webcam window
./docker/ros2-arm handfollow latency_probe:=true  # per-stage timing every 3 s

# Native (ROS 2 Jazzy) — see README.md for the numpy-pinned mediapipe install:
cd ros2_ws && colcon build --packages-select robot_arm_description robot_arm_moveit_config
source install/setup.bash
ros2 launch robot_arm_moveit_config handfollow.launch.py model_path:=$PWD/../hand_landmarker.task preview:=true

# pause / resume without killing the node:
ros2 service call /follow_enable std_srvs/srv/SetBool "{data: false}"
# acceptance meters (second shell/container while it streams):
python3 ros2_ws/tools/hand_accept.py 20      # rates + smoothness (expect ~20 Hz)
python3 ros2_ws/tools/track_bench.py         # IK sweep (verified median 0.38 ms)
```

Verified numbers: 20.03 Hz stream, IK median 0.38 ms (err < 0.1 mm), FK ≡ TF
at 4 poses, MediaPipe 35.4 FPS on CPU, glass-to-RViz ≈ 100–150 ms
(`pipeline` gate: capture+infer+ik < 70 ms median).

## Gotchas

- **numpy must stay 1.26.4** (repo constraints law) — install mediapipe with
  `pip install --break-system-packages -c ../../constraints.txt mediapipe==0.10.35 numpy==1.26.4`.
- MediaPipe handedness labels assume a **mirrored** feed — the node flips every
  frame first; don't remove the `cv2.flip`.
- `hand_world_landmarks` are hand-centered and unusable for position; the node
  uses normalized image landmarks + wrist→middle-MCP size as a depth proxy.
- The node aborts if `/joint_states` doesn't report exactly `joint1..6` —
  start the `robot_arm_moveit_config` demo arm, not another robot.
- In Docker, `hand_follow.py` runs from the mounted source path, so edits take
  effect on the next launch with no rebuild.
