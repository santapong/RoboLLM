# hand_follow — webcam LEFT-hand teleop of a 6-DOF arm (RViz, CPU-only)

Wave your **left hand** at a webcam and a 6-DOF sim arm follows it live in
RViz at 20 Hz. No GPU, no Gazebo, no hardware.

```
/dev/video0 ─▶ MediaPipe HandLandmarker (tasks API, 21 landmarks, ~35 FPS CPU)
            ─▶ LEFT-hand filter (mirror-corrected handedness)
            ─▶ One-Euro smoothing ─▶ workspace-clamped mirror mapping
            ─▶ warm-start IK (arm_ik.solve_track, median 0.4 ms)
            ─▶ single-point JointTrajectory @ 20 Hz ─▶ ros2_control ─▶ RViz
```

The LLM angle (see `docs/handfollow-inception.md` for the full researched
theory): the 20 Hz control loop is deliberately classical/deterministic — an
LLM belongs in a *supervisory* role only (ROSA/MCP pattern: "stop following",
"half speed" via `/follow_enable` + parameters), never inside the loop. That
supervisory layer is this repo's natural next step via `ros2_mcp_server.py`.

## Layout

| Path | What |
|---|---|
| `ros2_ws/src/robot_arm_description/` | the 6-DOF arm URDF |
| `ros2_ws/src/robot_arm_moveit_config/` | MoveIt config + `scripts/hand_follow.py` + `launch/handfollow.launch.py` |
| `ros2_ws/tools/` | `arm_ik.py` (FK/IK + tracking mode) + rerunnable acceptance meters |
| `docker/` | the verified Docker route (`Dockerfile` for the image, `ros2-arm` launcher) |
| `docs/` | theory report (ROS2×LLM + teleop pipeline) and the runbook |

## Run — native (this laptop, ROS 2 Jazzy)

```bash
cd examples/hand_follow

# deps — NOTE the numpy pin, per this repo's constraints law (verified working):
pip install --break-system-packages -c ../../constraints.txt mediapipe==0.10.35 numpy==1.26.4
# model file (7.8 MB):
curl -Lo hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task

# build the vendored workspace (installs hand_follow.py AND arm_ik.py into the
# package share, so no PYTHONPATH is needed):
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_description robot_arm_moveit_config
source install/setup.bash

# no camera attached? smoke-test first:  ... handfollow.launch.py synthetic:=true
ros2 launch robot_arm_moveit_config handfollow.launch.py \
    model_path:=$PWD/../hand_landmarker.task preview:=true
```

The launch file picks `python3` automatically when the Docker venv
(`/opt/mpvenv/bin/python`) is absent.

## Run — Docker (as originally verified end-to-end)

```bash
cd examples/hand_follow
docker build -t ros2-arm:jazzy docker/     # bakes mediapipe venv + model file
./docker/ros2-arm handfollow synthetic     # no-camera smoke test
./docker/ros2-arm handfollow preview:=true # live: demo + follower + webcam window
```

The launcher auto-detects this example's vendored `ros2_ws/` (override with
`WS=/path`), and colcon-builds it inside the container on first run (~1 min).

## Knobs (all launch args — see `docs/handfollow-run.md`)

`min_cutoff`/`beta` (One-Euro smoothing vs lag) · `s_near`/`s_far` (depth from
hand size) · `hand_conf` (handedness acceptance) · `preview:=true` (annotated
webcam window: green=tracked, yellow=low conf, red=Right/ignored) ·
`latency_probe:=true` (per-stage timing) · `/follow_enable` SetBool service.

## Verified numbers

20.03 Hz command stream · IK median 0.38 ms, err <0.1 mm · FK ≡ TF at 4 poses ·
MediaPipe 35.4 FPS (2 hands, i3-9100 CPU) · glass-to-RViz ≈ 100–150 ms.
Gotchas already handled in code: handedness labels assume a mirrored feed
(`cv2.flip` first); `hand_world_landmarks` are hand-centered (unusable for
position — normalized image landmarks + hand-size depth proxy are used);
Uno-style: see `track_bench.py` / `hand_accept.py` to re-run acceptance.
