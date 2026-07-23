# gen3_pick_place — gesture-driven pick-and-place on a Kinova Gen3 lite (RViz, CPU-only)

Guide a real **Kinova Gen3 lite** (6-DOF + integrated 2-finger gripper) with
your **LEFT hand** on a webcam; **make a fist to grip, open your palm to
release**. A box in the MoveIt planning scene gets picked up where it stands
and placed wherever your hand carries it. No GPU, no Gazebo, no hardware.

```
/dev/video0 ─▶ MediaPipe GestureRecognizer (ONE inference: Closed_Fist/Open_Palm
            + handedness + 21 landmarks, ~27 ms CPU) ─▶ LEFT-hand filter
            ─▶ One-Euro smoothing ─▶ workspace map + palm-frame orientation
               (palm-rigid landmarks only — wrist + 3 MCPs; fingertips move
                during a fist) with graceful degradation to top-down
            ─▶ /compute_ik (warm-seeded, ~5.6 ms median) ─▶ 20 Hz JointTrajectory
            ─▶ gesture state machine (asymmetric hysteresis + tilted-palm
               fallback release) ─▶ gripper JTC + planning-scene attach/detach
```

| Gesture (LEFT hand) | Robot |
|---|---|
| move / tilt palm | arm follows, gripper orientation follows a camera-facing palm |
| fist within 6 cm of the box | gripper closes, box **attaches** and rides along |
| fist far from box | gripper closes, attach keeps re-evaluating as you carry the fist over |
| open palm (or openness fallback when tilted) | box **detaches** right there |
| hand lost > 2 s | arm drifts home; box stays; `/box_reset` respawns it |

## Layout

| Path | What |
|---|---|
| `ros2_ws/src/gen3_pick_place/` | the package: node, gesture SM (14 unit tests), IK client, scene/box manager, launch, RViz config |
| `docker/` | shared `ros2-arm` launcher + `Dockerfile` (Kinova pkgs + SHA-pinned xacro fix + both MediaPipe models baked) |
| `docs/` | `pickplace-theory.md` (hand-teleop + orientation research, 28 sources) · `pickplace-run.md` (runbook) |

## Run — Docker (recommended; the verified route)

```bash
cd examples/gen3_pick_place
docker build -t ros2-arm:jazzy docker/    # ~10 min first time (Kinova apt layer)
./docker/ros2-arm pickplace synthetic     # scripted full pick-and-place, no camera
./docker/ros2-arm pickplace preview:=true # live: RViz + gesture webcam window
```

The launcher auto-detects this example's vendored `ros2_ws/`, colcon-builds on
first run, and needs `/dev/video0` only for camera mode.

## Run — native (ROS 2 Jazzy laptop)

Heavier than the hand_follow example — the arm needs the Kinova packages and a
patched macro (upstream 0.2.6 ships broken; details + the pinned fix are in the
`docker/Dockerfile`, reproduce its `kortex` layer):

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
     ros-jazzy-controller-manager ros-jazzy-xacro ros-jazzy-rviz2 \
     ros-jazzy-kortex-description ros-jazzy-kinova-gen3-lite-moveit-config
# apply the gen3_lite_macro.xacro fix exactly as the Dockerfile does (SHA-pinned)
pip install --break-system-packages -c ../../constraints.txt mediapipe==0.10.35 numpy==1.26.4
curl -Lo gesture_recognizer.task https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task
cd ros2_ws && colcon build --packages-select gen3_pick_place && source install/setup.bash
ros2 launch gen3_pick_place pickplace.launch.py model_path:=$PWD/../gesture_recognizer.task
```

## Verified numbers (i3-9100, CPU-only)

Synthetic full pick-and-place 6/6: attach at d=0.000 m (gate 0.06), box carried
8.9 s / 0.27 m through 37 planning-scene updates, detached 0.179 m away, max
joint step 0.0085 rad. GestureRecognizer 27.5 ms/frame (36.4 FPS). Oriented IK
sweep 200/200 (auto-degrading to top-down when the palm angle is shallow).
Live-verified: `Closed_Fist 0.77 → grip → carry → Open_Palm 0.60 → detach`.

## Gotchas already handled (don't rediscover)

Upstream `kortex_description` 0.2.6 gen3_lite macro is broken (missing
`gripper` param) — Dockerfile pins the fixed file by commit SHA + sha256.
Stock gripper controller (`GripperActionController`) silently ignores topic
streams — replaced by a second JTC. Gesture must be read at the **same result
index** as the tracked hand. Palm-frame orientation uses palm-rigid landmarks
only. numpy stays 1.26.4 native (repo law); the Docker venv is self-contained.
