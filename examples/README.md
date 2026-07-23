# examples — a hands-on path through robot software

Runnable, commented examples you can read, run, and break. Each ROS 2 file is a
single script — run it with the project venv while ROS 2 is sourced:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Desktop/robot-llm-loop
.venv/bin/python examples/ros2_py/01_hello_node.py
```

## Suggested order

### 1 · ROS 2 fundamentals (`ros2_py/`)
The concepts every ROS 2 robot is built from. No sim needed for 01–03.
| File | You learn |
|------|-----------|
| `01_hello_node.py` | Node, timer, logging, the spin loop |
| `02_publisher.py` | Publishing messages on a topic |
| `03_subscriber.py` | Subscribing + the callback pattern (run 02 alongside) |
| `04_drive_square.py` | Publishing `/cmd_vel` to move the robot (open loop) |
| `05_obstacle_stop.py` | **Closed loop**: read `/scan`, act on it — a real behavior |
| `06_send_nav_goal.py` | The **action** pattern: send a Nav2 goal, stream feedback |
| `07_moveit_joint_goal.py` | Plan+execute an **arm** motion via the MoveIt MoveGroup action |
| `08_service_and_params.py` | **Services** (request→reply) + **parameters** — self-contained, verified |
| `09_tf2_transforms.py` | **TF2**: broadcast a transform tree, convert points between frames — self-contained, verified |
| `10_color_follow.py` | **Vision→action**: chase a red object via the camera. `--test` verifies the math without a sim; live needs `waffle_pi` |

04–06 and live-10 need the sim running: `sim/launch_turtlebot.sh` (own terminal;
`TURTLEBOT3_MODEL=waffle_pi` for the camera). 01–03, 08, 09, and `10 --test` run
with no sim at all. Introspect anything live with the MCP `run_ros2` tool or the
CLI, e.g. `ros2 topic echo /scan --once`, `ros2 interface show sensor_msgs/msg/LaserScan`.

Technical: [`ros2_py/TECHNICAL.md`](ros2_py/TECHNICAL.md) — per-script internals, topic tables, diagram.

### 1c · Your first real package (`colcon_pkg/patrol_bot/`)
Everything above is standalone scripts; real ROS 2 work ships as **packages**.
`patrol_bot` is example 04 grown up: an installed executable (`ros2 run`), ROS
parameters, and a launch file. Build it into your workspace:
```bash
mkdir -p ~/ros2_ws/src
cp -r ~/Desktop/robot-llm-loop/examples/colcon_pkg/patrol_bot ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select patrol_bot
source install/setup.bash
ros2 launch patrol_bot patrol.launch.py side_m:=0.6 laps:=2   # with the sim up
```
(Verified: builds with colcon, `ros2 run patrol_bot patrol` completes a lap.)

Technical: [`colcon_pkg/patrol_bot/TECHNICAL.md`](colcon_pkg/patrol_bot/TECHNICAL.md) — package anatomy, params, launch plumbing.

### 1d · IK on your own CAD arm (`pybullet/arm_ik_control.py`)
Closed-form 2-link inverse kinematics on the arm you built in FreeCAD (`../cad`):
give an (x, z) target, the math finds the joint angles, PyBullet proves the tip
lands there (<1 mm). Run `--gui` to watch it reach. Verified headless.

Technical: [`pybullet/TECHNICAL.md`](pybullet/TECHNICAL.md) — the IK math, both scripts, diagram.

### 1e · The manipulation pipeline, end to end (`panda_arm/`)
Five demos on the 7-DOF Panda that build the full chain: FK/IK with the math
visible → pick & place streaming joint angles to a **virtual Arduino** →
vision that **solves object positions from camera pixels** (pinhole model +
OpenCV) → trapezoidal trajectories. Ends exactly where `../hardware` begins:
servo commands over serial. See `panda_arm/README.md`.
```bash
ros2 launch examples/panda_arm/05_vision_sort.launch.py   # the whole pipeline
```
Technical: [`panda_arm/TECHNICAL.md`](panda_arm/TECHNICAL.md) — the five demos layer by layer, topics, diagram.

### 1f · Teleop by hand-tracking (`hand_follow/`)
Wave your **LEFT hand** at the webcam and a 6-DOF arm follows it live in RViz
at 20 Hz — MediaPipe hand landmarks → One-Euro smoothing → warm-start IK →
JointTrajectory streaming, CPU-only, no Gazebo. Self-contained (vendors its
own arm URDF + MoveIt config) with a verified Docker route and a
`synthetic:=true` no-camera smoke test. The researched ROS2×LLM theory
(LLM strictly *supervisory*, never in the control loop) lives in
`hand_follow/docs/`. See `hand_follow/README.md`.
```bash
ros2 launch robot_arm_moveit_config handfollow.launch.py preview:=true
```
Technical: [`hand_follow/TECHNICAL.md`](hand_follow/TECHNICAL.md) — node internals, topic/param tables, verified latencies.

### 1g · Gesture pick-and-place on a real arm model (`gen3_pick_place/`)
The hand_follow idea grown up: a **Kinova Gen3 lite** (6-DOF + gripper) follows
your LEFT hand with palm-derived gripper orientation; **fist = grip, open palm
= release** — pick the planning-scene box up and place it wherever your hand
carries it. MediaPipe GestureRecognizer + `/compute_ik` streaming + planning-
scene attach/detach; CPU-only, self-contained, scripted synthetic acceptance.
See `gen3_pick_place/README.md`.
```bash
./gen3_pick_place/docker/ros2-arm pickplace synthetic   # no camera needed
```
Technical: [`gen3_pick_place/TECHNICAL.md`](gen3_pick_place/TECHNICAL.md) — gesture state machine, IK streaming, scene attach/detach.

### 1h · Gesture-triggered automation: weld a wall (`wall_weld/`)
The teleop examples put your hand in the loop; this one puts it **at the
trigger**: show the webcam an ArUco marker to place (or live-move, with
`wall_track:=true`) a wall in the MoveIt scene, then **fist** — the weld arm
autonomously rasters the whole wall (serpentine coverage path, growing bead,
sparks), **palm aborts** mid-weld. Collision-checked against the wall
(101/101 states valid), adversarially reviewed. See `wall_weld/README.md`.
```bash
./wall_weld/docker/ros2-arm wallweld synthetic   # scripted full weld, no camera
```
Technical: [`wall_weld/TECHNICAL.md`](wall_weld/TECHNICAL.md) — state machine, raster planner, tracking, diagram.

### 1b · Navigation, mapping & manipulation (launch helpers in `sim/`)
Build a map, navigate it, and move an arm — the three pillars beyond teleop.
| Terminal command | What it does |
|------------------|--------------|
| `sim/launch_slam.sh` | **SLAM** (cartographer): drive around to build a map, then save it |
| `sim/launch_nav2.sh [map.yaml]` | **Nav2**: load the map & plan paths (set 2D Pose Estimate first) |
| `sim/launch_moveit_panda.sh` | **MoveIt**: Panda arm + RViz (self-contained, no Gazebo) |

Typical flow:
```bash
# Terminal 1: sim   ·   Terminal 2: SLAM
sim/launch_turtlebot.sh
sim/launch_slam.sh
# drive around (web dashboard :8080 / teleop / ask Claude) until the map is full, then:
ros2 run nav2_map_server map_saver_cli -f ~/map
# Terminal 2 now: navigation with your map
sim/launch_nav2.sh ~/map.yaml
# set "2D Pose Estimate" in RViz, then:
.venv/bin/python examples/ros2_py/06_send_nav_goal.py --x 1.5 --y 0.5   # (or ask Claude, or "Nav2 Goal")

# Manipulation (separate, no sim needed):
sim/launch_moveit_panda.sh
.venv/bin/python examples/ros2_py/07_moveit_joint_goal.py --pose ready
```

### 2 · Physics simulators without a GPU
Learn robot dynamics / URDFs / RL locally; scale to GPU sim on your cloud later.
| File | Software | You learn |
|------|----------|-----------|
| `pybullet/load_robot.py` | PyBullet | Load a URDF, gravity, stepping physics, joints |
| `mujoco/hello_mujoco.py` | MuJoCo | MJCF models, the engine behind modern robot-learning |

```bash
.venv/bin/python examples/pybullet/load_robot.py            # GUI window
.venv/bin/python examples/pybullet/load_robot.py --headless # just prints
.venv/bin/python examples/mujoco/hello_mujoco.py            # prints the swing
.venv/bin/python examples/mujoco/hello_mujoco.py --view     # 3D viewer
```
Technical: [`pybullet/TECHNICAL.md`](pybullet/TECHNICAL.md) · [`mujoco/TECHNICAL.md`](mujoco/TECHNICAL.md) — engine internals, MJCF vs URDF, diagrams.

## The whole map of robot software (where each piece fits)
- **Middleware**: ROS 2 (Jazzy) — you have it. The nervous system: topics/services/actions.
- **Sim (CPU-friendly, local)**: Gazebo (you have it), PyBullet, MuJoCo. For learning + testing behaviors.
- **Sim (GPU, cloud)**: Isaac Sim / Isaac Lab, MuJoCo MJX — massively parallel RL. Put these on GCP/AWS.
- **Navigation**: Nav2 — mapping (SLAM), localization, path planning. Wired via `navigate_to`.
- **Manipulation**: MoveIt 2 — arm motion planning.
- **Learning / policies**: LeRobot, Isaac GR00T, OpenVLA — imitation & VLA models (GPU → cloud).
- **CAD → robot**: FreeCAD + CROSS → URDF (the `freecad` MCP), or scan real parts (`../scan3d`).

Grow this folder as you go — add a MoveIt arm example, a Nav2 map, an RL training
loop. Ask Claude to write the next one and drop it here.
