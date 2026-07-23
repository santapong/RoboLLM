# gen3_pick_place — technical notes

`examples/gen3_pick_place/` is gesture-driven pick-and-place on a simulated
**Kinova Gen3 lite** (6-DOF + gen3_lite_2f gripper, mock `ros2_control`
hardware — no kortex_driver, no GPU, no Gazebo). The `hand_pick_place` node
runs MediaPipe **GestureRecognizer** on a webcam, tracks the LEFT hand, maps
it into a tuned workspace box, derives gripper orientation from palm-rigid
landmarks (degrading to a fixed top-down quat), solves warm-seeded IK via
`/compute_ik`, and streams 20 Hz single-point `JointTrajectory` to two
controllers. **Closed_Fist grips** — a 4 cm planning-scene box attaches when
the tool is within `D_ATTACH` — **Open_Palm releases** it where carried.
Theory: [docs/pickplace-theory.md](docs/pickplace-theory.md); runbook:
[docs/pickplace-run.md](docs/pickplace-run.md). It shares the hand_follow
teleop skeleton (and Docker image), so the same component diagram applies:

![Hand-teleop component diagram](../../docs/architecture/c4-component-handteleop.svg)

## Component walkthrough

- **`hand_pick_place.py`** — the node (`hand_pick_place`). Vision thread:
  `ThreadedCapture` → mirror flip → `recognize_for_video` →
  `select_left_hand()` (the ONLY place the parallel result arrays are
  indexed) → One-Euro filter → `GestureSM.update()`. 20 Hz control tick
  (manual `spin_once` loop): slew target ≤ `max_step_m`, `IKClient.solve`,
  clamp joint deltas to `max_joint_step_rad`, publish; `_scene_reconcile`
  re-checks the attach gate every tick while fisted (attach FIRST, then
  pinch). Startup homes to `HOME_TARGET`; loss during a grip never releases.
- **`ik_client.py`** — `IKClient` on `/compute_ik` + `/compute_fk`:
  degradation chain `oriented → fixed_down → failed`, recovery-seed retry
  ladder (warm seed → recent successes → `DEFAULT_SEED` → bounded random,
  12 ms per attempt); `recovered=True` freezes target slew one tick. Also
  owns `palm_orientation()` and the workspace constants.
- **`scene_box.py`** — `SceneBoxManager` (node `pickplace_scene`, own
  background executor): spawn / absorb-attach / detach the `demo_box` via
  `/apply_planning_scene`, serves `/box_reset`. Small TOTAL timeouts per
  call so a wedged move_group can't stall the 20 Hz loop.
- **`pickplace.launch.py`** — full bring-up: includes `pickplace_demo.launch.py`
  (stack only), then the node after a 22 s settle; every node parameter is
  forwarded as a launch argument (`key:=value`).

## Controller topology (two JTCs)

`ros2_control_node` (mock GenericSystem ×2, `update_rate: 250`) runs
`joint_state_broadcaster` plus **two** JointTrajectoryControllers:
`joint_trajectory_controller` (`joint_1..6`) and `gripper_controller`
(`right_finger_bottom_joint`, URDF range −0.1…0.96 rad) — the stock
`GripperActionController` silently ignores topic streams;
`moveit_controllers.yaml` maps both as `FollowJointTrajectory`.

## Gesture state machine (`gesture_sm.py`)

States `RELEASED` / `GRIPPING`; events `grip` / `release`, once per commit.
Transition latch: onset (raw score ≥ 0.30 or openness evidence) freezes the
target to the ring-buffer entry ~150 ms *before* onset; `position_hold` lasts
through a 0.30 s settle; a candidate aborts after 3 evidence-free frames or 1 s.

| Transition | Trigger (hysteresis Ns) |
|---|---|
| RELEASED → GRIPPING | `Closed_Fist`, score ≥ 0.60, **n_grip = 3** consecutive frames |
| GRIPPING → RELEASED (primary) | `Open_Palm`, score ≥ 0.60, **n_release = 4** frames |
| GRIPPING → RELEASED (secondary) | openness ≥ 3 fingers extended for **n_secondary = 6** frames AND no confident fist for ≥ 0.25 s (`none_dwell_s`) — the tilted-palm fallback |
| tracking loss | **never releases** (loss during carry = hold) |

Openness = wrist-anchored 3-D extension ratios on metric world landmarks with
per-finger hysteresis: extended at `e ≥ 1.55`, curled at `e ≤ 1.25`.

## Key parameters and constants

| Name | Value | Where / meaning |
|---|---|---|
| `WORKSPACE_BOX` / `WS_BOX` | x 0.26–0.40, y ±0.16, z 0.10–0.30 m | ik_client + scene_box (reconciled); tuned so a 5×5×5 fixed-down IK grid solves 125/125 |
| `HOME_TARGET` / `BOX_SPAWN` | (0.32, 0, 0.20) / (0.34, 0, 0.16) | home decay target / 4 cm cube center, ≥ `D_ATTACH` margin to every box face |
| `D_ATTACH` | 0.06 m | proximity gate: fist attaches only when \|tool − box\| ≤ this, re-checked every tick |
| `GRIP_PINCH` / `GRIP_OPEN` / `GRIP_CLOSE_EMPTY` | 0.50 / 0.0 / 0.80 rad | pinch on the 4 cm box / open / empty-fist close (full close 0.96) |
| `acq_frames` | 3 | acquisition debounce: frames before tracking (re)commits — ghost detections never move the arm |
| `max_jump_rad`, `ik_fail_freeze` | 0.5, 5 | branch-flip guard; consecutive IK failures freezing target advance |
| `model_path` | `/opt/models/gesture_recognizer.task` | GestureRecognizer model (baked into the image) |

The teleop-skeleton parameters (`rate_hz` 20, `max_step_m`, `loss_hold_sec`,
`hand_conf`, `s_near`/`s_far`, One-Euro `min_cutoff`/`beta`, `synthetic`,
`preview`, `latency_probe`, …) match
[hand_follow](../hand_follow/TECHNICAL.md); all are launch args.

## Topics and services (`hand_pick_place` + `pickplace_scene`)

| Name | Type | Direction | Notes |
|---|---|---|---|
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | publish | single point, `joint_1..6`, 20 Hz |
| `/gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | publish | `right_finger_bottom_joint`, on change |
| `/joint_states` | `sensor_msgs/JointState` | subscribe | seed + preflight (6 arm joints required, finger/mimic extras tolerated) |
| `/follow_enable` | `std_srvs/SetBool` | service server | pause/resume; resume re-seeds from `/joint_states` + `/compute_fk` — no teleport |
| `/box_reset` | `std_srvs/Trigger` | service server | detach if needed, respawn box at `BOX_SPAWN` |
| `/compute_ik`, `/compute_fk`, `/apply_planning_scene`, `/get_planning_scene` | MoveIt srvs | client | IK chain; FK reseed/error; box spawn/attach/detach/poll |

## Key files

| File (under `ros2_ws/src/gen3_pick_place/`) | Role |
|---|---|
| `gen3_pick_place/hand_pick_place.py` | main node (`hand_pick_place`) |
| `gen3_pick_place/gesture_sm.py` | gesture state machine (pure Python) |
| `gen3_pick_place/ik_client.py` | IK/FK client, palm orientation, workspace constants |
| `gen3_pick_place/scene_box.py` | box manager node (`pickplace_scene`), `/box_reset` |
| `gen3_pick_place/workspace_sweep.py` | validation: 5×5×5 IK grid + 200-step oriented sweep + spawn margins |
| `config/ros2_controllers.yaml` + `config/moveit_controllers.yaml` | two-JTC controller stack |
| `launch/pickplace.launch.py` / `launch/pickplace_demo.launch.py` | full bring-up / stack only |
| `test/test_gesture_sm.py`, `test/test_hand_pick_place.py` | unit tests (SM + pure logic) |
| `docker/Dockerfile`, `docker/ros2-arm` (example root) | verified container route |

## Run and verify

```bash
cd examples/gen3_pick_place
docker build -t ros2-arm:jazzy docker/       # ~10 min first time
./docker/ros2-arm pickplace synthetic        # scripted full pick-and-place, no camera
./docker/ros2-arm pickplace preview:=true    # live: RViz + gesture webcam window
# native (README.md has the Kinova packages + pinned xacro fix):
ros2 launch gen3_pick_place pickplace.launch.py model_path:=$PWD/../gesture_recognizer.task
ros2 service call /box_reset std_srvs/srv/Trigger                       # respawn box
ros2 service call /follow_enable std_srvs/srv/SetBool "{data: false}"   # pause
```

Verified (i3-9100): synthetic pick-and-place 6/6 — attach at gate 0.06, box
carried 8.9 s / 0.27 m, detached 0.179 m away; GestureRecognizer 27.5 ms/frame;
oriented IK sweep 200/200; live grip→carry→detach confirmed.

## Gotchas

- Upstream `kortex_description` 0.2.6 gen3_lite macro is broken — the
  Dockerfile pins the fixed file by commit SHA + sha256; reproduce for native.
- Read the gesture at the **same result index** as the tracked hand — the
  GestureRecognizer outputs are parallel arrays.
- Palm orientation uses palm-rigid landmarks only (wrist + 3 MCPs); the π
  tool-x flip is deliberate — joint_6 spans < π and the un-flipped branch is
  out of limits across most of the workspace.
- numpy stays 1.26.4 native (repo law); the Docker venv (`/opt/mpvenv`) is
  self-contained.
