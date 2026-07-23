# Gen3-lite Hand-Guided Pick-and-Place — Run Guide

Webcam gesture control of a simulated Kinova Gen3 lite (6-DOF +
gen3_lite_2f gripper, mock hardware — no robot, no kortex_driver): your
LEFT hand steers the tool, **Closed_Fist grips** the 4 cm demo box,
**Open_Palm releases** it. The box lives in the MoveIt planning scene and
genuinely attaches to the gripper (it follows the arm in RViz while carried).

Why each mechanism exists (oriented IK + fallback chain, gesture debounce,
freeze/latch, proximity gate, absorb-attach): see
[pickplace-theory.md](pickplace-theory.md). The hand-tracking foundations
(One-Euro filter, workspace mapping, slew/freeze/reseed) are shared with
handfollow-run.md (next to this file in the dev workspace; `../../hand_follow/docs/` in the RoboLLM examples).

## Prerequisites

- `ros2-arm:jazzy` image built from the Dockerfile (dev ws: `~/ros2_ws/.docker-arm/`; RoboLLM example: `../docker/` — `docker build -t ros2-arm:jazzy ../docker/`)
  (bakes MoveIt2 + ros2_control, the Kinova gen3-lite description + MoveIt
  config, the **pinned xacro fix** for the broken released gen3_lite macro
  (commit `f0a2d4c39bcd3e80da22d725a8b8936fd875b267`, sha256-verified), the
  mediapipe 0.10.35 venv at `/opt/mpvenv`, and the sha256-verified
  GestureRecognizer model at `/opt/models/gesture_recognizer.task`).
- the `ros2-arm` launcher (dev ws: `~/.local/bin/ros2-arm` on PATH; RoboLLM example: `../docker/ros2-arm`, run by path or add to PATH).
- Webcam at `/dev/video0` (camera mode only; the launcher passes it
  through when present).
- Fast DDS is forced (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) — CycloneDDS
  discovery breaks on this multi-NIC host. Do not override it.

## One-command start

```sh
ros2-arm pickplace              # webcam gesture control + RViz
ros2-arm pickplace synthetic    # scripted pick-and-place, no camera
ros2-arm pickplace rviz:=false  # headless (either mode)
ros2-arm stop                   # tear down (container name: armpickplace)
```

First run auto-builds `gen3_pick_place` with colcon (~15 s). One command
brings up, in order:

1. `robot_state_publisher` + `ros2_control_node` on the xacro-processed
   `gen3_lite_gen3_lite_2f` URDF (`use_fake_hardware`, 2x mock GenericSystem)
2. spawners: `joint_state_broadcaster`, `joint_trajectory_controller` (arm),
   `gripper_controller` — a **second JTC** on `right_finger_bottom_joint`;
   the stock GripperActionController silently ignores topic streams
3. `move_group` (OMPL; serves /compute_ik, /compute_fk,
   /apply_planning_scene, /get_planning_scene)
4. RViz with `gen3_pick_place/rviz/pickplace.rviz` — RobotModel +
   PlanningScene display subscribed to **/monitored_planning_scene**, so
   the box and its attach/detach state are always visible
5. after a 22 s settle: `hand_pick_place` under `/opt/mpvenv/bin/python`
   (spawns the demo box, runs preflight, homes to (0.32, 0, 0.20), then
   tracks). The box spawns at (0.34, 0, 0.16), inside the workspace box
   with >= D_ATTACH margin on every axis.

Streaming goes straight to the controllers as single-point
JointTrajectory at 20 Hz (`/joint_trajectory_controller/joint_trajectory`,
`/gripper_controller/joint_trajectory`).

## Gesture cheat-sheet

LEFT hand only (mirror view; handedness confidence >= `hand_conf`).
Gesture and landmarks are always read at the SAME result index as the
selected hand.

| You do | Robot does |
|---|---|
| move open hand | tool follows: image u -> base +Y, v(up) -> +Z, hand size (depth proxy) -> +X |
| tilt/rotate palm | tool orientation follows the palm (oriented IK; falls back to fixed top-down, then position-hold, if IK rejects it) |
| **Closed_Fist** | gripper closes. Within `D_ATTACH` (6 cm) of the box: box **attaches** first, then fingers close to a pinch (0.50 rad) that visually grips the 4 cm cube. Far from the box: fingers close empty (0.80 rad) — carry the fist to the box and it attaches then (the gate keeps re-evaluating while fisted) |
| **Open_Palm** | gripper opens; an attached box **detaches** and stays where you dropped it |
| tilted-palm release (fallback) | if the classifier can't see Open_Palm, extending 3+ fingers of index/middle/ring/pinky (world-landmark openness) after >= 0.25 s of fist-silence (None-score dwell) also releases |
| hand leaves view | target holds `loss_hold_sec`, then decays slowly to HOME; no hand at all = arm holds (camera smoke-tested) |

Gesture transitions are debounced (N consecutive frames + settle) and the
position target is **frozen/latched** during a transition — making a fist
perturbs the wrist/depth signal, and without the latch the arm would lurch
at every grip.

Box gone off somewhere / released mid-air? Reset it to spawn:

```sh
# from a second ros2-arm container (same DDS graph):
ros2-arm ros2 service call /box_reset std_srvs/srv/Trigger
```

Pause/resume the follower (same semantics as handfollow):

```sh
ros2-arm ros2 service call /follow_enable std_srvs/srv/SetBool "{data: false}"
ros2-arm ros2 service call /follow_enable std_srvs/srv/SetBool "{data: true}"   # reseeds from FK first
```

## Tuning knobs

All forwarded as `ros2-arm pickplace [synthetic] key:=value`:

| Knob | Default | Meaning |
|---|---|---|
| `rate_hz` | 20.0 | control tick / command rate |
| `time_from_start` | 0.12 | seconds-ahead stamp on each traj point |
| `min_cutoff` / `beta` | 1.0 / 0.5 | One-Euro filter (smoother vs snappier) |
| `max_step_m` | 0.03 | per-tick Cartesian slew clamp while tracking |
| `decay_step_m` | 0.005 | per-tick slew while decaying to HOME |
| `loss_hold_sec` | 2.0 | hold after hand loss before homing |
| `max_joint_step_rad` | 0.10 | per-tick joint-space delta clamp |
| `hand_conf` | 0.6 | min handedness confidence for a Left hand |
| `acq_frames` | 3 | consecutive frames before tracking (re)commits (ghost-detection guard) |
| `s_near` / `s_far` | 0.30 / 0.10 | hand-size scale mapped to near/far X (depth) |
| `max_jump_rad` | 0.5 | reject IK solutions that flip a joint branch |
| `ik_fail_freeze` | 5 | consecutive IK failures that freeze target advance |
| `camera` | /dev/video0 | capture device |
| `model_path` | /opt/models/gesture_recognizer.task | GestureRecognizer model |
| `latency_probe` | false | per-stage capture/infer/IK/publish timing every 3 s |
| `preview` | false | live webcam window with detections drawn (X11) |

Workspace/grasp constants (edit `gen3_pick_place/scene_box.py` + `ik_client.py`, plus `GRIP_CLOSE_EMPTY` in `hand_pick_place.py`;
no rebuild needed — the node runs from source):
`WS_BOX` x 0.26–0.40, y ±0.16, z 0.10–0.30 m; `SPAWN_XYZ` (0.34, 0, 0.16);
`D_ATTACH` 0.06; `GRIP_PINCH` 0.50; `GRIP_CLOSE_EMPTY` 0.80. A module-load
assert keeps the spawn inside the workspace with `D_ATTACH` margin — if you
retune `WS_BOX`, the node refuses to start rather than spawn an unreachable
box.

## Troubleshooting

- **`Invalid parameter "gripper"` from xacro** — the image lost the pinned
  gen3_lite macro fix; rebuild `ros2-arm:jazzy` (the Dockerfile overwrites
  the broken released macro and sha256-checks it).
- **Gripper never moves** — check the fingers controller is the second JTC:
  `ros2 control list_controllers` must show `gripper_controller
  [joint_trajectory_controller/JointTrajectoryController] active`. The
  stock GripperActionController accepts topic publishes silently and does
  nothing.
- **Node exits in preflight** — it waits for /joint_states (6 arm joints as
  a SUBSET — extra finger joints are fine), IK/FK services, and the box
  spawn. Usually the stack wasn't up yet (slow first RViz start): rerun, or
  raise the 22 s TimerAction in `launch/pickplace.launch.py`.
- **`camera /dev/video0 failed to open`** — container started without the
  device (it's only passed through when present at `docker run` time), or
  another process holds it. Close it, `ros2-arm stop`, relaunch.
- **Arm twitches at grip/release** — raise `acq_frames`, lower `beta`; the
  freeze/latch already suppresses the fist-transition wrist bounce.
- **Palm release not recognized (tilted palm)** — expected; the openness
  fallback needs 3+ extended fingers (of index/middle/ring/pinky) and 0.25 s without a fist score. Face
  the palm to the camera for the fast path.
- **Box floating mid-air after a release** — that's a legal MoveIt world
  pose; `ros2 service call /box_reset std_srvs/srv/Trigger` respawns it.
- **No graph traffic between containers** — both must be Fast DDS on the
  host network (`--net=host --ipc=host`); the launcher sets this up.
- **RViz shows no box** — the PlanningScene display must subscribe
  `/monitored_planning_scene` (pickplace.rviz does; if you swapped configs,
  add it back).

## Live-hand procedure (Operation)

1. `ros2-arm pickplace` and wait for RViz + the node's `preflight OK` /
   `startup homing` log, then the arm settling at HOME.
2. Sit ~0.5–1 m from the webcam, LEFT hand open, palm to camera, fingers
   spread; hold until tracking commits (`acq_frames` consecutive frames).
3. Steer the tool near the box (spawn: 34 cm forward, table height 16 cm;
   depth = move hand toward/away from camera). Watch RViz, not your hand.
4. Make a fist and hold it ~0.5 s: gripper closes; within 6 cm of the box
   it attaches (box turns purple/attached in RViz and follows the tool).
   Fisted too far away? Keep the fist, carry it to the box — it attaches
   on arrival.
5. Carry to the drop zone, open the palm (or extend 3+ fingers): the box
   detaches and stays.
6. `/box_reset` to run it again; `ros2-arm stop` to finish.

Verification tooling shipped with the package: the 14 gesture state-machine
unit tests (`pytest ros2_ws/src/gen3_pick_place/test/`) and the workspace
sweep (`ros2 run gen3_pick_place workspace_sweep --steps 200` against a
running stack). Note: at Ctrl-C, `move_group` may print a segfault during
teardown — a known upstream MoveIt shutdown race, harmless.
