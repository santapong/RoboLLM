# humanoid_mirror — technical notes

`examples/humanoid_mirror/` is webcam **whole-upper-body** teleop of the ROBOTIS
FFW semi-humanoid: left arm, right arm and head, CPU-only and RViz-based (no
Gazebo, no GPU, no hardware). Unlike [hand_follow](../hand_follow/) it does
**not** vendor a URDF — the robot is apt-installed, so the package shape follows
[gen3_pick_place](../gen3_pick_place/): a single `ament_python` package that
consumes an external description.

**Implemented so far: M0 (image), M1 (bring-up + acceptance), M2 (the humanoid
moves).** The tracking and retargeting layers below are designed and measured
but not yet written; they are documented here so M3–M5 has a spec to build
against, and each is marked.

## Why FFW and not something from MoveIt

MoveIt ships no humanoid. `moveit_resources` is Panda + Fanuc + a PR2 that is
description-only, whose SRDF is a 75-line stub with one `disable_collisions`
pair, no head group, and `<test_depend>` status in `moveit_core`. FFW is the
only apt-installable Jazzy robot whose MoveIt config already defines `arm_l`,
`arm_r` **and `head`**. Full comparison in the [README](README.md).

## Component walkthrough

- **`humanoid_mirror/ffw_config.py`** — the description plumbing, and the
  reason this package exists at all. `build_moveit_configs()` is the corrected
  `MoveItConfigsBuilder` chain: `ffw_moveit_config`'s own `moveit.launch.py`
  calls `.robot_description_semantic()` but never `.robot_description()`, so it
  dies with `XML_ERROR_EMPTY_DOCUMENT` → `[FATAL] Unable to configure planning
  scene monitor` → SIGABRT. It also pins the variant (`ffw_bg2_rev4_follower`)
  and exports the group/joint-name constants everything else imports. Lives in
  the python package, not `launch/`, because launch files installed side by side
  in `share/` cannot import each other.
- **`launch/mock_bringup.launch.py`** — the full stack: `robot_state_publisher`
  + `ros2_control_node` (`mock_components/GenericSystem`) + `move_group` +
  RViz, then `joint_state_broadcaster` followed by the four JTCs chained on
  `OnProcessExit` so they activate in a deterministic order. Passing
  `use_mock_hardware:=true` through to the xacro is **not optional** — the
  URDF's default hardware plugin is `dynamixel_hardware_interface/DynamixelHardware`,
  which opens `/dev/follower` and waits for real servos.
- **`launch/ffw_moveit.launch.py`** — `move_group` alone, for headless checks.
- **`config/ros2_controllers.yaml`** — four JTCs over **disjoint** joint sets.
  The names are not ours to pick: `ffw_moveit_config/config/moveit_controllers.yaml`
  already declares `arm_l_controller` / `arm_r_controller` / `head_controller` /
  `lift_controller`, and MoveIt's simple controller manager only finds ours if
  they match exactly.
- **`humanoid_mirror/mirror_node.py`** — the node (`mirror_node`). One process,
  a pose source plus a 50 Hz control timer. Each tick: read the source →
  One-Euro filter **per output joint angle** → clamp into
  `[lower+margin, upper-margin]` → slew by at most `max_joint_speed / rate_hz`
  → publish a single-point `JointTrajectory` to each of the four controllers.
  Seeds `q_cmd` from `/joint_states` before the first command, so start-up
  cannot produce a jump; on tracking loss it holds `loss_hold_sec` then decays
  to HOME at a gentler `decay_speed`.
  **The architectural upgrade over `hand_follow`: input rate and command rate
  are decoupled.** `hand_follow` ticks at 20 Hz because that is what MediaPipe
  sustains. Here the timer runs at 50 Hz and interpolates toward the latest
  observation, so when M4 adds PoseLandmarker (24.8 ms, ~13 Hz) the robot still
  moves at 50 Hz — smoother motion for zero extra vision CPU.
- **`humanoid_mirror/pose_source.py`** — sources of joint targets, behind one
  interface: `read(t) -> {joint: angle} | None`. M2 ships `SyntheticPoseSource`
  (a scripted whole-body sweep); M3/M4's `BodyPoseSource` plugs in behind the
  same interface and the node does not change. Pure math — no ROS, no numpy —
  so the acceptance tools import it directly.
- **`humanoid_mirror/joint_limits.py`** — the `MEASURED` limit table, a URDF
  limit parser, `clamp`/`slew`, and `OneEuro`. Limits are read from the **live**
  URDF when available and cross-checked against `MEASURED`; a disagreement
  warns loudly rather than silently trusting either, because it means the robot
  is not the variant this example's retargeting constants were written for.
- **`humanoid_mirror/ffw_check.py`** — the M1 acceptance test (`ros2 run
  humanoid_mirror ffw_check`). No camera, no MediaPipe, no RViz. Asserts the
  descriptions are non-empty and name-matched, the SRDF really defines all four
  groups, `/joint_states` carries all 19 mock joints, the four controllers are
  active over disjoint sets, and `/compute_ik` returns `error_code=1` for
  **both** 7-DOF arms.

## Measured facts about FFW (from the expanded `bg2_rev4` xacro)

25 actuated joints total; 19 exposed by the `ros2_control` block (the other 6
gripper joints are mimics). Every arm/head/lift joint has a velocity limit of
**4.8 rad/s**.

| Joint | Axis | Meaning | Lower | Upper |
|---|---|---|---|---|
| `arm_l_joint1` | Y | shoulder pitch | −3.14 | 3.14 |
| `arm_l_joint2` | X | shoulder roll | **0.0** | **3.14** |
| `arm_l_joint3` | Z | humeral yaw | −3.14 | 3.14 |
| `arm_l_joint4` | Y | elbow | −2.9361 | 1.0786 |
| `arm_r_joint2` | X | shoulder roll | **−3.14** | **0.0** |
| `arm_r_joint7` | X | wrist yaw | −1.5804 | 1.8201 |
| `head_joint1` | Y | **pitch** (+ = down) | −0.2317 | 0.6951 |
| `head_joint2` | Z | **yaw** | −0.35 | 0.35 |
| `lift_joint` | Z | prismatic | −0.5 | 0.0 |

Three consequences worth internalising:

1. **`arm_*_joint2` is one-sided and mirrored.** Left is `0 … 3.14`, right is
   `−3.14 … 0`. A symmetric seed pose is out of range on one side — this is the
   single most common way a first attempt at this robot fails, and it is why
   `ffw_check.py`'s two seeds differ in sign.
2. **The elbow flexes negative** (`−2.9361 … 1.0786`), so retargeting needs
   `q4 = -gain * elbow`.
3. **`head_joint1` is pitch, `head_joint2` is yaw** — the opposite of the
   "pan/tilt" reading ROBOTIS's docs suggest. Read the axes, not the docs.

## Topics and services

| Name | Type | Direction | Notes |
|---|---|---|---|
| `/arm_l_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | publish | `arm_l_joint1..7`, single point, 50 Hz |
| `/arm_r_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | publish | `arm_r_joint1..7` |
| `/head_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | publish | `head_joint1,2` |
| `/lift_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | publish | `lift_joint`, gated on `use_lift` |
| `/joint_states` | `sensor_msgs/JointState` | subscribe | 100 Hz; seed / re-seed |
| `/mirror_enable` | `std_srvs/SetBool` | service | deadman; `false` freezes, `true` re-seeds — no jump |
| `/body/markers` | `visualization_msgs/MarkerArray` | publish *(M3)* | 35-connection skeleton |
| `/body/tracked` | `std_msgs/Bool` | publish *(M3)* | visibility-gated |
| `/compute_ik` | `moveit_msgs/GetPositionIK` | client | M1 check only; not in the control path |

## Node parameters (all also launch args of `mirror.launch.py`)

| Parameter | Default | Meaning |
|---|---|---|
| `synthetic` | `false` | scripted whole-body sweep instead of the camera. **M2 requires `true`** — camera mode raises `NotImplementedError` |
| `rate_hz` | `50.0` | control tick / command rate |
| `time_from_start` | `0.06` | seconds-ahead stamp on each trajectory point |
| `max_joint_speed` | `2.0` | rad/s slew ceiling (FFW's URDF limit is 4.8) |
| `limit_margin` | `0.05` | stay this far off every joint limit |
| `min_cutoff` / `beta` | `1.0` / `0.5` | One-Euro on arm angles |
| `head_min_cutoff` | `0.5` | slower One-Euro for the head (tiny travel) |
| `use_lift` | `true` | also drive the prismatic lift |
| `loss_hold_sec` | `2.0` | hold after tracking loss before homing |
| `decay_speed` | `0.5` | rad/s while decaying home |
| `sweep_period_s` | `14.0` | synthetic sweep period |
| `latency_probe` | `false` | log tick rate + clamp/slew hits every 3 s |

`start_delay` (default 18 s) is a launch-only argument: how long to wait for
`move_group` and the four controller spawners before starting the node.

## M2 acceptance — and a measurement trap worth remembering

`tools/mirror_accept.py` (`ros2-arm mirror-accept`) subscribes for a window and
checks publish rates, joint limits, the per-tick slew budget, that the robot
**actually moved**, and that the mock hardware follows. Measured over 10 s:

```
publish rates      50.8 Hz on all four controller topics
joint limits       0 violations (margin 0.05)
per-tick slew      0 violations (<= 0.0400 rad/tick)
velocity           0 violations (<= 2.00 rad/s)
motion             11/11 swept joints moved
mock hardware      11/11 joints tracking the last command
```

⚠️ **Do not compute joint speed from subscriber arrival times.** The first
version of this tool did, and reported phantom violations up to 6.68 rad/s on
`arm_l_joint4` — against a node that provably clamps every tick to 0.04 rad.
DDS delivers in bursts, so two messages published 20 ms apart can arrive 6 ms
apart, and `delta / arrival_dt` then reports 3× the truth. Fixes, both applied:
measure speed from the publisher's **header stamp**, and — better — assert the
timing-free invariant directly, that consecutive commands never differ by more
than `max_joint_speed / rate_hz`. The same trap applies to any rate-limit
assertion over a ROS topic.

The `slew_hits` counter settling at a constant (141 on this box) and never
growing is the expected signature: the clamp fires only during the start-up
transient from the seed pose into the sweep, then the sweep stays under it.

## Planned tracking layer (M3+)

No new dependencies beyond a model file — `PoseLandmarker` ships in the same
mediapipe wheel `hand_follow` already uses.

- **Model:** `pose_landmarker_full`, baked into the image at
  `/opt/models/pose_landmarker_full.task` (sha256-pinned). Measured 24.8 ms on
  the i3-9100 vs 18.1 ms for `lite`, but 2–3× less jitter (wrist σz 1.52 mm vs
  10.88 mm). Inference cost is **resolution-independent** — the model resizes
  internally — so downscaling the capture buys nothing and costs pixel precision.
- **Landmarks:** torso frame from `11/12/23/24`; left arm `11,13,15`; right arm
  `12,14,16`; head from `NOSE=0` + `LEFT_EAR=7` + `RIGHT_EAR=8`. Use
  `pose_world_landmarks` (metres), re-anchored at the shoulder.
- **Head orientation from Pose only** — no `FaceLandmarker`. It would cost
  +11.7 ms to drive 20° of yaw travel. Not a trade worth making.
- **Filter angles, not landmarks.** Reuse `OneEuro` from `hand_follow.py`
  verbatim, one instance per output joint angle (4/arm + 2 neck + 1 lift = 11).
  Filtering landmarks and then computing angles lets noise through the
  nonlinearity.
- **Gate on `visibility`** (present on pose landmarks, absent from the hand
  API). MediaPipe *hallucinates* occluded limbs with plausible coordinates.

⚠️ **`mp.solutions` no longer exists in mediapipe 0.10.35.** `dir(mediapipe)`
returns only `['Image', 'ImageFormat', 'tasks']`. Roughly 90% of MediaPipe pose
tutorials online will not run on the pinned version — use
`mediapipe.tasks.python.vision` exclusively. Drawing connections live at
`PoseLandmarksConnections.POSE_LANDMARKS` (35 connections).

## Planned retargeting (M4+): direct joint angles, not IK

Link-length invariant by construction, no singularities, no unreachable
targets, no branch flips, microseconds to evaluate, and human-looking elbows
for free. It also **dissolves the multi-group problem entirely** — left arm,
right arm and head become three independent pure functions of one landmark set,
each publishing to its own JTC over a disjoint joint set, so simultaneity costs
nothing.

```python
# 1. torso frame — kills camera pose, standing position, torso rotation
mid_s = (P[11] + P[12]) / 2;  mid_h = (P[23] + P[24]) / 2
u = unit(mid_s - mid_h)                 # torso up      -> +Z
l = unit(P[11] - P[12]); l = unit(l - dot(l, u) * u)   # across chest -> +Y
R_t = columns(cross(l, u), l, u)        # REP-103: x fwd, y left, z up

# 2. arm vectors, shoulder-anchored (removes the hip-midpoint origin too)
a = unit(R_t.T @ (P[13] - P[11]))       # upper arm
b = unit(R_t.T @ (P[15] - P[13]))       # forearm

# 3. decompose against FFW's Y-X-Z-Y chain
roll  = asin(clamp(a[1], -1, 1))
pitch = atan2(-a[0], -a[2])
elbow = acos(clamp(dot(a, b), -1, 1))
n = unit(cross(a, b)); r = unit(cross(a, (0, 0, 1)))
yaw   = atan2(dot(cross(r, n), a), dot(r, n))

M = 0.05                                 # limit margin
q1 = clip( gain * pitch, -3.14 + M,  3.14 - M)
q2 = clip( gain * roll,    0.0 + M,  3.14 - M)   # LEFT; mirror the range for right
q4 = clip(-gain * elbow, -2.9361 + M, 1.0786 - M)

# 4. neck — direct mapping, gains well below 1
h = unit(R_t.T @ (P[0] - (P[7] + P[8]) / 2))
q_head2 = clip(0.33 * atan2(h[1], h[0]),        -0.35 + M,   0.35 - M)   # YAW
q_head1 = clip(0.60 * -asin(clamp(h[2], -1, 1)), -0.2317 + M, 0.6951 - M) # PITCH
```

**Fallback when hips are occluded** (common seated at a desk): keep the previous
`R_t` and use shoulder-relative vectors — the same degradation-chain pattern
`gen3_pick_place`'s `ik_client.py` already uses.

## Sharp edges

1. **`cv2.flip` swaps MediaPipe *pose* left/right labels** — the opposite of
   hands, where handedness explicitly assumes a mirrored image (which is why
   `hand_follow`'s flip is correct and mandatory). Measured: `LEFT_WRIST` stays
   at image x≈0.700 in both the normal and flipped frame. **Run Pose on the RAW
   frame, any hand model on the flipped frame, and flip only the preview.** M3
   must ship a regression assertion so a future refactor cannot silently
   reintroduce it.
2. **Velocity clamp.** `hand_follow`'s `max_joint_step_rad = 0.10` at 20 Hz is
   *exactly* 2.0 rad/s — its docstring claims "under" the URDF limit, but it is
   **at** it. Copy that default to a 50 Hz loop and you silently get 5.0 rad/s.
   Use `max_joint_step_rad = 0.04` here: 2.0 rad/s at 50 Hz, well under FFW's 4.8.
3. **Single-camera depth is the weak axis.** `pose_world_landmarks` are
   hip-relative with no absolute camera distance, and the "metric" scale is a
   *canonical* body, not yours — measured shoulder width came out 0.318–0.331 m
   across model variants on one identical frame, against a real adult
   0.36–0.41 m. Direct joint mapping is largely immune (it copies angles, not
   positions), but sagittal angles — shoulder pitch and elbow flexion — both
   depend on z and will be the noisiest outputs.
4. **Self-collision.** Direct mapping has no collision awareness. In cost/benefit
   order: tighten per-joint limits so the reachable set excludes the torso; add
   6–12 capsule-pair checks in pure Python; and only then consider
   `GetStateValidity` at 10–20 Hz **in a separate thread** — never in the 50 Hz
   path, it is a service round-trip. Do not blanket-disable arm↔torso pairs in
   the ACM. The one-sided `arm_*_joint2` range already prevents the worst
   cross-body case for free.
5. **MoveIt Servo is a trap for this robot.** `move_group_name`,
   `command_out_topic`, `command_out_type`, `publish_period` and `use_smoothing`
   are all `read_only` — one node is one group is one controller, permanently.
   `active_subgroup` is time-multiplexed, not concurrent, so it cannot drive
   both arms at once; and a combined multi-chain group has no instantiable IK
   solver (KDL/TRAC-IK/pick-ik are all *chain* solvers). Worth noting that no
   published humanoid teleop system uses Servo — Open-TeleVision, Unitree's
   `xr_teleoperate` and ACE all use Pinocchio optimisation IK.
6. **Version drift.** apt ships FFW 1.2.1 (Jun 2026) while the jazzy branch is
   at 2.0.2 (Jul 2026). Pin to apt; don't mix in git-branch files.

## The numpy fix that shipped with M0

`/opt/mpvenv` had **numpy 2.5.1 and opencv-contrib-python 5.0.0**, shadowing the
system's 1.26.4 — and since `handfollow.launch.py` runs its node under
`/opt/mpvenv/bin/python`, `hand_follow` and `gen3_pick_place` were *already*
running on numpy 2.x, in direct violation of the repo's numpy law. Root cause:
mediapipe 0.10.35 declares **both** numpy and opencv-contrib-python unpinned,
and pip resolves the latter to 5.x, which hard-requires numpy≥2.

Measured symptom: `cv_bridge`'s C extension was compiled against numpy 1.x, so
`CvBridge().cv2_to_imgmsg(...)` raises `KeyError: 16` — no node could publish an
annotated `sensor_msgs/Image`.

The Dockerfile now pins both and **verifies at build time** (a numpy/cv2 version
assert plus a real `cv_bridge` roundtrip), so a regression fails the build
instead of shipping. `constraints.txt` gained `opencv-contrib-python<5` so the
native route can't regress either. This had to be a clean build layer — a
retrofit downgrade inside a populated venv leaves it corrupted
(`numpy._core.multiarray failed to import`).

Note the verification step runs under `SHELL ["/bin/bash", "-c"]` and sources
`setup.bash` first: `cv_bridge` only joins `PYTHONPATH` after sourcing, and
`setup.bash` is bash-only.

## Key files

| File | Role |
|---|---|
| `ros2_ws/src/humanoid_mirror/humanoid_mirror/ffw_config.py` | corrected MoveIt config chain + variant pin + joint constants |
| `ros2_ws/src/humanoid_mirror/humanoid_mirror/mirror_node.py` | the node — 50 Hz control tick, filter → clamp → slew → publish |
| `ros2_ws/src/humanoid_mirror/humanoid_mirror/pose_source.py` | pose sources; M2's synthetic sweep, M4's camera plugs in behind it |
| `ros2_ws/src/humanoid_mirror/humanoid_mirror/joint_limits.py` | limit table + URDF parser, clamp/slew, OneEuro |
| `ros2_ws/src/humanoid_mirror/humanoid_mirror/ffw_check.py` | M1 acceptance (`ros2-arm humanoid-check`) |
| `ros2_ws/tools/mirror_accept.py` | M2 acceptance (`ros2-arm mirror-accept`) |
| `ros2_ws/src/humanoid_mirror/launch/mirror.launch.py` | bring-up + node (`ros2-arm mirror synthetic`) |
| `ros2_ws/src/humanoid_mirror/launch/mock_bringup.launch.py` | full mock stack (`ros2-arm humanoid`) |
| `ros2_ws/src/humanoid_mirror/launch/ffw_moveit.launch.py` | `move_group` alone, headless |
| `ros2_ws/src/humanoid_mirror/config/ros2_controllers.yaml` | four JTCs, disjoint joint sets |
| `docker/Dockerfile` | shared image: numpy pin + FFW + pose model |
