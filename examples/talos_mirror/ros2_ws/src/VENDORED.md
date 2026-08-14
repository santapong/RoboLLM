# Vendored: PAL Robotics TALOS description

Vendored for `examples/talos_mirror` (webcam-teleop target robot description
only — no launch/control code depends on this yet). Verified against ROS 2
Jazzy in `ros2-arm:jazzy`; see "Verification" below to reproduce.

## Sources

| Package | Upstream repo | Branch | Commit | Upstream version |
|---|---|---|---|---|
| `talos_description` | https://github.com/pal-robotics/talos_robot | `humble-devel` | `79709aa4f4f453ec18a5e467e65f84ff58ea24d0` (2026-02-19) | 2.10.3 |
| `talos_description_calibration` | https://github.com/pal-robotics/talos_robot | `humble-devel` | `79709aa4f4f453ec18a5e467e65f84ff58ea24d0` (2026-02-19) | 2.10.3 |
| `talos_description_inertial` | https://github.com/pal-robotics/talos_robot | `humble-devel` | `79709aa4f4f453ec18a5e467e65f84ff58ea24d0` (2026-02-19) | 2.10.3 |
| `pal_urdf_utils` (trimmed) | https://github.com/pal-robotics/pal_urdf_utils | `humble-devel` | `775cdd6886296e6c00f17dbdfd9bcdd20e0e6622` (2026-07-03) | 2.9.2 |

Both repos ship an Apache-2.0 `LICENSE` at their root; that file is copied
into each vendored package directory here (`<pkg>/LICENSE`). No secrets, no
PAL-internal URLs, no proprietary binaries were copied — only xacro/URDF/
mesh source files and the two license files.

`talos_robot`'s other packages (`talos_bringup`, `talos_controller_configuration`,
`talos_robot` meta-package, `talos_description`'s `robots/talos_arm_right.urdf.xacro`
etc. variants) were **not** vendored — only what `robots/talos_full_v2.urdf.xacro`
actually pulls in via `xacro:include`/`$(find ...)`.

## What was pruned and why

1. **`talos_description/urdf/head/head.urdf.xacro`, `head_type == 'lidar'`
   branch** — deleted outright (not just left unreferenced). It pulled
   `$(find realsense2_description)/urdf/_d435.urdf.xacro`,
   `_t265.urdf.xacro`, and `$(find pal_urdf_utils)/urdf/camera/OS1-64.urdf.xacro`,
   none of which are vendored. `talos_full_v2.urdf.xacro`'s default
   `head_type` (per `talos_description/launch/robot_state_publisher.launch.py`'s
   `DeclareLaunchArgument`) is `default` (Orbbec Astra Pro — kept), so this
   branch was dead code for this example and was removed rather than left as
   a `$(find)` trap for anyone who flips `head_type:=lidar` later. See the
   comment left in place in `head.urdf.xacro`.

2. **`pal_urdf_utils` — vendored as ~11 files (~1.3 MB) out of the upstream
   package's ~29 MB.** Upstream ships macros for a dozen lidar/laser/camera/
   IMU/FT-sensor product lines (RealSense, Sick, YDLidar, Hokuyo, Ouster,
   Robosense, Rokubi FT, …) used by *other* PAL robots. `talos_full_v2`'s
   xacro tree only reaches: `urdf/deg_to_rad.urdf.xacro` (constant),
   `urdf/ft_sensors/ati_talos.urdf.xacro` (+ `.gazebo.xacro`, `ftsensor.ros2_control.xacro`
   — the wrist FT sensor link topology the grippers attach to),
   `urdf/interaction_sensors/imu_talos.urdf.xacro` (+ `.gazebo.xacro`,
   `imu_sensors.ros2_control.xacro` — the torso IMU link), and
   `urdf/camera/orbbec_astra_pro.urdf.xacro` (+ `.gazebo.xacro` +
   `meshes/camera/orbbec.STL` — the default head camera). Only those files
   (plus `package.xml`/`CMakeLists.txt`, rewritten minimally, and `LICENSE`)
   were copied. `package.xml`'s `exec_depend` on `realsense2_description` was
   dropped since none of the vendored files reference it.

3. **Nothing was pruned from `talos_description`, `talos_description_calibration`,
   or `talos_description_inertial`** — vendored in full (21 MB / 52 KB / 48 KB).
   `<gazebo>` tags and the `<ros2_control>` block's `robot_control/RobotControl`
   and `gazebo_ros2_control/GazeboSystem` `<plugin>` entries were **left as-is**:
   they're inert string content inside custom top-level XML elements that
   `check_urdf`/urdfdom silently skip, they don't `$(find)` any un-vendored
   package, and pruning them would have made the vendored xacro diverge from
   upstream for no verification benefit.

## Required xacro args

`talos_full_v2.urdf.xacro` reads four `$(arg …)` that have **no** `<xacro:arg
default=…>` in the description package itself — only as
`DeclareLaunchArgument` defaults in `talos_description/launch/robot_state_publisher.launch.py`,
which is not vendored. Pass them explicitly:

```
test:=false foot_collision:=thinbox head_type:=default flexibility:=false
```

(`enable_crane`, `use_fixed_base`, `use_sim`, `multiple`, `gazebo_version`,
`use_capsule_collision`, `disable_gazebo_camera` all default inside
`talos_full_common.urdf.xacro` and don't need to be passed.)

## Verification (reproduce)

`$(find <pkg>)` inside xacro resolves via the ament index, which only exists
after a colcon build+install — the bare "clone + point xacro at the source
tree" one-liner fails with `PackageNotFoundError`. Build the four vendored
packages first, then run the acceptance command against the installed
overlay:

```bash
docker run --rm -v /path/to/RoboLLM:/work ros2-arm:jazzy bash -c '
  source /opt/ros/jazzy/setup.bash
  cd /work/examples/talos_mirror/ros2_ws
  colcon build --packages-select talos_description_inertial talos_description_calibration pal_urdf_utils talos_description
  source install/setup.bash
  xacro src/talos_description/robots/talos_full_v2.urdf.xacro test:=false foot_collision:=thinbox head_type:=default flexibility:=false > /tmp/talos.urdf
  check_urdf /tmp/talos.urdf
'
```

Result (2026-07-27, `ros2-arm:jazzy`): xacro exits 0, `check_urdf` prints
"Successfully Parsed XML" and the full link tree, exit 0. All 47 unique
`package://` mesh references in the expanded URDF resolve to files that
exist in this vendored tree (checked by walking `install/<pkg>/share/<pkg>/…`
for every `package://pkg/path` found by regex — 0 missing).

`build/`, `install/`, `log/` under `ros2_ws/` are gitignored (repo-wide `ros
build` rule) and were removed after verification; they are not part of the
vendored deliverable.

### Joint count

44 `revolute` joints total, 0 `prismatic`, 37 `fixed`. Of the 44:

- **32 independently actuated** (no `<mimic>` tag) — matches the "32 actuated
  joints" expectation exactly once grippers are split into primary +
  mimic: `torso_1_joint`, `torso_2_joint` (2 torso); `head_1_joint`,
  `head_2_joint` (2 neck); `arm_{left,right}_{1..7}_joint` (2×7 = 14 arm);
  `leg_{left,right}_{1..6}_joint` (2×6 = 12 leg); `gripper_{left,right}_joint`
  (2 gripper primaries). 2+2+14+12+2 = 32.
- **12 additional `<mimic>` joints**, 6 per gripper
  (`gripper_{side}_inner_double_joint`, `_inner_single_joint`,
  `_motor_single_joint`, `_fingertip_{1,2,3}_joint`, each `mimic`-linked to
  `gripper_{side}_joint`) — the "plus gripper joints" the task anticipated.

32 + 12 = 44, matching the total revolute-joint count above.

## Total vendored size

23 MB (`talos_description` 21 MB, `pal_urdf_utils` trimmed 1.3 MB,
`talos_description_calibration` 52 KB, `talos_description_inertial` 48 KB).
Under the ~60 MB concern threshold — no lower-res mesh substitution was
needed. `talos_description/meshes/` was vendored in full (not trimmed to only
the 47 referenced files) since it was already well under budget as shipped.

## License note (decided 27 Jul 2026)

The talos_robot repository is licensed Apache-2.0 (its root LICENSE, copied
into each vendored package), but several upstream xacro files carry older
CC BY-NC-ND 3.0 headers — an upstream inconsistency, present verbatim in
PAL's own public repo. We redistribute those files unchanged, with
attribution, in a non-commercial learning repository, and treat the repo
LICENSE as governing. Final call re-confirmed at the release/push gate.
If this ever becomes a concern, the fallback robot is JVRC-1 (BSD-2-Clause).
