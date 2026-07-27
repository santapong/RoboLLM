# talos_moveit_config

Our own MoveIt 2 configuration for the vendored PAL Robotics TALOS humanoid
(`../talos_description`). No Jazzy MoveIt config exists upstream for TALOS
(PAL never shipped one for ROS 2), so this package is hand-authored on the
pattern `examples/humanoid_mirror` proved for the ROBOTIS FFW: our own
mock-hardware `ros2_control` xacro, chain-based SRDF groups, and headless
`move_group` launch files -- with one difference from FFW: FFW's
`ffw_moveit_config` is apt-installed (we only patch its broken launch file);
TALOS has no such package to patch, so every file under `config/` here is
new, not ported.

## Groups

`arm_l`, `arm_r`, `head`, `torso`, `leg_l`, `leg_r` -- chain-based, matching
`docs/joint-inventory.md`'s `group_hint` column exactly (30 of TALOS's 32
actuated joints; the two `gripper_l`/`gripper_r` joints are out of scope,
see `config/talos.urdf.xacro`'s header comment for why they're not even
built into this package's URDF).

| Group | Base link | Tip link | Joints |
|---|---|---|---|
| `torso` | `base_link` | `torso_2_link` | `torso_1,2_joint` |
| `arm_l` | `torso_2_link` | `wrist_left_ft_tool_link` | `arm_left_1..7_joint` |
| `arm_r` | `torso_2_link` | `wrist_right_ft_tool_link` | `arm_right_1..7_joint` |
| `head` | `torso_2_link` | `head_2_link` | `head_1,2_joint` |
| `leg_l` | `base_link` | `left_sole_link` | `leg_left_1..6_joint` |
| `leg_r` | `base_link` | `right_sole_link` | `leg_right_1..6_joint` |

## Files

- `config/talos.urdf.xacro` -- the planning URDF: every include and macro
  call from `talos_description`'s own `talos_full_common.urdf.xacro`
  (torso, head, arms, force-torque sensors, legs), minus its real-hardware
  `ros2_control` block and the gripper macros, plus our own mock hardware.
- `config/mock_hardware.ros2_control.xacro` -- the mock `<ros2_control>`
  block: unconditionally `mock_components/GenericSystem` over the 30 group
  joints, booted at the PAL "zeros" default standing pose (see below).
- `config/talos.srdf` -- groups, one `home` group_state per group, and a
  `disable_collisions` matrix computed with MoveIt Setup Assistant's
  headless CLI (command below), not hand-curated.
- `config/kinematics.yaml` -- KDL for all six groups (no vendor IK plugin
  exists for TALOS on Jazzy).
- `config/joint_limits.yaml` -- velocity limits measured from the URDF's
  own `<limit velocity="...">` values; acceleration limits are an
  engineering default (`max_acceleration = max_velocity`), not measured --
  see the file's own header for why they're there at all.
- `config/ros2_controllers.yaml` / `config/moveit_controllers.yaml` --
  `controller_manager` config and MoveIt's controller-name mapping, six
  `JointTrajectoryController`s over disjoint joint sets, one per group.
- `launch/talos_moveit.launch.py` -- `move_group` alone, `use_rviz` arg.
- `launch/mock_bringup.launch.py` -- the full stack: `robot_state_publisher`
  + `ros2_control_node` + six controller spawners chained on
  `OnProcessExit` + `move_group` + optional RViz.
- `rviz/talos_moveit.rviz` -- a `MotionPlanning` display defaulted to the
  `arm_l` group, so the demo opens showing the robot rather than an empty
  grid (works with either launch file: `MotionPlanning` draws its own Scene
  Robot from the `robot_description*` parameters passed straight to the
  RViz node, it does not need `robot_state_publisher`'s topic).

## Why the default pose is not all-zero

`arm_left_2_joint`'s range is `[0.0087, 2.871]` and `arm_right_2_joint`'s is
`[-2.871, -0.0087]` -- literal `0.0` is out of range on both shoulders by
about half a degree. The mock hardware boots at, and the SRDF's `home`
group_states are, PAL's own `talos_description/config/default_configuration_zeros.yaml`
("zeros") pose: `arm_left_2_joint = 0.2`, `arm_right_2_joint = -0.2`,
`arm_*_4_joint = -0.02`, everything else `0.0`.

## Reproducing the disable_collisions matrix

```bash
# inside the ros2-talos:jazzy image, workspace sourced
xacro install/talos_moveit_config/share/talos_moveit_config/config/talos.urdf.xacro \
  > /tmp/talos_mock.urdf

ros2 run moveit_setup_assistant collisions_updater \
  --urdf /tmp/talos_mock.urdf \
  --srdf src/talos_moveit_config/config/talos.srdf \
  --output src/talos_moveit_config/config/talos.srdf \
  --default --always --trials 20000 --min-collision-fraction 0.95
```

(`--urdf` was pointed at a pre-expanded file rather than the `.xacro`
directly: `collisions_updater`'s own xacro-in-place loading did not
reliably resolve this package's `$(find talos_moveit_config)` include for
`mock_hardware.ros2_control.xacro` when invoked before the package was
findable on the ament index in a fresh build.)

`--default` and `--always` are what actually let the tool *disable*
pairs it puts in the "in collision at the default pose" and "in collision
across every sample" buckets -- both are always computed, but left active
(not disabled) without those flags, on the theory that a link pair that is
*always* touching is more likely to be a modeling problem worth a human's
attention than a safe optimization. TALOS came back with 0 "Always" pairs
and 5 "Default" ones, all real (see `config/talos.srdf`'s header for which).
Re-run this whenever `config/talos.urdf.xacro` or `config/talos.srdf`'s
`<group>` definitions change.

## Verification performed (headless, one stack per container run)

Colcon-built and run against `ros2-talos:jazzy`, mounted at `/work`:

- `mock_bringup.launch.py use_rviz:=false`: `robot_state_publisher` +
  `ros2_control_node` (`mock_components/GenericSystem`) + `move_group` +
  all six `JointTrajectoryController`s (`joint_state_broadcaster` first,
  then the six chained on `OnProcessExit`) come up with **zero**
  SRDF/collision-matrix errors -- only the expected cosmetic lines
  (`FIFO RT scheduling` permission warning, `No 3D sensor plugin(s)
  defined for octomap updates`, a `KDL` root-link-inertia warning per
  group). All 7 controllers report `active` via
  `ros2 control list_controllers`.
- `/compute_fk` answers for both `arm_l` (`wrist_left_ft_tool_link`) and
  `arm_r` (`wrist_right_ft_tool_link`) at the `home` pose, `error_code=1`
  (`SUCCESS`) for both, mirrored `y` position as expected
  (`(0.0115, 0.4145, -0.2435)` / `(0.0115, -0.4145, -0.2435)`).
- `/plan_kinematic_path` for `arm_l`, a small joint-space goal from `home`,
  returns `error_code=1` (`SUCCESS`) with an 11-point time-parameterized
  trajectory.
- `talos_moveit.launch.py use_rviz:=false` (`move_group` alone, no
  `robot_state_publisher`, no `ros2_control`) also comes up clean.

The acceleration-limits requirement was found by this exact process, not
guessed up front: the first `/plan_kinematic_path` call against a
`joint_limits.yaml` with only velocity limits failed with `No acceleration
limit was defined for joint arm_left_1_joint!` from MoveIt's
`AddTimeOptimalParameterization` response adapter -- which is why that file
carries acceleration limits at all despite none being measured. A second,
separate bug in the same file (a leftover `ros__parameters:` YAML wrapper,
correct for a plain launch-loaded controller yaml but not for
`moveit_configs_utils.load_yaml()`, which expects `joint_limits.<name>: ...`
directly) produced the *same* error message even after adding the values,
which is the trap worth remembering: that error means "MoveIt did not find
an acceleration limit for this joint," not "you forgot to write one."
