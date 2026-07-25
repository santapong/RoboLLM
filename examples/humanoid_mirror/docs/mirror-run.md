# humanoid_mirror — runbook

Operational notes for running the FFW semi-humanoid. For *why* FFW and not a
MoveIt-shipped humanoid (there isn't one), see [../README.md](../README.md);
for the component walkthrough and retargeting math see
[../TECHNICAL.md](../TECHNICAL.md).

**Implemented today: M0 (image) + M1 (bring-up + acceptance).** Body tracking
is M2–M5 — the `mirror` verb does not exist yet.

## Docker route

```bash
cd examples/humanoid_mirror
./docker/ros2-arm humanoid                 # FFW in RViz, mock hardware
./docker/ros2-arm humanoid rviz:=false     # headless
./docker/ros2-arm humanoid-check           # M1 acceptance, second terminal
./docker/ros2-arm stop                     # kill any stuck containers
```

First run colcon-builds the vendored `ros2_ws/` (~2 s — one small python
package; the robot itself comes from apt inside the image).

## Native route (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-ffw-description ros-jazzy-ffw-moveit-config \
                 ros-jazzy-ffw-bringup ros-jazzy-realsense2-description \
                 ros-jazzy-pick-ik

cd examples/humanoid_mirror/ros2_ws
colcon build --packages-select humanoid_mirror
source install/setup.bash
ros2 launch humanoid_mirror mock_bringup.launch.py
# second terminal, same sourcing:
ros2 run humanoid_mirror ffw_check
```

## What a healthy start looks like

```
[move_group-3] ... Successfully loaded planner 'OMPL'
[move_group-3] ... You can start planning now!
[ros2-4] ... Configured and activated joint_state_broadcaster
[ros2-5] ... Configured and activated arm_l_controller
[ros2-6] ... Configured and activated arm_r_controller
[ros2-7] ... Configured and activated head_controller
[ros2-8] ... Configured and activated lift_controller
```

`ffw_check` then prints 19 `[PASS]` lines and exits 0.

## Expected noise — do not "fix" these

**7 warnings** of the form:

```
Link 'lift_link' is not known to URDF. Cannot disable/enable collisons.
Link 'swerve_..._wheel_link' is not known to URDF. Cannot disable/enable collisons.
```

These are `disable_collisions` entries in the shared SRDF for links that exist
only on the **SG2 swerve** variant. They are cosmetic. Switching to
`ffw_sg2_rev1_follower` to silence them would trade 7 harmless warnings for a
real robot-name mismatch (`ffw_sg2_follower` ≠ the SRDF's `ffw_bg2_follower`)
plus 3 broken `${swerve_meshes_dir}` meshes. Leave it alone.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `XML_ERROR_EMPTY_DOCUMENT`, `[FATAL] Unable to configure planning scene monitor`, exit −6 | You launched `ffw_moveit_config`'s own `moveit.launch.py`. It never calls `.robot_description()`. Use ours. |
| `PackageNotFoundError` during xacro | `ffw-bringup` or `realsense2-description` missing. They are mandatory but **not declared** as dependencies. |
| Bring-up hangs looking for `/dev/follower` | `use_mock_hardware:=true` didn't reach the xacro. The URDF's default plugin is `dynamixel_hardware_interface/DynamixelHardware` — real servos. |
| `Semantic description is not specified for the same robot as the URDF` | You switched to `sg2_rev1`. Its `<robot name>` doesn't match the SRDF. |
| A controller never activates | Two controllers claiming one joint. The four JTCs must stay over **disjoint** joint sets. |
| IK fails for one arm with a seed that works on the other | `arm_l_joint2` is `0…3.14`, `arm_r_joint2` is `−3.14…0`. Symmetric seeds are out of range on one side. |
| `cv_bridge` raises `KeyError: 16` | The venv drifted back to numpy 2.x. Rebuild the image — the pin must be a build layer, not a retrofit. |

## RViz performance

`bg2_rev4` is 25 meshes / 26.8 MB of geometry. That is light, but this box has
no discrete GPU, so RViz runs on software GL. If the frame rate is poor, drop
the MotionPlanning display's "Show Robot Visual" or use `rviz:=false` plus
`humanoid-check` for headless work.
