# RoboLLM · Humanoid mirroring runbook

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../../README.md) · [Example](../README.md) · [Technical notes](../TECHNICAL.md) · [Examples](../../README.md)

**Environment:** FFW in RViz · **Success:** acceptance tools pass and tracked
arms/head mirror without publishing during freeze or tracking loss.

Operational notes for running the FFW semi-humanoid. For *why* FFW and not a
MoveIt-shipped humanoid (there isn't one), see [../README.md](../README.md);
for the component walkthrough and retargeting math see
[../TECHNICAL.md](../TECHNICAL.md).

**Implemented today: M0 (image) + M1 (bring-up) + M2 (the humanoid moves).**
Camera tracking is M3–M4 — `mirror` requires `synthetic` for now.

## Docker route

```bash
cd examples/humanoid_mirror
./docker/ros2-arm mirror synthetic         # the humanoid MOVES in RViz
./docker/ros2-arm mirror-accept            # M2 acceptance, second terminal

./docker/ros2-arm humanoid                 # FFW in RViz, no motion
./docker/ros2-arm humanoid rviz:=false     # headless
./docker/ros2-arm humanoid-check           # M1 acceptance, second terminal
./docker/ros2-arm stop                     # kill any stuck containers
```

Useful overrides on `mirror synthetic`:

```bash
rviz:=false             # headless — what mirror-accept runs against
latency_probe:=true     # log tick rate + clamp/slew hits every 3 s
sweep_period_s:=8.0     # faster sweep
use_lift:=false         # arms and head only
max_joint_speed:=1.0    # slower, more cautious motion
start_delay:=25.0       # if move_group is slow to come up on your box
```

Deadman, live:

```bash
ros2 service call /mirror_enable std_srvs/srv/SetBool "{data: false}"  # freeze
ros2 service call /mirror_enable std_srvs/srv/SetBool "{data: true}"   # resume
```

Frozen means the node publishes **nothing** — not "publishes the same value".
Resume re-seeds from `/joint_states` and slews, so there is no jump.

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
| Camera/model open or MediaPipe import error | Live M4 mirroring needs a readable webcam and the `/opt/mpvenv` environment. Use `mirror synthetic` for the camera-free acceptance path. |
| Robot never moves, node logs nothing | The node is waiting on `/joint_states`. It refuses to publish before seeding, because guessing the start pose is how you get a jump. Check the controllers spawned. |
| `mirror-accept` reports phantom speed violations | You are on an old copy that timed by callback arrival. DDS bursts make that read ~3× high — see TECHNICAL.md. |
| `ModuleNotFoundError: No module named 'mediapipe'` | The node is under the system python. Launch via `mirror.launch.py` / `ros2-arm track`, which set `prefix=/opt/mpvenv/bin/python`. Synthetic mode works without it, so this looks like a camera fault but isn't. |
| Tracking works but the robot never moves | Expected in `track_only` — that IS M3. Live mirroring is M4. |
| `/body/tracked` is always false | Your shoulders or nose are below `min_visibility`. Sit back so your upper body is in frame; check with `preview:=true`. |
| Skeleton in RViz but no `human/*` TF | Set the RViz **Fixed Frame** to `base_link` and add a TF display. The frames hang off `camera_link`, which `mirror.launch.py` static-publishes from `base_link`. |

### ⚠️ Run one stack per container

Two launches in the same container collide: the second `move_group` and
`controller_manager` fight the first over node names and the DDS graph, and
acceptance tools then report failures that have nothing to do with the code.
This bit during M3 verification and looked exactly like an M1/M2 regression.

Each `ros2-arm <verb>` gets its own container (`--name armhumanoid`,
`armmirror`, `armtrack`), so the launcher is already safe. Only hand-rolled
`docker run ... bash -lc '<two launches>'` invocations hit it. `ros2-arm stop`
clears everything.

## RViz performance

`bg2_rev4` is 25 meshes / 26.8 MB of geometry. That is light, but this box has
no discrete GPU, so RViz runs on software GL. If the frame rate is poor, drop
the MotionPlanning display's "Show Robot Visual" or use `rviz:=false` plus
`humanoid-check` for headless work.
