# talos_mirror — technical notes

`examples/talos_mirror/` is webcam **whole-body** teleop of the vendored PAL
TALOS full-size humanoid: both arms, head, torso, and both legs (pelvis
pinned), CPU-only and RViz-based (no Gazebo, no GPU, no hardware).

STATUS: skeleton only -- section stubs below name the gotchas the c2-qp (M5)
review surfaced (traps to avoid, not yet written up). Full write-up is a
docs-phase task; each stub is a placeholder for a full explanation in the
style of ../humanoid_mirror/TECHNICAL.md (root cause, symptom, fix, and the
test/check that now catches it), not the explanation itself.

## Leg asin-branch discontinuity

TODO: retarget.py's `_pick_gimbal` ACOS-vs-ASIN branch-labelling trap (an
earlier asin form cost 24 deg of round-trip error on the leg solver because
its second branch silently jumped onto the physically different acos-branch
solution between adjacent search samples) -- see `_pick_gimbal`'s own
docstring for the derivation in the meantime.

## Bench kwargs-int-key trap

TODO: retarget-bench's landmark-dict construction gotcha (kwargs vs.
integer-keyed dicts) that produced a false pass/fail signal before it was
caught.

## Diagonal-step 1-DOF knee residual

TODO: `solve_leg_hip_knee`'s free-axis-swap workaround (hip YAW swept
instead of hip pitch, unlike the arm's shoulder/elbow decomposition) still
leaves a small residual on a diagonal (non-sagittal, non-lateral) stepping
motion -- see that function's own docstring for the geometric argument in
the meantime.

## Head yaw sign-degeneracy under clamp

TODO: `_head_angles`'s yaw = atan2(head_dir_y, head_dir_x) loses a
well-defined sign once `HEAD_YAW_LIMITS` clamps it near the range boundary,
under a specific combination of gain and observed pitch.

## Launcher single-quote bash -c trap

TODO: the `bash -c '...'` sourcing convention this repo's verification
commands use (`source /opt/ros/jazzy/setup.bash && ...`) breaks silently if
a single quote appears anywhere inside the quoted command -- house
convention, not specific to this package, but talos_mirror's longer launch
invocations make it easy to hit by accident.

## CPU profile (measured, 27 Jul 2026)

`docker exec <container> top -b -d 2 -n 20` (PID-filtered via `pgrep -af`,
cross-checked against `docker stats --no-stream`), steady state, n=20
samples/process. Host was shared with an active browser, so treat absolute
Hz/CPU numbers as approximate and same-run process rankings as the
trustworthy signal (docker stats totals matched summed PIDs within ~1pt in
both runs below).

**Before** -- `ros2-arm talos use_rviz:=false` (mock bring-up + move_group,
idle-hold, no mirror/track):

| process | avg %CPU |
|---|---|
| ros2_control_node (6 JTCs in-process, update_rate was 100 Hz) | 14.00 |
| move_group | 3.12 |
| robot_state_publisher | ~1.5-2.5 (steady-state) |

**Before** -- `ros2-arm mirror synthetic use_rviz:=false` (adds talos_track +
mirror_node, live 50 Hz commands):

| process | avg %CPU |
|---|---|
| mirror_node (python, 50 Hz tick) | 34.91 |
| ros2_control_node (update_rate was 100 Hz) | 21.20 |
| track_node (synthetic 30 Hz tick) | 6.15 |
| move_group (idle -- grep-confirmed zero FK/IK calls from mirror_node/track_node/retarget.py) | 3.50 |
| robot_state_publisher | 1.95 |

`docker stats` totals: 15.06% (talos idle) / 67.10% (mirror synthetic),
matching the summed PIDs within ~1pt in both runs.

**What was cut, by measured impact:**

1. **move_group off by default on the mirror/sweep path**
   (`use_moveit:=false`, wired through `mock_bringup.launch.py` ->
   `mirror.launch.py` / `sweep.launch.py`). move_group's own cost was small
   (~3-3.5%, the cheapest process in the profile) but 100% wasted here:
   neither `track_node` nor `mirror_node` ever calls `/compute_fk` or
   `/compute_ik` (grep-confirmed across `track_node.py`, `mirror_node.py`,
   `retarget.py`, `qp_retarget.py`, `qp_pose_source.py`, `pin_model.py`).
   It also removes one BEST_EFFORT subscriber each off `/joint_states` and
   `/tf`. `talos-check` and `retarget-bench --fk` still need it: the
   `talos` verb keeps `use_moveit:=true` as its default (unchanged), and
   `mirror`/`sweep` accept `use_moveit:=true` to opt back in for `--fk`
   runs against those containers.
2. **`controller_manager.update_rate` dropped from 100 Hz to 60 Hz**
   (`talos_moveit_config/config/ros2_controllers.yaml`, shared by every
   verb). The old value's own comment admitted it was unmeasured headroom.
   Measured real command cadence on this path is 50 Hz (both
   `mirror_rate_hz` and sweep's `rate_hz` default to 50.0); `/joint_states`
   was measured running 92.6-100 Hz against that 50 Hz input, and
   `ros2_control_node`'s CPU rose 14.00% -> 21.20% between idle-hold and
   live-command runs -- the control loop was genuinely firing ~2x per
   command, not just idling faster. 60 Hz keeps headroom above the 50 Hz
   command rate without doubling the loop. `joint_state_broadcaster` has no
   independent output-rate knob (its rate is coupled to
   `controller_manager.update_rate`), so this one change also caps its
   overshoot -- no separate JSB setting exists to tune.
3. **Not touched (measured cheap, left alone):** `track_node` (6.15%,
   already proportionate to its own 30 Hz synthetic tick),
   `robot_state_publisher` (cheapest process in both runs; `/tf` was
   measured at 18-48 Hz, never sustained above the 50 Hz need, so it is
   not overshooting), and each JTC's own `state_publish_rate: 100.0` (a
   different topic than `/joint_states`, not measured in this profile --
   left at its existing value rather than guessed at).
4. **RViz cost (doc-only, not re-measured):** `mirror.launch.py`'s own
   `rviz/mirror.rviz` (plain `RobotModel` + `MarkerArray(/body/markers)` +
   `TF`) was already the lighter of the two shipped RViz configs before
   this pass -- `talos_moveit_config`'s `talos_moveit.rviz` (used by
   `use_rviz:=true` on the `talos` verb) additionally loads
   `moveit_rviz_plugin/MotionPlanning`, which embeds its own
   `PlanningSceneMonitor` client, interactive markers, and a trajectory
   slider on top of whatever GL cost the scene already pays. Both configs
   render on `LIBGL_ALWAYS_SOFTWARE=1` in this container (required on this
   host -- hardware GL raises "GLX drawable fail" here), so neither gets
   hardware-GL frame rates; prefer `mirror.rviz`'s plain display set for
   visualization-only sessions where `MotionPlanning`'s extras are not
   needed.

**mirror_node's own per-tick Python cost** (per-joint OneEuro filter fan-out,
six separate `JointTrajectory` publishes per tick) was the single largest
process in the profile (34.91% avg, >50% of the mirror-synthetic
container's total CPU) but is NOT changed by this pass -- it needs a code
restructuring (batching the 30 per-joint filter updates, coalescing the 6
per-controller publishes, or lowering `mirror_rate_hz`) rather than a
config/launch-level cut, and is out of scope here.

## Stale root-owned install/ masking builds

TODO: a container run that leaves `ros2_ws/build/`, `ros2_ws/install/`, or
`__pycache__/` owned by root (root-in-container writing into a bind-mounted
host `ros2_ws/`) makes a subsequent `colcon build` -- or even a plain
`python -m py_compile` from the host -- fail with a permission error that
reads as a code problem rather than an ownership one; the fix is reclaiming
ownership (or rebuilding clean), not editing the source that happened to be
open at the time.
