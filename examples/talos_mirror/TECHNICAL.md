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

## Stale root-owned install/ masking builds

TODO: a container run that leaves `ros2_ws/build/`, `ros2_ws/install/`, or
`__pycache__/` owned by root (root-in-container writing into a bind-mounted
host `ros2_ws/`) makes a subsequent `colcon build` -- or even a plain
`python -m py_compile` from the host -- fail with a permission error that
reads as a code problem rather than an ownership one; the fix is reclaiming
ownership (or rebuilding clean), not editing the source that happened to be
open at the time.
