# Verification — 9 September 2026

Scope: a supported Unitree G1 in MuJoCo, classical IK and position servos,
browser controls, and an opt-in local camera input. The source is on
`experiment/harness-vlm-wam`; no humanoid learning or ROS integration was run.
Two agents independently implemented/reviewed simulation and camera input;
the primary agent integrated the viewer and reran the combined checks.

## Measured simulation

Pinned Menagerie revision `8161bba264d7fa7c99ca301e91e7fb44737676ad`;
38 upstream files verified. The simulator has 29 qpos, 29 velocity DOFs and
29 actuators after removing the floating-base joint in memory. Physics timestep
is 2 ms; controller/viewer updates are 40 ms.

Each episode resets, sends one target, and simulates five seconds under gravity.

| Arm | Target XYZ (m) | Initial error | Final error |
| --- | --- | --- | --- |
| Right | 0.30, −0.26, 1.10 | 328.1 mm | 9.0 mm |
| Right | 0.34, −0.32, 1.22 | 463.9 mm | 13.7 mm |
| Left | 0.30, 0.26, 1.10 | 328.1 mm | 9.1 mm |

No joint-limit violations or numerical faults were reported in these episodes.
The demo's `reached` field means instantaneous error below 35 mm; it is not the
UR5e VLA-bed dwell/safety success metric. This is a controller smoke experiment,
not a population success estimate or evidence of learned-policy generalization.

[Raw report](results/scripted-reaches-2026-09-09.json) records model revision,
library versions, controller, timing and per-episode state. EGL renders in
ignored `artifacts/reach-*.png` were visually inspected.

## Automated checks

- 14 dynamics tests: both arms, fixed pelvis, joint setpoint slew, invalid
  commands without state mutation, workspace clamp, hold, poses, a moving-target
  demo and reset.
- 3 observation adapter tests: missing/stale/invalid observations, fixed depth,
  coordinate bounds and Euclidean target slew.
- 3 app regressions using real MuJoCo and fake GUI/camera handles: selecting an
  arm preserves hold; same-tick arm/manual activation uses the selected arm's
  workspace; same-tick arm/camera activation holds the selected arm.
- 8 camera tests using generated pixels/fakes: opt-in lifecycle, image mapping,
  lost/stale/invalid detections, release and preview clearing.
- 19 bridge cases: actual capture freshness despite repeated packets, invalid
  timestamps, startup, old-reader/restart races, EOF, reaping a stuck owned
  worker, and preserving useful camera startup errors.

**47 tests passed**, with no skips in the combined verification runs. The
repository's Ruff configuration passes for all new Python files. Camera tests
ran in the existing Elexhub environment; app/dynamics tests ran in the existing
Viser environment. No dependencies were installed.

## Browser and actual camera

The local viewer rendered the G1, support stand, target, hand trail and controls
at about 24–25 updates/s on this machine. Scripted reaching, wave, hold and arm
selection were exercised through the visible browser. Dragging the target's
vertical handle changed its height; the right hand settled to 10.6 mm error
at target `[0.31283, -0.30333, 1.11371]` with finite state and no joint-limit
violation. The app also completed
an eight-second camera-off startup/shutdown smoke check.

An earlier bounded direct camera check captured **20 actual frames over three
seconds** from `/dev/video0`, initialized the MediaPipe hand backend, and
produced in-memory JPEG previews. It detected **no hand**. Capture stopped, the
worker exited, and preview memory was cleared. This verifies capture/inference
availability, not a demonstrated person-to-humanoid motion sequence.

During later browser integration, `/dev/video0` was occupied by a separate
`examples/elexhub/run.py` process (observed with `fuser` and its entry point).
Opening the camera therefore failed. The humanoid stayed in HOLD with finite
joint state and no limit violation. The other application's process was left
running. The viewer now reports the missing/busy cause and allows a retry after
the user closes the other camera session. Camera ownership is volatile and
must be checked again before making a present-tense availability claim.

No camera frames were persisted during verification. No physical robot, Pi,
Kaggle job, cloud model/API, or public deployment was involved.
