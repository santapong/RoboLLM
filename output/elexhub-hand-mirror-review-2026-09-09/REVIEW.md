# Elexhub hand-mirror verification — 9 September 2026

The photo-referenced robot now supports local webcam detection and relative
left-hand control in MuJoCo at `http://127.0.0.1:8767/mirror`. Camera and movement
start separately. Left/right and up/down hand movement steer the gripper;
apparent palm size supplies a small forward/back control; thumb–index pinching
closes the claw. This is gripper steering, not human shoulder/elbow imitation.

## Verified software and model

- **117 unique relevant tests passed** across the existing model/experiment and
  camera suites, plus the new hand camera, controller and web integration suites.
  See `VALIDATION.json` for the separate runs.
- A 120-sample synthetic hand sweep exercised the real controller, IK and MuJoCo.
  Actual TCP spans were **16.56 / 24.79 / 24.47 mm** in X/Y/Z, with zero rejected
  steps and **1.617 mm** error at the final target. The gripper traversed fully
  open to closed. These are software/simulation measurements.
- The 54 reviewed model artifact hashes, 36 protected profile/firmware/frozen
  evaluation hashes, CAD descriptor identity, and all 77 ZIP manifest members
  still match. Starting the console now loads existing assets rather than
  regenerating the reviewed CAD/model files. See `MODEL-INTEGRITY.json`.
- Right/uncertain/ambiguous hands and samples older than 350 ms cannot drive the
  arm. Loss of input clears delayed commands and holds the current simulated
  pose. Reacquisition anchors at the current pose. A lost control heartbeat stops
  following after 1.5 seconds. Task execution and mirroring exclude each other.
- Camera calibration and mirroring share exclusive device ownership. Capture
  workers release their own devices. Hardware/shadow modes reject mirroring.

## Real camera and browser observations

Camera **0** delivered a real **640×480** preview through the locally cached,
checksum-verified MediaPipe model. A six-second, 24-sample observation saw roughly
**8.2–15.8 fps**, with frame ages **59–448 ms**; older samples correctly reported
unavailable for control. The detector saw no hand during that observation and
the simulated TCP remained unchanged while waiting for the left hand.

Visible browser checks passed: camera startup, the two live panels, missing
device 63 showing a persistent explanatory error, recovery on camera 0,
Start mirroring waiting for a hand, Hold position, and Camera off. Browser console
error logs were empty. The final camera frame endpoint returned HTTP 204 after
stopping; the simulation image remained a separate source.

No camera photos or videos were recorded. The page was left open with camera and
mirroring **off**. The loopback console is a task-local process; check availability
before reusing it. No commit, push or deployment was performed.

## Remaining verification

The operator has not yet demonstrated anatomical left versus right, comfortable
movement mapping, or pinch/reacquisition with this camera orientation. Synthetic
hand tests do not establish that live user result. Start camera, show the left
palm, then Start mirroring to perform it. Instructions are in
`docs/physical-arm/elexhub/HAND-MIRROR.md`.

Robot dimensions, joint zeros, masses and actuator behavior remain provisional.
The visible gripper linkage still uses coupled sliding contact jaws. This work
does not establish a measured digital twin or physical robot readiness. Physical
calibration, hardware motion, PCB work, Pi operations and model training were
outside this task.
