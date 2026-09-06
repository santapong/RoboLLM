# Implementation plan and completion gates

## Research question

For a small hobby-servo arm controlled by Mega/PCA9685, does camera-observed TCP correction improve tabletop reaching and pick/place over calibrated open-loop commands and actuator-model compensation?

The first deliverable is a reproducible measured-robot experiment. Learning a new VLA policy, transferring a UR5e checkpoint, and changing Kaggle training are outside this implementation. The two embodiments have different kinematics and action spaces; a five-joint Elexhub policy cannot be obtained by relabeling UR5e joint actions.

## Stages

| Stage | Implementation delivered | Acceptance gate still requiring hardware |
|---|---|---|
| 1. Inventory and measurement | Single profile, readiness checker, printable worksheet | Photographs; link/joint centers; servo labels; power-chain ratings; gripper topology |
| 2. Digital model | Parameterized CAD/STEP/OBJ/MJCF/URDF; FK parity and collision checks | Replace provisional geometry, verify several measured poses and jaw travel |
| 3. Actuation | Variable joint count; PCA protocol 3.0; handshake, OE, bounds, slew, timeout; PTY peer | Wiring inspection; power/current checks; polarity; cutoff; pulse calibration; repeated home tests |
| 4. Webcam | ChArUco calibration, mat pose, metric ray-plane projection, tool/box tags, hand selection, offline speech | Calibrate the user's webcam, measure base/tag transforms, test held-out points and latency |
| 5. Task execution | Preview/token/expiry/confirmation; bounded reach and pick/place; abort and outcome verification | Shadow checks followed by slow physical reach; only then pick/place |
| 6. Evaluation | Frozen suite, manifests, paired summaries, Wilson intervals, blank physical schedules | Actual A/B/C runs and intent/pick-place trials, with failed attempts retained |

## Architecture and contracts

The console selects an object and destination, parses a small command vocabulary, and produces a preview. After confirmation, the executor calls either the CPU simulator or the measured-arm adapter. The hardware adapter uses the existing `get_bridge()` singleton. `robo_arm_driver` is the only serial owner and sends commands through Mega to PCA9685. Camera measurements remain separate from commanded joint state.

The checked-in firmware cannot enable an actuator: both enable flags are false and unknown pulse endpoints compile to zero. A simulation profile is allowed to open only an explicit `/dev/pts/` peer; it cannot enumerate or open USB. A hardware profile needs evidence before the serial port can be opened. At connection, host and firmware must agree on protocol, backend, profile hash, runtime and joint count.

All coordinates use meters and radians. Table +X runs right across the printed grid, +Y up the grid, +Z out of the table. The demonstrated table-to-base transform is provisional and must agree with the measured registration before physical execution. Five hinges control the arm; the sixth motor controls opening. No extra Cartesian orientation degree of freedom is invented. Position-first reaches and feasible downward grasps use bounded IK; unreachable targets are rejected.

Nominal joint paths are collision-checked at interpolated samples, then watched during simulated execution and settling. Grasped-object motion is predicted for collision checks, while the actual simulated box always follows contact physics. Preview tokens expire after 30 seconds and are consumed once. A changed starting pose or selected object invalidates execution. Camera references older than 300 ms, changed calibration, missing markers, detected hands or faults stop execution. RGB human detection cannot guarantee an empty workspace.

The firmware watchdog requires fresh motion/hold commands within 750 ms; `Q` queries never refresh it. `K` holds the current commanded state rather than continuing an old target. The ROS supervisor lease expires after 600 ms without the operator heartbeat. The local console refreshes that heartbeat only while connected and, in hardware mode, while the camera checks pass. OE disables PWM; a physical cutoff disconnects the servo supply. All reported joint angles are commanded values, not encoder measurements.

## Experiment definitions

| Condition | Controller | Observation used for correction |
|---|---|---|
| A | Nominal joint-position servo model | None |
| B | A plus gravity-bias compensation from the actuator model | None |
| C | B plus bounded corrections, at most 2 mm per correction and 12 corrections | Simulator TCP in this release's synthetic run; calibrated webcam TCP in physical mode |

The physical B/C path refuses provisional actuator compensation. It requires an actuator-identification record and updated model parameters. Without current/encoder sensing, fit an empirical command-to-observed-motion/deflection model from camera data and held-out poses; do not report torque identification from PCA9685 data.

The synthetic suite uses seed `20260906`, 100 fixed goal/height samples and all three conditions on identical scenes. Seeds identify goal sampling, not randomized physics. Order rotates across conditions. Success requires TCP error at most 15 mm continuously over 0.5 simulation seconds. Store every failure, the full profile, source snapshot, versions and suite digest. Report continuous error as well as success rate. A suite on which every condition succeeds is a ceiling effect, not proof of real-world robustness.

For physical reaching, use 12 targets × 3 repetitions × 3 conditions = 108 attempts, keeping target positions and initial poses paired. Pilot reachability before freezing the physical schedule; edits create a new schedule ID. Log webcam TCP, commanded joints, condition, calibrated target, completion time, collision/abort outcome and all failures. Report Wilson success intervals and paired comparisons; report missing observations explicitly.

Separately perform 30 interface trials including nearby/ambiguous objects, and 20 tagged red-box pick/place trials. Report source/destination interpretation accuracy, clarification rate and selection latency separately from motor success. Pick/place requires an observed lift above 35 mm, release within 20 mm XY of the destination, and 0.5-second stable verification. Record tag loss, hand intrusion, disconnects and failure phase.

## Order for the physical experiment

1. Fill the measurement sheet and source profile. Verify power/polarity with servo power disconnected.
2. Test the cutoff and default-disabled OE. Commission one mechanically supported, unloaded actuator in a narrow range.
3. Measure raw endpoints, sign, zero, joint limits, rates, backlash and home repeatability. Generate fresh firmware/profile hashes.
4. Replace CAD dimensions and compare FK against several measured configurations. Verify gripper linkage/travel and conservative collisions.
5. Print at 100% scale, measure the scale bars, calibrate the webcam, register base/tool markers, and measure held-out image-to-table errors.
6. Run webcam shadow mode. Verify object/target selection, instruction transcription, stale-camera rejection and hand-clear behavior without motion.
7. With the physical cutoff within reach, explicitly enable the measured profile and execute one low-speed reach. Validate abort, host disconnect, webcam disconnect and firmware watchdog under controlled support.
8. Freeze and collect the A/B/C reach dataset. Only after reliable reaching, execute tagged red-box pick/place and interface trials.
9. Review failures and confidence intervals before considering any new learning experiment or Kaggle integration.
