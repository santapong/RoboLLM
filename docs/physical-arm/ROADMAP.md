# RoboLLM · Physical-arm delivery roadmap

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Documentation](../README.md) · [Architecture](ARCHITECTURE.md) · [Hardware worksheet](HARDWARE_WORKSHEET.md)

Status date: **2026-08-14**.

This track implements **RoboLLM — A Hybrid Classical and Learning-Based
Manipulation Platform for Language-Guided Robotics**. The repository-wide
`ROADMAP.md` remains the research/learning map; this document is the execution
record for the one physical arm.

## Honest completion status

| Phase | Target | Current status | Evidence / blocker |
|---|---|---|---|
| Phase 0 | Hardware foundation | **Software ready; hardware pending** | Fail-closed config, firmware, simulator, worksheet, and tests exist. Servo electrical/mechanical values and cutoff tests are still TODO. |
| Phase 1 | ROS 2 physical control | **Partial — v0.2.1 simulation-verified** | `robo_arm_driver` builds on Jazzy; standard `FollowJointTrajectory` success/cancel/rejection, compatibility topic, `/joint_states`, validation, status, serial PTY, and sim launch exist. No deployment-workstation, measured URDF, MoveIt hardware execution, or five-pose bench evidence yet. |
| Phase 2 | Webcam human mirroring | **Reusable simulation example only** | `examples/hand_follow` proves MediaPipe → filtering → IK → trajectory in RViz. It is not calibrated or safety-accepted on this arm. |
| Phase 3 | Autonomous vision pick/place | **Not started on the physical arm** | Gesture/MoveIt examples exist, but marker detection, camera calibration, base-camera TF, grasp state machine, and 20-trial physical report are absent. |
| Phase 4 | VLA / robot learning | **Sim dataset path ready; physical data pending** | The hardware logger refuses unmeasured state by default; the scripted MuJoCo arm writes and reads LeRobot v3 video episodes on CPU. No accepted physical demonstrations, trained policy, or classical-vs-learned evaluation exists. |
| Phase 5 | LLM robot planner | **Not started for this arm** | The repository has generic MCP/LLM tools, but no physical-arm skill schema, allowlist validator, world-state gate, or end-to-end language task. |

Therefore the project does **not** yet finish every phase in the proposed plan.
The current deliverable is a safe foundation that lets hardware work proceed
without creating a second software stack.

## Version milestones

| Version | Capability | Gate |
|---|---|---|
| v0.1 | Arduino commissioning | worksheet complete; HOME and cutoff repeatable |
| v0.2 | ROS 2 physical driver | named trajectory moves six joints; honest state follows |
| v0.3 | Measured URDF + MoveIt | five repeatable Cartesian poses on hardware |
| v0.4 | Webcam mirroring | filtered, clamped hand-to-pose control and gripper gesture |
| v0.5 | Marker-first pick/place | calibrated camera/TF and explicit skill state machine |
| v0.6 | Natural-object detection + verification | measured 20-trial result and failure breakdown |
| v0.7 | Demonstration dataset | synchronized image/state/action/instruction episodes |
| v0.8 | Learned/VLA manipulation | classical-vs-policy evaluation with safety wrapper |
| v0.9 | LLM skill planner | schema-valid, allowlisted, world-state-checked plans |
| v1.0 | Hybrid LLM + VLA + classical robotics | end-to-end task with measured success and recovery |

## Immediate parallel work

### Hardware owner

- Complete [HARDWARE_WORKSHEET.md](HARDWARE_WORKSHEET.md).
- Verify external power, common ground, cutoff, servo direction, safe limits,
  home values, axes, gripper endpoints, and link dimensions.
- Update `joints.yaml`, regenerate firmware configuration, review the diff, and
  only then set `calibrated: true`.
- Record photos, measurements, failures, and acceptance evidence.

### Software track after measurements arrive

- Generate `robo_arm_description` from measured axes and dimensions.
- Create measured description/bringup packages after hardware dimensions arrive.
- Build the MoveIt configuration from the measured model.
- Run RViz ↔ arm direction checks, HOME, and five repeatable poses.
- Promote the existing hand-follow pipeline only after the Phase 1 safety gate.

## Acceptance gates

### Phase 0 / v0.1

- [x] Physical profile defaults to `calibrated: false`.
- [x] Commissioning commands are narrow, raw, and single-joint only.
- [x] Firmware config is generated from the canonical host YAML.
- [x] Simulator mirrors calibration lock, limits, dynamics, and watchdog.
- [ ] Servo voltage and stall current recorded.
- [ ] External supply, common ground, and physical cutoff verified.
- [ ] Six joint limits, directions, home values, and gripper endpoints measured.
- [ ] Invalid command and USB-loss torque-off observed on the physical bench.
- [ ] HOME repeated ten times without binding, brownout, or Arduino reset.

### Phase 1 / v0.2–v0.3

- [x] Exact joint-name, finite-value, limit, time, and velocity validation.
- [x] Valid trajectories interpolated at the configured control rate in tests.
- [x] `/joint_states` publishes configured names with honest provenance.
- [x] `/arm/status` reports calibration, source, activity, and error state.
- [x] Standard `FollowJointTrajectory` goal, feedback, cancel, result, and busy-goal boundary implemented.
- [x] Explicit PTY simulation launch profile is separate from the fail-closed physical profile.
- [x] ROS 2 Jazzy container build, launch, action feedback/success,
  cancellation, and invalid-goal rejection pass against the PTY protocol simulator.
- [ ] ROS 2 Jazzy package builds and launches on the deployment machine.
- [ ] Arduino firmware compiles/flashes with the real toolchain.
- [ ] RViz and physical motion agree in joint direction.
- [ ] Measured URDF represents joint origins, axes, links, and gripper.
- [ ] MoveIt plans and executes HOME plus five repeatable target poses.

### Phase 2

- [ ] Camera-to-normalized-human-to-robot workspace mapping calibrated.
- [ ] Low-pass/One-Euro smoothing, dead zone, and velocity limits tuned.
- [ ] Workspace, IK, collision, joint, and staleness gates verified.
- [ ] Open/fist gripper behavior works without unsafe transients.
- [ ] Latency, jitter, and successful teleoperation runs recorded.

### Phase 3

- [ ] Intrinsic calibration and camera-to-base transform measured.
- [ ] ArUco/AprilTag pose becomes `base_link` object pose through TF.
- [ ] SEARCH → APPROACH → GRASP → LIFT → PLACE → VERIFY skills implemented.
- [ ] Marker task passes before natural-object detection begins.
- [ ] Twenty physical attempts report detection, planning, grasp, placement,
  overall success, and execution time.

### Phase 4

- [x] LeRobot v3 schema records synchronized frames, honest state, action,
  gripper, instruction, and camera lag; commanded-state recording is an
  explicit simulation-only override.
- [x] Scripted MuJoCo arm policy writes and reads a LeRobot v3 video episode.
- [ ] A physical dataset is recorded after encoder state is available.
- [ ] Demonstrations pass quality checks before training.
- [ ] Fixed-task policy is tested in simulation before physical deployment.
- [ ] Learned actions cross the same safety validator as classical actions.
- [ ] Classical and learned pipelines are compared under matched conditions.

### Phase 5

- [ ] High-level skill schemas and allowlist are versioned.
- [ ] LLM plans contain no joint/PWM/serial commands.
- [ ] World-state and precondition validation rejects impossible plans.
- [ ] Every skill routes through MoveIt or a safety-wrapped policy.
- [ ] Scene inspection and success verification close the task loop.
- [ ] Language task success, failure, and recovery are measured.

The next phase opens only when the preceding physical acceptance gate has
evidence. C4 and 4+1 diagrams are maintained in [ARCHITECTURE.md](ARCHITECTURE.md).
