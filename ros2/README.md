# RoboLLM · ROS 2 physical-arm workspace

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../README.md) · [Documentation](../docs/README.md) · [Arm roadmap](../docs/physical-arm/ROADMAP.md) · [Driver package](robo_arm_driver/)

**Status:** v0.2.1 driver foundation builds and passes its action path in a ROS
2 Jazzy container; deployment-workstation and physical-motion acceptance remain
open. No physical calibration or firmware setting is changed by the simulation
workflow below.

This tree contains the installable ROS 2 packages for the physical RoboLLM arm.
Simulation and learning examples remain under `examples/`; hardware-facing code
graduates here only when it is part of the supported physical-arm path.

Current milestone: **v0.2.1** (standard `FollowJointTrajectory` action or
compatibility `JointTrajectory` topic -> validated Arduino commands).
The URDF and MoveIt packages will be added after the Phase 0 measurement sheet
is complete so that guessed geometry never becomes a hidden safety assumption.

This milestone is code-complete only at the hardware-free foundation level.
Deployment-workstation launch evidence and physical motion acceptance remain
open in `../docs/physical-arm/ROADMAP.md`.

## Runtime interfaces

| Interface | Contract |
|---|---|
| `/arm_controller/follow_joint_trajectory` | Standard `control_msgs/action/FollowJointTrajectory`; one active goal, feedback, cancellation, terminal result |
| `/arm_controller/joint_trajectory` | Compatibility topic; rejected while an action goal is active |
| `/joint_states` | Configured names and logical radians; `commanded` provenance until encoders exist |
| `/arm/status` | Calibration, state source, topic/action activity, last error, Arduino time |

The action boundary rejects unsupported scheduled starts and non-empty
path/goal tolerances. Those tolerances would imply measured feedback that the
current hobby-servo arm does not have. Invalid names, values, limits, timing,
or speed are rejected before serial execution.

## Hardware-free ROS bringup

Build and source the package in a ROS 2 Jazzy workspace, then keep these in two
terminals from the repository root:

```bash
# Terminal A: no Arduino or servos
ARM_CONFIG=ros2/robo_arm_driver/config/joints.sim.yaml \
  python3 hardware/sim_uno.py

# Terminal B: use the /dev/pts/N printed by Terminal A
ros2 launch robo_arm_driver simulation.launch.py port:=/dev/pts/N
```

Send a standard action goal:

```bash
ros2 action send_goal --feedback \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.15, 0.10, 0.05, 0.0, -0.05, -0.10], time_from_start: {sec: 2}}]}}"
```

This verifies the ROS/action/validation/serial protocol path against the PTY
simulator. It is not evidence that ROS has moved the physical arm.

For the repeatable success/cancel/rejection probe, run this in a third sourced
terminal after the simulator and launch file are active:

```bash
python3 tests/integration/ros/arm_driver_action_probe.py
```
