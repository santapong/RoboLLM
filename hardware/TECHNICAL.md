# RoboLLM · Physical-arm technical notes

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../README.md) · [Hardware guide](README.md) · [Arm architecture](../docs/physical-arm/ARCHITECTURE.md) · [Serial protocol](docs/serial_protocol.md)

The Arduino Mega 2560 intentionally runs a small line-oriented serial
controller. ROS 2, trajectory sampling, calibration
mapping, and host-side validation live in `ros2/robo_arm_driver`. The firmware
only accepts validated raw targets, enforces the same generated hard limits,
slews the servos, and de-energizes them when communication stops.

![Hardware architecture](docs/hardware-architecture.svg)

System-wide C4 context/container/component diagrams and the 4+1 architectural
views live in `../docs/physical-arm/ARCHITECTURE.md`. They label future
containers as planned so the existing simulation examples are not confused
with completed physical-arm phases.

## Safety layers

1. `validate_trajectory` checks exact joint names, finite values, logical joint
   limits, strictly increasing timestamps, and segment velocity.
2. `ArmSerial` converts radians once and rejects (never clamps) invalid values.
3. arm-fw 2.1 rejects raw values outside generated per-joint limits.
4. The firmware command watchdog detaches every servo after the configured
   timeout (750 ms initially).
5. A physical servo-power cutoff remains mandatory.

`state_source: commanded` is honest provenance: until real encoders replace
`readEncoderDeg()`, `/joint_states` reports the firmware's slewed command
estimate, not measured shaft position.

## Commissioning and production modes

With `calibrated: false`, firmware allows `Q`, `X`, `P`, `L`, and `C`; it
rejects `S`, `H`, and `E`. `C channel degree` moves one joint only inside the
narrow configured raw window. Once all six joints have measured limits and the
profile is reviewed, set `calibrated: true`, regenerate `arm_config.h`, compile,
and flash. Production firmware then disables `C` and accepts full trajectories.

## ROS 2 driver

```bash
cd ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch robo_arm_driver driver.launch.py port:=/dev/ttyACM0
```

| Interface | Type | Direction |
|---|---|---|
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | input |
| `/joint_states` | `sensor_msgs/JointState` | output |
| `/arm/status` | `std_msgs/String` JSON | output |
| `port` | parameter | serial device; empty means auto-detect |
| `config_file` | parameter | physical or simulation YAML |
| `enable_on_start` | parameter | ignored while commissioning lock is active |

The bridge samples validated trajectories at the configured control rate and
sends one batch action per tick. It relaxes the arm after any I/O exception and
on clean shutdown. `FollowJointTrajectory` action integration is intentionally
deferred to v0.3, when real timing and MoveIt behavior can be measured.

## Protocol simulator

`sim_uno.py` uses the same YAML, limits, rates, mode lock, and timeout as the
firmware. Use `joints.sim.yaml` explicitly for broad virtual motion; the default
physical profile stays locked. The simulator is the hardware-free development
target for the driver, teleoperation, and future controller tests.

## Toolchain notes

- Arduino CLI is expected at `~/.local/bin/arduino-cli` with the AVR core and
  Servo library installed rootlessly.
- Opening the Mega serial port toggles DTR and resets it; the host waits 2.5 s.
- ROS 2 Jazzy on Ubuntu 24.04 uses system Python/NumPy ABI; keep the repository
  constraints and use `--system-site-packages` virtual environments.
- Raspberry Pi 5 and laptop must share `ROS_DOMAIN_ID` when the driver runs on
  the Pi and RViz/MoveIt run on the laptop.
