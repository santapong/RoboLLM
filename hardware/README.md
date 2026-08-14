# RoboLLM · DIY 6-DOF arm hardware

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../README.md) · [Arm roadmap](../docs/physical-arm/ROADMAP.md) · [Worksheet](../docs/physical-arm/HARDWARE_WORKSHEET.md) · [Technical notes](TECHNICAL.md)

**Status:** code-ready and bench-gated. The checked-in physical profile is
fail-closed until real measurements are recorded and reviewed.

The physical path is deliberately layered:

```text
JointTrajectory (rad) → robo_arm_driver → serial (deg) → arm-fw → PWM
```

AI code never sends PWM or raw servo angles. The installable ROS package and
canonical joint configuration live in `ros2/robo_arm_driver/`; this directory
owns the Uno firmware, simulator, bench tools, and compatibility launchers.

## Start here

1. Read and complete `../docs/physical-arm/HARDWARE_WORKSHEET.md`.
2. Provide an external 5–6 V servo supply with common ground and a cutoff.
3. Install serial permissions once, then reconnect your session:

   ```bash
   sudo usermod -aG dialout "$USER"
   ```

4. Plug in the Uno and run `hardware/check_arduino.sh`.
5. Commission one unloaded or mechanically safe joint at a time.

The physical config starts with `calibrated: false` and 85–95° limits. Normal
`HOME`, `ENABLE`, and six-joint commands are rejected until calibration is
complete. The only motion available before then is the bounded single-joint
commissioning command:

```bash
python3 hardware/arm_serial.py commission 0 91
python3 hardware/arm_serial.py relax
```

After editing `ros2/robo_arm_driver/config/joints.yaml`, synchronize firmware:

```bash
python3 hardware/generate_firmware_config.py
make -C hardware/firmware/arm_firmware
```

Review both the YAML and generated header before flashing.

## Files

| Path | Role |
|---|---|
| `firmware/arm_firmware/` | arm-fw 2.1, strict limits, slew control, timeout, e-stop |
| `generate_firmware_config.py` | generates firmware limits from canonical YAML |
| `arm_serial.py` | compatibility CLI for `robo_arm_driver.arm_serial` |
| `arm_bridge_node.py` | compatibility launcher for the ROS package node |
| `sim_uno.py` | protocol/dynamics simulator on a pseudo-terminal |
| `check_arduino.sh` | toolchain, port, compile, flash, ping, and LED check |
| `acceptance_test.py` | state/motion/camera synchronization bench check |
| `camera_logger.py` | synchronized image/state/action episode logger |
| `lerobot_logger.py` | minimal one-camera LeRobot v3 demonstration recorder |

## LeRobot dataset recording

LeRobot 0.6 needs NumPy 2.x, so keep it separate from the ROS Jazzy venv,
which must remain on NumPy 1.26.4:

```bash
python3 -m venv .venv-lerobot
.venv-lerobot/bin/pip install -r requirements-lerobot.txt
```

After encoders are installed and `state_source: measured` is configured:

```bash
ARM_PORT=/dev/ttyACM0 .venv-lerobot/bin/python hardware/lerobot_logger.py \
  --task "pick up the blue block" --steps 150 --fps 15
```

The recorder writes LeRobot's standard video, Parquet, task, and metadata
layout under `datasets/robollm-arm/`. It refuses commanded-state data by
default so an open-loop servo target cannot be mislabeled as a measurement.
Use `--allow-commanded-state` only for a simulation pipeline check.

## Simulation without hardware

Terminal 1:

```bash
ARM_CONFIG=ros2/robo_arm_driver/config/joints.sim.yaml \
  python3 hardware/sim_uno.py
```

Terminal 2, using the printed pseudo-terminal:

```bash
ARM_CONFIG=ros2/robo_arm_driver/config/joints.sim.yaml \
ARM_PORT=/dev/pts/N python3 hardware/arm_serial.py state
```

## Wiring invariant

- Joint signals: Uno pins 3, 5, 6, 9, 10, 11; gripper signal: pin 4.
- Servo power: external supply sized from measured stall current.
- Ground: supply GND ↔ Uno GND ↔ every servo GND.
- Never power six servos from the Uno 5 V pin.

The protocol reference is `docs/serial_protocol.md`; the ROS interface is
documented with C4 and 4+1 SVGs in
`../docs/physical-arm/ARCHITECTURE.md`. Phase completion and bench evidence are
tracked in `../docs/physical-arm/ROADMAP.md`.
