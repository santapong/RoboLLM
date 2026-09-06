# Hardware and webcam runbook

## Measurements required from the actual arm

Capture front, side, top and gripper photographs with a ruler in the same plane. Record joint-center distances, axis directions, link widths/depths, base mounting height, jaw opening, jaw linkage and approximate link masses. Record the raw home pose and actual servo model at each channel. Product descriptions such as “6 DOF” do not establish the number of independently controlled arm axes.

Read the battery label and identify chemistry, series-cell count, full/discharged voltage, continuous-discharge rating, BMS, matching charger, regulator input range, regulated output voltage, continuous/peak current, wire ratings, cutoff rating and fuse. Read servo datasheets for their voltage range and stall current. These fields are intentionally blank in the profile. Do not choose a supply voltage from the PCA9685 board name or assume an unspecified Li-ion pack is compatible.

The checker requires the regulated rail to fall inside every servo's rated voltage range, checks battery/regulator input compatibility and upstream fuse ratings, and conservatively requires continuous regulator capacity at least the sum of servo stall currents. This is a data consistency gate; the measurement and power-review records must come from actual inspection. Evaluate regulator efficiency, transient loads, wire heating and protection placement during the bench review.

## Wiring

| Connection | Destination |
|---|---|
| Pi/computer USB | Arduino Mega USB |
| Mega D20 (SDA) | PCA9685 SDA |
| Mega D21 (SCL) | PCA9685 SCL |
| Mega 5 V logic | PCA9685 VCC, following the module's logic requirements |
| Mega GND | PCA9685 GND and regulated servo-supply common ground |
| Mega D22 | PCA9685 OE; verify a pull-up/default-disabled state during reset |
| PCA channels 0–4 signal | Base, shoulder, elbow, wrist pitch, wrist roll |
| PCA channel 5 signal | Gripper servo |
| Protected battery | Correctly rated fuse and physical cutoff, then regulator/BEC |
| Regulated rail | Properly rated servo power distribution / PCA V+ where its connector and traces are rated for the load |

PCA9685 VCC is logic power; V+ is the separate servo rail on common breakout modules. Do not power six servos from Mega 5 V or Pi USB. Check actual board terminal polarity and servo connector pin order. Supply distribution may need to bypass an inadequately rated breakout power terminal while retaining shared ground and PWM signals.

OE high stops PWM outputs; it does not remove motor power and does not prove that the servo releases torque. The physical cutoff must interrupt the servo supply through a reviewed power path. Mechanically support the arm during disable, cutoff and reset tests because gravity can move it. The implementation never claims measured position, current or torque from PCA9685.

Reference sources: [NXP PCA9685 datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf), [Adafruit breakout wiring guide](https://learn.adafruit.com/16-channel-pwm-servo-driver/hooking-it-up). Use the supplied wiring PDF and `wiring.html` for the connection overview.

## Firmware and profile workflow

`calibrated: false` and `commissioning_allowed: false` are the shipped settings. First fill the power, polarity, cutoff, default-OE and topology evidence; identify all six servos and pulse ranges. A commissioning profile needs a documented home and a **maximum four-raw-unit** window for each actuator. Keep `calibrated: false`, set `commissioning_allowed: true` only for the reviewed bench session, and regenerate the header.

`pulse_min_us` and `pulse_max_us` correspond to the configured raw minimum and maximum, respectively. They are linearly mapped over that configured range, then quantized into 50 Hz PCA ticks. Joint raw units are degrees; gripper raw units are percent. They are command units, not measured angles. An oscilloscope/pulse check must establish the relationship for the actual module oscillator and servo before mechanical commissioning. The supplied zero pulse fields prevent outputs.

```sh
.venv/bin/python hardware/generate_firmware_config.py --config ros2/robo_arm_driver/config/elexhub.yaml --output hardware/firmware/elexhub_pca9685/arm_config.h
arduino-cli compile --fqbn arduino:avr:mega hardware/firmware/elexhub_pca9685
```

Compilation is hardware-free. Upload is a later, explicit bench operation to the verified board/port. Do not use the legacy `hardware/check_arduino.sh` for this task: that workflow can flash the older direct-PWM firmware.

Protocol 3.0: `P` capabilities; `Q` state; `E` explicit enable; `S q0 … q4 gripper` raw setpoint; `H` home; `C index raw` narrow single-actuator commissioning; `K` hold current commanded state; `X` disable PWM. Errors disable outputs and latch faults. Ordinary motion needs a new explicit `E` after inspection. A commissioning fault requires a controlled board reset because `E` remains unavailable to an uncalibrated profile. Stop and support the mechanism before reset; startup keeps PWM disabled.

The host-facing API accepts radians and normalized gripper values. The canonical driver handles raw conversion. A protocol/profile mismatch closes the connection. A read query cannot keep a pending motion alive. Verify all these behaviors on the bench before setting every actuator's `calibration_verified: true` and the global `calibrated: true`.

## Print and calibrate

Print the reference pack on **A4, actual size / 100%, no fit-to-page**. Measure the 100 mm scale. ChArUco uses `DICT_5X5_100`, 7 × 5 squares, 30 mm square width, 22 mm marker width. The workspace grid is 180 × 120 mm, 10 mm cells. Workspace marker IDs are 50–53. Tool tag 60 has a 20 mm black edge; box tag 70 has a 15 mm black edge. The white margin is not part of the marker's measured size. Use a rigid, flat print; mark the robot-base axes on the bench separately.

Capture at least 12 usable views (20 recommended) at the final camera resolution, varying board position and tilt:

```sh
.venv/bin/python examples/elexhub/run.py capture-calibration --camera 0 --output captures/calibration
.venv/bin/python examples/elexhub/run.py calibrate captures/calibration/*.png --output captures/intrinsics.json
```

The first command opens a real webcam and saves local files, so run it when the board and workspace are ready. It is not run by the simulator. Calibration requires at least ten ChArUco corners per accepted view, sufficient tilt diversity, consistent resolution and reprojection RMS ≤1 pixel. Raw captures are ignored by Git.

Measure at least six non-collinear table-to-base point pairs and six separate held-out points. Create a JSON file following `registration-example.json`; **the example contains no measurements and must not pass a physical gate**. Measure the transform from tag 60 to the tool center point, including rotation. Transforms map column vectors from the named source frame into the named destination frame, in meters. The mat is right handed, with +Z out of the tabletop.

```sh
.venv/bin/python examples/elexhub/run.py register --calibration captures/intrinsics.json --measurements captures/registration-measurements.json --output captures/camera-registered.json
```

Registration computes a rigid least-squares fit and evaluates held-out base-coordinate errors; it rejects overlapping fit/validation points and RMSE above 5 mm. Separately check projected webcam points against ruler/grid positions across the working area; registration residual alone is not an independent camera-accuracy measurement. Update `geometry.table` from the measured transform; hardware mode rejects a disagreement with the camera calibration. Recalibrate after moving the camera, mat, tool tag or robot base. Never edit a calibration file and retain its old identity.

## Webcam shadow mode

```sh
.venv/bin/python examples/elexhub/run.py serve --mode shadow --calibration captures/camera-registered.json
```

Show the workspace markers, tagged red box and tool tag. Switch to Webcam. Point by touching/hovering above a mat location, then click **Latch pointed object** or **Latch pointed destination**. Pixel projection is not a distant arm-pointing ray. A click on the webcam can also choose a destination. Resolve ambiguity before motion. The task parser supports only the displayed reach/move vocabulary; speech merely fills the text field. Shadow mode can preview and record perception but refuses execution.

Require a fresh reference, stable target, visible tool marker, clear workspace and acceptable end-to-end latency. Test missing/occluded markers, two hands, ambiguous red regions, camera disconnect and stale calibration. The learned hand detector can miss a hand; keep an independent physical cutoff and a controlled operator workspace.

## Hardware mode: after bench verification

Source the project's ROS 2 environment, build `robo_arm_driver`, and launch its `arm_bridge` executable with the measured `config_file`, explicitly selected `port`, and `enable_on_start:=false`. The ROS package must include `std_srvs`; this change adds `/arm/enable` and `/arm/gripper_command`. Never run the driver and another serial client on the same board simultaneously.

The console adapter reuses the existing shared `robollm.bridge.get_bridge()` node. Start `serve --mode hardware --calibration captures/camera-registered.json` only with the measured physical profile. It refuses the shipped profile before opening a camera or constructing ROS transport. Startup does not enable PWM. Use the explicit enable control only after mechanically establishing the calibrated home configuration and reviewing the bench checklist. Arm motion then still needs preview and confirmation.

A/B/C controller labels are unchanged between domains, but physical B/C requires `actuator_identification.verified: true`, a real identification `record`, and measured values in `geometry.actuator_reference`. There is no automatic identification routine that treats PWM values as torque/encoder feedback. Begin with A and camera measurement; fit and validate the actuator model offline before comparing B/C.

This release has no Pi service installation and has not tested ROS transport against the real arm. Validate the ROS action, gripper, supervisor lease, serial watchdog and cutoff together before collecting physical trials. Record all rejected or interrupted attempts in the prepared CSV rather than deleting them.
