# RoboLLM · Phase 0 hardware worksheet

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Documentation](../README.md) · [Arm roadmap](ROADMAP.md) · [Architecture](ARCHITECTURE.md)

**Status:** bench-gated record. Do not mark a checkbox from simulation or
assumption; attach the actual measurement or observed safety result.

Complete this at the bench before changing `calibrated: false` to `true`.
Photograph the wiring and write measured values here; do not rely on servo
labels or generic 0–180° assumptions.

## Power and cutoff

- Servo model(s): **TODO**
- Rated voltage: **TODO V**
- Stall current per servo: **TODO A**
- External supply rating: **TODO V / TODO A**
- Common ground verified: **TODO**
- Physical power cutoff location/test: **TODO**
- Arduino USB remains powered when servo power is cut: **TODO**

Never power the servos from the Mega 5 V pin. For first motion, disconnect the
linkage or lift the horn off the spline, center the unloaded servo at 90°, cut
servo power, then attach the horn at the intended mechanical zero.

## Joint calibration

Fill one row at a time. Approach an endpoint slowly and stop before binding,
high current, buzzing, or link collision. Back away from the observed endpoint
to create an operating margin.

| Joint | Servo/model | Pin | Axis | Raw safe min ° | Raw home ° | Raw safe max ° | Sign | Max °/s | Link length mm | Verified |
|---|---|---:|---|---:|---:|---:|---:|---:|---:|---|
| joint1 | TODO | 3 | TODO | TODO | TODO | TODO | TODO | TODO | TODO | ☐ |
| joint2 | TODO | 5 | TODO | TODO | TODO | TODO | TODO | TODO | TODO | ☐ |
| joint3 | TODO | 6 | TODO | TODO | TODO | TODO | TODO | TODO | TODO | ☐ |
| joint4 | TODO | 9 | TODO | TODO | TODO | TODO | TODO | TODO | TODO | ☐ |
| joint5 | TODO | 10 | TODO | TODO | TODO | TODO | TODO | TODO | TODO | ☐ |
| joint6 | TODO | 11 | TODO | TODO | TODO | TODO | TODO | TODO | TODO | ☐ |
| gripper | TODO | 4 | open/close | TODO | open: TODO | TODO | — | TODO | — | ☐ |

## Commissioning loop

1. Leave `calibrated: false`.
2. Edit only the narrow limit for the joint currently on the bench.
3. Run `python3 hardware/generate_firmware_config.py` and flash the firmware.
4. Start at the configured home value, with a hand on the power cutoff.
5. Move in 1–2° increments:

   ```bash
   python3 hardware/arm_serial.py commission 0 91
   python3 hardware/arm_serial.py commission 0 92
   python3 hardware/arm_serial.py relax
   ```

6. Record conservative endpoints and repeatability, then return to home.
7. After all rows are verified, update the YAML, regenerate the header, review
   the diff, set `calibrated: true`, regenerate again, compile, and flash.

## Phase 0 exit record

- HOME repeated 10 times without binding/reset: **TODO**
- Invalid raw command rejected: **TODO**
- USB disconnect causes torque-off within 750 ms: **TODO**
- Emergency cutoff tested under motion: **TODO**
- Supply voltage at worst-case motion: **TODO V**
- Notes/video link: **TODO**
