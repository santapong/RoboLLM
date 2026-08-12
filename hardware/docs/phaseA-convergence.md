# RoboLLM · Phase A convergence record

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Hardware guide](../README.md) · [Arm roadmap](../../docs/physical-arm/ROADMAP.md) · [Serial protocol](serial_protocol.md)

> Historical decision record. Steps 1–4 were absorbed into arm-fw 2.0 and are
> now superseded by the fail-closed arm-fw 2.1 / `robo_arm_driver` foundation.
> Physical encoders, bench acceptance, and the LeRobot logger upgrade remain
> open; current status is maintained in `docs/physical-arm/ROADMAP.md`.

Two independent Uno+arm serial stacks exist for the same physical arm:

| | `RoboLLM/hardware/` (this repo) | research bundle `02_phaseA_code/` |
|---|---|---|
| Firmware | 124-line `arm_firmware.ino`; `PING/EN/SET/JOINTS/LED` word commands; slew-rate limiting; **no encoders** — `JOINTS` reports commanded/slewed position | 183-line `arm_firmware.ino`; compact `S/Q/H/E/X` commands; **measured encoder state** via `readEncoderDeg()` (stubbed, user fills); per-reply `millis()` timestamp |
| Driver | `arm_serial.py` (find-port, ping, per-joint set) + `arm_bridge_node.py` (ROS 2) + `sim_uno.py` (fake Uno on a pty) | `robot_driver.py` (`get_state`/`set_action`, radians+calibration on the Pi side, verified against a mock) |
| Logging | — | `camera_logger.py` (camera-synced episode folders) + `acceptance_test.py` |
| Tooling | `check_arduino.sh` 6-step health check, `pi5_setup.sh`, Makefile, C4 docs | protocol design doc (`serial_protocol.md`) with rationale |

## Decision: research protocol wins on the wire; RoboLLM stack wins around it

The research protocol is the keeper because it has the three properties
imitation learning needs and the current firmware lacks:

1. **Measured state, never commanded** — the `s` reply reads encoders. This is
   the whole point of H5 (measured-vs-commanded ablation) and honest IL data.
   The current `JOINTS` reply is the slewed *target*, which would silently
   poison Phase B datasets.
2. **Batch all-joints set + state in one round-trip** — one `S d0..d5 g` per
   tick supports 20–50 Hz teleop; per-joint `SET ch deg` cannot.
3. **On-wire timestamp + e-stop (`X`)** — needed by the camera-synced logger
   and by bench safety.

Everything *around* the wire stays RoboLLM: port autodetect, `sim_uno.py`
(port it to the new protocol — developing with no hardware is this repo's
superpower), `arm_bridge_node.py`, `check_arduino.sh`, the Makefile, and the
C4 docs pattern.

## Port order (implementation record)

1. **Done** — **Firmware**: replace command handling with `S/Q/H/E/X`; keep the existing
   slew-rate limiter and LED smoke test; leave `readEncoderDeg()` stubbed to
   return the slewed position until real encoders are wired (that stub IS the
   commanded-state baseline for H5).
2. **Done** — **`sim_uno.py`**: speak the new protocol (echo `s` lines with fake
   dynamics + `millis`).
3. **Done** — **`arm_serial.py`**: absorb `robot_driver.py`'s `get_state`/`set_action`
   API (radians + calibration Pi-side, per the protocol doc); keep the CLI.
4. **Done in the canonical ROS package** — publish state as `/joint_states`.
5. **Partial** — bring over `camera_logger.py` + `acceptance_test.py`; upgrade the
   logger's episode folders to LeRobot dataset format (Phase B needs it, and
   the hand_follow/gen3 teleop stack is the natural demo source).
6. **Done** — retire the bundle's duplicate files; `serial_protocol.md` moves here as
   the protocol reference.

## Gates

- After 1–4: `check_arduino.sh` green against `sim_uno.py`, then against the
  real Uno (needs the pending `dialout` group + bench session).
- After 5: one recorded episode replays with measured≠commanded visibly
  logged (that's H5's instrumentation working).

Research bundle source: private `arm_vla_project_bundle.zip` outside this
public repository.
(`02_phaseA_code/`). Gap→product context: `project_summary_and_gaps.md` there.
