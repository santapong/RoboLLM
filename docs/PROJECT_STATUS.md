# RoboLLM · Project checkpoint

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../README.md) · [Documentation](README.md) · [Roadmap](../ROADMAP.md) · [Architecture](ARCHITECTURE.md)

Status date: **2026-08-14** · Branch: **`develop`** · Software milestone:
**A3 complete**.

This page is the short, honest answer to “what phase is RoboLLM in now?” It
separates reproducible software evidence from work that needs the physical arm.

## Current position

| Track | Status | Evidence | Next gate |
|---|---|---|---|
| Shared dataset boundary (A2) | **Code-ready** | LeRobot v3 camera/state/action/task recorder; commanded state is refused by default | Real encoders and accepted demonstrations |
| Simulation dataset path (A3) | **Verified on CPU** | Scripted 6-DOF MuJoCo arm writes, reloads, and decodes a LeRobot v3 video episode | B1 policy fine-tuning and measured failure study |
| Arduino controller | **Code-ready** | Mega 2560 is the default; arm-fw 2.1 compiles with AVR core 1.8.8 and Servo 1.3.0 | Flash, wiring, cutoff, and unloaded commissioning |
| Physical Phase 0 | **Bench-gated** | Fail-closed profile, generated limits, watchdog, simulator, and worksheet exist | Record electrical/mechanical measurements and repeatable HOME evidence |
| Physical Phase 1 | **Simulation-verified; bench-gated** | ROS 2 trajectory action, validation, serial PTY, state/status, success/cancel/rejection tests | Measured URDF, MoveIt execution, and five repeatable poses |
| Physical Phases 2–5 | **Planned / reusable examples only** | Teleoperation, manipulation, and planning examples exist elsewhere in the repository | Promote one phase at a time after the preceding physical gate passes |

## What “A3 complete” means

`examples/mujoco/arm_dataset.py` provides one deliberately small pipeline:

```text
inline 6-DOF arm MJCF + gripper
              ↓
smooth deterministic joint policy
              ↓
MuJoCo physics + offscreen front camera
              ↓
LeRobot v3 video + state + action + task
              ↓
reload and decode acceptance check
```

The accepted CPU run recorded 20 frames at 20 Hz, reloaded one episode, decoded
the `3 × 240 × 320` front-camera frame, and preserved seven-element state and
action vectors. The five-second policy validation measured **0.0856 rad RMSE**.
This proves the simulation-to-dataset plumbing, not physical accuracy or policy
quality.

## Hardware boundary

The selected controller is an **Arduino Mega 2560**. It handles servo PWM,
generated raw limits, slew limiting, commissioning lock, and the communication
watchdog. Raspberry Pi 5 remains responsible for ROS 2, trajectory sampling,
calibration mapping, planning, and learning-system integration.

Before physical movement:

1. Power servos from a correctly sized external supply, never the Mega 5 V pin.
2. Tie supply, Mega, and servo grounds together and provide a physical cutoff.
3. Keep `calibrated: false`; commission one unloaded joint within 85–95°.
4. Measure direction, safe limits, home, velocity, link geometry, and current.
5. Regenerate and review firmware configuration before enabling full-arm motion.

The authoritative bench checklist remains
[`physical-arm/HARDWARE_WORKSHEET.md`](physical-arm/HARDWARE_WORKSHEET.md).

## Next software phase

The next unblocked software milestone is **B1**:

1. Generate a larger, task-specific simulation dataset.
2. Fine-tune a small policy such as SmolVLA on rented GPU capacity.
3. Evaluate success, error, latency, and unsafe outputs on fixed scenarios.
4. Deliberately introduce failures and measure detection, refusal, and abort.

This work must remain behind the same deterministic validator used by classical
trajectories. It does not remove or weaken the physical Phase 0/1 gates.

## Diagram contract

The maintained SVGs use the visual language in [`STYLE_GUIDE.md`](STYLE_GUIDE.md):
light canvas, dark ink, blue implemented components, amber bench-gated work,
gray dashed planned work, an accent top rule, shared typography, and accessible
title/description metadata. Diagram status labels must match this checkpoint;
simulation evidence must never be drawn as physical completion.
