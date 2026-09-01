# RoboLLM · Project checkpoint

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../README.md) · [Documentation](README.md) · [Roadmap](../ROADMAP.md) · [Architecture](ARCHITECTURE.md)

Status date: **2026-09-01** · Branch: **`develop`** · Software milestone:
**B1 preparation complete and rescaled; learned-policy work paused—GPU required**.

This page is the short, honest answer to “what phase is RoboLLM in now?” It
separates reproducible software evidence from work that needs the physical arm.

## Current position

| Track | Status | Evidence | Next gate |
|---|---|---|---|
| Repository organization | **Complete** | Shared code, apps, config, dependencies, operations, and tests have canonical ownership; compatibility launchers preserve existing commands | Enforce the placement contract in review |
| Shared dataset boundary (A2) | **Code-ready** | LeRobot v3 camera/state/action/task recorder; commanded state is refused by default | Real encoders and accepted demonstrations |
| Simulation dataset path (A3) | **Verified on CPU** | Scripted 6-DOF MuJoCo arm writes, reloads, and decodes a LeRobot v3 video episode | B1 policy fine-tuning and measured failure study |
| B1 benchmark preparation | **Complete on CPU** | Frozen visual target task; 50 balanced episodes/587 decoded frames; fixed suites; 100/100 oracle; safety faults rejected | Reproduced 2026-09-01 with identical numbers |
| B1 training dataset (v2) | **Complete on CPU** | Noise-injected expert (scale 1.75, 100% success); 500 balanced episodes/15,163 decoded frames; split-isolated; `chunk_size=20` sized from measured episode lengths | Train and evaluate a real SmolVLA checkpoint on GPU |
| B1 SmolVLA fine-tune/study | **Paused—GPU required** | Pinned local-only configuration and guarded transfer/train/evaluate/retrieve workflow; preflight green in dry-run; all commands dry-run verified | Provision GPU, explicitly confirm training, retrieve compact results |
| Arduino controller | **Code-ready** | Mega 2560 is the default; arm-fw 2.1 compiles with AVR core 1.8.8 and Servo 1.3.0 | Flash, wiring, cutoff, and unloaded commissioning |
| Physical Phase 0 | **Bench-gated** | Fail-closed profile, generated limits, watchdog, simulator, and worksheet exist | Record electrical/mechanical measurements and repeatable HOME evidence |
| Physical Phase 1 | **Simulation-verified; bench-gated** | ROS 2 trajectory action, validation, serial PTY, state/status, success/cancel/rejection tests | Measured URDF, MoveIt execution, and five repeatable poses |
| Physical Phases 2–5 | **Planned / reusable examples only** | Teleoperation, manipulation, and planning examples exist elsewhere in the repository | Promote one phase at a time after the preceding physical gate passes |

## Why the dataset was rescaled (2026-09-01)

The prepared benchmark was reproducible but too small to justify GPU spend. The
oracle drives straight down the slew limit, so a reach finished in ~12 frames
and the whole set was **587 frames** — against which 20,000 steps at batch 64 is
**~2,700 effective epochs**, and SmolVLA's default 50-step action chunk was
**87% padding** with exactly one inference per episode.

A learned policy failing the robustness suites under those conditions would have
measured dataset size, not the perturbation. Two additive changes fixed it
without touching the frozen task contract — a noise-injected data-collection
expert and ten times the episodes — giving **15,163 frames**, ~105 effective
epochs, and 31% padding at the chosen `chunk_size=20`. The original recipe
remains the default and still reproduces 587 frames exactly. Full numbers and
the priced go/no-go: [B1 GPU gate](../examples/mujoco/B1-GPU-GATE.md).

## What “B1 preparation complete” means

The original A3 joint-wave pipeline remains intact. B1 adds a separate frozen
visual target-reaching benchmark:

```text
inline 6-DOF arm MJCF + gripper + red target
              ↓
five seeded goal families + oracle demonstrations
              ↓
MuJoCo physics + offscreen front camera
              ↓
40 train + 10 evaluation LeRobot v3 episodes
              ↓
decode/integrity/split validation + frozen policy suites
```

The accepted CPU run generated **50 episodes** balanced at 10 per goal family,
reloaded isolated train/evaluation splits, and decoded all **587 rendered video
frames**. The oracle succeeded on **100/100 fixed seeds** and on each frozen
20-episode robustness suite. Hold/noise policies remained materially below the
oracle. NaN, out-of-range, and overspeed actions were rejected before MuJoCo;
camera dropout aborted before one 20 Hz control step.

This proves benchmark plumbing, task solvability, and evaluator refusal
behavior. It does not prove learned-policy quality, physical accuracy, or
sim-to-real transfer. See the [B1 runbook](../examples/mujoco/B1.md) and the
[GPU gate](../examples/mujoco/B1-GPU-GATE.md).

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

## Remaining B1 gate

The design and CPU preparation are finished, and the dataset is now large
enough for the result to mean something. B1 is intentionally not marked complete
until a compatible `lerobot/smolvla_base` fine-tune is trained for the checked
20,000-step configuration and its 5k/10k/20k candidates are evaluated on the
frozen suites. That work requires a GPU and explicit execution flags.
No model download, paid compute, W&B run, Hub upload, or learned-policy claim
was made during preparation.

This work must remain behind the same deterministic validator used by classical
trajectories. It does not remove or weaken the physical Phase 0/1 gates.

## Diagram contract

The maintained SVGs use the visual language in [`STYLE_GUIDE.md`](STYLE_GUIDE.md):
light canvas, dark ink, blue implemented components, amber bench-gated work,
gray dashed planned work, an accent top rule, shared typography, and accessible
title/description metadata. Diagram status labels must match this checkpoint;
simulation evidence must never be drawn as physical completion.
