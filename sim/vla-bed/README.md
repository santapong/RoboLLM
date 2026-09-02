# RoboLLM · UR5e VLA sim bed

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Specification](SDD.md) · [References](REFERENCES.md) · [Notices](NOTICES.md) · [B1 runbook](../../examples/mujoco/B1.md) · [Documentation](../../docs/README.md)

**Status: Planned.** Nothing in this directory runs yet. The
[specification](SDD.md) is written first, on purpose, so every later number has
a gate it was measured against.

## What it is

A UR5e with a Robotiq 2F-85 in **MuJoCo**, served from the always-on Raspberry
Pi and watched in a browser on the workstation through **mjviser**. The same
red-target task, recorder, GPU workflow and frozen evaluation suite as
[B1](../../examples/mujoco/B1.md), with two deliberate changes: the embodiment
(UR5e instead of the DIY arm) and the action space (end-effector deltas instead
of joint targets), so simulated demonstrations line up with the Open
X-Embodiment UR5 dataset already published in LeRobot format.

Why not the simulator we evaluated first (OmniSim): it cannot run on the Pi
without a Qt patch, its CPU physics is far slower than MuJoCo for a 6-DoF arm,
and its camera path is unproven headless. It stays as an optional second bed on
the workstation; the ARM64 fix goes upstream. Details and the rejected list are
in the specification, §2.

## Shortest path (once Phase 0 lands)

```bash
# on the Pi, inside the bed's Python 3.13 venv
python sim/vla-bed/viewer.py --scene scene/ur5e_red_target.xml --host <tailnet-ip> --port <free-port>
# on the workstation
xdg-open http://<pi-tailnet-ip>:<free-port>
```

## Phase gates at a glance

| Phase | Gate |
|---|---|
| P0 | non-black camera frame rendered on the Pi; steps/s recorded; browser on the workstation shows the scene |
| P1 | oracle expert 100 % on five goal families through mink IK |
| P2 | v1/v2 datasets valid; SmolVLA CPU seconds-per-chunk measured |
| P3 | OXE UR5 episode replayed on the sim UR5e; joint/quaternion map committed |
| P4 | priced GPU gate; fine-tune; checkpoint kept private until the weights-license item closes |
| P5 | closed-loop success rate with confidence intervals; cross-embodiment row against B1 |

Full table, limits, interfaces and the honesty rules: [`SDD.md`](SDD.md).
