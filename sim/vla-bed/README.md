# RoboLLM · UR5e VLA sim bed

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Specification](SDD.md) · [References](REFERENCES.md) · [Notices](NOTICES.md) · [B1 runbook](../../examples/mujoco/B1.md) · [Documentation](../../docs/README.md)

**Status: Phase 0 Verified** on the Pi and the workstation (3 Sep 2026);
Phases 1–5 are Planned. The [specification](SDD.md) was written first, on
purpose, so every number below had a gate it was measured against.

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

## Shortest path

```bash
# once per machine (Python 3.12/3.13): venv, pins, sparse Menagerie clone at the pinned commit
bash scripts/pi_setup.sh                      # or VLA_BED_PYTHON=/usr/bin/python3.13 bash scripts/pi_setup.sh

# the gate: renders results/p0/<host>/frame.png and writes bench.json, exits 1 on FAIL
MUJOCO_GL=egl .venv/bin/python p0_gate.py      # MUJOCO_GL=osmesa is the fallback (apt install libosmesa6)

# the viewer, on the Pi, bound to its tailnet address only (never 0.0.0.0)
nice -n 10 .venv/bin/python viewer.py --host 100.74.8.82 --port 8090
# on the workstation
xdg-open http://100.74.8.82:8090
```

The scene is composed in memory by `scene/build_scene.py` from the unmodified
Menagerie files (UR5e `scene.xml` + `2f85.xml` attached at `attachment_site`);
`--export` writes a git-ignored compiled XML if you want to read it.

## Phase 0 evidence (3 Sep 2026)

| | Pi 5 (aarch64, EGL, `nice -n 10`) | Workstation (i3-9100, UHD 630, EGL) |
|---|---|---|
| Frame 224×224 | mean 75.8, std 32.5, 33 red-target pixels | mean 76.3, std 32.6, 33 |
| Physics (nq 14, 59 geoms) | **13,444 steps/s = 26.9× real time** | 35,518 steps/s = 71× |
| Render fps at 224² | 18.1 | 102 |
| Cold start (import → home pose) | 1.8 s | 0.58 s |
| Viewer | served from the Pi, seen in the workstation browser at 1.00× real time, 60 fps | loopback smoke test |

Files: `results/p0/santapong-dev/` (Pi) and `results/p0/santapong/` (workstation).

## Phase gates at a glance

| Phase | Gate |
|---|---|
| P0 | **Verified** — non-black camera frame rendered on the Pi; steps/s recorded; browser on the workstation shows the scene |
| P1 | oracle expert 100 % on five goal families through mink IK |
| P2 | v1/v2 datasets valid; SmolVLA CPU seconds-per-chunk measured |
| P3 | OXE UR5 episode replayed on the sim UR5e; joint/quaternion map committed |
| P4 | priced GPU gate; fine-tune; checkpoint kept private until the weights-license item closes |
| P5 | closed-loop success rate with confidence intervals; cross-embodiment row against B1 |

Full table, limits, interfaces and the honesty rules: [`SDD.md`](SDD.md).
