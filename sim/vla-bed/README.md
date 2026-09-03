# RoboLLM · UR5e VLA sim bed

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Specification](SDD.md) · [References](REFERENCES.md) · [Notices](NOTICES.md) · [B1 runbook](../../examples/mujoco/B1.md) · [Documentation](../../docs/README.md)

**Status: Phases 0–3 Verified** (P0–P1 on the Pi and the workstation, P2–P3 on the
workstation, 3–4 Sep 2026); Phases 4–5 are Planned; the P2b LIBERO calibration is running. The [specification](SDD.md) was written first, on
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

## Phase 1 evidence (3 Sep 2026)

```bash
.venv/bin/python families.py --freeze     # IK-verify the 20 cells → configs/families.json
MUJOCO_GL=egl .venv/bin/python p1_gate.py   # oracle + noisy expert, 100 episodes each
```

| | Oracle | Noisy expert (σ = 0.5 × per-step limit) |
|---|---|---|
| (SR, Safety, SBU, VSI) | (100, 100, 0, 0) | (100, 100, 0, 0) |
| Wilson 95 % on success | [0.963, 1.0] | [0.963, 1.0] |
| Mean episode length (frames, 20 Hz) | 25.3 [23.6, 27.2] | 29.7 [27.5, 32.0] |
| Clean-label faults | — | 0 |
| Gate wall time | workstation 254 s · Pi 536 s | (both experts, rendered) |

Results were identical on the workstation and the Pi, episode by episode. Design
rules behind the expert, the safety spec families and the action frame are cited
in the specification's §14.

## Phase 2 evidence (3 Sep 2026, workstation)

```bash
uv pip install --python .venv-lerobot/bin/python -r sim/vla-bed/requirements-record.txt
MUJOCO_GL=egl .venv-lerobot/bin/python sim/vla-bed/record.py --recipe v2 --split train   # and evaluation; v1, v2b likewise
.venv-lerobot/bin/python sim/vla-bed/record.py --recipe v2 --validate
.venv-lerobot/bin/python sim/vla-bed/cpu_bench.py
.venv-lerobot/bin/python sim/vla-bed/p2_gate.py
```

| Recipe | Expert, σ | Episodes (train / eval) | Frames | Success | Mean length | Padding @20 / @50 |
|---|---|---|---|---|---|---|
| v1 | oracle | 40 / 10 | 1,023 / 238 | 100 % | 25.6 / 23.8 | 29.9 % / 48.9 % |
| v2 | noisy, 0.5× limit (20 % clean) | 400 / 100 | 11,506 / 2,917 | 100 % | 28.8 / 29.2 | 27.0 % / 44.4 % |
| v2b | noisy, 0.25× limit (20 % clean) | 400 / 100 | 10,735 / 2,721 | 100 % | 26.8 / 27.2 | 28.3 % / 47.1 % |

Every clean label and every executed action passes S1–S4 offline (0 faults); all
video frames decode. Each frame stores the clean label as `action` and the applied
action as `action.executed`. Datasets live under `datasets/vla-bed/` (git-ignored);
the acceptance record is `results/p2/santapong/dataset_acceptance.json`.

## Phase 3 evidence (4 Sep 2026, workstation)

```bash
.venv-lerobot/bin/python sim/vla-bed/oxe/fetch.py          # meta + 4.6 MB table, five seeded episodes
.venv-lerobot/bin/python sim/vla-bed/oxe/map.py --verify   # data-verified map → configs/oxe_ur5_map.yaml
MUJOCO_GL=egl .venv-lerobot/bin/python sim/vla-bed/oxe_replay.py
.venv-lerobot/bin/python sim/vla-bed/p3_gate.py
```

What the data said: quaternions are xyzw, gripper 1 means open, and the recorded
actions live in a frame rotated from the state frame (x↔y swapped, z flipped) with
the real controller realising only 61 % of each command per step. Aligned at 90°
about z plus a 0.28 m lift, the sim UR5e tracks all five real pose trajectories to
0.7–1.2 cm mean and ≤ 0.4 cm final error; integrating the real commands open-loop
drifts 2–9 cm, 2.5–5× less than without the measured gain. Side-by-side PNGs and
`summary.json` are under `results/p3/santapong/`. Peak RAM 0.60 GB.

**SmolVLA-base on this CPU** (`cpu_bench.json`): 36.0 s median per 50-action chunk
(p95 50.9 s) with the checkpoint's three 512×512 camera slots and 10 flow steps on
4 threads — 0.28 Hz observation refresh at `n_action_steps` = 10. Lockstep means
this only costs wall-clock in the bed. Recording ran on the workstation because
lerobot 0.6.0's torch/torchcodec pairing has no aarch64 wheels; P1 showed the Pi
produces identical episodes.

## Phase gates at a glance

| Phase | Gate |
|---|---|
| P0 | **Verified** — non-black camera frame rendered on the Pi; steps/s recorded; browser on the workstation shows the scene |
| P1 | **Verified** — oracle and noisy expert both (SR 100, Safety 100, SBU 0, VSI 0) over 100 episodes on 20 IK-verified cells, identical on both machines; `results/p1/` |
| P2 | **Verified** — v1, v2 (σ 0.5×) and v2b (σ 0.25×) valid, 100 % success, 0 clean-label faults over 29,140 frames; SmolVLA-base 36.0 s per chunk on this CPU; `results/p2/` |
| P3 | **Verified** — five real UR5 episodes replayed on the sim UR5e: state tracking ≤ 1.2 cm mean / ≤ 0.4 cm final; the action frame turned out to be a rotated teleop frame with a 0.61 controller gain, measured from the data; `configs/oxe_ur5_map.yaml`, `results/p3/` |
| P4 | priced GPU gate; fine-tune; checkpoint kept private until the weights-license item closes |
| P5 | closed-loop success rate with confidence intervals; cross-embodiment row against B1 |

Full table, limits, interfaces and the honesty rules: [`SDD.md`](SDD.md).
