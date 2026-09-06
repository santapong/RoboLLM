# RoboLLM · UR5e VLA sim bed

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Specification](SDD.md) · [References](REFERENCES.md) · [Notices](NOTICES.md) · [B1 runbook](../../examples/mujoco/B1.md) · [Documentation](../../docs/README.md)

**Status: Phases 0–3 Verified** (P0–P1 on the Pi and the workstation, P2–P3 on the
workstation, 3–4 Sep 2026); Phase 4's baseline was trained and evaluated free on Kaggle (selected checkpoint 7.5k, 0.20 [0.133, 0.289] closed-loop success; charts in [`results/REPORT-2026-09-04.md`](results/REPORT-2026-09-04.md)); the P2b LIBERO calibration is done. The [specification](SDD.md) was written first, on
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
MUJOCO_GL=egl .venv-lerobot/bin/python sim/vla-bed/record.py --recipe v2 --split train   # and evaluation; v1, v2b, v3 likewise
.venv-lerobot/bin/python sim/vla-bed/record.py --recipe v2 --validate
.venv-lerobot/bin/python sim/vla-bed/cpu_bench.py
.venv-lerobot/bin/python sim/vla-bed/p2_gate.py
```

| Recipe | Expert, σ | Episodes (train / eval) | Frames | Success | Mean length | Padding @20 / @50 |
|---|---|---|---|---|---|---|
| v1 | oracle | 40 / 10 | 1,023 / 238 | 100 % | 25.6 / 23.8 | 29.9 % / 48.9 % |
| v2 | noisy, 0.5× limit (20 % clean) | 400 / 100 | 11,506 / 2,917 | 100 % | 28.8 / 29.2 | 27.0 % / 44.4 % |
| v2b | noisy, 0.25× limit (20 % clean) | 400 / 100 | 10,735 / 2,721 | 100 % | 26.8 / 27.2 | 28.3 % / 47.1 % |
| v3 | noisy, 0.5× limit (20 % clean), **label cap 0.7 × limits, camera azimuth ±20° on train** | 400 / 100 | 14,197 / 3,588 | 100 % | 35.5 / 35.9 | 19.5 % / 38.8 % |
| v4 | v3 + **camera translation ±0.20 m (x, y), ±0.05 m (z) on train, orientation kept** | 400 / 100 | 14,197 / 3,588 | 100 % | 35.5 / 35.9 | 19.5 % / 38.8 % |
| v5a | v4 with **expert noise 0.25× limit** (20 % clean; headroom 0.7, camera jitter + translation kept) | 400 / 100 | 13,217 / 3,359 | 100 % | 33.0 / 33.6 | 21.5 % / 41.5 % |
| v6 | v5a + **wrist camera** (second stream `observation.images.wrist`, 224², on `wrist_3_link`; identical seeds and labels to v5a) | 400 / 100 | 13,217 / 3,359 | 100 % | 33.0 / 33.6 | 21.5 % / 41.5 % |

Every clean label and every executed action passes S1–S4 offline (0 faults); all
video frames decode. Each frame stores the clean label as `action` and the applied
action as `action.executed`. Datasets live under `datasets/vla-bed/` (git-ignored);
the acceptance record is `results/p2/santapong/dataset_acceptance.json`.

## LIBERO calibration result (P2b, 4 Sep 2026, workstation CPU)

`lerobot/smolvla_libero` on `libero_spatial`, 10 tasks × 5 episodes, `n_action_steps=10`,
camera mapping as in `scripts/libero_calib.sh`: **41/50 = 82 %** (Wilson 95 % [0.692, 0.902]),
per task 0: 5/5, 1: 5/5, 2: 5/5, 3: 5/5, 4: 4/5, 5: 0/5, 6: 4/5, 7: 4/5, 8: 4/5, 9: 5/5; 7.0 h wall, 504 s per rollout. The published SmolVLA-0.45B Spatial
score is 90 at 10 trials per task on a GPU, inside the interval: the same evaluator path
reproduces a published number within its uncertainty. Videos and `eval_info.json` under
`results/p2b/full/` (videos git-ignored).

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

## Phase 4 pre-checks (4 Sep 2026, workstation + Pi) — zero spend

```bash
.venv-lerobot/bin/python sim/vla-bed/gpu/train.py --run baseline --mode cpu-smoke --execute   # 2 steps on CPU
.venv-lerobot/bin/python sim/vla-bed/gpu/train.py --run chunkwise --mode cpu-smoke --execute  # the label transform too
MUJOCO_GL=egl .venv-lerobot/bin/python sim/vla-bed/evaluate.py --policy oracle             # control: 100/100
MUJOCO_GL=egl .venv-lerobot/bin/python sim/vla-bed/evaluate.py --policy hold               # control: 0/100
.venv-lerobot/bin/python sim/vla-bed/gpu/preflight.py                                       # dry run
```

`lerobot/smolvla_base` accepts the bed's 14-dim state, 7-dim action and one camera
(renamed to `camera1`) without surgery; only the action expert trains (100 M of 450 M
parameters). The chunk-wise and gripper-frame label representations are applied to each
sampled window at train time (`labels.py`, exact inverses) and their statistics reach
the normaliser, so the checkpoint unnormalises into the same representation the
evaluator inverts. The frozen suite is the v2 evaluation split (100 held-out seeds):
oracle 100/100 (Wilson [0.963, 1.0]), hold 0/100, and the Pi reproduces every episode
row bit for bit. The GPU route is **Kaggle first (free)**: [`kaggle/README.md`](kaggle/README.md) is the
user's checklist, `kaggle/smoke.ipynb` measures steps/s on the free T4/P100 (no native
bfloat16 there, so a float16 cast of the frozen VLM is timed too), `kaggle/train.ipynb`
runs one fine-tune per session with a wall-clock guard and evaluates it in place. RunPod
(4090, est. $2–4 for the baseline, cap $16) is the fallback, written out in
[`GPU-GATE.md`](GPU-GATE.md).

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
| P4 | **Baseline (v2) and baseline-v3 ran on Kaggle for $0 (4–5 Sep 2026)** — v2: selected 7.5k at 0.20 [0.133, 0.289], 42 % of steps rejected at the cap; session A probes: no trainer bias, inference-side clip/gain/ensemble inside the noise; v3 (headroom 0.7 + camera jitter): 0 % over-cap, safety 0.46–0.72, selected 5k at 0.20 — success unchanged, camera_shift 0.05; all three result zips imported (per-episode files under `results/p5/`), paired v2-best vs v3-best 0.20 → 0.20 (p = 1); **v4 (+ camera translation jitter, 5–6 Sep 2026)**: nominal 5k 0.13 (not separable from v3), camera_shift **0.13** vs v3 0.05 / v2 0.04 (paired +0.08 [+0.01, +0.15], p = 0.057; vs v2 p = 0.022), camera_shift_far 0.06 — viewpoint-invariant inside the jittered range, not beyond it; **v5a (expert noise 0.25×, 6 Sep 2026)**: nominal 7.5k 0.16, every pair vs v4 inside the noise (lower-noise prediction not confirmed at n = 100) but camera_shift **0.21** (vs v3 +0.16 [+0.08, +0.24], p = 0.0004; vs v4 +0.08, p = 0.077) — the base recipe for the variants; plastic on v5a: batch 32 OOMs, batch 8 float16 diverges to NaN, float32 re-run in progress; **v6 (v5a + wrist camera, 7 Sep 2026): 0.89 [0.81, 0.94]** on the frozen suite, paired +0.79 [+0.71, +0.87] over v5a on identical seeds (79 vs 0 discordant, p = 3e-24), 2.5k already 0.80; camera_shift 0.86 / far 0.75 / lighting 0.85 / target_relocation 0.90, gain probe now hurts (0.75) — the single third-person view was the precision ceiling; gripper / chunkwise wait for quota |
| P5 | closed-loop success rate with confidence intervals; cross-embodiment row against B1 |

Full table, limits, interfaces and the honesty rules: [`SDD.md`](SDD.md).
