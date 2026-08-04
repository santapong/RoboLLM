# Roadmap

The spine: **bench afternoon → encoders → LeRobot logger → collect demos.**
Everything else hangs off it. Tiered 4 Aug 2026 after the Phase A convergence;
gap triggers live in [CLAUDE.md](CLAUDE.md) (stack-gap backlog). Re-tier when
a Tier S item lands.

## Tier S — next (unblocks everything, ~1 hour combined, hardware in hand)

| # | Item | Proves | Status |
|---|------|--------|--------|
| S1 | **scan3d physical validation** — print ChArUco mat at 100%, scan a calliper-measurable object, compare STL dims (target ~1–2%); scan the same object with KIRI Engine as baseline | the whole scan→print pipeline on real optics; gates scan3d develop→main | ⏳ user bench |
| S2 | **Arm bench check** — `sudo usermod -aG dialout santapong`, plug Uno in, `hardware/check_arduino.sh` (flashes arm-fw 2.0) | the measured-state stack on real hardware | ⏳ user bench |

## Tier A — high value, cheap, already de-risked (after S)

| # | Item | Why | Status |
|---|------|-----|--------|
| A1 | Wire real encoders into `readEncoderDeg()` (after S2) | turns on H5 (measured-vs-commanded — the novel research axis); makes demos honest | ⏳ bench |
| A2 | Upgrade `camera_logger` episodes to **LeRobot dataset format** | the format Phases B–D and community tooling consume; code-only | ⏳ code (Claude can do) |

## Tier B — the compounding build (pick ONE main quest)

| # | Item | Why |
|---|------|-----|
| B1 | **Phase B demo collection** via the hand-teleop stack — 30–50 episodes on scanned objects | the imitation-learning main quest; exercises arm + logger + diversity-kit idea (research Gap 1) |
| B2 | **Grasp planning on scanned meshes** → MoveIt pick | the autonomy-first alternative; bridges scan3d assets to pick-and-place, no GPU needed |

## Tier C — triggered, not scheduled

RTAB-Map SLAM (first live-map need / G1 capstone Phase 3) · EKF odom fusion
(arrives with G1 Nav2) · VGGT rescue path + Phase C VLA fine-tune (cloud GPU)
· actuator test bench (weekend spin-off once the bench works; research Gap 3).

## Tier D — explicitly parked (triggers in CLAUDE.md; do not open early)

Sim-to-real randomization · behavior trees · force/tactile · voice interface
· carrier PCB (only after an Orbiter-style rig exists) · transfer-prediction
(research Gap 5 — data-centre economics) · task-memory middleware (Gap 6).

## Context documents

- `hardware/docs/phaseA-convergence.md` — how the two arm stacks merged
- `scan3d/` README + TECHNICAL — the scan→print/URDF pipeline
- `~/Downloads/research/project_summary_and_gaps.md` — the research framing
  (five hypotheses, gap→product map) this roadmap serialises
