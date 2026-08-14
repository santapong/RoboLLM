# RoboLLM · Learning and research roadmap

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](README.md) · [Documentation](docs/README.md) · [Physical-arm roadmap](docs/physical-arm/ROADMAP.md) · [Architecture](docs/ARCHITECTURE.md)

**Objective of this repo: LEARNING.** RoboLLM is the workbench where the
five-layer robot stack is learned by building — every subsystem exists to
teach its layer, publicly and reproducibly. Product-shaped and OSS-tool
outputs do NOT live here: they graduate to the **RLM** repo
(github.com/santapong/RLM — working name) once they stop being lessons and
start being deliverables.

The execution status for the DIY physical arm is tracked separately in
[`docs/physical-arm/ROADMAP.md`](docs/physical-arm/ROADMAP.md). As of
2026-08-13, only the Phase 0 software foundation and v0.2 driver core are
implemented; later physical-arm phases are not marked complete by the presence
of related simulation examples.

```
RoboLLM (learn it here)  ──graduates──▶  RLM (ship it there)
```

**Graduation rule:** an artifact moves to RLM when a stranger would want to
use it without wanting to learn from it. Candidates already identified (the
research doc's gap→products): diversity-first data-collection kit ·
per-axis evaluation harness · measured-spec actuator bench + datasheets ·
the reference benchmark-arm report. NOTE: RLM currently holds the research
bundle's original Phase A code — superseded by this repo's arm-fw 2.1
convergence (`hardware/docs/phaseA-convergence.md`); RLM should consume
RoboLLM's stack, not fork it.

---

## The learning map (five layers, from the research framing)

| Layer | What it is | Where this repo teaches it |
|---|---|---|
| L5 cognitive brain | LLM/VLM planning, 1–10 Hz | MCP server + web dashboard (done); Phase D planner (future) |
| L4 robotics large model | VLA action expert, 10–50 Hz | sim track: SmolVLA fine-tune + break-and-measure |
| L3 whole-body control | WBC/MPC, locomotion | humanoid/talos mirrors (done); G1 capstone (Nav2 + policy @50 Hz) |
| L2 joint control | PID, estimation, 1 kHz | arm-fw 2.1 measured-state/safety stack (code done, bench pending); EKF (triggered) |
| L1 hardware | actuators, sensors, structure | DIY arm bench; scan3d → printed parts; actuator test bench (future) |

Cross-cutting: **perception** (scan3d done; SLAM triggered) and
**evaluation rigor** — the thesis edge both research docs converge on.

---

## Two tracks, one shared artifact

- **Sim track — unblocked today** (laptop + rented GPU hours):
  MuJoCo scene → LeRobot dataset in sim → SmolVLA fine-tune (~450M,
  3090/A100 hours) → break it deliberately and MEASURE the failure.
  Steps 1–3 are table stakes; step 4 is the contribution. Thesis rows:
  failure detection · mid-chunk abort (the G1 capstone's 50/200 Hz split is
  the same pattern) · safety/refusal.
- **Real-arm track — bench-gated**: bench afternoon → encoders → collect
  demos via hand-teleop → Phase B imitation (ACT/DP) → Phase C VLA.
- Shared: the **LeRobot-format logger** (A2) — both tracks record into it.

## Tiers (re-tier when a Tier S item lands)

### Tier S — next, ~1 hour combined, hardware in hand
| # | Item | Proves |
|---|------|--------|
| S1 | scan3d physical validation (ChArUco mat @100%, calliper object, STL vs callipers ~1–2%; KIRI baseline on the same object) | scan→print pipeline on real optics; gates scan3d develop→main |
| S2 | Arm bench check (`dialout`, Phase 0 worksheet, `hardware/check_arduino.sh` — flashes arm-fw 2.1) | measured-state and fail-closed safety stack on real hardware |

### Tier A — high value, cheap, de-risked
| # | Item | Track | Who |
|---|------|-------|-----|
| A1 | Real encoders into `readEncoderDeg()` (turns on H5 — the novel research axis) | real-arm | bench |
| A2 | `camera_logger` → LeRobot dataset format (**code ready; real recording waits for encoders**) | both | Claude, code-only |
| A3 | MuJoCo arm scene + scripted policy + sim episode recording (extends `examples/mujoco/`) | sim | Claude, code-only |

### Tier B — the main quests (pick per track)
| # | Item | Learning payoff |
|---|------|-----------------|
| B1 | Sim ladder steps 2–4: sim dataset → SmolVLA fine-tune (rented GPU) → measured failure study | L4, evaluation rigor; the thesis seed |
| B2 | Phase B demo collection via hand-teleop on scanned objects (30–50 episodes) | L2+L4 data; diversity-kit lessons → graduates to RLM |
| B3 | G1 locomotion capstone (private research brief: MuJoCo G1 + ros2_control + Nav2, phase-gated) | L3; brings EKF + Nav2 + BTs in on their triggers |

### Tier C — triggered, not scheduled
RTAB-Map SLAM (first live-map need / G1 Phase 3) · EKF odom fusion (G1
Nav2) · grasp planning on scanned meshes (arm bench + one validated scan)
· VGGT rescue path + Phase C fine-tune of a 3B-class model (cloud GPU) ·
actuator test bench (weekend spin-off; lessons → RLM datasheets).

### Tier D — parked with triggers (CLAUDE.md stack-gap backlog is authoritative)
Sim2real randomization · behavior trees · force/tactile · voice · carrier
PCB · transfer-prediction (Gap 5) · task-memory (Gap 6).

---

## Milestone arc (the story this repo is trying to tell)

1. **Foundations** (done): ROS 2 examples 01–10, MCP tools, dashboard,
   teleop family (hand → gen3 → wall_weld → humanoid → talos), CAD, scan3d.
2. **Honest hardware** (in progress): arm-fw 2.1 measured state + safety → bench →
   encoders → H5 instrumented.
3. **First learned policy** (sim track): a SmolVLA you fine-tuned, and a
   failure study you measured — the MSc-application artifact.
4. **Imitation on real hardware** (real-arm track): Phase B demos → ACT/DP
   baselines with CIs.
5. **The full stack, one robot** (G1 capstone): language → plan → walk —
   L5 through L1 exercised end to end.
6. **Graduations**: eval harness, diversity kit, actuator datasheets,
   benchmark-arm report → RLM as products/OSS.

Repository context: `hardware/docs/phaseA-convergence.md` and scan3d
README/TECHNICAL. The gap→product summary and VLA field map remain private
research inputs outside this public repository.
