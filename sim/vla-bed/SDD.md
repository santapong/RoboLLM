# RoboLLM · UR5e VLA sim bed specification

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Bed README](README.md) · [References](REFERENCES.md) · [Notices](NOTICES.md) · [B1 runbook](../../examples/mujoco/B1.md) · [Documentation](../../docs/README.md) · [Style guide](../../docs/STYLE_GUIDE.md)

**Status: Planned; Phases 0–3 Verified** (P0–P1 on both machines, P2–P3 on the workstation), 3–4 Sep 2026 (evidence in `results/p0/` … `results/p3/`); P2b LIBERO calibration running. This document is normative. It defines the simulation
bed, where each process runs, what every interface carries, what evidence
closes each phase, and what the bed structurally cannot show. It is written
**before** any of it exists, so that a disappointing measurement is reportable
rather than negotiable.

Scope: a UR5e in simulation, served from the Raspberry Pi and viewed and driven
from the workstation, used to record LeRobot datasets, fine-tune SmolVLA, and
evaluate it closed-loop. The physical arm, `scan3d`, and the B1 DIY-arm bench
are not re-specified; this bed reuses B1's code and changes only the embodiment
and the action space.

![UR5e VLA sim bed topology](docs/vla-bed-topology.svg)

*Source: [`docs/vla-bed-topology.svg`](docs/vla-bed-topology.svg)*

## 0. Vocabulary

| Term | Meaning in this document |
|---|---|
| **Pi** | Raspberry Pi 5, `aarch64`, Debian 13, 16 GB RAM, 4 cores, always on; already hosts the physical arm's ROS 2 side. Reached over the tailnet or LAN (`ssh pi-tailscale` / `pi-lan`). |
| **Workstation** | The x86_64 desktop (Kali, i3-9100, 31 GB RAM, Intel UHD 630, no NVIDIA GPU). Runs the browser, the policy, training-data staging, and Route O. |
| **Route M** | The primary bed: MuJoCo physics + MuJoCo Menagerie UR5e + mjviser browser viewer. |
| **Route O** | Optional second bed: an unmodified OmniSim build on the workstation. |
| **EE** | End effector: the tool frame at the UR5e wrist, or the 2F-85 fingertip frame once the gripper is attached. |
| **Frozen suite** | B1's evaluation method: seeded goal families, fixed episode count, success rate with a confidence interval, per-family breakdown. |
| **Lockstep** | The simulation advances only when the client asks for a step. Wall-clock latency never changes what the policy sees. |
| **Cell** | One of 20 seeded sub-regions of the five goal families (2×2 per family); a target is drawn inside a cell. |
| **Progress score** | 1 on success; 0.5 if the EE ever came within 2× the success threshold (6 cm); else 0. Partial credit as in ZETA (2609.02546), rule R8 of §14. |
| **Spec family** | One safety clause with a threshold and a severity anchor K. S1–S5 reject before execution; S6–S7 measure after. SafeVLA-Bench pattern (2606.00773), rule R7. |
| **Clean label / executed action** | `action` is the clean expert action that is recorded; `action.executed` is what was applied — noisy for the noisy expert. DART (1703.09327) and Zhang et al. (2507.09061), rule R4. |
| Status words | **Planned / Code-ready / Bench-gated / Verified** exactly as defined in [`../../docs/STYLE_GUIDE.md`](../../docs/STYLE_GUIDE.md). Phases 0–1 are Verified on both machines and Phases 2–3 on the workstation (3–4 Sep 2026); everything after them is Planned. |

## 1. Questions this bed answers

| # | Question | Closed by |
|---|---|---|
| **Q-A** | Can the simulator run on the Pi as an always-on server, with the viewer and the policy on the workstation, without forking anything? | Phase 0 gate G0 and Phase 1 |
| **Q-B** | Can this hardware (no NVIDIA GPU anywhere) record LeRobot datasets, fine-tune SmolVLA on a rented GPU, and evaluate it closed-loop in the same bed, using B1's existing code? | Phases 2, 4, 5 |
| **Q-C** | Does the Open X-Embodiment (OXE) UR5 dataset transfer into this bed: can a real episode be replayed kinematically on the simulated UR5e, and can it be co-trained with simulated demonstrations? | Phase 3, optional Phase 4 co-training run |
| **Q-D** | Does OmniSim add anything measurable over MuJoCo for this task (camera realism, step cost, tooling), given it runs only on x86? | Route O, optional |

## 2. Route register

| Route | Physics | Embodiment | Viewer | Server host | Status | Why it is here |
|---|---|---|---|---|---|---|
| **M** | MuJoCo 3.10.0 | Menagerie `universal_robots_ur5e` + `robotiq_2f85` | mjviser (browser) | Pi | **Phases 0–3 Verified** (3–4 Sep 2026); P4–P5 Planned | Native `aarch64` wheels run on the Pi at 26.9× real time (measured 3 Sep 2026); pure-Python viewer; same engine as B1 |
| **O** | OmniSim v8.1.x (Newton on Warp, CPU) | OmniSim UR5e PROTO | OmniSim `--stream=w3d` (browser) | Workstation only | **Planned, optional** | Vendor asked for feedback; a second simulator is a real cross-bed measurement |

### 2.1 Evaluated and rejected (kept so nobody re-evaluates them)

| Candidate | Reason |
|---|---|
| OmniSim on the Pi | Its Qt installer hard-codes an x86-64 download; Qt ships `linux_arm64` only from 6.7.0. Patchable (see §12), but Newton-on-Warp CPU physics measured 27.6 s to first step and 0.86 s+ per harness step on the workstation (1 Sep 2026); the Pi is slower still. |
| Gazebo Harmonic `-s`/`-g` split | Remote GUI is unreliable (gz-sim issue #2698, empty window on remote server); ogre2 on llvmpipe on the Pi; MoveIt-shaped stack the bed does not need. |
| robosuite / LIBERO as the primary bed | robosuite pins `mujoco<3.10`, which conflicts with `requirements/smolvla.txt` (`mujoco==3.10.0`); LIBERO is Franka-only. LIBERO stays as an optional calibration run in its own venv (§8, P2b). |
| Isaac Lab, Genesis, mjlab | GPU-only. |
| PyBullet | Unmaintained viewer/server split; no maintained UR5e + gripper model of Menagerie quality. |

## 3. Relationship to B1

B1 (`examples/mujoco/`) is the DIY seven-joint arm in MuJoCo with a red-target
reaching task, a scripted oracle and noisy expert, a LeRobot recorder, a frozen
evaluation suite, and a guarded GPU workflow. This bed **reuses all of that
code** and changes exactly two things:

| | B1 | This bed |
|---|---|---|
| Embodiment | `robollm_diy_arm` (inline MJCF, 6 hinge + 1 slide) | UR5e + Robotiq 2F-85 from Menagerie, unmodified, at a pinned commit |
| Action space | 7 joint position targets, slew-limited | **EE deltas** `[dx, dy, dz, droll, dpitch, dyaw, gripper]` in the robot base frame, clamped per step |
| Expert | straight-line joint interpolation toward the oracle posture | clipped straight-line EE delta toward the target, realised by mink differential IK |
| Action frame stored | joint space, no frame question | **base-frame deltas + full EE pose** in the file (OXE's convention); gripper-frame (ZETA, R3) and chunk-wise (Feng et al., R1) variants derived at training time — decision of 3 Sep 2026 |
| Labels | the executed (noisy) action is the label | **clean expert label** recorded as `action`; the executed action stored beside it as `action.executed` (R4) |
| Controller | joint position targets applied directly | commanded-pose integrator: deltas integrate on the controller's commanded EE pose, mink gives joint targets, bias-force-compensated position servos track them (§7.3) |
| Everything else | task contract, recorder, dataset manifests, `train.py`, evaluator, safety-wrapper pattern | **identical** |

Why EE deltas: it is the action convention of `lerobot/berkeley_autolab_ur5`
(§6.3), so simulated demonstrations and real UR5 episodes become
action-compatible. That is what makes the OXE part of Q-C real instead of
nominal. Joint positions remain in the observation, so a joint-space variant is
a recorder flag, not a redesign.

The cross-embodiment row this creates: **same task, same simulator, two arms**
(B1's DIY arm and this UR5e). That is a cleaner comparison than "same task, two
simulators", which Route O adds only if it passes its own gate. It is exactly
ZETA's "arm-only" shift category (2609.02546, Table 3), where the gripper-frame
state representation lifted progress from 38.5 to 69.8 points; §8/P5 therefore
evaluates a state-free variant as well.

## 4. Deployment topology

| Process | Host | Binds | Talks to | Notes |
|---|---|---|---|---|
| `scene/build_scene.py` overlay | Pi | — | — | loads Menagerie's `scene.xml` and `2f85.xml` unchanged with `MjSpec`, attaches the gripper at `attachment_site`, adds the fixed front camera and the mocap red target; adopts the gripper's elliptic cone / impratio 10 |
| `expert.py` + `record.py` | **Workstation** (`.venv-lerobot` + mink) | — | MuJoCo in-process | Amended 3 Sep 2026: lerobot 0.6.0's torch 2.10 / torchcodec 0.10 pairing has no aarch64 wheels (only torch 2.14 + torchcodec 0.16 resolve on the Pi), and P1 showed both machines produce bit-identical episodes, so recording moved to the workstation with no change to the data. The Pi keeps the viewer and the P5 server |
| `viewer.py` (mjviser `ViserMujocoScene`) | Pi | tailnet IP, one free TCP port (**not 8080**, taken by nginx) | browser on the workstation | Viewer only; the physics loop is ours |
| `sim_server.py` (ZeroMQ REQ/REP) | Pi | tailnet IP, one free TCP port | `policy_runner.py` | Needed only for closed-loop evaluation (P5) |
| `policy_runner.py` + `evaluate.py` | Workstation | — | `sim_server.py` | SmolVLA on CPU, or a checkpoint trained on the rented GPU |
| Browser | Workstation (or any tailnet device) | — | `viewer.py` | `http://<pi-tailnet-ip>:<port>` |
| Route O: OmniSim headless | Workstation | 1234 (viewer + extern controller), 6789 loopback (harness) | browser, controller | Unmodified build, x86 only |

**Network rules.** Neither viser nor the ZeroMQ server authenticates. Both bind
to the Pi's tailnet address only, never `0.0.0.0`, never a Funnel route. Ports
are chosen with `ss -ltnp` on the Pi at Phase 0 and recorded in
[`NOTICES.md`](NOTICES.md) next to the version pins. Whether viser honours a
non-default `host=` is itself a Phase 0 check.

**Resources (measured or estimated, environment named).**

| Quantity | Value | Basis |
|---|---|---|
| Pi RAM free | 11 GB of 16 GB | measured, Pi, 2 Sep 2026 |
| Pi disk free | 81 GB on the SD card | measured, Pi, 2 Sep 2026; the exFAT Seagate is not used for venvs (no symlinks) |
| Pi containers already running | 22 (n8n, monitoring, Excalidraw, ROS 2, databases) | measured, Pi, 2 Sep 2026; the bed runs under `nice` |
| Route M install | ~0.6 GB venv + 46 MB sparse Menagerie clone | measured, Pi, 3 Sep 2026 |
| Route O install | 4.5 GB, 1.76 GB RSS, ~1 core | measured, workstation, 1 Sep 2026 |
| MuJoCo physics, UR5e + 2F-85 (nq 14, 59 geoms), Pi | **13,444 steps/s = 26.9× real time**; 224×224 EGL render 18.1 fps; cold start 1.8 s | measured, Pi 5 (`nice -n 10`, EGL on Mesa V3D), 3 Sep 2026, `results/p0/santapong-dev/bench.json` |
| Same, workstation | 35,518 steps/s = 71× real time; render 102 fps; cold start 0.58 s | measured, i3-9100 + UHD 630 EGL, 3 Sep 2026, `results/p0/santapong/bench.json` |
| Viewer from the workstation | scene at 1.00× real time, 60 fps in the browser, ~2.8 MB page, WebGL on the workstation's Intel GPU | measured, 3 Sep 2026, `results/p0/santapong-dev/viewer-from-workstation.jpg` |
| P1 gate wall time (200 rendered episodes + 100 label-check episodes) | workstation **254 s**; Pi **536 s** under `nice -n 10` | measured, 3 Sep 2026, `results/p1/<host>/summary.json` |
| RAM, SmolVLA inference process (LIBERO calibration, CPU) | **3.09 GB resident, 3.74 GB peak**, 8.4 GB virtual, 20 threads, ≈ 2 cores busy | measured from `/proc` on the workstation, 3 Sep 2026 23:41 |
| RAM, OXE replay (MuJoCo + mink + PIL) | **0.60 GB peak**, 69 s CPU for 486 s wall, 15 threads | measured, `results/p3/santapong/summary.json` |
| Downloads on the workstation | `smolvla_base` 865 MB · `smolvla_libero` 865 MB · SmolVLM2-500M-Video-Instruct 1.9 GB · OXE UR5 table 4.6 MB (no videos) | `~/.cache/huggingface`, 3 Sep 2026 |
| Recording throughput (render + IK + AV1 encode) | ≈ 1.2 s per episode; v2 train 400 episodes in 468 s; 47 MB per 500-episode recipe on disk | measured, workstation, 3 Sep 2026, manifests under `datasets/vla-bed/` |
| SmolVLA-base CPU inference, workstation | **36.0 s median per 50-action chunk** (p95 50.9 s, min 29.4 s), 4 torch threads, three 512×512 camera slots, 10 flow-matching steps, 6-dim SO-100 state; implied refresh 0.03 Hz (n_action_steps 1), 0.28 Hz (10), 1.4 Hz (50) against the bed's 20 Hz | measured, i3-9100, 3 Sep 2026, `results/p2/santapong/cpu_bench.json`; lockstep makes this a wall-clock cost only. A one-camera fine-tune should cost roughly a third of the vision tokens (estimate, not measured) |

**Fallback, recorded in advance:** if G0 fails on the Pi after both renderer
options, every process above runs on the workstation unchanged. The Pi has no
role in that case, and Q-A is answered "no" with the failing line quoted.

## 5. Component register

| Component | File (planned) | Status | Depends on |
|---|---|---|---|
| Pi environment | `sim/vla-bed/requirements.txt`, `scripts/pi_setup.sh` | **Verified** (Pi, 3 Sep 2026) | Python 3.13 venv, pins in §11 |
| Menagerie checkout | git-ignored `sim/vla-bed/assets/mujoco_menagerie/` at the pinned commit (sparse: UR5e + 2F-85 only) | **Verified** (both machines) | `scripts/pi_setup.sh` |
| Scene overlay | `sim/vla-bed/scene/build_scene.py` — composes Menagerie's `scene.xml` + `2f85.xml` in memory with `MjSpec.attach` at `attachment_site`, adds the `front` camera and the mocap red target; `--export` writes a git-ignored compiled XML for inspection | **Verified** (both machines) | Menagerie UR5e + 2F-85 |
| Goal families | `sim/vla-bed/families.py` + IK-verified `configs/families.json` (20/20 cells, worst residual < 5 mm) | **Verified** (both machines) | mink |
| Expert + controller | `sim/vla-bed/expert.py`: `MinkController` (commanded-pose integrator), `OracleExpert`, `NoisyExpert` (clean label + executed action) | **Verified** (P1 gate, both machines) | mink 1.3.0, daqp |
| Safety wrapper | `sim/vla-bed/safety.py`: spec families S1–S7, severity depth, quadruple | **Verified** (unit tests + P1 gate) | MuJoCo contacts |
| Environment | `sim/vla-bed/env.py`: `BedEnv` (B1 observation keys, state[14], variations nominal / camera_shift / lighting / target_relocation), `run_episode` | **Verified** | scene, expert, safety |
| Statistics | `sim/vla-bed/stats.py`: Wilson interval, episode bootstrap | **Verified** (unit tests) | — |
| Phase 1 gate | `sim/vla-bed/p1_gate.py` → `results/p1/<host>/summary.json` | **Verified** | env, families |
| Recorder + validator | `sim/vla-bed/dataset.py` (recipes v1/v2/v2b, LeRobot v3 writer, manifest `robollm.vla-bed.dataset-manifest.v1`, validator with the offline S1–S4 clean-label check, summary with chunk-padding and distance-to-target coverage), `record.py` CLI | **Verified** (workstation, 3 Sep 2026) | LeRobot 0.6.0, ffmpeg libsvtav1 |
| CPU bench | `sim/vla-bed/cpu_bench.py` → `results/p2/<host>/cpu_bench.json`; batch built from the checkpoint's `input_features`; processor device overridden to CPU (the checkpoint pins `cuda`; B1's adapter lacks this override) | **Verified** | `lerobot/smolvla_base` |
| Phase 2 gate | `sim/vla-bed/p2_gate.py` → `results/p2/<host>/dataset_acceptance.json` | **Verified** | dataset.py |
| LIBERO calibration (P2b) | `sim/vla-bed/libero_calibration.md`, `scripts/libero_calib.sh`, `.venv-libero` | Planned → see §8 | `lerobot/smolvla_libero`, hf-libero |
| Viewer | `sim/vla-bed/viewer.py` | **Verified** (Pi → workstation browser, 3 Sep 2026) | mjviser |
| Sim server | `sim/vla-bed/sim_server.py` | Planned | pyzmq |
| Policy runner + evaluator | `sim/vla-bed/policy_runner.py`, `evaluate.py` (reuses `examples/mujoco/evaluate_reaching.py` suite logic) | Planned | LeRobot, ZeroMQ |
| OXE replayer | `sim/vla-bed/oxe/fetch.py` (meta + 4.6 MB parquet, seeded episode pick), `oxe/map.py` (data-verified map → `configs/oxe_ur5_map.yaml`), `oxe/geom.py`, `oxe_replay.py` (state + action modes, alignment search, side-by-side PNGs), `p3_gate.py` | **Verified** (workstation, 4 Sep 2026) | `lerobot/berkeley_autolab_ur5` @ `c4e26a6`, mink |
| GPU gate | `sim/vla-bed/GPU-GATE.md`, `configs/training/vla_bed_smolvla.json`, `sim/vla-bed/gpu/preflight.py`. B1's `train.py` is config-driven and reusable; its `preflight.py` pins B1's repo id, manifest and instruction (`scripts/learning/b1/gpu/preflight.py:21-52`) and is not | Planned | pattern of `examples/mujoco/B1-GPU-GATE.md` |
| Phase 0 gate | `sim/vla-bed/p0_gate.py` → `results/p0/<host>/{frame.png,bench.json}` | **Verified** | scene, PIL |
| Route O world + controller | `sim/vla-bed/route_o/` | Planned, optional | OmniSim unmodified |

## 6. Interfaces

### 6.1 Observation and action (LeRobot features)

Feature names are B1's, so `scripts/learning/b1/gpu/train.py` needs only a new
config file. Shapes change; SmolVLA re-initialises its state and action
projection layers for the new sizes when fine-tuning, which is expected.

| Feature | dtype / shape | Names | Note |
|---|---|---|---|
| `observation.images.front` | video, `(224, 224, 3)` | height, width, channels | fixed camera, front-left of the base, looking at the workspace; identical intrinsics in every episode |
| `observation.state` | float32 `(14,)` | `ee_x ee_y ee_z ee_qw ee_qx ee_qy ee_qz gripper q1 … q6` | EE pose in the base frame, gripper opening in [0, 1], six joint angles in radians |
| `action` | float32 `(7,)` | `dx dy dz droll dpitch dyaw gripper` | **clean expert label**: per-control-step deltas of the commanded EE pose in the base frame (rotation as a rotation vector); gripper command in [-1, 1] (−1 open, +1 close, 0 hold) |
| `action.executed` | float32 `(7,)` | same names | what was applied that step; equals `action` for the oracle, clean + truncated Gaussian noise for the noisy expert (R4) |
| `observation.noise_sigma` | float32 `(1,)` | noise_sigma | the σ (as a fraction of the per-step limit) in force for the episode; 0 for clean episodes |
| `observation.camera_lag_ms` | float32 `(1,)` | camera_lag_ms | kept from B1; zero unless a lag variation is active |

Control rate **20 Hz** (25 physics substeps of 2 ms). Episode cap **100 frames**.

Two training-time action variants are derived from `observation.state[0:7]` and
never stored: **chunk-wise deltas** (each action in a chunk re-expressed relative
to the pose at the chunk start; Feng et al. 2602.23408 found them superior to
step-wise deltas) and **gripper-frame deltas** (each delta rotated into the
current tool frame; ZETA 2609.02546). Both are P4 configuration options (§8).

### 6.2 Safety spec families (the wrapper contract)

Seven spec families, in the SafeVLA-Bench pattern (2606.00773): each has a
physical basis, a threshold, and a severity anchor K; the violation depth is
`min(1, excess / (K · threshold))`. S1–S5 **reject before execution** (the arm
holds its last command); S6–S7 **measure after** the step.

| Spec | Clause | Threshold | K | Mode | Basis |
|---|---|---|---|---|---|
| S1 | all seven values finite, shape (7,) | — | — | reject | hard |
| S2 | per-step translation `max|dx,dy,dz|` | 0.010 m (= 0.20 m/s at 20 Hz) | 2 | reject | tuned in P1 (oracle reaches every cell in ≤ 45 frames) |
| S3 | per-step rotation `max|droll,dpitch,dyaw|` | 0.05 rad | 2 | reject | tuned in P1 |
| S4 | resulting commanded EE inside the workspace box x ∈ [−0.45, 0.35], y ∈ [0.20, 0.70], z ∈ [0.05, 0.55] m | excess distance 0.05 m | 2 | reject | the union of the family regions plus margin; keeps fingertips off the floor |
| S5 | IK feasibility: mink residual after the solve | 0.02 m | 2 | reject | configuration limits are inside the QP |
| S6 | self-collision between arm/gripper geoms | any contact | — | measure | MuJoCo contacts; measurable here (contype/conaffinity allow arm–gripper pairs) |
| S7 | floor contact force on any robot geom | 50 N | 2.5 | measure | SafeVLA's 200 N / 500 N ISO/TS 15066 proxies, scaled for a tabletop reach |

Gripper commands are clipped to [−1, 1] without rejection. Reported per phase:
the quadruple **(SR, Safety, SBU, VSI)** — success rate, fraction of safe
episodes, successful-but-unsafe fraction, mean worst depth — with **Wilson
95 % intervals** on the proportions and an episode-level bootstrap on continuous
metrics (`stats.py`). A rejected action is a fault of the expert in recording
(the episode is discarded) and a policy fault in evaluation (counted, the arm
holds).

### 6.3 OXE alignment (`lerobot/berkeley_autolab_ur5`) — verified 4 Sep 2026

Every row below was read from the data (`oxe/map.py --verify`, 97,939 frames,
revision `c4e26a6`), not from documentation alone. The machine-readable form is
[`configs/oxe_ur5_map.yaml`](configs/oxe_ur5_map.yaml).

| | Real dataset (measured) | This bed |
|---|---|---|
| Rate | 5 fps | 20 Hz: one real action = four bed steps of Δ/4 |
| State | `[x, y, z, qx, qy, qz, qw, gripper_is_closed]` = `robot_state[6:14]` (Octo's transform); base frame; **quaternion xyzw** (under that reading the tool axis points down, mean (0.009, 0.002, −0.98)); x ∈ [0.20, 0.66], y ∈ [−0.28, 0.39], z ∈ [−0.21, 0.21] m | EE pose 7 (wxyz) + gripper + joints |
| Action | `[Δx, Δy, Δz, Δroll, Δpitch, Δyaw, gripper]`; 100 % within the documented ±0.02 m / ±1/15 rad; 9.5 % of translation components sit at the limit; gripper **absolute, 1 = open** (P(next closed \| 1) = 0.09, \| 0) = 0.86) | Δ of the commanded pose, base frame; gripper −1 open / +1 close |
| **Action frame ≠ state frame** | Δstate ≈ **gain · P · action** with P = [[0,1,0],[1,0,0],[0,0,−1]] (x↔y swapped, z flipped; a proper rotation) and gain ≈ **0.61** (singular values 0.626 / 0.616 / 0.583), R² 0.951 at zero lag; the same P fits the rotation deltas (gain 0.60). A two-step model Δs_t ≈ 0.43·P·a_t + 0.19·P·a_(t−1) raises R² to 0.963: the real controller realised ~43 % of a command in-step and ~19 % in the next | the bed's actions are realised ≈ exactly (commanded-pose integrator), so real actions enter the bed through P; the gain is the real controller's lag, not part of the convention |
| Bed alignment | rotation about z by **90°** and a z lift of **0.284 m** (lowest real pose 0.10 m above the floor), chosen by the lowest state-replay loss among k·90° (the other three put 2,044 steps outside the workspace box) | — |

**Replay evidence** (`results/p3/santapong/`, five episodes, two modes):

| Episode | Task | Frames | State replay L_transl mean / final (cm) | L_rot (°) | Action replay, measured gain: mean / final (cm) | Action replay, unit gain: mean (cm) |
|---|---|---|---|---|---|---|
| 95 | take the tiger out of the red bowl and | 104 | 0.81 / 0.07 | 0.36 | 2.3 / 3.0 | 30.9 |
| 819 | take the tiger out of the red bowl and | 97 | 0.72 / 0.04 | 0.41 | 3.4 / 7.9 | 8.4 |
| 243 | sweep the green cloth to the left side | 79 | 1.15 / 0.06 | 0.48 | 9.3 / 8.2 | 33.1 |
| 138 | pick up the blue cup and put it into t | 123 | 0.68 / 0.37 | 0.38 | 2.2 / 3.8 | 12.8 |
| 828 | put the ranch bottle into the pot | 113 | 0.69 / 0.03 | 0.68 | 6.4 / 11.3 | 23.0 |

Reading: **state replay** tracks every episode to a mean of 0.7–1.2 cm and a
final error ≤ 0.4 cm with no safety rejections; the mean is the bed's own
servo lag at the real arm's speed (up to 2 cm per 0.2 s), not a mismatch.
**Action replay** with the measured gain stays inside the workspace and beats
unit-gain integration by 2.5–5× on every episode, which confirms P and the gain,
but drifts 2–9 cm over 80–120 steps: the real teleoperation loop was closed by a
human, so open-loop integration of the commands cannot reproduce the path — the
same reason SimplerEnv (2405.05941 §IV-A) had to identify the controller before
replaying. Rotation error is reported as the full geodesic angle; SimplerEnv's
eq. 4 as written equals half of it.

**Consequence for P4 co-training.** The bed's labels are near-exactly realised
commanded deltas; the real labels are commands realised at ≈ 0.61 with a
one-step lag. Co-training therefore offers two action sources for the real data,
both as config options: `command` (P · action, the dataset's own labels) and
`achieved` (Δstate, the motion that actually happened). The SDD does not choose
between them; P4 measures.

Gate G3 thresholds were revised after this first measurement and the revision is
recorded in `p3_gate.py`: state mean ≤ 1.5 cm, final ≤ 0.5 cm, rotation ≤ 5°;
action mean ≤ 10 cm, inside the workspace, and ≥ 2× better than unit gain.

### 6.4 ZeroMQ lockstep contract

REQ/REP, msgpack-encoded dictionaries, one outstanding request at a time. The
server steps physics **only** inside a `step` request.

| Request | Reply |
|---|---|
| `{"cmd": "reset", "seed": int, "family": int, "variation": str}` | `{"obs": {...}, "t": 0}` |
| `{"cmd": "step", "action": [7 floats]}` | `{"obs": {...}, "success": bool, "done": bool, "fault": str or null, "t": int}` |
| `{"cmd": "info"}` | pins, scene hash, limits from §6.2, `steps_per_control` |
| `{"cmd": "close"}` | `{"ok": true}` |

`obs` carries `image` as raw `uint8` bytes of shape `(224, 224, 3)` plus
`state` as 14 floats. A request that fails validation returns
`{"error": code, "message": str}` with HTTP-style semantics: the episode is
not advanced.

### 6.5 Viewer

mjviser in library mode: `ViserMujocoScene(server, model)` updated from our own
loop after every control step. Viewers can pause, resume and reset from the
browser only when the bed is in **viewer-owned** mode (Phase 0–1); during
recording and evaluation the browser is read-only, so a stray click cannot
alter a dataset.

## 7. Task contract

Carried from B1 **verbatim** so the two embodiments are comparable
(`examples/mujoco/B1.md`, constants in `examples/mujoco/reaching.py`):

- Instruction: `touch the red target`.
- Input: front RGB image, the state vector of §6.1, and camera lag. The target
  coordinates and the expert's privileged solution never enter observations.
- Output: seven finite action values at 20 Hz inside the limits of §6.2.
- Success: EE distance to the target at or below **3 cm** (`SUCCESS_DISTANCE_M`)
  for **5 consecutive frames** (`SUCCESS_FRAMES`).
- Data: five goal families × 4 cells (§7.1), balanced by round-robin over
  cells, at most 100 frames per episode. Two recipes, as in B1: **v1** (oracle,
  40 train / 10 evaluation) preserved as the reproduction anchor, and **v2**
  (noisy expert, 400 / 100) as the training set, at the noise level of §7.2.

### 7.1 Goal families and cells

Five named regions of the reachable workspace on the +y side of the base (the
home EE is at (−0.134, 0.492, 0.332) m with the tool pointing down), each split
into a 2×2 grid along x and y → **20 cells**. This is the grid-coverage protocol
of Feng et al. (2602.23408, 6×6 grid) at the bed's scale (R8).

| Family | x (m) | y (m) | z (m) |
|---|---|---|---|
| front_high | [−0.15, 0.10] | [0.45, 0.62] | [0.30, 0.45] |
| front_low | [−0.15, 0.10] | [0.45, 0.62] | [0.08, 0.20] |
| left | [−0.38, −0.20] | [0.32, 0.55] | [0.15, 0.35] |
| right | [0.12, 0.30] | [0.32, 0.55] | [0.15, 0.35] |
| near | [−0.15, 0.10] | [0.28, 0.40] | [0.15, 0.35] |

Per episode: target = seeded uniform draw inside the cell; initial state = home
joints + seeded uniform jitter ±0.06 rad (B1's value); train and evaluation seed
blocks disjoint (`families.episode_specs`). Reachability is verified, not
assumed: `families.py --freeze` solves IK from home to every cell centre and
corner and commits the verified list to `configs/families.json` (20/20 cells,
worst residual < 5 mm, 3 Sep 2026); the gate refuses unverified cells. The
`target_relocation` variation redraws the target inside the same cell with an
independent seed (B1's +77 777 offset).

### 7.2 Noise recipe

Following DART (1703.09327, Alg. 1) and Zhang et al. (2507.09061, Practice 2):
the noisy expert **executes** `clean + N(0, σ²I)` on the six motion dimensions
(gripper untouched) and **records the clean action as the label**; the executed
action is stored beside it. Noise is truncated at the per-step limits so every
executed action passes S1–S5. σ is a fraction of the per-step limit: **0.5×**
(5 mm, 0.025 rad) is the default measured in P1; **0.25×** is the second level
recorded in P2. 20 % of training episodes are clean (a clean/noisy mixture,
Zhang et al. §4). Measured on both machines, 3 Sep 2026: the noisy expert
lengthens episodes by 1.17× (29.7 vs 25.3 frames) while keeping 100 % success —
less spread than B1's 2.6×, because the truncation bounds the disturbance. No
paper measures a noise recipe for a scripted reaching expert; the level is
therefore a P2/P4 measurement, not a literature value.

### 7.3 Controller

Deltas integrate on the controller's **commanded** EE pose (the teleoperation
convention OXE data was collected with); mink differential IK (`FrameTask` on
`2f85/pinch`, position cost 1.0, orientation cost 0.2, posture cost 1e-3,
configuration and π rad/s velocity limits, `daqp`) turns the commanded pose into
joint position targets; the Menagerie position servos track them with
**bias-force compensation** on the six arm joints, as the real UR5e controller
does internally. Both choices were forced by measurement on 3 Sep 2026: without
compensation the servos sag ≈ 0.017 rad under gravity and a delta re-planned
from the sagged pose creeps at < 1 mm per step; re-planning from the actual pose
lost six of twenty cells to the 100-frame cap. Observations always report the
**actual** pose.

**Optional grasp variant (after P5):** `lift the red cube` using the 2F-85.
Not part of the gates below; it exists so the gripper is not decorative.

## 8. Phases and gates

Every phase names its machine. A phase without a number is not done.

| Phase | What | Gate / evidence | Time box | On fail |
|---|---|---|---|---|
| **P0** | Pi venv with the §11 pins; Menagerie at the pinned commit; overlay scene with camera; `MUJOCO_GL=egl` render; viewer on a free port bound to the tailnet IP; fill pins and ports into `NOTICES.md` | **G0 — Verified 3 Sep 2026.** Pi: EGL worked first try; `results/p0/santapong-dev/` (frame mean 75.8, std 32.5, 33 red pixels; 13,444 steps/s; 18.1 fps). Workstation: `results/p0/santapong/`. Browser on the workstation showed the running scene from the Pi (`viewer-from-workstation.jpg`). OSMesa fallback was not needed | ½ session (used) | — |
| **P1** | Port the task: red target, oracle and noisy expert through mink in EE space, safety wrapper, limits tuned | **G1 — Verified 3 Sep 2026.** 20 cells × 5 seeds = 100 evaluation episodes per expert, identical results on both machines. Oracle **(SR 100, Safety 100, SBU 0, VSI 0)**, Wilson 95 % ≥ 0.963, mean 25.3 frames [23.6, 27.2]; noisy σ = 0.5× limit **(100, 100, 0, 0)**, 29.7 frames [27.5, 32.0], 0 clean-label faults, length ratio 1.17. `results/p1/santapong/` (workstation, 254 s) and `results/p1/santapong-dev/` (Pi, 536 s). 26 unit tests green | 1 session (used) | — |
| **P2** | Record v1 (40/10), v2 (400/100, σ = 0.5×) and v2b (400/100, σ = 0.25×) with one clean episode in five; `--validate` re-checks every clean label and executed action against S1–S4; SmolVLA-base CPU seconds-per-chunk | **G2 — Verified 3 Sep 2026** (workstation, `results/p2/santapong/dataset_acceptance.json`). v1 1,261 frames; **v2 11,506 + 2,917 frames**, mean 28.8 / 29.2 frames (max 61), chunk padding 27.0 % @20 / 44.4 % @50; **v2b 10,735 + 2,721 frames**, mean 26.8 / 27.2 (max 55), padding 28.3 % / 47.1 %. Every split 100 % success, **0 clean-label faults, 0 executed-action faults**, all 29,140 video frames decode. Noisy episodes are only 1.09× longer than clean ones at σ = 0.5× and equal at 0.25×; mean distance-to-target over recorded frames 0.151 m (clean) vs 0.137 m (noisy). Recording ≈ 1.2 s per episode incl. AV1 encoding (v2 train 468 s). CPU bench: 36.0 s per chunk (§4) | 1–2 sessions (used ½) | — |
| **P2b** | LIBERO-Spatial calibration of the evaluation plumbing: `lerobot/smolvla_libero` (Apache-2.0) in `.venv-libero`, CPU, `n_action_steps` = 10, against the paper's Spatial 90 % at 10 trials per task (2506.01844, Table 2; its protocol refreshes every step on GPU). Runbook `libero_calibration.md` | Within the paper's Wilson band (≈ [82.6, 94.5] at n = 100), or the deviation explained; per-rollout wall time recorded from a one-episode probe first | overnight CPU; 5 episodes per task if a rollout exceeds 10 min | Report the shortfall; the bed's own P5 numbers stand on their own protocol |
| **P3** | OXE replay: five episodes of `lerobot/berkeley_autolab_ur5` (2 tiger, 1 cloth, 1 cup, 1 bottle), map verified from the data, state and action modes, alignment search, side-by-side PNGs | **G3 — Verified 4 Sep 2026** (workstation, `results/p3/santapong/`). Map verified (xyzw quaternion, gripper 1 = open, action frame = P · state frame with gain 0.61, R² 0.95). Alignment 90° about z + 0.284 m lift. State replay mean 0.7–1.2 cm, final ≤ 0.4 cm, rotation ≤ 0.7°, zero rejections; action replay (measured gain) 2.2–9.3 cm mean, 2.5–5× better than unit gain, zero rejections. 486 s wall, 0.60 GB peak RSS. Five PNGs. `configs/oxe_ur5_map.yaml` committed with the chosen alignment | 1 session (used) | — |
| **P4** | `GPU-GATE.md` priced go/no-go; `smoke_train.sh` first; 20 k-step fine-tune on v2. Training-time variants (config flags, one run each only if budget allows): chunk-wise deltas (R1), gripper-frame deltas (R3), `n_action_steps ≤ chunk_size/2` (R1); optional co-training with OXE-UR5 at mixing ratios {natural, 0.1, 0.3} with a `sim:` / `real:` prefix in the task string as the domain tag (R9) | Checkpoint, training curve, cost line; **checkpoint not published** until the weights-license item in §12 closes | user's go, est. $8–16 | — |
| **P5** | `sim_server.py` on the Pi; policy on the workstation; frozen suite (the P1 schedule, seeds shared by every policy); cross-embodiment check against the B1 DIY-arm checkpoint; **blank-image probe** (success must collapse, else the policy runs on state alone, R6) and a **state-free** variant (R3) | Quadruple (SR, Safety, SBU, VSI) with Wilson CIs, progress score, per-family breakdown; report under `sim/vla-bed/results/` in the §9 schema | 1 session | — |
| **O0–O2** (optional) | OmniSim unmodified on the workstation: headless camera gate, UR5e world, same schema; issues A–E filed; ARM64 fix offered upstream | G0-style frame test; step cost from OmniSim's `/capabilities` | ≤ 2 sessions total | Drop Route O; issues are still filed |

P0 is the anchor. If it fails on both machines the bed stops and this document
records why, rather than proceeding on a different simulator by drift.

## 9. Results schema

One compact summary per phase, committed; raw datasets, frames and checkpoints
stay git-ignored.

```json
{
  "schema": "robollm.vla-bed.phase-summary.v1",
  "phase": "P5",
  "route": "M",
  "verdict": "PASS",
  "host": {"hostname": "santapong-dev", "machine": "aarch64", "python": "3.13.5", "MUJOCO_GL": "egl"},
  "task": "touch the red target",
  "embodiment": "ur5e+2f85",
  "schedule": {"seed": 70000, "split": "evaluation", "episodes": 100, "cells": 20},
  "limits": {"xyz_step_m": 0.01, "rpy_step_rad": 0.05},
  "noise": {"fraction_of_limit": 0.5, "sigma_xyz_m": 0.005, "sigma_rpy_rad": 0.025},
  "<policy>": {
    "n": 100, "success_rate": 0.0, "safety": 0.0, "sbu": 0.0, "vsi": 0.0,
    "ci95_wilson_success": [0.0, 0.0], "ci95_wilson_safety": [0.0, 0.0],
    "progress_mean": 0.0, "episode_len_mean": 0.0, "episode_len_ci95_bootstrap": [0.0, 0.0],
    "per_family_success": {"front_high": 0.0, "front_low": 0.0, "left": 0.0, "right": 0.0, "near": 0.0},
    "faults": {"rejected": {}, "measured": {}}
  },
  "episodes": {"<policy>": [{"seed": 0, "family": "", "cell": 0, "variation": "nominal", "success": false, "progress": 0.0, "frames": 0, "final_error_m": 0.0, "min_error_m": 0.0, "safe": true, "rejections": {}, "measured": {}, "worst_depth": 0.0}]}
}
```

## 10. Limits (what this bed cannot show)

- **Nothing here is physical evidence.** A UR5e in MuJoCo says nothing about the
  DIY arm on the bench, and the two must never be reported in one number.
- **OXE replay is kinematic.** The real episode's scene, objects and cameras do
  not exist in the bed. Replay checks the embodiment bridge; it is not a task.
- **CPU inference is not real time.** Lockstep hides latency from the policy;
  wall-clock numbers are reported separately and are not a claim about
  deployment speed.
- **No MoveIt, no ROS 2 in the loop.** The bed is a Python process on the Pi;
  the physical arm's ROS 2 path is a separate, later integration.
- **The Pi is slower than the workstation.** Hosting there is a design choice
  (always on, sim/real symmetry), not a performance claim.
- **Route O is x86-only** and its camera path is unverified headless.

## 11. Cost ledger and pins

| Item | Cost | Basis |
|---|---|---|
| Rented GPU for P4 (one run) | est. **$8–16** | B1-GPU-GATE estimate for a 20 k-step SmolVLA fine-tune on a 4090-class host; a co-training run roughly doubles it |
| Hardware | none | — |
| Pi disk | < 2 GB | estimated |
| OXE download | 15.5 GB if taken whole; **selected episodes only** for P3 | dataset card |

Pins (from `pip index` on the workstation, 3 Sep 2026; frozen at P0 into
`requirements.txt` and `NOTICES.md`):

| Package | Pin | Why this one |
|---|---|---|
| `mujoco` | 3.10.0 | matches `requirements/smolvla.txt`; `aarch64` cp313 wheel resolves on the Pi (measured 2 Sep 2026) |
| `mjviser` | 0.0.14 | pure Python; v0.0.x, so pinned exactly |
| `viser` | 1.1.0 | pulled by mjviser |
| `mink` | 1.3.0 | latest at time of writing; pulls `qpsolvers` 4.13.0 + `daqp` 0.9.1 (the QP solver used) |
| `pyzmq` | 27.2.0 | `aarch64` wheels available |
| `lerobot[smolvla]` | 0.6.0 | repo pin; Python 3.13 venv (3.14 is unsupported); pulls `transformers` 5.5.4, `accelerate` 1.14.0, `num2words` 0.5.14 (`requirements-record.txt`) |
| `.venv-libero` (P2b only) | `lerobot[smolvla,libero]==0.6.0`, torch 2.10.0+cpu, torchcodec 0.10.0, hf-libero 0.1.4, robosuite 1.4.0, **mujoco 3.8.1** | separate venv because hf-libero pins mujoco < 3.10; needs `CMAKE_POLICY_VERSION_MINIMUM=3.5` on CMake 4 and a pre-written `~/.libero/config.yaml` |
| MuJoCo Menagerie | `e4049d0` (2026-09-01) | confirmed at P0 (3 Sep 2026) |
| `msgpack` | 1.2.2 | ZeroMQ payloads (P5) |
| `numpy`, `pillow` | ≥2.0, ≥10 (resolved 2.5.2, 12.3.0 on 3 Sep 2026) | floating on purpose; frozen only if a phase needs it |

## 12. Provenance and attribution

Principle: **use upstream unmodified at a pinned version, keep adaptations in
our own files, cite everything, redistribute nothing that is not needed.** The
full table with copyright lines is [`NOTICES.md`](NOTICES.md); the BibTeX is
[`REFERENCES.md`](REFERENCES.md). The rules that shape the design:

- **RoboLLM is Apache-2.0** (root `LICENSE`, `NOTICE`), added 3 Sep 2026.
- **OmniSim** (Apache-2.0; `NOTICE`, `TRADEMARKS.md`, `CITATION.cff`; DCO, no
  CLA). Route M copies no OmniSim code. Route O runs an unmodified build.
  Permitted wording without permission: "compatible with OmniSim", "runs on
  OmniSim". The ARM64 build fix is contributed **upstream** as an issue and a
  `Signed-off-by` PR (their CONTRIBUTING asks for a measurement, not a
  screenshot; a build-only patch's measurement is the Pi build log and
  `doctor` READY). If any OmniSim file is ever adapted: keep its header, add a
  modification notice, ship their LICENSE and NOTICE, use none of their names
  or the orb mark on our artefacts, and never copy their NOTICE carve-outs
  (Code2000 fonts, the OmniLink typeface, brand artwork).
- **MuJoCo Menagerie models** are BSD (UR5e: BSD-3, ROS-Industrial Consortium;
  2F-85: BSD-2, ROS-Industrial). They are **not vendored**: the overlay
  `<include>`s the upstream files from a git-ignored checkout, so nothing is
  redistributed. If vendoring ever becomes necessary, the model's own `LICENSE`
  is copied beside it, following `examples/talos_mirror/ros2_ws/src/VENDORED.md`.
- **`lerobot/berkeley_autolab_ur5`** (revision `c4e26a6`, table only, 4.6 MB;
  no videos downloaded) is CC-BY-4.0. The P3 side-by-side PNGs plot its
  end-effector paths and are derivatives; they carry the attribution in
  `NOTICES.md`. Any checkpoint co-trained on it is a derivative too.
- **SmolVLA weights** (`lerobot/smolvla_base`, revision `c83c3163`, downloaded
  3 Sep 2026 for the CPU measurement) carry **no license tag** on the model
  card, while the base VLM (SmolVLM2-500M) and the LeRobot code are Apache-2.0.
  Open item: ask on the model card or a LeRobot issue. Until it closes,
  fine-tuned checkpoints are used for research and **not published**. The P2b
  checkpoint `lerobot/smolvla_libero` is Apache-2.0 and is only evaluated.
- **mink, viser, mjviser, LeRobot, MuJoCo** are Apache-2.0 pip dependencies;
  each is cited.
- **Design rules** (§14) cite the eight papers read on 3 Sep 2026 through alphaXiv:
  `laskey2017dart`, `zhang2025actionchunking`, `feng2026actionspace`, `yan2026zeta`,
  `zhao2026proprio`, `fan2026safevlabench`, `lei2026cotraining`, `qiu2025mpcsafegil`
  (BibTeX in [`REFERENCES.md`](REFERENCES.md)).

## 13. Honesty rules

Carried from [`../../docs/STYLE_GUIDE.md`](../../docs/STYLE_GUIDE.md) because
this document exists to be held to:

- Never promote a simulated result to physical evidence. Name the machine
  behind every measured number.
- A route or embodiment performing worse is a **result**, not a failure to
  hide. An unmeasured route is the only unacceptable outcome.
- Status labels here must match `../../docs/PROJECT_STATUS.md` once phases
  start landing.

## 14. Design rules from the literature

Read on 3 Sep 2026 through alphaXiv; every number below is the paper's, with its
protocol, so that the bed's choices can be re-examined when the field moves.

| # | Rule adopted | Source · protocol · number | Applied in |
|---|---|---|---|
| R1 | Delta actions beat absolute; chunk-wise deltas beat step-wise; delta control wants a shorter execution horizon | 2602.23408 (Feng et al.): 13,000+ real rollouts, 500+ models, 6×6 grid initial conditions, 3 trials × 10 rollouts; Table 1 overall avg abs-EE 63.4 → delta-EE 78.4 (ACT), 71.9 → 82.9 (DP); delta peaks at k=30 vs abs at k=60 at 30 Hz | §6.1 (step-wise stored, chunk-wise derived), §8/P4 `n_action_steps ≤ chunk_size/2` |
| R2 | Task-space actions transfer across embodiments better than joint space; joint space wins only with abundant single-robot data | same paper, §4.3.2, Fig. 6 (cross-embodiment and π0 transfer regimes) | §3 |
| R3 | Gripper-frame ("EEF-delta") actions and states transfer better than base-frame ones | 2609.02546 (ZETA): 6,300 sim rollouts per model, 140 real; Table 3 avg 60.3 → 64.6 (actions) and 73.4 → 75.7 (with EEF-delta state); Table 4 real 56.0 → 61.6 → 89.9; arm-only shift 38.5 → 69.8 | §3, §6.1 (derived variant), §8/P4–P5 |
| R4 | Execute the noisy action, record the clean label; mix clean and noisy trajectories; isotropic noise; the level matters | 1703.09327 (DART) Alg. 1, Fig. 5 (Tr Σ = 0.5 good, 0.005 and 5.0 bad), real grasping 49 → 79 % (α=3) but 72 % (α=6); 2507.09061 Practice 2 and §4. Counter-evidence: 2508.03129 found random noise useless on navigation | §7.2 |
| R5 | Executed chunk length stabilises behaviour cloning; requisite lengths are small; relies on open-loop stability, typically end-effector control | 2507.09061 Thm 1, Fig. 5 (robomimic tool_hang chunk sweep) | §8/P4 |
| R6 | Proprioceptive state helps modestly; joint vs EE state is secondary; a state shortcut can let a policy ignore vision | 2608.03052: π0.5 scaffold, RoboCasa365, 45 atomic tasks × 50 rollouts; state prompt +3.1 pts (only CI-supported gain); K=8 history to the action head +10.8 on composite tasks | §6.1, §8/P5 blank-image probe |
| R7 | Report safety separately from success: (SR, Safety, SBU, VSI), Wilson 95 % CIs, fixed seeds across models | 2606.00773: LIBERO n=200/cell, RoboCasa n=900; ≥94 % SR policies still 13–15 % unsafe; 36–56 % of RoboCasa successes violate a clause | §6.2, §9, `stats.py` |
| R8 | Standardised spatial coverage of initial conditions, progress scores, enough rollouts | 2602.23408 grid protocol; 2609.02546 progress ∈ {0, 0.5, 1}, 100 rollouts per cell | §7.1, §0 |
| R9 | Sim-and-real co-training works in a balanced mixing band and needs domain discernibility | 2604.13645: balanced w ∈ (0.016, 0.3) best (200 trials × 3 checkpoints in sim, 30 real); representation alignment ≈ 50 % of variance, mixing ratio ≈ 20 %; a domain label adds ≈ 20 % | §8/P4 |

Empty result recorded: no paper measures a noise-injection recipe for a scripted
*reaching* expert; the nearest evidence is locomotion, robomimic and navigation.
The noise level is therefore measured in P2/P4, not taken from the literature.
