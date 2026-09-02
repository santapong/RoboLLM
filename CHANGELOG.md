# RoboLLM · Changelog

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](README.md) · [Documentation](docs/README.md) · [Roadmap](ROADMAP.md) · [Architecture](docs/ARCHITECTURE.md)

Notable changes to **RoboLLM**. Format follows
[Keep a Changelog](https://keepachangelog.com/); this project is not yet
versioned — entries are grouped by date on `develop` (merged to `main`
after the touched demos verifiably run).

## 2026-09-03 — UR5e VLA sim bed: specification (experiment/ur5e-vla-bed)

### Added

- `sim/vla-bed/SDD.md` — normative specification for a second sim bed, written
  before anything runs: a Menagerie UR5e + Robotiq 2F-85 in MuJoCo served from
  the Raspberry Pi, viewed in a browser through mjviser, driven from the
  workstation over a ZeroMQ lockstep contract; B1's task, recorder, GPU gate and
  evaluator reused; end-effector-delta actions chosen so simulated demos align
  with `lerobot/berkeley_autolab_ur5` (Open X-Embodiment). Phase gates P0–P5,
  limits, results schema, cost ledger and pins. Diagram at
  `sim/vla-bed/docs/vla-bed-topology.svg`, hand-authored to `docs/STYLE_GUIDE.md`.
- `sim/vla-bed/REFERENCES.md` and `sim/vla-bed/NOTICES.md` — BibTeX for every
  upstream tool, model and dataset, and the license/attribution register
  (MuJoCo, Menagerie BSD models, mink, viser, mjviser, LeRobot, SmolVLA, the
  CC-BY-4.0 UR5 dataset, OmniSim's Apache-2.0 + trademark terms).

### Decided

- OmniSim is **not** forked. It cannot run on the Pi without a Qt patch, its
  CPU physics is far slower than MuJoCo for a 6-DoF arm, and its camera path
  is unproven headless; it remains an optional Route O on x86, and the ARM64
  build fix is contributed upstream (DCO-signed) instead.

## 2026-09-03 — Apache-2.0 license

### Added

- Repository-wide `LICENSE` (Apache License, Version 2.0), `NOTICE`, the
  `license` field in `pyproject.toml`, and a README license section. The
  repository was public without any license, which by default reserved all
  rights; the per-module `NOTICES.md` / `REFERENCES.md` attribution pattern is
  introduced at the same time so third-party models, datasets, and tools are
  cited where they are used.

## 2026-09-01 — scan3d: capture SDD, and Route E (turntable photogrammetry)

### Added

- `scan3d/SDD.md` — the normative capture and route specification, written
  before the measurements exist so a disappointing result stays reportable:
  per-route capture contract, what each route structurally *cannot* recover, the
  no-recompression transfer contract, the Route E mask interface, per-phase
  acceptance, and the results schema. Diagram at `scan3d/docs/scan3d-routes.svg`,
  hand-authored to `docs/STYLE_GUIDE.md`.
- **Route E — turntable photogrammetry** (`scan3d/masks.py` +
  `--ImageReader.mask_path` in `reconstruct_cpu.sh`). Route C solves a static
  scene with a moving camera; a fixed camera watching a rotating object is the
  inverse and reconstructs the room instead. Masking the static background makes
  the relative motion equivalent to an orbiting camera. Masks follow COLMAP's
  contract — `<image filename>.png`, same dimensions, zero-intensity ignored —
  and are verified against it on a synthetic session.
- `scan3d/segmentation.py` — `silhouette()` extracted from `visual_hull.py` so
  Routes A and E share one segmentation. It depends on cv2 and numpy only;
  `visual_hull.py` also needs trimesh and skimage, which are not installed
  wherever masks are generated. Both callers provably reference the same
  function object.

### Fixed

- `scale_mat.py make` now embeds a PNG `pHYs` chunk declaring 300 DPI. The mat's
  pixels were always exactly A4-at-300-DPI, but `cv2.imwrite` writes no `pHYs`,
  and a PNG without one is assumed to be 72 DPI — which reads the mat as
  ~1237x875 mm. A print dialog would then tile it across pages or silently
  "fit to page" at an unknown scale, so the 30 mm square the mat exists to
  provide would not have been 30 mm. The instruction printed on the mat itself,
  "PRINT AT 100%", was not followable. Regenerated; ImageMagick now reports a
  296.9 x 210.0 mm print size and the board still detects 24/24 corners.
- `reconstruct_cpu.sh` searches `$SCAN3D_PYTHON`, `../.venv`, `../.venv-lerobot`
  then `python3` for an interpreter carrying cv2, instead of assuming `python3`
  has it. It no longer does — the 3.14 rolling upgrade removed it — and the
  previous code silently skipped the ChArUco solve, losing the metric scale the
  scan3d accuracy gate is measured with.

### Changed

- `scan3d/README.md` gains the phone-transfer runbook (and the prohibition on
  chat apps, which recompress and destroy SIFT features) and the Route E
  walkthrough; `TECHNICAL.md` gains the Route E gotchas, including the open
  question that OpenMVS densification still sees masked-out background.

### Notes

Route E is **built but not yet validated on real optics** — no physical scan has
been run. Phases 1–3 of the capture plan are operator-gated on the printed
ChArUco mat, callipers, a matte object and a turntable.

## 2026-09-01 — B1: environment revived, benchmark rescaled, GPU gate priced

### Fixed

- Repointed the `.venv-lerobot` / `.venv-b1` interpreter symlinks at
  `/usr/bin/python3.13`. Both venvs were created from the unversioned
  `/usr/bin/python`, which the distribution moved to 3.14 — LeRobot 0.6.0 does
  not support 3.14, so every B1 command failed on import while the 161 installed
  packages were in fact intact.
- `NoisyExpert` holds a 0.02 rad interior margin away from the actuator bounds.
  Clamping hard to the bound let the `float32` cast round a hair outside it, and
  `ReachingEnv.step` checks bounds with no tolerance. Covered by a regression
  test that fails against the unmargined version.
- `gpu/evaluate.sh` now writes each candidate to
  `artifacts/b1-results/by-checkpoint/<step:06d>/`, which is where
  `select_checkpoint.py` looks. It previously wrote all suites flat, so the
  5k/10k/20k candidates overwrote each other and the selector found nothing —
  a failure that would only have surfaced on the rented GPU. The step is derived
  from the checkpoint path (forcing base 10; `printf %06d` reads `010000` as
  octal and would have written to `004096`).

### Added

- `NoisyExpert` and `make_expert()` in `examples/mujoco/reaching.py`, selected by
  `reaching_dataset.py --expert {oracle,noisy} --expert-noise N`. It perturbs the
  goal posture with seeded noise scaled by the remaining per-joint distance, so
  trajectories wander while far from the target and settle on arrival. Tuned to
  1.75: 100% expert success and zero truncated episodes over 100 seeds, at 2.6x
  the oracle's trajectory length. The emitted command is still clamped and
  slew-limited, so the frozen task boundary is unchanged.
- `scripts/learning/b1/dataset_stats.py` — episode-length distribution, action
  chunk padding ratio, and effective training epochs for a manifest.
- `examples/mujoco/B1-GPU-GATE.md` — the priced go/no-go for the fine-tune, with
  the measured before/after table and the full `--execute` sequence.
- Acceptance artifacts `b1_dataset_acceptance_v2.json` and `b1_dataset_stats.json`.

### Changed

- The B1 training target is the new **v2 dataset** (`datasets/b1-red-target-v2`,
  `local/robollm-red-target-v2`): 400 train + 100 evaluation episodes,
  **15,163 decoded frames** against the frozen set's 587, families balanced,
  splits seed- and target-isolated. The frozen 50-episode recipe stays the
  default and still reproduces 474 / 113 / 587 frames and `valid: true`.
- `configs/training/b1_smolvla.json` pins `chunk_size=20` / `n_action_steps=10`,
  sized from the measured ~30-frame episodes rather than left at SmolVLA's
  50-step default, which was 87% padding on v1 and allowed exactly one inference
  per episode. `chunk_size` sizes only action query tokens, never a learned
  parameter, so `smolvla_base` weights still load cleanly. `train.py` passes both
  flags and `preflight.py` asserts them.
- Regenerated `b1_cpu_acceptance.json` and `b1_dataset_acceptance.json`. All four
  acceptance booleans still hold and every measured value is identical to the
  2026-08-15 run apart from float noise below 1e-9.

## 2026-08-24 — scan3d: physical validation (S1) started, parked at capture

### Changed

- Generated the ChArUco scale mat (`scan3d/scale_mat.py make`, A4 landscape
  @300 DPI) and saved it to `assets/scan/scale_mat.png` (git-ignored;
  regenerate with the same command). Docker images `colmap/colmap` and
  `openmvs/openmvs-ubuntu` verified present on the Kali box.
- Parked awaiting the physical steps: print the mat at 100% and verify the
  30 mm square, calliper-measure a matte textured object, orbit 40–80 phone
  photos on the mat (plus a KIRI Engine baseline scan of the same object),
  then run `scan3d/reconstruct_cpu.sh` and compare the STL against callipers
  (target ~1–2%). This validation still gates scan3d `develop` → `main`.

## 2026-08-23 — Physical arm: Mega flashed, bench commissioning begins

### Changed

- Flashed arm-fw 2.1 onto the real Arduino Mega 2560 (CH340, `/dev/ttyUSB0`)
  with `hardware/check_arduino.sh`: 6/6 checks, PING and LED verified.
  Recorded the controller facts in the hardware worksheet and ticked the
  real-toolchain flash gate in the physical-arm roadmap. The 6-DOF arm is
  assembled; servo wiring and Phase 0 calibration are next.

## 2026-08-15 — B1 preparation complete; learned policy GPU-paused

### Changed

- Reorganized the full repository around `src/robollm`, `apps`, `configs`,
  `requirements`, categorized `scripts`, and split unit/ROS integration tests.
- Preserved established ROS packages, examples, hardware, CAD, and scan
  domains; retained small compatibility launchers for existing user commands.
- Updated CI, MCP bundle packaging, dependency paths, runbooks, architecture
  references, and a checked project-structure contract for the new layout.

### Added

- Added the 20 Hz visual red-target task with five balanced, seeded reachable
  goal families, seven-axis slew-limited actions, and five-frame success gate.
- Added reproducible 40-train/10-evaluation LeRobot generation, manifests, full
  video/schema/timing/bounds/split validation, and compact acceptance evidence.
- Added frozen nominal, camera, lighting, occlusion, and relocation evaluator
  suites with Oracle, hold, noise, and future SmolVLA policy adapters.
- Added fail-closed whole-chunk validation and NaN/range/overspeed/camera-loss
  fault injection, queue flushing, hold-last-safe behavior, and JSON metrics.
- Added a pinned, non-ROS SmolVLA environment/config plus dry-run-first scripts
  for preflight, transfer, smoke/full training, evaluation, selection, result
  retrieval, and recoverable cleanup.

### Verified

- The oracle passed 100/100 fixed seeds and every 20-episode robustness suite;
  hold/noise baselines remained materially below it.
- All injected invalid actions were rejected before MuJoCo, and camera loss
  aborted before one 20 Hz control step.
- Generated 50 balanced episodes (587 frames), reloaded both isolated splits,
  and decoded/validated all 587 video frames with LeRobot 0.6 on CPU.
- No model was downloaded and no GPU or paid infrastructure was used. SmolVLA
  fine-tuning and the learned-policy study remain paused until GPU access.

## 2026-08-14 — LeRobot recorder, MuJoCo A3, and Mega target

### Added

- Added a minimal LeRobot v3 recorder for one front camera, six arm joints,
  gripper state/action, task text, and camera/state synchronization lag.
- Added an isolated LeRobot dependency environment because LeRobot 0.6 needs
  NumPy 2.x while the ROS Jazzy environment must stay on NumPy 1.26.4.
- Refuse commanded-state recording by default; the override is explicitly for
  simulation pipeline checks until physical encoders are installed.
- Added a compact MuJoCo 6-DOF arm plus gripper, smooth scripted policy,
  offscreen camera, and direct LeRobot v3 simulation recorder.

### Changed

- Made Arduino Mega 2560 the default firmware/check target while preserving an
  Uno compile override.
- Pin TorchCodec 0.10 for compatibility with CPU-only PyTorch 2.10.
- Added a single project-checkpoint page for current evidence, physical gates,
  and the next software phase.
- Audited all 20 maintained SVGs, normalized their accessible titles and shared
  typography, and refreshed the MuJoCo and physical-arm status diagrams for
  A2/A3, Mega 2560, and the current bench gates.

### Fixed

- Corrected swapped full-arm/commissioning servo attach paths in arm-fw 2.1;
  the full-set parser previously referenced an undefined channel variable.

### Verified

- Hardware-free tests cover the official create/add/save/finalize flow, schema,
  float32 vectors, task labels, and BGR-to-RGB camera conversion.
- A 20-frame rendered episode writes, finalizes, reloads, and decodes through
  LeRobot 0.6 on CPU; the Mega and optional Uno targets both compile with the
  Arduino AVR 1.8.8 core and Servo 1.3.0.

## 2026-08-13 — unified documentation theme

### Changed

- Adopted the RoboLLM identity and **Build → Observe → Measure → Learn** theme
  across first-party documentation.
- Added consistent navigation, status vocabulary, and evidence boundaries to
  root, module, runbook, architecture, and research pages.
- Added a repository banner SVG and documentation style guide; vendored
  upstream documents remain untouched.
- Rebuilt the root README as a portfolio-quality landing page with architecture,
  capability maturity, quick starts, safety boundaries, and validation guidance.
- Restyled all 20 repository SVGs with shared typography, semantic colors,
  accessible titles/descriptions, consistent naming, and a RoboLLM accent rule.
- Replaced machine-specific `robot-llm-loop` paths with repository-relative or
  `/path/to/RoboLLM` examples.

## 2026-08-12 — physical arm v0.2 foundation and architecture baseline

### Added

- Installable `robo_arm_driver` package with named `JointTrajectory` input,
  exact name/limit/time/velocity validation, trajectory sampling,
  `/joint_states`, and honest `/arm/status` provenance.
- arm-fw 2.1 commissioning lock, generated per-arm limits, strict command
  rejection, single-joint commissioning path, and communication watchdog.
- Canonical physical/simulation YAML profiles, firmware config generator,
  pseudo-terminal integration tests, and Phase 0 hardware worksheet.
- Physical-arm phase matrix with explicit evidence gates for Phases 0–5.
- C4 context, container, and driver-component SVGs plus a 4+1 architectural
  view SVG. Planned capabilities are visually distinct from delivered code.

### Verified

- Native suite: 30 passed, 1 environment-dependent skip.
- Serial driver ↔ simulated Uno contract passes in calibrated and commissioning
  modes; host and firmware reject unsafe/bypassing commands.
- Ruff, Python syntax, YAML, shell syntax, package metadata, XML, and generated
  firmware-config synchronization pass.

### Pending hardware/environment evidence

- ROS 2 Jazzy/colcon build, Arduino CLI compilation/flash, electrical and
  mechanical calibration, measured URDF/MoveIt, and all later physical phases.

## 2026-07-25 — humanoid_mirror: mirror direction + preview window (both user-reported)

### Fixed
- **The mirror map negated X as well as Y, so reaching FORWARD drove the
  robot's arm BACKWARD.** A mirror reflects through the plane BETWEEN you
  and the robot, which negates only the world axis joining you; in the
  robot's own frame that leaves forward alone. Correct map is
  `(hx, hy, hz) -> (hx, -hy, hz)` — negate Y only.
  **The existing test could not have caught this**: a sideways raise
  `(0,1,0)` maps to `(0,-1,0)` whether or not x is flipped, so the one
  case I tested was the one case that does not discriminate. Found by the
  user watching the robot, not by the suite. `retarget-bench` now checks
  a forward reach and an overhead raise, and those checks were themselves
  verified to FAIL against the old formula before being accepted.
  Head retargeting now applies the same `mirror_vec()` and reads angles
  off the result instead of negating yaw separately, so the arms and the
  head cannot drift apart on this convention again.
- **The webcam preview was a black rectangle with a working toolbar.**
  `cv2.imshow` was being called from the vision thread; OpenCV HighGUI is
  main-thread only, and on the Qt backend it creates the window but never
  paints. The vision thread now only renders the annotated frame and a
  20 Hz rclpy timer (which runs on the executor = the spin thread) does
  the imshow/waitKey. Also `namedWindow` + `resizeWindow` on first draw:
  left alone the Qt backend opens at ~370x127 and squashes a 640x480
  frame into an unreadable thumbnail.

## 2026-07-25 — humanoid_mirror M4: it mirrors you

### Added
- `ffw_arm.py` — exact FK for one FFW arm plus the retargeting solve, pure
  math. **Verified against MoveIt's own `/compute_fk` to 0.0000 deg** on
  both arms, directions AND link lengths.
- `retarget.py` — body observation -> joint targets, mirror/direct modes,
  per-arm visibility gating, head yaw/pitch with sub-unity gains.
- `tools/retarget_bench.py` (`ros2-arm retarget-bench`) — FK-vs-MoveIt
  tier (`--fk`), pure-math tier (round-trip, mirror semantics, gating,
  continuity, speed, and comparison against an INDEPENDENT brute-force
  optimum), and a live tier (`--ros`).
- `ros2-arm mirror` now does live mirroring; `mirror synthetic` unchanged.

### Measured
- FK vs MoveIt `/compute_fk`: **0.0000 deg**, both arms.
- Retarget round-trip: worst **0.27 deg** over 240 reachable poses.
- Continuity: largest frame-to-frame joint step **0.019 rad**.
- Speed: **0.87 ms** per arm (2 arms = 8.7% of a 20 ms tick).
- Live: both arms commanded, **0** limit violations.

### Geometry findings that contradicted the researched design
- **Both arms are GEOMETRICALLY IDENTICAL** — same axes, same offsets;
  only the base y-offset and the joint2/joint7 LIMITS mirror. One formula
  serves both. The design guessed the right arm needed `asin(-a_y)`; that
  would have driven right-arm roll positive into a limit it can never
  satisfy, clamping to ~0 — a right arm that never lifts, looking like a
  tracking fault.
- The **+-0.041 m elbow offset** is not ignorable: shoulder->elbow tilts
  7.8 deg forward, elbow->wrist 6.9 deg back, and the joint centres
  zigzag **14.6 deg** at q=0 despite a dead-straight net arm.
- `q3` sits BELOW the shoulder gimbal, so it swings that offset around a
  7.78 deg cone and moves the upper-arm direction by up to **15.6 deg**.
  The coupling is NOT weak. Two solvers were written and discarded:
  damped gradient descent (20 deg error, 3 rad jumps) and alternating
  closed-form blocks (spurious fixed points worth exactly 15.6 deg). The
  shipped solve is a 1-D search over q3 with the shoulder exact for any
  q3, each branch swept separately.

### Traps found by testing, not by reading
- **`acos` branches are only correct modulo 2*pi.** The straight-arm elbow
  solution arrives as +6.028 rad, whose wrapped value -0.255 is the one in
  range. Range-checking before wrapping discards it and returns the far
  branch — 15-80 deg of error in a pose that still looks plausible.
- **The shoulder cannot reach every direction at a given q3** (|a_y| <=
  0.9908), but the bound is q3-DEPENDENT, so a T-pose is reachable after
  all. Bailing out instead of clamping collapsed the arm to its seed
  pose: 87 deg of error.
- **Straight-arm degeneracy**: humeral yaw is unobservable when the
  forearm is collinear with the upper arm — measured 0.79 rad steps
  between adjacent frames at 0.0000 deg error (the humerus spinning on a
  straight arm). Hold the previous yaw below 0.06 rad of bend; setting
  that threshold at 14 deg instead cost 12 deg of round-trip error.
- **A self-consistent solver cannot detect a wrong model.** The solver was
  internally exact while the first FK-vs-MoveIt harness reported 31 deg
  disagreement — the harness was comparing link3->link4 against
  link1->link4. In URDF a child link's frame IS its joint's origin.

### Notes
- **Raise your arms into frame to mirror.** Measured elbow visibility at a
  desk is 0.02-0.09 with arms at rest, so gating is PER-ARM: an unseen arm
  is held, never guessed. Whole-body gating would mean constant dropout or
  chasing invented limbs.
- Wrist joints 5-7 are parked at 0 — MediaPipe Pose carries no hand
  orientation, and inventing one would be a lie the robot acts on.
- Mirror derivation: human forward is -x_world and human LEFT is -y_world,
  so `(hx, hy, hz)` in the human torso frame is `(-hx, -hy, hz)` in the
  robot's; feed it to the OPPOSITE arm. Head mirroring flips YAW only.

## 2026-07-25 — humanoid_mirror M3: body tracking (robot parked)

### Added
- `body_track.py` — MediaPipe PoseLandmarker on the **RAW** frame ->
  torso-relative body frame. Pure-math top half (no cv2/mediapipe/ROS/
  numpy) so the geometry is unit-testable with no camera; camera classes
  import vision libs lazily.
- `mirror_node track_only:=true` (`ros2-arm track`) — vision on its own
  thread publishing `/body/tracked`, `/body/markers` (visibility-gated
  skeleton) and TF `camera_link -> human/{l,r}_{shoulder,elbow,wrist}`,
  `human/head`, plus **`human/torso` with the torso frame's full
  orientation** — the debugging aid that matters for M4. Robot parked:
  verified 0 messages on all four controller topics.
- `tools/body_accept.py` (`ros2-arm body-accept`) — three tiers:
  synthetic (26 known-answer geometry checks, no camera, CI-able),
  `--live` (camera, incl. the flip regression guard), `--ros` (topics).

### Measured — and two findings CONTRADICT the researched design
- **Axis convention, measured not read**: `body_x=-world_z`,
  `body_y=+world_x`, `body_z=-world_y` (from
  LEFT-RIGHT_SHOULDER x +0.304, SHOULDER-HIP y -0.485, NOSE-EAR z -0.112).
- **HIPS ARE INVISIBLE at a desk** — measured visibility 0.00-0.01 vs
  1.00 for shoulders. The designed shoulder-to-hip torso "up" vector does
  not exist in practice, so the camera-up fallback is the PRIMARY path.
  Live runs log `frame=camera_up`; that is not a warning state. For M4:
  arms must be RAISED INTO FRAME to mirror (elbow visibility drops to
  0.09 at rest), so gating must be per-arm, not whole-body.
- **The flip trap is real**: `|flip.LEFT-(1-raw.RIGHT)| = 0.018-0.022`
  vs `|flip.LEFT-(1-raw.LEFT)| = 0.445-0.670`, a 20-38x separation.
  POSE labels follow ANATOMY, so cv2.flip swaps them — the OPPOSITE of
  the hand API, where handedness assumes a mirrored image. Pose runs on
  the RAW frame; only the preview is flipped. Permanently guarded by
  `body-accept --live`.
- `pose_landmarker_full` on this box: **median 28-31 ms (~28-32 Hz),
  p95 47 ms, 100% detection** — inside the 70 ms U5 gate. (The design's
  24.8 ms was the i3-9100 laptop.)
- Tracking loss publishes `DELETEALL`, never a stale skeleton: verified
  173/173 `tracked=false`, 171/171 `DELETEALL`.

### Fixed
- **The node must run under `/opt/mpvenv/bin/python`.** mediapipe is not
  in the system python that ament console-scripts are shebanged to, so
  tracking died with `ModuleNotFoundError: No module named 'mediapipe'`
  *while synthetic mode kept working* — which reads as a camera fault.
  `mirror.launch.py` now sets `prefix=/opt/mpvenv/bin/python` (the same
  fix hand_follow uses), and `_make_tracker()` catches the error and
  explains it. Caught by the M3 ROS-tier check, not by inspection.

## 2026-07-25 — humanoid_mirror M2: the humanoid moves

### Added
- `mirror_node` + `mirror.launch.py` (`ros2-arm mirror synthetic`) — a
  scripted whole-body sweep drives both 7-DOF arms, the 2-DOF head and
  the lift in RViz at 50 Hz. **No camera, and MediaPipe is never
  imported** (vision imports are lazy, inside the camera branch), so the
  demo cannot be broken by a missing webcam or a drifted venv. Camera
  mode raises `NotImplementedError` with a pointer to the build plan
  rather than failing obscurely.
- `pose_source.py` — pose sources behind one interface
  (`read(t) -> {joint: angle} | None`); M4's camera source plugs in
  without touching the node. Pure math, no ROS/numpy, so tools import it.
- `joint_limits.py` — `MEASURED` limit table + a URDF parser. Limits are
  read from the **live** URDF and cross-checked against the table; a
  mismatch warns loudly, since it means the robot is not the variant the
  retargeting constants were written for.
- `tools/mirror_accept.py` (`ros2-arm mirror-accept`) — M2 acceptance.
  Measured over 10 s: **50.8 Hz on all four controller topics, 0
  joint-limit violations, 0 per-tick slew violations, 11/11 swept joints
  moved, mock hardware tracking every command.** Emits `RESULT:{json}`.
- `/mirror_enable` (`std_srvs/SetBool`) landed early from M5 — the
  control loop needed a freeze path anyway. Verified: frozen publishes
  **nothing**, resume re-seeds from `/joint_states` (max step on resume
  0.0164 rad, under the 0.0400 budget — no jump).

### Notes
- Input rate and command rate are **decoupled**: the timer runs at 50 Hz
  and interpolates toward the latest observation, so when M4 adds
  PoseLandmarker (24.8 ms, ~13 Hz) the robot still moves at 50 Hz.
- `max_joint_speed` is sized from the rate (2.0 rad/s → 0.04 rad/tick at
  50 Hz), never copied. hand_follow's 0.10 at 20 Hz is *exactly* 2.0
  rad/s despite its docstring claiming "under" the limit; copied into a
  50 Hz loop that silently becomes 5.0 rad/s.
- **Measurement trap, found the hard way:** never compute joint speed
  from subscriber *arrival* times. DDS delivers in bursts, so messages
  published 20 ms apart can arrive 6 ms apart — the first version of
  mirror_accept reported phantom 6.68 rad/s violations against a node
  that provably clamps to 0.04 rad/tick. Use the publisher's header
  stamp, and prefer asserting the timing-free per-tick invariant.

## 2026-07-25 — examples/humanoid_mirror: a humanoid in MoveIt (M0 + M1)

### Added
- **`examples/humanoid_mirror/`** — the start of webcam whole-upper-body
  teleop (left arm + right arm + head) of a humanoid. **MoveIt ships no
  humanoid**: `moveit_resources` is Panda + Fanuc + a PR2 that is
  description-only, whose SRDF is a 75-line stub with one
  `disable_collisions` pair, no head group, and `<test_depend>` status.
  We use **ROBOTIS FFW "AI Worker"** (`ffw_bg2_rev4_follower`,
  Apache-2.0) — the only apt-installable ROS 2 Jazzy robot whose MoveIt
  config already defines `arm_l` / `arm_r` / **`head`** (+ `lift`), with
  418 real `disable_collisions` pairs. 2×7-DOF arms, 2-DOF neck,
  prismatic lift; 25 meshes, 26.8 MB. It is a *semi-humanoid* — torso +
  arms + head on a lift column, **no legs**.
- `humanoid_mirror/ffw_config.py` — corrected `MoveItConfigsBuilder`
  chain. `ffw_moveit_config`'s own `moveit.launch.py` **crashes**: it
  calls `.robot_description_semantic()` but never `.robot_description()`
  and declares no dependency on `ffw_description`, so it dies
  `XML_ERROR_EMPTY_DOCUMENT` → `[FATAL] Unable to configure planning
  scene monitor` → SIGABRT. Bug is in jazzy-branch HEAD too.
- `launch/mock_bringup.launch.py` (`ros2-arm humanoid`) — RSP +
  `mock_components/GenericSystem` + `move_group` + RViz, with
  `joint_state_broadcaster` and four JTCs chained on `OnProcessExit`.
- `ffw_check.py` (`ros2-arm humanoid-check`) — M1 acceptance, no camera:
  18 checks covering descriptions, all four SRDF groups, 19 mock joints,
  four active controllers over disjoint joint sets, and `/compute_ik`
  success for **both** 7-DOF arms. All green.
- `pose_landmarker_full.task` baked into the image (sha256-pinned,
  versioned URL) for M3+, plus `ros-jazzy-pick-ik`.

### Fixed
- **The numpy law was being violated in the image.** `/opt/mpvenv` held
  numpy **2.5.1** and opencv-contrib-python **5.0.0**, shadowing the
  system 1.26.4 — and since `handfollow.launch.py` runs its node under
  `/opt/mpvenv/bin/python`, `hand_follow` and `gen3_pick_place` were
  already running on numpy 2.x. Root cause: mediapipe 0.10.35 declares
  *both* numpy and opencv-contrib-python unpinned, and pip resolves the
  latter to 5.x, which hard-requires numpy≥2. Measured symptom:
  `cv_bridge`'s numpy-1.x C extension raises `KeyError: 16`, so no node
  could publish an annotated `sensor_msgs/Image`. Both are now pinned in
  all four Dockerfile copies **and verified at build time** (version
  assert + a real `cv_bridge` roundtrip); `constraints.txt` gained
  `opencv-contrib-python<5` so the native route can't regress.

### Notes
- FFW's head axes are the **opposite** of the "pan/tilt" reading its own
  docs suggest: `head_joint1` is axis Y = **pitch** (−13°…+40°, positive
  = looking down), `head_joint2` is axis Z = **yaw** (**±20° only**).
  Head mirroring will be a nod and a glance, not a look-around.
- `arm_l_joint2` is one-sided `0…3.14` and `arm_r_joint2` mirrors it at
  `−3.14…0` — a symmetric seed pose is out of range on one side.
- `ffw-bringup` and `realsense2-description` are mandatory but **not
  declared** as dependencies; without them xacro dies `PackageNotFoundError`.
- Use `bg2_rev4`, not `sg2_rev1`: the latter's `<robot name>` mismatches
  the SRDF and it has 3 broken `${swerve_meshes_dir}` meshes.

## 2026-07-23 — tests + CI: the testing pyramid

### Added
- **First CI**: `.github/workflows/ci-fast.yml` — native no-ROS gate on
  every push/PR (<3 min, blocking): the 14 gesture-SM tests (collected
  from the vendored source via a re-export shim, never copied), a new
  hypothesis property suite over the shared `arm_ik.py` (FK agreement,
  clamp invariants, sub-mm solve_track accuracy in its operating regime,
  jump-flag consistency), a byte-identity guard for the duplicated
  `arm_ik` copies, an executable numpy==1.26.4 law check, and
  errors-only ruff over the root glue.
- `.github/workflows/build-image.yml` — builds `ros2-arm:jazzy` and
  publishes to `ghcr.io/santapong/robollm/ros2-arm:jazzy` on develop
  pushes touching a Dockerfile (registry-cache; 90 min timeout).
- `.github/workflows/ci-container.yml` + `ci/run_scenario.sh` — container
  tier: wallweld selftest (30 checks) + the 16 rclpy gen3 tests + a
  5-scenario acceptance matrix (wallweld full/abort/idle, pickplace +
  handfollow synthetic), all verified green locally. **Manual dispatch
  only for now** — it targets a self-hosted runner that is not yet
  registered (see Security below).
- `tests_ros/test_robot_bridge.py` — deadman via injected clock (wall
  sleeps not required), 20 Hz teleop tick, safe-mode forward block,
  singleton identity; `robot_bridge.py` gained an injectable time source
  (behavior-preserving).
- `hand_accept.py` now emits a machine-readable `RESULT:{json}` line
  (parity with `wallweld_accept.py`).

### Fixed
- Supply chain: `hand_landmarker.task` was fetched from a mutable
  `/latest/` URL and its sha256 recorded but never checked — now pinned
  to the versioned URL and `sha256sum -c`-verified at build time, in all
  four Dockerfile copies.
- One real lint error (unused import in `web/server.py`).

### Security
- The Fable audit caught `ci-container.yml` triggering on pull_request
  against a self-hosted runner in a public repo (arbitrary code
  execution on the runner box) and hanging forever with no runner
  registered — switched to `workflow_dispatch` until a runner strategy
  (self-hosted vs GHCR-pull on hosted runners) is decided.

## 2026-07-23 — wall_weld: gesture-triggered automation

### Added
- `examples/wall_weld/`: show the webcam an ArUco marker to place — or
  **live-track** (`wall_track:=true`) — a wall in the MoveIt planning scene;
  a held **fist** triggers an autonomous serpentine weld of the entire wall
  face (growing bead + spark markers), an **open palm** aborts mid-weld.
  Collision-checked raster (101/101 sampled states valid at the 15 mm
  standoff), reachability precheck with shrink-to-fit, `/wall_reset`,
  synthetic no-camera acceptance mode, `ros2-arm wallweld` launcher verb.
- `CHANGELOG.md` (this file).

### Fixed (found by adversarial review before release)
- TOCTOU race between marker capture / `/wall_reset` and the 20 Hz control
  tick — a wall plan can no longer be swapped under an in-flight weld.
- Degenerate-raster crash when margins exceed the (possibly shrunk) wall;
  plans now happen before the scene moves, with clean failure events.
- 5 mm torch standoff shipped 88 % collision-valid — the tool's collision
  body is thicker than its tip; the verified default is 15 mm.

## 2026-07-23 — documentation: the C4 pattern

### Added
- `docs/ARCHITECTURE.md` + `docs/architecture/`: hand-crafted C4 SVG
  diagrams (L1 context, L2 containers, L3 hand-teleop pipeline).
- Per-module `TECHNICAL.md` + pipeline diagram for every example
  (`ros2_py`, `patrol_bot`, `pybullet`, `mujoco`, `panda_arm`,
  `hand_follow`, `gen3_pick_place`) and subsystem (`hardware`, `web`,
  `scan3d`, `cad`); `docs/README.md` doc index.

### Fixed
- README's stale "8 MCP tools" → 22; several doc claims corrected against
  sources (patrol_bot's `/scan` is published but not consumed; pybullet IK
  is closed-form, not the PyBullet solver; MCP `spawn_object` is
  primitives-only).

## 2026-07-23 — examples/gen3_pick_place

### Added
- Gesture-driven pick-and-place on a **Kinova Gen3 lite** (6-DOF +
  integrated gripper, official Jazzy packages): LEFT hand guides the arm
  with palm-derived gripper orientation, **fist = grip**, **palm =
  release**; a box in the planning scene is picked and placed via
  attach/detach. One MediaPipe GestureRecognizer inference per frame,
  warm-seeded `/compute_ik` streaming at 20 Hz.
- Shared `docker/` image bakes the Kinova packages plus an SHA-pinned fix
  for the broken upstream `gen3_lite` xacro macro (0.2.6).

## 2026-07-22 — examples/hand_follow

### Added
- Webcam **LEFT-hand teleoperation** of a vendored 6-DOF arm: MediaPipe
  HandLandmarker → One-Euro smoothing → warm-start IK (~0.4 ms) → 20 Hz
  JointTrajectory streaming; live preview window, synthetic test mode,
  latency probe. Runs CPU-only in RViz; verified Docker route with
  auto-building launcher.

### Fixed
- Made the example runnable from a fresh clone (installed-share script
  resolution, `arm_ik` packaging, workspace auto-detection, first-run
  colcon build); scrubbed machine paths and personal email from the
  public tree.

## 2026-07-14 and earlier — the workbench

### Added
- Core loop: `robot_bridge.py` (single shared rclpy node),
  `ros2_mcp_server.py` (22 MCP tools: drive, navigate_to, camera, rosbag,
  TF2, MoveIt arm, Gazebo world control), FastAPI web dashboard with safe
  deadman teleop, TurtleBot3/SLAM/Nav2/MoveIt launch helpers,
  `launch_all.sh`, `.mcpb` bundle for Claude Desktop.
- Learning path `examples/` 01–10 + `patrol_bot` colcon package +
  `panda_arm` manipulation series; `cad/` FreeCAD→URDF pipeline verified
  in PyBullet; `scan3d/` webcam→mesh→URDF scanner.
- `hardware/`: the real DIY arm — Uno R3 firmware (text serial protocol,
  115200), rootless arduino-cli toolchain, `sim_uno.py` pty emulator,
  Pi 5 setup, 6-step health check.
- Project conventions: `CLAUDE.md`, branching workflow
  (`main ← develop ← experiment/*`), public-repo hygiene, numpy 1.26.4 law.
