# RoboLLM · Changelog

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](README.md) · [Documentation](docs/README.md) · [Roadmap](ROADMAP.md) · [Architecture](docs/ARCHITECTURE.md)

Notable changes to **RoboLLM**. Format follows
[Keep a Changelog](https://keepachangelog.com/); this project is not yet
versioned — entries are grouped by date on `develop` (merged to `main`
after the touched demos verifiably run).

## 2026-09-07 — UR5e VLA sim bed: the wrist camera (recipe v6) lifts success from 0.10 to 0.89 for $0

### Results

- **Sessions I + K** (baseline trained on recipe v6 = v5a plus a wrist camera as a second image stream —
  identical seeds, labels and physics; 10k steps at 0.513 steps/s, 5.43 h, 7.1 GB VRAM; evaluated on the
  identical frozen suite in 1.66 h): nominal 2.5k 0.80 / 5k 0.87 / 7.5k 0.89 / 10k **0.89** [0.81, 0.94],
  selected 10k (5k and 7.5k within its CI). Paired on the same seeds, v5a → v6 is **+0.79** [+0.71, +0.87]
  with 79 vs 0 discordant pairs (McNemar p = 3e-24); the previous best of any recipe (0.20) → v6 is +0.69
  [+0.58, +0.79] (p = 4e-18). The 2.5k checkpoint alone (0.80) beats every single-camera number.
- The variations move with it: `camera_shift` 0.86 (v5a 0.21; not separable from v6's nominal, p = 0.63),
  lighting 0.85, target_relocation 0.90 (inside the noise of nominal), `camera_shift_far` 0.75 (v5a 0.05;
  −0.14 vs nominal, p = 0.004, still the weakest view), blank image 0.05, rejected steps 0.3 %. The
  gain-0.61 probe that helped every single-camera recipe now hurts (0.75, −0.14, p = 0.004).
- Reading: the tolerance sweep said the failures were centimetre misses; the eye-in-hand view removes the
  depth ambiguity of one third-person 224² image and turns a 0.1–0.2 policy into a 0.9 policy at the same
  demonstration count and 1.8× the training compute (Curse-of-Precision wrist ablation 2607.23108, Fourier
  features 2606.12334). v6 becomes the base recipe for the DAgger round and the gripper / chunkwise variants.
- Transcripts `results/p5/kaggle/{train-v6,eval-v6}-partial.json`; per-episode files, 18 paired comparisons
  and precision curves under `results/p5/bd883c780fba` (zip sha256 159efdbd…, checksum verified); P2 gate
  PASS with v6; Kaggle dataset `vla-bed-v6`.
- Plastic variant on v5a re-run with the VLM in float32 (session H #4): fits at 10 GB, 2.29 s/step, loss
  finite; 10k steps exceed the 6 h guard, so the run stops at ≈ 9.4k with checkpoints 2.5k/5k/7.5k.
- Corrected the P5 wording: the wire policy run matched the Kaggle in-process rate (3/20) but not the
  episodes (14/20 agreement); per-episode parity is claimed for the deterministic oracle/hold suites only.

## 2026-09-06 — UR5e VLA sim bed: v4 (camera-translation jitter) trained and evaluated for $0

### Added (next-phase plan, steps 0–5 and P5)

- `sim/vla-bed/precision.py` — success against the acceptance radius from the committed per-episode rows
  (Wilson CIs, power-law / Curse-of-Precision fits, paired at every tolerance); chart and section in the report.
  Every recipe's success climbs three- to fourfold from 0.03 to 0.06 m: the failures are centimetre misses.
- Far camera shift on the v2 and v3 final checkpoints (two 15-min probe sessions, `probe.ipynb MODE = "far"`):
  0.03 / 0.04 / 0.06 for v2 / v3 / v4, all pairs inside the noise — the far view defeats every recipe.
- Recipes **v5a** (v4 with expert noise 0.25×; recorded, gate PASS, Kaggle dataset, trained: loss 0.591 at 10k,
  quick check 0.06 on 50 episodes) and v5b (clean labels, registered only); the P2 gate iterates the required
  recipes so an unrecorded optional recipe does not fail it.
- **P5 ZeroMQ split verified**: `sim_server.py` (REQ/REP, msgpack) on the Pi + `remote_env.RemoteEnv` +
  `evaluate.py --env zmq://host:5555`; oracle/hold suites over the wire reproduce the in-process rows with
  0 mismatches; the v3 5k checkpoint on the workstation CPU drove the Pi for 20 episodes (0.15, 3/20 — the same rate as in-process on
  these seeds but on different episodes, 14/20 outcome agreement: a stochastic policy on CPU vs GPU is not expected to
  match per episode; the parity evidence is the oracle/hold diff; the wire cost 0.9 % of 94 min). SDD §6.4 carries the as-built contract, §8 P5 reads Verified.
- Wrist camera plumbing (flag-gated, recipe v6 pending): flange camera in the scene, second image stream in
  env / dataset / evaluator / server / probe; the trainer renames only the cameras a dataset has.
- `dagger.py` (one DAgger round: the policy drives fresh seeds, the capped oracle labels; LeRobot
  `aggregate_datasets` merge), recipe v7, `kaggle/dagger.ipynb`, and a merge step in `train.ipynb`.

### Results

- **Sessions D + E** (baseline retrained on recipe v4 = v3 + per-episode camera translation
  ±0.20 m x/y, ±0.05 m z with the orientation kept; 10k steps at 0.935 steps/s, 2.98 h; evaluated
  on the identical frozen suite in 1.58 h): under `camera_shift` the 10k policy scores **0.13**
  [0.077, 0.209] against 0.05 (v3) and 0.04 (v2) on the same seeds — paired v3 → v4 +0.08
  [+0.01, +0.15] (11 vs 3 discordant, McNemar p = 0.057), v2 → v4 +0.09 [+0.02, +0.16]
  (p = 0.022), progress +0.14 [+0.08, +0.21] — and that score is not separable from its own
  nominal 0.09 (p = 0.34): the policy is viewpoint-invariant inside the jittered range.
- It does not extrapolate: the new `camera_shift_far` variation (0.30, 0.20, 0) m gives 0.06
  [0.027, 0.124] with progress 0.12, as Cai et al. 2603.26757 predicted for views outside the
  training range.
- Nominal success did not improve: 2.5k 0.07 / 5k 0.13 / 7.5k 0.09 / 10k 0.09 (selected 5k,
  R11); every v3-vs-v4 nominal pair, lighting (0.07) and target_relocation (0.09) are inside the
  noise of n = 100; over-cap steps 0 %, rejected 4–9 % (workspace exits only). The gain-0.61
  probe again beats nominal on 10k (+0.11 [+0.04, +0.18], p = 0.0074).
- Transcripts `results/p5/kaggle/{train-v4,eval-v4}-partial.json`; per-episode files and the
  paired comparisons under `results/p5/145880075d6f` (zip sha256 766925ed…, checksum verified).
  Kaggle dataset `vla-bed-v4` created from split parts; quota used this week ≈ 4.6 h of 30.
- **Sessions F + G** (baseline retrained on recipe v5a = v4 with the expert noise halved to 0.25× the
  limits; 10k steps at 0.917 steps/s, 3.04 h; evaluated on the identical frozen suite in 1.72 h): nominal
  2.5k 0.11 / 5k 0.09 / 7.5k **0.16** [0.10, 0.24] / 10k 0.10 (selected 7.5k, R11); every v4 → v5a
  checkpoint pair inside the noise (+0.04, −0.04, +0.07, +0.01; McNemar p 0.19–1), best-vs-best v3 5k →
  v5a 7.5k −0.04 (p = 0.54) — the Geometric-Entropy prediction (less injected diversity → higher success
  on a fine-tuned VLA) is not confirmed at n = 100, so v5b (clean labels) is not recorded.
- v5a is the best recipe under the shifted camera: `camera_shift` **0.21** [0.14, 0.30] on 10k, above its
  own nominal (+0.11 [+0.02, +0.20], p = 0.035), above v3 (+0.16 [+0.08, +0.24], 18 vs 2 discordant,
  p = 0.0004) and v4 (+0.08 [0.00, +0.16], p = 0.077); `camera_shift_far` 0.05, lighting 0.09,
  target_relocation 0.11, gain 0.61 → 0.15, blank 0.00. v5a becomes the base recipe for the plastic,
  wrist-camera (v6) and DAgger (v7) runs.
- Transcripts `results/p5/kaggle/{train-v5a,eval-v5a}-partial.json`; per-episode files and 16 paired
  comparisons under `results/p5/8af08da6f6a2` (zip sha256 9e1a4261…, checksum verified); Kaggle dataset
  `vla-bed-v5a`.
- **Plastic variant on v5a** (VLM + vision encoder unfrozen, lr 2.5e-5): batch 32 OOMs on the 15 GB T4;
  batch 8 with the float16-cast VLM trains at 1.73 steps/s (6.2 GB) but diverges to NaN actions by 10k
  (`train-plastic-v5a-partial.json`; AdamW through float16 parameters), so `gpu/train.py --vlm-dtype
  float32` was added and the run relaunched (session H #4). Recipe **v6** (v5a + wrist camera; 13,217 /
  3,359 frames, 100 % success) recorded, uploaded as `vla-bed-v6`, training at 1.94 s/step (session I).

## 2026-09-05 — UR5e VLA sim bed: probe session, v3 retrain, both for $0 on Kaggle

### Results

- **Session A** (probes on the v2 checkpoints): the open-loop magnitude probe measured no
  trainer bias (predicted / label L∞ ratio 0.98); clip-to-limit 0.19, gain 0.61 → 0.17 and a
  linear temporal ensemble 0.18 are all inside the noise of the 7.5k nominal suite 0.24 (paired
  McNemar p 0.21–0.46); lighting 0.11 and target_relocation 0.07 are significantly worse.
  Suites now take 8 min per 100 episodes (was 65).
- **Sessions B + C** (baseline retrained on recipe v3, evaluated on the identical frozen
  suite): steps over the cap 0 %, rejected steps 4–6 % (workspace exits only), safety
  0.46–0.72 (v2: 0.00) — and success unchanged: 0.12 / 0.20 / 0.16 / 0.10 over the four
  checkpoints, selected 5k at 0.20; camera_shift 0.05 (v2 0.04). The safety cap bounded
  safety, not success; the policy's ceiling with 400 demonstrations is precision.
- The Kaggle dataset `vla-bed-v3` was created from split 9 MB parts; the notebooks join and
  checksum them (`kaggle/*.ipynb`, `RECIPE = "auto"`).
- All three Kaggle result zips imported (checksums verified) and paired on identical seeds:
  v2-best (7.5k) vs v3-best (5k) 0.20 → 0.20, 15 vs 15 discordant, McNemar p = 1; no v2-vs-v3
  pair separable at n = 100; within v3, gain 0.61 beats nominal on the 10k checkpoint
  (+0.12 [+0.04, +0.20], p = 0.0075). P4 row in the SDD reads Verified.

## 2026-09-04 (evening) — UR5e VLA sim bed: evaluator v2, paired comparisons, probes, recipe v3

### Added

- `sim/vla-bed/evaluate.py` schema v2: every episode row records the rejected-step fraction and
  the commanded / executed per-step magnitudes; policy-side post-processors `--post clip` and
  `--post ensemble` (linear temporal ensemble, Lazzati et al. 2608.02547; `--replan-every`,
  `--ensemble-horizon`), `--vlm-dtype`, `--oracle-headroom`, `--shard i/n` + `--merge`.
- `sim/vla-bed/compare.py` — paired comparison of two suites on the same seeds (discordant
  pairs, exact McNemar, paired bootstrap of the difference).
- `sim/vla-bed/gpu/magnitude_probe.py` — a checkpoint's predicted step magnitudes against the
  training labels (bias vs spread) on N training frames.
- Recipe **v3** (`dataset.py`): expert label capped at 0.7 × the S2/S3 limits (headroom) and a
  seeded per-episode camera azimuth jitter of ±20° on the train split
  (`BedEnv.set_camera_azimuth`); the evaluation split keeps v2's seeds, targets and camera.
- `kaggle/probe.ipynb` — the probe session on existing checkpoints; `eval.ipynb` and
  `train.ipynb` take a `RECIPE`; `make_bundle.sh <recipe>`.

### Changed

- `BedEnv.step(render=False)` / `observation(render)`: the evaluator renders only when the
  policy is queried (rows unchanged; oracle suite 113 s → 70 s on the workstation, a 100-episode
  SmolVLA suite 65 min → 8 min on Kaggle with two sharded workers).
- The report's and SDD's "policy steps ≈ 1.6× the labels" line replaced by the audited
  statement: the v2 labels sit exactly on the S2 cap (84 % saturate), so regression spread is
  rejected one-sidedly; the magnitude probe measured no bias (ratio 0.98).

## 2026-09-04 — UR5e VLA sim bed: Phase 4 baseline trained and evaluated on Kaggle ($0)

### Added

- `sim/vla-bed/kaggle/eval.ipynb` — full frozen-suite evaluation of a training run's
  checkpoints as its own Kaggle session (value-ordered: final checkpoint, its blank-image
  and gain-0.61 probes, the earlier checkpoints, then the variations; `MAX_HOURS` guard;
  `gpu/select_checkpoint.py`; packs `vla-bed-<run>-eval.zip`).
- `sim/vla-bed/report.py` — regenerates `results/REPORT-<date>.md` and seven hand-drawn
  SVG charts (success rates with Wilson whiskers, learning curve, per-family, safety
  rejections, Kaggle timings, OXE replay tracking, LIBERO per task) from the committed
  JSONs; `--html` writes the single-page report. Kaggle numbers transcribed from the logs
  live in `results/p5/kaggle/eval-v2-partial.json` until the packed zip is imported.
- `sim/vla-bed/evaluate.py` seeds the policy's action sampling per episode
  (`--sampling-seed`); two suites of one checkpoint had differed by 13 points.

### Results (Kaggle T4, free, 4 Sep 2026)

- Baseline `baseline` run: 10k steps at batch 32 with the frozen VLM cast to float16,
  0.8745 steps/s, 3.19 h, 4.87 GB VRAM, checkpoints at 2.5k/5k/7.5k/10k.
- Closed-loop success on the 100 held-out seeds: 2.5k 0.09, 5k 0.13, 7.5k **0.20
  [0.133, 0.289]**, 10k 0.13 [0.078, 0.210]; selected checkpoint 7.5k (by success, R11).
- Probes on the 10k checkpoint: camera blanked 0.00 (the policy uses vision); commands
  scaled by the measured 0.61 gain 0.28 [0.201, 0.375] with safety 0.86 and no per-step
  rejections (the policy's steps are about 1.6× its labels; the wrapper's holds were the
  bottleneck); camera_shift 0.04 [0.016, 0.098] with 268 self-collision steps.
- Controls: scripted oracle 1.00, do-nothing 0.00; LIBERO-Spatial calibration 0.82 with the
  published 0.90 inside the interval. Money spent: $0; Kaggle quota ≈ 12 h of 30.
- Evaluation is CPU-bound on Kaggle (≈ 1.1 h per 100 episodes); lighting and
  target_relocation were cut by the 7.5 h budget. Packed JSONs not yet imported.

## 2026-09-04 — UR5e VLA sim bed: Phase 4 prepared (priced GPU gate, zero spend so far)

### Added

- **Route K (Kaggle, free)** — `sim/vla-bed/kaggle/{README.md,make_bundle.sh,smoke.ipynb,train.ipynb}`, `gpu/kaggle_import.sh`; `gpu/train.py` gains `--dataset-root/--output-root/--steps/--batch-size/--save-freq`, a wall-clock `--max-hours` guard (`StepClock`, steps/s in `run_record.json`) and `--vlm-dtype float16` (cast of the frozen VLM for GPUs without native bfloat16); `gpu/preflight.py --dataset-root`; `runpodctl` route kept as the fallback.

- `sim/vla-bed/labels.py` — train-time action-label representations
  (gripper-frame, chunk-wise cumulative) with exact inverses; `gpu/config.json`
  (runs baseline / gripper / chunkwise / plastic), `gpu/train.py` (LeRobot's
  trainer in-process with the label wrapper and a std floor for the never-moving
  gripper channel), `gpu/preflight.py`, the session scripts
  (`transfer/smoke/full/eval_all/download/cleanup.sh`, `select_checkpoint.py`),
  `evaluate.py` (frozen suite, oracle/hold/smolvla, quadruple + Wilson +
  bootstrap, blank-image and gain probes), `GPU-GATE.md`, unit tests.

### Measured (free)

- P2b LIBERO calibration finished: `lerobot/smolvla_libero` scores 41/50 = 82 % on
  `libero_spatial` (Wilson [0.692, 0.902]) on this CPU in 7.0 h, versus the published 90 —
  inside the interval.
- Kaggle smoke (free T4): float16 VLM 0.76 steps/s at batch 32 (bfloat16 0.21), evaluator and
  renderer working on the box; baseline training launched there.
- `lerobot/smolvla_base` fine-tunes on the bed's features as-is (2-step CPU
  smokes, baseline and chunk-wise, exit 0; 3.4–3.5 GB peak RSS).
- Controls on the 100 held-out seeds: oracle 100/100, hold 0/100; the Pi
  reproduces every episode row bit for bit.
- Design rules R10–R12 from Lin 2410.18647, Seo 2604.14484, Ferchau 2607.10172;
  the sim-vs-real controller-amplification limit recorded in SDD §10.

## 2026-09-04 — UR5e VLA sim bed: Phase 3 verified (Open X-Embodiment UR5 replay)

### Added

- `sim/vla-bed/oxe/` — `fetch.py` (meta + the 4.6 MB state/action table of
  `lerobot/berkeley_autolab_ur5`, no videos; five seeded replay episodes),
  `map.py` (every mapping field verified from the data and written to
  `configs/oxe_ur5_map.yaml`), `geom.py`; `oxe_replay.py` (state and action
  replay through the bed's own controller, alignment search, side-by-side
  PNGs); `p3_gate.py`; `resources.py` (peak RSS / threads in every evidence
  file); `tests/unit/test_vla_bed_oxe.py`.

### Measured

- The dataset's **quaternion is xyzw**, the **gripper action 1 means open**, and
  the **action frame is not the state frame**: Δstate ≈ 0.61 · P · action with
  P swapping x and y and flipping z (R² 0.95, zero lag); a two-step model gives
  0.43 / 0.19 (first-order controller lag). 9.5 % of commands sit at the ±2 cm limit.
- Aligned at 90° about z with a 0.284 m lift, the sim UR5e tracks all five real
  pose trajectories to 0.7–1.2 cm mean and ≤ 0.4 cm final error with zero
  safety rejections (**G3 PASS**). Open-loop integration of the real commands
  drifts 2–9 cm (2.5–5× less than at unit gain): the teleop loop was closed by
  a human, as SimplerEnv's controller identification also implies.
- Gate thresholds were revised after this measurement and the reasons are in
  `p3_gate.py` and SDD §6.3. Replay: 486 s wall, 0.60 GB peak RSS.

## 2026-09-03 — UR5e VLA sim bed: Phase 2 verified (datasets, validator, CPU bench)

### Added

- `sim/vla-bed/dataset.py` + `record.py` — LeRobot v3 recorder for recipes v1
  (oracle 40/10), v2 (noisy σ = 0.5× limit, 400/100, one clean episode in five)
  and v2b (σ = 0.25×). Each frame stores the clean expert label as `action`, the
  applied action as `action.executed`, and the episode σ; frames pair the
  observation the expert acted on with its action. Validator re-checks S1–S4 on
  both action vectors from the recorded EE position, σ per episode, frame and
  timestamp contiguity, video decode; summary reports chunk padding and the
  distance-to-target coverage per σ.
- `sim/vla-bed/cpu_bench.py` — SmolVLA-base inference cost from the checkpoint's
  own input features; `p2_gate.py`; `requirements-record.txt`;
  `libero_calibration.md` + `scripts/libero_calib.sh` (P2b, separate
  `.venv-libero`); `tests/unit/test_vla_bed_dataset.py` (fake LeRobot dataset).

### Verified

- **G2 PASS** on the workstation: v2 11,506 + 2,917 frames, v2b 10,735 + 2,721,
  v1 1,261; every split 100 % success; 0 clean-label and 0 executed-action
  faults; all 29,140 frames decode. ≈ 1.2 s per recorded episode.

### Measured

- **SmolVLA-base on the i3-9100 CPU: 36.0 s median per 50-action chunk** (p95
  50.9 s, 4 threads, three 512×512 camera slots, 10 flow steps) → 0.03 / 0.28 /
  1.4 Hz refresh at n_action_steps 1 / 10 / 50. The checkpoint's processor
  pipelines pin `device='cuda'` and need an explicit CPU override (B1's adapter
  lacks it). Recording moved to the workstation: lerobot 0.6.0's torch 2.10 +
  torchcodec 0.10 pairing has no aarch64 wheels.

## 2026-09-03 — UR5e VLA sim bed: Phase 1 verified (expert, safety, goal families)

### Added

- `sim/vla-bed/families.py` — five goal families × 2×2 cells = 20 seeded cells,
  train/evaluation seed isolation, IK-verified list in `configs/families.json`.
- `sim/vla-bed/expert.py` — mink differential-IK controller (commanded-pose
  integrator, bias-force-compensated servos), oracle expert, and a noisy expert
  that executes noise but records the clean label (DART / Zhang et al.).
- `sim/vla-bed/safety.py` — spec families S1–S7 with severity depth and the
  (SR, Safety, SBU, VSI) quadruple (SafeVLA-Bench pattern); `stats.py` — Wilson
  and bootstrap intervals; `env.py` — `BedEnv` with B1's observation keys,
  state[14], four variations; `p1_gate.py`; 26 unit tests (skip without mink).
- SDD §14 "Design rules from the literature": nine rules with protocol and numbers
  from eight papers read through alphaXiv; `REFERENCES.md` +8 BibTeX entries.

### Verified

- **G1 passes on both machines with identical results**: oracle and noisy expert
  (100, 100, 0, 0) over 100 evaluation episodes on 20 cells; mean 25.3 / 29.7
  frames; zero clean-label faults. Workstation 254 s, Pi 536 s under `nice`.

### Measured

- Menagerie's UR5e position servos sag ≈ 0.017 rad under gravity; re-planning a
  1 cm delta from the sagged pose crept at < 1 mm per step. Fixed by bias-force
  compensation on the arm joints plus integrating deltas on the commanded pose.

## 2026-09-03 — UR5e VLA sim bed: Phase 0 verified on the Pi

### Added

- `sim/vla-bed/scene/build_scene.py` — composes the bed's scene in memory from
  the unmodified MuJoCo Menagerie files (UR5e `scene.xml`, `2f85.xml` attached
  at `attachment_site` with `MjSpec.attach`), adds the fixed `front` camera and
  the mocap red target. Nothing upstream is edited or vendored.
- `sim/vla-bed/p0_gate.py` — gate G0: renders the 224×224 front frame, checks it
  is non-black and shows the target, benchmarks physics and rendering, writes
  `results/p0/<host>/{frame.png,bench.json}` and exits non-zero on FAIL.
- `sim/vla-bed/viewer.py` — mjviser in library mode, our loop owns physics,
  binds to one address only. `scripts/pi_setup.sh` + `requirements.txt` — venv,
  pins, sparse Menagerie clone at the pinned commit.

### Verified

- **G0 passes on the Raspberry Pi 5** (aarch64, Debian 13, EGL on Mesa V3D,
  `nice -n 10`): 13,444 physics steps/s (26.9× real time), 18.1 fps at 224²,
  cold start 1.8 s. Workstation: 35,518 steps/s, 102 fps. The viewer served
  from the Pi's tailnet address renders in the workstation browser at 1.00×
  real time, 60 fps. Evidence under `sim/vla-bed/results/p0/`.

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
