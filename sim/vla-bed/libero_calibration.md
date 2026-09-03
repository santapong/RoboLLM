# RoboLLM · P2b — LIBERO calibration of the SmolVLA evaluation plumbing

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Bed README](README.md) · [Specification](SDD.md) · [Notices](NOTICES.md) · [Documentation](../../docs/README.md)

**Status: Planned** until `results/p2b/` holds an evaluation file. This page is the
runbook and the decision record for an optional step of the specification
(§8, P2b): before trusting the bed's own closed-loop numbers, run a policy the
community has already measured through the same LeRobot evaluation path and
see whether this machine reproduces the published range.

## Why LIBERO, why this checkpoint

- SmolVLA's paper evaluates on LIBERO (2506.01844, §4.1): 40 tasks, 10 trials per
  task, binary success. Table 2 reports the 0.45 B model fine-tuned on LIBERO at
  **Spatial 90 / Object 96 / Goal 92 / Long 71 / avg 87.3**. The simulation
  protocol predicts a new chunk **after every executed action** (§4.3) with 10
  flow-matching steps, on GPU.
- `lerobot/smolvla_libero` is the LeRobot team's own LIBERO fine-tune (Apache-2.0,
  0.5 B parameters). It is the only checkpoint whose published number and
  evaluation code both exist independently of this repository.
- LIBERO-Spatial is used because it has the shortest horizon (280 steps cap in
  LeRobot's env, longest demo 193 steps), which matters on CPU.

## What is different here, stated up front

| | Paper | This run |
|---|---|---|
| Device | GPU | 4-core i3-9100, CPU (`cpu_bench.json`: SmolVLA-base ≈ 36 s per 50-action chunk with three 512×512 camera slots) |
| Observation refresh | every executed action (`n_action_steps` = 1) | `n_action_steps` = 10; Table 13 of the paper shows 10 ≥ 1 (82.8 vs 80.3 avg) in its from-scratch ablation, so this should not lower the number |
| Trials | 10 per task, 40 tasks | 10 per task on the 10 Spatial tasks (100 rollouts); fallback 5 per task if a rollout exceeds the time box |
| Cameras | LIBERO's two (agent view + wrist) | same |

Expected reading: the paper's 90 % on Spatial at n = 100 has a Wilson 95 %
interval of roughly [82.6, 94.5]. A result inside that band means the
evaluation plumbing on this machine matches published practice; a result far
below it is a finding about this setup (CPU numerics, env version, action
refresh), not about the bed.

## Environment

`.venv-libero` (Python 3.13, uv) with CPU torch 2.10.0, torchcodec 0.10.0,
`lerobot[smolvla,libero]==0.6.0` → `hf-libero` 0.1.4, `robosuite` 1.4.0,
**mujoco 3.8.1** (hf-libero pins `<3.10`; this is why the bed's venv is not used).
Two traps met on 3 Sep 2026:

- `egl-probe` builds a C helper with an old CMake minimum; on CMake 4.x install
  with `CMAKE_POLICY_VERSION_MINIMUM=3.5`.
- `import libero` prompts on stdin for a dataset path when `~/.libero/config.yaml`
  is missing (and a piped answer did not persist it). Write the file yourself
  with the package's default paths (see `scripts/libero_calib.sh` history in the
  changelog); after that the import is silent and LIBERO assets download from the
  Hub on first use.

## Run

```bash
bash sim/vla-bed/scripts/libero_calib.sh probe        # 1 episode, task 0: read wall time and RSS
bash sim/vla-bed/scripts/libero_calib.sh full 10      # overnight; or `full 5` if the probe says > 10 min per rollout
```

The probe's `/usr/bin/time -v` line gives the per-rollout wall time; multiply
by 100 (or 50) before starting `full`. The full run is detached (`nohup`) and
writes `results/p2b/full/eval_info.json`.

## Reading the result

Report per task and aggregate success with a Wilson 95 % interval
(`stats.wilson_interval`), the mean episode length, the wall time per rollout,
and the two protocol differences above. Record it in SDD §8 (P2b row) and in the
README's evidence table. Nothing derived from the checkpoint is kept beyond the
evaluation JSON and the log.
