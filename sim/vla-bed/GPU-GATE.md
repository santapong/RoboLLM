# GPU gate — Phase 4 fine-tune of SmolVLA on the UR5e bed

**Status: PREPARED — every free check is green; awaiting the owner's go** (4 Sep 2026 01:20). This is the priced
go/no-go for the first spend of the bed. Everything free has been run; the
rented session is a script; the smoke step measures the real cost before the
priced command starts. Nothing below is executed without `--execute`.

## What is green (measured on the workstation, 4 Sep 2026)

| Check | Result |
|---|---|
| Base checkpoint accepts the bed's features (state 14, action 7, one camera renamed to `camera1`) | **yes** — `gpu/train.py --run baseline --mode cpu-smoke`: 2 steps on CPU, exit 0, checkpoint written; 100 M trainable of 450 M parameters (action expert only); 362 s wall, 3.44 GB peak RSS |
| Chunk-wise label transform through LeRobot's batching | **yes** — `--run chunkwise --mode cpu-smoke`: 2 steps, exit 0; the checkpoint's saved unnormaliser carries the transformed statistics (action std ≈ 0.07 m / 0.12 rad vs 0.007 / 0.021 per-step; gripper channel floored to 1); 561 s wall, 3.48 GB peak RSS |
| The checkpoint loads back through `evaluate.py` and returns a (20, 7) chunk | **yes** — `evaluate.py --policy smolvla --run baseline --checkpoint <cpu-smoke ckpt> --episodes 1`: the checkpoint loads with its saved processors, returns a finite (20, 7) chunk, and the episode runs 10 chunks / 100 frames end to end; the 2-step model's actions were rejected by S2/S3 86 times, which the evaluator counted (0 % success, as it should be). 89 s per chunk on the contended CPU, 3.74 GB peak RSS |
| Frozen-suite controls (100 held-out seeds) | oracle **100/100** (Wilson [0.963, 1.0]), safety 1.0, 25.8 frames mean, zero S1–S7 faults, 113 s; hold **0/100** ([0, 0.037]), 100 frames each, 417 s; both 0.41 GB peak RSS |
| Same controls on the Pi (bit-identity, as P0/P1) | **yes** — oracle and hold on the Pi (`results/p5/santapong-dev/`): 0 mismatches over 2 × 100 episodes × 8 fields; Pi wall 163 s / 653 s, 0.40 GB peak |
| Unit tests (`labels`, `evaluate`, all bed suites) | 96 passed + 3 skipped in the fast-CI venv (no torch/mujoco); 46 bed tests in `.venv-lerobot` |
| `gpu/preflight.py` dry run | valid except "no CUDA" on the workstation; `smolvla_base` revision `c83c3163b8ca9b7e67c509fffd9121e66cb96205` |

## The runs (`gpu/config.json`)

| Run | Data | Steps | Trains | Labels | Why |
|---|---|---|---|---|---|
| `baseline` | v2 train (400 ep, 11,506 frames) | 20 k, checkpoints every 5 k | action expert only (SmolVLA's recipe, 2506.01844 §4.3) | stored per-step base-frame deltas | the reference |
| `gripper` | v2 | 10 k | expert only | deltas in the EE frame at chunk start (R3) | action-representation ablation |
| `chunkwise` | v2 | 10 k | expert only | cumulative from chunk start (R1) | action-representation ablation |
| `plastic` | v2 | 10 k | expert + VLM + vision encoder, lr 2.5e-5 | stored | R12: does the frozen VLM cost success? (lr is a named confound) |

`chunk_size=20` (27 % padding on v2; 50 would be 44 %), `n_action_steps=10`,
batch 64, seed 20260904. Checkpoints are selected by closed-loop success on the
held-out suite (`gpu/select_checkpoint.py`), never by loss (R11).

## Cost (public 4090 rates $0.35–0.70/h; **replace with the smoke measurement**)

| Item | Estimate |
|---|---|
| baseline 20 k steps at ~1.5–2 it/s | ~3 h |
| gripper, chunkwise 10 k each | ~1.5 h each |
| plastic 10 k (more parameters train) | ~2 h |
| evaluation: 10 checkpoints × 100 episodes nominal + variations/probes on 4 finals | ~1–1.5 h (MuJoCo on the pod's CPU, inference on the GPU) |
| install, transfer, idle | ~1 h |
| **all-in** | **~9–10 h ≈ $4–7**; cap **$16**; baseline alone ≈ $2–4 |

**Stop rule.** After `smoke.sh`, project `steps / (it/s)` for every enabled run.
If the total exceeds the cap, disable in the order plastic → chunkwise →
gripper (edit `enabled` in `gpu/config.json`) before `full.sh` starts. Write the
measured it/s and the decision into this file.

## RunPod session

Pod: RTX 4090, the official PyTorch template (CUDA 12.x, Python ≥ 3.11), a
**30 GB volume at `/workspace`** (survives a stop; storage still bills), your
ssh public key added in RunPod settings. Connect with the pod's direct address
(`--host root@POD_IP --port PORT`) or the proxy (`--host PODID@ssh.runpod.io`).
No RunPod API key is stored or committed anywhere in this repo.

```bash
# workstation
sim/vla-bed/gpu/transfer.sh --host root@POD_IP --port PORT --remote-root /workspace/robollm --execute

# pod
cd /workspace/robollm
python3 -m venv .venv-gpu && . .venv-gpu/bin/activate
pip install -r sim/vla-bed/requirements-record.txt      # lerobot 0.6.0 + smolvla extras, mujoco, mink, pyyaml
python sim/vla-bed/gpu/preflight.py --execute          # CUDA, LeRobot 0.6.0, revision, decoded v2, 30 GB
sim/vla-bed/gpu/smoke.sh --run baseline --execute      # 10 steps: it/s + VRAM → cost; apply the stop rule
sim/vla-bed/gpu/smoke.sh --run plastic --execute       # separate VRAM check for the unfrozen run
for RUN in baseline gripper chunkwise plastic; do
  sim/vla-bed/gpu/full.sh --run $RUN --execute
  MUJOCO_GL=egl sim/vla-bed/gpu/eval_all.sh --run $RUN --execute
done

# workstation
sim/vla-bed/gpu/download.sh --host root@POD_IP --port PORT --remote-root /workspace/robollm --execute
sim/vla-bed/gpu/cleanup.sh  --host root@POD_IP --port PORT --remote-root /workspace/robollm --execute
# then STOP the pod, verify the download, TERMINATE the pod (the volume bills until then)
```

`cleanup.sh` moves the remote directory into a sibling `.robollm-trash`
rather than deleting it. `download.sh` pulls only `results/p5/` and each
checkpoint's `pretrained_model/` (≈ 865 MB each; 10 checkpoints ≈ 9 GB — pass
`--include` edits if only the selected ones are wanted).

## What comes back

- `sim/vla-bed/results/p5/<pod-host>/<run>/<step>/{nominal,camera_shift,lighting,target_relocation}.json`, `nominal_blank.json`, `nominal_gain0.61.json` for the final checkpoint of each run, `selected.json` per run — all in the SDD §9 schema.
- `artifacts/vla-bed/<run>/full/run_record.json` (wall, peak RSS, argv) and `train_config.json`.
- Checkpoints stay **unpublished** (SmolVLA weights carry no licence tag; SDD §12).

## Recommendation

**Go, on a 4090, all four runs**, with the stop rule armed. Expect the baseline
to solve `nominal` and degrade on the variations; the three variants answer the
questions the SDD asked (R1, R3, R12) with 100 held-out episodes each and
Wilson intervals that separate differences larger than ~10 points. A 0 %
result is still a result: the blank-image and gain probes say why.
