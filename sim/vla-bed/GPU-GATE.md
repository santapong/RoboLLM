# GPU gate — Phase 4 fine-tune of SmolVLA on the UR5e bed

**Status: Route K — baseline TRAINED and EVALUATED on Kaggle, $0** (`santapongsondhi/vla-bed-train` v1: float16 VLM, batch 32, 10k steps, checkpoints at 2.5k/5k/7.5k/10k, 3 h 56 min wall, output `vla-bed-baseline-output.zip` 3.52 GB; `santapongsondhi/vla-bed-eval` v2: 7 of 9 suites in 7.81 h — lighting and target_relocation cut by the 7.5 h budget — selected checkpoint 7.5k at 0.20 [0.133, 0.289], output `vla-bed-baseline-eval.zip` sha256 4118f3f7…8645e9737, **import pending** via `gpu/kaggle_import.sh`; numbers transcribed in `results/p5/kaggle/eval-v2-partial.json`, charts in `results/REPORT-2026-09-04.md`). Nothing billed; Kaggle quota used ≈ 12 h of 30 this week. This is the priced
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

## Route K — Kaggle first (free)

The objective (SDD Q-B) needs *a* GPU for a few hours, not a rented 4090. Kaggle
gives 30 GPU-hours a week on a T4 x2 or P100, 12-hour sessions with background
execution, and the user's account is phone-verified. Files: `kaggle/README.md`,
`kaggle/make_bundle.sh` (45 MB private dataset `vla-bed-v2`), `kaggle/smoke.ipynb`,
`kaggle/train.ipynb`, `gpu/kaggle_import.sh`.

**What the smoke measures** (≈ 15 min of quota): three 10-step timings —
bfloat16 as LeRobot ships it (the VLM is loaded in bfloat16, hard-coded;
T4/P100 have no native bfloat16), the frozen VLM cast to float16
(`--vlm-dtype float16`), and a smaller batch — plus the renderer (EGL or OSMesa),
a 5-episode oracle control, and a 2-episode SmolVLA rollout for GPU inference
latency. Its last cell prints:

| trial | steps/s | VRAM GB | 5k h | 10k h | 20k h | fits 8 h? |
|---|---|---|---|---|---|---|
| bf16-b32 (as LeRobot ships it) | 0.203 | 5.9 | 6.9 | 13.7 | 27.4 | 5k only |
| **fp16-b32** (frozen VLM cast to float16) | **0.716** | 4.9 | 1.9 | **3.9** | 7.8 | **10k** |
| bf16-b16 | 0.351 | 3.6 | 4.0 | 7.9 | 15.8 | 5k only |

**Measured 4 Sep 2026, Kaggle version 4 of `santapongsondhi/smoke`, Tesla T4 15.6 GB,
torch 2.10.0+cu128, LeRobot 0.6.0, repo `52107c6`.** Preflight on the box: 14,423
frames decoded, 0 clean-label faults, `smolvla_base` revision `c83c316`. Clone +
install took 45 s; each 10-step trial includes model load. bfloat16 runs 3.5×
slower than float16 on the T4 (no native bfloat16 there), as expected. Peak RSS
4.0 GB. Renderer: EGL OK. Version 5 (570 s) repeated the timings (fp16-b32 **0.763**, bf16-b32 0.214, bf16-b16 0.393 steps/s) with the Menagerie clone in place: oracle 5/5 on the box, and the 10-step checkpoint ran 2 episodes through the evaluator at **0.54 s per chunk** on the GPU. **MuJoCo stepping on Kaggle's 4 vCPUs is ≈ 15× slower than the workstation (≈ 66 s per 100-frame episode vs 4 s)**, so the full suite cannot share an 8 h session with training: `train.ipynb` scores only the last checkpoint on 50 episodes and packs all checkpoints; `eval.ipynb` runs the full suite from that output in its own session.

**DECISION (printed by the notebook): Route K with fp16-b32, 10k steps.**

**Decision rule.** Route K goes if some trial fits **10k steps + evaluation of
4 checkpoints × 100 episodes inside 8 h** (5k steps is the reduced unit if only
that fits). Then `train.ipynb` runs one run per session with `MAX_HOURS` stopping
training early enough to evaluate and pack: `baseline` first, then `gripper`,
`chunkwise`, `plastic` within the weekly quota. Cost $0; calendar cost about a
week. **RunPod** (below) if nothing fits 8 h, or Kaggle refuses a GPU twice.
Either way: checkpoints chosen by closed-loop success (R11), never published.

**Kaggle limits that shaped the notebooks.** A version killed at 12 h keeps no
output, so the run is sized from the smoke, never guessed; the 20 GB output cap
means only the selected checkpoint is packed; no Kaggle API token is needed.

## RunPod session (runpodctl 2.12.0, installed rootless in `~/.local/bin`, 4 Sep 2026)

Authentication is the owner's: `runpodctl doctor` (interactive: pastes the API key
from https://console.runpod.io/user/settings into `~/.runpod/config.toml` and
registers `~/.ssh/id_ed25519.pub` with the account). No key is stored in this
repo. The plugin's rule: a real key first, because the MCP's OAuth alone leaves
the CLI blocked.

Pod: official template **`runpod-torch-v280`** (`runpod/pytorch:1.0.2-cu1281-torch280-ubuntu2404`,
CUDA 12.8), one **RTX 4090**, secure cloud, **30 GB volume at `/workspace`**
(survives a stop; storage still bills), 20 GB container disk, ssh on. Creation is
the first billed action and happens only on the owner's go.

```bash
# workstation — create, wait for ssh, read the connection details
runpodctl gpu list | grep -i 4090                      # id + current price
runpodctl pod create --name vla-bed --template-id runpod-torch-v280 \
    --gpu-id "NVIDIA GeForce RTX 4090" --volume-in-gb 30 --container-disk-in-gb 20 \
    --ports 22/tcp --ssh --wait --wait-timeout 10m
runpodctl pod get <pod-id>                             # ssh host + port (direct) — or `runpodctl ssh info <pod-id>`
sim/vla-bed/gpu/transfer.sh --host root@POD_IP --port PORT --remote-root /workspace/robollm --execute

# pod (over ssh)
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
runpodctl pod stop <pod-id>        # GPU billing stops; verify the download
runpodctl pod terminate <pod-id>   # the volume bills until this
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
