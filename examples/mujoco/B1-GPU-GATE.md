# B1 GPU gate — go/no-go

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [B1 runbook](B1.md) · [Project status](../../docs/PROJECT_STATUS.md) · [Roadmap](../../ROADMAP.md)

Everything B1 can do without a GPU is done and green. This page is the decision
record for the one remaining step, which costs money: renting a CUDA host to
fine-tune `lerobot/smolvla_base` and evaluate the frozen suites.

**Nothing here has been executed.** No model was downloaded, no host rented, no
paid infrastructure created.

## What changed before this gate, and why

The prepared benchmark was too small to buy GPU time for. Measured on the
frozen 50-episode set:

| | frozen set (v1) | scaled set (v2) |
|---|---|---|
| Episodes | 50 | 500 |
| Frames | 587 | **15,163** |
| Train frames | 474 | **12,155** |
| Mean episode length | 11.9 | **30.4** |
| Expert success | 100% | 100% |
| Chunk padding @ 50 | **86.9%** | 63.9% |
| Chunk padding @ chosen size | — | **31.0%** |
| Effective epochs (20k × 64) | **2,700** | **105** |

The oracle drives straight down the slew limit and finishes a reach in ~12
frames, so 20,000 steps at batch 64 meant ~2,700 passes over 474 frames — the
run would have memorised the training set, and the robustness suites would then
have failed for a reason that has nothing to do with camera shift or occlusion.
That result would not have supported the claim B1 exists to make.

Two changes fixed it, both additive; the frozen v1 recipe still reproduces
exactly (587 frames, `valid: true`).

- **`NoisyExpert`** (`reaching.py`, `--expert noisy`): perturbs the goal posture
  with seeded noise scaled by the *remaining* per-joint distance, so the arm
  wanders while far away and quiets down as it arrives. Trajectories get 2.6×
  longer and the dataset gains off-optimal states paired with the corrective
  action — states the straight-line oracle never visits. Tuned to **1.75**:
  100% expert success and zero truncated episodes over 100 seeds. (2.0 reaches
  48-frame episodes but truncates 2%, which would clone failed demonstrations.)
- **400 train / 100 evaluation episodes**, families balanced, split-isolated.

## Chunk size: 20, executing 10

SmolVLA defaults to `chunk_size=50, n_action_steps=50`. Against 30-frame
episodes that is 64% padding, and — more damaging — `n_action_steps=50` means
**one inference per episode**, which makes the roadmap's mid-chunk-abort and
failure-detection rows untestable, because there is never a second chunk.

`chunk_size` in `modeling_smolvla.py` only sizes action query tokens, the
attention mask, and the noise tensor; it never sizes a learned parameter. So
reducing it **loads the `smolvla_base` pretrained weights cleanly** — there is
no trade-off against pretraining here.

Chosen: `chunk_size=20` (1.0 s lookahead at 20 Hz, 31% padding),
`n_action_steps=10` (0.5 s executed, then re-observe). Both are pinned in
`configs/training/b1_smolvla.json` and asserted by `preflight.py`.

## Cost estimate

`smoke_train.sh` runs 10 steps. **Run it first and read the real it/s off it** —
then multiply, rather than trusting the table below. These are estimates from
public rates, not measurements, and hourly prices move.

| Host | Est. throughput @ batch 64 | 20k steps | Est. rate | Est. train cost |
|---|---|---|---|---|
| RTX 4090 24 GB | ~1.5–2 it/s | ~3–4 h | ~$0.35–0.70/h | ~$1–3 |
| A100 80 GB | ~2.5–4 it/s | ~1.5–2.5 h | ~$1.60–2.00/h | ~$3–5 |

Add evaluation (15 suite runs — 3 checkpoints × 5 suites × 20 episodes, MuJoCo
on CPU) and setup. **Realistic all-in: roughly $5–15**, dominated by idle time
while installing, not by training. The 4090 is the better value unless batch 64
will not fit in 24 GB — which the smoke run settles.

Transfer is trivial: the v2 dataset is **12 MB**. `preflight.py --execute`
requires 30 GB free for checkpoints.

## The sequence

Provision a CUDA host, install `requirements/smolvla.txt` into a fresh non-ROS
environment, then:

```bash
scripts/learning/b1/gpu/transfer.sh --host USER@GPU --remote-root /srv/robollm --execute

# On the GPU host:
python scripts/learning/b1/gpu/preflight.py --execute     # CUDA, LeRobot 0.6.0, 30 GB, decoded dataset
scripts/learning/b1/gpu/smoke_train.sh --execute          # 10 steps: proves the stack, measures it/s
scripts/learning/b1/gpu/full_train.sh --execute           # 20k steps, checkpoints at 5k/10k/20k

for step in 005000 010000 020000; do
  scripts/learning/b1/gpu/evaluate.sh --execute \
    --checkpoint artifacts/b1-smolvla/full/checkpoints/$step/pretrained_model
done
python scripts/learning/b1/gpu/select_checkpoint.py \
  --results-root artifacts/b1-results/by-checkpoint --execute

# Back on the laptop:
scripts/learning/b1/gpu/download_results.sh --host USER@GPU --remote-root /srv/robollm --execute
scripts/learning/b1/gpu/cleanup.sh --host USER@GPU --remote-root /srv/robollm --execute
```

`cleanup.sh` is recoverable — it moves the remote directory into a sibling
`.robollm-trash` rather than deleting it.

## Recommendation

**Go, on a 4090.** The preparation is green, the dataset is 26× larger, the
padding problem is fixed, and the whole run is a single-digit dollar amount. The
smoke step bounds the cost before the expensive command runs.

The honest caveat: 12,155 training frames and 105 effective epochs is a
*reasonable* fine-tune regime, not a generous one, and all five goal families
share one scene. Expect a policy that solves `nominal` and degrades on the
perturbed suites. That degradation is the measurement B1 is for — but it should
be reported as "a policy trained on 12k frames of a single scene", never as a
statement about SmolVLA in general.
