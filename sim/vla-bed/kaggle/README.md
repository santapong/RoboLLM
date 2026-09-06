# Kaggle route (Route K) — free GPU for the bed's fine-tune

Why: SDD Q-B needs *a* GPU for a few hours, not a rented 4090. Kaggle gives
30 GPU-hours a week (T4 x2 or P100), 12-hour sessions, and background runs.
Everything here is free; RunPod (`../GPU-GATE.md`) is the fallback if the
smoke shows a 10k-step run cannot fit an 8-hour session.

## Once

1. Account: phone-verified (unlocks GPU + Internet in notebooks).
2. `sim/vla-bed/kaggle/make_bundle.sh` → `artifacts/vla-bed/vla-bed-v2.zip` (45 MB).
   kaggle.com → Datasets → **New Dataset** → upload the zip → title **vla-bed-v2** → Private.
   Kaggle unzips it to `/kaggle/input/vla-bed-v2/v2/`.
3. The branch `experiment/ur5e-vla-bed` must be **pushed** — the notebooks clone it.

## Smoke (≈ 15 min of quota)

kaggle.com → Code → **New Notebook** → File → Import Notebook → `smoke.ipynb`.
Right panel: **Add Input** → your dataset `vla-bed-v2`; Settings → Accelerator
**GPU T4 x2** (P100 if T4 is unavailable); **Internet ON**. Run all.
The last cell prints the projection table and `DECISION:`; paste it into the session.

## Training (one run per session, unattended) — then `eval.ipynb`

Import `train.ipynb`, same inputs and settings. Edit the first cell from the
projection table (`RUN`, `STEPS`, `BATCH`, `VLM_DTYPE`, `SAVE_EVERY`), then
**Save Version → Save & Run All (Commit)**. It trains, scores the last checkpoint on 50 nominal episodes, and packs **all**
checkpoints into `vla-bed-<run>-output.zip` (Output tab). Then import `eval.ipynb`,
add the dataset **and** the train notebook's output as inputs (Add Input → Notebook
output), and run it: every checkpoint on the 100 held-out episodes, variations and
probes on the last, selection by closed-loop success, packed as `vla-bed-<run>-eval.zip`.
Why two sessions: MuJoCo on Kaggle's CPU costs ≈ 66 s per failing episode (smoke v5). Back on the workstation:

    sim/vla-bed/gpu/kaggle_import.sh ~/Downloads/vla-bed-baseline-output.zip <sha256 printed by the notebook>

Runs, in order of value: `baseline` → `gripper` → `chunkwise` → `plastic`.

## Limits that shape the settings

- A version killed at 12 h keeps **no** output: `MAX_HOURS` stops training early
  and the rest evaluates; keep `STEPS/steps_per_s + eval` under 8 h.
- No bfloat16 on T4/P100: the smoke times bfloat16 (emulated) against a
  float16 cast of the frozen VLM (`--vlm-dtype float16`).
- 20 GB output: the pack step keeps only the selected checkpoint.
- Nothing needs your Kaggle API token; uploads and downloads happen in the browser.

## Probe session on existing checkpoints — `probe.ipynb` (no training)

Same inputs as `eval.ipynb` (dataset `vla-bed-v2` + the train notebook's output), GPU T4 x2,
Internet on, **Save & Run All**. It runs the open-loop magnitude probe
(`gpu/magnitude_probe.py`: predicted vs label step magnitudes on 500 training frames) for the
selected and the final checkpoint, then closed-loop probes on the selected checkpoint with the
schema-v2 evaluator (commanded magnitudes and rejected fraction per episode): nominal, clip to
the S2/S3 limits (`--post clip`), gain 0.61, linear temporal ensemble (`--post ensemble
--replan-every 5`), `lighting`, `target_relocation`, and a 20-episode float16-VLM consistency
check; `compare.py` pairs every probe with the nominal suite on the same seeds. Every suite runs
as `WORKERS` shards (`evaluate.py --shard i/n`, merged with `--merge`), and checkpoints are
unpacked under `/tmp`, so the Output tab holds only `vla-bed-<run>-probes.zip` (results JSONs,
< 1 MB) → `gpu/kaggle_import.sh <zip> <sha256>` on the workstation.

## DAgger round (recipe v7, plan step 5)

`dagger.ipynb` — inputs: the base dataset `vla-bed-<recipe>` and the train notebook's output whose selected checkpoint should drive (attach the eval output too if `selected.json` should pick the step). The checkpoint runs 400 fresh train-split seeds (seed block 20 000) while the capped oracle labels every frame; packs `vla-bed-v7.zip` (≈ 50 MB). Then `train.ipynb` with inputs `vla-bed-<recipe>` **and** the dagger notebook's output merges base ∪ rollouts (`dagger.py merge`) and trains on the union; its output zip is named `vla-bed-baseline-v7-output-<recipe>.zip`, so `eval.ipynb` needs `RUN_TAG = "baseline-v7"`.
