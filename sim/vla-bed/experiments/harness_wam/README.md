# Harness, VLM, WAM and policy learning in RoboLLM

Research checked 2026-09-08. Experiment branch: `experiment/harness-vlm-wam`.
Base: `9a4a6a2c51e0c9bd20056a0279ea10794a85699e`, the existing UR5e v6
experiment. This intentionally inherits that work instead of starting from the
older integration branch. The original checkout remains on
`experiment/ur5e-vla-bed`.

## What the terms mean

| Concept | Input → output | What changes when it learns? | RoboLLM example |
|---|---|---|---|
| VLM: vision-language model | Images + text → visual/language representations or text | Perception/reasoning weights | SmolVLA's pretrained visual/language backbone |
| Policy, π(a\|o, instruction) | Observation + instruction → robot action or action chunk | Parameters that choose actions | SmolVLA's flow-matching action expert |
| World model | Observation + action → predicted next observation/state | Predictive dynamics weights | A predictor of what the cameras/robot will observe after a command |
| WAM: world action model | Observation + instruction → actions **and** future observations/representations | Coupled action and predictive representations | DreamZero's joint video/action model; latent approaches also exist |
| Harness | Policy + observations/history → managed execution | May change prompts, retrieved memory or execution decisions while policy weights stay fixed | Re-observation, bounded intervention and logging around the policy |

These components can coexist. For example: images and an instruction enter a
VLM; its representations condition the policy; a future-prediction objective
helps train the shared representation; a harness manages execution; the
existing controller and SafetyWrapper apply and measure the command.
The harness is software around the learner, not a synonym for a neural network.

**Policy learning** describes training the action-producing function.
Behavior cloning imitates demonstrated actions. Flow matching is the generative
action-learning objective used by SmolVLA. Reinforcement learning instead learns
from reward-driven interaction; DAgger collects learner-visited states and asks
an expert for labels. This pilot uses supervised imitation, with an optional
predictive auxiliary loss. It performs neither RL nor DAgger.

## Papers and what we actually checked

1. [SmolVLA — Shukor et al., 2506.01844v1](https://arxiv.org/abs/2506.01844v1).
   Selected primary sections: architecture, training, evaluation and ablations.
   A pretrained VLM conditions a flow-matching action expert; the main recipe
   freezes the VLM. The paper describes 450M total parameters, about 100M in the
   expert. This explains RoboLLM's existing learner. Its async experiment uses
   real pick-place trials; those timings are not a CPU performance guarantee.
   Local LeRobot adapter/training code was inspected; this task did not rerun
   the paper's benchmarks. See [the existing train driver](../../gpu/train.py).

2. [RoboHarness — Li et al., 2603.24060v3](https://arxiv.org/abs/2603.24060v3).
   Selected primary sections: III–IV, pp. 3–8. Success/failure memories guide
   Qwen3-VL-32B orchestration of perception, instruction and execution tools
   around frozen policies. Its simulated LIBERO experiments specify 199
   randomized rollouts per task and a preliminary pass to populate memory.
   Reported overhead is amortized across 1,000 simulator steps; it is not a
   per-call VLM latency. Some memory-quality ratings use the orchestrator model
   itself. Transfer to this UR5e task and physical robots remains unproven here.
   [Official code](https://github.com/LZY-1021/RoboHarness) was located, not audited
   or executed. Our visibility hold is only a primitive harness intervention.

3. [DreamZero: World Action Models are Zero-shot Policies — Ye et al.,
   2602.15922v1](https://arxiv.org/abs/2602.15922v1).
   Selected primary sections: model formulation, evaluation and limitations.
   It jointly predicts video and actions from a pretrained video-diffusion
   backbone. The 14B model's reported 7 Hz control uses two GB200s, according to
   the limitations section. Its real-robot results concern different robots,
   data and tasks; a headline generalization ratio is not a UR5e success rate.
   Trial counts were not fully audited in this selective reading, so no benchmark
   score is adopted as an experiment target. [Official project](https://dreamzero0.github.io).
   We do not attempt to train or serve this backbone locally.

4. [JEPA-WAM — Lin et al., 2608.09381v1](https://arxiv.org/abs/2608.09381v1).
   Selected primary sections: §3.1 and appendices A–B. A frozen V-JEPA encoder
   defines spatially structured current–future targets; transition supervision
   shapes a shared predictor used for action generation. Its auxiliary objective
   provides motivation for this pilot. The paper's policy training uses 60K steps
   on eight GPUs. We retain the idea of shared predictive supervision while
   replacing its encoders, spatial tokens, loss and action expert with a tiny
   feature MLP. This is an adaptation, not a JEPA-WAM reproduction. Exact trial
   counts and all evaluation tables were not audited here.

Primary source existence, selected-section reading, implementation and independent
reproduction are separate evidence levels. The four papers were read through the
connected alphaXiv reader and their arXiv identities checked directly.

## Existing baseline and bounded question

The current v6 result at `results/p5/bd883c780fba/baseline/010000/nominal.json`
contains 89/100 successful trials, 89/100 safe trials, and 80/100 satisfying both.
This is previously imported simulated reaching evidence, re-counted from its raw
rows for this task. Success means within 3 cm for five frames. The suite has been
used for repeated model selection and is development evidence.

v6 already demonstrates the importance of front+wrist observations. It does not
establish whether predictive supervision improves learning, or whether a
particular runtime intervention helps. Our question is narrower: **with identical
observations and action labels, does a small auxiliary dynamics objective help a
CPU policy, and does a visibility hold change success and safety?**

The local trained checkpoint available for SmolVLA was v3; v6 evaluation rows and
demonstrations were available, but a local trained v6 model was not found. This
pilot does not replace v6 with v3 or compare tiny-policy scores as if model scale,
training budget and execution horizon were matched.

## Frozen pilot protocol

The protocol and implementation are recorded before running the experiment.

- **Data:** existing v6, 400 training and 100 evaluation episodes. Fit on the first
  320 training episodes; reserve 80 training episodes for calibration/diagnostics.
  Each block contains all 20 cells. The 100 existing evaluation episodes are
  development data; no final generalization test is claimed.
- **Inputs:** 14 proprioceptive values and four fixed red-target features from
  each RGB camera: visibility, centroid u/v and square-root area. No target
  coordinates, episode IDs, reward or simulator errors enter the network.
  The single instruction is fixed, so no language competence is tested.
- **Alignment:** pair adjacent frames only inside an episode. Exclude its final
  frame from all training arms. Clean `action` is the imitation label;
  `action.executed` conditions the world branch, because the observed future
  resulted from that noisy executed command. Using the clean label to explain
  the observed transition would be a data error. Validate video timestamps/counts
  and table ordering; hash every input file.
- **Models:** identical two-layer 128-unit shared encoders and action/forecast
  heads. BC optimizes normalized action MSE. Auxiliary forecasting adds 0.1 ×
  normalized next-feature-delta MSE, conditioned on the executed command.
  The shuffled-future control permutes delta targets within fit data, preserving
  their marginal distribution. This is supervised feature dynamics, not a
  generative WAM. Future labels are fixed, avoiding a learned-target collapse.
- **Training:** seeds 7, 17, 27; 1,000 AdamW steps per arm, batch 256, learning
  rate 0.001, weight decay 0.0001, gradient norm limit 1.0. Same initialization
  and minibatch schedule for corresponding arms. All normalization is fit-only.
  Use the final step for every arm; no checkpoint or hyperparameter selection.
- **Offline metrics:** episode-averaged action XYZ MAE, rotation MAE, and
  predictive XYZ MAE against a zero-motion persistence control. Future error
  uses executed-action conditioning. Report all three training seeds.
- **Closed loop:** preselected training seed 7, first 20 development episodes
  (one per cell), nominal view, 100-frame limit, one action per observation.
  Call the existing `evaluate.run_policy_episode` and `aggregate` functions.
  Its success detector, controller and SafetyWrapper remain unchanged.
- **Runtime arms:** each of the three policies alone and with visibility hold.
  Hold submits zero deltas for the current frame when both cameras have no
  detected red pixels, then re-observes. It cannot find an occluded target and
  can stall; it has no memory, VLM planner, rollback or safety guarantee.
  Oracle and hold are positive/negative controls. BC with blank visual features
  tests reliance on vision. Observation detector code is shared across data and
  rollout; compression differences remain a possible domain gap.
- **Reporting:** success, safety, joint success-and-safety, rejections, measured
  faults, hold counts, latency and wall time. Require equal unique seed sets
  before every paired comparison; bootstrap across episodes. One closed-loop
  training seed and 20 related scenes are feasibility evidence, not a decisive
  superiority or rare-event safety estimate.
- **Resource bound:** CPU only, one Torch/decode thread, no API calls or cloud
  jobs. Nine training fits × 256,000 sampled transitions; nine rollout arms ×
  20 episodes. The initial command has a 15-minute process limit; partial results
  remain identifiable and must not be reported complete. Record actual costs.

**Decision rule:** stop on mismatched provenance, nonfinite data/training, or
failed oracle/hold controls. A direction merits further study only if action
error improves consistently over BC across all three seeds, the correctly
paired future objective beats shuffled targets, and closed-loop joint
success-and-safety improves without more measured faults. A tie or failure is
an inconclusive/negative pilot, not a reason to tune on these same results.
Passing only motivates a properly powered v6-based follow-up with an untouched
test set. It does not establish WAM or RoboHarness superiority.

## Run and inspect

Reuse the existing Python 3.13 LeRobot environment; no installation is needed
on this workstation. The simulator assets are referenced read-only from the
original checkout. Substitute paths on another machine after setting up the
existing bed dependencies and pinned Menagerie assets.

```bash
cd /home/santapong/RoboLLM-harness-wam
VLA_BED_MENAGERIE=/home/santapong/RoboLLM/sim/vla-bed/assets/mujoco_menagerie \
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
timeout 15m nice -n 19 /home/santapong/RoboLLM/.venv-lerobot/bin/python \
  sim/vla-bed/experiments/harness_wam/experiment.py \
  --dataset /home/santapong/RoboLLM/datasets/vla-bed/v6 \
  --out artifacts/harness-wam/pilot-2026-09-08
```

`--rollout-episodes 0` runs the offline part only. Existing output directories
are refused, preventing accidental replacement of evidence. Models and detailed
run records stay in ignored `artifacts/`; a compact checked result is filed next
to this protocol. Checkpoint files contain local state dictionaries; load with
`torch.load(..., weights_only=True)`.

## Follow-up on the actual v6 VLA

1. Reuse the selected v6 10K model with its exact two-camera schema and normalizer.
   Hold all fine-tuning, chunk, device and evaluator settings fixed.
2. Compare unchanged v6, a deterministic intervention, and a VLM-directed
   intervention with the same sensing/call budget. Use training-only memories;
   keep test memories frozen and prevent cross-test outcome leakage.
3. Separately compare action-only v6 training with a carefully aligned future
   representation objective and shuffled-target control. Fix the known notebook
   preparation defects before a cloud run; see the current experiment skill.
4. Reserve fresh evaluation seeds/scenes before selecting any intervention.
   Record GPU/VLM costs and use the joint success-and-safety endpoint. Full
   generative WAM training is a separate compute proposal, not a consequence
   of this CPU pilot.

No cloud training, physical-device operation, remote push or publication is part
of the completed local pilot. Its results cannot validate physical transfer.
