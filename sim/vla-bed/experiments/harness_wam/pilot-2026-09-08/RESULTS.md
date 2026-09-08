# CPU harness and predictive-policy pilot — results

The pilot **did not meet the local exploration gate**. Interpret offline forecast/action errors separately from task completion.

Run started 2026-09-08T16:54:34.861383+00:00; source `4d71c29b206a469602792b3badf39b535455a99e` (clean at launch: True). Completed on CPU in **447.4 seconds**. No cloud job, API model call or physical robot was used.

## Offline development evaluation

Episode means on 100 recorded development episodes. 3 training seeds; this table averages their separate episode means, not independent scenes across seeds.

| Policy | Action XYZ MAE (mm), mean across seeds | Future XYZ MAE (mm) |
|---|---:|---:|
| bc | 1.936 | not trained |
| aux_future | 1.912 | 0.979 |
| shuffled_future | 1.936 | 3.317 |

Predicting zero position change gives **3.616 mm** future XYZ MAE. Future prediction is conditioned on the recorded executed command. The action learner imitates the separate clean expert label.

## Closed-loop development pilot

Training seed 7; 20 matched nominal episodes, one observation/action per step, unchanged 3 cm/five-frame success rule and safety checks.

| Arm | Successful | Safe | Both successful and safe | Harness hold frames |
|---|---:|---:|---:|---:|
| oracle | 20/20 | 20/20 | 20/20 | 0 |
| hold | 0/20 | 20/20 | 0/20 | 0 |
| bc | 17/20 | 19/20 | 17/20 | 0 |
| bc+visibility | 16/20 | 19/20 | 16/20 | 100 |
| aux_future | 16/20 | 19/20 | 16/20 | 0 |
| aux_future+visibility | 15/20 | 19/20 | 15/20 | 112 |
| shuffled_future | 16/20 | 20/20 | 16/20 | 0 |
| shuffled_future+visibility | 15/20 | 20/20 | 15/20 | 100 |
| bc+blank | 0/20 | 0/20 | 0/20 | 0 |

| Paired comparison | Success change | Improved / regressed episodes | 95% episode bootstrap interval |
|---|---:|---:|---|
| bc_vs_aux_future | -5% | 0 / 1 | [-15%, +0%] |
| bc_vs_shuffled_future | -5% | 0 / 1 | [-15%, +0%] |
| bc_vs_visibility | -5% | 0 / 1 | [-15%, +0%] |
| aux_future_vs_visibility | -5% | 0 / 1 | [-15%, +0%] |
| shuffled_future_vs_visibility | -5% | 0 / 1 | [-15%, +0%] |

## Interpretation and limits

Auxiliary forecasting reduced action error versus BC across every training seed: **True**. It beat shuffled supervision on action error across every seed: **True**. It beat shuffled forecasts and persistence across every seed: **True**. Its closed-loop joint endpoint changed by **-1/20 episodes** versus BC.

Visibility hold is a bounded intervention that can stall when re-observation cannot reveal the target; it is not recovery planning. Its holds, success and safety must be read together.

The miniature model has 40,605 parameters and handcrafted target-color features. Its task prior, model, training budget and action horizon differ from SmolVLA v6. Do not rank these scores against v6's full-suite result or describe them as WAM/VLM reproduction. There is one closed-loop training initialization and a small, repeatedly used development suite; paired intervals do not include all training or scene uncertainty.

Retain v6 as the existing VLA baseline. A passing pilot would only motivate a larger controlled study. Full-model predictive training and VLM-directed harness recovery remain separate proposals requiring fresh evaluation data and explicit compute budgets.

## Evidence and reproduction

- [Frozen protocol and paper reading](../README.md)
- [Complete run record and per-episode offline errors](record.json)
- [Archive verification and SHA256 values](verification.json)
- Per-arm JSON files beside this report retain all closed-loop episode rows.
- 9 model state dictionaries and normalization values remain in the ignored artifact directory named in the verification record.

Archive checks recomputed counts and paired results from raw rows and verified all saved model hashes. See the protocol and repository validation record for tests run in this session.
