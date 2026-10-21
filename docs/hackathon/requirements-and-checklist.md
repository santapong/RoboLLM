# Entry gates and deadline

Checked 21 October 2026 against the
[official rules](https://nebiusglobalaihackathon.devpost.com/rules).
Event: [Nebius × NVIDIA Global AI Hackathon](https://nebiusglobalaihackathon.devpost.com/).

## Verified event requirements

- Deadline: **30 October 2026, 10:00 PDT = 31 October, 00:00 Thailand**. Treat
  Friday evening, 30 October, as the final submission evening.
- Eligible development period: **26 August, 16:00 UTC–30 October, 17:00 UTC**.
- Register through Devpost and obtain required platform access.
- A working entry must use an NVIDIA open-source model and call Token Factory
  at runtime or run on eligible Nebius AI Cloud compute.
- Physical AI accepts demonstrations of operating software modules without
  hardware; reserve at least one minute for those modules.
- Supply a public repository with source/assets, setup/run instructions and a
  detectable open-source license; explain substantial eligible-period changes
  when reusing a project.
- Supply a project description, track selection and platform feedback.
- Video: **under three minutes**, publicly visible on **YouTube**. Submission
  materials must be English or translated.
- A hosted demo URL is waived for Physical AI. Keep reviewer testing access
  available through judging; do not assume that waiver removes reproducibility.

## Current gates

Unchecked items remain unverified, not necessarily absent.

| Gate | Evidence needed | State |
| --- | --- | --- |
| Project choice | User selects or accepts the proposed ReachGuard concept | Proposed |
| Registration/eligibility | Entrant checks eligibility, joins event and can edit a draft | Unverified |
| Nebius access | Account can use the selected NVIDIA model; credits/billing understood | Unverified |
| Repository license | Root Apache-2.0 license retained; asset notices preserved | Existing source present |
| Local implementation | Runner and strict decision validation present | Implemented in draft PR #9 |
| Offline function | Fresh scenario suite, measured error, refusal and budget behavior | Verified: 15 runs; 25 tests and 46 subtests |
| Live integration | Actual NVIDIA response via Nebius influences a validated simulated reach | Unverified until live run |
| Demonstration evidence | Reviewed render/trace and public YouTube video | Offline preview/trace verified; final public video pending |
| Public final code | Exact public commit with complete setup/run instructions | Public experiment branch and draft PR; final submission revision open |
| Submitted entry | Completed form and visible submission status before deadline | Unverified |

## Engineering acceptance gates

Freeze these before the final benchmark; do not move thresholds to rescue a
failing run.

| Scenario | Required observation |
| --- | --- |
| Nominal reach | Measured hand error reaches the runner's documented tolerance within its fixed step/time budget |
| Target displacement | Monitor exposes mismatch; recovery uses a fresh validated target; endpoint is measured against that target |
| Stale/malformed observation | No new reach is authorized from the invalid observation; trace explains the hold/refusal |
| Invalid model output | Unknown action, extra authority, malformed JSON and invalid values fail closed |
| Recovery exhaustion | No additional retry after the fixed maximum; state and reason remain visible |
| Provider failure | Timeout/API/parse failure is surfaced and does not silently become a fixture success |
| Simulation health | Finite state, reported joint-limit violations and numerical faults are checked for every run |
| Trace integrity | Provider mode, model identity, observation/event, accepted decision and outcome can be connected to one run |

The existing simulator reports a 35 mm reach criterion. State the final
runner's criterion explicitly and preserve the actual error even on failure.
Joint setpoint limits do not prove physical safety or collision avoidance.

## Thailand delivery plan

This is a proposed work plan, not an automation or a claim of booked time.
Each date has a stop condition so the entry stays small.

| Thailand date | Outcome and gate |
| --- | --- |
| 21 October | Complete branch-aware audit; prepare isolated implementation; keep registration and credentials marked unknown |
| 22 October | Confirm concept, registration and Nebius access; stop expanding scope if access is blocked |
| 23 October | Pass local nominal/displacement scenarios with recorded telemetry |
| 24 October | Complete one live NVIDIA/Nebius-to-simulation run; inspect sanitized evidence |
| 25 October | Pass stale/invalid/budget/provider-failure cases; freeze scenario definitions |
| 26 October | Repeat from clean setup; resolve installation and asset gaps |
| 27 October | Record the 2:45 storyboard; verify all visible claims against logs |
| 28 October | Publish exact code revision and public video; test reviewer instructions signed out |
| 29 October | Complete Devpost description/feedback/change disclosure and review the draft |
| 30 October, by 21:00 | Submit and verify status; reserve three hours before the midnight deadline |

If the live integration gate has not passed by 24–25 October, prioritize access
and a single end-to-end run. Extra robot models, training, hardware, ROS and new
perception features do not unblock that gate.
