# ReachGuard verification — 21 October 2026

**Local simulation and offline reference verified. Live NVIDIA/Nebius inference
remains unverified.** The draft entry is [PR #9](https://github.com/santapong/RoboLLM/pull/9).
Code was tested at public implementation revision
[`bd278cfc934de5a820a991bedd18f99dc7b1663c`](https://github.com/santapong/RoboLLM/commit/bd278cfc934de5a820a991bedd18f99dc7b1663c).
The subsequent evidence commit adds records and documentation only.

## Measured results

Five declared scenarios, each on seeds **0, 1 and 2**, all using the explicit
deterministic `fixture` reference. All 15 acceptance checks passed; six
successful reach tasks, three deliberately failed baseline tasks, and six
correct hold outcomes. This distinction is retained in every JSON report.

| Scenario | Final measured error, three seeds | Outcome |
| --- | --- | --- |
| Nominal | 10.44 / 9.87 / 10.60 mm | 3/3 reaches passed |
| Displaced target + recovery | 16.75 / 16.19 / 16.95 mm | 3/3 passed; one recovery and two decisions each |
| Old-target baseline | 125.19 / 124.67 / 125.28 mm | 3/3 task failures, as expected |
| Stale observation | Task deliberately withheld | 3/3 held; zero reach commands |
| Out-of-workspace observation | Task deliberately withheld | 3/3 held; zero reach commands |

Reach success requires error below **35 mm throughout the last ten controller
samples**, finite state, no recorded dynamics fault and sampled joint-limit
violation at most 0.001 rad. All runs had zero sampled joint-limit violation.
Safety monitoring covers all 40 ms controller-step endpoints, including command
issuance, rather than a continuous-time collision proof.

The target moves approximately 116 mm at 1.2 simulated seconds. An independent
monitor reads the actual target-body telemetry, detects a version/goal mismatch
above 40 mm, holds and requests a supervisory decision. Measured task error also
uses the actual target-body telemetry. Local validation resolves the target;
the model cannot supply coordinates, arbitrary tools or motor commands.

The fixture is a deterministic recovery policy. This comparison verifies the
monitoring/recovery mechanism; it does not establish that an LLM improves on a
deterministic recovery policy. The selected model must still be evaluated live.

## Evidence and reproducibility

- [Suite summary](evidence/fixture-summary.json), all fifteen individual reports
  alongside it, and [displacement event trace](evidence/displaced-0-events.jsonl).
- [Measured trajectory](evidence/displaced-0-trajectory.jsonl).
- [10-second actual simulation preview](evidence/fixture-preview.mp4) and
  [representative frame](evidence/fixture-preview.png), visibly labeled offline.
- [Test record](evidence/tests.txt): **25 tests and 46 subtests passed** across
  provider, guard/recovery, inherited Concept01 and target-adapter checks.
- [Artifact SHA256 manifest](evidence/sha256.json).
- [Setup and exact commands](../../examples/nebius_recovery/README.md).

Verified environment: Python 3.12.14, MuJoCo 3.10.0, NumPy 2.3.5, Pillow 12.3.0,
pytest 9.1.1; EGL headless rendering and ffmpeg H.264 encoding. Core simulation
also passed while imports of SciPy, Viser, mjviser, trimesh, OpenCV, MediaPipe,
Torch and ROS were denied, confirming the small runtime dependency path.

The committed suite and preview reports identify the public implementation
revision above, with `source_tree_dirty: false`. The source fingerprint is:

```text
a59c971202bbfdd667e4afd3ac7dfdfc90d43bebb0578813c99f99eff370afe3
```

It covers the runner, provider, inherited simulation, Concept01 model metadata,
MJCF and meshes. The inherited `model_sha256` covers decorated scene XML and
includes absolute mesh paths; it can differ between checkout locations. Use the
source fingerprint and public revision for portable provenance. Test execution
preceded the connector-created public commit but used its identical Git tree
(`1bd29ef316f9b4508a95e3e7004e5d12acf5aab0`). The complete suite and preview were
then rerun on the public implementation revision with a clean working tree.

## Model adapter and remaining gate

No configured `NEBIUS_API_KEY` was found. The live CLI with that variable absent
returned exit 2 with a helpful message, without starting simulation or falling
back to a fixture. Contract tests exercise request parameters and malformed,
refused, truncated, wrong-model, timeout and transport-error responses. Actual
authentication, model availability, server JSON compatibility, latency, token
usage and successful runtime inference remain **unverified**.

The candidate is `nvidia/Nemotron-3_5-Lightning`, using the documented Token
Factory endpoint. The model is a text supervisor over simulation telemetry.
Its output is restricted to `reach`, `reobserve`, or `hold`; local guards have
the authority to reject motion. The simulation pauses during the request and
reads scene telemetry again afterwards. Recorded videos omit the network wait,
so playback speed is not inference latency evidence.

Registration, eligibility/account checks, an actual NVIDIA/Nebius-driven run,
platform feedback from use, the final public YouTube video and Devpost
submission still require completion. The 10-second fixture preview is
development evidence; it does not fulfill the final video/runtime requirement.

## Scope and reusable checkpoint

The body is supported at the pelvis, physical parameters are estimates and
self-collision is disabled. There is no physical robot, image perception,
grasping, walking, balance control, or learned VLA/WAM policy in this demo.

For the next session, start here, then read the implementation README and
change manifest. Reuse the standalone environment and shipped Concept01
assets; the richer experiment branch is the correct source, rather than the
older default-branch README. Preserve the boundary between source inspection,
offline dynamics, live inference, media publication and submitted status.
