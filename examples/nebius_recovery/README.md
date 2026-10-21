# ReachGuard — bounded recovery for humanoid reaching

Small proposed Nebius × NVIDIA Physical AI entry built on RoboLLM's existing
Concept 01 humanoid. A simulated inspection target moves during a reach. A
supervisor chooses **reach**, **reobserve**, or **hold**; deterministic checks
and the existing classical controller own all coordinates and joint commands.

**Current status: locally implemented; live NVIDIA/Nebius inference is
unverified.** Offline fixture output cannot satisfy the platform requirement.
See the [verification record](../../docs/hackathon/verification.md) for measured
results and the [comparison](../../docs/hackathon/comparison.md) for the repo audit.

## Scope

- Real MuJoCo joint dynamics, measured hand error, an independently observed
  target, displacement monitoring, one bounded recovery, and invalid-data hold.
- Structured **simulator telemetry**, rather than image perception, goes to a
  text model. This is a supervision experiment for robotics learners.
- Supported pelvis, estimated physics, self-collision disabled. No walking,
  balance, grasping, VLA/WAM inference or physical-safety claim.
- The simulation pauses during a blocking API request; latency is recorded
  separately. The controller checks the simulated scene again before motion.
  This does not establish freshness of physical sensors during a network call.

## Reproduce locally

Use Python 3.12 and a fresh environment, separate from the ROS/Jazzy environment.
No ROS, GPU, webcam, hardware or additional robot-asset download is needed.

```bash
python3.12 -m venv .venv-nebius
source .venv-nebius/bin/activate
python -m pip install -r examples/nebius_recovery/requirements.txt
python -m pytest -q tests/unit/test_nebius_provider.py tests/unit/test_nebius_recovery.py
python -m examples.nebius_recovery.evaluate --provider fixture --output artifacts/nebius/fixture-suite
```

This evaluates all five scenarios on declared seeds 0, 1 and 2. A complete
live suite makes at most 18 inference requests; there are no API retries.
The comparison tests the monitoring/recovery mechanism, **not LLM superiority**.
The fixture acts as the deterministic reference policy.

| Scenario | Required outcome |
| --- | --- |
| `nominal` | Reach observed target within 35 mm throughout the last ten samples; valid bounded dynamics |
| `baseline` | Continue toward old target after displacement; measured task failure expected |
| `displaced` | Monitor detects displacement; one reobserve/recovery reaches the new target |
| `stale` | Hold; zero reach commands |
| `out_of_bounds` | Hold; zero reach commands |

Each run writes `report.json`, `events.jsonl`, and `trajectory.jsonl` with source
hashes, model provenance, measured outcomes and provider labels. Expected
baseline failure can pass the evaluation acceptance check while `task_success`
remains false. These are separate fields.

## Render the observable demo

```bash
MUJOCO_GL=egl python -m examples.nebius_recovery.run --provider fixture --scenario displaced --render --output artifacts/nebius/fixture-video
python -m examples.nebius_recovery.video artifacts/nebius/fixture-video
```

The second command requires `ffmpeg`. It encodes actual rendered frames at the
recorded simulation frame rate into `simulation.mp4`. New output directories
prevent stale frames from contaminating evidence. On a desktop, use the local
OpenGL default by omitting `MUJOCO_GL=egl`; EGL needs a compatible driver.
Physics tests and non-rendered runs do not create an OpenGL rendering context.

Green marks the observed point; orange marks the commanded point. The panel
shows scenario, provider, time, event, decision, measured error and budgets.
Offline output is visibly marked **FIXTURE — no cloud inference**. API wait
wall time is excluded from the encoded physics clip and must not be presented
as inference speed. The short clip is development evidence, not the final
submission video.

## Live NVIDIA integration gate

1. Join the hackathon yourself, obtain Token Factory account access/credits,
   and create an API key. Registration and account access are not verified here.
2. Set `NEBIUS_API_KEY` in your local shell without committing it. Do not paste
   a key into a public issue, source file or recording.
3. Check the selected model is available to your account, then execute:

```bash
# Bash: hidden prompt; the key is not written into this command or source.
read -rsp 'Nebius API key: ' NEBIUS_API_KEY
export NEBIUS_API_KEY
export NEBIUS_MODEL=nvidia/Nemotron-3_5-Lightning
MUJOCO_GL=egl python -m examples.nebius_recovery.run --provider nebius --scenario displaced --render --output artifacts/nebius/live-displaced
unset NEBIUS_API_KEY
```

The default model is the NVIDIA model named in the current
[official Nebius cookbook](https://github.com/nebius/token-factory-cookbook/blob/main/models/nemotron/README.md).
Requests use the fixed
[`https://api.tokenfactory.nebius.com/v1/chat/completions`](https://docs.tokenfactory.nebius.com/quickstart)
endpoint, JSON output, temperature 0, 512 output tokens, a 20-second socket
timeout, no redirects and no retries. Only `nvidia/` model IDs are accepted.
The exact returned model must match the requested model; a catalog listing is
not successful inference. Model availability and JSON behavior still need the
live test. A socket timeout is not a hard total wall-clock deadline.

The adapter rejects invalid/duplicate JSON keys, extra fields, tool calls,
refusals, truncated output and wrong model identity. The controller separately
rejects stale, nonfinite, out-of-workspace or changed-scene requests and limits
recovery to one attempt and inference to two calls per scenario. Errors hold;
there is **no silent fixture fallback**. A hold can be the correct safe outcome
without satisfying the reach task.

Sanitized evidence records the symbolic decision, requested/served model,
timestamp, request identifier, latency and token usage. It excludes credentials
and raw transport/error bodies. Audit success must accompany a successful
robotics outcome before marking a live scenario verified.

## Submission materials

- [Editable submission copy](../../docs/hackathon/submission-draft.md)
- [2:45 storyboard and recording checklist](../../docs/hackathon/demo-script.md)
- [Official requirements, gates and Thailand schedule](../../docs/hackathon/requirements-and-checklist.md)
- [Existing work versus substantial new changes](../../docs/hackathon/change-manifest.md)
- [Apache-2.0 license](../../LICENSE); inherited [model provenance](../humanoid_harness/MODEL_NOTICE.md)

Deadline: **31 October 2026 at 00:00 Thailand**, the beginning of that day.
Target completion and upload on 30 October. The
[official rules](https://nebiusglobalaihackathon.devpost.com/rules) allow
simulation-only Physical AI, require at least one minute of operating modules
in a public YouTube video under three minutes, and waive the hosted-demo URL.
Public licensed code, setup instructions, an English description, platform
feedback, and disclosure of significant updates are still required. Final
registration, live evidence, public video and Devpost submission remain open.
