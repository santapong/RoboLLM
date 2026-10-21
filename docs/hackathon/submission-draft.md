# ReachGuard — bounded recovery for humanoid reaching

**Working title; proposed Physical AI entry.** This is editable submission copy,
not a submitted Devpost entry. Replace the evidence fields only after running
the corresponding checks. Registration, account access, live inference and the
public video remain separate gates.

## Elevator pitch

A robot should notice when its target has moved before repeating a failed
action. ReachGuard studies that decision in a small humanoid simulation: a text
supervisor proposes a recovery, while deterministic code decides whether the
next reach is permitted.

## Inspiration and audience

Robotics developers often need to explain a failure before they can improve a
controller. A moving target, a stale observation and an unreachable request can
look similar in a video, but demand different responses. ReachGuard is intended
for learners and prototype developers who need an observable, reproducible
example of that distinction without buying new hardware.

The concrete scenario is a supported humanoid reaching toward an inspection
point. The point can move between observations. The application exposes target
state, measured hand position, error and the recovery decision together so a
viewer can see what changed and why the controller continued or held.

## What the proposed entry does

1. Run the existing original Concept 01 humanoid in MuJoCo with a visibly
   supported pelvis, classical IK and position servos.
2. Measure progress toward the current observed target and surface a displaced
   target or invalid observation as a structured event.
3. Send compact event telemetry to an NVIDIA Nemotron text model through Nebius
   Token Factory. The supervisor selects from a small permitted decision set.
4. Validate that decision independently before any new target is executed.
   Workspace, finiteness, observation validity and recovery-budget checks belong
   to application code.
5. Produce a trace and a rendered simulation with measurable acceptance results.

These describe the intended entry behavior. The implementation and current
verification record in the root README determine which claims are ready for
publication.

## NVIDIA and Nebius integration

The candidate provider is Nebius Token Factory's OpenAI-compatible text API,
with the currently documented model ID `nvidia/Nemotron-3_5-Lightning`. Account
availability must be checked before the final run. This model is a reasoning
supervisor over structured simulation telemetry; it is not a learned motor
policy or image-perception system.

The model must contribute an actual runtime decision to the recorded robotics
flow. Merely storing this model name in a config, using fixture output, or
displaying a canned response does not demonstrate the required integration.
Preserve the requested and returned model identities, sanitized request
metadata, decision, latency and resulting execution outcome. Never publish API
keys or authorization headers.

## How it is built

The foundation is the existing Apache-2.0 RoboLLM experiment at `f65f692`.
The local Concept 01 assets avoid additional robot-model downloads. New
ReachGuard code adds the event supervisor, provider adapter, bounded recovery
flow, evaluation scenarios and submission evidence. See
[the change manifest](change-manifest.md) for the separation between existing
work and changes made for this entry.

The local fixture provider supports reproducible offline development. Its output
must be labeled **fixture**, and it cannot substitute for a live NVIDIA/Nebius
run in submission evidence.

## Results to fill from measured evidence

| Evidence | Current draft value |
| --- | --- |
| Exact submission commit | Pending final commit |
| Local test command and result | Pending root verification record |
| Nominal reach error and completion | Pending measured run |
| Target-displacement recovery outcome | Pending measured run |
| Invalid/stale observation outcome | Pending measured run |
| Invalid model decision outcome | Pending measured run |
| Recovery-budget exhaustion outcome | Pending measured run |
| Live provider/model, call count and latency | Unverified account/runtime access |
| Public repository revision URL | Pending publication of the submission revision |
| Public YouTube video URL and duration | Pending recording/upload |

## Limitations and what comes next

The simulated body is supported at the pelvis; its physical parameters are
design estimates. Its fingers are fixed and robot self-collision is disabled.
The entry demonstrates target-reaching supervision under specified test
conditions. It does not demonstrate walking, balance, grasping, VLA/VLM/WAM
inference, physical deployment or general robot safety.

The next research step would be a held-out target/fault suite comparing the
model supervisor with an explicit deterministic recovery policy. Such a
comparison needs measured trials; the presence of a model does not itself
establish improved success or efficiency.

## Platform feedback draft

Complete this after the live run: model discoverability and account access;
JSON-format reliability; latency distribution; token usage/cost when available;
timeout behavior; and one concrete improvement suggested by observed behavior.
Do not fabricate platform feedback from fixture results.
