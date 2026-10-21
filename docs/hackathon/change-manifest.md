# Change and provenance manifest

Prepared 21 October 2026. This separates the repository's existing robotics
work from the ReachGuard extension. It is not a claim that every item has been
executed or submitted.

## Frozen inspection baseline

| Source | Revision | Treatment |
| --- | --- | --- |
| RoboLLM `main` | `6413ef98652d4ec53914b4908ecd2107d40ad3a3` | Existing ROS 2, MCP and browser workbench |
| RoboLLM `develop` | `838eafeca4d8312eae4f197ab1919d0680b55088` | Existing platform/arm/dataset organization and implementation |
| RoboLLM `experiment/harness-vlm-wam` | `f65f692bf60a8f88a312e63ff5d4e1589f2b05fa` | Selected parent of `experiment/nebius-recovery` |
| POC-RobotArm `main` | `39cc9b9` | Compared alternative; its code was not selected for import |

The full eligible interval is **26 August 2026 16:00 UTC through 30 October
2026 17:00 UTC**. The selected RoboLLM experiment includes earlier September
work inside that interval, but it was already present when this task started.
Do not describe that work as newly created on 21 October. Commit dates and
diffs are evidence of repository history; they do not establish registration
or acceptance by the organizer.

## Reused unchanged foundation

- `examples/humanoid_harness/simulation.py`: supported-pelvis MuJoCo scene,
  classical damped-least-squares IK, position servos, workspace/joint limits,
  measured state and rendering.
- `examples/humanoid_harness/models/concept01/`: original authored geometry,
  MJCF, metadata and earlier verification assets. Its physics are design
  estimates; fingers are fixed and robot self-collision is disabled.
- `examples/humanoid_harness/harness.py`, gallery/app and optional camera
  components: existing local interaction and target-adaptation code. Optional
  components are not automatically part of the new demo.
- Root Apache-2.0 license and existing model notices. No license change is
  required for the selected repository; preserve applicable notices.
- Existing VLA/WAM research files and September pilot reports remain separate
  experiments. They are not evidence that ReachGuard trains or runs those
  models.

## ReachGuard work started 21 October

Record the final paths and commit hashes after implementation. The scope is:

| Addition | Why it is substantive | Completion evidence |
| --- | --- | --- |
| NVIDIA/Nebius text supervisor adapter | Adds an external runtime decision path absent from the selected baseline | Live sanitized request/response and validated robotics outcome; pending account/run verification |
| Strict supervisor decision contract | Separates model advice from executable authority | Tests reject malformed or unpermitted decisions |
| Execution monitor and bounded recovery | Connects changed/invalid observations to explicit recovery or hold behavior | Scenario trace with fixed thresholds and retry/time budgets |
| Fixture provider | Makes local scenario development reproducible without cloud access | Clearly labeled offline tests; not evidence of required platform use |
| Measured scenario/evidence runner | Makes success and refusal behavior inspectable | Exact command, dependency versions, JSON trace, summary and rendered output |
| Submission documentation | Explains use case, scope, existing work and review path | These files, root README and final exact-revision links |

Implementation and tests may still be in progress while this manifest is
prepared. A file's presence does not establish a successful run. The final root
README and generated run evidence must carry the measured results.

## Final change inventory

Before publication, run from the repository root:

```bash
git diff --stat f65f692bf60a8f88a312e63ff5d4e1589f2b05fa...HEAD
git diff --name-status f65f692bf60a8f88a312e63ff5d4e1589f2b05fa...HEAD
git log --format='%H %cI %s' f65f692bf60a8f88a312e63ff5d4e1589f2b05fa..HEAD
git status --short
```

Those commands cover committed changes only; inspect the working tree before
concluding the inventory is complete. Add the final submission commit, live run
identifier, evidence hashes and video URL to the submission draft. Do not
include credentials, raw authorization headers, unrelated repository files or
unreviewed cloud logs in the public evidence.

## Suggested existing-work disclosure

“ReachGuard builds on my existing RoboLLM humanoid simulation, including the
original Concept 01 model, classical IK controller and local visualization.
For this entry I added a bounded NVIDIA-model supervisor through Nebius,
execution-monitoring and recovery scenarios, strict decision validation and
reproducible evidence. The referenced change manifest and final commit identify
the exact additions and their verification.”

Use that wording only after the corresponding implementation and live evidence
are complete. Until then describe those additions as in progress. Do not state
that the whole workbench or Concept 01 model was created for this task.
