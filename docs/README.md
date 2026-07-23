# docs — index

| Doc | One line |
|-----|----------|
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | The C4 tour of the workbench: context → containers → components, diagrams + narrative. |
| [`architecture/c4-context.svg`](architecture/c4-context.svg) | C4 level 1 — who uses what: you, Claude, the ROS 2/Gazebo stack, the real arm. |
| [`architecture/c4-container.svg`](architecture/c4-container.svg) | C4 level 2 — the runnable pieces and the one shared `robot_bridge.py` node. |
| [`architecture/c4-component-handteleop.svg`](architecture/c4-component-handteleop.svg) | C4 level 3 — inside the hand-teleop pipeline (MediaPipe → IK → 20 Hz trajectory). |
| [`live-session.md`](live-session.md) | Guided ~30 min demo playbook: Claude sees, spawns, drives, maps, navigates, moves an arm. |
| [`branching.md`](branching.md) | Branch tiers: `main` (stable) / `develop` (daily) / `experiment/*` (throwaway). |

## Per-example docs

| Doc | One line |
|-----|----------|
| [`../examples/README.md`](../examples/README.md) | The learning path: ROS 2 fundamentals → Nav2/MoveIt → physics sims → hand teleop. |
| [`../examples/hand_follow/README.md`](../examples/hand_follow/README.md) | Webcam LEFT-hand teleop of a 6-DOF arm — run it, tune it, verified numbers. |
| [`../examples/hand_follow/docs/handfollow-inception.md`](../examples/hand_follow/docs/handfollow-inception.md) | Theory: the ROS 2 × LLM landscape + the hand-following teleop pipeline research. |
| [`../examples/hand_follow/docs/handfollow-run.md`](../examples/hand_follow/docs/handfollow-run.md) | hand_follow runbook: launch args, knobs, latency probe. |
| [`../examples/gen3_pick_place/README.md`](../examples/gen3_pick_place/README.md) | Gesture pick-and-place on a Kinova Gen3 lite: fist = grip, open palm = release. |
| [`../examples/gen3_pick_place/docs/pickplace-theory.md`](../examples/gen3_pick_place/docs/pickplace-theory.md) | Theory: hand-guided pick-and-place design rationale (gestures, palm orientation, IK). |
| [`../examples/gen3_pick_place/docs/pickplace-run.md`](../examples/gen3_pick_place/docs/pickplace-run.md) | gen3_pick_place runbook: Docker/native bring-up and overrides. |
| [`../examples/panda_arm/README.md`](../examples/panda_arm/README.md) | 7-DOF Panda series: FK/IK → serial → virtual Arduino → vision pick & place. |

Module READMEs live next to their code: [`../cad/`](../cad/README.md)
(FreeCAD → URDF), [`../scan3d/`](../scan3d/README.md) (webcam → mesh → URDF),
[`../hardware/`](../hardware/README.md) (the real Uno arm),
[`../mcpb/`](../mcpb/README.md) (Claude Desktop bundle).
