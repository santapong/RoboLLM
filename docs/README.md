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

## Technical deep-dives (per module)

| Doc | One line |
|-----|----------|
| [`../examples/ros2_py/TECHNICAL.md`](../examples/ros2_py/TECHNICAL.md) | The 10 rclpy lessons: per-script internals, topic tables, run + verify. |
| [`../examples/colcon_pkg/patrol_bot/TECHNICAL.md`](../examples/colcon_pkg/patrol_bot/TECHNICAL.md) | The first real ament_python package: anatomy, params, launch plumbing. |
| [`../examples/pybullet/TECHNICAL.md`](../examples/pybullet/TECHNICAL.md) | PyBullet quickstart + closed-form 2-link IK on the CAD arm. |
| [`../examples/mujoco/TECHNICAL.md`](../examples/mujoco/TECHNICAL.md) | MuJoCo pendulum: inline MJCF, headless stepping, the viewer. |
| [`../examples/panda_arm/TECHNICAL.md`](../examples/panda_arm/TECHNICAL.md) | The five Panda demos layer by layer: FK/IK → serial → vision sort. |
| [`../examples/hand_follow/TECHNICAL.md`](../examples/hand_follow/TECHNICAL.md) | hand_follow node internals: MediaPipe → One-Euro → IK → 20 Hz stream. |
| [`../examples/gen3_pick_place/TECHNICAL.md`](../examples/gen3_pick_place/TECHNICAL.md) | Gesture state machine, `/compute_ik` streaming, planning-scene attach/detach. |
| [`../hardware/TECHNICAL.md`](../hardware/TECHNICAL.md) | The real Uno arm stack: firmware, serial protocol, driver, ROS 2 bridge. |
| [`../examples/ros2_py/docs/ros2_py-architecture.svg`](../examples/ros2_py/docs/ros2_py-architecture.svg) | Diagram — the 10 scripts vs the sim's topic/action surfaces. |
| [`../examples/colcon_pkg/patrol_bot/docs/patrol_bot-architecture.svg`](../examples/colcon_pkg/patrol_bot/docs/patrol_bot-architecture.svg) | Diagram — package build → install → launch → node flow. |
| [`../examples/pybullet/docs/pybullet-architecture.svg`](../examples/pybullet/docs/pybullet-architecture.svg) | Diagram — IK math → PyBullet verification loop. |
| [`../examples/mujoco/docs/mujoco-architecture.svg`](../examples/mujoco/docs/mujoco-architecture.svg) | Diagram — MJCF string → model/data → step loop. |
| [`../examples/panda_arm/docs/panda_arm-architecture.svg`](../examples/panda_arm/docs/panda_arm-architecture.svg) | Diagram — the five-demo manipulation pipeline. |
| [`../hardware/docs/hardware-architecture.svg`](../hardware/docs/hardware-architecture.svg) | Diagram — ROS 2 bridge → serial protocol → Uno → servos. |

Module READMEs live next to their code: [`../cad/`](../cad/README.md)
(FreeCAD → URDF), [`../scan3d/`](../scan3d/README.md) (webcam → mesh → URDF),
[`../hardware/`](../hardware/README.md) (the real Uno arm),
[`../mcpb/`](../mcpb/README.md) (Claude Desktop bundle).
