# RoboLLM · Documentation hub

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../README.md) · [Roadmap](../ROADMAP.md) · [Architecture](ARCHITECTURE.md) · [Style guide](STYLE_GUIDE.md)

Use this page as the repository’s table of contents. Start with a README to
run something, a TECHNICAL page to understand it, and a runbook when you need
repeatable operations or acceptance evidence.

| Doc | One line |
|-----|----------|
| [`../CHANGELOG.md`](../CHANGELOG.md) | Project changelog — notable changes by date. |
| [`PROJECT_STATUS.md`](PROJECT_STATUS.md) | Current checkpoint — B1 preparation evidence, GPU pause, and unchanged physical gates. |
| [`PROJECT_STRUCTURE.md`](PROJECT_STRUCTURE.md) | Canonical directory ownership, placement rules, and compatibility paths. |
| [`STYLE_GUIDE.md`](STYLE_GUIDE.md) | Theme, status vocabulary, navigation, and evidence conventions. |
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | The C4 tour of the workbench: context → containers → components, diagrams + narrative. |
| [`architecture/c4-context.svg`](architecture/c4-context.svg) | C4 level 1 — humans, language-model clients, ROS 2/Gazebo, and the real arm. |
| [`architecture/c4-container.svg`](architecture/c4-container.svg) | C4 level 2 — the runnable pieces and the one shared `robollm.bridge` node. |
| [`architecture/c4-component-handteleop.svg`](architecture/c4-component-handteleop.svg) | C4 level 3 — inside the hand-teleop pipeline (MediaPipe → IK → 20 Hz trajectory). |
| [`live-session.md`](live-session.md) | Guided ~30 min demo playbook: observe, spawn, drive, map, navigate, and move an arm through MCP. |
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
| [`../examples/humanoid_mirror/README.md`](../examples/humanoid_mirror/README.md) | CPU-only webcam body mirroring on the FFW humanoid model. |
| [`../examples/humanoid_mirror/docs/mirror-run.md`](../examples/humanoid_mirror/docs/mirror-run.md) | Humanoid Docker/native runbook and acceptance signals. |
| [`../examples/wall_weld/README.md`](../examples/wall_weld/README.md) | Gesture-triggered autonomous raster planning in RViz; no welding hardware. |
| [`../examples/wall_weld/docs/wallweld-run.md`](../examples/wall_weld/docs/wallweld-run.md) | Wall-weld synthetic/full/abort/idle runbook. |
| [`../examples/talos_mirror/docs/mirror-run.md`](../examples/talos_mirror/docs/mirror-run.md) | TALOS live-mirroring bringup and acceptance runbook. |
| [`../examples/talos_mirror/docs/joint-inventory.md`](../examples/talos_mirror/docs/joint-inventory.md) | Joint-name and control-group inventory used by TALOS retargeting. |

## Technical deep-dives (per module)

| Doc | One line |
|-----|----------|
| [`../examples/ros2_py/TECHNICAL.md`](../examples/ros2_py/TECHNICAL.md) | The 10 rclpy lessons: per-script internals, topic tables, run + verify. |
| [`../examples/colcon_pkg/patrol_bot/TECHNICAL.md`](../examples/colcon_pkg/patrol_bot/TECHNICAL.md) | The first real ament_python package: anatomy, params, launch plumbing. |
| [`../examples/pybullet/TECHNICAL.md`](../examples/pybullet/TECHNICAL.md) | PyBullet quickstart + closed-form 2-link IK on the CAD arm. |
| [`../examples/mujoco/TECHNICAL.md`](../examples/mujoco/TECHNICAL.md) | MuJoCo pendulum plus scripted 6-DOF LeRobot dataset recording. |
| [`../examples/mujoco/B1.md`](../examples/mujoco/B1.md) | Red-target benchmark, dataset/evaluator interfaces, CPU evidence, and guarded GPU handoff. |
| [`../examples/panda_arm/TECHNICAL.md`](../examples/panda_arm/TECHNICAL.md) | The five Panda demos layer by layer: FK/IK → serial → vision sort. |
| [`../examples/hand_follow/TECHNICAL.md`](../examples/hand_follow/TECHNICAL.md) | hand_follow node internals: MediaPipe → One-Euro → IK → 20 Hz stream. |
| [`../examples/gen3_pick_place/TECHNICAL.md`](../examples/gen3_pick_place/TECHNICAL.md) | Gesture state machine, `/compute_ik` streaming, planning-scene attach/detach. |
| [`../examples/humanoid_mirror/TECHNICAL.md`](../examples/humanoid_mirror/TECHNICAL.md) | FFW geometry, tracking, retargeting, measured traps, and acceptance evidence. |
| [`../examples/talos_mirror/TECHNICAL.md`](../examples/talos_mirror/TECHNICAL.md) | TALOS model, QP/classical retargeting, limits, and test harnesses. |
| [`../examples/wall_weld/TECHNICAL.md`](../examples/wall_weld/TECHNICAL.md) | Fist-triggered autonomous wall welding: raster planner, marker tracking, weld state machine. |
| [`../hardware/TECHNICAL.md`](../hardware/TECHNICAL.md) | The real Mega arm stack: firmware, serial protocol, driver, ROS 2 bridge. |
| [`physical-arm/ARCHITECTURE.md`](physical-arm/ARCHITECTURE.md) | Physical-arm safety boundary, ROS interfaces, and configuration ownership. |
| [`physical-arm/HARDWARE_WORKSHEET.md`](physical-arm/HARDWARE_WORKSHEET.md) | Phase 0 bench measurements and commissioning procedure. |
| [`physical-arm/ROADMAP.md`](physical-arm/ROADMAP.md) | Honest Phase 0–5 status plus v0.1→v1.0 evidence gates. |
| [`physical-arm/diagrams/c4-context.svg`](physical-arm/diagrams/c4-context.svg) | Physical arm C4 level 1 — people and external systems. |
| [`physical-arm/diagrams/c4-container.svg`](physical-arm/diagrams/c4-container.svg) | Physical arm C4 level 2 — delivered, partial, and planned runtime containers. |
| [`physical-arm/diagrams/c4-component-driver.svg`](physical-arm/diagrams/c4-component-driver.svg) | Physical arm C4 level 3 — validated driver components. |
| [`physical-arm/diagrams/architecture-4plus1.svg`](physical-arm/diagrams/architecture-4plus1.svg) | Logical, process, development, physical, and scenario (+1) views. |
| [`../apps/dashboard/TECHNICAL.md`](../apps/dashboard/TECHNICAL.md) | The dashboard internals: FastAPI + WebSocket over the shared bridge, endpoint tables. |
| [`../scan3d/TECHNICAL.md`](../scan3d/TECHNICAL.md) | Webcam scanner internals: visual hull vs COLMAP routes, mesh → URDF tail. |
| [`../sim/vla-bed/SDD.md`](../sim/vla-bed/SDD.md) | UR5e VLA sim bed specification: Pi-hosted MuJoCo, mjviser viewer, OXE-aligned actions, phase gates, limits. |
| [`../sim/vla-bed/NOTICES.md`](../sim/vla-bed/NOTICES.md) | Third-party licenses, pins, and attribution obligations for the sim bed (with `REFERENCES.md` BibTeX). |
| [`../cad/TECHNICAL.md`](../cad/TECHNICAL.md) | FreeCAD → URDF pipeline: headless build, joint-frame discipline, PyBullet verify. |
| [`../examples/ros2_py/docs/ros2_py-architecture.svg`](../examples/ros2_py/docs/ros2_py-architecture.svg) | Diagram — the 10 scripts vs the sim's topic/action surfaces. |
| [`../examples/colcon_pkg/patrol_bot/docs/patrol_bot-architecture.svg`](../examples/colcon_pkg/patrol_bot/docs/patrol_bot-architecture.svg) | Diagram — package build → install → launch → node flow. |
| [`../examples/pybullet/docs/pybullet-architecture.svg`](../examples/pybullet/docs/pybullet-architecture.svg) | Diagram — IK math → PyBullet verification loop. |
| [`../examples/mujoco/docs/mujoco-architecture.svg`](../examples/mujoco/docs/mujoco-architecture.svg) | Diagram — pendulum lesson plus scripted arm → LeRobot v3 dataset. |
| [`../examples/panda_arm/docs/panda_arm-architecture.svg`](../examples/panda_arm/docs/panda_arm-architecture.svg) | Diagram — the five-demo manipulation pipeline. |
| [`../examples/humanoid_mirror/docs/humanoid_mirror-architecture.svg`](../examples/humanoid_mirror/docs/humanoid_mirror-architecture.svg) | Diagram — webcam pose through filtering and retargeting to FFW controllers. |
| [`../examples/talos_mirror/docs/talos_mirror-architecture.svg`](../examples/talos_mirror/docs/talos_mirror-architecture.svg) | Diagram — whole-body TALOS tracking, retargeting, control, and RViz path. |
| [`../examples/wall_weld/docs/wall_weld-architecture.svg`](../examples/wall_weld/docs/wall_weld-architecture.svg) | Diagram — gesture trigger, raster planner, state machine, MoveIt, and RViz. |
| [`../hardware/docs/hardware-architecture.svg`](../hardware/docs/hardware-architecture.svg) | Diagram — ROS 2 bridge → serial protocol → Mega → servos. |
| [`../apps/dashboard/docs/web-architecture.svg`](../apps/dashboard/docs/web-architecture.svg) | Diagram — browser ↔ FastAPI ↔ shared bridge ↔ ROS 2 stack. |
| [`../scan3d/docs/scan3d-architecture.svg`](../scan3d/docs/scan3d-architecture.svg) | Diagram — capture → visual hull / COLMAP → mesh → URDF. |
| [`../sim/vla-bed/docs/vla-bed-topology.svg`](../sim/vla-bed/docs/vla-bed-topology.svg) | Diagram — Pi sim server (MuJoCo, mink, recorder, mjviser, ZeroMQ) ↔ workstation client (browser, policy, GPU gate). |
| [`../cad/docs/cad-architecture.svg`](../cad/docs/cad-architecture.svg) | Diagram — freecadcmd build → mesh export → URDF → PyBullet check. |
| [`brand/robollm-banner.svg`](brand/robollm-banner.svg) | Project banner and control-invariant identity. |

Module READMEs live next to their code: [`../cad/`](../cad/README.md)
(FreeCAD → URDF), [`../scan3d/`](../scan3d/README.md) (webcam → mesh → URDF),
[`../hardware/`](../hardware/README.md) (the real Mega arm),
[`../mcpb/`](../mcpb/README.md) (Claude Desktop bundle).

## Module and operations guides

| Doc | One line |
|---|---|
| [`../README.md`](../README.md) | Project landing page, delivery status, and shortest paths. |
| [`../ROADMAP.md`](../ROADMAP.md) | Learning/research priorities and graduation rules. |
| [`../CLAUDE.md`](../CLAUDE.md) | Public maintainer/agent context and repository invariants. |
| [`../cad/README.md`](../cad/README.md) | Run the FreeCAD → URDF → PyBullet reference pipeline. |
| [`../scan3d/README.md`](../scan3d/README.md) | Capture and reconstruct an object using the available scan routes. |
| [`../sim/vla-bed/README.md`](../sim/vla-bed/README.md) | UR5e VLA sim bed: what it is, why not OmniSim on the Pi, phase gates at a glance. |
| [`../hardware/README.md`](../hardware/README.md) | Commission the DIY arm without bypassing calibration locks. |
| [`../hardware/docs/serial_protocol.md`](../hardware/docs/serial_protocol.md) | arm-fw 2.1 wire contract and error behavior. |
| [`../hardware/docs/phaseA-convergence.md`](../hardware/docs/phaseA-convergence.md) | Historical convergence decisions and remaining research gates. |
| [`../ros2/README.md`](../ros2/README.md) | Physical-arm ROS 2 package status and boundaries. |
| [`../mcpb/README.md`](../mcpb/README.md) | Claude Code registration and Claude Desktop bundle packaging. |
