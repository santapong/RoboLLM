# Project structure and ownership

RoboLLM uses a domain-oriented layout with a standard Python `src` boundary.
The goal is to make ownership obvious without moving ROS packages or runnable
examples away from the paths their build systems expect.

```text
RoboLLM/
├── apps/                    # runnable MCP and dashboard applications
├── src/robollm/             # reusable Python runtime code
├── configs/                 # checked experiment/runtime configuration
├── requirements/            # isolated dependency environments
├── scripts/                 # launch, CI, learning, and setup operations
├── tests/
│   ├── unit/                # default native pytest suite
│   └── integration/ros/     # ROS/container-bound tests
├── ros2/                    # first-party ROS 2 packages
├── examples/                # self-contained learning and research stacks
├── hardware/                # firmware, serial simulation, and bench tools
├── cad/                     # CAD-to-URDF pipeline
├── scan3d/                  # image-to-mesh pipeline
├── docs/                    # project-wide architecture and runbooks
├── assets/                  # small checked assets; large captures ignored
└── mcpb/                    # distributable MCP bundle tooling
```

## Placement rules

1. Shared importable Python belongs in `src/robollm/`; runnable surfaces only
   adapt that code under `apps/`.
2. ROS packages remain under `ros2/`, and self-contained example workspaces
   remain under `examples/`. Their package-relative paths are build contracts.
3. Human-invoked automation belongs in a named `scripts/` category. Generated
   output goes to ignored `artifacts/`, `datasets/`, or `checkpoints/` paths.
4. Dependency files live in `requirements/`, separated at environment
   boundaries. ROS NumPy 1.26 and LeRobot NumPy 2 must never be mixed.
5. Fast tests live in `tests/unit`; ROS graph/device tests live in
   `tests/integration/ros` and never enter the default test collection.
6. Repository-wide documentation lives in `docs/`; module-specific README and
   TECHNICAL files stay beside the code they explain.

## Compatibility paths

The root `run-server.sh` and `launch_all.sh`, the `sim/launch_*.sh` commands,
and `web/run-web.sh` remain thin forwarding launchers. Root Python modules and
requirement files similarly forward to canonical locations. They protect saved
commands and downstream scripts during migration; new references must use the
canonical paths above.
