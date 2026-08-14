# Tests

- `unit/` is the native, no-ROS fast gate and is the default `pytest` target.
- `integration/ros/` requires a sourced ROS 2 Jazzy environment or the project
  container image.
- `conftest.py` exposes canonical pure-Python example and hardware modules to
  the unit suite without copying their source.
