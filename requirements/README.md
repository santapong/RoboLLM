# Dependency environments

- `ros.txt` is the main ROS-compatible application environment.
- `ros-constraints.txt` enforces the ROS 2 Jazzy NumPy ABI boundary.
- `lerobot.txt` is the isolated CPU dataset environment.
- `smolvla.txt` is the isolated, deferred GPU training environment.

Never combine the ROS and LeRobot/SmolVLA environments: they intentionally use
different NumPy major versions. Root-level requirement files are compatibility
includes for existing commands.
