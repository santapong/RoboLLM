"""pytest bootstrap for the native (no-ROS) fast gate.

Two sys.path shims, no copied code:

1. The 14 gesture-state-machine unit tests already live, vendored, at
   examples/gen3_pick_place/ros2_ws/src/gen3_pick_place/test/test_gesture_sm.py
   (pure python, no rclpy). We put that directory on sys.path so the thin
   re-export module ``test_gesture_sm_suite.py`` in this folder can pull in the
   *same* test functions instead of duplicating their source — they collect and
   run exactly once, and the two copies can never drift.

2. The wall_weld ``arm_ik.py`` (pure math) is made importable for the hypothesis
   property suite.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)

# Shim 1 — vendored gesture-SM test directory.
_GESTURE_TEST_DIR = os.path.join(
    _ROOT, "examples", "gen3_pick_place", "ros2_ws", "src",
    "gen3_pick_place", "test",
)
if _GESTURE_TEST_DIR not in sys.path:
    sys.path.insert(0, _GESTURE_TEST_DIR)

# Shim 2 — canonical (wall_weld) arm_ik tools directory.
_ARM_IK_DIR = os.path.join(
    _ROOT, "examples", "wall_weld", "ros2_ws", "tools",
)
if _ARM_IK_DIR not in sys.path:
    sys.path.insert(0, _ARM_IK_DIR)

# Shim 3 — canonical physical-arm ROS package (pure config/safety modules).
_ROBO_ARM_DRIVER_DIR = os.path.join(_ROOT, "ros2", "robo_arm_driver")
if _ROBO_ARM_DRIVER_DIR not in sys.path:
    sys.path.insert(0, _ROBO_ARM_DRIVER_DIR)
