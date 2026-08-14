"""Collect the 14 vendored gesture-SM unit tests through the unit test run.

The real tests live at
examples/gen3_pick_place/ros2_ws/src/gen3_pick_place/test/test_gesture_sm.py
and are pure-python (no rclpy). Rather than copy them here — which would let the
two sets silently drift — we re-export the actual test function objects from the
vendored module. conftest.py puts that module's directory on sys.path; pytest
then collects the same ``test_*`` callables in this module, so each runs exactly
once with no duplication.
"""
from test_gesture_sm import *  # noqa: F401,F403
