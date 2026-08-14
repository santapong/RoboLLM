"""The numpy 1.26.4 law.

ROS 2 Jazzy is built against numpy 1.26.x; numpy 2.x breaks rclpy message
conversions. requirements/ros-constraints.txt MUST keep the exact pin, and any numpy that happens
to be importable in the running environment must satisfy 1.26.x.
"""
import os
import re

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_CONSTRAINTS = os.path.join(_ROOT, "requirements", "ros-constraints.txt")


def test_constraints_pins_numpy_1_26_4():
    with open(_CONSTRAINTS, encoding="utf-8") as f:
        text = f.read()
    assert re.search(r"(?m)^\s*numpy==1\.26\.4\s*$", text), (
        "requirements/ros-constraints.txt must pin numpy==1.26.4 (ROS 2 Jazzy ABI law)"
    )


def test_numpy_runtime_matches_pin_if_present():
    try:
        import numpy
    except ImportError:
        pytest.skip("numpy not importable here; the pin-only check covers it")
    assert numpy.__version__.startswith("1.26"), (
        "installed numpy %s violates the 1.26.x law" % numpy.__version__
    )
