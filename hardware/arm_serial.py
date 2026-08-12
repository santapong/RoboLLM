#!/usr/bin/env python3
"""Compatibility CLI for the canonical ROS 2 arm-driver package."""
from pathlib import Path
import sys

_PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "ros2" / "robo_arm_driver"
sys.path.insert(0, str(_PACKAGE_ROOT))

from robo_arm_driver.arm_serial import *  # noqa: F401,F403,E402
from robo_arm_driver.arm_serial import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
