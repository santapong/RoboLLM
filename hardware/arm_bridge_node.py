#!/usr/bin/env python3
"""Compatibility launcher for ``ros2 run robo_arm_driver arm_bridge``."""
from pathlib import Path
import sys

_PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "ros2" / "robo_arm_driver"
sys.path.insert(0, str(_PACKAGE_ROOT))

from robo_arm_driver.node import ArmBridge, main  # noqa: E402,F401


if __name__ == "__main__":
    main()
