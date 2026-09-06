#!/usr/bin/env python3
"""Source-checkout entry point; never opens hardware implicitly."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "ros2" / "robo_arm_driver")]

if __name__ == "__main__":
    from robollm.elexhub.cli import main
    raise SystemExit(main())
