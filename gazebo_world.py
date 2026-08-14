#!/usr/bin/env python3
"""Compatibility import for :mod:`robollm.gazebo_world`."""

from __future__ import annotations

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from robollm import gazebo_world as _implementation

sys.modules[__name__] = _implementation
