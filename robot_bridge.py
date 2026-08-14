#!/usr/bin/env python3
"""Compatibility import for :mod:`robollm.bridge`.

New code should import ``robollm.bridge``. This alias keeps existing ROS tools
and third-party scripts working while the project migrates to the ``src``
layout.
"""

from __future__ import annotations

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from robollm import bridge as _implementation

sys.modules[__name__] = _implementation
