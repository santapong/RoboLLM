#!/usr/bin/env python3
"""Compatibility launcher for :mod:`apps.mcp.server`."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
_SRC = _ROOT / "src"
for _path in (_ROOT, _SRC):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from apps.mcp.entrypoint import mcp


if __name__ == "__main__":
    mcp.run(transport="stdio")
