#!/usr/bin/env python3
"""Executable entrypoint for the MCP application and bundled distribution."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
for _path in (_ROOT, _ROOT / "src"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from apps.mcp.server import mcp


if __name__ == "__main__":
    mcp.run(transport="stdio")
