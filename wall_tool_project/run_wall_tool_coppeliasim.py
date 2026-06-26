#!/usr/bin/env python3
"""PyCharm-friendly entry point for the CoppeliaSim wall-tool scene."""

from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
WALL_TOOL_2D_ROOT = PROJECT_ROOT / "wall_tool_2d"
WALL_TOOL_3D_ROOT = PROJECT_ROOT / "wall_tool_3d"
for path in (WALL_TOOL_3D_ROOT, WALL_TOOL_2D_ROOT, PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from coppeliasim_wall_tool.run_wall_tool_scene import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
