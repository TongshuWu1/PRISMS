#!/usr/bin/env python3
"""Create or validate a CoppeliaSim independent-plant calibration profile."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
for path in (ROOT, ROOT / "wall_tool_2d", ROOT / "wall_tool_3d"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from coppeliasim_vector_tool.calibration_cli import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
