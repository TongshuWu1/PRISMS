"""Chosen cable-efficient wall-tool controller package.

Package-level exports are loaded lazily so importing a small helper submodule
does not pull in the full simulator/controller stack.
"""

from __future__ import annotations

__all__ = [
    "BEST_PATH_SPEED",
    "BEST_PLANNER",
    "COVERAGE_CORNER_SPEED",
    "MISSION_TRAJECTORY",
    "WORK_PLANNER",
    "ControllerScenario",
    "best_params",
    "default_scenario",
]


def __getattr__(name: str):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from cable_hybrid_controller import controller

    return getattr(controller, name)
