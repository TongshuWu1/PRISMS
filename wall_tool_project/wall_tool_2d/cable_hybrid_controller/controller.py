#!/usr/bin/env python3
"""Chosen controller configuration and simulation runner.

This package is intentionally opinionated: it exposes one controller stack,
not a benchmark menu. The chosen stack is a nonlinear MPC that tracks the
tool-head path while optimizing side-motor thrust, cable tension, reel motion,
and payload attitude over a finite horizon. In the plant, cable tension is
realized by a speed-controlled reel with load-cell feedback rather than applied
directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from cable_hybrid_controller.config import (
    BEST_PATH_SPEED,
    BEST_PLANNER,
    CONTROLLER_OVERRIDES,
    COVERAGE_CORNER_SPEED,
    DEFAULT_SCENARIO_DURATION_S,
    FACADE_MISSION_OVERRIDES,
    MISSION_TRAJECTORY,
    WORK_PLANNER,
)
from cable_hybrid_controller.facade import FacadeMission, cleaning_targets, configure_skyscraper_params
from wall_tool_sim.wall_tool_ui import SimParams, SimState, Vec2, WallToolSimulator


@dataclass(frozen=True)
class ControllerScenario:
    name: str
    targets: tuple[Vec2, ...]
    duration_s: float
    description: str
    facade_mission: FacadeMission | None = None


def default_scenario() -> ControllerScenario:
    mission = FacadeMission(**FACADE_MISSION_OVERRIDES)
    return ControllerScenario(
        name=mission.name,
        targets=cleaning_targets(mission),
        duration_s=DEFAULT_SCENARIO_DURATION_S,
        description=mission.description,
        facade_mission=mission,
    )


def best_params(mission: FacadeMission | None = None) -> SimParams:
    params = configure_skyscraper_params(mission)
    return SimParams(
        **{
            **params.__dict__,
            **CONTROLLER_OVERRIDES,
        }
    )


def make_simulator(params: SimParams | None = None) -> WallToolSimulator:
    return WallToolSimulator(params or best_params())


def command_controller(simulator: WallToolSimulator, targets: Sequence[Vec2]) -> None:
    if MISSION_TRAJECTORY == "coverage-smooth":
        simulator.set_corner_smooth_path(targets, corner_speed=COVERAGE_CORNER_SPEED)
        return
    for target in targets:
        simulator.append_stop_target(target, planner=WORK_PLANNER)


def run_controller_session(
    scenario: ControllerScenario | None = None,
    params: SimParams | None = None,
) -> tuple[ControllerScenario, SimParams, list[SimState]]:
    active_scenario = scenario or default_scenario()
    active_params = params or best_params(active_scenario.facade_mission)
    simulator = make_simulator(active_params)
    command_controller(simulator, active_scenario.targets)

    states: list[SimState] = []
    settled_time = 0.0
    settle_required_s = 4.0
    for _ in range(max(1, int(active_scenario.duration_s / active_params.dt))):
        state = simulator.step()
        states.append(state)
        speed = (state.payload_velocity[0] ** 2 + state.payload_velocity[1] ** 2) ** 0.5
        if state.active_waypoints == 0 and state.tool_error < 0.10 and speed < 0.06:
            settled_time += active_params.dt
            if settled_time >= settle_required_s:
                break
        else:
            settled_time = 0.0
    return active_scenario, active_params, states
