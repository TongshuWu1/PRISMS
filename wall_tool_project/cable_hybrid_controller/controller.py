#!/usr/bin/env python3
"""Chosen controller configuration and simulation runner.

This package is intentionally opinionated: it exposes one controller stack,
not a benchmark menu. The chosen stack is a mixed-input energy-shaping
controller: reel velocity regulates radial cable geometry, drone acceleration
regulates tangential tracking and swing energy, and the reference governor
keeps facade contact valid.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from cable_hybrid_controller.facade import FacadeMission, cleaning_targets, configure_skyscraper_params
from wall_tool_sim.wall_tool_ui import PLANNER_DIRECT, PLANNER_PREDICTIVE, SimParams, SimState, Vec2, WallToolSimulator


BEST_PLANNER = PLANNER_PREDICTIVE
WORK_PLANNER = PLANNER_DIRECT
BEST_PATH_SPEED = 0.30
COVERAGE_CORNER_SPEED = 0.075
MISSION_TRAJECTORY = "coverage-smooth"


@dataclass(frozen=True)
class ControllerScenario:
    name: str
    targets: tuple[Vec2, ...]
    duration_s: float
    description: str
    facade_mission: FacadeMission | None = None


def default_scenario() -> ControllerScenario:
    mission = FacadeMission()
    return ControllerScenario(
        name=mission.name,
        targets=cleaning_targets(mission),
        duration_s=430.0,
        description=mission.description,
        facade_mission=mission,
    )


def best_params(mission: FacadeMission | None = None) -> SimParams:
    params = configure_skyscraper_params(mission)
    return SimParams(
        **{
            **params.__dict__,
            "control_law": "miesc",
            "path_speed": BEST_PATH_SPEED,
            "reference_accel_limit_mps2": 0.40,
            "reference_jerk_limit_mps3": 2.1,
            "reference_min_segment_duration_s": 0.65,
            "reference_speed_min": 0.15,
            "contact_governor_turn_min_scale": 0.18,
            "contact_governor_geometry_efficiency": 0.62,
            "contact_governor_geometry_min_scale": 0.28,
            "contact_governor_tracking_ratio": 0.42,
            "contact_governor_tracking_min_scale": 0.16,
            "spool_accel_limit_mps2": 0.38,
            "miesc_spool_accel_limit_mps2": 0.38,
            "miesc_tangential_frequency_rad_s": 3.00,
            "miesc_tangential_damping_ratio": 0.90,
            "miesc_clf_decay_rate": 2.35,
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
