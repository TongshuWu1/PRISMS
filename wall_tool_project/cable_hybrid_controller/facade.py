#!/usr/bin/env python3
"""Facade mission model for skyscraper cleaning/inspection studies."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from wall_tool_sim.wall_tool_ui import SimParams, SimState, Vec2


@dataclass(frozen=True)
class FacadeMission:
    name: str = "skyscraper_facade_cleaning"
    x_min: float = -2.10
    x_max: float = 2.10
    z_min: float = 1.10
    z_max: float = 5.25
    lane_spacing_m: float = 0.42
    tool_width_m: float = 0.34
    desired_contact_force_N: float = 0.55
    min_contact_force_N: float = 0.25
    max_contact_force_N: float = 0.95
    inspection_standoff_m: float = 0.10
    coverage_cell_m: float = 0.10
    max_cleaning_speed_m_s: float = 0.36
    max_tracking_error_m: float = 0.12
    max_angular_rate_rad_s: float = 1.5
    description: str = (
        "Serpentine skyscraper facade cleaning/inspection pass with cable safety, "
        "2.5D normal contact dynamics, and contact-gated coverage tracking."
    )


def cleaning_targets(mission: FacadeMission) -> tuple[Vec2, ...]:
    targets: list[Vec2] = []
    lane_z_values: list[float] = []
    z = mission.z_min
    while z <= mission.z_max + 1e-9:
        lane_z_values.append(z)
        z += mission.lane_spacing_m
    if not lane_z_values or abs(lane_z_values[-1] - mission.z_max) > 1e-9:
        lane_z_values.append(mission.z_max)

    left_to_right = True
    for z in lane_z_values:
        if left_to_right:
            targets.append((mission.x_min, z))
            targets.append((mission.x_max, z))
        else:
            targets.append((mission.x_max, z))
            targets.append((mission.x_min, z))
        left_to_right = not left_to_right
    targets.append((0.0, min(mission.z_max, max(mission.z_min, 2.0))))
    return tuple(targets)


def configure_skyscraper_params(mission: FacadeMission | None = None) -> SimParams:
    active_mission = mission or FacadeMission()
    nominal_params = SimParams()
    initial_contact_gap = -active_mission.desired_contact_force_N / max(
        nominal_params.normal_contact_stiffness_N_m,
        1e-9,
    )
    return SimParams(
        path_speed=0.25,
        wind_enabled=False,
        wind_force_x=0.0,
        wind_force_z=0.0,
        wind_gust_force=0.0,
        edge_wind_gain=0.0,
        normal_contact_enabled=True,
        normal_initial_gap_m=initial_contact_gap,
        normal_standoff_m=active_mission.inspection_standoff_m,
        desired_contact_force_N=active_mission.desired_contact_force_N,
        min_contact_force_N=active_mission.min_contact_force_N,
        max_contact_force_N=active_mission.max_contact_force_N,
        contact_work_enabled=True,
        contact_work_x_min=active_mission.x_min,
        contact_work_x_max=active_mission.x_max,
        contact_work_z_min=active_mission.z_min,
        contact_work_z_max=active_mission.z_max,
        work_contact_speed_limit_mps=active_mission.max_cleaning_speed_m_s,
        work_contact_tracking_limit_m=active_mission.max_tracking_error_m,
        work_contact_angular_rate_limit_rad_s=active_mission.max_angular_rate_rad_s,
        normal_wind_force_N=0.0,
        normal_wind_gust_force_N=0.0,
    )


def in_work_region(point: Vec2, mission: FacadeMission, margin_m: float = 0.0) -> bool:
    return (
        mission.x_min - margin_m <= point[0] <= mission.x_max + margin_m
        and mission.z_min - margin_m <= point[1] <= mission.z_max + margin_m
    )


def valid_work_contact(state: SimState, mission: FacadeMission) -> bool:
    speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
    footprint_margin = 0.5 * mission.tool_width_m
    return (
        in_work_region(state.tool_head, mission, footprint_margin)
        and mission.min_contact_force_N <= state.contact_force <= mission.max_contact_force_N
        and state.tool_error <= mission.max_tracking_error_m
        and speed <= mission.max_cleaning_speed_m_s
        and abs(state.angular_velocity) <= mission.max_angular_rate_rad_s
    )


def coverage_fraction(states: Sequence[SimState], mission: FacadeMission) -> float:
    cols = max(1, int(math.ceil((mission.x_max - mission.x_min) / mission.coverage_cell_m)))
    rows = max(1, int(math.ceil((mission.z_max - mission.z_min) / mission.coverage_cell_m)))
    covered: set[tuple[int, int]] = set()
    footprint_radius = 0.5 * mission.tool_width_m
    cell_radius = max(0, int(math.ceil(footprint_radius / mission.coverage_cell_m)))

    for state in states:
        x, z = state.tool_head
        if not valid_work_contact(state, mission):
            continue
        col = int((x - mission.x_min) / mission.coverage_cell_m)
        row = int((z - mission.z_min) / mission.coverage_cell_m)
        for dc in range(-cell_radius, cell_radius + 1):
            for dr in range(-cell_radius, cell_radius + 1):
                cc = col + dc
                rr = row + dr
                if 0 <= cc < cols and 0 <= rr < rows:
                    covered.add((cc, rr))
    return len(covered) / max(1, cols * rows)


def contact_quality(state: SimState, params: SimParams, mission: FacadeMission) -> float:
    _ = params
    if not in_work_region(state.tool_head, mission, 0.5 * mission.tool_width_m):
        return 0.0
    contact = state.contact_force
    if contact < mission.min_contact_force_N:
        return contact / max(mission.min_contact_force_N, 1e-9)
    if contact > mission.max_contact_force_N:
        excess = contact - mission.max_contact_force_N
        return max(0.0, 1.0 - excess / max(mission.max_contact_force_N, 1e-9))
    return 1.0


def blur_risk(state: SimState) -> float:
    speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
    angular_speed = abs(state.angular_velocity)
    return min(1.0, speed / 0.35 + angular_speed / 5.0)


def facade_safety_margin(state: SimState, params: SimParams) -> float:
    thrust_margin = 1.0 - max(state.left_thrust, state.right_thrust) / max(params.max_thrust_per_drone, 1e-9)
    tension_margin = min(
        state.tension / max(params.min_tracking_tension, 1e-9) - 1.0,
        params.max_spool_tension / max(state.tension, 1e-9) - 1.0,
    )
    wind_ratio = math.hypot(state.wind_force[0], state.wind_force[1]) / max(params.total_mass * params.gravity, 1e-9)
    if state.work_mode:
        contact_low_margin = state.contact_force / max(params.min_contact_force_N, 1e-9) - 1.0
        contact_high_margin = params.max_contact_force_N / max(state.contact_force, 1e-9) - 1.0
        contact_margin = min(contact_low_margin, contact_high_margin)
    else:
        contact_margin = 1.0
    return min(thrust_margin, tension_margin, 1.0 - wind_ratio, contact_margin)
