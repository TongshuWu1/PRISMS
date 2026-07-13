#!/usr/bin/env python3
"""Facade mission model for non-contact inspection studies."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from wall_tool_sim.wall_tool_ui import SimParams, SimState, Vec2


@dataclass(frozen=True)
class FacadeMission:
    name: str = "single_tether_facade_inspection"
    x_min: float = -2.10
    x_max: float = 2.10
    z_min: float = 1.10
    z_max: float = 5.25
    lane_spacing_m: float = 0.42
    sensor_footprint_m: float = 0.34
    inspection_standoff_m: float = 0.10
    coverage_cell_m: float = 0.10
    max_inspection_speed_m_s: float = 0.30
    max_tracking_error_m: float = 0.08
    max_attitude_error_rad: float = math.radians(8.0)
    max_angular_rate_rad_s: float = 0.60
    description: str = (
        "Serpentine non-contact facade inspection pass with one active cable, "
        "two independently vectorable propellers, and trajectory-quality-gated coverage."
    )


def inspection_targets(mission: FacadeMission) -> tuple[Vec2, ...]:
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
    return SimParams(
        path_speed=0.25,
        wind_enabled=False,
        wind_force_x=0.0,
        wind_force_z=0.0,
        wind_gust_force=0.0,
        edge_wind_gain=0.0,
        normal_contact_enabled=False,
        normal_initial_gap_m=active_mission.inspection_standoff_m,
        normal_standoff_m=active_mission.inspection_standoff_m,
        desired_contact_force_N=0.0,
        min_contact_force_N=0.0,
        max_contact_force_N=0.0,
        contact_work_enabled=False,
        contact_work_x_min=active_mission.x_min,
        contact_work_x_max=active_mission.x_max,
        contact_work_z_min=active_mission.z_min,
        contact_work_z_max=active_mission.z_max,
        work_contact_speed_limit_mps=active_mission.max_inspection_speed_m_s,
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


def valid_inspection_sample(state: SimState, mission: FacadeMission) -> bool:
    speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
    footprint_margin = 0.5 * mission.sensor_footprint_m
    return (
        in_work_region(state.tool_head, mission, footprint_margin)
        and state.tool_error <= mission.max_tracking_error_m
        and speed <= mission.max_inspection_speed_m_s
        and abs(state.attitude) <= mission.max_attitude_error_rad
        and abs(state.angular_velocity) <= mission.max_angular_rate_rad_s
    )


def coverage_fraction(states: Sequence[SimState], mission: FacadeMission) -> float:
    cols = max(1, int(math.ceil((mission.x_max - mission.x_min) / mission.coverage_cell_m)))
    rows = max(1, int(math.ceil((mission.z_max - mission.z_min) / mission.coverage_cell_m)))
    covered: set[tuple[int, int]] = set()
    footprint_radius = 0.5 * mission.sensor_footprint_m
    cell_radius = max(0, int(math.ceil(footprint_radius / mission.coverage_cell_m)))

    for state in states:
        x, z = state.tool_head
        if not valid_inspection_sample(state, mission):
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


def inspection_quality(state: SimState, params: SimParams, mission: FacadeMission) -> float:
    if not in_work_region(state.tool_head, mission, 0.5 * mission.sensor_footprint_m):
        return 0.0
    tracking_score = max(0.0, 1.0 - state.tool_error / max(mission.max_tracking_error_m, 1e-9))
    attitude_error = abs(state.attitude - params.nominal_attitude_rad)
    attitude_score = max(0.0, 1.0 - attitude_error / max(mission.max_attitude_error_rad, 1e-9))
    motion_score = max(0.0, 1.0 - blur_risk(state))
    return tracking_score * attitude_score * motion_score


def blur_risk(state: SimState) -> float:
    speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
    angular_speed = abs(state.angular_velocity)
    return min(1.0, speed / 0.30 + angular_speed / 1.20)


def facade_safety_margin(state: SimState, params: SimParams) -> float:
    thrust_margin = 1.0 - max(state.left_thrust, state.right_thrust) / max(params.max_thrust_per_drone, 1e-9)
    tension_margin = min(
        state.tension / max(params.min_tracking_tension, 1e-9) - 1.0,
        params.max_spool_tension / max(state.tension, 1e-9) - 1.0,
    )
    wind_ratio = math.hypot(state.wind_force[0], state.wind_force[1]) / max(params.total_mass * params.gravity, 1e-9)
    gimbal_margin = 1.0 - max(
        abs(state.left_gimbal_angle), abs(state.right_gimbal_angle)
    ) / max(params.gimbal_max_angle_rad, 1e-9)
    return min(thrust_margin, tension_margin, 1.0 - wind_ratio, gimbal_margin)
