"""Data contracts for the wall-tool nonlinear MPC."""

from __future__ import annotations

from dataclasses import dataclass


Vec2 = tuple[float, float]


@dataclass(frozen=True)
class MPCConfig:
    horizon_steps: int
    horizon_dt: float
    control_period_s: float
    mass: float
    inertia: float
    gravity: float
    anchor: Vec2
    wall_width: float
    wall_height: float
    wall_margin: float
    payload_hex_radius: float
    payload_half_length: float
    module_gap: float
    left_center_offset_zero: Vec2
    right_center_offset_zero: Vec2
    hex_face_tilt_rad: float
    nominal_attitude_rad: float
    rotational_damping: float
    max_thrust_per_drone: float
    max_cable_tension: float
    max_cable_support_fraction: float
    min_cable_vertical_efficiency: float
    min_cable_length: float
    max_cable_length: float
    max_spool_speed: float
    spool_accel_limit_mps2: float
    attitude_limit_rad: float
    slack_limit_m: float
    tracking_position_weight: float
    tracking_velocity_weight: float
    terminal_position_weight: float
    terminal_velocity_weight: float
    drone_effort_weight: float
    cable_effort_weight: float
    reel_speed_weight: float
    input_rate_weight: float
    attitude_rate_weight: float
    attitude_weight: float
    slack_weight: float
    solver_max_iter: int
    solver_tolerance: float


@dataclass(frozen=True)
class MPCReferenceHorizon:
    positions: tuple[Vec2, ...]
    velocities: tuple[Vec2, ...]


@dataclass(frozen=True)
class MPCSolution:
    success: bool
    status: str
    solve_time_s: float
    objective: float
    left_thrust: float
    right_thrust: float
    cable_tension: float
    spool_velocity: float
    predicted_positions: tuple[Vec2, ...]
    predicted_attitudes: tuple[float, ...]
    predicted_tensions: tuple[float, ...]
    predicted_spool_speeds: tuple[float, ...]
