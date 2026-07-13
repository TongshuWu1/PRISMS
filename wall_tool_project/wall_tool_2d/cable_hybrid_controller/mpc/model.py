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
    max_payload_speed_m_s: float
    payload_hex_radius: float
    payload_half_length: float
    module_gap: float
    left_center_offset_zero: Vec2
    right_center_offset_zero: Vec2
    hex_face_tilt_rad: float
    nominal_attitude_rad: float
    rotational_damping: float
    passive_attitude_stiffness_Nm_rad: float
    passive_attitude_damping_Nm_s_rad: float
    motor_thrust_time_constant_s: float
    max_thrust_per_drone: float
    thrust_command_slew_limit_N_s: float
    max_gimbal_angle_rad: float
    max_gimbal_rate_rad_s: float
    gimbal_command_slew_limit_rad_s: float
    max_gimbal_acceleration_rad_s2: float
    gimbal_natural_frequency_rad_s: float
    gimbal_damping_ratio: float
    max_cable_tension: float
    min_tracking_tension: float
    max_cable_support_fraction: float
    desired_cable_support_fraction: float
    min_cable_vertical_efficiency: float
    min_cable_length: float
    max_cable_length: float
    max_spool_speed: float
    reel_velocity_time_constant_s: float
    reel_stall_line_force_N: float
    reel_velocity_slew_limit_mps2: float
    cable_tension_rate_limit_N_s: float
    cable_stiffness_N_m: float
    cable_damping_N_s_m: float
    cable_tension_time_constant_s: float
    max_cable_extension_m: float
    cable_mass_per_length_kg_m: float
    cable_payload_weight_fraction: float
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
    gimbal_angle_weight: float
    gimbal_rate_weight: float
    cable_support_weight: float
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
    left_gimbal_angle: float
    right_gimbal_angle: float
    cable_tension: float
    spool_velocity: float
    predicted_positions: tuple[Vec2, ...]
    predicted_attitudes: tuple[float, ...]
    predicted_left_gimbal_angles: tuple[float, ...]
    predicted_right_gimbal_angles: tuple[float, ...]
    predicted_tensions: tuple[float, ...]
    predicted_spool_speeds: tuple[float, ...]
