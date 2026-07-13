"""Active wall-tool controller tuning.

This is the file to edit when tuning the current controller. The dictionaries
below are merged into `SimParams` by `controller.py`.
"""

from __future__ import annotations

from wall_tool_sim.reel_motor import ReelMotorSpec
from wall_tool_sim.gimbal_servo import GimbalServoSpec
from wall_tool_sim.wall_tool_ui import PLANNER_DIRECT
from wall_tool_sim.steel_cable import SteelCableSpec


NOMINAL_CABLE_LENGTH_M = 4.0
STEEL_CABLE = SteelCableSpec()
REEL_MOTOR = ReelMotorSpec()
GIMBAL_SERVO = GimbalServoSpec()

# Mission and UI/run behavior.
BEST_PLANNER = PLANNER_DIRECT
WORK_PLANNER = PLANNER_DIRECT
MISSION_TRAJECTORY = "coverage-smooth"
DEFAULT_SCENARIO_DURATION_S = 520.0


# Non-contact inspection mission and trajectory-quality limits.
FACADE_MISSION_OVERRIDES = {
    "name": "single_tether_facade_inspection",
    "x_min": -2.10,
    "x_max": 2.10,
    "z_min": 1.10,
    "z_max": 5.25,
    "lane_spacing_m": 0.42,
    "sensor_footprint_m": 0.34,
    "inspection_standoff_m": 0.10,
    "coverage_cell_m": 0.10,
    "max_inspection_speed_m_s": 0.30,
    "max_tracking_error_m": 0.08,
    "max_attitude_error_rad": 0.13962634015954636,
    "max_angular_rate_rad_s": 0.60,
}


# Desired path generation.
BEST_PATH_SPEED = 0.14
COVERAGE_CORNER_SPEED = 0.035
REFERENCE_CONFIG = {
    "path_speed": BEST_PATH_SPEED,
    "reference_accel_limit_mps2": 0.08,
    "reference_jerk_limit_mps3": 0.30,
    "reference_min_segment_duration_s": 0.90,
    "reference_preview_time_s": 1.2,
    "reference_preview_min_distance_m": 0.18,
    "reference_turn_lateral_accel_m_s2": 0.06,
}


# Active controller selection.
ACTIVE_CONTROLLER = {
    "control_law": "vector_thrust_nmpc",
}


# Passive Y-bridle attitude stabilization. Pitch remains a simulated state;
# no ideal orientation constraint is imposed.
PASSIVE_SUSPENSION_CONFIG = {
    "payload_pitch_constrained": False,
    "passive_attitude_stiffness_Nm_rad": 0.20,
    "passive_attitude_damping_Nm_s_rad": 0.055,
}


GIMBAL_CONFIG = {
    "gimbal_max_angle_rad": GIMBAL_SERVO.max_angle_rad,
    "gimbal_max_rate_rad_s": GIMBAL_SERVO.max_rate_rad_s,
    "gimbal_max_acceleration_rad_s2": GIMBAL_SERVO.max_acceleration_rad_s2,
    "gimbal_command_slew_limit_rad_s": 1.0471975511965976,
    "gimbal_natural_frequency_rad_s": GIMBAL_SERVO.natural_frequency_rad_s,
    "gimbal_damping_ratio": GIMBAL_SERVO.damping_ratio,
    "gimbal_command_min_pulse_us": GIMBAL_SERVO.command_min_pulse_us,
    "gimbal_command_max_pulse_us": GIMBAL_SERVO.command_max_pulse_us,
    "gimbal_command_resolution_us": GIMBAL_SERVO.command_resolution_us,
    # Residual shaft/propeller zero errors after ordinary mechanical setup.
    # They affect the plant but are deliberately absent from the controller's
    # nominal servo-state predictor.
    "gimbal_left_zero_error_rad": 0.006108652381980153,
    "gimbal_right_zero_error_rad": -0.004363323129985824,
}


# Explicit sensor assumptions. These values are never replaced by ideal
# ground-truth measurements when the internal plant is active.
SENSOR_CONFIG = {
    "sensor_random_seed": 2804,
    "sensor_sample_period_s": 0.010,
    "cable_angle_encoder_counts_per_rev": 16384,
    "reel_encoder_counts_per_rev": 4096,
    "cable_angle_noise_std_rad": 0.00035,
    "reel_length_noise_std_m": 0.00005,
    "load_cell_noise_std_N": 0.008,
    "imu_angle_noise_std_rad": 0.0025,
    "imu_rate_noise_std_rad_s": 0.010,
    "velocity_estimator_time_constant_s": 0.12,
}


# NMPC horizon and solver.
MPC_SOLVER_CONFIG = {
    "mpc_horizon_steps": 15,
    "mpc_horizon_dt": 0.120,
    "mpc_control_period_s": 0.075,
    "mpc_solver_max_iter": 160,
    "mpc_solver_tolerance": 2e-5,
    "mpc_energy_plot_limit_J": 0.015,
}


# NMPC hard-constraint style limits.
MPC_CONSTRAINT_CONFIG = {
    "mpc_attitude_limit_rad": 0.2617993877991494,
    "mpc_slack_limit_m": 0.004,
    "max_cable_support_fraction": 0.90,
    "desired_cable_support_fraction": 0.75,
}


# NMPC objective weights. Tracking should dominate. Effort, reel motion, input
# rate, attitude rate, unnecessary tilt, and slack are regularizers.
MPC_OBJECTIVE_WEIGHTS = {
    "mpc_tracking_position_weight": 450.0,
    "mpc_tracking_velocity_weight": 90.0,
    "mpc_terminal_position_weight": 800.0,
    "mpc_terminal_velocity_weight": 120.0,
    "mpc_drone_effort_weight": 0.18,
    "mpc_cable_effort_weight": 0.008,
    "mpc_reel_speed_weight": 0.050,
    "mpc_input_rate_weight": 0.35,
    "mpc_attitude_rate_weight": 2.5,
    "mpc_attitude_weight": 35.0,
    "mpc_gimbal_angle_weight": 0.018,
    "mpc_gimbal_rate_weight": 0.080,
    "mpc_cable_support_weight": 10.0,
    "mpc_slack_weight": 600.0,
    "mpc_hold_integral_gain_s_inv": 0.8,
    "mpc_hold_integral_limit_m_s": 0.060,
}


# Reel and cable limits used by the active NMPC plant branch and diagnostics.
CABLE_REEL_CONFIG = {
    "max_spool_speed": REEL_MOTOR.max_line_speed_m_s,
    "reel_velocity_slew_limit_mps2": 0.25,
    "min_tracking_tension": 0.15,
    "max_spool_tension": min(5.0, REEL_MOTOR.continuous_line_force_N),
    "reel_motor_voltage_v": REEL_MOTOR.voltage_v,
    "reel_motor_gear_ratio": REEL_MOTOR.gear_ratio,
    "reel_motor_no_load_rpm": REEL_MOTOR.no_load_output_rpm,
    "reel_motor_stall_torque_kg_cm": REEL_MOTOR.stall_torque_kg_cm,
    "reel_spool_radius_m": REEL_MOTOR.spool_radius_m,
    "reel_velocity_time_constant_s": REEL_MOTOR.velocity_time_constant_s,
    "reel_continuous_torque_fraction": REEL_MOTOR.continuous_torque_fraction,
    "cable_taut_band": 0.002,
    "cable_stiffness_N_m": STEEL_CABLE.axial_stiffness_N_m(NOMINAL_CABLE_LENGTH_M),
    "cable_damping_N_s_m": STEEL_CABLE.damping_N_s_m(NOMINAL_CABLE_LENGTH_M, 0.18),
    "steel_cable_diameter_m": STEEL_CABLE.diameter_m,
    "steel_cable_youngs_modulus_pa": STEEL_CABLE.youngs_modulus_pa,
    "steel_cable_density_kg_m3": STEEL_CABLE.density_kg_m3,
    "steel_cable_structural_compliance_m_N": STEEL_CABLE.structural_compliance_m_N,
    "steel_cable_damping_ratio": STEEL_CABLE.damping_ratio,
    "steel_cable_payload_weight_fraction": STEEL_CABLE.payload_weight_fraction,
    "cable_tension_rate_limit_N_s": 80.0,
    "cable_tension_time_constant_s": 0.030,
    "max_cable_extension_m": 0.009,
    "reel_tension_kp_mps_N": 0.055,
    "reel_tension_ki_mps_Ns": 0.010,
    "reel_tension_integral_limit_Ns": 5.0,
    "load_cell_filter_tau_s": 0.018,
}


# Payload/drone dynamics and actuator authority.
DYNAMICS_CONFIG = {
    "max_thrust_per_drone": 0.150 * 9.80665,
    "mpc_thrust_command_fraction": 0.92,
    "motor_thrust_time_constant_s": 0.060,
    "thrust_command_slew_limit_N_s": 0.70,
    "rotational_damping": 0.090,
}


# Normal-to-wall contact model and facade work checks.
CONTACT_CONFIG = {
    "normal_contact_enabled": False,
    "contact_work_enabled": False,
    "normal_standoff_m": FACADE_MISSION_OVERRIDES["inspection_standoff_m"],
    "desired_contact_force_N": 0.0,
    "min_contact_force_N": 0.0,
    "max_contact_force_N": 0.0,
    "contact_work_x_min": FACADE_MISSION_OVERRIDES["x_min"],
    "contact_work_x_max": FACADE_MISSION_OVERRIDES["x_max"],
    "contact_work_z_min": FACADE_MISSION_OVERRIDES["z_min"],
    "contact_work_z_max": FACADE_MISSION_OVERRIDES["z_max"],
    "work_contact_speed_limit_mps": FACADE_MISSION_OVERRIDES["max_inspection_speed_m_s"],
    "work_contact_tracking_limit_m": FACADE_MISSION_OVERRIDES["max_tracking_error_m"],
    "work_contact_angular_rate_limit_rad_s": FACADE_MISSION_OVERRIDES["max_angular_rate_rad_s"],
    "inspection_attitude_limit_rad": FACADE_MISSION_OVERRIDES["max_attitude_error_rad"],
    "wind_enabled": True,
    "wind_force_x": 0.020,
    "wind_force_z": 0.0,
    "wind_gust_force": 0.045,
    "edge_wind_gain": 0.20,
    "normal_wind_force_N": 0.0,
    "normal_wind_gust_force_N": 0.0,
}


CONTROLLER_OVERRIDES = {
    **ACTIVE_CONTROLLER,
    **PASSIVE_SUSPENSION_CONFIG,
    **GIMBAL_CONFIG,
    **SENSOR_CONFIG,
    **REFERENCE_CONFIG,
    **MPC_SOLVER_CONFIG,
    **MPC_CONSTRAINT_CONFIG,
    **MPC_OBJECTIVE_WEIGHTS,
    **CABLE_REEL_CONFIG,
    **DYNAMICS_CONFIG,
    **CONTACT_CONFIG,
}
