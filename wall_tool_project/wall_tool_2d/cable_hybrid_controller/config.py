"""Active wall-tool controller tuning.

This is the file to edit when tuning the current controller. The dictionaries
below are merged into `SimParams` by `controller.py`.
"""

from __future__ import annotations

from wall_tool_sim.wall_tool_ui import PLANNER_DIRECT


# Mission and UI/run behavior.
BEST_PLANNER = PLANNER_DIRECT
WORK_PLANNER = PLANNER_DIRECT
MISSION_TRAJECTORY = "coverage-smooth"
DEFAULT_SCENARIO_DURATION_S = 430.0


# Facade mission. These values define the coverage path and contact-quality
# limits used by the logged facade mission.
FACADE_MISSION_OVERRIDES = {
    "name": "skyscraper_facade_cleaning",
    "x_min": -2.10,
    "x_max": 2.10,
    "z_min": 1.10,
    "z_max": 5.25,
    "lane_spacing_m": 0.42,
    "tool_width_m": 0.34,
    "desired_contact_force_N": 0.55,
    "min_contact_force_N": 0.25,
    "max_contact_force_N": 0.95,
    "inspection_standoff_m": 0.10,
    "coverage_cell_m": 0.10,
    "max_cleaning_speed_m_s": 0.36,
    "max_tracking_error_m": 0.12,
    "max_angular_rate_rad_s": 1.50,
}


# Desired path generation.
BEST_PATH_SPEED = 0.3
COVERAGE_CORNER_SPEED = 0.040
REFERENCE_CONFIG = {
    "path_speed": BEST_PATH_SPEED,
    "reference_accel_limit_mps2": 0.24,
    "reference_jerk_limit_mps3": 1.20,
    "reference_min_segment_duration_s": 0.90,
}


# Active controller selection.
ACTIVE_CONTROLLER = {
    "control_law": "tool_head_nmpc",
}


# NMPC horizon and solver.
MPC_SOLVER_CONFIG = {
    "mpc_horizon_steps": 15,
    "mpc_horizon_dt": 0.200,
    "mpc_control_period_s": 0.080,
    "mpc_solver_max_iter": 40,
    "mpc_solver_tolerance": 1e-5,
    "mpc_energy_plot_limit_J": 0.015,
}


# NMPC hard-constraint style limits.
MPC_CONSTRAINT_CONFIG = {
    "mpc_attitude_limit_rad": 1.05,
    "mpc_slack_limit_m": 0.012,
    "max_cable_support_fraction": 1.0,
}


# NMPC objective weights. Tracking should dominate. Effort, reel motion, input
# rate, attitude rate, unnecessary tilt, and slack are regularizers.
MPC_OBJECTIVE_WEIGHTS = {
    "mpc_tracking_position_weight": 260.0,
    "mpc_tracking_velocity_weight": 22.0,
    "mpc_terminal_position_weight": 520.0,
    "mpc_terminal_velocity_weight": 36.0,
    "mpc_drone_effort_weight": 0.42,
    "mpc_cable_effort_weight": 0.018,
    "mpc_reel_speed_weight": 0.050,
    "mpc_input_rate_weight": 0.030,
    "mpc_attitude_rate_weight": 0.45,
    "mpc_attitude_weight": 0.025,
    "mpc_slack_weight": 180.0,
}


# Reel and cable limits used by the active NMPC plant branch and diagnostics.
CABLE_REEL_CONFIG = {
    "max_spool_speed": 0.58,
    "spool_accel_limit_mps2": 0.80,
    "min_tracking_tension": 0.10,
    "max_spool_tension": 24.0,
    "cable_taut_band": 0.006,
    "cable_stiffness_N_m": 750.0,
    "cable_damping_N_s_m": 1.20,
    "reel_tension_kp_mps_N": 0.055,
    "reel_tension_ki_mps_Ns": 0.010,
    "reel_tension_integral_limit_Ns": 5.0,
    "load_cell_filter_tau_s": 0.018,
}


# Payload/drone dynamics and actuator authority.
DYNAMICS_CONFIG = {
    "max_thrust_per_drone": 0.150 * 9.80665,
    "rotational_damping": 0.090,
}


# Normal-to-wall contact model and facade work checks.
CONTACT_CONFIG = {
    "normal_contact_enabled": True,
    "contact_work_enabled": True,
    "normal_standoff_m": FACADE_MISSION_OVERRIDES["inspection_standoff_m"],
    "desired_contact_force_N": FACADE_MISSION_OVERRIDES["desired_contact_force_N"],
    "min_contact_force_N": FACADE_MISSION_OVERRIDES["min_contact_force_N"],
    "max_contact_force_N": FACADE_MISSION_OVERRIDES["max_contact_force_N"],
    "contact_work_x_min": FACADE_MISSION_OVERRIDES["x_min"],
    "contact_work_x_max": FACADE_MISSION_OVERRIDES["x_max"],
    "contact_work_z_min": FACADE_MISSION_OVERRIDES["z_min"],
    "contact_work_z_max": FACADE_MISSION_OVERRIDES["z_max"],
    "work_contact_speed_limit_mps": FACADE_MISSION_OVERRIDES["max_cleaning_speed_m_s"],
    "work_contact_tracking_limit_m": FACADE_MISSION_OVERRIDES["max_tracking_error_m"],
    "work_contact_angular_rate_limit_rad_s": FACADE_MISSION_OVERRIDES["max_angular_rate_rad_s"],
    "wind_enabled": False,
    "wind_force_x": 0.0,
    "wind_force_z": 0.0,
    "wind_gust_force": 0.0,
    "edge_wind_gain": 0.0,
    "normal_wind_force_N": 0.0,
    "normal_wind_gust_force_N": 0.0,
}


CONTROLLER_OVERRIDES = {
    **ACTIVE_CONTROLLER,
    **REFERENCE_CONFIG,
    **MPC_SOLVER_CONFIG,
    **MPC_CONSTRAINT_CONFIG,
    **MPC_OBJECTIVE_WEIGHTS,
    **CABLE_REEL_CONFIG,
    **DYNAMICS_CONFIG,
    **CONTACT_CONFIG,
}
