# Controller Session Report

- Scenario: `skyscraper_facade_cleaning`
- Planner/controller: `predictive`
- Duration: `373.5299999997328` s

## Key Metrics

- `final_error_m`: `0.01022`
- `max_error_m`: `0.07694`
- `rms_error_m`: `0.04961`
- `p95_error_m`: `0.06769`
- `mean_payload_speed_m_s`: `0.14997`
- `p95_payload_speed_m_s`: `0.29665`
- `max_payload_speed_m_s`: `0.33814`
- `mean_payload_acceleration_m_s2`: `0.19808`
- `p95_payload_acceleration_m_s2`: `0.67548`
- `max_payload_acceleration_m_s2`: `1.10138`
- `p95_payload_jerk_m_s3`: `19.76218`
- `max_payload_jerk_m_s3`: `36.19802`
- `p95_tracking_error_ratio`: `0.56406`
- `p95_payload_speed_ratio`: `0.82402`
- `p95_body_rate_ratio`: `0.13742`
- `p95_body_rate_rad_s`: `0.20614`
- `p99_body_rate_rad_s`: `0.36118`
- `p95_cable_rate_rad_s`: `0.13592`
- `p95_radial_error_m`: `0.02247`
- `p95_tangential_error_m`: `0.01116`
- `mean_swing_energy_J`: `0.00003`
- `p95_swing_energy_J`: `0.00013`
- `max_swing_energy_J`: `0.00077`
- `p95_abs_swing_power_W`: `0.00034`
- `clf_violation_fraction`: `0.22376`
- `mean_drone_power_ratio`: `0.33333`
- `mean_cable_support_fraction`: `0.70836`
- `mean_cable_vertical_efficiency`: `0.83684`
- `min_cable_vertical_efficiency`: `0.28986`
- `max_thrust_fraction`: `0.77584`
- `max_tension_N`: `1.89264`
- `mean_abs_spool_velocity_m_s`: `0.06813`
- `p95_spool_acceleration_m_s2`: `0.38000`
- `max_spool_acceleration_m_s2`: `0.38000`
- `reel_in_work_J`: `16.27110`
- `thrust_limit_active_fraction`: `0.00000`
- `allocation_residual_active_fraction`: `0.01916`
- `slack_sample_fraction`: `0.00000`
- `coverage_fraction`: `1.00000`
- `contact_valid_fraction`: `1.00000`
- `work_mode_contact_valid_fraction`: `1.00000`
- `mean_contact_force_N`: `0.55000`
- `mean_contact_quality`: `1.00000`
- `mean_normal_gap_m`: `-0.00344`
- `mean_blur_risk`: `0.43997`
- `min_facade_safety_margin`: `0.22416`
- `max_wind_force_N`: `0.00000`

## Controller Settings

- `wall_width_m`: `6.00000`
- `wall_height_m`: `6.00000`
- `max_cable_length_m`: `7.00000`
- `mission_trajectory`: `coverage-smooth`
- `control_law`: `miesc`
- `path_speed_m_s`: `0.30000`
- `reference_accel_limit_mps2`: `0.40000`
- `reference_jerk_limit_mps3`: `2.10000`
- `reference_min_segment_duration_s`: `0.65000`
- `coverage_corner_speed_m_s`: `0.07500`
- `max_spool_speed_m_s`: `0.58000`
- `spool_accel_limit_mps2`: `0.38000`
- `reference_speed_min`: `0.15000`
- `tracking_error_slowdown_m`: `0.07500`
- `tracking_error_full_slow_m`: `0.16000`
- `contact_governor_enabled`: `True`
- `contact_governor_turn_distance_m`: `1.20000`
- `contact_governor_turn_min_scale`: `0.18000`
- `contact_governor_geometry_efficiency`: `0.62000`
- `contact_governor_geometry_min_scale`: `0.28000`
- `contact_governor_tracking_ratio`: `0.42000`
- `contact_governor_tracking_min_scale`: `0.16000`
- `contact_governor_speed_ratio`: `0.68000`
- `contact_governor_speed_min_scale`: `0.30000`
- `max_thrust_per_drone_N`: `1.47100`
- `hold_equilibrium_tilt_gain`: `1.00000`
- `hold_cable_support_fraction`: `0.72000`
- `pendulum_theta_kp`: `10.00000`
- `pendulum_theta_kd`: `8.00000`
- `max_pendulum_theta_ddot`: `3.20000`
- `max_tangential_accel`: `2.80000`
- `miesc_radial_frequency_rad_s`: `1.35000`
- `miesc_radial_damping_ratio`: `1.15000`
- `miesc_tangential_frequency_rad_s`: `3.00000`
- `miesc_tangential_damping_ratio`: `0.90000`
- `miesc_clf_decay_rate`: `2.35000`
- `miesc_spool_accel_limit_mps2`: `0.38000`
- `reference_slowdown_rate`: `2.50000`
- `reference_recovery_rate`: `1.10000`

## Invalid Contact Reasons

- `region`: `0.00000`
- `contact_low`: `0.00000`
- `contact_high`: `0.00000`
- `tracking`: `0.00000`
- `speed`: `0.00000`
- `angular_rate`: `0.00000`

## Interpretation

- Low tracking error with moderate cable support means the cable is doing useful work.
- High thrust-limit activity means the route/control request is too aggressive.
- High allocation residual activity means the requested force/torque combination is not fully feasible.
- Slack or tension saturation should be treated as a controller failure mode, not a cosmetic plot issue.
- For cleaning, coverage is counted only when normal contact force, speed, attitude rate, and tracking are valid.
- Low contact quality means the tool is under-pressing, over-pressing, or outside the facade work bay.
- Blur risk matters for inspection because images become less useful as vibration and speed rise.