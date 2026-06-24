# Controller Session Report

- Scenario: `skyscraper_facade_cleaning`
- Planner/controller: `predictive`
- Duration: `354.58499999975004` s

## Key Metrics

- `final_error_m`: `0.00446`
- `max_error_m`: `0.11894`
- `rms_error_m`: `0.04909`
- `p95_error_m`: `0.06594`
- `mean_payload_speed_m_s`: `0.15731`
- `p95_payload_speed_m_s`: `0.30096`
- `max_payload_speed_m_s`: `0.35265`
- `mean_payload_acceleration_m_s2`: `0.16489`
- `p95_payload_acceleration_m_s2`: `0.51036`
- `max_payload_acceleration_m_s2`: `0.89990`
- `p95_payload_jerk_m_s3`: `15.00225`
- `max_payload_jerk_m_s3`: `29.56486`
- `p95_tracking_error_ratio`: `0.54948`
- `p95_payload_speed_ratio`: `0.83599`
- `p95_body_rate_ratio`: `0.12024`
- `p95_body_rate_rad_s`: `0.18036`
- `p99_body_rate_rad_s`: `0.30672`
- `p95_cable_rate_rad_s`: `0.13970`
- `mean_drone_power_ratio`: `0.34940`
- `mean_cable_support_fraction`: `0.69709`
- `mean_cable_vertical_efficiency`: `0.82596`
- `min_cable_vertical_efficiency`: `0.28474`
- `max_thrust_fraction`: `0.82009`
- `max_tension_N`: `1.81804`
- `mean_abs_spool_velocity_m_s`: `0.07013`
- `p95_spool_acceleration_m_s2`: `0.65000`
- `max_spool_acceleration_m_s2`: `0.65000`
- `reel_in_work_J`: `15.90460`
- `thrust_limit_active_fraction`: `0.00000`
- `allocation_residual_active_fraction`: `0.00353`
- `slack_sample_fraction`: `0.00000`
- `coverage_fraction`: `1.00000`
- `contact_valid_fraction`: `1.00000`
- `mean_contact_force_N`: `0.55000`
- `mean_contact_quality`: `1.00000`
- `mean_normal_gap_m`: `-0.00344`
- `mean_blur_risk`: `0.45968`
- `min_facade_safety_margin`: `0.17991`
- `max_wind_force_N`: `0.00000`

## Controller Settings

- `wall_width_m`: `6.00000`
- `wall_height_m`: `6.00000`
- `max_cable_length_m`: `7.00000`
- `mission_trajectory`: `stop`
- `path_speed_m_s`: `0.26000`
- `coverage_corner_speed_m_s`: `0.00000`
- `max_spool_speed_m_s`: `0.58000`
- `spool_accel_limit_mps2`: `0.65000`
- `reference_speed_min`: `0.24000`
- `tracking_error_slowdown_m`: `0.07500`
- `tracking_error_full_slow_m`: `0.16000`
- `contact_governor_enabled`: `True`
- `contact_governor_turn_distance_m`: `1.20000`
- `contact_governor_turn_min_scale`: `0.22000`
- `contact_governor_geometry_efficiency`: `0.38000`
- `contact_governor_geometry_min_scale`: `0.58000`
- `contact_governor_tracking_ratio`: `0.54000`
- `contact_governor_tracking_min_scale`: `0.30000`
- `contact_governor_speed_ratio`: `0.72000`
- `contact_governor_speed_min_scale`: `0.34000`
- `max_thrust_per_drone_N`: `1.47100`
- `hold_equilibrium_tilt_gain`: `1.00000`
- `hold_cable_support_fraction`: `0.72000`
- `pendulum_theta_kp`: `12.00000`
- `pendulum_theta_kd`: `8.50000`
- `max_pendulum_theta_ddot`: `3.80000`
- `max_tangential_accel`: `3.20000`
- `reference_slowdown_rate`: `3.20000`
- `reference_recovery_rate`: `1.40000`

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