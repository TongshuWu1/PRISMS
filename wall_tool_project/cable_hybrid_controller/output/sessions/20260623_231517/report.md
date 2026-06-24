# Controller Session Report

- Scenario: `skyscraper_facade_cleaning`
- Planner/controller: `predictive`
- Duration: `356.3049999997485` s

## Key Metrics

- `final_error_m`: `0.00441`
- `max_error_m`: `0.11717`
- `rms_error_m`: `0.04894`
- `p95_error_m`: `0.06617`
- `mean_payload_speed_m_s`: `0.15689`
- `p95_payload_speed_m_s`: `0.30590`
- `max_payload_speed_m_s`: `0.35640`
- `p95_tracking_error_ratio`: `0.55139`
- `p95_payload_speed_ratio`: `0.84972`
- `p95_body_rate_ratio`: `0.17773`
- `mean_drone_power_ratio`: `0.33938`
- `mean_cable_support_fraction`: `0.70266`
- `mean_cable_vertical_efficiency`: `0.83277`
- `min_cable_vertical_efficiency`: `0.28560`
- `max_thrust_fraction`: `0.82894`
- `max_tension_N`: `1.82237`
- `mean_abs_spool_velocity_m_s`: `0.06989`
- `p95_spool_acceleration_m_s2`: `1.00000`
- `max_spool_acceleration_m_s2`: `1.00000`
- `reel_in_work_J`: `15.91883`
- `thrust_limit_active_fraction`: `0.00000`
- `allocation_residual_active_fraction`: `0.04882`
- `slack_sample_fraction`: `0.00000`
- `coverage_fraction`: `1.00000`
- `contact_valid_fraction`: `1.00000`
- `mean_contact_force_N`: `0.55000`
- `mean_contact_quality`: `1.00000`
- `mean_normal_gap_m`: `-0.00344`
- `mean_blur_risk`: `0.46252`
- `min_facade_safety_margin`: `0.17106`
- `max_wind_force_N`: `0.00000`

## Controller Settings

- `wall_width_m`: `6.00000`
- `wall_height_m`: `6.00000`
- `max_cable_length_m`: `7.00000`
- `mission_trajectory`: `stop`
- `path_speed_m_s`: `0.27000`
- `coverage_corner_speed_m_s`: `0.00000`
- `max_spool_speed_m_s`: `0.58000`
- `spool_accel_limit_mps2`: `1.00000`
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