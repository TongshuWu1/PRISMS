# Controller Session Report

- Scenario: `skyscraper_facade_cleaning`
- Planner/controller: `predictive`
- Duration: `357.5799999997473` s

## Key Metrics

- `final_error_m`: `0.00551`
- `max_error_m`: `0.14586`
- `rms_error_m`: `0.06422`
- `mean_drone_power_ratio`: `0.35653`
- `mean_cable_support_fraction`: `0.69199`
- `max_thrust_fraction`: `0.81547`
- `max_tension_N`: `1.82373`
- `mean_abs_spool_velocity_m_s`: `0.07017`
- `p95_spool_acceleration_m_s2`: `1.00000`
- `max_spool_acceleration_m_s2`: `1.00000`
- `reel_in_work_J`: `16.09331`
- `thrust_limit_active_fraction`: `0.00000`
- `allocation_residual_active_fraction`: `0.00330`
- `slack_sample_fraction`: `0.00000`
- `coverage_fraction`: `1.00000`
- `contact_valid_fraction`: `0.96076`
- `mean_contact_force_N`: `0.54994`
- `mean_contact_quality`: `0.99995`
- `mean_normal_gap_m`: `-0.00339`
- `mean_blur_risk`: `0.45624`
- `min_facade_safety_margin`: `-0.72229`
- `max_wind_force_N`: `0.00000`

## Controller Settings

- `wall_width_m`: `6.00000`
- `wall_height_m`: `6.00000`
- `max_cable_length_m`: `7.00000`
- `mission_trajectory`: `stop`
- `path_speed_m_s`: `0.22000`
- `coverage_corner_speed_m_s`: `0.00000`
- `max_spool_speed_m_s`: `0.58000`
- `spool_accel_limit_mps2`: `1.00000`
- `reference_speed_min`: `0.32000`
- `tracking_error_slowdown_m`: `0.07500`
- `tracking_error_full_slow_m`: `0.16000`
- `max_thrust_per_drone_N`: `1.47100`
- `hold_equilibrium_tilt_gain`: `1.00000`
- `hold_cable_support_fraction`: `0.72000`

## Interpretation

- Low tracking error with moderate cable support means the cable is doing useful work.
- High thrust-limit activity means the route/control request is too aggressive.
- High allocation residual activity means the requested force/torque combination is not fully feasible.
- Slack or tension saturation should be treated as a controller failure mode, not a cosmetic plot issue.
- For cleaning, coverage is counted only when normal contact force, speed, attitude rate, and tracking are valid.
- Low contact quality means the tool is under-pressing, over-pressing, or outside the facade work bay.
- Blur risk matters for inspection because images become less useful as vibration and speed rise.