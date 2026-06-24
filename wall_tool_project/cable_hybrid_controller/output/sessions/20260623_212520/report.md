# Controller Session Report

- Scenario: `skyscraper_facade_cleaning`
- Planner/controller: `predictive`
- Duration: `351.26499999975306` s

## Key Metrics

- `final_error_m`: `0.00616`
- `max_error_m`: `0.20978`
- `rms_error_m`: `0.06929`
- `mean_drone_power_ratio`: `0.35732`
- `mean_cable_support_fraction`: `0.69046`
- `max_thrust_fraction`: `0.81782`
- `max_tension_N`: `1.82556`
- `mean_abs_spool_velocity_m_s`: `0.07108`
- `p95_spool_acceleration_m_s2`: `1.00000`
- `max_spool_acceleration_m_s2`: `1.00000`
- `reel_in_work_J`: `15.94813`
- `thrust_limit_active_fraction`: `0.00000`
- `allocation_residual_active_fraction`: `0.00730`
- `slack_sample_fraction`: `0.00000`
- `coverage_fraction`: `0.98526`
- `contact_valid_fraction`: `0.90235`
- `mean_contact_force_N`: `0.51176`
- `mean_contact_quality`: `0.93077`
- `mean_normal_gap_m`: `0.00376`
- `mean_blur_risk`: `0.46479`
- `min_facade_safety_margin`: `-0.72229`
- `max_wind_force_N`: `0.00000`

## Controller Settings

- `mission_trajectory`: `coverage-smooth`
- `path_speed_m_s`: `0.23000`
- `coverage_corner_speed_m_s`: `0.03000`
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