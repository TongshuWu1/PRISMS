# Controller Session Report

- Scenario: `skyscraper_facade_cleaning`
- Planner/controller: `predictive`
- Duration: `153.96499999993247` s

## Key Metrics

- `final_error_m`: `0.00938`
- `max_error_m`: `0.12825`
- `rms_error_m`: `0.06343`
- `mean_drone_power_ratio`: `0.34840`
- `mean_cable_support_fraction`: `0.70364`
- `max_thrust_fraction`: `0.77915`
- `max_tension_N`: `1.82852`
- `reel_in_work_J`: `6.99917`
- `thrust_limit_active_fraction`: `0.00000`
- `allocation_residual_active_fraction`: `0.00578`
- `slack_sample_fraction`: `0.00000`
- `coverage_fraction`: `1.00000`
- `contact_valid_fraction`: `0.96509`
- `mean_contact_force_N`: `0.54986`
- `mean_contact_quality`: `0.99989`
- `mean_normal_gap_m`: `-0.00333`
- `mean_blur_risk`: `0.43623`
- `min_facade_safety_margin`: `-0.72229`
- `max_wind_force_N`: `0.00000`

## Controller Settings

- `path_speed_m_s`: `0.22000`
- `max_spool_speed_m_s`: `0.58000`
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