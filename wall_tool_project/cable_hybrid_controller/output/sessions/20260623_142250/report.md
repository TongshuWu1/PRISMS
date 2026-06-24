# Controller Session Report

- Scenario: `skyscraper_facade_cleaning`
- Planner/controller: `predictive`
- Duration: `392.94999999971515` s

## Key Metrics

- `final_error_m`: `0.03659`
- `max_error_m`: `0.14538`
- `rms_error_m`: `0.07057`
- `mean_drone_power_ratio`: `0.40713`
- `mean_cable_support_fraction`: `0.61390`
- `max_thrust_fraction`: `1.00000`
- `max_tension_N`: `3.76188`
- `reel_in_work_J`: `243.07552`
- `thrust_limit_active_fraction`: `0.00020`
- `allocation_residual_active_fraction`: `0.55474`
- `slack_sample_fraction`: `0.00000`
- `coverage_fraction`: `1.00000`
- `contact_valid_fraction`: `0.94567`
- `mean_contact_force_N`: `0.53947`
- `mean_contact_quality`: `0.99995`
- `mean_normal_gap_m`: `-0.00333`
- `mean_blur_risk`: `0.19407`
- `min_facade_safety_margin`: `-0.72669`
- `max_wind_force_N`: `0.09931`

## Controller Settings

- `path_speed_m_s`: `0.18000`
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