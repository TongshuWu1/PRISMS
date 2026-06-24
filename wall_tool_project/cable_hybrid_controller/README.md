# Cable Hybrid Controller

This is the main controller package for the wall-tool project.

The chosen architecture is:

- **MIESC** mixed-input energy-shaping control,
- reel velocity control for radial cable length and load support,
- drone acceleration/force control for tangential tracking, swing damping, and body torque,
- CLF-style tangential energy projection to avoid persistent cable-swing injection,
- boundary-aware smooth coverage references that stay inside the facade work bay,
- contact-valid cable-efficient reference governor for cleaning work mode,
- acceleration/jerk-limited quintic timing for each coverage segment,
- preview braking before lane-end reversals and poor cable geometry,
- pre-limit speed/tracking governor before contact validity is lost,
- time-scaled reference velocity/acceleration when the governor slows the trajectory clock,
- lower reel acceleration limit to avoid cable command chatter,
- direct facade-lane following inside the governed cleaning work mode,
- predictive cable-aware routing for difficult transit/approach moves,
- non-zero cable-supported equilibrium attitude for hold,
- hover/hold allocation that prioritizes position damping over perfect attitude torque,
- 2.5D normal-to-wall contact dynamics for facade cleaning/inspection quality.
- no nominal wind disturbance while tuning fast, smooth, cable-efficient motion.
- `6.0 m x 6.0 m` wall with a larger `4.2 m x 4.15 m` cleaning bay.
- nominal facade speed of `0.30 m/s`, with the governor reducing local speed only near contact-risk events.

The normal contact model is deliberately labeled 2.5D: the simulator still uses
wall-plane cable dynamics for `x-z` motion, plus a separate normal `gap` state
for wall contact. It is not a full 3D aerodynamic model. Coverage is only counted
when contact force, tracking error, tool speed, and angular rate are inside the
mission limits.

## Run

Use the single project runfile:

```text
run_wall_tool_controller.py
```

Modes:

```text
--mode qt       hybrid Matplotlib scene + PyQtGraph evaluation UI
--mode tk       Tkinter fallback controller UI
--mode log      full logged controller session
--mode ui       legacy Matplotlib simulator
--mode quick    short smoke test
```

In PyCharm, right-click `run_wall_tool_controller.py` and run it. The default is
`--mode qt`.

In the Qt UI, Matplotlib renders the wall/robot scene and PyQtGraph renders the
fast evaluation dashboard. Click the wall to send a single target, Shift-click
to append a target, or hold the mouse button and drag to draw a smooth path. The
path is committed when you release the mouse.

## Outputs

Logged sessions are written under:

```text
cable_hybrid_controller/output/sessions/
```

Each session contains:

- `session_log.csv`: time-series data for tracking, cable, reel, thrust, attitude, normal contact, efficiency, and MIESC energy terms.
- `segment_summary.csv`: per-waypoint tracking, power, cable support, contact force, and valid-contact fraction.
- `summary.json`: numeric performance summary.
- `report.md`: short readable report with the main metrics and controller settings.
- `tracking.png`: wall path and tracking error.
- `actuation.png`: tension, spool velocity, thrust, and allocation residual.
- `attitude_cable.png`: body attitude, cable angle, and cable/spool lengths.
- `efficiency.png`: cable support, drone power proxy, reference speed, and reel-in power.
- `facade_work.png`: work-region path, contact force, normal gap/actuator, valid contact, blur risk, and disturbance channels.
- `controller_dashboard.png`: path validity, scorecard, tracking-error CDF, limit ratios, efficiency, and invalid-contact reasons.
- `coverage_map.png`: contact-gated covered cells and spatial locations where valid contact was lost.
- `limit_margins.png`: normalized tracking, speed, body-rate, thrust, allocation, tension, and reel-smoothness margins.
- `smoothness.png`: reference speed, payload acceleration/jerk, body/cable rate, MIESC energy shaping, and reel acceleration.
- `efficiency_phase.png`: cable geometry, load sharing, reel work, and actuator-feasibility phase plots.
- `segment_scorecard.png`: per-segment duration, tracking error, valid contact, cable support, and drone power.

## What To Watch

- Tracking error should settle without large residual oscillation.
- Cable tension should stay positive and smooth.
- Cable support fraction should be meaningful; otherwise the drones are doing too much work.
- Max thrust fraction near `1.0` means the path is too aggressive or geometry is poor.
- Allocation residual means the requested force/torque combination is not fully feasible.
- Slack, tension saturation, high allocation residual, and invalid contact are controller failure modes.
- Coverage fraction is contact-gated; it should not look good unless the tool is actually in usable wall contact.
- A faster run can be tested by changing `BEST_PATH_SPEED`, but treat contact validity and coverage as primary cleaning metrics.

## Publishable Controller Idea

Working name: **MIESC**, a Mixed-Input Energy-Shaping Controller.

The idea is to respect the different actuator types instead of hiding them
inside one Cartesian PID loop. The reel remains a velocity-controlled
cable-length actuator. The drone pair remains an acceleration/force-controlled
wall-plane actuator. MIESC splits the task in cable polar coordinates:

- radial cable-length error goes to the reel velocity policy,
- tangential wall-plane error goes to the drone acceleration command,
- a CLF-style storage function penalizes swing/tracking energy,
- a contact-valid governor slows the reference before cleaning constraints are violated.

The governor modulates the trajectory clock using task constraints:

- previewed lane-end turn risk from the queued waypoint geometry,
- cable vertical-efficiency risk near upper facade corners,
- pre-limit tracking-error ratio,
- pre-limit tool-speed ratio for inspection blur/contact validity.

This is intentionally lighter than full nonlinear MPC, but it is MPC-like in
spirit: it anticipates constraint violations and slows the reference before the
low-level hybrid system saturates. The nominal reference itself is also bounded:
quintic segment duration is chosen from speed, acceleration, and jerk limits, so
the command is feasible before any corrective governor action. The key
implementation detail is that reference velocity and acceleration are time-scaled
with the slowed trajectory clock, so feed-forward terms stay consistent.

The current result is the important research signal: compared with the previous
clean large-wall run, MIESC keeps full contact-gated coverage and full contact
validity while increasing mean cable support from `0.693` to `0.708`, reducing
mean drone power ratio from `0.355` to `0.333`, reducing p95 reel acceleration
from `0.500 m/s^2` to `0.380 m/s^2`, and keeping RMS tracking essentially equal
at `0.0496 m`.

## Current Reference Run

Latest clean logged run:

```text
output/sessions/20260624_001654/
```

Key numbers:

- wall size: `6.0 m x 6.0 m`
- cleaning bay: `x = [-2.10, 2.10] m`, `z = [1.10, 5.25] m`
- trajectory: `coverage-smooth`
- control law: `miesc`
- path speed: `0.300 m/s`
- reference acceleration limit: `0.400 m/s^2`
- reference jerk limit: `2.100 m/s^3`
- duration: `373.53 s`
- final error: `0.0102 m`
- max error: `0.0769 m`
- RMS error: `0.0496 m`
- 95th-percentile error: `0.0677 m`
- max payload speed: `0.3381 m/s`
- 95th-percentile body rate: `0.206 rad/s`
- 95th-percentile payload jerk: `19.76 m/s^3`
- mean swing energy: `0.000026 J`
- 95th-percentile swing energy: `0.000127 J`
- contact-gated coverage: `1.000`
- contact-valid fraction: `1.000`
- mean contact force: `0.550 N`
- mean contact quality: `1.000`
- mean cable support fraction: `0.708`
- mean drone power ratio: `0.333`
- max thrust fraction: `0.776`
- mean absolute spool velocity: `0.0681 m/s`
- 95th-percentile spool acceleration: `0.380 m/s^2`
- max wind force: `0.000 N`
- thrust-limit active fraction: `0.0000`
- slack fraction: `0.0000`

Main limitation: the normal contact axis is currently a proxy actuator. The next
serious research step is a constrained 3D allocation/MPC layer that accounts for
drone attitude, normal force authority, cable tension, and facade contact limits
together.
