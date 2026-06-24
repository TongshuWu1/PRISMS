# Wall Tool Controller Project

This project contains the cable-suspended wall-tool controller work.

The main controller folder is:

```text
cable_hybrid_controller/
```

The simulator backend is:

```text
wall_tool_sim/
```

## Run In PyCharm

Right-click this one file:

- `run_wall_tool_controller.py`

Default mode opens the hybrid Qt controller UI. It uses Matplotlib for the
normal wall/robot render and PyQtGraph for fast realtime evaluation plots. Use
script arguments when needed:

```text
--mode qt       hybrid Matplotlib scene + PyQtGraph evaluation UI
--mode tk       Tkinter fallback simulator UI
--mode ui       legacy pure Matplotlib simulator UI
--mode log      full logged controller session
--mode quick    short smoke test
```

The chosen controller is **MIESC**: a mixed-input energy-shaping controller for
the hybrid reel/drone system:

- reel velocity regulates radial cable length and load support,
- drone acceleration regulates tangential tracking and swing energy,
- a CLF-style energy projection prevents the tangential command from injecting persistent swing,
- boundary-aware corner smoothing keeps the cleaning reference inside the facade work bay,
- contact-valid reference-governor speed caps before lane reversals,
- acceleration/jerk-limited quintic segment timing,
- pre-limit slowing from tracking, tool speed, and cable-geometry risk,
- time-scaled reference velocity/acceleration so feed-forward terms match the slowed clock,
- predictive setup routing for difficult transit targets,
- reel acceleration limiting to avoid cable command bumps,
- smooth facade trajectory for full larger-wall coverage,
- drone force control for wall-plane swing and body torque,
- non-zero cable-supported equilibrium tilt for hold.
- 2.5D wall-normal contact dynamics for cleaning/inspection quality.
- `6.0 m x 6.0 m` wall with a larger `4.2 m x 4.15 m` cleaning bay.
- nominal no-wind facade mission speed of `0.30 m/s`, with local governor slowdowns.

The live Qt UI lets you click a target, Shift-click to append a target, or hold
the mouse button down and draw a smooth path for the robot to follow. PyQtGraph
shows four fast evaluation plots: task validity, smoothness/energy,
cable/actuator use, and reel/governor behavior. All limit plots use a normalized
`1.0` line so crossings are immediately meaningful.

## Outputs

Logged sessions are written under:

```text
cable_hybrid_controller/output/sessions/
```

Each run writes `session_log.csv`, `summary.json`, `report.md`, and diagnostic plots including a controller dashboard, coverage map, limit margins, smoothness plot, efficiency phase plots, and per-segment scorecard.

Current clean reference run:

```text
cable_hybrid_controller/output/sessions/20260624_001654/
```

Key metrics: `373.53 s` duration on the larger wall, `0.0496 m` RMS tracking
error, `0.0769 m` max tracking error, full contact-gated coverage, `1.000`
valid-contact fraction, `0.708` mean cable support fraction, `0.333` mean
drone power ratio, mean swing energy `0.000026 J`, p95 body rate
`0.206 rad/s`, p95 jerk `19.76 m/s^3`, p95 reel acceleration `0.380 m/s^2`,
zero wind, zero slack, and zero thrust-limit activity.
