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
--mode ui       pure Matplotlib simulator UI
--mode log      full logged controller session
--mode quick    short smoke test
```

The chosen controller is `tool_head_nmpc`: a nonlinear model-predictive
controller for tool-head path tracking with cable-efficient load sharing:

- the desired input is the actual path horizon, not a hidden moving reference
  point,
- the NMPC optimizes future payload position, velocity, attitude, cable length,
  left/right drone thrust, cable tension, and reel speed,
- hard constraints include wall bounds, attitude limits, drone thrust limits,
  cable tension limits, reel speed/acceleration limits, and taut steel-cable
  geometry,
- the cable is treated as an inextensible unilateral support in the MPC branch:
  it can pull when taut, cannot push, and cannot carry more than 100% vertical
  support,
- body tilt is not a commanded task; the solver tilts only when that improves
  tracking or reduces drone effort over the lookahead horizon,
- point clicks and dragged paths use the same sampled path horizon,
- the Qt and Matplotlib UIs show the single chosen MPC prediction horizon,
- smooth facade trajectory support remains available for larger-wall coverage,
- 2.5D wall-normal contact dynamics remain available for cleaning/inspection
  quality,
- `6.0 m x 6.0 m` wall with a larger `4.2 m x 4.15 m` cleaning bay,
- nominal no-wind facade mission speed of `0.16 m/s`.

The live Qt UI lets you click a target, Shift-click to append a target, or hold
the mouse button down and draw a smooth path for the robot to follow. PyQtGraph
shows four fast evaluation plots: task validity, smoothness/energy,
cable/actuator use, and reel behavior. All limit plots use a normalized
`1.0` line so crossings are immediately meaningful.

Tune the selected controller in:

```text
cable_hybrid_controller/config.py
```

That file contains mission geometry, contact limits, path speed, MPC horizon,
solver settings, objective weights, cable/reel limits, and actuator limits.

## Check Current Controller

```text
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_controller.py --mode quick --duration 12
```

For a full generated report, run:

```text
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_controller.py --mode log
```

Generated output folders are intentionally not kept in this cleaned checkout.
