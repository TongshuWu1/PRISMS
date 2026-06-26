# Cable Hybrid Controller

This is the main controller package for the wall-tool project.

The chosen architecture is:

- `tool_head_nmpc` nonlinear MPC over the wall-plane payload/cable/drone model,
- direct optimization of left side-motor thrust, right side-motor thrust, desired cable
  support/tension, and reel feedforward velocity,
- the simulated hardware only actuates reel speed; desired tension is realized
  through a load-cell feedback reel-speed loop and a stiff cable model,
- prediction of payload position, velocity, tilt, angular rate, and paid-out
  cable length over the horizon,
- hard constraints for wall bounds, actuator limits, cable tension/support
  limits, taut cable geometry, attitude limits, and reel speed/acceleration,
- tool-head tracking is the primary objective; drone power, reel motion, input
  rate, slack, and unnecessary tilt are secondary costs,
- cable support is cheap but bounded, so the optimizer uses cable strength when
  it reduces motor effort without exceeding 100% vertical support,
- one integrated wall-tool payload carries the tool head and both canted side
  motors; old internal `drone_*` names remain only as controller variable names,
- boundary-aware smooth coverage references that stay inside the facade work bay,
- acceleration/jerk-limited quintic timing for each coverage segment,
- geometric path-horizon sampling instead of a dt-advanced moving reference point,
- direct click-to-point targeting for interactive commands,
- non-inverted attitude limits; body pose is an optimized internal variable,
- inextensible unilateral steel-cable logic in the active NMPC branch,
- 2.5D normal-to-wall contact dynamics for facade cleaning/inspection quality.
- no nominal wind disturbance while tuning fast, smooth, cable-efficient motion.
- `6.0 m x 6.0 m` wall with a larger `4.2 m x 4.15 m` cleaning bay.
- nominal facade speed of `0.16 m/s`.

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
--mode ui       Matplotlib simulator
--mode quick    short smoke test
```

In PyCharm, right-click `run_wall_tool_controller.py` and run it. The default is
`--mode qt`.

In the Qt UI, Matplotlib renders the wall/robot scene and PyQtGraph renders the
fast evaluation dashboard. Click the wall to send a single target, Shift-click
to append a target, or hold the mouse button and drag to draw a smooth path. The
path is committed when you release the mouse.

## Generated Reports

Use `--mode log` only when you want to generate fresh reports. Generated output folders are intentionally not kept in this cleaned checkout.

## What To Watch

- Tracking error should settle without large residual oscillation.
- Cable tension should stay positive and smooth.
- Cable support fraction should be meaningful; otherwise the side motors are doing too much work.
- Max thrust fraction near `1.0` means the path is too aggressive or geometry is poor.
- MPC status should normally be `Solve_Succeeded`; a held previous feasible
  command means the last nonlinear solve failed.
- Slack, tension saturation, high MPC solve time, and invalid contact are
  controller failure modes.
- Coverage fraction is contact-gated; it should not look good unless the tool is actually in usable wall contact.
- A faster run can be tested by changing `BEST_PATH_SPEED`, but treat contact validity and coverage as primary cleaning metrics.

## Active NMPC Solver

The active controller uses CasADi/IPOPT through:

```text
cable_hybrid_controller/mpc/
```

At each receding-horizon solve, the simulator samples the desired path over the
future horizon and passes the measured state
`[x, z, vx, vz, attitude, attitude_rate, cable_length]` to the solver. The first
optimized command is applied to the plant:

```text
[left_motor_thrust, right_motor_thrust, cable_tension, reel_velocity]
```

The UI draws the chosen predicted path as a single purple dash-dot horizon.
The active path has one chosen prediction horizon and no rollout cloud.

Tune the selected controller in:

```text
cable_hybrid_controller/config.py
```

That file contains mission geometry, contact limits, path speed, planner
choice, MPC horizon, objective weights, cable/reel limits, actuator limits, and
solver tolerances.

## Check Current Controller

```text
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_controller.py --mode quick --duration 12
```

Use `--mode log` only when you want to generate fresh reports.
