# PRISMS Wall Tool Simulator Backend

Standalone wall-plane simulator backend for the cable-suspended wall tooling idea.
For day-to-day controller work, use `..\cable_hybrid_controller` through the
project-level `..\..\run_wall_tool_controller.py` entry point.

## Run

From the `wall_tool_project` folder:

```powershell
python wall_tool_2d\wall_tool_sim\wall_tool_ui.py
```

Use the constrained predictive planner for harder upper-corner tests:

```powershell
python wall_tool_2d\wall_tool_sim\wall_tool_ui.py --planner predictive
```

From the PRISMS workspace root, prefix the path with `wall_tool_project\`.

## Concept

- A fixed anchor and spool at the top of the wall carries the payload weight.
- The robot is one integrated wall-tool payload with a central tool head and two canted side motors.
- In the nominal flat pose, the left motor thrust axis points up-right and the right motor thrust axis points up-left, about 35 degrees from vertical.
- Each side motor assembly keeps the previous 50 g / 150 gf equivalent thrust authority.
- The central tool payload is modeled as 75 g.
- The side motors do not hover the system by themselves; they inject wall-plane force to swing and stabilize the suspended tool head.
- The spool controls cable length velocity, so the system can reach wall targets with lower hover energy than a fully airborne tool carrier.

## Model

- The suspended assembly is simulated as wall-plane translation plus one planar tilt state.
- Facade work adds a 2.5D normal-to-wall contact state: positive gap means standoff, negative gap means wall penetration through a spring-damper contact model.
- Total suspended mass is the 75 g central tool payload plus two 50 g side motor assemblies, so gravity acts on 175 g total.
- Each side motor can generate up to 150 gf, modeled as `0.150 * g = 1.47 N` of aggregate thrust along its tilted thrust axis.
- The cable attaches to the top of the integrated payload body, so cable force can create torque about the tool center.
- The spool state is paid-out cable length, not tool position.
- The spool actuator commands cable velocity and has a maximum reel speed.
- The active `tool_head_nmpc` controller uses an inextensible unilateral steel-cable branch: cable length is constrained against anchor-to-mount distance, tension is selected by the NMPC only when taut, and vertical cable support is capped at 100% of suspended weight.
- The controller state estimate comes from the simulated sensor channels: cable angle/rate, spool encoder length/velocity, cable tension, and IMU tilt/rate.
- The active NMPC path samples the desired path over a finite horizon; there is no speed-scaling supervisor or fallback controller.
- References are kept away from the near-anchor singular region where a tiny cable length would make the wall-plane model physically unrealistic.
- The active NMPC controller optimizes payload position/velocity, tilt/rate, paid-out cable length, left/right motor thrust, cable tension, and reel velocity directly.
- The spool and cable are optimized together with side-motor thrust; there is no separate radial policy or tangential pendulum controller.
- In the active NMPC branch, tilt is not scheduled as a task. It is an internal optimization state constrained away from inversion; the solver uses tilt only when it improves tracking or reduces future motor effort.
- Thrust allocation is part of the nonlinear program. The first optimized left motor thrust, right motor thrust, cable tension, and reel velocity command is applied each control period.
- The tool head is colocated with the center of the payload module in this 2-D model.
- Click commands are converted into a moving Cartesian reference with position, velocity, and acceleration.
- Single-click control creates a quintic straight-line reference from the current payload position to the clicked point.
- The `predictive` planner option chooses a cable-friendly intermediate waypoint by scoring candidate routes against cable geometry, angular change, cable-length change, detour, and hard-target safety margins before issuing the smooth reference.
- Append mode queues clicked points as a smooth quintic waypoint trajectory.
- The UI draws a single integrated body with two small canted motor pods instead of separate cages.

## Feedback Assumptions

- Anchor-side feedback provides cable angle and cable tension.
- The spool motor encoder provides cable payout length and payout velocity.
- Body IMU feedback provides the integrated assembly planar tilt and tilt rate.
- The controller receives no direct wall-plane tool position measurement.
- Ground-truth position, velocity, and actual actuator forces are still stored for plotting and evaluation, but the controller does not use those perfect values directly.

## UI

- Normal click replaces the current command with one smooth straight-line move.
- Turn on `Append` and click multiple points to follow a smooth queued trajectory.
- Turn on `Draw`, drag a path on the wall, and release to convert the stroke into one continuous smooth curve. The drawn stroke only eases at the beginning and end, not at every intermediate point.
- Main wall view stays clear of telemetry text.
- The teal solid line is the true tool-head trajectory.
- The red dotted line is the desired tool-head trajectory already issued by the reference generator.
- The red dashed line is the remaining future desired path. In NMPC mode the purple dash-dot line is the chosen prediction horizon.
- Right panel shows only realistic live feedback and trajectory error.
- The live feedback block contains anchor cable angle/rate, cable tension, spool encoder payout length/velocity, manipulator IMU tilt/rate, and live efficiency estimates.
- Live efficiency estimates show cable vertical support, drone vertical support, drone induced-power proxy relative to no-cable hover, and spool mechanical power.
- The live trajectory-error plot compares sensor-estimated tracking error with simulator ground-truth error for evaluation.
- Bottom controls provide pause/play, reset, path/trace clear, append mode, draw mode, playback speed, trace toggle, target toggle, path toggle, and force-vector toggle.

## Notes

- Controller architecture notes are kept in `..\controller_discussion.txt`.
- Generated output folders are intentionally not kept in this cleaned checkout.
