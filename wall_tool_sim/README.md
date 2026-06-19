# PRISMS Wall Tool Simulator

Standalone 2-D simulator for the cable-suspended wall tooling idea.
This folder is separate from the CoppeliaSim flight-control work.

## Run

From the PRISMS repo root:

```powershell
python wall_tool_sim\wall_tool_ui.py
```

Batch render a PNG without opening a window:

```powershell
python wall_tool_sim\wall_tool_ui.py --no-show --save-fig wall_tool_sim\output\wall_tool_ui.png
```

## Concept

- A fixed anchor and spool at the top of the wall carries the payload weight.
- The passive payload/tool is inside its own truncated-octahedral PRISMS cage.
- Two manipulator drone modules dock to the payload through side hex faces.
- The left drone thrust axis points up-right and the right drone thrust axis points up-left.
- Each manipulator drone is modeled as 150 g with 300 gf aggregate thrust authority.
- The payload/tool module is modeled as 100 g.
- The manipulator drones do not hover the system; they inject wall-plane force to swing and stabilize the suspended tool head.
- The spool controls cable length velocity, so the system can reach wall targets with lower hover energy than a fully airborne tool carrier.

## Model

- The suspended assembly is simulated as a Cartesian point-mass tool head under force integration.
- Total suspended mass is the 100 g passive payload/tool module plus two 150 g manipulator drones, so gravity acts on 400 g total.
- Each manipulator drone can generate up to 300 gf, modeled as `0.300 * g = 2.94 N` of aggregate thrust along its tilted thrust axis.
- The spool state is paid-out cable length, not tool position.
- The spool actuator commands cable velocity and has a maximum reel speed.
- Cable tension is a unilateral spring-damper force toward the anchor: it can pull, it cannot push, it can go slack, and it saturates at a finite maximum tension.
- The controller closes spool velocity feedback on actual anchor-to-tool distance and cable tension, so it can reel in to build a target load-sharing tension.
- The default load sharing asks the cable to carry about 70% of static weight; if the cable/spool tension limit is too low, the two manipulator drones allocate extra vertical thrust to carry the missing support.
- The tool head is colocated with the center of the payload module in this 2-D model.
- Click commands are converted into a moving Cartesian reference with position, velocity, and acceleration.
- Single-click control creates a quintic straight-line reference from the current payload position to the clicked point.
- Append mode queues clicked points as a smooth quintic waypoint trajectory.
- Drone control is acceleration control: the controller computes `a_cmd = a_ref + Kp e_p + Kd e_v`, subtracts the force currently provided by cable tension and gravity, then allocates the remaining 2-D force through the two tilted drone axes.
- Spool control is velocity control: the controller computes `length_dot_cmd = length_dot_ref + K(length_ref - length)` and saturates it by spool speed.
- The cage drawing is projected from the actual truncated-octahedron vertex set: all permutations of `(0, +/-1, +/-2)`, with 6 square faces and 8 hex faces.
- Red face outlines show the hex faces used for payload-drone docking.

## UI

- Normal click replaces the current command with one smooth straight-line move.
- Turn on `Append` and click multiple points to follow a smooth queued trajectory.
- Main wall view stays clear of telemetry text.
- The teal solid line is the true tool-head trajectory.
- The red dotted line is the desired tool-head trajectory already issued by the reference generator.
- The red dashed line is the remaining future reference path, and the blue dot is the current moving reference.
- Right panel shows state, reference acceleration, actual anchor distance, paid-out spool length, cable stretch/slack, spool velocity command, drone acceleration command, gravity, max thrust, cable tension, thrust, allocation residual, and saturation.
- Bottom controls provide pause/play, reset, path/trace clear, append mode, playback speed, trace toggle, target toggle, path toggle, and force-vector toggle.
