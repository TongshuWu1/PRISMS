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

Generate trajectory-error and efficiency plots:

```powershell
python wall_tool_sim\plot_analysis.py
```

## Concept

- A fixed anchor and spool at the top of the wall carries the payload weight.
- The passive payload/tool is inside a custom straight hex-ended payload body, not a truncated-octahedral cage.
- Two manipulator drone modules dock to the left and right payload hex faces, so the payload and drones form a flat same-surface line.
- In the nominal flat docked pose, the left drone thrust axis points up-right and the right drone thrust axis points up-left.
- Each manipulator drone is modeled as 50 g with 150 gf aggregate thrust authority.
- The payload/tool module is modeled as 75 g.
- The manipulator drones do not hover the system; they inject wall-plane force to swing and stabilize the suspended tool head.
- The spool controls cable length velocity, so the system can reach wall targets with lower hover energy than a fully airborne tool carrier.

## Model

- The suspended assembly is simulated as wall-plane translation plus one planar tilt state.
- Total suspended mass is the 75 g passive payload/tool module plus two 50 g manipulator drones, so gravity acts on 175 g total.
- Each manipulator drone can generate up to 150 gf, modeled as `0.150 * g = 1.47 N` of aggregate thrust along its tilted thrust axis.
- The cable attaches to the top of the hex-ended payload body, so cable force can create torque about the tool center.
- The spool state is paid-out cable length, not tool position.
- The spool actuator commands cable velocity and has a maximum reel speed.
- Cable tension is a unilateral spring-damper force toward the anchor: it can pull, it cannot push, it can go slack, and it saturates at a finite maximum tension.
- Cable damping is modeled as a bounded Kelvin-Voigt term, `T = k stretch + clamp(c stretch_dot)`, plus the controller's pretension floor. The damping force is capped so fast reel/payload relative motion damps oscillation without creating unrealistic tension spikes.
- The cable is initialized with static pre-tension instead of starting slack, matching the intended suspended-anchor setup.
- The controller does not read perfect simulator truth. It reconstructs the tool center from cable angle, spool encoder length, cable tension, and IMU tilt/rate.
- Cable stretch is estimated from the cable spring-damper model, `T = k stretch + c stretch_dot`, using the tension sensor and spool/angle kinematics.
- Cable stretch is estimated with the same bounded damping law used by the plant, so the sensor-side line-length reconstruction stays consistent with the simulated cable tension.
- The reference generator is guarded by a cable-aware speed governor; it slows down when tracking error, thrust use, allocation residual, or poor cable geometry indicates the path is becoming dynamically risky. This is important because cable-efficient motion must wait for the reel and suspended dynamics instead of forcing the drones to chase an aggressive reference.
- References are kept away from the near-anchor singular region where a tiny cable length would make the wall-plane model physically unrealistic.
- The controller is now organized in pendulum coordinates, not Cartesian force tracking. The controlled coordinates are cable length `L`, cable angle `theta`, and docked-body tilt.
- The spool is the primary radial actuator. For upward/radial-in motion it reels in and carries most of the vertical load; for downward/radial-out motion it maintains pretension and lets gravity pull the assembly down as cable is paid out.
- The drones are primarily tangential pendulum actuators. Their job is to swing the payload toward the desired cable angle, hold the cable angle once the tool reaches the target, and regulate the docked assembly tilt.
- Tilt regulation is scheduled. During transport, the drones do not try to keep the payload level; they only damp angular rate and reject enough cable torque to avoid uncontrolled spin. After the tool reaches the commanded spot and the estimated velocity is small, a hold controller levels the body.
- Drone allocation matches tangential force and body torque, with a soft radial-length task when the line is too short. This avoids the failure mode where tilted thrust satisfies swing/torque commands but accidentally pulls the tool back toward the anchor.
- Cable tension is computed from a radial pendulum policy. Upward moves use the radial equation and a high cable-support floor so the reel lifts the system; lowering moves drop to a pretension floor so gravity supplies the downward motion without allowing slack.
- Hold mode is latched with hysteresis after arrival. This avoids unstable switching between transport and hover control when the tool has small residual error around the target.
- Hold allocation uses a geometry-dependent force/torque balance: good cable geometry prioritizes leveling tightly, while shallow cable geometry relaxes the torque residual scale so the saturated tilted drones settle instead of entering a slow hover oscillation.
- The tool head is colocated with the center of the payload module in this 2-D model.
- Click commands are converted into a moving Cartesian reference with position, velocity, and acceleration.
- Single-click control creates a quintic straight-line reference from the current payload position to the clicked point.
- Append mode queues clicked points as a smooth quintic waypoint trajectory.
- The length loop computes a radial length acceleration from `L_ref - L` and `Ldot_ref - Ldot`. Radial-in commands become a reel-in/tension target; radial-out commands become controlled payout with a minimum pretension target.
- The angle loop computes `theta_ddot_cmd` from `theta_ref - theta` and `thetadot_ref - thetadot`, converts it into tangential acceleration, and then into the tangential drone force needed after gravity's tangential component is included.
- Payout is tension-limited. If the cable tension is too low, the reel cannot freely dump cable; it either holds payout or reels in to recover tautness.
- During commanded lowering, low-tension payout is allowed only as a slow gravity-following motion; the taut-cable constraint still prevents the displayed system from operating with a slack cable.
- The manipulator drone cages are projected from the actual truncated-octahedron vertex set: all permutations of `(0, +/-1, +/-2)`, with 6 square faces and 8 hex faces.
- The payload drawing is a straight body with left/right hexagonal docking faces.
- Black docking seams mark the payload hex faces that mate to the manipulator hex faces.

## Feedback Assumptions

- Anchor-side feedback provides cable angle and cable tension.
- The spool motor encoder provides cable payout length and payout velocity.
- Manipulator-module IMU feedback provides the docked assembly planar tilt and tilt rate.
- The controller receives no direct wall-plane tool position measurement.
- Ground-truth position, velocity, and actual actuator forces are still stored for plotting and evaluation, but the controller does not use those perfect values directly.

## UI

- Normal click replaces the current command with one smooth straight-line move.
- Turn on `Append` and click multiple points to follow a smooth queued trajectory.
- Turn on `Draw`, drag a path on the wall, and release to convert the stroke into one continuous smooth curve. The drawn stroke only eases at the beginning and end, not at every intermediate point.
- Main wall view stays clear of telemetry text.
- The teal solid line is the true tool-head trajectory.
- The red dotted line is the desired tool-head trajectory already issued by the reference generator.
- The red dashed line is the remaining future reference path, and the blue dot is the current moving reference.
- Right panel shows only realistic live feedback and trajectory error.
- The live feedback block contains anchor cable angle/rate, cable tension, spool encoder payout length/velocity, manipulator IMU tilt/rate, and live efficiency estimates.
- Live efficiency estimates show cable vertical support, drone vertical support, drone induced-power proxy relative to no-cable hover, and spool mechanical power.
- The live trajectory-error plot compares sensor-estimated tracking error with simulator ground-truth error for evaluation.
- Bottom controls provide pause/play, reset, path/trace clear, append mode, draw mode, playback speed, trace toggle, target toggle, path toggle, and force-vector toggle.

## Analysis Plots

- `plot_analysis.py` saves PNG plots and CSV logs under `wall_tool_sim\output`.
- Trajectory error is the difference between the true tool-head position and the moving desired tool-head reference.
- Load-sharing efficiency is shown as cable vertical support and drone vertical support normalized by total suspended weight.
- Drone energetic efficiency uses an induced-power proxy, `P_index = f_left^(3/2) + f_right^(3/2)`, normalized by the no-cable tilted-drone hover baseline `2 * (mg / (2 cos(alpha)))^(3/2)`.
- Tilt and torque plots show whether trajectory error is caused by translation limits or by attitude/wrench infeasibility.
- Spool work is estimated from cable tension and commanded spool velocity; positive work is counted when the spool reels in under tension.
