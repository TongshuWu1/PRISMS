# MIESC Research Note

Working title: **Mixed-Input Energy-Shaping Control for Cable-Suspended Facade Robots**

## Problem

For skyscraper cleaning or inspection, a controller should not be judged only by
geometric path tracking. The useful task is contact-gated coverage: the tool must
stay in the work region, maintain contact force, stay below blur/speed limits,
avoid high body rate, keep cable tension positive, and use the cable for load
support instead of making the drones behave like a free hover system.

The hard part is hybrid actuation:

- the reel is naturally a velocity/length actuator,
- the drone pair is naturally an acceleration/force actuator,
- the cable is most efficient when it carries vertical load,
- lane-end reversals create tracking and speed spikes that invalidate cleaning.

## Proposed Controller

MIESC separates the hybrid system by actuator type instead of collapsing
everything into a Cartesian tracking loop:

- reel input: radial cable-length velocity,
- drone input: tangential wall-plane acceleration/force and body torque,
- allocation: bounded drone thrusts plus cable tension,
- safety/task layer: contact-valid reference governor.

The tangential command uses a storage function over tangential position/velocity
tracking error. A practical CLF projection prevents the command from injecting
persistent swing energy while still allowing decisive motion when velocity is
near zero and position error is large.

The nominal path is also generated as a constrained quintic reference. Segment
duration is chosen so that the commanded move respects speed, acceleration, and
jerk limits before any feedback correction is applied.

The coverage reference is boundary-aware: at the cleaning bay edges, corner
velocities are projected to avoid leaving the facade work region. This matters
because a smooth spline that cuts outside the work bay can look good
geometrically while invalidating the actual cleaning task.

Governor inputs:

- queued waypoint geometry and distance to the active lane-end target,
- turn alignment between incoming and outgoing path segments,
- predicted cable vertical efficiency at the active target,
- measured tracking-error ratio before the tracking limit is reached,
- measured tool-speed ratio before the cleaning/inspection speed limit is reached.

Governor output:

- a reference-speed cap in `[reference_speed_min, 1]`.

Important implementation detail: when the trajectory clock is slowed, reference
velocity is scaled by the same factor and reference acceleration is scaled by the
square of that factor. This keeps feed-forward terms physically consistent.

## Why This Is Publishable

The contribution is not simply another PID tuning pass. The research claim is:

> For cable-suspended facade robots with mixed reel velocity control and drone
> acceleration control, radial-tangential energy shaping plus a contact-valid
> reference governor can improve task-valid coverage and cable-use efficiency
> without requiring a full nonlinear MPC solve at every servo step.

This gives a clean experimental story:

- compare Cartesian PD, reactive-only speed scaling, CV-CERG, and MIESC,
- evaluate contact-gated coverage, valid-contact fraction, tracking error,
  speed-limit margin, payload jerk, body-rate oscillation, reel acceleration,
  cable support, drone power ratio, swing energy, slack, and thrust margin,
- show that mixed-input radial/tangential control uses the reel and drones in
  physically distinct roles while the governor removes lane-reversal/contact
  violations.

## Current Evidence

Latest full run: `output/sessions/20260624_001654/`

Metrics on the `6.0 m x 6.0 m` wall:

- duration: `373.53 s`
- contact-gated coverage: `1.000`
- valid-contact fraction: `1.000`
- RMS tracking error: `0.0496 m`
- p95 tracking error: `0.0677 m`
- max tracking error: `0.0769 m`
- max payload speed: `0.3381 m/s` under the `0.36 m/s` limit
- p95 body rate: `0.206 rad/s`
- p95 payload jerk: `19.76 m/s^3`
- p95 spool acceleration: `0.380 m/s^2`
- mean swing energy: `0.000026 J`
- mean cable support fraction: `0.708`
- mean drone power ratio: `0.333`
- max thrust fraction: `0.776`
- slack fraction: `0.000`
- thrust-limit active fraction: `0.000`

Previous clean large-wall reference run:

- duration: `373.62 s`
- valid-contact fraction: `1.000`
- RMS tracking error: `0.0493 m`
- mean cable support fraction: `0.693`
- mean drone power ratio: `0.355`
- p95 spool acceleration: `0.500 m/s^2`

## Next Research Experiments

1. Ablation: Cartesian PD, MIESC without CLF projection, MIESC without
   boundary-aware smoothing, MIESC without governor, full MIESC.
2. Disturbance tests: restore wind and normal-force gusts after the no-wind
   nominal controller is stable.
3. Robustness sweep: cable stiffness/damping, payload mass, thrust limit,
   lane spacing, wall size, and anchor height.
4. 3D extension: replace the normal-contact proxy with constrained allocation
   for wall-normal force, drone attitude, cable tension, and tool torque.
5. Learning layer: learn residual speed-cap corrections from failed segments,
   but keep the governor constraints as the safety envelope.
