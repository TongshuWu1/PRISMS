# Single-Tether Vector-Thrust Inspection Controller

This package contains the active controller for the non-contact facade
inspection platform. The controlled plant has one encoder-equipped reel, one
unilateral compliant steel cable, a passively stabilized payload, and two
propellers independently rotated by 270-degree position servos.

## Active architecture

- `vector_thrust_nmpc` is the only accepted control law.
- The NMPC commands left/right thrust, left/right tilt-servo position, and reel line
  velocity over a 1.8 s prediction horizon.
- Each servo shaft is normal to the wall. Its propeller axis rotates in the
  wall-parallel x-z plane.
- Cable tension is a predicted/measured state produced by cable stretch and
  reel motion. It is not a directly commandable input.
- A finite Y-bridle stiffness/damping model stabilizes payload attitude. The
  simulator does not lock attitude to zero.
- Motor thrust, reel velocity, cable tension, and tilt-servo motion all have
  finite bandwidth. Servos also have hard travel, rate, and acceleration limits.
- NMPC targets are executed through continuous per-physics-step command ramps.
  Hard thrust, reel, and tilt-command slew constraints are enforced inside the
  horizon and again at the actuator interface; 0.075 s optimizer updates are
  never applied as discontinuous zero-order-held command jumps.
- Once the moving path is complete, a bounded measured-position integral
  correction makes the stationary NMPC offset-free against mean wind. The
  integral state is cleared whenever trajectory motion resumes.
- The controller sends finite-resolution PWM position commands. It does not
  receive the servo's internal potentiometer signal or an invented external
  angle encoder; NMPC propagates nominal servo angle/rate states from commands.
- Encoder quantization, sensor sample-and-hold, load-cell filtering, and
  explicit noise are applied before payload position/velocity reconstruction.
- The internal controller never receives ground-truth payload position or
  velocity.
- No backup controller or previous-feasible-command recovery is applied. A
  solver, parameter, measurement, or actuator-interface failure raises an
  exception with the failing condition.
- Wall-normal contact force is not part of this inspection model. The x-z model
  assumes a fixed inspection plane and records a constant stand-off only for
  visualization.

## Trajectory generation

The default mission is a serpentine facade inspection path. The geometric
reference is acceleration/jerk limited, samples upcoming curvature, and slows
before turns. Reference progress is explicitly bounded by the physics step so
it cannot silently advance faster than the commanded inspection speed.

## Run

From the repository root:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_controller.py --mode qt
```

Other modes:

```text
--mode tk       Tkinter controller UI
--mode ui       Matplotlib simulator UI
--mode quick    fail-fast point-to-point check
--mode log      full mission and generated diagnostics
```

## Failure criteria

Treat any of the following as a failed run:

- NMPC non-convergence or constraint violation;
- non-finite or out-of-range actuator command;
- cable slack or operational tension saturation;
- sustained thrust, gimbal, or reel saturation;
- payload attitude outside the inspection limit;
- tracking error outside the mission limit;
- MPC solve time above its control deadline.

The logged evaluation reports RMS, p95, maximum, and final trajectory error;
payload attitude; cable support; thrust and gimbal use; slack/saturation; and
MPC deadline misses.
