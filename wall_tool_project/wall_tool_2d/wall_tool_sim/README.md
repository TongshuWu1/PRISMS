# PRISMS Wall-Plane Inspection Plant

This simulator is the physics testbed for the single-tether facade inspection
robot. It is intentionally used before transferring the controller to
CoppeliaSim.

## Physical layout

- One roof anchor and speed-controlled reel support a 175 g suspended assembly.
- A single steel cable terminates at the top of a passively stabilized payload.
- Two 50 g propeller modules are mounted symmetrically on the payload sides at
  approximately center-of-mass height.
- Each propeller is rotated directly by an independent 270-degree position
  servo. The servo shaft is normal to the wall, so the thrust axis rotates in
  the wall-parallel x-z plane.
- The payload carries an inspection sensor; no cleaning contact force is
  commanded or simulated.

## Plant fidelity

- Cartesian payload translation and planar attitude are integrated at 5 ms.
- The payload attitude remains dynamic. A finite Y-bridle restoring model is
  used instead of an ideal no-tilt constraint.
- The steel cable is unilateral and compliant, with length-dependent axial
  stiffness, damping, mass, and payload-supported cable weight.
- The reel uses its measured gearmotor speed/torque envelope and finite velocity
  response.
- Propeller thrust has a finite time constant.
- Thrust, reel-speed, and tilt-servo setpoints pass through continuous command
  rate limiters instead of stepping at the NMPC update boundary.
- Tilt servos have second-order dynamics, 270-degree travel,
  rate/acceleration limits, finite PWM command resolution, and explicit
  saturation reporting.
- Cable-angle and reel encoders, the load cell, and the payload IMU include
  sample rate, quantization, filtering, and noise.
- A hobby servo's internal potentiometer is not exposed to the flight
  controller. Servo angle/rate states used by NMPC are propagated from the PWM
  command and nominal servo model; actual shaft angles remain plant-only. Small
  left/right mechanical zero errors keep the plant and predictor from being
  artificially identical.
- Controller position is reconstructed from cable angle, cable payout,
  load-cell-based stretch estimate, and IMU attitude. Ground truth is used only
  for evaluation.
- A deterministic wall-plane wind field provides mean, gust, and edge-scaled
  disturbance components.

The external CoppeliaSim plant interface intentionally raises until that scene
implements the two independent tilt-servo command channels. Fixed-axis
substitution is not used.

## Run

From the repository root:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_controller.py --mode quick --duration 12
```

Use `--mode log` for the long serpentine mission and complete diagnostics.
