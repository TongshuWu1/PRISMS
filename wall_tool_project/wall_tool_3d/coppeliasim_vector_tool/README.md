# Vector-Thrust CoppeliaSim Remake

This package is the clean non-contact inspection plant. It replaces the old
pen/contact simulator instead of modifying that legacy scene in place.

## Physical system

- One fixed roof reel and one compliant steel cable.
- One free six-degree-of-freedom payload rigid body.
- Two propellers whose thrust axes are independently positioned by 270-degree
  servos.
- A speed-controlled encoder reel with load-dependent motor dynamics.
- Variable effective drum radius, cable layering, gearbox losses, velocity
  deadband, and reversal backlash. The shaft-encoder payout is therefore not
  silently equated with true paid-out cable.
- A unilateral viscoelastic cable with termination compliance, nonlinear
  stiffening, relaxation, pulley capstan friction, distributed cable weight,
  and a first transverse vibration mode.
- Independent left/right rotor response and servo rate, acceleration, play,
  deadband, and zero-offset dynamics.
- No wall contact force, guide rail, ideal pitch constraint, or hidden Cartesian
  stabilizer.
- CoppeliaSim owns rigid-body mass, inertia, gravity, pose, and velocity.
- Python applies only modeled propeller, cable, cable-weight, passive pitch, and
  configured in-plane wind wrenches.

The experiment starts in a documented pre-armed trim condition: the reel has
preloaded the cable and both rotors are already spinning before the payload is
released. This avoids injecting an artificial motor-start transient into a
hover experiment.

## Controller boundary

The plant accepts exactly five commands:

1. left thrust,
2. right thrust,
3. reel line velocity,
4. left servo position,
5. right servo position.

The NMPC and validation plant do not share actuator or cable state equations.
The plant uses the immutable parameter contract in `validation_plant.py`; the
NMPC retains its smaller prediction model. This is intentional model mismatch,
not duplicated tuning.

The NMPC never receives CoppeliaSim Cartesian truth. The 100 Hz sensor suite
uses a cable-angle encoder, reel encoder, reel load cell, and payload IMU to
reconstruct the planar state. A seven-state covariance estimator fuses those
measurements and estimates position, velocity, pitch, pitch rate, and gyro
bias. Sensor noise, quantization, filtering, and sample-and-hold are explicit.
The servos are commanded by position and expose no extra servo encoder
measurement to the controller. Physics runs at 200 Hz; the five drive commands
are held between sensor frames.

The solver has no backup controller. Invalid timestamps, missing objects,
failed optimization, cable slack, excess attitude, or 3D escape fail loudly.

## Run

From the repository root:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim_ui.py
```

The interactive UI is the normal trajectory-planning workflow:

- **Click / drag**: each short click appends another numbered waypoint to one
  connected trajectory. Holding and dragging appends a spacing-filtered smooth
  stroke to that same path, without replacing earlier points. Right-click or
  **Undo** removes the last point.
- **Freehand only** ignores isolated clicks and accepts drag strokes only. All
  user paths are bounded to 48 points before reaching the controller.
- **Coverage preset**: creates a serpentine inspection route that can be edited.
- Set path speed, nonzero corner speed, and the Coppelia camera view, then press
  **Start trajectory**.
- **Pause** freezes synchronous physics and controller time. **Resume** continues
  from exactly that state. **Stop** ends the worker but leaves CoppeliaSim open.
- The live facade display rotates the payload with its measured pitch, draws the
  cable to its attachment, and shows both vector-thrust directions.
- Live telemetry shows actual/estimated/reference position, actual/estimated
  velocity, roll/pitch/yaw and angular rates, cable angle/rate and physical
  tension, wall-normal drift, reel payout, thrust, servo angles, remaining path
  state, NMPC status, and maximum tracking error.

The UI loads the validated saved detailed scene by default. Scene rebuilding is
an explicit slow developer option and is not needed when planning new paths.

For non-interactive validation runs:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --scenario hover
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --scenario point
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --scenario turns
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --scenario mission
```

The four stages default to 12 s, 60 s, 90 s, and the complete facade mission.
Use `--duration SECONDS` only for a deliberately shortened shakedown. Scene
generation is on by default and saves:

```text
wall_tool_3d/scene/vector_thrust_inspection_scene.ttt
wall_tool_3d/scene/vector_thrust_payload.ttm
```

JSON evaluation summaries are written under `wall_tool_3d/vector_thrust_runs/`.
A normal visible launch detects and cleanly replaces a stale headless validation
server on port 23000, so an earlier `--headless` run cannot make the GUI launch
silently attach to an invisible process.

For unattended long validation, add `--headless`. This avoids desktop dialogs
pausing synchronous stepping, and a headless process launched by the runner is
closed after its summary is written. Interactive/visible sessions are never
closed by batch cleanup.

## Validation-plant profiles and calibration

Normal runs use `datasheet-independent-v1`. It is deliberately labeled
`calibrated=false`: its component values are datasheet values plus documented
engineering assumptions, not fabricated hardware measurements.

Create a profile template for a real identification campaign:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_validation_profile.py template `
  --output hardware_data\payload_A_validation_profile.json
```

Record the raw cable extension/tension, reel command/encoder/paid-out length,
rotor command/thrust, and servo command/angle experiments; identify every
parameter in the four profile blocks; hash the immutable raw dataset; then set
`calibrated=true` and fill `hardware_id`, `recorded_utc`, and
`raw_data_sha256`. Validate it before simulation:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_validation_profile.py validate `
  --profile hardware_data\payload_A_validation_profile.json
```

Run with the identified plant only by selecting it explicitly:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py `
  --scenario mission --validation-profile calibrated `
  --calibration-file hardware_data\payload_A_validation_profile.json
```

Calibrated mode has no nominal fallback. A missing file, placeholder profile,
wrong schema version, missing provenance, or invalid parameter fails before
CoppeliaSim starts.

## Acceptance gates

- wall-normal drift no more than 25 mm,
- roll/yaw no more than 5 degrees,
- pitch no more than 5 degrees (or the NMPC limit, if lower),
- tension inside the reel/load-cell range with zero slack samples,
- non-hover RMS error no more than 25 mm, p95 error no more than 50 mm, and
  maximum error no more than 120 mm,
- sensor-fusion p95 position error no more than 20 mm,
- solver deadline misses no more than 2% on statistically meaningful runs.
  Short shakedowns are allowed at most two misses' worth of sample resolution.

Use `--no-strict` only while diagnosing a known failed stage; it does not
activate a fallback or alter the controller.

The plant is now suitable for model-mismatch validation, but it is still not a
full Cosserat-rod or finite-element wire-rope solver. Pulley contact, individual
wire bending/torsion, spool cross-lay geometry, cable-wall collision, and wire
fatigue remain outside the model and must not be claimed as validated physics.
