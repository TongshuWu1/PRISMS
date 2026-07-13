# CoppeliaSim Wall Tool

This package creates the 3D CoppeliaSim version of the PRISMS wall-inspection
tool. The default run mode is dynamic: CoppeliaSim owns the payload motion while
the shared sensor-cascade supplies side-motor thrust and reel-velocity commands;
cable tension is regulated from the measured load-cell signal rather than
applied as an ideal force command. Controller feedback comes through a hardware-like sensor
estimator rather than exact CoppeliaSim position and velocity.

## Run

From the repo root:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py
```

The launcher starts CoppeliaSim when it is not already listening on
`localhost:23000`. Use `--no-launch-coppeliasim` if you want it to fail instead
of launching the GUI, or `--coppeliasim-exe` if your install path differs.

The default run mode is:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --plant-mode dynamic
```

The default `--feedback-mode sensor` simulates the proposed real sensor suite:

- reel motor encoder for paid-out cable length and reel speed,
- absolute cable-angle encoder at the top guide,
- inline reel/load-cell cable tension,
- raw payload IMU angular rate and specific force. Attitude is reconstructed
  with gyro integration, accelerometer correction, and gyro-bias estimation.

For estimator comparison, exact simulator state remains available explicitly:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --feedback-mode ground-truth
```

Encoder resolution and sensor-noise experiments can be changed without editing
code using `--reel-encoder-counts-per-output-rev`,
`--cable-angle-encoder-counts-per-rev`, and `--no-sensor-noise`.

This opens the native 2D wall-tool UI as a controller/spectator by default.
Click the wall, use append mode, or draw a path in that same 2D UI to send
targets to the live CoppeliaSim plant. The UI plots actual 3D pen error and
cable tension, and shows payload position, pen position, motor thrust, RPM,
controller mode, controller status, and a CoppeliaSim sensor block. The default
command is open-ended; close the UI or press Ctrl+C to stop.

Useful variants:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --no-control-ui --duration 2
```

Batch multi-corner paths can be exercised directly with:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --no-control-ui --duration 55 --path-points "0.8,2.0;0.8,1.3;-0.8,1.3;-0.8,2.0;0.0,2.0"
```

Batch runs print a loop realtime factor and fail if it drops below the default
`--min-realtime-factor 0.45`. The default CoppeliaSim/controller step is 100 Hz
(`--time-step 0.01`), while visual-only cable and propeller geometry refreshes
at 10 Hz (`--cable-visual-update-period 0.1`,
`--prop-visual-update-period 0.1`) so rendering does not throttle the plant.
The controller's desired drawing path is visible directly on the CoppeliaSim
wall by default (`--show-desired-path`) as blue path cylinders with start/end
markers. The default path preview budget is 24 segments; increase
`--desired-path-max-segments` only when you need more visual detail.

The generator saves:

```text
wall_tool_project\wall_tool_3d\scene\wall_tool_pen_scene.ttt
wall_tool_project\wall_tool_3d\scene\wall_tool_payload_model.ttm
```

## Scene Convention

- Wall plane: world `X-Z`, facade face near `Y = 0`.
- Robot/payload: in front of the wall at negative `Y`.
- Payload planar attitude: rotation about world `Y`.
- 2D simulator `(x, z)` maps to CoppeliaSim `[x, -standoff, z]`.
- Pen tip points along positive `Y` and reaches the wall face.

## Important Aliases

```text
/facade_wall
/facade_work_bay
/anchor_reel_mount
/reel_spool
/wall_tool_payload
/wall_tool_payload_cage_rod_##
/wall_tool_payload_cage_node_##
/wall_tool_left_motor_frame
/wall_tool_right_motor_frame
/wall_tool_left_motor_frame_motor_can
/wall_tool_right_motor_frame_motor_can
/wall_tool_left_motor_frame_motor_hub
/wall_tool_right_motor_frame_motor_hub
/wall_tool_left_motor_force_arrow_stem
/wall_tool_left_motor_force_arrow_head
/wall_tool_right_motor_force_arrow_stem
/wall_tool_right_motor_force_arrow_head
/wall_tool_left_propeller_spin_joint
/wall_tool_right_propeller_spin_joint
/wall_tool_cable_mount
/wall_tool_cable_mount_post
/wall_tool_cable_mount_hook
/wall_tool_cable
/wall_tool_cable_segment_##
/pen_barrel
/pen_nib
/pen_tip
/inspection_target
```

The dynamic run bridge stamps `/ink_dot_####` cylinders on the wall when the
actual pen tip is near the wall and measured 3D contact force is inside the
configured work band.
The desired command path uses `/wall_tool_desired_path_segment_##` cylinders
and `/wall_tool_desired_path_marker_##` endpoint disks, so planned path and
actual ink are visually separate.

## Dynamic Plant

The 3D plant currently includes:

- cable reel velocity command changes paid-out cable length,
- the reel is modeled as a 12 V encoder gearmotor velocity actuator: 43.8:1,
  251 RPM no-load output speed, 18 kg.cm stall torque,
- reel line velocity is clamped by spool radius and the gearmotor torque-speed envelope,
- segmented steel-cable visualization shows tension-dependent sag,
- cable tension comes only from measured stretch/stretch-rate, not a controller force floor,
- cable mass contributes a configurable payload-carried weight term,
- a visible top hook/eyelet makes the payload read as hanging from the cable,
- the visible payload is a thicker rectangular rod-and-node cage in the same visual style as the truncated-octahedra drone cage,
- side motor thrusts act at cylindrical motor frames along the same canted axes,
- orange arrows visualize the live motor force vectors,
- motor angular speed drives the propeller spin joints,
- a wall-normal guide and pen-tip contact model provide measured 3D wall contact.
- wall-normal preload and roll/yaw restraint are modeled as passive mechanical
  guide stiffness/damping, not as feedback from an unavailable position sensor.
- passive wall rollers strongly constrain pitch with a spring-damper guide;
  low wall resistance uses a smooth roller-friction law instead of discontinuous
  Coulomb stick-slip.
- physics/control and visual rendering are decoupled: cable tension is updated
  every plant step, but the segmented cable cylinders are refreshed at the
  configured visual cadence.
- terminal and live UI telemetry report controller efficiency: tracking error,
  thrust utilization, prop power index, reel mechanical work, saturation time,
  controller timing, and peak motor/reel speeds.

The estimator interface is isolated in `sensor_estimator.py`, so the simulated
sensor producers can later be replaced by real encoder, load-cell, and IMU
drivers without changing the controller interface. Wall-normal contact and the
remaining 3D attitude axes are still explicit passive plant assumptions rather
than measurements in the current sensor list.
