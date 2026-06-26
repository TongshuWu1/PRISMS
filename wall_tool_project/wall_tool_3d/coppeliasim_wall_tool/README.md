# CoppeliaSim Wall Tool

This package creates the 3D CoppeliaSim version of the PRISMS wall-inspection
tool. The default run mode is dynamic: CoppeliaSim owns the payload motion while
the existing 2D NMPC supplies side-motor thrust, desired cable tension, and reel
velocity commands.

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

This opens the native 2D wall-tool UI as a controller/spectator by default.
Click the wall, use append mode, or draw a path in that same 2D UI to send
targets to the live CoppeliaSim plant. The UI plots actual 3D pen error and
cable tension, and shows payload position, pen position, motor thrust, RPM,
controller mode, MPC status, and a CoppeliaSim sensor block. The default
command is open-ended; close the UI or press Ctrl+C to stop.

Useful variants:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --no-control-ui --duration 2
```

For the old kinematic comparison view:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --plant-mode mirror
```

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
/pen_barrel
/pen_nib
/pen_tip
/inspection_target
```

The dynamic run bridge stamps `/ink_dot_####` cylinders on the wall when the
actual pen tip is near the wall and the controller is in work/contact mode.

## Dynamic Plant

The 3D plant currently includes:

- cable reel velocity command changes paid-out cable length,
- cable tension acts between `/anchor_reel_mount` and `/wall_tool_cable_mount`,
- a visible top hook/eyelet makes the payload read as hanging from the cable,
- the visible payload is a thicker rectangular rod-and-node cage in the same visual style as the truncated-octahedra drone cage,
- side motor thrusts act at cylindrical motor frames along the same canted axes,
- orange arrows visualize the live motor force vectors,
- motor angular speed drives the propeller spin joints,
- a wall-normal standoff guide holds the planar controller in the inspection plane.

The next controller step is replacing the adapted 2D state sync with a native
3D estimator/controller interface.
