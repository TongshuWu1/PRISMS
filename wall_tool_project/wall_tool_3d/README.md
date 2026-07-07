# Wall Tool 3D

CoppeliaSim layer for the wall-inspection and pen-on-wall prototype.

```text
coppeliasim_wall_tool/   scene generator, run bridge, remote API helpers
scene/                   generated .ttt scene and .ttm payload model
```

Run from the repo root:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py
```

The default run opens the native 2D wall-tool UI as a controller/spectator.
Click the wall, use append mode, or draw a path in the 2D UI to command the
CoppeliaSim payload; the same UI shows live 3D feedback for payload/pen
position, cable tension, tracking error, thrust, and RPM. The default command
is open-ended; use `--no-control-ui --duration 2` for batch smoke tests.
Batch smoke tests print a realtime factor and fail below the default
`--min-realtime-factor 0.5`.
The desired drawing path is rendered on the CoppeliaSim wall by default, while
actual ink/contact marks remain separate black dots. Batch and live UI runs also
report controller efficiency metrics: tracking error, actuator utilization,
reel work, saturation time, and NMPC solve timing.

The root run file launches CoppeliaSim when needed, waits for the ZMQ remote
API server, regenerates the scene, starts simulation, and runs the default
dynamic plant:

- one thicker integrated rectangular cage payload with mass/inertia,
- two cylindrical side motors that apply force and torque at the canted motor frames,
- orange force-vector arrows at the motor axes,
- propeller spin joints driven by motor angular speed,
- segmented steel-cable visualization with tension-dependent sag,
- length-dependent cable stiffness/damping and payload-carried cable weight,
- 12 V encoder gearmotor reel modeled as cable line-velocity control,
- 100 Hz default plant/controller stepping with 10 Hz visual-only cable and prop refresh,
- a visible payload hook/eyelet at the cable mount,
- a pen toolhead with measured 3D wall-contact force for ink/contact validity,
- visible desired-path cylinders plus separate measured ink marks.
