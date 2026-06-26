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

The root run file launches CoppeliaSim when needed, waits for the ZMQ remote
API server, regenerates the scene, starts simulation, and runs the default
dynamic plant:

- one thicker integrated rectangular cage payload with mass/inertia,
- two cylindrical side motors that apply force and torque at the canted motor frames,
- orange force-vector arrows at the motor axes,
- propeller spin joints driven by motor angular speed,
- taut unilateral cable tension from the anchor/reel to the payload mount,
- a visible payload hook/eyelet at the cable mount,
- a pen toolhead that stamps ink on the wall in work/contact mode.

For visual comparison with the original 2D run, use:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --plant-mode mirror
```
