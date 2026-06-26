# Wall Tool Project

Cable-suspended wall inspection and drawing prototype for PRISMS.

The project is split into two layers:

```text
wall_tool_2d/
  cable_hybrid_controller/    validated 2D NMPC controller, diagnostics, UI
  wall_tool_sim/              2.5D wall-plane simulator and geometry helpers

wall_tool_3d/
  coppeliasim_wall_tool/      CoppeliaSim scene generator and 3D run bridge
  scene/                      generated CoppeliaSim scene/model outputs
```

Root run files stay at this level so PyCharm remains simple:

```text
run_wall_tool_controller.py   2D Qt/Tk/Matplotlib/log/quick controller runner
run_wall_tool_coppeliasim.py  3D CoppeliaSim wall-tool scene runner
```

## Run 2D Controller

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_controller.py --mode qt
```

Useful modes:

```text
--mode qt       hybrid Matplotlib scene + PyQtGraph evaluation UI
--mode tk       Tkinter fallback simulator UI
--mode ui       pure Matplotlib simulator UI
--mode log      full logged controller session
--mode quick    short smoke test
```

Smoke check:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_controller.py --mode quick --duration 12
```

Tune the selected controller in:

```text
wall_tool_2d/cable_hybrid_controller/config.py
```

## Run 3D CoppeliaSim Scene

The 3D runner launches CoppeliaSim if it is not already listening, regenerates
the scene, starts the simulation, and runs the wall tool as a dynamic
CoppeliaSim plant. The payload is a single integrated body, the two side motors
apply thrust at the same canted axes as the 2D model, the reel enforces a taut
non-elastic cable approximation, and the propeller joints spin from motor RPM.
It also opens the native 2D wall-tool UI as a controller/spectator: click the
wall, use append mode, or draw a path in the 2D UI, and the live 3D
CoppeliaSim payload follows while tension, pen error, motor RPM, and trace
update from CoppeliaSim feedback.

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py
```

That default command is open-ended: close the 2D controller board or press
Ctrl+C in the terminal to stop it.

For a batch smoke test without the controller board:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --no-control-ui --duration 2
```

If CoppeliaSim is installed somewhere else:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --coppeliasim-exe "C:\path\to\coppeliaSim.exe"
```

Generated outputs:

```text
wall_tool_3d/scene/wall_tool_pen_scene.ttt
wall_tool_3d/scene/wall_tool_payload_model.ttm
```

Use the old visual playback mode for comparison:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --plant-mode mirror
```

The dynamic mode still reuses the 2D NMPC as the command source, but it syncs
position, velocity, pitch, reel length, and tension from the CoppeliaSim body
before each solve. The next engineering step is a controller pass that treats
the CoppeliaSim plant as the primary estimator/plant instead of adapting the
2D simulator state.

## Active Model

The current robot is one integrated rectangular-cage wall-tool payload with:

- two fixed side motors canted about 35 degrees from vertical,
- a top cable mount connected to an anchor/reel,
- a pen/toolhead at the wall face,
- 2D controller state `x = [p_x, p_z, v_x, v_z, phi, omega, l]^T`,
- command `u = [F_L, F_R, T, ldot_cmd]^T`.

The selected control law is `tool_head_nmpc`.
