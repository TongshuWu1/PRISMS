# Wall Tool 2D

Validated 2D/2.5D controller layer.

```text
cable_hybrid_controller/   controller facade, tuning, diagnostics, Qt/Tk UIs
wall_tool_sim/             wall-plane simulator and geometry helpers
```

Run from the repo root through the project-level launcher:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_controller.py --mode qt
```

This layer remains the source of truth for the current NMPC formulation while
the 3D CoppeliaSim scene is being brought up.

