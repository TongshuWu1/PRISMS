# Wall Tool 3D

CoppeliaSim plants for the cable-supported inspection research platform.

```text
coppeliasim_vector_tool/  current non-contact vector-thrust plant and runner
coppeliasim_wall_tool/    preserved legacy pen/contact simulator
scene/                    generated CoppeliaSim scenes and payload models
tests/                    controller-boundary and sensor tests
```

Run the current remake from the repository root:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim_ui.py
```

This opens the facade trajectory planner and live CoppeliaSim control panel.
For a scripted validation stage instead, run:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py --scenario hover
```

Then use `point`, `turns`, and `mission` for progressively harder validation.
See [coppeliasim_vector_tool/README.md](coppeliasim_vector_tool/README.md) for
the independent validation plant, five-channel controller boundary,
sensor-fusion estimator, hardware-calibration contract, physical limitations,
and acceptance gates.

The legacy `coppeliasim_wall_tool` code and its pen scene remain available for
historical comparison, but the main launcher no longer imports that plant.
