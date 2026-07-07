# PRISMS Workspace

This repository now contains two separate PRISMS project areas:

```text
PRISMS/
  drone_project/       Original CoppeliaSim truncated-octahedral drone project
  wall_tool_project/   Cable-suspended wall-tool controller project
```

The repo-level `.venv`, `.git`, and editor/cache folders stay at the workspace
root. Project code and docs live in the two folders above.

## Install

From the PRISMS workspace root:

```powershell
python -m pip install -r requirements.txt
```

For only one project, install from that project folder instead:

```powershell
cd drone_project
python -m pip install -r requirements.txt

cd ..\wall_tool_project
python -m pip install -r requirements.txt
```

## Run

Drone/CoppeliaSim project:

```powershell
python drone_project\scripts\launchers\run_two_drone_position_ui.py
```

Wall-tool controller:

```powershell
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_controller.py
.\.venv\Scripts\python.exe wall_tool_project\run_wall_tool_coppeliasim.py
```
