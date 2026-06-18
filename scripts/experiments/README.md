# Experiments

Runnable CoppeliaSim experiments.

These scripts initialize scenes, create UI windows when needed, call the high-level and low-level controllers, call simulation helpers, log telemetry, and step CoppeliaSim.

- `single_drone_position_ui.py`: one drone with target UI.
- `two_drone_position_ui.py`: multi-drone docking UI. It discovers loaded drones, commands all/individual/assembly targets, auto-detects docked configurations from latched face connectors, treats body `+X` as the red-propeller forward direction for docking yaw, commands quick square docking from actual vertical square/diamond face geometry, stores docked module yaw offsets relative to the leader module, keeps free drones on independent cascaded control, and controls latched assemblies through one center-of-mass target with full-wrench docked motor allocation, residual telemetry, and a cyan net assembly-force arrow at the assembly center of mass.
- `two_drone_hover.py`: two-drone hover baseline without UI.
- `xbox_fpv.py`: gamepad FPV/rate experiment.

Reusable UI widgets live in `ui/`.
