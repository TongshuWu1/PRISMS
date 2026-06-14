# Simulation

CoppeliaSim-specific plant and interaction code.

- `magnetic_docking.py`: reusable drone discovery, connector geometry, face-compatible magnetic capture, and finite-force latch modeling.

Face docking is physical-only. The code does not teleport bodies or create
hidden loop closures. A compatible face pair first feels a weak,
short-range magnetic capture force. Once the face corners are close enough, each
corner pair switches to a stiffer spring-damper latch with a finite force limit
and break distance. The default latch rest distance is 4 mm, matching the cage
collision node diameter, so the corner nodes are pulled into contact without
requiring geometric overlap.

This folder is not for flight-control laws. It models what the simulator does around the controller.
