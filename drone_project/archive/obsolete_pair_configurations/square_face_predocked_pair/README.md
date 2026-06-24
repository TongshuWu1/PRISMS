# Square-Face Pre-Docked Pair

This configuration starts from a geometry-only docking baseline. It places two
complete truncated-octahedral Crazyflie modules upright at the same nominal
altitude, with opposing vertical square/diamond cage faces separated by a
small clearance.

It now also includes a separate dynamic Space-thrust scene for first physics
checks. The dynamic scene is still not a latch model; it is a two-body thrust
smoke test.

## Files

```text
square_face_predocked_pair/
  README.md
  controller/
    magnetic_docking_pair_test.py
    space_thrust_pair_test.py
  model/
    predocked_square_face_pair.ttm
    predocked_square_face_pair_metadata.json
    predocked_square_face_pair_space_thrust_plant.ttm
    predocked_square_face_pair_space_thrust_metadata.json
  scene/
    predocked_square_face_pair_inspection.ttt
    predocked_square_face_pair_space_thrust.ttt
  scripts/
    generate_predocked_square_face_pair.py
    generate_space_thrust_pair_scene.py
    run_magnetic_docking_pair_test.py
    run_space_thrust_pair_test.py
```

## Generate

From the project root:

```powershell
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\generate_predocked_square_face_pair.py
```

The default docking face is `pos_xy_to_neg_xy`. Drone A's `+XY` square face is
opposed to Drone B's `-XY` square face, so the two drones are placed diagonally
in the world `XY` plane. Both drones remain upright and share the same yaw,
following the planar ModQuad-style docking assumption while using the actual
truncated-octahedral square face.

## Color

Use `model/predocked_square_face_pair.ttm` or
`scene/predocked_square_face_pair_inspection.ttt` for inspection. The generator
uses the same CoppeliaSim-native coloring method as the single-drone model:
separate mesh shapes plus `sim.setShapeColor()`.

- Crazyflie body: gray-green
- carbon-fiber cage rods: black
- corner connectors: red
- forward propellers, local `+X`, indices `0` and `2`: red
- rear propellers, local `-X`, indices `1` and `3`: black

## Dynamic Space-Thrust Test

Start CoppeliaSim with the ZMQ remote API server running, then from the project
root:

```powershell
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\run_space_thrust_pair_test.py
```

This launcher regenerates `scene/predocked_square_face_pair_space_thrust.ttt`,
loads it, and opens a small control window. Focus that window:

- hold `Space`: apply the same collective thrust to both drones
- release `Space`: thrust command returns to idle
- `R`: reset both drones to the pre-docked pose
- `Esc` or `Q`: quit

The low-level controller is the same body-rate/motor-mixer plant used by the
single-drone test, but instantiated twice: one controller state and four motor
outputs per drone. The eight propeller meshes spin from those motor speeds.

Useful options:

```powershell
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\run_space_thrust_pair_test.py --show-collision
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\run_space_thrust_pair_test.py --space-thrust-scale 1.6
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\run_space_thrust_pair_test.py --log-csv
```

## Magnetic Docking Test

Run:

```powershell
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\run_magnetic_docking_pair_test.py
```

This uses the same dynamic two-drone plant, but starts with a larger default
connector face gap of `30 mm` so the simplified collision connector spheres do
not begin interpenetrating. The controller then applies connector-level docking
forces:

- four magnet pairs on the selected square face
- pre-latch saturated spring-damper attraction inside the capture radius
- automatic latch when all four connector pairs meet distance, relative-speed,
  and opposing-face angle limits
- stronger virtual latch after docking
- break limits for connector force, net latch force, and net latch torque

Controls:

- `Space`: apply thrust to both drones
- `M`: enable/disable magnet forces
- `L`: manually release the latch
- `R`: reset both drones
- `Esc` or `Q`: quit

Useful tuning examples:

```powershell
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\run_magnetic_docking_pair_test.py --show-collision
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\run_magnetic_docking_pair_test.py --capture-radius 0.08 --magnet-stiffness 8.0
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\run_magnetic_docking_pair_test.py --connector-break-force 0.5 --break-torque 0.04
.\.venv\Scripts\python.exe configurations\square_face_predocked_pair\scripts\run_magnetic_docking_pair_test.py --log-csv
```

The latch is a virtual force constraint, not a permanently welded CoppeliaSim
assembly. That is deliberate for controller research: capture, latch, and break
events are explicit and logged.
