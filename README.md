Clean CoppeliaSim baseline for a truncated-octahedral Crazyflie drone with Python flight control and magnetic docking experiments.

## Folder Structure

```text
truncated_octahedral_drone/
  assets/
    meshes/
      crazyflie_cage_body_no_propellers.stl
      crazyflie_propellers_aligned.stl
  controller/
    allocation/
      geometry.py
      wrench_allocator.py
    low_level/
      rate_control.py
      body_rate.py
    high_level/
      assembly_geometric.py
      position.py
    common/
      telemetry.py
  simulation/
    magnetic_docking.py
  ui/
    plot_widgets.py
  scripts/
    generation/
      generate_drone_plant_scene.py
      spawn_hex_face_pair.py
      spawn_two_drones.py
    experiments/
      single_drone_position_ui.py
      two_drone_position_ui.py
      two_drone_hover.py
      xbox_fpv.py
    launchers/
      run_position_controller.py
      run_position_ui_controller.py
      run_hex_pair_position_ui.py
      run_two_drone_position_ui.py
      run_two_drone_hover.py
      run_xbox_fpv_controller.py
    analysis/
      analyze_telemetry.py
  model/
    truncated_octahedral_crazyflie_plant.ttm
  scene/
    body_rate_controller_demo_scene.ttt
    two_drone_spawn_scene.ttt
  logs/
```

## Boundaries

`controller/` is only for control algorithms and shared controller utilities.

- `controller/low_level/rate_control.py`: simulator-independent body-rate PI controller and motor mixer. This is the real-robot-facing control law.
- `controller/low_level/body_rate.py`: CoppeliaSim wrapper around the low-level controller: state extraction, motor dynamics, propeller animation, and force/torque application.
- `controller/high_level/position.py`: high-level position/yaw controller. Its output is collective thrust plus roll, pitch, and yaw-rate commands.
- `controller/common/telemetry.py`: shared CSV logging utilities.

`simulation/` is for CoppeliaSim-specific physical interaction models.

- `simulation/magnetic_docking.py`: drone discovery, cage connector geometry, face-gated magnetic capture, and finite-force latch/contact modeling.

`ui/` is for reusable UI components.

- `ui/plot_widgets.py`: strip charts, RPM bars, and plot-unit helpers used by experiment windows.

`scripts/experiments/` contains runnable experiment programs. These initialize CoppeliaSim objects, create UI windows when needed, call the controllers, and step the simulator.

`scripts/launchers/` contains the PyCharm-friendly entry points. Use these when running experiments.

## Main Workflow

Start CoppeliaSim and make sure the ZMQ remote API server is running.

For multi-drone position control and docking, run this launcher from PyCharm:

```powershell
python scripts\launchers\run_two_drone_position_ui.py
```

This spawns separated reusable drone models, starts the simulation, and opens the docking-aware target UI. By default it spawns two drones; the UI itself discovers every loaded drone and can command `all`, an individual drone, or a docked assembly.

For the tilted hex-to-hex assembly test, run:

```powershell
python scripts\launchers\run_hex_pair_position_ui.py
```

This builds `scene\hex_face_pair_scene.ttt` from the STL-derived hex face geometry, places two reusable drone plants in hex-face contact, then starts the same UI with the docked assembly controller enabled. The default controlled-test connection is drone A hex face 13 to drone B hex face 7. The generated left module tilts right with motor axis approximately `[0.577, 0.000, 0.816]`, and the right module tilts left with motor axis approximately `[-0.577, 0.000, 0.816]`. The generator prints the connector pairing and verifies that both motor thrust axes seen by the controller match the generated body transforms. The Python latch memory is still created live by the magnetic docking loop; the scene is intentionally just the physical starting configuration.

When a face-gated latch is detected, the UI automatically creates one assembly target at the connected system center of mass. Yaw and translation commands for that assembly are applied about this center-of-mass target. The UI also reports the detected docking configuration, such as the square/diamond face pair and face indices, and can quickly command a nominal square-face docking approach.

Drone heading is explicit. Body `+X` is the drone forward direction, shown visually by the red front propellers. Quick docking lets you choose the yaw convention for the pair, including fronts inward, fronts outward, same heading, or lateral heading, because the docked configuration and later wrench allocation depend on each module's yaw inside the assembly. Quick square docking is face-based: it selects vertical square/diamond faces, aligns their transformed face normals in opposition, and commands the body positions from the actual face-center geometry.

When a docked assembly is created, the configuration yaw is not averaged across drones. It is anchored to the leader module's body `+X` heading. Every other module stores a fixed yaw offset relative to that leader. During assembly control, yaw commands rotate the COM target and preserve those module yaw offsets, so front-to-front and other rotated docking configurations remain meaningful to the controller.

The docked attitude controller also stores the leader module's full initial body axes when the group is created or reset. That reference attitude, not a level upright attitude, is the default docked orientation. Yaw commands rotate this stored reference about world `Z`, which lets the hex-pair controller hold the intended inward-tilted assembly instead of trying to level the leader drone.

For single-drone position control with UI:

```powershell
python scripts\launchers\run_position_ui_controller.py
```

For a two-drone hover baseline without UI:

```powershell
python scripts\launchers\run_two_drone_hover.py
```

## Controller Architecture

Free flight uses a cascaded controller:

```text
target position/yaw
  -> high-level position controller
  -> collective thrust and body-rate commands
  -> simulator-independent low-level body-rate controller
  -> four motor angular velocities
  -> CoppeliaSim force/torque plant
```

Each free drone uses its own independent cascaded controller. When magnetic latches create a connected component, the UI treats that component as a docked assembly. The assembly controller is selected automatically by default: it rebuilds the current motor geometry and full assembly inertia tensor relative to the assembly center of mass, computes a desired 6D wrench, and allocates that wrench across all motors in the docked configuration.

The docked controller does not switch between square/diamond and hex controller modes. It always sends the full requested world-frame wrench `[Fx, Fy, Fz, Tx, Ty, Tz]` to the allocation matrix built from the current motor positions and thrust axes. There is no collective-axis projection fallback for docked assemblies. If the present configuration cannot realize a lateral force or torque, that failure remains visible as `allocation_residual_norm`, `allocation_weighted_residual_norm`, `wrench_residual_*`, and motor saturation in the UI/CSV telemetry.

When the docked controller is active, the UI also draws a cyan assembly-force arrow from the assembly center of mass. By default this arrow shows `wrench_achieved[:3]`, the net force actually allocated to the motors after feasibility and saturation, so it points in the direction the connected drone system is trying to accelerate. Use `--assembly-force-arrow-source command` to visualize the requested force or `--assembly-force-arrow-source residual` to visualize the missing force component `desired - achieved`.

## Docking Model

Docking is face-gated. Single-corner contact does not latch. The docking code reconstructs the truncated-octahedron cage topology from the STL, checks compatible face types, checks opposed face normals, checks face-center alignment, and only then creates corner latches.

Square/diamond faces dock only to square/diamond faces. Larger polygon faces dock only to the same larger-face type.

When latched, the code still uses physical force application. It does not teleport the drones, create hidden CoppeliaSim joints, or enforce a stored rigid transform. The latch is modeled as a stiff but finite spring-damper connection at the matched face corners, with force limits and break thresholds. Collision geometry keeps the cage nodes from occupying the same space, so the rest distance is set to the physical connector size rather than zero.

## Notes

The `.ttm` model is only the physical drone plant: geometry, collision, propeller joints, and cage connector frames. Controllers are external Python code, which is intentional for controller research and tuning.

Use the files in `scripts/launchers/` as your normal run targets. The launchers regenerate the plant or spawn a fresh scene so controller experiments start from a known state.
