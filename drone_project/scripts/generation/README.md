# Generation

Scripts that build or load CoppeliaSim models and scenes.

- `generate_drone_plant_scene.py`: builds the reusable single-drone plant `.ttm`
  and demo `.ttt`.
- `spawn_two_drones.py`: loads two copies of the reusable plant model into a
  CoppeliaSim scene.
- `spawn_hex_face_pair.py`: loads two reusable plant models, computes a
  tilted hex-to-hex pose from the STL-derived cage faces, and saves
  `scene/hex_face_pair_scene.ttt` for docked allocation tests. Its default
  controlled-test connection is drone A hex face 13 to drone B hex face 7,
  with the left module tilting right and the right module tilting left. Both
  motor thrust axes are validated against the controller's body-axis
  convention.

The active architecture is one reusable drone plant plus external Python
controllers. Pre-generated two-drone combined models are archived.
