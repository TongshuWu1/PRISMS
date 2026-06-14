# Generation

Scripts that build or load CoppeliaSim models and scenes.

- `generate_drone_plant_scene.py`: builds the reusable single-drone plant `.ttm`
  and demo `.ttt`.
- `spawn_two_drones.py`: loads two copies of the reusable plant model into a
  CoppeliaSim scene.

The active architecture is one reusable drone plant plus external Python
controllers. Pre-generated two-drone combined models are archived.
