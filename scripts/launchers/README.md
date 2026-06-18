# Launchers

PyCharm-friendly entry points for running experiments. Launchers create a known
CoppeliaSim plant/scene before starting the controller, so a run does not depend
on leftover simulator state.

- `run_position_controller.py`: single-drone high-level position controller without UI.
- `run_position_ui_controller.py`: single-drone position-control UI.
- `run_hex_pair_position_ui.py`: tilted hex-to-hex two-drone assembly workflow; it generates `scene/hex_face_pair_scene.ttt` with default face pair A13-to-B7 and starts the docked allocation UI.
- `run_two_drone_position_ui.py`: main multi-drone docking UI workflow; it spawns two drones by default and can spawn more when configured in PyCharm.
- `run_two_drone_hover.py`: two-drone hover without UI.
- `run_xbox_fpv_controller.py`: gamepad FPV/rate experiment.
- `launcher_utils.py`: shared subprocess and argument helpers.
