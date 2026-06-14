#!/usr/bin/env python3
"""Spawn externally controlled drone plant models in CoppeliaSim.

This script does not create or embed any controller. It only loads two copies
of model/truncated_octahedral_crazyflie_plant.ttm by default, places them in
the scene, and optionally saves the result as a reusable multi-drone scene.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
GENERATION_DIR = Path(__file__).resolve().parent
if str(GENERATION_DIR) not in sys.path:
    sys.path.insert(0, str(GENERATION_DIR))

import generate_drone_plant_scene as plant  # noqa: E402


MODEL_PATH = PROJECT_ROOT / "model" / "truncated_octahedral_crazyflie_plant.ttm"
SCENE_OUTPUT = PROJECT_ROOT / "scene" / "two_drone_spawn_scene.ttt"
MODEL_ALIAS = plant.MODEL_ALIAS


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Spawn Python-controlled drone plant models in CoppeliaSim.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--model", default=str(MODEL_PATH), help="Reusable drone plant .ttm to load.")
    parser.add_argument("--scene-output", default=str(SCENE_OUTPUT), help="Where to save the spawned multi-drone scene.")
    parser.add_argument("--save-scene", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clear-existing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--collision-rod-diameter", type=float, default=None)
    parser.add_argument("--collision-node-diameter", type=float, default=None)
    parser.add_argument("--count", type=int, default=2, help="Number of drone models to spawn.")
    parser.add_argument("--height", type=float, default=0.50, help="Spawn height of drone centers [m].")
    parser.add_argument("--spacing", type=float, default=0.22, help="Center-to-center spacing between drones [m].")
    parser.add_argument(
        "--axis",
        choices=("x", "y", "xy", "x-neg-y"),
        default="x",
        help="Axis along which the pair is separated. Use xy for square-face docking.",
    )
    parser.add_argument("--yaw-a-deg", type=float, default=0.0, help="Yaw angle for drone A [deg].")
    parser.add_argument("--yaw-b-deg", type=float, default=180.0, help="Yaw angle for drone B [deg].")
    return parser.parse_args()


def refresh_model(args: argparse.Namespace) -> None:
    command = [
        sys.executable,
        str(GENERATION_DIR / "generate_drone_plant_scene.py"),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--connect-timeout",
        str(args.connect_timeout),
        "--model",
        str(MODEL_PATH),
    ]
    if args.collision_rod_diameter is not None:
        command.extend(["--collision-rod-diameter", str(args.collision_rod_diameter)])
    if args.collision_node_diameter is not None:
        command.extend(["--collision-node-diameter", str(args.collision_node_diameter)])
    print("Refreshing single-drone plant model...", flush=True)
    subprocess.run(command, cwd=PROJECT_ROOT, check=True)


def generated_drone_objects(sim) -> list[int]:
    objects = []
    prefixes = (f"/{MODEL_ALIAS}", "/position_target")
    for handle in plant.all_scene_objects(sim):
        if plant.object_alias(sim, handle).startswith(prefixes):
            objects.append(handle)
    return objects


def clear_existing_drones(sim) -> None:
    to_remove = generated_drone_objects(sim)
    if to_remove:
        sim.removeObjects(to_remove, False)
        print(f"Removed {len(to_remove)} old drone/target objects.")


def pair_positions(axis: str, spacing: float, height: float) -> tuple[list[float], list[float]]:
    half = spacing * 0.5
    if axis == "x":
        return [-half, 0.0, height], [half, 0.0, height]
    if axis == "y":
        return [0.0, -half, height], [0.0, half, height]
    diagonal = half / math.sqrt(2.0)
    if axis == "xy":
        return [-diagonal, -diagonal, height], [diagonal, diagonal, height]
    return [-diagonal, diagonal, height], [diagonal, -diagonal, height]


def spawn_positions(count: int, axis: str, spacing: float, height: float) -> list[list[float]]:
    if count < 1:
        raise ValueError("count must be at least 1.")
    if count == 1:
        return [[0.0, 0.0, height]]
    if count == 2:
        pos_a, pos_b = pair_positions(axis, spacing, height)
        return [pos_a, pos_b]

    columns = math.ceil(math.sqrt(count))
    rows = math.ceil(count / columns)
    origin_x = -0.5 * (columns - 1) * spacing
    origin_y = -0.5 * (rows - 1) * spacing
    positions = []
    for index in range(count):
        row = index // columns
        column = index % columns
        positions.append([origin_x + column * spacing, origin_y + row * spacing, height])
    return positions


def yaw_for_index(index: int, yaw_a_deg: float, yaw_b_deg: float) -> float:
    return yaw_a_deg if index % 2 == 0 else yaw_b_deg


def load_drone(sim, model_path: Path, position: list[float], yaw_deg: float, index: int) -> int:
    handle = sim.loadModel(str(model_path))
    if isinstance(handle, (list, tuple)):
        handle = handle[0]
    if handle < 0:
        raise RuntimeError(f"CoppeliaSim failed to load model: {model_path}")

    sim.setObjectAlias(handle, MODEL_ALIAS, 1)
    sim.setObjectPosition(handle, -1, position)
    sim.setObjectOrientation(handle, -1, [0.0, 0.0, math.radians(yaw_deg)])
    sim.resetDynamicObject(handle)
    print(
        f"Spawned drone {index}: handle={handle}, "
        f"alias={plant.object_alias(sim, handle)}, "
        f"pos=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}], "
        f"yaw={yaw_deg:.1f} deg"
    )
    return int(handle)


def main() -> int:
    args = parse_args()
    model_path = Path(args.model)
    refresh_model(args)
    if not model_path.exists():
        raise FileNotFoundError(
            f"{model_path}\nRun: python scripts\\generation\\generate_drone_plant_scene.py"
        )

    sim = plant.connect(args)
    plant.stop_if_running(sim)
    if args.clear_existing:
        clear_existing_drones(sim)

    positions = spawn_positions(args.count, args.axis, args.spacing, args.height)
    handles = [
        load_drone(sim, model_path, position, yaw_for_index(index, args.yaw_a_deg, args.yaw_b_deg), index)
        for index, position in enumerate(positions)
    ]
    sim.setObjectSel(handles)

    if args.save_scene:
        scene_output = Path(args.scene_output)
        scene_output.parent.mkdir(parents=True, exist_ok=True)
        sim.saveScene(str(scene_output))
        print(f"Saved multi-drone scene: {scene_output}")

    print("Next: run scripts\\launchers\\run_two_drone_position_ui.py from PyCharm.")
    time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
