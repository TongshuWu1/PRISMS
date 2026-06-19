#!/usr/bin/env python3
"""One-command launcher for the tilted hex-face two-drone assembly."""

from __future__ import annotations

import argparse

from launcher_utils import (
    PROJECT_ROOT,
    add_connection_args,
    extend_connection_args,
    extend_optional_arg,
    python_command,
    run_step,
)


HEX_PAIR_SPAWNER = PROJECT_ROOT / "scripts" / "generation" / "spawn_hex_face_pair.py"
CONTROLLER = PROJECT_ROOT / "scripts" / "experiments" / "two_drone_position_ui.py"
DEFAULT_SCENE = PROJECT_ROOT / "scene" / "hex_face_pair_scene.ttt"


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Spawn a tilted hex-face pair and run the multi-drone docking UI."
    )
    add_connection_args(parser)
    parser.add_argument("--height", type=float, default=0.45, help="Drone A center height [m].")
    parser.add_argument("--contact-distance", type=float, default=None)
    parser.add_argument("--face-a-index", type=int, default=None)
    parser.add_argument("--face-b-index", type=int, default=None)
    parser.add_argument("--min-module-tilt-deg", type=float, default=None)
    parser.add_argument("--max-pairing-error", type=float, default=None)
    parser.add_argument("--auto-face-pair", action="store_true")
    parser.add_argument("--collision-rod-diameter", type=float, default=None)
    parser.add_argument("--collision-node-diameter", type=float, default=None)
    parser.add_argument("--refresh-model", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--scene-output", default=str(DEFAULT_SCENE))
    return parser.parse_known_args()


def hex_spawn_command(args: argparse.Namespace) -> list[str]:
    command = extend_connection_args(python_command(HEX_PAIR_SPAWNER), args)
    command.extend(["--height", str(args.height), "--scene-output", str(args.scene_output)])
    for name in (
        "contact_distance",
        "face_a_index",
        "face_b_index",
        "min_module_tilt_deg",
        "max_pairing_error",
        "collision_rod_diameter",
        "collision_node_diameter",
    ):
        extend_optional_arg(command, args, name)
    if not args.refresh_model:
        command.append("--no-refresh-model")
    if args.auto_face_pair:
        command.append("--auto-face-pair")
    return command


def controller_command(args: argparse.Namespace, controller_args: list[str]) -> list[str]:
    command = extend_connection_args(python_command(CONTROLLER), args)
    command.extend(["--scene", str(args.scene_output), "--load-scene", "--docked-controller"])
    command.extend(controller_args)
    return command


def main() -> int:
    args, controller_args = parse_args()
    run_step(hex_spawn_command(args), "Spawning tilted hex-face pair scene...", check=True)
    return run_step(controller_command(args, controller_args), "Starting hex-pair docked allocation UI...")


if __name__ == "__main__":
    raise SystemExit(main())
