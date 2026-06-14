#!/usr/bin/env python3
"""Shared helpers for CoppeliaSim controller launchers."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SINGLE_DRONE_GENERATOR = PROJECT_ROOT / "scripts" / "generation" / "generate_drone_plant_scene.py"
TWO_DRONE_SPAWNER = PROJECT_ROOT / "scripts" / "generation" / "spawn_two_drones.py"


def python_command(script: Path, *args: str) -> list[str]:
    return [sys.executable, str(script), *args]


def run_step(command: list[str], message: str, *, check: bool = False) -> int:
    print(message, flush=True)
    return subprocess.run(command, cwd=PROJECT_ROOT, check=check).returncode


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)


def extend_connection_args(command: list[str], args: argparse.Namespace) -> list[str]:
    command.extend(["--host", args.host, "--port", str(args.port), "--connect-timeout", str(args.connect_timeout)])
    return command


def extend_optional_arg(command: list[str], args: argparse.Namespace, name: str, flag: str | None = None) -> list[str]:
    value = getattr(args, name)
    if value is not None:
        command.extend([flag or f"--{name.replace('_', '-')}", str(value)])
    return command


def add_single_drone_generation_args(parser: argparse.ArgumentParser) -> None:
    add_connection_args(parser)
    parser.add_argument("--mass", type=float, default=None, help="Total drone mass [kg].")
    parser.add_argument("--start-height", type=float, default=None, help="Initial drone center height [m].")
    parser.add_argument("--collision-rod-diameter", type=float, default=None)
    parser.add_argument("--collision-node-diameter", type=float, default=None)
    parser.add_argument("--show-collision", action="store_true")


def extend_single_drone_runtime_args(command: list[str], args: argparse.Namespace) -> list[str]:
    extend_connection_args(command, args)
    extend_optional_arg(command, args, "mass")
    extend_optional_arg(command, args, "start_height")
    return command


def extend_single_drone_generator_args(command: list[str], args: argparse.Namespace) -> list[str]:
    extend_single_drone_runtime_args(command, args)
    extend_optional_arg(command, args, "collision_rod_diameter")
    extend_optional_arg(command, args, "collision_node_diameter")
    if args.show_collision:
        command.append("--show-collision")
    return command


def add_two_drone_spawn_args(parser: argparse.ArgumentParser) -> None:
    add_connection_args(parser)
    parser.add_argument("--count", type=int, default=2, help="Number of drones to spawn.")
    parser.add_argument("--height", type=float, default=0.50)
    parser.add_argument("--spacing", type=float, default=0.32, help="Initial center spacing [m].")
    parser.add_argument("--axis", choices=("x", "y", "xy", "x-neg-y"), default="xy")
    parser.add_argument("--yaw-a-deg", type=float, default=0.0)
    parser.add_argument("--yaw-b-deg", type=float, default=180.0)
    parser.add_argument("--collision-rod-diameter", type=float, default=None)
    parser.add_argument("--collision-node-diameter", type=float, default=None)


def extend_two_drone_spawn_args(command: list[str], args: argparse.Namespace) -> list[str]:
    extend_connection_args(command, args)
    command.extend(
        [
            "--height",
            str(args.height),
            "--count",
            str(args.count),
            "--spacing",
            str(args.spacing),
            "--axis",
            args.axis,
            "--yaw-a-deg",
            str(args.yaw_a_deg),
            "--yaw-b-deg",
            str(args.yaw_b_deg),
        ]
    )
    extend_optional_arg(command, args, "collision_rod_diameter")
    extend_optional_arg(command, args, "collision_node_diameter")
    return command
