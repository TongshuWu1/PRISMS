#!/usr/bin/env python3
r"""One-command launcher for the square-face magnetic docking test.

Normal use:

    python configurations\square_face_predocked_pair\scripts\run_magnetic_docking_pair_test.py
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


CONFIG_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = CONFIG_ROOT.parents[1]
GENERATOR = CONFIG_ROOT / "scripts" / "generate_space_thrust_pair_scene.py"
CONTROLLER = CONFIG_ROOT / "controller" / "magnetic_docking_pair_test.py"


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Generate the dynamic pair scene and run connector-level magnetic docking."
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--mass", type=float, default=None, help="Mass per drone module [kg].")
    parser.add_argument("--height", type=float, default=None, help="Initial pair center height [m].")
    parser.add_argument("--gap", type=float, default=0.030, help="Initial connector face gap [m].")
    parser.add_argument("--square-face-support", type=float, default=None)
    parser.add_argument(
        "--docking-face",
        choices=("pos_xy_to_neg_xy", "pos_x_neg_y_to_neg_x_pos_y"),
        default=None,
    )
    parser.add_argument("--collision-rod-diameter", type=float, default=None)
    parser.add_argument("--collision-node-diameter", type=float, default=None)
    parser.add_argument("--show-collision", action="store_true")
    parser.add_argument("--skip-generate", action="store_true", help="Reuse the existing pair scene/model files.")
    return parser.parse_known_args()


def add_common_args(command: list[str], args: argparse.Namespace) -> list[str]:
    command.extend(["--host", args.host, "--port", str(args.port), "--connect-timeout", str(args.connect_timeout)])
    if args.mass is not None:
        command.extend(["--mass", str(args.mass)])
    if args.height is not None:
        command.extend(["--height", str(args.height)])
    command.extend(["--gap", str(args.gap)])
    if args.square_face_support is not None:
        command.extend(["--square-face-support", str(args.square_face_support)])
    if args.docking_face is not None:
        command.extend(["--docking-face", args.docking_face])
    return command


def add_generator_args(command: list[str], args: argparse.Namespace) -> list[str]:
    if args.collision_rod_diameter is not None:
        command.extend(["--collision-rod-diameter", str(args.collision_rod_diameter)])
    if args.collision_node_diameter is not None:
        command.extend(["--collision-node-diameter", str(args.collision_node_diameter)])
    if args.show_collision:
        command.append("--show-collision")
    return command


def main() -> int:
    args, controller_args = parse_args()

    if not args.skip_generate:
        generator_command = add_common_args([sys.executable, str(GENERATOR)], args)
        add_generator_args(generator_command, args)
        print("Generating dynamic two-drone docking scene...", flush=True)
        subprocess.run(generator_command, cwd=PROJECT_ROOT, check=True)

    controller_command = add_common_args(
        [sys.executable, str(CONTROLLER), "--load-scene"],
        args,
    )
    controller_command.extend(controller_args)
    print("Starting magnetic docking controller...", flush=True)
    return subprocess.run(controller_command, cwd=PROJECT_ROOT).returncode


if __name__ == "__main__":
    raise SystemExit(main())
