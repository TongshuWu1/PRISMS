#!/usr/bin/env python3
"""One-command launcher for Xbox/gamepad FPV rate control."""

from __future__ import annotations

import argparse

from launcher_utils import (
    PROJECT_ROOT,
    SINGLE_DRONE_GENERATOR,
    add_single_drone_generation_args,
    extend_single_drone_generator_args,
    extend_single_drone_runtime_args,
    python_command,
    run_step,
)

XBOX_CONTROLLER = PROJECT_ROOT / "scripts" / "experiments" / "xbox_fpv.py"


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Generate the CoppeliaSim drone plant and run the Xbox FPV controller."
    )
    add_single_drone_generation_args(parser)
    parser.add_argument("--list-controllers", action="store_true", help="List detected gamepads and exit.")
    return parser.parse_known_args()


def main() -> int:
    args, controller_args = parse_args()

    if args.list_controllers:
        return run_step(python_command(XBOX_CONTROLLER, "--list-controllers"), "Listing detected gamepads...")

    generator_command = extend_single_drone_generator_args(python_command(SINGLE_DRONE_GENERATOR), args)
    run_step(generator_command, "Generating CoppeliaSim drone plant scene...", check=True)

    controller_command = extend_single_drone_runtime_args(python_command(XBOX_CONTROLLER, "--load-scene"), args)
    controller_command.extend(controller_args)
    return run_step(controller_command, "Starting Xbox FPV controller...")


if __name__ == "__main__":
    raise SystemExit(main())
