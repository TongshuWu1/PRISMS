#!/usr/bin/env python3
"""One-command launcher for high-level position control."""

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

POSITION_CONTROLLER = PROJECT_ROOT / "controller" / "high_level" / "position.py"


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Generate the CoppeliaSim drone plant and run the high-level position controller."
    )
    add_single_drone_generation_args(parser)
    return parser.parse_known_args()


def main() -> int:
    args, controller_args = parse_args()

    generator_command = extend_single_drone_generator_args(python_command(SINGLE_DRONE_GENERATOR), args)
    run_step(generator_command, "Generating CoppeliaSim drone plant scene...", check=True)

    controller_command = extend_single_drone_runtime_args(python_command(POSITION_CONTROLLER, "--load-scene"), args)
    controller_command.extend(controller_args)
    return run_step(controller_command, "Starting high-level position controller...")


if __name__ == "__main__":
    raise SystemExit(main())
