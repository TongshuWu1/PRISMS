#!/usr/bin/env python3
"""One-command launcher: spawn two drones, then hover them with Python control."""

from __future__ import annotations

import argparse

from launcher_utils import (
    PROJECT_ROOT,
    TWO_DRONE_SPAWNER,
    add_two_drone_spawn_args,
    extend_connection_args,
    extend_two_drone_spawn_args,
    python_command,
    run_step,
)

CONTROLLER = PROJECT_ROOT / "scripts" / "experiments" / "two_drone_hover.py"


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Spawn two drones and run the two-drone hover controller.")
    add_two_drone_spawn_args(parser)
    return parser.parse_known_args()


def main() -> int:
    args, controller_args = parse_args()

    spawn_command = extend_two_drone_spawn_args(python_command(TWO_DRONE_SPAWNER), args)
    run_step(spawn_command, "Spawning two drone plant models...", check=True)

    controller_command = extend_connection_args(python_command(CONTROLLER), args)
    controller_command.extend(controller_args)
    return run_step(controller_command, "Starting two-drone hover controller...")


if __name__ == "__main__":
    raise SystemExit(main())
