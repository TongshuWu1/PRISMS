"""Create and validate independent hardware calibration profiles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from cable_hybrid_controller.controller import best_params

from .validation_plant import (
    datasheet_validation_profile,
    load_calibrated_validation_profile,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manage validation-plant calibration profiles")
    commands = parser.add_subparsers(dest="command", required=True)
    template = commands.add_parser(
        "template",
        help="write an explicitly uncalibrated JSON profile to populate from hardware identification",
    )
    template.add_argument("--output", type=Path, required=True)
    template.add_argument("--force", action="store_true")
    validate = commands.add_parser(
        "validate",
        help="strictly validate a completed calibrated profile",
    )
    validate.add_argument("--profile", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.command == "template":
        output = args.output.resolve()
        if output.exists() and not args.force:
            raise FileExistsError(f"refusing to overwrite existing calibration profile: {output}")
        profile = datasheet_validation_profile(best_params())
        data = profile.to_json_dict()
        data["profile_name"] = "REPLACE_WITH_HARDWARE_PROFILE_NAME"
        data["provenance"] = {
            "hardware_id": "REQUIRED",
            "recorded_utc": "REQUIRED_ISO_8601_UTC",
            "raw_data_sha256": "REQUIRED_64_HEX_DIGITS",
            "notes": "Replace every assumed parameter with an identified value before setting calibrated=true.",
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        print(f"Uncalibrated template written to {output}")
        print("It is intentionally rejected by calibrated simulation mode until calibrated=true and provenance are valid.")
        return 0
    profile = load_calibrated_validation_profile(args.profile)
    print(
        f"Valid calibrated profile: {profile.profile_name}; "
        f"hardware_id={profile.provenance['hardware_id']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
