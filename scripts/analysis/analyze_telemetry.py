#!/usr/bin/env python3
"""Summarize a controller telemetry CSV file."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze a drone telemetry CSV file.")
    parser.add_argument("csv_file", help="Telemetry CSV path.")
    parser.add_argument("--max-motor-speed", type=float, default=2600.0, help="Motor speed limit [rad/s].")
    parser.add_argument("--saturation-ratio", type=float, default=0.98, help="Fraction of max speed counted as saturated.")
    return parser.parse_args()


def resolve_path(path_text: str) -> Path:
    path = Path(path_text)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def read_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            row: dict[str, float] = {}
            for key, value in raw.items():
                if value is None or value == "":
                    continue
                try:
                    row[key] = float(value)
                except ValueError:
                    continue
            rows.append(row)
    return rows


def vector_norm(row: dict[str, float], keys: tuple[str, str, str]) -> float | None:
    if not all(key in row for key in keys):
        return None
    return math.sqrt(sum(row[key] * row[key] for key in keys))


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def rms(values: list[float]) -> float:
    return math.sqrt(mean([value * value for value in values])) if values else 0.0


def extrema(rows: list[dict[str, float]], key: str) -> tuple[float, float] | None:
    values = [row[key] for row in rows if key in row]
    if not values:
        return None
    return min(values), max(values)


def find_vector_prefix(rows: list[dict[str, float]], base: str) -> tuple[str, str, str] | None:
    candidates = [
        (f"{base}_x", f"{base}_y", f"{base}_z"),
        (f"high_{base}_x", f"high_{base}_y", f"high_{base}_z"),
        (f"low_{base}_x", f"low_{base}_y", f"low_{base}_z"),
    ]
    available = set()
    for row in rows[: min(10, len(rows))]:
        available.update(row.keys())
    for candidate in candidates:
        if all(key in available for key in candidate):
            return candidate
    return None


def omega_keys(rows: list[dict[str, float]]) -> list[str]:
    available = set()
    for row in rows[: min(10, len(rows))]:
        available.update(row.keys())
    for prefix in ("low_omega", "omega"):
        keys = [f"{prefix}_{index}" for index in range(4)]
        if all(key in available for key in keys):
            return keys
    return []


def main() -> int:
    args = parse_args()
    path = resolve_path(args.csv_file)
    if not path.exists():
        raise FileNotFoundError(path)

    rows = read_rows(path)
    if not rows:
        raise RuntimeError(f"No numeric telemetry rows found in {path}")

    times = [row["time"] for row in rows if "time" in row]
    duration = max(times) - min(times) if len(times) >= 2 else 0.0
    sample_rate = (len(times) - 1) / duration if duration > 0.0 else 0.0

    print(f"Telemetry: {path}")
    print(f"  samples: {len(rows)}")
    print(f"  duration: {duration:.3f} s")
    print(f"  logged sample rate: {sample_rate:.1f} Hz")

    error_keys = find_vector_prefix(rows, "pos_error")
    if error_keys:
        error_norms = [value for row in rows if (value := vector_norm(row, error_keys)) is not None]
        final_error = error_norms[-1]
        print("  position tracking:")
        print(f"    RMS error norm: {rms(error_norms):.4f} m")
        print(f"    max error norm: {max(error_norms):.4f} m")
        print(f"    final error norm: {final_error:.4f} m")

    position_keys = find_vector_prefix(rows, "pos")
    if position_keys:
        z_limits = extrema(rows, position_keys[2])
        if z_limits:
            print(f"  z range: {z_limits[0]:.4f} to {z_limits[1]:.4f} m")

    rate_error_keys = None
    for prefix in ("low_rate_error", "rate_error"):
        candidate = (f"{prefix}_p", f"{prefix}_q", f"{prefix}_r")
        if all(candidate_key in rows[0] for candidate_key in candidate):
            rate_error_keys = candidate
            break
    if rate_error_keys:
        rate_error_norms = [value for row in rows if (value := vector_norm(row, rate_error_keys)) is not None]
        print("  body-rate tracking:")
        print(f"    RMS pqr error norm: {rms(rate_error_norms):.4f} rad/s")
        print(f"    max pqr error norm: {max(rate_error_norms):.4f} rad/s")

    motor_keys = omega_keys(rows)
    if motor_keys:
        max_omega = max(max(row.get(key, 0.0) for key in motor_keys) for row in rows)
        threshold = args.saturation_ratio * args.max_motor_speed
        saturated_rows = [
            row for row in rows if any(row.get(key, 0.0) >= threshold for key in motor_keys)
        ]
        print("  motors:")
        print(f"    max omega: {max_omega:.1f} rad/s")
        print(f"    saturation fraction: {len(saturated_rows) / len(rows):.3f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
