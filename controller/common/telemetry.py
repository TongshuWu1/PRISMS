#!/usr/bin/env python3
"""CSV telemetry helpers for CoppeliaSim drone controller experiments."""

from __future__ import annotations

import argparse
import csv
import math
from collections.abc import Mapping, Sequence
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LOG_DIR = PROJECT_ROOT / "logs"

VECTOR_LABELS = {
    "pos": ("x", "y", "z"),
    "target": ("x", "y", "z"),
    "pos_error": ("x", "y", "z"),
    "pos_integral": ("x", "y", "z"),
    "lin_vel": ("x", "y", "z"),
    "ang_vel_world": ("x", "y", "z"),
    "rate": ("p", "q", "r"),
    "rate_cmd": ("p", "q", "r"),
    "rate_error": ("p", "q", "r"),
    "force_world": ("x", "y", "z"),
    "torque_world": ("x", "y", "z"),
    "torque_body": ("x", "y", "z"),
    "moment_cmd": ("x", "y", "z"),
    "att_error": ("x", "y", "z"),
    "accel_cmd": ("x", "y", "z"),
    "pid_accel_p": ("x", "y", "z"),
    "pid_accel_i": ("x", "y", "z"),
    "pid_accel_d": ("x", "y", "z"),
    "pid_accel_sum_raw": ("x", "y", "z"),
    "body_z": ("x", "y", "z"),
    "desired_b3": ("x", "y", "z"),
    "omega": ("0", "1", "2", "3"),
    "omega_cmd": ("0", "1", "2", "3"),
    "motor_thrust": ("0", "1", "2", "3"),
    "motor_thrust_cmd": ("0", "1", "2", "3"),
}


def add_logging_args(parser: argparse.ArgumentParser, default_period: float = 0.01) -> None:
    parser.add_argument(
        "--log-csv",
        nargs="?",
        const="auto",
        default=None,
        help=(
            "Write controller telemetry to CSV. Use '--log-csv' for an automatic "
            "logs/<timestamp> file, or '--log-csv path/to/file.csv'."
        ),
    )
    parser.add_argument(
        "--log-sample-period",
        type=float,
        default=default_period,
        help="Telemetry CSV sample period [s]. Use 0 to log every simulation step.",
    )


def resolve_log_path(log_csv: str | None, prefix: str) -> Path | None:
    if not log_csv:
        return None

    if log_csv == "auto":
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return DEFAULT_LOG_DIR / f"{timestamp}_{prefix}.csv"

    path = Path(log_csv)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path


def _is_scalar(value: Any) -> bool:
    return isinstance(value, (str, int, float, bool)) or value is None


def _clean_scalar(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return ""
    if isinstance(value, bool):
        return int(value)
    return value


def flatten_sample(sample: Mapping[str, Any], prefix: str = "") -> dict[str, Any]:
    row: dict[str, Any] = {}
    for key, value in sample.items():
        full_key = f"{prefix}{key}"
        if _is_scalar(value):
            row[full_key] = _clean_scalar(value)
        elif isinstance(value, Mapping):
            row.update(flatten_sample(value, f"{full_key}_"))
        elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
            labels = VECTOR_LABELS.get(key)
            for index, item in enumerate(value):
                label = labels[index] if labels and index < len(labels) else str(index)
                row[f"{full_key}_{label}"] = _clean_scalar(item)
    return row


def merge_samples(
    *samples: tuple[str, Mapping[str, Any] | None],
    extra: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    row: dict[str, Any] = {}
    for prefix, sample in samples:
        if sample is None:
            continue
        row.update(flatten_sample(sample, f"{prefix}_" if prefix else ""))
    if extra:
        row.update(flatten_sample(extra))
    return row


class CsvTelemetryLogger:
    """Small fixed-header CSV logger for controller telemetry."""

    def __init__(self, log_csv: str | None, prefix: str, sample_period: float) -> None:
        self.path = resolve_log_path(log_csv, prefix)
        self.sample_period = max(0.0, sample_period)
        self.next_time = 0.0
        self._file = None
        self._writer: csv.DictWriter | None = None
        self._fieldnames: list[str] | None = None

    @property
    def enabled(self) -> bool:
        return self.path is not None

    def write(self, sim_time: float, row: Mapping[str, Any]) -> None:
        if self.path is None:
            return
        if self.sample_period > 0.0 and sim_time + 1e-12 < self.next_time:
            return
        if self.sample_period > 0.0:
            while self.next_time <= sim_time + 1e-12:
                self.next_time += self.sample_period

        output = {"time": sim_time}
        output.update(row)
        if self._writer is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.path.open("w", newline="", encoding="utf-8")
            self._fieldnames = list(output.keys())
            self._writer = csv.DictWriter(self._file, fieldnames=self._fieldnames, extrasaction="ignore")
            self._writer.writeheader()
            print(f"Telemetry CSV: {self.path}")

        self._writer.writerow(output)

    def close(self) -> None:
        if self._file:
            self._file.close()
            self._file = None
            if self.path:
                print(f"Telemetry saved: {self.path}")
