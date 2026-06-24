#!/usr/bin/env python3
"""Generate trajectory-error and efficiency plots for the wall-tool simulator."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from wall_tool_sim.wall_tool_ui import SimParams, Vec2, WallToolSimulator  # noqa: E402


@dataclass(frozen=True)
class Scenario:
    name: str
    title: str
    waypoints: tuple[Vec2, ...]
    duration: float
    mode: str = "smooth"


def cumulative_integral(values: Sequence[float], dt: float) -> list[float]:
    total = 0.0
    result: list[float] = []
    for value in values:
        total += value * dt
        result.append(total)
    return result


def run_scenario(scenario: Scenario, params: SimParams) -> list[dict[str, float]]:
    sim = WallToolSimulator(params)
    if scenario.mode == "straight":
        sim.set_target(scenario.waypoints[-1])
    else:
        for waypoint in scenario.waypoints:
            sim.append_target(waypoint)

    weight = params.total_mass * params.gravity
    no_cable_hover_each = weight / (2.0 * math.cos(params.hex_face_tilt_rad))
    no_cable_power_index = 2.0 * no_cable_hover_each**1.5

    rows: list[dict[str, float]] = []
    for _ in range(max(1, int(scenario.duration / params.dt))):
        state = sim.step()
        drone_power_index = state.left_thrust**1.5 + state.right_thrust**1.5
        spool_power_in = state.tension * max(0.0, -state.spool_velocity_cmd)
        spool_power_abs = state.tension * abs(state.spool_velocity_cmd)
        rows.append(
            {
                "t": state.t,
                "x": state.tool_head[0],
                "z": state.tool_head[1],
                "measured_x": state.measured_payload[0],
                "measured_z": state.measured_payload[1],
                "ref_x": state.reference[0],
                "ref_z": state.reference[1],
                "error_x": state.tool_head[0] - state.reference[0],
                "error_z": state.tool_head[1] - state.reference[1],
                "error_norm": state.tool_error,
                "measured_error_norm": state.measured_tool_error,
                "cable_support_fraction": state.cable_vertical_force / weight,
                "drone_support_fraction": state.drone_vertical_force / weight,
                "total_support_fraction": (state.cable_vertical_force + state.drone_vertical_force) / weight,
                "left_thrust_fraction": state.left_thrust / params.max_thrust_per_drone,
                "right_thrust_fraction": state.right_thrust / params.max_thrust_per_drone,
                "cable_tension_fraction": state.tension / params.max_spool_tension,
                "allocation_residual_fraction": state.allocation_residual / weight,
                "left_thrust_n": state.left_thrust,
                "right_thrust_n": state.right_thrust,
                "cable_tension_n": state.tension,
                "measured_tension_n": state.measured_tension,
                "measured_cable_length_m": state.measured_cable_length,
                "measured_line_length_m": state.measured_line_length,
                "cable_angle_deg": math.degrees(state.measured_theta),
                "attitude_deg": math.degrees(state.attitude),
                "angular_velocity_deg_s": math.degrees(state.angular_velocity),
                "desired_attitude_torque_nm": state.desired_attitude_torque,
                "net_attitude_torque_nm": state.attitude_torque,
                "cable_torque_nm": state.cable_torque,
                "left_torque_nm": state.left_torque,
                "right_torque_nm": state.right_torque,
                "desired_cable_tension_n": state.desired_cable_tension,
                "drone_power_index": drone_power_index,
                "no_cable_power_index": no_cable_power_index,
                "drone_power_ratio": drone_power_index / no_cable_power_index,
                "instant_power_saving_fraction": 1.0 - drone_power_index / no_cable_power_index,
                "spool_power_in_w": spool_power_in,
                "spool_power_abs_w": spool_power_abs,
                "spool_velocity_cmd": state.spool_velocity_cmd,
                "cable_stretch": state.cable_stretch,
                "reference_speed_scale": state.reference_speed_scale,
            }
        )

    drone_power_cumulative = cumulative_integral([row["drone_power_index"] for row in rows], params.dt)
    baseline_power_cumulative = cumulative_integral([row["no_cable_power_index"] for row in rows], params.dt)
    spool_work_in = cumulative_integral([row["spool_power_in_w"] for row in rows], params.dt)
    spool_work_abs = cumulative_integral([row["spool_power_abs_w"] for row in rows], params.dt)
    for index, row in enumerate(rows):
        baseline = max(1e-12, baseline_power_cumulative[index])
        row["cumulative_drone_power_saving_fraction"] = 1.0 - drone_power_cumulative[index] / baseline
        row["cumulative_spool_work_in_j"] = spool_work_in[index]
        row["cumulative_spool_work_abs_j"] = spool_work_abs[index]
    return rows


def save_csv(rows: Sequence[dict[str, float]], path: Path) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def mean(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / len(values)


def rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def plot_scenario(scenario: Scenario, params: SimParams, rows: Sequence[dict[str, float]], output_path: Path) -> None:
    times = [row["t"] for row in rows]
    errors = [row["error_norm"] for row in rows]
    measured_errors = [row["measured_error_norm"] for row in rows]
    x_errors = [row["error_x"] for row in rows]
    z_errors = [row["error_z"] for row in rows]
    actual_x = [row["x"] for row in rows]
    actual_z = [row["z"] for row in rows]
    ref_x = [row["ref_x"] for row in rows]
    ref_z = [row["ref_z"] for row in rows]
    cable_support = [row["cable_support_fraction"] for row in rows]
    drone_support = [row["drone_support_fraction"] for row in rows]
    total_support = [row["total_support_fraction"] for row in rows]
    left_thrust = [row["left_thrust_fraction"] for row in rows]
    right_thrust = [row["right_thrust_fraction"] for row in rows]
    cable_tension = [row["cable_tension_fraction"] for row in rows]
    residual = [row["allocation_residual_fraction"] for row in rows]
    power_ratio = [row["drone_power_ratio"] for row in rows]
    cumulative_saving = [row["cumulative_drone_power_saving_fraction"] for row in rows]
    spool_work_in = [row["cumulative_spool_work_in_j"] for row in rows]
    spool_work_abs = [row["cumulative_spool_work_abs_j"] for row in rows]
    attitude = [row["attitude_deg"] for row in rows]
    angular_velocity = [row["angular_velocity_deg_s"] for row in rows]
    desired_torque = [row["desired_attitude_torque_nm"] for row in rows]
    net_torque = [row["net_attitude_torque_nm"] for row in rows]

    fig, axes = plt.subplots(4, 2, figsize=(15.5, 15.0), constrained_layout=True)
    fig.suptitle(scenario.title, fontsize=16, fontweight="bold")

    ax = axes[0][0]
    ax.add_patch(
        Rectangle(
            (-params.wall_width / 2.0, 0.0),
            params.wall_width,
            params.wall_height,
            facecolor="#f4f1e8",
            edgecolor="#555555",
            linewidth=1.5,
        )
    )
    ax.plot(ref_x, ref_z, color="#d44a3a", linestyle=":", linewidth=2.0, label="desired tool path")
    ax.plot(actual_x, actual_z, color="#1f8a89", linewidth=2.2, label="true tool path")
    ax.scatter([params.anchor[0]], [params.anchor[1]], s=90, color="#333333", label="anchor")
    if scenario.waypoints:
        ax.scatter(
            [point[0] for point in scenario.waypoints],
            [point[1] for point in scenario.waypoints],
            s=24,
            color="#d44a3a",
            marker="o",
            edgecolors="white",
            linewidths=0.7,
            label="commanded points",
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-params.wall_width / 2.0 - 0.18, params.wall_width / 2.0 + 0.18)
    ax.set_ylim(-0.10, params.wall_height + 0.35)
    ax.set_xlabel("wall x [m]")
    ax.set_ylabel("wall z [m]")
    ax.set_title("Tool Trajectory")
    ax.grid(True, color="#d8d1c3")
    ax.legend(loc="lower right", fontsize=8)

    ax = axes[0][1]
    ax.plot(times, errors, color="#111111", linewidth=2.0, label="norm")
    ax.plot(times, measured_errors, color="#777777", linestyle="--", linewidth=1.3, label="measured norm")
    ax.plot(times, x_errors, color="#4b83c4", linewidth=1.2, label="x error")
    ax.plot(times, z_errors, color="#c46b4b", linewidth=1.2, label="z error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error [m]")
    ax.set_title("Trajectory Tracking Error")
    ax.grid(True, color="#dddddd")
    ax.legend(fontsize=8)

    ax = axes[1][0]
    ax.plot(times, cable_support, color="#584b2f", linewidth=2.0, label="cable vertical support / weight")
    ax.plot(times, drone_support, color="#2d7f78", linewidth=2.0, label="drone vertical support / weight")
    ax.plot(times, total_support, color="#111111", linestyle="--", linewidth=1.2, label="total vertical support / weight")
    ax.axhline(1.0, color="#777777", linewidth=1.0, linestyle=":")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("support fraction")
    ax.set_ylim(-0.05, max(1.45, max(total_support, default=1.0) + 0.10))
    ax.set_title("Cable vs Drone Load Sharing")
    ax.grid(True, color="#dddddd")
    ax.legend(fontsize=8)

    ax = axes[1][1]
    ax.plot(times, left_thrust, color="#34699a", linewidth=1.6, label="left thrust / max")
    ax.plot(times, right_thrust, color="#b45f4d", linewidth=1.6, label="right thrust / max")
    ax.plot(times, cable_tension, color="#6e5a2e", linewidth=1.6, label="cable tension / max")
    ax.plot(times, residual, color="#111111", linestyle="--", linewidth=1.2, label="wrench residual / weight")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("normalized actuator use")
    ax.set_ylim(-0.03, 1.08)
    ax.set_title("Actuator Usage and Allocation Residual")
    ax.grid(True, color="#dddddd")
    ax.legend(fontsize=8)

    ax = axes[2][0]
    ax.plot(
        times,
        [100.0 * value for value in power_ratio],
        color="#7a3e9d",
        linewidth=2.0,
        label="instant drone power / baseline",
    )
    ax.plot(
        times,
        [100.0 * value for value in cumulative_saving],
        color="#2c7a45",
        linewidth=2.0,
        label="cumulative drone power saving",
    )
    ax.axhline(100.0, color="#777777", linewidth=1.0, linestyle=":", label="no-cable hover baseline")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("percent [%]")
    ax.set_title("Efficiency vs No-Cable Hover Baseline")
    ax.grid(True, color="#dddddd")
    ax.legend(fontsize=8)

    ax = axes[2][1]
    ax.plot(times, attitude, color="#111111", linewidth=1.8, label="attitude")
    ax.plot(times, angular_velocity, color="#777777", linewidth=1.1, linestyle="--", label="angular rate")
    ax.axhline(0.0, color="#777777", linewidth=0.8, linestyle=":")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("deg, deg/s")
    ax.set_title("Planar Tilt State")
    ax.grid(True, color="#dddddd")
    ax.legend(fontsize=8, loc="upper left")
    torque_ax = ax.twinx()
    torque_ax.plot(times, desired_torque, color="#3b6ea8", linewidth=1.2, label="desired torque")
    torque_ax.plot(times, net_torque, color="#a85d3b", linewidth=1.2, label="net torque")
    torque_ax.set_ylabel("torque [N m]")
    torque_ax.legend(fontsize=8, loc="upper right")

    ax = axes[3][0]
    ax.plot(times, spool_work_in, color="#8a5b22", linewidth=2.0, label="positive spool work")
    ax.plot(times, spool_work_abs, color="#8a5b22", linestyle="--", linewidth=1.4, label="absolute spool work")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("spool mechanical work [J]")
    ax.set_title("Spool Work")
    ax.grid(True, color="#dddddd")
    ax.legend(fontsize=8)

    summary = (
        f"RMS error: {rms(errors) * 1000.0:.1f} mm\n"
        f"RMS measured error: {rms(measured_errors) * 1000.0:.1f} mm\n"
        f"Max error: {max(errors, default=0.0) * 1000.0:.1f} mm\n"
        f"Mean cable support: {100.0 * mean(cable_support):.1f}% weight\n"
        f"Mean drone support: {100.0 * mean(drone_support):.1f}% weight\n"
        f"Mean drone power index: {100.0 * mean(power_ratio):.1f}% baseline\n"
        f"Final cumulative saving: {100.0 * cumulative_saving[-1]:.1f}%\n"
        f"Max residual: {100.0 * max(residual, default=0.0):.2f}% weight\n"
        f"Positive spool work: {spool_work_in[-1]:.3f} J"
    )
    axes[3][1].axis("off")
    axes[3][1].text(
        0.02,
        0.98,
        summary,
        transform=axes[3][1].transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "#aaaaaa", "alpha": 0.90},
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate wall-tool simulation analysis plots.")
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "wall_tool_sim" / "output"),
        help="Directory for PNG and CSV analysis outputs.",
    )
    parser.add_argument("--show", action="store_true", help="Open plots after saving.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir)
    if not output_dir.is_absolute():
        output_dir = PROJECT_ROOT / output_dir

    params = SimParams()
    scenarios = (
        Scenario(
            name="nominal_wall_scan",
            title="PRISMS Wall Tool: Nominal Wall Scan",
            waypoints=((0.70, 1.35), (1.15, 2.20), (0.45, 2.85), (-0.85, 2.45), (-1.15, 1.35), (0.0, 2.00)),
            duration=34.0,
            mode="smooth",
        ),
        Scenario(
            name="stress_upper_sweep",
            title="PRISMS Wall Tool: Upper-Edge Stress Sweep",
            waypoints=((0.85, 1.35), (1.55, 3.45), (-1.55, 3.45), (-1.00, 1.40), (0.0, 2.00)),
            duration=44.0,
            mode="smooth",
        ),
        Scenario(
            name="shallow_upper_right",
            title="PRISMS Wall Tool: Shallow-Cable Upper-Right Target",
            waypoints=((1.55, 3.45),),
            duration=20.0,
            mode="straight",
        ),
    )

    saved_paths: list[Path] = []
    for scenario in scenarios:
        rows = run_scenario(scenario, params)
        png_path = output_dir / f"analysis_{scenario.name}.png"
        csv_path = output_dir / f"analysis_{scenario.name}.csv"
        save_csv(rows, csv_path)
        plot_scenario(scenario, params, rows, png_path)
        saved_paths.extend((png_path, csv_path))

        final_saving = rows[-1]["cumulative_drone_power_saving_fraction"] if rows else 0.0
        max_error = max((row["error_norm"] for row in rows), default=0.0)
        print(
            f"{scenario.name}: max_error={max_error:.4f} m, "
            f"mean_power_ratio={mean([row['drone_power_ratio'] for row in rows]):.3f}, "
            f"final_saving={final_saving:.3f}"
        )

    for path in saved_paths:
        print(f"saved {path}")

    if args.show:
        for path in saved_paths:
            if path.suffix.lower() == ".png":
                img = plt.imread(path)
                plt.figure(figsize=(10, 7))
                plt.imshow(img)
                plt.axis("off")
                plt.title(path.name)
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
