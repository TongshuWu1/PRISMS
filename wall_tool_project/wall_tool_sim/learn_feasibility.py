#!/usr/bin/env python3
"""Learn a lightweight wall-tool feasibility/cost map.

This is an offline diagnostic model, not a controller. It samples wall targets,
labels each target with a static hold search, then fits polynomial ridge models
that predict hold cost, drone power, allocation residual, and stable hold tilt.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
import numpy as np  # noqa: E402


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from wall_tool_sim.wall_tool_ui import (  # noqa: E402
    SimParams,
    Vec2,
    WallToolSimulator,
    clamp,
    cross2,
    normalize2,
    scale2,
    sub2,
)


@dataclass(frozen=True)
class HoldLabel:
    x: float
    z: float
    cable_angle_rad: float
    cable_length_m: float
    vertical_efficiency: float
    hold_attitude_rad: float
    cable_tension_n: float
    left_thrust_n: float
    right_thrust_n: float
    cable_support_fraction: float
    drone_power_ratio: float
    residual_fraction: float
    max_thrust_fraction: float
    static_cost: float
    feasible: bool


def polynomial_feature_names(base_names: Sequence[str], degree: int) -> list[str]:
    names = ["bias"]
    if degree >= 1:
        names.extend(base_names)
    if degree >= 2:
        for i, left in enumerate(base_names):
            for right in base_names[i:]:
                names.append(f"{left}*{right}")
    if degree >= 3:
        for i, left in enumerate(base_names):
            for j, middle in enumerate(base_names[i:], start=i):
                for right in base_names[j:]:
                    names.append(f"{left}*{middle}*{right}")
    return names


def polynomial_features(values: Sequence[float], degree: int) -> list[float]:
    vals = [float(value) for value in values]
    features = [1.0]
    if degree >= 1:
        features.extend(vals)
    if degree >= 2:
        for i, left in enumerate(vals):
            for right in vals[i:]:
                features.append(left * right)
    if degree >= 3:
        for i, left in enumerate(vals):
            for j, middle in enumerate(vals[i:], start=i):
                for right in vals[j:]:
                    features.append(left * middle * right)
    return features


def target_features(point: Vec2, params: SimParams) -> list[float]:
    mount = (point[0], point[1] + params.payload_hex_radius)
    anchor_to_mount = sub2(mount, params.anchor)
    distance = max(1e-9, math.hypot(anchor_to_mount[0], anchor_to_mount[1]))
    theta = math.atan2(anchor_to_mount[0], params.anchor[1] - mount[1])
    vertical_efficiency = max(0.0, -anchor_to_mount[1] / distance)
    return [
        point[0] / max(params.wall_width * 0.5, 1e-9),
        point[1] / max(params.wall_height, 1e-9),
        distance / max(params.max_cable_length, 1e-9),
        math.sin(theta),
        math.cos(theta),
        vertical_efficiency,
    ]


def attitude_candidates(sim: WallToolSimulator, cable_axis: Vec2) -> list[float]:
    params = sim.params
    target = sim._hold_attitude_target(cable_axis)
    spread = math.radians(36.0)
    candidates = [target + spread * index / 4.0 for index in range(-4, 5)]
    candidates.append(params.nominal_attitude_rad)
    limit = params.hold_equilibrium_tilt_limit_rad
    return sorted(
        set(
            round(clamp(value, params.nominal_attitude_rad - limit, params.nominal_attitude_rad + limit), 10)
            for value in candidates
        )
    )


def tension_candidates(point: Vec2, attitude: float, cable_axis: Vec2, params: SimParams) -> list[float]:
    weight = params.total_mass * params.gravity
    vertical_efficiency = max(0.0, cable_axis[1])
    useful_hover_tension = weight / max(vertical_efficiency, params.min_cable_vertical_efficiency)
    upper = clamp(1.35 * useful_hover_tension, params.min_tracking_tension, params.max_spool_tension)
    values = [params.min_tracking_tension]
    values.extend(params.min_tracking_tension + (upper - params.min_tracking_tension) * i / 36.0 for i in range(37))
    values.append(upper)
    return sorted(set(round(value, 8) for value in values))


def label_static_hold(point: Vec2, sim: WallToolSimulator) -> HoldLabel:
    params = sim.params
    weight = params.total_mass * params.gravity
    no_cable_hover_each = weight / (2.0 * math.cos(params.hex_face_tilt_rad))
    no_cable_power_index = 2.0 * no_cable_hover_each**1.5

    nominal_mount = sim._cable_mount_position(point, params.nominal_attitude_rad)
    nominal_cable_axis = normalize2((params.anchor[0] - nominal_mount[0], params.anchor[1] - nominal_mount[1]))
    best: tuple[float, float, float, float, float, float, float] | None = None

    for attitude in attitude_candidates(sim, nominal_cable_axis):
        mount = sim._cable_mount_position(point, attitude)
        cable_axis = normalize2((params.anchor[0] - mount[0], params.anchor[1] - mount[1]))
        cable_arm = sim._cable_mount_offset(attitude)
        left_axis, right_axis = sim._drone_axes(attitude)
        left_arm, right_arm = sim._module_center_offsets(attitude)
        torque_scale = max(params.torque_residual_length_scale, 1e-6)

        for tension in tension_candidates(point, attitude, cable_axis, params):
            required_force = sub2((0.0, weight), scale2(cable_axis, tension))
            required_torque = -tension * cross2(cable_arm, cable_axis)
            values, residual = WallToolSimulator._solve_bounded_allocation(
                required=(required_force[0], required_force[1], required_torque / torque_scale),
                axes=(
                    (left_axis[0], left_axis[1], cross2(left_arm, left_axis) / torque_scale),
                    (right_axis[0], right_axis[1], cross2(right_arm, right_axis) / torque_scale),
                ),
                upper_bounds=(params.max_thrust_per_drone, params.max_thrust_per_drone),
                effort_costs=(params.drone_thrust_cost, params.drone_thrust_cost),
            )
            left, right = values
            residual_fraction = residual / max(weight, 1e-9)
            max_thrust_fraction = max(left, right) / max(params.max_thrust_per_drone, 1e-9)
            drone_power_ratio = (left**1.5 + right**1.5) / max(no_cable_power_index, 1e-12)
            cable_support_fraction = max(0.0, tension * cable_axis[1]) / max(weight, 1e-9)
            saturation_penalty = max(0.0, max_thrust_fraction - 0.94)
            over_cable_penalty = max(0.0, cable_support_fraction - 1.20)
            static_cost = (
                drone_power_ratio
                + 3.0 * residual_fraction
                + 2.5 * saturation_penalty * saturation_penalty
                + 0.5 * over_cable_penalty * over_cable_penalty
            )
            if best is None or static_cost < best[0]:
                best = (
                    static_cost,
                    attitude,
                    tension,
                    left,
                    right,
                    residual_fraction,
                    cable_support_fraction,
                )

    if best is None:
        raise RuntimeError("static hold search produced no candidate")

    static_cost, attitude, tension, left, right, residual_fraction, cable_support_fraction = best
    mount = sim._cable_mount_position(point, attitude)
    anchor_delta = sub2(mount, params.anchor)
    cable_length = math.hypot(anchor_delta[0], anchor_delta[1])
    cable_angle = math.atan2(anchor_delta[0], params.anchor[1] - mount[1])
    cable_axis = normalize2((params.anchor[0] - mount[0], params.anchor[1] - mount[1]))
    vertical_efficiency = max(0.0, cable_axis[1])
    max_thrust_fraction = max(left, right) / max(params.max_thrust_per_drone, 1e-9)
    drone_power_ratio = (left**1.5 + right**1.5) / max(no_cable_power_index, 1e-12)
    feasible = residual_fraction <= 0.08 and max_thrust_fraction <= 1.0
    return HoldLabel(
        x=point[0],
        z=point[1],
        cable_angle_rad=cable_angle,
        cable_length_m=cable_length,
        vertical_efficiency=vertical_efficiency,
        hold_attitude_rad=attitude,
        cable_tension_n=tension,
        left_thrust_n=left,
        right_thrust_n=right,
        cable_support_fraction=cable_support_fraction,
        drone_power_ratio=drone_power_ratio,
        residual_fraction=residual_fraction,
        max_thrust_fraction=max_thrust_fraction,
        static_cost=static_cost,
        feasible=feasible,
    )


def sample_points(params: SimParams, grid_x: int, grid_z: int) -> list[Vec2]:
    margin = max(params.cage_radius, params.payload_half_length, params.payload_hex_radius) * 1.4
    xs = np.linspace(-params.wall_width / 2.0 + margin, params.wall_width / 2.0 - margin, grid_x)
    zs = np.linspace(margin, params.wall_height - margin, grid_z)
    return [(float(x), float(z)) for z in zs for x in xs]


def labels_to_rows(labels: Sequence[HoldLabel]) -> list[dict[str, float | int]]:
    return [
        {
            "x": label.x,
            "z": label.z,
            "cable_angle_deg": math.degrees(label.cable_angle_rad),
            "cable_length_m": label.cable_length_m,
            "vertical_efficiency": label.vertical_efficiency,
            "hold_attitude_deg": math.degrees(label.hold_attitude_rad),
            "cable_tension_n": label.cable_tension_n,
            "left_thrust_n": label.left_thrust_n,
            "right_thrust_n": label.right_thrust_n,
            "cable_support_fraction": label.cable_support_fraction,
            "drone_power_ratio": label.drone_power_ratio,
            "residual_fraction": label.residual_fraction,
            "max_thrust_fraction": label.max_thrust_fraction,
            "static_cost": label.static_cost,
            "feasible": int(label.feasible),
        }
        for label in labels
    ]


def save_csv(rows: Sequence[dict[str, float | int]], path: Path) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def fit_ridge_model(features: np.ndarray, targets: np.ndarray, ridge: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    feature_mean = features[:, 1:].mean(axis=0)
    feature_std = features[:, 1:].std(axis=0)
    feature_std[feature_std < 1e-9] = 1.0
    scaled = features.copy()
    scaled[:, 1:] = (features[:, 1:] - feature_mean) / feature_std

    penalty = np.eye(scaled.shape[1])
    penalty[0, 0] = 0.0
    coefficients = np.linalg.solve(scaled.T @ scaled + ridge * penalty, scaled.T @ targets)
    return coefficients, feature_mean, feature_std


def predict_ridge(features: np.ndarray, coefficients: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    scaled = features.copy()
    scaled[:, 1:] = (features[:, 1:] - mean) / std
    return scaled @ coefficients


def train_models(labels: Sequence[HoldLabel], params: SimParams, degree: int, ridge: float) -> dict[str, object]:
    base_names = ["x_norm", "z_norm", "length_norm", "sin_theta", "cos_theta", "vertical_efficiency"]
    feature_names = polynomial_feature_names(base_names, degree)
    feature_matrix = np.array(
        [polynomial_features(target_features((label.x, label.z), params), degree) for label in labels],
        dtype=float,
    )
    targets = {
        "static_cost": np.array([label.static_cost for label in labels], dtype=float),
        "drone_power_ratio": np.array([label.drone_power_ratio for label in labels], dtype=float),
        "residual_fraction": np.array([label.residual_fraction for label in labels], dtype=float),
        "hold_attitude_rad": np.array([label.hold_attitude_rad for label in labels], dtype=float),
    }

    rng = np.random.default_rng(7)
    indices = np.arange(len(labels))
    rng.shuffle(indices)
    split = max(1, int(0.8 * len(indices)))
    train_idx = indices[:split]
    test_idx = indices[split:] if split < len(indices) else indices[:split]

    models: dict[str, object] = {
        "feature_names": feature_names,
        "degree": degree,
        "ridge": ridge,
        "targets": {},
    }
    for name, target in targets.items():
        coefficients, mean, std = fit_ridge_model(feature_matrix[train_idx], target[train_idx], ridge)
        pred_train = predict_ridge(feature_matrix[train_idx], coefficients, mean, std)
        pred_test = predict_ridge(feature_matrix[test_idx], coefficients, mean, std)
        models["targets"][name] = {
            "coefficients": coefficients.tolist(),
            "feature_mean": mean.tolist(),
            "feature_std": std.tolist(),
            "train_rmse": float(np.sqrt(np.mean((pred_train - target[train_idx]) ** 2))),
            "test_rmse": float(np.sqrt(np.mean((pred_test - target[test_idx]) ** 2))),
        }
    return models


def add_predictions(labels: Sequence[HoldLabel], rows: list[dict[str, float | int]], params: SimParams, model: dict[str, object]) -> None:
    degree = int(model["degree"])
    features = np.array(
        [polynomial_features(target_features((label.x, label.z), params), degree) for label in labels],
        dtype=float,
    )
    targets = model["targets"]
    for name, target_model in targets.items():
        coefficients = np.array(target_model["coefficients"], dtype=float)
        mean = np.array(target_model["feature_mean"], dtype=float)
        std = np.array(target_model["feature_std"], dtype=float)
        predictions = predict_ridge(features, coefficients, mean, std)
        for row, prediction in zip(rows, predictions):
            row[f"pred_{name}"] = float(prediction)


def plot_map(labels: Sequence[HoldLabel], rows: Sequence[dict[str, float | int]], params: SimParams, path: Path) -> None:
    xs = np.array([label.x for label in labels], dtype=float)
    zs = np.array([label.z for label in labels], dtype=float)
    actual_cost = np.array([label.static_cost for label in labels], dtype=float)
    pred_cost = np.array([float(row["pred_static_cost"]) for row in rows], dtype=float)
    power = np.array([label.drone_power_ratio for label in labels], dtype=float)
    attitude = np.array([math.degrees(label.hold_attitude_rad) for label in labels], dtype=float)
    feasible = np.array([1.0 if label.feasible else 0.0 for label in labels], dtype=float)

    fig, axes = plt.subplots(2, 3, figsize=(13.0, 7.8), constrained_layout=True)
    plots = (
        (actual_cost, "Physics Label: Static Cost", "magma_r"),
        (pred_cost, "Learned Prediction: Static Cost", "magma_r"),
        (np.abs(pred_cost - actual_cost), "Prediction Error", "viridis"),
        (power, "Drone Power Ratio", "plasma_r"),
        (attitude, "Stable Hold Tilt [deg]", "coolwarm"),
        (feasible, "Static Feasible", "Greens"),
    )

    for ax, (values, title, cmap) in zip(axes.flat, plots):
        scatter = ax.scatter(xs, zs, c=values, cmap=cmap, s=26, edgecolors="none")
        ax.add_patch(
            Rectangle(
                (-params.wall_width / 2.0, 0.0),
                params.wall_width,
                params.wall_height,
                fill=False,
                edgecolor="#555555",
                linewidth=1.0,
            )
        )
        ax.scatter([params.anchor[0]], [params.anchor[1]], s=42, c="#222222")
        ax.set_title(title)
        ax.set_xlabel("wall x [m]")
        ax.set_ylabel("wall z [m]")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color="#dddddd", linewidth=0.6)
        fig.colorbar(scatter, ax=ax, shrink=0.82)

    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a lightweight feasibility model for the wall-tool simulator.")
    parser.add_argument("--grid-x", type=int, default=45)
    parser.add_argument("--grid-z", type=int, default=45)
    parser.add_argument("--degree", type=int, choices=(1, 2, 3), default=3)
    parser.add_argument("--ridge", type=float, default=1e-4)
    parser.add_argument("--output-dir", default="wall_tool_sim/output/learning")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir)
    if not output_dir.is_absolute():
        output_dir = PROJECT_ROOT / output_dir

    params = SimParams()
    sim = WallToolSimulator(params)
    points = sample_points(params, max(3, args.grid_x), max(3, args.grid_z))
    labels = [label_static_hold(point, sim) for point in points]
    rows = labels_to_rows(labels)
    model = train_models(labels, params, args.degree, args.ridge)
    add_predictions(labels, rows, params, model)

    csv_path = output_dir / "feasibility_samples.csv"
    model_path = output_dir / "feasibility_model.json"
    plot_path = output_dir / "learned_feasibility_map.png"
    save_csv(rows, csv_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    with model_path.open("w", encoding="utf-8") as handle:
        json.dump(model, handle, indent=2)
    plot_map(labels, rows, params, plot_path)

    feasible_fraction = sum(label.feasible for label in labels) / max(1, len(labels))
    print(f"samples={len(labels)} feasible_fraction={feasible_fraction:.3f}")
    for name, target_model in model["targets"].items():
        print(
            f"{name}: train_rmse={target_model['train_rmse']:.4f}, "
            f"test_rmse={target_model['test_rmse']:.4f}"
        )
    print(f"saved {csv_path}")
    print(f"saved {model_path}")
    print(f"saved {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
