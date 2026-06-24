#!/usr/bin/env python3
"""Benchmark wall-tool planning/controller variants.

The low-level controller remains the realistic hybrid reel/drone controller.
This script compares higher-level reference strategies:

* direct: current straight or given waypoint command
* center_setup: hand-designed cable-friendly setup waypoint before hard targets
* learned_ranked: learned static-cost map ranks intermediate waypoints
* sim_refined: learned ranking plus dynamic simulation over the top candidates
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
import numpy as np  # noqa: E402


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from wall_tool_sim.learn_feasibility import polynomial_features, predict_ridge, target_features  # noqa: E402
from wall_tool_sim.plot_analysis import cumulative_integral  # noqa: E402
from wall_tool_sim.wall_tool_ui import (  # noqa: E402
    SimParams,
    Vec2,
    WallToolSimulator,
    center_setup_waypoint,
    distance2,
    predictive_waypoints,
)


@dataclass(frozen=True)
class BenchmarkCase:
    name: str
    title: str
    targets: tuple[Vec2, ...]
    duration: float


@dataclass(frozen=True)
class PlannerResult:
    case: str
    planner: str
    waypoints: tuple[Vec2, ...]
    final_error: float
    max_error: float
    rms_error: float
    mean_power_ratio: float
    final_saving: float
    max_thrust_fraction: float
    max_residual_fraction: float
    mean_cable_support: float
    score: float


def benchmark_cases() -> tuple[BenchmarkCase, ...]:
    return (
        BenchmarkCase(
            name="shallow_upper_right",
            title="Shallow Upper Right",
            targets=((1.55, 3.45),),
            duration=24.0,
        ),
        BenchmarkCase(
            name="shallow_upper_left",
            title="Shallow Upper Left",
            targets=((-1.55, 3.45),),
            duration=24.0,
        ),
        BenchmarkCase(
            name="right_work_point",
            title="Right Wall Work Point",
            targets=((1.35, 2.85),),
            duration=22.0,
        ),
        BenchmarkCase(
            name="nominal_scan",
            title="Nominal Scan",
            targets=((0.70, 1.35), (1.15, 2.20), (0.45, 2.85), (-0.85, 2.45), (-1.15, 1.35), (0.0, 2.00)),
            duration=34.0,
        ),
        BenchmarkCase(
            name="upper_sweep",
            title="Upper Sweep",
            targets=((0.85, 1.35), (1.55, 3.45), (-1.55, 3.45), (-1.00, 1.40), (0.0, 2.00)),
            duration=44.0,
        ),
    )


def direct_route(case: BenchmarkCase) -> tuple[Vec2, ...]:
    return snap_route(case.targets)


def snap_waypoint(point: Vec2) -> Vec2:
    return (round(float(point[0]), 5), round(float(point[1]), 5))


def snap_route(points: Sequence[Vec2]) -> tuple[Vec2, ...]:
    return tuple(snap_waypoint(point) for point in points)


def center_setup_route(case: BenchmarkCase, params: SimParams) -> tuple[Vec2, ...]:
    waypoints: list[Vec2] = []
    for target in case.targets:
        setup = center_setup_waypoint(target, params)
        if setup is not None:
            waypoints.append(snap_waypoint(setup))
        waypoints.append(snap_waypoint(target))
    return tuple(waypoints)


def predictive_route(case: BenchmarkCase, params: SimParams) -> tuple[Vec2, ...]:
    current = params.initial_payload
    waypoints: list[Vec2] = []
    for target in case.targets:
        route = predictive_waypoints(current, target, params)
        waypoints.extend(route)
        current = route[-1]
    return tuple(waypoints)


class LearnedCostModel:
    def __init__(self, path: Path, params: SimParams) -> None:
        self.params = params
        with path.open("r", encoding="utf-8") as handle:
            self.model = json.load(handle)
        self.degree = int(self.model["degree"])
        target_model = self.model["targets"]["static_cost"]
        self.coefficients = np.array(target_model["coefficients"], dtype=float)
        self.mean = np.array(target_model["feature_mean"], dtype=float)
        self.std = np.array(target_model["feature_std"], dtype=float)

    def predict_cost(self, point: Vec2) -> float:
        features = np.array([polynomial_features(target_features(point, self.params), self.degree)], dtype=float)
        return float(predict_ridge(features, self.coefficients, self.mean, self.std)[0])


def candidate_waypoints(params: SimParams, grid_x: int, grid_z: int) -> list[Vec2]:
    sim = WallToolSimulator(params)
    margin = max(params.cage_radius, params.payload_half_length, params.payload_hex_radius) * 1.4
    xs = np.linspace(-params.wall_width / 2.0 + margin, params.wall_width / 2.0 - margin, grid_x)
    zs = np.linspace(margin, params.wall_height - margin, grid_z)
    points: list[Vec2] = []
    for z in zs:
        for x in xs:
            point = sim._clamp_wall_point((float(x), float(z)))
            if point[1] <= 3.15:
                points.append(snap_waypoint(point))
    return points


def route_length(points: Sequence[Vec2], start: Vec2) -> float:
    total = 0.0
    current = start
    for point in points:
        total += distance2(current, point)
        current = point
    return total


def learned_ranked_route(case: BenchmarkCase, params: SimParams, cost_model: LearnedCostModel, candidates: Sequence[Vec2]) -> tuple[Vec2, ...]:
    if len(case.targets) != 1:
        return center_setup_route(case, params)
    start = params.initial_payload
    target = snap_waypoint(case.targets[-1])
    robust_route = predictive_route(case, params)
    if len(robust_route) > 1:
        return robust_route
    direct = route_length((target,), start)
    best: tuple[float, Vec2 | None] = (math.inf, None)
    for candidate in candidates:
        detour = route_length((candidate, target), start) / max(direct, 1e-9)
        if detour > 2.25:
            continue
        score = (
            1.35 * cost_model.predict_cost(candidate)
            + 0.35 * max(0.0, detour - 1.0)
            + 0.15 * abs(candidate[0] - 0.25 * target[0])
            + 0.08 * abs(candidate[1] - min(target[1], 2.25))
        )
        if score < best[0]:
            best = (score, candidate)
    if best[1] is None:
        return (target,)
    return (snap_waypoint(best[1]), target)


def run_route(waypoints: Sequence[Vec2], duration: float, params: SimParams) -> list[dict[str, float]]:
    sim = WallToolSimulator(params)
    for waypoint in waypoints:
        sim.append_target(waypoint)

    weight = params.total_mass * params.gravity
    no_cable_hover_each = weight / (2.0 * math.cos(params.hex_face_tilt_rad))
    no_cable_power_index = 2.0 * no_cable_hover_each**1.5

    rows: list[dict[str, float]] = []
    for _ in range(max(1, int(duration / params.dt))):
        state = sim.step()
        drone_power_index = state.left_thrust**1.5 + state.right_thrust**1.5
        rows.append(
            {
                "t": state.t,
                "x": state.tool_head[0],
                "z": state.tool_head[1],
                "ref_x": state.reference[0],
                "ref_z": state.reference[1],
                "error_norm": state.tool_error,
                "drone_power_ratio": drone_power_index / max(no_cable_power_index, 1e-12),
                "left_thrust_fraction": state.left_thrust / max(params.max_thrust_per_drone, 1e-9),
                "right_thrust_fraction": state.right_thrust / max(params.max_thrust_per_drone, 1e-9),
                "allocation_residual_fraction": state.allocation_residual / max(weight, 1e-9),
                "cable_support_fraction": state.cable_vertical_force / max(weight, 1e-9),
                "drone_power_index": drone_power_index,
                "no_cable_power_index": no_cable_power_index,
            }
        )
    drone_power_cumulative = cumulative_integral([row["drone_power_index"] for row in rows], params.dt)
    baseline_power_cumulative = cumulative_integral([row["no_cable_power_index"] for row in rows], params.dt)
    for index, row in enumerate(rows):
        row["cumulative_drone_power_saving_fraction"] = 1.0 - drone_power_cumulative[index] / max(
            baseline_power_cumulative[index], 1e-12
        )
    return rows


def summarize(case: BenchmarkCase, planner: str, waypoints: Sequence[Vec2], rows: Sequence[dict[str, float]]) -> PlannerResult:
    errors = [row["error_norm"] for row in rows]
    powers = [row["drone_power_ratio"] for row in rows]
    max_thrust = max(max(row["left_thrust_fraction"], row["right_thrust_fraction"]) for row in rows)
    max_residual = max(row["allocation_residual_fraction"] for row in rows)
    final_error = errors[-1]
    max_error = max(errors)
    rms_error = math.sqrt(sum(error * error for error in errors) / max(1, len(errors)))
    mean_power = sum(powers) / max(1, len(powers))
    final_saving = rows[-1]["cumulative_drone_power_saving_fraction"]
    mean_cable_support = sum(row["cable_support_fraction"] for row in rows) / max(1, len(rows))
    score = (
        2.2 * final_error
        + 0.9 * max_error
        + 0.5 * rms_error
        + 0.25 * mean_power
        + 0.25 * max(0.0, max_thrust - 0.92) ** 2
        + 0.8 * max_residual
    )
    return PlannerResult(
        case=case.name,
        planner=planner,
        waypoints=tuple(waypoints),
        final_error=final_error,
        max_error=max_error,
        rms_error=rms_error,
        mean_power_ratio=mean_power,
        final_saving=final_saving,
        max_thrust_fraction=max_thrust,
        max_residual_fraction=max_residual,
        mean_cable_support=mean_cable_support,
        score=score,
    )


def simulate_refined_route(
    case: BenchmarkCase,
    params: SimParams,
    cost_model: LearnedCostModel,
    candidates: Sequence[Vec2],
    top_k: int,
) -> tuple[Vec2, ...]:
    if len(case.targets) != 1:
        return center_setup_route(case, params)
    target = snap_waypoint(case.targets[-1])
    start = params.initial_payload
    direct = route_length((target,), start)
    ranked: list[tuple[float, Vec2]] = []
    for candidate in candidates:
        detour = route_length((candidate, target), start) / max(direct, 1e-9)
        if detour > 2.25:
            continue
        score = cost_model.predict_cost(candidate) + 0.22 * max(0.0, detour - 1.0)
        ranked.append((score, candidate))
    routes: list[tuple[Vec2, ...]] = []

    def add_route(route: Sequence[Vec2]) -> None:
        snapped = snap_route(route)
        if snapped not in routes:
            routes.append(snapped)

    add_route((target,))
    add_route(center_setup_route(case, params))
    add_route(predictive_waypoints(start, target, params))
    for _score, candidate in sorted(ranked)[:top_k]:
        add_route((candidate, target))
    best_score = math.inf
    best_route = routes[0]
    for route in routes:
        rows = run_route(route, case.duration, params)
        result = summarize(case, "candidate", route, rows)
        if result.score < best_score:
            best_score = result.score
            best_route = route
    return tuple(best_route)


def result_row(result: PlannerResult) -> dict[str, str | float]:
    return {
        "case": result.case,
        "planner": result.planner,
        "waypoints": " | ".join(f"({x:.3f},{z:.3f})" for x, z in result.waypoints),
        "score": result.score,
        "final_error": result.final_error,
        "max_error": result.max_error,
        "rms_error": result.rms_error,
        "mean_power_ratio": result.mean_power_ratio,
        "final_saving": result.final_saving,
        "max_thrust_fraction": result.max_thrust_fraction,
        "max_residual_fraction": result.max_residual_fraction,
        "mean_cable_support": result.mean_cable_support,
    }


def save_summary(results: Sequence[PlannerResult], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [result_row(result) for result in results]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(results: Sequence[PlannerResult], path: Path) -> None:
    cases = list(dict.fromkeys(result.case for result in results))
    planners = list(dict.fromkeys(result.planner for result in results))
    metrics = (
        ("score", "Composite Score"),
        ("final_error", "Final Error [m]"),
        ("max_error", "Max Error [m]"),
        ("mean_power_ratio", "Mean Drone Power Ratio"),
    )
    x = np.arange(len(cases))
    width = 0.8 / max(1, len(planners))
    fig, axes = plt.subplots(2, 2, figsize=(13.0, 7.5), constrained_layout=True)
    for ax, (metric, title) in zip(axes.flat, metrics):
        for idx, planner in enumerate(planners):
            values = []
            for case in cases:
                match = next(result for result in results if result.case == case and result.planner == planner)
                values.append(getattr(match, metric))
            ax.bar(x + (idx - (len(planners) - 1) / 2.0) * width, values, width=width, label=planner)
        ax.set_title(title)
        ax.set_xticks(x)
        ax.set_xticklabels(cases, rotation=25, ha="right")
        ax.grid(True, axis="y", color="#dddddd", linewidth=0.7)
    axes[0, 0].legend(fontsize=8)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark wall-tool planning/controller variants.")
    parser.add_argument("--output-dir", default="wall_tool_sim/output/planner_benchmark")
    parser.add_argument("--model", default="wall_tool_sim/output/learning/feasibility_model.json")
    parser.add_argument("--candidate-grid-x", type=int, default=17)
    parser.add_argument("--candidate-grid-z", type=int, default=17)
    parser.add_argument("--top-k", type=int, default=10)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir)
    if not output_dir.is_absolute():
        output_dir = PROJECT_ROOT / output_dir
    model_path = Path(args.model)
    if not model_path.is_absolute():
        model_path = PROJECT_ROOT / model_path

    params = SimParams()
    cost_model = LearnedCostModel(model_path, params)
    candidates = candidate_waypoints(params, max(3, args.candidate_grid_x), max(3, args.candidate_grid_z))
    results: list[PlannerResult] = []

    for case in benchmark_cases():
        planners: dict[str, tuple[Vec2, ...]] = {
            "direct": direct_route(case),
            "center_setup": center_setup_route(case, params),
            "predictive": predictive_route(case, params),
            "learned_ranked": learned_ranked_route(case, params, cost_model, candidates),
            "sim_refined": simulate_refined_route(case, params, cost_model, candidates, max(1, args.top_k)),
        }
        for planner_name, route in planners.items():
            rows = run_route(route, case.duration, params)
            result = summarize(case, planner_name, route, rows)
            results.append(result)
            print(
                f"{case.name}/{planner_name}: score={result.score:.3f}, "
                f"final={result.final_error:.3f}, max={result.max_error:.3f}, "
                f"power={result.mean_power_ratio:.3f}, waypoints={len(route)}"
            )

    summary_path = output_dir / "planner_summary.csv"
    plot_path = output_dir / "planner_summary.png"
    save_summary(results, summary_path)
    plot_summary(results, plot_path)
    print(f"saved {summary_path}")
    print(f"saved {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
