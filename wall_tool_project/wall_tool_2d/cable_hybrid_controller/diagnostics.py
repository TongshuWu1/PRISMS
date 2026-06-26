#!/usr/bin/env python3
"""Diagnostics and logging for the chosen controller session."""

from __future__ import annotations

import csv
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from cable_hybrid_controller.controller import (  # noqa: E402
    BEST_PLANNER,
    COVERAGE_CORNER_SPEED,
    MISSION_TRAJECTORY,
    WORK_PLANNER,
    ControllerScenario,
)
from cable_hybrid_controller.facade import (  # noqa: E402
    FacadeMission,
    blur_risk,
    contact_quality,
    coverage_fraction,
    facade_safety_margin,
    in_work_region,
    valid_work_contact,
)
from wall_tool_sim.wall_tool_ui import SimParams, SimState  # noqa: E402


def session_output_dir(base_dir: Path | None = None) -> Path:
    root = base_dir or Path(__file__).resolve().parent / "output" / "sessions"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return root / stamp


def no_cable_hover_power_index(params: SimParams) -> float:
    weight = params.total_mass * params.gravity
    no_cable_hover_each = weight / (2.0 * math.cos(params.hex_face_tilt_rad))
    return 2.0 * no_cable_hover_each**1.5


def state_row(state: SimState, params: SimParams, mission: FacadeMission | None = None) -> dict[str, float | int]:
    weight = params.total_mass * params.gravity
    no_cable_power = no_cable_hover_power_index(params)
    drone_power_index = state.left_thrust**1.5 + state.right_thrust**1.5
    spool_reel_in_power = max(0.0, -state.spool_velocity_cmd * state.tension)
    spool_abs_power = abs(state.spool_velocity_cmd) * state.tension
    max_thrust_fraction = max(state.left_thrust, state.right_thrust) / max(params.max_thrust_per_drone, 1e-9)
    contact_force = state.contact_force
    quality = contact_quality(state, params, mission) if mission else 0.0
    safety_margin = facade_safety_margin(state, params)
    wind_norm = math.hypot(state.wind_force[0], state.wind_force[1])
    payload_speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
    payload_acceleration = math.hypot(state.payload_acceleration[0], state.payload_acceleration[1])
    reference_acceleration = math.hypot(state.reference_acceleration[0], state.reference_acceleration[1])
    tracking_limit = mission.max_tracking_error_m if mission else params.work_contact_tracking_limit_m
    speed_limit = mission.max_cleaning_speed_m_s if mission else params.work_contact_speed_limit_mps
    angular_rate_limit = mission.max_angular_rate_rad_s if mission else params.work_contact_angular_rate_limit_rad_s
    cable_efficiency = max(0.0, math.cos(state.theta))
    return {
        "t_s": state.t,
        "tool_x_m": state.tool_head[0],
        "tool_z_m": state.tool_head[1],
        "ref_x_m": state.reference[0],
        "ref_z_m": state.reference[1],
        "active_target_x_m": state.active_target[0],
        "active_target_z_m": state.active_target[1],
        "final_target_x_m": state.target[0],
        "final_target_z_m": state.target[1],
        "tracking_error_m": state.tool_error,
        "tracking_error_ratio": state.tool_error / max(tracking_limit, 1e-9),
        "measured_error_m": state.measured_tool_error,
        "payload_speed_m_s": payload_speed,
        "payload_speed_ratio": payload_speed / max(speed_limit, 1e-9),
        "payload_accel_x_m_s2": state.payload_acceleration[0],
        "payload_accel_z_m_s2": state.payload_acceleration[1],
        "payload_acceleration_m_s2": payload_acceleration,
        "reference_acceleration_m_s2": reference_acceleration,
        "theta_rad": state.theta,
        "theta_dot_rad_s": state.theta_dot,
        "line_length_m": state.length,
        "line_length_dot_m_s": state.length_dot,
        "cable_vertical_efficiency": cable_efficiency,
        "body_attitude_rad": state.attitude,
        "body_rate_rad_s": state.angular_velocity,
        "body_rate_ratio": abs(state.angular_velocity) / max(angular_rate_limit, 1e-9),
        "cable_payout_m": state.cable_length,
        "cable_stretch_m": state.cable_stretch,
        "tension_N": state.tension,
        "desired_tension_N": state.desired_cable_tension,
        "spool_velocity_cmd_m_s": state.spool_velocity_cmd,
        "left_thrust_N": state.left_thrust,
        "right_thrust_N": state.right_thrust,
        "max_thrust_fraction": max_thrust_fraction,
        "cable_support_fraction": state.cable_vertical_force / max(weight, 1e-9),
        "drone_vertical_support_fraction": state.drone_vertical_force / max(weight, 1e-9),
        "drone_power_ratio": drone_power_index / max(no_cable_power, 1e-12),
        "spool_reel_in_power_W": spool_reel_in_power,
        "spool_abs_power_W": spool_abs_power,
        "wind_force_x_N": state.wind_force[0],
        "wind_force_z_N": state.wind_force[1],
        "wind_force_norm_N": wind_norm,
        "wind_force_fraction_weight": wind_norm / max(weight, 1e-9),
        "normal_gap_m": state.normal_gap,
        "normal_velocity_m_s": state.normal_velocity,
        "normal_acceleration_m_s2": state.normal_acceleration,
        "normal_actuator_force_N": state.normal_actuator_force,
        "normal_wind_force_N": state.normal_wind_force,
        "contact_force_N": contact_force,
        "desired_contact_force_N": state.desired_contact_force,
        "contact_quality": quality,
        "in_work_region": int(in_work_region(state.tool_head, mission)) if mission else 0,
        "work_mode": int(state.work_mode),
        "contact_valid": int(valid_work_contact(state, mission)) if mission else int(state.contact_valid),
        "blur_risk": blur_risk(state),
        "facade_safety_margin": safety_margin,
        "allocation_residual_N": state.allocation_residual,
        "radial_position_error_m": state.radial_position_error_m,
        "radial_velocity_error_m_s": state.radial_velocity_error_m_s,
        "tangential_position_error_m": state.tangential_position_error_m,
        "tangential_velocity_error_m_s": state.tangential_velocity_error_m_s,
        "swing_energy_J": state.swing_energy_J,
        "swing_power_W": state.swing_power_W,
        "clf_margin_W": state.clf_margin_W,
        "clf_projected_accel_m_s2": state.clf_projected_accel_m_s2,
        "active_waypoints": state.active_waypoints,
        "slack": int(state.cable_slack),
        "tension_saturated": int(state.cable_tension_saturated),
        "thrust_limit_active": int(max_thrust_fraction > 0.98),
        "allocation_residual_active": int(state.saturated),
    }


def rows_from_states(
    states: Sequence[SimState],
    params: SimParams,
    mission: FacadeMission | None = None,
) -> list[dict[str, float | int]]:
    return [state_row(state, params, mission) for state in states]


def write_csv(rows: Sequence[dict[str, float | int]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def segment_rows(rows: Sequence[dict[str, float | int]]) -> list[dict[str, float | int]]:
    if not rows:
        return []
    segments: list[list[dict[str, float | int]]] = []
    current: list[dict[str, float | int]] = []
    current_key: tuple[float, float] | None = None
    for row in rows:
        key = (round(float(row["active_target_x_m"]), 5), round(float(row["active_target_z_m"]), 5))
        if current and key != current_key:
            segments.append(current)
            current = []
        current.append(row)
        current_key = key
    if current:
        segments.append(current)

    output: list[dict[str, float | int]] = []
    for index, segment in enumerate(segments, start=1):
        errors = [float(row["tracking_error_m"]) for row in segment]
        powers = [float(row["drone_power_ratio"]) for row in segment]
        thrust = [float(row["max_thrust_fraction"]) for row in segment]
        residuals = [float(row["allocation_residual_N"]) for row in segment]
        support = [float(row["cable_support_fraction"]) for row in segment]
        contact_valid = [float(row["contact_valid"]) for row in segment]
        contact_force = [float(row["contact_force_N"]) for row in segment]
        output.append(
            {
                "segment": index,
                "active_target_x_m": float(segment[0]["active_target_x_m"]),
                "active_target_z_m": float(segment[0]["active_target_z_m"]),
                "start_t_s": float(segment[0]["t_s"]),
                "end_t_s": float(segment[-1]["t_s"]),
                "duration_s": float(segment[-1]["t_s"]) - float(segment[0]["t_s"]),
                "sample_count": len(segment),
                "rms_error_m": _rms(errors),
                "max_error_m": max(errors),
                "mean_drone_power_ratio": _mean(powers),
                "mean_cable_support_fraction": _mean(support),
                "max_thrust_fraction": max(thrust),
                "max_allocation_residual_N": max(residuals),
                "mean_contact_force_N": _mean(contact_force),
                "contact_valid_fraction": _mean(contact_valid),
                "allocation_residual_active_fraction": _mean(
                    [float(row["allocation_residual_active"]) for row in segment]
                ),
            }
        )
    return output


def _mean(values: Sequence[float]) -> float:
    return sum(values) / max(1, len(values))


def _rms(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / max(1, len(values)))


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    bounded_fraction = min(1.0, max(0.0, fraction))
    index = int(bounded_fraction * (len(ordered) - 1))
    return ordered[index]


def invalid_contact_reason_fractions(
    rows: Sequence[dict[str, float | int]],
    mission: FacadeMission | None,
) -> dict[str, float]:
    if not rows or mission is None:
        return {}

    invalid_rows = [row for row in rows if int(row["contact_valid"]) == 0 and int(row["work_mode"]) == 1]
    denominator = max(1, len(invalid_rows))
    footprint_margin = 0.5 * mission.tool_width_m
    counts = {
        "region": 0,
        "contact_low": 0,
        "contact_high": 0,
        "tracking": 0,
        "speed": 0,
        "angular_rate": 0,
    }
    for row in invalid_rows:
        point = (float(row["tool_x_m"]), float(row["tool_z_m"]))
        if not in_work_region(point, mission, footprint_margin):
            counts["region"] += 1
        if float(row["contact_force_N"]) < mission.min_contact_force_N:
            counts["contact_low"] += 1
        if float(row["contact_force_N"]) > mission.max_contact_force_N:
            counts["contact_high"] += 1
        if float(row["tracking_error_m"]) > mission.max_tracking_error_m:
            counts["tracking"] += 1
        if float(row["payload_speed_m_s"]) > mission.max_cleaning_speed_m_s:
            counts["speed"] += 1
        if abs(float(row["body_rate_rad_s"])) > mission.max_angular_rate_rad_s:
            counts["angular_rate"] += 1
    return {name: count / denominator for name, count in counts.items()}


def summarize_session(
    scenario: ControllerScenario,
    params: SimParams,
    states: Sequence[SimState],
    rows: Sequence[dict[str, float | int]],
) -> dict[str, object]:
    if not rows:
        return {"scenario": scenario.name, "planner": BEST_PLANNER, "sample_count": 0}

    dt = params.dt
    errors = [float(row["tracking_error_m"]) for row in rows]
    powers = [float(row["drone_power_ratio"]) for row in rows]
    tensions = [float(row["tension_N"]) for row in rows]
    thrust_fractions = [float(row["max_thrust_fraction"]) for row in rows]
    cable_support = [float(row["cable_support_fraction"]) for row in rows]
    residuals = [float(row["allocation_residual_N"]) for row in rows]
    spool_velocities = [float(row["spool_velocity_cmd_m_s"]) for row in rows]
    spool_accelerations = [
        abs((spool_velocities[index] - spool_velocities[index - 1]) / max(dt, 1e-9))
        for index in range(1, len(spool_velocities))
    ]
    reel_power = [float(row["spool_reel_in_power_W"]) for row in rows]
    abs_spool_power = [float(row["spool_abs_power_W"]) for row in rows]
    wind = [float(row["wind_force_norm_N"]) for row in rows]
    speeds = [float(row["payload_speed_m_s"]) for row in rows]
    accelerations = [float(row["payload_acceleration_m_s2"]) for row in rows]
    acceleration_x = [float(row["payload_accel_x_m_s2"]) for row in rows]
    acceleration_z = [float(row["payload_accel_z_m_s2"]) for row in rows]
    payload_jerks = [
        math.hypot(acceleration_x[index] - acceleration_x[index - 1], acceleration_z[index] - acceleration_z[index - 1])
        / max(dt, 1e-9)
        for index in range(1, len(rows))
    ]
    speed_ratios = [float(row["payload_speed_ratio"]) for row in rows]
    tracking_ratios = [float(row["tracking_error_ratio"]) for row in rows]
    cable_efficiencies = [float(row["cable_vertical_efficiency"]) for row in rows]
    body_rate_ratios = [float(row["body_rate_ratio"]) for row in rows]
    body_rates = [abs(float(row["body_rate_rad_s"])) for row in rows]
    cable_rates = [abs(float(row["theta_dot_rad_s"])) for row in rows]
    radial_errors = [abs(float(row["radial_position_error_m"])) for row in rows]
    tangential_errors = [abs(float(row["tangential_position_error_m"])) for row in rows]
    swing_energies = [float(row["swing_energy_J"]) for row in rows]
    swing_power_abs = [abs(float(row["swing_power_W"])) for row in rows]
    clf_margins = [float(row["clf_margin_W"]) for row in rows]
    contact = [float(row["contact_force_N"]) for row in rows]
    contact_scores = [float(row["contact_quality"]) for row in rows]
    blur_scores = [float(row["blur_risk"]) for row in rows]
    safety_margins = [float(row["facade_safety_margin"]) for row in rows]
    normal_gaps = [float(row["normal_gap_m"]) for row in rows]
    normal_actuator = [abs(float(row["normal_actuator_force_N"])) for row in rows]
    attitudes_deg = [math.degrees(state.attitude) for state in states]
    mission = scenario.facade_mission
    work_rows = [row for row in rows if int(row["work_mode"]) == 1]

    summary = {
        "scenario": scenario.name,
        "description": scenario.description,
        "planner": BEST_PLANNER,
        "facade_work_planner": WORK_PLANNER if scenario.facade_mission else "",
        "controller_params": {
            "wall_width_m": params.wall_width,
            "wall_height_m": params.wall_height,
            "max_cable_length_m": params.max_cable_length,
            "mission_trajectory": MISSION_TRAJECTORY,
            "control_law": params.control_law,
            "path_speed_m_s": params.path_speed,
            "reference_accel_limit_mps2": params.reference_accel_limit_mps2,
            "reference_jerk_limit_mps3": params.reference_jerk_limit_mps3,
            "reference_min_segment_duration_s": params.reference_min_segment_duration_s,
            "coverage_corner_speed_m_s": COVERAGE_CORNER_SPEED,
            "max_spool_speed_m_s": params.max_spool_speed,
            "spool_accel_limit_mps2": params.spool_accel_limit_mps2,
            "max_thrust_per_drone_N": params.max_thrust_per_drone,
            "max_tangential_accel": params.max_tangential_accel,
            "max_cable_support_fraction": params.max_cable_support_fraction,
            "max_spool_tension_N": params.max_spool_tension,
            "min_tracking_tension_N": params.min_tracking_tension,
            "min_cable_vertical_efficiency": params.min_cable_vertical_efficiency,
            "mpc_horizon_steps": params.mpc_horizon_steps,
            "mpc_horizon_dt_s": params.mpc_horizon_dt,
            "mpc_control_period_s": params.mpc_control_period_s,
            "mpc_attitude_limit_rad": params.mpc_attitude_limit_rad,
            "mpc_slack_limit_m": params.mpc_slack_limit_m,
            "mpc_tracking_position_weight": params.mpc_tracking_position_weight,
            "mpc_terminal_position_weight": params.mpc_terminal_position_weight,
            "mpc_drone_effort_weight": params.mpc_drone_effort_weight,
            "mpc_cable_effort_weight": params.mpc_cable_effort_weight,
            "mpc_reel_speed_weight": params.mpc_reel_speed_weight,
            "mpc_input_rate_weight": params.mpc_input_rate_weight,
            "mpc_attitude_rate_weight": params.mpc_attitude_rate_weight,
            "mpc_attitude_weight": params.mpc_attitude_weight,
            "mpc_slack_weight": params.mpc_slack_weight,
        },
        "duration_s": states[-1].t if states else 0.0,
        "max_duration_s": scenario.duration_s,
        "targets": [{"x_m": x, "z_m": z} for x, z in scenario.targets],
        "sample_count": len(rows),
        "final_error_m": errors[-1],
        "max_error_m": max(errors),
        "mean_error_m": _mean(errors),
        "rms_error_m": _rms(errors),
        "p95_error_m": _percentile(errors, 0.95),
        "mean_payload_speed_m_s": _mean(speeds),
        "p95_payload_speed_m_s": _percentile(speeds, 0.95),
        "max_payload_speed_m_s": max(speeds),
        "mean_payload_acceleration_m_s2": _mean(accelerations),
        "p95_payload_acceleration_m_s2": _percentile(accelerations, 0.95),
        "max_payload_acceleration_m_s2": max(accelerations),
        "p95_payload_jerk_m_s3": _percentile(payload_jerks, 0.95),
        "max_payload_jerk_m_s3": max(payload_jerks) if payload_jerks else 0.0,
        "p95_tracking_error_ratio": _percentile(tracking_ratios, 0.95),
        "p95_payload_speed_ratio": _percentile(speed_ratios, 0.95),
        "p95_body_rate_ratio": _percentile(body_rate_ratios, 0.95),
        "p95_body_rate_rad_s": _percentile(body_rates, 0.95),
        "p99_body_rate_rad_s": _percentile(body_rates, 0.99),
        "p95_cable_rate_rad_s": _percentile(cable_rates, 0.95),
        "p95_radial_error_m": _percentile(radial_errors, 0.95),
        "p95_tangential_error_m": _percentile(tangential_errors, 0.95),
        "mean_swing_energy_J": _mean(swing_energies),
        "p95_swing_energy_J": _percentile(swing_energies, 0.95),
        "max_swing_energy_J": max(swing_energies),
        "p95_abs_swing_power_W": _percentile(swing_power_abs, 0.95),
        "clf_violation_fraction": _mean([1.0 if value < -1e-8 else 0.0 for value in clf_margins]),
        "mean_drone_power_ratio": _mean(powers),
        "max_drone_power_ratio": max(powers),
        "mean_cable_support_fraction": _mean(cable_support),
        "min_cable_support_fraction": min(cable_support),
        "mean_cable_vertical_efficiency": _mean(cable_efficiencies),
        "min_cable_vertical_efficiency": min(cable_efficiencies),
        "mean_tension_N": _mean(tensions),
        "max_tension_N": max(tensions),
        "max_thrust_fraction": max(thrust_fractions),
        "mean_allocation_residual_N": _mean(residuals),
        "max_allocation_residual_N": max(residuals),
        "mean_abs_spool_velocity_m_s": _mean([abs(value) for value in spool_velocities]),
        "p95_spool_acceleration_m_s2": _percentile(spool_accelerations, 0.95),
        "max_spool_acceleration_m_s2": max(spool_accelerations) if spool_accelerations else 0.0,
        "reel_in_work_J": sum(reel_power) * dt,
        "absolute_spool_work_proxy_J": sum(abs_spool_power) * dt,
        "slack_sample_fraction": _mean([float(row["slack"]) for row in rows]),
        "tension_saturation_fraction": _mean([float(row["tension_saturated"]) for row in rows]),
        "thrust_limit_active_fraction": _mean([float(row["thrust_limit_active"]) for row in rows]),
        "allocation_residual_active_fraction": _mean([float(row["allocation_residual_active"]) for row in rows]),
        "mean_wind_force_N": _mean(wind),
        "max_wind_force_N": max(wind),
        "mean_contact_force_N": _mean(contact),
        "max_contact_force_N": max(contact),
        "min_contact_force_N": min(contact),
        "mean_contact_quality": _mean(contact_scores),
        "contact_valid_fraction": _mean([float(row["contact_valid"]) for row in rows]),
        "work_mode_contact_valid_fraction": _mean([float(row["contact_valid"]) for row in work_rows])
        if work_rows
        else 0.0,
        "invalid_contact_reason_fraction": invalid_contact_reason_fractions(rows, mission),
        "work_mode_fraction": _mean([float(row["work_mode"]) for row in rows]),
        "mean_normal_gap_m": _mean(normal_gaps),
        "min_normal_gap_m": min(normal_gaps),
        "max_normal_gap_m": max(normal_gaps),
        "mean_abs_normal_actuator_force_N": _mean(normal_actuator),
        "mean_blur_risk": _mean(blur_scores),
        "max_blur_risk": max(blur_scores),
        "min_facade_safety_margin": min(safety_margins),
        "min_attitude_deg": min(attitudes_deg),
        "max_attitude_deg": max(attitudes_deg),
    }
    if mission:
        summary["facade_mission"] = {
            "x_min_m": mission.x_min,
            "x_max_m": mission.x_max,
            "z_min_m": mission.z_min,
            "z_max_m": mission.z_max,
            "tool_width_m": mission.tool_width_m,
            "desired_contact_force_N": mission.desired_contact_force_N,
        }
        summary["coverage_fraction"] = coverage_fraction(states, mission)
    return summary


def write_summary(summary: dict[str, object], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")


def write_report(summary: dict[str, object], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    metric_names = (
        "final_error_m",
        "max_error_m",
        "rms_error_m",
        "p95_error_m",
        "mean_payload_speed_m_s",
        "p95_payload_speed_m_s",
        "max_payload_speed_m_s",
        "mean_payload_acceleration_m_s2",
        "p95_payload_acceleration_m_s2",
        "max_payload_acceleration_m_s2",
        "p95_payload_jerk_m_s3",
        "max_payload_jerk_m_s3",
        "p95_tracking_error_ratio",
        "p95_payload_speed_ratio",
        "p95_body_rate_ratio",
        "p95_body_rate_rad_s",
        "p99_body_rate_rad_s",
        "p95_cable_rate_rad_s",
        "p95_radial_error_m",
        "p95_tangential_error_m",
        "mean_swing_energy_J",
        "p95_swing_energy_J",
        "max_swing_energy_J",
        "p95_abs_swing_power_W",
        "clf_violation_fraction",
        "mean_drone_power_ratio",
        "mean_cable_support_fraction",
        "mean_cable_vertical_efficiency",
        "min_cable_vertical_efficiency",
        "max_thrust_fraction",
        "max_tension_N",
        "mean_abs_spool_velocity_m_s",
        "p95_spool_acceleration_m_s2",
        "max_spool_acceleration_m_s2",
        "reel_in_work_J",
        "thrust_limit_active_fraction",
        "allocation_residual_active_fraction",
        "slack_sample_fraction",
        "coverage_fraction",
        "contact_valid_fraction",
        "work_mode_contact_valid_fraction",
        "mean_contact_force_N",
        "mean_contact_quality",
        "mean_normal_gap_m",
        "mean_blur_risk",
        "min_facade_safety_margin",
        "max_wind_force_N",
    )
    lines = [
        "# Controller Session Report",
        "",
        f"- Scenario: `{summary.get('scenario')}`",
        f"- Planner/controller: `{summary.get('planner')}`",
        f"- Duration: `{summary.get('duration_s')}` s",
        "",
        "## Key Metrics",
        "",
    ]
    for name in metric_names:
        value = summary.get(name)
        if isinstance(value, float):
            lines.append(f"- `{name}`: `{value:.5f}`")
        else:
            lines.append(f"- `{name}`: `{value}`")
    controller_params = summary.get("controller_params")
    if isinstance(controller_params, dict):
        lines.append("")
        lines.append("## Controller Settings")
        lines.append("")
        for name, value in controller_params.items():
            if isinstance(value, float):
                lines.append(f"- `{name}`: `{value:.5f}`")
            else:
                lines.append(f"- `{name}`: `{value}`")
    invalid_reasons = summary.get("invalid_contact_reason_fraction")
    if isinstance(invalid_reasons, dict) and invalid_reasons:
        lines.append("")
        lines.append("## Invalid Contact Reasons")
        lines.append("")
        for name, value in invalid_reasons.items():
            if isinstance(value, float):
                lines.append(f"- `{name}`: `{value:.5f}`")
    lines.append("")
    lines.append("## Interpretation")
    lines.append("")
    lines.append("- Low tracking error with moderate cable support means the cable is doing useful work.")
    lines.append("- High thrust-limit activity means the route/control request is too aggressive.")
    lines.append("- High allocation residual activity means the requested force/torque combination is not fully feasible.")
    lines.append("- Slack or tension saturation should be treated as a controller failure mode, not a cosmetic plot issue.")
    lines.append("- For cleaning, coverage is counted only when normal contact force, speed, attitude rate, and tracking are valid.")
    lines.append("- Low contact quality means the tool is under-pressing, over-pressing, or outside the facade work bay.")
    lines.append("- Blur risk matters for inspection because images become less useful as vibration and speed rise.")
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_session(
    rows: Sequence[dict[str, float | int]],
    output_dir: Path,
    mission: FacadeMission | None = None,
    params: SimParams | None = None,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    t = [float(row["t_s"]) for row in rows]

    fig, axes = plt.subplots(2, 1, figsize=(10.5, 8.0), constrained_layout=True)
    axes[0].plot([float(row["tool_x_m"]) for row in rows], [float(row["tool_z_m"]) for row in rows], label="tool")
    axes[0].plot([float(row["ref_x_m"]) for row in rows], [float(row["ref_z_m"]) for row in rows], "--", label="reference")
    axes[0].scatter(
        [float(row["active_target_x_m"]) for row in rows[:: max(1, len(rows) // 80)]],
        [float(row["active_target_z_m"]) for row in rows[:: max(1, len(rows) // 80)]],
        s=8,
        label="active target",
    )
    axes[0].set_title("Wall-Plane Motion")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("z [m]")
    axes[0].axis("equal")
    axes[0].grid(True, color="#dddddd")
    axes[0].legend()
    axes[1].plot(t, [float(row["tracking_error_m"]) for row in rows], label="true error")
    axes[1].plot(t, [float(row["measured_error_m"]) for row in rows], "--", label="sensor-estimated error")
    axes[1].set_title("Tracking Error")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("error [m]")
    axes[1].grid(True, color="#dddddd")
    axes[1].legend()
    fig.savefig(output_dir / "tracking.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(11.0, 10.0), sharex=True, constrained_layout=True)
    axes[0].plot(t, [float(row["tension_N"]) for row in rows], label="actual")
    axes[0].plot(t, [float(row["desired_tension_N"]) for row in rows], "--", label="desired")
    axes[0].set_ylabel("tension [N]")
    axes[0].legend()
    axes[1].plot(t, [float(row["spool_velocity_cmd_m_s"]) for row in rows])
    axes[1].set_ylabel("spool v [m/s]")
    axes[2].plot(t, [float(row["left_thrust_N"]) for row in rows], label="left")
    axes[2].plot(t, [float(row["right_thrust_N"]) for row in rows], label="right")
    axes[2].set_ylabel("thrust [N]")
    axes[2].legend()
    axes[3].plot(t, [float(row["allocation_residual_N"]) for row in rows])
    axes[3].set_ylabel("alloc residual")
    axes[3].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(True, color="#dddddd")
    fig.savefig(output_dir / "actuation.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(11.0, 8.5), sharex=True, constrained_layout=True)
    axes[0].plot(t, [math.degrees(float(row["body_attitude_rad"])) for row in rows])
    axes[0].set_ylabel("body attitude [deg]")
    axes[1].plot(t, [math.degrees(float(row["theta_rad"])) for row in rows])
    axes[1].set_ylabel("cable angle [deg]")
    axes[2].plot(t, [float(row["line_length_m"]) for row in rows], label="line")
    axes[2].plot(t, [float(row["cable_payout_m"]) for row in rows], "--", label="spool payout")
    axes[2].set_ylabel("length [m]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend()
    for ax in axes:
        ax.grid(True, color="#dddddd")
    fig.savefig(output_dir / "attitude_cable.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(11.0, 8.5), sharex=True, constrained_layout=True)
    axes[0].plot(t, [float(row["cable_support_fraction"]) for row in rows], label="cable")
    axes[0].plot(t, [float(row["drone_vertical_support_fraction"]) for row in rows], label="side motors")
    axes[0].set_ylabel("weight support")
    axes[0].legend()
    axes[1].plot(t, [float(row["drone_power_ratio"]) for row in rows])
    axes[1].set_ylabel("motor power ratio")
    axes[2].plot(t, [float(row["spool_reel_in_power_W"]) for row in rows], label="reel-in power")
    axes[2].set_ylabel("power [W]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend()
    for ax in axes:
        ax.grid(True, color="#dddddd")
    fig.savefig(output_dir / "efficiency.png", dpi=180)
    plt.close(fig)

    if mission:
        fig, axes = plt.subplots(4, 1, figsize=(11.0, 12.0), constrained_layout=True)
        axes[0].plot([float(row["tool_x_m"]) for row in rows], [float(row["tool_z_m"]) for row in rows], label="tool")
        axes[0].plot([float(row["ref_x_m"]) for row in rows], [float(row["ref_z_m"]) for row in rows], "--", label="reference")
        rect_x = [mission.x_min, mission.x_max, mission.x_max, mission.x_min, mission.x_min]
        rect_z = [mission.z_min, mission.z_min, mission.z_max, mission.z_max, mission.z_min]
        axes[0].plot(rect_x, rect_z, color="#111111", linewidth=1.4, label="work region")
        axes[0].set_title("Facade Coverage Path")
        axes[0].set_xlabel("x [m]")
        axes[0].set_ylabel("z [m]")
        axes[0].axis("equal")
        axes[0].grid(True, color="#dddddd")
        axes[0].legend()
        axes[1].plot(t, [float(row["contact_force_N"]) for row in rows], label="contact")
        axes[1].plot(t, [float(row["desired_contact_force_N"]) for row in rows], "--", label="desired")
        axes[1].axhline(mission.min_contact_force_N, color="#777777", linestyle="--", linewidth=1.0, label="min")
        axes[1].axhline(mission.max_contact_force_N, color="#777777", linestyle=":", linewidth=1.0, label="max")
        axes[1].set_title("Cleaning Contact Force")
        axes[1].set_ylabel("force [N]")
        axes[1].grid(True, color="#dddddd")
        axes[1].legend()
        axes[2].plot(t, [1000.0 * float(row["normal_gap_m"]) for row in rows], label="normal gap")
        axes[2].plot(t, [float(row["normal_actuator_force_N"]) for row in rows], label="normal actuator")
        axes[2].plot(t, [float(row["normal_wind_force_N"]) for row in rows], label="normal wind")
        axes[2].set_title("Normal-Axis Contact Dynamics")
        axes[2].set_ylabel("mm / N")
        axes[2].grid(True, color="#dddddd")
        axes[2].legend()
        axes[3].plot(t, [float(row["contact_quality"]) for row in rows], label="contact quality")
        axes[3].plot(t, [float(row["contact_valid"]) for row in rows], label="valid contact")
        axes[3].plot(t, [float(row["blur_risk"]) for row in rows], label="blur risk")
        axes[3].plot(t, [float(row["wind_force_fraction_weight"]) for row in rows], label="wind / weight")
        axes[3].set_title("Facade Work Quality And Disturbance")
        axes[3].set_xlabel("time [s]")
        axes[3].grid(True, color="#dddddd")
        axes[3].legend()
        fig.savefig(output_dir / "facade_work.png", dpi=180)
        plt.close(fig)

    plot_controller_dashboard(rows, output_dir, mission, params)
    plot_coverage_map(rows, output_dir, mission)
    plot_limit_margins(rows, output_dir, mission, params)
    plot_smoothness(rows, output_dir, params)
    plot_efficiency_phase(rows, output_dir, params)
    plot_segment_scorecard(rows, output_dir)


def _values(rows: Sequence[dict[str, float | int]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def _sample_rows(rows: Sequence[dict[str, float | int]], max_points: int = 3500) -> list[dict[str, float | int]]:
    stride = max(1, len(rows) // max_points)
    return list(rows[::stride])


def _coverage_cells(rows: Sequence[dict[str, float | int]], mission: FacadeMission) -> tuple[int, int, set[tuple[int, int]]]:
    cols = max(1, int(math.ceil((mission.x_max - mission.x_min) / mission.coverage_cell_m)))
    grid_rows = max(1, int(math.ceil((mission.z_max - mission.z_min) / mission.coverage_cell_m)))
    covered: set[tuple[int, int]] = set()
    footprint_radius = 0.5 * mission.tool_width_m
    cell_radius = max(0, int(math.ceil(footprint_radius / mission.coverage_cell_m)))
    for row in rows:
        if int(row["contact_valid"]) == 0:
            continue
        col = int((float(row["tool_x_m"]) - mission.x_min) / mission.coverage_cell_m)
        grid_row = int((float(row["tool_z_m"]) - mission.z_min) / mission.coverage_cell_m)
        for dc in range(-cell_radius, cell_radius + 1):
            for dr in range(-cell_radius, cell_radius + 1):
                cc = col + dc
                rr = grid_row + dr
                if 0 <= cc < cols and 0 <= rr < grid_rows:
                    covered.add((cc, rr))
    return cols, grid_rows, covered


def plot_controller_dashboard(
    rows: Sequence[dict[str, float | int]],
    output_dir: Path,
    mission: FacadeMission | None,
    params: SimParams | None,
) -> None:
    if not rows:
        return
    t = _values(rows, "t_s")
    errors = _values(rows, "tracking_error_m")
    metrics = {
        "coverage": coverage_fraction_from_rows(rows, mission),
        "valid contact": _mean(_values(rows, "contact_valid")),
        "cable support": _mean(_values(rows, "cable_support_fraction")),
        "1 - motor power": max(0.0, 1.0 - _mean(_values(rows, "drone_power_ratio"))),
        "thrust margin": max(0.0, 1.0 - max(_values(rows, "max_thrust_fraction"))),
        "no slack": 1.0 - _mean(_values(rows, "slack")),
    }
    sample = _sample_rows(rows)

    fig, axes = plt.subplots(2, 3, figsize=(15.0, 8.8), constrained_layout=True)
    valid_points = [row for row in sample if int(row["contact_valid"]) == 1]
    invalid_points = [row for row in sample if int(row["contact_valid"]) == 0]
    axes[0, 0].plot(_values(sample, "ref_x_m"), _values(sample, "ref_z_m"), color="#777777", linewidth=1.0, label="reference")
    axes[0, 0].scatter(
        [float(row["tool_x_m"]) for row in valid_points],
        [float(row["tool_z_m"]) for row in valid_points],
        s=2,
        color="#2ca25f",
        label="valid",
    )
    axes[0, 0].scatter(
        [float(row["tool_x_m"]) for row in invalid_points],
        [float(row["tool_z_m"]) for row in invalid_points],
        s=4,
        color="#d95f0e",
        label="invalid",
    )
    if mission:
        rect_x = [mission.x_min, mission.x_max, mission.x_max, mission.x_min, mission.x_min]
        rect_z = [mission.z_min, mission.z_min, mission.z_max, mission.z_max, mission.z_min]
        axes[0, 0].plot(rect_x, rect_z, color="#111111", linewidth=1.2)
    axes[0, 0].set_title("Path Colored By Valid Contact")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("z [m]")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, color="#dddddd")
    axes[0, 0].legend(fontsize=7)

    axes[0, 1].bar(list(metrics.keys()), list(metrics.values()), color="#4c78a8")
    axes[0, 1].set_ylim(0.0, 1.05)
    axes[0, 1].set_title("Controller Scorecard")
    axes[0, 1].tick_params(axis="x", rotation=35, labelsize=7)
    axes[0, 1].grid(True, axis="y", color="#dddddd")

    sorted_errors = sorted(errors)
    cdf = [(index + 1) / len(sorted_errors) for index in range(len(sorted_errors))]
    axes[0, 2].plot(sorted_errors, cdf, color="#111111")
    if mission:
        axes[0, 2].axvline(mission.max_tracking_error_m, color="#d95f0e", linestyle="--", label="limit")
    axes[0, 2].set_title("Tracking Error CDF")
    axes[0, 2].set_xlabel("error [m]")
    axes[0, 2].set_ylabel("fraction")
    axes[0, 2].grid(True, color="#dddddd")
    axes[0, 2].legend(fontsize=7)

    axes[1, 0].plot(t, _values(rows, "tracking_error_ratio"), label="tracking / limit")
    axes[1, 0].plot(t, _values(rows, "payload_speed_ratio"), label="speed / limit")
    axes[1, 0].plot(t, _values(rows, "body_rate_ratio"), label="body rate / limit")
    axes[1, 0].axhline(1.0, color="#d95f0e", linestyle="--", linewidth=1.0)
    axes[1, 0].set_title("Cleaning Limit Ratios")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].grid(True, color="#dddddd")
    axes[1, 0].legend(fontsize=7)

    axes[1, 1].plot(t, _values(rows, "cable_support_fraction"), label="cable support")
    axes[1, 1].plot(t, _values(rows, "drone_power_ratio"), label="motor power ratio")
    axes[1, 1].plot(t, _values(rows, "max_thrust_fraction"), label="max thrust fraction")
    axes[1, 1].set_title("Efficiency And Actuator Use")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].grid(True, color="#dddddd")
    axes[1, 1].legend(fontsize=7)

    reason_fractions = invalid_contact_reason_fractions(rows, mission)
    if reason_fractions:
        axes[1, 2].bar(list(reason_fractions.keys()), list(reason_fractions.values()), color="#e45756")
        axes[1, 2].set_ylim(0.0, 1.05)
    axes[1, 2].set_title("Invalid Contact Reason Fractions")
    axes[1, 2].tick_params(axis="x", rotation=35, labelsize=7)
    axes[1, 2].grid(True, axis="y", color="#dddddd")
    fig.savefig(output_dir / "controller_dashboard.png", dpi=180)
    plt.close(fig)


def coverage_fraction_from_rows(rows: Sequence[dict[str, float | int]], mission: FacadeMission | None) -> float:
    if not rows or mission is None:
        return 0.0
    cols, grid_rows, covered = _coverage_cells(rows, mission)
    return len(covered) / max(1, cols * grid_rows)


def plot_coverage_map(rows: Sequence[dict[str, float | int]], output_dir: Path, mission: FacadeMission | None) -> None:
    if not rows or mission is None:
        return
    cols, grid_rows, covered = _coverage_cells(rows, mission)
    grid = [[0.0 for _ in range(cols)] for _ in range(grid_rows)]
    for col, row in covered:
        grid[row][col] = 1.0
    sample = _sample_rows(rows)

    fig, axes = plt.subplots(1, 2, figsize=(13.0, 5.5), constrained_layout=True)
    axes[0].imshow(
        grid,
        origin="lower",
        extent=(mission.x_min, mission.x_max, mission.z_min, mission.z_max),
        cmap="Greens",
        vmin=0.0,
        vmax=1.0,
        aspect="auto",
    )
    axes[0].plot(_values(sample, "tool_x_m"), _values(sample, "tool_z_m"), color="#1f1f1f", linewidth=0.8, alpha=0.72)
    axes[0].set_title("Contact-Gated Coverage Cells")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("z [m]")
    axes[0].grid(True, color="#dddddd", linewidth=0.5)

    invalid = [row for row in sample if int(row["contact_valid"]) == 0 and int(row["work_mode"]) == 1]
    axes[1].plot(_values(sample, "tool_x_m"), _values(sample, "tool_z_m"), color="#999999", linewidth=0.8)
    axes[1].scatter(
        [float(row["tool_x_m"]) for row in invalid],
        [float(row["tool_z_m"]) for row in invalid],
        s=5,
        color="#d95f0e",
        label="invalid samples",
    )
    axes[1].set_title("Where Valid Contact Was Lost")
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("z [m]")
    axes[1].axis("equal")
    axes[1].grid(True, color="#dddddd")
    axes[1].legend(fontsize=8)
    fig.savefig(output_dir / "coverage_map.png", dpi=180)
    plt.close(fig)


def plot_limit_margins(
    rows: Sequence[dict[str, float | int]],
    output_dir: Path,
    mission: FacadeMission | None,
    params: SimParams | None,
) -> None:
    if not rows:
        return
    t = _values(rows, "t_s")
    weight = params.total_mass * params.gravity if params else 1.0
    residual_ratio = [float(row["allocation_residual_N"]) / max(weight, 1e-9) for row in rows]
    spool_velocity = _values(rows, "spool_velocity_cmd_m_s")
    dt = (t[1] - t[0]) if len(t) > 1 else 1.0
    spool_accel = [0.0] + [abs((spool_velocity[index] - spool_velocity[index - 1]) / max(dt, 1e-9)) for index in range(1, len(rows))]
    spool_accel_limit = params.spool_accel_limit_mps2 if params else max(spool_accel + [1.0])
    spool_accel_ratio = [value / max(spool_accel_limit, 1e-9) for value in spool_accel]

    fig, axes = plt.subplots(5, 1, figsize=(12.0, 13.0), sharex=True, constrained_layout=True)
    axes[0].plot(t, _values(rows, "tracking_error_ratio"), label="tracking")
    axes[0].plot(t, _values(rows, "payload_speed_ratio"), label="speed")
    axes[0].plot(t, _values(rows, "body_rate_ratio"), label="body rate")
    axes[0].axhline(1.0, color="#d95f0e", linestyle="--")
    axes[0].set_ylabel("ratio")
    axes[0].set_title("Task Limit Ratios")
    axes[0].legend(fontsize=8)

    axes[1].plot(t, _values(rows, "contact_force_N"), label="contact force")
    if mission:
        axes[1].axhline(mission.min_contact_force_N, color="#777777", linestyle="--", label="min")
        axes[1].axhline(mission.max_contact_force_N, color="#777777", linestyle=":", label="max")
    axes[1].set_ylabel("force [N]")
    axes[1].set_title("Contact Force Margin")
    axes[1].legend(fontsize=8)

    axes[2].plot(t, _values(rows, "max_thrust_fraction"), label="max thrust")
    axes[2].plot(t, residual_ratio, label="allocation residual / weight")
    axes[2].axhline(1.0, color="#d95f0e", linestyle="--")
    axes[2].set_ylabel("ratio")
    axes[2].set_title("Actuator Feasibility")
    axes[2].legend(fontsize=8)

    axes[3].plot(t, _values(rows, "tension_N"), label="actual")
    axes[3].plot(t, _values(rows, "desired_tension_N"), "--", label="desired")
    axes[3].plot(t, _values(rows, "slack"), label="slack flag")
    axes[3].set_ylabel("N / flag")
    axes[3].set_title("Cable Tension Health")
    axes[3].legend(fontsize=8)

    axes[4].plot(t, spool_velocity, label="spool velocity")
    axes[4].plot(t, spool_accel_ratio, label="spool accel / limit")
    axes[4].axhline(1.0, color="#d95f0e", linestyle="--")
    axes[4].set_ylabel("m/s / ratio")
    axes[4].set_xlabel("time [s]")
    axes[4].set_title("Reel Smoothness")
    axes[4].legend(fontsize=8)
    for ax in axes:
        ax.grid(True, color="#dddddd")
    fig.savefig(output_dir / "limit_margins.png", dpi=180)
    plt.close(fig)


def plot_smoothness(rows: Sequence[dict[str, float | int]], output_dir: Path, params: SimParams | None) -> None:
    if not rows:
        return
    t = _values(rows, "t_s")
    dt = (t[1] - t[0]) if len(t) > 1 else 1.0
    accel_x = _values(rows, "payload_accel_x_m_s2")
    accel_z = _values(rows, "payload_accel_z_m_s2")
    acceleration = _values(rows, "payload_acceleration_m_s2")
    jerk = [0.0] + [
        math.hypot(accel_x[index] - accel_x[index - 1], accel_z[index] - accel_z[index - 1]) / max(dt, 1e-9)
        for index in range(1, len(rows))
    ]
    spool_velocity = _values(rows, "spool_velocity_cmd_m_s")
    spool_accel = [0.0] + [
        abs((spool_velocity[index] - spool_velocity[index - 1]) / max(dt, 1e-9))
        for index in range(1, len(rows))
    ]
    spool_accel_limit = params.spool_accel_limit_mps2 if params else max(spool_accel + [1.0])
    spool_accel_ratio = [value / max(spool_accel_limit, 1e-9) for value in spool_accel]

    fig, axes = plt.subplots(5, 1, figsize=(12.0, 13.0), sharex=True, constrained_layout=True)
    axes[0].plot(t, _values(rows, "payload_speed_m_s"), label="payload speed")
    axes[0].set_ylabel("m/s")
    axes[0].set_title("Tool Speed")
    axes[0].legend(fontsize=8)

    axes[1].plot(t, acceleration, label="payload acceleration")
    axes[1].plot(t, [value / 10.0 for value in jerk], label="payload jerk / 10")
    axes[1].set_ylabel("m/s^2")
    axes[1].set_title("Payload Smoothness")
    axes[1].legend(fontsize=8)

    axes[2].plot(t, [math.degrees(abs(float(row["body_rate_rad_s"]))) for row in rows], label="body rate")
    axes[2].plot(t, [math.degrees(abs(float(row["theta_dot_rad_s"]))) for row in rows], label="cable angular rate")
    axes[2].set_ylabel("deg/s")
    axes[2].set_title("Oscillation Indicators")
    axes[2].legend(fontsize=8)

    axes[3].plot(t, _values(rows, "swing_energy_J"), label="tracking storage")
    axes[3].plot(t, _values(rows, "swing_power_W"), label="storage rate")
    axes[3].plot(t, _values(rows, "clf_margin_W"), label="CLF margin")
    axes[3].axhline(0.0, color="#777777", linestyle="--", linewidth=1.0)
    axes[3].set_ylabel("J / W")
    axes[3].set_title("MPC Tracking Energy")
    axes[3].legend(fontsize=8)

    axes[4].plot(t, spool_velocity, label="spool velocity")
    axes[4].plot(t, spool_accel_ratio, label="spool accel / limit")
    axes[4].axhline(1.0, color="#d95f0e", linestyle="--", linewidth=1.0)
    axes[4].set_ylabel("m/s / ratio")
    axes[4].set_xlabel("time [s]")
    axes[4].set_title("Reel Smoothness")
    axes[4].legend(fontsize=8)

    for ax in axes:
        ax.grid(True, color="#dddddd")
    fig.savefig(output_dir / "smoothness.png", dpi=180)
    plt.close(fig)


def plot_efficiency_phase(rows: Sequence[dict[str, float | int]], output_dir: Path, params: SimParams | None) -> None:
    if not rows:
        return
    sample = _sample_rows(rows)
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 9.0), constrained_layout=True)
    scatter = axes[0, 0].scatter(
        _values(sample, "cable_vertical_efficiency"),
        _values(sample, "drone_power_ratio"),
        c=_values(sample, "tracking_error_m"),
        s=5,
        cmap="viridis",
    )
    axes[0, 0].set_title("Cable Geometry vs Motor Power")
    axes[0, 0].set_xlabel("cable vertical efficiency")
    axes[0, 0].set_ylabel("motor power ratio")
    fig.colorbar(scatter, ax=axes[0, 0], label="tracking error [m]")

    axes[0, 1].scatter(
        _values(sample, "cable_support_fraction"),
        _values(sample, "drone_power_ratio"),
        c=_values(sample, "max_thrust_fraction"),
        s=5,
        cmap="plasma",
    )
    axes[0, 1].set_title("Load Sharing Tradeoff")
    axes[0, 1].set_xlabel("cable support fraction")
    axes[0, 1].set_ylabel("motor power ratio")

    axes[1, 0].scatter(
        _values(sample, "spool_velocity_cmd_m_s"),
        _values(sample, "tension_N"),
        c=_values(sample, "spool_abs_power_W"),
        s=5,
        cmap="magma",
    )
    axes[1, 0].set_title("Reel Work Phase Plot")
    axes[1, 0].set_xlabel("spool velocity [m/s]")
    axes[1, 0].set_ylabel("tension [N]")

    weight = params.total_mass * params.gravity if params else 1.0
    residual_ratio = [float(row["allocation_residual_N"]) / max(weight, 1e-9) for row in sample]
    axes[1, 1].scatter(_values(sample, "max_thrust_fraction"), residual_ratio, c=_values(sample, "tracking_error_m"), s=5, cmap="viridis")
    axes[1, 1].set_title("Feasibility Phase Plot")
    axes[1, 1].set_xlabel("max thrust fraction")
    axes[1, 1].set_ylabel("allocation residual / weight")
    for ax in axes.ravel():
        ax.grid(True, color="#dddddd")
    fig.savefig(output_dir / "efficiency_phase.png", dpi=180)
    plt.close(fig)


def plot_segment_scorecard(rows: Sequence[dict[str, float | int]], output_dir: Path) -> None:
    segments = segment_rows(rows)
    if not segments:
        return
    segment_ids = [int(row["segment"]) for row in segments]
    fig, axes = plt.subplots(4, 1, figsize=(12.0, 10.0), sharex=True, constrained_layout=True)
    axes[0].bar(segment_ids, _values(segments, "duration_s"), color="#4c78a8")
    axes[0].set_ylabel("duration [s]")
    axes[0].set_title("Per-Segment Mission Scorecard")
    axes[1].bar(segment_ids, _values(segments, "rms_error_m"), color="#f58518", label="RMS")
    axes[1].plot(segment_ids, _values(segments, "max_error_m"), color="#111111", marker="o", label="max")
    axes[1].set_ylabel("error [m]")
    axes[1].legend(fontsize=8)
    axes[2].bar(segment_ids, _values(segments, "contact_valid_fraction"), color="#54a24b")
    axes[2].set_ylim(0.0, 1.05)
    axes[2].set_ylabel("valid contact")
    axes[3].plot(segment_ids, _values(segments, "mean_cable_support_fraction"), marker="o", label="cable support")
    axes[3].plot(segment_ids, _values(segments, "mean_drone_power_ratio"), marker="o", label="motor power")
    axes[3].set_xlabel("segment")
    axes[3].set_ylabel("fraction")
    axes[3].legend(fontsize=8)
    for ax in axes:
        ax.grid(True, color="#dddddd", axis="y")
    fig.savefig(output_dir / "segment_scorecard.png", dpi=180)
    plt.close(fig)


def write_diagnostics(
    scenario: ControllerScenario,
    params: SimParams,
    states: Sequence[SimState],
    output_dir: Path | None = None,
) -> tuple[Path, dict[str, object]]:
    active_output_dir = output_dir or session_output_dir()
    mission = scenario.facade_mission
    rows = rows_from_states(states, params, mission)
    summary = summarize_session(scenario, params, states, rows)
    write_csv(rows, active_output_dir / "session_log.csv")
    write_csv(segment_rows(rows), active_output_dir / "segment_summary.csv")
    write_summary(summary, active_output_dir / "summary.json")
    write_report(summary, active_output_dir / "report.md")
    plot_session(rows, active_output_dir, mission, params)
    return active_output_dir, summary
