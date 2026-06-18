"""High-level controller for a docked multirotor assembly.

This controller treats latched drones as one rigid body. It computes a desired
world-frame wrench about the current assembly center of mass. A downstream
allocation layer maps that wrench to all motor speeds.

Yaw is defined in the assembly leader frame: the first drone passed into the
assembly geometry supplies the body attitude reference, and `target_yaw`
commands that leader/module-forward heading. Other docked modules keep fixed
yaw offsets in the experiment layer.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Sequence

from controller.allocation.geometry import AssemblyGeometry
from controller.high_level import position
from controller.low_level import body_rate as flight


DOCKED_XY_GAIN_SCALE = 0.45
DOCKED_Z_GAIN_SCALE = 0.70
DOCKED_INTEGRAL_SCALE = 0.35
DOCKED_MAX_HORIZONTAL_ACCEL = 1.0
DOCKED_MAX_VERTICAL_ACCEL = 2.4


@dataclass
class AssemblyControllerState:
    pos_integral: list[float]
    target_position: list[float] | None = None
    target_yaw: float = 0.0
    last_time: float | None = None
    engaged_at: float | None = None
    reference_axes: tuple[list[float], list[float], list[float]] | None = None
    reference_yaw: float = 0.0


def reset_to_current(state: AssemblyControllerState, geometry: AssemblyGeometry, target_yaw: float = 0.0) -> None:
    state.pos_integral = [0.0, 0.0, 0.0]
    state.target_position = geometry.com_world[:]
    state.target_yaw = float(target_yaw)
    state.last_time = None
    state.engaged_at = None
    state.reference_axes = flight.body_axes_from_matrix(geometry.leader_matrix)
    state.reference_yaw = float(target_yaw)


def average_target(targets: Sequence[Sequence[float]]) -> list[float]:
    count = max(1, len(targets))
    return [sum(target[axis] for target in targets) / count for axis in range(3)]


def _clamp_body_torque(torque_body: list[float], args: argparse.Namespace) -> list[float]:
    rp_limit = float(args.assembly_torque_limit_rp)
    yaw_limit = float(args.assembly_torque_limit_yaw)
    return [
        position.clamp(torque_body[0], -rp_limit, rp_limit),
        position.clamp(torque_body[1], -rp_limit, rp_limit),
        position.clamp(torque_body[2], -yaw_limit, yaw_limit),
    ]


def _cross(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _mat_vec3(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> list[float]:
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def _inertia_diag(inertia: Sequence[Sequence[float]]) -> list[float]:
    return [float(inertia[axis][axis]) for axis in range(3)]


def _inertia_offdiag(inertia: Sequence[Sequence[float]]) -> list[float]:
    return [float(inertia[0][1]), float(inertia[0][2]), float(inertia[1][2])]


def _inertia_shaped_torque(
    geometry: AssemblyGeometry,
    virtual_torque_body: list[float],
    body_rate: list[float],
    args: argparse.Namespace,
) -> dict[str, list[float]]:
    """Use the full assembly inertia while preserving existing gain scale."""

    inertia_body = geometry.inertia_body
    inertia_diag = _inertia_diag(inertia_body)
    alpha_cmd_body = [
        virtual_torque_body[axis] / max(abs(inertia_diag[axis]), 1e-9)
        for axis in range(3)
    ]
    inertia_torque_body = _mat_vec3(inertia_body, alpha_cmd_body)
    angular_momentum_body = _mat_vec3(inertia_body, body_rate)
    if bool(getattr(args, "assembly_gyroscopic_compensation", True)):
        gyroscopic_torque_body = _cross(body_rate, angular_momentum_body)
    else:
        gyroscopic_torque_body = [0.0, 0.0, 0.0]
    torque_body = [
        inertia_torque_body[axis] + gyroscopic_torque_body[axis]
        for axis in range(3)
    ]
    return {
        "torque_body": torque_body,
        "alpha_cmd_body": alpha_cmd_body,
        "inertia_torque_body": inertia_torque_body,
        "angular_momentum_body": angular_momentum_body,
        "gyroscopic_torque_body": gyroscopic_torque_body,
    }


def _rotate_z(vector: Sequence[float], yaw: float) -> list[float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [
        c * vector[0] - s * vector[1],
        s * vector[0] + c * vector[1],
        vector[2],
    ]


def _desired_axes_from_reference(
    state: AssemblyControllerState,
    geometry: AssemblyGeometry,
) -> tuple[list[float], list[float], list[float]]:
    if state.reference_axes is None:
        state.reference_axes = flight.body_axes_from_matrix(geometry.leader_matrix)
        state.reference_yaw = state.target_yaw
    yaw_delta = position.wrap_pi(state.target_yaw - state.reference_yaw)
    return tuple(_rotate_z(axis, yaw_delta) for axis in state.reference_axes)


def _collective_axis(geometry: AssemblyGeometry) -> list[float]:
    axis_sum = [0.0, 0.0, 0.0]
    for motor in geometry.motors:
        for axis in range(3):
            axis_sum[axis] += motor.axis_world[axis]
    if position.norm(axis_sum) < 1e-9:
        return [0.0, 0.0, 0.0]
    return position.normalize(axis_sum)


def controller_step(
    sim,
    geometry: AssemblyGeometry,
    state: AssemblyControllerState,
    args: argparse.Namespace,
    target_position: Sequence[float],
    target_yaw: float,
) -> dict[str, object]:
    now = sim.getSimulationTime()
    if state.engaged_at is None:
        state.engaged_at = now
    if state.last_time is None:
        dt = args.time_step
    else:
        dt = position.clamp(now - state.last_time, 1e-4, 0.05)
    state.last_time = now
    state.target_position = [float(value) for value in target_position]
    state.target_yaw = float(target_yaw)

    pos_error = [state.target_position[axis] - geometry.com_world[axis] for axis in range(3)]
    integral_limits = [
        DOCKED_INTEGRAL_SCALE * args.integral_limit_xy,
        DOCKED_INTEGRAL_SCALE * args.integral_limit_xy,
        DOCKED_INTEGRAL_SCALE * args.integral_limit_z,
    ]
    for axis in range(3):
        state.pos_integral[axis] = position.clamp(
            state.pos_integral[axis] + pos_error[axis] * dt,
            -integral_limits[axis],
            integral_limits[axis],
        )

    pid_accel_p = [
        DOCKED_XY_GAIN_SCALE * args.kp_xy * pos_error[0],
        DOCKED_XY_GAIN_SCALE * args.kp_xy * pos_error[1],
        DOCKED_Z_GAIN_SCALE * args.kp_z * pos_error[2],
    ]
    pid_accel_i = [
        DOCKED_XY_GAIN_SCALE * args.ki_xy * state.pos_integral[0],
        DOCKED_XY_GAIN_SCALE * args.ki_xy * state.pos_integral[1],
        DOCKED_Z_GAIN_SCALE * args.ki_z * state.pos_integral[2],
    ]
    pid_accel_d = [
        -DOCKED_XY_GAIN_SCALE * args.kd_xy * geometry.velocity_world[0],
        -DOCKED_XY_GAIN_SCALE * args.kd_xy * geometry.velocity_world[1],
        -DOCKED_Z_GAIN_SCALE * args.kd_z * geometry.velocity_world[2],
    ]
    pid_accel_sum_raw = [
        pid_accel_p[0] + pid_accel_i[0] + pid_accel_d[0],
        pid_accel_p[1] + pid_accel_i[1] + pid_accel_d[1],
        pid_accel_p[2] + pid_accel_i[2] + pid_accel_d[2],
    ]

    accel_xy = position.clamp_norm(pid_accel_sum_raw[:2], min(args.max_horizontal_accel, DOCKED_MAX_HORIZONTAL_ACCEL))
    accel_z = position.clamp(
        pid_accel_sum_raw[2],
        -min(args.max_vertical_accel, DOCKED_MAX_VERTICAL_ACCEL),
        min(args.max_vertical_accel, DOCKED_MAX_VERTICAL_ACCEL),
    )
    desired_force_world = [
        geometry.mass * accel_xy[0],
        geometry.mass * accel_xy[1],
        geometry.mass * (flight.G + accel_z),
    ]

    desired_axes = _desired_axes_from_reference(state, geometry)
    att_error = position.attitude_error_body(geometry.leader_matrix, desired_axes)
    body_rate = flight.world_to_body_rate(geometry.leader_matrix, geometry.leader_angular_velocity_world)
    ramp_time = max(0.0, float(args.assembly_control_ramp_time))
    if ramp_time > 0.0:
        control_gain = position.clamp((now - state.engaged_at) / ramp_time, 0.0, 1.0)
    else:
        control_gain = 1.0
    force_world = desired_force_world[:]
    virtual_torque_body = [
        control_gain * float(args.assembly_attitude_torque_gain_rp) * att_error[0]
        - float(args.assembly_rate_damping_rp) * body_rate[0],
        control_gain * float(args.assembly_attitude_torque_gain_rp) * att_error[1]
        - float(args.assembly_rate_damping_rp) * body_rate[1],
        control_gain * float(args.assembly_attitude_torque_gain_yaw) * att_error[2]
        - float(args.assembly_rate_damping_yaw) * body_rate[2],
    ]
    if bool(getattr(args, "assembly_inertia_aware_torque", True)):
        inertia_control = _inertia_shaped_torque(geometry, virtual_torque_body, body_rate, args)
        torque_body_raw = inertia_control["torque_body"]
    else:
        inertia_control = {
            "alpha_cmd_body": [0.0, 0.0, 0.0],
            "inertia_torque_body": virtual_torque_body[:],
            "angular_momentum_body": _mat_vec3(geometry.inertia_body, body_rate),
            "gyroscopic_torque_body": [0.0, 0.0, 0.0],
        }
        torque_body_raw = virtual_torque_body[:]
    torque_body = _clamp_body_torque(torque_body_raw, args)
    torque_limited_delta_body = [
        torque_body[axis] - torque_body_raw[axis]
        for axis in range(3)
    ]
    torque_world = flight.body_torque_to_world(geometry.leader_matrix, torque_body)
    wrench_cmd = force_world + torque_world

    return {
        "time": now,
        "dt": dt,
        "pos": geometry.com_world[:],
        "target": state.target_position[:],
        "target_yaw": state.target_yaw,
        "pos_error": pos_error,
        "pos_integral": state.pos_integral[:],
        "lin_vel": geometry.velocity_world[:],
        "ang_vel_world": geometry.angular_velocity_world[:],
        "body_rate": body_rate,
        "pid_accel_p": pid_accel_p,
        "pid_accel_i": pid_accel_i,
        "pid_accel_d": pid_accel_d,
        "pid_accel_sum_raw": pid_accel_sum_raw,
        "accel_cmd": [accel_xy[0], accel_xy[1], accel_z],
        "desired_force_world": desired_force_world,
        "force_world": force_world,
        "virtual_torque_body": virtual_torque_body,
        "torque_body_raw": torque_body_raw,
        "torque_body": torque_body,
        "torque_limited_delta_body": torque_limited_delta_body,
        "torque_world": torque_world,
        "wrench_cmd": wrench_cmd,
        "att_error": att_error,
        "desired_b1": desired_axes[0],
        "desired_b2": desired_axes[1],
        "desired_b3": desired_axes[2],
        "reference_b1": state.reference_axes[0] if state.reference_axes is not None else desired_axes[0],
        "reference_b2": state.reference_axes[1] if state.reference_axes is not None else desired_axes[1],
        "reference_b3": state.reference_axes[2] if state.reference_axes is not None else desired_axes[2],
        "reference_yaw": state.reference_yaw,
        "collective_axis": _collective_axis(geometry),
        "inertia_body_diag": _inertia_diag(geometry.inertia_body),
        "inertia_body_offdiag": _inertia_offdiag(geometry.inertia_body),
        "alpha_cmd_body": inertia_control["alpha_cmd_body"],
        "inertia_torque_body": inertia_control["inertia_torque_body"],
        "angular_momentum_body": inertia_control["angular_momentum_body"],
        "gyroscopic_torque_body": inertia_control["gyroscopic_torque_body"],
        "inertia_aware_torque": bool(getattr(args, "assembly_inertia_aware_torque", True)),
        "control_gain": control_gain,
        "control_mode": "docked_allocation",
    }
