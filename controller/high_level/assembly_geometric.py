"""High-level controller for a docked multirotor assembly.

This controller treats latched drones as one rigid body. It computes a desired
world-frame wrench about the current assembly center of mass. A downstream
allocation layer maps that wrench to all motor speeds.

Attitude commands are defined relative to the assembly attitude at engagement:
the first drone passed into the assembly geometry supplies the stored body
attitude reference. The UI commands roll, pitch, and yaw offsets from that
reference, and other docked modules keep fixed yaw offsets in the experiment
layer for target bookkeeping.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Sequence

from controller.allocation.geometry import AssemblyGeometry
from controller.high_level import position
from controller.low_level import body_rate as flight


DOCKED_XY_GAIN_SCALE = 1.0
DOCKED_Z_GAIN_SCALE = 1.0
DOCKED_INTEGRAL_SCALE = 0.50
DOCKED_MAX_HORIZONTAL_ACCEL = 2.8
DOCKED_MAX_VERTICAL_ACCEL = 4.5


@dataclass
class AssemblyControllerState:
    pos_integral: list[float]
    target_position: list[float] | None = None
    target_roll: float = 0.0
    target_pitch: float = 0.0
    target_yaw: float = 0.0
    last_time: float | None = None
    engaged_at: float | None = None
    reference_axes: tuple[list[float], list[float], list[float]] | None = None
    reference_yaw: float = 0.0


def reset_to_current(
    state: AssemblyControllerState,
    geometry: AssemblyGeometry,
    target_yaw: float = 0.0,
    target_roll: float = 0.0,
    target_pitch: float = 0.0,
) -> None:
    state.pos_integral = [0.0, 0.0, 0.0]
    state.target_position = geometry.com_world[:]
    state.target_roll = float(target_roll)
    state.target_pitch = float(target_pitch)
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


def _world_to_body_vector(matrix: Sequence[float], vector_world: Sequence[float]) -> list[float]:
    body_x, body_y, body_z = flight.body_axes_from_matrix(list(matrix))
    return [
        sum(body_x[axis] * vector_world[axis] for axis in range(3)),
        sum(body_y[axis] * vector_world[axis] for axis in range(3)),
        sum(body_z[axis] * vector_world[axis] for axis in range(3)),
    ]


def _matmul3(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    return [
        [sum(a[row][idx] * b[idx][col] for idx in range(3)) for col in range(3)]
        for row in range(3)
    ]


def _axes_to_matrix(axes: tuple[list[float], list[float], list[float]]) -> list[list[float]]:
    return [
        [axes[0][0], axes[1][0], axes[2][0]],
        [axes[0][1], axes[1][1], axes[2][1]],
        [axes[0][2], axes[1][2], axes[2][2]],
    ]


def _matrix_to_axes(matrix: Sequence[Sequence[float]]) -> tuple[list[float], list[float], list[float]]:
    return (
        [float(matrix[0][0]), float(matrix[1][0]), float(matrix[2][0])],
        [float(matrix[0][1]), float(matrix[1][1]), float(matrix[2][1])],
        [float(matrix[0][2]), float(matrix[1][2]), float(matrix[2][2])],
    )


def _rotation_x(angle: float) -> list[list[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rotation_y(angle: float) -> list[list[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _rotation_z(angle: float) -> list[list[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _rotation_axis_angle(axis: Sequence[float], angle: float) -> list[list[float]]:
    unit_axis = position.normalize([float(axis[0]), float(axis[1]), float(axis[2])])
    x, y, z = unit_axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    return [
        [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
        [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
        [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
    ]


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
    reference = _axes_to_matrix(state.reference_axes)
    yawed_reference = _matmul3(_rotation_z(yaw_delta), reference)
    local_attitude_command = _matmul3(_rotation_y(state.target_pitch), _rotation_x(state.target_roll))
    desired = _matmul3(yawed_reference, local_attitude_command)
    return _matrix_to_axes(desired)


def _axis_to_leader_body(geometry: AssemblyGeometry, axis_world: Sequence[float]) -> list[float]:
    body_x, body_y, body_z = flight.body_axes_from_matrix(geometry.leader_matrix)
    return [
        sum(body_x[axis] * axis_world[axis] for axis in range(3)),
        sum(body_y[axis] * axis_world[axis] for axis in range(3)),
        sum(body_z[axis] * axis_world[axis] for axis in range(3)),
    ]


def _collective_axis_body(geometry: AssemblyGeometry) -> list[float]:
    collective_world = _collective_axis(geometry)
    if position.norm(collective_world) < 1e-9:
        return [0.0, 0.0, 1.0]
    return position.normalize(_axis_to_leader_body(geometry, collective_world))


def _attitude_alignment_telemetry(
    base_axes: tuple[list[float], list[float], list[float]],
    correction_axis_world: Sequence[float],
    correction_angle: float,
) -> tuple[float, float, float]:
    rotvec_world = [float(correction_axis_world[axis]) * correction_angle for axis in range(3)]
    auto_roll = sum(base_axes[0][axis] * rotvec_world[axis] for axis in range(3))
    auto_pitch = sum(base_axes[1][axis] * rotvec_world[axis] for axis in range(3))
    auto_yaw = sum(base_axes[2][axis] * rotvec_world[axis] for axis in range(3))
    return auto_roll, auto_pitch, auto_yaw


def _attitude_error_world(
    matrix: Sequence[float],
    desired_axes: tuple[list[float], list[float], list[float]],
) -> list[float]:
    body_axes = flight.body_axes_from_matrix(list(matrix))
    error_world = [0.0, 0.0, 0.0]
    for current_axis, desired_axis in zip(body_axes, desired_axes):
        term = _cross(current_axis, desired_axis)
        for axis in range(3):
            error_world[axis] += 0.5 * term[axis]
    return error_world


def _project(vector: Sequence[float], axis: Sequence[float]) -> list[float]:
    axis_unit = position.normalize([float(axis[0]), float(axis[1]), float(axis[2])])
    scale = sum(float(vector[index]) * axis_unit[index] for index in range(3))
    return [scale * axis_unit[index] for index in range(3)]


def _subtract3(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [float(a[index]) - float(b[index]) for index in range(3)]


def _horizontal_unit_from_yaw(yaw: float) -> tuple[list[float], list[float]]:
    b1 = [math.cos(yaw), math.sin(yaw), 0.0]
    b2 = [-math.sin(yaw), math.cos(yaw), 0.0]
    return b1, b2


def _position_attitude_feedforward(
    state: AssemblyControllerState,
    accel_cmd: Sequence[float],
    args: argparse.Namespace,
) -> tuple[float, float]:
    if not bool(getattr(args, "assembly_position_attitude_feedforward", True)):
        return 0.0, 0.0
    gain = max(0.0, float(getattr(args, "assembly_position_attitude_gain", 1.0)))
    limit = math.radians(max(0.0, float(getattr(args, "assembly_position_attitude_limit_deg", 16.0))))
    if gain <= 0.0 or limit <= 0.0:
        return 0.0, 0.0

    b1_yaw, b2_yaw = _horizontal_unit_from_yaw(state.target_yaw)
    accel_forward = accel_cmd[0] * b1_yaw[0] + accel_cmd[1] * b1_yaw[1]
    accel_left = accel_cmd[0] * b2_yaw[0] + accel_cmd[1] * b2_yaw[1]
    auto_pitch = gain * math.atan2(accel_forward, flight.G)
    auto_roll = -gain * math.atan2(accel_left, flight.G)
    limited = position.clamp_norm([auto_roll, auto_pitch], limit)
    return limited[0], limited[1]


def _force_aligned_axes(
    base_axes: tuple[list[float], list[float], list[float]],
    collective_axis_body: Sequence[float],
    desired_force_world: Sequence[float],
    args: argparse.Namespace,
) -> tuple[tuple[list[float], list[float], list[float]], dict[str, object]]:
    gain = max(0.0, float(getattr(args, "assembly_force_alignment_gain", 1.0)))
    limit = math.radians(max(0.0, float(getattr(args, "assembly_force_alignment_limit_deg", 28.0))))
    force_norm = position.norm([float(value) for value in desired_force_world])
    if gain <= 0.0 or limit <= 0.0 or force_norm < 1e-9:
        return base_axes, {
            "auto_roll": 0.0,
            "auto_pitch": 0.0,
            "auto_yaw": 0.0,
            "force_alignment_angle": 0.0,
            "force_alignment_axis_world": [0.0, 0.0, 0.0],
            "force_alignment_limited": False,
            "alignment_collective_axis_world": base_axes[2][:],
            "alignment_force_axis_world": [0.0, 0.0, 1.0],
        }

    base_matrix = _axes_to_matrix(base_axes)
    base_collective_axis = position.normalize(_mat_vec3(base_matrix, collective_axis_body))
    force_axis = position.normalize([float(value) for value in desired_force_world])
    correction_axis = _cross(base_collective_axis, force_axis)
    axis_norm = position.norm(correction_axis)
    dot_value = position.clamp(sum(base_collective_axis[axis] * force_axis[axis] for axis in range(3)), -1.0, 1.0)
    raw_angle = math.atan2(axis_norm, dot_value)
    if axis_norm < 1e-9 or raw_angle < 1e-9:
        return base_axes, {
            "auto_roll": 0.0,
            "auto_pitch": 0.0,
            "auto_yaw": 0.0,
            "force_alignment_angle": 0.0,
            "force_alignment_axis_world": [0.0, 0.0, 0.0],
            "force_alignment_limited": False,
            "alignment_collective_axis_world": base_collective_axis,
            "alignment_force_axis_world": force_axis,
        }

    correction_axis = [value / axis_norm for value in correction_axis]
    commanded_angle = min(raw_angle * gain, limit)
    correction = _rotation_axis_angle(correction_axis, commanded_angle)
    desired_matrix = _matmul3(correction, base_matrix)
    desired_axes = _matrix_to_axes(desired_matrix)
    auto_roll, auto_pitch, auto_yaw = _attitude_alignment_telemetry(base_axes, correction_axis, commanded_angle)
    return desired_axes, {
        "auto_roll": auto_roll,
        "auto_pitch": auto_pitch,
        "auto_yaw": auto_yaw,
        "force_alignment_angle": commanded_angle,
        "force_alignment_axis_world": correction_axis,
        "force_alignment_limited": commanded_angle + 1e-9 < raw_angle * gain,
        "alignment_collective_axis_world": base_collective_axis,
        "alignment_force_axis_world": force_axis,
    }


def _coupled_desired_axes(
    state: AssemblyControllerState,
    geometry: AssemblyGeometry,
    desired_force_world: Sequence[float],
    accel_cmd: Sequence[float],
    args: argparse.Namespace,
) -> tuple[tuple[list[float], list[float], list[float]], dict[str, object]]:
    base_axes = _desired_axes_from_reference(state, geometry)
    mode = str(getattr(args, "assembly_attitude_coupling", "force-align"))
    if mode == "off":
        return base_axes, {
            "auto_roll": 0.0,
            "auto_pitch": 0.0,
            "auto_yaw": 0.0,
            "force_alignment_angle": 0.0,
            "force_alignment_axis_world": [0.0, 0.0, 0.0],
            "force_alignment_limited": False,
            "alignment_collective_axis_world": _mat_vec3(_axes_to_matrix(base_axes), _collective_axis_body(geometry)),
            "alignment_force_axis_world": position.normalize([float(value) for value in desired_force_world]),
            "attitude_coupling_mode": mode,
        }
    if mode == "feedforward":
        auto_roll, auto_pitch = _position_attitude_feedforward(state, accel_cmd, args)
        state_roll = state.target_roll
        state_pitch = state.target_pitch
        state.target_roll = state_roll + auto_roll
        state.target_pitch = state_pitch + auto_pitch
        try:
            desired_axes = _desired_axes_from_reference(state, geometry)
        finally:
            state.target_roll = state_roll
            state.target_pitch = state_pitch
        return desired_axes, {
            "auto_roll": auto_roll,
            "auto_pitch": auto_pitch,
            "auto_yaw": 0.0,
            "force_alignment_angle": 0.0,
            "force_alignment_axis_world": [0.0, 0.0, 0.0],
            "force_alignment_limited": False,
            "alignment_collective_axis_world": _mat_vec3(_axes_to_matrix(base_axes), _collective_axis_body(geometry)),
            "alignment_force_axis_world": position.normalize([float(value) for value in desired_force_world]),
            "attitude_coupling_mode": mode,
        }

    desired_axes, telemetry = _force_aligned_axes(
        base_axes,
        _collective_axis_body(geometry),
        desired_force_world,
        args,
    )
    telemetry["attitude_coupling_mode"] = "force-align"
    return desired_axes, telemetry


def _collective_axis(geometry: AssemblyGeometry) -> list[float]:
    axis_sum = [0.0, 0.0, 0.0]
    for motor in geometry.motors:
        for axis in range(3):
            axis_sum[axis] += motor.axis_world[axis]
    if position.norm(axis_sum) < 1e-9:
        return [0.0, 0.0, 0.0]
    return position.normalize(axis_sum)


def _attitude_control_torque_body(
    geometry: AssemblyGeometry,
    desired_axes: tuple[list[float], list[float], list[float]],
    control_gain: float,
    args: argparse.Namespace,
) -> dict[str, object]:
    body_rate = flight.world_to_body_rate(geometry.leader_matrix, geometry.leader_angular_velocity_world)
    if not bool(getattr(args, "assembly_world_yaw_attitude", True)):
        att_error_body = position.attitude_error_body(geometry.leader_matrix, desired_axes)
        virtual_torque_body = [
            control_gain * float(args.assembly_attitude_torque_gain_rp) * att_error_body[0]
            - float(args.assembly_rate_damping_rp) * body_rate[0],
            control_gain * float(args.assembly_attitude_torque_gain_rp) * att_error_body[1]
            - float(args.assembly_rate_damping_rp) * body_rate[1],
            control_gain * float(args.assembly_attitude_torque_gain_yaw) * att_error_body[2]
            - float(args.assembly_rate_damping_yaw) * body_rate[2],
        ]
        return {
            "att_error": att_error_body,
            "att_error_world": _attitude_error_world(geometry.leader_matrix, desired_axes),
            "att_error_tilt_world": [0.0, 0.0, 0.0],
            "att_error_yaw_world": [0.0, 0.0, 0.0],
            "angular_rate_tilt_world": [0.0, 0.0, 0.0],
            "angular_rate_yaw_world": [0.0, 0.0, 0.0],
            "body_rate": body_rate,
            "virtual_torque_body": virtual_torque_body,
            "virtual_torque_world": flight.body_torque_to_world(geometry.leader_matrix, virtual_torque_body),
            "attitude_error_frame": "leader_body",
        }

    yaw_axis_world = [0.0, 0.0, 1.0]
    att_error_world = _attitude_error_world(geometry.leader_matrix, desired_axes)
    att_error_yaw_world = _project(att_error_world, yaw_axis_world)
    att_error_tilt_world = _subtract3(att_error_world, att_error_yaw_world)
    yaw_error_limit = math.radians(max(0.0, float(getattr(args, "assembly_yaw_error_limit_deg", 6.0))))
    if yaw_error_limit > 0.0:
        att_error_yaw_world = position.clamp_norm(att_error_yaw_world, yaw_error_limit)
    angular_rate_yaw_world = _project(geometry.leader_angular_velocity_world, yaw_axis_world)
    angular_rate_tilt_world = _subtract3(geometry.leader_angular_velocity_world, angular_rate_yaw_world)
    virtual_torque_world = [
        control_gain * float(args.assembly_attitude_torque_gain_rp) * att_error_tilt_world[axis]
        + control_gain * float(args.assembly_attitude_torque_gain_yaw) * att_error_yaw_world[axis]
        - float(args.assembly_rate_damping_rp) * angular_rate_tilt_world[axis]
        - float(args.assembly_rate_damping_yaw) * angular_rate_yaw_world[axis]
        for axis in range(3)
    ]
    virtual_torque_body = _world_to_body_vector(geometry.leader_matrix, virtual_torque_world)
    return {
        "att_error": _world_to_body_vector(geometry.leader_matrix, att_error_world),
        "att_error_world": att_error_world,
        "att_error_tilt_world": att_error_tilt_world,
        "att_error_yaw_world": att_error_yaw_world,
        "angular_rate_tilt_world": angular_rate_tilt_world,
        "angular_rate_yaw_world": angular_rate_yaw_world,
        "body_rate": body_rate,
        "virtual_torque_body": virtual_torque_body,
        "virtual_torque_world": virtual_torque_world,
        "attitude_error_frame": "world_yaw",
    }


def controller_step(
    sim,
    geometry: AssemblyGeometry,
    state: AssemblyControllerState,
    args: argparse.Namespace,
    target_position: Sequence[float],
    target_yaw: float,
    target_roll: float = 0.0,
    target_pitch: float = 0.0,
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
    state.target_roll = float(target_roll)
    state.target_pitch = float(target_pitch)
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

    max_horizontal_accel = max(
        0.0,
        float(getattr(args, "assembly_max_horizontal_accel", DOCKED_MAX_HORIZONTAL_ACCEL)),
    )
    max_vertical_accel = max(
        0.0,
        float(getattr(args, "assembly_max_vertical_accel", DOCKED_MAX_VERTICAL_ACCEL)),
    )
    accel_xy = position.clamp_norm(pid_accel_sum_raw[:2], max_horizontal_accel)
    accel_z = position.clamp(
        pid_accel_sum_raw[2],
        -max_vertical_accel,
        max_vertical_accel,
    )
    desired_force_world = [
        geometry.mass * accel_xy[0],
        geometry.mass * accel_xy[1],
        geometry.mass * (flight.G + accel_z),
    ]

    desired_axes, attitude_coupling = _coupled_desired_axes(
        state,
        geometry,
        desired_force_world,
        [accel_xy[0], accel_xy[1], accel_z],
        args,
    )
    auto_roll = float(attitude_coupling["auto_roll"])
    auto_pitch = float(attitude_coupling["auto_pitch"])
    auto_yaw = float(attitude_coupling["auto_yaw"])
    ramp_time = max(0.0, float(args.assembly_control_ramp_time))
    if ramp_time > 0.0:
        control_gain = position.clamp((now - state.engaged_at) / ramp_time, 0.0, 1.0)
    else:
        control_gain = 1.0
    force_world = desired_force_world[:]
    attitude_control = _attitude_control_torque_body(geometry, desired_axes, control_gain, args)
    body_rate = attitude_control["body_rate"]  # type: ignore[assignment]
    att_error = attitude_control["att_error"]  # type: ignore[assignment]
    virtual_torque_body = attitude_control["virtual_torque_body"]  # type: ignore[assignment]
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
        "target_roll": state.target_roll,
        "target_pitch": state.target_pitch,
        "target_yaw": state.target_yaw,
        "auto_roll": auto_roll,
        "auto_pitch": auto_pitch,
        "auto_yaw": auto_yaw,
        "desired_roll": state.target_roll + auto_roll,
        "desired_pitch": state.target_pitch + auto_pitch,
        "desired_yaw": state.target_yaw + auto_yaw,
        "attitude_coupling_mode": attitude_coupling["attitude_coupling_mode"],
        "force_alignment_angle": attitude_coupling["force_alignment_angle"],
        "force_alignment_axis_world": attitude_coupling["force_alignment_axis_world"],
        "force_alignment_limited": attitude_coupling["force_alignment_limited"],
        "alignment_collective_axis_world": attitude_coupling["alignment_collective_axis_world"],
        "alignment_force_axis_world": attitude_coupling["alignment_force_axis_world"],
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
        "virtual_torque_world": attitude_control["virtual_torque_world"],
        "torque_body_raw": torque_body_raw,
        "torque_body": torque_body,
        "torque_limited_delta_body": torque_limited_delta_body,
        "torque_world": torque_world,
        "wrench_cmd": wrench_cmd,
        "att_error": att_error,
        "att_error_world": attitude_control["att_error_world"],
        "att_error_tilt_world": attitude_control["att_error_tilt_world"],
        "att_error_yaw_world": attitude_control["att_error_yaw_world"],
        "angular_rate_tilt_world": attitude_control["angular_rate_tilt_world"],
        "angular_rate_yaw_world": attitude_control["angular_rate_yaw_world"],
        "attitude_error_frame": attitude_control["attitude_error_frame"],
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
