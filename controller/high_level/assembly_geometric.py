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
DOCKED_MAX_TILT_DEG = 14.0
DOCKED_MAX_HORIZONTAL_ACCEL = 1.0
DOCKED_MAX_VERTICAL_ACCEL = 2.4


@dataclass
class AssemblyControllerState:
    pos_integral: list[float]
    target_position: list[float] | None = None
    target_yaw: float = 0.0
    last_time: float | None = None
    engaged_at: float | None = None


def reset_to_current(state: AssemblyControllerState, geometry: AssemblyGeometry, target_yaw: float = 0.0) -> None:
    state.pos_integral = [0.0, 0.0, 0.0]
    state.target_position = geometry.com_world[:]
    state.target_yaw = float(target_yaw)
    state.last_time = None
    state.engaged_at = None


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


def _collective_axis(geometry: AssemblyGeometry) -> list[float]:
    axis_sum = [0.0, 0.0, 0.0]
    for motor in geometry.motors:
        for axis in range(3):
            axis_sum[axis] += motor.axis_world[axis]
    if position.norm(axis_sum) < 1e-9:
        return flight.body_axes_from_matrix(geometry.leader_matrix)[2]
    return position.normalize(axis_sum)


def _attainable_force(desired_force_world: list[float], geometry: AssemblyGeometry) -> list[float]:
    """Project desired force onto the current collective thrust direction.

    The assembly has non-tilted Crazyflie propellers, so horizontal motion must
    come from tilting the docked body, not from direct lateral force allocation.
    """

    collective_axis = _collective_axis(geometry)
    thrust_along_axis = max(
        0.05 * geometry.mass * flight.G,
        position.dot(desired_force_world, collective_axis),
    )
    return [collective_axis[axis] * thrust_along_axis for axis in range(3)]


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

    max_tilt = math.radians(min(args.max_tilt_deg, DOCKED_MAX_TILT_DEG))
    horizontal_force = math.sqrt(desired_force_world[0] ** 2 + desired_force_world[1] ** 2)
    vertical_force = max(desired_force_world[2], 0.1 * geometry.mass * flight.G)
    max_horizontal_force = math.tan(max_tilt) * vertical_force
    if horizontal_force > max_horizontal_force:
        scale = max_horizontal_force / max(horizontal_force, 1e-12)
        desired_force_world[0] *= scale
        desired_force_world[1] *= scale

    desired_axes = position.desired_axes_from_force(desired_force_world, state.target_yaw)
    att_error = position.attitude_error_body(geometry.leader_matrix, desired_axes)
    body_rate = flight.world_to_body_rate(geometry.leader_matrix, geometry.leader_angular_velocity_world)
    ramp_time = max(0.0, float(args.assembly_control_ramp_time))
    if ramp_time > 0.0:
        control_gain = position.clamp((now - state.engaged_at) / ramp_time, 0.0, 1.0)
    else:
        control_gain = 1.0
    force_world = _attainable_force(desired_force_world, geometry)
    torque_body = [
        control_gain * float(args.assembly_attitude_torque_gain_rp) * att_error[0]
        - float(args.assembly_rate_damping_rp) * body_rate[0],
        control_gain * float(args.assembly_attitude_torque_gain_rp) * att_error[1]
        - float(args.assembly_rate_damping_rp) * body_rate[1],
        control_gain * float(args.assembly_attitude_torque_gain_yaw) * att_error[2]
        - float(args.assembly_rate_damping_yaw) * body_rate[2],
    ]
    torque_body = _clamp_body_torque(torque_body, args)
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
        "torque_body": torque_body,
        "torque_world": torque_world,
        "wrench_cmd": wrench_cmd,
        "att_error": att_error,
        "desired_b3": desired_axes[2],
        "collective_axis": _collective_axis(geometry),
        "control_gain": control_gain,
        "control_mode": "docked_allocation",
    }
