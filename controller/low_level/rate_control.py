"""Pure Crazyflie-style body-rate control.

This module has no CoppeliaSim dependency. It is the part that can be reused
when the simulator plant is replaced by a real Crazyflie interface: inputs are
collective thrust, measured body rates, and elapsed time; outputs are four motor
angular-velocity commands.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


G = 9.81
MOTORS = (
    {"pos": (0.029886, -0.031685, 0.012625), "spin": 1.0},
    {"pos": (-0.033400, 0.031601, 0.012625), "spin": 1.0},
    {"pos": (0.029886, 0.031601, 0.012625), "spin": -1.0},
    {"pos": (-0.033400, -0.031685, 0.012625), "spin": -1.0},
)


@dataclass
class RateControllerState:
    rate_integral: list[float]
    motor_speed: list[float]


@dataclass(frozen=True)
class RateCommand:
    thrust: float
    p: float
    q: float
    r: float


@dataclass(frozen=True)
class RateGains:
    kp_rp: float
    kp_yaw: float
    ki_rp: float
    ki_yaw: float
    integral_limit_rp: float
    integral_limit_yaw: float
    motor_tau_up: float
    motor_tau_down: float
    max_motor_speed: float
    max_thrust: float
    yaw_drag_arm: float


@dataclass(frozen=True)
class RateControlOutput:
    omega_cmd: list[float]
    omega: list[float]
    motor_thrust_cmd: list[float]
    motor_thrust: list[float]
    thrust_cmd: float
    moment_cmd: list[float]
    rate_error: list[float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def inverse_4x4(matrix: list[list[float]]) -> list[list[float]]:
    n = 4
    augmented = [row[:] + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(augmented[row][col]))
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        pivot_value = augmented[col][col]
        if abs(pivot_value) < 1e-12:
            raise ValueError("Motor mixer is singular.")
        augmented[col] = [value / pivot_value for value in augmented[col]]
        for row in range(n):
            if row == col:
                continue
            factor = augmented[row][col]
            augmented[row] = [augmented[row][i] - factor * augmented[col][i] for i in range(2 * n)]
    return [row[n:] for row in augmented]


def motor_mixer(yaw_drag_arm: float) -> list[list[float]]:
    allocation = [
        [1.0, 1.0, 1.0, 1.0],
        [motor["pos"][1] for motor in MOTORS],
        [-motor["pos"][0] for motor in MOTORS],
        [motor["spin"] * yaw_drag_arm for motor in MOTORS],
    ]
    return inverse_4x4(allocation)


def mat_vec(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> list[float]:
    return [sum(row[i] * vector[i] for i in range(len(vector))) for row in matrix]


def controller_step(
    state: RateControllerState,
    command: RateCommand,
    measured_rate: Sequence[float],
    dt: float,
    gains: RateGains,
    mixer: Sequence[Sequence[float]],
) -> RateControlOutput:
    if dt <= 0.0:
        raise ValueError("Rate controller dt must be positive.")
    if len(measured_rate) != 3:
        raise ValueError("measured_rate must contain [p, q, r].")

    rate_cmd = [command.p, command.q, command.r]
    rate = [float(value) for value in measured_rate]
    rate_error = [rate_cmd[i] - rate[i] for i in range(3)]

    limits = [gains.integral_limit_rp, gains.integral_limit_rp, gains.integral_limit_yaw]
    for i in range(3):
        state.rate_integral[i] = clamp(state.rate_integral[i] + rate_error[i] * dt, -limits[i], limits[i])

    thrust_cmd = clamp(command.thrust, 0.0, 4.0 * gains.max_thrust)
    moment_cmd = [
        gains.kp_rp * rate_error[0] + gains.ki_rp * state.rate_integral[0],
        gains.kp_rp * rate_error[1] + gains.ki_rp * state.rate_integral[1],
        gains.kp_yaw * rate_error[2] + gains.ki_yaw * state.rate_integral[2],
    ]

    motor_thrust_cmd = mat_vec(mixer, [thrust_cmd, moment_cmd[0], moment_cmd[1], moment_cmd[2]])
    motor_thrust_cmd = [clamp(value, 0.0, gains.max_thrust) for value in motor_thrust_cmd]

    k_f = gains.max_thrust / (gains.max_motor_speed * gains.max_motor_speed)
    omega_cmd = [math.sqrt(force / k_f) for force in motor_thrust_cmd]
    motor_thrust = []
    for i in range(4):
        tau = gains.motor_tau_up if omega_cmd[i] >= state.motor_speed[i] else gains.motor_tau_down
        alpha = dt / (tau + dt)
        state.motor_speed[i] = clamp(
            state.motor_speed[i] + alpha * (omega_cmd[i] - state.motor_speed[i]),
            0.0,
            gains.max_motor_speed,
        )
        motor_thrust.append(k_f * state.motor_speed[i] * state.motor_speed[i])

    return RateControlOutput(
        omega_cmd=omega_cmd,
        omega=state.motor_speed[:],
        motor_thrust_cmd=motor_thrust_cmd,
        motor_thrust=motor_thrust,
        thrust_cmd=thrust_cmd,
        moment_cmd=moment_cmd,
        rate_error=rate_error,
    )


def gains_from_args(args) -> RateGains:
    return RateGains(
        kp_rp=float(args.kp_rate_rp),
        kp_yaw=float(args.kp_rate_yaw),
        ki_rp=float(args.ki_rate_rp),
        ki_yaw=float(args.ki_rate_yaw),
        integral_limit_rp=float(args.integral_limit_rp),
        integral_limit_yaw=float(args.integral_limit_yaw),
        motor_tau_up=float(args.motor_tau_up),
        motor_tau_down=float(args.motor_tau_down),
        max_motor_speed=float(args.max_motor_speed),
        max_thrust=float(args.max_thrust),
        yaw_drag_arm=float(args.yaw_drag_arm),
    )


def hover_omega(mass: float, max_thrust: float, max_motor_speed: float) -> float:
    return math.sqrt((mass * G / 4.0) / (max_thrust / max_motor_speed**2))
