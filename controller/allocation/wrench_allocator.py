"""Least-squares motor allocation for docked multirotor assemblies."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from controller.allocation.geometry import AssemblyGeometry


@dataclass
class AllocationResult:
    motor_omega_cmd: list[float]
    motor_thrust_cmd: list[float]
    wrench_cmd: list[float]
    wrench_achieved: list[float]
    residual: list[float]
    residual_norm: float
    rank: int
    saturated_count: int
    matrix: list[list[float]]


def _cross(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _mat_vec(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> list[float]:
    return [sum(row[col] * vector[col] for col in range(len(vector))) for row in matrix]


def _transpose(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    if not matrix:
        return []
    return [[row[col] for row in matrix] for col in range(len(matrix[0]))]


def _solve_linear_system(matrix: Sequence[Sequence[float]], rhs: Sequence[float]) -> list[float]:
    n = len(rhs)
    augmented = [list(matrix[row]) + [float(rhs[row])] for row in range(n)]
    matrix_scale = max(max(abs(value) for value in row) for row in matrix)
    pivot_tolerance = max(1e-30, 1e-12 * matrix_scale)
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(augmented[row][col]))
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        pivot_value = augmented[col][col]
        if abs(pivot_value) < pivot_tolerance:
            raise ValueError("Regularized allocation matrix is singular.")
        augmented[col] = [value / pivot_value for value in augmented[col]]
        for row in range(n):
            if row == col:
                continue
            factor = augmented[row][col]
            augmented[row] = [augmented[row][idx] - factor * augmented[col][idx] for idx in range(n + 1)]
    return [augmented[row][-1] for row in range(n)]


def _rank(matrix: Sequence[Sequence[float]], relative_tolerance: float = 1e-9) -> int:
    rows = [list(row) for row in matrix]
    if not rows:
        return 0
    matrix_scale = max(max(abs(value) for value in row) for row in rows)
    tolerance = max(1e-30, relative_tolerance * matrix_scale)
    row_count = len(rows)
    col_count = len(rows[0])
    rank = 0
    for col in range(col_count):
        pivot = max(range(rank, row_count), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) <= tolerance:
            continue
        rows[rank], rows[pivot] = rows[pivot], rows[rank]
        pivot_value = rows[rank][col]
        rows[rank] = [value / pivot_value for value in rows[rank]]
        for row in range(row_count):
            if row == rank:
                continue
            factor = rows[row][col]
            rows[row] = [rows[row][idx] - factor * rows[rank][idx] for idx in range(col_count)]
        rank += 1
        if rank == row_count:
            break
    return rank


def build_allocation_matrix(
    geometry: AssemblyGeometry,
    max_motor_speed: float,
    max_thrust: float,
    yaw_drag_arm: float,
) -> list[list[float]]:
    """Return A where desired assembly wrench is approximately A @ omega^2."""

    k_f = max_thrust / (max_motor_speed * max_motor_speed)
    rows = [[0.0 for _ in geometry.motors] for _ in range(6)]
    for col, motor in enumerate(geometry.motors):
        thrust_axis = motor.axis_world
        torque_arm = _cross(motor.r_world, thrust_axis)
        yaw_axis_torque = [motor.spin * yaw_drag_arm * value for value in thrust_axis]
        for axis in range(3):
            rows[axis][col] = k_f * thrust_axis[axis]
            rows[axis + 3][col] = k_f * (torque_arm[axis] + yaw_axis_torque[axis])
    return rows


def allocate_wrench(
    geometry: AssemblyGeometry,
    wrench_cmd: Sequence[float],
    max_motor_speed: float,
    max_thrust: float,
    yaw_drag_arm: float,
    regularization: float = 1e-6,
) -> AllocationResult:
    """Allocate a world-frame assembly wrench to nonnegative motor speeds.

    The solve uses a damped pseudoinverse on omega squared, then clips each
    motor to the physical range. This is the same control-allocation layer used
    in modular multirotor work: geometry enters only through motor positions
    and thrust axes relative to the assembly COM.
    """

    desired = [float(value) for value in wrench_cmd]
    if len(desired) != 6:
        raise ValueError("wrench_cmd must contain [Fx, Fy, Fz, tau_x, tau_y, tau_z].")

    if not geometry.motors:
        raise ValueError("Cannot allocate a wrench for an assembly with no motors.")

    allocation = build_allocation_matrix(geometry, max_motor_speed, max_thrust, yaw_drag_arm)
    transposed = _transpose(allocation)
    base_gram = [
        [
            sum(allocation[row][col] * allocation[other][col] for col in range(len(geometry.motors)))
            for other in range(6)
        ]
        for row in range(6)
    ]
    diag_scale = max(max(abs(base_gram[row][row]) for row in range(6)), 1e-24)
    damping = max(regularization, 1e-12) * diag_scale

    gram = [row[:] for row in base_gram]
    for row in range(6):
        gram[row][row] += damping
    y = _solve_linear_system(gram, desired)

    omega_squared_cmd = [sum(transposed[col][row] * y[row] for row in range(6)) for col in range(len(transposed))]
    max_omega_squared = max_motor_speed * max_motor_speed
    clipped = [max(0.0, min(max_omega_squared, value)) for value in omega_squared_cmd]
    saturated_count = sum(
        1
        for raw, limited in zip(omega_squared_cmd, clipped)
        if abs(raw - limited) > 1e-6 * max(1.0, max_omega_squared)
    )

    motor_omega_cmd = [math.sqrt(value) for value in clipped]
    k_f = max_thrust / (max_motor_speed * max_motor_speed)
    motor_thrust_cmd = [k_f * value for value in clipped]
    achieved = _mat_vec(allocation, clipped)
    residual = [desired[row] - achieved[row] for row in range(6)]
    residual_norm = math.sqrt(sum(value * value for value in residual))
    return AllocationResult(
        motor_omega_cmd=motor_omega_cmd,
        motor_thrust_cmd=motor_thrust_cmd,
        wrench_cmd=desired,
        wrench_achieved=achieved,
        residual=residual,
        residual_norm=residual_norm,
        rank=_rank(allocation),
        saturated_count=saturated_count,
        matrix=allocation,
    )
