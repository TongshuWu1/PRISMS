"""Build motor geometry for a docked multirotor assembly.

The allocator needs the current world-frame position and thrust axis of every
motor. This module reconstructs those quantities from each CoppeliaSim drone
body pose and the motor layout used by the low-level Crazyflie plant model.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

from controller.low_level import body_rate as flight


@dataclass
class MotorGeometry:
    drone_index: int
    motor_index: int
    body_handle: int
    joint_handle: int | None
    local_position: tuple[float, float, float]
    position_world: list[float]
    axis_world: list[float]
    r_world: list[float]
    spin: float


@dataclass
class AssemblyGeometry:
    com_world: list[float]
    velocity_world: list[float]
    angular_velocity_world: list[float]
    mass: float
    motors: list[MotorGeometry]
    drone_positions: list[list[float]]
    drone_velocities: list[list[float]]
    drone_matrices: list[list[float]]
    drone_angular_velocities: list[list[float]]
    leader_matrix: list[float]
    leader_angular_velocity_world: list[float]


def _average(vectors: Sequence[Sequence[float]]) -> list[float]:
    count = max(1, len(vectors))
    return [sum(vector[axis] for vector in vectors) / count for axis in range(3)]


def _world_point(origin: Sequence[float], axes: tuple[list[float], list[float], list[float]], point: Sequence[float]) -> list[float]:
    return [
        origin[axis] + axes[0][axis] * point[0] + axes[1][axis] * point[1] + axes[2][axis] * point[2]
        for axis in range(3)
    ]


def _subtract(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [a[axis] - b[axis] for axis in range(3)]


def build_assembly_geometry(
    sim,
    drones: Sequence[Any],
    mass_per_drone: float,
) -> AssemblyGeometry:
    """Return current assembly COM state and all motor columns.

    `drones` is intentionally duck-typed: the caller passes the existing
    `simulation.magnetic_docking.DroneAgent` objects without introducing a hard
    dependency from controller code back into simulation code.
    """

    positions: list[list[float]] = []
    velocities: list[list[float]] = []
    matrices: list[list[float]] = []
    angular_velocities: list[list[float]] = []

    for drone in drones:
        positions.append(list(sim.getObjectPosition(drone.body, -1)))
        matrices.append(list(sim.getObjectMatrix(drone.body, -1)))
        lin_vel, ang_vel = sim.getVelocity(drone.body)
        velocities.append(list(lin_vel))
        angular_velocities.append(list(ang_vel))

    com_world = _average(positions)
    velocity_world = _average(velocities)
    angular_velocity_world = _average(angular_velocities)
    motors: list[MotorGeometry] = []

    for drone_slot, drone in enumerate(drones):
        if len(drone.joints) != len(flight.MOTORS):
            raise ValueError(f"Drone {drone_slot} must expose exactly {len(flight.MOTORS)} motor joints.")
        axes = flight.body_axes_from_matrix(matrices[drone_slot])
        for motor_index, motor in enumerate(flight.MOTORS):
            local_position = tuple(float(value) for value in motor["pos"])
            position_world = _world_point(positions[drone_slot], axes, local_position)
            motors.append(
                MotorGeometry(
                    drone_index=drone_slot,
                    motor_index=motor_index,
                    body_handle=int(drone.body),
                    joint_handle=int(drone.joints[motor_index]),
                    local_position=local_position,
                    position_world=position_world,
                    axis_world=axes[2][:],
                    r_world=_subtract(position_world, com_world),
                    spin=float(motor["spin"]),
                )
            )

    leader_matrix = matrices[0] if matrices else [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    leader_angular_velocity_world = angular_velocities[0] if angular_velocities else [0.0, 0.0, 0.0]
    return AssemblyGeometry(
        com_world=com_world,
        velocity_world=velocity_world,
        angular_velocity_world=angular_velocity_world,
        mass=max(0.0, float(mass_per_drone)) * len(drones),
        motors=motors,
        drone_positions=positions,
        drone_velocities=velocities,
        drone_matrices=matrices,
        drone_angular_velocities=angular_velocities,
        leader_matrix=leader_matrix,
        leader_angular_velocity_world=leader_angular_velocity_world,
    )
