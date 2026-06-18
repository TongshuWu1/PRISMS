"""Build motor geometry for a docked multirotor assembly.

The allocator needs the current world-frame position and thrust axis of every
motor. This module reconstructs those quantities from each CoppeliaSim drone
body pose and the motor layout used by the low-level Crazyflie plant model.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

from controller.low_level import body_rate as flight


DEFAULT_MODULE_INERTIA_BOX = (0.176, 0.176, 0.166)


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
    inertia_world: list[list[float]]
    inertia_body: list[list[float]]
    module_inertia_body: list[list[float]]
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


def _weighted_average(vectors: Sequence[Sequence[float]], weights: Sequence[float]) -> list[float]:
    total = sum(weights)
    if total <= 0.0:
        return _average(vectors)
    return [sum(vector[axis] * weights[index] for index, vector in enumerate(vectors)) / total for axis in range(3)]


def _world_point(origin: Sequence[float], axes: tuple[list[float], list[float], list[float]], point: Sequence[float]) -> list[float]:
    return [
        origin[axis] + axes[0][axis] * point[0] + axes[1][axis] * point[1] + axes[2][axis] * point[2]
        for axis in range(3)
    ]


def _subtract(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [a[axis] - b[axis] for axis in range(3)]


def _zeros3() -> list[list[float]]:
    return [[0.0, 0.0, 0.0] for _ in range(3)]


def _identity3() -> list[list[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _add_matrix3(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[a[row][col] + b[row][col] for col in range(3)] for row in range(3)]


def _transpose3(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[matrix[row][col] for row in range(3)] for col in range(3)]


def _matmul3(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    return [
        [sum(a[row][idx] * b[idx][col] for idx in range(3)) for col in range(3)]
        for row in range(3)
    ]


def _rotation_rows(matrix: Sequence[float]) -> list[list[float]]:
    return [
        [matrix[0], matrix[1], matrix[2]],
        [matrix[4], matrix[5], matrix[6]],
        [matrix[8], matrix[9], matrix[10]],
    ]


def _module_box_inertia_body(mass: float, box: Sequence[float]) -> list[list[float]]:
    lx = max(0.0, float(box[0]))
    ly = max(0.0, float(box[1]))
    lz = max(0.0, float(box[2]))
    m = max(0.0, float(mass))
    return [
        [m * (ly * ly + lz * lz) / 12.0, 0.0, 0.0],
        [0.0, m * (lx * lx + lz * lz) / 12.0, 0.0],
        [0.0, 0.0, m * (lx * lx + ly * ly) / 12.0],
    ]


def _rotate_inertia_to_world(inertia_body: Sequence[Sequence[float]], matrix: Sequence[float]) -> list[list[float]]:
    rotation = _rotation_rows(matrix)
    return _matmul3(_matmul3(rotation, inertia_body), _transpose3(rotation))


def _parallel_axis_term(mass: float, offset: Sequence[float]) -> list[list[float]]:
    distance_sq = sum(value * value for value in offset)
    identity = _identity3()
    return [
        [
            max(0.0, float(mass)) * (distance_sq * identity[row][col] - offset[row] * offset[col])
            for col in range(3)
        ]
        for row in range(3)
    ]


def _express_in_leader_body(inertia_world: Sequence[Sequence[float]], leader_matrix: Sequence[float]) -> list[list[float]]:
    rotation = _rotation_rows(leader_matrix)
    return _matmul3(_matmul3(_transpose3(rotation), inertia_world), rotation)


def _assembly_inertia(
    positions: Sequence[Sequence[float]],
    matrices: Sequence[Sequence[float]],
    com_world: Sequence[float],
    mass_per_drone: float,
    module_inertia_body: Sequence[Sequence[float]],
    leader_matrix: Sequence[float],
) -> tuple[list[list[float]], list[list[float]]]:
    inertia_world = _zeros3()
    for position, matrix in zip(positions, matrices):
        rotated = _rotate_inertia_to_world(module_inertia_body, matrix)
        offset = _subtract(position, com_world)
        inertia_world = _add_matrix3(
            inertia_world,
            _add_matrix3(rotated, _parallel_axis_term(mass_per_drone, offset)),
        )
    return inertia_world, _express_in_leader_body(inertia_world, leader_matrix)


def build_assembly_geometry(
    sim,
    drones: Sequence[Any],
    mass_per_drone: float,
    module_inertia_box: Sequence[float] = DEFAULT_MODULE_INERTIA_BOX,
) -> AssemblyGeometry:
    """Return current assembly COM state and all motor columns.

    `drones` is intentionally duck-typed: the caller passes the existing
    `simulation.magnetic_docking.DroneAgent` objects without introducing a hard
    dependency from controller code back into simulation code.

    The assembly inertia is computed from a box approximation of one module's
    body-frame inertia and a parallel-axis shift from each module center to the
    current assembly COM. The default box matches the dimensions used when the
    generated CoppeliaSim plant mass/inertia is assigned.
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

    drone_masses = [max(0.0, float(mass_per_drone)) for _ in drones]
    com_world = _weighted_average(positions, drone_masses)
    velocity_world = _weighted_average(velocities, drone_masses)
    angular_velocity_world = _weighted_average(angular_velocities, drone_masses)
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
    module_inertia_body = _module_box_inertia_body(mass_per_drone, module_inertia_box)
    inertia_world, inertia_body = _assembly_inertia(
        positions,
        matrices,
        com_world,
        max(0.0, float(mass_per_drone)),
        module_inertia_body,
        leader_matrix,
    )
    return AssemblyGeometry(
        com_world=com_world,
        velocity_world=velocity_world,
        angular_velocity_world=angular_velocity_world,
        mass=max(0.0, float(mass_per_drone)) * len(drones),
        inertia_world=inertia_world,
        inertia_body=inertia_body,
        module_inertia_body=module_inertia_body,
        motors=motors,
        drone_positions=positions,
        drone_velocities=velocities,
        drone_matrices=matrices,
        drone_angular_velocities=angular_velocities,
        leader_matrix=leader_matrix,
        leader_angular_velocity_world=leader_angular_velocity_world,
    )
