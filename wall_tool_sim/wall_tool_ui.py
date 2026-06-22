#!/usr/bin/env python3
"""Interactive 2-D PRISMS wall-tool simulator.

The model is intentionally small, but it is not just a drawing. The suspended
system is integrated as a Cartesian point mass under gravity, finite cable
tension, and two bounded tilted drone thrust axes. The spool commands cable
velocity and builds cable tension through a unilateral spring-damper cable.
Click any point on the wall to command a smooth straight-line move, or enable
append mode to queue a smooth multi-waypoint trajectory.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.low_level import rate_control  # noqa: E402

try:
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle
    from matplotlib.widgets import Button, Slider
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing matplotlib. Install project requirements first:\n"
        "  python -m pip install -r requirements.txt"
    ) from exc


Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]
VecN = tuple[float, ...]

SQRT5 = math.sqrt(5.0)
CAGE_ROT_Y_RAD = math.pi / 4.0
VISUAL_DEPTH_X = 0.0
VISUAL_DEPTH_Z = 0.0
DRONE_RIGHT_HEX = (1, 1, 1)
DRONE_LEFT_HEX = (-1, -1, -1)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def dot2(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1])


def dot3(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def sub3(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2])


def cross3(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    )


def norm3(a: Sequence[float]) -> float:
    return math.sqrt(dot3(a, a))


def normalize3(a: Sequence[float]) -> Vec3:
    length = norm3(a)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return float(a[0]) / length, float(a[1]) / length, float(a[2]) / length


def add2(a: Sequence[float], b: Sequence[float]) -> Vec2:
    return float(a[0]) + float(b[0]), float(a[1]) + float(b[1])


def sub2(a: Sequence[float], b: Sequence[float]) -> Vec2:
    return float(a[0]) - float(b[0]), float(a[1]) - float(b[1])


def scale2(vector: Sequence[float], gain: float) -> Vec2:
    return float(vector[0]) * gain, float(vector[1]) * gain


def rotate2(vector: Sequence[float], angle: float) -> Vec2:
    c = math.cos(angle)
    s = math.sin(angle)
    return c * float(vector[0]) - s * float(vector[1]), s * float(vector[0]) + c * float(vector[1])


def cross2(moment_arm: Sequence[float], force: Sequence[float]) -> float:
    return float(moment_arm[0]) * float(force[1]) - float(moment_arm[1]) * float(force[0])


def normalize2(vector: Sequence[float]) -> Vec2:
    length = math.hypot(float(vector[0]), float(vector[1]))
    if length < 1e-12:
        return (0.0, 0.0)
    return float(vector[0]) / length, float(vector[1]) / length


def limit_norm2(vector: Sequence[float], max_norm: float) -> Vec2:
    length = math.hypot(float(vector[0]), float(vector[1]))
    if length <= max_norm or length < 1e-12:
        return float(vector[0]), float(vector[1])
    scale = max_norm / length
    return float(vector[0]) * scale, float(vector[1]) * scale


def distance2(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def convex_hull(points: Sequence[Vec2]) -> list[Vec2]:
    unique_points = sorted(set((float(point[0]), float(point[1])) for point in points))
    if len(unique_points) <= 1:
        return list(unique_points)

    def cross(o: Vec2, a: Vec2, b: Vec2) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[Vec2] = []
    for point in unique_points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[Vec2] = []
    for point in reversed(unique_points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def dotn(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(x) * float(y) for x, y in zip(a, b))


def addn(a: Sequence[float], b: Sequence[float]) -> VecN:
    return tuple(float(x) + float(y) for x, y in zip(a, b))


def subn(a: Sequence[float], b: Sequence[float]) -> VecN:
    return tuple(float(x) - float(y) for x, y in zip(a, b))


def scalen(vector: Sequence[float], gain: float) -> VecN:
    return tuple(float(value) * gain for value in vector)


@dataclass(frozen=True)
class Face:
    kind: str
    normal_key: tuple[int, int, int]
    normal: Vec3
    indices: tuple[int, ...]
    center: Vec3


@dataclass(frozen=True)
class TruncatedOctahedronGeometry:
    vertices: tuple[Vec3, ...]
    faces: tuple[Face, ...]
    edges: tuple[tuple[int, int], ...]
    face_by_normal: dict[tuple[int, int, int], Face]


def ordered_face_indices(vertices: Sequence[Vec3], indices: Iterable[int], normal: Vec3) -> tuple[int, ...]:
    face_indices = list(indices)
    center = (
        sum(vertices[index][0] for index in face_indices) / len(face_indices),
        sum(vertices[index][1] for index in face_indices) / len(face_indices),
        sum(vertices[index][2] for index in face_indices) / len(face_indices),
    )
    n = normalize3(normal)
    reference = (0.0, 0.0, 1.0)
    if abs(dot3(n, reference)) > 0.90:
        reference = (0.0, 1.0, 0.0)
    u = normalize3(cross3(n, reference))
    v = normalize3(cross3(n, u))

    def angle(index: int) -> float:
        relative = sub3(vertices[index], center)
        return math.atan2(dot3(relative, v), dot3(relative, u))

    return tuple(sorted(face_indices, key=angle))


def build_truncated_octahedron() -> TruncatedOctahedronGeometry:
    vertices = tuple(
        sorted(
            set(
                itertools.chain.from_iterable(
                    itertools.permutations((0.0, one, two))
                    for one in (-1.0, 1.0)
                    for two in (-2.0, 2.0)
                )
            )
        )
    )
    faces: list[Face] = []

    for axis in range(3):
        for sign in (-1, 1):
            indices = [index for index, point in enumerate(vertices) if point[axis] == 2.0 * sign]
            normal_key = [0, 0, 0]
            normal_key[axis] = sign
            ordered = ordered_face_indices(vertices, indices, tuple(float(value) for value in normal_key))
            center = tuple(sum(vertices[index][dim] for index in ordered) / len(ordered) for dim in range(3))
            faces.append(
                Face(
                    kind="square",
                    normal_key=tuple(normal_key),
                    normal=normalize3(normal_key),
                    indices=ordered,
                    center=center,  # type: ignore[arg-type]
                )
            )

    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                indices = [
                    index
                    for index, point in enumerate(vertices)
                    if sx * point[0] + sy * point[1] + sz * point[2] == 3.0
                ]
                normal_key = (sx, sy, sz)
                ordered = ordered_face_indices(vertices, indices, normal_key)
                center = tuple(sum(vertices[index][dim] for index in ordered) / len(ordered) for dim in range(3))
                faces.append(
                    Face(
                        kind="hex",
                        normal_key=normal_key,
                        normal=normalize3(normal_key),
                        indices=ordered,
                        center=center,  # type: ignore[arg-type]
                    )
                )

    edge_set: set[tuple[int, int]] = set()
    for face in faces:
        for index, vertex_index in enumerate(face.indices):
            next_index = face.indices[(index + 1) % len(face.indices)]
            edge_set.add(tuple(sorted((vertex_index, next_index))))

    return TruncatedOctahedronGeometry(
        vertices=vertices,
        faces=tuple(faces),
        edges=tuple(sorted(edge_set)),
        face_by_normal={face.normal_key: face for face in faces},
    )


GEOMETRY = build_truncated_octahedron()


def rotate_cage(point: Vec3) -> Vec3:
    c = math.cos(CAGE_ROT_Y_RAD)
    s = math.sin(CAGE_ROT_Y_RAD)
    x, y, z = point
    return c * x + s * z, y, -s * x + c * z


def project_local(point: Vec3, radius: float) -> Vec2:
    scale = radius / SQRT5
    rotated = rotate_cage(point)
    return rotated[0] * scale, rotated[2] * scale


def project_local_visual(point: Vec3, radius: float, attitude: float = 0.0) -> Vec2:
    scale = radius / SQRT5
    rotated = rotate_cage(point)
    wall_point = rotate2((rotated[0] * scale, rotated[2] * scale), attitude)
    depth = rotated[1] * scale
    return wall_point[0] + VISUAL_DEPTH_X * depth, wall_point[1] + VISUAL_DEPTH_Z * depth


def projected_vertices(center: Vec2, radius: float, attitude: float = 0.0) -> list[Vec2]:
    return [add2(center, rotate2(project_local(vertex, radius), attitude)) for vertex in GEOMETRY.vertices]


def projected_faces(center: Vec2, radius: float, attitude: float = 0.0) -> list[list[Vec2]]:
    points = projected_vertices(center, radius, attitude)
    return [[points[index] for index in face.indices] for face in GEOMETRY.faces]


def projected_edges(center: Vec2, radius: float, attitude: float = 0.0) -> list[tuple[Vec2, Vec2]]:
    points = projected_vertices(center, radius, attitude)
    return [(points[start], points[end]) for start, end in GEOMETRY.edges]


def projected_face_polygon(center: Vec2, radius: float, normal_key: tuple[int, int, int], attitude: float = 0.0) -> list[Vec2]:
    face = GEOMETRY.face_by_normal[normal_key]
    points = projected_vertices(center, radius, attitude)
    return [points[index] for index in face.indices]


def projected_face_offset(radius: float, normal_key: tuple[int, int, int], attitude: float = 0.0) -> Vec2:
    return rotate2(project_local(GEOMETRY.face_by_normal[normal_key].center, radius), attitude)


def visual_projected_vertices(center: Vec2, radius: float, attitude: float = 0.0) -> list[Vec2]:
    return [add2(center, project_local_visual(vertex, radius, attitude)) for vertex in GEOMETRY.vertices]


def visual_projected_faces(center: Vec2, radius: float, attitude: float = 0.0) -> list[list[Vec2]]:
    points = visual_projected_vertices(center, radius, attitude)
    return [[points[index] for index in face.indices] for face in GEOMETRY.faces]


def visual_projected_edges(center: Vec2, radius: float, attitude: float = 0.0) -> list[tuple[Vec2, Vec2]]:
    points = visual_projected_vertices(center, radius, attitude)
    return [(points[start], points[end]) for start, end in GEOMETRY.edges]


def visual_projected_face_polygon(
    center: Vec2,
    radius: float,
    normal_key: tuple[int, int, int],
    attitude: float = 0.0,
) -> list[Vec2]:
    face = GEOMETRY.face_by_normal[normal_key]
    points = visual_projected_vertices(center, radius, attitude)
    return [points[index] for index in face.indices]


def visual_projected_face_offset(radius: float, normal_key: tuple[int, int, int], attitude: float = 0.0) -> Vec2:
    return project_local_visual(GEOMETRY.face_by_normal[normal_key].center, radius, attitude)


def module_extents(center: Vec2, radius: float, attitude: float = 0.0) -> tuple[float, float, float, float]:
    points = projected_vertices(center, radius, attitude)
    xs = [point[0] for point in points]
    zs = [point[1] for point in points]
    return min(xs), max(xs), min(zs), max(zs)


def cable_mount_offset(radius: float, attitude: float = 0.0) -> Vec2:
    points = projected_vertices((0.0, 0.0), radius, attitude)
    max_z = max(point[1] for point in points)
    top_points = [point for point in points if max_z - point[1] < 1e-8]
    return sum(point[0] for point in top_points) / len(top_points), max_z


def visual_cable_mount_offset(radius: float, attitude: float = 0.0) -> Vec2:
    points = visual_projected_vertices((0.0, 0.0), radius, attitude)
    max_z = max(point[1] for point in points)
    top_points = [point for point in points if max_z - point[1] < 1e-8]
    return sum(point[0] for point in top_points) / len(top_points), max_z


def regular_hexagon(center: Vec2, radius: float, attitude: float = 0.0) -> list[Vec2]:
    return [
        add2(center, rotate2((radius * math.cos(attitude + index * math.pi / 3.0), radius * math.sin(attitude + index * math.pi / 3.0)), 0.0))
        for index in range(6)
    ]


def payload_face_center(center: Vec2, half_length: float, attitude: float, side: int) -> Vec2:
    return add2(center, rotate2((side * half_length, 0.0), attitude))


def payload_face_polygon(center: Vec2, half_length: float, face_radius: float, attitude: float, side: int) -> list[Vec2]:
    face_center = payload_face_center(center, half_length, attitude, side)
    return regular_hexagon(face_center, face_radius, attitude + math.pi / 6.0)


def payload_body_polygon(center: Vec2, half_length: float, face_radius: float, attitude: float) -> list[Vec2]:
    left = payload_face_center(center, half_length, attitude, -1)
    right = payload_face_center(center, half_length, attitude, 1)
    top = rotate2((0.0, face_radius * 0.82), attitude)
    bottom = rotate2((0.0, -face_radius * 0.82), attitude)
    return [add2(left, top), add2(right, top), add2(right, bottom), add2(left, bottom)]


@dataclass(frozen=True)
class SimParams:
    wall_width: float = 4.0
    wall_height: float = 4.0
    dt: float = 0.005
    drone_mass: float = 0.050
    payload_tool_mass: float = 0.075
    gravity: float = rate_control.G
    cage_radius: float = 0.18
    payload_half_length: float = 0.190
    payload_hex_radius: float = 0.114
    module_gap: float = 0.0
    max_thrust_per_drone: float = 0.150 * rate_control.G
    drone_accel_kp: float = 18.0
    drone_accel_kd: float = 7.5
    radial_accel_kp: float = 8.0
    radial_accel_kd: float = 5.0
    tangential_accel_kp: float = 18.0
    tangential_accel_kd: float = 7.5
    pendulum_theta_kp: float = 17.0
    pendulum_theta_kd: float = 6.2
    max_pendulum_theta_ddot: float = 6.5
    pendulum_radial_force_penalty: float = 0.18
    move_cable_torque_comp_fraction: float = 1.0
    max_radial_accel: float = 3.6
    max_tangential_accel: float = 5.0
    max_cartesian_accel: float = 4.2
    spool_velocity_kp: float = 2.8
    spool_velocity_kd: float = 0.55
    spool_encoder_kp: float = 0.75
    spool_radial_accel_ff: float = 0.06
    drone_radial_accel_fraction: float = 1.0
    cable_first_assist_fraction: float = 0.0
    max_extra_cable_tension: float = 0.45
    cable_support_fraction_target: float = 0.40
    max_cable_side_load_fraction: float = 0.35
    cable_tension_feedforward_fraction: float = 0.45
    max_spool_speed: float = 0.58
    cable_stiffness: float = 260.0
    cable_damping: float = 2.0
    max_cable_damping_force: float = 1.2
    cable_taut_band: float = 0.006
    max_spool_tension: float = 24.0
    spool_tension_kv: float = 0.16
    tension_feedback_pay_out_deadband: float = 0.035
    tension_feedback_pay_out_release: float = 0.180
    min_tracking_tension: float = 0.10
    lower_cable_support_fraction: float = 0.16
    cable_support_floor_fraction: float = 0.28
    lift_cable_support_fraction: float = 0.78
    hold_cable_support_fraction: float = 0.72
    radial_motion_deadband: float = 0.018
    radial_length_deadband: float = 0.020
    slack_recovery_spool_speed: float = 0.26
    slack_pay_out_speed: float = 0.040
    gravity_lowering_payout_speed: float = 0.22
    taut_payout_buffer: float = 0.002
    cable_tension_cost: float = 0.004
    cable_geometry_cost: float = 4.0
    drone_thrust_cost: float = 0.080
    min_cable_vertical_efficiency: float = 0.08
    tension_filter_tau: float = 0.18
    max_tension_target_rate: float = 10.0
    reference_speed_min: float = 0.28
    reference_slowdown_rate: float = 5.0
    reference_recovery_rate: float = 2.6
    tracking_error_slowdown_m: float = 0.100
    tracking_error_full_slow_m: float = 0.220
    thrust_slowdown_fraction: float = 0.82
    residual_slowdown_fraction: float = 0.020
    geometry_slowdown_efficiency: float = 0.35
    min_control_cable_length: float = 0.62
    move_attitude_kp: float = 0.0
    move_attitude_kd: float = 0.8
    move_max_attitude_torque: float = 0.025
    hold_attitude_kp: float = 70.0
    hold_attitude_kd: float = 10.0
    hold_max_attitude_torque: float = 0.12
    attitude_hold_error_m: float = 0.06
    attitude_hold_speed_mps: float = 0.08
    attitude_hold_release_error_m: float = 0.28
    attitude_hold_release_speed_mps: float = 0.45
    hold_position_kp: float = 8.0
    hold_position_kd: float = 4.0
    hold_max_position_accel: float = 2.2
    hold_tension_residual_fraction: float = 0.05
    hold_tension_search_steps: int = 32
    rotational_damping: float = 0.010
    torque_residual_length_scale: float = 0.35
    shallow_hold_torque_residual_length_scale: float = 1.55
    shallow_hold_torque_scale_efficiency: float = 0.75
    nominal_attitude_rad: float = 0.0
    path_speed: float = 0.32
    waypoint_tolerance: float = 0.012
    min_cable_length: float = 0.10
    max_cable_length: float = 4.65
    initial_payload: Vec2 = (0.0, 2.00)

    @property
    def anchor(self) -> Vec2:
        return (0.0, self.wall_height)

    @property
    def total_mass(self) -> float:
        return self.payload_tool_mass + 2.0 * self.drone_mass

    @property
    def hex_face_tilt_rad(self) -> float:
        return math.atan2(1.0, math.sqrt(2.0))

    @property
    def assembly_inertia(self) -> float:
        radius = self.cage_radius
        payload_width = 2.0 * self.payload_half_length
        payload_height = 2.0 * self.payload_hex_radius
        payload_shape_inertia = self.payload_tool_mass * (payload_width * payload_width + payload_height * payload_height) / 12.0
        left_payload_offset = (-self.payload_half_length, 0.0)
        left_drone_offset = projected_face_offset(radius, DRONE_RIGHT_HEX)
        right_payload_offset = (self.payload_half_length, 0.0)
        right_drone_offset = projected_face_offset(radius, DRONE_LEFT_HEX)
        left_distance = distance2(left_payload_offset, left_drone_offset)
        right_distance = distance2(right_payload_offset, right_drone_offset)
        drone_shape_inertia = 0.25 * self.drone_mass * radius * radius
        return (
            payload_shape_inertia
            + drone_shape_inertia
            + self.drone_mass * left_distance * left_distance
            + drone_shape_inertia
            + self.drone_mass * right_distance * right_distance
        )


@dataclass
class SimState:
    t: float
    theta: float
    theta_dot: float
    length: float
    length_dot: float
    length_ddot: float
    attitude: float
    angular_velocity: float
    angular_acceleration: float
    cable_length: float
    cable_stretch: float
    cable_slack: bool
    cable_tension_saturated: bool
    payload_velocity: Vec2
    payload_acceleration: Vec2
    payload: Vec2
    measured_payload: Vec2
    estimated_payload_velocity: Vec2
    measured_theta: float
    measured_theta_dot: float
    measured_line_length: float
    measured_attitude: float
    measured_angular_velocity: float
    measured_cable_velocity: float
    tool_head: Vec2
    reference: Vec2
    desired_tool_head: Vec2
    reference_velocity: Vec2
    reference_acceleration: Vec2
    target: Vec2
    measured_tool_error: float
    spool_velocity_cmd: float
    drone_accel_cmd: float
    reference_speed_scale: float
    desired_cable_tension: float
    measured_cable_length: float
    measured_tension: float
    desired_drone_force: Vec2
    drone_force: Vec2
    cable_force: Vec2
    desired_attitude_torque: float
    attitude_torque: float
    cable_torque: float
    left_torque: float
    right_torque: float
    left_thrust: float
    right_thrust: float
    tension: float
    tangential_force: float
    desired_tangential_force: float
    allocation_residual: float
    drone_vertical_force: float
    cable_vertical_force: float
    path_error: float
    tool_error: float
    active_waypoints: int
    saturated: bool


@dataclass(frozen=True)
class ReferenceState:
    position: Vec2
    velocity: Vec2
    acceleration: Vec2
    final_target: Vec2
    active: bool
    waypoint_count: int


def solve3(matrix: Sequence[Sequence[float]], rhs: Sequence[float]) -> tuple[float, float, float]:
    rows = [[float(matrix[row][col]) for col in range(3)] + [float(rhs[row])] for row in range(3)]
    for pivot in range(3):
        best = max(range(pivot, 3), key=lambda row: abs(rows[row][pivot]))
        rows[pivot], rows[best] = rows[best], rows[pivot]
        pivot_value = rows[pivot][pivot]
        if abs(pivot_value) < 1e-12:
            raise ValueError("singular trajectory solve")
        for col in range(pivot, 4):
            rows[pivot][col] /= pivot_value
        for row in range(3):
            if row == pivot:
                continue
            scale = rows[row][pivot]
            for col in range(pivot, 4):
                rows[row][col] -= scale * rows[pivot][col]
    return rows[0][3], rows[1][3], rows[2][3]


def solve_linear_system(matrix: Sequence[Sequence[float]], rhs: Sequence[float]) -> tuple[float, ...]:
    size = len(rhs)
    rows = [[float(matrix[row][col]) for col in range(size)] + [float(rhs[row])] for row in range(size)]
    for pivot in range(size):
        best = max(range(pivot, size), key=lambda row: abs(rows[row][pivot]))
        rows[pivot], rows[best] = rows[best], rows[pivot]
        pivot_value = rows[pivot][pivot]
        if abs(pivot_value) < 1e-12:
            raise ValueError("singular allocation solve")
        for col in range(pivot, size + 1):
            rows[pivot][col] /= pivot_value
        for row in range(size):
            if row == pivot:
                continue
            scale = rows[row][pivot]
            for col in range(pivot, size + 1):
                rows[row][col] -= scale * rows[pivot][col]
    return tuple(rows[row][size] for row in range(size))


def quintic_coefficients(
    p0: float,
    v0: float,
    a0: float,
    p1: float,
    v1: float,
    a1: float,
    duration: float,
) -> tuple[float, float, float, float, float, float]:
    t = duration
    c0 = p0
    c1 = v0
    c2 = 0.5 * a0
    matrix = (
        (t**3, t**4, t**5),
        (3.0 * t**2, 4.0 * t**3, 5.0 * t**4),
        (6.0 * t, 12.0 * t**2, 20.0 * t**3),
    )
    rhs = (
        p1 - (c0 + c1 * t + c2 * t**2),
        v1 - (c1 + 2.0 * c2 * t),
        a1 - 2.0 * c2,
    )
    c3, c4, c5 = solve3(matrix, rhs)
    return c0, c1, c2, c3, c4, c5


@dataclass(frozen=True)
class QuinticSegment:
    duration: float
    coeff_x: tuple[float, float, float, float, float, float]
    coeff_z: tuple[float, float, float, float, float, float]
    end: Vec2

    @staticmethod
    def build(
        start: Vec2,
        start_velocity: Vec2,
        start_acceleration: Vec2,
        end: Vec2,
        end_velocity: Vec2,
        end_acceleration: Vec2,
        duration: float,
    ) -> "QuinticSegment":
        duration = max(0.20, duration)
        return QuinticSegment(
            duration=duration,
            coeff_x=quintic_coefficients(
                start[0],
                start_velocity[0],
                start_acceleration[0],
                end[0],
                end_velocity[0],
                end_acceleration[0],
                duration,
            ),
            coeff_z=quintic_coefficients(
                start[1],
                start_velocity[1],
                start_acceleration[1],
                end[1],
                end_velocity[1],
                end_acceleration[1],
                duration,
            ),
            end=end,
        )

    def sample(self, time_s: float) -> ReferenceState:
        t = clamp(time_s, 0.0, self.duration)
        px, vx, ax = self._sample_axis(self.coeff_x, t)
        pz, vz, az = self._sample_axis(self.coeff_z, t)
        return ReferenceState(
            position=(px, pz),
            velocity=(vx, vz),
            acceleration=(ax, az),
            final_target=self.end,
            active=t < self.duration,
            waypoint_count=1,
        )

    @staticmethod
    def _sample_axis(coefficients: Sequence[float], t: float) -> tuple[float, float, float]:
        c0, c1, c2, c3, c4, c5 = coefficients
        position = c0 + c1 * t + c2 * t**2 + c3 * t**3 + c4 * t**4 + c5 * t**5
        velocity = c1 + 2.0 * c2 * t + 3.0 * c3 * t**2 + 4.0 * c4 * t**3 + 5.0 * c5 * t**4
        acceleration = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t**2 + 20.0 * c5 * t**3
        return position, velocity, acceleration


@dataclass(frozen=True)
class SampledPathSegment:
    samples: tuple[Vec2, ...]
    lengths: tuple[float, ...]
    duration: float
    end: Vec2

    @staticmethod
    def build(points: Sequence[Vec2], speed: float) -> "SampledPathSegment":
        cleaned: list[Vec2] = []
        for point in points:
            if not cleaned or distance2(point, cleaned[-1]) > 1e-6:
                cleaned.append(point)
        if len(cleaned) < 2:
            cleaned = [cleaned[0], cleaned[0]] if cleaned else [(0.0, 0.0), (0.0, 0.0)]

        samples = SampledPathSegment._catmull_rom_samples(cleaned)
        lengths = [0.0]
        for index in range(1, len(samples)):
            lengths.append(lengths[-1] + distance2(samples[index - 1], samples[index]))
        total_length = max(lengths[-1], 1e-9)
        return SampledPathSegment(
            samples=tuple(samples),
            lengths=tuple(lengths),
            duration=max(0.45, total_length / max(speed, 1e-6)),
            end=samples[-1],
        )

    def sample(self, time_s: float) -> ReferenceState:
        if len(self.samples) < 2 or self.duration <= 1e-9:
            point = self.end
            return ReferenceState(point, (0.0, 0.0), (0.0, 0.0), self.end, False, 1)

        u = clamp(time_s / self.duration, 0.0, 1.0)
        sigma = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        sigma_dot = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / self.duration
        sigma_ddot = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / (self.duration * self.duration)
        total_length = self.lengths[-1]
        distance_along = sigma * total_length
        index = self._length_index(distance_along)
        segment_length = max(self.lengths[index + 1] - self.lengths[index], 1e-9)
        local = (distance_along - self.lengths[index]) / segment_length
        start = self.samples[index]
        end = self.samples[index + 1]
        tangent = scale2(sub2(end, start), 1.0 / segment_length)
        position = add2(start, scale2(sub2(end, start), local))
        velocity = scale2(tangent, sigma_dot * total_length)
        acceleration = scale2(tangent, sigma_ddot * total_length)
        return ReferenceState(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            final_target=self.end,
            active=time_s < self.duration,
            waypoint_count=1,
        )

    def _length_index(self, distance_along: float) -> int:
        for index in range(len(self.lengths) - 1):
            if distance_along <= self.lengths[index + 1]:
                return index
        return max(0, len(self.lengths) - 2)

    @staticmethod
    def _catmull_rom_samples(points: Sequence[Vec2]) -> list[Vec2]:
        samples: list[Vec2] = []
        for index in range(len(points) - 1):
            p0 = points[max(0, index - 1)]
            p1 = points[index]
            p2 = points[index + 1]
            p3 = points[min(len(points) - 1, index + 2)]
            span = distance2(p1, p2)
            sample_count = max(5, int(span / 0.030))
            for sample_index in range(sample_count):
                u = sample_index / sample_count
                samples.append(SampledPathSegment._catmull_rom_point(p0, p1, p2, p3, u))
        samples.append(points[-1])
        filtered: list[Vec2] = []
        for sample in samples:
            if not filtered or distance2(sample, filtered[-1]) > 1e-5:
                filtered.append(sample)
        return filtered

    @staticmethod
    def _catmull_rom_point(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, u: float) -> Vec2:
        u2 = u * u
        u3 = u2 * u
        x = 0.5 * (
            2.0 * p1[0]
            + (-p0[0] + p2[0]) * u
            + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * u2
            + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * u3
        )
        z = 0.5 * (
            2.0 * p1[1]
            + (-p0[1] + p2[1]) * u
            + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * u2
            + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * u3
        )
        return x, z


class ReferenceTrajectory:
    """Smooth reference generator for straight moves and waypoint curves."""

    def __init__(self, initial_position: Vec2, speed: float, tolerance: float) -> None:
        self.speed = speed
        self.tolerance = tolerance
        self.position = initial_position
        self.velocity: Vec2 = (0.0, 0.0)
        self.acceleration: Vec2 = (0.0, 0.0)
        self.goals: list[Vec2] = []
        self.segments: list[QuinticSegment] = []
        self.segment_time = 0.0
        self.final_target = initial_position
        self.mode = "hold"

    def reset(self, position: Vec2) -> None:
        self.position = position
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals.clear()
        self.segments.clear()
        self.segment_time = 0.0
        self.final_target = position
        self.mode = "hold"

    def command_straight(self, start: Vec2, goal: Vec2) -> None:
        self.position = start
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals = [goal]
        self.final_target = goal
        self.mode = "straight"
        duration = self._segment_duration(start, goal)
        self.segments = [
            QuinticSegment.build(
                start,
                (0.0, 0.0),
                (0.0, 0.0),
                goal,
                (0.0, 0.0),
                (0.0, 0.0),
                duration,
            )
        ]
        self.segment_time = 0.0

    def append_smooth_waypoint(self, start: Vec2, goal: Vec2) -> None:
        if not self.segments:
            self.position = start
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
        self.goals.append(goal)
        self.final_target = goal
        self.mode = "smooth"
        self._rebuild_smooth_segments()

    def command_smooth_path(self, start: Vec2, goals: Sequence[Vec2]) -> None:
        self.position = start
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals = [goals[-1]] if goals else []
        self.final_target = self.goals[-1] if self.goals else start
        self.mode = "draw"
        points = [start, *goals]
        self.segments = [SampledPathSegment.build(points, self.speed)] if len(points) >= 2 else []
        self.segment_time = 0.0

    def clear(self, hold_position: Vec2) -> None:
        self.reset(hold_position)

    def advance(self, dt: float) -> ReferenceState:
        if not self.segments:
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            self.final_target = self.position
            return self.state()

        remaining_dt = dt
        while self.segments and remaining_dt >= 0.0:
            segment = self.segments[0]
            time_left = segment.duration - self.segment_time
            if remaining_dt < time_left:
                self.segment_time += remaining_dt
                sample = segment.sample(self.segment_time)
                self.position = sample.position
                self.velocity = sample.velocity
                self.acceleration = sample.acceleration
                break

            sample = segment.sample(segment.duration)
            self.position = sample.position
            self.velocity = sample.velocity
            self.acceleration = sample.acceleration
            remaining_dt -= max(0.0, time_left)
            self.segments.pop(0)
            if self.goals:
                self.goals.pop(0)
            self.segment_time = 0.0
            if remaining_dt <= 1e-12:
                break

        if not self.segments:
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            self.final_target = self.position
            self.mode = "hold"
        return self.state()

    def state(self) -> ReferenceState:
        return ReferenceState(
            position=self.position,
            velocity=self.velocity,
            acceleration=self.acceleration,
            final_target=self.final_target,
            active=bool(self.segments),
            waypoint_count=len(self.goals),
        )

    def pending_path(self) -> list[Vec2]:
        if not self.segments:
            return [self.position]
        points: list[Vec2] = []
        for index, segment in enumerate(self.segments):
            start_time = self.segment_time if index == 0 else 0.0
            sample_count = max(6, int((segment.duration - start_time) / 0.12))
            for sample_index in range(sample_count + 1):
                u = sample_index / sample_count
                t = start_time + u * (segment.duration - start_time)
                points.append(segment.sample(t).position)
        return points

    def _rebuild_smooth_segments(self) -> None:
        points = [self.position, *self.goals]
        if len(points) < 2:
            self.segments.clear()
            self.segment_time = 0.0
            return

        velocities = self._waypoint_velocities(points)
        velocities[0] = self.velocity
        velocities[-1] = (0.0, 0.0)
        accelerations = [(0.0, 0.0) for _ in points]
        accelerations[0] = self.acceleration

        self.segments = []
        for index in range(len(points) - 1):
            duration = self._segment_duration(points[index], points[index + 1])
            self.segments.append(
                QuinticSegment.build(
                    points[index],
                    velocities[index],
                    accelerations[index],
                    points[index + 1],
                    velocities[index + 1],
                    accelerations[index + 1],
                    duration,
                )
            )
        self.segment_time = 0.0

    def _waypoint_velocities(self, points: Sequence[Vec2]) -> list[Vec2]:
        velocities = [(0.0, 0.0) for _ in points]
        for index in range(1, len(points) - 1):
            chord = sub2(points[index + 1], points[index - 1])
            direction = normalize2(chord)
            prev_distance = distance2(points[index], points[index - 1])
            next_distance = distance2(points[index + 1], points[index])
            local_speed = min(self.speed, 0.5 * (prev_distance + next_distance) / max(self._segment_duration(points[index - 1], points[index + 1]), 1e-6))
            velocities[index] = scale2(direction, local_speed)
        return velocities

    def _segment_duration(self, start: Vec2, end: Vec2) -> float:
        return max(0.45, distance2(start, end) / max(self.speed, 1e-6))


class WallToolSimulator:
    def __init__(self, params: SimParams) -> None:
        self.params = params
        self.default_target = params.initial_payload
        self.trajectory = ReferenceTrajectory(
            params.initial_payload,
            speed=params.path_speed,
            tolerance=params.waypoint_tolerance,
        )
        self.reset()

    def reset(self) -> None:
        self.t = 0.0
        self.position = self.default_target
        self.velocity: Vec2 = (0.0, 0.0)
        self.acceleration: Vec2 = (0.0, 0.0)
        self.theta = 0.0
        self.theta_dot = 0.0
        self.length = 0.0
        self.length_dot = 0.0
        self.length_ddot = 0.0
        self.attitude = self.params.nominal_attitude_rad
        self.angular_velocity = 0.0
        self.angular_acceleration = 0.0
        initial_distance = self._point_to_polar(self.default_target)[1]
        initial_tension = self._static_cable_tension_target(self.default_target)
        self.cable_length = clamp(
            initial_distance - initial_tension / max(self.params.cable_stiffness, 1e-9),
            self.params.min_cable_length,
            self.params.max_cable_length,
        )
        self.cable_stretch = initial_distance - self.cable_length
        self.cable_slack = False
        self.cable_tension_saturated = False
        self.reference_speed_scale = 1.0
        self.hold_latched = False
        self.filtered_cable_tension_target = initial_tension
        self.actual_tension = initial_tension
        self.last_spool_velocity_cmd = 0.0
        self.measured_payload = self.position
        self.estimated_payload_velocity = (0.0, 0.0)
        self.measured_theta = 0.0
        self.measured_theta_dot = 0.0
        self.measured_line_length = initial_distance
        self.measured_attitude = self.attitude
        self.measured_angular_velocity = 0.0
        self.measured_cable_length = self.cable_length
        self.measured_cable_velocity = 0.0
        self.measured_tension = initial_tension
        self._update_cable_coordinates()
        self._update_sensor_estimate()
        self.trajectory.reset(self.default_target)
        self.history: list[SimState] = [
            self.snapshot(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                False,
                self.trajectory.state(),
            )
        ]

    def _clamp_wall_point(self, point: Vec2) -> Vec2:
        margin = max(self.params.cage_radius, self.params.payload_half_length, self.params.payload_hex_radius) * 1.4
        return (
            clamp(point[0], -self.params.wall_width / 2.0 + margin, self.params.wall_width / 2.0 - margin),
            clamp(point[1], margin, self.params.wall_height - margin),
        )

    def set_target(self, point: Vec2) -> None:
        self.hold_latched = False
        self.trajectory.command_straight(self._payload_from_state(), self._clamp_wall_point(point))

    def append_target(self, point: Vec2) -> None:
        self.hold_latched = False
        self.trajectory.append_smooth_waypoint(self._payload_from_state(), self._clamp_wall_point(point))

    def set_smooth_path(self, points: Sequence[Vec2]) -> None:
        clamped_points = [self._clamp_wall_point(point) for point in points]
        if not clamped_points:
            return
        self.hold_latched = False
        self.trajectory.command_smooth_path(self._payload_from_state(), clamped_points)

    def clear_trajectory(self) -> None:
        self.trajectory.clear(self._payload_from_state())
        if self.history:
            last = self.history[-1]
            self.history[-1] = self.snapshot(
                last.left_thrust,
                last.right_thrust,
                last.tension,
                last.tangential_force,
                last.spool_velocity_cmd,
                last.drone_accel_cmd,
                last.desired_cable_tension,
                last.desired_drone_force,
                last.drone_force,
                last.cable_force,
                last.saturated,
                self.trajectory.state(),
                desired_tangential_force=last.desired_tangential_force,
            )

    def _cable_mount_offset(self, attitude: float) -> Vec2:
        return rotate2((0.0, self.params.payload_hex_radius), attitude)

    def _cable_mount_position(self, payload: Vec2, attitude: float) -> Vec2:
        return add2(payload, self._cable_mount_offset(attitude))

    def _module_center_offsets(self, attitude: float) -> tuple[Vec2, Vec2]:
        radius = self.params.cage_radius
        left_payload_offset = rotate2((-self.params.payload_half_length, 0.0), attitude)
        left_drone_offset = projected_face_offset(radius, DRONE_RIGHT_HEX, attitude)
        right_payload_offset = rotate2((self.params.payload_half_length, 0.0), attitude)
        right_drone_offset = projected_face_offset(radius, DRONE_LEFT_HEX, attitude)

        left_gap = rotate2((-self.params.module_gap, 0.0), attitude)
        right_gap = rotate2((self.params.module_gap, 0.0), attitude)
        left_center_offset = sub2(add2(left_payload_offset, left_gap), left_drone_offset)
        right_center_offset = sub2(add2(right_payload_offset, right_gap), right_drone_offset)
        return left_center_offset, right_center_offset

    def _drone_axes(self, attitude: float) -> tuple[Vec2, Vec2]:
        attitude_error = attitude - self.params.nominal_attitude_rad
        left_axis = rotate2((math.sin(self.params.hex_face_tilt_rad), math.cos(self.params.hex_face_tilt_rad)), attitude_error)
        right_axis = rotate2((-math.sin(self.params.hex_face_tilt_rad), math.cos(self.params.hex_face_tilt_rad)), attitude_error)
        return left_axis, right_axis

    def _point_to_polar(self, point: Vec2) -> tuple[float, float]:
        attach_point = self._cable_mount_position(point, self.params.nominal_attitude_rad)
        dx = attach_point[0] - self.params.anchor[0]
        dz_down = self.params.anchor[1] - attach_point[1]
        length = clamp(math.hypot(dx, dz_down), self.params.min_cable_length, self.params.max_cable_length)
        theta = math.atan2(dx, dz_down)
        return theta, length

    def _reference_to_polar(self, reference: ReferenceState) -> tuple[float, float, float, float]:
        theta, length = self._point_to_polar(reference.position)
        e_r = (math.sin(theta), -math.cos(theta))
        e_theta = (math.cos(theta), math.sin(theta))
        length_dot = dot2(reference.velocity, e_r)
        theta_dot = dot2(reference.velocity, e_theta) / max(length, 1e-6)
        return theta, length, theta_dot, length_dot

    def _payload_from_state(self) -> Vec2:
        return self.position

    def _payload_velocity_from_state(self) -> Vec2:
        return self.velocity

    def _cable_damping_force(self, stretch_rate: float) -> float:
        return clamp(
            self.params.cable_damping * stretch_rate,
            -self.params.max_cable_damping_force,
            self.params.max_cable_damping_force,
        )

    def _update_sensor_estimate(self) -> None:
        params = self.params
        self.measured_theta = self.theta
        self.measured_theta_dot = self.theta_dot
        self.measured_cable_length = self.cable_length
        self.measured_cable_velocity = self.last_spool_velocity_cmd
        self.measured_tension = clamp(self.actual_tension, 0.0, params.max_spool_tension)
        self.measured_attitude = self.attitude
        self.measured_angular_velocity = self.angular_velocity

        extension_rate_estimate = self.length_dot - self.last_spool_velocity_cmd
        damping_force_estimate = self._cable_damping_force(extension_rate_estimate)
        stretch_estimate = max(
            0.0,
            (self.measured_tension - damping_force_estimate)
            / max(params.cable_stiffness, 1e-9),
        )
        self.measured_line_length = clamp(
            self.measured_cable_length + stretch_estimate,
            params.min_cable_length,
            params.max_cable_length + stretch_estimate,
        )
        cable_out = (math.sin(self.measured_theta), -math.cos(self.measured_theta))
        tangential_axis = (math.cos(self.measured_theta), math.sin(self.measured_theta))
        attach_position = add2(params.anchor, scale2(cable_out, self.measured_line_length))
        attach_velocity = add2(
            scale2(cable_out, self.length_dot),
            scale2(tangential_axis, self.measured_line_length * self.measured_theta_dot),
        )
        mount_offset = self._cable_mount_offset(self.measured_attitude)
        mount_velocity = scale2((-mount_offset[1], mount_offset[0]), self.measured_angular_velocity)
        self.measured_payload = sub2(attach_position, mount_offset)
        self.estimated_payload_velocity = sub2(attach_velocity, mount_velocity)

    def _static_cable_tension_target(self, point: Vec2) -> float:
        params = self.params
        attach_point = self._cable_mount_position(point, params.nominal_attitude_rad)
        cable_axis = normalize2((params.anchor[0] - attach_point[0], params.anchor[1] - attach_point[1]))
        left_axis, right_axis = self._drone_axes(params.nominal_attitude_rad)
        required_force = (0.0, params.total_mass * params.gravity)
        desired_cable_tension, _left, _right, _residual = self._allocate_support_force(
            required_force,
            cable_axis,
            left_axis,
            right_axis,
            params,
        )
        return desired_cable_tension

    def _update_reference_speed_scale(self) -> None:
        params = self.params
        if not self.history or not self.trajectory.segments:
            target_scale = 1.0
        else:
            last = self.history[-1]
            weight = params.total_mass * params.gravity
            thrust_fraction = max(last.left_thrust, last.right_thrust) / max(params.max_thrust_per_drone, 1e-9)
            thrust_risk = clamp(
                (thrust_fraction - params.thrust_slowdown_fraction)
                / max(1e-6, 1.0 - params.thrust_slowdown_fraction),
                0.0,
                1.0,
            )
            residual_risk = clamp(
                last.allocation_residual / max(weight * params.residual_slowdown_fraction, 1e-9),
                0.0,
                1.0,
            )
            error_risk = clamp(
                (last.measured_tool_error - params.tracking_error_slowdown_m)
                / max(1e-6, params.tracking_error_full_slow_m - params.tracking_error_slowdown_m),
                0.0,
                1.0,
            )
            cable_axis = (-math.sin(self.measured_theta), math.cos(self.measured_theta))
            vertical_efficiency = max(0.0, cable_axis[1])
            geometry_risk = clamp(
                (params.geometry_slowdown_efficiency - vertical_efficiency)
                / max(params.geometry_slowdown_efficiency, 1e-6),
                0.0,
                1.0,
            )
            risk = max(thrust_risk, residual_risk, error_risk, 0.75 * geometry_risk)
            target_scale = 1.0 - (1.0 - params.reference_speed_min) * risk

        rate = params.reference_slowdown_rate if target_scale < self.reference_speed_scale else params.reference_recovery_rate
        alpha = 1.0 - math.exp(-rate * params.dt)
        self.reference_speed_scale += alpha * (target_scale - self.reference_speed_scale)
        self.reference_speed_scale = clamp(self.reference_speed_scale, params.reference_speed_min, 1.0)

    def _safe_reference(self, reference: ReferenceState) -> ReferenceState:
        params = self.params
        point = self._clamp_wall_point(reference.position)
        anchor_to_point = sub2(self._cable_mount_position(point, params.nominal_attitude_rad), params.anchor)
        distance = math.hypot(anchor_to_point[0], anchor_to_point[1])
        clamped = distance2(point, reference.position) > 1e-8
        if distance < params.min_control_cable_length:
            if distance < 1e-9:
                direction = (0.0, -1.0)
            else:
                direction = (anchor_to_point[0] / distance, anchor_to_point[1] / distance)
            mount_point = add2(params.anchor, scale2(direction, params.min_control_cable_length))
            point = sub2(mount_point, self._cable_mount_offset(params.nominal_attitude_rad))
            point = self._clamp_wall_point(point)
            clamped = True
        if not clamped:
            return reference
        return ReferenceState(
            position=point,
            velocity=(0.0, 0.0),
            acceleration=(0.0, 0.0),
            final_target=reference.final_target,
            active=reference.active,
            waypoint_count=reference.waypoint_count,
        )

    def _filter_cable_tension_target(self, raw_target: float) -> float:
        params = self.params
        alpha = params.dt / max(params.dt, params.tension_filter_tau + params.dt)
        lowpass_target = self.filtered_cable_tension_target + alpha * (raw_target - self.filtered_cable_tension_target)
        self.filtered_cable_tension_target += clamp(
            lowpass_target - self.filtered_cable_tension_target,
            -params.max_tension_target_rate * params.dt,
            params.max_tension_target_rate * params.dt,
        )
        self.filtered_cable_tension_target = clamp(
            self.filtered_cable_tension_target,
            0.0,
            params.max_spool_tension,
        )
        return self.filtered_cable_tension_target

    def _hold_feasible_cable_tension(self, reference: ReferenceState, requested_tension: float) -> float:
        params = self.params
        lower = params.min_tracking_tension
        upper = clamp(requested_tension, lower, params.max_spool_tension)
        if upper <= lower:
            return lower

        attitude = params.nominal_attitude_rad
        mount = self._cable_mount_position(reference.position, attitude)
        cable_axis = normalize2((params.anchor[0] - mount[0], params.anchor[1] - mount[1]))
        left_axis, right_axis = self._drone_axes(attitude)
        cable_arm = self._cable_mount_offset(attitude)
        left_arm, right_arm = self._module_center_offsets(attitude)
        torque_scale = max(params.torque_residual_length_scale, 1e-6)
        residual_limit = params.hold_tension_residual_fraction * params.total_mass * params.gravity

        best = lower
        for index in range(params.hold_tension_search_steps + 1):
            tension = lower + (upper - lower) * index / max(1, params.hold_tension_search_steps)
            required_force = (
                -tension * cable_axis[0],
                params.total_mass * params.gravity - tension * cable_axis[1],
            )
            required_torque = -tension * cross2(cable_arm, cable_axis)
            values, residual = self._solve_bounded_allocation(
                required=(required_force[0], required_force[1], required_torque / torque_scale),
                axes=(
                    (left_axis[0], left_axis[1], cross2(left_arm, left_axis) / torque_scale),
                    (right_axis[0], right_axis[1], cross2(right_arm, right_axis) / torque_scale),
                ),
                upper_bounds=(params.max_thrust_per_drone, params.max_thrust_per_drone),
                effort_costs=(params.drone_thrust_cost, params.drone_thrust_cost),
            )
            if residual <= residual_limit and max(values) <= 0.96 * params.max_thrust_per_drone:
                best = tension
        return best

    def _update_cable_coordinates(self) -> None:
        mount_offset = self._cable_mount_offset(self.attitude)
        mount_position = add2(self.position, mount_offset)
        mount_velocity = add2(self.velocity, scale2((-mount_offset[1], mount_offset[0]), self.angular_velocity))
        anchor_to_mount = sub2(mount_position, self.params.anchor)
        distance = max(1e-9, math.hypot(anchor_to_mount[0], anchor_to_mount[1]))
        e_out = (anchor_to_mount[0] / distance, anchor_to_mount[1] / distance)
        e_theta = (-e_out[1], e_out[0])
        self.length = distance
        self.theta = math.atan2(mount_position[0] - self.params.anchor[0], self.params.anchor[1] - mount_position[1])
        self.length_dot = dot2(mount_velocity, e_out)
        self.theta_dot = dot2(mount_velocity, e_theta) / distance
        self.length_ddot = dot2(self.acceleration, e_out)

    def step(self) -> SimState:
        params = self.params
        mass = params.total_mass
        self._update_cable_coordinates()
        self._update_sensor_estimate()
        self._update_reference_speed_scale()
        reference = self._safe_reference(self.trajectory.advance(params.dt * self.reference_speed_scale))
        _target_theta, target_length, _target_theta_dot, target_length_dot = self._reference_to_polar(reference)

        control_cable_out = (math.sin(self.measured_theta), -math.cos(self.measured_theta))
        control_cable_axis = (-control_cable_out[0], -control_cable_out[1])
        control_tangential_axis = (math.cos(self.measured_theta), math.sin(self.measured_theta))

        gravity_force = (0.0, -mass * params.gravity)
        left_axis, right_axis = self._drone_axes(self.measured_attitude)
        cable_arm = self._cable_mount_offset(self.measured_attitude)
        left_arm, right_arm = self._module_center_offsets(self.measured_attitude)
        measured_position_error = distance2(self.measured_payload, reference.position)
        measured_speed = math.hypot(self.estimated_payload_velocity[0], self.estimated_payload_velocity[1])
        hold_arrived = (
            not reference.active
            and measured_position_error < params.attitude_hold_error_m
            and measured_speed < params.attitude_hold_speed_mps
        )
        hold_release = (
            reference.active
            or measured_position_error > params.attitude_hold_release_error_m
            or measured_speed > params.attitude_hold_release_speed_mps
        )
        if hold_arrived:
            self.hold_latched = True
        elif hold_release:
            self.hold_latched = False
        hold_attitude_ready = self.hold_latched and not reference.active

        line_length = max(self.measured_line_length, params.min_cable_length)
        line_velocity = self.length_dot
        length_error = target_length - line_length
        length_dot_error = target_length_dot - line_velocity
        radial_length_accel_cmd = clamp(
            dot2(reference.acceleration, control_cable_out)
            + params.radial_accel_kp * length_error
            + params.radial_accel_kd * length_dot_error,
            -params.max_radial_accel,
            params.max_radial_accel,
        )

        target_theta, _target_length, target_theta_dot, _target_length_dot = self._reference_to_polar(reference)
        theta_error = wrap_angle(target_theta - self.measured_theta)
        theta_dot_error = target_theta_dot - self.measured_theta_dot
        theta_ddot_cmd = clamp(
            params.pendulum_theta_kp * theta_error + params.pendulum_theta_kd * theta_dot_error,
            -params.max_pendulum_theta_ddot,
            params.max_pendulum_theta_ddot,
        )
        tangential_accel_cmd = clamp(
            line_length * theta_ddot_cmd + 2.0 * line_velocity * self.measured_theta_dot,
            -params.max_tangential_accel,
            params.max_tangential_accel,
        )
        gravity_tangential_force = dot2(gravity_force, control_tangential_axis)
        desired_tangential_force = mass * tangential_accel_cmd - gravity_tangential_force

        gravity_radial_force = dot2(gravity_force, control_cable_out)
        radial_accel_component_cmd = radial_length_accel_cmd - line_length * self.measured_theta_dot * self.measured_theta_dot
        radial_tension_target = gravity_radial_force - mass * radial_accel_component_cmd
        vertical_efficiency = control_cable_axis[1]
        radial_in_request = (
            target_length_dot < -params.radial_motion_deadband
            or length_error < -params.radial_length_deadband
        )
        radial_out_request = (
            target_length_dot > params.radial_motion_deadband
            or length_error > params.radial_length_deadband
        )
        if hold_attitude_ready:
            support_fraction = params.hold_cable_support_fraction
        elif radial_in_request:
            support_fraction = params.lift_cable_support_fraction
        elif radial_out_request:
            support_fraction = params.lower_cable_support_fraction
        else:
            support_fraction = params.cable_support_floor_fraction
        if vertical_efficiency > params.min_cable_vertical_efficiency:
            support_tension_floor = support_fraction * mass * params.gravity / vertical_efficiency
        else:
            support_tension_floor = params.min_tracking_tension
        if radial_out_request and not radial_in_request and not hold_attitude_ready:
            tension_request = max(support_tension_floor, params.min_tracking_tension)
        else:
            tension_request = max(radial_tension_target, support_tension_floor, params.min_tracking_tension)
        raw_desired_cable_tension = clamp(
            tension_request,
            params.min_tracking_tension,
            params.max_spool_tension,
        )
        if hold_attitude_ready:
            raw_desired_cable_tension = min(
                raw_desired_cable_tension,
                self._hold_feasible_cable_tension(reference, raw_desired_cable_tension),
            )
        if hold_attitude_ready:
            attitude_kp = params.hold_attitude_kp
            attitude_kd = params.hold_attitude_kd
            max_attitude_torque = params.hold_max_attitude_torque
        else:
            attitude_kp = params.move_attitude_kp
            attitude_kd = params.move_attitude_kd
            max_attitude_torque = params.move_max_attitude_torque
        desired_attitude_torque = clamp(
            params.assembly_inertia
            * (
                -attitude_kp * wrap_angle(self.measured_attitude - params.nominal_attitude_rad)
                - attitude_kd * self.measured_angular_velocity
            ),
            -max_attitude_torque,
            max_attitude_torque,
        )
        desired_cable_tension = self._filter_cable_tension_target(
            max(raw_desired_cable_tension, params.min_tracking_tension)
        )
        desired_radial_drone_force = 0.0
        if hold_attitude_ready:
            hold_position_error = sub2(reference.position, self.measured_payload)
            hold_accel_cmd = limit_norm2(
                add2(
                    scale2(hold_position_error, params.hold_position_kp),
                    scale2(self.estimated_payload_velocity, -params.hold_position_kd),
                ),
                params.hold_max_position_accel,
            )
            hold_drone_force = sub2(
                add2((0.0, mass * params.gravity), scale2(hold_accel_cmd, mass)),
                scale2(control_cable_axis, desired_cable_tension),
            )
            desired_tangential_force = dot2(hold_drone_force, control_tangential_axis)
            desired_radial_drone_force = dot2(hold_drone_force, control_cable_out)
        elif radial_out_request and length_error > 0.0:
            desired_radial_drone_force = max(
                0.0,
                mass * radial_accel_component_cmd - gravity_radial_force + desired_cable_tension,
            )
        desired_drone_force = add2(
            scale2(control_tangential_axis, desired_tangential_force),
            scale2(control_cable_out, desired_radial_drone_force),
        )

        target_encoder_length = clamp(
            target_length - desired_cable_tension / max(params.cable_stiffness, 1e-9),
            params.min_cable_length,
            params.max_cable_length,
        )
        line_length_error = target_length - self.measured_line_length
        encoder_length_error = target_encoder_length - self.measured_cable_length
        cable_velocity_error = target_length_dot - self.measured_cable_velocity
        tension_velocity_feedback = -params.spool_tension_kv * (desired_cable_tension - self.measured_tension)
        if line_length_error > params.radial_length_deadband and tension_velocity_feedback < 0.0:
            tension_velocity_feedback = 0.0
        spool_velocity_cmd = clamp(
            target_length_dot
            + params.spool_velocity_kp * line_length_error
            + params.spool_velocity_kd * cable_velocity_error
            + params.spool_encoder_kp * encoder_length_error
            + params.spool_radial_accel_ff * radial_length_accel_cmd
            + tension_velocity_feedback,
            -params.max_spool_speed,
            params.max_spool_speed,
        )
        tension_fraction = self.measured_tension / max(desired_cable_tension, 1e-9)
        if self.measured_tension < params.min_tracking_tension:
            if line_length_error <= 0.0:
                slack_fraction = 1.0 - self.measured_tension / max(params.min_tracking_tension, 1e-9)
                spool_velocity_cmd = min(spool_velocity_cmd, -params.slack_recovery_spool_speed * slack_fraction)
            else:
                spool_velocity_cmd = clamp(spool_velocity_cmd, 0.0, params.slack_pay_out_speed)
        elif tension_fraction < 0.85 and spool_velocity_cmd > 0.0:
            if radial_out_request and line_length_error > 0.0:
                payout_limit = clamp(
                    params.gravity_lowering_payout_speed * line_length_error / max(0.25, line_length),
                    params.slack_pay_out_speed,
                    params.gravity_lowering_payout_speed,
                )
            else:
                payout_limit = params.slack_pay_out_speed * clamp((tension_fraction - 0.45) / 0.40, 0.0, 1.0)
            spool_velocity_cmd = min(spool_velocity_cmd, payout_limit)
        spool_velocity_cmd = clamp(spool_velocity_cmd, -params.max_spool_speed, params.max_spool_speed)
        previous_cable_length = self.cable_length
        self.cable_length = clamp(
            self.cable_length + spool_velocity_cmd * params.dt,
            params.min_cable_length,
            params.max_cable_length,
        )
        if self.cable_length in (params.min_cable_length, params.max_cable_length):
            spool_velocity_cmd = (self.cable_length - previous_cable_length) / params.dt

        true_cable_arm = self._cable_mount_offset(self.attitude)
        true_mount_position = add2(self.position, true_cable_arm)
        true_mount_velocity = add2(self.velocity, scale2((-true_cable_arm[1], true_cable_arm[0]), self.angular_velocity))
        anchor_to_true_mount = sub2(true_mount_position, params.anchor)
        true_distance = max(1e-9, math.hypot(anchor_to_true_mount[0], anchor_to_true_mount[1]))
        true_cable_out = (anchor_to_true_mount[0] / true_distance, anchor_to_true_mount[1] / true_distance)
        true_cable_axis = (-true_cable_out[0], -true_cable_out[1])
        true_radial_speed = dot2(true_mount_velocity, true_cable_out)

        taut_clamped = False
        taut_length_limit = true_distance - params.min_tracking_tension / max(params.cable_stiffness, 1e-9)
        if self.cable_length > taut_length_limit:
            self.cable_length = clamp(taut_length_limit, params.min_cable_length, params.max_cable_length)
            spool_velocity_cmd = (self.cable_length - previous_cable_length) / max(params.dt, 1e-9)
            taut_clamped = True
        self.last_spool_velocity_cmd = spool_velocity_cmd

        self.cable_stretch = true_distance - self.cable_length
        extension_rate = 0.0 if taut_clamped else true_radial_speed - spool_velocity_cmd
        raw_tension = 0.0
        if self.cable_stretch >= -params.cable_taut_band:
            spring_tension = params.cable_stiffness * max(0.0, self.cable_stretch)
            damping_tension = self._cable_damping_force(extension_rate)
            raw_tension = max(
                params.min_tracking_tension,
                spring_tension + damping_tension,
            )
        self.cable_tension_saturated = raw_tension > params.max_spool_tension
        tension = clamp(raw_tension, 0.0, params.max_spool_tension)
        self.actual_tension = tension
        self.cable_slack = tension <= 1e-9 and self.cable_stretch < -params.cable_taut_band
        cable_force = scale2(true_cable_axis, tension)

        drone_accel_cmd = abs(tangential_accel_cmd)
        measured_cable_force = scale2(control_cable_axis, self.measured_tension)
        measured_cable_torque = cross2(cable_arm, measured_cable_force)
        if hold_attitude_ready:
            desired_drone_torque = desired_attitude_torque - measured_cable_torque
        else:
            desired_drone_torque = desired_attitude_torque - params.move_cable_torque_comp_fraction * measured_cable_torque
        torque_scale_value = params.torque_residual_length_scale
        if hold_attitude_ready:
            shallow_ratio = clamp(
                (params.shallow_hold_torque_scale_efficiency - max(0.0, vertical_efficiency))
                / max(params.shallow_hold_torque_scale_efficiency - params.min_cable_vertical_efficiency, 1e-6),
                0.0,
                1.0,
            )
            torque_scale_value += shallow_ratio * (
                params.shallow_hold_torque_residual_length_scale - params.torque_residual_length_scale
            )
        torque_scale = max(torque_scale_value, 1e-6)
        radial_force_scale = params.pendulum_radial_force_penalty
        drone_values, allocation_residual = self._solve_bounded_allocation(
            required=(
                desired_tangential_force,
                desired_drone_torque / torque_scale,
                radial_force_scale * desired_radial_drone_force,
            ),
            axes=(
                (
                    dot2(left_axis, control_tangential_axis),
                    cross2(left_arm, left_axis) / torque_scale,
                    radial_force_scale * dot2(left_axis, control_cable_out),
                ),
                (
                    dot2(right_axis, control_tangential_axis),
                    cross2(right_arm, right_axis) / torque_scale,
                    radial_force_scale * dot2(right_axis, control_cable_out),
                ),
            ),
            upper_bounds=(params.max_thrust_per_drone, params.max_thrust_per_drone),
            effort_costs=(params.drone_thrust_cost, params.drone_thrust_cost),
        )
        left_thrust, right_thrust = drone_values
        true_left_axis, true_right_axis = self._drone_axes(self.attitude)
        true_left_arm, true_right_arm = self._module_center_offsets(self.attitude)
        left_force = scale2(true_left_axis, left_thrust)
        right_force = scale2(true_right_axis, right_thrust)
        drone_force = add2(left_force, right_force)
        cable_torque = cross2(true_cable_arm, cable_force)
        left_torque = cross2(true_left_arm, left_force)
        right_torque = cross2(true_right_arm, right_force)
        net_attitude_torque = cable_torque + left_torque + right_torque - params.rotational_damping * self.angular_velocity
        e_theta = (math.cos(self.theta), math.sin(self.theta))
        tangential_force = dot2(drone_force, e_theta)
        saturated = allocation_residual > 0.05
        net_force = add2(add2(drone_force, cable_force), gravity_force)
        self.acceleration = scale2(net_force, 1.0 / mass)
        self.angular_acceleration = net_attitude_torque / max(params.assembly_inertia, 1e-9)
        self.velocity = add2(self.velocity, scale2(self.acceleration, params.dt))
        self.position = add2(self.position, scale2(self.velocity, params.dt))
        self.angular_velocity += self.angular_acceleration * params.dt
        self.attitude += self.angular_velocity * params.dt
        self.t += params.dt
        self._update_cable_coordinates()
        self._update_sensor_estimate()
        state = self.snapshot(
            left_thrust,
            right_thrust,
            tension,
            tangential_force,
            spool_velocity_cmd,
            drone_accel_cmd,
            desired_cable_tension,
            desired_drone_force,
            drone_force,
            cable_force,
            saturated,
            reference,
            desired_tangential_force=desired_tangential_force,
            desired_attitude_torque=desired_attitude_torque,
            attitude_torque=net_attitude_torque,
            cable_torque=cable_torque,
            left_torque=left_torque,
            right_torque=right_torque,
            allocation_residual=allocation_residual,
        )
        self.history.append(state)
        if len(self.history) > 6000:
            self.history = self.history[-6000:]
        return state

    @staticmethod
    def _allocate_support_force(
        required_force: Vec2,
        cable_axis: Vec2,
        left_axis: Vec2,
        right_axis: Vec2,
        params: SimParams,
    ) -> tuple[float, float, float, float]:
        """Bounded weighted allocation over cable tension and two drone thrusts."""

        vertical_efficiency = max(0.0, cable_axis[1])
        efficiency_floor = max(1e-3, params.min_cable_vertical_efficiency)
        geometry_ratio = (1.0 - vertical_efficiency) / max(vertical_efficiency, efficiency_floor)
        cable_cost = params.cable_tension_cost * (1.0 + params.cable_geometry_cost * geometry_ratio * geometry_ratio)
        values, residual = WallToolSimulator._solve_bounded_force_allocation(
            required_force=required_force,
            axes=(cable_axis, left_axis, right_axis),
            upper_bounds=(params.max_spool_tension, params.max_thrust_per_drone, params.max_thrust_per_drone),
            effort_costs=(cable_cost, params.drone_thrust_cost, params.drone_thrust_cost),
        )
        return values[0], values[1], values[2], residual

    @staticmethod
    def _allocate_support_wrench(
        required_force: Vec2,
        required_torque: float,
        cable_axis: Vec2,
        left_axis: Vec2,
        right_axis: Vec2,
        cable_arm: Vec2,
        left_arm: Vec2,
        right_arm: Vec2,
        params: SimParams,
    ) -> tuple[float, float, float, float]:
        vertical_efficiency = max(0.0, cable_axis[1])
        efficiency_floor = max(1e-3, params.min_cable_vertical_efficiency)
        geometry_ratio = (1.0 - vertical_efficiency) / max(vertical_efficiency, efficiency_floor)
        cable_cost = params.cable_tension_cost * (1.0 + params.cable_geometry_cost * geometry_ratio * geometry_ratio)
        torque_scale = max(params.torque_residual_length_scale, 1e-6)
        required_wrench = (required_force[0], required_force[1], required_torque / torque_scale)
        values, residual = WallToolSimulator._solve_bounded_allocation(
            required=required_wrench,
            axes=(
                (cable_axis[0], cable_axis[1], cross2(cable_arm, cable_axis) / torque_scale),
                (left_axis[0], left_axis[1], cross2(left_arm, left_axis) / torque_scale),
                (right_axis[0], right_axis[1], cross2(right_arm, right_axis) / torque_scale),
            ),
            upper_bounds=(params.max_spool_tension, params.max_thrust_per_drone, params.max_thrust_per_drone),
            effort_costs=(cable_cost, params.drone_thrust_cost, params.drone_thrust_cost),
        )
        return values[0], values[1], values[2], residual

    @staticmethod
    def _solve_bounded_force_allocation(
        required_force: Vec2,
        axes: Sequence[Vec2],
        upper_bounds: Sequence[float],
        effort_costs: Sequence[float],
    ) -> tuple[tuple[float, ...], float]:
        return WallToolSimulator._solve_bounded_allocation(required_force, axes, upper_bounds, effort_costs)

    @staticmethod
    def _solve_bounded_allocation(
        required: Sequence[float],
        axes: Sequence[Sequence[float]],
        upper_bounds: Sequence[float],
        effort_costs: Sequence[float],
    ) -> tuple[tuple[float, ...], float]:
        """Active-set solve: least force residual first, minimum effort second."""

        count = len(axes)
        dimension = len(required)
        best_values: tuple[float, ...] | None = None
        best_residual_squared = math.inf
        best_residual = math.inf
        best_effort = math.inf
        states = ("free", "low", "high")

        for active_state in itertools.product(states, repeat=count):
            values = [0.0] * count
            free_indices: list[int] = []
            for index, state in enumerate(active_state):
                if state == "free":
                    free_indices.append(index)
                elif state == "high":
                    values[index] = float(upper_bounds[index])

            fixed_force = tuple(0.0 for _ in range(dimension))
            for index, value in enumerate(values):
                if index not in free_indices:
                    fixed_force = addn(fixed_force, scalen(axes[index], value))
            remaining_force = subn(required, fixed_force)

            if free_indices:
                try:
                    if len(free_indices) <= dimension:
                        normal_matrix = tuple(
                            tuple(
                                sum(axes[row_index][dim] * axes[col_index][dim] for dim in range(dimension))
                                + (1e-10 if row_index == col_index else 0.0)
                                for col_index in free_indices
                            )
                            for row_index in free_indices
                        )
                        normal_rhs = tuple(dotn(axes[index], remaining_force) for index in free_indices)
                        free_values = solve_linear_system(normal_matrix, normal_rhs)
                    else:
                        inverse_costs = [1.0 / max(1e-12, effort_costs[index]) for index in free_indices]
                        weighted_axis_matrix = tuple(
                            tuple(
                                sum(
                                    inverse_costs[local] * axes[index][row] * axes[index][col]
                                    for local, index in enumerate(free_indices)
                                )
                                + (1e-10 if row == col else 0.0)
                                for col in range(dimension)
                            )
                            for row in range(dimension)
                        )
                        multipliers = solve_linear_system(weighted_axis_matrix, remaining_force)
                        free_values = tuple(
                            inverse_costs[local] * dotn(axes[index], multipliers)
                            for local, index in enumerate(free_indices)
                        )
                except ValueError:
                    continue
                feasible = True
                for local_index, value in enumerate(free_values):
                    actuator_index = free_indices[local_index]
                    if value < -1e-8 or value > upper_bounds[actuator_index] + 1e-8:
                        feasible = False
                        break
                    values[actuator_index] = clamp(value, 0.0, upper_bounds[actuator_index])
                if not feasible:
                    continue

            produced_force = tuple(0.0 for _ in range(dimension))
            effort_objective = 0.0
            for index, value in enumerate(values):
                produced_force = addn(produced_force, scalen(axes[index], value))
                effort_objective += effort_costs[index] * value * value
            force_error = subn(produced_force, required)
            residual_squared = dotn(force_error, force_error)
            if (
                residual_squared < best_residual_squared - 1e-12
                or abs(residual_squared - best_residual_squared) <= 1e-12
                and effort_objective < best_effort
            ):
                best_residual_squared = residual_squared
                best_values = tuple(values)
                best_residual = math.sqrt(residual_squared)
                best_effort = effort_objective

        if best_values is None:
            raise RuntimeError("bounded force allocation has no feasible active set")
        return best_values, best_residual

    def snapshot(
        self,
        left_thrust: float,
        right_thrust: float,
        tension: float,
        tangential_force: float,
        spool_velocity_cmd: float,
        drone_accel_cmd: float,
        desired_cable_tension: float,
        desired_drone_force: Vec2,
        drone_force: Vec2,
        cable_force: Vec2,
        saturated: bool,
        reference: ReferenceState,
        desired_tangential_force: float = 0.0,
        desired_attitude_torque: float = 0.0,
        attitude_torque: float = 0.0,
        cable_torque: float = 0.0,
        left_torque: float = 0.0,
        right_torque: float = 0.0,
        allocation_residual: float | None = None,
    ) -> SimState:
        payload = self._payload_from_state()
        tool_head = payload
        desired_tool_head = reference.position
        return SimState(
            t=self.t,
            theta=self.theta,
            theta_dot=self.theta_dot,
            length=self.length,
            length_dot=self.length_dot,
            length_ddot=self.length_ddot,
            attitude=self.attitude,
            angular_velocity=self.angular_velocity,
            angular_acceleration=self.angular_acceleration,
            cable_length=self.cable_length,
            cable_stretch=self.cable_stretch,
            cable_slack=self.cable_slack,
            cable_tension_saturated=self.cable_tension_saturated,
            payload_velocity=self.velocity,
            payload_acceleration=self.acceleration,
            payload=payload,
            measured_payload=self.measured_payload,
            estimated_payload_velocity=self.estimated_payload_velocity,
            measured_theta=self.measured_theta,
            measured_theta_dot=self.measured_theta_dot,
            measured_line_length=self.measured_line_length,
            measured_attitude=self.measured_attitude,
            measured_angular_velocity=self.measured_angular_velocity,
            measured_cable_velocity=self.measured_cable_velocity,
            tool_head=tool_head,
            reference=reference.position,
            desired_tool_head=desired_tool_head,
            reference_velocity=reference.velocity,
            reference_acceleration=reference.acceleration,
            target=reference.final_target,
            measured_tool_error=distance2(self.measured_payload, desired_tool_head),
            spool_velocity_cmd=spool_velocity_cmd,
            drone_accel_cmd=drone_accel_cmd,
            reference_speed_scale=self.reference_speed_scale,
            desired_cable_tension=desired_cable_tension,
            measured_cable_length=self.measured_cable_length,
            measured_tension=self.measured_tension,
            desired_drone_force=desired_drone_force,
            drone_force=drone_force,
            cable_force=cable_force,
            desired_attitude_torque=desired_attitude_torque,
            attitude_torque=attitude_torque,
            cable_torque=cable_torque,
            left_torque=left_torque,
            right_torque=right_torque,
            left_thrust=left_thrust,
            right_thrust=right_thrust,
            tension=tension,
            tangential_force=tangential_force,
            desired_tangential_force=desired_tangential_force,
            allocation_residual=distance2(drone_force, desired_drone_force)
            if allocation_residual is None
            else allocation_residual,
            drone_vertical_force=max(0.0, drone_force[1]),
            cable_vertical_force=max(0.0, cable_force[1]),
            path_error=distance2(tool_head, desired_tool_head),
            tool_error=distance2(tool_head, desired_tool_head),
            active_waypoints=reference.waypoint_count,
            saturated=saturated,
        )


class ModuleArtist:
    def __init__(
        self,
        ax,
        radius: float,
        face_color: str,
        edge_color: str,
        label: str,
        fill_alpha: float,
        zorder: int,
    ) -> None:
        self.radius = radius
        self.silhouette = Polygon(
            [(0.0, 0.0)] * 6,
            closed=True,
            facecolor=face_color,
            edgecolor=edge_color,
            linewidth=1.6,
            alpha=1.0,
            zorder=zorder,
        )
        ax.add_patch(self.silhouette)
        self.label = ax.text(
            0.0,
            0.0,
            label,
            ha="center",
            va="center",
            fontsize=7,
            visible=bool(label),
            zorder=zorder + 3,
        )

    def update(self, center: Vec2, attitude: float = 0.0) -> None:
        points = visual_projected_vertices(center, self.radius, attitude)
        self.silhouette.set_xy(convex_hull(points))
        self.label.set_position(center)


class PayloadArtist:
    def __init__(self, ax, params: SimParams, zorder: int) -> None:
        self.params = params
        self.body = Polygon(
            [(0.0, 0.0)] * 4,
            closed=True,
            facecolor="#f2cc60",
            edgecolor="#5c4512",
            linewidth=1.4,
            alpha=1.0,
            zorder=zorder,
        )
        self.left_face = Polygon(
            [(0.0, 0.0)] * 6,
            closed=True,
            facecolor="#f7d978",
            edgecolor="#5c4512",
            linewidth=1.4,
            alpha=1.0,
            zorder=zorder + 1,
        )
        self.right_face = Polygon(
            [(0.0, 0.0)] * 6,
            closed=True,
            facecolor="#f7d978",
            edgecolor="#5c4512",
            linewidth=1.4,
            alpha=1.0,
            zorder=zorder + 1,
        )
        ax.add_patch(self.body)
        ax.add_patch(self.left_face)
        ax.add_patch(self.right_face)

    def update(self, center: Vec2, attitude: float = 0.0) -> None:
        self.body.set_xy(
            payload_body_polygon(center, self.params.payload_half_length, self.params.payload_hex_radius, attitude)
        )
        self.left_face.set_xy(
            payload_face_polygon(center, self.params.payload_half_length, self.params.payload_hex_radius, attitude, -1)
        )
        self.right_face.set_xy(
            payload_face_polygon(center, self.params.payload_half_length, self.params.payload_hex_radius, attitude, 1)
        )


class WallToolApp:
    def __init__(self, simulator: WallToolSimulator) -> None:
        self.sim = simulator
        self.params = simulator.params
        self.playing = True
        self.show_trace = True
        self.show_target = True
        self.show_path = True
        self.show_forces = True
        self.append_mode = False
        self.draw_mode = False
        self.is_drawing = False
        self.draw_points: list[Vec2] = []
        self.draw_min_spacing = 0.055
        self.draw_max_points = 28
        self.live_window_s = 12.0
        self._last_frame_wall_time = time.perf_counter()

        self.fig = plt.figure(figsize=(13.5, 8.6), constrained_layout=False)
        grid = self.fig.add_gridspec(
            2,
            2,
            width_ratios=[1.0, 0.42],
            height_ratios=[1.0, 0.18],
            left=0.055,
            right=0.975,
            bottom=0.08,
            top=0.92,
            wspace=0.08,
            hspace=0.18,
        )
        self.ax = self.fig.add_subplot(grid[0, 0])
        panel_grid = grid[0, 1].subgridspec(2, 1, height_ratios=[0.58, 0.42], hspace=0.30)
        self.panel_ax = self.fig.add_subplot(panel_grid[0])
        self.error_ax = self.fig.add_subplot(panel_grid[1])
        self.control_ax = self.fig.add_subplot(grid[1, :])
        self.control_ax.axis("off")
        self.panel_ax.axis("off")
        self.fig.suptitle("PRISMS Cable-Suspended Wall Tool Simulator", fontsize=14)

        self._build_scene()
        self._build_panel()
        self._build_live_plots()
        self._build_controls()
        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.draw()

    def _build_scene(self) -> None:
        params = self.params
        self.ax.set_aspect("equal", adjustable="box")
        margin = 0.35
        self.ax.set_xlim(-params.wall_width / 2.0 - margin, params.wall_width / 2.0 + margin)
        self.ax.set_ylim(-0.10, params.wall_height + 0.35)
        self.ax.set_xlabel("wall x [m]")
        self.ax.set_ylabel("wall z [m]")

        self.wall = Rectangle(
            (-params.wall_width / 2.0, 0.0),
            params.wall_width,
            params.wall_height,
            facecolor="#f3f1ea",
            edgecolor="#6d6a62",
            linewidth=2.0,
        )
        self.ax.add_patch(self.wall)
        self.ax.grid(True, color="#d8d4c9", linewidth=0.8)

        self.spool = Circle(params.anchor, 0.075, facecolor="#444444", edgecolor="black", zorder=5)
        self.ax.add_patch(self.spool)
        self.ax.text(params.anchor[0], params.anchor[1] + 0.13, "anchor + spool", ha="center", fontsize=9)

        self.cable_line, = self.ax.plot([], [], color="#222222", linewidth=2.0, zorder=3)
        self.trace_line, = self.ax.plot([], [], color="#2b7a78", linewidth=2.0, alpha=0.80, zorder=2)
        self.desired_trace_line, = self.ax.plot([], [], color="#8a5b22", linewidth=1.8, linestyle=":", alpha=0.90, zorder=2)
        self.path_line, = self.ax.plot([], [], color="#555555", linewidth=1.5, linestyle="--", alpha=0.72, zorder=4)
        self.draw_preview_line, = self.ax.plot([], [], color="#f39c12", linewidth=2.0, alpha=0.85, zorder=8)
        self.structure_line, = self.ax.plot([], [], color="#4a4a4a", linewidth=2.2, alpha=0.55, zorder=5)
        self.cable_mount_point, = self.ax.plot([], [], marker="o", linestyle="none", color="#111111", markersize=4.2, zorder=13)
        self.attitude_line, = self.ax.plot([], [], color="#111111", linewidth=1.6, alpha=0.85, zorder=13)
        self.reference_point, = self.ax.plot([], [], marker="o", color="#1f77b4", markersize=5.0, zorder=9)
        self.waypoint_points, = self.ax.plot([], [], marker="o", linestyle="none", color="#8a5b22", markersize=3.0, alpha=0.45, zorder=9)
        self.target_point, = self.ax.plot(
            [],
            [],
            marker="o",
            linestyle="none",
            markerfacecolor="none",
            markeredgecolor="#8a5b22",
            markersize=8.0,
            mew=1.8,
            zorder=9,
        )
        self.tool_line, = self.ax.plot([], [], marker="o", linestyle="none", color="#8a4f00", markersize=6.0, zorder=13)

        radius = params.cage_radius
        self.payload_artist = PayloadArtist(self.ax, params, 6)
        self.left_artist = ModuleArtist(self.ax, radius, "#f7f7f7", "black", "", 0.16, 6)
        self.right_artist = ModuleArtist(self.ax, radius, "#f7f7f7", "black", "", 0.16, 6)

        self.left_dock_seam, = self.ax.plot([], [], color="#111111", linewidth=3.0, solid_capstyle="round", zorder=14)
        self.right_dock_seam, = self.ax.plot([], [], color="#111111", linewidth=3.0, solid_capstyle="round", zorder=14)

        self.left_axis_guide, = self.ax.plot([], [], color="#777777", linestyle="--", linewidth=1.0, zorder=10)
        self.right_axis_guide, = self.ax.plot([], [], color="#777777", linestyle="--", linewidth=1.0, zorder=10)
        self.left_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=14, color="#1f77b4", zorder=12)
        self.right_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=14, color="#1f77b4", zorder=12)
        self.gravity_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=15, color="#333333", zorder=12)
        self.tension_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=14, color="#6a3d9a", zorder=12)
        for arrow in (self.left_arrow, self.right_arrow, self.gravity_arrow, self.tension_arrow):
            self.ax.add_patch(arrow)

    def _build_panel(self) -> None:
        self.panel_ax.text(0.0, 0.98, "Sensor Feedback", fontsize=12, fontweight="bold", va="top")
        self.state_text = self.panel_ax.text(
            0.0,
            0.90,
            "",
            fontsize=7.0,
            family="monospace",
            va="top",
            linespacing=1.05,
            clip_on=True,
        )

    def _build_live_plots(self) -> None:
        self.error_norm_line, = self.error_ax.plot([], [], color="#111111", linewidth=1.8, label="true norm")
        self.measured_error_line, = self.error_ax.plot([], [], color="#777777", linewidth=1.4, linestyle="--", label="sensor-est norm")
        self.error_x_line, = self.error_ax.plot([], [], color="#4b83c4", linewidth=1.1, label="true x")
        self.error_z_line, = self.error_ax.plot([], [], color="#c46b4b", linewidth=1.1, label="true z")
        self.error_ax.set_title("Trajectory Error", fontsize=9.5)
        self.error_ax.set_xlabel("time [s]", fontsize=8.0)
        self.error_ax.set_ylabel("error [m]", fontsize=8.0)
        self.error_ax.grid(True, color="#dddddd", linewidth=0.7)
        self.error_ax.legend(loc="upper right", fontsize=6.5, framealpha=0.90)
        self.error_ax.tick_params(axis="both", labelsize=7.5)

    def _build_controls(self) -> None:
        self.play_ax = self.fig.add_axes([0.055, 0.026, 0.080, 0.038])
        self.reset_ax = self.fig.add_axes([0.145, 0.026, 0.070, 0.038])
        self.clear_ax = self.fig.add_axes([0.225, 0.026, 0.070, 0.038])
        self.append_ax = self.fig.add_axes([0.305, 0.026, 0.105, 0.038])
        self.draw_ax = self.fig.add_axes([0.420, 0.026, 0.090, 0.038])
        self.speed_ax = self.fig.add_axes([0.565, 0.035, 0.175, 0.024])
        self.trace_ax = self.fig.add_axes([0.760, 0.052, 0.078, 0.034])
        self.target_ax = self.fig.add_axes([0.844, 0.052, 0.078, 0.034])
        self.path_ax = self.fig.add_axes([0.760, 0.012, 0.078, 0.034])
        self.forces_ax = self.fig.add_axes([0.844, 0.012, 0.078, 0.034])

        self.play_button = Button(self.play_ax, "Pause")
        self.reset_button = Button(self.reset_ax, "Reset")
        self.clear_button = Button(self.clear_ax, "Clear")
        self.append_button = Button(self.append_ax, "Append Off")
        self.draw_button = Button(self.draw_ax, "Draw Off")
        self.speed_slider = Slider(self.speed_ax, "speed", 0.25, 4.0, valinit=1.0)
        self.trace_button = Button(self.trace_ax, "Trace On")
        self.target_button = Button(self.target_ax, "Target On")
        self.path_button = Button(self.path_ax, "Path On")
        self.forces_button = Button(self.forces_ax, "Forces On")

        self.play_button.on_clicked(self.toggle_play)
        self.reset_button.on_clicked(self.reset)
        self.clear_button.on_clicked(self.clear_trace)
        self.append_button.on_clicked(self.toggle_append)
        self.draw_button.on_clicked(self.toggle_draw)
        self.trace_button.on_clicked(lambda _event: self.toggle_layer("trace"))
        self.target_button.on_clicked(lambda _event: self.toggle_layer("target"))
        self.path_button.on_clicked(lambda _event: self.toggle_layer("path"))
        self.forces_button.on_clicked(lambda _event: self.toggle_layer("forces"))

    def module_centers(self, payload: Vec2, attitude: float) -> tuple[Vec2, Vec2]:
        radius = self.params.cage_radius
        left_payload_offset = rotate2((-self.params.payload_half_length, 0.0), attitude)
        left_drone_offset = visual_projected_face_offset(radius, DRONE_RIGHT_HEX, attitude)
        right_payload_offset = rotate2((self.params.payload_half_length, 0.0), attitude)
        right_drone_offset = visual_projected_face_offset(radius, DRONE_LEFT_HEX, attitude)
        left_gap = rotate2((-self.params.module_gap, 0.0), attitude)
        right_gap = rotate2((self.params.module_gap, 0.0), attitude)
        left_offset = sub2(add2(left_payload_offset, left_gap), left_drone_offset)
        right_offset = sub2(add2(right_payload_offset, right_gap), right_drone_offset)
        left_center = add2(payload, left_offset)
        right_center = add2(payload, right_offset)
        return left_center, right_center

    def draw(self) -> None:
        state = self.sim.history[-1]
        params = self.params
        x, z = state.payload
        radius = params.cage_radius

        if self.show_trace:
            self.trace_line.set_data(
                [sample.tool_head[0] for sample in self.sim.history],
                [sample.tool_head[1] for sample in self.sim.history],
            )
            self.desired_trace_line.set_data(
                [sample.desired_tool_head[0] for sample in self.sim.history],
                [sample.desired_tool_head[1] for sample in self.sim.history],
            )
        else:
            self.trace_line.set_data([], [])
            self.desired_trace_line.set_data([], [])
        pending_path = self.sim.trajectory.pending_path()
        if self.show_path and len(pending_path) >= 1:
            self.path_line.set_data([point[0] for point in pending_path], [point[1] for point in pending_path])
            self.reference_point.set_data([state.reference[0]], [state.reference[1]])
            self.waypoint_points.set_data([], [])
        else:
            self.path_line.set_data([], [])
            self.reference_point.set_data([], [])
            self.waypoint_points.set_data([], [])
        if self.show_target:
            self.target_point.set_data([state.target[0]], [state.target[1]])
        else:
            self.target_point.set_data([], [])
        if self.draw_points:
            self.draw_preview_line.set_data([point[0] for point in self.draw_points], [point[1] for point in self.draw_points])
        else:
            self.draw_preview_line.set_data([], [])

        attitude = state.attitude
        left_center, right_center = self.module_centers(state.payload, attitude)
        self.payload_artist.update(state.payload, attitude)
        self.left_artist.update(left_center, attitude)
        self.right_artist.update(right_center, attitude)
        self.structure_line.set_data(
            [left_center[0], state.payload[0], right_center[0]],
            [left_center[1], state.payload[1], right_center[1]],
        )

        left_seam_center = payload_face_center(state.payload, params.payload_half_length, attitude, -1)
        right_seam_center = payload_face_center(state.payload, params.payload_half_length, attitude, 1)
        seam_half = params.payload_hex_radius
        left_seam_start = add2(left_seam_center, rotate2((0.0, -seam_half), attitude))
        left_seam_end = add2(left_seam_center, rotate2((0.0, seam_half), attitude))
        right_seam_start = add2(right_seam_center, rotate2((0.0, -seam_half), attitude))
        right_seam_end = add2(right_seam_center, rotate2((0.0, seam_half), attitude))
        self.left_dock_seam.set_data([left_seam_start[0], left_seam_end[0]], [left_seam_start[1], left_seam_end[1]])
        self.right_dock_seam.set_data([right_seam_start[0], right_seam_end[0]], [right_seam_start[1], right_seam_end[1]])

        cable_mount = add2(state.payload, rotate2((0.0, params.payload_hex_radius), attitude))
        self.cable_line.set_data([params.anchor[0], cable_mount[0]], [params.anchor[1], cable_mount[1]])
        self.cable_line.set_linestyle("--" if state.cable_slack else "-")
        self.cable_mount_point.set_data([cable_mount[0]], [cable_mount[1]])
        attitude_tip = add2(state.payload, rotate2((0.0, params.payload_hex_radius * 1.35), attitude))
        self.attitude_line.set_data([state.payload[0], attitude_tip[0]], [state.payload[1], attitude_tip[1]])
        self.tool_line.set_data([state.tool_head[0]], [state.tool_head[1]])

        left_axis, right_axis = self.sim._drone_axes(attitude)
        guide_length = 0.30
        self.left_axis_guide.set_data(
            [left_center[0], left_center[0] + left_axis[0] * guide_length],
            [left_center[1], left_center[1] + left_axis[1] * guide_length],
        )
        self.right_axis_guide.set_data(
            [right_center[0], right_center[0] + right_axis[0] * guide_length],
            [right_center[1], right_center[1] + right_axis[1] * guide_length],
        )

        force_scale = 0.25 / params.max_thrust_per_drone
        self._set_arrow(self.left_arrow, left_center, add2(left_center, scale2(left_axis, 0.035 + force_scale * state.left_thrust)))
        self._set_arrow(self.right_arrow, right_center, add2(right_center, scale2(right_axis, 0.035 + force_scale * state.right_thrust)))
        self._set_arrow(self.gravity_arrow, state.payload, (x, z - 0.20))
        cable_direction = normalize2((params.anchor[0] - x, params.anchor[1] - z))
        self._set_arrow(self.tension_arrow, state.payload, add2(state.payload, scale2(cable_direction, 0.18)))
        for arrow in (self.left_arrow, self.right_arrow, self.gravity_arrow, self.tension_arrow):
            arrow.set_visible(self.show_forces)
        self.left_axis_guide.set_visible(self.show_forces)
        self.right_axis_guide.set_visible(self.show_forces)

        efficiency_text = self._efficiency_text(state)
        self.state_text.set_text(
            f"t                         {state.t:6.2f} s\n\n"
            f"ANCHOR SENSOR\n"
            f"cable angle theta         {math.degrees(state.measured_theta):+7.2f} deg\n"
            f"cable angle rate          {math.degrees(state.measured_theta_dot):+7.2f} deg/s\n"
            f"cable tension             {state.measured_tension:7.3f} N\n\n"
            f"SPOOL ENCODER\n"
            f"payout length             {state.measured_cable_length:7.3f} m\n"
            f"payout velocity           {state.measured_cable_velocity:+7.3f} m/s\n\n"
            f"MANIPULATOR IMU\n"
            f"assembly tilt             {math.degrees(state.measured_attitude):+7.2f} deg\n"
            f"assembly tilt rate        {math.degrees(state.measured_angular_velocity):+7.2f} deg/s\n\n"
            f"{efficiency_text}"
        )
        self._update_live_plots()

    def _efficiency_text(self, state: SimState) -> str:
        params = self.params
        weight = max(params.total_mass * params.gravity, 1e-9)
        cable_vertical = max(0.0, state.measured_tension * math.cos(state.measured_theta))
        left_axis, right_axis = self.sim._drone_axes(state.measured_attitude)
        drone_force_est = add2(scale2(left_axis, state.left_thrust), scale2(right_axis, state.right_thrust))
        drone_vertical = max(0.0, drone_force_est[1])
        no_cable_hover_each = weight / max(2.0 * math.cos(params.hex_face_tilt_rad), 1e-9)
        no_cable_power_index = max(2.0 * no_cable_hover_each**1.5, 1e-9)
        drone_power_index = state.left_thrust**1.5 + state.right_thrust**1.5
        drone_power_ratio = drone_power_index / no_cable_power_index
        spool_power_abs = state.measured_tension * abs(state.measured_cable_velocity)
        return (
            "EFFICIENCY EST.\n"
            f"cable support            {100.0 * cable_vertical / weight:7.1f} %W\n"
            f"drone support            {100.0 * drone_vertical / weight:7.1f} %W\n"
            f"drone power proxy        {100.0 * drone_power_ratio:7.1f} %hover\n"
            f"spool mech power         {spool_power_abs:7.3f} W"
        )

    def _update_live_plots(self) -> None:
        if not self.sim.history:
            return
        latest_t = self.sim.history[-1].t
        start_t = max(0.0, latest_t - self.live_window_s)
        samples = [sample for sample in self.sim.history if sample.t >= start_t]
        times = [sample.t for sample in samples]
        x_errors = [sample.tool_head[0] - sample.reference[0] for sample in samples]
        z_errors = [sample.tool_head[1] - sample.reference[1] for sample in samples]
        errors = [sample.tool_error for sample in samples]
        measured_errors = [sample.measured_tool_error for sample in samples]

        self.error_norm_line.set_data(times, errors)
        self.measured_error_line.set_data(times, measured_errors)
        self.error_x_line.set_data(times, x_errors)
        self.error_z_line.set_data(times, z_errors)

        x_right = max(self.live_window_s, latest_t)
        x_left = max(0.0, x_right - self.live_window_s)
        self.error_ax.set_xlim(x_left, x_right)
        max_error = max([abs(value) for value in x_errors + z_errors] + errors + measured_errors + [0.015])
        self.error_ax.set_ylim(-1.15 * max_error, 1.15 * max_error)

    @staticmethod
    def _set_arrow(arrow: FancyArrowPatch, start: Vec2, end: Vec2) -> None:
        arrow.set_positions(start, end)

    def input_mode_label(self) -> str:
        if self.draw_mode:
            return "draw"
        if self.append_mode:
            return "append"
        return "single"

    def animate(self, _frame: int):
        now = time.perf_counter()
        wall_dt = clamp(now - self._last_frame_wall_time, 0.0, 0.12)
        self._last_frame_wall_time = now
        if self.playing:
            speed = float(self.speed_slider.val)
            sim_dt = speed * wall_dt
            steps = max(1, int(round(sim_dt / self.params.dt))) if sim_dt > 0.0 else 0
            for _ in range(steps):
                self.sim.step()
            self.draw()
        return []

    def on_press(self, event) -> None:
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        clicked_point = self.sim._clamp_wall_point((float(event.xdata), float(event.ydata)))
        if self.draw_mode:
            self.is_drawing = True
            self.draw_points = [clicked_point]
            self.playing = False
            self._last_frame_wall_time = time.perf_counter()
            self.play_button.label.set_text("Play")
            self.draw()
            self.fig.canvas.draw_idle()
            return
        if self.append_mode:
            self.sim.append_target(clicked_point)
        else:
            self.sim.set_target(clicked_point)
        self.playing = True
        self._last_frame_wall_time = time.perf_counter()
        self.play_button.label.set_text("Pause")
        self.draw()
        self.fig.canvas.draw_idle()

    def on_motion(self, event) -> None:
        if not self.draw_mode or not self.is_drawing:
            return
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        point = self.sim._clamp_wall_point((float(event.xdata), float(event.ydata)))
        if self._append_draw_point(point):
            self.draw_preview_line.set_data(
                [draw_point[0] for draw_point in self.draw_points],
                [draw_point[1] for draw_point in self.draw_points],
            )
            self.fig.canvas.draw_idle()

    def on_release(self, event) -> None:
        if not self.draw_mode or not self.is_drawing:
            return
        if event.inaxes is self.ax and event.xdata is not None and event.ydata is not None:
            self._append_draw_point(self.sim._clamp_wall_point((float(event.xdata), float(event.ydata))))
        self.is_drawing = False
        smooth_path = self._simplify_draw_points(self.draw_points)
        self.draw_points = []
        if smooth_path:
            self.sim.set_smooth_path(smooth_path)
            self.playing = True
            self._last_frame_wall_time = time.perf_counter()
            self.play_button.label.set_text("Pause")
        self.draw()
        self.fig.canvas.draw_idle()

    def _append_draw_point(self, point: Vec2) -> bool:
        if not self.draw_points or distance2(point, self.draw_points[-1]) >= self.draw_min_spacing:
            self.draw_points.append(point)
            return True
        return False

    def _simplify_draw_points(self, points: Sequence[Vec2]) -> list[Vec2]:
        if not points:
            return []
        filtered = [points[0]]
        for point in points[1:]:
            if distance2(point, filtered[-1]) >= self.draw_min_spacing:
                filtered.append(point)
        if distance2(points[-1], filtered[-1]) >= 1e-6:
            filtered.append(points[-1])
        if len(filtered) <= self.draw_max_points:
            return filtered
        keep: list[Vec2] = []
        last_index = len(filtered) - 1
        for sample_index in range(self.draw_max_points):
            source_index = round(sample_index * last_index / (self.draw_max_points - 1))
            point = filtered[source_index]
            if not keep or distance2(point, keep[-1]) >= 1e-6:
                keep.append(point)
        return keep

    def toggle_play(self, _event) -> None:
        self.playing = not self.playing
        self._last_frame_wall_time = time.perf_counter()
        self.play_button.label.set_text("Pause" if self.playing else "Play")

    def reset(self, _event) -> None:
        self.sim.reset()
        self.playing = False
        self._last_frame_wall_time = time.perf_counter()
        self.append_mode = False
        self.draw_mode = False
        self.is_drawing = False
        self.draw_points = []
        self.play_button.label.set_text("Play")
        self.append_button.label.set_text("Append Off")
        self.draw_button.label.set_text("Draw Off")
        self.draw()
        self.fig.canvas.draw_idle()

    def clear_trace(self, _event) -> None:
        self.sim.clear_trajectory()
        self.sim.history = self.sim.history[-1:]
        self.draw_points = []
        self.draw()
        self.fig.canvas.draw_idle()

    def toggle_append(self, _event) -> None:
        self.append_mode = not self.append_mode
        if self.append_mode:
            self.draw_mode = False
            self.is_drawing = False
            self.draw_points = []
            self.draw_button.label.set_text("Draw Off")
        self.append_button.label.set_text("Append On" if self.append_mode else "Append Off")

    def toggle_draw(self, _event) -> None:
        self.draw_mode = not self.draw_mode
        self.is_drawing = False
        self.draw_points = []
        if self.draw_mode:
            self.append_mode = False
            self.append_button.label.set_text("Append Off")
        self.draw_button.label.set_text("Draw On" if self.draw_mode else "Draw Off")
        self.draw()
        self.fig.canvas.draw_idle()

    def toggle_layer(self, label: str) -> None:
        if label == "trace":
            self.show_trace = not self.show_trace
        elif label == "target":
            self.show_target = not self.show_target
        elif label == "path":
            self.show_path = not self.show_path
        elif label == "forces":
            self.show_forces = not self.show_forces
        self._update_layer_button_labels()
        self.draw()
        self.fig.canvas.draw_idle()

    def _update_layer_button_labels(self) -> None:
        self.trace_button.label.set_text("Trace On" if self.show_trace else "Trace Off")
        self.target_button.label.set_text("Target On" if self.show_target else "Target Off")
        self.path_button.label.set_text("Path On" if self.show_path else "Path Off")
        self.forces_button.label.set_text("Forces On" if self.show_forces else "Forces Off")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive 2-D PRISMS wall-tool simulator.")
    parser.add_argument("--duration", type=float, default=8.0, help="Batch-simulation duration for --save-fig.")
    parser.add_argument("--dt", type=float, default=SimParams.dt)
    parser.add_argument(
        "--save-fig",
        default="",
        help="Optional PNG path for the current/final frame. Use with --no-show for batch export.",
    )
    parser.add_argument("--no-show", action="store_true", help="Run and save without opening a window.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    params = SimParams(dt=float(args.dt))
    simulator = WallToolSimulator(params)
    app = WallToolApp(simulator)

    if args.save_fig:
        simulator.set_target((0.65, 1.15))
        for _ in range(max(0, int(float(args.duration) / params.dt))):
            simulator.step()
        app.draw()
        output = Path(args.save_fig)
        if not output.is_absolute():
            output = PROJECT_ROOT / output
        output.parent.mkdir(parents=True, exist_ok=True)
        app.fig.savefig(output, dpi=180)
        print(f"Saved frame: {output}")
    if not args.no_show:
        ani = animation.FuncAnimation(app.fig, app.animate, interval=40, blit=False)
        app.fig._prisms_animation = ani
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
