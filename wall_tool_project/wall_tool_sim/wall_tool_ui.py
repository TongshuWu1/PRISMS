#!/usr/bin/env python3
"""Interactive 2.5D PRISMS wall-tool simulator.

The model is intentionally small, but it is not just a drawing. The suspended
system is integrated as a Cartesian point mass under gravity, finite cable
tension, and two bounded tilted drone thrust axes. The controller is a nonlinear
MPC with an inextensible unilateral cable model.
Facade work adds a separate normal-to-wall gap/contact state.
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

from cable_hybrid_controller.mpc import MPCConfig, MPCReferenceHorizon, MPCSolution, WallToolNMPC


Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]

DEFAULT_GRAVITY = 9.80665
PLANNER_DIRECT = "direct"
PLANNER_CENTER_SETUP = "center-setup"
PLANNER_PREDICTIVE = "predictive"
PLANNER_CHOICES = (PLANNER_DIRECT, PLANNER_CENTER_SETUP, PLANNER_PREDICTIVE)

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
    wall_width: float = 6.0
    wall_height: float = 6.0
    dt: float = 0.005
    drone_mass: float = 0.050
    payload_tool_mass: float = 0.075
    gravity: float = DEFAULT_GRAVITY
    cage_radius: float = 0.18
    payload_half_length: float = 0.190
    payload_hex_radius: float = 0.114
    module_gap: float = 0.0
    max_thrust_per_drone: float = 0.150 * DEFAULT_GRAVITY

    # The runtime controller is nonlinear MPC only. This name is kept for
    # reports and logs so runs state the selected controller explicitly.
    control_law: str = "tool_head_nmpc"

    # NMPC horizon, solver, constraints, and objective weights.
    mpc_horizon_steps: int = 18
    mpc_horizon_dt: float = 0.080
    mpc_control_period_s: float = 0.040
    mpc_attitude_limit_rad: float = 1.05
    mpc_slack_limit_m: float = 0.012
    mpc_tracking_position_weight: float = 260.0
    mpc_tracking_velocity_weight: float = 22.0
    mpc_terminal_position_weight: float = 520.0
    mpc_terminal_velocity_weight: float = 36.0
    mpc_drone_effort_weight: float = 0.42
    mpc_cable_effort_weight: float = 0.018
    mpc_reel_speed_weight: float = 0.050
    mpc_input_rate_weight: float = 0.030
    mpc_attitude_rate_weight: float = 0.45
    mpc_attitude_weight: float = 0.025
    mpc_slack_weight: float = 180.0
    mpc_solver_max_iter: int = 90
    mpc_solver_tolerance: float = 1e-5
    mpc_energy_plot_limit_J: float = 0.015

    # Cable and reel limits used by the active inextensible-cable plant.
    max_cable_support_fraction: float = 1.0
    max_spool_speed: float = 0.58
    spool_accel_limit_mps2: float = 0.80
    cable_taut_band: float = 0.006
    max_spool_tension: float = 24.0
    min_tracking_tension: float = 0.10
    min_cable_vertical_efficiency: float = 0.08
    min_control_cable_length: float = 0.62

    # Planner cost scales. These are route-selection terms, not a controller.
    max_tangential_accel: float = 2.8

    # Payload attitude dynamics. Attitude is chosen by the NMPC optimizer.
    rotational_damping: float = 0.090
    nominal_attitude_rad: float = 0.0

    # Desired path generation.
    path_speed: float = 0.16
    reference_accel_limit_mps2: float = 0.24
    reference_jerk_limit_mps3: float = 1.2
    reference_min_segment_duration_s: float = 0.90
    waypoint_tolerance: float = 0.012

    # Geometry and disturbances.
    min_cable_length: float = 0.10
    max_cable_length: float = 7.0
    initial_payload: Vec2 = (0.0, 2.00)
    wind_enabled: bool = False
    wind_force_x: float = 0.0
    wind_force_z: float = 0.0
    wind_gust_force: float = 0.0
    wind_gust_period_s: float = 11.0
    wind_gust_vertical_fraction: float = 0.35
    edge_wind_gain: float = 0.0

    # Normal-to-wall contact and work-quality checks. The planar controller still
    # tracks the desired x-z path; this state models facade standoff/force.
    normal_contact_enabled: bool = False
    normal_standoff_m: float = 0.10
    normal_initial_gap_m: float = 0.10
    normal_gap_min_m: float = -0.030
    normal_gap_max_m: float = 0.250
    normal_position_kp: float = 70.0
    normal_position_kd: float = 8.0
    normal_air_damping: float = 0.16
    normal_push_force_limit_N: float = 1.35
    normal_retract_force_limit_N: float = 0.85
    normal_contact_stiffness_N_m: float = 160.0
    normal_contact_damping_N_s_m: float = 1.8
    normal_contact_force_limit_N: float = 2.2
    desired_contact_force_N: float = 0.55
    min_contact_force_N: float = 0.25
    max_contact_force_N: float = 0.95
    contact_work_enabled: bool = False
    contact_work_x_min: float = -2.10
    contact_work_x_max: float = 2.10
    contact_work_z_min: float = 1.10
    contact_work_z_max: float = 5.25
    contact_work_margin_m: float = 0.04
    work_contact_speed_limit_mps: float = 0.36
    work_contact_tracking_limit_m: float = 0.12
    work_contact_angular_rate_limit_rad_s: float = 1.5
    normal_wind_force_N: float = 0.0
    normal_wind_gust_force_N: float = 0.0
    normal_wind_gust_period_s: float = 9.5

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


def clamp_wall_point_for_params(point: Vec2, params: SimParams) -> Vec2:
    margin = max(params.cage_radius, params.payload_half_length, params.payload_hex_radius) * 1.4
    return (
        clamp(point[0], -params.wall_width / 2.0 + margin, params.wall_width / 2.0 - margin),
        clamp(point[1], margin, params.wall_height - margin),
    )


def center_setup_waypoint(target: Vec2, params: SimParams) -> Vec2 | None:
    if target[1] < 2.65 or abs(target[0]) < 0.75:
        return None
    if target[1] > 3.20 and abs(target[0]) > 1.20:
        return clamp_wall_point_for_params((0.0, 1.70), params)
    setup_z = max(1.25, min(2.35, target[1] - 1.05))
    setup_x = 0.25 * target[0]
    return clamp_wall_point_for_params((setup_x, setup_z), params)


def snap_wall_point(point: Vec2) -> Vec2:
    return (round(float(point[0]), 5), round(float(point[1]), 5))


def cable_geometry_proxy(point: Vec2, params: SimParams) -> tuple[float, float, float]:
    dx = float(point[0]) - params.anchor[0]
    dz = params.anchor[1] - float(point[1])
    length = max(1e-9, math.hypot(dx, dz))
    theta = math.atan2(dx, dz)
    vertical_efficiency = dz / length
    return length, theta, vertical_efficiency


def route_length(points: Sequence[Vec2], start: Vec2) -> float:
    total = 0.0
    current = start
    for point in points:
        total += distance2(current, point)
        current = point
    return total


def predictive_setup_candidates(start: Vec2, target: Vec2, params: SimParams) -> tuple[Vec2, ...]:
    candidates: list[Vec2] = []

    def add(point: Vec2 | None) -> None:
        if point is None:
            return
        candidate = snap_wall_point(clamp_wall_point_for_params(point, params))
        if candidate == snap_wall_point(target) or candidate in candidates:
            return
        candidates.append(candidate)

    add(center_setup_waypoint(target, params))
    sign = 1.0 if target[0] >= 0.0 else -1.0
    setup_z_values = (
        1.45,
        1.70,
        1.85,
        2.05,
        max(1.30, min(2.35, target[1] - 1.25)),
        max(1.30, min(2.45, target[1] - 0.95)),
    )
    setup_x_values = (
        0.0,
        0.15 * target[0],
        0.25 * target[0],
        0.40 * target[0],
        sign * 0.22,
    )
    direct = max(distance2(start, target), 1e-6)
    for z in setup_z_values:
        for x in setup_x_values:
            candidate = snap_wall_point(clamp_wall_point_for_params((x, z), params))
            if route_length((candidate, target), start) / direct <= 2.35:
                add(candidate)
    return tuple(candidates)


def predictive_route_cost(start: Vec2, route: Sequence[Vec2], params: SimParams) -> float:
    if not route:
        return math.inf
    target = route[-1]
    direct_length = max(distance2(start, target), 1e-6)
    total_length = route_length(route, start)
    target_length, _target_theta, target_efficiency = cable_geometry_proxy(target, params)
    hard_side_target = target[1] > 2.65 and abs(target[0]) > 0.75
    very_hard_target = target[1] > 3.20 and abs(target[0]) > 1.20

    cost = 0.40 * total_length / direct_length
    current = start
    current_length, current_theta, _current_efficiency = cable_geometry_proxy(current, params)
    for point in route:
        length, theta, efficiency = cable_geometry_proxy(point, params)
        segment_distance = max(distance2(current, point), 1e-6)
        segment_time = max(0.45, segment_distance / max(params.path_speed, 1e-6))
        theta_step = abs(wrap_angle(theta - current_theta))
        length_step = abs(length - current_length)
        angular_accel_proxy = length * theta_step / max(segment_time * segment_time, 1e-6)
        radial_speed_proxy = length_step / segment_time
        shallow_penalty = max(0.0, 0.48 - efficiency)
        cost += (
            0.70 * theta_step * theta_step
            + 0.16 * length_step
            + 4.50 * shallow_penalty * shallow_penalty
            + 0.20 * max(0.0, angular_accel_proxy / max(params.max_tangential_accel, 1e-6) - 1.0) ** 2
            + 0.18 * max(0.0, radial_speed_proxy / max(params.max_spool_speed, 1e-6) - 1.0) ** 2
        )
        if efficiency < params.min_cable_vertical_efficiency or length < params.min_control_cable_length:
            cost += 50.0
        current = point
        current_length = length
        current_theta = theta

    if hard_side_target and len(route) == 1:
        cost += 0.55 + 1.50 * max(0.0, 0.45 - target_efficiency)
        if very_hard_target:
            cost += 1.25 + 0.60 * abs(start[0]) + 0.40 * max(0.0, start[1] - 2.10)
    if hard_side_target and len(route) > 1:
        setup = route[0] if len(route) > 1 else start
        setup_length, _setup_theta, setup_efficiency = cable_geometry_proxy(setup, params)
        preferred_setup_x = 0.0 if very_hard_target else 0.25 * target[0]
        preferred_setup_z = 1.70 if very_hard_target else max(1.55, min(1.85, target[1] - 1.05))
        setup_center_error = abs(setup[0] - preferred_setup_x)
        setup_height_error = setup[1] - preferred_setup_z
        setup_target_clearance = max(0.0, setup[1] - (target[1] - 1.15))
        cost += (
            1.80 * setup_center_error
            + 8.00 * setup_height_error * setup_height_error
            + 2.50 * setup_target_clearance * setup_target_clearance
            + 0.90 * max(0.0, 0.88 - setup_efficiency) ** 2
            + 0.03 * setup_length
        )
        if very_hard_target and setup[1] > 1.90:
            cost += 20.0 * (setup[1] - 1.90) ** 2
    return cost


def predictive_waypoints(start: Vec2, target: Vec2, params: SimParams) -> tuple[Vec2, ...]:
    target = snap_wall_point(clamp_wall_point_for_params(target, params))
    if target[1] <= 2.65 or abs(target[0]) <= 0.75:
        return (target,)
    routes: list[tuple[Vec2, ...]] = [(target,)]
    for candidate in predictive_setup_candidates(start, target, params):
        route = (candidate, target)
        if route not in routes:
            routes.append(route)
    return min(routes, key=lambda route: predictive_route_cost(start, route, params))


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
    active_target: Vec2
    measured_tool_error: float
    spool_velocity_cmd: float
    drone_accel_cmd: float
    desired_cable_tension: float
    measured_cable_length: float
    measured_tension: float
    desired_drone_force: Vec2
    drone_force: Vec2
    cable_force: Vec2
    wind_force: Vec2
    normal_gap: float
    normal_velocity: float
    normal_acceleration: float
    normal_actuator_force: float
    normal_wind_force: float
    contact_force: float
    desired_contact_force: float
    contact_valid: bool
    work_mode: bool
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
    radial_position_error_m: float
    radial_velocity_error_m_s: float
    tangential_position_error_m: float
    tangential_velocity_error_m_s: float
    swing_energy_J: float
    swing_power_W: float
    clf_margin_W: float
    clf_projected_accel_m_s2: float
    mpc_predicted_path: tuple[Vec2, ...] = ()
    mpc_predicted_attitudes: tuple[float, ...] = ()
    mpc_predicted_tensions: tuple[float, ...] = ()
    mpc_predicted_spool_speeds: tuple[float, ...] = ()
    mpc_status: str = ""
    mpc_solve_time_s: float = 0.0
    mpc_objective: float = 0.0


@dataclass(frozen=True)
class ReferenceState:
    position: Vec2
    velocity: Vec2
    acceleration: Vec2
    final_target: Vec2
    active_target: Vec2
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
            active_target=self.end,
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
            return ReferenceState(point, (0.0, 0.0), (0.0, 0.0), self.end, self.end, False, 1)

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
            active_target=self.end,
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

    def __init__(
        self,
        initial_position: Vec2,
        speed: float,
        tolerance: float,
        accel_limit: float = math.inf,
        jerk_limit: float = math.inf,
        min_segment_duration: float = 0.45,
    ) -> None:
        self.speed = speed
        self.tolerance = tolerance
        self.accel_limit = accel_limit
        self.jerk_limit = jerk_limit
        self.min_segment_duration = min_segment_duration
        self.position = initial_position
        self.velocity: Vec2 = (0.0, 0.0)
        self.acceleration: Vec2 = (0.0, 0.0)
        self.goals: list[Vec2] = []
        self.segments: list[QuinticSegment] = []
        self.segment_time = 0.0
        self.geometric_progress_m = 0.0
        self._path_sample_cache: list[Vec2] | None = None
        self.final_target = initial_position
        self.mode = "hold"

    def _invalidate_path_cache(self) -> None:
        self._path_sample_cache = None

    def reset(self, position: Vec2) -> None:
        self.position = position
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals.clear()
        self.segments.clear()
        self.segment_time = 0.0
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        self.final_target = position
        self.mode = "hold"

    def command_straight(self, start: Vec2, goal: Vec2) -> None:
        self.position = start
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals = [goal]
        self.final_target = goal
        self.mode = "straight"
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
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
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        self._rebuild_smooth_segments()

    def append_stop_waypoint(self, start: Vec2, goal: Vec2) -> None:
        if not self.segments:
            self.position = start
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            segment_start = start
            self.segment_time = 0.0
        else:
            segment_start = self.segments[-1].end
        self.goals.append(goal)
        self.final_target = goal
        self.mode = "stop"
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        duration = self._segment_duration(segment_start, goal)
        self.segments.append(
            QuinticSegment.build(
                segment_start,
                (0.0, 0.0),
                (0.0, 0.0),
                goal,
                (0.0, 0.0),
                (0.0, 0.0),
                duration,
            )
        )

    def command_smooth_path(self, start: Vec2, goals: Sequence[Vec2]) -> None:
        self.position = start
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals = [goals[-1]] if goals else []
        self.final_target = self.goals[-1] if self.goals else start
        self.mode = "draw"
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        points = [start, *goals]
        self.segments = [SampledPathSegment.build(points, self.speed)] if len(points) >= 2 else []
        self.segment_time = 0.0

    def command_corner_smooth_path(self, start: Vec2, goals: Sequence[Vec2], corner_speed: float) -> None:
        self.position = start
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals = list(goals)
        self.final_target = self.goals[-1] if self.goals else start
        self.mode = "coverage-smooth"
        self.segment_time = 0.0
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        if not goals:
            self.segments = []
            return

        points = [start, *goals]
        velocities = self._corner_smooth_velocities(points, corner_speed)
        self.segments = []
        for index in range(len(points) - 1):
            duration = self._segment_duration(points[index], points[index + 1])
            self.segments.append(
                QuinticSegment.build(
                    points[index],
                    velocities[index],
                    (0.0, 0.0),
                    points[index + 1],
                    velocities[index + 1],
                    (0.0, 0.0),
                    duration,
                )
            )

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
            self._invalidate_path_cache()
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

    def geometric_reference(self, tool_position: Vec2, tool_velocity: Vec2, dt: float) -> ReferenceState:
        """Path-following reference from closest tool-head projection, not time advance."""

        if not self.segments:
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            return self.state()

        samples = self._path_samples()
        projected_point, _tangent, _remaining_length, path_progress = self._project_tool_to_path(tool_position, samples)
        projection_error = distance2(tool_position, projected_point)
        capture_radius = max(0.12, 6.0 * self.tolerance)
        if projection_error <= capture_radius and path_progress > self.geometric_progress_m:
            # Prevent projection from jumping across nearby drawn-path segments.
            max_progress_step = max(0.004, 3.0 * self.speed * max(dt, 0.0))
            self.geometric_progress_m = min(path_progress, self.geometric_progress_m + max_progress_step)
        projection, tangent, remaining_length = self._sample_path_at_progress(samples, self.geometric_progress_m)
        tracking_error = distance2(tool_position, projection)
        speed = self.speed
        slowdown_distance = self._geometric_slowdown_distance(speed)
        speed_scale = clamp(remaining_length / max(slowdown_distance, 1e-9), 0.0, 1.0)
        target_speed = speed * speed_scale
        if tracking_error > capture_radius or remaining_length <= self.tolerance:
            target_speed = 0.0

        self.position = projection
        self.velocity = scale2(tangent, target_speed)
        self.acceleration = (0.0, 0.0)

        completion_distance = max(3.0 * self.tolerance, 0.050)
        complete = (
            remaining_length <= completion_distance
            and distance2(tool_position, self.final_target) <= completion_distance
            and math.hypot(tool_velocity[0], tool_velocity[1]) <= 0.075
        )
        if complete:
            self.position = self.final_target
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            self.goals.clear()
            self.segments.clear()
            self.segment_time = 0.0
            self._invalidate_path_cache()
            self.mode = "hold"
            return self.state()

        return ReferenceState(
            position=self.position,
            velocity=self.velocity,
            acceleration=self.acceleration,
            final_target=self.final_target,
            active_target=self.goals[0] if self.goals else self.final_target,
            active=True,
            waypoint_count=max(1, len(self.goals)),
        )

    def state(self) -> ReferenceState:
        return ReferenceState(
            position=self.position,
            velocity=self.velocity,
            acceleration=self.acceleration,
            final_target=self.final_target,
            active_target=self.goals[0] if self.goals else self.final_target,
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

    def _path_samples(self) -> list[Vec2]:
        if not self.segments:
            return [self.position]
        if self._path_sample_cache is not None:
            return self._path_sample_cache

        points: list[Vec2] = []
        for segment in self.segments:
            if isinstance(segment, SampledPathSegment):
                segment_points = list(segment.samples)
            else:
                sample_count = max(12, int(segment.duration / 0.06))
                segment_points = [segment.sample(segment.duration * index / sample_count).position for index in range(sample_count + 1)]
            for point in segment_points:
                if not points or distance2(point, points[-1]) > 1e-6:
                    points.append(point)
        self._path_sample_cache = points if len(points) >= 2 else [self.position, self.final_target]
        return self._path_sample_cache

    def _sample_path_at_progress(self, samples: Sequence[Vec2], progress_m: float) -> tuple[Vec2, Vec2, float]:
        if len(samples) < 2:
            return samples[0], (1.0, 0.0), 0.0

        total_length = sum(distance2(samples[index], samples[index + 1]) for index in range(len(samples) - 1))
        progress_m = clamp(progress_m, 0.0, total_length)
        distance_before = 0.0
        for index in range(len(samples) - 1):
            start = samples[index]
            end = samples[index + 1]
            segment_length = distance2(start, end)
            if segment_length <= 1e-9:
                continue
            if progress_m < distance_before + segment_length or index == len(samples) - 2:
                local = clamp((progress_m - distance_before) / segment_length, 0.0, 1.0)
                tangent = normalize2(sub2(end, start))
                position = add2(start, scale2(sub2(end, start), local))
                return position, tangent, max(0.0, total_length - progress_m)
            distance_before += segment_length

        tangent = normalize2(sub2(samples[-1], samples[-2]))
        return samples[-1], tangent, 0.0

    def _project_tool_to_path(self, tool_position: Vec2, samples: Sequence[Vec2]) -> tuple[Vec2, Vec2, float, float]:
        best_projection = samples[0]
        best_tangent = normalize2(sub2(samples[min(1, len(samples) - 1)], samples[0]))
        best_distance_squared = math.inf
        best_remaining_length = 0.0
        best_progress = 0.0
        total_length = 0.0
        segment_lengths: list[float] = []
        for index in range(len(samples) - 1):
            segment_length = distance2(samples[index], samples[index + 1])
            segment_lengths.append(segment_length)
            total_length += segment_length

        distance_before = 0.0
        for index, segment_length in enumerate(segment_lengths):
            if segment_length <= 1e-9:
                continue
            start = samples[index]
            end = samples[index + 1]
            delta = sub2(end, start)
            local = clamp(dot2(sub2(tool_position, start), delta) / (segment_length * segment_length), 0.0, 1.0)
            projection = add2(start, scale2(delta, local))
            error = sub2(tool_position, projection)
            distance_squared = dot2(error, error)
            progress = distance_before + local * segment_length
            if (
                distance_squared < best_distance_squared - 1e-12
                or abs(distance_squared - best_distance_squared) <= 1e-12
                and progress > best_progress
            ):
                best_distance_squared = distance_squared
                best_projection = projection
                best_tangent = scale2(delta, 1.0 / segment_length)
                best_progress = progress
                best_remaining_length = max(0.0, total_length - progress)
            distance_before += segment_length

        if abs(best_tangent[0]) + abs(best_tangent[1]) <= 1e-9:
            best_tangent = (1.0, 0.0)
        return best_projection, normalize2(best_tangent), best_remaining_length, best_progress

    def _geometric_slowdown_distance(self, speed: float) -> float:
        dynamic_distance = 0.0
        if math.isfinite(self.accel_limit) and self.accel_limit > 1e-9:
            dynamic_distance = speed * speed / (2.0 * self.accel_limit)
        return max(0.12, 1.4 * dynamic_distance, 2.0 * self.tolerance)

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

    def _corner_smooth_velocities(self, points: Sequence[Vec2], corner_speed: float) -> list[Vec2]:
        velocities = [(0.0, 0.0) for _ in points]
        bounded_corner_speed = clamp(corner_speed, 0.0, self.speed)
        x_values = [point[0] for point in points]
        z_values = [point[1] for point in points]
        min_x = min(x_values)
        max_x = max(x_values)
        min_z = min(z_values)
        max_z = max(z_values)
        boundary_eps = 1e-6
        for index in range(1, len(points) - 1):
            incoming = normalize2(sub2(points[index], points[index - 1]))
            outgoing = normalize2(sub2(points[index + 1], points[index]))
            turn_alignment = dot2(incoming, outgoing)
            if turn_alignment > 0.95:
                waypoint_speed = self.speed
            elif turn_alignment > 0.20:
                waypoint_speed = min(self.speed, 2.0 * bounded_corner_speed)
            else:
                waypoint_speed = bounded_corner_speed
            velocity = scale2(normalize2(sub2(points[index + 1], points[index - 1])), waypoint_speed)
            x, z = points[index]
            vx, vz = velocity
            if x <= min_x + boundary_eps or x >= max_x - boundary_eps:
                vx = 0.0
            if z <= min_z + boundary_eps or z >= max_z - boundary_eps:
                vz = 0.0
            velocities[index] = (vx, vz)
        return velocities

    def _segment_duration(self, start: Vec2, end: Vec2) -> float:
        distance = distance2(start, end)
        speed_duration = distance / max(self.speed, 1e-6)
        accel_duration = 0.0
        if math.isfinite(self.accel_limit) and self.accel_limit > 1e-9:
            accel_duration = math.sqrt(5.7736 * distance / self.accel_limit)
        jerk_duration = 0.0
        if math.isfinite(self.jerk_limit) and self.jerk_limit > 1e-9:
            jerk_duration = (60.0 * distance / self.jerk_limit) ** (1.0 / 3.0)
        return max(self.min_segment_duration, speed_duration, accel_duration, jerk_duration)


class WallToolSimulator:
    def __init__(self, params: SimParams) -> None:
        self.params = params
        self.default_target = params.initial_payload
        self.trajectory = ReferenceTrajectory(
            params.initial_payload,
            speed=params.path_speed,
            tolerance=params.waypoint_tolerance,
            accel_limit=params.reference_accel_limit_mps2,
            jerk_limit=params.reference_jerk_limit_mps3,
            min_segment_duration=params.reference_min_segment_duration_s,
        )
        self._nmpc: WallToolNMPC | None = None
        self._nmpc_next_solve_t = 0.0
        self._nmpc_last_solution: MPCSolution | None = None
        self._nmpc_previous_command = (0.0, 0.0, 0.0, 0.0)
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
        initial_tension = 0.0
        self.cable_length = clamp(
            initial_distance,
            self.params.min_cable_length,
            self.params.max_cable_length,
        )
        self.cable_stretch = 0.0
        self.cable_slack = False
        self.cable_tension_saturated = False
        self.filtered_cable_tension_target = initial_tension
        self.actual_tension = initial_tension
        self.last_spool_velocity_cmd = 0.0
        self._nmpc_next_solve_t = 0.0
        self._nmpc_last_solution = None
        self._nmpc_previous_command = (0.0, 0.0, initial_tension, 0.0)
        self.measured_payload = self.position
        self.estimated_payload_velocity = (0.0, 0.0)
        self.measured_theta = 0.0
        self.measured_theta_dot = 0.0
        self.measured_line_length = initial_distance
        self.measured_line_velocity = 0.0
        self.measured_cable_stretch = self.cable_stretch
        self.measured_attitude = self.attitude
        self.measured_angular_velocity = 0.0
        self.measured_cable_length = self.cable_length
        self.measured_cable_velocity = 0.0
        self.measured_tension = initial_tension
        self.normal_gap = clamp(
            self.params.normal_initial_gap_m,
            self.params.normal_gap_min_m,
            self.params.normal_gap_max_m,
        )
        self.normal_velocity = 0.0
        self.normal_acceleration = 0.0
        self.normal_actuator_force = 0.0
        self.normal_wind_force = 0.0
        self.contact_force = 0.0
        self.desired_contact_force = 0.0
        self.contact_work_mode = False
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
                (0.0, 0.0),
                False,
                self.trajectory.state(),
            )
        ]

    def _clamp_wall_point(self, point: Vec2) -> Vec2:
        return clamp_wall_point_for_params(point, self.params)

    def planned_waypoints(self, point: Vec2, planner: str = PLANNER_DIRECT) -> tuple[Vec2, ...]:
        target = self._clamp_wall_point(point)
        if planner == PLANNER_DIRECT:
            return (snap_wall_point(target),)
        if planner == PLANNER_CENTER_SETUP:
            setup = center_setup_waypoint(target, self.params)
            return (snap_wall_point(setup), snap_wall_point(target)) if setup is not None else (snap_wall_point(target),)
        if planner == PLANNER_PREDICTIVE:
            return predictive_waypoints(self._payload_from_state(), target, self.params)
        raise ValueError(f"Unknown planner '{planner}'. Choose one of: {', '.join(PLANNER_CHOICES)}")

    def set_target(self, point: Vec2, planner: str = PLANNER_DIRECT) -> None:
        start = self._payload_from_state()
        waypoints = self.planned_waypoints(point, planner)
        # Point clicks use the same geometric path representation as drag paths.
        # With the direct planner this is a single segment to the clicked point,
        # not a setup waypoint or dt-advanced target.
        self.trajectory.command_smooth_path(start, waypoints)

    def append_target(self, point: Vec2, planner: str = PLANNER_DIRECT) -> None:
        for waypoint in self.planned_waypoints(point, planner):
            self.trajectory.append_smooth_waypoint(self._payload_from_state(), waypoint)

    def append_stop_target(self, point: Vec2, planner: str = PLANNER_DIRECT) -> None:
        for waypoint in self.planned_waypoints(point, planner):
            self.trajectory.append_stop_waypoint(self._payload_from_state(), waypoint)

    def set_smooth_path(self, points: Sequence[Vec2]) -> None:
        clamped_points = [self._clamp_wall_point(point) for point in points]
        if not clamped_points:
            return
        self.trajectory.command_smooth_path(self._payload_from_state(), clamped_points)

    def set_corner_smooth_path(self, points: Sequence[Vec2], corner_speed: float) -> None:
        clamped_points = [self._clamp_wall_point(point) for point in points]
        if not clamped_points:
            return
        self.trajectory.command_corner_smooth_path(self._payload_from_state(), clamped_points, corner_speed)

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
                last.wind_force,
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

    def _update_sensor_estimate(self) -> None:
        params = self.params
        self.measured_theta = self.theta
        self.measured_theta_dot = self.theta_dot
        self.measured_cable_length = self.cable_length
        self.measured_cable_velocity = self.last_spool_velocity_cmd
        self.measured_tension = clamp(self.actual_tension, 0.0, params.max_spool_tension)
        self.measured_attitude = self.attitude
        self.measured_angular_velocity = self.angular_velocity

        self.measured_cable_stretch = 0.0
        self.measured_line_velocity = self.length_dot
        self.measured_line_length = self.length
        self.measured_payload = self.position
        self.estimated_payload_velocity = self.velocity

    def _safe_reference(self, reference: ReferenceState) -> ReferenceState:
        return ReferenceState(
            position=self._clamp_wall_point(reference.position),
            velocity=reference.velocity,
            acceleration=reference.acceleration,
            final_target=self._clamp_wall_point(reference.final_target),
            active_target=self._clamp_wall_point(reference.active_target),
            active=reference.active,
            waypoint_count=reference.waypoint_count,
        )

    def _wind_force(self) -> Vec2:
        params = self.params
        if not params.wind_enabled:
            return (0.0, 0.0)
        phase = 2.0 * math.pi * self.t / max(params.wind_gust_period_s, 1e-6)
        gust = params.wind_gust_force * (0.55 * math.sin(phase) + 0.25 * math.sin(2.7 * phase + 0.6))
        edge_ratio = abs(self.position[0]) / max(params.wall_width / 2.0, 1e-6)
        edge_gain = 1.0 + params.edge_wind_gain * edge_ratio * edge_ratio
        return (
            edge_gain * (params.wind_force_x + gust),
            params.wind_force_z + params.wind_gust_vertical_fraction * gust * math.sin(0.43 * phase + 1.2),
        )

    def _normal_wind_force(self) -> float:
        params = self.params
        if not params.normal_contact_enabled:
            return 0.0
        phase = 2.0 * math.pi * self.t / max(params.normal_wind_gust_period_s, 1e-6)
        gust = params.normal_wind_gust_force_N * (0.55 * math.sin(phase + 0.4) + 0.22 * math.sin(2.1 * phase))
        return params.normal_wind_force_N + gust

    @staticmethod
    def _cable_support_tension_limit(cable_axis: Vec2, params: SimParams) -> float:
        vertical_efficiency = max(0.0, cable_axis[1])
        if vertical_efficiency <= 1e-9:
            return 0.0
        support_limited_tension = (
            params.max_cable_support_fraction * params.total_mass * params.gravity / vertical_efficiency
        )
        return clamp(support_limited_tension, 0.0, params.max_spool_tension)

    def _in_contact_work_region(self, point: Vec2) -> bool:
        params = self.params
        margin = params.contact_work_margin_m
        return (
            params.contact_work_enabled
            and params.contact_work_x_min - margin <= point[0] <= params.contact_work_x_max + margin
            and params.contact_work_z_min - margin <= point[1] <= params.contact_work_z_max + margin
        )

    def _contact_work_mode_for_reference(self, reference: ReferenceState) -> bool:
        return self._in_contact_work_region(reference.position) or self._in_contact_work_region(self.position)

    def _surface_contact_force(self, gap: float, velocity: float) -> float:
        params = self.params
        penetration = max(0.0, -gap)
        contact = params.normal_contact_stiffness_N_m * penetration - params.normal_contact_damping_N_s_m * velocity
        return clamp(contact, 0.0, params.normal_contact_force_limit_N)

    def _update_normal_contact(self, reference: ReferenceState) -> None:
        params = self.params
        if not params.normal_contact_enabled:
            self.normal_gap = params.normal_standoff_m
            self.normal_velocity = 0.0
            self.normal_acceleration = 0.0
            self.normal_actuator_force = 0.0
            self.normal_wind_force = 0.0
            self.contact_force = 0.0
            self.desired_contact_force = 0.0
            self.contact_work_mode = False
            return

        self.contact_work_mode = self._contact_work_mode_for_reference(reference)
        self.desired_contact_force = params.desired_contact_force_N if self.contact_work_mode else 0.0
        desired_gap = params.normal_standoff_m
        feedforward_force = 0.0
        if self.contact_work_mode:
            desired_gap = -self.desired_contact_force / max(params.normal_contact_stiffness_N_m, 1e-9)
            feedforward_force = self.desired_contact_force

        contact_before = self._surface_contact_force(self.normal_gap, self.normal_velocity)
        normal_error = self.normal_gap - desired_gap
        actuator_force = feedforward_force + params.normal_position_kp * normal_error + params.normal_position_kd * self.normal_velocity
        actuator_force = clamp(
            actuator_force,
            -params.normal_retract_force_limit_N,
            params.normal_push_force_limit_N,
        )
        normal_wind = self._normal_wind_force()
        normal_damping = params.normal_air_damping * self.normal_velocity
        self.normal_acceleration = (
            contact_before + normal_wind - actuator_force - normal_damping
        ) / max(params.total_mass, 1e-9)
        self.normal_velocity += self.normal_acceleration * params.dt
        self.normal_gap += self.normal_velocity * params.dt
        if self.normal_gap < params.normal_gap_min_m:
            self.normal_gap = params.normal_gap_min_m
            self.normal_velocity = max(0.0, self.normal_velocity)
        elif self.normal_gap > params.normal_gap_max_m:
            self.normal_gap = params.normal_gap_max_m
            self.normal_velocity = min(0.0, self.normal_velocity)
        self.normal_actuator_force = actuator_force
        self.normal_wind_force = normal_wind
        self.contact_force = self._surface_contact_force(self.normal_gap, self.normal_velocity)

    def _contact_valid_for_reference(self, reference: ReferenceState) -> bool:
        params = self.params
        if not (params.normal_contact_enabled and self.contact_work_mode):
            return False
        speed = math.hypot(self.velocity[0], self.velocity[1])
        tracking_error = distance2(self._payload_from_state(), reference.position)
        return (
            params.min_contact_force_N <= self.contact_force <= params.max_contact_force_N
            and speed <= params.work_contact_speed_limit_mps
            and tracking_error <= params.work_contact_tracking_limit_m
            and abs(self.angular_velocity) <= params.work_contact_angular_rate_limit_rad_s
        )

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

    def _nmpc_config(self) -> MPCConfig:
        params = self.params
        left_offset_zero, right_offset_zero = self._module_center_offsets(0.0)
        wall_margin = max(params.cage_radius, params.payload_half_length, params.payload_hex_radius) * 1.4
        return MPCConfig(
            horizon_steps=max(2, params.mpc_horizon_steps),
            horizon_dt=max(params.dt, params.mpc_horizon_dt),
            control_period_s=max(params.dt, params.mpc_control_period_s),
            mass=params.total_mass,
            inertia=params.assembly_inertia,
            gravity=params.gravity,
            anchor=params.anchor,
            wall_width=params.wall_width,
            wall_height=params.wall_height,
            wall_margin=wall_margin,
            payload_hex_radius=params.payload_hex_radius,
            payload_half_length=params.payload_half_length,
            module_gap=params.module_gap,
            left_center_offset_zero=left_offset_zero,
            right_center_offset_zero=right_offset_zero,
            hex_face_tilt_rad=params.hex_face_tilt_rad,
            nominal_attitude_rad=params.nominal_attitude_rad,
            rotational_damping=params.rotational_damping,
            max_thrust_per_drone=params.max_thrust_per_drone,
            max_cable_tension=params.max_spool_tension,
            max_cable_support_fraction=params.max_cable_support_fraction,
            min_cable_vertical_efficiency=params.min_cable_vertical_efficiency,
            min_cable_length=params.min_cable_length,
            max_cable_length=params.max_cable_length,
            max_spool_speed=params.max_spool_speed,
            spool_accel_limit_mps2=params.spool_accel_limit_mps2,
            attitude_limit_rad=params.mpc_attitude_limit_rad,
            slack_limit_m=params.mpc_slack_limit_m,
            tracking_position_weight=params.mpc_tracking_position_weight,
            tracking_velocity_weight=params.mpc_tracking_velocity_weight,
            terminal_position_weight=params.mpc_terminal_position_weight,
            terminal_velocity_weight=params.mpc_terminal_velocity_weight,
            drone_effort_weight=params.mpc_drone_effort_weight,
            cable_effort_weight=params.mpc_cable_effort_weight,
            reel_speed_weight=params.mpc_reel_speed_weight,
            input_rate_weight=params.mpc_input_rate_weight,
            attitude_rate_weight=params.mpc_attitude_rate_weight,
            attitude_weight=params.mpc_attitude_weight,
            slack_weight=params.mpc_slack_weight,
            solver_max_iter=max(20, params.mpc_solver_max_iter),
            solver_tolerance=max(1e-8, params.mpc_solver_tolerance),
        )

    def _ensure_nmpc(self) -> WallToolNMPC:
        if self._nmpc is None:
            self._nmpc = WallToolNMPC(self._nmpc_config())
        return self._nmpc

    def _nmpc_reference_horizon(self, reference: ReferenceState) -> MPCReferenceHorizon:
        params = self.params
        steps = max(2, params.mpc_horizon_steps)
        dt = max(params.dt, params.mpc_horizon_dt)
        if not reference.active or not self.trajectory.segments:
            positions = tuple(reference.position for _ in range(steps + 1))
            velocities = tuple((0.0, 0.0) for _ in range(steps + 1))
            return MPCReferenceHorizon(positions=positions, velocities=velocities)

        samples = self.trajectory._path_samples()
        progress0 = self.trajectory.geometric_progress_m
        speed = params.path_speed
        slowdown_distance = self.trajectory._geometric_slowdown_distance(speed)
        positions: list[Vec2] = []
        velocities: list[Vec2] = []
        for index in range(steps + 1):
            progress = progress0 + speed * dt * index
            point, tangent, remaining = self.trajectory._sample_path_at_progress(samples, progress)
            speed_scale = clamp(remaining / max(slowdown_distance, 1e-9), 0.0, 1.0)
            target_speed = speed * speed_scale
            if remaining <= params.waypoint_tolerance:
                target_speed = 0.0
            positions.append(point)
            velocities.append(scale2(tangent, target_speed))
        positions[0] = reference.position
        velocities[0] = reference.velocity
        return MPCReferenceHorizon(positions=tuple(positions), velocities=tuple(velocities))

    def _step_tool_head_nmpc(self) -> SimState:
        params = self.params
        mass = params.total_mass
        self._update_cable_coordinates()
        self._update_sensor_estimate()
        reference = self._safe_reference(
            self.trajectory.geometric_reference(self.measured_payload, self.estimated_payload_velocity, params.dt)
        )
        self._update_normal_contact(reference)

        true_mount = self._cable_mount_position(self.position, self.attitude)
        true_distance = distance2(params.anchor, true_mount)
        if self.cable_length < true_distance:
            self.cable_length = clamp(true_distance, params.min_cable_length, params.max_cable_length)
        measured_state = (
            self.measured_payload[0],
            self.measured_payload[1],
            self.estimated_payload_velocity[0],
            self.estimated_payload_velocity[1],
            self.measured_attitude,
            self.measured_angular_velocity,
            clamp(max(self.cable_length, true_distance), params.min_cable_length, params.max_cable_length),
        )

        if self._nmpc_last_solution is None or self.t + 1e-12 >= self._nmpc_next_solve_t:
            horizon = self._nmpc_reference_horizon(reference)
            solution = self._ensure_nmpc().solve(
                measured_state=measured_state,
                reference=horizon,
                previous_command=self._nmpc_previous_command,
            )
            self._nmpc_last_solution = solution
            self._nmpc_next_solve_t = self.t + max(params.dt, params.mpc_control_period_s)
        else:
            solution = self._nmpc_last_solution

        left_thrust = clamp(solution.left_thrust, 0.0, params.max_thrust_per_drone)
        right_thrust = clamp(solution.right_thrust, 0.0, params.max_thrust_per_drone)
        desired_cable_tension = clamp(solution.cable_tension, 0.0, params.max_spool_tension)
        max_spool_delta = params.spool_accel_limit_mps2 * params.dt
        spool_velocity_cmd = clamp(
            solution.spool_velocity,
            self.last_spool_velocity_cmd - max_spool_delta,
            self.last_spool_velocity_cmd + max_spool_delta,
        )
        spool_velocity_cmd = clamp(spool_velocity_cmd, -params.max_spool_speed, params.max_spool_speed)

        previous_cable_length = self.cable_length
        self.cable_length = clamp(
            self.cable_length + spool_velocity_cmd * params.dt,
            params.min_cable_length,
            params.max_cable_length,
        )
        if self.cable_length < true_distance:
            self.cable_length = clamp(true_distance, params.min_cable_length, params.max_cable_length)
            spool_velocity_cmd = (self.cable_length - previous_cable_length) / max(params.dt, 1e-9)
        self.last_spool_velocity_cmd = spool_velocity_cmd

        true_cable_arm = self._cable_mount_offset(self.attitude)
        true_mount_position = add2(self.position, true_cable_arm)
        true_mount_velocity = add2(self.velocity, scale2((-true_cable_arm[1], true_cable_arm[0]), self.angular_velocity))
        anchor_to_mount = sub2(true_mount_position, params.anchor)
        true_distance = max(1e-9, math.hypot(anchor_to_mount[0], anchor_to_mount[1]))
        true_cable_out = (anchor_to_mount[0] / true_distance, anchor_to_mount[1] / true_distance)
        true_cable_axis = (-true_cable_out[0], -true_cable_out[1])
        slack_gap = max(0.0, self.cable_length - true_distance)
        actual_tension_limit = self._cable_support_tension_limit(true_cable_axis, params)
        self.cable_tension_saturated = desired_cable_tension > actual_tension_limit
        if slack_gap <= params.cable_taut_band:
            tension = clamp(desired_cable_tension, 0.0, actual_tension_limit)
        else:
            tension = 0.0
        self.actual_tension = tension
        self.cable_stretch = 0.0
        self.cable_slack = slack_gap > params.cable_taut_band or tension <= 1e-9
        cable_force = scale2(true_cable_axis, tension)

        true_left_axis, true_right_axis = self._drone_axes(self.attitude)
        true_left_arm, true_right_arm = self._module_center_offsets(self.attitude)
        left_force = scale2(true_left_axis, left_thrust)
        right_force = scale2(true_right_axis, right_thrust)
        drone_force = add2(left_force, right_force)
        cable_torque = cross2(true_cable_arm, cable_force)
        left_torque = cross2(true_left_arm, left_force)
        right_torque = cross2(true_right_arm, right_force)
        net_attitude_torque = cable_torque + left_torque + right_torque - params.rotational_damping * self.angular_velocity
        gravity_force = (0.0, -mass * params.gravity)
        wind_force = self._wind_force()
        net_force = add2(add2(add2(drone_force, cable_force), gravity_force), wind_force)
        self.acceleration = scale2(net_force, 1.0 / mass)
        self.angular_acceleration = net_attitude_torque / max(params.assembly_inertia, 1e-9)

        self.velocity = add2(self.velocity, scale2(self.acceleration, params.dt))
        self.position = add2(self.position, scale2(self.velocity, params.dt))
        self.angular_velocity += self.angular_acceleration * params.dt
        self.attitude = wrap_angle(self.attitude + self.angular_velocity * params.dt)
        self.t += params.dt
        self._update_cable_coordinates()
        self._update_sensor_estimate()
        self._nmpc_previous_command = (left_thrust, right_thrust, desired_cable_tension, spool_velocity_cmd)

        control_tangential_axis = (math.cos(self.theta), math.sin(self.theta))
        tangential_force = dot2(drone_force, control_tangential_axis)
        desired_drone_force = drone_force
        speed_error = sub2(reference.velocity, self.estimated_payload_velocity)
        position_error = sub2(reference.position, self.measured_payload)
        swing_energy = 0.5 * mass * (
            dot2(speed_error, speed_error)
            + dot2(position_error, position_error)
        )
        state = self.snapshot(
            left_thrust,
            right_thrust,
            tension,
            tangential_force,
            spool_velocity_cmd,
            math.hypot(drone_force[0], drone_force[1]) / max(mass, 1e-9),
            desired_cable_tension,
            desired_drone_force,
            drone_force,
            cable_force,
            wind_force,
            self.cable_tension_saturated,
            reference,
            desired_tangential_force=tangential_force,
            desired_attitude_torque=0.0,
            attitude_torque=net_attitude_torque,
            cable_torque=cable_torque,
            left_torque=left_torque,
            right_torque=right_torque,
            allocation_residual=0.0,
            radial_position_error_m=position_error[1],
            radial_velocity_error_m_s=speed_error[1],
            tangential_position_error_m=position_error[0],
            tangential_velocity_error_m_s=speed_error[0],
            swing_energy_J=swing_energy,
            mpc_solution=solution,
        )
        self.history.append(state)
        if len(self.history) > 6000:
            self.history = self.history[-6000:]
        return state

    def step(self) -> SimState:
        return self._step_tool_head_nmpc()

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
        wind_force: Vec2,
        saturated: bool,
        reference: ReferenceState,
        desired_tangential_force: float = 0.0,
        desired_attitude_torque: float = 0.0,
        attitude_torque: float = 0.0,
        cable_torque: float = 0.0,
        left_torque: float = 0.0,
        right_torque: float = 0.0,
        allocation_residual: float | None = None,
        radial_position_error_m: float = 0.0,
        radial_velocity_error_m_s: float = 0.0,
        tangential_position_error_m: float = 0.0,
        tangential_velocity_error_m_s: float = 0.0,
        swing_energy_J: float = 0.0,
        swing_power_W: float = 0.0,
        clf_margin_W: float = 0.0,
        clf_projected_accel_m_s2: float = 0.0,
        mpc_solution: MPCSolution | None = None,
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
            active_target=reference.active_target,
            measured_tool_error=distance2(self.measured_payload, desired_tool_head),
            spool_velocity_cmd=spool_velocity_cmd,
            drone_accel_cmd=drone_accel_cmd,
            desired_cable_tension=desired_cable_tension,
            measured_cable_length=self.measured_cable_length,
            measured_tension=self.measured_tension,
            desired_drone_force=desired_drone_force,
            drone_force=drone_force,
            cable_force=cable_force,
            wind_force=wind_force,
            normal_gap=self.normal_gap,
            normal_velocity=self.normal_velocity,
            normal_acceleration=self.normal_acceleration,
            normal_actuator_force=self.normal_actuator_force,
            normal_wind_force=self.normal_wind_force,
            contact_force=self.contact_force,
            desired_contact_force=self.desired_contact_force,
            contact_valid=self._contact_valid_for_reference(reference),
            work_mode=self.contact_work_mode,
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
            radial_position_error_m=radial_position_error_m,
            radial_velocity_error_m_s=radial_velocity_error_m_s,
            tangential_position_error_m=tangential_position_error_m,
            tangential_velocity_error_m_s=tangential_velocity_error_m_s,
            swing_energy_J=swing_energy_J,
            swing_power_W=swing_power_W,
            clf_margin_W=clf_margin_W,
            clf_projected_accel_m_s2=clf_projected_accel_m_s2,
            mpc_predicted_path=mpc_solution.predicted_positions if mpc_solution is not None else (),
            mpc_predicted_attitudes=mpc_solution.predicted_attitudes if mpc_solution is not None else (),
            mpc_predicted_tensions=mpc_solution.predicted_tensions if mpc_solution is not None else (),
            mpc_predicted_spool_speeds=mpc_solution.predicted_spool_speeds if mpc_solution is not None else (),
            mpc_status=mpc_solution.status if mpc_solution is not None else "",
            mpc_solve_time_s=mpc_solution.solve_time_s if mpc_solution is not None else 0.0,
            mpc_objective=mpc_solution.objective if mpc_solution is not None else 0.0,
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
    def __init__(self, simulator: WallToolSimulator, planner: str = PLANNER_DIRECT) -> None:
        self.sim = simulator
        self.params = simulator.params
        self.planner = planner
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
        self.live_window_s = 16.0
        self._last_frame_wall_time = time.perf_counter()

        self.fig = plt.figure(figsize=(15.0, 9.2), constrained_layout=False)
        grid = self.fig.add_gridspec(
            2,
            2,
            width_ratios=[1.0, 0.54],
            height_ratios=[1.0, 0.18],
            left=0.055,
            right=0.975,
            bottom=0.08,
            top=0.92,
            wspace=0.08,
            hspace=0.18,
        )
        self.ax = self.fig.add_subplot(grid[0, 0])
        panel_grid = grid[0, 1].subgridspec(
            5,
            1,
            height_ratios=[0.38, 0.22, 0.22, 0.22, 0.22],
            hspace=0.42,
        )
        self.panel_ax = self.fig.add_subplot(panel_grid[0])
        self.task_ax = self.fig.add_subplot(panel_grid[1])
        self.smooth_ax = self.fig.add_subplot(panel_grid[2])
        self.cable_ax = self.fig.add_subplot(panel_grid[3])
        self.reel_ax = self.fig.add_subplot(panel_grid[4])
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
        if params.contact_work_enabled:
            self.work_region_patch = Rectangle(
                (params.contact_work_x_min, params.contact_work_z_min),
                params.contact_work_x_max - params.contact_work_x_min,
                params.contact_work_z_max - params.contact_work_z_min,
                facecolor="#ffffff",
                edgecolor="#2f6f4e",
                linewidth=1.8,
                linestyle="-",
                alpha=0.30,
                zorder=1,
            )
            self.ax.add_patch(self.work_region_patch)
            self.ax.text(
                params.contact_work_x_min,
                params.contact_work_z_max + 0.06,
                "cleaning bay",
                color="#2f6f4e",
                fontsize=8.5,
                va="bottom",
            )
        self.ax.grid(True, color="#d8d4c9", linewidth=0.8)

        self.spool = Circle(params.anchor, 0.075, facecolor="#444444", edgecolor="black", zorder=5)
        self.ax.add_patch(self.spool)
        self.ax.text(params.anchor[0], params.anchor[1] + 0.13, "anchor + spool", ha="center", fontsize=9)

        self.cable_line, = self.ax.plot([], [], color="#222222", linewidth=2.0, zorder=3)
        self.trace_line, = self.ax.plot([], [], color="#2b7a78", linewidth=2.0, alpha=0.80, zorder=2)
        self.desired_trace_line, = self.ax.plot([], [], color="#8a5b22", linewidth=1.8, linestyle=":", alpha=0.90, zorder=2)
        self.path_line, = self.ax.plot([], [], color="#555555", linewidth=1.5, linestyle="--", alpha=0.72, zorder=4)
        self.mpc_prediction_line, = self.ax.plot([], [], color="#6b46c1", linewidth=1.7, linestyle="-.", alpha=0.90, zorder=6)
        self.draw_preview_line, = self.ax.plot([], [], color="#f39c12", linewidth=2.0, alpha=0.85, zorder=8)
        self.structure_line, = self.ax.plot([], [], color="#4a4a4a", linewidth=2.2, alpha=0.55, zorder=5)
        self.cable_mount_point, = self.ax.plot([], [], marker="o", linestyle="none", color="#111111", markersize=4.2, zorder=13)
        self.attitude_line, = self.ax.plot([], [], color="#111111", linewidth=1.6, alpha=0.85, zorder=13)
        self.reference_point, = self.ax.plot([], [], marker="o", color="#1f77b4", markersize=5.0, zorder=9, visible=False)
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
        self.panel_ax.text(0.0, 0.98, "Mission Monitor", fontsize=12, fontweight="bold", va="top")
        self.state_text = self.panel_ax.text(
            0.0,
            0.88,
            "",
            fontsize=7.0,
            family="monospace",
            va="top",
            linespacing=1.10,
            clip_on=True,
        )

    def _build_live_plots(self) -> None:
        self.task_error_line, = self.task_ax.plot([], [], color="#111111", linewidth=1.5, label="tracking")
        self.task_speed_line, = self.task_ax.plot([], [], color="#2b6cb0", linewidth=1.3, label="speed")
        self.task_contact_line, = self.task_ax.plot([], [], color="#2f855a", linewidth=1.2, linestyle="--", label="valid")
        self._format_ratio_axis(self.task_ax, "Task Validity", "limit ratio")
        self.task_ax.tick_params(labelbottom=False)

        self.smooth_accel_line, = self.smooth_ax.plot([], [], color="#6b46c1", linewidth=1.35, label="accel")
        self.smooth_body_line, = self.smooth_ax.plot([], [], color="#c05621", linewidth=1.25, label="body rate")
        self.smooth_cable_rate_line, = self.smooth_ax.plot([], [], color="#718096", linewidth=1.1, label="cable rate")
        self.smooth_energy_line, = self.smooth_ax.plot([], [], color="#2f855a", linewidth=1.1, label="swing E")
        self._format_ratio_axis(self.smooth_ax, "Smoothness", "ratio")
        self.smooth_ax.tick_params(labelbottom=False)

        self.support_line, = self.cable_ax.plot([], [], color="#2f855a", linewidth=1.35, label="cable support")
        self.power_line, = self.cable_ax.plot([], [], color="#6b46c1", linewidth=1.25, label="drone power")
        self.thrust_fraction_line, = self.cable_ax.plot([], [], color="#c53030", linewidth=1.2, label="max thrust")
        self._format_ratio_axis(self.cable_ax, "Cable And Actuators", "fraction")
        self.cable_ax.tick_params(labelbottom=False)

        self.spool_velocity_ratio_line, = self.reel_ax.plot([], [], color="#2b6cb0", linewidth=1.2, label="spool speed")
        self.spool_accel_ratio_line, = self.reel_ax.plot([], [], color="#c53030", linewidth=1.0, alpha=0.78, label="spool accel")
        self._format_ratio_axis(self.reel_ax, "Reel Command", "ratio")
        self.reel_ax.set_xlabel("time [s]", fontsize=7.8)

    @staticmethod
    def _format_ratio_axis(ax, title: str, ylabel: str) -> None:
        ax.axhline(1.0, color="#d95f0e", linestyle="--", linewidth=0.9, alpha=0.85)
        ax.set_title(title, fontsize=9.2)
        ax.set_ylabel(ylabel, fontsize=7.8)
        ax.set_ylim(-0.04, 1.18)
        ax.grid(True, color="#dddddd", linewidth=0.7)
        ax.legend(loc="upper right", fontsize=5.9, framealpha=0.90, ncol=2)
        ax.tick_params(axis="both", labelsize=7.2)

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
            self.reference_point.set_data([], [])
            self.waypoint_points.set_data([], [])
        else:
            self.path_line.set_data([], [])
            self.reference_point.set_data([], [])
            self.waypoint_points.set_data([], [])
        if state.mpc_predicted_path:
            self.mpc_prediction_line.set_data(
                [point[0] for point in state.mpc_predicted_path],
                [point[1] for point in state.mpc_predicted_path],
            )
        else:
            self.mpc_prediction_line.set_data([], [])
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

        self.state_text.set_text(self._efficiency_text(state))
        self._update_live_plots()

    def _efficiency_text(self, state: SimState) -> str:
        params = self.params
        weight = max(params.total_mass * params.gravity, 1e-9)
        no_cable_hover_each = weight / max(2.0 * math.cos(params.hex_face_tilt_rad), 1e-9)
        no_cable_power_index = max(2.0 * no_cable_hover_each**1.5, 1e-9)
        drone_power_index = state.left_thrust**1.5 + state.right_thrust**1.5
        drone_power_ratio = drone_power_index / no_cable_power_index
        max_thrust_fraction = max(state.left_thrust, state.right_thrust) / max(params.max_thrust_per_drone, 1e-9)
        residual_fraction = state.allocation_residual / weight
        speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
        acceleration = math.hypot(state.payload_acceleration[0], state.payload_acceleration[1])
        tracking_ratio = state.tool_error / max(params.work_contact_tracking_limit_m, 1e-9)
        speed_ratio = speed / max(params.work_contact_speed_limit_mps, 1e-9)
        accel_ratio = acceleration / max(params.reference_accel_limit_mps2, 1e-9)
        body_rate_ratio = abs(state.angular_velocity) / max(params.work_contact_angular_rate_limit_rad_s, 1e-9)
        swing_energy_ratio = state.swing_energy_J / max(params.mpc_energy_plot_limit_J, 1e-9)
        spool_speed_ratio = abs(state.spool_velocity_cmd) / max(params.max_spool_speed, 1e-9)
        spool_accel_ratio = self._latest_spool_accel_ratio()
        cable_support = state.cable_vertical_force / weight
        contact_state = "OK" if state.contact_valid else ("BAD" if state.work_mode else "OFF")
        controller_state = "OK"
        if max(tracking_ratio, speed_ratio, body_rate_ratio, spool_accel_ratio, max_thrust_fraction) > 1.0:
            controller_state = "LIMIT"
        if state.cable_slack:
            controller_state = "SLACK"
        window_text = self._window_metrics_text()
        return (
            f"t {state.t:6.1f}s  wp {state.active_waypoints:2d}  {controller_state}\n"
            f"contact {contact_state:>3s} {state.contact_force:4.2f}N  mpc {1000.0 * state.mpc_solve_time_s:4.1f}ms\n"
            f"task   trk {tracking_ratio:4.2f}  spd {speed_ratio:4.2f}  body {body_rate_ratio:4.2f}\n"
            f"smooth acc {accel_ratio:4.2f}  E {swing_energy_ratio:4.2f}  reel {spool_accel_ratio:4.2f}\n"
            f"cable  sup {100.0 * cable_support:4.0f}%  power {100.0 * drone_power_ratio:4.0f}%  res {100.0 * residual_fraction:4.1f}%\n"
            f"{window_text}"
        )

    def _window_metrics_text(self) -> str:
        if not self.sim.history:
            return ""
        latest_t = self.sim.history[-1].t
        start_t = max(0.0, latest_t - self.live_window_s)
        samples = [sample for sample in self.sim.history if sample.t >= start_t]
        if not samples:
            return ""
        params = self.params
        weight = max(params.total_mass * params.gravity, 1e-9)
        no_cable_hover_each = weight / max(2.0 * math.cos(params.hex_face_tilt_rad), 1e-9)
        no_cable_power_index = max(2.0 * no_cable_hover_each**1.5, 1e-9)
        rms_error = math.sqrt(sum(sample.tool_error * sample.tool_error for sample in samples) / len(samples))
        valid_fraction = sum(1.0 if sample.contact_valid else 0.0 for sample in samples) / len(samples)
        cable_support = sum(sample.cable_vertical_force / weight for sample in samples) / len(samples)
        drone_power = sum((sample.left_thrust**1.5 + sample.right_thrust**1.5) / no_cable_power_index for sample in samples) / len(samples)
        max_thrust = max(max(sample.left_thrust, sample.right_thrust) / max(params.max_thrust_per_drone, 1e-9) for sample in samples)
        body_rates = sorted(abs(sample.angular_velocity) for sample in samples)
        p95_body = body_rates[int(0.95 * (len(body_rates) - 1))] if body_rates else 0.0
        jerks: list[float] = []
        for index in range(1, len(samples)):
            dt = max(samples[index].t - samples[index - 1].t, 1e-9)
            da = sub2(samples[index].payload_acceleration, samples[index - 1].payload_acceleration)
            jerks.append(math.hypot(da[0], da[1]) / dt)
        sorted_jerks = sorted(jerks)
        p95_jerk = sorted_jerks[int(0.95 * (len(sorted_jerks) - 1))] if sorted_jerks else 0.0
        return (
            f"last {self.live_window_s:.0f}s rms {rms_error:5.3f}m  valid {100.0 * valid_fraction:4.0f}%\n"
            f"p95 body {p95_body:4.2f}rad/s  jerk {p95_jerk:4.1f}\n"
            f"avg sup {100.0 * cable_support:4.0f}%  power {100.0 * drone_power:4.0f}%  peak {100.0 * max_thrust:4.0f}%"
        )

    def _latest_spool_accel_ratio(self) -> float:
        if len(self.sim.history) < 2:
            return 0.0
        current = self.sim.history[-1]
        previous = self.sim.history[-2]
        dt = max(current.t - previous.t, 1e-9)
        spool_accel = abs((current.spool_velocity_cmd - previous.spool_velocity_cmd) / dt)
        return spool_accel / max(self.params.spool_accel_limit_mps2, 1e-9)

    def _update_live_plots(self) -> None:
        if not self.sim.history:
            return
        latest_t = self.sim.history[-1].t
        start_t = max(0.0, latest_t - self.live_window_s)
        samples = [sample for sample in self.sim.history if sample.t >= start_t]
        times = [sample.t for sample in samples]
        params = self.params
        weight = max(params.total_mass * params.gravity, 1e-9)
        no_cable_hover_each = weight / max(2.0 * math.cos(params.hex_face_tilt_rad), 1e-9)
        no_cable_power_index = max(2.0 * no_cable_hover_each**1.5, 1e-9)

        tracking_ratio = [sample.tool_error / max(params.work_contact_tracking_limit_m, 1e-9) for sample in samples]
        speed_ratio = [
            math.hypot(sample.payload_velocity[0], sample.payload_velocity[1])
            / max(params.work_contact_speed_limit_mps, 1e-9)
            for sample in samples
        ]
        contact_valid = [1.0 if sample.contact_valid else 0.0 for sample in samples]
        self.task_error_line.set_data(times, tracking_ratio)
        self.task_speed_line.set_data(times, speed_ratio)
        self.task_contact_line.set_data(times, contact_valid)

        accel_ratio = [
            math.hypot(sample.payload_acceleration[0], sample.payload_acceleration[1])
            / max(params.reference_accel_limit_mps2, 1e-9)
            for sample in samples
        ]
        body_rate_ratio = [
            abs(sample.angular_velocity) / max(params.work_contact_angular_rate_limit_rad_s, 1e-9)
            for sample in samples
        ]
        cable_rate_ratio = [
            abs(sample.theta_dot) / max(params.work_contact_angular_rate_limit_rad_s, 1e-9)
            for sample in samples
        ]
        swing_energy_ratio = [
            sample.swing_energy_J / max(params.mpc_energy_plot_limit_J, 1e-9)
            for sample in samples
        ]
        display_accel_ratio = self._moving_average(accel_ratio, 20)
        self.smooth_accel_line.set_data(times, display_accel_ratio)
        self.smooth_body_line.set_data(times, body_rate_ratio)
        self.smooth_cable_rate_line.set_data(times, cable_rate_ratio)
        self.smooth_energy_line.set_data(times, self._moving_average(swing_energy_ratio, 20))

        cable_support = [sample.cable_vertical_force / weight for sample in samples]
        drone_power = [(sample.left_thrust**1.5 + sample.right_thrust**1.5) / no_cable_power_index for sample in samples]
        thrust_fraction = [
            max(sample.left_thrust, sample.right_thrust) / max(params.max_thrust_per_drone, 1e-9)
            for sample in samples
        ]
        self.support_line.set_data(times, cable_support)
        self.power_line.set_data(times, drone_power)
        self.thrust_fraction_line.set_data(times, thrust_fraction)

        spool_speed_ratio = [
            abs(sample.spool_velocity_cmd) / max(params.max_spool_speed, 1e-9)
            for sample in samples
        ]
        spool_accel_ratio = [0.0]
        for index in range(1, len(samples)):
            dt = max(samples[index].t - samples[index - 1].t, 1e-9)
            spool_accel = abs((samples[index].spool_velocity_cmd - samples[index - 1].spool_velocity_cmd) / dt)
            spool_accel_ratio.append(spool_accel / max(params.spool_accel_limit_mps2, 1e-9))
        display_spool_accel_ratio = self._moving_average(spool_accel_ratio, 20)
        self.spool_velocity_ratio_line.set_data(times, spool_speed_ratio)
        self.spool_accel_ratio_line.set_data(times, display_spool_accel_ratio)

        x_right = max(self.live_window_s, latest_t)
        x_left = max(0.0, x_right - self.live_window_s)
        plot_groups = (
            (self.task_ax, tracking_ratio + speed_ratio + contact_valid),
            (self.smooth_ax, display_accel_ratio + body_rate_ratio + cable_rate_ratio + swing_energy_ratio),
            (self.cable_ax, cable_support + drone_power + thrust_fraction),
            (self.reel_ax, spool_speed_ratio + display_spool_accel_ratio),
        )
        for axis, values in plot_groups:
            axis.set_xlim(x_left, x_right)
            ymax = max(values + [1.0])
            axis.set_ylim(-0.04, min(1.65, max(1.18, 1.12 * ymax)))

    @staticmethod
    def _moving_average(values: Sequence[float], window: int) -> list[float]:
        if window <= 1 or not values:
            return list(values)
        smoothed: list[float] = []
        running_sum = 0.0
        queue: list[float] = []
        for value in values:
            queue.append(value)
            running_sum += value
            if len(queue) > window:
                running_sum -= queue.pop(0)
            smoothed.append(running_sum / len(queue))
        return smoothed

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
            self.sim.append_target(clicked_point, planner=self.planner)
        else:
            self.sim.set_target(clicked_point, planner=self.planner)
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
    parser = argparse.ArgumentParser(description="Interactive 2.5D PRISMS wall-tool simulator.")
    parser.add_argument("--duration", type=float, default=8.0, help="Batch-simulation duration for --save-fig.")
    parser.add_argument("--dt", type=float, default=SimParams.dt)
    parser.add_argument(
        "--planner",
        choices=PLANNER_CHOICES,
        default=PLANNER_DIRECT,
        help="Reference planner used for click targets and batch export.",
    )
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
    app = WallToolApp(simulator, planner=str(args.planner))

    if args.save_fig:
        simulator.set_target((0.65, 1.15), planner=str(args.planner))
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
