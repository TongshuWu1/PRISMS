#!/usr/bin/env python3
"""Interactive 2-D PRISMS wall-tool simulator.

The model is intentionally small, but it is not just a drawing. The suspended
system is treated as a variable-length pendulum under gravity. The spool
controls cable length, and the two docked PRISMS drone modules inject
wall-plane tangential force through their tilted thrust axes. Click any point on
the wall to command a smooth straight-line move, or enable append mode to queue
a smooth multi-waypoint trajectory.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
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
    from matplotlib.collections import LineCollection, PolyCollection
    from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle
    from matplotlib.widgets import Button, CheckButtons, Slider
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing matplotlib. Install project requirements first:\n"
        "  python -m pip install -r requirements.txt"
    ) from exc


Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]

SQRT5 = math.sqrt(5.0)
CAGE_ROT_Y_RAD = math.pi / 4.0
PAYLOAD_LEFT_HEX = (-1, -1, -1)
PAYLOAD_RIGHT_HEX = (1, 1, 1)
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


def normalize2(vector: Sequence[float]) -> Vec2:
    length = math.hypot(float(vector[0]), float(vector[1]))
    if length < 1e-12:
        return (0.0, 0.0)
    return float(vector[0]) / length, float(vector[1]) / length


def distance2(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


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


def projected_vertices(center: Vec2, radius: float) -> list[Vec2]:
    return [add2(center, project_local(vertex, radius)) for vertex in GEOMETRY.vertices]


def projected_faces(center: Vec2, radius: float) -> list[list[Vec2]]:
    points = projected_vertices(center, radius)
    return [[points[index] for index in face.indices] for face in GEOMETRY.faces]


def projected_edges(center: Vec2, radius: float) -> list[tuple[Vec2, Vec2]]:
    points = projected_vertices(center, radius)
    return [(points[start], points[end]) for start, end in GEOMETRY.edges]


def projected_face_polygon(center: Vec2, radius: float, normal_key: tuple[int, int, int]) -> list[Vec2]:
    face = GEOMETRY.face_by_normal[normal_key]
    points = projected_vertices(center, radius)
    return [points[index] for index in face.indices]


def projected_face_offset(radius: float, normal_key: tuple[int, int, int]) -> Vec2:
    return project_local(GEOMETRY.face_by_normal[normal_key].center, radius)


def module_extents(center: Vec2, radius: float) -> tuple[float, float, float, float]:
    points = projected_vertices(center, radius)
    xs = [point[0] for point in points]
    zs = [point[1] for point in points]
    return min(xs), max(xs), min(zs), max(zs)


@dataclass(frozen=True)
class SimParams:
    wall_width: float = 3.0
    wall_height: float = 3.0
    dt: float = 0.005
    drone_mass: float = 0.150
    payload_tool_mass: float = 0.100
    gravity: float = rate_control.G
    cage_radius: float = 0.18
    module_gap: float = 0.0
    max_thrust_per_drone: float = 0.300 * rate_control.G
    drone_accel_kp: float = 18.0
    drone_accel_kd: float = 7.5
    spool_velocity_kp: float = 2.8
    max_spool_speed: float = 0.42
    cable_stiffness: float = 260.0
    cable_damping: float = 16.0
    cable_taut_band: float = 0.006
    max_spool_tension: float = 24.0
    spool_support_fraction: float = 0.70
    spool_tension_kv: float = 0.018
    path_speed: float = 0.34
    waypoint_tolerance: float = 0.012
    min_cable_length: float = 0.35
    max_cable_length: float = 3.40
    initial_payload: Vec2 = (0.0, 1.55)

    @property
    def anchor(self) -> Vec2:
        return (0.0, self.wall_height)

    @property
    def total_mass(self) -> float:
        return self.payload_tool_mass + 2.0 * self.drone_mass

    @property
    def hex_face_tilt_rad(self) -> float:
        return math.atan2(1.0, math.sqrt(2.0))


@dataclass
class SimState:
    t: float
    theta: float
    theta_dot: float
    length: float
    length_dot: float
    length_ddot: float
    cable_length: float
    cable_stretch: float
    cable_slack: bool
    cable_tension_saturated: bool
    payload_velocity: Vec2
    payload_acceleration: Vec2
    payload: Vec2
    tool_head: Vec2
    reference: Vec2
    desired_tool_head: Vec2
    reference_velocity: Vec2
    reference_acceleration: Vec2
    target: Vec2
    spool_velocity_cmd: float
    drone_accel_cmd: float
    desired_cable_tension: float
    desired_drone_force: Vec2
    drone_force: Vec2
    cable_force: Vec2
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
        self.cable_length = self._point_to_polar(self.default_target)[1]
        self.cable_stretch = 0.0
        self.cable_slack = False
        self.cable_tension_saturated = False
        self._update_cable_coordinates()
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
        margin = self.params.cage_radius * 1.4
        return (
            clamp(point[0], -self.params.wall_width / 2.0 + margin, self.params.wall_width / 2.0 - margin),
            clamp(point[1], margin, self.params.wall_height - margin),
        )

    def set_target(self, point: Vec2) -> None:
        self.trajectory.command_straight(self._payload_from_state(), self._clamp_wall_point(point))

    def append_target(self, point: Vec2) -> None:
        self.trajectory.append_smooth_waypoint(self._payload_from_state(), self._clamp_wall_point(point))

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

    def _point_to_polar(self, point: Vec2) -> tuple[float, float]:
        dx = point[0] - self.params.anchor[0]
        dz_down = self.params.anchor[1] - point[1]
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

    def _update_cable_coordinates(self) -> None:
        anchor_to_payload = sub2(self.position, self.params.anchor)
        distance = max(1e-9, math.hypot(anchor_to_payload[0], anchor_to_payload[1]))
        e_out = (anchor_to_payload[0] / distance, anchor_to_payload[1] / distance)
        e_theta = (-e_out[1], e_out[0])
        self.length = distance
        self.theta = math.atan2(self.position[0] - self.params.anchor[0], self.params.anchor[1] - self.position[1])
        self.length_dot = dot2(self.velocity, e_out)
        self.theta_dot = dot2(self.velocity, e_theta) / distance
        self.length_ddot = dot2(self.acceleration, e_out)

    def step(self) -> SimState:
        params = self.params
        mass = params.total_mass
        reference = self.trajectory.advance(params.dt)
        _target_theta, target_length, _target_theta_dot, target_length_dot = self._reference_to_polar(reference)

        anchor_to_payload = sub2(self.position, params.anchor)
        distance = max(1e-9, math.hypot(anchor_to_payload[0], anchor_to_payload[1]))
        cable_out = (anchor_to_payload[0] / distance, anchor_to_payload[1] / distance)
        radial_speed = dot2(self.velocity, cable_out)
        current_stretch = distance - self.cable_length
        current_tension_estimate = 0.0
        if current_stretch >= -params.cable_taut_band:
            current_tension_estimate = max(
                0.0,
                params.cable_stiffness * max(0.0, current_stretch)
                + params.cable_damping * radial_speed,
            )
        vertical_per_tension = max(0.05, -cable_out[1])
        desired_cable_vertical_support = params.spool_support_fraction * mass * params.gravity
        desired_cable_tension = clamp(
            desired_cable_vertical_support / vertical_per_tension,
            0.0,
            params.max_spool_tension,
        )
        spool_velocity_cmd = clamp(
            target_length_dot
            + params.spool_velocity_kp * (target_length - self.length)
            - params.spool_tension_kv * (desired_cable_tension - current_tension_estimate),
            -params.max_spool_speed,
            params.max_spool_speed,
        )
        previous_cable_length = self.cable_length
        self.cable_length = clamp(
            self.cable_length + spool_velocity_cmd * params.dt,
            params.min_cable_length,
            params.max_cable_length,
        )
        if self.cable_length in (params.min_cable_length, params.max_cable_length):
            spool_velocity_cmd = (self.cable_length - previous_cable_length) / params.dt

        payload = self._payload_from_state()
        payload_velocity = self._payload_velocity_from_state()
        position_error = sub2(reference.position, payload)
        velocity_error = sub2(reference.velocity, payload_velocity)
        cartesian_accel_cmd = add2(
            reference.acceleration,
            add2(scale2(position_error, params.drone_accel_kp), scale2(velocity_error, params.drone_accel_kd)),
        )

        self.cable_stretch = distance - self.cable_length
        extension_rate = radial_speed - spool_velocity_cmd
        raw_tension = 0.0
        if self.cable_stretch >= -params.cable_taut_band:
            raw_tension = max(
                0.0,
                params.cable_stiffness * max(0.0, self.cable_stretch)
                + params.cable_damping * extension_rate,
            )
        self.cable_tension_saturated = raw_tension > params.max_spool_tension
        tension = clamp(raw_tension, 0.0, params.max_spool_tension)
        self.cable_slack = tension <= 1e-9 and self.cable_stretch < -params.cable_taut_band
        cable_force = scale2(cable_out, -tension)
        gravity_force = (0.0, -mass * params.gravity)

        e_theta = (math.cos(self.theta), math.sin(self.theta))
        drone_accel_cmd = dot2(cartesian_accel_cmd, e_theta)
        left_axis = (math.sin(params.hex_face_tilt_rad), math.cos(params.hex_face_tilt_rad))
        right_axis = (-math.sin(params.hex_face_tilt_rad), math.cos(params.hex_face_tilt_rad))

        desired_net_force = scale2(cartesian_accel_cmd, mass)
        desired_drone_force = sub2(sub2(desired_net_force, cable_force), gravity_force)
        left_thrust, right_thrust, allocation_residual = self._allocate_planar_drone_force(
            desired_drone_force,
            left_axis,
            right_axis,
            params.max_thrust_per_drone,
        )
        drone_force = add2(scale2(left_axis, left_thrust), scale2(right_axis, right_thrust))
        tangential_force = dot2(drone_force, e_theta)
        desired_tangential_force = dot2(desired_drone_force, e_theta)
        saturated = allocation_residual > 0.05
        net_force = add2(add2(drone_force, cable_force), gravity_force)
        self.acceleration = scale2(net_force, 1.0 / mass)
        self.velocity = add2(self.velocity, scale2(self.acceleration, params.dt))
        self.position = add2(self.position, scale2(self.velocity, params.dt))
        self.t += params.dt
        self._update_cable_coordinates()
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
        )
        self.history.append(state)
        if len(self.history) > 6000:
            self.history = self.history[-6000:]
        return state

    @staticmethod
    def _allocate_planar_drone_force(
        desired_force: Vec2,
        left_axis: Vec2,
        right_axis: Vec2,
        max_thrust: float,
    ) -> tuple[float, float, float]:
        """Bounded least-squares allocation for the two tilted drone axes."""

        def residual(left: float, right: float) -> float:
            produced = add2(scale2(left_axis, left), scale2(right_axis, right))
            error = sub2(produced, desired_force)
            return dot2(error, error)

        candidates: list[tuple[float, float]] = []
        determinant = left_axis[0] * right_axis[1] - left_axis[1] * right_axis[0]
        if abs(determinant) > 1e-9:
            left = (desired_force[0] * right_axis[1] - desired_force[1] * right_axis[0]) / determinant
            right = (left_axis[0] * desired_force[1] - left_axis[1] * desired_force[0]) / determinant
            candidates.append((clamp(left, 0.0, max_thrust), clamp(right, 0.0, max_thrust)))

        for left in (0.0, max_thrust):
            remaining = sub2(desired_force, scale2(left_axis, left))
            right = clamp(dot2(right_axis, remaining), 0.0, max_thrust)
            candidates.append((left, right))
        for right in (0.0, max_thrust):
            remaining = sub2(desired_force, scale2(right_axis, right))
            left = clamp(dot2(left_axis, remaining), 0.0, max_thrust)
            candidates.append((left, right))
        for left in (0.0, max_thrust):
            for right in (0.0, max_thrust):
                candidates.append((left, right))

        best_left, best_right = min(candidates, key=lambda pair: residual(pair[0], pair[1]))
        return best_left, best_right, math.sqrt(residual(best_left, best_right))

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
            cable_length=self.cable_length,
            cable_stretch=self.cable_stretch,
            cable_slack=self.cable_slack,
            cable_tension_saturated=self.cable_tension_saturated,
            payload_velocity=self.velocity,
            payload_acceleration=self.acceleration,
            payload=payload,
            tool_head=tool_head,
            reference=reference.position,
            desired_tool_head=desired_tool_head,
            reference_velocity=reference.velocity,
            reference_acceleration=reference.acceleration,
            target=reference.final_target,
            spool_velocity_cmd=spool_velocity_cmd,
            drone_accel_cmd=drone_accel_cmd,
            desired_cable_tension=desired_cable_tension,
            desired_drone_force=desired_drone_force,
            drone_force=drone_force,
            cable_force=cable_force,
            left_thrust=left_thrust,
            right_thrust=right_thrust,
            tension=tension,
            tangential_force=tangential_force,
            desired_tangential_force=desired_tangential_force,
            allocation_residual=distance2(drone_force, desired_drone_force),
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
        self.face_collection = PolyCollection(
            [],
            closed=True,
            facecolors=face_color,
            edgecolors="none",
            alpha=fill_alpha,
            zorder=zorder,
        )
        self.edge_collection = LineCollection([], colors=edge_color, linewidths=1.35, zorder=zorder + 1)
        ax.add_collection(self.face_collection)
        ax.add_collection(self.edge_collection)
        self.nodes, = ax.plot([], [], "o", color="#d62728", markersize=2.4, zorder=zorder + 2)
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

    def update(self, center: Vec2) -> None:
        self.face_collection.set_verts(projected_faces(center, self.radius))
        self.edge_collection.set_segments(projected_edges(center, self.radius))
        points = projected_vertices(center, self.radius)
        self.nodes.set_data([point[0] for point in points], [point[1] for point in points])
        self.label.set_position(center)


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

        self.fig = plt.figure(figsize=(12.5, 8.0), constrained_layout=False)
        grid = self.fig.add_gridspec(
            2,
            2,
            width_ratios=[1.0, 0.34],
            height_ratios=[1.0, 0.18],
            left=0.055,
            right=0.975,
            bottom=0.08,
            top=0.92,
            wspace=0.08,
            hspace=0.18,
        )
        self.ax = self.fig.add_subplot(grid[0, 0])
        self.panel_ax = self.fig.add_subplot(grid[0, 1])
        self.control_ax = self.fig.add_subplot(grid[1, :])
        self.control_ax.axis("off")
        self.panel_ax.axis("off")
        self.fig.suptitle("PRISMS Cable-Suspended Wall Tool Simulator", fontsize=14)

        self._build_scene()
        self._build_panel()
        self._build_controls()
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
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
        self.desired_trace_line, = self.ax.plot([], [], color="#d62728", linewidth=1.8, linestyle=":", alpha=0.90, zorder=2)
        self.path_line, = self.ax.plot([], [], color="#d62728", linewidth=1.5, linestyle="--", alpha=0.72, zorder=4)
        self.reference_point, = self.ax.plot([], [], marker="o", color="#1f77b4", markersize=5.0, zorder=9)
        self.waypoint_points, = self.ax.plot([], [], marker="x", linestyle="none", color="#d62728", markersize=7.0, mew=1.8, zorder=9)
        self.target_point, = self.ax.plot([], [], marker="+", color="#d62728", markersize=12, mew=2.2, zorder=9)
        self.tool_line, = self.ax.plot([], [], marker="o", linestyle="none", color="#8a4f00", markersize=6.0, zorder=13)

        radius = params.cage_radius
        self.payload_artist = ModuleArtist(self.ax, radius, "#f2cc60", "#5c4512", "", 0.58, 6)
        self.left_artist = ModuleArtist(self.ax, radius, "#f7f7f7", "black", "", 0.16, 6)
        self.right_artist = ModuleArtist(self.ax, radius, "#f7f7f7", "black", "", 0.16, 6)

        self.dock_polygons = [
            Polygon([(0.0, 0.0)] * 6, closed=True, fill=False, edgecolor="#d62728", linewidth=2.5, zorder=11)
            for _ in range(4)
        ]
        for polygon in self.dock_polygons:
            self.ax.add_patch(polygon)

        self.left_axis_guide, = self.ax.plot([], [], color="#777777", linestyle="--", linewidth=1.0, zorder=10)
        self.right_axis_guide, = self.ax.plot([], [], color="#777777", linestyle="--", linewidth=1.0, zorder=10)
        self.left_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=14, color="#1f77b4", zorder=12)
        self.right_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=14, color="#1f77b4", zorder=12)
        self.gravity_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=15, color="#333333", zorder=12)
        self.tension_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=14, color="#6a3d9a", zorder=12)
        for arrow in (self.left_arrow, self.right_arrow, self.gravity_arrow, self.tension_arrow):
            self.ax.add_patch(arrow)

    def _build_panel(self) -> None:
        self.panel_ax.text(0.0, 0.98, "System State", fontsize=12, fontweight="bold", va="top")
        self.state_text = self.panel_ax.text(
            0.0,
            0.90,
            "",
            fontsize=8.5,
            family="monospace",
            va="top",
            linespacing=1.18,
        )

    def _build_controls(self) -> None:
        self.play_ax = self.fig.add_axes([0.055, 0.026, 0.080, 0.038])
        self.reset_ax = self.fig.add_axes([0.145, 0.026, 0.070, 0.038])
        self.clear_ax = self.fig.add_axes([0.225, 0.026, 0.070, 0.038])
        self.append_ax = self.fig.add_axes([0.305, 0.026, 0.105, 0.038])
        self.speed_ax = self.fig.add_axes([0.490, 0.035, 0.230, 0.024])
        self.check_ax = self.fig.add_axes([0.765, 0.000, 0.170, 0.092])

        self.play_button = Button(self.play_ax, "Pause")
        self.reset_button = Button(self.reset_ax, "Reset")
        self.clear_button = Button(self.clear_ax, "Clear")
        self.append_button = Button(self.append_ax, "Append Off")
        self.speed_slider = Slider(self.speed_ax, "speed", 0.25, 4.0, valinit=1.0)
        self.checks = CheckButtons(self.check_ax, ["trace", "target", "path", "forces"], [True, True, True, True])

        self.play_button.on_clicked(self.toggle_play)
        self.reset_button.on_clicked(self.reset)
        self.clear_button.on_clicked(self.clear_trace)
        self.append_button.on_clicked(self.toggle_append)
        self.checks.on_clicked(self.toggle_layer)

    def module_centers(self, payload: Vec2) -> tuple[Vec2, Vec2]:
        radius = self.params.cage_radius
        left_payload_offset = projected_face_offset(radius, PAYLOAD_LEFT_HEX)
        left_drone_offset = projected_face_offset(radius, DRONE_RIGHT_HEX)
        right_payload_offset = projected_face_offset(radius, PAYLOAD_RIGHT_HEX)
        right_drone_offset = projected_face_offset(radius, DRONE_LEFT_HEX)

        left_gap = scale2(normalize2(left_payload_offset), self.params.module_gap)
        right_gap = scale2(normalize2(right_payload_offset), self.params.module_gap)
        left_center = sub2(add2(add2(payload, left_payload_offset), left_gap), left_drone_offset)
        right_center = sub2(add2(add2(payload, right_payload_offset), right_gap), right_drone_offset)
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
            self.waypoint_points.set_data(
                [point[0] for point in pending_path[1:]],
                [point[1] for point in pending_path[1:]],
            )
        else:
            self.path_line.set_data([], [])
            self.reference_point.set_data([], [])
            self.waypoint_points.set_data([], [])
        if self.show_target:
            self.target_point.set_data([state.target[0]], [state.target[1]])
        else:
            self.target_point.set_data([], [])

        left_center, right_center = self.module_centers(state.payload)
        self.payload_artist.update(state.payload)
        self.left_artist.update(left_center)
        self.right_artist.update(right_center)

        dock_faces = [
            projected_face_polygon(state.payload, radius, PAYLOAD_LEFT_HEX),
            projected_face_polygon(left_center, radius, DRONE_RIGHT_HEX),
            projected_face_polygon(state.payload, radius, PAYLOAD_RIGHT_HEX),
            projected_face_polygon(right_center, radius, DRONE_LEFT_HEX),
        ]
        for polygon, face_points in zip(self.dock_polygons, dock_faces):
            polygon.set_xy(face_points)

        _min_x, _max_x, min_z, max_z = module_extents(state.payload, radius)
        cable_mount = (x, max_z)
        self.cable_line.set_data([params.anchor[0], cable_mount[0]], [params.anchor[1], cable_mount[1]])
        self.cable_line.set_linestyle("--" if state.cable_slack else "-")
        self.tool_line.set_data([state.tool_head[0]], [state.tool_head[1]])

        left_axis = (math.sin(params.hex_face_tilt_rad), math.cos(params.hex_face_tilt_rad))
        right_axis = (-math.sin(params.hex_face_tilt_rad), math.cos(params.hex_face_tilt_rad))
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

        tilt_deg = math.degrees(params.hex_face_tilt_rad)
        self.state_text.set_text(
            f"t                  {state.t:6.2f} s\n"
            f"tool x,z           {state.tool_head[0]:+5.2f}, {state.tool_head[1]:+5.2f} m\n"
            f"ref x,z            {state.reference[0]:+5.2f}, {state.reference[1]:+5.2f} m\n"
            f"ref ax,az          {state.reference_acceleration[0]:+5.2f}, {state.reference_acceleration[1]:+5.2f} m/s2\n"
            f"target x,z         {state.target[0]:+5.2f}, {state.target[1]:+5.2f} m\n"
            f"tool error         {state.tool_error:6.3f} m\n"
            f"theta              {math.degrees(state.theta):+6.2f} deg\n"
            f"distance           {state.length:6.2f} m\n"
            f"spool length       {state.cable_length:6.2f} m\n"
            f"cable stretch      {state.cable_stretch:+6.3f} m\n"
            f"spool v cmd        {state.spool_velocity_cmd:+6.2f} m/s\n"
            f"drone accel cmd    {state.drone_accel_cmd:+6.2f} m/s2\n"
            f"gravity            {params.total_mass * params.gravity:6.2f} N\n"
            f"max thrust/drone   {params.max_thrust_per_drone:6.2f} N\n"
            f"drone support      {state.drone_vertical_force:6.2f} N\n"
            f"cable support      {state.cable_vertical_force:6.2f} N\n"
            f"cable target T     {state.desired_cable_tension:6.2f} N\n"
            f"cable tension      {state.tension:6.2f} N\n"
            f"left thrust        {state.left_thrust:6.2f} N\n"
            f"right thrust       {state.right_thrust:6.2f} N\n"
            f"axis tilt          +/-{tilt_deg:5.2f} deg\n"
            f"waypoints          {state.active_waypoints:6d}\n"
            f"click mode         {'append' if self.append_mode else 'single'}\n"
            f"traj mode          {self.sim.trajectory.mode:>8}\n"
            f"force residual     {state.allocation_residual:+6.2f} N\n"
            f"drone saturated    {'YES' if state.saturated else 'no'}\n"
            f"cable slack        {'YES' if state.cable_slack else 'no'}\n"
            f"tension saturated  {'YES' if state.cable_tension_saturated else 'no'}"
        )

    @staticmethod
    def _set_arrow(arrow: FancyArrowPatch, start: Vec2, end: Vec2) -> None:
        arrow.set_positions(start, end)

    def animate(self, _frame: int):
        if self.playing:
            speed = float(self.speed_slider.val)
            frame_dt = 0.04
            steps = max(1, int(round(speed * frame_dt / self.params.dt)))
            for _ in range(steps):
                self.sim.step()
            self.draw()
        return []

    def on_click(self, event) -> None:
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        clicked_point = (float(event.xdata), float(event.ydata))
        if self.append_mode:
            self.sim.append_target(clicked_point)
        else:
            self.sim.set_target(clicked_point)
        self.playing = True
        self.play_button.label.set_text("Pause")
        self.draw()
        self.fig.canvas.draw_idle()

    def toggle_play(self, _event) -> None:
        self.playing = not self.playing
        self.play_button.label.set_text("Pause" if self.playing else "Play")

    def reset(self, _event) -> None:
        self.sim.reset()
        self.playing = False
        self.append_mode = False
        self.play_button.label.set_text("Play")
        self.append_button.label.set_text("Append Off")
        self.draw()
        self.fig.canvas.draw_idle()

    def clear_trace(self, _event) -> None:
        self.sim.clear_trajectory()
        self.sim.history = self.sim.history[-1:]
        self.draw()
        self.fig.canvas.draw_idle()

    def toggle_append(self, _event) -> None:
        self.append_mode = not self.append_mode
        self.append_button.label.set_text("Append On" if self.append_mode else "Append Off")

    def toggle_layer(self, label: str) -> None:
        if label == "trace":
            self.show_trace = not self.show_trace
        elif label == "target":
            self.show_target = not self.show_target
        elif label == "path":
            self.show_path = not self.show_path
        elif label == "forces":
            self.show_forces = not self.show_forces
        self.draw()
        self.fig.canvas.draw_idle()


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
