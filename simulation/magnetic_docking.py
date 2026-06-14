#!/usr/bin/env python3
"""Connector-level magnetic docking helpers for reusable drone modules.

Open or build a CoppeliaSim scene containing one or more copies of
model/truncated_octahedral_crazyflie_plant.ttm. This module discovers loaded
drone models, reconstructs the cage docking geometry, and applies magnetic
capture/latch forces between compatible cage faces.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.low_level import body_rate as flight  # noqa: E402

GENERATION_SCRIPTS = PROJECT_ROOT / "scripts" / "generation"
if str(GENERATION_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(GENERATION_SCRIPTS))

import generate_drone_plant_scene as plant  # noqa: E402


MODEL_ALIAS = "truncated_octahedral_crazyflie"
DEFAULT_CONNECTOR_CONTACT_DISTANCE = 0.004
DEFAULT_LATCH_DISTANCE = 0.007

Vector3 = tuple[float, float, float]
LatchKey = tuple[int, int, int, int]


@dataclass
class DroneAgent:
    index: int
    path: str
    body: int
    joints: list[int]
    start_position: Vector3
    start_orientation: Vector3
    state: flight.ControllerState


@dataclass
class WorldDrone:
    origin: Vector3
    matrix: list[float]
    linear_velocity: Vector3
    angular_velocity: Vector3
    connector_position: list[Vector3]
    connector_velocity: list[Vector3]


@dataclass(frozen=True)
class DockFace:
    index: int
    connector_ids: tuple[int, ...]
    face_type: str
    center: Vector3
    normal: Vector3


@dataclass(frozen=True)
class DockingGeometry:
    connectors: list[Vector3]
    faces: list[DockFace]


@dataclass(frozen=True)
class FaceDockCandidate:
    face_a: DockFace
    face_b: DockFace
    pairs: tuple[LatchKey, ...]
    pair_distances: tuple[float, ...]
    max_distance: float
    mean_distance: float
    center_distance: float
    normal_dot: float
    max_relative_speed: float


@dataclass
class DockingMemory:
    latched: set[LatchKey] = field(default_factory=set)
    latched_at: dict[LatchKey, float] = field(default_factory=dict)
    cooldown_until: dict[LatchKey, float] = field(default_factory=dict)
    last_break_reason: str = ""
    last_latch_event: str = ""


def add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(vector: Vector3, gain: float) -> Vector3:
    return (vector[0] * gain, vector[1] * gain, vector[2] * gain)


def dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(vector: Vector3) -> float:
    return math.sqrt(dot(vector, vector))


def unit(vector: Vector3) -> Vector3:
    length = norm(vector)
    if length < 1e-12:
        raise ValueError("Cannot normalize zero-length vector.")
    return scale(vector, 1.0 / length)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def matrix_translation(matrix: list[float]) -> Vector3:
    return (matrix[3], matrix[7], matrix[11])


def transform_point(matrix: list[float], point: Vector3) -> Vector3:
    return (
        matrix[0] * point[0] + matrix[1] * point[1] + matrix[2] * point[2] + matrix[3],
        matrix[4] * point[0] + matrix[5] * point[1] + matrix[6] * point[2] + matrix[7],
        matrix[8] * point[0] + matrix[9] * point[1] + matrix[10] * point[2] + matrix[11],
    )


def transform_vector(matrix: list[float], vector: Vector3) -> Vector3:
    return (
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[4] * vector[0] + matrix[5] * vector[1] + matrix[6] * vector[2],
        matrix[8] * vector[0] + matrix[9] * vector[1] + matrix[10] * vector[2],
    )


def point_velocity(linear_velocity: Vector3, angular_velocity: Vector3, origin: Vector3, point: Vector3) -> Vector3:
    return add(linear_velocity, cross(angular_velocity, sub(point, origin)))


def rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 60.0 / (2.0 * math.pi)


def object_alias(sim, handle: int) -> str:
    return str(sim.getObjectAlias(handle, 1))


def all_scene_objects(sim) -> list[int]:
    handles = []
    index = 0
    while True:
        handle = sim.getObjects(index, sim.handle_all)
        if handle < 0:
            break
        handles.append(handle)
        index += 1
    return handles


def alias_tail(path: str) -> str:
    return path.rsplit("/", 1)[-1]


def has_alias_base(tail: str, base: str) -> bool:
    return tail == base or tail.startswith(f"{base}#") or tail.startswith(f"{base}[")


def is_drone_root_path(path: str) -> bool:
    return has_alias_base(alias_tail(path), MODEL_ALIAS)


def descendant_by_alias_base(sim, root: int, base_alias: str) -> int:
    descendants = sim.getObjectsInTree(root, sim.handle_all, 0)
    for handle in descendants:
        tail = alias_tail(object_alias(sim, handle))
        if has_alias_base(tail, base_alias):
            return handle
    raise RuntimeError(f"Could not find descendant alias {base_alias}.")


def blank_state(initial_speed: float = 0.0) -> flight.ControllerState:
    return flight.ControllerState(
        rate_integral=[0.0, 0.0, 0.0],
        motor_speed=[initial_speed, initial_speed, initial_speed, initial_speed],
        prop_phase=[0.0, 0.0, 0.0, 0.0],
    )


def connector_locations(body_stl: Path) -> list[Vector3]:
    vertices, _edges = plant.derive_cage_collision_graph(body_stl)
    return [(float(vertex[0]), float(vertex[1]), float(vertex[2])) for vertex in vertices]


def face_type_from_count(count: int) -> str:
    if count == 4:
        return "square"
    if count == 6:
        return "hex"
    return f"{count}_corner"


def ordered_face_vertices(vertices: list[Vector3], connector_ids: tuple[int, ...], normal: Vector3) -> tuple[int, ...]:
    center = tuple(
        sum(vertices[index][axis] for index in connector_ids) / len(connector_ids)
        for axis in range(3)
    )
    reference = (1.0, 0.0, 0.0) if abs(dot(normal, (1.0, 0.0, 0.0))) < 0.9 else (0.0, 1.0, 0.0)
    axis_u = unit(cross(normal, reference))
    axis_v = cross(normal, axis_u)
    return tuple(
        sorted(
            connector_ids,
            key=lambda index: math.atan2(
                dot(sub(vertices[index], center), axis_v),
                dot(sub(vertices[index], center), axis_u),
            ),
        )
    )


def derive_docking_faces(connectors: list[Vector3]) -> list[DockFace]:
    plane_tolerance = 0.002
    faces_by_connectors: dict[tuple[int, ...], DockFace] = {}
    for i, j, k in itertools.combinations(range(len(connectors)), 3):
        try:
            normal = unit(cross(sub(connectors[j], connectors[i]), sub(connectors[k], connectors[i])))
        except ValueError:
            continue
        plane_offset = dot(normal, connectors[i])
        signed_distances = [dot(normal, point) - plane_offset for point in connectors]
        max_distance = max(signed_distances)
        min_distance = min(signed_distances)
        if max_distance > plane_tolerance and min_distance < -plane_tolerance:
            continue

        connector_ids = tuple(index for index, distance in enumerate(signed_distances) if abs(distance) <= plane_tolerance)
        if len(connector_ids) < 4:
            continue
        if max_distance <= plane_tolerance:
            outward_normal = normal
        else:
            outward_normal = scale(normal, -1.0)

        center = tuple(sum(connectors[index][axis] for index in connector_ids) / len(connector_ids) for axis in range(3))
        if dot(center, outward_normal) < 0.0:
            outward_normal = scale(outward_normal, -1.0)
        ordered_ids = ordered_face_vertices(connectors, tuple(sorted(connector_ids)), outward_normal)
        key = tuple(sorted(ordered_ids))
        if key in faces_by_connectors:
            continue
        faces_by_connectors[key] = DockFace(
            index=len(faces_by_connectors),
            connector_ids=ordered_ids,
            face_type=face_type_from_count(len(ordered_ids)),
            center=center,
            normal=outward_normal,
        )

    faces = sorted(faces_by_connectors.values(), key=lambda face: (len(face.connector_ids), face.center))
    return [
        DockFace(
            index=index,
            connector_ids=face.connector_ids,
            face_type=face.face_type,
            center=face.center,
            normal=face.normal,
        )
        for index, face in enumerate(faces)
    ]


def docking_geometry(body_stl: Path) -> DockingGeometry:
    connectors = connector_locations(body_stl)
    faces = derive_docking_faces(connectors)
    expected_counts = {4: 6, 6: 8}
    observed_counts: dict[int, int] = defaultdict(int)
    for face in faces:
        observed_counts[len(face.connector_ids)] += 1
    for vertex_count, expected in expected_counts.items():
        observed = observed_counts.get(vertex_count, 0)
        if observed != expected:
            raise ValueError(f"Expected {expected} docking faces with {vertex_count} corners, found {observed}.")
    return DockingGeometry(connectors=connectors, faces=faces)


def discover_drones(sim, initial_motor_speed: float) -> list[DroneAgent]:
    drones = []
    for handle in all_scene_objects(sim):
        path = object_alias(sim, handle)
        if not is_drone_root_path(path):
            continue
        joints = [
            descendant_by_alias_base(sim, handle, f"propeller_{index}_spin_joint")
            for index in range(4)
        ]
        position = tuple(float(value) for value in sim.getObjectPosition(handle, -1))
        orientation = tuple(float(value) for value in sim.getObjectOrientation(handle, -1))
        drones.append(
            DroneAgent(
                index=len(drones),
                path=path,
                body=handle,
                joints=joints,
                start_position=position,
                start_orientation=orientation,
                state=blank_state(initial_motor_speed),
            )
        )
    return drones


def reset_drone(sim, drone: DroneAgent, initial_motor_speed: float) -> None:
    sim.setObjectPosition(drone.body, -1, list(drone.start_position))
    sim.setObjectOrientation(drone.body, -1, list(drone.start_orientation))
    sim.resetDynamicObject(drone.body)
    drone.state = blank_state(initial_motor_speed)


def compute_world_drones(sim, drones: list[DroneAgent], connectors: list[Vector3]) -> dict[int, WorldDrone]:
    world = {}
    for drone in drones:
        matrix = sim.getObjectMatrix(drone.body, -1)
        origin = matrix_translation(matrix)
        linear_velocity_raw, angular_velocity_raw = sim.getVelocity(drone.body)
        linear_velocity = tuple(float(value) for value in linear_velocity_raw)
        angular_velocity = tuple(float(value) for value in angular_velocity_raw)
        positions = [transform_point(matrix, connector) for connector in connectors]
        velocities = [
            point_velocity(linear_velocity, angular_velocity, origin, position)
            for position in positions
        ]
        world[drone.index] = WorldDrone(
            origin=origin,
            matrix=matrix,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            connector_position=positions,
            connector_velocity=velocities,
        )
    return world


def latch_key(a_index: int, a_connector: int, b_index: int, b_connector: int) -> LatchKey:
    if a_index < b_index:
        return (a_index, a_connector, b_index, b_connector)
    return (b_index, b_connector, a_index, a_connector)


def spring_force(
    p_a: Vector3,
    v_a: Vector3,
    p_b: Vector3,
    v_b: Vector3,
    rest_distance: float,
    stiffness: float,
    damping: float,
    force_limit: float,
) -> tuple[Vector3, float, float, float]:
    delta = sub(p_b, p_a)
    distance = norm(delta)
    if distance < 1e-9:
        return (0.0, 0.0, 0.0), distance, 0.0, 0.0
    direction = scale(delta, 1.0 / distance)
    relative_speed = dot(sub(v_b, v_a), direction)
    requested_force = stiffness * (distance - rest_distance) + damping * relative_speed
    applied_force = clamp(requested_force, -force_limit, force_limit)
    return scale(direction, applied_force), distance, relative_speed, requested_force


def add_force(
    force_accumulator: dict[int, Vector3],
    torque_accumulator: dict[int, Vector3],
    world: dict[int, WorldDrone],
    drone_index: int,
    point: Vector3,
    force: Vector3,
) -> None:
    force_accumulator[drone_index] = add(force_accumulator[drone_index], force)
    torque_accumulator[drone_index] = add(
        torque_accumulator[drone_index],
        cross(sub(point, world[drone_index].origin), force),
    )


def force_for_pair(
    world: dict[int, WorldDrone],
    key: LatchKey,
    rest_distance: float,
    stiffness: float,
    damping: float,
    force_limit: float,
) -> tuple[Vector3, float, float, float]:
    a_index, a_connector, b_index, b_connector = key
    p_a = world[a_index].connector_position[a_connector]
    v_a = world[a_index].connector_velocity[a_connector]
    p_b = world[b_index].connector_position[b_connector]
    v_b = world[b_index].connector_velocity[b_connector]
    return spring_force(p_a, v_a, p_b, v_b, rest_distance, stiffness, damping, force_limit)


def clear_docking_memory(
    sim,
    memory: DockingMemory,
    clear_cooldowns: bool = True,
    break_reason: str = "",
) -> None:
    memory.latched.clear()
    memory.latched_at.clear()
    if clear_cooldowns:
        memory.cooldown_until.clear()
    memory.last_break_reason = break_reason
    memory.last_latch_event = ""


def world_face_center(world_drone: WorldDrone, face: DockFace) -> Vector3:
    return tuple(
        sum(world_drone.connector_position[index][axis] for index in face.connector_ids) / len(face.connector_ids)
        for axis in range(3)
    )


def world_face_normal(world_drone: WorldDrone, face: DockFace) -> Vector3:
    return unit(transform_vector(world_drone.matrix, face.normal))


def best_face_corner_pairs(
    world: dict[int, WorldDrone],
    a_index: int,
    face_a: DockFace,
    b_index: int,
    face_b: DockFace,
) -> tuple[tuple[LatchKey, ...], tuple[float, ...], float, float, float]:
    count = len(face_a.connector_ids)
    best_pairs: tuple[LatchKey, ...] = ()
    best_distances: tuple[float, ...] = ()
    best_max_distance = float("inf")
    best_mean_distance = float("inf")
    best_max_relative_speed = 0.0
    for direction in (1, -1):
        for shift in range(count):
            pairs = []
            distances = []
            relative_speeds = []
            for a_offset, a_connector in enumerate(face_a.connector_ids):
                b_connector = face_b.connector_ids[(shift + direction * a_offset) % count]
                key = latch_key(a_index, a_connector, b_index, b_connector)
                p_a = world[a_index].connector_position[a_connector]
                p_b = world[b_index].connector_position[b_connector]
                v_a = world[a_index].connector_velocity[a_connector]
                v_b = world[b_index].connector_velocity[b_connector]
                delta = sub(p_b, p_a)
                distance = norm(delta)
                if distance > 1e-9:
                    relative_speed = dot(sub(v_b, v_a), scale(delta, 1.0 / distance))
                else:
                    relative_speed = 0.0
                pairs.append(key)
                distances.append(distance)
                relative_speeds.append(abs(relative_speed))
            max_distance = max(distances)
            mean_distance = sum(distances) / count
            if (max_distance, mean_distance) < (best_max_distance, best_mean_distance):
                best_pairs = tuple(pairs)
                best_distances = tuple(distances)
                best_max_distance = max_distance
                best_mean_distance = mean_distance
                best_max_relative_speed = max(relative_speeds)
    return best_pairs, best_distances, best_max_distance, best_mean_distance, best_max_relative_speed


def compatible_face_candidate(
    world: dict[int, WorldDrone],
    a_index: int,
    face_a: DockFace,
    b_index: int,
    face_b: DockFace,
    args: argparse.Namespace,
) -> FaceDockCandidate | None:
    if len(face_a.connector_ids) != len(face_b.connector_ids):
        return None
    if face_a.face_type != face_b.face_type:
        return None
    if len(face_a.connector_ids) > args.max_magnet_pairs_per_drone_pair:
        return None

    normal_a = world_face_normal(world[a_index], face_a)
    normal_b = world_face_normal(world[b_index], face_b)
    normal_dot = dot(normal_a, normal_b)
    max_normal_dot = -math.cos(math.radians(args.face_normal_tolerance_deg))
    if normal_dot > max_normal_dot:
        return None

    center_a = world_face_center(world[a_index], face_a)
    center_b = world_face_center(world[b_index], face_b)
    center_distance = norm(sub(center_b, center_a))
    if center_distance > args.face_center_tolerance:
        return None

    pairs, pair_distances, max_distance, mean_distance, max_relative_speed = best_face_corner_pairs(
        world,
        a_index,
        face_a,
        b_index,
        face_b,
    )
    if max_distance > args.capture_radius:
        return None

    return FaceDockCandidate(
        face_a=face_a,
        face_b=face_b,
        pairs=pairs,
        pair_distances=pair_distances,
        max_distance=max_distance,
        mean_distance=mean_distance,
        center_distance=center_distance,
        normal_dot=normal_dot,
        max_relative_speed=max_relative_speed,
    )


def face_latch_ready(candidate: FaceDockCandidate, args: argparse.Namespace) -> bool:
    required_fraction = clamp(args.face_latch_required_fraction, 0.0, 1.0)
    required_count = max(1, math.ceil(len(candidate.pairs) * required_fraction))
    close_count = sum(1 for distance in candidate.pair_distances if distance <= args.latch_distance)
    speed_ok = candidate.max_relative_speed <= args.latch_speed
    return close_count >= required_count and speed_ok


def apply_magnetic_docking(
    sim,
    drones: list[DroneAgent],
    geometry: DockingGeometry,
    memory: DockingMemory,
    args: argparse.Namespace,
    magnets_enabled: bool,
) -> dict[str, object]:
    if not magnets_enabled or len(drones) < 2:
        if not magnets_enabled and memory.latched:
            clear_docking_memory(sim, memory, clear_cooldowns=False, break_reason="magnets off")
        return {
            "mode": "magnets_off" if not magnets_enabled else "single_drone",
            "capture_pairs": 0,
            "latched_pairs": len(memory.latched),
            "broken_pairs": 0,
            "min_distance": 0.0,
            "max_distance": 0.0,
            "max_force": 0.0,
            "latch_contacts": len(memory.latched),
            "last_break_reason": memory.last_break_reason,
            "last_latch_event": memory.last_latch_event,
        }

    now = sim.getSimulationTime()
    connectors = geometry.connectors
    world = compute_world_drones(sim, drones, connectors)
    force_accumulator = defaultdict(lambda: (0.0, 0.0, 0.0))
    torque_accumulator = defaultdict(lambda: (0.0, 0.0, 0.0))
    reserved_connectors: set[tuple[int, int]] = set()
    distances: list[float] = []
    max_force = 0.0
    broken_pairs = 0
    capture_pairs = 0
    memory.last_latch_event = ""

    for key in list(memory.latched):
        a_index, a_connector, b_index, b_connector = key
        if a_index >= len(drones) or b_index >= len(drones):
            memory.latched.remove(key)
            memory.latched_at.pop(key, None)
            continue
        distance = norm(sub(world[b_index].connector_position[b_connector], world[a_index].connector_position[a_connector]))
        ramp_time = max(0.0, float(args.latch_stiffness_ramp_time))
        if ramp_time > 0.0:
            latch_age = max(0.0, now - memory.latched_at.get(key, now))
            latch_gain = clamp(latch_age / ramp_time, 0.0, 1.0)
        else:
            latch_gain = 1.0
        force_a, distance, _speed, requested_force = force_for_pair(
            world,
            key,
            args.latch_rest_distance,
            latch_gain * args.latch_stiffness,
            latch_gain * args.latch_damping,
            max(0.0, latch_gain * args.latch_force_limit),
        )
        force_magnitude = norm(force_a)
        tensile_load = max(0.0, requested_force)
        distances.append(distance)
        if tensile_load > args.connector_break_force or distance > args.latch_break_distance:
            memory.latched.remove(key)
            memory.latched_at.pop(key, None)
            memory.cooldown_until[key] = now + args.relatch_delay
            memory.last_break_reason = (
                f"d{a_index}:c{a_connector} to d{b_index}:c{b_connector} "
                f"broke at load={tensile_load:.2f} N, d={distance:.3f} m"
            )
            broken_pairs += 1
            continue
        max_force = max(max_force, force_magnitude)
        add_force(force_accumulator, torque_accumulator, world, a_index, world[a_index].connector_position[a_connector], force_a)
        add_force(force_accumulator, torque_accumulator, world, b_index, world[b_index].connector_position[b_connector], scale(force_a, -1.0))
        reserved_connectors.add((a_index, a_connector))
        reserved_connectors.add((b_index, b_connector))

    used_connectors = set(reserved_connectors)
    if args.face_docking:
        face_candidates: list[FaceDockCandidate] = []
        for drone_a, drone_b in itertools.combinations(drones, 2):
            for face_a in geometry.faces:
                if any((drone_a.index, connector) in reserved_connectors for connector in face_a.connector_ids):
                    continue
                for face_b in geometry.faces:
                    if any((drone_b.index, connector) in reserved_connectors for connector in face_b.connector_ids):
                        continue
                    candidate = compatible_face_candidate(world, drone_a.index, face_a, drone_b.index, face_b, args)
                    if candidate is None:
                        continue
                    if any(key in memory.latched or memory.cooldown_until.get(key, 0.0) > now for key in candidate.pairs):
                        continue
                    face_candidates.append(candidate)

        face_candidates.sort(key=lambda candidate: (candidate.max_distance, candidate.center_distance, candidate.mean_distance))
        selected_drone_pairs: set[tuple[int, int]] = set()
        for candidate in face_candidates:
            if not candidate.pairs:
                continue
            a_index, _a_connector, b_index, _b_connector = candidate.pairs[0]
            drone_pair = (a_index, b_index)
            if drone_pair in selected_drone_pairs:
                continue
            if any((key[0], key[1]) in used_connectors or (key[2], key[3]) in used_connectors for key in candidate.pairs):
                continue

            selected_drone_pairs.add(drone_pair)
            face_latch_keys: list[LatchKey] = []
            for key in candidate.pairs:
                a_index, a_connector, b_index, b_connector = key
                force_a, distance, _relative_speed, _requested_force = force_for_pair(
                    world,
                    key,
                    args.magnet_rest_distance,
                    args.magnet_stiffness,
                    args.magnet_damping,
                    args.magnet_force_limit,
                )
                force_magnitude = norm(force_a)
                distances.append(distance)
                max_force = max(max_force, force_magnitude)
                capture_pairs += 1
                used_connectors.add((a_index, a_connector))
                used_connectors.add((b_index, b_connector))
                add_force(force_accumulator, torque_accumulator, world, a_index, world[a_index].connector_position[a_connector], force_a)
                add_force(force_accumulator, torque_accumulator, world, b_index, world[b_index].connector_position[b_connector], scale(force_a, -1.0))
                face_latch_keys.append(key)

            if face_latch_ready(candidate, args):
                for key in face_latch_keys:
                    memory.latched.add(key)
                    memory.latched_at[key] = now
                memory.last_latch_event = (
                    f"d{drone_pair[0]}:{candidate.face_a.face_type} face {candidate.face_a.index} "
                    f"latched to d{drone_pair[1]}:{candidate.face_b.face_type} face {candidate.face_b.index} "
                    f"with {len(face_latch_keys)} corners"
                )
    else:
        candidates: list[tuple[float, LatchKey]] = []
        for drone_a, drone_b in itertools.combinations(drones, 2):
            for connector_a in range(len(connectors)):
                if (drone_a.index, connector_a) in reserved_connectors:
                    continue
                p_a = world[drone_a.index].connector_position[connector_a]
                for connector_b in range(len(connectors)):
                    if (drone_b.index, connector_b) in reserved_connectors:
                        continue
                    key = latch_key(drone_a.index, connector_a, drone_b.index, connector_b)
                    if key in memory.latched:
                        continue
                    if memory.cooldown_until.get(key, 0.0) > now:
                        continue
                    distance = norm(sub(world[drone_b.index].connector_position[connector_b], p_a))
                    if distance <= args.capture_radius:
                        candidates.append((distance, key))

        candidates.sort(key=lambda item: item[0])
        selected_by_drone_pair: dict[tuple[int, int], int] = defaultdict(int)
        for _distance, key in candidates:
            a_index, a_connector, b_index, b_connector = key
            drone_pair = (a_index, b_index)
            if selected_by_drone_pair[drone_pair] >= args.max_magnet_pairs_per_drone_pair:
                continue
            if (a_index, a_connector) in used_connectors or (b_index, b_connector) in used_connectors:
                continue

            force_a, distance, relative_speed, _requested_force = force_for_pair(
                world,
                key,
                args.magnet_rest_distance,
                args.magnet_stiffness,
                args.magnet_damping,
                args.magnet_force_limit,
            )
            force_magnitude = norm(force_a)
            distances.append(distance)
            max_force = max(max_force, force_magnitude)
            capture_pairs += 1
            selected_by_drone_pair[drone_pair] += 1
            used_connectors.add((a_index, a_connector))
            used_connectors.add((b_index, b_connector))

            add_force(force_accumulator, torque_accumulator, world, a_index, world[a_index].connector_position[a_connector], force_a)
            add_force(force_accumulator, torque_accumulator, world, b_index, world[b_index].connector_position[b_connector], scale(force_a, -1.0))

            speed_ok = abs(relative_speed) <= args.latch_speed
            if distance <= args.latch_distance and speed_ok:
                memory.latched.add(key)
                memory.latched_at[key] = now
                memory.last_latch_event = f"d{a_index}:c{a_connector} latched to d{b_index}:c{b_connector}"

    for drone in drones:
        force = force_accumulator[drone.index]
        torque = torque_accumulator[drone.index]
        if norm(force) > 0.0 or norm(torque) > 0.0:
            sim.addForceAndTorque(drone.body, list(force), list(torque))

    if broken_pairs:
        mode = "broken"
    elif memory.latched:
        mode = "latched"
    elif capture_pairs:
        mode = "magnetic_capture"
    else:
        mode = "free"

    return {
        "mode": mode,
        "capture_pairs": capture_pairs,
        "latched_pairs": len(memory.latched),
        "broken_pairs": broken_pairs,
        "min_distance": min(distances) if distances else 0.0,
        "max_distance": max(distances) if distances else 0.0,
        "max_force": max_force,
        "latch_contacts": len(memory.latched),
        "last_break_reason": memory.last_break_reason,
        "last_latch_event": memory.last_latch_event,
    }
