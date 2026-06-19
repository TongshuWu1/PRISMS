#!/usr/bin/env python3
"""Spawn two reusable drone plants in a hex-face contact configuration.

The saved scene contains two independent drone models. The live Python docking
controller still owns latch state; when the UI starts with magnets enabled, this
geometry should satisfy the face gate and latch immediately. If the geometry
does not satisfy the configured tolerances, the controller will show that
failure through the docking status instead of using a hidden fallback.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
GENERATION_DIR = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(GENERATION_DIR) not in sys.path:
    sys.path.insert(0, str(GENERATION_DIR))

import generate_drone_plant_scene as plant  # noqa: E402
from controller.low_level import rate_control  # noqa: E402
from simulation import magnetic_docking as multi  # noqa: E402


MODEL_PATH = PROJECT_ROOT / "model" / "truncated_octahedral_crazyflie_plant.ttm"
BODY_STL = PROJECT_ROOT / "assets" / "meshes" / "crazyflie_cage_body_no_propellers.stl"
SCENE_OUTPUT = PROJECT_ROOT / "scene" / "hex_face_pair_scene.ttt"
MODEL_ALIAS = plant.MODEL_ALIAS
DEFAULT_FACE_A_INDEX = 13
DEFAULT_FACE_B_INDEX = 7

Vector3 = tuple[float, float, float]
Matrix3 = list[list[float]]


@dataclass(frozen=True)
class FacePairing:
    direction: int
    shift: int
    connector_pairs: tuple[tuple[int, int], ...]
    distances: tuple[float, ...]
    max_distance: float
    mean_distance: float


@dataclass(frozen=True)
class HexPlacement:
    face_a: multi.DockFace
    face_b: multi.DockFace
    rotation_a: Matrix3
    rotation_b: Matrix3
    spin_a: float
    spin_b: float
    pairing: FacePairing
    contact_normal_world: Vector3
    a_body_z_world: Vector3
    b_body_z_world: Vector3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Spawn a hex-to-hex tilted two-drone test scene.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--model", default=str(MODEL_PATH), help="Reusable drone plant .ttm to load.")
    parser.add_argument("--body-stl", default=str(BODY_STL), help="Cage STL used to derive connector geometry.")
    parser.add_argument("--scene-output", default=str(SCENE_OUTPUT), help="Where to save the hex-pair scene.")
    parser.add_argument("--save-scene", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clear-existing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--refresh-model", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--collision-rod-diameter", type=float, default=None)
    parser.add_argument("--collision-node-diameter", type=float, default=None)
    parser.add_argument("--height", type=float, default=0.45, help="Drone A center height [m].")
    parser.add_argument(
        "--contact-distance",
        type=float,
        default=multi.DEFAULT_CONNECTOR_CONTACT_DISTANCE,
        help="Face-center separation along drone A's outward face normal [m].",
    )
    parser.add_argument(
        "--face-a-index",
        type=int,
        default=DEFAULT_FACE_A_INDEX,
        help="Specific hex face index for drone A.",
    )
    parser.add_argument(
        "--face-b-index",
        type=int,
        default=DEFAULT_FACE_B_INDEX,
        help="Specific hex face index for drone B.",
    )
    parser.add_argument(
        "--auto-face-pair",
        action="store_true",
        help="Search for a tilted hex pair instead of using the explicit face indices.",
    )
    parser.add_argument(
        "--min-module-tilt-deg",
        type=float,
        default=10.0,
        help="Minimum tilt of both module body +Z axes from world +Z for automatic face-pair selection.",
    )
    parser.add_argument(
        "--max-pairing-error",
        type=float,
        default=0.002 * rate_control.DEFAULT_GEOMETRY_SCALE,
        help="Maximum allowed in-plane connector mismatch after face alignment [m].",
    )
    return parser.parse_args()


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def cross(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    )


def norm(vector: Sequence[float]) -> float:
    return math.sqrt(dot(vector, vector))


def unit(vector: Sequence[float]) -> Vector3:
    length = norm(vector)
    if length < 1e-12:
        raise ValueError("Cannot normalize a zero-length vector.")
    return (float(vector[0]) / length, float(vector[1]) / length, float(vector[2]) / length)


def scale(vector: Sequence[float], gain: float) -> Vector3:
    return (float(vector[0]) * gain, float(vector[1]) * gain, float(vector[2]) * gain)


def add(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2]))


def sub(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2]))


def mat_vec(matrix: Matrix3, vector: Sequence[float]) -> Vector3:
    return (
        matrix[0][0] * float(vector[0]) + matrix[0][1] * float(vector[1]) + matrix[0][2] * float(vector[2]),
        matrix[1][0] * float(vector[0]) + matrix[1][1] * float(vector[1]) + matrix[1][2] * float(vector[2]),
        matrix[2][0] * float(vector[0]) + matrix[2][1] * float(vector[1]) + matrix[2][2] * float(vector[2]),
    )


def mat_mul(a: Matrix3, b: Matrix3) -> Matrix3:
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def rotation_axis_angle(axis: Sequence[float], angle: float) -> Matrix3:
    x, y, z = unit(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    c1 = 1.0 - c
    return [
        [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
        [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
        [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
    ]


def rotation_from_to(source: Sequence[float], target: Sequence[float]) -> Matrix3:
    a = unit(source)
    b = unit(target)
    c = max(-1.0, min(1.0, dot(a, b)))
    if c > 1.0 - 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    if c < -1.0 + 1e-12:
        reference = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
        return rotation_axis_angle(cross(a, reference), math.pi)

    v = cross(a, b)
    vx = [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
    vx2 = mat_mul(vx, vx)
    gain = 1.0 / (1.0 + c)
    return [
        [
            (1.0 if row == col else 0.0) + vx[row][col] + gain * vx2[row][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def object_matrix(rotation: Matrix3, translation: Sequence[float]) -> list[float]:
    return [
        rotation[0][0],
        rotation[0][1],
        rotation[0][2],
        float(translation[0]),
        rotation[1][0],
        rotation[1][1],
        rotation[1][2],
        float(translation[1]),
        rotation[2][0],
        rotation[2][1],
        rotation[2][2],
        float(translation[2]),
    ]


def vector_string(vector: Sequence[float]) -> str:
    return "[" + ", ".join(f"{float(value): .4f}" for value in vector) + "]"


def refresh_model(args: argparse.Namespace) -> None:
    if not args.refresh_model:
        return
    command = [
        sys.executable,
        str(GENERATION_DIR / "generate_drone_plant_scene.py"),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--connect-timeout",
        str(args.connect_timeout),
        "--model",
        str(MODEL_PATH),
    ]
    if args.collision_rod_diameter is not None:
        command.extend(["--collision-rod-diameter", str(args.collision_rod_diameter)])
    if args.collision_node_diameter is not None:
        command.extend(["--collision-node-diameter", str(args.collision_node_diameter)])
    print("Refreshing single-drone plant model...", flush=True)
    subprocess.run(command, cwd=PROJECT_ROOT, check=True)


def generated_drone_objects(sim) -> list[int]:
    objects = []
    prefixes = (f"/{MODEL_ALIAS}", "/position_target")
    for handle in plant.all_scene_objects(sim):
        if plant.object_alias(sim, handle).startswith(prefixes):
            objects.append(handle)
    return objects


def clear_existing_drones(sim) -> None:
    to_remove = generated_drone_objects(sim)
    if to_remove:
        sim.removeObjects(to_remove, False)
        print(f"Removed {len(to_remove)} old drone/target objects.")


def hex_face_by_index(geometry: multi.DockingGeometry, index: int) -> multi.DockFace:
    for face in geometry.faces:
        if face.index == index and face.face_type == "hex":
            return face
    raise ValueError(f"Face index {index} is not a hex face.")


def best_face_pairing(
    connectors: list[Vector3],
    face_a: multi.DockFace,
    face_b: multi.DockFace,
    rotation_a: Matrix3,
    rotation_b: Matrix3,
) -> FacePairing:
    rotated_center_a = mat_vec(rotation_a, face_a.center)
    offsets_a = [
        sub(mat_vec(rotation_a, connectors[index]), rotated_center_a)
        for index in face_a.connector_ids
    ]
    rotated_center_b = mat_vec(rotation_b, face_b.center)
    offsets_b = [
        sub(mat_vec(rotation_b, connectors[index]), rotated_center_b)
        for index in face_b.connector_ids
    ]
    best: FacePairing | None = None
    count = len(face_a.connector_ids)
    for direction in (1, -1):
        for shift in range(count):
            distances = [
                norm(sub(offsets_a[index], offsets_b[(shift + direction * index) % count]))
                for index in range(count)
            ]
            connector_pairs = tuple(
                (
                    face_a.connector_ids[index],
                    face_b.connector_ids[(shift + direction * index) % count],
                )
                for index in range(count)
            )
            candidate = FacePairing(
                direction=direction,
                shift=shift,
                connector_pairs=connector_pairs,
                distances=tuple(distances),
                max_distance=max(distances),
                mean_distance=sum(distances) / count,
            )
            if best is None or (candidate.max_distance, candidate.mean_distance) < (
                best.max_distance,
                best.mean_distance,
            ):
                best = candidate
    if best is None:
        raise RuntimeError("Could not evaluate hex connector pairing.")
    return best


def hex_placement_for_pair(
    connectors: list[Vector3],
    face_a: multi.DockFace,
    face_b: multi.DockFace,
    contact_normal_world: Vector3,
) -> HexPlacement:
    contact_normal = unit(contact_normal_world)
    rotation_a = rotation_from_to(face_a.normal, contact_normal)
    rotation_b = rotation_from_to(face_b.normal, scale(contact_normal, -1.0))
    pairing = best_face_pairing(connectors, face_a, face_b, rotation_a, rotation_b)
    return HexPlacement(
        face_a=face_a,
        face_b=face_b,
        rotation_a=rotation_a,
        rotation_b=rotation_b,
        spin_a=0.0,
        spin_b=0.0,
        pairing=pairing,
        contact_normal_world=contact_normal,
        a_body_z_world=mat_vec(rotation_a, (0.0, 0.0, 1.0)),
        b_body_z_world=mat_vec(rotation_b, (0.0, 0.0, 1.0)),
    )


def choose_hex_placement(
    geometry: multi.DockingGeometry,
    face_a_index: int | None,
    face_b_index: int | None,
    max_pairing_error: float,
    min_module_tilt_deg: float,
    contact_normal_world: Vector3 = (1.0, 0.0, 0.0),
) -> HexPlacement:
    connectors = geometry.connectors
    contact_normal = unit(contact_normal_world)
    if (face_a_index is None) != (face_b_index is None):
        raise ValueError("--face-a-index and --face-b-index must be specified together.")
    if face_a_index is not None and face_b_index is not None:
        face_a = hex_face_by_index(geometry, face_a_index)
        face_b = hex_face_by_index(geometry, face_b_index)
        placement = hex_placement_for_pair(connectors, face_a, face_b, contact_normal)
        if placement.pairing.max_distance > max_pairing_error:
            raise RuntimeError(
                f"Requested hex pair in-plane error {placement.pairing.max_distance:.6f} m "
                f"exceeds --max-pairing-error {max_pairing_error:.6f} m."
            )
        return placement

    min_tilt_cos = math.cos(math.radians(max(0.0, min_module_tilt_deg)))
    candidates: list[tuple[tuple[float, float, int, int], HexPlacement]] = []
    hex_faces = [face for face in geometry.faces if face.face_type == "hex"]
    for face_a in hex_faces:
        for face_b in hex_faces:
            placement = hex_placement_for_pair(connectors, face_a, face_b, contact_normal)
            if placement.pairing.max_distance > max_pairing_error:
                continue
            a_z = placement.a_body_z_world
            b_z = placement.b_body_z_world
            if dot(a_z, contact_normal) <= 0.0 or dot(b_z, contact_normal) >= 0.0:
                continue
            if a_z[2] <= 0.0 or b_z[2] <= 0.0:
                continue
            if a_z[2] > min_tilt_cos or b_z[2] > min_tilt_cos:
                continue
            symmetry = norm(add(a_z, b_z))
            score = (placement.pairing.max_distance, symmetry, face_a.index, face_b.index)
            candidates.append((score, placement))
    if not candidates:
        raise RuntimeError(
            "No tilted hex-face pair satisfies the connector alignment and tilt constraints. "
            "Relax --max-pairing-error or --min-module-tilt-deg if this is intentional."
        )
    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def load_drone(sim, model_path: Path, rotation: Matrix3, position: Sequence[float], index: int) -> int:
    handle = sim.loadModel(str(model_path))
    if isinstance(handle, (list, tuple)):
        handle = handle[0]
    if handle < 0:
        raise RuntimeError(f"CoppeliaSim failed to load model: {model_path}")

    sim.setObjectAlias(handle, MODEL_ALIAS, 1)
    sim.setObjectMatrix(handle, -1, object_matrix(rotation, position))
    sim.resetDynamicObject(handle)
    print(
        f"Spawned drone {index}: handle={handle}, alias={plant.object_alias(sim, handle)}, "
        f"pos={vector_string(position)}"
    )
    return int(handle)


def body_axes_from_object_matrix(rotation: Matrix3, position: Sequence[float]) -> tuple[Vector3, Vector3, Vector3]:
    matrix = object_matrix(rotation, position)
    return (
        (matrix[0], matrix[4], matrix[8]),
        (matrix[1], matrix[5], matrix[9]),
        (matrix[2], matrix[6], matrix[10]),
    )


def transformed_connector_positions(
    connectors: list[Vector3],
    rotation: Matrix3,
    position: Sequence[float],
) -> list[Vector3]:
    return [add(mat_vec(rotation, connector), position) for connector in connectors]


def predicted_pair_distances(
    connectors: list[Vector3],
    placement: HexPlacement,
    position_a: Sequence[float],
    position_b: Sequence[float],
) -> list[float]:
    points_a = transformed_connector_positions(connectors, placement.rotation_a, position_a)
    points_b = transformed_connector_positions(connectors, placement.rotation_b, position_b)
    return [
        norm(sub(points_a[a_connector], points_b[b_connector]))
        for a_connector, b_connector in placement.pairing.connector_pairs
    ]


def controller_motor_axes(rotation: Matrix3, position: Sequence[float]) -> tuple[Vector3, list[Vector3]]:
    _body_x, _body_y, body_z = body_axes_from_object_matrix(rotation, position)
    motor_positions = [add(mat_vec(rotation, motor["pos"]), position) for motor in rate_control.MOTORS]
    return body_z, motor_positions


def validate_controller_axis_mapping(
    placement: HexPlacement,
    position_a: Sequence[float],
    position_b: Sequence[float],
) -> None:
    controller_a_z, _motor_positions_a = controller_motor_axes(placement.rotation_a, position_a)
    controller_b_z, _motor_positions_b = controller_motor_axes(placement.rotation_b, position_b)
    axis_error_a = norm(sub(controller_a_z, placement.a_body_z_world))
    axis_error_b = norm(sub(controller_b_z, placement.b_body_z_world))
    if max(axis_error_a, axis_error_b) > 1e-9:
        raise RuntimeError(
            "Controller body-axis mapping does not match generated rotation: "
            f"axis_error_a={axis_error_a:.3e}, axis_error_b={axis_error_b:.3e}."
        )


def main() -> int:
    args = parse_args()
    model_path = Path(args.model)
    body_stl = Path(args.body_stl)
    if not body_stl.exists():
        raise FileNotFoundError(body_stl)

    refresh_model(args)
    if not model_path.exists():
        raise FileNotFoundError(
            f"{model_path}\nRun: python scripts\\generation\\generate_drone_plant_scene.py"
        )

    geometry = multi.docking_geometry(body_stl)
    placement = choose_hex_placement(
        geometry,
        None if args.auto_face_pair else args.face_a_index,
        None if args.auto_face_pair else args.face_b_index,
        args.max_pairing_error,
        args.min_module_tilt_deg,
    )

    position_a = (0.0, 0.0, float(args.height))
    face_center_a_world = add(position_a, mat_vec(placement.rotation_a, placement.face_a.center))
    position_b = add(
        sub(face_center_a_world, mat_vec(placement.rotation_b, placement.face_b.center)),
        scale(placement.contact_normal_world, float(args.contact_distance)),
    )
    validate_controller_axis_mapping(placement, position_a, position_b)
    connector_distances = predicted_pair_distances(
        geometry.connectors,
        placement,
        position_a,
        position_b,
    )
    axis_a, motor_positions_a = controller_motor_axes(placement.rotation_a, position_a)
    axis_b, motor_positions_b = controller_motor_axes(placement.rotation_b, position_b)

    sim = plant.connect(args)
    plant.stop_if_running(sim)
    if args.clear_existing:
        clear_existing_drones(sim)

    handles = [
        load_drone(sim, model_path, placement.rotation_a, position_a, 0),
        load_drone(sim, model_path, placement.rotation_b, position_b, 1),
    ]
    sim.setObjectSel(handles)

    if args.save_scene:
        scene_output = Path(args.scene_output)
        scene_output.parent.mkdir(parents=True, exist_ok=True)
        sim.saveScene(str(scene_output))
        print(f"Saved hex-face pair scene: {scene_output}")

    tilt_a_deg = math.degrees(math.acos(max(-1.0, min(1.0, placement.a_body_z_world[2]))))
    tilt_b_deg = math.degrees(math.acos(max(-1.0, min(1.0, placement.b_body_z_world[2]))))
    face_a_normal_world = mat_vec(placement.rotation_a, placement.face_a.normal)
    face_b_normal_world = mat_vec(placement.rotation_b, placement.face_b.normal)
    print("Hex pair geometry:")
    print(
        f"  drone A hex face {placement.face_a.index} normal={vector_string(placement.face_a.normal)} "
        f"center={vector_string(placement.face_a.center)}"
    )
    print(
        f"  drone B hex face {placement.face_b.index} normal={vector_string(placement.face_b.normal)} "
        f"center={vector_string(placement.face_b.center)}"
    )
    print(f"  world face normal d0={vector_string(face_a_normal_world)}")
    print(f"  world face normal d1={vector_string(face_b_normal_world)}")
    print(f"  drone A body +Z in world={vector_string(placement.a_body_z_world)} tilt={tilt_a_deg:.2f} deg")
    print(f"  drone B body +Z in world={vector_string(placement.b_body_z_world)} tilt={tilt_b_deg:.2f} deg")
    print(f"  controller motor axis d0={vector_string(axis_a)}")
    print(f"  controller motor axis d1={vector_string(axis_b)}")
    print(
        f"  spin_a={math.degrees(placement.spin_a):.2f} deg "
        f"spin_b={math.degrees(placement.spin_b):.2f} deg "
        f"in_plane_error_max={placement.pairing.max_distance:.6f} m "
        f"mean={placement.pairing.mean_distance:.6f} m"
    )
    print(
        "  connector pairs="
        + ", ".join(f"a{a}:b{b}" for a, b in placement.pairing.connector_pairs)
    )
    print(
        "  predicted connector distances="
        + "[" + ", ".join(f"{distance:.6f}" for distance in connector_distances) + "] m"
    )
    print(
        "  d0 motor positions="
        + "[" + ", ".join(vector_string(position) for position in motor_positions_a) + "]"
    )
    print(
        "  d1 motor positions="
        + "[" + ", ".join(vector_string(position) for position in motor_positions_b) + "]"
    )
    print("Next: run scripts\\launchers\\run_hex_pair_position_ui.py from PyCharm.")
    time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
