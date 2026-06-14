#!/usr/bin/env python3
"""Generate a colored CoppeliaSim model for the pre-docked square-face pair.

This configuration is visual/inspection only. It mirrors the single-drone
coloring workflow: split the STL into CoppeliaSim mesh shapes, then apply
colors with sim.setShapeColor().
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path


CONFIG_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = CONFIG_ROOT.parents[1]
ASSET_DIR = PROJECT_ROOT / "assets" / "meshes"
DEFAULT_BODY_STL = ASSET_DIR / "crazyflie_cage_body_no_propellers.stl"
DEFAULT_PROP_STL = ASSET_DIR / "crazyflie_propellers_aligned.stl"
OUTPUT_MODEL = CONFIG_ROOT / "model" / "predocked_square_face_pair.ttm"
OUTPUT_SCENE = CONFIG_ROOT / "scene" / "predocked_square_face_pair_inspection.ttt"
OUTPUT_METADATA = CONFIG_ROOT / "model" / "predocked_square_face_pair_metadata.json"

LOCAL_COPPELIASIM_CLIENT = Path(
    r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu"
    r"\programming\zmqRemoteApi\clients\python\src"
)
if LOCAL_COPPELIASIM_CLIENT.exists():
    sys.path.insert(0, str(LOCAL_COPPELIASIM_CLIENT))

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing CoppeliaSim remote API client. Run:\n"
        "  python -m pip install -r requirements.txt"
    ) from exc


MODEL_ALIAS = "predocked_square_face_pair"
GENERATED_PREFIXES = (f"/{MODEL_ALIAS}",)
PROPELLERS = (
    {
        "name": "propeller_ccw_1",
        "pos": (0.029886, -0.031685, 0.012625),
    },
    {
        "name": "propeller_ccw_2",
        "pos": (-0.033400, 0.031601, 0.012625),
    },
    {
        "name": "propeller_cw_1",
        "pos": (0.029886, 0.031601, 0.012625),
    },
    {
        "name": "propeller_cw_2",
        "pos": (-0.033400, -0.031685, 0.012625),
    },
)

BODY_COLOR = [0.62, 0.68, 0.60]
CAGE_COLOR = [0.005, 0.005, 0.005]
CONNECTOR_COLOR = [0.95, 0.02, 0.01]
FORWARD_PROPELLER_COLOR = [0.95, 0.02, 0.01]
REAR_PROPELLER_COLOR = [0.03, 0.03, 0.03]

Triangle = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate the colored CoppeliaSim pre-docked pair model.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--body-stl", default=str(DEFAULT_BODY_STL))
    parser.add_argument("--propeller-stl", default=str(DEFAULT_PROP_STL))
    parser.add_argument("--model", default=str(OUTPUT_MODEL))
    parser.add_argument("--scene", default=str(OUTPUT_SCENE))
    parser.add_argument("--metadata", default=str(OUTPUT_METADATA))
    parser.add_argument("--gap", type=float, default=0.001, help="Face-to-face clearance [m].")
    parser.add_argument("--height", type=float, default=0.5, help="Inspection height in CoppeliaSim [m].")
    parser.add_argument(
        "--docking-face",
        choices=("pos_xy_to_neg_xy", "pos_x_neg_y_to_neg_x_pos_y"),
        default="pos_xy_to_neg_xy",
        help="Opposing vertical square/diamond faces used for face-to-face docking.",
    )
    parser.add_argument("--square-face-support", type=float, default=0.080, help="Square face plane distance from cage center [m].")
    return parser.parse_args()


def connect(args: argparse.Namespace):
    print(f"Connecting to CoppeliaSim ZMQ remote API at {args.host}:{args.port} ...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(2.0)
        if sock.connect_ex((args.host, args.port)) != 0:
            raise RuntimeError(
                f"CoppeliaSim is not listening at {args.host}:{args.port}. "
                "Start CoppeliaSim and wait for the ZMQ remote API server."
            )
    client = RemoteAPIClient(host=args.host, port=args.port)
    client.initialTimeout = int(args.connect_timeout)
    return client.require("sim")


def object_alias(sim, handle: int) -> str:
    try:
        return str(sim.getObjectAlias(handle, 1))
    except Exception:
        return ""


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


def stop_if_running(sim) -> None:
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation(True)
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)


def remove_previous_generated(sim) -> None:
    to_remove = []
    for handle in all_scene_objects(sim):
        if object_alias(sim, handle).startswith(GENERATED_PREFIXES):
            to_remove.append(handle)
    if to_remove:
        sim.removeObjects(to_remove, False)
        print(f"Removed {len(to_remove)} previous pre-docked pair objects.")


def hide_object(sim, handle: int) -> None:
    sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, 0)


def set_respondable(sim, handle: int, enabled: bool) -> None:
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1 if enabled else 0)


def read_stl_triangles(path: Path) -> list[Triangle]:
    data = path.read_bytes()
    tri_count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if 84 + tri_count * 50 == len(data):
        triangles: list[Triangle] = []
        offset = 84
        for _ in range(tri_count):
            offset += 12
            values = struct.unpack_from("<9f", data, offset)
            offset += 38
            triangles.append(
                (
                    (values[0], values[1], values[2]),
                    (values[3], values[4], values[5]),
                    (values[6], values[7], values[8]),
                )
            )
        return triangles

    triangles = []
    vertices = []
    for raw_line in data.decode("utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line.startswith("vertex "):
            continue
        _, x, y, z = line.split()
        vertices.append((float(x), float(y), float(z)))
        if len(vertices) == 3:
            triangles.append((vertices[0], vertices[1], vertices[2]))
            vertices = []
    if not triangles:
        raise ValueError(f"Could not read STL triangles: {path}")
    return triangles


def connected_stl_components(path: Path) -> list[dict[str, object]]:
    triangles = read_stl_triangles(path)
    vertex_ids: dict[tuple[float, float, float], int] = {}
    vertices: list[tuple[float, float, float]] = []
    triangle_vertex_ids = []

    for triangle in triangles:
        ids = []
        for vertex in triangle:
            key = tuple(round(value, 6) for value in vertex)
            index = vertex_ids.get(key)
            if index is None:
                index = len(vertices)
                vertex_ids[key] = index
                vertices.append(key)
            ids.append(index)
        triangle_vertex_ids.append(ids)

    parent = list(range(len(vertices)))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(a: int, b: int) -> None:
        root_a = find(a)
        root_b = find(b)
        if root_a != root_b:
            parent[root_b] = root_a

    for a, b, c in triangle_vertex_ids:
        union(a, b)
        union(a, c)

    grouped: dict[int, list[int]] = {}
    for triangle_index, ids in enumerate(triangle_vertex_ids):
        grouped.setdefault(find(ids[0]), []).append(triangle_index)

    components = []
    for triangle_indices in grouped.values():
        points = [vertices[v] for t in triangle_indices for v in triangle_vertex_ids[t]]
        minimum = [min(point[i] for point in points) for i in range(3)]
        maximum = [max(point[i] for point in points) for i in range(3)]
        size = [maximum[i] - minimum[i] for i in range(3)]
        center = [(minimum[i] + maximum[i]) / 2.0 for i in range(3)]
        components.append(
            {
                "triangles": [triangles[index] for index in triangle_indices],
                "triangle_count": len(triangle_indices),
                "size": size,
                "center": center,
            }
        )
    return components


def classify_visual_component(component: dict[str, object]) -> str:
    triangle_count = int(component["triangle_count"])
    size = component["size"]
    center = component["center"]
    max_size = max(size)
    center_radius = math.sqrt(center[0] ** 2 + center[1] ** 2 + center[2] ** 2)
    if triangle_count >= 80 and max_size <= 0.011 and center_radius >= 0.025:
        return "joint"
    if triangle_count <= 220 and center_radius >= 0.018:
        return "cage"
    return "body"


def mesh_bounds(triangles: list[Triangle]) -> dict[str, list[float]]:
    points = [point for triangle in triangles for point in triangle]
    minimum = [min(point[index] for point in points) for index in range(3)]
    maximum = [max(point[index] for point in points) for index in range(3)]
    return {
        "min": minimum,
        "max": maximum,
        "size": [maximum[index] - minimum[index] for index in range(3)],
        "center": [(maximum[index] + minimum[index]) * 0.5 for index in range(3)],
    }


def transform_point(point: tuple[float, float, float], translation: tuple[float, float, float]) -> tuple[float, float, float]:
    return (point[0] + translation[0], point[1] + translation[1], point[2] + translation[2])


def transform_triangles(triangles: list[Triangle], translation: tuple[float, float, float]) -> list[Triangle]:
    return [
        (
            transform_point(triangle[0], translation),
            transform_point(triangle[1], translation),
            transform_point(triangle[2], translation),
        )
        for triangle in triangles
    ]


def triangles_to_mesh(triangles: list[Triangle]) -> tuple[list[float], list[int]]:
    vertices: list[float] = []
    indices: list[int] = []
    vertex_index: dict[tuple[float, float, float], int] = {}
    for triangle in triangles:
        for vertex in triangle:
            key = tuple(round(value, 9) for value in vertex)
            index = vertex_index.get(key)
            if index is None:
                index = len(vertex_index)
                vertex_index[key] = index
                vertices.extend(key)
            indices.append(index)
    return vertices, indices


def split_propeller_triangles(path: Path) -> dict[str, list[Triangle]]:
    centers = {prop["name"]: tuple(prop["pos"]) for prop in PROPELLERS}
    grouped = {prop["name"]: [] for prop in PROPELLERS}
    for triangle in read_stl_triangles(path):
        centroid = tuple(sum(vertex[i] for vertex in triangle) / 3.0 for i in range(3))
        nearest = min(centers, key=lambda name: sum((centroid[i] - centers[name][i]) ** 2 for i in range(3)))
        grouped[nearest].append(triangle)
    for name, triangles in grouped.items():
        if not triangles:
            raise ValueError(f"No propeller triangles assigned to {name}.")
    return grouped


def make_mesh_shape(sim, parent: int, alias: str, triangles: list[Triangle], color: list[float]) -> int | None:
    if not triangles:
        return None
    vertices, indices = triangles_to_mesh(triangles)
    shape = sim.createShape(0, math.radians(20.0), vertices, indices)
    sim.setObjectAlias(shape, alias, 1)
    sim.setShapeColor(shape, None, sim.colorcomponent_ambient_diffuse, color)
    sim.setObjectInt32Param(shape, sim.shapeintparam_static, 1)
    set_respondable(sim, shape, False)
    sim.setObjectParent(shape, parent, True)
    sim.setObjectPosition(shape, parent, [0.0, 0.0, 0.0])
    sim.setObjectOrientation(shape, parent, [0.0, 0.0, 0.0])
    return shape


def docking_face_normal(face: str) -> tuple[float, float, float]:
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    if face == "pos_xy_to_neg_xy":
        return (inv_sqrt2, inv_sqrt2, 0.0)
    if face == "pos_x_neg_y_to_neg_x_pos_y":
        return (inv_sqrt2, -inv_sqrt2, 0.0)
    raise ValueError(f"Unsupported docking face: {face}")


def rounded(values: list[float] | tuple[float, ...]) -> list[float]:
    return [round(value, 9) for value in values]


def create_drone_visual(
    sim,
    root: int,
    label: str,
    translation: tuple[float, float, float],
    body_categories: dict[str, list[Triangle]],
    propeller_groups: dict[str, list[Triangle]],
) -> int:
    drone = sim.createDummy(0.002)
    sim.setObjectAlias(drone, label, 1)
    sim.setObjectParent(drone, root, True)
    sim.setObjectPosition(drone, root, [0.0, 0.0, 0.0])
    sim.setObjectOrientation(drone, root, [0.0, 0.0, 0.0])
    hide_object(sim, drone)

    make_mesh_shape(sim, drone, f"{label}_crazyflie_body_visual", transform_triangles(body_categories["body"], translation), BODY_COLOR)
    make_mesh_shape(sim, drone, f"{label}_cage_rods_black_visual", transform_triangles(body_categories["cage"], translation), CAGE_COLOR)
    make_mesh_shape(sim, drone, f"{label}_cage_corner_joints_red_visual", transform_triangles(body_categories["joint"], translation), CONNECTOR_COLOR)
    for index, propeller in enumerate(PROPELLERS):
        is_forward = propeller["pos"][0] > 0.0
        color = FORWARD_PROPELLER_COLOR if is_forward else REAR_PROPELLER_COLOR
        direction_label = "front_red" if is_forward else "rear_black"
        make_mesh_shape(
            sim,
            drone,
            f"{label}_propeller_{index}_{direction_label}_visual",
            transform_triangles(propeller_groups[propeller["name"]], translation),
            color,
        )
    return drone


def save_outputs(sim, root: int, model_path: Path, scene_path: Path) -> None:
    model_path.parent.mkdir(parents=True, exist_ok=True)
    scene_path.parent.mkdir(parents=True, exist_ok=True)
    sim.saveModel(root, str(model_path))
    sim.saveScene(str(scene_path))


def main() -> int:
    args = parse_args()
    body_stl = Path(args.body_stl)
    prop_stl = Path(args.propeller_stl)
    if not body_stl.exists():
        raise FileNotFoundError(body_stl)
    if not prop_stl.exists():
        raise FileNotFoundError(prop_stl)
    if args.gap < 0.0:
        raise ValueError("--gap must be non-negative.")

    body_categories = {"body": [], "cage": [], "joint": []}
    for component in connected_stl_components(body_stl):
        body_categories[classify_visual_component(component)].extend(component["triangles"])
    propeller_groups = split_propeller_triangles(prop_stl)
    propeller_triangles = [triangle for triangles in propeller_groups.values() for triangle in triangles]

    normal = docking_face_normal(args.docking_face)
    center_spacing = 2.0 * args.square_face_support + args.gap
    center_offset = 0.5 * center_spacing
    translation_a = (-center_offset * normal[0], -center_offset * normal[1], args.height)
    translation_b = (center_offset * normal[0], center_offset * normal[1], args.height)

    sim = connect(args)
    stop_if_running(sim)
    remove_previous_generated(sim)

    root = sim.createDummy(0.003)
    sim.setObjectAlias(root, MODEL_ALIAS, 1)
    sim.setObjectPosition(root, -1, [0.0, 0.0, 0.0])
    sim.setObjectOrientation(root, -1, [0.0, 0.0, 0.0])
    hide_object(sim, root)
    try:
        sim.setModelProperty(root, sim.getModelProperty(root) & ~sim.modelproperty_not_model)
    except Exception:
        sim.setModelProperty(root, 0)

    create_drone_visual(sim, root, "drone_a", translation_a, body_categories, propeller_groups)
    create_drone_visual(sim, root, "drone_b", translation_b, body_categories, propeller_groups)
    sim.setObjectSel([root])

    model_path = Path(args.model)
    scene_path = Path(args.scene)
    metadata_path = Path(args.metadata)
    save_outputs(sim, root, model_path, scene_path)

    body_a_all = transform_triangles(
        body_categories["body"] + body_categories["cage"] + body_categories["joint"],
        translation_a,
    )
    body_b_all = transform_triangles(
        body_categories["body"] + body_categories["cage"] + body_categories["joint"],
        translation_b,
    )
    full_triangles = body_a_all + body_b_all + transform_triangles(propeller_triangles, translation_a) + transform_triangles(propeller_triangles, translation_b)
    metadata = {
        "configuration": "square_face_predocked_pair",
        "description": "Two upright complete drone modules, diagonally placed through opposing vertical square/diamond faces.",
        "outputs": {
            "model_ttm": str(model_path),
            "scene_ttt": str(scene_path),
        },
        "color_method": "CoppeliaSim sim.setShapeColor, matching the single-drone generator.",
        "colors_rgb": {
            "crazyflie_body": BODY_COLOR,
            "carbon_fiber_cage_rods": CAGE_COLOR,
            "corner_connectors": CONNECTOR_COLOR,
            "forward_propellers_local_pos_x": FORWARD_PROPELLER_COLOR,
            "rear_propellers_local_neg_x": REAR_PROPELLER_COLOR,
        },
        "forward_direction_convention": "Each drone's local +X direction is forward. Propellers with positive local x are red.",
        "forward_propeller_indices": [
            index for index, propeller in enumerate(PROPELLERS) if propeller["pos"][0] > 0.0
        ],
        "rear_propeller_indices": [
            index for index, propeller in enumerate(PROPELLERS) if propeller["pos"][0] <= 0.0
        ],
        "source_body_stl": str(body_stl),
        "source_propeller_stl": str(prop_stl),
        "docking_face": args.docking_face,
        "docking_face_normal_world": rounded(normal),
        "gap_m": args.gap,
        "square_face_support_m": args.square_face_support,
        "height_m": args.height,
        "drone_a_translation_m": rounded(translation_a),
        "drone_b_translation_m": rounded(translation_b),
        "measured_square_face_gap_m": center_spacing - 2.0 * args.square_face_support,
        "pair_full_bounds_m": {key: rounded(value) for key, value in mesh_bounds(full_triangles).items()},
        "notes": [
            "Inspection-only geometry: no controller, no latch, no dynamics.",
            "Both modules remain upright at nominal attitude.",
            "The chosen docking faces are actual vertical square faces of the truncated-octahedral cage.",
            "The modules are placed diagonally in XY so A's +face normal opposes B's -face normal.",
        ],
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    print("Saved colored CoppeliaSim square-face pre-docked pair:")
    print(f"  model:    {model_path}")
    print(f"  scene:    {scene_path}")
    print(f"  metadata: {metadata_path}")
    print(f"  docking face: {args.docking_face}, gap={args.gap * 1000.0:.2f} mm")
    print(f"  face normal: {rounded(normal)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
