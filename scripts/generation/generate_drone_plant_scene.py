#!/usr/bin/env python3
"""Generate a clean CoppeliaSim drone plant scene for external flight control.

Scope of this file is intentionally narrow:
1. Load the truncated-octahedral Crazyflie visual model.
2. Build one dynamic rigid-body drone plant.
3. Attach realistic propeller meshes.
4. Attach connector frames at all 24 cage corner nodes for docking logic.

Run scripts/launchers/run_position_ui_controller.py for single-drone position
control, or scripts/launchers/run_two_drone_position_ui.py for docking work.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
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

from controller.low_level import rate_control  # noqa: E402


MAIN_STL = PROJECT_ROOT / "assets" / "meshes" / "crazyflie_cage_body_no_propellers.stl"
PROP_STL = PROJECT_ROOT / "assets" / "meshes" / "crazyflie_propellers_aligned.stl"
OUTPUT_MODEL = PROJECT_ROOT / "model" / "truncated_octahedral_crazyflie_plant.ttm"
OUTPUT_SCENE = PROJECT_ROOT / "scene" / "body_rate_controller_demo_scene.ttt"

MODEL_ALIAS = "truncated_octahedral_crazyflie"
GENERATED_PREFIXES = (f"/{MODEL_ALIAS}", "/position_target")

PROPELLER_NAMES = (
    "propeller_ccw_1",
    "propeller_ccw_2",
    "propeller_cw_1",
    "propeller_cw_2",
)
PROPELLER_MESH_CENTERS = tuple(
    {"name": name, "pos": position}
    for name, position in zip(PROPELLER_NAMES, rate_control.LEGACY_PROPELLER_MESH_CENTERS_M)
)
PROPELLERS = tuple(
    {
        "name": mesh_center["name"],
        "mesh_center": mesh_center["pos"],
        "pos": rate_control.MOTORS[index]["pos"],
        "sign": rate_control.MOTORS[index]["spin"],
    }
    for index, mesh_center in enumerate(PROPELLER_MESH_CENTERS)
)
FORWARD_PROPELLER_COLOR = [0.95, 0.02, 0.01]
REAR_PROPELLER_COLOR = [0.03, 0.03, 0.03]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create the clean externally controlled drone plant scene.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--start-height", type=float, default=0.5)
    parser.add_argument("--mass", type=float, default=rate_control.DEFAULT_MASS_KG, help="Total mass [kg], drone plus cage.")
    parser.add_argument(
        "--geometry-scale",
        type=float,
        default=rate_control.DEFAULT_GEOMETRY_SCALE,
        help="Uniform scale applied to the STL-derived cage/body/connectors.",
    )
    parser.add_argument("--inertia-length-x", type=float, default=rate_control.DEFAULT_MODULE_INERTIA_BOX_M[0])
    parser.add_argument("--inertia-length-y", type=float, default=rate_control.DEFAULT_MODULE_INERTIA_BOX_M[1])
    parser.add_argument("--inertia-length-z", type=float, default=rate_control.DEFAULT_MODULE_INERTIA_BOX_M[2])
    parser.add_argument(
        "--propeller-visual-radius",
        type=float,
        default=rate_control.DEFAULT_PROP_RADIUS_M,
        help="Visual propeller radius [m]. Use 0 to keep the STL propeller size.",
    )
    parser.add_argument("--collision-rod-diameter", type=float, default=rate_control.DEFAULT_COLLISION_ROD_DIAMETER_M)
    parser.add_argument("--collision-node-diameter", type=float, default=rate_control.DEFAULT_COLLISION_NODE_DIAMETER_M)
    parser.add_argument("--show-collision", action="store_true")
    parser.add_argument("--main-stl", default=str(MAIN_STL))
    parser.add_argument("--propeller-stl", default=str(PROP_STL))
    parser.add_argument("--model", default=str(OUTPUT_MODEL))
    parser.add_argument("--scene", default=str(OUTPUT_SCENE))
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
        print(f"Removed {len(to_remove)} previous generated drone objects.")


def set_respondable(sim, handle: int, enabled: bool) -> None:
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1 if enabled else 0)


def hide_object(sim, handle: int) -> None:
    sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, 0)


def read_stl_triangles(path: Path) -> list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]]:
    data = path.read_bytes()
    tri_count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if 84 + tri_count * 50 == len(data):
        triangles = []
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
    center_radius = (center[0] ** 2 + center[1] ** 2 + center[2] ** 2) ** 0.5
    if triangle_count >= 80 and max_size <= 0.011 and center_radius >= 0.025:
        return "joint"
    if triangle_count <= 220 and center_radius >= 0.018:
        return "cage"
    return "body"


def triangles_to_mesh(triangles) -> tuple[list[float], list[int]]:
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


def scale_point(point: tuple[float, float, float], geometry_scale: float) -> tuple[float, float, float]:
    scale = max(1e-9, float(geometry_scale))
    return (point[0] * scale, point[1] * scale, point[2] * scale)


def scale_triangles(triangles, geometry_scale: float):
    scale = max(1e-9, float(geometry_scale))
    if abs(scale - 1.0) < 1e-12:
        return triangles
    return tuple(tuple(scale_point(vertex, scale) for vertex in triangle) for triangle in triangles)


def make_mesh_shape(sim, parent: int, alias: str, triangles, color: list[float], geometry_scale: float = 1.0) -> int | None:
    if not triangles:
        return None
    vertices, indices = triangles_to_mesh(scale_triangles(triangles, geometry_scale))
    shape = sim.createShape(0, math.radians(20.0), vertices, indices)
    sim.setObjectAlias(shape, alias, 1)
    sim.setShapeColor(shape, None, sim.colorcomponent_ambient_diffuse, color)
    set_respondable(sim, shape, False)
    sim.setObjectParent(shape, parent, True)
    sim.setObjectPosition(shape, parent, [0.0, 0.0, 0.0])
    sim.setObjectOrientation(shape, parent, [0.0, 0.0, 0.0])
    return shape


def split_propeller_stl(path: Path) -> dict[str, tuple[list[float], list[int]]]:
    centers = {prop["name"]: tuple(prop["mesh_center"]) for prop in PROPELLERS}
    grouped = {prop["name"]: [] for prop in PROPELLERS}
    for triangle in read_stl_triangles(path):
        centroid = tuple(sum(vertex[i] for vertex in triangle) / 3.0 for i in range(3))
        nearest = min(centers, key=lambda name: sum((centroid[i] - centers[name][i]) ** 2 for i in range(3)))
        grouped[nearest].append(triangle)

    meshes = {}
    for name, triangles in grouped.items():
        center = centers[name]
        vertices: list[float] = []
        indices: list[int] = []
        vertex_index: dict[tuple[float, float, float], int] = {}
        for triangle in triangles:
            for vertex in triangle:
                local = tuple(round(vertex[i] - center[i], 9) for i in range(3))
                index = vertex_index.get(local)
                if index is None:
                    index = len(vertex_index)
                    vertex_index[local] = index
                    vertices.extend(local)
                indices.append(index)
        if not indices:
            raise ValueError(f"No propeller triangles assigned to {name}.")
        meshes[name] = (vertices, indices)
    return meshes


def scale_xy_to_radius(vertices: list[float], radius: float) -> list[float]:
    target_radius = max(0.0, float(radius))
    if target_radius <= 0.0:
        return vertices
    current_radius = max(
        math.hypot(vertices[index], vertices[index + 1])
        for index in range(0, len(vertices), 3)
    )
    if current_radius <= 1e-12:
        raise ValueError("Cannot scale a propeller mesh with zero xy radius.")
    scale = target_radius / current_radius
    scaled = vertices[:]
    for index in range(0, len(scaled), 3):
        scaled[index] *= scale
        scaled[index + 1] *= scale
    return scaled


def vector_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> list[float]:
    return [a[i] - b[i] for i in range(3)]


def vector_norm(vector: list[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def vector_scale(vector: list[float], scale: float) -> list[float]:
    return [value * scale for value in vector]


def vector_cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def normalized(vector: list[float]) -> list[float]:
    norm = vector_norm(vector)
    if norm < 1e-12:
        raise ValueError("Cannot normalize zero-length vector.")
    return vector_scale(vector, 1.0 / norm)


def cylinder_matrix_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    z_offset: float,
) -> tuple[list[float], float]:
    axis = vector_sub(end, start)
    length = vector_norm(axis)
    z_axis = vector_scale(axis, 1.0 / length)
    reference = [0.0, 0.0, 1.0]
    if abs(sum(reference[i] * z_axis[i] for i in range(3))) > 0.95:
        reference = [0.0, 1.0, 0.0]
    x_axis = normalized(vector_cross(reference, z_axis))
    y_axis = vector_cross(z_axis, x_axis)
    midpoint = [(start[i] + end[i]) * 0.5 for i in range(3)]
    midpoint[2] += z_offset
    return (
        [
            x_axis[0], y_axis[0], z_axis[0], midpoint[0],
            x_axis[1], y_axis[1], z_axis[1], midpoint[1],
            x_axis[2], y_axis[2], z_axis[2], midpoint[2],
        ],
        length,
    )


def derive_cage_collision_graph(
    main_stl: Path,
    geometry_scale: float = 1.0,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int]]]:
    centers = []
    for component in connected_stl_components(main_stl):
        if classify_visual_component(component) == "joint" and int(component["triangle_count"]) >= 196:
            centers.append(tuple(component["center"]))

    clusters: list[dict[str, object]] = []
    for center in sorted(centers):
        best_index = None
        best_distance = float("inf")
        for index, cluster in enumerate(clusters):
            distance = math.dist(center, cluster["center"])
            if distance < best_distance:
                best_distance = distance
                best_index = index
        if best_index is not None and best_distance < 0.008:
            cluster = clusters[best_index]
            count = int(cluster["count"]) + 1
            cluster["center"] = tuple(
                (cluster["center"][i] * int(cluster["count"]) + center[i]) / count for i in range(3)
            )
            cluster["count"] = count
        else:
            clusters.append({"center": center, "count": 1})

    vertices = [cluster["center"] for cluster in clusters]
    if len(vertices) != 24:
        raise ValueError(f"Expected 24 cage collision nodes, found {len(vertices)}.")

    edges = []
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            distance = math.dist(vertices[i], vertices[j])
            if 0.052 <= distance <= 0.061:
                edges.append((i, j))
    if len(edges) != 36:
        raise ValueError(f"Expected 36 cage collision rods, found {len(edges)}.")
    return [scale_point(vertex, geometry_scale) for vertex in vertices], edges


def create_cage_collision_parts(
    sim,
    main_stl: Path,
    start_height: float,
    geometry_scale: float,
    rod_diameter: float,
    node_diameter: float,
    show_collision: bool,
) -> list[int]:
    vertices, edges = derive_cage_collision_graph(main_stl, geometry_scale)
    parts = []

    for edge_index, (i, j) in enumerate(edges):
        matrix, length = cylinder_matrix_between(vertices[i], vertices[j], start_height)
        rod = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [rod_diameter, rod_diameter, length], 0)
        sim.setObjectAlias(rod, f"cage_collision_rod_{edge_index:02d}", 1)
        sim.setObjectMatrix(rod, -1, matrix)
        sim.setShapeColor(rod, None, sim.colorcomponent_ambient_diffuse, [0.0, 0.8, 0.05])
        parts.append(rod)

    for node_index, vertex in enumerate(vertices):
        node = sim.createPrimitiveShape(
            sim.primitiveshape_spheroid,
            [node_diameter, node_diameter, node_diameter],
            0,
        )
        sim.setObjectAlias(node, f"cage_collision_node_{node_index:02d}", 1)
        sim.setObjectPosition(node, -1, [vertex[0], vertex[1], vertex[2] + start_height])
        sim.setShapeColor(node, None, sim.colorcomponent_ambient_diffuse, [1.0, 0.0, 0.0])
        parts.append(node)

    for part in parts:
        sim.setObjectInt32Param(part, sim.shapeintparam_static, 0)
        set_respondable(sim, part, True)
        if not show_collision:
            hide_object(sim, part)
    return parts


def create_dynamic_body(
    sim,
    mass: float,
    start_height: float,
    main_stl: Path,
    geometry_scale: float,
    inertia_box: tuple[float, float, float],
    rod_diameter: float,
    node_diameter: float,
    show_collision: bool,
) -> int:
    body_box = [value * max(1e-9, float(geometry_scale)) for value in rate_control.LEGACY_BODY_COLLISION_BOX_M]
    body = sim.createPrimitiveShape(sim.primitiveshape_cuboid, body_box, 0)
    sim.setObjectAlias(body, "drone_body_collision", 1)
    sim.setObjectPosition(body, -1, [0.0, 0.0, start_height])
    sim.setShapeColor(body, None, sim.colorcomponent_ambient_diffuse, [0.0, 0.35, 1.0])

    cage_collision_parts = create_cage_collision_parts(
        sim,
        main_stl,
        start_height,
        geometry_scale,
        rod_diameter,
        node_diameter,
        show_collision,
    )
    body = sim.groupShapes(cage_collision_parts + [body], False)
    sim.setObjectAlias(body, MODEL_ALIAS, 1)
    if not show_collision:
        hide_object(sim, body)
    sim.setObjectInt32Param(body, sim.shapeintparam_static, 0)
    set_respondable(sim, body, True)

    lx, ly, lz = (max(1e-6, float(value)) for value in inertia_box)
    ixx = mass * (ly * ly + lz * lz) / 12.0
    iyy = mass * (lx * lx + lz * lz) / 12.0
    izz = mass * (lx * lx + ly * ly) / 12.0
    inertia = [ixx, 0.0, 0.0, 0.0, iyy, 0.0, 0.0, 0.0, izz]
    sim.setShapeMassAndInertia(body, mass, inertia, [0.0, 0.0, 0.0], sim.getObjectMatrix(body, -1))
    sim.setModelProperty(body, sim.getModelProperty(body) & ~sim.modelproperty_not_model)
    return body


def attach_visual_model(sim, body: int, main_stl: Path, geometry_scale: float) -> None:
    categories = {"body": [], "cage": [], "joint": []}
    for component in connected_stl_components(main_stl):
        categories[classify_visual_component(component)].extend(component["triangles"])
    make_mesh_shape(sim, body, "crazyflie_body_visual", categories["body"], [0.62, 0.68, 0.60], geometry_scale)
    make_mesh_shape(sim, body, "cage_rods_black_visual", categories["cage"], [0.005, 0.005, 0.005], geometry_scale)
    make_mesh_shape(sim, body, "cage_corner_joints_red_visual", categories["joint"], [0.95, 0.02, 0.01], geometry_scale)


def attach_propellers(sim, body: int, prop_stl: Path, propeller_visual_radius: float) -> None:
    meshes = split_propeller_stl(prop_stl)
    for index, prop in enumerate(PROPELLERS):
        root = sim.createDummy(0.0005)
        sim.setObjectAlias(root, f"propeller_{index}_root", 1)
        sim.setObjectParent(root, body, False)
        sim.setObjectPosition(root, body, list(prop["pos"]))
        sim.setObjectOrientation(root, body, [0.0, 0.0, 0.0])
        hide_object(sim, root)

        joint = sim.createJoint(sim.joint_revolute_subtype, sim.jointmode_kinematic, 0, [0.003, 0.001])
        sim.setObjectAlias(joint, f"propeller_{index}_spin_joint", 1)
        sim.setObjectParent(joint, root, False)
        sim.setObjectPosition(joint, root, [0.0, 0.0, 0.0])
        sim.setObjectOrientation(joint, root, [0.0, 0.0, 0.0])
        hide_object(sim, joint)

        vertices, indices = meshes[prop["name"]]
        vertices = scale_xy_to_radius(vertices, propeller_visual_radius)
        mesh = sim.createShape(0, math.radians(20.0), vertices, indices)
        sim.setObjectAlias(mesh, f"propeller_{index}_mesh", 1)
        sim.setObjectParent(mesh, joint, False)
        sim.setObjectPosition(mesh, joint, [0.0, 0.0, 0.0])
        sim.setObjectOrientation(mesh, joint, [0.0, 0.0, 0.0])
        propeller_color = FORWARD_PROPELLER_COLOR if prop["pos"][0] > 0.0 else REAR_PROPELLER_COLOR
        sim.setShapeColor(mesh, None, sim.colorcomponent_ambient_diffuse, propeller_color)
        set_respondable(sim, mesh, False)


def attach_connector_frames(sim, body: int, main_stl: Path, geometry_scale: float) -> None:
    vertices, _edges = derive_cage_collision_graph(main_stl, geometry_scale)
    dummy_size = 0.001 * max(1e-9, float(geometry_scale))
    for index, vertex in enumerate(vertices):
        connector = sim.createDummy(dummy_size)
        sim.setObjectAlias(connector, f"magnet_connector_{index:02d}", 1)
        sim.setObjectParent(connector, body, False)
        sim.setObjectPosition(connector, body, list(vertex))
        sim.setObjectOrientation(connector, body, [0.0, 0.0, 0.0])
        hide_object(sim, connector)


def save_outputs(sim, body: int, model_path: Path, scene_path: Path) -> None:
    model_path.parent.mkdir(parents=True, exist_ok=True)
    scene_path.parent.mkdir(parents=True, exist_ok=True)
    sim.saveModel(body, str(model_path))
    sim.saveScene(str(scene_path))


def main() -> int:
    args = parse_args()
    main_stl = Path(args.main_stl)
    prop_stl = Path(args.propeller_stl)
    if not main_stl.exists():
        raise FileNotFoundError(main_stl)
    if not prop_stl.exists():
        raise FileNotFoundError(prop_stl)
    geometry_scale = max(1e-9, float(args.geometry_scale))

    sim = connect(args)
    stop_if_running(sim)
    remove_previous_generated(sim)
    body = create_dynamic_body(
        sim,
        args.mass,
        args.start_height,
        main_stl,
        geometry_scale,
        (args.inertia_length_x, args.inertia_length_y, args.inertia_length_z),
        args.collision_rod_diameter,
        args.collision_node_diameter,
        args.show_collision,
    )
    attach_visual_model(sim, body, main_stl, geometry_scale)
    attach_propellers(sim, body, prop_stl, args.propeller_visual_radius)
    attach_connector_frames(sim, body, main_stl, geometry_scale)
    sim.setObjectSel([body])
    save_outputs(sim, body, Path(args.model), Path(args.scene))

    print("Saved clean externally controlled drone plant model and scene:")
    print(f"  model: {Path(args.model)}")
    print(f"  scene: {Path(args.scene)}")
    print(f"Geometry scale: {geometry_scale:.4f}x STL coordinates.")
    print("Connector frames: 24 hidden magnet_connector_## dummies at the cage nodes.")
    print("Run scripts\\launchers\\run_position_ui_controller.py for single-drone position control.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
