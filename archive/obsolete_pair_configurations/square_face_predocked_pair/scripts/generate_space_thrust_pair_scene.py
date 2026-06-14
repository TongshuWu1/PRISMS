#!/usr/bin/env python3
"""Generate a dynamic CoppeliaSim plant scene for the pre-docked drone pair.

This is the physics-test companion to the inspection-only pair model. It builds
two independent dynamic drone bodies in the same square-face pre-docked pose,
each with cage collision, visible colored cage/body geometry, and four spinning
propeller joints.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


CONFIG_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = CONFIG_ROOT.parents[1]
PROJECT_SCRIPTS = PROJECT_ROOT / "scripts"
if str(PROJECT_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(PROJECT_SCRIPTS))

import generate_drone_plant_scene as plant  # noqa: E402


OUTPUT_MODEL = CONFIG_ROOT / "model" / "predocked_square_face_pair_space_thrust_plant.ttm"
OUTPUT_SCENE = CONFIG_ROOT / "scene" / "predocked_square_face_pair_space_thrust.ttt"
OUTPUT_METADATA = CONFIG_ROOT / "model" / "predocked_square_face_pair_space_thrust_metadata.json"

ROOT_ALIAS = "predocked_space_thrust_pair"
GENERATED_PREFIXES = (f"/{ROOT_ALIAS}",)

BODY_COLOR = [0.62, 0.68, 0.60]
CAGE_COLOR = [0.005, 0.005, 0.005]
CONNECTOR_COLOR = [0.95, 0.02, 0.01]
FORWARD_PROPELLER_COLOR = [0.95, 0.02, 0.01]
REAR_PROPELLER_COLOR = [0.03, 0.03, 0.03]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create the dynamic two-drone Space-thrust test scene.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--body-stl", default=str(plant.MAIN_STL))
    parser.add_argument("--propeller-stl", default=str(plant.PROP_STL))
    parser.add_argument("--model", default=str(OUTPUT_MODEL))
    parser.add_argument("--scene", default=str(OUTPUT_SCENE))
    parser.add_argument("--metadata", default=str(OUTPUT_METADATA))
    parser.add_argument("--mass", type=float, default=0.060, help="Mass per drone module [kg].")
    parser.add_argument("--collision-rod-diameter", type=float, default=0.003)
    parser.add_argument("--collision-node-diameter", type=float, default=0.008)
    parser.add_argument("--show-collision", action="store_true")
    parser.add_argument("--gap", type=float, default=0.001, help="Face-to-face clearance [m].")
    parser.add_argument("--height", type=float, default=0.5, help="Initial center height [m].")
    parser.add_argument(
        "--docking-face",
        choices=("pos_xy_to_neg_xy", "pos_x_neg_y_to_neg_x_pos_y"),
        default="pos_xy_to_neg_xy",
        help="Opposing vertical square/diamond faces used for face-to-face docking.",
    )
    parser.add_argument("--square-face-support", type=float, default=0.080, help="Square face plane distance from cage center [m].")
    return parser.parse_args()


def docking_face_normal(face: str) -> tuple[float, float, float]:
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    if face == "pos_xy_to_neg_xy":
        return (inv_sqrt2, inv_sqrt2, 0.0)
    if face == "pos_x_neg_y_to_neg_x_pos_y":
        return (inv_sqrt2, -inv_sqrt2, 0.0)
    raise ValueError(f"Unsupported docking face: {face}")


def rounded(values: list[float] | tuple[float, ...]) -> list[float]:
    return [round(value, 9) for value in values]


def translated(
    point: tuple[float, float, float],
    translation: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        point[0] + translation[0],
        point[1] + translation[1],
        point[2] + translation[2],
    )


def remove_previous_generated(sim) -> None:
    to_remove = []
    for handle in plant.all_scene_objects(sim):
        if plant.object_alias(sim, handle).startswith(GENERATED_PREFIXES):
            to_remove.append(handle)
    if to_remove:
        sim.removeObjects(to_remove, False)
        print(f"Removed {len(to_remove)} previous dynamic pair objects.")


def create_cage_collision_parts(
    sim,
    body_stl: Path,
    label: str,
    translation: tuple[float, float, float],
    rod_diameter: float,
    node_diameter: float,
    show_collision: bool,
) -> list[int]:
    vertices, edges = plant.derive_cage_collision_graph(body_stl)
    parts = []

    for edge_index, (i, j) in enumerate(edges):
        start = translated(vertices[i], translation)
        end = translated(vertices[j], translation)
        matrix, length = plant.cylinder_matrix_between(start, end, 0.0)
        rod = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [rod_diameter, rod_diameter, length], 0)
        sim.setObjectAlias(rod, f"{label}_cage_collision_rod_{edge_index:02d}", 1)
        sim.setObjectMatrix(rod, -1, matrix)
        sim.setShapeColor(rod, None, sim.colorcomponent_ambient_diffuse, [0.0, 0.8, 0.05])
        parts.append(rod)

    for node_index, vertex in enumerate(vertices):
        node = sim.createPrimitiveShape(
            sim.primitiveshape_spheroid,
            [node_diameter, node_diameter, node_diameter],
            0,
        )
        sim.setObjectAlias(node, f"{label}_cage_collision_node_{node_index:02d}", 1)
        sim.setObjectPosition(node, -1, list(translated(vertex, translation)))
        sim.setShapeColor(node, None, sim.colorcomponent_ambient_diffuse, CONNECTOR_COLOR)
        parts.append(node)

    for part in parts:
        sim.setObjectInt32Param(part, sim.shapeintparam_static, 0)
        plant.set_respondable(sim, part, True)
        if not show_collision:
            plant.hide_object(sim, part)
    return parts


def create_dynamic_drone(
    sim,
    label: str,
    translation: tuple[float, float, float],
    body_stl: Path,
    propeller_stl: Path,
    body_categories: dict[str, list[plant.Triangle]] | dict[str, list],
    mass: float,
    rod_diameter: float,
    node_diameter: float,
    show_collision: bool,
) -> int:
    body_box = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [0.09, 0.09, 0.026], 0)
    sim.setObjectAlias(body_box, f"{label}_body_collision", 1)
    sim.setObjectPosition(body_box, -1, list(translation))
    sim.setShapeColor(body_box, None, sim.colorcomponent_ambient_diffuse, [0.0, 0.35, 1.0])
    sim.setObjectInt32Param(body_box, sim.shapeintparam_static, 0)
    plant.set_respondable(sim, body_box, True)

    cage_collision_parts = create_cage_collision_parts(
        sim,
        body_stl,
        label,
        translation,
        rod_diameter,
        node_diameter,
        show_collision,
    )
    body = sim.groupShapes(cage_collision_parts + [body_box], False)
    sim.setObjectAlias(body, label, 1)
    if not show_collision:
        plant.hide_object(sim, body)
    sim.setObjectInt32Param(body, sim.shapeintparam_static, 0)
    plant.set_respondable(sim, body, True)

    lx, ly, lz = 0.176, 0.176, 0.166
    ixx = mass * (ly * ly + lz * lz) / 12.0
    iyy = mass * (lx * lx + lz * lz) / 12.0
    izz = mass * (lx * lx + ly * ly) / 12.0
    inertia = [ixx, 0.0, 0.0, 0.0, iyy, 0.0, 0.0, 0.0, izz]
    sim.setShapeMassAndInertia(body, mass, inertia, [0.0, 0.0, 0.0], sim.getObjectMatrix(body, -1))

    plant.make_mesh_shape(sim, body, f"{label}_crazyflie_body_visual", body_categories["body"], BODY_COLOR)
    plant.make_mesh_shape(sim, body, f"{label}_cage_rods_black_visual", body_categories["cage"], CAGE_COLOR)
    plant.make_mesh_shape(sim, body, f"{label}_cage_corner_joints_red_visual", body_categories["joint"], CONNECTOR_COLOR)
    attach_propellers(sim, body, propeller_stl)
    return body


def attach_propellers(sim, body: int, propeller_stl: Path) -> None:
    meshes = plant.split_propeller_stl(propeller_stl)
    for index, propeller in enumerate(plant.PROPELLERS):
        root = sim.createDummy(0.0005)
        sim.setObjectAlias(root, f"propeller_{index}_root", 1)
        sim.setObjectParent(root, body, False)
        sim.setObjectPosition(root, body, list(propeller["pos"]))
        sim.setObjectOrientation(root, body, [0.0, 0.0, 0.0])
        plant.hide_object(sim, root)

        joint = sim.createJoint(sim.joint_revolute_subtype, sim.jointmode_kinematic, 0, [0.003, 0.001])
        sim.setObjectAlias(joint, f"propeller_{index}_spin_joint", 1)
        sim.setObjectParent(joint, root, False)
        sim.setObjectPosition(joint, root, [0.0, 0.0, 0.0])
        sim.setObjectOrientation(joint, root, [0.0, 0.0, 0.0])
        plant.hide_object(sim, joint)

        vertices, indices = meshes[propeller["name"]]
        mesh = sim.createShape(0, math.radians(20.0), vertices, indices)
        direction_label = "front_red" if propeller["pos"][0] > 0.0 else "rear_black"
        sim.setObjectAlias(mesh, f"propeller_{index}_{direction_label}_mesh", 1)
        sim.setObjectParent(mesh, joint, False)
        sim.setObjectPosition(mesh, joint, [0.0, 0.0, 0.0])
        sim.setObjectOrientation(mesh, joint, [0.0, 0.0, 0.0])
        color = FORWARD_PROPELLER_COLOR if propeller["pos"][0] > 0.0 else REAR_PROPELLER_COLOR
        sim.setShapeColor(mesh, None, sim.colorcomponent_ambient_diffuse, color)
        plant.set_respondable(sim, mesh, False)


def body_visual_categories(body_stl: Path) -> dict[str, list]:
    categories = {"body": [], "cage": [], "joint": []}
    for component in plant.connected_stl_components(body_stl):
        categories[plant.classify_visual_component(component)].extend(component["triangles"])
    return categories


def pair_translations(args: argparse.Namespace) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    normal = docking_face_normal(args.docking_face)
    center_spacing = 2.0 * args.square_face_support + args.gap
    center_offset = 0.5 * center_spacing
    translation_a = (-center_offset * normal[0], -center_offset * normal[1], args.height)
    translation_b = (center_offset * normal[0], center_offset * normal[1], args.height)
    return normal, translation_a, translation_b


def save_outputs(sim, root: int, model_path: Path, scene_path: Path) -> None:
    model_path.parent.mkdir(parents=True, exist_ok=True)
    scene_path.parent.mkdir(parents=True, exist_ok=True)
    sim.saveModel(root, str(model_path))
    sim.saveScene(str(scene_path))


def main() -> int:
    args = parse_args()
    body_stl = Path(args.body_stl)
    propeller_stl = Path(args.propeller_stl)
    if not body_stl.exists():
        raise FileNotFoundError(body_stl)
    if not propeller_stl.exists():
        raise FileNotFoundError(propeller_stl)
    if args.gap < 0.0:
        raise ValueError("--gap must be non-negative.")

    categories = body_visual_categories(body_stl)
    normal, translation_a, translation_b = pair_translations(args)

    sim = plant.connect(args)
    plant.stop_if_running(sim)
    remove_previous_generated(sim)

    root = sim.createDummy(0.003)
    sim.setObjectAlias(root, ROOT_ALIAS, 1)
    sim.setObjectPosition(root, -1, [0.0, 0.0, 0.0])
    sim.setObjectOrientation(root, -1, [0.0, 0.0, 0.0])
    plant.hide_object(sim, root)
    try:
        sim.setModelProperty(root, sim.getModelProperty(root) & ~sim.modelproperty_not_model)
    except Exception:
        sim.setModelProperty(root, 0)

    drone_a = create_dynamic_drone(
        sim,
        "drone_a",
        translation_a,
        body_stl,
        propeller_stl,
        categories,
        args.mass,
        args.collision_rod_diameter,
        args.collision_node_diameter,
        args.show_collision,
    )
    drone_b = create_dynamic_drone(
        sim,
        "drone_b",
        translation_b,
        body_stl,
        propeller_stl,
        categories,
        args.mass,
        args.collision_rod_diameter,
        args.collision_node_diameter,
        args.show_collision,
    )
    sim.setObjectParent(drone_a, root, True)
    sim.setObjectParent(drone_b, root, True)
    sim.setObjectSel([root])

    model_path = Path(args.model)
    scene_path = Path(args.scene)
    save_outputs(sim, root, model_path, scene_path)

    metadata = {
        "configuration": "square_face_predocked_pair_space_thrust",
        "description": "Two independent dynamic drone modules in the square-face pose for Space-thrust and magnetic-docking controller tests.",
        "outputs": {
            "model_ttm": str(model_path),
            "scene_ttt": str(scene_path),
        },
        "root_alias": ROOT_ALIAS,
        "drone_aliases": ["drone_a", "drone_b"],
        "mass_per_drone_kg": args.mass,
        "docking_face": args.docking_face,
        "docking_face_normal_world": rounded(normal),
        "gap_m": args.gap,
        "square_face_support_m": args.square_face_support,
        "height_m": args.height,
        "drone_a_translation_m": rounded(translation_a),
        "drone_b_translation_m": rounded(translation_b),
        "collision": {
            "rod_count_per_drone": 36,
            "node_count_per_drone": 24,
            "rod_diameter_m": args.collision_rod_diameter,
            "node_diameter_m": args.collision_node_diameter,
            "body_box_m": [0.09, 0.09, 0.026],
        },
        "propellers": {
            "spinning_joint_count": 8,
            "forward_propeller_indices_per_drone": [
                index for index, propeller in enumerate(plant.PROPELLERS) if propeller["pos"][0] > 0.0
            ],
            "rear_propeller_indices_per_drone": [
                index for index, propeller in enumerate(plant.PROPELLERS) if propeller["pos"][0] <= 0.0
            ],
        },
        "notes": [
            "This is a dynamic plant; docking forces, latch state, and break limits are applied by external Python controllers.",
            "Each drone receives its own low-level motor mixer and force/torque application.",
            "Use controller/magnetic_docking_pair_test.py for connector-level magnet and virtual latch dynamics.",
            "Use the inspection model for pure docking-geometry review.",
        ],
    }
    metadata_path = Path(args.metadata)
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    print("Saved dynamic two-drone Space-thrust plant:")
    print(f"  model:    {model_path}")
    print(f"  scene:    {scene_path}")
    print(f"  metadata: {metadata_path}")
    print(f"  drone A:  {rounded(translation_a)}")
    print(f"  drone B:  {rounded(translation_b)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
