#!/usr/bin/env python3
"""Generate the 3D CoppeliaSim wall-tool inspection scene.

The generated scene is the 3D counterpart of the current 2D wall-tool model:
a vertical facade, top anchor/reel, cable, integrated payload body, two canted
side motors, and a pen tool that touches/draws on the wall.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WALL_TOOL_PROJECT_ROOT = PROJECT_ROOT.parent
WALL_TOOL_2D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_2d"
for path in (PROJECT_ROOT, WALL_TOOL_2D_ROOT, WALL_TOOL_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from coppeliasim_wall_tool import sim_utils  # noqa: E402
from cable_hybrid_controller.controller import best_params  # noqa: E402
from wall_tool_sim.wall_tool_ui import integrated_motor_center_offsets  # noqa: E402


SCENE_OUTPUT = PROJECT_ROOT / "scene" / "wall_tool_pen_scene.ttt"
MODEL_OUTPUT = PROJECT_ROOT / "scene" / "wall_tool_payload_model.ttm"

PAYLOAD_ALIAS = "wall_tool_payload"
CABLE_ALIAS = "wall_tool_cable"
ANCHOR_ALIAS = "anchor_reel_mount"
PEN_TIP_ALIAS = "pen_tip"
TARGET_ALIAS = "inspection_target"
LEFT_MOTOR_FRAME_ALIAS = "wall_tool_left_motor_frame"
RIGHT_MOTOR_FRAME_ALIAS = "wall_tool_right_motor_frame"
LEFT_PROP_JOINT_ALIAS = "wall_tool_left_propeller_spin_joint"
RIGHT_PROP_JOINT_ALIAS = "wall_tool_right_propeller_spin_joint"
LEFT_FORCE_ARROW_STEM_ALIAS = "wall_tool_left_motor_force_arrow_stem"
LEFT_FORCE_ARROW_HEAD_ALIAS = "wall_tool_left_motor_force_arrow_head"
RIGHT_FORCE_ARROW_STEM_ALIAS = "wall_tool_right_motor_force_arrow_stem"
RIGHT_FORCE_ARROW_HEAD_ALIAS = "wall_tool_right_motor_force_arrow_head"
CAGE_ROD_COLOR = [0.005, 0.005, 0.005]
CAGE_NODE_COLOR = [0.95, 0.02, 0.01]
MOTOR_CAN_COLOR = [0.08, 0.09, 0.10]
MOTOR_HUB_COLOR = [0.16, 0.16, 0.15]
FORCE_ARROW_COLOR = [1.0, 0.55, 0.02]
MOUNT_HOOK_COLOR = [0.18, 0.18, 0.17]
PEN_BODY_COLOR = [0.02, 0.12, 0.45]
PEN_NIB_COLOR = [0.02, 0.02, 0.02]
FORCE_ARROW_BASE_OFFSET = 0.075
FORCE_ARROW_INITIAL_LENGTH = 0.095
FORCE_ARROW_HEAD_LENGTH = 0.028


def parse_args() -> argparse.Namespace:
    params = best_params()
    parser = argparse.ArgumentParser(description="Generate the PRISMS wall-tool CoppeliaSim scene.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--scene-output", type=Path, default=SCENE_OUTPUT)
    parser.add_argument("--model-output", type=Path, default=MODEL_OUTPUT)
    parser.add_argument("--wall-width", type=float, default=params.wall_width)
    parser.add_argument("--wall-height", type=float, default=params.wall_height)
    parser.add_argument("--wall-thickness", type=float, default=0.050)
    parser.add_argument("--standoff", type=float, default=params.normal_standoff_m)
    parser.add_argument("--payload-x", type=float, default=params.initial_payload[0])
    parser.add_argument("--payload-z", type=float, default=params.initial_payload[1])
    parser.add_argument("--body-depth", type=float, default=0.140)
    parser.add_argument("--motor-depth", type=float, default=0.060)
    parser.add_argument("--cable-radius", type=float, default=0.006)
    parser.add_argument("--pen-radius", type=float, default=0.009)
    parser.add_argument("--save-model", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clear-existing", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def create_wall(sim, args: argparse.Namespace) -> None:
    wall_y = 0.5 * args.wall_thickness
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cuboid,
        [args.wall_width, args.wall_thickness, args.wall_height],
        "facade_wall",
        [0.0, wall_y, 0.5 * args.wall_height],
        [0.86, 0.85, 0.79],
        static=True,
        respondable=True,
    )

    params = best_params()
    work_width = params.contact_work_x_max - params.contact_work_x_min
    work_height = params.contact_work_z_max - params.contact_work_z_min
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cuboid,
        [work_width, 0.006, work_height],
        "facade_work_bay",
        [
            0.5 * (params.contact_work_x_min + params.contact_work_x_max),
            -0.004,
            0.5 * (params.contact_work_z_min + params.contact_work_z_max),
        ],
        [0.62, 0.80, 0.65],
        static=True,
        respondable=False,
    )

    grid_color = [0.60, 0.59, 0.54]
    grid_y = -0.007
    x_min = -0.5 * args.wall_width
    x_max = 0.5 * args.wall_width
    for index in range(math.floor(x_min), math.ceil(x_max) + 1):
        sim_utils.create_shape(
            sim,
            sim.primitiveshape_cuboid,
            [0.006, 0.004, args.wall_height],
            f"facade_grid_x_{index:+d}",
            [float(index), grid_y, 0.5 * args.wall_height],
            grid_color,
            static=True,
            respondable=False,
        )
    for index in range(0, math.ceil(args.wall_height) + 1):
        sim_utils.create_shape(
            sim,
            sim.primitiveshape_cuboid,
            [args.wall_width, 0.004, 0.006],
            f"facade_grid_z_{index:02d}",
            [0.0, grid_y, float(index)],
            grid_color,
            static=True,
            respondable=False,
        )


def create_anchor_and_reel(sim, args: argparse.Namespace) -> int:
    anchor = [0.0, -args.standoff, args.wall_height]
    mount = sim_utils.create_shape(
        sim,
        sim.primitiveshape_spheroid,
        [0.080, 0.080, 0.080],
        ANCHOR_ALIAS,
        anchor,
        [0.12, 0.12, 0.12],
        static=True,
        respondable=False,
    )
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cylinder,
        [0.150, 0.150, 0.105],
        "reel_spool",
        [0.0, -args.standoff, args.wall_height + 0.005],
        [0.20, 0.20, 0.20],
        orientation=[0.0, math.pi / 2.0, 0.0],
        static=True,
        respondable=False,
    )
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cuboid,
        [0.35, 0.040, 0.055],
        "anchor_reel_bracket",
        [0.0, -0.5 * args.standoff, args.wall_height + 0.085],
        [0.18, 0.18, 0.16],
        static=True,
        respondable=False,
    )
    return mount


def box_inertia(mass: float, size: tuple[float, float, float]) -> list[float]:
    lx, ly, lz = (max(1e-6, float(value)) for value in size)
    ixx = mass * (ly * ly + lz * lz) / 12.0
    iyy = mass * (lx * lx + lz * lz) / 12.0
    izz = mass * (lx * lx + ly * ly) / 12.0
    return [ixx, 0.0, 0.0, 0.0, iyy, 0.0, 0.0, 0.0, izz]


def create_local_cylinder_between(
    sim,
    parent: int,
    alias: str,
    start: Sequence[float],
    end: Sequence[float],
    diameter: float,
    color: Sequence[float],
) -> int:
    delta = [float(end[index]) - float(start[index]) for index in range(3)]
    length = max(1e-6, math.sqrt(sum(value * value for value in delta)))
    center = [(float(start[index]) + float(end[index])) * 0.5 for index in range(3)]
    handle = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [diameter, diameter, length], 0)
    sim.setObjectAlias(handle, alias, 1)
    sim.setObjectParent(handle, parent, False)
    sim.setObjectMatrix(handle, parent, sim_utils.matrix_from_z_axis(center, delta))
    sim_utils.color_shape(sim, handle, color)
    sim_utils.set_static(sim, handle, True)
    sim_utils.set_respondable(sim, handle, False)
    return int(handle)


def create_rectangular_payload_cage(
    sim,
    parent: int,
    body_length: float,
    body_depth: float,
    body_height: float,
) -> None:
    half_length = 0.5 * float(body_length)
    half_depth = 0.5 * float(body_depth)
    half_height = 0.5 * float(body_height)
    rod_diameter = max(0.010, min(body_length, body_depth, body_height) * 0.075)
    node_diameter = rod_diameter * 1.9

    corners = [
        (sx * half_length, sy * half_depth, sz * half_height)
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
        for sz in (-1.0, 1.0)
    ]
    edges: list[tuple[int, int]] = []
    for i, start in enumerate(corners):
        for j, end in enumerate(corners[i + 1 :], i + 1):
            different_axes = sum(abs(start[axis] - end[axis]) > 1e-9 for axis in range(3))
            if different_axes == 1:
                edges.append((i, j))

    for edge_index, (i, j) in enumerate(edges):
        create_local_cylinder_between(
            sim,
            parent,
            f"wall_tool_payload_cage_rod_{edge_index:02d}",
            corners[i],
            corners[j],
            rod_diameter,
            CAGE_ROD_COLOR,
        )
    for node_index, corner in enumerate(corners):
        sim_utils.create_shape(
            sim,
            sim.primitiveshape_spheroid,
            [node_diameter, node_diameter, node_diameter],
            f"wall_tool_payload_cage_node_{node_index:02d}",
            corner,
            CAGE_NODE_COLOR,
            static=True,
            respondable=False,
            parent=parent,
        )


def create_motor_frame(
    sim,
    body: int,
    frame_alias: str,
    joint_alias: str,
    force_stem_alias: str,
    force_head_alias: str,
    local_position: Sequence[float],
    local_axis: Sequence[float],
    prop_radius: float,
    motor_length: float,
) -> tuple[int, int]:
    frame = sim.createDummy(0.014)
    sim.setObjectAlias(frame, frame_alias, 1)
    sim.setObjectParent(frame, body, False)
    sim.setObjectMatrix(frame, body, sim_utils.matrix_from_z_axis(local_position, local_axis))

    motor_radius = max(0.020, prop_radius * 0.34)
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cylinder,
        [2.0 * motor_radius, 2.0 * motor_radius, motor_length],
        f"{frame_alias}_motor_can",
        [0.0, 0.0, -0.5 * motor_length],
        MOTOR_CAN_COLOR,
        static=True,
        respondable=False,
        parent=frame,
    )
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cylinder,
        [2.0 * motor_radius * 0.55, 2.0 * motor_radius * 0.55, motor_length * 0.36],
        f"{frame_alias}_motor_hub",
        [0.0, 0.0, 0.0],
        MOTOR_HUB_COLOR,
        static=True,
        respondable=False,
        parent=frame,
    )

    joint = sim.createJoint(sim.joint_revolute_subtype, sim.jointmode_kinematic, 0, [0.003, 0.001])
    sim.setObjectAlias(joint, joint_alias, 1)
    sim.setObjectParent(joint, frame, False)
    sim.setObjectPosition(joint, frame, [0.0, 0.0, 0.0])
    sim.setObjectOrientation(joint, frame, [0.0, 0.0, 0.0])
    sim_utils.set_visible(sim, joint, False)

    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cylinder,
        [2.0 * prop_radius, 2.0 * prop_radius, 0.004],
        f"{frame_alias}_propeller_blur",
        [0.0, 0.0, 0.010],
        [0.18, 0.20, 0.22],
        static=True,
        respondable=False,
        parent=joint,
    )
    for index, angle in enumerate((0.0, math.pi / 2.0)):
        sim_utils.create_shape(
            sim,
            sim.primitiveshape_cuboid,
            [prop_radius * 1.75, 0.018, 0.005],
            f"{frame_alias}_propeller_blade_{index}",
            [0.0, 0.0, 0.012],
            [0.03, 0.03, 0.03],
            orientation=[0.0, 0.0, angle],
            static=True,
            respondable=False,
            parent=joint,
        )

    stem_length = FORCE_ARROW_INITIAL_LENGTH - FORCE_ARROW_HEAD_LENGTH
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cylinder,
        [0.008, 0.008, stem_length],
        force_stem_alias,
        [0.0, 0.0, FORCE_ARROW_BASE_OFFSET + 0.5 * stem_length],
        FORCE_ARROW_COLOR,
        static=True,
        respondable=False,
        parent=frame,
    )
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cone,
        [0.026, 0.026, FORCE_ARROW_HEAD_LENGTH],
        force_head_alias,
        [0.0, 0.0, FORCE_ARROW_BASE_OFFSET + stem_length + 0.5 * FORCE_ARROW_HEAD_LENGTH],
        FORCE_ARROW_COLOR,
        static=True,
        respondable=False,
        parent=frame,
    )
    return int(frame), int(joint)


def create_payload(sim, args: argparse.Namespace) -> int:
    params = best_params()
    position, orientation = sim_utils.payload_pose_to_world(args.payload_x, args.payload_z, args.standoff, 0.0)

    left_offset, right_offset = integrated_motor_center_offsets(params, 0.0)
    motor_x = max(abs(left_offset[0]), abs(right_offset[0]))
    body_length = 2.0 * (motor_x + params.cage_radius * 0.50)
    body_height = max(2.0 * params.payload_hex_radius * 0.72, params.cage_radius * 0.66)
    body_depth = args.body_depth

    root = sim_utils.create_shape(
        sim,
        sim.primitiveshape_cuboid,
        [body_length, body_depth, body_height],
        PAYLOAD_ALIAS,
        position,
        [0.95, 0.74, 0.22],
        orientation=orientation,
        static=False,
        respondable=True,
    )
    sim_utils.set_visible(sim, root, False)
    inertia_size = (body_length, max(body_depth, args.standoff * 0.55), body_height)
    sim.setShapeMassAndInertia(
        root,
        params.total_mass,
        box_inertia(params.total_mass, inertia_size),
        [0.0, 0.0, 0.0],
        sim.getObjectMatrix(root, -1),
    )
    sim.setModelProperty(root, sim.getModelProperty(root) & ~sim.modelproperty_not_model)
    cage_depth = max(body_depth, args.standoff * 1.35)
    create_rectangular_payload_cage(sim, root, body_length, cage_depth, body_height)
    tilt = params.hex_face_tilt_rad
    prop_radius = max(0.050, params.cage_radius * 0.38)
    create_motor_frame(
        sim,
        root,
        LEFT_MOTOR_FRAME_ALIAS,
        LEFT_PROP_JOINT_ALIAS,
        LEFT_FORCE_ARROW_STEM_ALIAS,
        LEFT_FORCE_ARROW_HEAD_ALIAS,
        [left_offset[0], 0.0, left_offset[1]],
        [math.sin(tilt), 0.0, math.cos(tilt)],
        prop_radius,
        args.motor_depth,
    )
    create_motor_frame(
        sim,
        root,
        RIGHT_MOTOR_FRAME_ALIAS,
        RIGHT_PROP_JOINT_ALIAS,
        RIGHT_FORCE_ARROW_STEM_ALIAS,
        RIGHT_FORCE_ARROW_HEAD_ALIAS,
        [right_offset[0], 0.0, right_offset[1]],
        [-math.sin(tilt), 0.0, math.cos(tilt)],
        prop_radius,
        args.motor_depth,
    )
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cylinder,
        [args.pen_radius * 2.5, args.pen_radius * 2.5, args.standoff * 0.74],
        "pen_barrel",
        [0.0, args.standoff * 0.40, 0.0],
        PEN_BODY_COLOR,
        orientation=[math.pi / 2.0, 0.0, 0.0],
        static=True,
        respondable=False,
        parent=root,
    )
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_cone,
        [args.pen_radius * 2.2, args.pen_radius * 2.2, args.standoff * 0.26],
        "pen_nib",
        [0.0, args.standoff * 0.86, 0.0],
        PEN_NIB_COLOR,
        orientation=[math.pi / 2.0, 0.0, 0.0],
        static=True,
        respondable=False,
        parent=root,
    )
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_spheroid,
        [args.pen_radius * 1.4, args.pen_radius * 1.4, args.pen_radius * 1.4],
        PEN_TIP_ALIAS,
        [0.0, args.standoff, 0.0],
        PEN_NIB_COLOR,
        static=True,
        respondable=False,
        parent=root,
    )
    mount = sim.createDummy(0.014)
    sim.setObjectAlias(mount, "wall_tool_cable_mount", 1)
    sim.setObjectParent(mount, root, False)
    sim.setObjectPosition(mount, root, [0.0, 0.0, params.payload_hex_radius])
    sim.setObjectOrientation(mount, root, [0.0, 0.0, 0.0])
    create_local_cylinder_between(
        sim,
        root,
        "wall_tool_cable_mount_post",
        [0.0, 0.0, params.payload_hex_radius * 0.74],
        [0.0, 0.0, params.payload_hex_radius + 0.035],
        0.012,
        MOUNT_HOOK_COLOR,
    )
    sim_utils.create_shape(
        sim,
        sim.primitiveshape_spheroid,
        [0.038, 0.038, 0.038],
        "wall_tool_cable_mount_hook",
        [0.0, 0.0, params.payload_hex_radius + 0.043],
        MOUNT_HOOK_COLOR,
        static=True,
        respondable=False,
        parent=root,
    )

    target = sim_utils.create_shape(
        sim,
        sim.primitiveshape_spheroid,
        [0.055, 0.055, 0.055],
        TARGET_ALIAS,
        [args.payload_x + 0.65, -0.010, max(0.3, args.payload_z - 0.8)],
        [0.05, 0.35, 1.0],
        static=True,
        respondable=False,
    )
    sim_utils.set_visible(sim, target, True)
    sim.setObjectSel([root])
    return int(root)


def create_cable(sim, args: argparse.Namespace) -> int:
    params = best_params()
    start = [0.0, -args.standoff, args.wall_height]
    end = [args.payload_x, -args.standoff, args.payload_z + params.payload_hex_radius]
    length = math.dist(start, end)
    cable = sim_utils.create_shape(
        sim,
        sim.primitiveshape_cylinder,
        [args.cable_radius * 2.0, args.cable_radius * 2.0, max(1e-6, length)],
        CABLE_ALIAS,
        [(start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5],
        [0.02, 0.02, 0.02],
        static=True,
        respondable=False,
    )
    sim_utils.update_cylinder_between(sim, cable, start, end, length)
    return cable


def build_scene(sim, args: argparse.Namespace) -> dict[str, int]:
    sim_utils.stop_if_running(sim)
    if args.clear_existing:
        sim_utils.remove_generated(sim)
    create_wall(sim, args)
    anchor = create_anchor_and_reel(sim, args)
    payload = create_payload(sim, args)
    cable = create_cable(sim, args)
    return {"anchor": anchor, "payload": payload, "cable": cable}


def save_scene(sim, handles: dict[str, int], args: argparse.Namespace) -> None:
    args.scene_output.parent.mkdir(parents=True, exist_ok=True)
    sim.saveScene(str(args.scene_output))
    if args.save_model:
        args.model_output.parent.mkdir(parents=True, exist_ok=True)
        sim.saveModel(handles["payload"], str(args.model_output))


def main() -> int:
    args = parse_args()
    sim = sim_utils.connect(args.host, args.port, args.connect_timeout)
    handles = build_scene(sim, args)
    save_scene(sim, handles, args)
    print("Saved wall-tool CoppeliaSim scene:")
    print(f"  scene: {args.scene_output}")
    if args.save_model:
        print(f"  model: {args.model_output}")
    print("Aliases: /facade_wall, /anchor_reel_mount, /wall_tool_payload, /wall_tool_cable, /pen_tip")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
