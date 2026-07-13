"""Build the detailed non-contact single-tether inspection scene."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from wall_tool_sim.steel_cable import SteelCableSpec
from wall_tool_sim.wall_tool_ui import SimParams, integrated_motor_center_offsets

from . import remote


SCENE_DIR = Path(__file__).resolve().parents[1] / "scene"
SCENE_PATH = SCENE_DIR / "vector_thrust_inspection_scene.ttt"
MODEL_PATH = SCENE_DIR / "vector_thrust_payload.ttm"

WALL = "vt_facade_wall"
WORK_BAY = "vt_inspection_region"
ANCHOR = "vt_anchor_reel"
PAYLOAD = "vt_payload"
CABLE_MOUNT = "vt_cable_mount"
LEFT_SERVO = "vt_left_tilt_servo"
RIGHT_SERVO = "vt_right_tilt_servo"
TARGET = "vt_target"
CABLE_PREFIX = "vt_cable_strand_"
CABLE_SEGMENTS = 24
CABLE_STRANDS = 3


STEEL = (0.34, 0.38, 0.43)
BRIGHT_STEEL = (0.60, 0.64, 0.69)
BLACK_ALUMINUM = (0.035, 0.042, 0.050)
CARBON = (0.025, 0.030, 0.034)
SAFETY_ORANGE = (0.95, 0.28, 0.035)


@dataclass(frozen=True)
class SceneHandles:
    wall: int
    anchor: int
    payload: int
    cable_mount: int
    left_servo: int
    right_servo: int
    target: int
    cable_segments: tuple[int, ...]


def cable_alias(strand: int, segment: int) -> str:
    return f"{CABLE_PREFIX}{strand}_{segment:02d}"


def _norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def _normalize(vector: Sequence[float]) -> list[float]:
    length = _norm(vector)
    if length <= 1e-12:
        raise ValueError("cannot orient geometry along a zero-length vector")
    return [float(value) / length for value in vector]


def _cross(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    ]


def _matrix_from_z_axis(origin: Sequence[float], z_axis_input: Sequence[float]) -> list[float]:
    z_axis = _normalize(z_axis_input)
    reference = [0.0, 1.0, 0.0]
    if abs(sum(reference[i] * z_axis[i] for i in range(3))) > 0.95:
        reference = [1.0, 0.0, 0.0]
    x_axis = _normalize(_cross(reference, z_axis))
    y_axis = _cross(z_axis, x_axis)
    return [
        x_axis[0], y_axis[0], z_axis[0], float(origin[0]),
        x_axis[1], y_axis[1], z_axis[1], float(origin[1]),
        x_axis[2], y_axis[2], z_axis[2], float(origin[2]),
    ]


def _rod(
    sim,
    alias: str,
    start: Sequence[float],
    end: Sequence[float],
    diameter: float,
    color: Sequence[float],
    *,
    parent: int = -1,
    specular: Sequence[float] = (0.25, 0.25, 0.25),
    emission: Sequence[float] | None = None,
) -> int:
    delta = [float(end[i]) - float(start[i]) for i in range(3)]
    length = _norm(delta)
    center = [0.5 * (float(start[i]) + float(end[i])) for i in range(3)]
    handle = remote.create_shape(
        sim,
        sim.primitiveshape_cylinder,
        [diameter, diameter, length],
        alias,
        center,
        color,
        static=True,
        respondable=False,
        parent=parent,
        specular=specular,
        emission=emission,
    )
    sim.setObjectMatrix(handle, parent, _matrix_from_z_axis(center, delta))
    return handle


def _box_inertia(mass: float, size: tuple[float, float, float]) -> list[float]:
    x, y, z = size
    return [
        mass * (y * y + z * z) / 12.0, 0.0, 0.0,
        0.0, mass * (x * x + z * z) / 12.0, 0.0,
        0.0, 0.0, mass * (x * x + y * y) / 12.0,
    ]


def _create_facade(sim, params: SimParams) -> int:
    wall = remote.create_shape(
        sim,
        sim.primitiveshape_cuboid,
        [params.wall_width, 0.080, params.wall_height + 0.45],
        WALL,
        [0.0, 0.040, 0.5 * params.wall_height + 0.10],
        [0.12, 0.13, 0.15],
        static=True,
        respondable=False,
        smooth=False,
    )
    columns = 4
    rows = 6
    horizontal_margin = 0.20
    vertical_margin = 0.20
    mullion = 0.075
    panel_width = (params.wall_width - 2.0 * horizontal_margin - (columns + 1) * mullion) / columns
    panel_height = (params.wall_height - 2.0 * vertical_margin - (rows + 1) * mullion) / rows
    for row in range(rows):
        for column in range(columns):
            x = (
                -0.5 * params.wall_width
                + horizontal_margin
                + mullion
                + 0.5 * panel_width
                + column * (panel_width + mullion)
            )
            z = (
                vertical_margin
                + mullion
                + 0.5 * panel_height
                + row * (panel_height + mullion)
            )
            shade = 0.02 * ((row + column) % 3)
            remote.create_shape(
                sim,
                sim.primitiveshape_cuboid,
                [panel_width, 0.010, panel_height],
                f"vt_window_{row}_{column}",
                [x, -0.006, z],
                [0.105 + shade, 0.19 + shade, 0.25 + shade],
                static=True,
                respondable=False,
                specular=(0.70, 0.78, 0.84),
                smooth=False,
            )
    for column in range(columns + 1):
        x = -0.5 * params.wall_width + horizontal_margin + column * (panel_width + mullion)
        remote.create_shape(
            sim,
            sim.primitiveshape_cuboid,
            [mullion, 0.022, params.wall_height - 2.0 * vertical_margin],
            f"vt_vertical_mullion_{column}",
            [x, -0.015, 0.5 * params.wall_height],
            [0.055, 0.060, 0.068],
            static=True,
            respondable=False,
            specular=(0.38, 0.40, 0.42),
            smooth=False,
        )
    for row in range(rows + 1):
        z = vertical_margin + row * (panel_height + mullion)
        remote.create_shape(
            sim,
            sim.primitiveshape_cuboid,
            [params.wall_width - 2.0 * horizontal_margin, 0.022, mullion],
            f"vt_horizontal_mullion_{row}",
            [0.0, -0.015, z],
            [0.055, 0.060, 0.068],
            static=True,
            respondable=False,
            specular=(0.38, 0.40, 0.42),
            smooth=False,
        )
    bay_width = 4.2
    bay_height = 4.15
    bay_center_z = 3.175
    for edge, size, position in (
        ("top", (bay_width, 0.010, 0.018), (0.0, -0.030, bay_center_z + 0.5 * bay_height)),
        ("bottom", (bay_width, 0.010, 0.018), (0.0, -0.030, bay_center_z - 0.5 * bay_height)),
        ("left", (0.018, 0.010, bay_height), (-0.5 * bay_width, -0.030, bay_center_z)),
        ("right", (0.018, 0.010, bay_height), (0.5 * bay_width, -0.030, bay_center_z)),
    ):
        remote.create_shape(
            sim,
            sim.primitiveshape_cuboid,
            size,
            WORK_BAY if edge == "top" else f"vt_inspection_region_{edge}",
            position,
            [0.04, 0.46, 0.70],
            static=True,
            respondable=False,
            specular=(0.30, 0.45, 0.55),
            emission=(0.01, 0.08, 0.14),
            transparency=0.28,
            smooth=False,
        )
    remote.create_shape(
        sim,
        sim.primitiveshape_cuboid,
        [params.wall_width + 0.30, 0.28, 0.28],
        "vt_roof_parapet",
        [0.0, 0.05, params.wall_height + 0.14],
        [0.22, 0.23, 0.24],
        static=True,
        respondable=False,
        smooth=False,
    )
    return wall


def _create_reel_assembly(sim, params: SimParams, standoff_m: float) -> int:
    anchor_position = [params.anchor[0], -standoff_m, params.anchor[1]]
    anchor = remote.create_shape(
        sim,
        sim.primitiveshape_spheroid,
        [0.008, 0.008, 0.008],
        ANCHOR,
        anchor_position,
        STEEL,
        static=True,
        respondable=False,
        visible=False,
    )
    remote.create_shape(
        sim, sim.primitiveshape_cuboid, [0.48, 0.18, 0.045], "vt_winch_base",
        [-0.08, -standoff_m, params.anchor[1] + 0.245], BLACK_ALUMINUM,
        static=True, respondable=False, specular=(0.30, 0.32, 0.34), smooth=False,
    )
    for side, x in (("left", -0.19), ("right", 0.03)):
        remote.create_shape(
            sim, sim.primitiveshape_cuboid, [0.025, 0.15, 0.22], f"vt_winch_cheek_{side}",
            [x, -standoff_m, params.anchor[1] + 0.145], [0.15, 0.16, 0.18],
            static=True, respondable=False, specular=(0.42, 0.44, 0.46), smooth=False,
        )
    spool_center = [-0.08, -standoff_m, params.anchor[1] + 0.155]
    spool_width = 0.070
    spool_diameter = 2.0 * params.reel_spool_radius_m
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [spool_diameter, spool_diameter, spool_width],
        "vt_reel_drum", spool_center, [0.20, 0.22, 0.24],
        orientation=[math.pi / 2.0, 0.0, 0.0], static=True, respondable=False,
        specular=(0.72, 0.75, 0.78),
    )
    for side, y in (("front", -standoff_m - 0.038), ("rear", -standoff_m + 0.038)):
        remote.create_shape(
            sim, sim.primitiveshape_cylinder, [0.076, 0.076, 0.006], f"vt_reel_flange_{side}",
            [spool_center[0], y, spool_center[2]], BRIGHT_STEEL,
            orientation=[math.pi / 2.0, 0.0, 0.0], static=True, respondable=False,
            specular=(0.90, 0.90, 0.90),
        )
    for band in range(7):
        y = -standoff_m - 0.027 + band * 0.009
        remote.create_shape(
            sim, sim.primitiveshape_cylinder,
            [spool_diameter + 0.003, spool_diameter + 0.003, 0.006],
            f"vt_wound_cable_band_{band}", [spool_center[0], y, spool_center[2]], STEEL,
            orientation=[math.pi / 2.0, 0.0, 0.0], static=True, respondable=False,
            specular=(0.85, 0.88, 0.92),
        )
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [0.070, 0.070, 0.11], "vt_winch_motor",
        [-0.255, -standoff_m, spool_center[2]], BLACK_ALUMINUM,
        orientation=[math.pi / 2.0, 0.0, 0.0], static=True, respondable=False,
        specular=(0.32, 0.34, 0.36),
    )
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [0.052, 0.052, 0.024], "vt_reel_encoder",
        [-0.255, -standoff_m - 0.067, spool_center[2]], [0.08, 0.16, 0.22],
        orientation=[math.pi / 2.0, 0.0, 0.0], static=True, respondable=False,
        specular=(0.48, 0.55, 0.60),
    )
    load_cell_center = [0.04, -standoff_m, params.anchor[1] + 0.165]
    remote.create_shape(
        sim, sim.primitiveshape_cuboid, [0.045, 0.050, 0.095], "vt_reel_load_cell",
        load_cell_center, [0.72, 0.08, 0.045], static=True, respondable=False,
        specular=(0.60, 0.34, 0.28), smooth=False,
    )
    for end_z in (-0.036, 0.036):
        remote.create_shape(
            sim, sim.primitiveshape_spheroid, [0.018, 0.018, 0.018],
            f"vt_load_cell_bolt_{'lower' if end_z < 0 else 'upper'}",
            [load_cell_center[0], -standoff_m - 0.027, load_cell_center[2] + end_z],
            BRIGHT_STEEL, static=True, respondable=False, specular=(0.9, 0.9, 0.9),
        )
    guide_center = [0.0, -standoff_m, params.anchor[1] + 0.052]
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [0.096, 0.096, 0.025], "vt_cable_angle_pulley",
        guide_center, [0.14, 0.15, 0.17], orientation=[math.pi / 2.0, 0.0, 0.0],
        static=True, respondable=False, specular=(0.78, 0.80, 0.82),
    )
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [0.030, 0.030, 0.030], "vt_cable_angle_encoder",
        [guide_center[0], -standoff_m - 0.031, guide_center[2]], [0.06, 0.16, 0.22],
        orientation=[math.pi / 2.0, 0.0, 0.0], static=True, respondable=False,
        specular=(0.50, 0.58, 0.62),
    )
    _rod(
        sim, "vt_fixed_winch_cable", [-0.055, -standoff_m, spool_center[2] - 0.018],
        [-0.034, -standoff_m, guide_center[2] + 0.033], params.steel_cable_diameter_m,
        STEEL, specular=(0.90, 0.90, 0.92),
    )
    return anchor


def _create_propeller_guard(sim, frame: int, alias: str, radius: float = 0.092) -> None:
    sections = 12
    for index in range(sections):
        a0 = 2.0 * math.pi * index / sections
        a1 = 2.0 * math.pi * (index + 1) / sections
        _rod(
            sim,
            f"{alias}_guard_{index:02d}",
            [radius * math.cos(a0), radius * math.sin(a0), 0.030],
            [radius * math.cos(a1), radius * math.sin(a1), 0.030],
            0.004,
            SAFETY_ORANGE,
            parent=frame,
            specular=(0.32, 0.20, 0.12),
        )


def _create_servo_visual(
    sim,
    payload: int,
    alias: str,
    offset: tuple[float, float],
    side_sign: float,
) -> int:
    remote.create_shape(
        sim, sim.primitiveshape_cuboid, [0.066, 0.075, 0.070], f"{alias}_housing",
        [offset[0], 0.0, offset[1]], [0.10, 0.11, 0.13], static=True,
        respondable=False, parent=payload, specular=(0.38, 0.40, 0.43), smooth=False,
    )
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [0.025, 0.025, 0.105], f"{alias}_pivot_shaft",
        [offset[0], 0.0, offset[1]], BRIGHT_STEEL,
        orientation=[math.pi / 2.0, 0.0, 0.0], static=True, respondable=False,
        parent=payload, specular=(0.90, 0.90, 0.90),
    )
    frame = int(sim.createDummy(0.012))
    sim.setObjectAlias(frame, alias, 1)
    sim.setObjectParent(frame, payload, False)
    sim.setObjectPosition(frame, payload, [offset[0], 0.0, offset[1]])
    remote.create_shape(
        sim, sim.primitiveshape_cuboid, [0.085, 0.065, 0.020], f"{alias}_gimbal_yoke",
        [0.0, 0.0, -0.035], SAFETY_ORANGE, static=True, respondable=False,
        parent=frame, specular=(0.42, 0.26, 0.14), smooth=False,
    )
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [0.058, 0.058, 0.050], f"{alias}_brushless_motor",
        [0.0, 0.0, -0.005], BLACK_ALUMINUM, static=True, respondable=False,
        parent=frame, specular=(0.42, 0.44, 0.46),
    )
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [0.026, 0.026, 0.018], f"{alias}_propeller_hub",
        [0.0, 0.0, 0.031], BRIGHT_STEEL, static=True, respondable=False,
        parent=frame, specular=(0.90, 0.90, 0.90),
    )
    for blade in range(2):
        remote.create_shape(
            sim, sim.primitiveshape_cuboid, [0.160, 0.018, 0.003],
            f"{alias}_propeller_blade_{blade}", [0.0, 0.0, 0.035], [0.055, 0.060, 0.065],
            orientation=[0.0, 0.0, blade * math.pi / 2.0 + side_sign * 0.12],
            static=True, respondable=False, parent=frame, specular=(0.26, 0.27, 0.28),
            smooth=False,
        )
    _create_propeller_guard(sim, frame, alias)
    return frame


def _create_payload(sim, params: SimParams, standoff_m: float) -> tuple[int, int, int, int]:
    left_offset, right_offset = integrated_motor_center_offsets(params, 0.0)
    payload_width = 2.0 * max(abs(left_offset[0]), abs(right_offset[0])) + 0.20
    inertial_size = (payload_width, 0.16, 0.23)
    payload = remote.create_shape(
        sim, sim.primitiveshape_cuboid, inertial_size, PAYLOAD,
        [params.initial_payload[0], -standoff_m, params.initial_payload[1]],
        [0.0, 0.0, 0.0], static=False, respondable=False, visible=False,
    )
    inertia = _box_inertia(params.total_mass, inertial_size)
    # Match the measured/identified pitch inertia used by NMPC while retaining
    # physically positive roll/yaw inertias for the free six-DOF body.
    inertia[4] = params.assembly_inertia
    sim.setShapeMassAndInertia(
        payload, params.total_mass, inertia, [0.0, 0.0, 0.0], sim.getObjectMatrix(payload, -1)
    )
    sim.setModelProperty(payload, sim.getModelProperty(payload) & ~sim.modelproperty_not_model)

    rail_length = payload_width - 0.08
    for rail, y, z in (
        ("front_top", -0.055, 0.075), ("rear_top", 0.055, 0.075),
        ("front_bottom", -0.055, -0.075), ("rear_bottom", 0.055, -0.075),
    ):
        remote.create_shape(
            sim, sim.primitiveshape_cuboid, [rail_length, 0.014, 0.014], f"vt_frame_{rail}",
            [0.0, y, z], CARBON, static=True, respondable=False, parent=payload,
            specular=(0.20, 0.22, 0.24), smooth=False,
        )
    for side, x in (("left", -0.22), ("right", 0.22)):
        remote.create_shape(
            sim, sim.primitiveshape_cuboid, [0.014, 0.125, 0.18], f"vt_frame_cross_{side}",
            [x, 0.0, 0.0], CARBON, static=True, respondable=False, parent=payload,
            specular=(0.20, 0.22, 0.24), smooth=False,
        )
    remote.create_shape(
        sim, sim.primitiveshape_cuboid, [0.28, 0.13, 0.115], "vt_electronics_enclosure",
        [0.0, 0.0, -0.005], [0.22, 0.235, 0.255], static=True, respondable=False,
        parent=payload, specular=(0.45, 0.47, 0.50), smooth=False,
    )
    remote.create_shape(
        sim, sim.primitiveshape_cuboid, [0.16, 0.075, 0.052], "vt_battery_pack",
        [0.0, -0.075, -0.010], [0.055, 0.060, 0.070], static=True, respondable=False,
        parent=payload, specular=(0.22, 0.24, 0.26), smooth=False,
    )
    remote.create_shape(
        sim, sim.primitiveshape_cuboid, [0.050, 0.035, 0.016], "vt_payload_imu",
        [0.0, 0.073, 0.040], [0.02, 0.20, 0.28], static=True, respondable=False,
        parent=payload, specular=(0.30, 0.52, 0.60), smooth=False,
    )
    remote.create_shape(
        sim, sim.primitiveshape_cuboid, [0.085, 0.055, 0.065], "vt_inspection_camera_body",
        [0.0, 0.082, 0.0], BLACK_ALUMINUM, static=True, respondable=False,
        parent=payload, specular=(0.32, 0.34, 0.36), smooth=False,
    )
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [0.038, 0.038, 0.035], "vt_inspection_camera_lens",
        [0.0, 0.122, 0.0], [0.015, 0.020, 0.025],
        orientation=[-math.pi / 2.0, 0.0, 0.0], static=True, respondable=False,
        parent=payload, specular=(0.78, 0.84, 0.90),
    )
    for side, x in (("left", -0.075), ("right", 0.075)):
        remote.create_shape(
            sim, sim.primitiveshape_cuboid, [0.042, 0.020, 0.055], f"vt_cable_clevis_{side}",
            [x, 0.0, params.payload_hex_radius - 0.025], BRIGHT_STEEL,
            static=True, respondable=False, parent=payload, specular=(0.85, 0.87, 0.90),
            smooth=False,
        )
    remote.create_shape(
        sim, sim.primitiveshape_cylinder, [0.022, 0.022, 0.17], "vt_cable_clevis_pin",
        [0.0, 0.0, params.payload_hex_radius - 0.005], BRIGHT_STEEL,
        orientation=[0.0, math.pi / 2.0, 0.0], static=True, respondable=False,
        parent=payload, specular=(0.90, 0.90, 0.92),
    )
    mount = int(sim.createDummy(0.010))
    sim.setObjectAlias(mount, CABLE_MOUNT, 1)
    sim.setObjectParent(mount, payload, False)
    sim.setObjectPosition(mount, payload, [0.0, 0.0, params.payload_hex_radius])
    left_servo = _create_servo_visual(sim, payload, LEFT_SERVO, left_offset, 1.0)
    right_servo = _create_servo_visual(sim, payload, RIGHT_SERVO, right_offset, -1.0)
    return payload, mount, left_servo, right_servo


def _create_initial_steel_cable(
    sim,
    params: SimParams,
    start: Sequence[float],
    end: Sequence[float],
) -> tuple[int, ...]:
    cable = SteelCableSpec(
        diameter_m=params.steel_cable_diameter_m,
        youngs_modulus_pa=params.steel_cable_youngs_modulus_pa,
        density_kg_m3=params.steel_cable_density_kg_m3,
        structural_compliance_m_N=params.steel_cable_structural_compliance_m_N,
        damping_ratio=params.steel_cable_damping_ratio,
        payload_weight_fraction=params.steel_cable_payload_weight_fraction,
    )
    delta = [float(end[i]) - float(start[i]) for i in range(3)]
    handles: list[int] = []
    strand_diameter = 0.48 * cable.diameter_m
    for segment in range(CABLE_SEGMENTS):
        u0 = segment / CABLE_SEGMENTS
        u1 = (segment + 1) / CABLE_SEGMENTS
        p0 = [float(start[i]) + u0 * delta[i] for i in range(3)]
        p1 = [float(start[i]) + u1 * delta[i] for i in range(3)]
        for strand in range(CABLE_STRANDS):
            shade = 0.045 * strand
            handles.append(_rod(
                sim,
                cable_alias(strand, segment),
                p0,
                p1,
                strand_diameter,
                [0.56 + shade, 0.59 + shade, 0.63 + shade],
                specular=(0.92, 0.94, 0.96),
                emission=(0.025, 0.027, 0.030),
            ))
    return tuple(handles)


def aim_default_camera(
    sim,
    target: Sequence[float],
    position: Sequence[float] = (3.0, -16.0, 3.40),
) -> None:
    try:
        camera = int(sim.getObject("/DefaultCamera"))
        position = [float(value) for value in position]
        view = _normalize([float(target[i]) - position[i] for i in range(3)])
        # The CoppeliaSim 4.1 scene camera renders along local +Z and displays
        # local +Y upward. Project world +Z into the image plane explicitly.
        world_up = [0.0, 0.0, 1.0]
        camera_z = view
        up_projection = sum(world_up[i] * camera_z[i] for i in range(3))
        camera_y = _normalize([
            world_up[i] - up_projection * camera_z[i] for i in range(3)
        ])
        right = _cross(camera_y, camera_z)
        sim.setObjectMatrix(camera, -1, [
            right[0], camera_y[0], camera_z[0], position[0],
            right[1], camera_y[1], camera_z[1], position[1],
            right[2], camera_y[2], camera_z[2], position[2],
        ])
    except Exception:
        pass


def configure_camera(sim, params: SimParams, view: str) -> None:
    if view == "overview":
        aim_default_camera(sim, [0.0, -0.05, 3.10], [3.0, -16.0, 3.40])
        return
    if view == "payload":
        aim_default_camera(
            sim,
            [params.initial_payload[0], -0.20, params.initial_payload[1]],
            [0.95, -1.65, params.initial_payload[1] + 0.72],
        )
        return
    if view == "winch":
        aim_default_camera(
            sim,
            [params.anchor[0] - 0.06, -0.20, params.anchor[1] + 0.13],
            [0.62, -1.25, params.anchor[1] + 0.48],
        )
        return
    raise ValueError(f"unknown CoppeliaSim camera view: {view}")


def create_planned_path_visual(sim, points: Sequence[tuple[float, float]]) -> tuple[int, ...]:
    """Render a non-physical desired path just in front of the facade."""

    handles: list[int] = []
    for index, (start, end) in enumerate(zip(points, points[1:])):
        p0 = [float(start[0]), -0.038, float(start[1])]
        p1 = [float(end[0]), -0.038, float(end[1])]
        if _norm([p1[i] - p0[i] for i in range(3)]) <= 1e-6:
            continue
        handles.append(_rod(
            sim,
            f"vt_planned_path_{index:02d}",
            p0,
            p1,
            0.010,
            [0.96, 0.45, 0.035],
            specular=(0.34, 0.22, 0.10),
            emission=(0.18, 0.055, 0.004),
        ))
    return tuple(handles)


def build_scene(sim, params: SimParams, *, standoff_m: float = 0.20) -> SceneHandles:
    remote.stop_if_running(sim)
    sim.loadScene("")
    if hasattr(sim, "arrayparam_gravity"):
        sim.setArrayParam(sim.arrayparam_gravity, [0.0, 0.0, -params.gravity])

    wall = _create_facade(sim, params)
    anchor = _create_reel_assembly(sim, params, standoff_m)
    payload, mount, left_servo, right_servo = _create_payload(sim, params, standoff_m)
    target = remote.create_shape(
        sim, sim.primitiveshape_spheroid, [0.045, 0.012, 0.045], TARGET,
        [params.initial_payload[0], -0.038, params.initial_payload[1]], [0.03, 0.22, 0.42],
        static=True, respondable=False, specular=(0.35, 0.58, 0.70),
        emission=(0.02, 0.20, 0.38), transparency=0.20,
    )
    anchor_position = [params.anchor[0], -standoff_m, params.anchor[1]]
    mount_position = [
        params.initial_payload[0],
        -standoff_m,
        params.initial_payload[1] + params.payload_hex_radius,
    ]
    cable_segments = _create_initial_steel_cable(sim, params, anchor_position, mount_position)
    configure_camera(sim, params, "overview")
    sim.setObjectSel([])
    return SceneHandles(
        wall=wall,
        anchor=anchor,
        payload=payload,
        cable_mount=mount,
        left_servo=left_servo,
        right_servo=right_servo,
        target=target,
        cable_segments=cable_segments,
    )


def save_scene(sim, handles: SceneHandles) -> None:
    SCENE_DIR.mkdir(parents=True, exist_ok=True)
    sim.saveScene(str(SCENE_PATH))
    sim.saveModel(handles.payload, str(MODEL_PATH))


def resolve_handles(sim) -> SceneHandles:
    return SceneHandles(
        wall=remote.object_handle(sim, WALL),
        anchor=remote.object_handle(sim, ANCHOR),
        payload=remote.object_handle(sim, PAYLOAD),
        cable_mount=remote.object_handle(sim, CABLE_MOUNT),
        left_servo=remote.object_handle(sim, LEFT_SERVO),
        right_servo=remote.object_handle(sim, RIGHT_SERVO),
        target=remote.object_handle(sim, TARGET),
        cable_segments=tuple(
            remote.object_handle(sim, cable_alias(strand, segment))
            for segment in range(CABLE_SEGMENTS)
            for strand in range(CABLE_STRANDS)
        ),
    )
