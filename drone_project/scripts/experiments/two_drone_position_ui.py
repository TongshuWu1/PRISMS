#!/usr/bin/env python3
"""Multi-drone position UI with external Python control and face-gated docking."""

from __future__ import annotations

import argparse
import math
import sys
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import ttk
from types import SimpleNamespace


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.allocation import geometry as allocation_geometry  # noqa: E402
from controller.allocation import wrench_allocator  # noqa: E402
from controller.common import telemetry  # noqa: E402
from controller.high_level import assembly_geometric  # noqa: E402
from controller.high_level import position  # noqa: E402
from controller.low_level import body_rate as flight  # noqa: E402
from simulation import magnetic_docking as multi  # noqa: E402


DEFAULT_SCENE = PROJECT_ROOT / "scene" / "two_drone_spawn_scene.ttt"
DEFAULT_BODY_STL = PROJECT_ROOT / "assets" / "meshes" / "crazyflie_cage_body_no_propellers.stl"
TARGET_ALIAS_PREFIX = "ui_target"
HOVER_TARGET_PREFIX = "hover_target"
MAGNET_VISUAL_PREFIX = "magnet_visual"
ASSEMBLY_FORCE_ARROW_PREFIX = "assembly_force_arrow"
ASSEMBLY_FORCE_ARROW_COLOR = (0.0, 0.85, 1.0)
ASSEMBLY_COMMAND_ARROW_COLOR = (1.0, 0.62, 0.05)
ASSEMBLY_RESIDUAL_ARROW_COLOR = (1.0, 0.10, 0.85)


@dataclass
class ControlledDrone:
    drone: multi.DroneAgent
    high_state: position.PositionControllerState
    low_state: flight.ControllerState
    args: argparse.Namespace
    target_handle: int
    command_target: list[float]
    command_yaw: float


@dataclass(frozen=True)
class Selection:
    kind: str
    index: int | None = None
    group: tuple[int, ...] = ()


@dataclass(frozen=True)
class DockedFaceLink:
    a_index: int
    b_index: int
    face_type: str
    a_face_index: int | None
    b_face_index: int | None
    corner_count: int


@dataclass(frozen=True)
class AssemblyConfiguration:
    key: tuple[int, ...]
    links: tuple[DockedFaceLink, ...]
    controller_name: str


@dataclass
class GroupCommand:
    key: tuple[int, ...]
    target: list[float]
    roll: float
    pitch: float
    yaw: float
    command_target: list[float]
    command_roll: float
    command_pitch: float
    command_yaw: float
    yaw_rate: float
    local_offsets: dict[int, list[float]]
    yaw_offsets: dict[int, float]
    state: assembly_geometric.AssemblyControllerState
    target_handle: int
    configuration: AssemblyConfiguration | None = None


@dataclass(frozen=True)
class QuickDockResult:
    pair: tuple[int, int]
    face_a: multi.DockFace
    face_b: multi.DockFace
    yaw_a: float
    yaw_b: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run multi-drone position UI control with magnetic docking.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--scene", default=str(DEFAULT_SCENE))
    parser.add_argument("--load-scene", action="store_true")
    parser.add_argument("--body-stl", default=str(DEFAULT_BODY_STL))

    parser.add_argument("--duration", type=float, default=0.0, help="Duration [s]. Use 0 until the UI closes.")
    parser.add_argument("--time-step", type=float, default=0.010)
    parser.add_argument("--reset-state", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--target-radius", type=float, default=0.025)
    parser.add_argument("--ui-step", type=float, default=0.12, help="Continuous UI jog speed [m/s].")
    parser.add_argument("--attitude-ui-step-deg", type=float, default=35.0, help="Continuous UI roll/pitch jog speed [deg/s].")
    parser.add_argument("--yaw-ui-step-deg", type=float, default=18.0, help="Continuous UI yaw jog speed [deg/s].")
    parser.add_argument("--target-slew-speed", type=float, default=0.14, help="Max filtered controller target speed [m/s].")
    parser.add_argument("--target-slew-roll-pitch-deg", type=float, default=45.0, help="Max filtered assembly roll/pitch command speed [deg/s].")
    parser.add_argument("--target-slew-yaw-deg", type=float, default=18.0, help="Max filtered controller yaw speed [deg/s].")
    parser.add_argument("--assembly-target-slew-yaw-deg", type=float, default=10.0, help="Max filtered docked assembly yaw speed [deg/s].")
    parser.add_argument("--assembly-target-yaw-accel-deg", type=float, default=35.0, help="Max docked assembly yaw command acceleration [deg/s^2].")
    parser.add_argument(
        "--quick-dock-spacing",
        type=float,
        default=0.18,
        help="Legacy quick-dock center spacing. Face-based quick square dock uses the cage geometry.",
    )
    parser.add_argument("--quick-open-spacing", type=float, default=0.36, help="Quick open-pair target spacing [m].")
    parser.add_argument(
        "--quick-dock-axis",
        choices=("x", "y", "xy", "x-neg-y"),
        default="xy",
        help="Nominal approach axis for quick square-face docking.",
    )
    parser.add_argument(
        "--quick-dock-yaw-mode",
        choices=("fronts-in", "fronts-out", "same-forward", "same-reverse", "lateral-left", "lateral-right"),
        default="fronts-in",
        help="Yaw convention for quick docking. Body +X is the drone front, marked by red propellers.",
    )

    parser.add_argument("--mass", type=float, default=flight.DEFAULT_MASS)
    parser.add_argument("--max-motor-speed", type=float, default=flight.DEFAULT_MAX_MOTOR_SPEED)
    parser.add_argument("--max-thrust", type=float, default=flight.DEFAULT_MAX_THRUST)
    parser.add_argument("--yaw-drag-arm", type=float, default=flight.DEFAULT_YAW_DRAG_ARM)

    parser.add_argument("--kp-xy", type=float, default=1.6)
    parser.add_argument("--kd-xy", type=float, default=1.7)
    parser.add_argument("--ki-xy", type=float, default=0.015)
    parser.add_argument("--kp-z", type=float, default=4.5)
    parser.add_argument("--kd-z", type=float, default=3.2)
    parser.add_argument("--ki-z", type=float, default=0.18)
    parser.add_argument("--integral-limit-xy", type=float, default=0.22)
    parser.add_argument("--integral-limit-z", type=float, default=0.18)

    parser.add_argument("--attitude-gain-rp", type=float, default=5.0)
    parser.add_argument("--attitude-gain-yaw", type=float, default=2.5)
    parser.add_argument("--max-roll-rate-deg", type=float, default=140.0)
    parser.add_argument("--max-pitch-rate-deg", type=float, default=140.0)
    parser.add_argument("--max-yaw-rate-deg", type=float, default=90.0)
    parser.add_argument("--max-tilt-deg", type=float, default=18.0)
    parser.add_argument("--max-horizontal-accel", type=float, default=1.6)
    parser.add_argument("--max-vertical-accel", type=float, default=3.5)

    parser.add_argument("--kp-rate-rp", type=float, default=0.0075)
    parser.add_argument("--kp-rate-yaw", type=float, default=0.0016)
    parser.add_argument("--ki-rate-rp", type=float, default=0.00045)
    parser.add_argument("--ki-rate-yaw", type=float, default=0.00012)
    parser.add_argument("--integral-limit-rp", type=float, default=0.20)
    parser.add_argument("--integral-limit-yaw", type=float, default=0.30)
    parser.add_argument("--motor-tau-up", type=float, default=0.050)
    parser.add_argument("--motor-tau-down", type=float, default=0.080)
    parser.add_argument("--linear-drag-xy", type=float, default=0.018)
    parser.add_argument("--linear-drag-z", type=float, default=0.006)
    parser.add_argument("--angular-drag-rp", type=float, default=0.00055)
    parser.add_argument("--angular-drag-yaw", type=float, default=0.00020)
    flight.add_propeller_visual_args(parser)

    parser.add_argument("--magnets-start-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--capture-radius", type=float, default=0.012 * flight.DEFAULT_GEOMETRY_SCALE)
    parser.add_argument("--max-magnet-pairs-per-drone-pair", type=int, default=8)
    parser.add_argument("--face-docking", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--face-normal-tolerance-deg", type=float, default=20.0)
    parser.add_argument("--face-center-tolerance", type=float, default=0.014 * flight.DEFAULT_GEOMETRY_SCALE)
    parser.add_argument("--face-latch-required-fraction", type=float, default=1.0)
    parser.add_argument("--magnet-rest-distance", type=float, default=multi.DEFAULT_CONNECTOR_CONTACT_DISTANCE)
    parser.add_argument("--magnet-stiffness", type=float, default=1.0)
    parser.add_argument("--magnet-damping", type=float, default=0.02)
    parser.add_argument("--magnet-force-limit", type=float, default=0.008)
    parser.add_argument("--latch-distance", type=float, default=0.005 * flight.DEFAULT_GEOMETRY_SCALE)
    parser.add_argument("--latch-speed", type=float, default=0.06)
    parser.add_argument("--latch-rest-distance", type=float, default=multi.DEFAULT_CONNECTOR_CONTACT_DISTANCE)
    parser.add_argument("--latch-stiffness", type=float, default=260.0)
    parser.add_argument("--latch-damping", type=float, default=0.30)
    parser.add_argument("--latch-force-limit", type=float, default=1.20)
    parser.add_argument("--latch-stiffness-ramp-time", type=float, default=0.20)
    parser.add_argument("--connector-break-force", type=float, default=7.00)
    parser.add_argument("--latch-break-distance", type=float, default=0.020 * flight.DEFAULT_GEOMETRY_SCALE)
    parser.add_argument("--relatch-delay", type=float, default=0.60)
    parser.add_argument(
        "--show-latched-corners",
        "--show-magnet-markers",
        dest="show_magnet_markers",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--latched-corner-radius",
        "--magnet-marker-radius",
        dest="magnet_marker_radius",
        type=float,
        default=0.008 * flight.DEFAULT_GEOMETRY_SCALE,
    )

    parser.add_argument("--log-period", type=float, default=0.25)
    parser.add_argument("--ui-event-period", type=float, default=0.02)
    parser.add_argument("--ui-update-period", type=float, default=0.10)
    parser.add_argument("--target-visual-update-period", type=float, default=0.05)
    parser.add_argument("--marker-update-period", type=float, default=0.10)
    parser.add_argument("--docked-controller", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--assembly-force-arrow",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show the net docked-assembly force vector at the assembly center of mass.",
    )
    parser.add_argument(
        "--assembly-force-arrow-source",
        choices=(
            "command-accel",
            "achieved-accel",
            "residual-accel",
            "net-accel",
            "net",
            "net-command",
            "achieved",
            "command",
            "residual",
        ),
        default="command-accel",
        help=(
            "Vector to visualize. Default command-accel shows the controller-requested "
            "COM acceleration after removing gravity. Achieved and residual comparison "
            "modes are also available."
        ),
    )
    parser.add_argument(
        "--assembly-force-arrow-scale",
        type=float,
        default=0.08,
        help="Arrow length per source unit. For acceleration sources, units are [m/(m/s^2)].",
    )
    parser.add_argument("--assembly-force-arrow-min-length", type=float, default=0.050)
    parser.add_argument("--assembly-force-arrow-max-length", type=float, default=0.400)
    parser.add_argument("--assembly-force-arrow-head-length", type=float, default=0.035)
    parser.add_argument("--assembly-force-arrow-radius", type=float, default=0.004)
    parser.add_argument(
        "--assembly-force-arrow-hide-threshold",
        type=float,
        default=0.030,
        help="Hide the assembly force arrow below this source-vector magnitude.",
    )
    parser.add_argument(
        "--assembly-force-arrow-update-period",
        type=float,
        default=0.030,
        help="Minimum simulation time between assembly force arrow updates [s].",
    )
    parser.add_argument("--allocation-regularization", type=float, default=1e-6)
    parser.add_argument("--allocation-weight-force-x", type=float, default=1.0)
    parser.add_argument("--allocation-weight-force-y", type=float, default=1.0)
    parser.add_argument("--allocation-weight-force-z", type=float, default=1.0)
    parser.add_argument("--allocation-weight-torque-x", type=float, default=1.0)
    parser.add_argument("--allocation-weight-torque-y", type=float, default=1.0)
    parser.add_argument("--allocation-weight-torque-z", type=float, default=1.0)
    parser.add_argument("--assembly-control-ramp-time", type=float, default=0.25)
    parser.add_argument("--assembly-max-horizontal-accel", type=float, default=2.8)
    parser.add_argument("--assembly-max-vertical-accel", type=float, default=4.5)
    parser.add_argument(
        "--assembly-position-attitude-feedforward",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable the legacy roll/pitch feedforward when --assembly-attitude-coupling feedforward is selected.",
    )
    parser.add_argument("--assembly-position-attitude-gain", type=float, default=1.0)
    parser.add_argument("--assembly-position-attitude-limit-deg", type=float, default=16.0)
    parser.add_argument(
        "--assembly-attitude-coupling",
        choices=("force-align", "feedforward", "off"),
        default="force-align",
        help="How docked translation demand changes desired attitude.",
    )
    parser.add_argument("--assembly-force-alignment-gain", type=float, default=1.0)
    parser.add_argument("--assembly-force-alignment-limit-deg", type=float, default=28.0)
    parser.add_argument(
        "--assembly-world-yaw-attitude",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Decompose docked attitude error into world-Z yaw and world-horizontal tilt components.",
    )
    parser.add_argument("--assembly-yaw-error-limit-deg", type=float, default=8.0, help="Clamp docked world-yaw attitude error before torque control [deg].")
    parser.add_argument("--assembly-attitude-torque-gain-rp", type=float, default=0.040)
    parser.add_argument("--assembly-attitude-torque-gain-yaw", type=float, default=0.006)
    parser.add_argument("--assembly-rate-damping-rp", type=float, default=0.0110)
    parser.add_argument("--assembly-rate-damping-yaw", type=float, default=0.0060)
    parser.add_argument("--assembly-torque-limit-rp", type=float, default=0.050)
    parser.add_argument("--assembly-torque-limit-yaw", type=float, default=0.010)
    parser.add_argument("--module-inertia-length-x", type=float, default=flight.DEFAULT_MODULE_INERTIA_BOX[0], help="Module inertia box x length [m].")
    parser.add_argument("--module-inertia-length-y", type=float, default=flight.DEFAULT_MODULE_INERTIA_BOX[1], help="Module inertia box y length [m].")
    parser.add_argument("--module-inertia-length-z", type=float, default=flight.DEFAULT_MODULE_INERTIA_BOX[2], help="Module inertia box z length [m].")
    parser.add_argument(
        "--assembly-inertia-aware-torque",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Shape docked attitude torque through the full assembly inertia tensor.",
    )
    parser.add_argument(
        "--assembly-gyroscopic-compensation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Add omega x J omega compensation when inertia-aware assembly torque is enabled.",
    )
    telemetry.add_logging_args(parser)
    return parser.parse_args()


def make_drone_args(base_args: argparse.Namespace, target: tuple[float, float, float], yaw: float) -> argparse.Namespace:
    drone_args = SimpleNamespace(**vars(base_args))
    drone_args.target_x = float(target[0])
    drone_args.target_y = float(target[1])
    drone_args.target_z = float(target[2])
    drone_args.target_yaw = float(yaw)
    drone_args.thrust = drone_args.mass * flight.G
    drone_args.p_cmd = 0.0
    drone_args.q_cmd = 0.0
    drone_args.r_cmd = 0.0
    drone_args.takeoff_pulse = False
    drone_args.pulse_scale = 1.0
    drone_args.pulse_duration = 0.0
    return drone_args


def remove_old_targets(sim) -> None:
    to_remove = []
    for handle in multi.all_scene_objects(sim):
        tail = multi.alias_tail(multi.object_alias(sim, handle))
        if (
            tail.startswith(TARGET_ALIAS_PREFIX)
            or tail.startswith(HOVER_TARGET_PREFIX)
            or tail.startswith(MAGNET_VISUAL_PREFIX)
            or tail.startswith(ASSEMBLY_FORCE_ARROW_PREFIX)
        ):
            to_remove.append(handle)
    if to_remove:
        sim.removeObjects(to_remove, False)


def create_target_sphere(sim, index: int, target: tuple[float, float, float], radius: float) -> int:
    sphere = sim.createPrimitiveShape(sim.primitiveshape_spheroid, [radius, radius, radius], 0)
    sim.setObjectAlias(sphere, f"{TARGET_ALIAS_PREFIX}_{index}", 1)
    sim.setObjectPosition(sphere, -1, list(target))
    sim.setShapeColor(sphere, None, sim.colorcomponent_ambient_diffuse, [0.05, 0.35, 1.0])
    sim.setObjectInt32Param(sphere, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(sphere, sim.shapeintparam_respondable, 0)
    return int(sphere)


def create_group_target_sphere(sim, key: tuple[int, ...], target: list[float], radius: float) -> int:
    members = "_".join(str(index) for index in key)
    sphere = sim.createPrimitiveShape(sim.primitiveshape_spheroid, [1.45 * radius, 1.45 * radius, 1.45 * radius], 0)
    sim.setObjectAlias(sphere, f"{TARGET_ALIAS_PREFIX}_assembly_{members}", 1)
    sim.setObjectPosition(sphere, -1, list(target))
    sim.setShapeColor(sphere, None, sim.colorcomponent_ambient_diffuse, [0.0, 0.80, 0.90])
    sim.setObjectInt32Param(sphere, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(sphere, sim.shapeintparam_respondable, 0)
    return int(sphere)


def set_visible(sim, handle: int, visible: bool) -> None:
    sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, 1 if visible else 0)


class LatchedCornerColorizer:
    def __init__(self, sim, max_pairs: int, radius: float) -> None:
        self.sim = sim
        self.marker_handles: list[int] = []
        self.active_count = 0
        self.max_markers = max(1, max_pairs) * 2
        for index in range(self.max_markers):
            marker = sim.createPrimitiveShape(sim.primitiveshape_spheroid, [radius, radius, radius], 0)
            sim.setObjectAlias(marker, f"{MAGNET_VISUAL_PREFIX}_latched_corner_{index:02d}", 1)
            sim.setObjectPosition(marker, -1, [0.0, 0.0, -10.0])
            sim.setShapeColor(marker, None, sim.colorcomponent_ambient_diffuse, [0.0, 1.0, 0.18])
            sim.setObjectInt32Param(marker, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(marker, sim.shapeintparam_respondable, 0)
            sim.setObjectInt32Param(marker, sim.objintparam_visibility_layer, 0)
            self.marker_handles.append(int(marker))

    def clear(self) -> None:
        for handle in self.marker_handles[: self.active_count]:
            self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 0)
        self.active_count = 0

    def update(
        self,
        world: dict[int, multi.WorldDrone],
        latched_keys: list[multi.LatchKey],
    ) -> None:
        marker_index = 0
        for key in latched_keys:
            if marker_index + 1 >= len(self.marker_handles):
                break
            a_index, a_connector, b_index, b_connector = key
            points = [
                world[a_index].connector_position[a_connector],
                world[b_index].connector_position[b_connector],
            ]
            for point in points:
                handle = self.marker_handles[marker_index]
                self.sim.setObjectPosition(handle, -1, list(point))
                self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 1)
                marker_index += 1
        for handle in self.marker_handles[marker_index : self.active_count]:
            self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 0)
        self.active_count = marker_index


class AssemblyForceArrowVisualizer:
    FORCE_FIELDS = {
        "achieved": "wrench_achieved",
        "command": "wrench_cmd",
        "residual": "wrench_residual",
    }

    def __init__(self, sim, args: argparse.Namespace) -> None:
        self.sim = sim
        self.source = str(args.assembly_force_arrow_source)
        if "command" in self.source:
            self.color = ASSEMBLY_COMMAND_ARROW_COLOR
        elif "residual" in self.source:
            self.color = ASSEMBLY_RESIDUAL_ARROW_COLOR
        else:
            self.color = ASSEMBLY_FORCE_ARROW_COLOR
        self.length_scale = max(0.0, float(args.assembly_force_arrow_scale))
        self.radius = max(0.0002, float(args.assembly_force_arrow_radius))
        self.head_length = max(0.004, float(args.assembly_force_arrow_head_length))
        self.min_length = max(self.head_length + 0.001, float(args.assembly_force_arrow_min_length))
        self.max_length = max(self.min_length, float(args.assembly_force_arrow_max_length))
        self.hide_threshold = max(0.0, float(args.assembly_force_arrow_hide_threshold))
        self.arrow_handles: dict[str, tuple[int, int]] = {}
        self.stem_lengths: dict[str, float] = {}
        self.active_keys: set[str] = set()

    def configure_shape(self, handle: int) -> None:
        self.sim.setShapeColor(handle, None, self.sim.colorcomponent_ambient_diffuse, list(self.color))
        self.sim.setShapeColor(handle, None, self.sim.colorcomponent_emission, [0.00, 0.08, 0.10])
        self.sim.setObjectInt32Param(handle, self.sim.shapeintparam_static, 1)
        self.sim.setObjectInt32Param(handle, self.sim.shapeintparam_respondable, 0)
        self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 0)

    def create_arrow(self, key: str) -> tuple[int, int]:
        stem_length = max(0.001, self.min_length - self.head_length)
        stem = self.sim.createPrimitiveShape(
            self.sim.primitiveshape_cylinder,
            [2.0 * self.radius, 2.0 * self.radius, stem_length],
            0,
        )
        self.sim.setObjectAlias(stem, f"{ASSEMBLY_FORCE_ARROW_PREFIX}_{key}_stem", 1)
        self.configure_shape(stem)

        head = self.sim.createPrimitiveShape(
            self.sim.primitiveshape_cone,
            [4.0 * self.radius, 4.0 * self.radius, self.head_length],
            0,
        )
        self.sim.setObjectAlias(head, f"{ASSEMBLY_FORCE_ARROW_PREFIX}_{key}_head", 1)
        self.configure_shape(head)

        handles = (int(stem), int(head))
        self.arrow_handles[key] = handles
        self.stem_lengths[key] = stem_length
        return handles

    def ensure_arrow(self, key: str) -> tuple[int, int]:
        if key not in self.arrow_handles:
            return self.create_arrow(key)
        return self.arrow_handles[key]

    def hide_key(self, key: str) -> None:
        if key not in self.arrow_handles:
            return
        stem, head = self.arrow_handles[key]
        for handle in (stem, head):
            self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 0)

    def clear(self) -> None:
        for key in list(self.active_keys):
            self.hide_key(key)
        self.active_keys.clear()

    def force_vector(self, sample: dict[str, object]) -> list[float]:
        mass = max(1e-9, float(sample["assembly_mass"]))
        if self.source in ("command-accel", "achieved-accel", "net-accel", "residual-accel"):
            if self.source == "residual-accel":
                values = sample["wrench_residual"]
                return [float(values[axis]) / mass for axis in range(3)]  # type: ignore[index]
            field = "wrench_achieved" if self.source in ("achieved-accel", "net-accel") else "wrench_cmd"
            values = sample[field]
            force = [float(values[axis]) for axis in range(3)]  # type: ignore[index]
            force[2] -= mass * flight.G
            return [value / mass for value in force]
        if self.source in ("net", "net-command"):
            field = "wrench_achieved" if self.source == "net" else "wrench_cmd"
            values = sample[field]
            force = [float(values[axis]) for axis in range(3)]  # type: ignore[index]
            force[2] -= mass * flight.G
            return force
        field = self.FORCE_FIELDS[self.source]
        values = sample[field]
        return [float(values[axis]) for axis in range(3)]  # type: ignore[index]

    def force_length(self, force: list[float]) -> float:
        return max(self.min_length, min(self.max_length, vector_norm3(force) * self.length_scale))

    def update(self, allocation_by_group: dict[str, object]) -> None:
        active: set[str] = set()
        for raw_key, raw_sample in sorted(allocation_by_group.items()):
            sample = raw_sample  # type: ignore[assignment]
            key = str(raw_key)
            force = self.force_vector(sample)  # type: ignore[arg-type]
            force_norm = vector_norm3(force)
            if force_norm < self.hide_threshold:
                self.hide_key(key)
                continue

            base = [float(value) for value in sample["assembly_pos"]]  # type: ignore[index]
            direction = vector_scale3(force, 1.0 / force_norm)
            total_length = self.force_length(force)
            stem_length = max(0.001, total_length - self.head_length)
            stem, head = self.ensure_arrow(key)
            old_stem_length = self.stem_lengths[key]
            if abs(stem_length - old_stem_length) > 1e-5:
                self.sim.scaleObject(stem, 1.0, 1.0, stem_length / old_stem_length, 0)
                self.stem_lengths[key] = stem_length

            stem_center = [base[axis] + 0.5 * stem_length * direction[axis] for axis in range(3)]
            head_center = [
                base[axis] + (stem_length + 0.5 * self.head_length) * direction[axis]
                for axis in range(3)
            ]
            self.sim.setObjectMatrix(stem, -1, world_matrix_from_z_axis(stem_center, direction))
            self.sim.setObjectMatrix(head, -1, world_matrix_from_z_axis(head_center, direction))
            for handle in (stem, head):
                self.sim.setObjectInt32Param(handle, self.sim.objintparam_visibility_layer, 1)
            active.add(key)

        for key in self.active_keys - active:
            self.hide_key(key)
        self.active_keys = active


def latched_corner_visual_keys(
    sim,
    drones: list[multi.DroneAgent],
    connectors: list[multi.Vector3],
    memory: multi.DockingMemory,
    magnets_enabled: bool,
) -> tuple[dict[int, multi.WorldDrone], list[multi.LatchKey]]:
    world = multi.compute_world_drones(sim, drones, connectors)
    latched_keys = sorted(memory.latched)
    return world, latched_keys if magnets_enabled else []


def hover_omega(args: argparse.Namespace) -> float:
    return math.sqrt((args.mass * flight.G / 4.0) / (args.max_thrust / args.max_motor_speed**2))


def module_inertia_box(args: argparse.Namespace) -> tuple[float, float, float]:
    return (
        float(args.module_inertia_length_x),
        float(args.module_inertia_length_y),
        float(args.module_inertia_length_z),
    )


def allocation_weights(args: argparse.Namespace) -> list[float]:
    return [
        float(args.allocation_weight_force_x),
        float(args.allocation_weight_force_y),
        float(args.allocation_weight_force_z),
        float(args.allocation_weight_torque_x),
        float(args.allocation_weight_torque_y),
        float(args.allocation_weight_torque_z),
    ]


def format_vector(values: list[float] | tuple[float, float, float]) -> str:
    return "[" + ", ".join(f"{float(value): .3f}" for value in values) + "]"


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def average_positions(positions: list[list[float]]) -> list[float]:
    count = max(1, len(positions))
    return [sum(position[axis] for position in positions) / count for axis in range(3)]


def rotate_z(vector: list[float], yaw: float) -> list[float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [
        c * vector[0] - s * vector[1],
        s * vector[0] + c * vector[1],
        vector[2],
    ]


def dot3(a: list[float] | tuple[float, float, float], b: list[float] | tuple[float, float, float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def vector_norm3(vector: list[float] | tuple[float, float, float]) -> float:
    return math.sqrt(dot3(vector, vector))


def vector_scale3(vector: list[float], scale: float) -> list[float]:
    return [value * scale for value in vector]


def vector_cross3(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def normalized3(vector: list[float]) -> list[float]:
    norm = vector_norm3(vector)
    if norm < 1e-12:
        raise ValueError("Cannot normalize zero-length vector.")
    return vector_scale3(vector, 1.0 / norm)


def world_matrix_from_z_axis(center: list[float], z_axis: list[float]) -> list[float]:
    z_axis = normalized3(z_axis)
    reference = [0.0, 0.0, 1.0]
    if abs(dot3(reference, z_axis)) > 0.95:
        reference = [0.0, 1.0, 0.0]
    x_axis = normalized3(vector_cross3(reference, z_axis))
    y_axis = vector_cross3(z_axis, x_axis)
    return [
        x_axis[0], y_axis[0], z_axis[0], center[0],
        x_axis[1], y_axis[1], z_axis[1], center[1],
        x_axis[2], y_axis[2], z_axis[2], center[2],
    ]


def unit_xy_from_heading(heading: float) -> list[float]:
    return [math.cos(heading), math.sin(heading), 0.0]


def pair_offsets(axis: str, spacing: float) -> tuple[list[float], list[float]]:
    half = spacing * 0.5
    if axis == "x":
        return [-half, 0.0, 0.0], [half, 0.0, 0.0]
    if axis == "y":
        return [0.0, -half, 0.0], [0.0, half, 0.0]
    diagonal = half / math.sqrt(2.0)
    if axis == "xy":
        return [-diagonal, -diagonal, 0.0], [diagonal, diagonal, 0.0]
    return [-diagonal, diagonal, 0.0], [diagonal, -diagonal, 0.0]


def heading_between_offsets(offset_a: list[float], offset_b: list[float]) -> float:
    return math.atan2(offset_b[1] - offset_a[1], offset_b[0] - offset_a[0])


def front_vector_from_yaw(yaw: float) -> list[float]:
    return [math.cos(yaw), math.sin(yaw), 0.0]


def quick_dock_yaws(axis: str, yaw_mode: str) -> tuple[float, float]:
    offset_a, offset_b = pair_offsets(axis, 1.0)
    approach_heading = heading_between_offsets(offset_a, offset_b)
    if yaw_mode == "fronts-in":
        return wrap_pi(approach_heading), wrap_pi(approach_heading + math.pi)
    if yaw_mode == "fronts-out":
        return wrap_pi(approach_heading + math.pi), wrap_pi(approach_heading)
    if yaw_mode == "same-forward":
        return wrap_pi(approach_heading), wrap_pi(approach_heading)
    if yaw_mode == "same-reverse":
        return wrap_pi(approach_heading + math.pi), wrap_pi(approach_heading + math.pi)
    if yaw_mode == "lateral-left":
        return wrap_pi(approach_heading + 0.5 * math.pi), wrap_pi(approach_heading + 0.5 * math.pi)
    if yaw_mode == "lateral-right":
        return wrap_pi(approach_heading - 0.5 * math.pi), wrap_pi(approach_heading - 0.5 * math.pi)
    raise ValueError(f"Unknown quick dock yaw mode: {yaw_mode}")


def yaw_mode_label(yaw_mode: str) -> str:
    labels = {
        "fronts-in": "fronts inward",
        "fronts-out": "fronts outward",
        "same-forward": "same heading forward",
        "same-reverse": "same heading reverse",
        "lateral-left": "fronts lateral left",
        "lateral-right": "fronts lateral right",
    }
    return labels.get(yaw_mode, yaw_mode)


def vertical_square_faces(geometry: multi.DockingGeometry) -> list[multi.DockFace]:
    return [
        face
        for face in geometry.faces
        if face.face_type == "square" and abs(face.normal[2]) < 0.25
    ]


def choose_quick_square_faces(
    geometry: multi.DockingGeometry,
    axis: str,
    yaw_a: float,
    yaw_b: float,
) -> tuple[multi.DockFace, multi.DockFace]:
    faces = vertical_square_faces(geometry)
    if not faces:
        raise RuntimeError("No vertical square/diamond faces are available for quick docking.")
    offset_a, offset_b = pair_offsets(axis, 1.0)
    approach = unit_xy_from_heading(heading_between_offsets(offset_a, offset_b))

    best_score = float("inf")
    best_pair: tuple[multi.DockFace, multi.DockFace] | None = None
    for face_a in faces:
        normal_a = rotate_z(list(face_a.normal), yaw_a)
        for face_b in faces:
            normal_b = rotate_z(list(face_b.normal), yaw_b)
            normal_opposition_error = abs(dot3(normal_a, normal_b) + 1.0)
            approach_alignment_error = 1.0 - dot3(normal_a, approach)
            score = 10.0 * normal_opposition_error + approach_alignment_error + 1e-4 * (face_a.index + face_b.index)
            if score < best_score:
                best_score = score
                best_pair = (face_a, face_b)
    if best_pair is None:
        raise RuntimeError("Could not choose a square/diamond face pair for quick docking.")
    return best_pair


def command_square_face_dock_pose(
    sim,
    controlled: list[ControlledDrone],
    pair: tuple[int, int],
    face_a: multi.DockFace,
    face_b: multi.DockFace,
    yaw_a: float,
    yaw_b: float,
    rest_distance: float,
) -> None:
    a_index, b_index = pair
    positions = [current_drone_position(sim, controlled[a_index]), current_drone_position(sim, controlled[b_index])]
    center = average_positions(positions)
    face_center_a = rotate_z(list(face_a.center), yaw_a)
    face_center_b = rotate_z(list(face_b.center), yaw_b)
    face_normal_a = rotate_z(list(face_a.normal), yaw_a)
    body_delta = [
        face_center_a[axis] - face_center_b[axis] + rest_distance * face_normal_a[axis]
        for axis in range(3)
    ]
    target_a = [center[axis] - 0.5 * body_delta[axis] for axis in range(3)]
    target_b = [center[axis] + 0.5 * body_delta[axis] for axis in range(3)]
    set_controlled_target(controlled[a_index], target_a, yaw_a)
    set_controlled_target(controlled[b_index], target_b, yaw_b)


def current_drone_position(sim, item: ControlledDrone) -> list[float]:
    return [float(value) for value in sim.getObjectPosition(item.drone.body, -1)]


def current_drone_yaw(sim, item: ControlledDrone) -> float:
    return float(sim.getObjectOrientation(item.drone.body, -1)[2])


def representative_command_yaw(
    controlled: list[ControlledDrone],
    group_commands: dict[tuple[int, ...], GroupCommand],
) -> float:
    if group_commands:
        first_key = sorted(group_commands)[0]
        return group_commands[first_key].command_yaw
    if controlled:
        return controlled[0].command_yaw
    return 0.0


def set_controller_target_now(item: ControlledDrone) -> None:
    item.args.target_x = item.command_target[0]
    item.args.target_y = item.command_target[1]
    item.args.target_z = item.command_target[2]
    item.args.target_yaw = item.command_yaw


def set_controlled_target(item: ControlledDrone, target: list[float], yaw: float, immediate: bool = False) -> None:
    item.command_target = [float(target[0]), float(target[1]), float(target[2])]
    item.command_yaw = wrap_pi(float(yaw))
    if immediate:
        set_controller_target_now(item)


def slew_vector(current: list[float], target: list[float], max_delta: float) -> list[float]:
    delta = [target[axis] - current[axis] for axis in range(3)]
    distance = math.sqrt(sum(value * value for value in delta))
    if distance <= max_delta or distance < 1e-12:
        return target[:]
    scale = max_delta / distance
    return [current[axis] + scale * delta[axis] for axis in range(3)]


def slew_angle(current: float, target: float, max_delta: float) -> float:
    delta = wrap_pi(target - current)
    if abs(delta) <= max_delta:
        return wrap_pi(target)
    return wrap_pi(current + math.copysign(max_delta, delta))


def advance_angle_with_rate(
    current: float,
    target: float,
    current_rate: float,
    max_rate: float,
    max_accel: float,
    dt: float,
) -> tuple[float, float]:
    if dt <= 0.0:
        return wrap_pi(current), current_rate
    delta = wrap_pi(target - current)
    if abs(delta) < 1e-8 and abs(current_rate) < 1e-8:
        return wrap_pi(target), 0.0

    limited_max_rate = max(0.0, float(max_rate))
    limited_max_accel = max(0.0, float(max_accel))
    desired_rate = position.clamp(delta / dt, -limited_max_rate, limited_max_rate)
    if limited_max_accel > 0.0:
        rate_step = position.clamp(
            desired_rate - current_rate,
            -limited_max_accel * dt,
            limited_max_accel * dt,
        )
        new_rate = current_rate + rate_step
    else:
        new_rate = desired_rate
    new_rate = position.clamp(new_rate, -limited_max_rate, limited_max_rate)

    step = new_rate * dt
    if delta * step > 0.0 and abs(step) >= abs(delta):
        return wrap_pi(target), 0.0
    return wrap_pi(current + step), new_rate


def advance_controlled_target(item: ControlledDrone, args: argparse.Namespace, dt: float) -> None:
    current = [float(item.args.target_x), float(item.args.target_y), float(item.args.target_z)]
    filtered = slew_vector(current, item.command_target, max(0.0, float(args.target_slew_speed)) * dt)
    item.args.target_x = filtered[0]
    item.args.target_y = filtered[1]
    item.args.target_z = filtered[2]
    item.args.target_yaw = slew_angle(
        float(item.args.target_yaw),
        item.command_yaw,
        math.radians(max(0.0, float(args.target_slew_yaw_deg))) * dt,
    )


def advance_group_target(command: GroupCommand, args: argparse.Namespace, dt: float) -> None:
    command.target = slew_vector(command.target, command.command_target, max(0.0, float(args.target_slew_speed)) * dt)
    roll_pitch_delta = math.radians(max(0.0, float(args.target_slew_roll_pitch_deg))) * dt
    command.roll = slew_angle(command.roll, command.command_roll, roll_pitch_delta)
    command.pitch = slew_angle(command.pitch, command.command_pitch, roll_pitch_delta)
    command.yaw, command.yaw_rate = advance_angle_with_rate(
        command.yaw,
        command.command_yaw,
        command.yaw_rate,
        math.radians(max(0.0, float(args.assembly_target_slew_yaw_deg))),
        math.radians(max(0.0, float(args.assembly_target_yaw_accel_deg))),
        dt,
    )


def reset_controller_integrators(controlled: list[ControlledDrone], omega: float, reset_motors: bool = False) -> None:
    for item in controlled:
        item.high_state = position.PositionControllerState(pos_integral=[0.0, 0.0, 0.0])
        item.low_state.rate_integral = [0.0, 0.0, 0.0]
        item.low_state.last_time = None
        if reset_motors:
            item.low_state.motor_speed = [omega, omega, omega, omega]


def reset_controlled_drone(sim, controlled: ControlledDrone, omega: float) -> None:
    multi.reset_drone(sim, controlled.drone, omega)
    pos = current_drone_position(sim, controlled)
    yaw = current_drone_yaw(sim, controlled)
    controlled.high_state = position.PositionControllerState(pos_integral=[0.0, 0.0, 0.0])
    controlled.low_state = flight.ControllerState(
        rate_integral=[0.0, 0.0, 0.0],
        motor_speed=[omega, omega, omega, omega],
        prop_phase=[0.0, 0.0, 0.0, 0.0],
    )
    controlled.args = make_drone_args(controlled.args, tuple(pos), yaw)
    set_controlled_target(controlled, pos, yaw, immediate=True)
    sim.setObjectPosition(controlled.target_handle, -1, pos)
    sim.setObjectOrientation(controlled.target_handle, -1, [0.0, 0.0, yaw])


def sync_target_visuals(
    sim,
    controlled: list[ControlledDrone],
    group_commands: dict[tuple[int, ...], GroupCommand],
) -> None:
    grouped_indices = {index for command in group_commands.values() for index in command.key}
    for item in controlled:
        is_grouped = item.drone.index in grouped_indices
        sim.setObjectPosition(item.target_handle, -1, item.command_target)
        sim.setObjectOrientation(item.target_handle, -1, [0.0, 0.0, item.command_yaw])
        set_visible(sim, item.target_handle, not is_grouped)
    for command in group_commands.values():
        sim.setObjectPosition(command.target_handle, -1, command.command_target)
        sim.setObjectOrientation(command.target_handle, -1, [command.command_roll, command.command_pitch, command.command_yaw])
        set_visible(sim, command.target_handle, True)


def build_controlled_drones(sim, drones: list[multi.DroneAgent], args: argparse.Namespace) -> list[ControlledDrone]:
    remove_old_targets(sim)
    omega = hover_omega(args)
    controlled = []
    for drone in drones:
        target = tuple(float(value) for value in sim.getObjectPosition(drone.body, -1))
        yaw = float(sim.getObjectOrientation(drone.body, -1)[2])
        controlled.append(
            ControlledDrone(
                drone=drone,
                high_state=position.PositionControllerState(pos_integral=[0.0, 0.0, 0.0]),
                low_state=flight.ControllerState(
                    rate_integral=[0.0, 0.0, 0.0],
                    motor_speed=[omega, omega, omega, omega],
                    prop_phase=[0.0, 0.0, 0.0, 0.0],
                ),
                args=make_drone_args(args, target, yaw),
                target_handle=create_target_sphere(sim, drone.index, target, args.target_radius),
                command_target=[float(target[0]), float(target[1]), float(target[2])],
                command_yaw=yaw,
            )
        )
    return controlled


def docking_groups_from_memory(drone_count: int, memory: multi.DockingMemory, magnets_enabled: bool) -> list[tuple[int, ...]]:
    if not magnets_enabled:
        return []
    parent = list(range(drone_count))

    def find(value: int) -> int:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    def union(a: int, b: int) -> None:
        root_a = find(a)
        root_b = find(b)
        if root_a != root_b:
            parent[root_b] = root_a

    for key in memory.latched:
        a_index, _a_connector, b_index, _b_connector = key
        if 0 <= a_index < drone_count and 0 <= b_index < drone_count:
            union(a_index, b_index)

    components: dict[int, list[int]] = {}
    for index in range(drone_count):
        components.setdefault(find(index), []).append(index)
    return sorted(tuple(values) for values in components.values() if len(values) > 1)


def face_for_connectors(geometry: multi.DockingGeometry, connector_ids: set[int]) -> multi.DockFace | None:
    exact = [face for face in geometry.faces if set(face.connector_ids) == connector_ids]
    if exact:
        return exact[0]
    containing = [face for face in geometry.faces if connector_ids.issubset(set(face.connector_ids))]
    if not containing:
        return None
    return min(containing, key=lambda face: len(face.connector_ids))


def configuration_for_group(
    group: tuple[int, ...],
    memory: multi.DockingMemory,
    geometry: multi.DockingGeometry,
) -> AssemblyConfiguration:
    group_set = set(group)
    links = []
    keys_by_pair: dict[tuple[int, int], list[multi.LatchKey]] = {}
    for key in memory.latched:
        a_index, _a_connector, b_index, _b_connector = key
        if a_index in group_set and b_index in group_set:
            keys_by_pair.setdefault((a_index, b_index), []).append(key)

    for (a_index, b_index), keys in sorted(keys_by_pair.items()):
        a_connectors = {key[1] for key in keys}
        b_connectors = {key[3] for key in keys}
        face_a = face_for_connectors(geometry, a_connectors)
        face_b = face_for_connectors(geometry, b_connectors)
        if face_a is not None and face_b is not None and face_a.face_type == face_b.face_type:
            face_type = face_a.face_type
        elif face_a is not None:
            face_type = face_a.face_type
        elif face_b is not None:
            face_type = face_b.face_type
        else:
            face_type = f"{len(keys)}-corner"
        links.append(
            DockedFaceLink(
                a_index=a_index,
                b_index=b_index,
                face_type=face_type,
                a_face_index=face_a.index if face_a is not None else None,
                b_face_index=face_b.index if face_b is not None else None,
                corner_count=len(keys),
            )
        )

    return AssemblyConfiguration(
        key=group,
        links=tuple(links),
        controller_name="assembly_geometric_wrench_allocator",
    )


def configurations_from_memory(
    groups: list[tuple[int, ...]],
    memory: multi.DockingMemory,
    geometry: multi.DockingGeometry,
) -> dict[tuple[int, ...], AssemblyConfiguration]:
    return {group: configuration_for_group(group, memory, geometry) for group in groups}


def configuration_summary(configuration: AssemblyConfiguration | None) -> str:
    if configuration is None or not configuration.links:
        return "configuration=pending"
    labels = []
    for link in configuration.links:
        face_a = "?" if link.a_face_index is None else str(link.a_face_index)
        face_b = "?" if link.b_face_index is None else str(link.b_face_index)
        labels.append(
            f"{link.face_type} d{link.a_index}:f{face_a}<->d{link.b_index}:f{face_b} "
            f"({link.corner_count} corners)"
        )
    return "; ".join(labels)


def heading_summary(controlled: list[ControlledDrone], key: tuple[int, ...], assembly_yaw: float) -> str:
    parts = []
    for index in key:
        relative_yaw = math.degrees(wrap_pi(controlled[index].command_yaw - assembly_yaw))
        parts.append(f"d{index}:{math.degrees(controlled[index].command_yaw): .0f}deg rel={relative_yaw: .0f}deg")
    return ", ".join(parts)


def assembly_yaw_from_leader(sim, controlled: list[ControlledDrone], key: tuple[int, ...]) -> float:
    return current_drone_yaw(sim, controlled[key[0]])


def command_from_current_group(
    sim,
    controlled: list[ControlledDrone],
    key: tuple[int, ...],
    target_handle: int,
    configuration: AssemblyConfiguration | None,
) -> GroupCommand:
    positions = [current_drone_position(sim, controlled[index]) for index in key]
    target = average_positions(positions)
    yaw = assembly_yaw_from_leader(sim, controlled, key)
    local_offsets = {
        index: rotate_z([positions[slot][axis] - target[axis] for axis in range(3)], -yaw)
        for slot, index in enumerate(key)
    }
    yaw_offsets = {
        index: wrap_pi(current_drone_yaw(sim, controlled[index]) - yaw)
        for index in key
    }
    state = assembly_geometric.AssemblyControllerState(pos_integral=[0.0, 0.0, 0.0])
    return GroupCommand(
        key=key,
        target=target,
        roll=0.0,
        pitch=0.0,
        yaw=yaw,
        command_target=target[:],
        command_roll=0.0,
        command_pitch=0.0,
        command_yaw=yaw,
        yaw_rate=0.0,
        local_offsets=local_offsets,
        yaw_offsets=yaw_offsets,
        state=state,
        target_handle=target_handle,
        configuration=configuration,
    )


def apply_group_command(controlled: list[ControlledDrone], command: GroupCommand, immediate: bool = False) -> None:
    for index in command.key:
        offset = rotate_z(command.local_offsets[index], command.yaw)
        target = [command.target[axis] + offset[axis] for axis in range(3)]
        member_yaw = wrap_pi(command.yaw + command.yaw_offsets[index])
        set_controlled_target(controlled[index], target, member_yaw, immediate=immediate)


def update_group_command_to_current(
    sim,
    controlled: list[ControlledDrone],
    command: GroupCommand,
    args: argparse.Namespace,
    configuration: AssemblyConfiguration | None = None,
) -> None:
    refreshed = command_from_current_group(
        sim,
        controlled,
        command.key,
        command.target_handle,
        configuration if configuration is not None else command.configuration,
    )
    command.target = refreshed.target
    command.roll = refreshed.roll
    command.pitch = refreshed.pitch
    command.yaw = refreshed.yaw
    command.command_target = refreshed.command_target
    command.command_roll = refreshed.command_roll
    command.command_pitch = refreshed.command_pitch
    command.command_yaw = refreshed.command_yaw
    command.yaw_rate = refreshed.yaw_rate
    command.local_offsets = refreshed.local_offsets
    command.yaw_offsets = refreshed.yaw_offsets
    command.configuration = refreshed.configuration
    assembly_geom = allocation_geometry.build_assembly_geometry(
        sim,
        [controlled[index].drone for index in command.key],
        args.mass,
        module_inertia_box(args),
    )
    assembly_geometric.reset_to_current(command.state, assembly_geom, command.yaw, command.roll, command.pitch)
    apply_group_command(controlled, command, immediate=True)


def clear_group_commands(sim, group_commands: dict[tuple[int, ...], GroupCommand]) -> None:
    for command in group_commands.values():
        sim.removeObjects([command.target_handle], False)
    group_commands.clear()


def refresh_group_commands(
    sim,
    controlled: list[ControlledDrone],
    groups: list[tuple[int, ...]],
    group_commands: dict[tuple[int, ...], GroupCommand],
    configurations: dict[tuple[int, ...], AssemblyConfiguration],
    args: argparse.Namespace,
    omega: float,
) -> tuple[list[tuple[int, ...]], list[tuple[int, ...]]]:
    current = set(groups)
    old = set(group_commands)
    removed = sorted(old - current)
    added = sorted(current - old)

    for key in removed:
        for index in key:
            pos = current_drone_position(sim, controlled[index])
            yaw = current_drone_yaw(sim, controlled[index])
            set_controlled_target(controlled[index], pos, yaw, immediate=True)
        reset_controller_integrators([controlled[index] for index in key], omega)
        sim.removeObjects([group_commands[key].target_handle], False)
        del group_commands[key]

    for key in added:
        positions = [current_drone_position(sim, controlled[index]) for index in key]
        target_handle = create_group_target_sphere(sim, key, average_positions(positions), args.target_radius)
        command = command_from_current_group(sim, controlled, key, target_handle, configurations.get(key))
        assembly_geom = allocation_geometry.build_assembly_geometry(
            sim,
            [controlled[index].drone for index in key],
            args.mass,
            module_inertia_box(args),
        )
        assembly_geometric.reset_to_current(command.state, assembly_geom, command.yaw, command.roll, command.pitch)
        apply_group_command(controlled, command, immediate=True)
        reset_controller_integrators([controlled[index] for index in key], omega)
        group_commands[key] = command

    for key in sorted(current & old):
        group_commands[key].configuration = configurations.get(key)

    return added, removed


def set_targets_to_current_all(
    sim,
    controlled: list[ControlledDrone],
    group_commands: dict[tuple[int, ...], GroupCommand],
    args: argparse.Namespace,
) -> None:
    for item in controlled:
        set_controlled_target(item, current_drone_position(sim, item), current_drone_yaw(sim, item), immediate=True)
    for command in group_commands.values():
        update_group_command_to_current(sim, controlled, command, args)


def set_selection_to_current(
    sim,
    selection: Selection,
    controlled: list[ControlledDrone],
    group_commands: dict[tuple[int, ...], GroupCommand],
    args: argparse.Namespace,
) -> None:
    if selection.kind == "all":
        set_targets_to_current_all(sim, controlled, group_commands, args)
        return
    if selection.kind == "drone" and selection.index is not None:
        item = controlled[selection.index]
        set_controlled_target(item, current_drone_position(sim, item), current_drone_yaw(sim, item), immediate=True)
        return
    if selection.kind == "group" and selection.group in group_commands:
        update_group_command_to_current(sim, controlled, group_commands[selection.group], args)


def distance_between(sim, controlled: list[ControlledDrone], a_index: int, b_index: int) -> float:
    a_pos = current_drone_position(sim, controlled[a_index])
    b_pos = current_drone_position(sim, controlled[b_index])
    return math.sqrt(sum((a_pos[axis] - b_pos[axis]) ** 2 for axis in range(3)))


def nearest_pair(sim, controlled: list[ControlledDrone], candidates: list[int]) -> tuple[int, int] | None:
    best_pair: tuple[int, int] | None = None
    best_distance = float("inf")
    for i, a_index in enumerate(candidates):
        for b_index in candidates[i + 1 :]:
            distance = distance_between(sim, controlled, a_index, b_index)
            if distance < best_distance:
                best_distance = distance
                best_pair = (a_index, b_index)
    return best_pair


def grouped_indices(group_commands: dict[tuple[int, ...], GroupCommand]) -> set[int]:
    return {index for command in group_commands.values() for index in command.key}


def quick_pair_from_selection(
    sim,
    selection: Selection,
    controlled: list[ControlledDrone],
    group_commands: dict[tuple[int, ...], GroupCommand],
    prefer_free: bool,
) -> tuple[int, int] | None:
    if len(controlled) < 2:
        return None
    if selection.kind == "group" and len(selection.group) >= 2:
        return selection.group[0], selection.group[1]
    if selection.kind == "drone" and selection.index is not None:
        candidates = [index for index in range(len(controlled)) if index != selection.index]
        if not candidates:
            return None
        nearest_other = min(candidates, key=lambda index: distance_between(sim, controlled, selection.index, index))
        return selection.index, nearest_other

    if prefer_free:
        free = [index for index in range(len(controlled)) if index not in grouped_indices(group_commands)]
        pair = nearest_pair(sim, controlled, free)
        if pair is not None:
            return pair
    return nearest_pair(sim, controlled, list(range(len(controlled))))


def command_pair_spacing(
    sim,
    controlled: list[ControlledDrone],
    pair: tuple[int, int],
    spacing: float,
    axis: str,
    yaw_a: float,
    yaw_b: float,
) -> None:
    a_index, b_index = pair
    positions = [current_drone_position(sim, controlled[a_index]), current_drone_position(sim, controlled[b_index])]
    center = average_positions(positions)
    offset_a, offset_b = pair_offsets(axis, spacing)
    target_a = [center[axis_i] + offset_a[axis_i] for axis_i in range(3)]
    target_b = [center[axis_i] + offset_b[axis_i] for axis_i in range(3)]
    set_controlled_target(controlled[a_index], target_a, yaw_a)
    set_controlled_target(controlled[b_index], target_b, yaw_b)


def quick_square_dock(
    sim,
    selection: Selection,
    controlled: list[ControlledDrone],
    group_commands: dict[tuple[int, ...], GroupCommand],
    geometry: multi.DockingGeometry,
    args: argparse.Namespace,
    yaw_mode: str,
) -> QuickDockResult | None:
    pair = quick_pair_from_selection(sim, selection, controlled, group_commands, prefer_free=True)
    if pair is None:
        return None
    yaw_a, yaw_b = quick_dock_yaws(args.quick_dock_axis, yaw_mode)
    face_a, face_b = choose_quick_square_faces(geometry, args.quick_dock_axis, yaw_a, yaw_b)
    command_square_face_dock_pose(
        sim,
        controlled,
        pair,
        face_a,
        face_b,
        yaw_a,
        yaw_b,
        max(args.latch_rest_distance, args.magnet_rest_distance),
    )
    return QuickDockResult(pair=pair, face_a=face_a, face_b=face_b, yaw_a=yaw_a, yaw_b=yaw_b)


def quick_open_pair(
    sim,
    selection: Selection,
    controlled: list[ControlledDrone],
    group_commands: dict[tuple[int, ...], GroupCommand],
    args: argparse.Namespace,
) -> tuple[int, int] | None:
    pair = quick_pair_from_selection(sim, selection, controlled, group_commands, prefer_free=False)
    if pair is None:
        return None
    command_pair_spacing(
        sim,
        controlled,
        pair,
        args.quick_open_spacing,
        args.quick_dock_axis,
        current_drone_yaw(sim, controlled[pair[0]]),
        current_drone_yaw(sim, controlled[pair[1]]),
    )
    return pair


class MultiDronePositionUI:
    def __init__(self, args: argparse.Namespace, drone_count: int) -> None:
        self.args = args
        self.running = True
        self.root = tk.Tk()
        self.root.title("Truncated-Octahedral Drone Station")
        self.root.geometry("1180x760")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.magnets_enabled = tk.BooleanVar(value=args.magnets_start_enabled)
        self.docked_controller_enabled = tk.BooleanVar(value=args.docked_controller)
        self.auto_select_assembly_enabled = tk.BooleanVar(value=True)
        self.quick_dock_yaw_mode_var = tk.StringVar(value=args.quick_dock_yaw_mode)
        self.selection_var = tk.StringVar(value="all")
        self.x_var = tk.DoubleVar(value=0.0)
        self.y_var = tk.DoubleVar(value=0.0)
        self.z_var = tk.DoubleVar(value=0.0)
        self.roll_deg_var = tk.DoubleVar(value=0.0)
        self.pitch_deg_var = tk.DoubleVar(value=0.0)
        self.yaw_deg_var = tk.DoubleVar(value=0.0)
        self.linear_speed_var = tk.DoubleVar(value=args.ui_step)
        self.attitude_speed_var = tk.DoubleVar(value=args.attitude_ui_step_deg)
        self.yaw_speed_var = tk.DoubleVar(value=args.yaw_ui_step_deg)
        self.assembly_yaw_speed_var = tk.DoubleVar(value=args.assembly_target_slew_yaw_deg)
        self.assembly_yaw_accel_var = tk.DoubleVar(value=args.assembly_target_yaw_accel_deg)
        self.assembly_yaw_error_limit_var = tk.DoubleVar(value=args.assembly_yaw_error_limit_deg)

        self.reset_requested = False
        self.release_latches_requested = False
        self.detach_and_hold_requested = False
        self.targets_to_current_requested = False
        self.all_targets_to_current_requested = False
        self.quick_square_dock_requested = False
        self.quick_open_pair_requested = False
        self.selection_changed_requested = False
        self.command_dirty = False
        self.quick_dock_status = "Quick dock: not commanded"
        self._selection_map: dict[str, Selection] = {}
        self._motion_keys = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}
        self._motion_buttons = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}
        self._last_groups: list[tuple[int, ...]] = []

        self._build_layout()
        self.set_options(drone_count, [])
        self._bind_keys()

    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        controls = ttk.Frame(self.root, padding=10)
        controls.grid(row=0, column=0, sticky="ns")
        status = ttk.Frame(self.root, padding=10)
        status.grid(row=0, column=1, sticky="nsew")
        status.columnconfigure(0, weight=1)
        status.rowconfigure(1, weight=1)

        ttk.Label(controls, text="Control Scope").grid(row=0, column=0, sticky="w")
        self.selection_combo = ttk.Combobox(controls, textvariable=self.selection_var, state="readonly", width=26)
        self.selection_combo.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(2, 10))
        self.selection_combo.bind("<<ComboboxSelected>>", self._selection_changed)

        target = ttk.LabelFrame(controls, text="Target")
        target.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(0, 10))
        for row, (label, variable) in enumerate(
            (
                ("X [m]", self.x_var),
                ("Y [m]", self.y_var),
                ("Z [m]", self.z_var),
                ("Roll [deg]", self.roll_deg_var),
                ("Pitch [deg]", self.pitch_deg_var),
                ("Yaw [deg]", self.yaw_deg_var),
            )
        ):
            ttk.Label(target, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=3)
            entry = ttk.Entry(target, textvariable=variable, width=12)
            entry.grid(row=row, column=1, sticky="ew", padx=4, pady=3)
            entry.bind("<Return>", self._mark_command_dirty)
            entry.bind("<FocusOut>", self._mark_command_dirty)
        ttk.Button(target, text="Apply", command=self._mark_command_dirty).grid(row=6, column=0, columnspan=2, sticky="ew", padx=4, pady=(6, 4))
        ttk.Button(target, text="Selection = Current", command=self.request_targets_to_current).grid(
            row=7,
            column=0,
            columnspan=2,
            sticky="ew",
            padx=4,
            pady=3,
        )
        ttk.Button(target, text="All = Current", command=self.request_all_targets_to_current).grid(
            row=8,
            column=0,
            columnspan=2,
            sticky="ew",
            padx=4,
            pady=3,
        )
        target.columnconfigure(1, weight=1)

        jog = ttk.LabelFrame(controls, text="Continuous Jog")
        jog.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(0, 10))
        self._jog_button(jog, "-X", "x", -1, 0, 0)
        self._jog_button(jog, "+X", "x", 1, 0, 1)
        self._jog_button(jog, "-Y", "y", -1, 1, 0)
        self._jog_button(jog, "+Y", "y", 1, 1, 1)
        self._jog_button(jog, "-Z", "z", -1, 2, 0)
        self._jog_button(jog, "+Z", "z", 1, 2, 1)
        self._jog_button(jog, "-Roll", "roll", -1, 3, 0)
        self._jog_button(jog, "+Roll", "roll", 1, 3, 1)
        self._jog_button(jog, "-Pitch", "pitch", -1, 4, 0)
        self._jog_button(jog, "+Pitch", "pitch", 1, 4, 1)
        self._jog_button(jog, "-Yaw", "yaw", -1, 5, 0)
        self._jog_button(jog, "+Yaw", "yaw", 1, 5, 1)
        ttk.Label(jog, text="Linear speed").grid(row=6, column=0, sticky="w", padx=4, pady=(8, 2))
        ttk.Scale(jog, variable=self.linear_speed_var, from_=0.05, to=1.50, orient=tk.HORIZONTAL).grid(
            row=6,
            column=1,
            sticky="ew",
            padx=4,
            pady=(8, 2),
        )
        ttk.Label(jog, text="Attitude speed").grid(row=7, column=0, sticky="w", padx=4, pady=2)
        ttk.Scale(jog, variable=self.attitude_speed_var, from_=5.0, to=180.0, orient=tk.HORIZONTAL).grid(
            row=7,
            column=1,
            sticky="ew",
            padx=4,
            pady=2,
        )
        ttk.Label(jog, text="Yaw speed").grid(row=8, column=0, sticky="w", padx=4, pady=2)
        ttk.Scale(jog, variable=self.yaw_speed_var, from_=10.0, to=220.0, orient=tk.HORIZONTAL).grid(
            row=8,
            column=1,
            sticky="ew",
            padx=4,
            pady=2,
        )
        ttk.Label(jog, text="Dock yaw speed").grid(row=9, column=0, sticky="w", padx=4, pady=(8, 2))
        ttk.Scale(jog, variable=self.assembly_yaw_speed_var, from_=2.0, to=30.0, orient=tk.HORIZONTAL).grid(
            row=9,
            column=1,
            sticky="ew",
            padx=4,
            pady=(8, 2),
        )
        ttk.Label(jog, text="Dock yaw accel").grid(row=10, column=0, sticky="w", padx=4, pady=2)
        ttk.Scale(jog, variable=self.assembly_yaw_accel_var, from_=5.0, to=100.0, orient=tk.HORIZONTAL).grid(
            row=10,
            column=1,
            sticky="ew",
            padx=4,
            pady=2,
        )
        ttk.Label(jog, text="Yaw error clamp").grid(row=11, column=0, sticky="w", padx=4, pady=2)
        ttk.Scale(jog, variable=self.assembly_yaw_error_limit_var, from_=2.0, to=20.0, orient=tk.HORIZONTAL).grid(
            row=11,
            column=1,
            sticky="ew",
            padx=4,
            pady=2,
        )
        jog.columnconfigure(1, weight=1)

        docking = ttk.LabelFrame(controls, text="Docking")
        docking.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(0, 10))
        ttk.Checkbutton(docking, text="Magnets enabled", variable=self.magnets_enabled).grid(row=0, column=0, sticky="w", padx=4, pady=3)
        ttk.Checkbutton(docking, text="Assembly controller", variable=self.docked_controller_enabled).grid(
            row=1,
            column=0,
            sticky="w",
            padx=4,
            pady=3,
        )
        ttk.Checkbutton(docking, text="Auto-select new assembly", variable=self.auto_select_assembly_enabled).grid(
            row=2,
            column=0,
            sticky="w",
            padx=4,
            pady=3,
        )
        ttk.Label(docking, text="Quick dock heading").grid(row=3, column=0, sticky="w", padx=4, pady=(8, 2))
        self.quick_dock_yaw_combo = ttk.Combobox(
            docking,
            textvariable=self.quick_dock_yaw_mode_var,
            state="readonly",
            values=("fronts-in", "fronts-out", "same-forward", "same-reverse", "lateral-left", "lateral-right"),
            width=22,
        )
        self.quick_dock_yaw_combo.grid(row=4, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(docking, text="Quick Square Dock", command=self.request_quick_square_dock).grid(
            row=5,
            column=0,
            sticky="ew",
            padx=4,
            pady=(8, 3),
        )
        ttk.Button(docking, text="Open Pair", command=self.request_quick_open_pair).grid(row=6, column=0, sticky="ew", padx=4, pady=3)
        ttk.Button(docking, text="Release Latches", command=self.request_release_latches).grid(row=7, column=0, sticky="ew", padx=4, pady=3)
        ttk.Button(docking, text="Detach And Hold", command=self.request_detach_and_hold).grid(row=8, column=0, sticky="ew", padx=4, pady=3)
        ttk.Button(docking, text="Reset Scene", command=self.request_reset).grid(row=9, column=0, sticky="ew", padx=4, pady=(8, 4))
        docking.columnconfigure(0, weight=1)

        ttk.Label(status, text="Live State").grid(row=0, column=0, sticky="w")
        self.status_text = tk.Text(status, width=105, height=38, wrap="none", font=("Consolas", 10))
        self.status_text.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
        self.status_text.configure(state="disabled")

    def _jog_button(self, parent: tk.Widget, text: str, axis: str, direction: int, row: int, column: int) -> None:
        button = ttk.Button(parent, text=text)
        button.grid(row=row, column=column, sticky="ew", padx=4, pady=3)
        button.bind("<ButtonPress-1>", lambda _event: self.set_button_motion(axis, direction))
        button.bind("<ButtonRelease-1>", lambda _event: self.set_button_motion(axis, 0))
        button.bind("<Leave>", lambda _event: self.set_button_motion(axis, 0))

    def _bind_keys(self) -> None:
        bindings = {
            "<KeyPress-Up>": ("y", 1),
            "<KeyRelease-Up>": ("y", 0),
            "<KeyPress-Down>": ("y", -1),
            "<KeyRelease-Down>": ("y", 0),
            "<KeyPress-Right>": ("x", 1),
            "<KeyRelease-Right>": ("x", 0),
            "<KeyPress-Left>": ("x", -1),
            "<KeyRelease-Left>": ("x", 0),
            "<KeyPress-Prior>": ("z", 1),
            "<KeyRelease-Prior>": ("z", 0),
            "<KeyPress-Next>": ("z", -1),
            "<KeyRelease-Next>": ("z", 0),
            "<KeyPress-a>": ("roll", -1),
            "<KeyRelease-a>": ("roll", 0),
            "<KeyPress-d>": ("roll", 1),
            "<KeyRelease-d>": ("roll", 0),
            "<KeyPress-w>": ("pitch", 1),
            "<KeyRelease-w>": ("pitch", 0),
            "<KeyPress-s>": ("pitch", -1),
            "<KeyRelease-s>": ("pitch", 0),
            "<KeyPress-q>": ("yaw", 1),
            "<KeyRelease-q>": ("yaw", 0),
            "<KeyPress-e>": ("yaw", -1),
            "<KeyRelease-e>": ("yaw", 0),
        }
        for event, (axis, direction) in bindings.items():
            self.root.bind(event, lambda _event, a=axis, d=direction: self.set_key_motion(a, d))

    def set_options(
        self,
        drone_count: int,
        groups: list[tuple[int, ...]],
        group_commands: dict[tuple[int, ...], GroupCommand] | None = None,
    ) -> None:
        labels = ["all"]
        mapping = {"all": Selection("all")}
        for index in range(drone_count):
            label = f"d{index}"
            labels.append(label)
            mapping[label] = Selection("drone", index=index)
        for group_index, key in enumerate(groups):
            command = group_commands.get(key) if group_commands is not None else None
            label = self.group_label(group_index, key, command)
            labels.append(label)
            mapping[label] = Selection("group", group=key)

        previous_selection = self.selection()
        self._selection_map = mapping
        self.selection_combo.configure(values=labels)
        next_label = "all"
        for label, selection in mapping.items():
            if selection == previous_selection:
                next_label = label
                break
        self.selection_var.set(next_label)
        self._last_groups = groups[:]

    def group_label(self, group_index: int, key: tuple[int, ...], command: GroupCommand | None = None) -> str:
        members = ",".join(str(index) for index in key)
        if command is not None and command.configuration is not None and command.configuration.links:
            face_type = command.configuration.links[0].face_type
            return f"g{group_index} d[{members}] {face_type}"
        return f"g{group_index} d[{members}]"

    def select_group(
        self,
        key: tuple[int, ...],
        drone_count: int,
        groups: list[tuple[int, ...]],
        group_commands: dict[tuple[int, ...], GroupCommand],
    ) -> None:
        self.set_options(drone_count, groups, group_commands)
        for label, selection in self._selection_map.items():
            if selection.kind == "group" and selection.group == key:
                self.selection_var.set(label)
                self.selection_changed_requested = True
                break

    def selection(self) -> Selection:
        return self._selection_map.get(self.selection_var.get(), Selection("all"))

    def _selection_changed(self, _event=None) -> None:
        self.selection_changed_requested = True
        self.command_dirty = False

    def _mark_command_dirty(self, _event=None) -> None:
        self.command_dirty = True

    def set_key_motion(self, axis: str, direction: int) -> None:
        self._motion_keys[axis] = direction

    def set_button_motion(self, axis: str, direction: int) -> None:
        self._motion_buttons[axis] = direction

    def motion_direction(self, axis: str) -> int:
        key_direction = self._motion_keys[axis]
        button_direction = self._motion_buttons[axis]
        return button_direction if button_direction else key_direction

    def advance_continuous_motion(self, dt: float) -> None:
        linear_delta = max(0.0, self.linear_speed_var.get()) * dt
        attitude_delta = max(0.0, self.attitude_speed_var.get()) * dt
        yaw_delta = max(0.0, self.yaw_speed_var.get()) * dt
        moved = False
        for axis, variable in (("x", self.x_var), ("y", self.y_var), ("z", self.z_var)):
            direction = self.motion_direction(axis)
            if direction:
                variable.set(variable.get() + direction * linear_delta)
                moved = True
        for axis, variable in (("roll", self.roll_deg_var), ("pitch", self.pitch_deg_var)):
            direction = self.motion_direction(axis)
            if direction:
                variable.set(variable.get() + direction * attitude_delta)
                moved = True
        yaw_direction = self.motion_direction("yaw")
        if yaw_direction:
            self.yaw_deg_var.set(self.yaw_deg_var.get() + yaw_direction * yaw_delta)
            moved = True
        if moved:
            self.command_dirty = True

    def sync_runtime_tuning(self, args: argparse.Namespace) -> None:
        args.assembly_target_slew_yaw_deg = max(0.0, float(self.assembly_yaw_speed_var.get()))
        args.assembly_target_yaw_accel_deg = max(0.0, float(self.assembly_yaw_accel_var.get()))
        args.assembly_yaw_error_limit_deg = max(0.0, float(self.assembly_yaw_error_limit_var.get()))

    def load_selection_target(
        self,
        controlled: list[ControlledDrone],
        group_commands: dict[tuple[int, ...], GroupCommand],
    ) -> None:
        selection = self.selection()
        if selection.kind == "drone" and selection.index is not None:
            item = controlled[selection.index]
            target = item.command_target
            roll = 0.0
            pitch = 0.0
            yaw = item.command_yaw
        elif selection.kind == "group" and selection.group in group_commands:
            command = group_commands[selection.group]
            target = command.command_target
            roll = command.command_roll
            pitch = command.command_pitch
            yaw = command.command_yaw
        else:
            targets = [item.command_target for item in controlled]
            target = average_positions(targets)
            roll = 0.0
            pitch = 0.0
            yaw = representative_command_yaw(controlled, group_commands)

        self.x_var.set(target[0])
        self.y_var.set(target[1])
        self.z_var.set(target[2])
        self.roll_deg_var.set(math.degrees(roll))
        self.pitch_deg_var.set(math.degrees(pitch))
        self.yaw_deg_var.set(math.degrees(yaw))
        self.command_dirty = False

    def command_target(self) -> tuple[list[float], float, float, float]:
        return (
            [self.x_var.get(), self.y_var.get(), self.z_var.get()],
            math.radians(self.roll_deg_var.get()),
            math.radians(self.pitch_deg_var.get()),
            math.radians(self.yaw_deg_var.get()),
        )

    def apply_current_command(
        self,
        controlled: list[ControlledDrone],
        group_commands: dict[tuple[int, ...], GroupCommand],
    ) -> None:
        target, roll, pitch, yaw = self.command_target()
        selection = self.selection()
        if selection.kind == "drone" and selection.index is not None:
            set_controlled_target(controlled[selection.index], target, yaw)
        elif selection.kind == "group" and selection.group in group_commands:
            command = group_commands[selection.group]
            command.command_target = target
            command.command_roll = wrap_pi(roll)
            command.command_pitch = wrap_pi(pitch)
            command.command_yaw = wrap_pi(yaw)
            apply_group_command(controlled, command)
        else:
            current_center = average_positions([item.command_target for item in controlled])
            delta = [target[axis] - current_center[axis] for axis in range(3)]
            for item in controlled:
                shifted = [item.command_target[axis] + delta[axis] for axis in range(3)]
                set_controlled_target(item, shifted, yaw)
            for command in group_commands.values():
                command.command_target = [command.command_target[axis] + delta[axis] for axis in range(3)]
                command.command_roll = wrap_pi(roll)
                command.command_pitch = wrap_pi(pitch)
                command.command_yaw = wrap_pi(yaw)
                apply_group_command(controlled, command)
        self.command_dirty = False

    def request_reset(self) -> None:
        self.reset_requested = True

    def request_targets_to_current(self) -> None:
        self.targets_to_current_requested = True

    def request_all_targets_to_current(self) -> None:
        self.all_targets_to_current_requested = True

    def request_quick_square_dock(self) -> None:
        self.quick_square_dock_requested = True

    def request_quick_open_pair(self) -> None:
        self.quick_open_pair_requested = True

    def request_release_latches(self) -> None:
        self.release_latches_requested = True

    def request_detach_and_hold(self) -> None:
        self.detach_and_hold_requested = True

    def close(self) -> None:
        self.running = False
        if self.root.winfo_exists():
            self.root.destroy()

    def pump_events(self) -> None:
        if not self.running:
            return
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self.running = False

    def update(
        self,
        sim_time: float,
        controlled: list[ControlledDrone],
        groups: list[tuple[int, ...]],
        group_commands: dict[tuple[int, ...], GroupCommand],
        high_samples: list[dict[str, object] | None],
        low_samples: list[dict[str, object] | None],
        docking_sample: dict[str, object],
    ) -> None:
        if groups != self._last_groups:
            self.set_options(len(controlled), groups, group_commands)

        group_by_drone: dict[int, str] = {}
        for group_index, key in enumerate(groups):
            label = self.group_label(group_index, key, group_commands.get(key))
            for index in key:
                group_by_drone[index] = label.split(" ", 1)[0]

        lines = [
            f"time={sim_time:6.2f}s  magnets={int(self.magnets_enabled.get())}  "
            f"assembly_controller={int(self.docked_controller_enabled.get())}  "
            f"rtf={float(docking_sample.get('real_time_factor', 0.0)):4.2f}  "
            f"docking={docking_sample.get('mode', 'unknown')}  "
            f"latched_pairs={docking_sample.get('latched_pairs', 0)}",
            f"docked_yaw: speed={self.assembly_yaw_speed_var.get():.1f}deg/s  "
            f"accel={self.assembly_yaw_accel_var.get():.1f}deg/s^2  "
            f"error_clamp={self.assembly_yaw_error_limit_var.get():.1f}deg",
        ]
        if groups:
            lines.append("Docked groups:")
            for group_index, key in enumerate(groups):
                command = group_commands.get(key)
                if command is None:
                    continue
                assembly_pos = None
                assembly_error = None
                control_gain = 0.0
                allocation_rank = 0
                allocation_residual = 0.0
                allocation_weighted_residual = 0.0
                allocation_saturated = 0
                auto_roll = 0.0
                auto_pitch = 0.0
                force_alignment_angle = 0.0
                force_alignment_limited = False
                attitude_coupling_mode = "unknown"
                attitude_error_frame = "unknown"
                first_sample = high_samples[key[0]]
                if first_sample is not None:
                    assembly_pos = first_sample.get("assembly_pos")
                    assembly_error = first_sample.get("assembly_pos_error")
                    control_gain = float(first_sample.get("control_gain", 0.0))
                    allocation_rank = int(first_sample.get("allocation_rank", 0))
                    allocation_residual = float(first_sample.get("allocation_residual_norm", 0.0))
                    allocation_weighted_residual = float(
                        first_sample.get("allocation_weighted_residual_norm", 0.0)
                    )
                    allocation_saturated = int(first_sample.get("allocation_saturated", 0))
                    auto_roll = float(first_sample.get("auto_roll", 0.0))
                    auto_pitch = float(first_sample.get("auto_pitch", 0.0))
                    force_alignment_angle = float(first_sample.get("force_alignment_angle", 0.0))
                    force_alignment_limited = bool(first_sample.get("force_alignment_limited", False))
                    attitude_coupling_mode = str(first_sample.get("attitude_coupling_mode", "unknown"))
                    attitude_error_frame = str(first_sample.get("attitude_error_frame", "unknown"))
                lines.append(
                    f"  {self.group_label(group_index, key, command):18s} "
                    f"COM={format_vector(assembly_pos if assembly_pos is not None else command.target)} "
                    f"target={format_vector(command.target)} "
                    f"err={format_vector(assembly_error if assembly_error is not None else [0.0, 0.0, 0.0])} "
                    f"rpy=[{math.degrees(command.roll): .1f}, "
                    f"{math.degrees(command.pitch): .1f}, {math.degrees(command.yaw): .1f}] deg "
                    f"yaw_rate={math.degrees(command.yaw_rate): .1f}deg/s"
                )
                lines.append(
                    f"    config: {configuration_summary(command.configuration)} | "
                    f"controller={command.configuration.controller_name if command.configuration else 'pending'}"
                )
                lines.append(
                    f"    assembly: gain={control_gain:.2f} alloc_rank={allocation_rank} "
                    f"residual={allocation_residual:.3f} weighted={allocation_weighted_residual:.3f} "
                    f"saturated_motors={allocation_saturated} "
                    f"coupling={attitude_coupling_mode}/{attitude_error_frame} "
                    f"align={math.degrees(force_alignment_angle):.1f}deg "
                    f"limited={int(force_alignment_limited)} "
                    f"auto_rp=[{math.degrees(auto_roll): .1f}, {math.degrees(auto_pitch): .1f}] deg"
                )
                lines.append(f"    member heading: {heading_summary(controlled, key, command.yaw)}")
        else:
            lines.append("Docked groups: none")

        if docking_sample.get("last_latch_event"):
            lines.append(f"last latch: {docking_sample['last_latch_event']}")
        if docking_sample.get("last_break_reason"):
            lines.append(f"last break: {docking_sample['last_break_reason']}")
        lines.append(self.quick_dock_status)

        lines.extend(
            [
                "",
                "id  group  yaw   mode                true_xyz                 target_xyz               error_xyz                T[N]   rpm[0..3]",
                "--  -----  ----  ------------------  -----------------------  -----------------------  -----------------------  -----  -----------------------",
            ]
        )
        for index, item in enumerate(controlled):
            high = high_samples[index]
            low = low_samples[index]
            if high is None or low is None:
                continue
            pos = high.get("pos", [0.0, 0.0, 0.0])
            target = high.get("target", item.command_target)
            error = high.get("pos_error", [0.0, 0.0, 0.0])
            thrust = float(low.get("total_thrust", 0.0))
            omega = low.get("omega", [0.0, 0.0, 0.0, 0.0])
            rpm = [multi.rad_s_to_rpm(float(value)) for value in omega]
            group_name = group_by_drone.get(index, "-")
            mode = str(high.get("control_mode", "independent"))
            yaw_deg = math.degrees(item.command_yaw)
            lines.append(
                f"d{index:<1d}  {group_name:<5s}  {yaw_deg:4.0f}  {mode:<18s}  "
                f"{format_vector(pos):23s}  {format_vector(target):23s}  {format_vector(error):23s}  "
                f"{thrust:5.3f}  "
                f"[{rpm[0]:5.0f},{rpm[1]:5.0f},{rpm[2]:5.0f},{rpm[3]:5.0f}]"
            )

        self.status_text.configure(state="normal")
        self.status_text.delete("1.0", tk.END)
        self.status_text.insert(tk.END, "\n".join(lines))
        self.status_text.configure(state="disabled")


def docked_high_samples_for_ui(
    sim,
    controlled: list[ControlledDrone],
    assembly_sample: dict[str, object],
    allocation: wrench_allocator.AllocationResult,
) -> list[dict[str, object]]:
    samples = []
    assembly_pos = assembly_sample["pos"]
    assembly_target = assembly_sample["target"]
    assembly_pos_error = assembly_sample["pos_error"]
    for index, item in enumerate(controlled):
        pos = list(sim.getObjectPosition(item.drone.body, -1))
        lin_vel = list(sim.getVelocity(item.drone.body)[0])
        member_target = [float(item.args.target_x), float(item.args.target_y), float(item.args.target_z)]
        motor_slice = slice(4 * index, 4 * index + 4)
        drone_thrust_cmd = sum(allocation.motor_thrust_cmd[motor_slice])
        sample = dict(assembly_sample)
        sample.update(
            {
                "pos": pos,
                "target": assembly_target,
                "member_target": member_target,
                "target_roll": assembly_sample.get("target_roll", 0.0),
                "target_pitch": assembly_sample.get("target_pitch", 0.0),
                "target_yaw": item.args.target_yaw,
                "auto_roll": assembly_sample.get("auto_roll", 0.0),
                "auto_pitch": assembly_sample.get("auto_pitch", 0.0),
                "desired_roll": assembly_sample.get("desired_roll", 0.0),
                "desired_pitch": assembly_sample.get("desired_pitch", 0.0),
                "attitude_coupling_mode": assembly_sample.get("attitude_coupling_mode", "unknown"),
                "attitude_error_frame": assembly_sample.get("attitude_error_frame", "unknown"),
                "force_alignment_angle": assembly_sample.get("force_alignment_angle", 0.0),
                "force_alignment_limited": assembly_sample.get("force_alignment_limited", False),
                "att_error_world": assembly_sample.get("att_error_world", [0.0, 0.0, 0.0]),
                "att_error_yaw_world": assembly_sample.get("att_error_yaw_world", [0.0, 0.0, 0.0]),
                "att_error_tilt_world": assembly_sample.get("att_error_tilt_world", [0.0, 0.0, 0.0]),
                "pos_error": assembly_pos_error,
                "lin_vel": lin_vel,
                "thrust_cmd": drone_thrust_cmd,
                "rate_cmd": [0.0, 0.0, 0.0],
                "assembly_pos": assembly_pos,
                "assembly_target": assembly_target,
                "assembly_pos_error": assembly_pos_error,
                "allocation_residual_norm": allocation.residual_norm,
                "allocation_weighted_residual_norm": allocation.weighted_residual_norm,
                "allocation_rank": allocation.rank,
                "allocation_saturated": allocation.saturated_count,
                "wrench_weights": allocation.wrench_weights,
                "wrench_residual": allocation.residual,
                "wrench_weighted_residual": allocation.weighted_residual,
                "control_mode": "docked_allocation",
            }
        )
        samples.append(sample)
    return samples


def docked_allocation_step(
    sim,
    controlled: list[ControlledDrone],
    docked_state: assembly_geometric.AssemblyControllerState,
    args: argparse.Namespace,
    target_position: list[float],
    target_roll: float,
    target_pitch: float,
    target_yaw: float,
) -> tuple[list[dict[str, object]], list[dict[str, object]], dict[str, object]]:
    assembly_geom = allocation_geometry.build_assembly_geometry(
        sim,
        [item.drone for item in controlled],
        args.mass,
        module_inertia_box(args),
    )
    assembly_sample = assembly_geometric.controller_step(
        sim,
        assembly_geom,
        docked_state,
        args,
        target_position,
        target_yaw,
        target_roll=target_roll,
        target_pitch=target_pitch,
    )
    allocation = wrench_allocator.allocate_wrench(
        assembly_geom,
        assembly_sample["wrench_cmd"],
        args.max_motor_speed,
        args.max_thrust,
        args.yaw_drag_arm,
        args.allocation_regularization,
        allocation_weights(args),
    )
    low_samples: list[dict[str, object]] = []
    for index, item in enumerate(controlled):
        omega_cmd = allocation.motor_omega_cmd[4 * index : 4 * index + 4]
        low_samples.append(
            flight.apply_motor_speed_commands(sim, item.drone.body, item.drone.joints, item.low_state, omega_cmd, item.args)
        )
    high_samples = docked_high_samples_for_ui(sim, controlled, assembly_sample, allocation)
    allocation_sample = {
        "members": [item.drone.index for item in controlled],
        "assembly_pos": assembly_sample["pos"],
        "assembly_mass": assembly_geom.mass,
        "assembly_target": assembly_sample["target"],
        "assembly_pos_error": assembly_sample["pos_error"],
        "target_roll": assembly_sample["target_roll"],
        "target_pitch": assembly_sample["target_pitch"],
        "target_yaw": assembly_sample["target_yaw"],
        "auto_roll": assembly_sample["auto_roll"],
        "auto_pitch": assembly_sample["auto_pitch"],
        "desired_roll": assembly_sample["desired_roll"],
        "desired_pitch": assembly_sample["desired_pitch"],
        "attitude_coupling_mode": assembly_sample["attitude_coupling_mode"],
        "attitude_error_frame": assembly_sample["attitude_error_frame"],
        "force_alignment_angle": assembly_sample["force_alignment_angle"],
        "force_alignment_axis_world": assembly_sample["force_alignment_axis_world"],
        "force_alignment_limited": assembly_sample["force_alignment_limited"],
        "alignment_collective_axis_world": assembly_sample["alignment_collective_axis_world"],
        "alignment_force_axis_world": assembly_sample["alignment_force_axis_world"],
        "att_error_world": assembly_sample["att_error_world"],
        "att_error_yaw_world": assembly_sample["att_error_yaw_world"],
        "att_error_tilt_world": assembly_sample["att_error_tilt_world"],
        "allocation_rank": allocation.rank,
        "allocation_residual_norm": allocation.residual_norm,
        "allocation_weighted_residual_norm": allocation.weighted_residual_norm,
        "allocation_saturated": allocation.saturated_count,
        "wrench_weights": allocation.wrench_weights,
        "wrench_cmd": allocation.wrench_cmd,
        "wrench_achieved": allocation.wrench_achieved,
        "wrench_residual": allocation.residual,
        "wrench_weighted_residual": allocation.weighted_residual,
    }
    return high_samples, low_samples, allocation_sample


def simulation_stopped_by_gui(sim) -> bool:
    return sim.getSimulationState() == sim.simulation_stopped


def release_stepping(client) -> None:
    client.setStepping(False)


def main() -> int:
    args = parse_args()
    client, sim = flight.connect(args)
    flight.stop_if_running(sim)
    if args.load_scene:
        scene_path = Path(args.scene)
        if not scene_path.exists():
            raise FileNotFoundError(scene_path)
        sim.loadScene(str(scene_path))

    flight.set_time_step(sim, args.time_step)
    omega = hover_omega(args)
    drones = multi.discover_drones(sim, omega)
    if not drones:
        raise RuntimeError(
            f"Expected at least one /{multi.MODEL_ALIAS} model. "
            "Run scripts\\launchers\\run_two_drone_position_ui.py from PyCharm."
        )
    if args.reset_state:
        for drone in drones:
            multi.reset_drone(sim, drone, omega)

    controlled = build_controlled_drones(sim, drones, args)
    ui = MultiDronePositionUI(args, len(controlled))
    ui.load_selection_target(controlled, {})

    geometry = multi.docking_geometry(Path(args.body_stl))
    connectors = geometry.connectors
    docking_memory = multi.DockingMemory()
    drone_pair_count = max(1, len(controlled) * (len(controlled) - 1) // 2)
    max_marker_pairs = args.max_magnet_pairs_per_drone_pair * drone_pair_count
    visualizer = (
        LatchedCornerColorizer(sim, max_marker_pairs, args.magnet_marker_radius)
        if args.show_magnet_markers
        else None
    )
    assembly_force_visualizer = AssemblyForceArrowVisualizer(sim, args) if args.assembly_force_arrow else None
    mixer = flight.motor_mixer(args.yaw_drag_arm)
    logger = telemetry.CsvTelemetryLogger(args.log_csv, "multi_drone_position_ui", args.log_sample_period)
    group_commands: dict[tuple[int, ...], GroupCommand] = {}

    print("Multi-drone docking UI controller:")
    print(f"  discovered {len(controlled)} drone model(s)")
    print("  free drones use independent cascaded position/body-rate control")
    print("  latched face-connected components use assembly wrench allocation when enabled")
    print("  select all, one drone, or a docked assembly in the UI to command targets")

    client.setStepping(True)
    sim.startSimulation()
    sim_start = sim.getSimulationTime()
    wall_start = time.perf_counter()
    next_log = 0.0
    ui_period = max(args.ui_update_period, args.time_step)
    ui_event_period = max(args.ui_event_period, args.time_step)
    marker_period = max(args.marker_update_period, args.time_step)
    force_arrow_period = max(args.assembly_force_arrow_update_period, args.time_step)
    target_visual_period = max(args.target_visual_update_period, args.time_step)
    last_ui_update = -1e9
    last_marker_update = -1e9
    last_force_arrow_update = -1e9
    last_target_visual_update = -1e9
    last_ui_event_wall = 0.0

    try:
        while ui.running:
            loop_wall = time.perf_counter()
            if loop_wall - last_ui_event_wall >= ui_event_period:
                ui.pump_events()
                last_ui_event_wall = loop_wall
                if not ui.running:
                    break
            ui.advance_continuous_motion(args.time_step)
            ui.sync_runtime_tuning(args)
            if simulation_stopped_by_gui(sim):
                print("CoppeliaSim simulation stopped; exiting Python UI controller.")
                break

            if ui.reset_requested:
                multi.clear_docking_memory(sim, docking_memory, clear_cooldowns=True, break_reason="manual reset")
                clear_group_commands(sim, group_commands)
                if assembly_force_visualizer is not None:
                    assembly_force_visualizer.clear()
                for item in controlled:
                    reset_controlled_drone(sim, item, omega)
                ui.magnets_enabled.set(args.magnets_start_enabled)
                ui.set_options(len(controlled), [], group_commands)
                ui.load_selection_target(controlled, group_commands)
                last_marker_update = -1e9
                last_target_visual_update = -1e9
                ui.reset_requested = False

            if ui.detach_and_hold_requested:
                multi.clear_docking_memory(sim, docking_memory, clear_cooldowns=True, break_reason="manual detach")
                clear_group_commands(sim, group_commands)
                if assembly_force_visualizer is not None:
                    assembly_force_visualizer.clear()
                ui.magnets_enabled.set(False)
                set_targets_to_current_all(sim, controlled, group_commands, args)
                reset_controller_integrators(controlled, omega)
                ui.set_options(len(controlled), [], group_commands)
                ui.load_selection_target(controlled, group_commands)
                last_marker_update = -1e9
                last_target_visual_update = -1e9
                ui.detach_and_hold_requested = False

            if ui.release_latches_requested:
                multi.clear_docking_memory(sim, docking_memory, clear_cooldowns=True, break_reason="manual release")
                clear_group_commands(sim, group_commands)
                if assembly_force_visualizer is not None:
                    assembly_force_visualizer.clear()
                set_targets_to_current_all(sim, controlled, group_commands, args)
                reset_controller_integrators(controlled, omega)
                ui.set_options(len(controlled), [], group_commands)
                ui.load_selection_target(controlled, group_commands)
                last_marker_update = -1e9
                ui.release_latches_requested = False

            if ui.quick_square_dock_requested:
                yaw_mode = ui.quick_dock_yaw_mode_var.get()
                quick_dock = quick_square_dock(sim, ui.selection(), controlled, group_commands, geometry, args, yaw_mode)
                if quick_dock is not None:
                    pair = quick_dock.pair
                    ui.magnets_enabled.set(True)
                    reset_controller_integrators([controlled[index] for index in pair], omega)
                    last_target_visual_update = -1e9
                    ui.quick_dock_status = (
                        f"Quick dock: d{pair[0]} square face {quick_dock.face_a.index} "
                        f"to d{pair[1]} square face {quick_dock.face_b.index}, "
                        f"{yaw_mode_label(yaw_mode)}"
                    )
                    print(
                        f"Quick square dock command: d{pair[0]}:face{quick_dock.face_a.index} "
                        f"to d{pair[1]}:face{quick_dock.face_b.index}, "
                        f"heading={yaw_mode_label(yaw_mode)}"
                    )
                ui.quick_square_dock_requested = False

            if ui.quick_open_pair_requested:
                pair = quick_open_pair(sim, ui.selection(), controlled, group_commands, args)
                multi.clear_docking_memory(sim, docking_memory, clear_cooldowns=True, break_reason="quick open")
                clear_group_commands(sim, group_commands)
                ui.magnets_enabled.set(False)
                if pair is not None:
                    reset_controller_integrators([controlled[index] for index in pair], omega)
                    ui.quick_dock_status = f"Quick open: d{pair[0]} and d{pair[1]}"
                    print(f"Quick open command: d{pair[0]} and d{pair[1]}")
                ui.set_options(len(controlled), [], group_commands)
                ui.load_selection_target(controlled, group_commands)
                last_marker_update = -1e9
                last_target_visual_update = -1e9
                ui.quick_open_pair_requested = False

            groups = docking_groups_from_memory(len(controlled), docking_memory, ui.magnets_enabled.get())
            configurations = configurations_from_memory(groups, docking_memory, geometry)
            added_groups, removed_groups = refresh_group_commands(
                sim,
                controlled,
                groups,
                group_commands,
                configurations,
                args,
                omega,
            )
            if added_groups or removed_groups:
                ui.set_options(len(controlled), groups, group_commands)
                if added_groups and ui.auto_select_assembly_enabled.get():
                    ui.select_group(added_groups[0], len(controlled), groups, group_commands)
                ui.load_selection_target(controlled, group_commands)

            if ui.targets_to_current_requested:
                set_selection_to_current(sim, ui.selection(), controlled, group_commands, args)
                reset_controller_integrators(controlled, omega)
                ui.load_selection_target(controlled, group_commands)
                last_target_visual_update = -1e9
                ui.targets_to_current_requested = False

            if ui.all_targets_to_current_requested:
                set_targets_to_current_all(sim, controlled, group_commands, args)
                reset_controller_integrators(controlled, omega)
                ui.load_selection_target(controlled, group_commands)
                last_target_visual_update = -1e9
                ui.all_targets_to_current_requested = False

            if ui.selection_changed_requested:
                ui.load_selection_target(controlled, group_commands)
                ui.selection_changed_requested = False

            if ui.command_dirty:
                ui.apply_current_command(controlled, group_commands)
                last_target_visual_update = -1e9

            for command in group_commands.values():
                advance_group_target(command, args, args.time_step)
                apply_group_command(controlled, command)
            for item in controlled:
                advance_controlled_target(item, args, args.time_step)

            high_samples: list[dict[str, object] | None] = [None for _ in controlled]
            low_samples: list[dict[str, object] | None] = [None for _ in controlled]
            allocation_by_group: dict[str, object] = {}
            controlled_by_group: set[int] = set()

            if ui.docked_controller_enabled.get():
                for group_index, key in enumerate(groups):
                    command = group_commands[key]
                    apply_group_command(controlled, command)
                    group_items = [controlled[index] for index in key]
                    group_high, group_low, allocation_sample = docked_allocation_step(
                        sim,
                        group_items,
                        command.state,
                        args,
                        command.target,
                        command.roll,
                        command.pitch,
                        command.yaw,
                    )
                    allocation_by_group[f"group_{group_index}"] = allocation_sample
                    for slot, index in enumerate(key):
                        high_samples[index] = group_high[slot]
                        low_samples[index] = group_low[slot]
                        controlled_by_group.add(index)

            for index, item in enumerate(controlled):
                if index in controlled_by_group:
                    continue
                high = position.high_level_step(sim, item.drone.body, None, item.high_state, item.args)
                low = flight.controller_step(sim, item.drone.body, item.drone.joints, item.low_state, mixer, item.args)
                high["control_mode"] = "independent"
                high_samples[index] = high
                low_samples[index] = low

            docking_sample = multi.apply_magnetic_docking(
                sim,
                [item.drone for item in controlled],
                geometry,
                docking_memory,
                args,
                ui.magnets_enabled.get(),
            )
            docking_sample.update(allocation_by_group)
            groups_after = docking_groups_from_memory(len(controlled), docking_memory, ui.magnets_enabled.get())
            configurations_after = configurations_from_memory(groups_after, docking_memory, geometry)
            added_groups, removed_groups = refresh_group_commands(
                sim,
                controlled,
                groups_after,
                group_commands,
                configurations_after,
                args,
                omega,
            )
            if added_groups or removed_groups:
                ui.set_options(len(controlled), groups_after, group_commands)
                if added_groups and ui.auto_select_assembly_enabled.get():
                    ui.select_group(added_groups[0], len(controlled), groups_after, group_commands)
                ui.load_selection_target(controlled, group_commands)
                last_target_visual_update = -1e9

            sim_time = float(sim.getSimulationTime()) - sim_start
            real_time_factor = sim_time / max(time.perf_counter() - wall_start, 1e-6)
            docking_sample["real_time_factor"] = real_time_factor
            if assembly_force_visualizer is not None and sim_time - last_force_arrow_update >= force_arrow_period:
                assembly_force_visualizer.update(allocation_by_group if ui.docked_controller_enabled.get() else {})
                last_force_arrow_update = sim_time
            if visualizer is not None and sim_time - last_marker_update >= marker_period:
                world, latched_keys = latched_corner_visual_keys(
                    sim,
                    [item.drone for item in controlled],
                    connectors,
                    docking_memory,
                    ui.magnets_enabled.get(),
                )
                visualizer.update(world, latched_keys)
                last_marker_update = sim_time

            telemetry_pairs: list[tuple[str, dict[str, object] | None]] = []
            for index in range(len(controlled)):
                telemetry_pairs.append((f"d{index}_high", high_samples[index]))
                telemetry_pairs.append((f"d{index}_low", low_samples[index]))
            telemetry_pairs.append(("dock", docking_sample))
            logger.write(
                float(sim.getSimulationTime()),
                telemetry.merge_samples(
                    *telemetry_pairs,
                    extra={
                        "magnets_enabled": ui.magnets_enabled.get(),
                        "docked_controller_enabled": ui.docked_controller_enabled.get(),
                        "docked_groups": len(groups_after),
                    },
                ),
            )

            client.step()
            if simulation_stopped_by_gui(sim):
                print("CoppeliaSim simulation stopped; exiting Python UI controller.")
                break

            if sim_time - last_target_visual_update >= target_visual_period:
                sync_target_visuals(sim, controlled, group_commands)
                last_target_visual_update = sim_time
            if sim_time - last_ui_update >= ui_period:
                ui.update(sim_time, controlled, groups_after, group_commands, high_samples, low_samples, docking_sample)
                last_ui_update = sim_time
            if sim_time >= next_log:
                error_parts = []
                for index, high in enumerate(high_samples):
                    if high is not None:
                        error_parts.append(f"d{index}err={format_vector(high['pos_error'])}")
                print(
                    f"t={sim_time:5.2f}s "
                    f"rtf={real_time_factor:4.2f} "
                    f"dock={docking_sample['mode']} latched={docking_sample['latched_pairs']} "
                    f"groups={len(groups_after)} "
                    + " ".join(error_parts)
                )
                next_log += args.log_period

            if args.duration > 0.0 and sim_time >= args.duration:
                break
            elapsed = time.perf_counter() - loop_wall
            if elapsed < args.time_step:
                time.sleep(args.time_step - elapsed)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        logger.close()
        if visualizer is not None:
            visualizer.clear()
        if assembly_force_visualizer is not None:
            assembly_force_visualizer.clear()
        if ui.running:
            ui.close()
        if args.stop_on_exit:
            sim.stopSimulation(True)
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.05)
        release_stepping(client)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
