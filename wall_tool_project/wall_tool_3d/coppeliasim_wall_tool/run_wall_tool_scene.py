#!/usr/bin/env python3
"""Run the wall-tool controller as a 3D CoppeliaSim pen-on-wall demo.

The default mode is a force-driven CoppeliaSim plant: the payload is a dynamic
body, the two side motors apply forces and torques, the reel enforces a taut
unilateral cable, and propeller joints spin from motor angular speed.
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WALL_TOOL_PROJECT_ROOT = PROJECT_ROOT.parent
WALL_TOOL_2D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_2d"
for path in (PROJECT_ROOT, WALL_TOOL_2D_ROOT, WALL_TOOL_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def configure_matplotlib_backend() -> None:
    """Prefer a real desktop window for the reused 2D controller UI."""
    if os.environ.get("PRISMS_KEEP_MPLBACKEND"):
        return
    import matplotlib

    os.environ["MPLBACKEND"] = "TkAgg"
    matplotlib.use("TkAgg", force=True)


configure_matplotlib_backend()


def is_interactive_matplotlib_backend(backend_name: str) -> bool:
    """Return True for GUI backends such as TkAgg/QtAgg, not plain file renderers."""
    normalized = (backend_name or "").lower()
    non_interactive = {"agg", "cairo", "pdf", "pgf", "ps", "svg", "template"}
    return normalized not in non_interactive and "backend_inline" not in normalized


def focus_matplotlib_window(fig, title: str) -> None:
    """Best-effort raise for desktop Matplotlib windows."""
    manager = getattr(fig.canvas, "manager", None)
    if manager is None:
        return
    try:
        manager.set_window_title(title)
    except Exception:
        pass
    window = getattr(manager, "window", None)
    if window is None:
        return
    # TkAgg exposes tkinter methods, while Qt exposes QWidget-style methods.
    for method_name in ("deiconify", "show", "lift", "raise_", "activateWindow", "focus_force"):
        method = getattr(window, method_name, None)
        if method is None:
            continue
        try:
            method()
        except Exception:
            pass
    try:
        window.attributes("-topmost", True)
        window.after(250, lambda: window.attributes("-topmost", False))
    except Exception:
        pass

from cable_hybrid_controller.controller import (  # noqa: E402
    BEST_PLANNER,
    COVERAGE_CORNER_SPEED,
    make_simulator,
)
from coppeliasim_wall_tool import generate_wall_tool_scene as scene_gen  # noqa: E402
from coppeliasim_wall_tool import sim_utils  # noqa: E402
from coppeliasim_wall_tool.sensor_estimator import (  # noqa: E402
    EstimatedWallToolState,
    SensorConfig,
    SensorTruth,
    WallToolSensorPipeline,
)
from wall_tool_sim.reel_motor import ReelMotorSpec  # noqa: E402
from wall_tool_sim.steel_cable import SteelCableSpec  # noqa: E402
from wall_tool_sim.wall_tool_ui import integrated_motor_center_offsets  # noqa: E402


PLANT_MODES = ("dynamic",)
FEEDBACK_MODES = ("sensor", "ground-truth")
COPPELIASIM_DEFAULT_TIME_STEP_S = 0.010
DEFAULT_VISUAL_UPDATE_PERIOD_S = 0.100
DEFAULT_REALTIME_FACTOR_FLOOR = 0.45
DEFAULT_DESIRED_PATH_UPDATE_PERIOD_S = 0.250
DEFAULT_DESIRED_PATH_MAX_SEGMENTS = 24
DEFAULT_DESIRED_PATH_RADIUS_M = 0.006
DESIRED_PATH_WALL_Y = -0.012
DESIRED_PATH_COLOR = (0.05, 0.38, 0.95)
DESIRED_PATH_START_COLOR = (0.05, 0.65, 0.32)
DESIRED_PATH_END_COLOR = (1.0, 0.64, 0.12)


def parse_args() -> argparse.Namespace:
    params = make_simulator().params
    steel_cable = SteelCableSpec(
        diameter_m=params.steel_cable_diameter_m,
        youngs_modulus_pa=params.steel_cable_youngs_modulus_pa,
        density_kg_m3=params.steel_cable_density_kg_m3,
        structural_compliance_m_N=params.steel_cable_structural_compliance_m_N,
        damping_ratio=params.steel_cable_damping_ratio,
        payload_weight_fraction=params.steel_cable_payload_weight_fraction,
    )
    reel_motor = ReelMotorSpec(
        voltage_v=params.reel_motor_voltage_v,
        gear_ratio=params.reel_motor_gear_ratio,
        no_load_output_rpm=params.reel_motor_no_load_rpm,
        stall_torque_kg_cm=params.reel_motor_stall_torque_kg_cm,
        spool_radius_m=params.reel_spool_radius_m,
        velocity_time_constant_s=params.reel_velocity_time_constant_s,
        continuous_torque_fraction=params.reel_continuous_torque_fraction,
    )
    parser = argparse.ArgumentParser(description="Run the PRISMS wall-tool CoppeliaSim draw-on-wall demo.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--launch-coppeliasim", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--coppeliasim-exe", type=Path, default=sim_utils.DEFAULT_COPPELIASIM_EXE)
    parser.add_argument("--duration", type=float, default=0.0, help="Simulation duration [s]. Default 0 keeps the 2D UI running.")
    parser.add_argument("--plant-mode", choices=PLANT_MODES, default="dynamic")
    parser.add_argument(
        "--feedback-mode",
        choices=FEEDBACK_MODES,
        default="sensor",
        help="Use the proposed encoders/load-cell/IMU estimator, or exact CoppeliaSim state for comparison.",
    )
    parser.add_argument("--reel-encoder-counts-per-output-rev", type=int, default=2048)
    parser.add_argument("--cable-angle-encoder-counts-per-rev", type=int, default=4096)
    parser.add_argument("--sensor-noise", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--sensor-random-seed", type=int, default=7)
    parser.add_argument("--estimator-velocity-fusion-tau", type=float, default=0.080)
    parser.add_argument("--target-x", type=float, default=0.90)
    parser.add_argument("--target-z", type=float, default=1.50)
    parser.add_argument(
        "--path-points",
        default="",
        help="Optional semicolon-separated x,z path, for example '0.8,2.0;0.8,1.3;-0.8,1.3'.",
    )
    parser.add_argument("--standoff", type=float, default=params.normal_standoff_m)
    parser.add_argument("--wall-width", type=float, default=params.wall_width)
    parser.add_argument("--wall-height", type=float, default=params.wall_height)
    parser.add_argument("--wall-thickness", type=float, default=0.050)
    parser.add_argument("--update-period", type=float, default=0.020)
    parser.add_argument(
        "--time-step",
        type=float,
        default=COPPELIASIM_DEFAULT_TIME_STEP_S,
        help="CoppeliaSim/controller step [s]. Default 0.01 targets responsive 100 Hz external-force stepping.",
    )
    parser.add_argument("--max-motor-speed", type=float, default=900.0)
    parser.add_argument("--motor-tau-up", type=float, default=0.050)
    parser.add_argument("--motor-tau-down", type=float, default=0.080)
    parser.add_argument("--steel-cable-diameter", type=float, default=steel_cable.diameter_m)
    parser.add_argument("--steel-youngs-modulus", type=float, default=steel_cable.youngs_modulus_pa)
    parser.add_argument("--steel-density", type=float, default=steel_cable.density_kg_m3)
    parser.add_argument("--steel-cable-structural-compliance", type=float, default=steel_cable.structural_compliance_m_N)
    parser.add_argument("--steel-cable-damping-ratio", type=float, default=steel_cable.damping_ratio)
    parser.add_argument("--steel-cable-payload-weight-fraction", type=float, default=steel_cable.payload_weight_fraction)
    parser.add_argument("--steel-cable-min-visual-tension", type=float, default=steel_cable.min_visual_tension_N)
    parser.add_argument("--steel-cable-max-visual-sag", type=float, default=steel_cable.max_visual_sag_m)
    parser.add_argument("--cable-taut-band", type=float, default=params.cable_taut_band)
    parser.add_argument("--max-cable-tension", type=float, default=params.max_spool_tension)
    parser.add_argument("--cable-segments", type=int, default=scene_gen.CABLE_SEGMENT_COUNT)
    parser.add_argument("--reel-motor-voltage", type=float, default=reel_motor.voltage_v)
    parser.add_argument("--reel-gear-ratio", type=float, default=reel_motor.gear_ratio)
    parser.add_argument("--reel-no-load-rpm", type=float, default=reel_motor.no_load_output_rpm)
    parser.add_argument("--reel-stall-torque-kg-cm", type=float, default=reel_motor.stall_torque_kg_cm)
    parser.add_argument("--reel-spool-radius", type=float, default=reel_motor.spool_radius_m)
    parser.add_argument("--reel-velocity-tau", type=float, default=reel_motor.velocity_time_constant_s)
    parser.add_argument("--reel-continuous-torque-fraction", type=float, default=reel_motor.continuous_torque_fraction)
    parser.add_argument("--linear-drag-xz", type=float, default=0.020)
    parser.add_argument("--normal-standoff-kp", type=float, default=42.0)
    parser.add_argument("--normal-standoff-kd", type=float, default=2.2)
    parser.add_argument("--wall-contact-stiffness", type=float, default=params.normal_contact_stiffness_N_m)
    parser.add_argument("--wall-contact-damping", type=float, default=params.normal_contact_damping_N_s_m)
    parser.add_argument("--wall-contact-force-limit", type=float, default=params.normal_contact_force_limit_N)
    parser.add_argument("--wall-contact-friction", type=float, default=0.020)
    parser.add_argument("--wall-friction-transition-speed", type=float, default=0.040)
    parser.add_argument("--angular-drag-y", type=float, default=params.rotational_damping)
    parser.add_argument("--angular-drag-roll-yaw", type=float, default=0.004)
    parser.add_argument("--guide-roll-yaw-stiffness", type=float, default=0.060)
    parser.add_argument("--guide-pitch-stiffness", type=float, default=5.0)
    parser.add_argument("--guide-pitch-damping", type=float, default=0.50)
    parser.add_argument("--prop-visual-update-period", type=float, default=DEFAULT_VISUAL_UPDATE_PERIOD_S)
    parser.add_argument("--cable-visual-update-period", type=float, default=DEFAULT_VISUAL_UPDATE_PERIOD_S)
    parser.add_argument("--show-desired-path", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--desired-path-update-period", type=float, default=DEFAULT_DESIRED_PATH_UPDATE_PERIOD_S)
    parser.add_argument("--desired-path-max-segments", type=int, default=DEFAULT_DESIRED_PATH_MAX_SEGMENTS)
    parser.add_argument("--desired-path-radius", type=float, default=DEFAULT_DESIRED_PATH_RADIUS_M)
    parser.add_argument("--min-realtime-factor", type=float, default=DEFAULT_REALTIME_FACTOR_FLOOR)
    parser.add_argument("--log-period", type=float, default=0.50)
    parser.add_argument("--ink-spacing", type=float, default=0.018)
    parser.add_argument("--ink-radius", type=float, default=0.010)
    parser.add_argument("--max-ink-dots", type=int, default=1800)
    parser.add_argument("--control-ui", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--async-ui", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--ui-update-period", type=float, default=0.050)
    parser.add_argument("--ui-history-window", type=float, default=12.0)
    parser.add_argument("--regenerate-scene", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--save-generated-scene", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stop-simulation-on-exit", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def generator_args(args: argparse.Namespace, simulator) -> argparse.Namespace:
    params = simulator.params
    return argparse.Namespace(
        host=args.host,
        port=args.port,
        connect_timeout=args.connect_timeout,
        scene_output=scene_gen.SCENE_OUTPUT,
        model_output=scene_gen.MODEL_OUTPUT,
        wall_width=args.wall_width,
        wall_height=args.wall_height,
        wall_thickness=args.wall_thickness,
        standoff=args.standoff,
        reel_spool_radius=float(args.reel_spool_radius),
        payload_x=params.initial_payload[0],
        payload_z=params.initial_payload[1],
        body_depth=0.140,
        motor_depth=0.060,
        cable_radius=0.5 * float(args.steel_cable_diameter),
        cable_segments=int(args.cable_segments),
        pen_radius=0.009,
        save_model=bool(args.save_generated_scene),
        clear_existing=True,
    )


@dataclass
class SceneHandles:
    payload: int
    cable: int
    cable_segments: tuple[int, ...]
    anchor: int
    target: int
    pen_tip: int
    left_motor_frame: int
    right_motor_frame: int
    left_prop_joint: int
    right_prop_joint: int
    left_force_arrow_stem: int
    left_force_arrow_head: int
    right_force_arrow_stem: int
    right_force_arrow_head: int


@dataclass
class DynamicPlantState:
    reel_length: float | None = None
    reel_velocity: float = 0.0
    anchor_position: list[float] | None = None
    cable_visual_lengths: list[float] | None = None
    left_motor_speed: float = 0.0
    right_motor_speed: float = 0.0
    last_left_thrust: float = 0.0
    last_right_thrust: float = 0.0
    left_prop_phase: float = 0.0
    right_prop_phase: float = 0.0
    left_force_arrow_stem_length: float = scene_gen.FORCE_ARROW_INITIAL_LENGTH - scene_gen.FORCE_ARROW_HEAD_LENGTH
    right_force_arrow_stem_length: float = scene_gen.FORCE_ARROW_INITIAL_LENGTH - scene_gen.FORCE_ARROW_HEAD_LENGTH
    last_time: float | None = None
    last_prop_visual_time: float = -1.0
    last_cable_visual_time: float = -1.0
    last_log_time: float = -1.0
    last_tension: float = 0.0
    sensor_pipeline: WallToolSensorPipeline | None = None
    last_estimate: EstimatedWallToolState | None = None


@dataclass(frozen=True)
class DynamicBodySample:
    sim_time: float
    matrix: list[float]
    position: list[float]
    orientation: list[float]
    linear_velocity: list[float]
    angular_velocity: list[float]
    anchor: list[float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def coppelia_pitch_to_planar_attitude(pitch_rad: float) -> float:
    """Convert CoppeliaSim Y pitch to the 2D wall-tool attitude convention."""
    return wrap_angle(-float(pitch_rad))


def planar_attitude_to_coppelia_pitch(attitude_rad: float) -> float:
    """Convert the 2D wall-tool attitude convention to CoppeliaSim Y pitch."""
    return -float(attitude_rad)


def add3(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [float(a[index]) + float(b[index]) for index in range(3)]


def sub3(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [float(a[index]) - float(b[index]) for index in range(3)]


def scale3(vector: Sequence[float], gain: float) -> list[float]:
    return [float(vector[index]) * float(gain) for index in range(3)]


def dot3(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(a[index]) * float(b[index]) for index in range(3))


def norm3(vector: Sequence[float]) -> float:
    return math.sqrt(max(0.0, dot3(vector, vector)))


def normalize3(vector: Sequence[float]) -> list[float]:
    length = norm3(vector)
    if length < 1e-12:
        return [0.0, 0.0, 1.0]
    return [float(vector[index]) / length for index in range(3)]


def cross3(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    ]


def body_axes_from_matrix(matrix: Sequence[float]) -> tuple[list[float], list[float], list[float]]:
    body_x = [float(matrix[0]), float(matrix[4]), float(matrix[8])]
    body_y = [float(matrix[1]), float(matrix[5]), float(matrix[9])]
    body_z = [float(matrix[2]), float(matrix[6]), float(matrix[10])]
    return body_x, body_y, body_z


def local_vector_to_world(matrix: Sequence[float], local: Sequence[float]) -> list[float]:
    body_x, body_y, body_z = body_axes_from_matrix(matrix)
    return [
        body_x[index] * float(local[0])
        + body_y[index] * float(local[1])
        + body_z[index] * float(local[2])
        for index in range(3)
    ]


def local_point_to_world(matrix: Sequence[float], local: Sequence[float]) -> list[float]:
    origin = [float(matrix[3]), float(matrix[7]), float(matrix[11])]
    return add3(origin, local_vector_to_world(matrix, local))


def add_wrench_at_point(
    net_force: list[float],
    net_torque: list[float],
    force: Sequence[float],
    point: Sequence[float],
    center_of_mass: Sequence[float],
) -> None:
    arm = sub3(point, center_of_mass)
    torque = cross3(arm, force)
    for index in range(3):
        net_force[index] += float(force[index])
        net_torque[index] += float(torque[index])


def set_time_step(sim, time_step: float) -> None:
    sim.setFloatParam(sim.floatparam_simulation_time_step, float(time_step))


def get_handle(sim, alias: str) -> int:
    candidates = (f"/{alias}", f"/{scene_gen.PAYLOAD_ALIAS}/{alias}")
    last_error: Exception | None = None
    for path in candidates:
        try:
            return int(sim.getObject(path))
        except Exception as exc:  # pragma: no cover - remote API exception type is dynamic.
            last_error = exc
    raise RuntimeError(f"Could not find CoppeliaSim object alias '{alias}'.") from last_error


def read_dynamic_body_sample(sim, handles: SceneHandles, plant_state: DynamicPlantState) -> DynamicBodySample:
    matrix = [float(value) for value in sim.getObjectMatrix(handles.payload, -1)]
    position = [matrix[3], matrix[7], matrix[11]]
    orientation = [float(value) for value in sim.getObjectOrientation(handles.payload, -1)]
    linear_velocity, angular_velocity = sim.getVelocity(handles.payload)
    if plant_state.anchor_position is None:
        plant_state.anchor_position = [float(value) for value in sim.getObjectPosition(handles.anchor, -1)]
    return DynamicBodySample(
        sim_time=float(sim.getSimulationTime()),
        matrix=matrix,
        position=position,
        orientation=orientation,
        linear_velocity=[float(value) for value in linear_velocity],
        angular_velocity=[float(value) for value in angular_velocity],
        anchor=plant_state.anchor_position,
    )


def resolve_handles(sim, cable_segments: int) -> SceneHandles:
    if int(cable_segments) < 2:
        raise ValueError("--cable-segments must be at least 2")
    return SceneHandles(
        payload=get_handle(sim, scene_gen.PAYLOAD_ALIAS),
        cable=get_handle(sim, scene_gen.CABLE_ALIAS),
        cable_segments=tuple(
            get_handle(sim, f"{scene_gen.CABLE_SEGMENT_ALIAS_PREFIX}_{index:02d}")
            for index in range(int(cable_segments))
        ),
        anchor=get_handle(sim, scene_gen.ANCHOR_ALIAS),
        target=get_handle(sim, scene_gen.TARGET_ALIAS),
        pen_tip=get_handle(sim, scene_gen.PEN_TIP_ALIAS),
        left_motor_frame=get_handle(sim, scene_gen.LEFT_MOTOR_FRAME_ALIAS),
        right_motor_frame=get_handle(sim, scene_gen.RIGHT_MOTOR_FRAME_ALIAS),
        left_prop_joint=get_handle(sim, scene_gen.LEFT_PROP_JOINT_ALIAS),
        right_prop_joint=get_handle(sim, scene_gen.RIGHT_PROP_JOINT_ALIAS),
        left_force_arrow_stem=get_handle(sim, scene_gen.LEFT_FORCE_ARROW_STEM_ALIAS),
        left_force_arrow_head=get_handle(sim, scene_gen.LEFT_FORCE_ARROW_HEAD_ALIAS),
        right_force_arrow_stem=get_handle(sim, scene_gen.RIGHT_FORCE_ARROW_STEM_ALIAS),
        right_force_arrow_head=get_handle(sim, scene_gen.RIGHT_FORCE_ARROW_HEAD_ALIAS),
    )


def update_payload_pose(sim, payload: int, x: float, z: float, standoff: float, attitude: float) -> None:
    position, orientation = sim_utils.payload_pose_to_world(
        x,
        z,
        standoff,
        planar_attitude_to_coppelia_pitch(attitude),
    )
    sim.setObjectPosition(payload, -1, position)
    sim.setObjectOrientation(payload, -1, orientation)


def create_ink_dot(sim, index: int, x: float, z: float, radius: float) -> int:
    dot = sim_utils.create_shape(
        sim,
        sim.primitiveshape_cylinder,
        [2.0 * radius, 2.0 * radius, 0.003],
        f"ink_dot_{index:04d}",
        [x, -0.006, z],
        [0.06, 0.06, 0.05],
        orientation=[math.pi / 2.0, 0.0, 0.0],
        static=True,
        respondable=False,
    )
    return dot


def maybe_add_ink(
    sim,
    ink_handles: list[int],
    last_ink_point: tuple[float, float] | None,
    x: float,
    z: float,
    args: argparse.Namespace,
) -> tuple[float, float] | None:
    if last_ink_point is not None and math.hypot(x - last_ink_point[0], z - last_ink_point[1]) < args.ink_spacing:
        return last_ink_point
    ink_handles.append(create_ink_dot(sim, len(ink_handles), x, z, args.ink_radius))
    if len(ink_handles) > args.max_ink_dots:
        sim.removeObjects([ink_handles.pop(0)], False)
    return (x, z)


def _dedupe_path_points(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    clean: list[tuple[float, float]] = []
    for point in points:
        candidate = (float(point[0]), float(point[1]))
        if not clean or math.hypot(candidate[0] - clean[-1][0], candidate[1] - clean[-1][1]) > 1e-6:
            clean.append(candidate)
    return clean


def _resample_path_points(
    points: Sequence[tuple[float, float]],
    max_points: int,
) -> list[tuple[float, float]]:
    clean = _dedupe_path_points(points)
    if len(clean) <= max_points:
        return clean
    segment_lengths = [
        math.hypot(clean[index + 1][0] - clean[index][0], clean[index + 1][1] - clean[index][1])
        for index in range(len(clean) - 1)
    ]
    total_length = sum(segment_lengths)
    if total_length <= 1e-9:
        return [clean[0], clean[-1]]
    resampled: list[tuple[float, float]] = []
    target_distances = [total_length * index / float(max_points - 1) for index in range(max_points)]
    segment_index = 0
    distance_before = 0.0
    for target_distance in target_distances:
        while (
            segment_index < len(segment_lengths) - 1
            and distance_before + segment_lengths[segment_index] < target_distance
        ):
            distance_before += segment_lengths[segment_index]
            segment_index += 1
        start = clean[segment_index]
        end = clean[segment_index + 1]
        length = max(segment_lengths[segment_index], 1e-9)
        u = clamp((target_distance - distance_before) / length, 0.0, 1.0)
        resampled.append((start[0] + u * (end[0] - start[0]), start[1] + u * (end[1] - start[1])))
    return _dedupe_path_points(resampled)


def _path_signature(points: Sequence[tuple[float, float]]) -> tuple[tuple[int, int], ...]:
    return tuple((round(1000.0 * float(point[0])), round(1000.0 * float(point[1]))) for point in points)


def _wall_path_point(point: tuple[float, float]) -> list[float]:
    return [float(point[0]), DESIRED_PATH_WALL_Y, float(point[1])]


class DesiredPathPreview:
    """Visible CoppeliaSim preview of the controller's commanded drawing path."""

    def __init__(self, sim, args: argparse.Namespace) -> None:
        self.sim = sim
        self.args = args
        self.segment_handles: list[int] = []
        self.segment_lengths: list[float] = []
        self.marker_handles: list[int] = []
        self.last_signature: tuple[tuple[int, int], ...] | None = None
        self.last_update_time = -1.0

    def maybe_update(self, sim_time: float, trajectory, *, force: bool = False) -> int:
        if not bool(self.args.show_desired_path):
            self.clear()
            return 0
        if (
            not force
            and self.last_update_time >= 0.0
            and sim_time - self.last_update_time < float(self.args.desired_path_update_period)
        ):
            return max(0, len(self.last_signature or ()) - 1)
        points = trajectory.pending_path()
        visible_segments = self.update(points)
        self.last_update_time = sim_time
        return visible_segments

    def update(self, points: Sequence[tuple[float, float]]) -> int:
        max_points = int(self.args.desired_path_max_segments) + 1
        path_points = _resample_path_points(points, max_points)
        signature = _path_signature(path_points)
        if signature == self.last_signature:
            return max(0, len(path_points) - 1)
        self.last_signature = signature
        if len(path_points) < 2:
            self._hide_all()
            self._update_markers(path_points)
            return 0

        segment_count = len(path_points) - 1
        self._ensure_segments(segment_count)
        for index in range(segment_count):
            sim_utils.set_visible(self.sim, self.segment_handles[index], True)
            self.segment_lengths[index] = sim_utils.update_cylinder_between(
                self.sim,
                self.segment_handles[index],
                _wall_path_point(path_points[index]),
                _wall_path_point(path_points[index + 1]),
                self.segment_lengths[index],
            )
        for handle in self.segment_handles[segment_count:]:
            sim_utils.set_visible(self.sim, handle, False)
        self._update_markers(path_points)
        return segment_count

    def clear(self) -> None:
        self._hide_all()
        self.last_signature = None

    def _ensure_segments(self, count: int) -> None:
        radius = float(self.args.desired_path_radius)
        while len(self.segment_handles) < count:
            index = len(self.segment_handles)
            handle = sim_utils.create_shape(
                self.sim,
                self.sim.primitiveshape_cylinder,
                [2.0 * radius, 2.0 * radius, 0.001],
                f"wall_tool_desired_path_segment_{index:02d}",
                _wall_path_point((0.0, 0.0)),
                DESIRED_PATH_COLOR,
                static=True,
                respondable=False,
            )
            self.segment_handles.append(handle)
            self.segment_lengths.append(0.001)

    def _ensure_markers(self) -> None:
        radius = max(1.8 * float(self.args.desired_path_radius), 0.010)
        while len(self.marker_handles) < 2:
            index = len(self.marker_handles)
            color = DESIRED_PATH_START_COLOR if index == 0 else DESIRED_PATH_END_COLOR
            handle = sim_utils.create_shape(
                self.sim,
                self.sim.primitiveshape_cylinder,
                [2.0 * radius, 2.0 * radius, 0.004],
                f"wall_tool_desired_path_marker_{index:02d}",
                _wall_path_point((0.0, 0.0)),
                color,
                orientation=[math.pi / 2.0, 0.0, 0.0],
                static=True,
                respondable=False,
            )
            self.marker_handles.append(handle)

    def _update_markers(self, path_points: Sequence[tuple[float, float]]) -> None:
        self._ensure_markers()
        if not path_points:
            for handle in self.marker_handles:
                sim_utils.set_visible(self.sim, handle, False)
            return
        endpoints = (path_points[0], path_points[-1])
        for index, point in enumerate(endpoints):
            sim_utils.set_visible(self.sim, self.marker_handles[index], True)
            self.sim.setObjectPosition(self.marker_handles[index], -1, _wall_path_point(point))

    def _hide_all(self) -> None:
        for handle in self.segment_handles:
            sim_utils.set_visible(self.sim, handle, False)
        for handle in self.marker_handles:
            sim_utils.set_visible(self.sim, handle, False)


@dataclass
class EfficiencyMonitor:
    """Batch/live metrics for tracking quality and actuator efficiency."""

    elapsed_s: float = 0.0
    steps: int = 0
    tracking_error_integral_m_s: float = 0.0
    tracking_error_sq_integral_m2_s: float = 0.0
    max_tracking_error_m: float = 0.0
    thrust_util_integral_s: float = 0.0
    max_thrust_util: float = 0.0
    prop_power_index_integral_s: float = 0.0
    reel_abs_work_J: float = 0.0
    reel_in_work_J: float = 0.0
    reel_out_work_J: float = 0.0
    thrust_saturation_s: float = 0.0
    tension_saturation_s: float = 0.0
    contact_valid_s: float = 0.0
    work_command_s: float = 0.0
    solver_time_integral_s2: float = 0.0
    solver_time_max_s: float = 0.0
    solver_failure_count: int = 0
    max_motor_rpm: float = 0.0
    max_reel_rpm: float = 0.0
    max_abs_pitch_rad: float = 0.0

    def update(
        self,
        *,
        dt: float,
        tool_error_m: float,
        left_thrust_N: float,
        right_thrust_N: float,
        max_thrust_N: float,
        tension_N: float,
        max_tension_N: float,
        reel_velocity_m_s: float,
        motor_speed_rad_s: Sequence[float],
        reel_rpm: float,
        pitch_rad: float,
        mpc_solve_time_s: float,
        mpc_status: str,
        contact_valid: bool,
        work_mode: bool,
    ) -> None:
        dt = max(0.0, float(dt))
        if dt <= 0.0:
            return
        self.elapsed_s += dt
        self.steps += 1
        error = max(0.0, float(tool_error_m))
        self.tracking_error_integral_m_s += error * dt
        self.tracking_error_sq_integral_m2_s += error * error * dt
        self.max_tracking_error_m = max(self.max_tracking_error_m, error)
        max_thrust = max(float(max_thrust_N), 1e-9)
        left_ratio = clamp(float(left_thrust_N) / max_thrust, 0.0, 1.0)
        right_ratio = clamp(float(right_thrust_N) / max_thrust, 0.0, 1.0)
        thrust_util = max(left_ratio, right_ratio)
        self.thrust_util_integral_s += thrust_util * dt
        self.max_thrust_util = max(self.max_thrust_util, thrust_util)
        self.prop_power_index_integral_s += 0.5 * (left_ratio**1.5 + right_ratio**1.5) * dt
        reel_power = max(0.0, float(tension_N)) * float(reel_velocity_m_s)
        self.reel_abs_work_J += abs(reel_power) * dt
        if reel_power < 0.0:
            self.reel_in_work_J += -reel_power * dt
        else:
            self.reel_out_work_J += reel_power * dt
        if thrust_util >= 0.98:
            self.thrust_saturation_s += dt
        if max_tension_N > 0.0 and float(tension_N) >= 0.98 * float(max_tension_N):
            self.tension_saturation_s += dt
        if contact_valid:
            self.contact_valid_s += dt
        if work_mode:
            self.work_command_s += dt
        solve_time = max(0.0, float(mpc_solve_time_s))
        self.solver_time_integral_s2 += solve_time * dt
        self.solver_time_max_s = max(self.solver_time_max_s, solve_time)
        status = mpc_status or ""
        if status and not (
            status.startswith("Solve_Succeeded")
            or status.startswith("Solved_To_Acceptable_Level")
            or status.startswith("sensor-cascade")
        ):
            self.solver_failure_count += 1
        self.max_motor_rpm = max(
            self.max_motor_rpm,
            *(abs(float(speed)) * 60.0 / (2.0 * math.pi) for speed in motor_speed_rad_s),
        )
        self.max_reel_rpm = max(self.max_reel_rpm, abs(float(reel_rpm)))
        self.max_abs_pitch_rad = max(self.max_abs_pitch_rad, abs(float(pitch_rad)))

    def summary(self) -> dict[str, float]:
        elapsed = max(self.elapsed_s, 1e-9)
        rms_error = math.sqrt(self.tracking_error_sq_integral_m2_s / elapsed)
        contact_ratio = self.contact_valid_s / max(self.work_command_s, 1e-9) if self.work_command_s > 0.0 else 0.0
        return {
            "elapsed_s": self.elapsed_s,
            "mean_tracking_error_m": self.tracking_error_integral_m_s / elapsed,
            "rms_tracking_error_m": rms_error,
            "max_tracking_error_m": self.max_tracking_error_m,
            "mean_thrust_util": self.thrust_util_integral_s / elapsed,
            "max_thrust_util": self.max_thrust_util,
            "mean_prop_power_index": self.prop_power_index_integral_s / elapsed,
            "reel_abs_work_J": self.reel_abs_work_J,
            "reel_in_work_J": self.reel_in_work_J,
            "reel_out_work_J": self.reel_out_work_J,
            "thrust_saturation_pct": 100.0 * self.thrust_saturation_s / elapsed,
            "tension_saturation_pct": 100.0 * self.tension_saturation_s / elapsed,
            "contact_valid_pct_when_working": 100.0 * contact_ratio,
            "mean_mpc_solve_ms": 1000.0 * self.solver_time_integral_s2 / elapsed,
            "max_mpc_solve_ms": 1000.0 * self.solver_time_max_s,
            "solver_failure_count": float(self.solver_failure_count),
            "max_motor_rpm": self.max_motor_rpm,
            "max_reel_rpm": self.max_reel_rpm,
            "max_abs_pitch_rad": self.max_abs_pitch_rad,
        }

    def compact_text(self) -> str:
        data = self.summary()
        return (
            f"3D efficiency rms err {1000.0 * data['rms_tracking_error_m']:4.0f}mm  "
            f"mean thrust {100.0 * data['mean_thrust_util']:3.0f}%  "
            f"prop index {100.0 * data['mean_prop_power_index']:3.0f}%\n"
            f"3D reel work |E| {data['reel_abs_work_J']:5.3f}J  "
            f"sat thrust/tension {data['thrust_saturation_pct']:3.0f}%/{data['tension_saturation_pct']:3.0f}%  "
            f"solve {data['mean_mpc_solve_ms']:4.1f}/{data['max_mpc_solve_ms']:4.1f}ms"
        )


def print_efficiency_summary(efficiency: EfficiencyMonitor) -> None:
    data = efficiency.summary()
    print("3D controller/actuator efficiency:")
    print(
        f"  tracking error mean/rms/max: "
        f"{data['mean_tracking_error_m']:.4f} / {data['rms_tracking_error_m']:.4f} / "
        f"{data['max_tracking_error_m']:.4f} m"
    )
    print(
        f"  thrust utilization mean/max: "
        f"{100.0 * data['mean_thrust_util']:.1f}% / {100.0 * data['max_thrust_util']:.1f}%"
    )
    print(
        f"  prop power index mean: {100.0 * data['mean_prop_power_index']:.1f}% of max-thrust index"
    )
    print(
        f"  reel mechanical work abs/in/out: "
        f"{data['reel_abs_work_J']:.4f} / {data['reel_in_work_J']:.4f} / "
        f"{data['reel_out_work_J']:.4f} J"
    )
    print(
        f"  saturation time thrust/tension: "
        f"{data['thrust_saturation_pct']:.1f}% / {data['tension_saturation_pct']:.1f}%"
    )
    if data["max_mpc_solve_ms"] <= 0.0 and data["solver_failure_count"] <= 0.0:
        print("  controller solve: analytic cascade (no online nonlinear optimization)")
    else:
        print(
            f"  NMPC solve mean/max: {data['mean_mpc_solve_ms']:.1f} / "
            f"{data['max_mpc_solve_ms']:.1f} ms; solver failure samples: "
            f"{int(data['solver_failure_count'])}"
        )
    print(
        f"  peak motor/reel speed: {data['max_motor_rpm']:.0f} rpm / {data['max_reel_rpm']:.0f} rpm"
    )
    print(
        f"  peak payload pitch: {data['max_abs_pitch_rad']:.4f} rad "
        f"({math.degrees(data['max_abs_pitch_rad']):.2f} deg)"
    )


def motor_speed_step(
    current_speed: float,
    commanded_thrust: float,
    dt: float,
    args: argparse.Namespace,
    max_thrust: float,
) -> tuple[float, float]:
    max_speed = max(1e-6, float(args.max_motor_speed))
    k_f = max(1e-12, float(max_thrust) / (max_speed * max_speed))
    target_speed = math.sqrt(clamp(float(commanded_thrust), 0.0, float(max_thrust)) / k_f)
    tau = float(args.motor_tau_up) if target_speed >= current_speed else float(args.motor_tau_down)
    alpha = clamp(dt / max(tau + dt, 1e-9), 0.0, 1.0)
    speed = clamp(current_speed + alpha * (target_speed - current_speed), 0.0, max_speed)
    return speed, k_f * speed * speed


def update_force_arrow(
    sim,
    frame: int,
    stem: int,
    head: int,
    previous_stem_length: float,
    thrust: float,
    max_thrust: float,
) -> float:
    ratio = clamp(float(thrust) / max(float(max_thrust), 1e-9), 0.0, 1.0)
    total_length = 0.055 + 0.135 * ratio
    stem_length = max(0.010, total_length - scene_gen.FORCE_ARROW_HEAD_LENGTH)
    if previous_stem_length > 1e-9 and abs(stem_length - previous_stem_length) > 1e-5:
        sim.scaleObject(stem, 1.0, 1.0, stem_length / previous_stem_length, 0)
    sim.setObjectPosition(
        stem,
        frame,
        [0.0, 0.0, scene_gen.FORCE_ARROW_BASE_OFFSET + 0.5 * stem_length],
    )
    sim.setObjectPosition(
        head,
        frame,
        [
            0.0,
            0.0,
            scene_gen.FORCE_ARROW_BASE_OFFSET + stem_length + 0.5 * scene_gen.FORCE_ARROW_HEAD_LENGTH,
        ],
    )
    return stem_length


def steel_cable_spec_from_args(args: argparse.Namespace) -> SteelCableSpec:
    return SteelCableSpec(
        diameter_m=float(args.steel_cable_diameter),
        youngs_modulus_pa=float(args.steel_youngs_modulus),
        density_kg_m3=float(args.steel_density),
        structural_compliance_m_N=float(args.steel_cable_structural_compliance),
        damping_ratio=float(args.steel_cable_damping_ratio),
        payload_weight_fraction=float(args.steel_cable_payload_weight_fraction),
        min_visual_tension_N=float(args.steel_cable_min_visual_tension),
        max_visual_sag_m=float(args.steel_cable_max_visual_sag),
    )


def reel_motor_spec_from_args(args: argparse.Namespace) -> ReelMotorSpec:
    return ReelMotorSpec(
        voltage_v=float(args.reel_motor_voltage),
        gear_ratio=float(args.reel_gear_ratio),
        no_load_output_rpm=float(args.reel_no_load_rpm),
        stall_torque_kg_cm=float(args.reel_stall_torque_kg_cm),
        spool_radius_m=float(args.reel_spool_radius),
        velocity_time_constant_s=float(args.reel_velocity_tau),
        continuous_torque_fraction=float(args.reel_continuous_torque_fraction),
    )


def sensor_config_from_args(args: argparse.Namespace) -> SensorConfig:
    noise_enabled = bool(args.sensor_noise)
    return SensorConfig(
        reel_encoder_counts_per_output_rev=int(args.reel_encoder_counts_per_output_rev),
        cable_angle_encoder_counts_per_rev=int(args.cable_angle_encoder_counts_per_rev),
        reel_spool_radius_m=float(args.reel_spool_radius),
        reel_encoder_noise_counts_std=0.20 if noise_enabled else 0.0,
        cable_angle_noise_rad_std=math.radians(0.05) if noise_enabled else 0.0,
        load_cell_noise_N_std=0.010 if noise_enabled else 0.0,
        imu_gyro_noise_rad_s_std=math.radians(0.20) if noise_enabled else 0.0,
        imu_accel_noise_m_s2_std=0.030 if noise_enabled else 0.0,
        imu_gyro_bias_rad_s=math.radians(0.35) if noise_enabled else 0.0,
        imu_accel_bias_body_x_m_s2=0.020 if noise_enabled else 0.0,
        imu_accel_bias_body_z_m_s2=-0.015 if noise_enabled else 0.0,
        velocity_fusion_tau_s=float(args.estimator_velocity_fusion_tau),
        random_seed=int(args.sensor_random_seed),
    )


def validate_physical_args(args: argparse.Namespace) -> None:
    steel_cable_spec_from_args(args)
    reel_motor_spec_from_args(args)
    sensor_config_from_args(args)
    if int(args.cable_segments) < 2:
        raise ValueError("--cable-segments must be at least 2")
    if float(args.time_step) <= 0.0:
        raise ValueError("--time-step must be positive")
    if float(args.prop_visual_update_period) < 0.0:
        raise ValueError("--prop-visual-update-period cannot be negative")
    if float(args.cable_visual_update_period) < 0.0:
        raise ValueError("--cable-visual-update-period cannot be negative")
    if float(args.desired_path_update_period) < 0.0:
        raise ValueError("--desired-path-update-period cannot be negative")
    if int(args.desired_path_max_segments) < 1:
        raise ValueError("--desired-path-max-segments must be at least 1")
    if float(args.desired_path_radius) <= 0.0:
        raise ValueError("--desired-path-radius must be positive")
    if float(args.min_realtime_factor) < 0.0:
        raise ValueError("--min-realtime-factor cannot be negative")
    if float(args.max_motor_speed) <= 0.0:
        raise ValueError("--max-motor-speed must be positive")
    if float(args.max_cable_tension) <= 0.0:
        raise ValueError("--max-cable-tension must be positive")
    if float(args.standoff) <= 0.0:
        raise ValueError("--standoff must be positive")
    if float(args.wall_contact_stiffness) <= 0.0:
        raise ValueError("--wall-contact-stiffness must be positive")
    if float(args.wall_contact_damping) < 0.0:
        raise ValueError("--wall-contact-damping cannot be negative")
    if float(args.wall_contact_force_limit) <= 0.0:
        raise ValueError("--wall-contact-force-limit must be positive")
    if float(args.wall_contact_friction) < 0.0:
        raise ValueError("--wall-contact-friction cannot be negative")
    if float(args.wall_friction_transition_speed) <= 0.0:
        raise ValueError("--wall-friction-transition-speed must be positive")
    if float(args.guide_roll_yaw_stiffness) < 0.0:
        raise ValueError("--guide-roll-yaw-stiffness cannot be negative")
    if float(args.guide_pitch_stiffness) < 0.0:
        raise ValueError("--guide-pitch-stiffness cannot be negative")
    if float(args.guide_pitch_damping) < 0.0:
        raise ValueError("--guide-pitch-damping cannot be negative")
    if float(args.estimator_velocity_fusion_tau) < 0.0:
        raise ValueError("--estimator-velocity-fusion-tau cannot be negative")


def make_coppeliasim_simulator(args: argparse.Namespace):
    simulator = make_simulator(external_plant=True)
    requested_dt = float(args.time_step)
    if abs(simulator.params.dt - requested_dt) <= 1e-12:
        return simulator
    params_cls = simulator.params.__class__
    return make_simulator(
        params_cls(**{**simulator.params.__dict__, "dt": requested_dt}),
        external_plant=True,
    )


def steel_cable_visual_points(
    start: Sequence[float],
    end: Sequence[float],
    segment_count: int,
    tension_N: float,
    spec: SteelCableSpec,
    gravity: float,
) -> list[list[float]]:
    if segment_count < 2:
        raise ValueError("steel cable visual requires at least two segments")
    start_point = [float(value) for value in start]
    end_point = [float(value) for value in end]
    delta = sub3(end_point, start_point)
    horizontal_span = math.hypot(delta[0], delta[1])
    weight_per_length = spec.mass_per_length_kg_m * max(float(gravity), 0.0)
    visual_tension = max(float(tension_N), spec.min_visual_tension_N)
    sag = 0.0
    if horizontal_span > 1e-6:
        sag = weight_per_length * horizontal_span * horizontal_span / (8.0 * visual_tension)
        sag = clamp(sag, 0.0, spec.max_visual_sag_m)
    points: list[list[float]] = []
    for index in range(segment_count + 1):
        u = index / float(segment_count)
        point = [start_point[axis] + u * delta[axis] for axis in range(3)]
        point[2] -= 4.0 * sag * u * (1.0 - u)
        points.append(point)
    return points


def update_steel_cable_visual(
    sim,
    handles: SceneHandles,
    anchor: Sequence[float],
    mount: Sequence[float],
    previous_lengths: list[float] | None,
    tension_N: float,
    args: argparse.Namespace,
    params,
) -> list[float]:
    spec = steel_cable_spec_from_args(args)
    points = steel_cable_visual_points(anchor, mount, len(handles.cable_segments), tension_N, spec, params.gravity)
    if previous_lengths is None:
        previous_lengths = [
            max(1e-6, norm3(sub3(points[index + 1], points[index])))
            for index in range(len(handles.cable_segments))
        ]
    if len(previous_lengths) != len(handles.cable_segments):
        raise RuntimeError("steel cable visual length state does not match resolved CoppeliaSim segments")
    updated_lengths: list[float] = []
    for index, segment in enumerate(handles.cable_segments):
        updated_lengths.append(
            sim_utils.update_cylinder_between(
                sim,
                segment,
                points[index],
                points[index + 1],
                previous_lengths[index],
            )
        )
    return updated_lengths


def initialize_dynamic_body(sim, handles: SceneHandles, simulator, args: argparse.Namespace) -> DynamicPlantState:
    params = simulator.params
    update_payload_pose(
        sim,
        handles.payload,
        params.initial_payload[0],
        params.initial_payload[1],
        args.standoff,
        params.nominal_attitude_rad,
    )
    sim.resetDynamicObject(handles.payload)
    matrix = sim.getObjectMatrix(handles.payload, -1)
    mount = local_point_to_world(matrix, [0.0, 0.0, params.payload_hex_radius])
    anchor = [float(value) for value in sim.getObjectPosition(handles.anchor, -1)]
    cable_length = max(1e-6, norm3(sub3(mount, anchor)))
    hover_thrust = min(
        params.max_thrust_per_drone,
        params.total_mass * params.gravity
        / max(2.0 * math.cos(params.hex_face_tilt_rad), 1e-9),
    )
    hover_motor_speed = float(args.max_motor_speed) * math.sqrt(
        hover_thrust / max(params.max_thrust_per_drone, 1e-9)
    )
    plant_state = DynamicPlantState(
        reel_length=clamp(cable_length, params.min_cable_length, params.max_cable_length),
        anchor_position=anchor,
        left_motor_speed=hover_motor_speed,
        right_motor_speed=hover_motor_speed,
        last_left_thrust=hover_thrust,
        last_right_thrust=hover_thrust,
    )
    if str(args.feedback_mode) == "sensor":
        plant_state.sensor_pipeline = WallToolSensorPipeline(
            sensor_config_from_args(args),
            anchor_xz_m=(anchor[0], anchor[2]),
            cable_mount_radius_m=params.payload_hex_radius,
            gravity_m_s2=params.gravity,
            steel_cable=steel_cable_spec_from_args(args),
        )
    plant_state.cable_visual_lengths = update_steel_cable_visual(
        sim,
        handles,
        anchor,
        mount,
        plant_state.cable_visual_lengths,
        0.0,
        args,
        params,
    )
    return plant_state


def sync_controller_from_dynamic_body(
    sim,
    simulator,
    handles: SceneHandles,
    plant_state: DynamicPlantState,
    body_sample: DynamicBodySample | None = None,
) -> None:
    params = simulator.params
    body_sample = body_sample or read_dynamic_body_sample(sim, handles, plant_state)
    if plant_state.sensor_pipeline is None:
        xz_position = (float(body_sample.position[0]), float(body_sample.position[2]))
        xz_velocity = (float(body_sample.linear_velocity[0]), float(body_sample.linear_velocity[2]))
        attitude = coppelia_pitch_to_planar_attitude(float(body_sample.orientation[1]))
        angular_velocity = -float(body_sample.angular_velocity[1])
        reel_length = float(plant_state.reel_length or params.min_cable_length)
        reel_velocity = float(plant_state.reel_velocity)
        tension = float(plant_state.last_tension)
    else:
        cable_mount = local_point_to_world(
            body_sample.matrix,
            [0.0, 0.0, params.payload_hex_radius],
        )
        estimate = plant_state.sensor_pipeline.update(
            SensorTruth(
                timestamp_s=body_sample.sim_time,
                reel_length_m=float(plant_state.reel_length or params.min_cable_length),
                cable_tension_N=float(plant_state.last_tension),
                anchor_xz_m=(float(body_sample.anchor[0]), float(body_sample.anchor[2])),
                cable_mount_xz_m=(float(cable_mount[0]), float(cable_mount[2])),
                payload_attitude_rad=coppelia_pitch_to_planar_attitude(float(body_sample.orientation[1])),
                payload_angular_rate_rad_s=-float(body_sample.angular_velocity[1]),
                payload_velocity_xz_m_s=(
                    float(body_sample.linear_velocity[0]),
                    float(body_sample.linear_velocity[2]),
                ),
            )
        )
        plant_state.last_estimate = estimate
        xz_position = estimate.payload_position_xz_m
        xz_velocity = estimate.payload_velocity_xz_m_s
        attitude = estimate.payload_attitude_rad
        angular_velocity = estimate.payload_angular_rate_rad_s
        reel_length = estimate.reel_length_m
        reel_velocity = estimate.reel_velocity_m_s
        tension = estimate.cable_tension_N

    simulator.position = xz_position
    simulator.velocity = xz_velocity
    simulator.attitude = attitude
    simulator.angular_velocity = angular_velocity
    simulator.cable_length = clamp(reel_length, params.min_cable_length, params.max_cable_length)
    simulator.measured_cable_length = simulator.cable_length
    simulator.actual_reel_velocity = reel_velocity
    simulator.measured_cable_velocity = reel_velocity
    simulator.actual_left_thrust = float(plant_state.last_left_thrust)
    simulator.actual_right_thrust = float(plant_state.last_right_thrust)
    simulator.actual_tension = clamp(tension, 0.0, params.max_spool_tension)
    simulator.load_cell_tension = simulator.actual_tension
    simulator.measured_tension = simulator.actual_tension
    simulator.measured_payload = xz_position
    simulator.estimated_payload_velocity = xz_velocity
    simulator.measured_attitude = simulator.attitude
    simulator.measured_angular_velocity = simulator.angular_velocity


def apply_dynamic_wrenches(
    sim,
    handles: SceneHandles,
    plant_state: DynamicPlantState,
    command_state,
    args: argparse.Namespace,
    params,
    body_sample: DynamicBodySample | None = None,
) -> dict[str, object]:
    body_sample = body_sample or read_dynamic_body_sample(sim, handles, plant_state)
    now = body_sample.sim_time
    if plant_state.last_time is None:
        dt = max(1e-4, float(args.time_step))
    else:
        dt = clamp(now - plant_state.last_time, 1e-4, 0.05)
    plant_state.last_time = now

    matrix = body_sample.matrix
    position = body_sample.position
    attitude = coppelia_pitch_to_planar_attitude(float(body_sample.orientation[1]))
    linear_velocity = body_sample.linear_velocity
    angular_velocity = body_sample.angular_velocity

    plant_state.left_motor_speed, left_thrust = motor_speed_step(
        plant_state.left_motor_speed,
        command_state.left_thrust,
        dt,
        args,
        params.max_thrust_per_drone,
    )
    plant_state.right_motor_speed, right_thrust = motor_speed_step(
        plant_state.right_motor_speed,
        command_state.right_thrust,
        dt,
        args,
        params.max_thrust_per_drone,
    )
    plant_state.last_left_thrust = left_thrust
    plant_state.last_right_thrust = right_thrust

    net_force = [0.0, 0.0, 0.0]
    net_torque = [0.0, 0.0, 0.0]
    drone_force_xz = [0.0, 0.0]
    motor_torque_y = [0.0, 0.0]
    left_offset, right_offset = integrated_motor_center_offsets(params, 0.0)
    tilt = params.hex_face_tilt_rad
    motor_specs = (
        ([left_offset[0], 0.0, left_offset[1]], [math.sin(tilt), 0.0, math.cos(tilt)], left_thrust),
        ([right_offset[0], 0.0, right_offset[1]], [-math.sin(tilt), 0.0, math.cos(tilt)], right_thrust),
    )
    for index, (local_offset, local_axis, thrust) in enumerate(motor_specs):
        motor_point = local_point_to_world(matrix, local_offset)
        motor_axis = normalize3(local_vector_to_world(matrix, local_axis))
        motor_force = scale3(motor_axis, thrust)
        motor_arm = sub3(motor_point, position)
        motor_torque = cross3(motor_arm, motor_force)
        drone_force_xz[0] += motor_force[0]
        drone_force_xz[1] += motor_force[2]
        motor_torque_y[index] = motor_torque[1]
        add_wrench_at_point(net_force, net_torque, motor_force, motor_point, position)

    anchor = body_sample.anchor
    mount = local_point_to_world(matrix, [0.0, 0.0, params.payload_hex_radius])
    mount_arm = sub3(mount, position)
    mount_velocity = add3(linear_velocity, cross3(angular_velocity, mount_arm))
    anchor_to_mount = sub3(mount, anchor)
    cable_distance = max(1e-6, norm3(anchor_to_mount))
    cable_out = scale3(anchor_to_mount, 1.0 / cable_distance)
    cable_to_anchor = scale3(cable_out, -1.0)
    if plant_state.reel_length is None:
        plant_state.reel_length = cable_distance
    previous_reel_length = plant_state.reel_length
    reel_motor = reel_motor_spec_from_args(args)
    velocity_command = clamp(
        float(command_state.spool_velocity_cmd),
        -min(params.max_spool_speed, reel_motor.max_line_speed_m_s),
        min(params.max_spool_speed, reel_motor.max_line_speed_m_s),
    )
    plant_state.reel_velocity = reel_motor.velocity_step(
        plant_state.reel_velocity,
        velocity_command,
        plant_state.last_tension,
        dt,
    )
    plant_state.reel_length = clamp(
        plant_state.reel_length + plant_state.reel_velocity * dt,
        params.min_cable_length,
        params.max_cable_length,
    )
    actual_spool_velocity = (plant_state.reel_length - previous_reel_length) / max(dt, 1e-9)
    plant_state.reel_velocity = actual_spool_velocity
    length_rate = dot3(cable_out, mount_velocity)
    extension = cable_distance - plant_state.reel_length
    extension_rate = length_rate - actual_spool_velocity
    steel_cable = steel_cable_spec_from_args(args)
    cable_stiffness = steel_cable.axial_stiffness_N_m(cable_distance)
    cable_effective_mass = params.total_mass + steel_cable.mass_kg(cable_distance) / 3.0
    cable_damping = steel_cable.damping_N_s_m(cable_distance, cable_effective_mass)
    raw_tension = (
        cable_stiffness * max(0.0, extension)
        + cable_damping * extension_rate
    )
    taut = extension >= -abs(float(args.cable_taut_band))
    tension = clamp(raw_tension if taut else 0.0, 0.0, float(args.max_cable_tension))
    plant_state.last_tension = tension
    cable_force_world = scale3(cable_to_anchor, tension)
    cable_arm = sub3(mount, position)
    cable_torque_world = cross3(cable_arm, cable_force_world)
    add_wrench_at_point(net_force, net_torque, cable_force_world, mount, position)
    cable_weight_payload_N = steel_cable.payload_weight_fraction * steel_cable.weight_N(cable_distance, params.gravity)
    cable_weight_force = [0.0, 0.0, -cable_weight_payload_N]
    add_wrench_at_point(net_force, net_torque, cable_weight_force, mount, position)

    desired_contact_force = 0.0
    if bool(command_state.work_mode):
        desired_contact_force = max(0.0, float(command_state.desired_contact_force))
    desired_penetration = desired_contact_force / max(float(args.wall_contact_stiffness), 1e-9)
    guide_preload_deflection = desired_contact_force / max(float(args.normal_standoff_kp), 1e-9)
    # This is a passive mechanical guide equilibrium, not an unmeasured y-axis
    # feedback controller. At the desired penetration, guide preload balances
    # the wall reaction force.
    desired_y = -abs(float(args.standoff)) + desired_penetration + guide_preload_deflection
    net_force[1] += float(args.normal_standoff_kp) * (desired_y - position[1])
    net_force[1] -= float(args.normal_standoff_kd) * linear_velocity[1]
    pen_tip = local_point_to_world(matrix, [0.0, float(args.standoff), 0.0])
    pen_arm = sub3(pen_tip, position)
    pen_velocity = add3(linear_velocity, cross3(angular_velocity, pen_arm))
    wall_penetration = max(0.0, float(pen_tip[1]))
    wall_contact_force_N = 0.0
    if wall_penetration > 0.0:
        wall_contact_force_N = clamp(
            float(args.wall_contact_stiffness) * wall_penetration
            + float(args.wall_contact_damping) * max(0.0, float(pen_velocity[1])),
            0.0,
            float(args.wall_contact_force_limit),
        )
        wall_normal_force = [0.0, -wall_contact_force_N, 0.0]
        add_wrench_at_point(net_force, net_torque, wall_normal_force, pen_tip, position)
        tangential_velocity = [float(pen_velocity[0]), 0.0, float(pen_velocity[2])]
        tangential_speed = norm3(tangential_velocity)
        if tangential_speed > 1e-6 and float(args.wall_contact_friction) > 0.0:
            # Smooth roller/contact resistance. The tanh transition removes
            # the discontinuous near-zero Coulomb force that caused artificial
            # stick-slip and steady tracking bias.
            friction_magnitude = (
                float(args.wall_contact_friction)
                * wall_contact_force_N
                * math.tanh(tangential_speed / float(args.wall_friction_transition_speed))
            )
            friction_force = scale3(
                tangential_velocity,
                -friction_magnitude / tangential_speed,
            )
            add_wrench_at_point(net_force, net_torque, friction_force, pen_tip, position)
    net_force[0] -= float(args.linear_drag_xz) * linear_velocity[0]
    net_force[2] -= float(args.linear_drag_xz) * linear_velocity[2]
    net_torque[0] -= float(args.angular_drag_roll_yaw) * angular_velocity[0]
    net_torque[1] -= float(args.angular_drag_y) * angular_velocity[1]
    net_torque[2] -= float(args.angular_drag_roll_yaw) * angular_velocity[2]
    net_torque[0] -= float(args.guide_roll_yaw_stiffness) * float(body_sample.orientation[0])
    net_torque[2] -= float(args.guide_roll_yaw_stiffness) * float(body_sample.orientation[2])
    # Passive wall rollers/guide rails constrain payload pitch. This is a
    # mechanical spring-damper assumption, not an unmeasured active controller.
    net_torque[1] -= float(args.guide_pitch_stiffness) * float(body_sample.orientation[1])
    net_torque[1] -= float(args.guide_pitch_damping) * float(angular_velocity[1])

    if now - plant_state.last_prop_visual_time >= float(args.prop_visual_update_period):
        plant_state.left_prop_phase += plant_state.left_motor_speed * dt
        plant_state.right_prop_phase -= plant_state.right_motor_speed * dt
        sim.setJointPosition(handles.left_prop_joint, plant_state.left_prop_phase)
        sim.setJointPosition(handles.right_prop_joint, plant_state.right_prop_phase)
        plant_state.left_force_arrow_stem_length = update_force_arrow(
            sim,
            handles.left_motor_frame,
            handles.left_force_arrow_stem,
            handles.left_force_arrow_head,
            plant_state.left_force_arrow_stem_length,
            left_thrust,
            params.max_thrust_per_drone,
        )
        plant_state.right_force_arrow_stem_length = update_force_arrow(
            sim,
            handles.right_motor_frame,
            handles.right_force_arrow_stem,
            handles.right_force_arrow_head,
            plant_state.right_force_arrow_stem_length,
            right_thrust,
            params.max_thrust_per_drone,
        )
        plant_state.last_prop_visual_time = now

    if (
        plant_state.cable_visual_lengths is None
        or now - plant_state.last_cable_visual_time >= float(args.cable_visual_update_period)
    ):
        plant_state.cable_visual_lengths = update_steel_cable_visual(
            sim,
            handles,
            anchor,
            mount,
            plant_state.cable_visual_lengths,
            tension,
            args,
            params,
        )
        plant_state.last_cable_visual_time = now
    sim.addForceAndTorque(handles.payload, net_force, net_torque)
    return {
        "time": now,
        "dt": dt,
        "position": position,
        "attitude": attitude,
        "pen_tip_position": pen_tip,
        "motor_thrust": [left_thrust, right_thrust],
        "motor_speed": [plant_state.left_motor_speed, plant_state.right_motor_speed],
        "cable_mount": mount,
        "cable_distance": cable_distance,
        "cable_extension": extension,
        "cable_stiffness_N_m": cable_stiffness,
        "cable_damping_N_s_m": cable_damping,
        "cable_payload_weight_N": cable_weight_payload_N,
        "wall_contact_force_N": wall_contact_force_N,
        "wall_penetration_m": wall_penetration,
        "cable_slack": not taut or tension <= 1e-9,
        "reel_length": plant_state.reel_length,
        "reel_velocity_command": velocity_command,
        "reel_motor_rpm": reel_motor.line_speed_to_output_rpm(actual_spool_velocity),
        "reel_max_line_speed": reel_motor.max_line_speed_m_s,
        "reel_stall_line_force": reel_motor.stall_line_force_N,
        "reel_continuous_line_force": reel_motor.continuous_line_force_N,
        "actual_spool_velocity": actual_spool_velocity,
        "tension": tension,
        "drone_force_xz": drone_force_xz,
        "cable_force_xz": [cable_force_world[0], cable_force_world[2]],
        "cable_torque_y": -cable_torque_world[1],
        "left_torque_y": -motor_torque_y[0],
        "right_torque_y": -motor_torque_y[1],
        "linear_velocity": linear_velocity,
        "angular_velocity": angular_velocity,
        "net_force": net_force,
        "net_torque": net_torque,
    }


class CoppeliaSimWallToolAdapter:
    """Adapter that lets the native 2D UI drive and display the 3D plant."""

    def __init__(
        self,
        client,
        sim,
        handles: SceneHandles,
        simulator,
        plant_state: DynamicPlantState,
        args: argparse.Namespace,
    ) -> None:
        self.client = client
        self.sim = sim
        self.handles = handles
        self.controller = simulator
        self.params = simulator.params
        self.plant_state = plant_state
        self.args = args
        self.trajectory = simulator.trajectory
        self.state_lock = threading.RLock()
        self._command_queue: queue.Queue[tuple[str, tuple[Any, ...], dict[str, Any]]] = queue.Queue()
        self.async_running = False
        self.async_last_error: BaseException | None = None
        self.async_steps = 0
        self.sensor_text = ""
        self.ink_handles: list[int] = []
        self.last_ink_point: tuple[float, float] | None = None
        self.path_preview = DesiredPathPreview(sim, args)
        self.efficiency = EfficiencyMonitor()
        self._last_theta: float | None = None
        self._last_length: float | None = None
        self._last_length_dot = 0.0
        self._last_velocity: tuple[float, float] | None = None
        self._last_angular_velocity: float | None = None
        self._last_time: float | None = None
        self.history = [self._sensor_state_from_3d(simulator.history[-1], None)]
        self._update_desired_path_preview(force=True)

    def _run_or_queue(self, command: str, *args: Any, **kwargs: Any) -> None:
        if self.async_running:
            self._command_queue.put((command, args, kwargs))
            return
        with self.state_lock:
            self._run_command_locked(command, *args, **kwargs)

    def _run_command_locked(self, command: str, *args: Any, **kwargs: Any) -> None:
        method = getattr(self, f"_do_{command}")
        method(*args, **kwargs)

    def _drain_command_queue_locked(self) -> None:
        while True:
            try:
                command, args, kwargs = self._command_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._run_command_locked(command, *args, **kwargs)
            finally:
                self._command_queue.task_done()

    def process_pending_commands(self) -> None:
        with self.state_lock:
            self._drain_command_queue_locked()

    def latest_state(self):
        with self.state_lock:
            return self.history[-1]

    def ink_dot_count(self) -> int:
        with self.state_lock:
            return len(self.ink_handles)

    def step(self):
        with self.state_lock:
            self._drain_command_queue_locked()
            body_sample = read_dynamic_body_sample(self.sim, self.handles, self.plant_state)
            sync_controller_from_dynamic_body(
                self.sim,
                self.controller,
                self.handles,
                self.plant_state,
                body_sample,
            )
            command_state = self.controller.step()
            sample = apply_dynamic_wrenches(
                self.sim,
                self.handles,
                self.plant_state,
                command_state,
                self.args,
                self.params,
                body_sample,
            )
            self.client.step()
            sensor_state = self._sensor_state_from_3d(command_state, sample)
            self._update_efficiency(sensor_state, command_state, sample)
            self._update_desired_path_preview(sim_time=float(sensor_state.t))
            self._maybe_add_ink(sensor_state, command_state)
            self.history.append(sensor_state)
            self.async_steps += 1
            if len(self.history) > 5000:
                del self.history[: len(self.history) - 5000]
            return sensor_state

    def reset(self) -> None:
        self._run_or_queue("reset")

    def _do_reset(self) -> None:
        self.controller.reset()
        self.plant_state = initialize_dynamic_body(self.sim, self.handles, self.controller, self.args)
        self.history = [self._sensor_state_from_3d(self.controller.history[-1], None)]
        self.trajectory = self.controller.trajectory
        self.ink_handles.clear()
        self.last_ink_point = None
        self.sensor_text = ""
        self.efficiency = EfficiencyMonitor()
        self._update_desired_path_preview(force=True)

    def clear_trajectory(self) -> None:
        self._run_or_queue("clear_trajectory")

    def _do_clear_trajectory(self) -> None:
        self._sync_controller_state()
        self.controller.clear_trajectory()
        self._update_desired_path_preview(force=True)

    def clear_trace(self) -> None:
        self._run_or_queue("clear_trace")

    def _do_clear_trace(self) -> None:
        self._sync_controller_state()
        self.controller.clear_trajectory()
        self.history = self.history[-1:]
        self.efficiency = EfficiencyMonitor()
        self._update_desired_path_preview(force=True)

    def set_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._run_or_queue("set_target", point, planner=planner)

    def _do_set_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._sync_controller_state()
        self.controller.set_target(point, planner=planner)
        self._move_target_marker(point)
        self._update_desired_path_preview(force=True)

    def append_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._run_or_queue("append_target", point, planner=planner)

    def _do_append_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._sync_controller_state()
        self.controller.append_target(point, planner=planner)
        self._move_target_marker(point)
        self._update_desired_path_preview(force=True)

    def append_stop_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._run_or_queue("append_stop_target", point, planner=planner)

    def _do_append_stop_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._sync_controller_state()
        self.controller.append_stop_target(point, planner=planner)
        self._move_target_marker(point)
        self._update_desired_path_preview(force=True)

    def set_smooth_path(self, points: Sequence[tuple[float, float]]) -> None:
        self._run_or_queue("set_smooth_path", tuple(points))

    def _do_set_smooth_path(self, points: Sequence[tuple[float, float]]) -> None:
        self._sync_controller_state()
        self.controller.set_smooth_path(points)
        if points:
            self._move_target_marker(points[-1])
        self._update_desired_path_preview(force=True)

    def set_corner_smooth_path(self, points: Sequence[tuple[float, float]], corner_speed: float) -> None:
        self._run_or_queue("set_corner_smooth_path", tuple(points), corner_speed)

    def _do_set_corner_smooth_path(self, points: Sequence[tuple[float, float]], corner_speed: float) -> None:
        self._sync_controller_state()
        self.controller.set_corner_smooth_path(points, corner_speed)
        if points:
            self._move_target_marker(points[-1])
        self._update_desired_path_preview(force=True)

    def planned_waypoints(self, point: tuple[float, float], planner: str = BEST_PLANNER):
        with self.state_lock:
            self._sync_controller_state()
            return self.controller.planned_waypoints(point, planner)

    def _clamp_wall_point(self, point: tuple[float, float]) -> tuple[float, float]:
        return self.controller._clamp_wall_point(point)

    def _module_center_offsets(self, attitude: float):
        return self.controller._module_center_offsets(attitude)

    def _drone_axes(self, attitude: float):
        return self.controller._drone_axes(attitude)

    def _sync_controller_state(self) -> None:
        sync_controller_from_dynamic_body(self.sim, self.controller, self.handles, self.plant_state)

    def _move_target_marker(self, point: Sequence[float]) -> None:
        self.sim.setObjectPosition(self.handles.target, -1, [float(point[0]), -0.010, float(point[1])])
        print(f"3D target command: x={float(point[0]):.3f} m, z={float(point[1]):.3f} m", flush=True)

    def _update_desired_path_preview(self, sim_time: float | None = None, *, force: bool = False) -> None:
        if sim_time is None:
            sim_time = float(self.sim.getSimulationTime())
        self.path_preview.maybe_update(float(sim_time), self.controller.trajectory, force=force)

    def _update_efficiency(self, sensor_state, command_state, sample: dict[str, object]) -> None:
        self.efficiency.update(
            dt=float(sample["dt"]),
            tool_error_m=float(sensor_state.tool_error),
            left_thrust_N=float(sensor_state.left_thrust),
            right_thrust_N=float(sensor_state.right_thrust),
            max_thrust_N=float(self.params.max_thrust_per_drone),
            tension_N=float(sensor_state.measured_tension),
            max_tension_N=float(self.args.max_cable_tension),
            reel_velocity_m_s=float(sample["actual_spool_velocity"]),
            motor_speed_rad_s=sample["motor_speed"],
            reel_rpm=float(sample["reel_motor_rpm"]),
            pitch_rad=float(sample["attitude"]),
            mpc_solve_time_s=float(sensor_state.mpc_solve_time_s),
            mpc_status=str(sensor_state.mpc_status),
            contact_valid=bool(sensor_state.contact_valid),
            work_mode=bool(command_state.work_mode),
        )

    def _sensor_state_from_3d(self, base_state, sample: dict[str, object] | None):
        matrix = list(self.sim.getObjectMatrix(self.handles.payload, -1))
        position = [float(matrix[3]), float(matrix[7]), float(matrix[11])]
        attitude = coppelia_pitch_to_planar_attitude(float(self.sim.getObjectOrientation(self.handles.payload, -1)[1]))
        linear_velocity, angular_velocity = self.sim.getVelocity(self.handles.payload)
        linear_velocity = [float(value) for value in linear_velocity]
        angular_velocity = [float(value) for value in angular_velocity]
        pen_position = self.sim.getObjectPosition(self.handles.pen_tip, -1)
        anchor = self.sim.getObjectPosition(self.handles.anchor, -1)
        cable_mount = local_point_to_world(matrix, [0.0, 0.0, self.params.payload_hex_radius])
        anchor_to_mount = sub3(cable_mount, anchor)
        line_length = max(1e-6, norm3(anchor_to_mount))
        theta = math.atan2(cable_mount[0] - anchor[0], anchor[2] - cable_mount[2])

        now = float(self.sim.getSimulationTime())
        dt = max(1e-6, now - self._last_time) if self._last_time is not None else max(1e-6, self.params.dt)
        payload = (position[0], position[2])
        payload_velocity = (linear_velocity[0], linear_velocity[2])
        if self._last_velocity is None:
            payload_acceleration = (0.0, 0.0)
        else:
            payload_acceleration = (
                (payload_velocity[0] - self._last_velocity[0]) / dt,
                (payload_velocity[1] - self._last_velocity[1]) / dt,
            )
        if self._last_theta is None:
            theta_dot = 0.0
        else:
            theta_dot = (theta - self._last_theta + math.pi) % (2.0 * math.pi) - math.pi
            theta_dot /= dt
        length_dot = 0.0 if self._last_length is None else (line_length - self._last_length) / dt
        length_ddot = 0.0 if self._last_length is None else (length_dot - self._last_length_dot) / dt
        angular_velocity_y = -angular_velocity[1]
        angular_acceleration_y = (
            0.0 if self._last_angular_velocity is None else (angular_velocity_y - self._last_angular_velocity) / dt
        )

        tension = float(sample["tension"]) if sample is not None else self.plant_state.last_tension
        reel_length = (
            float(sample["reel_length"])
            if sample is not None and sample.get("reel_length") is not None
            else float(self.plant_state.reel_length or line_length)
        )
        cable_extension = max(0.0, line_length - reel_length)
        cable_slack = bool(sample["cable_slack"]) if sample is not None else tension <= 1e-9
        measured_cable_velocity = float(sample["actual_spool_velocity"]) if sample is not None else 0.0
        drone_force = tuple(sample["drone_force_xz"]) if sample is not None else base_state.drone_force
        cable_force = tuple(sample["cable_force_xz"]) if sample is not None else base_state.cable_force
        left_thrust, right_thrust = (
            tuple(sample["motor_thrust"]) if sample is not None else (base_state.left_thrust, base_state.right_thrust)
        )
        left_torque = float(sample["left_torque_y"]) if sample is not None else base_state.left_torque
        right_torque = float(sample["right_torque_y"]) if sample is not None else base_state.right_torque
        cable_torque = float(sample["cable_torque_y"]) if sample is not None else base_state.cable_torque
        spool_velocity = measured_cable_velocity if sample is not None else base_state.spool_velocity_cmd
        wall_contact_force = (
            float(sample["wall_contact_force_N"]) if sample is not None else float(base_state.contact_force)
        )

        tool_head = (float(pen_position[0]), float(pen_position[2]))
        desired_tool = base_state.desired_tool_head
        tool_error = math.hypot(tool_head[0] - desired_tool[0], tool_head[1] - desired_tool[1])
        contact_valid = bool(
            base_state.work_mode
            and self.params.min_contact_force_N <= wall_contact_force <= self.params.max_contact_force_N
            and abs(float(pen_position[1])) <= max(0.040, self.args.standoff * 0.45)
            and tool_error <= self.params.work_contact_tracking_limit_m
            and math.hypot(payload_velocity[0], payload_velocity[1]) <= self.params.work_contact_speed_limit_mps
            and abs(angular_velocity_y) <= self.params.work_contact_angular_rate_limit_rad_s
        )

        self._last_theta = theta
        self._last_length = line_length
        self._last_length_dot = length_dot
        self._last_velocity = payload_velocity
        self._last_angular_velocity = angular_velocity_y
        self._last_time = now

        motor_speed = sample["motor_speed"] if sample is not None else (0.0, 0.0)
        rpm_l = float(motor_speed[0]) * 60.0 / (2.0 * math.pi)
        rpm_r = float(motor_speed[1]) * 60.0 / (2.0 * math.pi)
        cable_stiffness = float(sample["cable_stiffness_N_m"]) if sample is not None else 0.0
        cable_weight = float(sample["cable_payload_weight_N"]) if sample is not None else 0.0
        reel_rpm = float(sample["reel_motor_rpm"]) if sample is not None else 0.0
        if self.plant_state.last_estimate is None:
            feedback_text = "feedback ground-truth CoppeliaSim state"
        else:
            estimate = self.plant_state.last_estimate
            estimate_position_error = math.hypot(
                estimate.payload_position_xz_m[0] - payload[0],
                estimate.payload_position_xz_m[1] - payload[1],
            )
            estimate_velocity_error = math.hypot(
                estimate.payload_velocity_xz_m_s[0] - payload_velocity[0],
                estimate.payload_velocity_xz_m_s[1] - payload_velocity[1],
            )
            feedback_text = (
                f"feedback sensors  est error p/v {1000.0 * estimate_position_error:4.1f}mm/"
                f"{estimate_velocity_error:4.2f}m/s  cable angle {math.degrees(estimate.cable_angle_rad):+5.1f}deg"
            )
        self.sensor_text = (
            f"3D sensors line {line_length:4.2f}m  reel {reel_length:4.2f}m  stretch {1000.0 * cable_extension:5.1f}mm\n"
            f"3D steel cable k {cable_stiffness:6.0f}N/m  carried weight {cable_weight:4.2f}N  slack {cable_slack}\n"
            f"3D pose y {position[1]:+.3f}m  pen_y {float(pen_position[1]):+.3f}m  contact {wall_contact_force:4.2f}N\n"
            f"3D motors rpm L/R {rpm_l:5.0f}/{rpm_r:5.0f}  reel {reel_rpm:+5.0f}\n"
            f"{feedback_text}\n"
            f"{self.efficiency.compact_text()}"
        )

        return replace(
            base_state,
            t=now,
            theta=theta,
            theta_dot=theta_dot,
            length=line_length,
            length_dot=length_dot,
            length_ddot=length_ddot,
            attitude=attitude,
            angular_velocity=angular_velocity_y,
            angular_acceleration=angular_acceleration_y,
            cable_length=reel_length,
            cable_stretch=cable_extension,
            cable_slack=cable_slack,
            cable_tension_saturated=tension >= self.args.max_cable_tension - 1e-6,
            payload_velocity=payload_velocity,
            payload_acceleration=payload_acceleration,
            payload=payload,
            measured_payload=payload,
            estimated_payload_velocity=payload_velocity,
            measured_theta=theta,
            measured_theta_dot=theta_dot,
            measured_line_length=line_length,
            measured_attitude=attitude,
            measured_angular_velocity=angular_velocity_y,
            measured_cable_velocity=measured_cable_velocity,
            tool_head=tool_head,
            measured_tool_error=tool_error,
            spool_velocity_cmd=spool_velocity,
            measured_cable_length=reel_length,
            measured_tension=tension,
            drone_force=drone_force,
            cable_force=cable_force,
            normal_gap=float(pen_position[1]),
            contact_force=wall_contact_force,
            contact_valid=contact_valid,
            cable_torque=cable_torque,
            left_torque=left_torque,
            right_torque=right_torque,
            left_thrust=float(left_thrust),
            right_thrust=float(right_thrust),
            tension=tension,
            drone_vertical_force=max(0.0, float(drone_force[1])),
            cable_vertical_force=max(0.0, float(cable_force[1])),
            path_error=tool_error,
            tool_error=tool_error,
            saturated=tension >= self.args.max_cable_tension - 1e-6,
        )

    def _maybe_add_ink(self, sensor_state, command_state) -> None:
        pen_x, pen_z = sensor_state.tool_head
        in_wall_bounds = abs(pen_x) <= 0.5 * self.args.wall_width and 0.0 <= pen_z <= self.args.wall_height
        if not in_wall_bounds or not sensor_state.contact_valid:
            return
        self.last_ink_point = maybe_add_ink(
            self.sim,
            self.ink_handles,
            self.last_ink_point,
            pen_x,
            pen_z,
            self.args,
        )


class AsyncCoppeliaSimUiRunner:
    """Run CoppeliaSim/controller steps off the Matplotlib UI thread."""

    def __init__(self, adapter: CoppeliaSimWallToolAdapter, app) -> None:
        self.adapter = adapter
        self.app = app
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self.adapter.async_last_error = None
        self.adapter.async_running = True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="wall-tool-coppeliasim-worker", daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, float(timeout_s)))
            if self._thread.is_alive():
                print("Warning: async CoppeliaSim worker did not stop before timeout.", flush=True)
                return
        self.adapter.async_running = False

    def _playing(self) -> bool:
        return bool(getattr(self.app, "playing", True))

    def _speed(self) -> float:
        try:
            return clamp(float(self.app.speed_slider.val), 0.05, 8.0)
        except Exception:
            return 1.0

    def _run(self) -> None:
        try:
            perf_wall_time = time.perf_counter()
            perf_sim_time = float(self.adapter.latest_state().t)
            while not self._stop_event.is_set():
                self.adapter.process_pending_commands()
                if not self._playing():
                    self._stop_event.wait(0.020)
                    continue

                step_start = time.perf_counter()
                self.adapter.step()
                now_wall = time.perf_counter()
                if now_wall - perf_wall_time >= 2.0:
                    now_sim = float(self.adapter.latest_state().t)
                    recent_realtime_factor = (now_sim - perf_sim_time) / max(now_wall - perf_wall_time, 1e-9)
                    if (
                        float(self.adapter.args.min_realtime_factor) > 0.0
                        and recent_realtime_factor < float(self.adapter.args.min_realtime_factor)
                    ):
                        print(
                            f"Warning: CoppeliaSim realtime factor {recent_realtime_factor:.2f}x "
                            f"is below target {float(self.adapter.args.min_realtime_factor):.2f}x.",
                            flush=True,
                        )
                    perf_wall_time = now_wall
                    perf_sim_time = now_sim
                target_period = max(0.0, float(self.adapter.args.time_step) / self._speed())
                remaining = target_period - (time.perf_counter() - step_start)
                if remaining > 0.0:
                    self._stop_event.wait(remaining)
                else:
                    self._stop_event.wait(0.001)
        except BaseException as exc:
            self.adapter.async_last_error = exc
            print(f"Async CoppeliaSim worker stopped: {exc}", flush=True)
            self._stop_event.set()
        finally:
            self.adapter.async_running = False


def run_dynamic_demo(client, sim, args: argparse.Namespace) -> None:
    simulator = make_coppeliasim_simulator(args)
    path_points: list[tuple[float, float]] = []
    if str(args.path_points).strip():
        for encoded_point in str(args.path_points).split(";"):
            coordinates = [part.strip() for part in encoded_point.split(",")]
            if len(coordinates) != 2:
                raise ValueError("--path-points entries must use x,z pairs separated by semicolons")
            path_points.append((float(coordinates[0]), float(coordinates[1])))
    if path_points:
        simulator.set_corner_smooth_path(path_points, corner_speed=COVERAGE_CORNER_SPEED)
        active_target = [path_points[-1][0], path_points[-1][1]]
    else:
        active_target = [float(args.target_x), float(args.target_z)]
        simulator.set_target((active_target[0], active_target[1]), planner=BEST_PLANNER)

    if args.regenerate_scene:
        gen_args = generator_args(args, simulator)
        generated_handles = scene_gen.build_scene(sim, gen_args)
        if args.save_generated_scene:
            scene_gen.save_scene(sim, generated_handles, gen_args)
    else:
        sim.loadScene(str(scene_gen.SCENE_OUTPUT))

    handles = resolve_handles(sim, int(args.cable_segments))
    sim.setObjectPosition(handles.target, -1, [active_target[0], -0.010, active_target[1]])
    sim.setObjectSel([handles.payload])
    set_time_step(sim, args.time_step)
    plant_state = initialize_dynamic_body(sim, handles, simulator, args)

    client.setStepping(True)
    if sim.getSimulationState() == sim.simulation_stopped:
        sim.startSimulation()
    path_preview = DesiredPathPreview(sim, args)
    path_segments = path_preview.maybe_update(0.0, simulator.trajectory, force=True)
    efficiency = EfficiencyMonitor()

    print("Starting dynamic CoppeliaSim wall-tool plant:")
    steel_cable = steel_cable_spec_from_args(args)
    reel_motor = reel_motor_spec_from_args(args)
    sensor_config = sensor_config_from_args(args)
    if str(args.feedback_mode) == "sensor":
        print(
            "  feedback: reel encoder + cable-angle encoder + load cell + payload IMU estimator "
            f"(reel {sensor_config.reel_encoder_counts_per_output_rev} count/rev, "
            f"angle {sensor_config.cable_angle_encoder_counts_per_rev} count/rev, "
            f"noise={bool(args.sensor_noise)})"
        )
    else:
        print("  feedback: exact CoppeliaSim position/velocity state (comparison mode)")
    print(
        f"  steel cable: diameter={1000.0 * steel_cable.diameter_m:.2f} mm, "
        f"mass/length={1000.0 * steel_cable.mass_per_length_kg_m:.1f} g/m, "
        f"segments={len(handles.cable_segments)}"
    )
    print(
        f"  reel motor: {reel_motor.voltage_v:.0f}V, {reel_motor.gear_ratio:.1f}:1, "
        f"{reel_motor.no_load_output_rpm:.0f} RPM, {reel_motor.stall_torque_kg_cm:.1f} kg.cm"
    )
    print(
        f"  reel velocity control: spool radius={1000.0 * reel_motor.spool_radius_m:.1f} mm, "
        f"max line speed={reel_motor.max_line_speed_m_s:.3f} m/s, "
        f"continuous line force={reel_motor.continuous_line_force_N:.1f} N, "
        f"tau={reel_motor.velocity_time_constant_s:.3f} s"
    )
    print(
        f"  motors: max speed={args.max_motor_speed:.0f} rad/s, "
        f"per-side thrust limit={simulator.params.max_thrust_per_drone:.3f} N"
    )
    print(
        f"  performance target: dt={float(args.time_step):.3f}s, "
        f"cable visual update={float(args.cable_visual_update_period):.3f}s, "
        f"minimum realtime factor={float(args.min_realtime_factor):.2f}x"
    )
    if bool(args.show_desired_path):
        print(
            f"  desired drawing path: visible, segments={path_segments}, "
            f"update={float(args.desired_path_update_period):.3f}s"
        )

    if args.control_ui:
        import matplotlib.pyplot as plt

        from coppeliasim_wall_tool.control_board import WallTool3DSpectatorApp

        adapter = CoppeliaSimWallToolAdapter(client, sim, handles, simulator, plant_state, args)
        app = WallTool3DSpectatorApp(adapter, planner=BEST_PLANNER)
        app.fig.suptitle("PRISMS 3D CoppeliaSim Wall Tool - Native 2D Controller UI", fontsize=14)
        backend_name = plt.get_backend()
        interactive_backend = is_interactive_matplotlib_backend(backend_name)
        print(f"2D UI Matplotlib backend: {backend_name} (interactive={interactive_backend})", flush=True)
        if float(args.duration) > 0.0:
            steps = max(1, int(float(args.duration) / max(float(args.time_step), 1e-6)))
            for index in range(steps):
                adapter.step()
                if index % max(1, int(float(args.ui_update_period) / max(float(args.time_step), 1e-6))) == 0:
                    app.draw()
                    app.fig.canvas.draw_idle()
                    if interactive_backend:
                        plt.pause(0.001)
            app.draw()
        else:
            print("Native 2D UI is controlling the 3D plant. Click the wall, append points, or draw a path.")
            if interactive_backend:
                app.draw()
                app.fig.canvas.draw_idle()
                focus_matplotlib_window(app.fig, "PRISMS 3D Control Board")

                runner = AsyncCoppeliaSimUiRunner(adapter, app) if bool(args.async_ui) else None
                if runner is not None:
                    runner.start()

                timer_interval_ms = max(15, int(1000.0 * float(args.ui_update_period)))
                ui_timer = app.fig.canvas.new_timer(interval=timer_interval_ms)

                def update_ui_from_coppeliasim() -> bool:
                    if adapter.async_last_error is not None:
                        return False
                    app.animate(None)
                    app.fig.canvas.draw_idle()
                    return True

                ui_timer.add_callback(update_ui_from_coppeliasim)
                ui_timer.start()
                app.fig._prisms_ui_timer = ui_timer
                mode = "Async" if runner is not None else "Synchronous"
                print(f"{mode} 2D control UI window is open; close it to stop the 3D run.", flush=True)
                try:
                    plt.show()
                except KeyboardInterrupt:
                    print("Interrupted by user.")
                finally:
                    try:
                        ui_timer.stop()
                    except Exception:
                        pass
                    if runner is not None:
                        runner.stop()
            else:
                raise RuntimeError(
                    "Matplotlib is using a non-interactive backend while --control-ui was requested. "
                    "Use a desktop backend, or run explicitly with --no-control-ui for batch checks."
                )
        latest = adapter.latest_state()
        print(f"Dynamic 3D wall-tool UI finished at t={latest.t:.2f}s")
        print(f"Final 3D pen tracking error [m]: {latest.tool_error:.4f}")
        print(f"Final cable tension [N]: {latest.measured_tension:.3f}")
        print(f"Ink dots drawn: {adapter.ink_dot_count()}")
        print_efficiency_summary(adapter.efficiency)
        return

    last_ink_point: tuple[float, float] | None = None
    ink_handles: list[int] = []
    max_steps = None if float(args.duration) <= 0.0 else max(1, int(args.duration / max(float(args.time_step), 1e-6)))

    last_sample: dict[str, object] | None = None
    step_index = 0
    loop_wall_start = time.perf_counter()
    try:
        while max_steps is None or step_index < max_steps:
            body_sample = read_dynamic_body_sample(sim, handles, plant_state)
            sync_controller_from_dynamic_body(sim, simulator, handles, plant_state, body_sample)
            command_state = simulator.step()
            sample = apply_dynamic_wrenches(
                sim,
                handles,
                plant_state,
                command_state,
                args,
                simulator.params,
                body_sample,
            )
            last_sample = sample
            pen_position = sample["pen_tip_position"]
            on_wall = abs(float(pen_position[1])) <= max(0.035, args.standoff * 0.40)
            in_wall_bounds = (
                abs(float(pen_position[0])) <= 0.5 * args.wall_width
                and 0.0 <= float(pen_position[2]) <= args.wall_height
            )
            measured_contact = (
                command_state.work_mode
                and simulator.params.min_contact_force_N
                <= float(sample["wall_contact_force_N"])
                <= simulator.params.max_contact_force_N
            )
            tool_error = math.hypot(
                float(pen_position[0]) - float(command_state.desired_tool_head[0]),
                float(pen_position[2]) - float(command_state.desired_tool_head[1]),
            )
            efficiency.update(
                dt=float(sample["dt"]),
                tool_error_m=tool_error,
                left_thrust_N=float(command_state.left_thrust),
                right_thrust_N=float(command_state.right_thrust),
                max_thrust_N=float(simulator.params.max_thrust_per_drone),
                tension_N=float(sample["tension"]),
                max_tension_N=float(args.max_cable_tension),
                reel_velocity_m_s=float(sample["actual_spool_velocity"]),
                motor_speed_rad_s=sample["motor_speed"],
                reel_rpm=float(sample["reel_motor_rpm"]),
                pitch_rad=float(sample["attitude"]),
                mpc_solve_time_s=float(command_state.mpc_solve_time_s),
                mpc_status=str(command_state.mpc_status),
                contact_valid=bool(measured_contact),
                work_mode=bool(command_state.work_mode),
            )
            path_preview.maybe_update(float(sample["time"]), simulator.trajectory)
            if on_wall and in_wall_bounds and measured_contact:
                last_ink_point = maybe_add_ink(
                    sim,
                    ink_handles,
                    last_ink_point,
                    float(pen_position[0]),
                    float(pen_position[2]),
                    args,
                )
            if float(sample["time"]) - plant_state.last_log_time >= float(args.log_period):
                omega = sample["motor_speed"]
                rpm = [float(value) * 60.0 / (2.0 * math.pi) for value in omega]
                pos = sample["position"]
                reel_rpm = float(sample["reel_motor_rpm"])
                estimator_log = ""
                if plant_state.last_estimate is not None:
                    estimate = plant_state.last_estimate
                    estimate_error = math.hypot(
                        estimate.payload_position_xz_m[0] - float(pos[0]),
                        estimate.payload_position_xz_m[1] - float(pos[2]),
                    )
                    estimator_log = f" est_err={1000.0 * estimate_error:.1f}mm"
                mpc_status = str(command_state.mpc_status)
                if "safety fallback active" in mpc_status:
                    mpc_log = "fallback"
                elif mpc_status.startswith("Feasible_Limited"):
                    mpc_log = "feasible-limited"
                else:
                    mpc_log = mpc_status.split(":", 1)[0] or "unknown"
                print(
                    f"t={float(sample['time']):5.2f}s "
                    f"xz=[{float(pos[0]): .2f},{float(pos[2]): .2f}]m "
                    f"pitch={math.degrees(float(sample['attitude'])):+.2f}deg "
                    f"T={float(sample['tension']):.2f}N "
                    f"rpm=[{rpm[0]:.0f},{rpm[1]:.0f}] "
                    f"reel={reel_rpm:+.0f}rpm{estimator_log} mpc={mpc_log}"
                )
                plant_state.last_log_time = float(sample["time"])
            client.step()
            step_index += 1
    except KeyboardInterrupt:
        print("Interrupted by user.")

    if last_sample is None:
        return
    final_position = last_sample["position"]
    final_error = math.hypot(float(final_position[0]) - active_target[0], float(final_position[2]) - active_target[1])
    loop_wall_elapsed = max(time.perf_counter() - loop_wall_start, 1e-9)
    simulated_elapsed = float(step_index) * float(args.time_step)
    realtime_factor = simulated_elapsed / loop_wall_elapsed
    print(f"Dynamic 3D wall-tool demo finished at t={float(last_sample['time']):.2f}s")
    print(f"Final payload-target error [m]: {final_error:.4f}")
    print(f"Final cable tension [N]: {float(last_sample['tension']):.3f}")
    print(f"Ink dots drawn: {len(ink_handles)}")
    print_efficiency_summary(efficiency)
    print(
        f"Realtime factor: {realtime_factor:.2f}x "
        f"(sim {simulated_elapsed:.2f}s / wall {loop_wall_elapsed:.2f}s)"
    )
    if float(args.min_realtime_factor) > 0.0 and realtime_factor < float(args.min_realtime_factor):
        raise RuntimeError(
            f"CoppeliaSim realtime factor {realtime_factor:.2f}x is below "
            f"--min-realtime-factor {float(args.min_realtime_factor):.2f}x"
        )


def main() -> int:
    args = parse_args()
    validate_physical_args(args)
    client, sim = sim_utils.connect_or_launch_client(
        args.host,
        args.port,
        args.connect_timeout,
        launch=args.launch_coppeliasim,
        exe_path=args.coppeliasim_exe,
    )
    try:
        run_dynamic_demo(client, sim, args)
    finally:
        client.setStepping(False)
        if args.stop_simulation_on_exit and sim.getSimulationState() != sim.simulation_stopped:
            sim.stopSimulation(True)
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
