#!/usr/bin/env python3
"""Run the wall-tool controller as a 3D CoppeliaSim pen-on-wall demo.

The default mode is a force-driven CoppeliaSim plant: the payload is a dynamic
body, the two side motors apply forces and torques, the reel enforces a taut
unilateral cable, and propeller joints spin from motor angular speed. A mirror
mode is kept for quick visual comparison with the original 2D model.
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
    try:
        import matplotlib

        os.environ["MPLBACKEND"] = "TkAgg"
        matplotlib.use("TkAgg", force=True)
    except Exception as exc:  # pragma: no cover - depends on local GUI stack.
        print(f"Warning: could not force Matplotlib TkAgg backend: {exc}", flush=True)


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

from cable_hybrid_controller.controller import BEST_PLANNER, make_simulator  # noqa: E402
from coppeliasim_wall_tool import generate_wall_tool_scene as scene_gen  # noqa: E402
from coppeliasim_wall_tool import sim_utils  # noqa: E402
from wall_tool_sim.wall_tool_ui import integrated_motor_center_offsets  # noqa: E402


PLANT_MODES = ("dynamic", "mirror")


def parse_args() -> argparse.Namespace:
    params = make_simulator().params
    parser = argparse.ArgumentParser(description="Run the PRISMS wall-tool CoppeliaSim draw-on-wall demo.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--launch-coppeliasim", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--coppeliasim-exe", type=Path, default=sim_utils.DEFAULT_COPPELIASIM_EXE)
    parser.add_argument("--duration", type=float, default=0.0, help="Simulation duration [s]. Default 0 keeps the 2D UI running.")
    parser.add_argument("--plant-mode", choices=PLANT_MODES, default="dynamic")
    parser.add_argument("--target-x", type=float, default=0.90)
    parser.add_argument("--target-z", type=float, default=1.50)
    parser.add_argument("--standoff", type=float, default=params.normal_standoff_m)
    parser.add_argument("--wall-width", type=float, default=params.wall_width)
    parser.add_argument("--wall-height", type=float, default=params.wall_height)
    parser.add_argument("--wall-thickness", type=float, default=0.050)
    parser.add_argument("--update-period", type=float, default=0.020)
    parser.add_argument("--time-step", type=float, default=params.dt)
    parser.add_argument("--max-motor-speed", type=float, default=900.0)
    parser.add_argument("--motor-tau-up", type=float, default=0.050)
    parser.add_argument("--motor-tau-down", type=float, default=0.080)
    parser.add_argument("--cable-stiffness", type=float, default=max(2500.0, params.cable_stiffness_N_m * 3.0))
    parser.add_argument("--cable-damping", type=float, default=max(12.0, params.cable_damping_N_s_m * 8.0))
    parser.add_argument("--cable-taut-band", type=float, default=0.002)
    parser.add_argument("--max-cable-tension", type=float, default=params.max_spool_tension)
    parser.add_argument("--desired-tension-feedforward", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--linear-drag-xz", type=float, default=0.020)
    parser.add_argument("--normal-standoff-kp", type=float, default=42.0)
    parser.add_argument("--normal-standoff-kd", type=float, default=2.2)
    parser.add_argument("--angular-drag-y", type=float, default=params.rotational_damping)
    parser.add_argument("--angular-drag-roll-yaw", type=float, default=0.004)
    parser.add_argument("--prop-visual-update-period", type=float, default=0.020)
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
        payload_x=params.initial_payload[0],
        payload_z=params.initial_payload[1],
        body_depth=0.140,
        motor_depth=0.060,
        cable_radius=0.006,
        pen_radius=0.009,
        save_model=bool(args.save_generated_scene),
        clear_existing=True,
    )


@dataclass
class SceneHandles:
    payload: int
    cable: int
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
    cable_visual_length: float = 1.0
    left_motor_speed: float = 0.0
    right_motor_speed: float = 0.0
    left_prop_phase: float = 0.0
    right_prop_phase: float = 0.0
    left_force_arrow_stem_length: float = scene_gen.FORCE_ARROW_INITIAL_LENGTH - scene_gen.FORCE_ARROW_HEAD_LENGTH
    right_force_arrow_stem_length: float = scene_gen.FORCE_ARROW_INITIAL_LENGTH - scene_gen.FORCE_ARROW_HEAD_LENGTH
    last_time: float | None = None
    last_prop_visual_time: float = -1.0
    last_log_time: float = -1.0
    last_tension: float = 0.0


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


def resolve_handles(sim) -> SceneHandles:
    return SceneHandles(
        payload=get_handle(sim, scene_gen.PAYLOAD_ALIAS),
        cable=get_handle(sim, scene_gen.CABLE_ALIAS),
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
    anchor = sim.getObjectPosition(handles.anchor, -1)
    cable_length = max(1e-6, norm3(sub3(mount, anchor)))
    plant_state = DynamicPlantState(
        reel_length=clamp(cable_length, params.min_cable_length, params.max_cable_length),
        cable_visual_length=cable_length,
    )
    plant_state.cable_visual_length = sim_utils.update_cylinder_between(
        sim,
        handles.cable,
        anchor,
        mount,
        plant_state.cable_visual_length,
    )
    return plant_state


def sync_controller_from_dynamic_body(
    sim,
    simulator,
    handles: SceneHandles,
    plant_state: DynamicPlantState,
) -> None:
    params = simulator.params
    position = sim.getObjectPosition(handles.payload, -1)
    orientation = sim.getObjectOrientation(handles.payload, -1)
    linear_velocity, angular_velocity = sim.getVelocity(handles.payload)
    xz_position = (float(position[0]), float(position[2]))
    xz_velocity = (float(linear_velocity[0]), float(linear_velocity[2]))

    simulator.position = xz_position
    simulator.velocity = xz_velocity
    simulator.attitude = coppelia_pitch_to_planar_attitude(float(orientation[1]))
    simulator.angular_velocity = -float(angular_velocity[1])
    if plant_state.reel_length is not None:
        simulator.cable_length = clamp(plant_state.reel_length, params.min_cable_length, params.max_cable_length)
        simulator.measured_cable_length = simulator.cable_length
    simulator.actual_tension = clamp(plant_state.last_tension, 0.0, params.max_spool_tension)
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
) -> dict[str, object]:
    now = float(sim.getSimulationTime())
    if plant_state.last_time is None:
        dt = max(1e-4, float(args.time_step))
    else:
        dt = clamp(now - plant_state.last_time, 1e-4, 0.05)
    plant_state.last_time = now

    matrix = list(sim.getObjectMatrix(handles.payload, -1))
    position = [float(matrix[3]), float(matrix[7]), float(matrix[11])]
    attitude = coppelia_pitch_to_planar_attitude(float(sim.getObjectOrientation(handles.payload, -1)[1]))
    linear_velocity, angular_velocity = sim.getVelocity(handles.payload)
    linear_velocity = [float(value) for value in linear_velocity]
    angular_velocity = [float(value) for value in angular_velocity]

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

    anchor = sim.getObjectPosition(handles.anchor, -1)
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
    plant_state.reel_length = clamp(
        plant_state.reel_length + float(command_state.spool_velocity_cmd) * dt,
        params.min_cable_length,
        params.max_cable_length,
    )
    actual_spool_velocity = (plant_state.reel_length - previous_reel_length) / max(dt, 1e-9)
    length_rate = dot3(cable_out, mount_velocity)
    extension = cable_distance - plant_state.reel_length
    extension_rate = length_rate - actual_spool_velocity
    raw_tension = (
        float(args.cable_stiffness) * max(0.0, extension)
        + float(args.cable_damping) * max(0.0, extension_rate)
    )
    taut = extension >= -abs(float(args.cable_taut_band))
    if taut and bool(args.desired_tension_feedforward):
        raw_tension = max(raw_tension, float(command_state.desired_cable_tension))
    tension = clamp(raw_tension if taut else 0.0, 0.0, float(args.max_cable_tension))
    plant_state.last_tension = tension
    cable_force_world = scale3(cable_to_anchor, tension)
    cable_arm = sub3(mount, position)
    cable_torque_world = cross3(cable_arm, cable_force_world)
    add_wrench_at_point(net_force, net_torque, cable_force_world, mount, position)

    desired_y = -abs(float(args.standoff))
    net_force[1] += float(args.normal_standoff_kp) * (desired_y - position[1])
    net_force[1] -= float(args.normal_standoff_kd) * linear_velocity[1]
    net_force[0] -= float(args.linear_drag_xz) * linear_velocity[0]
    net_force[2] -= float(args.linear_drag_xz) * linear_velocity[2]
    net_torque[0] -= float(args.angular_drag_roll_yaw) * angular_velocity[0]
    net_torque[1] -= float(args.angular_drag_y) * angular_velocity[1]
    net_torque[2] -= float(args.angular_drag_roll_yaw) * angular_velocity[2]

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

    plant_state.cable_visual_length = sim_utils.update_cylinder_between(
        sim,
        handles.cable,
        anchor,
        mount,
        plant_state.cable_visual_length,
    )
    sim.addForceAndTorque(handles.payload, net_force, net_torque)
    return {
        "time": now,
        "dt": dt,
        "position": position,
        "attitude": attitude,
        "motor_thrust": [left_thrust, right_thrust],
        "motor_speed": [plant_state.left_motor_speed, plant_state.right_motor_speed],
        "cable_mount": mount,
        "cable_distance": cable_distance,
        "cable_extension": extension,
        "cable_slack": not taut or tension <= 1e-9,
        "reel_length": plant_state.reel_length,
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
        self._last_theta: float | None = None
        self._last_length: float | None = None
        self._last_length_dot = 0.0
        self._last_velocity: tuple[float, float] | None = None
        self._last_angular_velocity: float | None = None
        self._last_time: float | None = None
        self.history = [self._sensor_state_from_3d(simulator.history[-1], None)]

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
            sync_controller_from_dynamic_body(self.sim, self.controller, self.handles, self.plant_state)
            command_state = self.controller.step()
            sample = apply_dynamic_wrenches(
                self.sim,
                self.handles,
                self.plant_state,
                command_state,
                self.args,
                self.params,
            )
            self.client.step()
            sensor_state = self._sensor_state_from_3d(command_state, sample)
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

    def clear_trajectory(self) -> None:
        self._run_or_queue("clear_trajectory")

    def _do_clear_trajectory(self) -> None:
        self._sync_controller_state()
        self.controller.clear_trajectory()

    def clear_trace(self) -> None:
        self._run_or_queue("clear_trace")

    def _do_clear_trace(self) -> None:
        self._sync_controller_state()
        self.controller.clear_trajectory()
        self.history = self.history[-1:]

    def set_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._run_or_queue("set_target", point, planner=planner)

    def _do_set_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._sync_controller_state()
        self.controller.set_target(point, planner=planner)
        self._move_target_marker(point)

    def append_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._run_or_queue("append_target", point, planner=planner)

    def _do_append_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._sync_controller_state()
        self.controller.append_target(point, planner=planner)
        self._move_target_marker(point)

    def append_stop_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._run_or_queue("append_stop_target", point, planner=planner)

    def _do_append_stop_target(self, point: tuple[float, float], planner: str = BEST_PLANNER) -> None:
        self._sync_controller_state()
        self.controller.append_stop_target(point, planner=planner)
        self._move_target_marker(point)

    def set_smooth_path(self, points: Sequence[tuple[float, float]]) -> None:
        self._run_or_queue("set_smooth_path", tuple(points))

    def _do_set_smooth_path(self, points: Sequence[tuple[float, float]]) -> None:
        self._sync_controller_state()
        self.controller.set_smooth_path(points)
        if points:
            self._move_target_marker(points[-1])

    def set_corner_smooth_path(self, points: Sequence[tuple[float, float]], corner_speed: float) -> None:
        self._run_or_queue("set_corner_smooth_path", tuple(points), corner_speed)

    def _do_set_corner_smooth_path(self, points: Sequence[tuple[float, float]], corner_speed: float) -> None:
        self._sync_controller_state()
        self.controller.set_corner_smooth_path(points, corner_speed)
        if points:
            self._move_target_marker(points[-1])

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

        contact_valid = bool(base_state.contact_valid and abs(float(pen_position[1])) <= max(0.040, self.args.standoff * 0.45))
        tool_head = (float(pen_position[0]), float(pen_position[2]))
        desired_tool = base_state.desired_tool_head
        tool_error = math.hypot(tool_head[0] - desired_tool[0], tool_head[1] - desired_tool[1])

        self._last_theta = theta
        self._last_length = line_length
        self._last_length_dot = length_dot
        self._last_velocity = payload_velocity
        self._last_angular_velocity = angular_velocity_y
        self._last_time = now

        motor_speed = sample["motor_speed"] if sample is not None else (0.0, 0.0)
        rpm_l = float(motor_speed[0]) * 60.0 / (2.0 * math.pi)
        rpm_r = float(motor_speed[1]) * 60.0 / (2.0 * math.pi)
        self.sensor_text = (
            f"3D sensors line {line_length:4.2f}m  reel {reel_length:4.2f}m  stretch {1000.0 * cable_extension:5.1f}mm\n"
            f"3D pose y {position[1]:+.3f}m  pen_y {float(pen_position[1]):+.3f}m  slack {cable_slack}\n"
            f"3D motors rpm L/R {rpm_l:5.0f}/{rpm_r:5.0f}"
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
        if not in_wall_bounds or not (command_state.contact_valid or command_state.work_mode):
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
            while not self._stop_event.is_set():
                self.adapter.process_pending_commands()
                if not self._playing():
                    self._stop_event.wait(0.020)
                    continue

                step_start = time.perf_counter()
                self.adapter.step()
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


def run_mirror_demo(sim, args: argparse.Namespace) -> None:
    simulator = make_simulator()
    simulator.set_target((args.target_x, args.target_z), planner=BEST_PLANNER)

    if args.regenerate_scene:
        gen_args = generator_args(args, simulator)
        generated_handles = scene_gen.build_scene(sim, gen_args)
        if args.save_generated_scene:
            scene_gen.save_scene(sim, generated_handles, gen_args)
    else:
        sim.loadScene(str(scene_gen.SCENE_OUTPUT))

    payload = get_handle(sim, scene_gen.PAYLOAD_ALIAS)
    cable = get_handle(sim, scene_gen.CABLE_ALIAS)
    target = get_handle(sim, scene_gen.TARGET_ALIAS)

    sim.setObjectPosition(target, -1, [args.target_x, -0.010, args.target_z])
    sim.setObjectSel([payload])
    if sim.getSimulationState() == sim.simulation_stopped:
        sim.startSimulation()

    cable_length = 1.0
    last_update_t = -1.0
    last_ink_point: tuple[float, float] | None = None
    ink_handles: list[int] = []
    steps = max(1, int(args.duration / simulator.params.dt))
    wall_start = time.perf_counter()

    for _ in range(steps):
        state = simulator.step()
        if state.t - last_update_t < args.update_period and state.t < args.duration:
            continue
        last_update_t = state.t

        update_payload_pose(sim, payload, state.payload[0], state.payload[1], args.standoff, state.attitude)
        cable_mount = simulator._cable_mount_position(state.payload, state.attitude)
        cable_start = [0.0, -args.standoff, args.wall_height]
        cable_end = [cable_mount[0], -args.standoff, cable_mount[1]]
        cable_length = sim_utils.update_cylinder_between(sim, cable, cable_start, cable_end, cable_length)
        if state.contact_valid or state.work_mode:
            last_ink_point = maybe_add_ink(sim, ink_handles, last_ink_point, state.tool_head[0], state.tool_head[1], args)
        time.sleep(max(0.0, args.update_period - (time.perf_counter() - wall_start - state.t)))

    print(f"3D wall-tool demo finished at t={simulator.history[-1].t:.2f}s")
    print(f"Final tracking error [m]: {simulator.history[-1].tool_error:.4f}")
    print(f"Ink dots drawn: {len(ink_handles)}")


def run_dynamic_demo(client, sim, args: argparse.Namespace) -> None:
    simulator = make_simulator()
    active_target = [float(args.target_x), float(args.target_z)]
    simulator.set_target((active_target[0], active_target[1]), planner=BEST_PLANNER)

    if args.regenerate_scene:
        gen_args = generator_args(args, simulator)
        generated_handles = scene_gen.build_scene(sim, gen_args)
        if args.save_generated_scene:
            scene_gen.save_scene(sim, generated_handles, gen_args)
    else:
        sim.loadScene(str(scene_gen.SCENE_OUTPUT))

    handles = resolve_handles(sim)
    sim.setObjectPosition(handles.target, -1, [active_target[0], -0.010, active_target[1]])
    sim.setObjectSel([handles.payload])
    set_time_step(sim, args.time_step)
    plant_state = initialize_dynamic_body(sim, handles, simulator, args)

    client.setStepping(True)
    if sim.getSimulationState() == sim.simulation_stopped:
        sim.startSimulation()

    print("Starting dynamic CoppeliaSim wall-tool plant:")
    print(
        f"  cable: unilateral taut length, stiffness={args.cable_stiffness:.1f} N/m, "
        f"damping={args.cable_damping:.1f} N*s/m"
    )
    print(
        f"  motors: max speed={args.max_motor_speed:.0f} rad/s, "
        f"per-side thrust limit={simulator.params.max_thrust_per_drone:.3f} N"
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
                print(
                    "Matplotlib is using a non-interactive backend, so the UI cannot stay open. "
                    "Use a desktop backend or run with --duration > 0 for a timed check."
                )
        latest = adapter.latest_state()
        print(f"Dynamic 3D wall-tool UI finished at t={latest.t:.2f}s")
        print(f"Final 3D pen tracking error [m]: {latest.tool_error:.4f}")
        print(f"Final cable tension [N]: {latest.measured_tension:.3f}")
        print(f"Ink dots drawn: {adapter.ink_dot_count()}")
        return

    last_ink_point: tuple[float, float] | None = None
    ink_handles: list[int] = []
    max_steps = None if float(args.duration) <= 0.0 else max(1, int(args.duration / max(float(args.time_step), 1e-6)))

    last_sample: dict[str, object] | None = None
    step_index = 0
    try:
        while max_steps is None or step_index < max_steps:
            sync_controller_from_dynamic_body(sim, simulator, handles, plant_state)
            command_state = simulator.step()
            sample = apply_dynamic_wrenches(sim, handles, plant_state, command_state, args, simulator.params)
            last_sample = sample
            pen_position = sim.getObjectPosition(handles.pen_tip, -1)
            on_wall = abs(float(pen_position[1])) <= max(0.035, args.standoff * 0.40)
            in_wall_bounds = (
                abs(float(pen_position[0])) <= 0.5 * args.wall_width
                and 0.0 <= float(pen_position[2]) <= args.wall_height
            )
            if on_wall and in_wall_bounds and (command_state.contact_valid or command_state.work_mode):
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
                print(
                    f"t={float(sample['time']):5.2f}s "
                    f"xz=[{float(pos[0]): .2f},{float(pos[2]): .2f}]m "
                    f"T={float(sample['tension']):.2f}N "
                    f"rpm=[{rpm[0]:.0f},{rpm[1]:.0f}]"
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
    print(f"Dynamic 3D wall-tool demo finished at t={float(last_sample['time']):.2f}s")
    print(f"Final payload-target error [m]: {final_error:.4f}")
    print(f"Final cable tension [N]: {float(last_sample['tension']):.3f}")
    print(f"Ink dots drawn: {len(ink_handles)}")


def main() -> int:
    args = parse_args()
    client, sim = sim_utils.connect_or_launch_client(
        args.host,
        args.port,
        args.connect_timeout,
        launch=args.launch_coppeliasim,
        exe_path=args.coppeliasim_exe,
    )
    try:
        if args.plant_mode == "dynamic":
            run_dynamic_demo(client, sim, args)
        else:
            run_mirror_demo(sim, args)
    finally:
        if args.plant_mode == "dynamic":
            try:
                client.setStepping(False)
            except Exception:
                pass
        if args.stop_simulation_on_exit and sim.getSimulationState() != sim.simulation_stopped:
            sim.stopSimulation(True)
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
