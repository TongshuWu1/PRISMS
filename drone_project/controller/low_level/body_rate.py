#!/usr/bin/env python3
"""Run an external Python thrust/body-rate controller for the CoppeliaSim drone.

This is the low-level interface used by many multirotor stacks:

    command input  -> collective thrust T [N] and body rates p,q,r [rad/s]
    controller out -> four motor angular velocities omega_i [rad/s]
    plant model    -> f_i = k_f omega_i^2 and tau_i = k_m omega_i^2

The standalone script does not use absolute position. A real fixed-position
hover uses an upstream estimator and position loop; this module exposes the
same rate-command boundary through `controller.low_level.rate_control`.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.common import telemetry  # noqa: E402
from controller.low_level import rate_control  # noqa: E402

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


MODEL_ALIAS = "truncated_octahedral_crazyflie"
SCENE_PATH = PROJECT_ROOT / "scene" / "body_rate_controller_demo_scene.ttt"

G = rate_control.G
MOTORS = rate_control.MOTORS
DEFAULT_MASS = rate_control.DEFAULT_MASS_KG
DEFAULT_MAX_MOTOR_SPEED = rate_control.DEFAULT_MAX_MOTOR_SPEED
DEFAULT_MAX_THRUST = rate_control.DEFAULT_MAX_THRUST_N
DEFAULT_YAW_DRAG_ARM = rate_control.DEFAULT_YAW_DRAG_ARM_M
DEFAULT_GEOMETRY_SCALE = rate_control.DEFAULT_GEOMETRY_SCALE
DEFAULT_MODULE_INERTIA_BOX = rate_control.DEFAULT_MODULE_INERTIA_BOX_M
DEFAULT_BODY_COLLISION_BOX = rate_control.DEFAULT_BODY_COLLISION_BOX_M
DEFAULT_PROP_RADIUS = rate_control.DEFAULT_PROP_RADIUS_M
DEFAULT_COLLISION_ROD_DIAMETER = rate_control.DEFAULT_COLLISION_ROD_DIAMETER_M
DEFAULT_COLLISION_NODE_DIAMETER = rate_control.DEFAULT_COLLISION_NODE_DIAMETER_M
PROPELLER_ARROW_MIN_LENGTH = 0.006
PROPELLER_ARROW_MAX_LENGTH = 0.034
PROPELLER_ARROW_RADIUS = 0.0008
PROPELLER_ARROW_HEAD_LENGTH = 0.0045
PROPELLER_ARROW_BASE_OFFSET = 0.004
PROPELLER_ARROW_COLOR = (1.0, 0.58, 0.02)


@dataclass
class ControllerState:
    rate_integral: list[float]
    motor_speed: list[float]
    prop_phase: list[float]
    last_time: float | None = None
    last_prop_visual_time: float | None = None
    prop_force_arrow_handles: list[tuple[int, int]] = field(default_factory=list)
    prop_force_arrow_lengths: list[float] = field(default_factory=list)


def add_propeller_visual_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--prop-force-arrows", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prop-visual-update-period", type=float, default=0.033)
    parser.add_argument("--prop-force-arrow-min-length", type=float, default=PROPELLER_ARROW_MIN_LENGTH)
    parser.add_argument("--prop-force-arrow-max-length", type=float, default=PROPELLER_ARROW_MAX_LENGTH)
    parser.add_argument("--prop-force-arrow-radius", type=float, default=PROPELLER_ARROW_RADIUS)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a Python thrust/body-rate controller in CoppeliaSim.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--scene", default=str(SCENE_PATH))
    parser.add_argument("--load-scene", action="store_true", help="Load the generated scene before running.")

    parser.add_argument("--duration", type=float, default=12.0, help="Simulation duration [s]. Use 0 for Ctrl+C.")
    parser.add_argument("--time-step", type=float, default=0.005, help="Simulation/controller step [s].")
    parser.add_argument("--start-height", type=float, default=0.5)
    parser.add_argument("--reset-state", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--mass", type=float, default=DEFAULT_MASS, help="Total mass [kg].")
    parser.add_argument("--max-motor-speed", type=float, default=DEFAULT_MAX_MOTOR_SPEED, help="Motor speed limit [rad/s].")
    parser.add_argument("--max-thrust", type=float, default=DEFAULT_MAX_THRUST, help="Per-motor thrust at max speed [N].")
    parser.add_argument("--yaw-drag-arm", type=float, default=DEFAULT_YAW_DRAG_ARM, help="Yaw torque per motor thrust [m].")

    parser.add_argument("--thrust", type=float, default=None, help="Collective thrust command [N]. Default is mass*g.")
    parser.add_argument("--p-cmd", type=float, default=0.0, help="Desired roll body rate p [rad/s].")
    parser.add_argument("--q-cmd", type=float, default=0.0, help="Desired pitch body rate q [rad/s].")
    parser.add_argument("--r-cmd", type=float, default=0.0, help="Desired yaw body rate r [rad/s].")
    parser.add_argument("--takeoff-pulse", action="store_true", help="Use a short open-loop thrust pulse, then mass*g.")
    parser.add_argument("--pulse-scale", type=float, default=1.15, help="Pulse thrust as a multiple of mass*g.")
    parser.add_argument("--pulse-duration", type=float, default=0.30, help="Pulse duration [s].")

    parser.add_argument("--kp-rate-rp", type=float, default=0.010, help="Roll/pitch rate P gain [N*m/(rad/s)].")
    parser.add_argument("--kp-rate-yaw", type=float, default=0.0022, help="Yaw rate P gain [N*m/(rad/s)].")
    parser.add_argument("--ki-rate-rp", type=float, default=0.0010, help="Roll/pitch rate I gain [N*m/rad].")
    parser.add_argument("--ki-rate-yaw", type=float, default=0.00025, help="Yaw rate I gain [N*m/rad].")
    parser.add_argument("--integral-limit-rp", type=float, default=0.20, help="Roll/pitch rate integrator limit [rad].")
    parser.add_argument("--integral-limit-yaw", type=float, default=0.30, help="Yaw rate integrator limit [rad].")

    parser.add_argument("--motor-tau-up", type=float, default=0.050, help="Motor spin-up time constant [s].")
    parser.add_argument("--motor-tau-down", type=float, default=0.080, help="Motor spin-down time constant [s].")
    parser.add_argument("--linear-drag-xy", type=float, default=0.018, help="Body drag coefficient in world x/y.")
    parser.add_argument("--linear-drag-z", type=float, default=0.006, help="Body drag coefficient in world z.")
    parser.add_argument("--angular-drag-rp", type=float, default=0.00055, help="Roll/pitch angular drag [N*m/(rad/s)].")
    parser.add_argument("--angular-drag-yaw", type=float, default=0.00020, help="Yaw angular drag [N*m/(rad/s)].")
    add_propeller_visual_args(parser)
    parser.add_argument("--log-period", type=float, default=0.25)
    telemetry.add_logging_args(parser)
    return parser.parse_args()


clamp = rate_control.clamp


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
    return client, client.require("sim")


def stop_if_running(sim) -> None:
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation(True)
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)


motor_mixer = rate_control.motor_mixer


def get_body_handles(sim) -> tuple[int, list[int]]:
    body = sim.getObject(f"/{MODEL_ALIAS}")
    joints = []
    for index in range(4):
        joints.append(
            sim.getObject(f"/{MODEL_ALIAS}/propeller_{index}_root/propeller_{index}_spin_joint")
        )
    return body, joints


def reset_body(sim, body: int, start_height: float) -> None:
    sim.setObjectPosition(body, -1, [0.0, 0.0, start_height])
    sim.setObjectOrientation(body, -1, [0.0, 0.0, 0.0])
    sim.resetDynamicObject(body)


def set_time_step(sim, time_step: float) -> None:
    sim.setFloatParam(sim.floatparam_simulation_time_step, time_step)


def propeller_arrow_length(thrust: float, args: argparse.Namespace) -> float:
    min_length = max(0.0, float(args.prop_force_arrow_min_length))
    max_length = max(min_length, float(args.prop_force_arrow_max_length))
    max_thrust = max(1e-9, float(args.max_thrust))
    thrust_ratio = clamp(thrust / max_thrust, 0.0, 1.0)
    return min_length + thrust_ratio * (max_length - min_length)


def configure_visual_shape(sim, handle: int, color: tuple[float, float, float]) -> None:
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, list(color))
    sim.setShapeColor(handle, None, sim.colorcomponent_emission, [0.10, 0.06, 0.00])
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)


def object_alias_tail(sim, handle: int) -> str:
    return str(sim.getObjectAlias(handle, 1)).rsplit("/", 1)[-1]


def alias_matches_base(tail: str, base: str) -> bool:
    return tail == base or tail.startswith(f"{base}#") or tail.startswith(f"{base}[")


def remove_existing_propeller_visuals(sim, joint: int, index: int) -> None:
    bases = (
        f"propeller_{index}_force_arrow_stem",
        f"propeller_{index}_force_arrow_head",
        f"propeller_{index}_rpm_blur",
    )
    descendants = sim.getObjectsInTree(joint, sim.handle_all, 0)
    to_remove = []
    for handle in descendants:
        tail = object_alias_tail(sim, handle)
        if any(alias_matches_base(tail, base) for base in bases):
            to_remove.append(handle)
    if to_remove:
        sim.removeObjects(to_remove, False)


def create_propeller_force_arrow(sim, joint: int, index: int, args: argparse.Namespace) -> tuple[int, int]:
    remove_existing_propeller_visuals(sim, joint, index)
    radius = max(0.0002, float(args.prop_force_arrow_radius))
    min_length = max(0.001, float(args.prop_force_arrow_min_length))
    stem_length = max(0.001, min_length - PROPELLER_ARROW_HEAD_LENGTH)

    stem = sim.createPrimitiveShape(
        sim.primitiveshape_cylinder,
        [2.0 * radius, 2.0 * radius, stem_length],
        0,
    )
    sim.setObjectAlias(stem, f"propeller_{index}_force_arrow_stem", 1)
    sim.setObjectParent(stem, joint, False)
    sim.setObjectPosition(stem, joint, [0.0, 0.0, PROPELLER_ARROW_BASE_OFFSET + 0.5 * stem_length])
    sim.setObjectOrientation(stem, joint, [0.0, 0.0, 0.0])
    configure_visual_shape(sim, stem, PROPELLER_ARROW_COLOR)

    head = sim.createPrimitiveShape(
        sim.primitiveshape_cone,
        [4.0 * radius, 4.0 * radius, PROPELLER_ARROW_HEAD_LENGTH],
        0,
    )
    sim.setObjectAlias(head, f"propeller_{index}_force_arrow_head", 1)
    sim.setObjectParent(head, joint, False)
    sim.setObjectPosition(
        head,
        joint,
        [0.0, 0.0, PROPELLER_ARROW_BASE_OFFSET + stem_length + 0.5 * PROPELLER_ARROW_HEAD_LENGTH],
    )
    sim.setObjectOrientation(head, joint, [0.0, 0.0, 0.0])
    configure_visual_shape(sim, head, PROPELLER_ARROW_COLOR)
    return int(stem), int(head)


def ensure_propeller_force_arrows(
    sim,
    joints: list[int],
    state: ControllerState,
    args: argparse.Namespace,
) -> list[tuple[int, int]]:
    if not bool(args.prop_force_arrows):
        return []
    if len(state.prop_force_arrow_handles) == len(joints):
        return state.prop_force_arrow_handles
    state.prop_force_arrow_handles = [
        create_propeller_force_arrow(sim, joint, index, args)
        for index, joint in enumerate(joints)
    ]
    min_length = max(0.001, float(args.prop_force_arrow_min_length))
    stem_length = max(0.001, min_length - PROPELLER_ARROW_HEAD_LENGTH)
    state.prop_force_arrow_lengths = [stem_length for _ in state.prop_force_arrow_handles]
    return state.prop_force_arrow_handles


def set_arrow_visibility(sim, arrow_handles: list[tuple[int, int]], visible: bool) -> None:
    layer = 1 if visible else 0
    for stem, head in arrow_handles:
        for handle in (stem, head):
            sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, layer)


def update_propeller_visuals(
    sim,
    joints: list[int],
    state: ControllerState,
    motor_thrust: list[float],
    args: argparse.Namespace,
) -> None:
    if not bool(args.prop_force_arrows):
        set_arrow_visibility(sim, state.prop_force_arrow_handles, False)
        return
    arrows = ensure_propeller_force_arrows(sim, joints, state, args)
    set_arrow_visibility(sim, arrows, True)
    for index, (stem, head) in enumerate(arrows[:4]):
        total_length = propeller_arrow_length(motor_thrust[index], args)
        stem_length = max(0.001, total_length - PROPELLER_ARROW_HEAD_LENGTH)
        old_length = state.prop_force_arrow_lengths[index]
        if abs(stem_length - old_length) > 1e-5:
            sim.scaleObject(stem, 1.0, 1.0, stem_length / old_length, 0)
            state.prop_force_arrow_lengths[index] = stem_length
        sim.setObjectPosition(stem, joints[index], [0.0, 0.0, PROPELLER_ARROW_BASE_OFFSET + 0.5 * stem_length])
        sim.setObjectPosition(
            head,
            joints[index],
            [0.0, 0.0, PROPELLER_ARROW_BASE_OFFSET + stem_length + 0.5 * PROPELLER_ARROW_HEAD_LENGTH],
        )


def update_propeller_render_if_due(
    sim,
    joints: list[int],
    state: ControllerState,
    motor_thrust: list[float],
    args: argparse.Namespace,
    now: float,
) -> None:
    period = max(float(args.prop_visual_update_period), float(args.time_step))
    if state.last_prop_visual_time is not None and now - state.last_prop_visual_time < period:
        return
    state.last_prop_visual_time = now
    for i in range(4):
        sim.setJointPosition(joints[i], state.prop_phase[i])
    update_propeller_visuals(sim, joints, state, motor_thrust, args)


def resolve_scene(sim, args: argparse.Namespace) -> tuple[int, list[int]]:
    stop_if_running(sim)
    if args.load_scene:
        scene_path = Path(args.scene)
        if not scene_path.exists():
            raise FileNotFoundError(scene_path)
        sim.loadScene(str(scene_path))
    return get_body_handles(sim)


def body_axes_from_matrix(matrix: list[float]) -> tuple[list[float], list[float], list[float]]:
    body_x = [matrix[0], matrix[4], matrix[8]]
    body_y = [matrix[1], matrix[5], matrix[9]]
    body_z = [matrix[2], matrix[6], matrix[10]]
    return body_x, body_y, body_z


def world_to_body_rate(matrix: list[float], angular_velocity_world: list[float]) -> list[float]:
    body_x, body_y, body_z = body_axes_from_matrix(matrix)
    return [
        sum(body_x[i] * angular_velocity_world[i] for i in range(3)),
        sum(body_y[i] * angular_velocity_world[i] for i in range(3)),
        sum(body_z[i] * angular_velocity_world[i] for i in range(3)),
    ]


def body_torque_to_world(matrix: list[float], torque_body: list[float]) -> list[float]:
    body_x, body_y, body_z = body_axes_from_matrix(matrix)
    return [
        body_x[i] * torque_body[0] + body_y[i] * torque_body[1] + body_z[i] * torque_body[2]
        for i in range(3)
    ]


def collective_thrust_command(args: argparse.Namespace, sim_time: float) -> float:
    hover_thrust = args.mass * G
    if args.takeoff_pulse and sim_time < args.pulse_duration:
        return args.pulse_scale * hover_thrust
    if args.thrust is not None:
        return args.thrust
    return hover_thrust


def controller_step(
    sim,
    body: int,
    joints: list[int],
    state: ControllerState,
    mixer: list[list[float]],
    args: argparse.Namespace,
) -> dict[str, object]:
    now = sim.getSimulationTime()
    if state.last_time is None:
        dt = args.time_step
    else:
        dt = clamp(now - state.last_time, 1e-4, 0.05)
    state.last_time = now

    pos = sim.getObjectPosition(body, -1)
    matrix = sim.getObjectMatrix(body, -1)
    lin_vel, ang_vel_world = sim.getVelocity(body)
    rate = world_to_body_rate(matrix, ang_vel_world)
    command = rate_control.RateCommand(
        thrust=collective_thrust_command(args, now),
        p=float(args.p_cmd),
        q=float(args.q_cmd),
        r=float(args.r_cmd),
    )
    rate_state = rate_control.RateControllerState(state.rate_integral, state.motor_speed)
    control = rate_control.controller_step(
        rate_state,
        command,
        rate,
        dt,
        rate_control.gains_from_args(args),
        mixer,
    )
    state.rate_integral = rate_state.rate_integral
    state.motor_speed = rate_state.motor_speed

    for i in range(4):
        state.prop_phase[i] += MOTORS[i]["spin"] * state.motor_speed[i] * dt
    update_propeller_render_if_due(sim, joints, state, control.motor_thrust, args, now)

    total_thrust = sum(control.motor_thrust)
    torque_body = [
        sum(MOTORS[i]["pos"][1] * control.motor_thrust[i] for i in range(4)),
        sum(-MOTORS[i]["pos"][0] * control.motor_thrust[i] for i in range(4)),
        sum(MOTORS[i]["spin"] * args.yaw_drag_arm * control.motor_thrust[i] for i in range(4)),
    ]
    torque_body[0] -= args.angular_drag_rp * rate[0]
    torque_body[1] -= args.angular_drag_rp * rate[1]
    torque_body[2] -= args.angular_drag_yaw * rate[2]

    _, _, body_z = body_axes_from_matrix(matrix)
    force_world = [
        body_z[0] * total_thrust - args.linear_drag_xy * lin_vel[0],
        body_z[1] * total_thrust - args.linear_drag_xy * lin_vel[1],
        body_z[2] * total_thrust - args.linear_drag_z * lin_vel[2],
    ]
    torque_world = body_torque_to_world(matrix, torque_body)
    sim.addForceAndTorque(body, force_world, torque_world)

    return {
        "time": now,
        "dt": dt,
        "pos": pos,
        "lin_vel": lin_vel,
        "ang_vel_world": ang_vel_world,
        "rate": rate,
        "rate_cmd": [command.p, command.q, command.r],
        "rate_error": control.rate_error,
        "thrust_cmd": control.thrust_cmd,
        "moment_cmd": control.moment_cmd,
        "motor_thrust_cmd": control.motor_thrust_cmd,
        "motor_thrust": control.motor_thrust,
        "total_thrust": total_thrust,
        "torque_body": torque_body,
        "force_world": force_world,
        "torque_world": torque_world,
        "body_z": body_z,
        "omega_cmd": control.omega_cmd,
        "omega": state.motor_speed[:],
    }


def apply_motor_speed_commands(
    sim,
    body: int,
    joints: list[int],
    state: ControllerState,
    omega_cmd: list[float],
    args: argparse.Namespace,
) -> dict[str, object]:
    """Apply externally allocated motor speeds through the same plant model."""

    now = sim.getSimulationTime()
    if state.last_time is None:
        dt = args.time_step
    else:
        dt = clamp(now - state.last_time, 1e-4, 0.05)
    state.last_time = now

    pos = sim.getObjectPosition(body, -1)
    matrix = sim.getObjectMatrix(body, -1)
    lin_vel, ang_vel_world = sim.getVelocity(body)
    rate = world_to_body_rate(matrix, ang_vel_world)
    if len(omega_cmd) != 4:
        raise ValueError("apply_motor_speed_commands requires exactly four motor commands.")
    omega_cmd = [clamp(float(value), 0.0, args.max_motor_speed) for value in omega_cmd]

    k_f = args.max_thrust / (args.max_motor_speed * args.max_motor_speed)
    motor_thrust_cmd = [k_f * value * value for value in omega_cmd]
    motor_thrust = []
    for i in range(4):
        tau = args.motor_tau_up if omega_cmd[i] >= state.motor_speed[i] else args.motor_tau_down
        alpha = dt / (tau + dt)
        state.motor_speed[i] = clamp(
            state.motor_speed[i] + alpha * (omega_cmd[i] - state.motor_speed[i]),
            0.0,
            args.max_motor_speed,
        )
        motor_thrust.append(k_f * state.motor_speed[i] * state.motor_speed[i])
        state.prop_phase[i] += MOTORS[i]["spin"] * state.motor_speed[i] * dt
    update_propeller_render_if_due(sim, joints, state, motor_thrust, args, now)

    total_thrust = sum(motor_thrust)
    torque_body = [
        sum(MOTORS[i]["pos"][1] * motor_thrust[i] for i in range(4)),
        sum(-MOTORS[i]["pos"][0] * motor_thrust[i] for i in range(4)),
        sum(MOTORS[i]["spin"] * args.yaw_drag_arm * motor_thrust[i] for i in range(4)),
    ]
    torque_body[0] -= args.angular_drag_rp * rate[0]
    torque_body[1] -= args.angular_drag_rp * rate[1]
    torque_body[2] -= args.angular_drag_yaw * rate[2]

    _, _, body_z = body_axes_from_matrix(matrix)
    force_world = [
        body_z[0] * total_thrust - args.linear_drag_xy * lin_vel[0],
        body_z[1] * total_thrust - args.linear_drag_xy * lin_vel[1],
        body_z[2] * total_thrust - args.linear_drag_z * lin_vel[2],
    ]
    torque_world = body_torque_to_world(matrix, torque_body)
    sim.addForceAndTorque(body, force_world, torque_world)

    thrust_cmd = sum(motor_thrust_cmd)
    moment_cmd = [
        sum(MOTORS[i]["pos"][1] * motor_thrust_cmd[i] for i in range(4)),
        sum(-MOTORS[i]["pos"][0] * motor_thrust_cmd[i] for i in range(4)),
        sum(MOTORS[i]["spin"] * args.yaw_drag_arm * motor_thrust_cmd[i] for i in range(4)),
    ]
    return {
        "time": now,
        "dt": dt,
        "pos": pos,
        "lin_vel": lin_vel,
        "ang_vel_world": ang_vel_world,
        "rate": rate,
        "rate_cmd": [float(args.p_cmd), float(args.q_cmd), float(args.r_cmd)],
        "rate_error": [0.0, 0.0, 0.0],
        "thrust_cmd": thrust_cmd,
        "moment_cmd": moment_cmd,
        "motor_thrust_cmd": motor_thrust_cmd,
        "motor_thrust": motor_thrust,
        "total_thrust": total_thrust,
        "torque_body": torque_body,
        "force_world": force_world,
        "torque_world": torque_world,
        "body_z": body_z,
        "omega_cmd": omega_cmd,
        "omega": state.motor_speed[:],
    }


def main() -> int:
    args = parse_args()
    client, sim = connect(args)
    body, joints = resolve_scene(sim, args)
    set_time_step(sim, args.time_step)
    if args.reset_state:
        reset_body(sim, body, args.start_height)

    hover_omega = rate_control.hover_omega(args.mass, args.max_thrust, args.max_motor_speed)
    state = ControllerState(
        rate_integral=[0.0, 0.0, 0.0],
        motor_speed=[hover_omega, hover_omega, hover_omega, hover_omega],
        prop_phase=[0.0, 0.0, 0.0, 0.0],
    )
    mixer = motor_mixer(args.yaw_drag_arm)

    print("Starting low-level controller:")
    print(f"  command input: thrust={args.thrust if args.thrust is not None else 'mass*g'} N, "
          f"rates=[{args.p_cmd}, {args.q_cmd}, {args.r_cmd}] rad/s")
    print("  controller output: four motor angular velocities [rad/s]")
    if args.takeoff_pulse:
        print(f"  takeoff pulse: {args.pulse_scale:.2f}*mass*g for {args.pulse_duration:.2f} s")
    logger = telemetry.CsvTelemetryLogger(args.log_csv, "body_rate", args.log_sample_period)

    client.setStepping(True)
    sim.startSimulation()
    start_wall = time.time()
    next_log = 0.0
    try:
        while True:
            sample = controller_step(sim, body, joints, state, mixer, args)
            logger.write(float(sample["time"]), telemetry.merge_samples(("", sample)))
            client.step()
            sim_time = float(sample["time"])
            if sim_time >= next_log:
                pos = sample["pos"]
                rate = sample["rate"]
                omega = sample["omega"]
                print(
                    f"t={sim_time:5.2f}s "
                    f"z={pos[2]: .3f}m "
                    f"pqr=[{rate[0]: .3f},{rate[1]: .3f},{rate[2]: .3f}]rad/s "
                    f"omega=[{omega[0]:.0f},{omega[1]:.0f},{omega[2]:.0f},{omega[3]:.0f}]rad/s"
                )
                next_log += args.log_period
            if args.duration > 0.0 and sim_time >= args.duration:
                break
            if args.duration <= 0.0 and time.time() - start_wall > 1e9:
                break
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        logger.close()
        if args.stop_on_exit:
            sim.stopSimulation(True)
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
