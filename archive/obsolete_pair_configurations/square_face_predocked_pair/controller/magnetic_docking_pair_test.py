#!/usr/bin/env python3
"""Magnetic docking and break-limit test for the square-face drone pair.

The docking model is explicit and connector-level:

1. Four connector nodes on Drone A's active square face are paired with the
   matching four connector nodes on Drone B's opposing square face.
2. Before latch, each pair applies a saturated spring-damper attraction inside
   a capture radius.
3. Once all four pairs satisfy distance, speed, and face-angle limits, the
   model switches to a stronger virtual latch.
4. The latch releases if any connector or the net latch wrench exceeds the
   configured break limits.

This is intentionally force-based rather than a permanently welded model, so we
can tune capture, impact, latch, and break behavior for docking-controller work.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path


CONFIG_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = CONFIG_ROOT.parents[1]
PROJECT_CONTROLLER = PROJECT_ROOT / "controller"
PROJECT_SCRIPTS = PROJECT_ROOT / "scripts"
for path in (PROJECT_CONTROLLER, PROJECT_SCRIPTS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import body_rate_controller as flight  # noqa: E402
import generate_drone_plant_scene as plant  # noqa: E402
import telemetry  # noqa: E402


ROOT_ALIAS = "predocked_space_thrust_pair"
DEFAULT_SCENE = CONFIG_ROOT / "scene" / "predocked_square_face_pair_space_thrust.ttt"
DEFAULT_BODY_STL = PROJECT_ROOT / "assets" / "meshes" / "crazyflie_cage_body_no_propellers.stl"


Vector3 = tuple[float, float, float]


@dataclass
class DroneHandles:
    label: str
    body: int
    joints: list[int]
    start_position: Vector3
    state: flight.ControllerState


@dataclass
class ConnectorPair:
    index: int
    local_a: Vector3
    local_b: Vector3


@dataclass
class DockingState:
    mode: str = "free"
    latched: bool = False
    broken_until: float = 0.0
    last_break_reason: str = ""
    connector_forces: list[float] = field(default_factory=list)
    connector_distances: list[float] = field(default_factory=list)
    max_connector_speed: float = 0.0
    face_angle_error: float = 0.0
    net_force: float = 0.0
    net_torque: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run magnetic docking and break-limit test for the two-drone pair.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--scene", default=str(DEFAULT_SCENE))
    parser.add_argument("--load-scene", action="store_true", help="Load the generated pair scene before running.")
    parser.add_argument("--body-stl", default=str(DEFAULT_BODY_STL))

    parser.add_argument("--duration", type=float, default=0.0, help="Duration [s]. Use 0 to run until the window closes.")
    parser.add_argument("--time-step", type=float, default=0.005)
    parser.add_argument("--height", type=float, default=0.5)
    parser.add_argument("--gap", type=float, default=0.030, help="Initial center-to-center connector face gap [m].")
    parser.add_argument(
        "--docking-face",
        choices=("pos_xy_to_neg_xy", "pos_x_neg_y_to_neg_x_pos_y"),
        default="pos_xy_to_neg_xy",
    )
    parser.add_argument("--square-face-support", type=float, default=0.080)
    parser.add_argument("--reset-state", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--mass", type=float, default=0.060, help="Mass per drone module [kg].")
    parser.add_argument("--max-motor-speed", type=float, default=2600.0)
    parser.add_argument("--max-thrust", type=float, default=0.294, help="Per-motor thrust limit [N].")
    parser.add_argument("--yaw-drag-arm", type=float, default=0.006)
    parser.add_argument("--space-thrust-scale", type=float, default=1.20, help="Spacebar thrust per drone as a multiple of mass*g.")
    parser.add_argument("--idle-thrust-scale", type=float, default=1.00, help="Released-space thrust per drone as a multiple of mass*g.")

    parser.add_argument("--kp-rate-rp", type=float, default=0.010)
    parser.add_argument("--kp-rate-yaw", type=float, default=0.0022)
    parser.add_argument("--ki-rate-rp", type=float, default=0.0010)
    parser.add_argument("--ki-rate-yaw", type=float, default=0.00025)
    parser.add_argument("--integral-limit-rp", type=float, default=0.20)
    parser.add_argument("--integral-limit-yaw", type=float, default=0.30)
    parser.add_argument("--motor-tau-up", type=float, default=0.050)
    parser.add_argument("--motor-tau-down", type=float, default=0.080)
    parser.add_argument("--linear-drag-xy", type=float, default=0.018)
    parser.add_argument("--linear-drag-z", type=float, default=0.006)
    parser.add_argument("--angular-drag-rp", type=float, default=0.00055)
    parser.add_argument("--angular-drag-yaw", type=float, default=0.00020)

    parser.add_argument("--capture-radius", type=float, default=0.025, help="Connector-pair magnetic capture radius [m].")
    parser.add_argument("--magnet-rest-distance", type=float, default=0.005, help="Pre-latch connector-pair rest distance [m].")
    parser.add_argument("--magnet-stiffness", type=float, default=1.0, help="Pre-latch connector spring gain [N/m].")
    parser.add_argument("--magnet-damping", type=float, default=0.02, help="Pre-latch connector damping [N/(m/s)].")
    parser.add_argument("--magnet-force-limit", type=float, default=0.008, help="Pre-latch force saturation per connector [N].")
    parser.add_argument("--latch-distance", type=float, default=0.020, help="Maximum connector distance for latch acquisition [m].")
    parser.add_argument("--latch-speed", type=float, default=0.0, help="Maximum connector relative speed for latch acquisition [m/s]. Use 0 to ignore speed.")
    parser.add_argument("--latch-angle-deg", type=float, default=8.0, help="Maximum opposing-face angle error for latch acquisition [deg].")
    parser.add_argument("--latch-rest-distance", type=float, default=0.0005, help="Latched connector-pair rest distance [m].")
    parser.add_argument("--latch-stiffness", type=float, default=20000.0, help="Latched connector spring gain [N/m].")
    parser.add_argument("--latch-damping", type=float, default=80.0, help="Latched connector damping [N/(m/s)].")
    parser.add_argument("--latch-force-limit", type=float, default=80.00, help="Latched force saturation per connector [N].")
    parser.add_argument("--connector-break-force", type=float, default=250.00, help="Break if any connector force exceeds this [N].")
    parser.add_argument("--break-force", type=float, default=500.00, help="Break if net latch force magnitude exceeds this [N].")
    parser.add_argument("--break-torque", type=float, default=30.000, help="Break if net latch torque magnitude exceeds this [N*m].")
    parser.add_argument("--relatch-delay", type=float, default=0.60, help="Cooldown after latch break before recapture [s].")
    parser.add_argument("--log-period", type=float, default=0.25)
    telemetry.add_logging_args(parser)
    return parser.parse_args()


def add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(vector: Vector3, gain: float) -> Vector3:
    return (vector[0] * gain, vector[1] * gain, vector[2] * gain)


def dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(vector: Vector3) -> float:
    return math.sqrt(dot(vector, vector))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalized(vector: Vector3) -> Vector3:
    length = norm(vector)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return scale(vector, 1.0 / length)


def rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 60.0 / (2.0 * math.pi)


def docking_face_normal(face: str) -> Vector3:
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    if face == "pos_xy_to_neg_xy":
        return (inv_sqrt2, inv_sqrt2, 0.0)
    if face == "pos_x_neg_y_to_neg_x_pos_y":
        return (inv_sqrt2, -inv_sqrt2, 0.0)
    raise ValueError(f"Unsupported docking face: {face}")


def pair_start_positions(args: argparse.Namespace) -> dict[str, Vector3]:
    face_normal = docking_face_normal(args.docking_face)
    center_spacing = 2.0 * args.square_face_support + args.gap
    center_offset = 0.5 * center_spacing
    return {
        "drone_a": scale(face_normal, -center_offset)[:2] + (args.height,),
        "drone_b": scale(face_normal, center_offset)[:2] + (args.height,),
    }


def matrix_translation(matrix: list[float]) -> Vector3:
    return (matrix[3], matrix[7], matrix[11])


def transform_point(matrix: list[float], point: Vector3) -> Vector3:
    return (
        matrix[0] * point[0] + matrix[1] * point[1] + matrix[2] * point[2] + matrix[3],
        matrix[4] * point[0] + matrix[5] * point[1] + matrix[6] * point[2] + matrix[7],
        matrix[8] * point[0] + matrix[9] * point[1] + matrix[10] * point[2] + matrix[11],
    )


def transform_direction(matrix: list[float], vector: Vector3) -> Vector3:
    return (
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[4] * vector[0] + matrix[5] * vector[1] + matrix[6] * vector[2],
        matrix[8] * vector[0] + matrix[9] * vector[1] + matrix[10] * vector[2],
    )


def point_velocity(linear_velocity: Vector3, angular_velocity: Vector3, origin: Vector3, point: Vector3) -> Vector3:
    return add(linear_velocity, cross(angular_velocity, sub(point, origin)))


def blank_state(initial_speed: float = 0.0) -> flight.ControllerState:
    return flight.ControllerState(
        rate_integral=[0.0, 0.0, 0.0],
        motor_speed=[initial_speed, initial_speed, initial_speed, initial_speed],
        prop_phase=[0.0, 0.0, 0.0, 0.0],
    )


def reset_drone(sim, drone: DroneHandles) -> None:
    sim.setObjectPosition(drone.body, -1, list(drone.start_position))
    sim.setObjectOrientation(drone.body, -1, [0.0, 0.0, 0.0])
    try:
        sim.resetDynamicObject(drone.body)
    except Exception:
        pass
    drone.state = blank_state()


def set_controller_command(args: argparse.Namespace, thrust: float) -> None:
    args.thrust = thrust
    args.p_cmd = 0.0
    args.q_cmd = 0.0
    args.r_cmd = 0.0
    args.takeoff_pulse = False
    args.pulse_scale = 1.0
    args.pulse_duration = 0.0


def get_drone_handles(sim, label: str, start_position: Vector3) -> DroneHandles:
    body = sim.getObject(f"/{ROOT_ALIAS}/{label}")
    joints = [
        sim.getObject(f"/{ROOT_ALIAS}/{label}/propeller_{index}_root/propeller_{index}_spin_joint")
        for index in range(4)
    ]
    return DroneHandles(label=label, body=body, joints=joints, start_position=start_position, state=blank_state())


def resolve_pair_scene(sim, args: argparse.Namespace) -> tuple[DroneHandles, DroneHandles]:
    flight.stop_if_running(sim)
    if args.load_scene:
        scene_path = Path(args.scene)
        if not scene_path.exists():
            raise FileNotFoundError(scene_path)
        sim.loadScene(str(scene_path))

    starts = pair_start_positions(args)
    try:
        return (
            get_drone_handles(sim, "drone_a", starts["drone_a"]),
            get_drone_handles(sim, "drone_b", starts["drone_b"]),
        )
    except Exception as exc:
        raise RuntimeError(
            f"Could not find /{ROOT_ALIAS}/drone_a and /{ROOT_ALIAS}/drone_b. "
            "Run the magnetic docking launcher so it can generate/load the pair scene."
        ) from exc


def reset_pair(sim, drones: tuple[DroneHandles, DroneHandles], docking_state: DockingState) -> None:
    for drone in drones:
        reset_drone(sim, drone)
    docking_state.mode = "free"
    docking_state.latched = False
    docking_state.broken_until = 0.0
    docking_state.last_break_reason = ""


def face_nodes(body_stl: Path, face_normal: Vector3, positive_side: bool) -> list[Vector3]:
    vertices, _edges = plant.derive_cage_collision_graph(body_stl)
    dots = [dot(vertex, face_normal) for vertex in vertices]
    target = max(dots) if positive_side else min(dots)
    nodes = [vertices[index] for index, value in enumerate(dots) if abs(value - target) < 0.004]
    if len(nodes) != 4:
        side = "positive" if positive_side else "negative"
        raise ValueError(f"Expected 4 {side} square-face connector nodes, found {len(nodes)}.")
    return [(float(node[0]), float(node[1]), float(node[2])) for node in nodes]


def connector_pairs(body_stl: Path, args: argparse.Namespace) -> list[ConnectorPair]:
    face_normal = docking_face_normal(args.docking_face)
    nodes_a = face_nodes(body_stl, face_normal, positive_side=True)
    nodes_b = face_nodes(body_stl, face_normal, positive_side=False)
    starts = pair_start_positions(args)

    remaining_b = set(range(len(nodes_b)))
    pairs = []
    for index_a, node_a in enumerate(nodes_a):
        nominal_a = add(starts["drone_a"], node_a)
        best_b = min(
            remaining_b,
            key=lambda index_b: norm(sub(add(starts["drone_b"], nodes_b[index_b]), nominal_a)),
        )
        remaining_b.remove(best_b)
        pairs.append(ConnectorPair(index=len(pairs), local_a=node_a, local_b=nodes_b[best_b]))
    return pairs


def spring_force(
    p_a: Vector3,
    v_a: Vector3,
    p_b: Vector3,
    v_b: Vector3,
    rest_distance: float,
    stiffness: float,
    damping: float,
    force_limit: float,
) -> tuple[Vector3, float, float]:
    delta = sub(p_b, p_a)
    distance = norm(delta)
    if distance < 1e-9:
        return (0.0, 0.0, 0.0), distance, 0.0
    direction = scale(delta, 1.0 / distance)
    relative_speed = dot(sub(v_b, v_a), direction)
    force_magnitude = stiffness * (distance - rest_distance) + damping * relative_speed
    force_magnitude = clamp(force_magnitude, -force_limit, force_limit)
    return scale(direction, force_magnitude), distance, relative_speed


def apply_wrench_at_connectors(
    sim,
    drone_a: DroneHandles,
    drone_b: DroneHandles,
    pairs: list[ConnectorPair],
    args: argparse.Namespace,
    docking_state: DockingState,
) -> dict[str, object]:
    now = sim.getSimulationTime()
    matrix_a = sim.getObjectMatrix(drone_a.body, -1)
    matrix_b = sim.getObjectMatrix(drone_b.body, -1)
    origin_a = matrix_translation(matrix_a)
    origin_b = matrix_translation(matrix_b)
    lin_a_raw, ang_a_raw = sim.getVelocity(drone_a.body)
    lin_b_raw, ang_b_raw = sim.getVelocity(drone_b.body)
    lin_a, ang_a = tuple(lin_a_raw), tuple(ang_a_raw)
    lin_b, ang_b = tuple(lin_b_raw), tuple(ang_b_raw)

    face_normal = docking_face_normal(args.docking_face)
    normal_a = normalized(transform_direction(matrix_a, face_normal))
    normal_b = normalized(transform_direction(matrix_b, scale(face_normal, -1.0)))
    docking_state.face_angle_error = math.degrees(math.acos(clamp(-dot(normal_a, normal_b), -1.0, 1.0)))

    active_forces: list[Vector3] = []
    connector_forces: list[float] = []
    connector_distances: list[float] = []
    connector_speeds: list[float] = []
    net_force_a = (0.0, 0.0, 0.0)
    net_torque_a = (0.0, 0.0, 0.0)
    net_force_b = (0.0, 0.0, 0.0)
    net_torque_b = (0.0, 0.0, 0.0)

    in_break_cooldown = now < docking_state.broken_until
    any_captured = False
    all_latch_ready = True
    max_connector_force = 0.0

    for pair in pairs:
        p_a = transform_point(matrix_a, pair.local_a)
        p_b = transform_point(matrix_b, pair.local_b)
        v_a = point_velocity(lin_a, ang_a, origin_a, p_a)
        v_b = point_velocity(lin_b, ang_b, origin_b, p_b)
        distance = norm(sub(p_b, p_a))
        connector_distances.append(distance)

        if docking_state.latched:
            force_a, _distance, speed = spring_force(
                p_a,
                v_a,
                p_b,
                v_b,
                args.latch_rest_distance,
                args.latch_stiffness,
                args.latch_damping,
                args.latch_force_limit,
            )
        elif in_break_cooldown or distance > args.capture_radius:
            force_a = (0.0, 0.0, 0.0)
            speed = dot(sub(v_b, v_a), normalized(sub(p_b, p_a)))
        else:
            any_captured = True
            force_a, _distance, speed = spring_force(
                p_a,
                v_a,
                p_b,
                v_b,
                args.magnet_rest_distance,
                args.magnet_stiffness,
                args.magnet_damping,
                args.magnet_force_limit,
            )

        force_b = scale(force_a, -1.0)
        force_norm = norm(force_a)
        max_connector_force = max(max_connector_force, force_norm)
        active_forces.append(force_a)
        connector_forces.append(force_norm)
        connector_speeds.append(abs(speed))

        net_force_a = add(net_force_a, force_a)
        net_torque_a = add(net_torque_a, cross(sub(p_a, origin_a), force_a))
        net_force_b = add(net_force_b, force_b)
        net_torque_b = add(net_torque_b, cross(sub(p_b, origin_b), force_b))

        speed_ok = args.latch_speed <= 0.0 or abs(speed) <= args.latch_speed
        if distance > args.latch_distance or not speed_ok:
            all_latch_ready = False

    docking_state.connector_forces = connector_forces
    docking_state.connector_distances = connector_distances
    docking_state.max_connector_speed = max(connector_speeds) if connector_speeds else 0.0
    docking_state.net_force = norm(net_force_a)
    docking_state.net_torque = norm(net_torque_a)

    if not docking_state.latched:
        if in_break_cooldown:
            docking_state.mode = "break_cooldown"
        elif any_captured:
            docking_state.mode = "magnetic_capture"
        else:
            docking_state.mode = "free"

        if (
            any_captured
            and all_latch_ready
            and docking_state.face_angle_error <= args.latch_angle_deg
        ):
            docking_state.latched = True
            docking_state.mode = "latched"
    else:
        docking_state.mode = "latched"
        if max_connector_force > args.connector_break_force:
            docking_state.latched = False
            docking_state.mode = "broken"
            docking_state.broken_until = now + args.relatch_delay
            docking_state.last_break_reason = f"connector force {max_connector_force:.2f} N"
        elif docking_state.net_force > args.break_force:
            docking_state.latched = False
            docking_state.mode = "broken"
            docking_state.broken_until = now + args.relatch_delay
            docking_state.last_break_reason = f"net force {docking_state.net_force:.2f} N"
        elif docking_state.net_torque > args.break_torque:
            docking_state.latched = False
            docking_state.mode = "broken"
            docking_state.broken_until = now + args.relatch_delay
            docking_state.last_break_reason = f"net torque {docking_state.net_torque:.3f} N*m"

    if norm(net_force_a) > 0.0 or norm(net_torque_a) > 0.0:
        sim.addForceAndTorque(drone_a.body, list(net_force_a), list(net_torque_a))
        sim.addForceAndTorque(drone_b.body, list(net_force_b), list(net_torque_b))

    return {
        "dock_mode": docking_state.mode,
        "dock_latched": docking_state.latched,
        "dock_last_break_reason": docking_state.last_break_reason,
        "dock_face_angle_error_deg": docking_state.face_angle_error,
        "dock_max_distance": max(connector_distances) if connector_distances else 0.0,
        "dock_min_distance": min(connector_distances) if connector_distances else 0.0,
        "dock_max_connector_speed": docking_state.max_connector_speed,
        "dock_net_force": docking_state.net_force,
        "dock_net_torque": docking_state.net_torque,
        "dock_max_connector_force": max_connector_force,
        "dock_connector_distance": connector_distances,
        "dock_connector_force": connector_forces,
    }


class MagneticDockingWindow:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.space_down = False
        self.reset_requested = False
        self.release_latch_requested = False
        self.magnets_enabled = True
        self.running = True

        self.root = tk.Tk()
        self.root.title("Magnetic Docking Pair Test")
        self.root.geometry("620x250")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.status = tk.StringVar(value="magnets on, latch free")
        self.telemetry = tk.StringVar(value="state=free  d=0.000 m  F=0.00 N  tau=0.000 N*m")
        self.motor_text = tk.StringVar(value="avgRPM A=0  B=0")

        tk.Label(self.root, text="Magnetic Docking Pair Test", font=("Segoe UI", 16, "bold")).pack(pady=(16, 6))
        tk.Label(self.root, textvariable=self.status, font=("Segoe UI", 12)).pack()
        tk.Label(self.root, textvariable=self.telemetry, font=("Consolas", 10)).pack(pady=(8, 2))
        tk.Label(self.root, textvariable=self.motor_text, font=("Consolas", 10)).pack()
        tk.Label(self.root, text="Space: thrust both    M: magnets on/off    L: release latch    R: reset    Esc/Q: quit", font=("Segoe UI", 9)).pack(pady=(10, 0))

        self.root.bind("<KeyPress-space>", self.on_space_press)
        self.root.bind("<KeyRelease-space>", self.on_space_release)
        self.root.bind("<KeyPress-m>", self.on_toggle_magnets)
        self.root.bind("<KeyPress-M>", self.on_toggle_magnets)
        self.root.bind("<KeyPress-l>", self.on_release_latch)
        self.root.bind("<KeyPress-L>", self.on_release_latch)
        self.root.bind("<KeyPress-r>", self.on_reset)
        self.root.bind("<KeyPress-R>", self.on_reset)
        self.root.bind("<Escape>", self.on_quit)
        self.root.bind("<KeyPress-q>", self.on_quit)
        self.root.bind("<KeyPress-Q>", self.on_quit)
        self.root.after(100, self.root.focus_force)

    def on_space_press(self, _event) -> None:
        self.space_down = True

    def on_space_release(self, _event) -> None:
        self.space_down = False

    def on_toggle_magnets(self, _event) -> None:
        self.magnets_enabled = not self.magnets_enabled

    def on_release_latch(self, _event) -> None:
        self.release_latch_requested = True

    def on_reset(self, _event) -> None:
        self.reset_requested = True

    def on_quit(self, _event) -> None:
        self.close()

    def close(self) -> None:
        self.running = False

    def update(
        self,
        docking_sample: dict[str, object],
        sample_a: dict[str, object],
        sample_b: dict[str, object],
    ) -> None:
        thrust_scale = self.args.space_thrust_scale if self.space_down else self.args.idle_thrust_scale
        thrust_per_drone = thrust_scale * self.args.mass * flight.G
        magnet_status = "on" if self.magnets_enabled else "off"
        self.status.set(
            f"magnets {magnet_status} | Space thrust {thrust_per_drone:.3f} N per drone ({thrust_scale:.2f}*mg)"
        )
        self.telemetry.set(
            f"state={docking_sample['dock_mode']}  "
            f"d=[{docking_sample['dock_min_distance']:.3f},{docking_sample['dock_max_distance']:.3f}] m  "
            f"Fmax={docking_sample['dock_max_connector_force']:.2f} N  "
            f"Fnet={docking_sample['dock_net_force']:.2f} N  "
            f"tau={docking_sample['dock_net_torque']:.3f} N*m  "
            f"ang={docking_sample['dock_face_angle_error_deg']:.1f} deg"
        )
        rpm_a = rad_s_to_rpm(sum(sample_a["omega"]) / 4.0)
        rpm_b = rad_s_to_rpm(sum(sample_b["omega"]) / 4.0)
        pos_a = sample_a["pos"]
        pos_b = sample_b["pos"]
        self.motor_text.set(f"zA={pos_a[2]:.3f} m  zB={pos_b[2]:.3f} m    avgRPM A={rpm_a:.0f}  B={rpm_b:.0f}")
        self.root.update_idletasks()
        self.root.update()


def main() -> int:
    args = parse_args()
    client, sim = flight.connect(args)
    drone_a, drone_b = resolve_pair_scene(sim, args)
    flight.set_time_step(sim, args.time_step)
    drones = (drone_a, drone_b)
    docking_state = DockingState()
    pairs = connector_pairs(Path(args.body_stl), args)

    if args.reset_state:
        reset_pair(sim, drones, docking_state)

    hover_omega = math.sqrt((args.mass * flight.G / 4.0) / (args.max_thrust / args.max_motor_speed**2))
    mixer = flight.motor_mixer(args.yaw_drag_arm)
    window = MagneticDockingWindow(args)
    logger = telemetry.CsvTelemetryLogger(args.log_csv, "magnetic_docking_pair", args.log_sample_period)

    print("Magnetic docking pair test:")
    print("  four connector-level magnet pairs on the selected square face")
    print("  Space -> thrust both drones")
    print("  M -> toggle magnet forces, L -> manually release latch, R -> reset pair")
    print(f"  hover speed reference per drone: {hover_omega:.0f} rad/s ({rad_s_to_rpm(hover_omega):.0f} rpm) per motor")
    print(f"  connector pairs: {len(pairs)}")

    client.setStepping(True)
    sim.startSimulation()
    next_log = 0.0
    sim_start = sim.getSimulationTime()
    empty_dock_sample: dict[str, object] = {
        "dock_mode": "magnets_off",
        "dock_latched": False,
        "dock_last_break_reason": "",
        "dock_face_angle_error_deg": 0.0,
        "dock_max_distance": 0.0,
        "dock_min_distance": 0.0,
        "dock_max_connector_speed": 0.0,
        "dock_net_force": 0.0,
        "dock_net_torque": 0.0,
        "dock_max_connector_force": 0.0,
        "dock_connector_distance": [0.0, 0.0, 0.0, 0.0],
        "dock_connector_force": [0.0, 0.0, 0.0, 0.0],
    }

    try:
        while window.running:
            loop_wall = time.time()
            thrust_scale = args.space_thrust_scale if window.space_down else args.idle_thrust_scale
            set_controller_command(args, thrust_scale * args.mass * flight.G)

            if window.reset_requested:
                reset_pair(sim, drones, docking_state)
                window.reset_requested = False

            if window.release_latch_requested:
                docking_state.latched = False
                docking_state.mode = "manual_release"
                docking_state.broken_until = sim.getSimulationTime() + args.relatch_delay
                docking_state.last_break_reason = "manual release"
                window.release_latch_requested = False

            sample_a = flight.controller_step(sim, drone_a.body, drone_a.joints, drone_a.state, mixer, args)
            sample_b = flight.controller_step(sim, drone_b.body, drone_b.joints, drone_b.state, mixer, args)
            docking_sample = (
                apply_wrench_at_connectors(sim, drone_a, drone_b, pairs, args, docking_state)
                if window.magnets_enabled
                else empty_dock_sample
            )

            sim_time = float(sample_a["time"]) - sim_start
            logger.write(
                float(sample_a["time"]),
                telemetry.merge_samples(
                    ("a", sample_a),
                    ("b", sample_b),
                    ("dock", docking_sample),
                    extra={
                        "space_down": window.space_down,
                        "magnets_enabled": window.magnets_enabled,
                        "thrust_scale": thrust_scale,
                        "thrust_per_drone": args.thrust,
                    },
                ),
            )
            client.step()
            window.update(docking_sample, sample_a, sample_b)

            if sim_time >= next_log:
                print(
                    f"t={sim_time:5.2f}s state={docking_sample['dock_mode']} "
                    f"d=[{docking_sample['dock_min_distance']:.3f},{docking_sample['dock_max_distance']:.3f}]m "
                    f"Fmax={docking_sample['dock_max_connector_force']:.2f}N "
                    f"Fnet={docking_sample['dock_net_force']:.2f}N "
                    f"tau={docking_sample['dock_net_torque']:.3f}Nm "
                    f"zA={sample_a['pos'][2]:.3f} zB={sample_b['pos'][2]:.3f}"
                )
                next_log += args.log_period

            if args.duration > 0.0 and sim_time >= args.duration:
                break

            elapsed = time.time() - loop_wall
            if elapsed < args.time_step:
                time.sleep(args.time_step - elapsed)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        logger.close()
        window.running = False
        try:
            window.root.destroy()
        except tk.TclError:
            pass
        if args.stop_on_exit:
            sim.stopSimulation(True)
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
