#!/usr/bin/env python3
"""High-level position/yaw controller for the CoppeliaSim drone.

This is a cascaded controller:

    position/yaw target
        -> desired acceleration and desired attitude
        -> T, p_cmd, q_cmd, r_cmd
        -> controller.low_level.body_rate
        -> motor speeds and plant force/torque

The blue target sphere is movable in CoppeliaSim. Drag it during simulation to
command a new position. This is a simulation high-level controller: it uses
perfect pose/velocity from CoppeliaSim, not an onboard estimator.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.common import telemetry  # noqa: E402
from controller.low_level import body_rate as flight  # noqa: E402


TARGET_ALIAS = "position_target"


@dataclass
class PositionControllerState:
    pos_integral: list[float]
    last_time: float | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run high-level position control through the low-level rate controller.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--scene", default=str(flight.SCENE_PATH))
    parser.add_argument("--load-scene", action="store_true", help="Load the generated scene before running.")

    parser.add_argument("--duration", type=float, default=20.0, help="Duration [s]. Use 0 for Ctrl+C.")
    parser.add_argument("--time-step", type=float, default=0.005)
    parser.add_argument("--start-height", type=float, default=0.5)
    parser.add_argument("--reset-state", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--target-x", type=float, default=0.0)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.75)
    parser.add_argument("--target-yaw", type=float, default=0.0, help="Target yaw [rad].")
    parser.add_argument("--target-sphere", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--target-radius", type=float, default=0.035)

    parser.add_argument("--mass", type=float, default=flight.DEFAULT_MASS)
    parser.add_argument("--max-motor-speed", type=float, default=flight.DEFAULT_MAX_MOTOR_SPEED)
    parser.add_argument("--max-thrust", type=float, default=flight.DEFAULT_MAX_THRUST, help="Per-motor thrust limit [N].")
    parser.add_argument("--yaw-drag-arm", type=float, default=flight.DEFAULT_YAW_DRAG_ARM)

    parser.add_argument("--kp-xy", type=float, default=4.0)
    parser.add_argument("--kd-xy", type=float, default=2.8)
    parser.add_argument("--ki-xy", type=float, default=0.03)
    parser.add_argument("--kp-z", type=float, default=7.0)
    parser.add_argument("--kd-z", type=float, default=4.5)
    parser.add_argument("--ki-z", type=float, default=0.35)
    parser.add_argument("--integral-limit-xy", type=float, default=0.35)
    parser.add_argument("--integral-limit-z", type=float, default=0.25)

    parser.add_argument("--attitude-gain-rp", type=float, default=8.0)
    parser.add_argument("--attitude-gain-yaw", type=float, default=4.0)
    parser.add_argument("--max-roll-rate-deg", type=float, default=280.0)
    parser.add_argument("--max-pitch-rate-deg", type=float, default=280.0)
    parser.add_argument("--max-yaw-rate-deg", type=float, default=160.0)
    parser.add_argument("--max-tilt-deg", type=float, default=34.0)
    parser.add_argument("--max-horizontal-accel", type=float, default=5.5)
    parser.add_argument("--max-vertical-accel", type=float, default=7.0)

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
    flight.add_propeller_visual_args(parser)
    parser.add_argument("--log-period", type=float, default=0.25)
    telemetry.add_logging_args(parser)
    return parser.parse_args()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_norm(vector: list[float], max_norm: float) -> list[float]:
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= max_norm or norm < 1e-12:
        return vector
    return [value * max_norm / norm for value in vector]


def dot(a: list[float], b: list[float]) -> float:
    return sum(a[i] * b[i] for i in range(3))


def cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def norm(vector: list[float]) -> float:
    return math.sqrt(dot(vector, vector))


def normalize(vector: list[float]) -> list[float]:
    length = norm(vector)
    if length < 1e-12:
        return [0.0, 0.0, 1.0]
    return [value / length for value in vector]


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


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


def find_target(sim) -> int | None:
    matches = [
        handle
        for handle in all_scene_objects(sim)
        if str(sim.getObjectAlias(handle, 1)).rsplit("/", 1)[-1] == TARGET_ALIAS
    ]
    if len(matches) > 1:
        raise RuntimeError(f"Expected at most one /{TARGET_ALIAS}, found {len(matches)}.")
    return matches[0] if matches else None


def create_or_update_target(sim, args: argparse.Namespace) -> int | None:
    if not args.target_sphere:
        return None
    target = find_target(sim)
    if target is None:
        target = sim.createPrimitiveShape(
            sim.primitiveshape_spheroid,
            [args.target_radius, args.target_radius, args.target_radius],
            0,
        )
        sim.setObjectAlias(target, TARGET_ALIAS, 1)
        sim.setShapeColor(target, None, sim.colorcomponent_ambient_diffuse, [0.05, 0.35, 1.0])
        sim.setObjectInt32Param(target, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(target, sim.shapeintparam_respondable, 0)
    sim.setObjectPosition(target, -1, [args.target_x, args.target_y, args.target_z])
    return target


def target_position(sim, args: argparse.Namespace, target_handle: int | None) -> list[float]:
    if target_handle is not None:
        return sim.getObjectPosition(target_handle, -1)
    return [args.target_x, args.target_y, args.target_z]


def desired_axes_from_force(force_world: list[float], yaw: float) -> tuple[list[float], list[float], list[float]]:
    b3_des = normalize(force_world)
    yaw_x = [math.cos(yaw), math.sin(yaw), 0.0]
    b2_des = cross(b3_des, yaw_x)
    if norm(b2_des) < 1e-6:
        b2_des = [0.0, 1.0, 0.0]
    b2_des = normalize(b2_des)
    b1_des = normalize(cross(b2_des, b3_des))
    return b1_des, b2_des, b3_des


def attitude_error_body(matrix: list[float], desired_axes: tuple[list[float], list[float], list[float]]) -> list[float]:
    body_axes = flight.body_axes_from_matrix(matrix)
    error_world = [0.0, 0.0, 0.0]
    for current_axis, desired_axis in zip(body_axes, desired_axes):
        term = cross(current_axis, desired_axis)
        for i in range(3):
            error_world[i] += 0.5 * term[i]
    return [
        dot(body_axes[0], error_world),
        dot(body_axes[1], error_world),
        dot(body_axes[2], error_world),
    ]


def high_level_step(
    sim,
    body: int,
    target_handle: int | None,
    state: PositionControllerState,
    args: argparse.Namespace,
) -> dict[str, object]:
    now = sim.getSimulationTime()
    if state.last_time is None:
        dt = args.time_step
    else:
        dt = clamp(now - state.last_time, 1e-4, 0.05)
    state.last_time = now

    pos = sim.getObjectPosition(body, -1)
    target = target_position(sim, args, target_handle)
    matrix = sim.getObjectMatrix(body, -1)
    lin_vel, _ = sim.getVelocity(body)

    pos_error = [target[i] - pos[i] for i in range(3)]
    integral_limits = [args.integral_limit_xy, args.integral_limit_xy, args.integral_limit_z]
    for i in range(3):
        state.pos_integral[i] = clamp(
            state.pos_integral[i] + pos_error[i] * dt,
            -integral_limits[i],
            integral_limits[i],
        )

    pid_accel_p = [
        args.kp_xy * pos_error[0],
        args.kp_xy * pos_error[1],
        args.kp_z * pos_error[2],
    ]
    pid_accel_i = [
        args.ki_xy * state.pos_integral[0],
        args.ki_xy * state.pos_integral[1],
        args.ki_z * state.pos_integral[2],
    ]
    pid_accel_d = [
        -args.kd_xy * lin_vel[0],
        -args.kd_xy * lin_vel[1],
        -args.kd_z * lin_vel[2],
    ]
    pid_accel_sum_raw = [
        pid_accel_p[0] + pid_accel_i[0] + pid_accel_d[0],
        pid_accel_p[1] + pid_accel_i[1] + pid_accel_d[1],
        pid_accel_p[2] + pid_accel_i[2] + pid_accel_d[2],
    ]

    accel_xy = clamp_norm(pid_accel_sum_raw[:2], args.max_horizontal_accel)
    accel_z = clamp(pid_accel_sum_raw[2], -args.max_vertical_accel, args.max_vertical_accel)

    force_world = [
        args.mass * accel_xy[0],
        args.mass * accel_xy[1],
        args.mass * (flight.G + accel_z),
    ]

    max_tilt = math.radians(args.max_tilt_deg)
    horizontal_force = math.sqrt(force_world[0] ** 2 + force_world[1] ** 2)
    vertical_force = max(force_world[2], 0.1 * args.mass * flight.G)
    max_horizontal_force = math.tan(max_tilt) * vertical_force
    if horizontal_force > max_horizontal_force:
        scale = max_horizontal_force / max(horizontal_force, 1e-12)
        force_world[0] *= scale
        force_world[1] *= scale

    body_axes = flight.body_axes_from_matrix(matrix)
    thrust_cmd = clamp(dot(force_world, body_axes[2]), 0.0, 4.0 * args.max_thrust)
    desired_axes = desired_axes_from_force(force_world, args.target_yaw)
    att_error = attitude_error_body(matrix, desired_axes)

    args.thrust = thrust_cmd
    args.p_cmd = clamp(
        args.attitude_gain_rp * att_error[0],
        -math.radians(args.max_roll_rate_deg),
        math.radians(args.max_roll_rate_deg),
    )
    args.q_cmd = clamp(
        args.attitude_gain_rp * att_error[1],
        -math.radians(args.max_pitch_rate_deg),
        math.radians(args.max_pitch_rate_deg),
    )
    args.r_cmd = clamp(
        args.attitude_gain_yaw * att_error[2],
        -math.radians(args.max_yaw_rate_deg),
        math.radians(args.max_yaw_rate_deg),
    )
    args.takeoff_pulse = False
    args.pulse_scale = 1.0
    args.pulse_duration = 0.0

    return {
        "time": now,
        "dt": dt,
        "pos": pos,
        "target": target,
        "target_yaw": args.target_yaw,
        "pos_error": pos_error,
        "pos_integral": state.pos_integral[:],
        "lin_vel": lin_vel,
        "pid_accel_p": pid_accel_p,
        "pid_accel_i": pid_accel_i,
        "pid_accel_d": pid_accel_d,
        "pid_accel_sum_raw": pid_accel_sum_raw,
        "accel_cmd": [accel_xy[0], accel_xy[1], accel_z],
        "force_world": force_world,
        "thrust_cmd": thrust_cmd,
        "rate_cmd": [args.p_cmd, args.q_cmd, args.r_cmd],
        "att_error": att_error,
        "desired_b3": desired_axes[2],
    }


def main() -> int:
    args = parse_args()
    client, sim = flight.connect(args)
    body, joints = flight.resolve_scene(sim, args)
    flight.set_time_step(sim, args.time_step)
    if args.reset_state:
        flight.reset_body(sim, body, args.start_height)
    target_handle = create_or_update_target(sim, args)

    hover_omega = math.sqrt((args.mass * flight.G / 4.0) / (args.max_thrust / args.max_motor_speed**2))
    low_state = flight.ControllerState(
        rate_integral=[0.0, 0.0, 0.0],
        motor_speed=[hover_omega, hover_omega, hover_omega, hover_omega],
        prop_phase=[0.0, 0.0, 0.0, 0.0],
    )
    high_state = PositionControllerState(pos_integral=[0.0, 0.0, 0.0])
    mixer = flight.motor_mixer(args.yaw_drag_arm)

    print("High-level position controller:")
    print("  target input: blue position_target sphere, movable in CoppeliaSim")
    print("  high-level output: T, p_cmd, q_cmd, r_cmd")
    print("  low-level output: four motor angular velocities")
    logger = telemetry.CsvTelemetryLogger(args.log_csv, "position", args.log_sample_period)

    client.setStepping(True)
    sim.startSimulation()
    start_wall = time.time()
    next_log = 0.0
    try:
        while True:
            high_sample = high_level_step(sim, body, target_handle, high_state, args)
            low_sample = flight.controller_step(sim, body, joints, low_state, mixer, args)
            logger.write(
                float(high_sample["time"]),
                telemetry.merge_samples(("high", high_sample), ("low", low_sample)),
            )
            client.step()

            sim_time = float(high_sample["time"])
            if sim_time >= next_log:
                pos = high_sample["pos"]
                target = high_sample["target"]
                err = high_sample["pos_error"]
                rate_cmd = high_sample["rate_cmd"]
                omega = low_sample["omega"]
                print(
                    f"t={sim_time:5.2f}s "
                    f"pos=[{pos[0]: .2f},{pos[1]: .2f},{pos[2]: .2f}] "
                    f"target=[{target[0]: .2f},{target[1]: .2f},{target[2]: .2f}] "
                    f"err=[{err[0]: .2f},{err[1]: .2f},{err[2]: .2f}] "
                    f"T={high_sample['thrust_cmd']:.3f}N "
                    f"pqr_cmd=[{rate_cmd[0]: .2f},{rate_cmd[1]: .2f},{rate_cmd[2]: .2f}] "
                    f"omega=[{omega[0]:.0f},{omega[1]:.0f},{omega[2]:.0f},{omega[3]:.0f}]"
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
