#!/usr/bin/env python3
"""Hover all spawned drone plant models with external Python control.

Each discovered drone gets its own high-level position controller state and
its own low-level body-rate/motor state. The hover target defaults to the
drone's initial position, so the pair should lift/hold as soon as this script
starts the CoppeliaSim simulation.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.common import telemetry  # noqa: E402
from controller.high_level import position  # noqa: E402
from controller.low_level import body_rate as flight  # noqa: E402
from simulation import magnetic_docking as multi  # noqa: E402


DEFAULT_SCENE = PROJECT_ROOT / "scene" / "two_drone_spawn_scene.ttt"
DEFAULT_BODY_STL = PROJECT_ROOT / "assets" / "meshes" / "crazyflie_cage_body_no_propellers.stl"
TARGET_ALIAS_PREFIX = "hover_target"


@dataclass
class HoverDrone:
    drone: multi.DroneAgent
    high_state: position.PositionControllerState
    low_state: flight.ControllerState
    args: argparse.Namespace
    target_handle: int | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Make two spawned drone plant models hover with Python control.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--scene", default=str(DEFAULT_SCENE))
    parser.add_argument("--load-scene", action="store_true", help="Load the saved two-drone scene before running.")
    parser.add_argument("--body-stl", default=str(DEFAULT_BODY_STL))

    parser.add_argument("--duration", type=float, default=0.0, help="Duration [s]. Use 0 for Ctrl+C.")
    parser.add_argument("--time-step", type=float, default=0.005)
    parser.add_argument("--reset-state", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--target-spheres", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--target-radius", type=float, default=0.025)

    parser.add_argument("--mass", type=float, default=flight.DEFAULT_MASS)
    parser.add_argument("--max-motor-speed", type=float, default=flight.DEFAULT_MAX_MOTOR_SPEED)
    parser.add_argument("--max-thrust", type=float, default=flight.DEFAULT_MAX_THRUST)
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

    parser.add_argument("--magnets-start-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--capture-radius", type=float, default=0.025 * flight.DEFAULT_GEOMETRY_SCALE, help="Maximum distance for each compatible face-corner magnet pair [m].")
    parser.add_argument("--max-magnet-pairs-per-drone-pair", type=int, default=8)
    parser.add_argument("--face-docking", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--face-normal-tolerance-deg", type=float, default=20.0)
    parser.add_argument("--face-center-tolerance", type=float, default=0.030 * flight.DEFAULT_GEOMETRY_SCALE)
    parser.add_argument("--face-latch-required-fraction", type=float, default=1.0)
    parser.add_argument("--magnet-rest-distance", type=float, default=multi.DEFAULT_CONNECTOR_CONTACT_DISTANCE)
    parser.add_argument("--magnet-stiffness", type=float, default=1.0, help="Pre-latch connector spring gain [N/m].")
    parser.add_argument("--magnet-damping", type=float, default=0.02, help="Pre-latch connector damping [N/(m/s)].")
    parser.add_argument("--magnet-force-limit", type=float, default=0.008, help="Pre-latch force saturation per connector [N].")
    parser.add_argument("--latch-distance", type=float, default=multi.DEFAULT_LATCH_DISTANCE)
    parser.add_argument("--latch-speed", type=float, default=0.12, help="Maximum connector relative speed for latch acquisition [m/s].")
    parser.add_argument("--latch-rest-distance", type=float, default=multi.DEFAULT_CONNECTOR_CONTACT_DISTANCE)
    parser.add_argument("--latch-stiffness", type=float, default=200.0, help="Latched connector spring gain [N/m].")
    parser.add_argument("--latch-damping", type=float, default=0.25, help="Latched connector damping [N/(m/s)].")
    parser.add_argument("--latch-force-limit", type=float, default=1.50, help="Latched force saturation per connector [N].")
    parser.add_argument("--latch-stiffness-ramp-time", type=float, default=0.25, help="Time to ramp latch stiffness after acquisition [s].")
    parser.add_argument("--connector-break-force", type=float, default=8.00, help="Break if a latched connector exceeds this force [N].")
    parser.add_argument("--latch-break-distance", type=float, default=0.040 * flight.DEFAULT_GEOMETRY_SCALE, help="Break if a latched connector stretches beyond this [m].")
    parser.add_argument("--relatch-delay", type=float, default=0.60, help="Cooldown after latch break [s].")

    parser.add_argument("--log-period", type=float, default=0.25)
    telemetry.add_logging_args(parser)
    return parser.parse_args()


def remove_old_targets(sim) -> None:
    to_remove = []
    for handle in multi.all_scene_objects(sim):
        alias = multi.object_alias(sim, handle)
        tail = multi.alias_tail(alias)
        if tail.startswith(TARGET_ALIAS_PREFIX):
            to_remove.append(handle)
    if to_remove:
        sim.removeObjects(to_remove, False)


def create_target_sphere(sim, index: int, target: list[float], radius: float) -> int:
    sphere = sim.createPrimitiveShape(sim.primitiveshape_spheroid, [radius, radius, radius], 0)
    sim.setObjectAlias(sphere, f"{TARGET_ALIAS_PREFIX}_{index}", 1)
    sim.setObjectPosition(sphere, -1, target)
    sim.setShapeColor(sphere, None, sim.colorcomponent_ambient_diffuse, [0.05, 0.35, 1.0])
    sim.setObjectInt32Param(sphere, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(sphere, sim.shapeintparam_respondable, 0)
    return int(sphere)


def controller_args_for_drone(base_args: argparse.Namespace, target: tuple[float, float, float], yaw: float) -> argparse.Namespace:
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


def build_hover_drones(sim, drones: list[multi.DroneAgent], hover_omega: float, args: argparse.Namespace) -> list[HoverDrone]:
    remove_old_targets(sim)
    hover_drones = []
    for drone in drones:
        target = tuple(float(value) for value in sim.getObjectPosition(drone.body, -1))
        orientation = tuple(float(value) for value in sim.getObjectOrientation(drone.body, -1))
        target_handle = create_target_sphere(sim, drone.index, list(target), args.target_radius) if args.target_spheres else None
        hover_drones.append(
            HoverDrone(
                drone=drone,
                high_state=position.PositionControllerState(pos_integral=[0.0, 0.0, 0.0]),
                low_state=flight.ControllerState(
                    rate_integral=[0.0, 0.0, 0.0],
                    motor_speed=[hover_omega, hover_omega, hover_omega, hover_omega],
                    prop_phase=[0.0, 0.0, 0.0, 0.0],
                ),
                args=controller_args_for_drone(args, target, orientation[2]),
                target_handle=target_handle,
            )
        )
    return hover_drones


def reset_hover_drone(sim, hover: HoverDrone, hover_omega: float) -> None:
    multi.reset_drone(sim, hover.drone, hover_omega)
    target = tuple(float(value) for value in sim.getObjectPosition(hover.drone.body, -1))
    orientation = tuple(float(value) for value in sim.getObjectOrientation(hover.drone.body, -1))
    hover.high_state = position.PositionControllerState(pos_integral=[0.0, 0.0, 0.0])
    hover.low_state = flight.ControllerState(
        rate_integral=[0.0, 0.0, 0.0],
        motor_speed=[hover_omega, hover_omega, hover_omega, hover_omega],
        prop_phase=[0.0, 0.0, 0.0, 0.0],
    )
    hover.args = controller_args_for_drone(hover.args, target, orientation[2])
    if hover.target_handle is not None:
        sim.setObjectPosition(hover.target_handle, -1, list(target))


def format_vector(values: list[float] | tuple[float, float, float]) -> str:
    return f"[{values[0]: .3f},{values[1]: .3f},{values[2]: .3f}]"


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
    hover_omega = math.sqrt((args.mass * flight.G / 4.0) / (args.max_thrust / args.max_motor_speed**2))
    drones = multi.discover_drones(sim, hover_omega)
    if len(drones) < 2:
        raise RuntimeError(
            f"Expected at least two /{multi.MODEL_ALIAS} models. "
            "Run: python scripts\\generation\\spawn_two_drones.py"
        )

    if args.reset_state:
        for drone in drones:
            multi.reset_drone(sim, drone, hover_omega)

    hover_drones = build_hover_drones(sim, drones, hover_omega, args)
    geometry = multi.docking_geometry(Path(args.body_stl))
    connectors = geometry.connectors
    docking_memory = multi.DockingMemory()
    mixer = flight.motor_mixer(args.yaw_drag_arm)
    logger = telemetry.CsvTelemetryLogger(args.log_csv, "two_drone_hover", args.log_sample_period)

    print("Two-drone hover controller:")
    print(f"  discovered drones: {len(hover_drones)}")
    for hover in hover_drones:
        target = [hover.args.target_x, hover.args.target_y, hover.args.target_z]
        print(f"    d{hover.drone.index}: {hover.drone.path}, target={format_vector(target)}")
    print("  high-level: position/yaw PID -> T, p_cmd, q_cmd, r_cmd")
    print("  low-level: body-rate PI -> four motor speeds")
    print(f"  magnetic connectors: {len(connectors)} per drone, enabled={args.magnets_start_enabled}")

    client.setStepping(True)
    sim.startSimulation()
    sim_start = sim.getSimulationTime()
    next_log = 0.0
    start_wall = time.time()
    try:
        while True:
            if simulation_stopped_by_gui(sim):
                print("CoppeliaSim simulation stopped; exiting Python hover controller.")
                break

            high_samples = []
            low_samples = []
            for hover in hover_drones:
                high = position.high_level_step(
                    sim,
                    hover.drone.body,
                    hover.target_handle,
                    hover.high_state,
                    hover.args,
                )
                low = flight.controller_step(
                    sim,
                    hover.drone.body,
                    hover.drone.joints,
                    hover.low_state,
                    mixer,
                    hover.args,
                )
                high_samples.append(high)
                low_samples.append(low)

            docking_sample = multi.apply_magnetic_docking(
                sim,
                [hover.drone for hover in hover_drones],
                geometry,
                docking_memory,
                args,
                args.magnets_start_enabled,
            )
            sim_time = float(high_samples[0]["time"]) - sim_start
            telemetry_samples = []
            for index, (high, low) in enumerate(zip(high_samples, low_samples)):
                telemetry_samples.append((f"d{index}_high", high))
                telemetry_samples.append((f"d{index}_low", low))
            logger.write(
                float(high_samples[0]["time"]),
                telemetry.merge_samples(
                    *telemetry_samples,
                    ("dock", docking_sample),
                    extra={"magnets_enabled": args.magnets_start_enabled},
                ),
            )

            client.step()
            if simulation_stopped_by_gui(sim):
                print("CoppeliaSim simulation stopped; exiting Python hover controller.")
                break

            if sim_time >= next_log:
                parts = []
                for index, (high, low) in enumerate(zip(high_samples, low_samples)):
                    err = high["pos_error"]
                    pos = high["pos"]
                    rpm = sum(low["omega"]) * 60.0 / (4.0 * 2.0 * math.pi)
                    parts.append(
                        f"d{index} pos={format_vector(pos)} err={format_vector(err)} rpm={rpm:.0f}"
                    )
                dock_text = (
                    f"dock={docking_sample['mode']} cap={docking_sample['capture_pairs']} "
                    f"latched={docking_sample['latched_pairs']} "
                    f"d=[{docking_sample['min_distance']:.3f},{docking_sample['max_distance']:.3f}] "
                    f"F={docking_sample['max_force']:.2f}N"
                )
                print(f"t={sim_time:5.2f}s {dock_text} | " + " | ".join(parts))
                next_log += args.log_period

            if args.duration > 0.0 and sim_time >= args.duration:
                break
            if args.duration <= 0.0 and time.time() - start_wall > 1e9:
                break
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        logger.close()
        multi.clear_docking_memory(sim, docking_memory, clear_cooldowns=True)
        release_stepping(client)
        if args.stop_on_exit and not simulation_stopped_by_gui(sim):
            sim.stopSimulation(True)
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
