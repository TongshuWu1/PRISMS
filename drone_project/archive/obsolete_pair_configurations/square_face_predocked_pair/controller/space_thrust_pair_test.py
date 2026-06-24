#!/usr/bin/env python3
"""Manual Space-key thrust test for the pre-docked two-drone plant.

Focus the small control window and hold Space. The same open-loop collective
thrust command is sent to both drone modules; each module still has its own
low-level body-rate controller, motor lag, mixer, and spinning propeller joints.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path


CONFIG_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = CONFIG_ROOT.parents[1]
PROJECT_CONTROLLER = PROJECT_ROOT / "controller"
if str(PROJECT_CONTROLLER) not in sys.path:
    sys.path.insert(0, str(PROJECT_CONTROLLER))

import body_rate_controller as flight  # noqa: E402
import telemetry  # noqa: E402


ROOT_ALIAS = "predocked_space_thrust_pair"
DEFAULT_SCENE = CONFIG_ROOT / "scene" / "predocked_square_face_pair_space_thrust.ttt"


@dataclass
class DroneHandles:
    label: str
    body: int
    joints: list[int]
    start_position: tuple[float, float, float]
    state: flight.ControllerState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Hold Space to apply collective thrust to the two-drone pair.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--scene", default=str(DEFAULT_SCENE))
    parser.add_argument("--load-scene", action="store_true", help="Load the generated pair scene before running.")

    parser.add_argument("--duration", type=float, default=0.0, help="Duration [s]. Use 0 to run until the window closes.")
    parser.add_argument("--time-step", type=float, default=0.005)
    parser.add_argument("--height", type=float, default=0.5)
    parser.add_argument("--gap", type=float, default=0.001)
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
    parser.add_argument("--space-thrust-scale", type=float, default=1.35, help="Spacebar thrust per drone as a multiple of mass*g.")
    parser.add_argument("--idle-thrust-scale", type=float, default=0.0, help="Released-space thrust per drone as a multiple of mass*g.")

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
    parser.add_argument("--log-period", type=float, default=0.25)
    telemetry.add_logging_args(parser)
    return parser.parse_args()


def docking_face_normal(face: str) -> tuple[float, float, float]:
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    if face == "pos_xy_to_neg_xy":
        return (inv_sqrt2, inv_sqrt2, 0.0)
    if face == "pos_x_neg_y_to_neg_x_pos_y":
        return (inv_sqrt2, -inv_sqrt2, 0.0)
    raise ValueError(f"Unsupported docking face: {face}")


def pair_start_positions(args: argparse.Namespace) -> dict[str, tuple[float, float, float]]:
    normal = docking_face_normal(args.docking_face)
    center_spacing = 2.0 * args.square_face_support + args.gap
    center_offset = 0.5 * center_spacing
    return {
        "drone_a": (-center_offset * normal[0], -center_offset * normal[1], args.height),
        "drone_b": (center_offset * normal[0], center_offset * normal[1], args.height),
    }


class SpaceThrustPairWindow:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.space_down = False
        self.reset_requested = False
        self.running = True

        self.root = tk.Tk()
        self.root.title("Two-Drone Space Thrust Test")
        self.root.geometry("520x210")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.status = tk.StringVar(value="Space released: thrust off")
        self.telemetry = tk.StringVar(value="zA=0.000 m  zB=0.000 m    rpmA=0  rpmB=0")

        tk.Label(self.root, text="Hold Space to thrust both drones", font=("Segoe UI", 16, "bold")).pack(pady=(18, 8))
        tk.Label(self.root, textvariable=self.status, font=("Segoe UI", 12)).pack()
        tk.Label(self.root, textvariable=self.telemetry, font=("Consolas", 10)).pack(pady=(10, 4))
        tk.Label(self.root, text="R: reset pair    Esc/Q: quit", font=("Segoe UI", 9)).pack(pady=(8, 0))

        self.root.bind("<KeyPress-space>", self.on_space_press)
        self.root.bind("<KeyRelease-space>", self.on_space_release)
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

    def on_reset(self, _event) -> None:
        self.reset_requested = True

    def on_quit(self, _event) -> None:
        self.close()

    def close(self) -> None:
        self.running = False

    def update(self, sample_a: dict[str, object] | None = None, sample_b: dict[str, object] | None = None) -> None:
        thrust_scale = self.args.space_thrust_scale if self.space_down else self.args.idle_thrust_scale
        thrust_per_drone = thrust_scale * self.args.mass * flight.G
        total_thrust = 2.0 * thrust_per_drone
        if self.space_down:
            self.status.set(
                f"Space pressed: {thrust_per_drone:.3f} N per drone, {total_thrust:.3f} N total ({thrust_scale:.2f}*mg each)"
            )
        else:
            self.status.set(f"Space released: {thrust_per_drone:.3f} N per drone")

        if sample_a and sample_b:
            pos_a = sample_a["pos"]
            pos_b = sample_b["pos"]
            rpm_a = rad_s_to_rpm(sum(sample_a["omega"]) / 4.0)
            rpm_b = rad_s_to_rpm(sum(sample_b["omega"]) / 4.0)
            self.telemetry.set(
                f"zA={pos_a[2]:.3f} m  zB={pos_b[2]:.3f} m    avgRPM A={rpm_a:.0f}  B={rpm_b:.0f}"
            )
        self.root.update_idletasks()
        self.root.update()


def rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 60.0 / (2.0 * math.pi)


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


def get_drone_handles(sim, label: str, start_position: tuple[float, float, float]) -> DroneHandles:
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
            "Run the pair generator or rerun this controller through the pair launcher."
        ) from exc


def reset_pair(sim, drones: tuple[DroneHandles, DroneHandles]) -> None:
    for drone in drones:
        reset_drone(sim, drone)


def main() -> int:
    args = parse_args()
    client, sim = flight.connect(args)
    drone_a, drone_b = resolve_pair_scene(sim, args)
    flight.set_time_step(sim, args.time_step)
    drones = (drone_a, drone_b)
    if args.reset_state:
        reset_pair(sim, drones)

    hover_omega = math.sqrt((args.mass * flight.G / 4.0) / (args.max_thrust / args.max_motor_speed**2))
    mixer = flight.motor_mixer(args.yaw_drag_arm)
    window = SpaceThrustPairWindow(args)

    print("Two-drone manual thrust test:")
    print("  focus the Two-Drone Space Thrust Test window")
    print("  hold Space -> same collective thrust on both drones")
    print("  release Space -> thrust off")
    print(f"  hover speed reference per drone: {hover_omega:.0f} rad/s ({rad_s_to_rpm(hover_omega):.0f} rpm) per motor")
    logger = telemetry.CsvTelemetryLogger(args.log_csv, "space_thrust_pair", args.log_sample_period)

    client.setStepping(True)
    sim.startSimulation()
    next_log = 0.0
    sim_start = sim.getSimulationTime()

    try:
        while window.running:
            loop_wall = time.time()
            thrust_scale = args.space_thrust_scale if window.space_down else args.idle_thrust_scale
            set_controller_command(args, thrust_scale * args.mass * flight.G)

            if window.reset_requested:
                reset_pair(sim, drones)
                window.reset_requested = False

            sample_a = flight.controller_step(sim, drone_a.body, drone_a.joints, drone_a.state, mixer, args)
            sample_b = flight.controller_step(sim, drone_b.body, drone_b.joints, drone_b.state, mixer, args)
            sim_time = float(sample_a["time"]) - sim_start
            logger.write(
                float(sample_a["time"]),
                telemetry.merge_samples(
                    ("a", sample_a),
                    ("b", sample_b),
                    extra={
                        "space_down": window.space_down,
                        "thrust_scale": thrust_scale,
                        "thrust_per_drone": args.thrust,
                        "total_commanded_thrust": 2.0 * args.thrust,
                    },
                ),
            )
            client.step()
            window.update(sample_a, sample_b)

            if sim_time >= next_log:
                pos_a = sample_a["pos"]
                pos_b = sample_b["pos"]
                omega_a = sample_a["omega"]
                omega_b = sample_b["omega"]
                print(
                    f"t={sim_time:5.2f}s thrust_each={args.thrust: .3f}N "
                    f"zA={pos_a[2]: .3f}m zB={pos_b[2]: .3f}m "
                    f"omegaA=[{omega_a[0]:.0f},{omega_a[1]:.0f},{omega_a[2]:.0f},{omega_a[3]:.0f}] "
                    f"omegaB=[{omega_b[0]:.0f},{omega_b[1]:.0f},{omega_b[2]:.0f},{omega_b[3]:.0f}]"
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
