#!/usr/bin/env python3
"""High-level position controller with a small target-command UI.

This avoids using CoppeliaSim's move tool. The UI updates the blue target
sphere, and the high-level controller follows that commanded target.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import tkinter as tk
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.common import telemetry  # noqa: E402
from controller.high_level import position  # noqa: E402
from controller.low_level import body_rate as flight  # noqa: E402
from ui.plot_widgets import MotorRpmBars, RPM_PER_RAD_PER_SEC, StripChart, vector_norm  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run position control with a small target-command UI.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--scene", default=str(flight.SCENE_PATH))
    parser.add_argument("--load-scene", action="store_true", help="Load the generated scene before running.")

    parser.add_argument("--duration", type=float, default=0.0, help="Duration [s]. Use 0 to run until the window closes.")
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
    parser.add_argument("--ui-step", type=float, default=0.05)

    parser.add_argument("--mass", type=float, default=0.060)
    parser.add_argument("--max-motor-speed", type=float, default=2600.0)
    parser.add_argument("--max-thrust", type=float, default=0.294, help="Per-motor thrust limit [N].")
    parser.add_argument("--yaw-drag-arm", type=float, default=0.006)

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
    parser.add_argument("--live-plots", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--plot-window", type=float, default=8.0, help="Live plot time window [s].")
    parser.add_argument("--plot-update-period", type=float, default=0.05, help="Live plot update period [s].")
    telemetry.add_logging_args(parser)
    return parser.parse_args()


class PositionTargetUI:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.running = True
        self.reset_drone_requested = False
        self.target_to_drone_requested = False

        self.root = tk.Tk()
        self.root.title("Position Target Controller")
        self.root.geometry("1180x850")
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.x = tk.DoubleVar(value=args.target_x)
        self.y = tk.DoubleVar(value=args.target_y)
        self.z = tk.DoubleVar(value=args.target_z)
        self.yaw_deg = tk.DoubleVar(value=math.degrees(args.target_yaw))
        self.step = tk.DoubleVar(value=args.ui_step)
        self.telemetry = tk.StringVar(value="drone=[0.00, 0.00, 0.00]  error=[0.00, 0.00, 0.00]")
        self.command = tk.StringVar(value="T=0.000 N  pqr_cmd=[0.00, 0.00, 0.00]")
        self.rpm_text = tk.StringVar(value="rpm=[0, 0, 0, 0]")
        self.last_plot_update = -1e9
        self.position_plot: StripChart | None = None
        self.error_plot: StripChart | None = None
        self.pid_plot: StripChart | None = None
        self.rate_plot: StripChart | None = None
        self.rpm_plot: StripChart | None = None
        self.rpm_bars: MotorRpmBars | None = None

        main = tk.Frame(self.root)
        main.pack(fill="both", expand=True)
        controls = tk.Frame(main, width=625)
        controls.pack(side="left", fill="y", padx=(10, 4), pady=8)
        plots = tk.Frame(main)
        plots.pack(side="right", fill="both", expand=True, padx=(4, 10), pady=8)

        tk.Label(controls, text="Position Target Controller", font=("Segoe UI", 16, "bold")).pack(pady=(4, 4))
        tk.Label(controls, text="Move the target here. The blue sphere in CoppeliaSim follows this UI.").pack()

        frame = tk.Frame(controls)
        frame.pack(pady=(10, 4), fill="x", padx=14)
        self._axis_control(frame, "X [m]", self.x, -1.5, 1.5, 0)
        self._axis_control(frame, "Y [m]", self.y, -1.5, 1.5, 1)
        self._axis_control(frame, "Z [m]", self.z, 0.05, 2.0, 2)
        self._axis_control(frame, "Yaw [deg]", self.yaw_deg, -180.0, 180.0, 3)

        nudge = tk.Frame(controls)
        nudge.pack(pady=8)
        tk.Label(nudge, text="Step [m/deg]").grid(row=0, column=0, padx=4)
        tk.Spinbox(nudge, from_=0.01, to=0.50, increment=0.01, textvariable=self.step, width=6).grid(row=0, column=1, padx=4)
        buttons = [
            ("X-", lambda: self.nudge(dx=-self.step.get())),
            ("X+", lambda: self.nudge(dx=self.step.get())),
            ("Y-", lambda: self.nudge(dy=-self.step.get())),
            ("Y+", lambda: self.nudge(dy=self.step.get())),
            ("Z-", lambda: self.nudge(dz=-self.step.get())),
            ("Z+", lambda: self.nudge(dz=self.step.get())),
            ("Yaw-", lambda: self.nudge(dyaw=-self.step.get() * 10.0)),
            ("Yaw+", lambda: self.nudge(dyaw=self.step.get() * 10.0)),
        ]
        for index, (label, command) in enumerate(buttons):
            tk.Button(nudge, text=label, width=6, command=command).grid(row=0, column=index + 2, padx=2)

        actions = tk.Frame(controls)
        actions.pack(pady=6)
        tk.Button(actions, text="Target = Drone", width=16, command=self.request_target_to_drone).grid(row=0, column=0, padx=4)
        tk.Button(actions, text="Reset Target", width=16, command=self.reset_target).grid(row=0, column=1, padx=4)
        tk.Button(actions, text="Reset Drone", width=16, command=self.request_reset_drone).grid(row=0, column=2, padx=4)
        tk.Button(actions, text="Quit", width=10, command=self.close).grid(row=0, column=3, padx=4)

        tk.Label(controls, textvariable=self.telemetry, font=("Consolas", 10)).pack(pady=(10, 2))
        tk.Label(controls, textvariable=self.command, font=("Consolas", 10)).pack()
        tk.Label(controls, textvariable=self.rpm_text, font=("Consolas", 10)).pack(pady=(2, 0))
        tk.Label(
            controls,
            text="Keyboard: W/S Y, A/D X, Q/E Z, Z/C yaw, F target=drone, R reset drone, Esc quit",
            font=("Segoe UI", 9),
        ).pack(pady=(10, 0))

        if args.live_plots:
            self.position_plot = StripChart(
                plots,
                "XYZ Actual vs Desired [m]",
                [
                    ("x", "x", "#e15759"),
                    ("xd", "xd", "#ff9d9a"),
                    ("y", "y", "#4e79a7"),
                    ("yd", "yd", "#9ecae9"),
                    ("z", "z", "#59a14f"),
                    ("zd", "zd", "#9bd489"),
                ],
                args.plot_window,
                unit="m",
            )
            self.position_plot.pack(fill="both", expand=True, pady=(0, 5))
            self.error_plot = StripChart(
                plots,
                "Position Error [m]",
                [("ex", "ex", "#e15759"), ("ey", "ey", "#4e79a7"), ("ez", "ez", "#59a14f")],
                args.plot_window,
                unit="m",
                symmetric=True,
            )
            self.error_plot.pack(fill="both", expand=True, pady=(0, 5))
            self.pid_plot = StripChart(
                plots,
                "Position PID Accel Norm [m/s^2]",
                [("P", "p", "#e15759"), ("I", "i", "#f28e2b"), ("D", "d", "#4e79a7"), ("cmd", "cmd", "#59a14f")],
                args.plot_window,
                unit="",
            )
            self.pid_plot.pack(fill="both", expand=True, pady=5)
            self.rate_plot = StripChart(
                plots,
                "Commanded Body Rates [deg/s]",
                [("p", "p", "#e15759"), ("q", "q", "#4e79a7"), ("r", "r", "#59a14f")],
                args.plot_window,
                symmetric=True,
            )
            self.rate_plot.pack(fill="both", expand=True, pady=5)
            max_rpm = args.max_motor_speed * RPM_PER_RAD_PER_SEC
            self.rpm_plot = StripChart(
                plots,
                "Motor RPM",
                [("m0", "m0", "#59a14f"), ("m1", "m1", "#4e79a7"), ("m2", "m2", "#f28e2b"), ("m3", "m3", "#e15759")],
                args.plot_window,
                fixed_range=(0.0, max_rpm),
            )
            self.rpm_plot.pack(fill="both", expand=True, pady=5)
            self.rpm_bars = MotorRpmBars(plots, args.max_motor_speed)
            self.rpm_bars.pack(fill="x", pady=(5, 0))

        self._bind_keys()
        self.root.after(100, self.root.focus_force)

    def _axis_control(self, parent: tk.Frame, label: str, variable: tk.DoubleVar, minimum: float, maximum: float, row: int) -> None:
        tk.Label(parent, text=label, width=10, anchor="w").grid(row=row, column=0, sticky="w", pady=3)
        tk.Scale(
            parent,
            from_=minimum,
            to=maximum,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            variable=variable,
            length=420,
        ).grid(row=row, column=1, sticky="ew", padx=6)
        tk.Entry(parent, textvariable=variable, width=8).grid(row=row, column=2, padx=4)

    def _bind_keys(self) -> None:
        bindings = {
            "<KeyPress-a>": lambda _event: self.nudge(dx=-self.step.get()),
            "<KeyPress-d>": lambda _event: self.nudge(dx=self.step.get()),
            "<KeyPress-s>": lambda _event: self.nudge(dy=-self.step.get()),
            "<KeyPress-w>": lambda _event: self.nudge(dy=self.step.get()),
            "<KeyPress-e>": lambda _event: self.nudge(dz=self.step.get()),
            "<KeyPress-q>": lambda _event: self.nudge(dz=-self.step.get()),
            "<KeyPress-z>": lambda _event: self.nudge(dyaw=-self.step.get() * 10.0),
            "<KeyPress-c>": lambda _event: self.nudge(dyaw=self.step.get() * 10.0),
            "<KeyPress-f>": lambda _event: self.request_target_to_drone(),
            "<KeyPress-r>": lambda _event: self.request_reset_drone(),
            "<Escape>": lambda _event: self.close(),
        }
        for key, command in bindings.items():
            self.root.bind(key, command)
            self.root.bind(key.replace("KeyPress-", "KeyPress-").replace(key[-2], key[-2].upper()) if len(key) > 12 else key, command)

    def nudge(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, dyaw: float = 0.0) -> None:
        self.x.set(round(self.x.get() + dx, 3))
        self.y.set(round(self.y.get() + dy, 3))
        self.z.set(round(max(0.05, self.z.get() + dz), 3))
        yaw = self.yaw_deg.get() + dyaw
        while yaw > 180.0:
            yaw -= 360.0
        while yaw < -180.0:
            yaw += 360.0
        self.yaw_deg.set(round(yaw, 2))

    def request_reset_drone(self) -> None:
        self.reset_drone_requested = True

    def request_target_to_drone(self) -> None:
        self.target_to_drone_requested = True

    def reset_target(self) -> None:
        self.x.set(0.0)
        self.y.set(0.0)
        self.z.set(0.75)
        self.yaw_deg.set(0.0)

    def reset_plots(self) -> None:
        self.last_plot_update = -1e9
        for plot in (self.position_plot, self.error_plot, self.pid_plot, self.rate_plot, self.rpm_plot):
            if plot:
                plot.reset()
        if self.rpm_bars:
            self.rpm_bars.reset()

    def close(self) -> None:
        self.running = False

    def target(self) -> tuple[list[float], float]:
        return [self.x.get(), self.y.get(), self.z.get()], math.radians(self.yaw_deg.get())

    def set_target_from_drone(self, drone_position: list[float]) -> None:
        self.x.set(round(drone_position[0], 3))
        self.y.set(round(drone_position[1], 3))
        self.z.set(round(max(0.05, drone_position[2]), 3))

    def update(self, high_sample: dict[str, object] | None, low_sample: dict[str, object] | None) -> None:
        if high_sample:
            pos = high_sample["pos"]
            target = high_sample["target"]
            err = high_sample["pos_error"]
            self.telemetry.set(
                f"true=[{pos[0]: .2f}, {pos[1]: .2f}, {pos[2]: .2f}]  "
                f"des=[{target[0]: .2f}, {target[1]: .2f}, {target[2]: .2f}]  "
                f"error=[{err[0]: .2f}, {err[1]: .2f}, {err[2]: .2f}]"
            )
            rate_cmd = high_sample["rate_cmd"]
            self.command.set(
                f"T={high_sample['thrust_cmd']:.3f} N  "
                f"pqr_cmd=[{rate_cmd[0]: .2f}, {rate_cmd[1]: .2f}, {rate_cmd[2]: .2f}]"
            )
        if low_sample:
            rpm = [value * RPM_PER_RAD_PER_SEC for value in low_sample["omega"]]
            self.rpm_text.set(f"rpm=[{rpm[0]:.0f}, {rpm[1]:.0f}, {rpm[2]:.0f}, {rpm[3]:.0f}]")

        if self.args.live_plots and high_sample and low_sample:
            sim_time = float(high_sample["time"])
            if sim_time - self.last_plot_update >= self.args.plot_update_period:
                self.last_plot_update = sim_time
                pos = high_sample["pos"]
                target = high_sample["target"]
                err = high_sample["pos_error"]
                pid_p = high_sample["pid_accel_p"]
                pid_i = high_sample["pid_accel_i"]
                pid_d = high_sample["pid_accel_d"]
                accel_cmd = high_sample["accel_cmd"]
                rate_cmd = high_sample["rate_cmd"]
                rpm = [value * RPM_PER_RAD_PER_SEC for value in low_sample["omega"]]

                if self.position_plot:
                    self.position_plot.add_sample(
                        sim_time,
                        {
                            "x": pos[0],
                            "xd": target[0],
                            "y": pos[1],
                            "yd": target[1],
                            "z": pos[2],
                            "zd": target[2],
                        },
                    )
                if self.error_plot:
                    self.error_plot.add_sample(sim_time, {"ex": err[0], "ey": err[1], "ez": err[2]})
                if self.pid_plot:
                    self.pid_plot.add_sample(
                        sim_time,
                        {
                            "p": vector_norm(pid_p),
                            "i": vector_norm(pid_i),
                            "d": vector_norm(pid_d),
                            "cmd": vector_norm(accel_cmd),
                        },
                    )
                if self.rate_plot:
                    self.rate_plot.add_sample(
                        sim_time,
                        {
                            "p": math.degrees(rate_cmd[0]),
                            "q": math.degrees(rate_cmd[1]),
                            "r": math.degrees(rate_cmd[2]),
                        },
                    )
                if self.rpm_plot:
                    self.rpm_plot.add_sample(
                        sim_time,
                        {"m0": rpm[0], "m1": rpm[1], "m2": rpm[2], "m3": rpm[3]},
                    )
                if self.rpm_bars:
                    self.rpm_bars.update(rpm)
        self.root.update_idletasks()
        self.root.update()


def reset_controller_states(
    sim,
    body: int,
    args: argparse.Namespace,
    high_state: position.PositionControllerState,
    low_state: flight.ControllerState,
) -> None:
    hover_omega = math.sqrt((args.mass * flight.G / 4.0) / (args.max_thrust / args.max_motor_speed**2))
    flight.reset_body(sim, body, args.start_height)
    high_state.pos_integral = [0.0, 0.0, 0.0]
    high_state.last_time = None
    low_state.rate_integral = [0.0, 0.0, 0.0]
    low_state.motor_speed = [hover_omega, hover_omega, hover_omega, hover_omega]
    low_state.last_time = None


def main() -> int:
    args = parse_args()
    client, sim = flight.connect(args)
    body, joints = flight.resolve_scene(sim, args)
    flight.set_time_step(sim, args.time_step)
    target_handle = position.create_or_update_target(sim, args)

    hover_omega = math.sqrt((args.mass * flight.G / 4.0) / (args.max_thrust / args.max_motor_speed**2))
    low_state = flight.ControllerState(
        rate_integral=[0.0, 0.0, 0.0],
        motor_speed=[hover_omega, hover_omega, hover_omega, hover_omega],
        prop_phase=[0.0, 0.0, 0.0, 0.0],
    )
    high_state = position.PositionControllerState(pos_integral=[0.0, 0.0, 0.0])
    if args.reset_state:
        reset_controller_states(sim, body, args, high_state, low_state)
    mixer = flight.motor_mixer(args.yaw_drag_arm)
    ui = PositionTargetUI(args)

    print("Position UI controller:")
    print("  use the Python UI to command the blue target sphere")
    print("  high-level output: T, p_cmd, q_cmd, r_cmd")
    print("  low-level output: four motor angular velocities")
    logger = telemetry.CsvTelemetryLogger(args.log_csv, "position_ui", args.log_sample_period)

    client.setStepping(True)
    sim.startSimulation()
    next_log = 0.0
    high_sample = None
    low_sample = None

    try:
        while ui.running:
            loop_wall = time.time()

            if ui.reset_drone_requested:
                reset_controller_states(sim, body, args, high_state, low_state)
                ui.reset_plots()
                ui.reset_drone_requested = False

            if ui.target_to_drone_requested:
                ui.set_target_from_drone(sim.getObjectPosition(body, -1))
                ui.target_to_drone_requested = False

            target, yaw = ui.target()
            args.target_x, args.target_y, args.target_z = target
            args.target_yaw = yaw
            if target_handle is not None:
                sim.setObjectPosition(target_handle, -1, target)

            high_sample = position.high_level_step(sim, body, target_handle, high_state, args)
            low_sample = flight.controller_step(sim, body, joints, low_state, mixer, args)
            logger.write(
                float(high_sample["time"]),
                telemetry.merge_samples(("high", high_sample), ("low", low_sample)),
            )
            client.step()
            ui.update(high_sample, low_sample)

            sim_time = float(high_sample["time"])
            if sim_time >= next_log:
                pos = high_sample["pos"]
                err = high_sample["pos_error"]
                target_now = high_sample["target"]
                print(
                    f"t={sim_time:5.2f}s "
                    f"target=[{target_now[0]: .2f},{target_now[1]: .2f},{target_now[2]: .2f}] "
                    f"pos=[{pos[0]: .2f},{pos[1]: .2f},{pos[2]: .2f}] "
                    f"err=[{err[0]: .2f},{err[1]: .2f},{err[2]: .2f}]"
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
        ui.running = False
        ui.root.destroy()
        if args.stop_on_exit:
            sim.stopSimulation(True)
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
