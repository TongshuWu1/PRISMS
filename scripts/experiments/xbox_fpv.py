#!/usr/bin/env python3
"""Xbox/USB gamepad FPV-style rate controller for the CoppeliaSim drone.

This is acro/rate mode:

    left stick Y      -> collective thrust around hover
    left stick X      -> yaw rate command
    right stick X     -> roll rate command
    right stick Y     -> pitch rate command

The script reads the gamepad in Python and sends T, p_cmd, q_cmd, r_cmd into
the existing low-level controller. CoppeliaSim remains the physics plant.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

try:
    import pygame
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing pygame. Install requirements first:\n"
        "  python -m pip install -r requirements.txt"
    ) from exc

from controller.common import telemetry  # noqa: E402
from controller.low_level import body_rate as flight  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fly the CoppeliaSim drone with an Xbox/gamepad controller.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=int, default=20)
    parser.add_argument("--scene", default=str(flight.SCENE_PATH))
    parser.add_argument("--load-scene", action="store_true", help="Load the generated scene before running.")
    parser.add_argument("--list-controllers", action="store_true")
    parser.add_argument("--joystick-index", type=int, default=0)

    parser.add_argument("--duration", type=float, default=0.0, help="Duration [s]. Use 0 to run until quit.")
    parser.add_argument("--time-step", type=float, default=0.005)
    parser.add_argument("--start-height", type=float, default=0.5)
    parser.add_argument("--reset-state", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--mass", type=float, default=0.060)
    parser.add_argument("--max-motor-speed", type=float, default=2600.0)
    parser.add_argument("--max-thrust", type=float, default=0.294, help="Per-motor thrust limit [N].")
    parser.add_argument("--yaw-drag-arm", type=float, default=0.006)

    parser.add_argument("--min-thrust-scale", type=float, default=0.0, help="Minimum thrust as a multiple of mass*g.")
    parser.add_argument("--hover-thrust-scale", type=float, default=1.0, help="Centered throttle thrust scale.")
    parser.add_argument("--max-thrust-scale", type=float, default=2.0, help="Maximum thrust as a multiple of mass*g.")
    parser.add_argument("--throttle-mode", choices=("left-stick", "right-trigger"), default="left-stick")

    parser.add_argument("--max-roll-rate-deg", type=float, default=180.0)
    parser.add_argument("--max-pitch-rate-deg", type=float, default=180.0)
    parser.add_argument("--max-yaw-rate-deg", type=float, default=120.0)
    parser.add_argument("--deadband", type=float, default=0.08)

    parser.add_argument("--axis-left-x", type=int, default=0)
    parser.add_argument("--axis-left-y", type=int, default=1)
    parser.add_argument("--axis-right-x", type=int, default=2)
    parser.add_argument("--axis-right-y", type=int, default=3)
    parser.add_argument("--axis-right-trigger", type=int, default=5)

    parser.add_argument("--button-arm", type=int, default=0, help="Default Xbox A button.")
    parser.add_argument("--button-reset", type=int, default=1, help="Default Xbox B button.")
    parser.add_argument("--button-quit", type=int, default=7, help="Default Xbox Start button.")

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


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) <= deadband:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadband) / (1.0 - deadband)


def safe_axis(joystick: pygame.joystick.Joystick, index: int, default: float = 0.0) -> float:
    if index < 0 or index >= joystick.get_numaxes():
        return default
    return float(joystick.get_axis(index))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def init_pygame(args: argparse.Namespace):
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()

    if args.list_controllers:
        if count == 0:
            print("No gamepad detected.")
        for index in range(count):
            joystick = pygame.joystick.Joystick(index)
            joystick.init()
            print(
                f"{index}: {joystick.get_name()} "
                f"axes={joystick.get_numaxes()} buttons={joystick.get_numbuttons()}"
            )
        raise SystemExit(0)

    if count == 0:
        raise SystemExit(
            "No Xbox/gamepad controller detected. Connect it, verify Windows sees it, then run again.\n"
            "You can check detection with:\n"
            "  scripts\\launchers\\run_xbox_fpv_controller.py from PyCharm, using the list-controllers option"
        )
    if args.joystick_index >= count:
        raise SystemExit(f"Joystick index {args.joystick_index} not found. Detected {count} controller(s).")

    joystick = pygame.joystick.Joystick(args.joystick_index)
    joystick.init()
    screen = pygame.display.set_mode((720, 340))
    pygame.display.set_caption("Xbox FPV Rate Controller")
    return joystick, screen, pygame.font.SysFont("consolas", 18), pygame.time.Clock()


def thrust_scale_from_gamepad(args: argparse.Namespace, joystick: pygame.joystick.Joystick) -> float:
    if args.throttle_mode == "right-trigger":
        raw = safe_axis(joystick, args.axis_right_trigger, -1.0)
        trigger = clamp((raw + 1.0) * 0.5, 0.0, 1.0)
        return args.min_thrust_scale + trigger * (args.max_thrust_scale - args.min_thrust_scale)

    throttle_stick = apply_deadband(-safe_axis(joystick, args.axis_left_y), args.deadband)
    if throttle_stick >= 0.0:
        return args.hover_thrust_scale + throttle_stick * (args.max_thrust_scale - args.hover_thrust_scale)
    return args.hover_thrust_scale + throttle_stick * (args.hover_thrust_scale - args.min_thrust_scale)


def command_from_gamepad(args: argparse.Namespace, joystick: pygame.joystick.Joystick, armed: bool) -> dict[str, float]:
    if not armed:
        return {"thrust": 0.0, "p_cmd": 0.0, "q_cmd": 0.0, "r_cmd": 0.0, "thrust_scale": 0.0}

    roll_input = apply_deadband(safe_axis(joystick, args.axis_right_x), args.deadband)
    pitch_input = apply_deadband(-safe_axis(joystick, args.axis_right_y), args.deadband)
    yaw_input = apply_deadband(safe_axis(joystick, args.axis_left_x), args.deadband)
    thrust_scale = thrust_scale_from_gamepad(args, joystick)

    return {
        "thrust": thrust_scale * args.mass * flight.G,
        "p_cmd": roll_input * math.radians(args.max_roll_rate_deg),
        "q_cmd": pitch_input * math.radians(args.max_pitch_rate_deg),
        "r_cmd": yaw_input * math.radians(args.max_yaw_rate_deg),
        "thrust_scale": thrust_scale,
    }


def set_controller_command(args: argparse.Namespace, command: dict[str, float]) -> None:
    args.thrust = command["thrust"]
    args.p_cmd = command["p_cmd"]
    args.q_cmd = command["q_cmd"]
    args.r_cmd = command["r_cmd"]
    args.takeoff_pulse = False
    args.pulse_scale = 1.0
    args.pulse_duration = 0.0


def draw_screen(
    screen,
    font,
    joystick,
    armed: bool,
    command: dict[str, float],
    sample: dict[str, object] | None,
) -> None:
    screen.fill((18, 22, 28))
    lines = [
        f"Controller: {joystick.get_name()}",
        f"State: {'ARMED' if armed else 'DISARMED'}    A: arm/disarm    B: reset    Start/Esc/Q: quit",
        "Mode: FPV acro/rate    left Y: throttle    left X: yaw    right X/Y: roll/pitch",
        f"Command: T={command['thrust']:.3f} N ({command['thrust_scale']:.2f}*mg)  "
        f"p={command['p_cmd']:.2f} q={command['q_cmd']:.2f} r={command['r_cmd']:.2f} rad/s",
    ]
    if sample:
        pos = sample["pos"]
        rate = sample["rate"]
        omega = sample["omega"]
        lines.extend(
            [
                f"State: z={pos[2]:.3f} m    pqr=[{rate[0]: .2f}, {rate[1]: .2f}, {rate[2]: .2f}] rad/s",
                f"Motor omega: [{omega[0]:.0f}, {omega[1]:.0f}, {omega[2]:.0f}, {omega[3]:.0f}] rad/s",
            ]
        )
    y = 22
    for line in lines:
        color = (255, 90, 70) if "DISARMED" in line else (225, 230, 235)
        screen.blit(font.render(line, True, color), (22, y))
        y += 42
    pygame.display.flip()


def reset_controller_state(state: flight.ControllerState) -> None:
    state.rate_integral = [0.0, 0.0, 0.0]
    state.motor_speed = [0.0, 0.0, 0.0, 0.0]
    state.last_time = None


def main() -> int:
    args = parse_args()
    joystick, screen, font, clock = init_pygame(args)

    client, sim = flight.connect(args)
    body, joints = flight.resolve_scene(sim, args)
    flight.set_time_step(sim, args.time_step)
    if args.reset_state:
        flight.reset_body(sim, body, args.start_height)

    state = flight.ControllerState(
        rate_integral=[0.0, 0.0, 0.0],
        motor_speed=[0.0, 0.0, 0.0, 0.0],
        prop_phase=[0.0, 0.0, 0.0, 0.0],
    )
    mixer = flight.motor_mixer(args.yaw_drag_arm)

    print("Xbox FPV rate controller:")
    print("  A toggles arm/disarm")
    print("  B resets drone")
    print("  Start, Esc, or Q quits")
    print("  left Y throttle, left X yaw, right X/Y roll/pitch")
    logger = telemetry.CsvTelemetryLogger(args.log_csv, "xbox_fpv", args.log_sample_period)

    armed = False
    running = True
    next_log = 0.0
    sim_start = 0.0
    latest_sample = None

    client.setStepping(True)
    sim.startSimulation()
    try:
        while running:
            loop_wall = time.time()
            reset_requested = False

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                    elif event.key == pygame.K_r:
                        reset_requested = True
                    elif event.key == pygame.K_a:
                        armed = not armed
                        if not armed:
                            reset_controller_state(state)
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == args.button_arm:
                        armed = not armed
                        if not armed:
                            reset_controller_state(state)
                    elif event.button == args.button_reset:
                        reset_requested = True
                    elif event.button == args.button_quit:
                        running = False

            if reset_requested:
                flight.reset_body(sim, body, args.start_height)
                reset_controller_state(state)

            command = command_from_gamepad(args, joystick, armed)
            set_controller_command(args, command)
            latest_sample = flight.controller_step(sim, body, joints, state, mixer, args)
            logger.write(
                float(latest_sample["time"]),
                telemetry.merge_samples(
                    ("low", latest_sample),
                    extra={
                        "armed": armed,
                        "cmd_thrust": command["thrust"],
                        "cmd_p": command["p_cmd"],
                        "cmd_q": command["q_cmd"],
                        "cmd_r": command["r_cmd"],
                        "cmd_thrust_scale": command["thrust_scale"],
                    },
                ),
            )
            client.step()
            draw_screen(screen, font, joystick, armed, command, latest_sample)

            sim_time = float(latest_sample["time"]) - sim_start
            if sim_time >= next_log:
                pos = latest_sample["pos"]
                rate = latest_sample["rate"]
                print(
                    f"t={sim_time:5.2f}s {'ARM' if armed else 'DIS'} "
                    f"T={command['thrust']:.3f}N z={pos[2]: .3f}m "
                    f"pqr=[{rate[0]: .2f},{rate[1]: .2f},{rate[2]: .2f}]"
                )
                next_log += args.log_period

            if args.duration > 0.0 and sim_time >= args.duration:
                break

            elapsed = time.time() - loop_wall
            if elapsed < args.time_step:
                time.sleep(args.time_step - elapsed)
            clock.tick(int(round(1.0 / args.time_step)))
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        logger.close()
        if args.stop_on_exit:
            sim.stopSimulation(True)
            while sim.getSimulationState() != sim.simulation_stopped:
                time.sleep(0.05)
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
