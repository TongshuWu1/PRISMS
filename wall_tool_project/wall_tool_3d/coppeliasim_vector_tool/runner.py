#!/usr/bin/env python3
"""Run staged validation of the clean CoppeliaSim vector-thrust plant."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent
WALL_TOOL_3D_ROOT = PACKAGE_DIR.parent
WALL_TOOL_PROJECT_ROOT = WALL_TOOL_3D_ROOT.parent
WALL_TOOL_2D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_2d"
for path in (WALL_TOOL_3D_ROOT, WALL_TOOL_2D_ROOT, WALL_TOOL_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cable_hybrid_controller.config import COVERAGE_CORNER_SPEED  # noqa: E402
from cable_hybrid_controller.controller import best_params, default_scenario  # noqa: E402

from . import remote, scene  # noqa: E402
from .controller import ExternalVectorThrustController  # noqa: E402
from .plant import CoppeliaVectorPlant  # noqa: E402
from .sensors import VectorToolSensorSuite  # noqa: E402
from .validation_plant import (  # noqa: E402
    datasheet_validation_profile,
    load_calibrated_validation_profile,
)


DEFAULT_OUTPUT = WALL_TOOL_3D_ROOT / "vector_thrust_runs"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the remade vector-thrust CoppeliaSim plant")
    parser.add_argument("--scenario", choices=("hover", "point", "turns", "mission"), default="hover")
    parser.add_argument("--camera", choices=("overview", "payload", "winch"), default="overview")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--connect-timeout", type=float, default=30.0)
    parser.add_argument("--coppeliasim-exe", type=Path, default=remote.DEFAULT_EXE)
    parser.add_argument("--launch", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--regenerate-scene", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--save-scene", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--strict", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--validation-profile",
        choices=("datasheet", "calibrated"),
        default="datasheet",
        help="independent plant parameter source; calibrated mode requires real identified data",
    )
    parser.add_argument(
        "--calibration-file",
        type=Path,
        default=None,
        help="strict JSON hardware profile; required with --validation-profile calibrated",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--log-every", type=float, default=2.0)
    return parser.parse_args(argv)


def scenario_definition(name: str, params):
    if name == "hover":
        return 12.0, ()
    if name == "point":
        return 60.0, ((0.90, 1.50),)
    if name == "turns":
        return 90.0, (
            (-1.20, 1.40),
            (1.20, 1.40),
            (1.20, 1.85),
            (-1.20, 1.85),
            (-1.20, 2.30),
            (1.20, 2.30),
        )
    scenario = default_scenario()
    return scenario.duration_s, scenario.targets


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[int(clamp_index(fraction * (len(ordered) - 1), len(ordered)))]


def clamp_index(value: float, length: int) -> int:
    return max(0, min(length - 1, int(value)))


def _summary(rows: list[dict[str, float]], params, scenario_name: str) -> dict[str, float | str | int]:
    if not rows:
        raise RuntimeError("CoppeliaSim run produced no samples")
    errors = [row["error_m"] for row in rows]
    speeds = [row["speed_m_s"] for row in rows]
    pitches = [abs(row["pitch_rad"]) for row in rows]
    y_drifts = [abs(row["y_drift_m"]) for row in rows]
    roll_yaw = [max(abs(row["roll_rad"]), abs(row["yaw_rad"])) for row in rows]
    tensions = [row["tension_N"] for row in rows]
    drum_tensions = [row["drum_tension_N"] for row in rows]
    estimation_errors = [row["estimation_error_m"] for row in rows]
    reel_encoder_errors = [abs(row["reel_encoder_error_m"]) for row in rows]
    solve_times = [row["solve_time_s"] for row in rows]
    return {
        "scenario": scenario_name,
        "duration_s": rows[-1]["t_s"],
        "samples": len(rows),
        "final_error_m": errors[-1],
        "rms_error_m": math.sqrt(sum(value * value for value in errors) / len(errors)),
        "p95_error_m": _percentile(errors, 0.95),
        "max_error_m": max(errors),
        "max_speed_m_s": max(speeds),
        "max_pitch_deg": math.degrees(max(pitches)),
        "max_wall_normal_drift_m": max(y_drifts),
        "max_roll_yaw_deg": math.degrees(max(roll_yaw)),
        "min_tension_N": min(tensions),
        "max_tension_N": max(tensions),
        "min_drum_tension_N": min(drum_tensions),
        "max_drum_tension_N": max(drum_tensions),
        "slack_fraction": sum(value < 0.05 for value in tensions) / len(tensions),
        "rms_estimation_error_m": math.sqrt(
            sum(value * value for value in estimation_errors) / len(estimation_errors)
        ),
        "p95_estimation_error_m": _percentile(estimation_errors, 0.95),
        "max_reel_encoder_error_m": max(reel_encoder_errors),
        "p95_solve_time_ms": 1000.0 * _percentile(solve_times, 0.95),
        "max_solve_time_ms": 1000.0 * max(solve_times),
        "deadline_miss_fraction": sum(value > params.mpc_control_period_s for value in solve_times) / len(solve_times),
    }


def _validate(summary: dict[str, float | str | int], params) -> None:
    failures: list[str] = []
    if float(summary["max_wall_normal_drift_m"]) > 0.025:
        failures.append(f"wall-normal drift {summary['max_wall_normal_drift_m']:.4f} m exceeds 0.025 m")
    if float(summary["max_roll_yaw_deg"]) > 5.0:
        failures.append(f"roll/yaw {summary['max_roll_yaw_deg']:.2f} deg exceeds 5 deg")
    pitch_limit_deg = min(5.0, math.degrees(params.mpc_attitude_limit_rad))
    if float(summary["max_pitch_deg"]) > pitch_limit_deg:
        failures.append(f"pitch {summary['max_pitch_deg']:.2f} deg exceeds {pitch_limit_deg:.2f} deg")
    if float(summary["max_tension_N"]) > params.max_spool_tension + 1e-6:
        failures.append("cable tension exceeded the physical limit")
    if float(summary["max_drum_tension_N"]) > params.max_spool_tension + 1e-6:
        failures.append("drum-side load-cell tension exceeded the physical limit")
    if float(summary["slack_fraction"]) > 0.0:
        failures.append(f"cable slack fraction is {summary['slack_fraction']:.6f}")
    if summary["scenario"] != "hover" and float(summary["max_error_m"]) > 0.12:
        failures.append(f"tracking error {summary['max_error_m']:.4f} m exceeds 0.12 m")
    if summary["scenario"] != "hover" and float(summary["rms_error_m"]) > 0.025:
        failures.append(f"RMS tracking error {summary['rms_error_m']:.4f} m exceeds 0.025 m")
    if summary["scenario"] != "hover" and float(summary["p95_error_m"]) > 0.050:
        failures.append(f"p95 tracking error {summary['p95_error_m']:.4f} m exceeds 0.050 m")
    if float(summary["p95_estimation_error_m"]) > 0.020:
        failures.append(
            f"p95 sensor-fusion error {summary['p95_estimation_error_m']:.4f} m exceeds 0.020 m"
        )
    duration_s = max(float(summary["duration_s"]), params.mpc_control_period_s)
    # A two-second smoke test has only a few dozen independent solves, so one
    # scheduling outlier should not numerically become a 4% systemic failure.
    # Permit at most two misses' worth of statistical resolution; long runs
    # converge to the research gate of 2%.
    deadline_miss_limit = max(0.02, 2.0 * params.mpc_control_period_s / duration_s)
    if float(summary["deadline_miss_fraction"]) > deadline_miss_limit:
        failures.append(
            f"solver deadline miss fraction {summary['deadline_miss_fraction']:.4f} "
            f"exceeds {deadline_miss_limit:.4f}"
        )
    if failures:
        raise RuntimeError("CoppeliaSim acceptance failed:\n- " + "\n- ".join(failures))


def run(args: argparse.Namespace) -> dict[str, float | str | int]:
    params = best_params()
    if args.validation_profile == "calibrated":
        if args.calibration_file is None:
            raise ValueError(
                "--validation-profile calibrated requires --calibration-file; "
                "no nominal fallback is allowed"
            )
        validation_profile = load_calibrated_validation_profile(args.calibration_file)
    else:
        if args.calibration_file is not None:
            raise ValueError(
                "--calibration-file is only valid with --validation-profile calibrated"
            )
        validation_profile = datasheet_validation_profile(params)
    print(
        f"Validation plant: {validation_profile.profile_name} "
        f"(calibrated={validation_profile.calibrated})",
        flush=True,
    )
    duration_default, targets = scenario_definition(args.scenario, params)
    duration = duration_default if args.duration is None else float(args.duration)
    if duration <= 0.0:
        raise ValueError("--duration must be positive")

    client, sim, launched_process = remote.connect_or_launch(
        args.host,
        args.port,
        args.connect_timeout,
        launch=args.launch,
        executable=args.coppeliasim_exe,
        headless=args.headless,
    )
    remote.stop_if_running(sim)
    if args.regenerate_scene:
        handles = scene.build_scene(sim, params)
        scene.configure_camera(sim, params, args.camera)
        if args.save_scene:
            scene.save_scene(sim, handles)
    else:
        if not scene.SCENE_PATH.exists():
            raise FileNotFoundError(f"generated scene is missing: {scene.SCENE_PATH}")
        sim.loadScene(str(scene.SCENE_PATH))
        handles = scene.resolve_handles(sim)
        scene.configure_camera(sim, params, args.camera)

    sim.setFloatParam(sim.floatparam_simulation_time_step, params.dt)
    plant = CoppeliaVectorPlant(sim, handles, params, validation_profile)
    initial_truth = plant.truth(0.0)
    sensors = VectorToolSensorSuite(params)
    initial_estimate = sensors.update(initial_truth, force=True)
    controller = ExternalVectorThrustController(params)
    if targets:
        if len(targets) == 1:
            controller.command_target(initial_estimate.payload_position_xz_m, targets[0])
        else:
            controller.command_corner_smooth_path(
                initial_estimate.payload_position_xz_m,
                targets,
                COVERAGE_CORNER_SPEED,
            )

    client.setStepping(True)
    sim.startSimulation()
    rows: list[dict[str, float]] = []
    command = None
    last_controller_sample_s = -math.inf
    next_log_s = 0.0
    start_y = initial_truth.position_world_m[1]
    steps = max(1, int(math.ceil(duration / params.dt)))
    run_error: BaseException | None = None
    try:
        for _ in range(steps):
            truth = plant.truth()
            estimate = sensors.update(truth)
            # The hardware estimator publishes at 100 Hz while CoppeliaSim
            # integrates at 200 Hz.  Run the digital controller only for a
            # fresh sensor frame and hold all five actuator commands between
            # frames, as the real motor/servo drives would.
            if command is None or estimate.timestamp_s > last_controller_sample_s + 1e-12:
                command = controller.step(estimate)
                last_controller_sample_s = estimate.timestamp_s
            sim.setObjectPosition(
                handles.target,
                -1,
                [command.reference_position_xz_m[0], -0.012, command.reference_position_xz_m[1]],
            )
            plant.apply(command, truth)
            error = math.hypot(
                truth.position_world_m[0] - command.reference_position_xz_m[0],
                truth.position_world_m[2] - command.reference_position_xz_m[1],
            )
            rows.append({
                "t_s": truth.timestamp_s,
                "error_m": error,
                "speed_m_s": math.hypot(
                    truth.linear_velocity_world_m_s[0],
                    truth.linear_velocity_world_m_s[2],
                ),
                "pitch_rad": -truth.orientation_world_rad[1],
                "roll_rad": truth.orientation_world_rad[0],
                "yaw_rad": truth.orientation_world_rad[2],
                "y_drift_m": truth.position_world_m[1] - start_y,
                "tension_N": truth.cable_tension_N,
                "drum_tension_N": truth.measured_load_cell_tension_N,
                "estimation_error_m": math.hypot(
                    truth.position_world_m[0] - estimate.payload_position_xz_m[0],
                    truth.position_world_m[2] - estimate.payload_position_xz_m[1],
                ),
                "reel_encoder_error_m": truth.measured_reel_length_m - truth.reel_length_m,
                "left_servo_rad": truth.left_servo_angle_rad,
                "right_servo_rad": truth.right_servo_angle_rad,
                "solve_time_s": command.solver_time_s,
            })
            if truth.timestamp_s + 1e-12 >= next_log_s:
                print(
                    f"t={truth.timestamp_s:7.2f}s error={1000.0 * error:6.1f}mm "
                    f"pos=({truth.position_world_m[0]:+.4f},{truth.position_world_m[2]:.4f}) "
                    f"est=({estimate.payload_position_xz_m[0]:+.4f},{estimate.payload_position_xz_m[1]:.4f}) "
                    f"tension={truth.cable_tension_N:4.2f}N "
                    f"pitch={math.degrees(-truth.orientation_world_rad[1]):+5.2f}deg "
                    f"y={truth.position_world_m[1] - start_y:+.4f}m "
                    f"cmd=({command.left_thrust_N:.3f},{command.right_thrust_N:.3f},"
                    f"{command.reel_velocity_m_s:+.4f}) "
                    f"servo={math.degrees(truth.left_servo_angle_rad):+5.1f}/"
                    f"{math.degrees(truth.right_servo_angle_rad):+5.1f}deg",
                    flush=True,
                )
                next_log_s += max(args.log_every, params.dt)
            client.step()
    except BaseException as exc:
        run_error = exc
        raise
    finally:
        try:
            remote.stop_if_running(sim)
            client.setStepping(False)
            if args.headless and launched_process is not None:
                try:
                    sim.quitSimulator()
                except Exception:
                    # Closing the server commonly drops the reply that
                    # acknowledges quitSimulator. Process exit is verified
                    # below, so a lost reply is not treated as success.
                    pass
                try:
                    launched_process.wait(timeout=10.0)
                except Exception as exc:
                    raise RuntimeError(
                        "batch-launched headless CoppeliaSim did not exit within 10 seconds"
                    ) from exc
        except Exception as cleanup_error:
            if run_error is None:
                raise
            # Keep the original solver/transport failure as the primary
            # exception. A dead ZMQ REQ socket cannot service cleanup calls.
            print(
                f"CoppeliaSim cleanup also failed after the primary error: {cleanup_error}",
                file=sys.stderr,
                flush=True,
            )

    summary = _summary(rows, params, args.scenario)
    summary["validation_profile"] = validation_profile.profile_name
    summary["validation_profile_calibrated"] = int(validation_profile.calibrated)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / f"{args.scenario}_summary.json"
    output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Summary written to {output}")
    if args.strict:
        _validate(summary, params)
    return summary


def main(argv: list[str] | None = None) -> int:
    run(parse_args(argv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
