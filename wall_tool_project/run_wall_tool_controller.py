#!/usr/bin/env python3
"""Single entry point for the wall-tool controller project.

PyCharm: right-click this file and run it.

Default mode opens the hybrid Qt controller UI: Matplotlib renders the wall
scene while PyQtGraph renders fast realtime evaluation plots. Pass `--mode log`
to write a logged controller session, or `--mode quick` for a short smoke test.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
WALL_TOOL_2D_ROOT = PROJECT_ROOT / "wall_tool_2d"
for path in (WALL_TOOL_2D_ROOT, PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

DEFAULT_MODE = "qt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the wall-tool controller.")
    parser.add_argument(
        "--mode",
        choices=("log", "qt", "tk", "ui", "quick"),
        default=DEFAULT_MODE,
        help=(
            "qt opens the hybrid Matplotlib/PyQtGraph UI, tk opens the Tk fallback, "
            "ui opens the Matplotlib UI, log writes a fresh report, quick runs a smoke test."
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=12.0,
        help="Duration in seconds for --mode quick.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional output directory for --mode log.",
    )
    return parser.parse_args()


def run_logged(output_dir: Path | None = None) -> int:
    from cable_hybrid_controller.controller import run_controller_session
    from cable_hybrid_controller.diagnostics import write_diagnostics

    print("Running logged facade mission. This can take about 30 seconds...", flush=True)
    scenario, params, states = run_controller_session()
    active_output_dir, summary = write_diagnostics(scenario, params, states, output_dir)
    controller_params = summary.get("controller_params", {})
    control_law = controller_params.get("control_law", "unknown") if isinstance(controller_params, dict) else "unknown"
    print(f"Controller session written to: {active_output_dir}")
    print(f"Control law: {control_law}")
    print(f"Duration [s]: {float(summary['duration_s']):.2f}")
    print(f"Final error [m]: {float(summary['final_error_m']):.4f}")
    print(f"RMS error [m]: {float(summary['rms_error_m']):.4f}")
    print(f"Max error [m]: {float(summary['max_error_m']):.4f}")
    print(f"Coverage fraction: {float(summary.get('coverage_fraction', 0.0)):.3f}")
    print(f"Contact valid fraction: {float(summary.get('contact_valid_fraction', 0.0)):.3f}")
    print(f"Work-mode contact valid fraction: {float(summary.get('work_mode_contact_valid_fraction', 0.0)):.3f}")
    print(f"Mean contact force [N]: {float(summary.get('mean_contact_force_N', 0.0)):.3f}")
    print(f"Mean cable support fraction: {float(summary['mean_cable_support_fraction']):.3f}")
    print(f"Mean motor power ratio: {float(summary['mean_drone_power_ratio']):.3f}")
    print(f"Mean swing energy [J]: {float(summary.get('mean_swing_energy_J', 0.0)):.6f}")
    print(f"Max thrust fraction: {float(summary['max_thrust_fraction']):.3f}")
    print(f"Thrust-limit active fraction: {float(summary['thrust_limit_active_fraction']):.4f}")
    print(f"Slack fraction: {float(summary['slack_sample_fraction']):.4f}")
    return 0


def run_quick(duration_s: float) -> int:
    from cable_hybrid_controller.controller import BEST_PLANNER, make_simulator

    simulator = make_simulator()
    simulator.set_target((0.90, 1.50), planner=BEST_PLANNER)
    steps = max(1, int(duration_s / simulator.params.dt))
    state = simulator.history[-1]
    for _ in range(steps):
        state = simulator.step()
    print(f"Quick check duration [s]: {state.t:.2f}")
    print(f"Tracking error [m]: {state.tool_error:.4f}")
    print(f"Contact force [N]: {state.contact_force:.3f}")
    print(f"Contact valid: {state.contact_valid}")
    print(f"Normal gap [mm]: {1000.0 * state.normal_gap:.1f}")
    print(f"Active waypoints: {state.active_waypoints}")
    return 0


def run_ui() -> int:
    try:
        import matplotlib

        _select_interactive_backend(matplotlib)
        import matplotlib.animation as animation
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing matplotlib. Install project requirements first:\n"
            "  python -m pip install -r requirements.txt"
        ) from exc

    from wall_tool_sim.wall_tool_ui import WallToolApp
    from cable_hybrid_controller.controller import BEST_PLANNER, make_simulator

    simulator = make_simulator()
    app = WallToolApp(simulator, planner=BEST_PLANNER)
    ani = animation.FuncAnimation(app.fig, app.animate, interval=40, blit=False, cache_frame_data=False)
    app.fig._prisms_animation = ani
    plt.show()
    return 0


def run_tk() -> int:
    from cable_hybrid_controller.tk_ui import run_tk_ui

    return run_tk_ui()


def run_qt() -> int:
    from cable_hybrid_controller.qt_eval_ui import run_qt_eval_ui

    return run_qt_eval_ui()


def _select_interactive_backend(matplotlib_module) -> None:
    current_backend = str(matplotlib_module.get_backend()).lower()
    if "agg" not in current_backend:
        return
    failures: list[str] = []
    for backend in ("TkAgg", "QtAgg", "Qt5Agg"):
        try:
            matplotlib_module.use(backend, force=True)
            return
        except Exception as exc:  # pragma: no cover - depends on local GUI packages.
            failures.append(f"{backend}: {exc}")
    details = "\n  ".join(failures)
    raise SystemExit(
        "Matplotlib is using a non-interactive backend and no GUI backend could be loaded.\n"
        "Try installing a GUI backend package, or run `--mode log` / `--mode quick`.\n"
        f"Backend failures:\n  {details}"
    )


def main() -> int:
    args = parse_args()
    if args.mode == "log":
        return run_logged(args.output_dir)
    if args.mode == "quick":
        return run_quick(float(args.duration))
    if args.mode == "qt":
        return run_qt()
    if args.mode == "tk":
        return run_tk()
    return run_ui()


if __name__ == "__main__":
    raise SystemExit(main())
