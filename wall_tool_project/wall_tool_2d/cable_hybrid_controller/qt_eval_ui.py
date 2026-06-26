#!/usr/bin/env python3
"""Hybrid live UI: Matplotlib scene plus PyQtGraph evaluation plots."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing PyQtGraph/Qt packages. Install requirements first:\n"
        "  python -m pip install -r requirements.txt"
    ) from exc

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle

from cable_hybrid_controller.controller import BEST_PLANNER, command_controller, default_scenario, make_simulator
from wall_tool_sim.wall_tool_ui import (
    IntegratedToolArtist,
    SimState,
    Vec2,
    add2,
    distance2,
    normalize2,
    scale2,
)


pg.setConfigOptions(antialias=True, background="#fbfcfa", foreground="#18201c")


def qt_ui_font(point_size: int = 9) -> QtGui.QFont:
    for font_path in (
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/tahoma.ttf",
    ):
        QtGui.QFontDatabase.addApplicationFont(font_path)
    families = set(QtGui.QFontDatabase.families())
    for family in ("Segoe UI", "Arial", "Tahoma"):
        if family in families:
            return QtGui.QFont(family, point_size)
    return QtGui.QFont("Sans Serif", point_size)


@dataclass
class EvalSample:
    t: float
    payload_x: float
    payload_z: float
    reference_x: float
    reference_z: float
    tracking_error_m: float
    tool_speed_m_s: float
    body_attitude_deg: float
    cable_angle_deg: float
    body_rate_deg_s: float
    cable_rate_deg_s: float
    desired_tension_N: float
    measured_tension_N: float
    cable_vertical_force_N: float
    spool_velocity_m_s: float
    mpc_solve_ms: float
    left_thrust: float
    right_thrust: float


class QtEvalWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.setFont(qt_ui_font(9))
        self.setWindowTitle("PRISMS Wall Tool Controller")
        self.resize(1560, 940)

        self.scenario = default_scenario()
        self.sim = make_simulator()
        command_controller(self.sim, self.scenario.targets)

        self.playing = True
        self.show_trace = True
        self.show_path = True
        self.show_forces = True
        self.drawing = False
        self.draw_points: list[Vec2] = []
        self.draw_min_spacing_m = 0.055
        self.draw_max_points = 48
        self.drag_start: Vec2 | None = None
        self.drag_started = False
        self.last_wall_time = time.perf_counter()
        self.last_scene_draw_time = 0.0
        self.eval_samples: list[EvalSample] = []
        self.speed_spin = QtWidgets.QDoubleSpinBox()
        self.telemetry_labels: dict[str, QtWidgets.QLabel] = {}

        self._build_ui()
        self._build_scene()
        self._build_plots()
        self._connect_matplotlib_events()
        self.update_metrics()
        self.update_evaluation_plots()
        self.update_scene()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(16)

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(8)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("PRISMS Wall Tool Controller")
        title.setStyleSheet("font-size: 18px; font-weight: 700;")
        self.status_label = QtWidgets.QLabel("starting")
        self.status_label.setStyleSheet("font-family: Consolas; color: #4b5850;")
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.status_label)
        outer.addLayout(header)

        main = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        main.setChildrenCollapsible(False)
        outer.addWidget(main, 1)

        self.fig = Figure(figsize=(8.2, 7.2), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        main.addWidget(self.canvas)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)
        main.addWidget(right)
        main.setSizes([980, 580])

        metrics = QtWidgets.QGridLayout()
        metrics.setHorizontalSpacing(6)
        metrics.setVerticalSpacing(6)
        self.metric_labels: dict[str, QtWidgets.QLabel] = {}
        self.metric_captions: dict[str, QtWidgets.QLabel] = {}
        metric_specs = (
            ("tracking", "Tracking Error", "#111111"),
            ("contact", "Contact Force", "#2f855a"),
            ("tension", "Cable Tension", "#2f855a"),
            ("speed", "Tool Speed", "#2563a8"),
            ("thrust", "Motor Thrust", "#6b46c1"),
            ("solve", "MPC Solve", "#c05621"),
        )
        for index, (key, label, color) in enumerate(metric_specs):
            box = QtWidgets.QFrame()
            box.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
            box.setMinimumHeight(58)
            box.setStyleSheet(
                "QFrame { background: #fbfcfa; border: 1px solid #d8ddd6; border-radius: 6px; }"
            )
            box_layout = QtWidgets.QVBoxLayout(box)
            box_layout.setContentsMargins(8, 5, 8, 5)
            caption = QtWidgets.QLabel(label)
            caption.setStyleSheet("border: 0; color: #64706a; font-size: 11px;")
            value = QtWidgets.QLabel("0")
            value.setStyleSheet(
                f"border: 0; font-family: Consolas; font-size: 16px; font-weight: 700; color: {color};"
            )
            box_layout.addWidget(caption)
            box_layout.addWidget(value)
            metrics.addWidget(box, index // 3, index % 3)
            self.metric_labels[key] = value
            self.metric_captions[key] = caption
        right_layout.addLayout(metrics)

        telemetry_box = QtWidgets.QFrame()
        telemetry_box.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        telemetry_box.setStyleSheet("QFrame { background: #fbfcfa; border: 1px solid #d8ddd6; border-radius: 6px; }")
        telemetry_layout = QtWidgets.QGridLayout(telemetry_box)
        telemetry_layout.setContentsMargins(8, 5, 8, 5)
        telemetry_layout.setHorizontalSpacing(8)
        telemetry_layout.setVerticalSpacing(3)
        telemetry_specs = (
            ("Cable tension", "tension"),
            ("Cable length", "cable_length"),
            ("Cable angle", "cable_angle"),
            ("Cable rate", "cable_rate"),
            ("Reel velocity", "reel_velocity"),
            ("IMU attitude", "imu_attitude"),
            ("IMU rate", "imu_rate"),
            ("Est pos", "est_pos"),
            ("Est vel", "est_vel"),
            ("True pos", "true_pos"),
            ("Path pos", "desired_pos"),
            ("Path velocity", "ref_velocity"),
            ("Left motor", "left_input"),
            ("Right motor", "right_input"),
            ("Desired force", "desired_force"),
            ("MPC solve", "mpc_solve"),
            ("MPC status", "mpc_status"),
        )
        for index, (caption_text, key) in enumerate(telemetry_specs):
            row = index // 2
            col = 2 * (index % 2)
            caption = QtWidgets.QLabel(caption_text)
            caption.setStyleSheet("color: #64706a; font-size: 10px;")
            value = QtWidgets.QLabel("--")
            value.setMinimumWidth(130)
            value.setStyleSheet("font-family: Consolas; color: #18201c; font-size: 11px; font-weight: 600;")
            telemetry_layout.addWidget(caption, row, col)
            telemetry_layout.addWidget(value, row, col + 1)
            self.telemetry_labels[key] = value
        right_layout.addWidget(telemetry_box)

        self.plot_tabs = QtWidgets.QTabWidget()
        self.plot_tabs.setDocumentMode(True)
        self.plot_tabs.setStyleSheet(
            "QTabWidget::pane { border: 1px solid #d8ddd6; border-radius: 4px; } "
            "QTabBar::tab { padding: 6px 12px; min-width: 78px; }"
        )
        right_layout.addWidget(self.plot_tabs, 1)

        toolbar = QtWidgets.QHBoxLayout()
        self.play_button = QtWidgets.QPushButton("Pause")
        self.play_button.clicked.connect(self.toggle_play)
        self.reset_mission_button = QtWidgets.QPushButton("Reset Mission")
        self.reset_mission_button.clicked.connect(self.reset_mission)
        self.hold_button = QtWidgets.QPushButton("Hold Reset")
        self.hold_button.clicked.connect(self.reset_hold)
        self.clear_button = QtWidgets.QPushButton("Clear Path")
        self.clear_button.clicked.connect(self.clear_path)
        self.trace_button = QtWidgets.QPushButton("Trace On")
        self.trace_button.clicked.connect(self.toggle_trace)
        self.path_button = QtWidgets.QPushButton("Path On")
        self.path_button.clicked.connect(self.toggle_path)
        self.force_button = QtWidgets.QPushButton("Forces On")
        self.force_button.clicked.connect(self.toggle_forces)
        self.speed_spin.setRange(0.0, 8.0)
        self.speed_spin.setSingleStep(0.25)
        self.speed_spin.setValue(1.0)
        self.speed_spin.setDecimals(2)
        toolbar.addWidget(self.play_button)
        toolbar.addWidget(self.reset_mission_button)
        toolbar.addWidget(self.hold_button)
        toolbar.addWidget(self.clear_button)
        toolbar.addWidget(self.trace_button)
        toolbar.addWidget(self.path_button)
        toolbar.addWidget(self.force_button)
        toolbar.addStretch(1)
        toolbar.addWidget(QtWidgets.QLabel("sim speed"))
        toolbar.addWidget(self.speed_spin)
        outer.addLayout(toolbar)

        hint = QtWidgets.QLabel("Click wall to target. Shift-click appends. Hold and drag to draw a smooth path.")
        hint.setStyleSheet("color: #64706a;")
        outer.addWidget(hint)

    def _build_scene(self) -> None:
        params = self.sim.params
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")
        margin = 0.35
        self.ax.set_xlim(-params.wall_width / 2.0 - margin, params.wall_width / 2.0 + margin)
        self.ax.set_ylim(-0.10, params.wall_height + 0.35)
        self.ax.set_xlabel("wall x [m]")
        self.ax.set_ylabel("wall z [m]")
        self.ax.grid(True, color="#d8d4c9", linewidth=0.8)
        self.ax.set_facecolor("#f5f6f2")
        self.fig.subplots_adjust(left=0.08, right=0.99, bottom=0.08, top=0.97)

        self.wall_patch = Rectangle(
            (-params.wall_width / 2.0, 0.0),
            params.wall_width,
            params.wall_height,
            facecolor="#f3f1ea",
            edgecolor="#6d6a62",
            linewidth=2.0,
        )
        self.ax.add_patch(self.wall_patch)
        self.work_region_patch = Rectangle(
            (params.contact_work_x_min, params.contact_work_z_min),
            params.contact_work_x_max - params.contact_work_x_min,
            params.contact_work_z_max - params.contact_work_z_min,
            facecolor="#ffffff",
            edgecolor="#2f855a",
            linewidth=1.8,
            alpha=0.35,
        )
        self.ax.add_patch(self.work_region_patch)
        self.ax.text(params.contact_work_x_min, params.contact_work_z_max + 0.06, "cleaning bay", color="#2f855a", fontsize=9)
        self.spool = Circle(params.anchor, 0.075, facecolor="#444444", edgecolor="black", zorder=5)
        self.ax.add_patch(self.spool)
        self.ax.text(params.anchor[0], params.anchor[1] + 0.13, "anchor + spool", ha="center", fontsize=9)

        self.cable_line, = self.ax.plot([], [], color="#222222", linewidth=2.2, zorder=3, label="cable")
        self.trace_line, = self.ax.plot([], [], color="#2b7a78", linewidth=2.0, alpha=0.82, zorder=2, label="tool trace")
        self.reference_trace_line, = self.ax.plot([], [], color="#8a5b22", linewidth=1.4, linestyle=":", zorder=2, label="path sample trace")
        self.pending_line, = self.ax.plot([], [], color="#777777", linewidth=1.3, linestyle="--", zorder=4, label="desired path")
        self.mpc_prediction_line, = self.ax.plot([], [], color="#6b46c1", linewidth=1.8, linestyle="-.", zorder=6, label="MPC horizon")
        self.draw_preview_line, = self.ax.plot([], [], color="#f39c12", linewidth=2.8, alpha=0.9, zorder=9, label="drawn path")
        self.reference_point, = self.ax.plot([], [], marker="o", color="#1f77b4", markersize=5.0, zorder=9, label="_nolegend_", visible=False)
        self.target_point, = self.ax.plot([], [], marker="o", markerfacecolor="none", markeredgecolor="#8a5b22", markersize=8.0, mew=1.8, zorder=9, label="target")
        self.tool_point, = self.ax.plot([], [], marker="o", color="#8a4f00", markersize=6.0, zorder=13, label="tool")
        self.structure_line, = self.ax.plot([], [], color="#111111", linewidth=2.4, alpha=0.45, zorder=7, label="integrated wall tool")
        self.tool_label = self.ax.text(0.0, 0.0, "tool", color="#8a4f00", fontsize=8, zorder=14)
        self.reference_label = self.ax.text(0.0, 0.0, "", color="#1f77b4", fontsize=8, zorder=14, visible=False)
        self.status_box = self.ax.text(
            0.01,
            0.99,
            "",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            family="Consolas",
            color="#18201c",
            bbox={"boxstyle": "round,pad=0.25", "facecolor": "#fbfcfa", "edgecolor": "#cfd6ce", "alpha": 0.92},
            zorder=20,
        )

        self.tool_artist = IntegratedToolArtist(self.ax, params, 8)

        self.left_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=13, color="#1f77b4", zorder=12)
        self.right_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=13, color="#1f77b4", zorder=12)
        self.tension_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=13, color="#6a3d9a", zorder=12)
        for arrow in (self.left_arrow, self.right_arrow, self.tension_arrow):
            self.ax.add_patch(arrow)
        self.ax.legend(loc="lower right", fontsize=7, framealpha=0.92, ncol=2)

    def _build_plots(self) -> None:
        motion_tab = self._make_plot_tab("Motion")
        self.tracking_plot = self._make_plot(motion_tab, "Tracking Error", row=0, ylabel="m")
        self.tracking_error_curve = self.tracking_plot.plot(pen=pg.mkPen("#111111", width=2), name="error [m]")

        self.trajectory_plot = self._make_plot(motion_tab, "Trajectory Real vs Desired", row=1, ylabel="m")
        self.payload_x_curve = self.trajectory_plot.plot(pen=pg.mkPen("#111111", width=2), name="real x")
        self.reference_x_curve = self.trajectory_plot.plot(pen=pg.mkPen("#111111", width=1, style=QtCore.Qt.PenStyle.DashLine), name="des x")
        self.payload_z_curve = self.trajectory_plot.plot(pen=pg.mkPen("#2563a8", width=2), name="real z")
        self.reference_z_curve = self.trajectory_plot.plot(pen=pg.mkPen("#2563a8", width=1, style=QtCore.Qt.PenStyle.DashLine), name="des z")

        self.speed_plot = self._make_plot(motion_tab, "Tool Speed", row=2, ylabel="m/s", show_bottom=True)
        self.tool_speed_curve = self.speed_plot.plot(pen=pg.mkPen("#2563a8", width=2), name="speed")

        actuation_tab = self._make_plot_tab("Actuation")
        self.input_plot = self._make_plot(actuation_tab, "Motor Inputs", row=0, ylabel="N")
        self.left_input_curve = self.input_plot.plot(pen=pg.mkPen("#1f77b4", width=2), name="left thrust [N]")
        self.right_input_curve = self.input_plot.plot(pen=pg.mkPen("#bf3b32", width=2), name="right thrust [N]")

        self.cable_plot = self._make_plot(actuation_tab, "Cable Tension And Vertical Support", row=1, ylabel="N")
        self.measured_tension_curve = self.cable_plot.plot(pen=pg.mkPen("#2f855a", width=2), name="measured tension")
        self.desired_tension_curve = self.cable_plot.plot(
            pen=pg.mkPen("#111111", width=1, style=QtCore.Qt.PenStyle.DashLine),
            name="desired tension",
        )
        self.cable_vertical_curve = self.cable_plot.plot(pen=pg.mkPen("#6b46c1", width=2), name="vertical support")

        self.reel_plot = self._make_plot(actuation_tab, "Reel Velocity", row=2, ylabel="m/s", show_bottom=True)
        self.spool_curve = self.reel_plot.plot(pen=pg.mkPen("#2563a8", width=2), name="reel velocity")

        attitude_tab = self._make_plot_tab("Attitude")
        self.attitude_plot = self._make_plot(attitude_tab, "Body And Cable Angle", row=0, ylabel="deg")
        self.body_attitude_curve = self.attitude_plot.plot(pen=pg.mkPen("#c05621", width=2), name="body angle")
        self.cable_angle_curve = self.attitude_plot.plot(pen=pg.mkPen("#2563a8", width=2), name="cable angle")

        self.rate_plot = self._make_plot(attitude_tab, "Body And Cable Rate", row=1, ylabel="deg/s", show_bottom=True)
        self.body_rate_curve = self.rate_plot.plot(pen=pg.mkPen("#c05621", width=2), name="body rate")
        self.cable_rate_curve = self.rate_plot.plot(pen=pg.mkPen("#2563a8", width=2), name="cable rate")

        mpc_tab = self._make_plot_tab("MPC")
        self.mpc_plot = self._make_plot(mpc_tab, "MPC Solve Time", row=0, ylabel="ms", show_bottom=True)
        self.mpc_solve_curve = self.mpc_plot.plot(pen=pg.mkPen("#c05621", width=2), name="solve time")

    def _make_plot_tab(self, title: str) -> pg.GraphicsLayoutWidget:
        widget = pg.GraphicsLayoutWidget()
        widget.ci.layout.setSpacing(4)
        self.plot_tabs.addTab(widget, title)
        return widget

    def _make_plot(
        self,
        layout: pg.GraphicsLayoutWidget,
        title: str,
        row: int,
        ylabel: str,
        show_bottom: bool = False,
    ) -> pg.PlotItem:
        plot = layout.addPlot(row=row, col=0, title=title)
        plot.addLegend(offset=(-8, 8), labelTextColor="#18201c")
        plot.showGrid(x=True, y=True, alpha=0.22)
        plot.getAxis("left").setWidth(42)
        for axis_name in ("left", "bottom"):
            axis = plot.getAxis(axis_name)
            axis.setTickFont(QtGui.QFont("Segoe UI", 8))
            axis.enableAutoSIPrefix(False)
        plot.setLabel("left", ylabel)
        if show_bottom:
            plot.setLabel("bottom", "time before now [s]")
        else:
            plot.hideAxis("bottom")
        return plot

    def _connect_matplotlib_events(self) -> None:
        self.canvas.mpl_connect("button_press_event", self.on_press)
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("button_release_event", self.on_release)

    def _tick(self) -> None:
        now = time.perf_counter()
        wall_dt = min(0.08, max(0.0, now - self.last_wall_time))
        self.last_wall_time = now
        speed = max(0.0, float(self.speed_spin.value()))
        if self.playing and speed > 0.0:
            steps = max(1, min(180, int(round(wall_dt * speed / max(self.sim.params.dt, 1e-9)))))
            try:
                for _ in range(steps):
                    self.sim.step()
            except RuntimeError as exc:
                self.playing = False
                self.play_button.setText("Play")
                self.status_label.setStyleSheet("font-family: Consolas; color: #9b1c1c;")
                self.status_label.setText(str(exc))
                return

        self.update_metrics()
        self.update_evaluation_plots()
        if now - self.last_scene_draw_time >= 0.045:
            self.update_scene()
            self.last_scene_draw_time = now

    def update_metrics(self) -> None:
        state = self.sim.history[-1]
        speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
        self.metric_labels["tracking"].setText(f"{state.tool_error:.3f} m")
        self.metric_labels["contact"].setText(f"{state.contact_force:.2f} N")
        self.metric_labels["tension"].setText(f"{state.measured_tension:.2f} N")
        self.metric_labels["speed"].setText(f"{speed:.2f} m/s")
        self.metric_labels["thrust"].setText(f"{state.left_thrust:.2f} / {state.right_thrust:.2f} N")
        self.metric_labels["solve"].setText(f"{1000.0 * state.mpc_solve_time_s:.1f} ms")
        self.status_label.setText(
            f"t {state.t:6.1f}s   wp {state.active_waypoints:2d}   "
            f"mpc {1000.0 * state.mpc_solve_time_s:5.1f}ms   error {state.tool_error:5.3f}m"
        )
        self.status_label.setStyleSheet("font-family: Consolas; color: #4b5850;")
        self.update_telemetry(state, speed)

    def update_telemetry(self, state: SimState, speed: float) -> None:
        labels = self.telemetry_labels
        labels["tension"].setText(f"{state.measured_tension:5.2f} / {state.desired_cable_tension:5.2f} N")
        labels["cable_length"].setText(f"{state.measured_cable_length:5.3f} m")
        labels["cable_angle"].setText(f"{math.degrees(state.measured_theta):6.1f} deg")
        labels["cable_rate"].setText(f"{state.measured_theta_dot:6.3f} rad/s")
        labels["reel_velocity"].setText(f"{state.spool_velocity_cmd:6.3f} m/s")
        labels["imu_attitude"].setText(f"{math.degrees(state.measured_attitude):6.1f} deg")
        labels["imu_rate"].setText(f"{state.measured_angular_velocity:6.3f} rad/s")
        labels["est_pos"].setText(f"({state.measured_payload[0]:+.2f}, {state.measured_payload[1]:+.2f})")
        labels["est_vel"].setText(f"({state.estimated_payload_velocity[0]:+.2f}, {state.estimated_payload_velocity[1]:+.2f})")
        labels["true_pos"].setText(f"({state.payload[0]:+.2f}, {state.payload[1]:+.2f})")
        labels["desired_pos"].setText(f"({state.reference[0]:+.2f}, {state.reference[1]:+.2f})")
        labels["ref_velocity"].setText(f"({state.reference_velocity[0]:+.2f}, {state.reference_velocity[1]:+.2f})")
        labels["left_input"].setText(f"{state.left_thrust:5.3f} N")
        labels["right_input"].setText(f"{state.right_thrust:5.3f} N")
        labels["desired_force"].setText(f"({state.desired_drone_force[0]:+.2f}, {state.desired_drone_force[1]:+.2f}) N")
        labels["mpc_solve"].setText(f"{1000.0 * state.mpc_solve_time_s:5.1f} ms  J {state.mpc_objective:7.2f}")
        labels["mpc_status"].setText(state.mpc_status[:34] if state.mpc_status else "inactive")

    def update_scene(self) -> None:
        state = self.sim.history[-1]
        params = self.sim.params
        cable_mount = self.sim._cable_mount_position(state.payload, state.attitude)
        self.cable_line.set_data([params.anchor[0], cable_mount[0]], [params.anchor[1], cable_mount[1]])
        self.cable_line.set_linestyle("--" if state.cable_slack else "-")

        if self.show_trace:
            history = self.sim.history[-1600:]
            stride = max(1, len(history) // 800)
            samples = history[::stride]
            self.trace_line.set_data([sample.tool_head[0] for sample in samples], [sample.tool_head[1] for sample in samples])
            self.reference_trace_line.set_data([sample.reference[0] for sample in samples], [sample.reference[1] for sample in samples])
        else:
            self.trace_line.set_data([], [])
            self.reference_trace_line.set_data([], [])

        if self.show_path:
            pending = self.sim.trajectory.pending_path()
            stride = max(1, len(pending) // 800)
            points = pending[::stride]
            self.pending_line.set_data([point[0] for point in points], [point[1] for point in points])
        else:
            self.pending_line.set_data([], [])

        if state.mpc_predicted_path:
            self.mpc_prediction_line.set_data(
                [point[0] for point in state.mpc_predicted_path],
                [point[1] for point in state.mpc_predicted_path],
            )
        else:
            self.mpc_prediction_line.set_data([], [])

        if self.drawing and self.draw_points:
            self.draw_preview_line.set_data([p[0] for p in self.draw_points], [p[1] for p in self.draw_points])
        else:
            self.draw_preview_line.set_data([], [])

        left_center, right_center = self._module_centers(state)
        self.structure_line.set_data(
            [left_center[0], state.payload[0], right_center[0]],
            [left_center[1], state.payload[1], right_center[1]],
        )
        self.tool_artist.update(state.payload, state.attitude, left_center, right_center)
        self.reference_point.set_data([], [])
        self.target_point.set_data([state.target[0]], [state.target[1]])
        self.tool_point.set_data([state.tool_head[0]], [state.tool_head[1]])
        self.tool_label.set_position((state.tool_head[0] + 0.05, state.tool_head[1] + 0.05))
        self.reference_label.set_position((state.reference[0] + 0.05, state.reference[1] + 0.05))
        self.status_box.set_text(
            f"tracking {state.tool_error:.3f} m\n"
            f"contact {state.contact_force:.2f} N\n"
            f"tension {state.measured_tension:.2f} N\n"
            f"mpc {1000.0 * state.mpc_solve_time_s:.1f} ms"
        )
        self._update_force_arrows(state, left_center, right_center)
        self.canvas.draw_idle()

    def _module_centers(self, state: SimState) -> tuple[Vec2, Vec2]:
        left_offset, right_offset = self.sim._module_center_offsets(state.attitude)
        return add2(state.payload, left_offset), add2(state.payload, right_offset)

    def _update_force_arrows(self, state: SimState, left_center: Vec2, right_center: Vec2) -> None:
        params = self.sim.params
        left_axis, right_axis = self.sim._drone_axes(state.attitude)
        left_end = add2(left_center, scale2(left_axis, 0.05 + 0.26 * state.left_thrust / max(params.max_thrust_per_drone, 1e-9)))
        right_end = add2(right_center, scale2(right_axis, 0.05 + 0.26 * state.right_thrust / max(params.max_thrust_per_drone, 1e-9)))
        cable_dir = normalize2((params.anchor[0] - state.payload[0], params.anchor[1] - state.payload[1]))
        tension_end = add2(state.payload, scale2(cable_dir, 0.24))
        for arrow, start, end in (
            (self.left_arrow, left_center, left_end),
            (self.right_arrow, right_center, right_end),
            (self.tension_arrow, state.payload, tension_end),
        ):
            arrow.set_positions(start, end)
            arrow.set_visible(self.show_forces)

    def update_evaluation_plots(self) -> None:
        self._append_eval_sample(self.sim.history[-1])
        samples = self.eval_samples[-900:]
        if len(samples) < 2:
            return
        t = np.array([sample.t for sample in samples], dtype=float)
        x = t - t[-1]
        self.tracking_error_curve.setData(x, [sample.tracking_error_m for sample in samples])
        self.tool_speed_curve.setData(x, [sample.tool_speed_m_s for sample in samples])
        self.payload_x_curve.setData(x, [sample.payload_x for sample in samples])
        self.reference_x_curve.setData(x, [sample.reference_x for sample in samples])
        self.payload_z_curve.setData(x, [sample.payload_z for sample in samples])
        self.reference_z_curve.setData(x, [sample.reference_z for sample in samples])
        self.body_attitude_curve.setData(x, [sample.body_attitude_deg for sample in samples])
        self.cable_angle_curve.setData(x, [sample.cable_angle_deg for sample in samples])
        self.body_rate_curve.setData(x, [sample.body_rate_deg_s for sample in samples])
        self.cable_rate_curve.setData(x, [sample.cable_rate_deg_s for sample in samples])
        self.left_input_curve.setData(x, [sample.left_thrust for sample in samples])
        self.right_input_curve.setData(x, [sample.right_thrust for sample in samples])
        self.measured_tension_curve.setData(x, [sample.measured_tension_N for sample in samples])
        self.desired_tension_curve.setData(x, [sample.desired_tension_N for sample in samples])
        self.cable_vertical_curve.setData(x, [sample.cable_vertical_force_N for sample in samples])
        self.spool_curve.setData(x, [sample.spool_velocity_m_s for sample in samples])
        self.mpc_solve_curve.setData(x, [sample.mpc_solve_ms for sample in samples])
        y_values = [sample.payload_x for sample in samples] + [sample.reference_x for sample in samples]
        y_values += [sample.payload_z for sample in samples] + [sample.reference_z for sample in samples]
        y_min = min(y_values)
        y_max = max(y_values)
        if y_max > y_min:
            margin = max(0.05, 0.08 * (y_max - y_min))
            self.trajectory_plot.setYRange(y_min - margin, y_max + margin, padding=0.0)
        for plot in (
            self.tracking_plot,
            self.trajectory_plot,
            self.speed_plot,
            self.attitude_plot,
            self.rate_plot,
            self.input_plot,
            self.cable_plot,
            self.reel_plot,
            self.mpc_plot,
        ):
            plot.setXRange(max(-18.0, x[0]), 0.0, padding=0.0)

    def _append_eval_sample(self, state: SimState) -> None:
        speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
        self.eval_samples.append(
            EvalSample(
                t=state.t,
                payload_x=state.payload[0],
                payload_z=state.payload[1],
                reference_x=state.reference[0],
                reference_z=state.reference[1],
                tracking_error_m=state.tool_error,
                tool_speed_m_s=speed,
                body_attitude_deg=math.degrees(state.attitude),
                cable_angle_deg=math.degrees(state.theta),
                body_rate_deg_s=math.degrees(state.angular_velocity),
                cable_rate_deg_s=math.degrees(state.theta_dot),
                desired_tension_N=state.desired_cable_tension,
                measured_tension_N=state.measured_tension,
                cable_vertical_force_N=state.cable_vertical_force,
                spool_velocity_m_s=state.spool_velocity_cmd,
                mpc_solve_ms=1000.0 * state.mpc_solve_time_s,
                left_thrust=state.left_thrust,
                right_thrust=state.right_thrust,
            )
        )
        if len(self.eval_samples) > 1200:
            self.eval_samples = self.eval_samples[-900:]

    def on_press(self, event) -> None:
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        point = self.sim._clamp_wall_point((float(event.xdata), float(event.ydata)))
        self.drawing = True
        self.drag_started = False
        self.drag_start = point
        self.draw_points = [point]

    def on_motion(self, event) -> None:
        if not self.drawing or event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        point = self.sim._clamp_wall_point((float(event.xdata), float(event.ydata)))
        if self.drag_start is not None and distance2(point, self.drag_start) >= self.draw_min_spacing_m:
            self.drag_started = True
            self.playing = False
            self.play_button.setText("Play")
        if self.drag_started and self._append_draw_point(point):
            self.update_scene()

    def on_release(self, event) -> None:
        if not self.drawing:
            return
        if event.inaxes is self.ax and event.xdata is not None and event.ydata is not None:
            self._append_draw_point(self.sim._clamp_wall_point((float(event.xdata), float(event.ydata))))
        points = self._simplify_draw_points(self.draw_points)
        self.drawing = False
        self.draw_points = []
        if self.drag_started and len(points) >= 2 and self._path_length(points) >= 0.08:
            self.sim.set_smooth_path(points)
        elif self.drag_start is not None:
            modifiers = QtWidgets.QApplication.keyboardModifiers()
            append = bool(modifiers & QtCore.Qt.KeyboardModifier.ShiftModifier)
            if append:
                self.sim.append_target(self.drag_start, planner=BEST_PLANNER)
            else:
                self.sim.set_target(self.drag_start, planner=BEST_PLANNER)
        self.playing = True
        self.play_button.setText("Pause")
        self.update_scene()

    def _append_draw_point(self, point: Vec2) -> bool:
        if not self.draw_points or distance2(point, self.draw_points[-1]) >= self.draw_min_spacing_m:
            self.draw_points.append(point)
            return True
        return False

    def _simplify_draw_points(self, points: Sequence[Vec2]) -> list[Vec2]:
        if not points:
            return []
        filtered = [points[0]]
        for point in points[1:]:
            if distance2(point, filtered[-1]) >= self.draw_min_spacing_m:
                filtered.append(point)
        if distance2(points[-1], filtered[-1]) >= 1e-6:
            filtered.append(points[-1])
        if len(filtered) <= self.draw_max_points:
            return filtered
        keep: list[Vec2] = []
        last_index = len(filtered) - 1
        for sample_index in range(self.draw_max_points):
            source_index = round(sample_index * last_index / max(1, self.draw_max_points - 1))
            point = filtered[source_index]
            if not keep or distance2(point, keep[-1]) >= 1e-6:
                keep.append(point)
        return keep

    @staticmethod
    def _path_length(points: Sequence[Vec2]) -> float:
        return sum(distance2(points[index], points[index - 1]) for index in range(1, len(points)))

    def toggle_play(self) -> None:
        self.playing = not self.playing
        self.play_button.setText("Pause" if self.playing else "Play")

    def reset_mission(self) -> None:
        self.sim = make_simulator()
        command_controller(self.sim, self.scenario.targets)
        self.eval_samples = []
        self.playing = True
        self.play_button.setText("Pause")
        self.update_scene()

    def reset_hold(self) -> None:
        self.sim = make_simulator()
        self.eval_samples = []
        self.playing = False
        self.play_button.setText("Play")
        self.update_scene()

    def clear_path(self) -> None:
        self.sim.clear_trajectory()
        self.draw_points = []
        self.drawing = False
        self.update_scene()

    def toggle_trace(self) -> None:
        self.show_trace = not self.show_trace
        self.trace_button.setText("Trace On" if self.show_trace else "Trace Off")
        self.update_scene()

    def toggle_path(self) -> None:
        self.show_path = not self.show_path
        self.path_button.setText("Path On" if self.show_path else "Path Off")
        self.update_scene()

    def toggle_forces(self) -> None:
        self.show_forces = not self.show_forces
        self.force_button.setText("Forces On" if self.show_forces else "Forces Off")
        self.update_scene()


def run_qt_eval_ui() -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = QtEvalWindow()
    window.show()
    return int(app.exec())
