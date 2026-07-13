"""PySide6 trajectory planner and live CoppeliaSim control panel."""

from __future__ import annotations

import math
import queue
import sys
from pathlib import Path
from typing import Sequence

from PySide6 import QtCore, QtGui, QtWidgets


PACKAGE_DIR = Path(__file__).resolve().parent
WALL_TOOL_3D_ROOT = PACKAGE_DIR.parent
WALL_TOOL_PROJECT_ROOT = WALL_TOOL_3D_ROOT.parent
WALL_TOOL_2D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_2d"
for path in (WALL_TOOL_3D_ROOT, WALL_TOOL_2D_ROOT, WALL_TOOL_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cable_hybrid_controller.controller import best_params  # noqa: E402
from wall_tool_sim.wall_tool_ui import clamp_wall_point_for_params  # noqa: E402

from .interactive_session import (  # noqa: E402
    InteractiveCoppeliaSession,
    SessionConfig,
    SessionEvent,
    SessionTelemetry,
)


Vec2 = tuple[float, float]


def distance2(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def simplify_drawn_path(
    points: Sequence[Vec2],
    *,
    min_spacing_m: float = 0.055,
    max_points: int = 48,
) -> list[Vec2]:
    if not points:
        return []
    filtered = [points[0]]
    for point in points[1:]:
        if distance2(point, filtered[-1]) >= min_spacing_m:
            filtered.append(point)
    if distance2(filtered[-1], points[-1]) > 1e-9:
        filtered.append(points[-1])
    if len(filtered) <= max_points:
        return filtered
    last_index = len(filtered) - 1
    return [
        filtered[round(index * last_index / max(1, max_points - 1))]
        for index in range(max_points)
    ]


def append_connected_stroke(
    existing: Sequence[Vec2],
    stroke: Sequence[Vec2],
    *,
    max_points: int = 48,
) -> list[Vec2]:
    """Append a click or drag stroke while preserving one connected path."""

    combined = list(existing)
    for point in stroke:
        if combined and distance2(point, combined[-1]) <= 1e-9:
            continue
        if len(combined) >= max_points:
            break
        combined.append(point)
    return combined


def smooth_preview_path(points: Sequence[Vec2], samples_per_segment: int = 10) -> list[Vec2]:
    """Cubic-Hermite preview of the controller's nonzero-velocity corners."""

    if len(points) < 2:
        return list(points)
    samples: list[Vec2] = []
    tangent_scale = 0.30
    for index in range(len(points) - 1):
        p0 = points[index]
        p1 = points[index + 1]
        previous = points[index - 1] if index > 0 else p0
        following = points[index + 2] if index + 2 < len(points) else p1
        m0 = (
            tangent_scale * (p1[0] - previous[0]),
            tangent_scale * (p1[1] - previous[1]),
        )
        m1 = (
            tangent_scale * (following[0] - p0[0]),
            tangent_scale * (following[1] - p0[1]),
        )
        start_sample = 0 if index == 0 else 1
        for sample_index in range(start_sample, samples_per_segment + 1):
            t = sample_index / samples_per_segment
            t2, t3 = t * t, t * t * t
            h00 = 2.0 * t3 - 3.0 * t2 + 1.0
            h10 = t3 - 2.0 * t2 + t
            h01 = -2.0 * t3 + 3.0 * t2
            h11 = t3 - t2
            samples.append((
                h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0],
                h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1],
            ))
    return samples


class FacadePlanner(QtWidgets.QWidget):
    path_changed = QtCore.Signal()

    def __init__(self, params, parent=None) -> None:
        super().__init__(parent)
        self.params = params
        self.points: list[Vec2] = []
        self.preview_points: list[Vec2] = []
        self.trail: list[Vec2] = []
        self.telemetry: SessionTelemetry | None = None
        self.mode = "smart"
        self.drawing = False
        self.press_point: Vec2 | None = None
        self.setMinimumSize(680, 650)
        self.setMouseTracking(True)
        self.setCursor(QtCore.Qt.CursorShape.CrossCursor)

    def set_mode(self, mode: str) -> None:
        if mode not in {"smart", "freehand"}:
            raise ValueError(f"unknown planner mode: {mode}")
        self.mode = mode
        self.preview_points = []
        self.drawing = False
        self.update()

    def clear_path(self) -> None:
        self.points = []
        self.preview_points = []
        self.path_changed.emit()
        self.update()

    def undo(self) -> None:
        if self.points:
            self.points.pop()
            self.path_changed.emit()
            self.update()

    def set_path(self, points: Sequence[Vec2]) -> None:
        self.points = [clamp_wall_point_for_params(point, self.params) for point in points][:48]
        self.preview_points = []
        self.path_changed.emit()
        self.update()

    def set_telemetry(self, sample: SessionTelemetry) -> None:
        self.telemetry = sample
        if not self.trail or distance2(sample.payload_xz_m, self.trail[-1]) >= 0.012:
            self.trail.append(sample.payload_xz_m)
            if len(self.trail) > 1200:
                self.trail = self.trail[-1200:]
        self.update()

    def reset_trace(self) -> None:
        self.trail = []
        self.telemetry = None
        self.update()

    def _plot_rect(self) -> QtCore.QRectF:
        margin = 28.0
        available_w = max(1.0, self.width() - 2.0 * margin)
        available_h = max(1.0, self.height() - 2.0 * margin)
        aspect = self.params.wall_width / self.params.wall_height
        if available_w / available_h > aspect:
            height = available_h
            width = height * aspect
        else:
            width = available_w
            height = width / aspect
        return QtCore.QRectF(
            0.5 * (self.width() - width),
            0.5 * (self.height() - height),
            width,
            height,
        )

    def _to_screen(self, point: Vec2) -> QtCore.QPointF:
        rect = self._plot_rect()
        x_fraction = (point[0] + 0.5 * self.params.wall_width) / self.params.wall_width
        z_fraction = point[1] / self.params.wall_height
        return QtCore.QPointF(
            rect.left() + x_fraction * rect.width(),
            rect.bottom() - z_fraction * rect.height(),
        )

    def _to_world(self, position: QtCore.QPointF) -> Vec2:
        rect = self._plot_rect()
        x = (position.x() - rect.left()) / max(rect.width(), 1e-9) * self.params.wall_width
        x -= 0.5 * self.params.wall_width
        z = (rect.bottom() - position.y()) / max(rect.height(), 1e-9) * self.params.wall_height
        return clamp_wall_point_for_params((x, z), self.params)

    def _inside(self, position: QtCore.QPointF) -> bool:
        return self._plot_rect().contains(position)

    def paintEvent(self, _event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#0b1118"))
        rect = self._plot_rect()
        painter.fillRect(rect, QtGui.QColor("#121e29"))

        columns, rows = 4, 6
        gap_x, gap_z = 0.10, 0.10
        panel_w = (self.params.wall_width - (columns + 1) * gap_x) / columns
        panel_h = (self.params.wall_height - (rows + 1) * gap_z) / rows
        painter.setPen(QtGui.QPen(QtGui.QColor("#263846"), 1.0))
        painter.setBrush(QtGui.QColor("#132c3a"))
        for row in range(rows):
            for column in range(columns):
                x0 = -0.5 * self.params.wall_width + gap_x + column * (panel_w + gap_x)
                z0 = gap_z + row * (panel_h + gap_z)
                top_left = self._to_screen((x0, z0 + panel_h))
                bottom_right = self._to_screen((x0 + panel_w, z0))
                painter.drawRect(QtCore.QRectF(top_left, bottom_right))

        painter.setPen(QtGui.QPen(QtGui.QColor("#324552"), 1.0, QtCore.Qt.PenStyle.DashLine))
        for meter in range(1, int(self.params.wall_height)):
            left = self._to_screen((-0.5 * self.params.wall_width, float(meter)))
            right = self._to_screen((0.5 * self.params.wall_width, float(meter)))
            painter.drawLine(left, right)
        for meter in range(math.ceil(-0.5 * self.params.wall_width), math.floor(0.5 * self.params.wall_width) + 1):
            bottom = self._to_screen((float(meter), 0.0))
            top = self._to_screen((float(meter), self.params.wall_height))
            painter.drawLine(bottom, top)

        safe_a = clamp_wall_point_for_params((-99.0, -99.0), self.params)
        safe_b = clamp_wall_point_for_params((99.0, 99.0), self.params)
        safe_top_left = self._to_screen((safe_a[0], safe_b[1]))
        safe_bottom_right = self._to_screen((safe_b[0], safe_a[1]))
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor("#2b6f75"), 1.5, QtCore.Qt.PenStyle.DashLine))
        painter.drawRect(QtCore.QRectF(safe_top_left, safe_bottom_right))

        anchor = self._to_screen(self.params.anchor)
        painter.setPen(QtGui.QPen(QtGui.QColor("#9aa8b4"), 2.0))
        painter.setBrush(QtGui.QColor("#c8d0d7"))
        painter.drawEllipse(anchor, 5.0, 5.0)

        if len(self.points) >= 2:
            self._draw_polyline(painter, self.points, "#81552d", 1.2, dashed=True)
            self._draw_polyline(painter, smooth_preview_path(self.points), "#ff8a28", 3.0)
        for index, point in enumerate(self.points):
            screen = self._to_screen(point)
            painter.setBrush(QtGui.QColor("#ff9d42"))
            painter.setPen(QtGui.QPen(QtGui.QColor("#2a1505"), 1.0))
            painter.drawEllipse(screen, 6.0, 6.0)
            painter.setPen(QtGui.QColor("#f7f9fb"))
            painter.drawText(screen + QtCore.QPointF(8.0, -7.0), str(index + 1))

        if len(self.preview_points) >= 2:
            self._draw_polyline(painter, self.preview_points, "#ffd166", 2.2, dashed=True)
        if len(self.trail) >= 2:
            self._draw_polyline(painter, self.trail, "#34d399", 2.0)

        if self.telemetry is not None:
            payload = self._to_screen(self.telemetry.payload_xz_m)
            reference = self._to_screen(self.telemetry.reference_xz_m)
            attitude = self.telemetry.pitch_rad
            c, s = math.cos(attitude), math.sin(attitude)

            def body_point(local_x: float, local_z: float) -> QtCore.QPointF:
                world = (
                    self.telemetry.payload_xz_m[0] + c * local_x - s * local_z,
                    self.telemetry.payload_xz_m[1] + s * local_x + c * local_z,
                )
                return self._to_screen(world)

            cable_mount = body_point(0.0, self.params.payload_hex_radius)
            painter.setPen(QtGui.QPen(QtGui.QColor("#c7d0d8"), 1.4))
            painter.drawLine(anchor, cable_mount)
            painter.setPen(QtGui.QPen(QtGui.QColor("#5eead4"), 2.0))
            painter.drawLine(reference + QtCore.QPointF(-7.0, 0.0), reference + QtCore.QPointF(7.0, 0.0))
            painter.drawLine(reference + QtCore.QPointF(0.0, -7.0), reference + QtCore.QPointF(0.0, 7.0))
            body_polygon = QtGui.QPolygonF([
                body_point(-0.19, -0.065), body_point(0.19, -0.065),
                body_point(0.19, 0.065), body_point(-0.19, 0.065),
            ])
            painter.setBrush(QtGui.QColor("#f97316"))
            painter.setPen(QtGui.QPen(QtGui.QColor("#111827"), 1.5))
            painter.drawPolygon(body_polygon)
            painter.setBrush(QtGui.QColor("#fb923c"))
            left_motor = body_point(-0.305, 0.0)
            right_motor = body_point(0.305, 0.0)
            painter.drawEllipse(left_motor, 5.0, 5.0)
            painter.drawEllipse(right_motor, 5.0, 5.0)
            painter.setPen(QtGui.QPen(QtGui.QColor("#67e8f9"), 2.0))
            for motor, servo_angle in (
                (left_motor, self.telemetry.left_servo_rad),
                (right_motor, self.telemetry.right_servo_rad),
            ):
                axis_angle = attitude + servo_angle
                pixels_per_m = self._plot_rect().height() / self.params.wall_height
                axis_tip = motor + QtCore.QPointF(
                    pixels_per_m * 0.18 * math.sin(axis_angle),
                    -pixels_per_m * 0.18 * math.cos(axis_angle),
                )
                painter.drawLine(motor, axis_tip)

        painter.setPen(QtGui.QColor("#9fb0bd"))
        painter.drawText(rect.adjusted(8.0, 8.0, -8.0, -8.0), QtCore.Qt.AlignmentFlag.AlignTop, "ROOF REEL")
        painter.drawText(
            rect.adjusted(8.0, 8.0, -8.0, -8.0),
            QtCore.Qt.AlignmentFlag.AlignBottom | QtCore.Qt.AlignmentFlag.AlignLeft,
            "Facade coordinates: x horizontal, z vertical (m)",
        )

    def _draw_polyline(
        self,
        painter: QtGui.QPainter,
        points: Sequence[Vec2],
        color: str,
        width: float,
        dashed: bool = False,
    ) -> None:
        pen = QtGui.QPen(QtGui.QColor(color), width)
        if dashed:
            pen.setStyle(QtCore.Qt.PenStyle.DashLine)
        painter.setPen(pen)
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        path = QtGui.QPainterPath(self._to_screen(points[0]))
        for point in points[1:]:
            path.lineTo(self._to_screen(point))
        painter.drawPath(path)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if not self.isEnabled() or not self._inside(event.position()):
            return
        if event.button() == QtCore.Qt.MouseButton.RightButton:
            self.undo()
            return
        if event.button() != QtCore.Qt.MouseButton.LeftButton:
            return
        point = self._to_world(event.position())
        self.drawing = True
        self.press_point = point
        self.preview_points = [point]
        self.update()

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        if not self.drawing or not self._inside(event.position()):
            return
        point = self._to_world(event.position())
        if not self.preview_points or distance2(point, self.preview_points[-1]) >= 0.055:
            self.preview_points.append(point)
            self.update()

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if not self.drawing or event.button() != QtCore.Qt.MouseButton.LeftButton:
            return
        self.drawing = False
        if self._inside(event.position()):
            point = self._to_world(event.position())
            if not self.preview_points or distance2(point, self.preview_points[-1]) > 1e-9:
                self.preview_points.append(point)
        drawn = simplify_drawn_path(self.preview_points)
        self.preview_points = []
        click = (
            self.press_point is not None
            and (len(drawn) <= 1 or distance2(self.press_point, drawn[-1]) < 0.055)
        )
        self.press_point = None
        if click and drawn and self.mode == "smart":
            self.points = append_connected_stroke(self.points, [drawn[-1]])
            self.path_changed.emit()
        elif drawn:
            self.points = append_connected_stroke(self.points, drawn)
            self.path_changed.emit()
        self.update()


class ControlWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.params = best_params()
        self.session = InteractiveCoppeliaSession()
        self.max_error_m = 0.0
        self.setWindowTitle("PRISMS — CoppeliaSim Trajectory Control")
        self.resize(1450, 900)
        self.setMinimumSize(1120, 720)
        self._build_ui()
        self._apply_style()
        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self._poll_session)
        self.poll_timer.start(50)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(18, 14, 18, 14)
        root.setSpacing(12)

        title_row = QtWidgets.QHBoxLayout()
        title_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("CoppeliaSim Trajectory Control")
        title.setObjectName("title")
        subtitle = QtWidgets.QLabel(
            "Plan on the facade, then run the sensor-driven five-actuator NMPC in the detailed 3D plant."
        )
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        title_row.addLayout(title_box, 1)
        self.status_badge = QtWidgets.QLabel("READY")
        self.status_badge.setObjectName("statusBadge")
        self.status_badge.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        title_row.addWidget(self.status_badge)
        root.addLayout(title_row)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.planner = FacadePlanner(self.params)
        self.planner.path_changed.connect(self._path_changed)
        splitter.addWidget(self.planner)
        side = self._build_side_panel()
        splitter.addWidget(side)
        splitter.setSizes([920, 450])
        root.addWidget(splitter, 1)

    def _build_side_panel(self) -> QtWidgets.QWidget:
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        panel = QtWidgets.QWidget()
        scroll.setWidget(panel)
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(10, 4, 10, 10)
        layout.setSpacing(12)

        planner_group = QtWidgets.QGroupBox("Trajectory planner")
        planner_layout = QtWidgets.QVBoxLayout(planner_group)
        mode_row = QtWidgets.QHBoxLayout()
        self.waypoint_button = QtWidgets.QPushButton("Click / drag")
        self.draw_button = QtWidgets.QPushButton("Freehand only")
        for button in (self.waypoint_button, self.draw_button):
            button.setCheckable(True)
            mode_row.addWidget(button)
        self.waypoint_button.setChecked(True)
        self.mode_group = QtWidgets.QButtonGroup(self)
        self.mode_group.setExclusive(True)
        self.mode_group.addButton(self.waypoint_button)
        self.mode_group.addButton(self.draw_button)
        self.waypoint_button.clicked.connect(lambda: self.planner.set_mode("smart"))
        self.draw_button.clicked.connect(lambda: self.planner.set_mode("freehand"))
        planner_layout.addLayout(mode_row)
        interaction_hint = QtWidgets.QLabel(
            "Click repeatedly for connected waypoints, or hold and drag to append a smooth stroke. "
            "Right-click removes the last point."
        )
        interaction_hint.setWordWrap(True)
        interaction_hint.setObjectName("hint")
        planner_layout.addWidget(interaction_hint)
        edit_row = QtWidgets.QHBoxLayout()
        self.undo_button = QtWidgets.QPushButton("Undo")
        self.clear_button = QtWidgets.QPushButton("Clear")
        self.coverage_button = QtWidgets.QPushButton("Coverage preset")
        self.undo_button.clicked.connect(self.planner.undo)
        self.clear_button.clicked.connect(self.planner.clear_path)
        self.coverage_button.clicked.connect(self._coverage_preset)
        edit_row.addWidget(self.undo_button)
        edit_row.addWidget(self.clear_button)
        edit_row.addWidget(self.coverage_button)
        planner_layout.addLayout(edit_row)
        self.point_list = QtWidgets.QListWidget()
        self.point_list.setMinimumHeight(125)
        planner_layout.addWidget(self.point_list)
        layout.addWidget(planner_group)

        settings_group = QtWidgets.QGroupBox("Controller and view")
        form = QtWidgets.QFormLayout(settings_group)
        self.speed_spin = QtWidgets.QDoubleSpinBox()
        self.speed_spin.setRange(0.02, 0.25)
        self.speed_spin.setDecimals(3)
        self.speed_spin.setSingleStep(0.01)
        self.speed_spin.setValue(self.params.path_speed)
        self.speed_spin.setSuffix(" m/s")
        self.corner_spin = QtWidgets.QDoubleSpinBox()
        self.corner_spin.setRange(0.0, 0.25)
        self.corner_spin.setDecimals(3)
        self.corner_spin.setSingleStep(0.01)
        self.corner_spin.setValue(0.04)
        self.corner_spin.setSuffix(" m/s")
        self.camera_combo = QtWidgets.QComboBox()
        self.camera_combo.addItems(["overview", "payload", "winch"])
        self.regenerate_check = QtWidgets.QCheckBox("Rebuild scene (slow developer option)")
        self.regenerate_check.setChecked(False)
        form.addRow("Path speed", self.speed_spin)
        form.addRow("Corner speed", self.corner_spin)
        form.addRow("Coppelia camera", self.camera_combo)
        form.addRow("", self.regenerate_check)
        layout.addWidget(settings_group)

        run_group = QtWidgets.QGroupBox("Simulation")
        run_layout = QtWidgets.QGridLayout(run_group)
        self.start_button = QtWidgets.QPushButton("Start trajectory")
        self.start_button.setObjectName("startButton")
        self.pause_button = QtWidgets.QPushButton("Pause")
        self.resume_button = QtWidgets.QPushButton("Resume")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setObjectName("stopButton")
        self.start_button.clicked.connect(self._start)
        self.pause_button.clicked.connect(self._pause)
        self.resume_button.clicked.connect(self._resume)
        self.stop_button.clicked.connect(self._stop)
        run_layout.addWidget(self.start_button, 0, 0, 1, 2)
        run_layout.addWidget(self.pause_button, 1, 0)
        run_layout.addWidget(self.resume_button, 1, 1)
        run_layout.addWidget(self.stop_button, 2, 0, 1, 2)
        layout.addWidget(run_group)

        telemetry_group = QtWidgets.QGroupBox("Live telemetry")
        telemetry_layout = QtWidgets.QGridLayout(telemetry_group)
        self.telemetry_labels: dict[str, QtWidgets.QLabel] = {}
        telemetry_fields = (
            ("phase", "Run phase"), ("time", "Simulation time"),
            ("position", "Actual position x / z"), ("estimated", "Estimated position x / z"),
            ("reference", "Reference position x / z"),
            ("velocity", "Actual velocity vx / vz"),
            ("estimated_velocity", "Estimated velocity vx / vz"),
            ("orientation", "Orientation roll / pitch / yaw"),
            ("angular_rate", "Angular rate roll / pitch / yaw"),
            ("wall_normal", "Wall-normal drift"),
            ("error", "Tracking error"), ("max_error", "Maximum error"),
            ("cable_angle", "Cable angle / rate"),
            ("tension", "Steel-cable tension"), ("reel", "Reel payout / speed"),
            ("thrust", "Left / right thrust"), ("servo", "Left / right servo"),
            ("solver", "NMPC solve"),
        )
        for row, (key, label) in enumerate(telemetry_fields):
            telemetry_layout.addWidget(QtWidgets.QLabel(label), row, 0)
            value = QtWidgets.QLabel("—")
            value.setObjectName("telemetryValue")
            value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
            telemetry_layout.addWidget(value, row, 1)
            self.telemetry_labels[key] = value
        layout.addWidget(telemetry_group)

        log_group = QtWidgets.QGroupBox("Session log")
        log_layout = QtWidgets.QVBoxLayout(log_group)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setMinimumHeight(120)
        log_layout.addWidget(self.log)
        layout.addWidget(log_group)
        layout.addStretch(1)
        self._set_running_controls(False)
        return scroll

    def _apply_style(self) -> None:
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #0d141c; color: #dbe5ec; font-size: 13px; }
            QLabel#title { font-size: 24px; font-weight: 700; color: #f4f8fb; }
            QLabel#subtitle { color: #8fa3b1; }
            QLabel#hint { color: #8296a5; font-size: 12px; }
            QLabel#statusBadge { background: #273442; color: #d8e2e9; border-radius: 11px;
                                 min-width: 110px; min-height: 30px; font-weight: 700; }
            QGroupBox { border: 1px solid #263644; border-radius: 8px; margin-top: 10px;
                        padding-top: 10px; font-weight: 650; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; color: #b8c8d3; }
            QPushButton { background: #1c2a36; border: 1px solid #34495a; border-radius: 6px;
                          padding: 8px 10px; }
            QPushButton:hover { background: #263948; }
            QPushButton:checked { background: #155e75; border-color: #22d3ee; }
            QPushButton:disabled { color: #53636f; background: #151d24; border-color: #202b34; }
            QPushButton#startButton { background: #166534; border-color: #22c55e; font-weight: 700; }
            QPushButton#stopButton { background: #7f1d1d; border-color: #ef4444; font-weight: 700; }
            QDoubleSpinBox, QComboBox, QListWidget, QPlainTextEdit { background: #101b24;
                border: 1px solid #2b3e4c; border-radius: 5px; padding: 5px; }
            QLabel#telemetryValue { color: #67e8f9; font-family: Consolas; font-weight: 600; }
            QScrollArea { border: none; }
        """)

    def _coverage_preset(self) -> None:
        points: list[Vec2] = []
        left, right = -1.8, 1.8
        for index, z in enumerate((1.20, 1.80, 2.40, 3.00, 3.60, 4.20, 4.80)):
            points.extend(((left, z), (right, z)) if index % 2 == 0 else ((right, z), (left, z)))
        self.planner.set_path(points)

    def _path_changed(self) -> None:
        self.point_list.clear()
        for index, (x, z) in enumerate(self.planner.points, start=1):
            self.point_list.addItem(f"{index:02d}    x={x:+.3f} m    z={z:.3f} m")
        self.start_button.setEnabled(bool(self.planner.points) and not self.session.running)

    def _start(self) -> None:
        try:
            corner_speed = min(self.corner_spin.value(), self.speed_spin.value())
            config = SessionConfig(
                path=tuple(self.planner.points),
                path_speed_m_s=self.speed_spin.value(),
                corner_speed_m_s=corner_speed,
                camera=self.camera_combo.currentText(),
                regenerate_scene=self.regenerate_check.isChecked(),
            )
            self.max_error_m = 0.0
            self.planner.reset_trace()
            self.session.start(config)
            self._set_running_controls(True)
            self._set_status("CONNECTING", "#854d0e")
        except Exception as exc:
            self._append_log(f"ERROR: {exc}")
            self._set_status("ERROR", "#991b1b")

    def _pause(self) -> None:
        try:
            self.session.pause()
            self._set_running_controls(True, paused=True)
        except Exception as exc:
            self._append_log(f"ERROR: {exc}")

    def _resume(self) -> None:
        try:
            self.session.resume()
            self._set_running_controls(True, paused=False)
        except Exception as exc:
            self._append_log(f"ERROR: {exc}")

    def _stop(self) -> None:
        self.session.stop()
        self.stop_button.setEnabled(False)

    def _set_running_controls(self, running: bool, paused: bool = False) -> None:
        self.start_button.setEnabled(not running and bool(self.planner.points))
        self.pause_button.setEnabled(running and not paused)
        self.resume_button.setEnabled(running and paused)
        self.stop_button.setEnabled(running)
        self.planner.setEnabled(not running)
        for widget in (
            self.waypoint_button, self.draw_button, self.undo_button, self.clear_button,
            self.coverage_button, self.point_list, self.speed_spin, self.corner_spin,
            self.camera_combo, self.regenerate_check,
        ):
            widget.setEnabled(not running)

    def _poll_session(self) -> None:
        while True:
            try:
                event = self.session.events.get_nowait()
            except queue.Empty:
                break
            self._handle_event(event)
        latest = None
        while True:
            try:
                latest = self.session.telemetry.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            self._show_telemetry(latest)

    def _handle_event(self, event: SessionEvent) -> None:
        self._append_log(f"{event.kind.upper()}: {event.message}")
        if event.kind == "running":
            self._set_status("RUNNING", "#166534")
            self._set_running_controls(True, paused=False)
        elif event.kind == "paused":
            self._set_status("PAUSED", "#9a6700")
            self._set_running_controls(True, paused=True)
        elif event.kind == "error":
            self._set_status("FAILED LOUDLY", "#991b1b")
        elif event.kind == "stopping":
            self._set_status("STOPPING", "#7c2d12")
        elif event.kind == "stopped":
            if self.status_badge.text() != "FAILED LOUDLY":
                self._set_status("STOPPED", "#334155")
            self._set_running_controls(False)
        elif event.kind == "connecting":
            self._set_status("CONNECTING", "#854d0e")

    def _show_telemetry(self, sample: SessionTelemetry) -> None:
        self.planner.set_telemetry(sample)
        self.max_error_m = max(self.max_error_m, sample.error_m)
        phase = "TRACKING" if sample.path_active else "HOLDING FINAL POINT"
        if self.session.paused:
            phase = "PAUSED"
        self.telemetry_labels["phase"].setText(
            f"{phase} | {sample.waypoints_remaining} remaining"
        )
        self.telemetry_labels["time"].setText(f"{sample.timestamp_s:8.2f} s")
        self.telemetry_labels["position"].setText(
            f"{sample.payload_xz_m[0]:+.3f} / {sample.payload_xz_m[1]:.3f} m"
        )
        self.telemetry_labels["estimated"].setText(
            f"{sample.estimated_xz_m[0]:+.3f} / {sample.estimated_xz_m[1]:.3f} m"
        )
        self.telemetry_labels["reference"].setText(
            f"{sample.reference_xz_m[0]:+.3f} / {sample.reference_xz_m[1]:.3f} m"
        )
        self.telemetry_labels["velocity"].setText(
            f"{sample.payload_velocity_xz_m_s[0]:+.3f} / "
            f"{sample.payload_velocity_xz_m_s[1]:+.3f} m/s"
        )
        self.telemetry_labels["estimated_velocity"].setText(
            f"{sample.estimated_velocity_xz_m_s[0]:+.3f} / "
            f"{sample.estimated_velocity_xz_m_s[1]:+.3f} m/s"
        )
        roll, pitch, yaw = (math.degrees(value) for value in sample.orientation_rpy_rad)
        self.telemetry_labels["orientation"].setText(
            f"{roll:+.2f} / {pitch:+.2f} / {yaw:+.2f} deg"
        )
        roll_rate, pitch_rate, yaw_rate = (
            math.degrees(value) for value in sample.angular_velocity_rpy_rad_s
        )
        self.telemetry_labels["angular_rate"].setText(
            f"{roll_rate:+.2f} / {pitch_rate:+.2f} / {yaw_rate:+.2f} deg/s"
        )
        self.telemetry_labels["wall_normal"].setText(
            f"{1000.0 * sample.wall_normal_drift_m:+.2f} mm"
        )
        self.telemetry_labels["error"].setText(f"{1000.0 * sample.error_m:7.2f} mm")
        self.telemetry_labels["max_error"].setText(f"{1000.0 * self.max_error_m:7.2f} mm")
        self.telemetry_labels["cable_angle"].setText(
            f"{math.degrees(sample.cable_angle_rad):+.2f} deg / "
            f"{math.degrees(sample.cable_angle_rate_rad_s):+.2f} deg/s"
        )
        self.telemetry_labels["tension"].setText(f"{sample.tension_N:7.3f} N")
        self.telemetry_labels["reel"].setText(
            f"{sample.reel_length_m:6.3f} m / {sample.reel_velocity_m_s:+.3f} m/s"
        )
        self.telemetry_labels["thrust"].setText(
            f"{sample.left_thrust_N:.3f} / {sample.right_thrust_N:.3f} N"
        )
        self.telemetry_labels["servo"].setText(
            f"{math.degrees(sample.left_servo_rad):+.1f} / "
            f"{math.degrees(sample.right_servo_rad):+.1f} deg"
        )
        self.telemetry_labels["solver"].setText(
            f"{1000.0 * sample.solver_time_s:.1f} ms | {sample.solver_status}"
        )
        if not sample.path_active and self.status_badge.text() == "RUNNING":
            self._set_status("HOLDING FINAL", "#0f766e")

    def _set_status(self, text: str, color: str) -> None:
        self.status_badge.setText(text)
        self.status_badge.setStyleSheet(
            f"background: {color}; color: #f8fafc; border-radius: 11px; "
            "min-width: 110px; min-height: 30px; font-weight: 700;"
        )

    def _append_log(self, message: str) -> None:
        self.log.appendPlainText(message)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.poll_timer.stop()
        self.session.stop()
        self.session.wait(5.0)
        event.accept()


def main() -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setApplicationName("PRISMS CoppeliaSim Control")
    window = ControlWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
