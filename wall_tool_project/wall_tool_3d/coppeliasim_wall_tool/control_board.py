"""3D CoppeliaSim sensor overlay for the existing 2D wall-tool UI."""

from __future__ import annotations

import math
import time

from wall_tool_sim.wall_tool_ui import SimState, WallToolApp, distance2


class WallTool3DSpectatorApp(WallToolApp):
    """Reuse the native 2D UI and append live CoppeliaSim sensor telemetry."""

    def __init__(self, *args, **kwargs) -> None:
        self._gesture_active = False
        self._gesture_dragging = False
        self._gesture_start_point = (0.0, 0.0)
        self._gesture_start_pixels = (0.0, 0.0)
        self._gesture_drag_pixel_threshold = 7.0
        super().__init__(*args, **kwargs)

    def draw(self) -> None:
        lock = getattr(self.sim, "state_lock", None)
        if lock is None:
            super().draw()
            return
        if getattr(self.sim, "async_running", False):
            super().draw()
            return
        with lock:
            super().draw()

    def animate(self, _frame: int):
        if getattr(self.sim, "async_running", False):
            self.draw()
            return []
        return super().animate(_frame)

    def on_press(self, event) -> None:
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        clicked_point = self.sim._clamp_wall_point((float(event.xdata), float(event.ydata)))
        self._gesture_active = True
        self._gesture_dragging = False
        self._gesture_start_point = clicked_point
        self._gesture_start_pixels = (float(event.x), float(event.y))
        self.is_drawing = True
        self.draw_points = [clicked_point]
        self.playing = False
        self._last_frame_wall_time = time.perf_counter()
        self.play_button.label.set_text("Play")
        self.draw()
        self.fig.canvas.draw_idle()

    def on_motion(self, event) -> None:
        if not self._gesture_active:
            return
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        point = self.sim._clamp_wall_point((float(event.xdata), float(event.ydata)))
        pixel_distance = math.hypot(float(event.x) - self._gesture_start_pixels[0], float(event.y) - self._gesture_start_pixels[1])
        wall_distance = distance2(point, self._gesture_start_point)
        if not self._gesture_dragging and (
            pixel_distance >= self._gesture_drag_pixel_threshold or wall_distance >= 0.75 * self.draw_min_spacing
        ):
            self._gesture_dragging = True
        if not self._gesture_dragging:
            return
        if self._append_draw_point(point):
            self.draw_preview_line.set_data(
                [draw_point[0] for draw_point in self.draw_points],
                [draw_point[1] for draw_point in self.draw_points],
            )
            self.fig.canvas.draw_idle()

    def on_release(self, event) -> None:
        if not self._gesture_active:
            return
        release_point = self._gesture_start_point
        if event.inaxes is self.ax and event.xdata is not None and event.ydata is not None:
            release_point = self.sim._clamp_wall_point((float(event.xdata), float(event.ydata)))
            if self._gesture_dragging:
                self._append_draw_point(release_point)

        if self._gesture_dragging:
            smooth_path = self._simplify_draw_points(self.draw_points)
            if len(smooth_path) >= 2:
                self.sim.set_smooth_path(smooth_path)
            else:
                self._command_single_target(release_point)
        else:
            self._command_single_target(release_point)

        self._gesture_active = False
        self._gesture_dragging = False
        self.is_drawing = False
        self.draw_points = []
        self.playing = True
        self._last_frame_wall_time = time.perf_counter()
        self.play_button.label.set_text("Pause")
        self.draw()
        self.fig.canvas.draw_idle()

    def _command_single_target(self, point: tuple[float, float]) -> None:
        if self.append_mode:
            self.sim.append_target(point, planner=self.planner)
        else:
            self.sim.set_target(point, planner=self.planner)

    def clear_trace(self, _event) -> None:
        clear_trace = getattr(self.sim, "clear_trace", None)
        if clear_trace is None:
            super().clear_trace(_event)
            return
        clear_trace()
        self.draw_points = []
        self.draw()
        self.fig.canvas.draw_idle()

    def _efficiency_text(self, state: SimState) -> str:
        base = super()._efficiency_text(state)
        sensor_text = getattr(self.sim, "sensor_text", "")
        if not sensor_text:
            return base
        return f"{base}\n{sensor_text}"
