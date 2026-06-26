#!/usr/bin/env python3
"""Native Tkinter live UI for the wall-tool controller."""

from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk
from typing import Sequence

from cable_hybrid_controller.controller import BEST_PLANNER, command_controller, default_scenario, make_simulator
from wall_tool_sim.wall_tool_ui import SimState, Vec2, add2, integrated_motor_axes, oriented_box_polygon, scale2


class TkWallToolApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PRISMS Wall Tool Controller")
        self.root.minsize(1120, 720)

        self.scenario = default_scenario()
        self.sim = make_simulator()
        command_controller(self.sim, self.scenario.targets)

        self.playing = True
        self.show_trace = True
        self.show_path = True
        self.show_forces = True
        self.mouse_down = False
        self.dragging_path = False
        self.draw_points: list[Vec2] = []
        self.draw_min_spacing_m = 0.055
        self.draw_max_points = 48
        self.drag_start_px = (0.0, 0.0)
        self.drag_start_state = 0
        self.drag_start_was_playing = True
        self.drag_start_threshold_px = 8.0
        self.last_wall_time = time.perf_counter()
        self.speed_var = tk.DoubleVar(value=1.0)

        self._build_layout()
        self._bind_events()
        self._tick()

    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1, minsize=680)
        self.root.columnconfigure(1, weight=0, minsize=390)
        self.root.rowconfigure(1, weight=1)

        header = ttk.Frame(self.root, padding=(10, 8))
        header.grid(row=0, column=0, columnspan=2, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="PRISMS Wall Tool Controller", font=("Segoe UI", 15, "bold")).grid(row=0, column=0, sticky="w")
        self.status_var = tk.StringVar(value="mission")
        ttk.Label(header, textvariable=self.status_var, font=("Consolas", 10)).grid(row=0, column=1, sticky="e")

        self.scene = tk.Canvas(self.root, background="#f5f6f2", highlightthickness=1, highlightbackground="#cfd6ce")
        self.scene.grid(row=1, column=0, sticky="nsew", padx=(10, 5), pady=(0, 8))

        panel = ttk.Frame(self.root, padding=(5, 0, 10, 8))
        panel.grid(row=1, column=1, sticky="nsew")
        panel.columnconfigure(0, weight=1)
        for row in range(5):
            panel.rowconfigure(row, weight=1 if row > 0 else 0)

        self.metric_vars = {
            "tracking": tk.StringVar(value="0.000 m"),
            "contact": tk.StringVar(value="off"),
            "support": tk.StringVar(value="0%"),
            "speed": tk.StringVar(value="0.00 m/s"),
            "power": tk.StringVar(value="0%"),
            "energy": tk.StringVar(value="0 mJ"),
        }
        metrics = ttk.Frame(panel)
        metrics.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        for col in range(3):
            metrics.columnconfigure(col, weight=1)
        metric_specs = (
            ("Tracking", "tracking"),
            ("Contact", "contact"),
            ("Cable Support", "support"),
            ("Speed", "speed"),
            ("Motor Power", "power"),
            ("Swing Energy", "energy"),
        )
        for index, (label, key) in enumerate(metric_specs):
            box = ttk.Frame(metrics, padding=(7, 5), relief="groove")
            box.grid(row=index // 3, column=index % 3, sticky="ew", padx=3, pady=3)
            ttk.Label(box, text=label, font=("Segoe UI", 8)).pack(anchor="w")
            ttk.Label(box, textvariable=self.metric_vars[key], font=("Consolas", 12, "bold")).pack(anchor="w")

        self.plot_task = tk.Canvas(panel, background="#fbfcfa", height=112, highlightthickness=1, highlightbackground="#d8ddd6")
        self.plot_smooth = tk.Canvas(panel, background="#fbfcfa", height=112, highlightthickness=1, highlightbackground="#d8ddd6")
        self.plot_act = tk.Canvas(panel, background="#fbfcfa", height=112, highlightthickness=1, highlightbackground="#d8ddd6")
        self.plot_reel = tk.Canvas(panel, background="#fbfcfa", height=112, highlightthickness=1, highlightbackground="#d8ddd6")
        self.plot_task.grid(row=1, column=0, sticky="nsew", pady=4)
        self.plot_smooth.grid(row=2, column=0, sticky="nsew", pady=4)
        self.plot_act.grid(row=3, column=0, sticky="nsew", pady=4)
        self.plot_reel.grid(row=4, column=0, sticky="nsew", pady=4)

        toolbar = ttk.Frame(self.root, padding=(10, 6))
        toolbar.grid(row=2, column=0, columnspan=2, sticky="ew")
        self.play_button = ttk.Button(toolbar, text="Pause", command=self.toggle_play)
        self.play_button.pack(side="left", padx=(0, 6))
        ttk.Button(toolbar, text="Reset Mission", command=self.reset_mission).pack(side="left", padx=6)
        ttk.Button(toolbar, text="Hold Reset", command=self.reset_hold).pack(side="left", padx=6)
        ttk.Button(toolbar, text="Clear Path", command=self.clear_path).pack(side="left", padx=6)
        self.trace_button = ttk.Button(toolbar, text="Trace On", command=self.toggle_trace)
        self.trace_button.pack(side="left", padx=6)
        self.path_button = ttk.Button(toolbar, text="Path On", command=self.toggle_path)
        self.path_button.pack(side="left", padx=6)
        self.force_button = ttk.Button(toolbar, text="Forces On", command=self.toggle_forces)
        self.force_button.pack(side="left", padx=6)
        ttk.Label(toolbar, text="speed").pack(side="left", padx=(18, 4))
        ttk.Scale(toolbar, from_=0.0, to=8.0, variable=self.speed_var, orient="horizontal", length=180).pack(side="left")
        self.speed_label_var = tk.StringVar(value="1.00x")
        ttk.Label(toolbar, textvariable=self.speed_label_var, font=("Consolas", 10)).pack(side="left", padx=7)
        ttk.Label(toolbar, text="Click targets. Hold and drag to draw a smooth path. Shift-click appends.", foreground="#64706a").pack(side="right")

    def _bind_events(self) -> None:
        self.scene.bind("<ButtonPress-1>", self.on_scene_press)
        self.scene.bind("<B1-Motion>", self.on_scene_drag)
        self.scene.bind("<ButtonRelease-1>", self.on_scene_release)
        self.root.bind("<space>", lambda _event: self.toggle_play())
        self.root.bind("r", lambda _event: self.reset_mission())

    def _tick(self) -> None:
        now = time.perf_counter()
        wall_dt = min(0.08, max(0.0, now - self.last_wall_time))
        self.last_wall_time = now
        speed = max(0.0, self.speed_var.get())
        self.speed_label_var.set(f"{speed:.2f}x")

        if self.playing and speed > 0.0:
            steps = max(1, min(180, int(round(wall_dt * speed / max(self.sim.params.dt, 1e-9)))))
            for _ in range(steps):
                self.sim.step()

        self.draw()
        self.root.after(16, self._tick)

    def draw(self) -> None:
        state = self.sim.history[-1]
        self.update_metrics(state)
        self.draw_scene(state)
        self.draw_plots()

    def update_metrics(self, state: SimState) -> None:
        params = self.sim.params
        weight = max(params.total_mass * params.gravity, 1e-9)
        speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
        no_cable_each = weight / max(2.0 * math.cos(params.hex_face_tilt_rad), 1e-9)
        no_cable_power = max(2.0 * no_cable_each**1.5, 1e-9)
        power = (state.left_thrust**1.5 + state.right_thrust**1.5) / no_cable_power
        support = state.cable_vertical_force / weight
        contact = f"{state.contact_force:.2f} N" if state.contact_valid else ("bad" if state.work_mode else "off")
        energy_text = f"{1000.0 * state.swing_energy_J:.3f} mJ" if state.swing_energy_J < 0.001 else f"{state.swing_energy_J:.4f} J"

        self.metric_vars["tracking"].set(f"{state.tool_error:.3f} m")
        self.metric_vars["contact"].set(contact)
        self.metric_vars["support"].set(f"{100.0 * support:.0f}%")
        self.metric_vars["speed"].set(f"{speed:.2f} m/s")
        self.metric_vars["power"].set(f"{100.0 * power:.0f}%")
        self.metric_vars["energy"].set(energy_text)
        self.status_var.set(
            f"t {state.t:6.1f}s   wp {state.active_waypoints:2d}   "
            f"mpc {1000.0 * state.mpc_solve_time_s:5.1f}ms   "
            f"error {state.tool_error:5.3f}m"
        )

    def draw_scene(self, state: SimState) -> None:
        canvas = self.scene
        canvas.delete("all")
        params = self.sim.params
        width = max(2, canvas.winfo_width())
        height = max(2, canvas.winfo_height())
        transform = SceneTransform(width, height, params.wall_width, params.wall_height)

        wall_a = transform.world((-params.wall_width / 2.0, params.wall_height))
        wall_b = transform.world((params.wall_width / 2.0, 0.0))
        canvas.create_rectangle(*wall_a, *wall_b, fill="#eeece4", outline="#6d6a62", width=2)
        self._draw_grid(canvas, transform)
        self._draw_work_bay(canvas, transform)
        self._draw_paths(canvas, transform)

        cable_mount = self.sim._cable_mount_position(state.payload, state.attitude)
        anchor_xy = transform.world(params.anchor)
        mount_xy = transform.world(cable_mount)
        canvas.create_line(*anchor_xy, *mount_xy, fill="#222222", width=3, dash=(7, 5) if state.cable_slack else None)
        canvas.create_oval(anchor_xy[0] - 8, anchor_xy[1] - 8, anchor_xy[0] + 8, anchor_xy[1] + 8, fill="#444444", outline="#111111")
        canvas.create_text(anchor_xy[0], anchor_xy[1] - 18, text="anchor + spool", font=("Segoe UI", 9), fill="#222222")

        self._draw_robot(canvas, transform, state)
        self._draw_force_bars(canvas, state)

    def _draw_grid(self, canvas: tk.Canvas, transform: "SceneTransform") -> None:
        params = self.sim.params
        for x in range(math.floor(-params.wall_width / 2), math.ceil(params.wall_width / 2) + 1):
            a = transform.world((float(x), 0.0))
            b = transform.world((float(x), params.wall_height))
            canvas.create_line(*a, *b, fill="#d7d2c7")
        for z in range(0, math.ceil(params.wall_height) + 1):
            a = transform.world((-params.wall_width / 2.0, float(z)))
            b = transform.world((params.wall_width / 2.0, float(z)))
            canvas.create_line(*a, *b, fill="#d7d2c7")

    def _draw_work_bay(self, canvas: tk.Canvas, transform: "SceneTransform") -> None:
        params = self.sim.params
        a = transform.world((params.contact_work_x_min, params.contact_work_z_max))
        b = transform.world((params.contact_work_x_max, params.contact_work_z_min))
        canvas.create_rectangle(*a, *b, outline="#2f855a", width=2)
        canvas.create_text(a[0] + 4, a[1] - 12, text="cleaning bay", anchor="w", fill="#2f855a", font=("Segoe UI", 10, "bold"))

    def _draw_paths(self, canvas: tk.Canvas, transform: "SceneTransform") -> None:
        if self.show_trace and len(self.sim.history) >= 2:
            trace = [state.tool_head for state in self.sim.history[-1200:]]
            ref = [state.reference for state in self.sim.history[-1200:]]
            self._draw_polyline(canvas, transform, trace, fill="#2b7a78", width=2)
            self._draw_polyline(canvas, transform, ref, fill="#9a6b25", width=1, dash=(4, 4))

        if self.show_path:
            pending = self.sim.trajectory.pending_path()
            self._draw_polyline(canvas, transform, pending, fill="#777777", width=1, dash=(6, 5))
            for target in self.scenario.targets:
                x, y = transform.world(target)
                canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#8a5b22", outline="")
        if self.dragging_path and self.draw_points:
            self._draw_polyline(canvas, transform, self.draw_points, fill="#f39c12", width=3)
            for point in self.draw_points[-8:]:
                x, y = transform.world(point)
                canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#f39c12", outline="")

    def _draw_robot(self, canvas: tk.Canvas, transform: "SceneTransform", state: SimState) -> None:
        params = self.sim.params
        left_offset, right_offset = self.sim._module_center_offsets(state.attitude)
        left_center = add2(state.payload, left_offset)
        right_center = add2(state.payload, right_offset)
        left_xy = transform.world(left_center)
        payload_xy = transform.world(state.payload)
        right_xy = transform.world(right_center)
        canvas.create_line(*left_xy, *payload_xy, *right_xy, fill="#111111", width=2)
        self._draw_integrated_tool(canvas, transform, state, left_center, right_center)

        ref_xy = transform.world(state.reference)
        target_xy = transform.world(state.target)
        canvas.create_oval(ref_xy[0] - 5, ref_xy[1] - 5, ref_xy[0] + 5, ref_xy[1] + 5, fill="#1f77b4", outline="")
        canvas.create_oval(target_xy[0] - 8, target_xy[1] - 8, target_xy[0] + 8, target_xy[1] + 8, outline="#8a5b22", width=2)

        if self.show_forces:
            left_axis, right_axis = integrated_motor_axes(params, state.attitude)
            self._draw_arrow(canvas, left_xy, left_axis, 18 + 26 * state.left_thrust / max(params.max_thrust_per_drone, 1e-9), "#2563a8")
            self._draw_arrow(canvas, right_xy, right_axis, 18 + 26 * state.right_thrust / max(params.max_thrust_per_drone, 1e-9), "#2563a8")
            cable_dir = (params.anchor[0] - state.payload[0], params.anchor[1] - state.payload[1])
            norm = math.hypot(cable_dir[0], cable_dir[1])
            if norm > 1e-9:
                self._draw_arrow(canvas, payload_xy, (cable_dir[0] / norm, cable_dir[1] / norm), 36, "#6a3d9a")

    def _draw_integrated_tool(
        self,
        canvas: tk.Canvas,
        transform: "SceneTransform",
        state: SimState,
        left_center: Vec2,
        right_center: Vec2,
    ) -> None:
        params = self.sim.params
        body_half_length = max(
            params.payload_half_length,
            _distance2(state.payload, left_center) + 0.09,
            _distance2(state.payload, right_center) + 0.09,
        )
        body_half_width = max(params.payload_hex_radius * 0.72, params.cage_radius * 0.33)
        self._draw_world_polygon(
            canvas,
            transform,
            oriented_box_polygon(state.payload, body_half_length, body_half_width, state.attitude),
            fill="#f2cc60",
            outline="#5c4512",
            width=2,
        )
        left_axis, right_axis = integrated_motor_axes(params, state.attitude)
        for center, axis in ((left_center, left_axis), (right_center, right_axis)):
            angle = math.atan2(axis[1], axis[0])
            self._draw_world_polygon(
                canvas,
                transform,
                oriented_box_polygon(center, params.cage_radius * 0.42, params.cage_radius * 0.22, angle),
                fill="#f8faf8",
                outline="#111111",
                width=2,
            )
            start = transform.world(center)
            end = transform.world(add2(center, scale2(axis, params.cage_radius * 0.50)))
            canvas.create_line(*start, *end, fill="#111111", width=2)
        cable_tab = oriented_box_polygon(
            add2(state.payload, self.sim._cable_mount_offset(state.attitude)),
            params.payload_hex_radius * 0.28,
            params.payload_hex_radius * 0.15,
            state.attitude,
        )
        self._draw_world_polygon(canvas, transform, cable_tab, fill="#d8b247", outline="#5c4512", width=1)
        tool_xy = transform.world(state.tool_head)
        pad_radius = max(4.0, params.payload_hex_radius * 0.34 * transform.scale)
        canvas.create_oval(
            tool_xy[0] - pad_radius,
            tool_xy[1] - pad_radius,
            tool_xy[0] + pad_radius,
            tool_xy[1] + pad_radius,
            fill="#8a4f00",
            outline="#4b2f05",
            width=1,
        )

    def _draw_polyline(
        self,
        canvas: tk.Canvas,
        transform: "SceneTransform",
        points: Sequence[Vec2],
        *,
        fill: str,
        width: int,
        dash: tuple[int, int] | None = None,
    ) -> None:
        if len(points) < 2:
            return
        stride = max(1, len(points) // 700)
        coords: list[float] = []
        for point in points[::stride]:
            x, y = transform.world(point)
            coords.extend((x, y))
        if len(coords) >= 4:
            canvas.create_line(*coords, fill=fill, width=width, dash=dash)

    def _draw_world_polygon(
        self,
        canvas: tk.Canvas,
        transform: "SceneTransform",
        points: Sequence[Vec2],
        *,
        fill: str,
        outline: str,
        width: int,
    ) -> None:
        coords: list[float] = []
        for point in points:
            x, y = transform.world(point)
            coords.extend((x, y))
        canvas.create_polygon(*coords, fill=fill, outline=outline, width=width)

    def _draw_arrow(
        self,
        canvas: tk.Canvas,
        start: tuple[float, float],
        direction: Vec2,
        length: float,
        fill: str,
    ) -> None:
        end = (start[0] + direction[0] * length, start[1] - direction[1] * length)
        canvas.create_line(*start, *end, fill=fill, width=2, arrow=tk.LAST, arrowshape=(9, 11, 4))

    def _draw_force_bars(self, canvas: tk.Canvas, state: SimState) -> None:
        params = self.sim.params
        width = max(2, canvas.winfo_width())
        height = max(2, canvas.winfo_height())
        speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
        values = (
            ("tracking", state.tool_error / max(params.work_contact_tracking_limit_m, 1e-9), "#111111"),
            ("speed", speed / max(params.work_contact_speed_limit_mps, 1e-9), "#2563a8"),
            ("thrust", max(state.left_thrust, state.right_thrust) / max(params.max_thrust_per_drone, 1e-9), "#bf3b32"),
        )
        x0 = 18
        y0 = height - 72
        bar_w = min(240, width * 0.32)
        for index, (name, value, color) in enumerate(values):
            y = y0 + 20 * index
            canvas.create_text(x0, y + 6, text=name, anchor="w", fill="#4b5850", font=("Segoe UI", 8))
            canvas.create_rectangle(x0 + 72, y, x0 + 72 + bar_w, y + 12, outline="#d8ddd6", fill="#ffffff")
            canvas.create_rectangle(x0 + 72, y, x0 + 72 + min(bar_w, bar_w * value), y + 12, outline="", fill=color)
            canvas.create_line(x0 + 72 + bar_w, y - 2, x0 + 72 + bar_w, y + 14, fill="#d95f0e")

    def draw_plots(self) -> None:
        samples = self.sim.history[-1400:]
        self._draw_plot(
            self.plot_task,
            "Task Validity",
            samples,
            (
                ("tracking", "#111111", lambda s: s.tool_error / max(self.sim.params.work_contact_tracking_limit_m, 1e-9)),
                ("speed", "#2563a8", lambda s: math.hypot(s.payload_velocity[0], s.payload_velocity[1]) / max(self.sim.params.work_contact_speed_limit_mps, 1e-9)),
                ("valid", "#2f855a", lambda s: 1.0 if s.contact_valid else 0.0),
            ),
        )
        self._draw_plot(
            self.plot_smooth,
            "Smoothness And Energy",
            samples,
            (
                ("body", "#c05621", lambda s: abs(s.angular_velocity) / max(self.sim.params.work_contact_angular_rate_limit_rad_s, 1e-9)),
                ("cable", "#718096", lambda s: abs(s.theta_dot) / max(self.sim.params.work_contact_angular_rate_limit_rad_s, 1e-9)),
                ("energy", "#2f855a", lambda s: s.swing_energy_J / max(self.sim.params.mpc_energy_plot_limit_J, 1e-9)),
            ),
        )
        weight = max(self.sim.params.total_mass * self.sim.params.gravity, 1e-9)
        no_cable_each = weight / max(2.0 * math.cos(self.sim.params.hex_face_tilt_rad), 1e-9)
        no_cable_power = max(2.0 * no_cable_each**1.5, 1e-9)
        self._draw_plot(
            self.plot_act,
            "Cable And Actuators",
            samples,
            (
                ("support", "#2f855a", lambda s: s.cable_vertical_force / weight),
                ("power", "#6b46c1", lambda s: (s.left_thrust**1.5 + s.right_thrust**1.5) / no_cable_power),
                ("thrust", "#bf3b32", lambda s: max(s.left_thrust, s.right_thrust) / max(self.sim.params.max_thrust_per_drone, 1e-9)),
            ),
        )
        self._draw_plot(
            self.plot_reel,
            "Reel Command",
            samples,
            (
                ("spool", "#2563a8", lambda s: abs(s.spool_velocity_cmd) / max(self.sim.params.max_spool_speed, 1e-9)),
            ),
        )

    def _draw_plot(
        self,
        canvas: tk.Canvas,
        title: str,
        samples: Sequence[SimState],
        series: Sequence[tuple[str, str, object]],
    ) -> None:
        canvas.delete("all")
        width = max(2, canvas.winfo_width())
        height = max(2, canvas.winfo_height())
        pad_l, pad_r, pad_t, pad_b = 36, 10, 24, 20
        canvas.create_text(8, 8, text=title, anchor="nw", fill="#18201c", font=("Segoe UI", 9, "bold"))
        if len(samples) < 2:
            return
        stride = max(1, len(samples) // 500)
        plotted = list(samples[::stride])
        t0 = plotted[0].t
        t1 = plotted[-1].t
        values_by_name: list[tuple[str, str, list[float]]] = []
        ymax = 1.05
        for name, color, getter in series:
            values = [float(getter(sample)) for sample in plotted]  # type: ignore[operator]
            ymax = max(ymax, max(values) if values else 0.0)
            values_by_name.append((name, color, values))
        ymax = min(1.8, max(1.05, ymax * 1.12))
        for index in range(4):
            y = pad_t + (height - pad_t - pad_b) * index / 3.0
            canvas.create_line(pad_l, y, width - pad_r, y, fill="#e2e6df")
        y_limit = pad_t + (height - pad_t - pad_b) * (1.0 - 1.0 / ymax)
        canvas.create_line(pad_l, y_limit, width - pad_r, y_limit, fill="#d95f0e", dash=(5, 4))
        for series_index, (name, color, values) in enumerate(values_by_name):
            coords: list[float] = []
            for sample, value in zip(plotted, values):
                x = pad_l + (width - pad_l - pad_r) * ((sample.t - t0) / max(1e-9, t1 - t0))
                y = pad_t + (height - pad_t - pad_b) * (1.0 - min(ymax, max(0.0, value)) / ymax)
                coords.extend((x, y))
            if len(coords) >= 4:
                canvas.create_line(*coords, fill=color, width=2)
            canvas.create_text(pad_l + 78 * series_index, height - 8, text=name, anchor="w", fill=color, font=("Segoe UI", 8))

    def on_scene_press(self, event: tk.Event) -> None:
        point = self._event_wall_point(event)
        self.mouse_down = True
        self.dragging_path = False
        self.draw_points = [point]
        self.drag_start_px = (float(event.x), float(event.y))
        self.drag_start_state = int(getattr(event, "state", 0))
        self.drag_start_was_playing = self.playing

    def on_scene_drag(self, event: tk.Event) -> None:
        if not self.mouse_down:
            return
        point = self._event_wall_point(event)
        dx = float(event.x) - self.drag_start_px[0]
        dy = float(event.y) - self.drag_start_px[1]
        world_motion = _distance2(point, self.draw_points[0]) if self.draw_points else 0.0
        if (
            not self.dragging_path
            and (
                math.hypot(dx, dy) >= self.drag_start_threshold_px
                or world_motion >= self.draw_min_spacing_m
            )
        ):
            self.dragging_path = True
            self.playing = False
            self.play_button.configure(text="Play")
        if self.dragging_path:
            self._append_draw_point(point)
            self.draw()

    def on_scene_release(self, event: tk.Event) -> None:
        if not self.mouse_down:
            return
        point = self._event_wall_point(event)
        self.mouse_down = False

        if self.dragging_path:
            self._append_draw_point(point)
            smooth_path = self._simplify_draw_points(self.draw_points)
            self.draw_points = []
            self.dragging_path = False
            if len(smooth_path) >= 2 and self._path_length(smooth_path) >= 0.08:
                self.sim.set_smooth_path(smooth_path)
                self.playing = True
                self.play_button.configure(text="Pause")
            else:
                self.playing = self.drag_start_was_playing
                self.play_button.configure(text="Pause" if self.playing else "Play")
            self.draw()
            return

        append = bool(self.drag_start_state & 0x0001)
        if append:
            self.sim.append_target(point, planner=BEST_PLANNER)
        else:
            self.sim.set_target(point, planner=BEST_PLANNER)
        self.draw_points = []
        self.playing = True
        self.play_button.configure(text="Pause")

    def _event_wall_point(self, event: tk.Event) -> Vec2:
        params = self.sim.params
        transform = SceneTransform(max(2, self.scene.winfo_width()), max(2, self.scene.winfo_height()), params.wall_width, params.wall_height)
        return self.sim._clamp_wall_point(transform.screen((float(event.x), float(event.y))))

    def _append_draw_point(self, point: Vec2) -> bool:
        if not self.draw_points or _distance2(point, self.draw_points[-1]) >= self.draw_min_spacing_m:
            self.draw_points.append(point)
            return True
        return False

    def _simplify_draw_points(self, points: Sequence[Vec2]) -> list[Vec2]:
        if not points:
            return []
        filtered = [points[0]]
        for point in points[1:]:
            if _distance2(point, filtered[-1]) >= self.draw_min_spacing_m:
                filtered.append(point)
        if _distance2(points[-1], filtered[-1]) >= 1e-6:
            filtered.append(points[-1])
        if len(filtered) <= self.draw_max_points:
            return filtered

        keep: list[Vec2] = []
        last_index = len(filtered) - 1
        for sample_index in range(self.draw_max_points):
            source_index = round(sample_index * last_index / max(1, self.draw_max_points - 1))
            point = filtered[source_index]
            if not keep or _distance2(point, keep[-1]) >= 1e-6:
                keep.append(point)
        return keep

    @staticmethod
    def _path_length(points: Sequence[Vec2]) -> float:
        return sum(_distance2(points[index], points[index - 1]) for index in range(1, len(points)))

    def toggle_play(self) -> None:
        self.playing = not self.playing
        self.play_button.configure(text="Pause" if self.playing else "Play")

    def reset_mission(self) -> None:
        self.sim = make_simulator()
        command_controller(self.sim, self.scenario.targets)
        self.draw_points = []
        self.mouse_down = False
        self.dragging_path = False
        self.playing = True
        self.play_button.configure(text="Pause")

    def reset_hold(self) -> None:
        self.sim = make_simulator()
        self.draw_points = []
        self.mouse_down = False
        self.dragging_path = False
        self.playing = False
        self.play_button.configure(text="Play")

    def clear_path(self) -> None:
        self.sim.clear_trajectory()
        self.draw_points = []
        self.mouse_down = False
        self.dragging_path = False

    def toggle_trace(self) -> None:
        self.show_trace = not self.show_trace
        self.trace_button.configure(text="Trace On" if self.show_trace else "Trace Off")

    def toggle_path(self) -> None:
        self.show_path = not self.show_path
        self.path_button.configure(text="Path On" if self.show_path else "Path Off")

    def toggle_forces(self) -> None:
        self.show_forces = not self.show_forces
        self.force_button.configure(text="Forces On" if self.show_forces else "Forces Off")


class SceneTransform:
    def __init__(self, width: int, height: int, wall_width: float, wall_height: float) -> None:
        margin = 36.0
        span_x = wall_width + 0.9
        span_z = wall_height + 0.6
        self.scale = min((width - 2.0 * margin) / span_x, (height - 2.0 * margin) / span_z)
        self.scale = max(1.0, self.scale)
        self.view_height = wall_height + 0.3
        self.ox = width / 2.0
        self.oy = margin

    def world(self, point: Vec2) -> tuple[float, float]:
        return self.ox + point[0] * self.scale, self.oy + (self.view_height - point[1]) * self.scale

    def screen(self, point: tuple[float, float]) -> Vec2:
        return (point[0] - self.ox) / self.scale, self.view_height - (point[1] - self.oy) / self.scale


def _distance2(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def run_tk_ui() -> int:
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise SystemExit(f"Could not start Tkinter UI: {exc}") from exc
    TkWallToolApp(root)
    root.mainloop()
    return 0
