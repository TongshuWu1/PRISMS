"""Reusable Tk plot widgets for controller experiment UIs."""

from __future__ import annotations

import math
import tkinter as tk


RPM_PER_RAD_PER_SEC = 60.0 / (2.0 * math.pi)


def vector_norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


class StripChart:
    def __init__(
        self,
        parent: tk.Widget,
        title: str,
        series: list[tuple[str, str, str]],
        window: float,
        unit: str = "",
        symmetric: bool = False,
        fixed_range: tuple[float, float] | None = None,
        height: int = 118,
    ) -> None:
        self.series = series
        self.window = window
        self.unit = unit
        self.symmetric = symmetric
        self.fixed_range = fixed_range
        self.samples: list[tuple[float, dict[str, float]]] = []
        self.frame = tk.LabelFrame(parent, text=title, padx=4, pady=4)
        self.canvas = tk.Canvas(self.frame, height=height, bg="#111821", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

    def pack(self, **kwargs) -> None:
        self.frame.pack(**kwargs)

    def reset(self) -> None:
        self.samples.clear()
        self.canvas.delete("all")

    def add_sample(self, sim_time: float, values: dict[str, float]) -> None:
        self.samples.append((sim_time, values))
        cutoff = sim_time - self.window
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.pop(0)
        self.draw()

    def draw(self) -> None:
        width = max(self.canvas.winfo_width(), 320)
        height = max(self.canvas.winfo_height(), 90)
        left, right, top, bottom = 48, width - 10, 16, height - 24
        plot_width = max(1, right - left)
        plot_height = max(1, bottom - top)

        self.canvas.delete("all")
        self.canvas.create_rectangle(0, 0, width, height, fill="#111821", outline="")
        self.canvas.create_rectangle(left, top, right, bottom, outline="#405166")

        visible_values = [
            values[key]
            for _time_value, values in self.samples
            for _label, key, _color in self.series
            if key in values
        ]
        if self.fixed_range is not None:
            y_min, y_max = self.fixed_range
        elif visible_values:
            y_min, y_max = min(visible_values), max(visible_values)
            if self.symmetric:
                max_abs = max(abs(y_min), abs(y_max), 1e-6)
                y_min, y_max = -max_abs, max_abs
            else:
                margin = max((y_max - y_min) * 0.12, 1e-3)
                y_min -= margin
                y_max += margin
        else:
            y_min, y_max = (-1.0, 1.0) if self.fixed_range is None else self.fixed_range
        if abs(y_max - y_min) < 1e-9:
            y_min -= 1.0
            y_max += 1.0

        for index in range(5):
            ratio = index / 4.0
            y = bottom - ratio * plot_height
            value = y_min + ratio * (y_max - y_min)
            self.canvas.create_line(left, y, right, y, fill="#233040")
            self.canvas.create_text(left - 6, y, text=f"{value:.2g}", fill="#b9c4cf", anchor="e", font=("Consolas", 8))

        if y_min < 0.0 < y_max:
            zero_y = bottom - (0.0 - y_min) / (y_max - y_min) * plot_height
            self.canvas.create_line(left, zero_y, right, zero_y, fill="#718399", dash=(4, 3))

        if not self.samples:
            return

        t_right = self.samples[-1][0]
        t_left = t_right - self.window
        self.canvas.create_text(left, bottom + 13, text=f"-{self.window:.1f}s", fill="#9fa9b5", anchor="w", font=("Consolas", 8))
        self.canvas.create_text(right, bottom + 13, text="now", fill="#9fa9b5", anchor="e", font=("Consolas", 8))

        legend_x = left + 4
        legend_spacing = max(66, min(118, (right - left) / max(len(self.series), 1)))
        latest_values = self.samples[-1][1]
        for label, key, color in self.series:
            latest = latest_values.get(key)
            legend_text = f"{label}={latest:.2f}{self.unit}" if latest is not None else label
            self.canvas.create_text(legend_x, top - 8, text=legend_text, fill=color, anchor="w", font=("Consolas", 8))
            legend_x += legend_spacing

        for _label, key, color in self.series:
            points: list[float] = []
            for sample_time, values in self.samples:
                if key not in values:
                    continue
                x = left + (sample_time - t_left) / self.window * plot_width
                y = bottom - (values[key] - y_min) / (y_max - y_min) * plot_height
                points.extend([x, y])
            if len(points) >= 4:
                self.canvas.create_line(points, fill=color, width=2)


class MotorRpmBars:
    def __init__(self, parent: tk.Widget, max_motor_speed: float) -> None:
        self.max_rpm = max_motor_speed * RPM_PER_RAD_PER_SEC
        self.values = [0.0, 0.0, 0.0, 0.0]
        self.frame = tk.LabelFrame(parent, text="Motor RPM Bars", padx=4, pady=4)
        self.canvas = tk.Canvas(self.frame, height=95, bg="#111821", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

    def pack(self, **kwargs) -> None:
        self.frame.pack(**kwargs)

    def reset(self) -> None:
        self.values = [0.0, 0.0, 0.0, 0.0]
        self.draw()

    def update(self, rpm_values: list[float]) -> None:
        self.values = rpm_values[:4]
        self.draw()

    def draw(self) -> None:
        width = max(self.canvas.winfo_width(), 320)
        height = max(self.canvas.winfo_height(), 80)
        left, right, top, bottom = 42, width - 12, 12, height - 25
        colors = ["#59a14f", "#4e79a7", "#f28e2b", "#e15759"]
        self.canvas.delete("all")
        self.canvas.create_rectangle(0, 0, width, height, fill="#111821", outline="")
        self.canvas.create_rectangle(left, top, right, bottom, outline="#405166")
        self.canvas.create_text(left - 8, top, text=f"{self.max_rpm:.0f}", fill="#b9c4cf", anchor="e", font=("Consolas", 8))
        self.canvas.create_text(left - 8, bottom, text="0", fill="#b9c4cf", anchor="e", font=("Consolas", 8))

        slot = (right - left) / 4.0
        for index, rpm in enumerate(self.values):
            x0 = left + index * slot + slot * 0.18
            x1 = left + (index + 1) * slot - slot * 0.18
            ratio = max(0.0, min(1.0, rpm / max(self.max_rpm, 1.0)))
            y0 = bottom - ratio * (bottom - top)
            self.canvas.create_rectangle(x0, y0, x1, bottom, fill=colors[index], outline="")
            self.canvas.create_text((x0 + x1) * 0.5, bottom + 13, text=f"M{index}", fill="#d8dee6", font=("Consolas", 8))
            self.canvas.create_text((x0 + x1) * 0.5, max(top + 8, y0 - 8), text=f"{rpm:.0f}", fill="#d8dee6", font=("Consolas", 8))

