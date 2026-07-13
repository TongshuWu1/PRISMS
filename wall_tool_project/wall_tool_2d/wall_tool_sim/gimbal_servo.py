"""Second-order 270-degree position-servo model for thrust vectoring.

The payload carries two ordinary position servos.  Each servo directly rotates
one propeller axis in the wall-parallel x-z plane.  The flight computer sends a
PWM position command; the servo's potentiometer belongs to its sealed internal
position loop and is not exposed as a controller sensor.  The plant models PWM
command quantization, finite bandwidth, rate, acceleration, and travel.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GimbalServoSpec:
    """Physical limits for one 270-degree geared position servo."""

    max_angle_rad: float = math.radians(135.0)
    max_rate_rad_s: float = math.radians(240.0)
    max_acceleration_rad_s2: float = math.radians(1200.0)
    natural_frequency_rad_s: float = 18.0
    damping_ratio: float = 0.85
    command_min_pulse_us: float = 500.0
    command_max_pulse_us: float = 2500.0
    command_resolution_us: float = 1.0

    def __post_init__(self) -> None:
        finite_positive = {
            "maximum angle": self.max_angle_rad,
            "maximum rate": self.max_rate_rad_s,
            "maximum acceleration": self.max_acceleration_rad_s2,
            "natural frequency": self.natural_frequency_rad_s,
            "damping ratio": self.damping_ratio,
        }
        for name, value in finite_positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"gimbal {name} must be finite and positive")
        if not math.isfinite(self.command_min_pulse_us):
            raise ValueError("servo minimum command pulse must be finite")
        if not math.isfinite(self.command_max_pulse_us):
            raise ValueError("servo maximum command pulse must be finite")
        if self.command_max_pulse_us <= self.command_min_pulse_us:
            raise ValueError("servo command pulse range must be positive")
        if not math.isfinite(self.command_resolution_us) or self.command_resolution_us <= 0.0:
            raise ValueError("servo command pulse resolution must be finite and positive")
        if self.command_resolution_us > self.command_max_pulse_us - self.command_min_pulse_us:
            raise ValueError("servo command pulse resolution exceeds its pulse range")

    @property
    def command_resolution_rad(self) -> float:
        pulse_span = self.command_max_pulse_us - self.command_min_pulse_us
        return 2.0 * self.max_angle_rad * self.command_resolution_us / pulse_span

    def validate_command(self, command_rad: float, *, tolerance_rad: float = 1.0e-7) -> float:
        command = float(command_rad)
        if not math.isfinite(command):
            raise ValueError("gimbal angle command must be finite")
        if abs(command) > self.max_angle_rad + tolerance_rad:
            raise ValueError(
                f"gimbal angle command {math.degrees(command):.3f} deg exceeds "
                f"the {math.degrees(self.max_angle_rad):.3f} deg travel limit"
            )
        return max(-self.max_angle_rad, min(self.max_angle_rad, command))

    def realize_pwm_command(self, command_rad: float) -> float:
        """Convert a requested shaft angle through the finite-resolution PWM interface."""

        command = self.validate_command(command_rad)
        fraction = (command + self.max_angle_rad) / (2.0 * self.max_angle_rad)
        pulse = self.command_min_pulse_us + fraction * (
            self.command_max_pulse_us - self.command_min_pulse_us
        )
        quantized_pulse = (
            self.command_min_pulse_us
            + round((pulse - self.command_min_pulse_us) / self.command_resolution_us)
            * self.command_resolution_us
        )
        quantized_pulse = max(
            self.command_min_pulse_us,
            min(self.command_max_pulse_us, quantized_pulse),
        )
        quantized_fraction = (quantized_pulse - self.command_min_pulse_us) / (
            self.command_max_pulse_us - self.command_min_pulse_us
        )
        return -self.max_angle_rad + 2.0 * self.max_angle_rad * quantized_fraction

    def acceleration_rad_s2(self, angle_rad: float, rate_rad_s: float, command_rad: float) -> float:
        command = self.validate_command(command_rad)
        raw = (
            self.natural_frequency_rad_s**2 * (command - float(angle_rad))
            - 2.0 * self.damping_ratio * self.natural_frequency_rad_s * float(rate_rad_s)
        )
        return max(-self.max_acceleration_rad_s2, min(self.max_acceleration_rad_s2, raw))

    def step(
        self,
        angle_rad: float,
        rate_rad_s: float,
        command_rad: float,
        dt_s: float,
    ) -> tuple[float, float, float, bool]:
        """Advance the servo with semi-implicit Euler integration.

        Returns ``(angle, rate, acceleration, saturated)``.  Out-of-range
        commands raise rather than being silently accepted.
        """

        dt = float(dt_s)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("gimbal integration step must be finite and positive")
        acceleration = self.acceleration_rad_s2(angle_rad, rate_rad_s, command_rad)
        raw_acceleration = (
            self.natural_frequency_rad_s**2 * (float(command_rad) - float(angle_rad))
            - 2.0 * self.damping_ratio * self.natural_frequency_rad_s * float(rate_rad_s)
        )
        saturated = abs(raw_acceleration) > self.max_acceleration_rad_s2 + 1.0e-12
        rate = float(rate_rad_s) + acceleration * dt
        if abs(rate) > self.max_rate_rad_s:
            rate = math.copysign(self.max_rate_rad_s, rate)
            saturated = True
        angle = float(angle_rad) + rate * dt
        if angle > self.max_angle_rad:
            angle = self.max_angle_rad
            rate = min(0.0, rate)
            saturated = True
        elif angle < -self.max_angle_rad:
            angle = -self.max_angle_rad
            rate = max(0.0, rate)
            saturated = True
        return angle, rate, acceleration, saturated
