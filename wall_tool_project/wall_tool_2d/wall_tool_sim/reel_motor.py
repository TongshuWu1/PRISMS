"""Velocity-controlled reel motor model for the wall-tool cable spool."""

from __future__ import annotations

import math
from dataclasses import dataclass


KG_CM_TO_N_M = 9.80665e-2


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class ReelMotorSpec:
    """12 V metal DC geared motor with encoder used as a reel velocity actuator.

    The controller sends a cable line-velocity command. The model limits that
    command using the gearmotor speed/torque envelope; it does not expose an
    acceleration command.
    """

    voltage_v: float = 12.0
    gear_ratio: float = 43.8
    no_load_output_rpm: float = 251.0
    stall_torque_kg_cm: float = 18.0
    spool_radius_m: float = 0.022
    velocity_time_constant_s: float = 0.045
    continuous_torque_fraction: float = 0.30

    def __post_init__(self) -> None:
        if self.voltage_v <= 0.0:
            raise ValueError("reel motor voltage must be positive")
        if self.gear_ratio <= 0.0:
            raise ValueError("reel motor gear ratio must be positive")
        if self.no_load_output_rpm <= 0.0:
            raise ValueError("reel motor no-load output RPM must be positive")
        if self.stall_torque_kg_cm <= 0.0:
            raise ValueError("reel motor stall torque must be positive")
        if self.spool_radius_m <= 0.0:
            raise ValueError("reel spool radius must be positive")
        if self.velocity_time_constant_s <= 0.0:
            raise ValueError("reel velocity time constant must be positive")
        if not 0.0 < self.continuous_torque_fraction <= 1.0:
            raise ValueError("reel continuous torque fraction must be within (0, 1]")

    @property
    def no_load_output_rad_s(self) -> float:
        return self.no_load_output_rpm * 2.0 * math.pi / 60.0

    @property
    def stall_torque_N_m(self) -> float:
        return self.stall_torque_kg_cm * KG_CM_TO_N_M

    @property
    def continuous_torque_N_m(self) -> float:
        return self.continuous_torque_fraction * self.stall_torque_N_m

    @property
    def max_line_speed_m_s(self) -> float:
        return self.no_load_output_rad_s * self.spool_radius_m

    @property
    def stall_line_force_N(self) -> float:
        return self.stall_torque_N_m / self.spool_radius_m

    @property
    def continuous_line_force_N(self) -> float:
        return self.continuous_torque_N_m / self.spool_radius_m

    def clamp_line_velocity_command(self, command_m_s: float, cable_tension_N: float) -> float:
        """Clamp a commanded line velocity by speed and torque capability.

        Positive velocity pays cable out. Negative velocity reels cable in, so
        cable tension consumes part of the gearmotor torque-speed envelope.
        """

        command = clamp(float(command_m_s), -self.max_line_speed_m_s, self.max_line_speed_m_s)
        tension = max(0.0, float(cable_tension_N))
        if command >= 0.0:
            return command
        reel_in_speed_fraction = clamp(1.0 - tension / max(self.stall_line_force_N, 1e-9), 0.0, 1.0)
        max_reel_in_speed = self.max_line_speed_m_s * reel_in_speed_fraction
        return max(command, -max_reel_in_speed)

    def velocity_step(self, current_m_s: float, command_m_s: float, cable_tension_N: float, dt_s: float) -> float:
        target = self.clamp_line_velocity_command(command_m_s, cable_tension_N)
        alpha = clamp(float(dt_s) / (self.velocity_time_constant_s + float(dt_s)), 0.0, 1.0)
        return float(current_m_s) + alpha * (target - float(current_m_s))

    def line_speed_to_output_rpm(self, line_speed_m_s: float) -> float:
        omega = float(line_speed_m_s) / self.spool_radius_m
        return omega * 60.0 / (2.0 * math.pi)
