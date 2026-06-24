#!/usr/bin/env python3
"""Mixed-input energy-shaping control for the wall-tool simulator.

The reel is treated as a radial velocity actuator. The drones are treated as
the tangential acceleration/force actuator. This keeps the controller aligned
with the physical split in the hardware instead of asking both subsystems to
fight the same Cartesian error.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class MixedInputEnergyCommand:
    radial_acceleration: float
    tangential_acceleration: float
    radial_position_error_m: float
    radial_velocity_error_m_s: float
    tangential_position_error_m: float
    tangential_velocity_error_m_s: float
    swing_energy_J: float
    swing_power_W: float
    clf_margin_W: float
    clf_projected_accel_m_s2: float


def mixed_input_energy_command(
    *,
    line_length_m: float,
    line_velocity_m_s: float,
    measured_theta_rad: float,
    measured_theta_dot_rad_s: float,
    target_length_m: float,
    target_length_dot_m_s: float,
    target_theta_rad: float,
    target_theta_dot_rad_s: float,
    reference_radial_accel_m_s2: float,
    reference_tangential_accel_m_s2: float,
    mass_kg: float,
    radial_frequency_rad_s: float,
    radial_damping_ratio: float,
    tangential_frequency_rad_s: float,
    tangential_damping_ratio: float,
    clf_decay_rate: float,
    max_radial_accel_m_s2: float,
    max_tangential_accel_m_s2: float,
) -> MixedInputEnergyCommand:
    """Return radial reel-compatible and tangential drone-compatible commands.

    The tangential command is a nominal second-order tracker projected through
    a one-step CLF inequality:

        dV/dt <= -lambda V

    where V is a tangential tracking storage function. This projection is the
    anti-swing part: if the nominal tracking command would add swing energy,
    the command is clipped to the closest acceleration that dissipates it.
    """

    line_length_m = max(1e-6, line_length_m)
    mass_kg = max(1e-9, mass_kg)
    radial_frequency_rad_s = max(0.0, radial_frequency_rad_s)
    radial_damping_ratio = max(0.0, radial_damping_ratio)
    tangential_frequency_rad_s = max(0.0, tangential_frequency_rad_s)
    tangential_damping_ratio = max(0.0, tangential_damping_ratio)
    clf_decay_rate = max(0.0, clf_decay_rate)

    radial_error = target_length_m - line_length_m
    radial_velocity_error = target_length_dot_m_s - line_velocity_m_s
    radial_accel = (
        reference_radial_accel_m_s2
        + radial_frequency_rad_s * radial_frequency_rad_s * radial_error
        + 2.0 * radial_damping_ratio * radial_frequency_rad_s * radial_velocity_error
    )
    radial_accel = clamp(radial_accel, -max_radial_accel_m_s2, max_radial_accel_m_s2)

    theta_error = wrap_angle(target_theta_rad - measured_theta_rad)
    theta_dot_error = target_theta_dot_rad_s - measured_theta_dot_rad_s
    tangential_error = line_length_m * theta_error
    tangential_velocity_error = line_length_m * theta_dot_error + line_velocity_m_s * theta_error

    tangential_omega_sq = tangential_frequency_rad_s * tangential_frequency_rad_s
    tangential_damping = 2.0 * tangential_damping_ratio * tangential_frequency_rad_s
    nominal_tangential_accel = (
        reference_tangential_accel_m_s2
        + tangential_omega_sq * tangential_error
        + tangential_damping * tangential_velocity_error
    )
    nominal_tangential_accel = clamp(
        nominal_tangential_accel,
        -max_tangential_accel_m_s2,
        max_tangential_accel_m_s2,
    )

    # Convert to actual-minus-reference errors for the storage derivative.
    actual_position_error = -tangential_error
    actual_velocity_error = -tangential_velocity_error
    storage_per_kg = 0.5 * (
        actual_velocity_error * actual_velocity_error
        + tangential_omega_sq * actual_position_error * actual_position_error
    )
    swing_energy = mass_kg * storage_per_kg

    lower = -max_tangential_accel_m_s2
    upper = max_tangential_accel_m_s2
    velocity_for_decay = abs(actual_velocity_error)
    position_speed_scale = tangential_frequency_rad_s * abs(actual_position_error)
    decay_fraction = velocity_for_decay / max(velocity_for_decay + position_speed_scale, 1e-9)
    decay_storage_per_kg = storage_per_kg * decay_fraction
    if abs(actual_velocity_error) > 0.025:
        rhs_per_kg = (
            -clf_decay_rate * decay_storage_per_kg
            - tangential_omega_sq * actual_position_error * actual_velocity_error
        )
        accel_bound = reference_tangential_accel_m_s2 + rhs_per_kg / actual_velocity_error
        if actual_velocity_error > 0.0:
            upper = min(upper, accel_bound)
        else:
            lower = max(lower, accel_bound)

    if lower <= upper:
        tangential_accel = clamp(nominal_tangential_accel, lower, upper)
    elif actual_velocity_error > 0.0:
        tangential_accel = -max_tangential_accel_m_s2
    else:
        tangential_accel = max_tangential_accel_m_s2

    storage_dot = mass_kg * (
        tangential_omega_sq * actual_position_error * actual_velocity_error
        + actual_velocity_error * (tangential_accel - reference_tangential_accel_m_s2)
    )
    clf_margin = -clf_decay_rate * mass_kg * decay_storage_per_kg - storage_dot

    return MixedInputEnergyCommand(
        radial_acceleration=radial_accel,
        tangential_acceleration=tangential_accel,
        radial_position_error_m=radial_error,
        radial_velocity_error_m_s=radial_velocity_error,
        tangential_position_error_m=tangential_error,
        tangential_velocity_error_m_s=tangential_velocity_error,
        swing_energy_J=swing_energy,
        swing_power_W=storage_dot,
        clf_margin_W=clf_margin,
        clf_projected_accel_m_s2=tangential_accel - nominal_tangential_accel,
    )
