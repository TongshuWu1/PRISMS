"""Explicit interfaces between the controller, sensors, and CoppeliaSim plant."""

from __future__ import annotations

import math
from dataclasses import dataclass


Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]


def _finite(name: str, values: tuple[float, ...]) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain only finite values")


@dataclass(frozen=True)
class PlantTruth:
    """Ground truth produced by CoppeliaSim; never passed directly to NMPC."""

    timestamp_s: float
    position_world_m: Vec3
    linear_velocity_world_m_s: Vec3
    orientation_world_rad: Vec3
    angular_velocity_world_rad_s: Vec3
    anchor_world_m: Vec3
    cable_mount_world_m: Vec3
    cable_mount_velocity_world_m_s: Vec3
    reel_length_m: float
    reel_velocity_m_s: float
    cable_tension_N: float
    left_servo_angle_rad: float
    right_servo_angle_rad: float
    left_thrust_N: float
    right_thrust_N: float
    # A shaft encoder reports rotation converted with a nominal drum radius.
    # With cable layering/backlash this differs from physical paid-out length.
    reel_encoder_length_m: float | None = None
    reel_encoder_velocity_m_s: float | None = None
    # The load cell is on the reel side of the guide pulley. Pulley friction
    # can make this differ from tension applied at the payload attachment.
    drum_tension_N: float | None = None

    def __post_init__(self) -> None:
        encoder_length = (
            self.reel_length_m
            if self.reel_encoder_length_m is None
            else self.reel_encoder_length_m
        )
        encoder_velocity = (
            self.reel_velocity_m_s
            if self.reel_encoder_velocity_m_s is None
            else self.reel_encoder_velocity_m_s
        )
        measured_drum_tension = (
            self.cable_tension_N
            if self.drum_tension_N is None
            else self.drum_tension_N
        )
        _finite("plant truth", (
            self.timestamp_s,
            *self.position_world_m,
            *self.linear_velocity_world_m_s,
            *self.orientation_world_rad,
            *self.angular_velocity_world_rad_s,
            *self.anchor_world_m,
            *self.cable_mount_world_m,
            *self.cable_mount_velocity_world_m_s,
            self.reel_length_m,
            self.reel_velocity_m_s,
            self.cable_tension_N,
            self.left_servo_angle_rad,
            self.right_servo_angle_rad,
            self.left_thrust_N,
            self.right_thrust_N,
            encoder_length,
            encoder_velocity,
            measured_drum_tension,
        ))

    @property
    def measured_reel_length_m(self) -> float:
        return (
            self.reel_length_m
            if self.reel_encoder_length_m is None
            else self.reel_encoder_length_m
        )

    @property
    def measured_reel_velocity_m_s(self) -> float:
        return (
            self.reel_velocity_m_s
            if self.reel_encoder_velocity_m_s is None
            else self.reel_encoder_velocity_m_s
        )

    @property
    def measured_load_cell_tension_N(self) -> float:
        return (
            self.cable_tension_N
            if self.drum_tension_N is None
            else self.drum_tension_N
        )


@dataclass(frozen=True)
class SensorEstimate:
    """Only feedback made available to the controller."""

    timestamp_s: float
    payload_position_xz_m: Vec2
    payload_velocity_xz_m_s: Vec2
    payload_attitude_rad: float
    payload_angular_rate_rad_s: float
    cable_angle_rad: float
    cable_angle_rate_rad_s: float
    geometric_cable_length_m: float
    reel_length_m: float
    reel_velocity_m_s: float
    cable_tension_N: float

    def __post_init__(self) -> None:
        _finite("sensor estimate", (
            self.timestamp_s,
            *self.payload_position_xz_m,
            *self.payload_velocity_xz_m_s,
            self.payload_attitude_rad,
            self.payload_angular_rate_rad_s,
            self.cable_angle_rad,
            self.cable_angle_rate_rad_s,
            self.geometric_cable_length_m,
            self.reel_length_m,
            self.reel_velocity_m_s,
            self.cable_tension_N,
        ))


@dataclass(frozen=True)
class ActuatorCommand:
    """The complete and only command interface exposed to the plant."""

    timestamp_s: float
    left_thrust_N: float
    right_thrust_N: float
    reel_velocity_m_s: float
    left_servo_angle_rad: float
    right_servo_angle_rad: float
    reference_position_xz_m: Vec2
    reference_velocity_xz_m_s: Vec2
    solver_status: str
    solver_time_s: float

    def __post_init__(self) -> None:
        _finite("actuator command", (
            self.timestamp_s,
            self.left_thrust_N,
            self.right_thrust_N,
            self.reel_velocity_m_s,
            self.left_servo_angle_rad,
            self.right_servo_angle_rad,
            *self.reference_position_xz_m,
            *self.reference_velocity_xz_m_s,
            self.solver_time_s,
        ))
        if not self.solver_status:
            raise ValueError("actuator command requires a solver status")
