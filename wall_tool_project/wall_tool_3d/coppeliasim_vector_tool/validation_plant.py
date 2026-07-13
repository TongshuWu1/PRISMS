"""Independent validation-plant models for the CoppeliaSim experiment.

This module deliberately does not import the NMPC model, ``SteelCableSpec``,
``ReelMotorSpec``, or ``GimbalServoSpec``.  The validation plant therefore has
its own parameter contract and its own state equations.  A controller can be
tested against this plant without silently sharing its actuator/cable model.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from wall_tool_sim.wall_tool_ui import SimParams


SCHEMA_VERSION = 1


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _finite_positive(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class CableValidationParameters:
    diameter_m: float
    axial_rigidity_N: float
    mass_per_length_kg_m: float
    termination_compliance_m_N: float
    damping_ratio: float
    nonlinear_stiffening_per_m: float
    relaxation_fraction: float
    relaxation_time_constant_s: float
    payload_weight_fraction: float
    pulley_friction_coefficient: float
    pulley_wrap_angle_rad: float
    pulley_slip_transition_m_s: float
    transverse_damping_ratio: float
    transverse_crosswind_N_m: float
    transverse_gust_period_s: float
    max_transverse_displacement_m: float

    def __post_init__(self) -> None:
        for name in (
            "diameter_m", "axial_rigidity_N", "mass_per_length_kg_m",
            "relaxation_time_constant_s", "pulley_slip_transition_m_s",
            "transverse_gust_period_s", "max_transverse_displacement_m",
        ):
            _finite_positive(f"cable {name}", getattr(self, name))
        if self.termination_compliance_m_N < 0.0:
            raise ValueError("cable termination compliance cannot be negative")
        if self.damping_ratio < 0.0 or self.transverse_damping_ratio < 0.0:
            raise ValueError("cable damping ratios cannot be negative")
        if self.nonlinear_stiffening_per_m < 0.0:
            raise ValueError("cable nonlinear stiffening cannot be negative")
        if not 0.0 <= self.relaxation_fraction < 1.0:
            raise ValueError("cable relaxation fraction must be in [0, 1)")
        if not 0.0 <= self.payload_weight_fraction <= 1.0:
            raise ValueError("cable payload weight fraction must be in [0, 1]")
        if self.pulley_friction_coefficient < 0.0 or self.pulley_wrap_angle_rad < 0.0:
            raise ValueError("pulley friction and wrap angle cannot be negative")
        if self.transverse_crosswind_N_m < 0.0:
            raise ValueError("cable transverse crosswind cannot be negative")

    def linear_stiffness_N_m(self, length_m: float) -> float:
        compliance = max(float(length_m), 1e-6) / self.axial_rigidity_N
        compliance += self.termination_compliance_m_N
        return 1.0 / max(compliance, 1e-12)

    def extension_for_tension_m(self, tension_N: float, length_m: float) -> float:
        """Static total extension, including the relaxed viscoelastic branch."""

        target = max(float(tension_N), 0.0)
        k = self.linear_stiffness_N_m(length_m)
        retained = 1.0 - self.relaxation_fraction
        nonlinear = self.nonlinear_stiffening_per_m
        if target <= 0.0:
            return 0.0
        if nonlinear <= 1e-12:
            elastic = target / k
        else:
            elastic = (-1.0 + math.sqrt(1.0 + 4.0 * nonlinear * target / k)) / (2.0 * nonlinear)
        return elastic / retained


@dataclass(frozen=True)
class ReelValidationParameters:
    encoder_radius_m: float
    drum_core_radius_m: float
    drum_width_m: float
    cable_pack_fill_fraction: float
    wound_cable_at_reference_m: float
    output_stall_torque_N_m: float
    no_load_output_speed_rad_s: float
    gearbox_efficiency: float
    velocity_time_constant_s: float
    command_gain: float
    command_deadband_m_s: float
    reversal_backlash_m: float
    coulomb_friction_torque_N_m: float

    def __post_init__(self) -> None:
        for name in (
            "encoder_radius_m", "drum_core_radius_m", "drum_width_m",
            "wound_cable_at_reference_m", "output_stall_torque_N_m",
            "no_load_output_speed_rad_s", "velocity_time_constant_s", "command_gain",
        ):
            _finite_positive(f"reel {name}", getattr(self, name))
        if not 0.0 < self.cable_pack_fill_fraction <= 1.0:
            raise ValueError("reel cable pack fill fraction must be in (0, 1]")
        if not 0.0 < self.gearbox_efficiency <= 1.0:
            raise ValueError("reel gearbox efficiency must be in (0, 1]")
        if self.command_deadband_m_s < 0.0 or self.reversal_backlash_m < 0.0:
            raise ValueError("reel deadband and backlash cannot be negative")
        if self.coulomb_friction_torque_N_m < 0.0:
            raise ValueError("reel Coulomb friction cannot be negative")


@dataclass(frozen=True)
class RotorValidationParameters:
    left_gain: float
    right_gain: float
    left_time_constant_s: float
    right_time_constant_s: float
    command_lag_time_constant_s: float
    left_max_thrust_N: float
    right_max_thrust_N: float
    zero_command_deadband_N: float

    def __post_init__(self) -> None:
        for name in (
            "left_gain", "right_gain", "left_time_constant_s", "right_time_constant_s",
            "command_lag_time_constant_s", "left_max_thrust_N", "right_max_thrust_N",
        ):
            _finite_positive(f"rotor {name}", getattr(self, name))
        if self.zero_command_deadband_N < 0.0:
            raise ValueError("rotor command deadband cannot be negative")


@dataclass(frozen=True)
class ServoValidationParameters:
    max_angle_rad: float
    max_rate_rad_s: float
    max_acceleration_rad_s2: float
    natural_frequency_rad_s: float
    damping_ratio: float
    left_zero_error_rad: float
    right_zero_error_rad: float
    backlash_rad: float
    command_deadband_rad: float

    def __post_init__(self) -> None:
        for name in (
            "max_angle_rad", "max_rate_rad_s", "max_acceleration_rad_s2",
            "natural_frequency_rad_s", "damping_ratio",
        ):
            _finite_positive(f"servo {name}", getattr(self, name))
        for name in ("left_zero_error_rad", "right_zero_error_rad", "backlash_rad", "command_deadband_rad"):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError(f"servo {name} must be finite")
        if self.backlash_rad < 0.0 or self.command_deadband_rad < 0.0:
            raise ValueError("servo backlash and deadband cannot be negative")


@dataclass(frozen=True)
class ValidationPlantProfile:
    profile_name: str
    calibrated: bool
    provenance: dict[str, str]
    cable: CableValidationParameters
    reel: ReelValidationParameters
    rotor: RotorValidationParameters
    servo: ServoValidationParameters

    def __post_init__(self) -> None:
        if not self.profile_name.strip():
            raise ValueError("validation profile requires a name")
        if not isinstance(self.provenance, dict):
            raise ValueError("validation profile provenance must be a mapping")

    def to_json_dict(self) -> dict[str, Any]:
        return {"schema_version": SCHEMA_VERSION, **asdict(self)}


def datasheet_validation_profile(params: SimParams) -> ValidationPlantProfile:
    """Return an uncalibrated, independent profile from component datasheets.

    Values intentionally include plausible assembly losses and left/right
    asymmetry.  They are engineering assumptions, not claimed measurements.
    """

    diameter = float(params.steel_cable_diameter_m)
    cable_area = math.pi * (0.5 * diameter) ** 2
    stall_torque = float(params.reel_motor_stall_torque_kg_cm) * 9.80665e-2
    no_load_speed = float(params.reel_motor_no_load_rpm) * 2.0 * math.pi / 60.0
    return ValidationPlantProfile(
        profile_name="datasheet-independent-v1",
        calibrated=False,
        provenance={
            "kind": "datasheet-and-engineering-assumptions",
            "warning": "not hardware calibrated",
        },
        cable=CableValidationParameters(
            diameter_m=diameter,
            axial_rigidity_N=float(params.steel_cable_youngs_modulus_pa) * cable_area,
            mass_per_length_kg_m=float(params.steel_cable_density_kg_m3) * cable_area,
            termination_compliance_m_N=1.08 * float(params.steel_cable_structural_compliance_m_N),
            damping_ratio=0.18,
            nonlinear_stiffening_per_m=30.0,
            relaxation_fraction=0.035,
            relaxation_time_constant_s=2.8,
            payload_weight_fraction=float(params.steel_cable_payload_weight_fraction),
            pulley_friction_coefficient=0.025,
            pulley_wrap_angle_rad=0.5 * math.pi,
            pulley_slip_transition_m_s=0.008,
            transverse_damping_ratio=0.035,
            transverse_crosswind_N_m=0.0015,
            transverse_gust_period_s=7.3,
            max_transverse_displacement_m=0.025,
        ),
        reel=ReelValidationParameters(
            encoder_radius_m=float(params.reel_spool_radius_m),
            drum_core_radius_m=0.0195,
            drum_width_m=0.032,
            cable_pack_fill_fraction=0.82,
            wound_cable_at_reference_m=7.5,
            output_stall_torque_N_m=stall_torque,
            no_load_output_speed_rad_s=no_load_speed,
            gearbox_efficiency=0.78,
            velocity_time_constant_s=1.18 * float(params.reel_velocity_time_constant_s),
            command_gain=0.985,
            command_deadband_m_s=0.0012,
            reversal_backlash_m=0.00030,
            coulomb_friction_torque_N_m=0.012,
        ),
        rotor=RotorValidationParameters(
            left_gain=0.985,
            right_gain=1.012,
            left_time_constant_s=1.14 * float(params.motor_thrust_time_constant_s),
            right_time_constant_s=1.06 * float(params.motor_thrust_time_constant_s),
            command_lag_time_constant_s=0.012,
            left_max_thrust_N=0.97 * float(params.max_thrust_per_drone),
            right_max_thrust_N=0.99 * float(params.max_thrust_per_drone),
            zero_command_deadband_N=0.006,
        ),
        servo=ServoValidationParameters(
            max_angle_rad=float(params.gimbal_max_angle_rad),
            max_rate_rad_s=0.94 * float(params.gimbal_max_rate_rad_s),
            max_acceleration_rad_s2=0.90 * float(params.gimbal_max_acceleration_rad_s2),
            natural_frequency_rad_s=0.92 * float(params.gimbal_natural_frequency_rad_s),
            damping_ratio=1.06 * float(params.gimbal_damping_ratio),
            left_zero_error_rad=float(params.gimbal_left_zero_error_rad),
            right_zero_error_rad=float(params.gimbal_right_zero_error_rad),
            backlash_rad=math.radians(0.16),
            command_deadband_rad=math.radians(0.08),
        ),
    )


_SHA256 = re.compile(r"^[0-9a-fA-F]{64}$")


def load_calibrated_validation_profile(path: Path) -> ValidationPlantProfile:
    """Load a strict hardware-identified profile and reject placeholders."""

    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f"calibrated validation profile does not exist: {source}")
    try:
        data = json.loads(source.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid validation profile JSON {source}: {exc}") from exc
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"validation profile schema_version must be {SCHEMA_VERSION}, "
            f"got {data.get('schema_version')!r}"
        )
    if data.get("calibrated") is not True:
        raise ValueError("calibrated mode refuses profiles not explicitly marked calibrated=true")
    provenance = data.get("provenance")
    required_provenance = {"hardware_id", "recorded_utc", "raw_data_sha256"}
    if not isinstance(provenance, dict) or not required_provenance.issubset(provenance):
        raise ValueError(
            "calibrated profile provenance requires hardware_id, recorded_utc, and raw_data_sha256"
        )
    if not _SHA256.fullmatch(str(provenance["raw_data_sha256"])):
        raise ValueError("calibrated profile raw_data_sha256 must contain exactly 64 hexadecimal digits")
    try:
        return ValidationPlantProfile(
            profile_name=str(data["profile_name"]),
            calibrated=True,
            provenance={str(k): str(v) for k, v in provenance.items()},
            cable=CableValidationParameters(**data["cable"]),
            reel=ReelValidationParameters(**data["reel"]),
            rotor=RotorValidationParameters(**data["rotor"]),
            servo=ServoValidationParameters(**data["servo"]),
        )
    except (KeyError, TypeError) as exc:
        raise ValueError(f"calibrated validation profile is incomplete: {exc}") from exc


@dataclass
class ReelState:
    paid_out_length_m: float = 0.0
    encoder_length_m: float = 0.0
    line_velocity_m_s: float = 0.0
    encoder_velocity_m_s: float = 0.0
    output_speed_rad_s: float = 0.0
    reference_paid_out_length_m: float = 0.0
    backlash_remaining_m: float = 0.0
    last_direction: int = 0
    initialized: bool = False


class IndependentReelModel:
    def __init__(self, parameters: ReelValidationParameters, cable_diameter_m: float) -> None:
        self.parameters = parameters
        self.cable_area_m2 = math.pi * (0.5 * cable_diameter_m) ** 2
        self.state = ReelState()

    def initialize(self, paid_out_length_m: float) -> None:
        length = float(paid_out_length_m)
        self.state = ReelState(
            paid_out_length_m=length,
            encoder_length_m=length,
            reference_paid_out_length_m=length,
            initialized=True,
        )

    def effective_radius_m(self) -> float:
        if not self.state.initialized:
            raise RuntimeError("reel model must be initialized before use")
        p = self.parameters
        wound = max(
            0.0,
            p.wound_cable_at_reference_m
            - (self.state.paid_out_length_m - self.state.reference_paid_out_length_m),
        )
        added_area = self.cable_area_m2 * wound / (
            math.pi * p.drum_width_m * p.cable_pack_fill_fraction
        )
        return math.sqrt(p.drum_core_radius_m ** 2 + added_area)

    def step(
        self,
        command_line_speed_m_s: float,
        cable_tension_N: float,
        dt_s: float,
        minimum_length_m: float,
        maximum_length_m: float,
    ) -> ReelState:
        if not self.state.initialized:
            raise RuntimeError("reel model must be initialized before stepping")
        dt = float(dt_s)
        _finite_positive("reel step dt", dt)
        p = self.parameters
        command = float(command_line_speed_m_s) * p.command_gain
        if abs(command) < p.command_deadband_m_s:
            command = 0.0
        target_speed = command / p.encoder_radius_m
        load_torque = max(float(cable_tension_N), 0.0) * self.effective_radius_m()
        available_stall = max(
            p.gearbox_efficiency * p.output_stall_torque_N_m - p.coulomb_friction_torque_N_m,
            1e-9,
        )
        load_fraction = _clamp(load_torque / available_stall, 0.0, 1.0)
        loaded_speed_limit = p.no_load_output_speed_rad_s * (1.0 - load_fraction)
        target_speed = _clamp(target_speed, -loaded_speed_limit, loaded_speed_limit)
        alpha = _clamp(dt / (p.velocity_time_constant_s + dt), 0.0, 1.0)
        self.state.output_speed_rad_s += alpha * (target_speed - self.state.output_speed_rad_s)
        self.state.encoder_velocity_m_s = self.state.output_speed_rad_s * p.encoder_radius_m
        self.state.encoder_length_m += self.state.encoder_velocity_m_s * dt

        radius = self.effective_radius_m()
        shaft_line_delta = self.state.output_speed_rad_s * radius * dt
        direction = 1 if shaft_line_delta > 1e-12 else -1 if shaft_line_delta < -1e-12 else 0
        if direction and self.state.last_direction and direction != self.state.last_direction:
            self.state.backlash_remaining_m = p.reversal_backlash_m
        if direction:
            self.state.last_direction = direction
        transmitted_delta = shaft_line_delta
        if self.state.backlash_remaining_m > 0.0:
            consumed = min(abs(transmitted_delta), self.state.backlash_remaining_m)
            self.state.backlash_remaining_m -= consumed
            transmitted_delta = math.copysign(max(0.0, abs(transmitted_delta) - consumed), transmitted_delta)
        previous = self.state.paid_out_length_m
        self.state.paid_out_length_m = _clamp(
            previous + transmitted_delta,
            minimum_length_m,
            maximum_length_m,
        )
        self.state.line_velocity_m_s = (self.state.paid_out_length_m - previous) / dt
        return self.state


@dataclass
class CableState:
    relaxation_extension_m: float = 0.0
    transverse_displacement_m: float = 0.0
    transverse_velocity_m_s: float = 0.0
    initialized: bool = False


@dataclass(frozen=True)
class CableResponse:
    payload_tension_N: float
    drum_tension_N: float
    tangent_slope: float
    stiffness_N_m: float
    extension_m: float


class IndependentCableModel:
    def __init__(self, parameters: CableValidationParameters) -> None:
        self.parameters = parameters
        self.state = CableState()

    def initialize(self, extension_m: float) -> None:
        extension = max(float(extension_m), 0.0)
        self.state = CableState(
            relaxation_extension_m=self.parameters.relaxation_fraction * extension,
            initialized=True,
        )

    def step(
        self,
        distance_m: float,
        paid_out_length_m: float,
        distance_rate_m_s: float,
        reel_line_velocity_m_s: float,
        effective_mass_kg: float,
        maximum_tension_N: float,
        taut_band_m: float,
        timestamp_s: float,
        dt_s: float,
    ) -> CableResponse:
        if not self.state.initialized:
            raise RuntimeError("cable model must be initialized before stepping")
        dt = float(dt_s)
        _finite_positive("cable step dt", dt)
        p = self.parameters
        length = max(float(distance_m), 1e-6)
        extension = length - float(paid_out_length_m)
        target_relaxation = p.relaxation_fraction * max(extension, 0.0)
        relaxation_alpha = _clamp(dt / (p.relaxation_time_constant_s + dt), 0.0, 1.0)
        self.state.relaxation_extension_m += relaxation_alpha * (
            target_relaxation - self.state.relaxation_extension_m
        )
        elastic_extension = max(0.0, extension - self.state.relaxation_extension_m)
        stiffness = p.linear_stiffness_N_m(length)
        spring_tension = stiffness * elastic_extension * (
            1.0 + p.nonlinear_stiffening_per_m * elastic_extension
        )
        damping = 2.0 * p.damping_ratio * math.sqrt(
            stiffness * max(float(effective_mass_kg), 1e-6)
        )
        relative_speed = float(distance_rate_m_s) - float(reel_line_velocity_m_s)
        raw_payload_tension = spring_tension + damping * relative_speed
        if extension < -float(taut_band_m):
            raw_payload_tension = 0.0
        payload_tension = _clamp(raw_payload_tension, 0.0, maximum_tension_N)

        slip = math.tanh(relative_speed / p.pulley_slip_transition_m_s)
        capstan_ratio = math.exp(p.pulley_friction_coefficient * p.pulley_wrap_angle_rad * slip)
        drum_tension = _clamp(payload_tension * capstan_ratio, 0.0, maximum_tension_N)

        modal_mass = max(0.5 * p.mass_per_length_kg_m * length, 1e-6)
        wave_frequency = math.pi / length * math.sqrt(
            max(payload_tension, 0.05) / p.mass_per_length_kg_m
        )
        wave_frequency = _clamp(wave_frequency, 1.0, 45.0)
        distributed_force = p.transverse_crosswind_N_m * length * (
            0.60 * math.sin(2.0 * math.pi * timestamp_s / p.transverse_gust_period_s)
            + 0.25 * math.sin(2.0 * math.pi * timestamp_s / (0.43 * p.transverse_gust_period_s) + 0.7)
        )
        modal_force = (2.0 / math.pi) * distributed_force
        acceleration = (
            modal_force / modal_mass
            - 2.0 * p.transverse_damping_ratio * wave_frequency * self.state.transverse_velocity_m_s
            - wave_frequency * wave_frequency * self.state.transverse_displacement_m
        )
        self.state.transverse_velocity_m_s += acceleration * dt
        self.state.transverse_displacement_m = _clamp(
            self.state.transverse_displacement_m + self.state.transverse_velocity_m_s * dt,
            -p.max_transverse_displacement_m,
            p.max_transverse_displacement_m,
        )
        tangent_slope = -math.pi * self.state.transverse_displacement_m / length
        return CableResponse(
            payload_tension_N=payload_tension,
            drum_tension_N=drum_tension,
            tangent_slope=tangent_slope,
            stiffness_N_m=stiffness,
            extension_m=extension,
        )


@dataclass
class RotorState:
    left_thrust_N: float = 0.0
    right_thrust_N: float = 0.0
    left_lag_command_N: float = 0.0
    right_lag_command_N: float = 0.0


class IndependentRotorModel:
    def __init__(self, parameters: RotorValidationParameters) -> None:
        self.parameters = parameters
        self.state = RotorState()

    def initialize(self, left_thrust_N: float, right_thrust_N: float) -> None:
        self.state = RotorState(
            left_thrust_N=float(left_thrust_N),
            right_thrust_N=float(right_thrust_N),
            left_lag_command_N=float(left_thrust_N),
            right_lag_command_N=float(right_thrust_N),
        )

    def step(self, left_command_N: float, right_command_N: float, dt_s: float) -> RotorState:
        dt = float(dt_s)
        _finite_positive("rotor step dt", dt)
        p = self.parameters
        lag_alpha = _clamp(dt / (p.command_lag_time_constant_s + dt), 0.0, 1.0)
        commands = [float(left_command_N), float(right_command_N)]
        commands = [0.0 if value < p.zero_command_deadband_N else value for value in commands]
        self.state.left_lag_command_N += lag_alpha * (commands[0] - self.state.left_lag_command_N)
        self.state.right_lag_command_N += lag_alpha * (commands[1] - self.state.right_lag_command_N)
        left_target = _clamp(p.left_gain * self.state.left_lag_command_N, 0.0, p.left_max_thrust_N)
        right_target = _clamp(p.right_gain * self.state.right_lag_command_N, 0.0, p.right_max_thrust_N)
        left_alpha = _clamp(dt / (p.left_time_constant_s + dt), 0.0, 1.0)
        right_alpha = _clamp(dt / (p.right_time_constant_s + dt), 0.0, 1.0)
        self.state.left_thrust_N += left_alpha * (left_target - self.state.left_thrust_N)
        self.state.right_thrust_N += right_alpha * (right_target - self.state.right_thrust_N)
        return self.state


@dataclass
class ServoAxisState:
    angle_rad: float = 0.0
    rate_rad_s: float = 0.0
    play_center_rad: float = 0.0


class IndependentServoModel:
    def __init__(self, parameters: ServoValidationParameters) -> None:
        self.parameters = parameters
        self.left = ServoAxisState()
        self.right = ServoAxisState()

    def _step_axis(self, state: ServoAxisState, command_rad: float, zero_error_rad: float, dt: float) -> None:
        p = self.parameters
        command = _clamp(float(command_rad) + zero_error_rad, -p.max_angle_rad, p.max_angle_rad)
        if command > state.play_center_rad + p.backlash_rad:
            state.play_center_rad = command - p.backlash_rad
        elif command < state.play_center_rad - p.backlash_rad:
            state.play_center_rad = command + p.backlash_rad
        error = state.play_center_rad - state.angle_rad
        if abs(error) < p.command_deadband_rad:
            error = 0.0
        acceleration = p.natural_frequency_rad_s ** 2 * error - (
            2.0 * p.damping_ratio * p.natural_frequency_rad_s * state.rate_rad_s
        )
        acceleration = _clamp(acceleration, -p.max_acceleration_rad_s2, p.max_acceleration_rad_s2)
        state.rate_rad_s = _clamp(
            state.rate_rad_s + acceleration * dt,
            -p.max_rate_rad_s,
            p.max_rate_rad_s,
        )
        next_angle = _clamp(state.angle_rad + state.rate_rad_s * dt, -p.max_angle_rad, p.max_angle_rad)
        if next_angle in (-p.max_angle_rad, p.max_angle_rad) and next_angle * state.rate_rad_s > 0.0:
            state.rate_rad_s = 0.0
        state.angle_rad = next_angle

    def step(self, left_command_rad: float, right_command_rad: float, dt_s: float) -> tuple[ServoAxisState, ServoAxisState]:
        dt = float(dt_s)
        _finite_positive("servo step dt", dt)
        self._step_axis(self.left, left_command_rad, self.parameters.left_zero_error_rad, dt)
        self._step_axis(self.right, right_command_rad, self.parameters.right_zero_error_rad, dt)
        return self.left, self.right
