"""Hardware-like sensors and state estimation for the CoppeliaSim wall tool.

The real wall-tool sensing concept is intentionally small:

* an encoder on the reel motor measures paid-out cable length and speed,
* an absolute encoder at the top guide measures the cable angle,
* an inline load cell measures cable tension, and
* a payload IMU measures pitch, pitch rate, and specific force.

This module keeps the simulated sensor layer separate from the estimator so the
same estimator contract can later be connected to real device drivers.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from wall_tool_sim.steel_cable import SteelCableSpec


Vec2 = tuple[float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def angle_delta(current: float, previous: float) -> float:
    return wrap_angle(float(current) - float(previous))


def rotate2(vector: Vec2, angle: float) -> Vec2:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]


def filter_alpha(dt: float, tau: float) -> float:
    if tau <= 0.0:
        return 1.0
    return clamp(float(dt) / (float(tau) + float(dt)), 0.0, 1.0)


@dataclass(frozen=True)
class SensorConfig:
    """Resolution, noise, and filter assumptions for the proposed sensors."""

    reel_encoder_counts_per_output_rev: int = 2048
    cable_angle_encoder_counts_per_rev: int = 4096
    reel_spool_radius_m: float = 0.022
    reel_encoder_noise_counts_std: float = 0.20
    cable_angle_noise_rad_std: float = math.radians(0.05)
    load_cell_noise_N_std: float = 0.010
    imu_gyro_noise_rad_s_std: float = math.radians(0.20)
    imu_accel_noise_m_s2_std: float = 0.030
    imu_gyro_bias_rad_s: float = math.radians(0.35)
    imu_accel_bias_body_x_m_s2: float = 0.020
    imu_accel_bias_body_z_m_s2: float = -0.015
    gyro_bias_learning_rate: float = 0.040
    accelerometer_trust_band_m_s2: float = 1.50
    load_cell_filter_tau_s: float = 0.018
    position_filter_tau_s: float = 0.018
    velocity_fusion_tau_s: float = 0.080
    angle_rate_filter_tau_s: float = 0.040
    reel_rate_filter_tau_s: float = 0.040
    attitude_filter_tau_s: float = 0.020
    gyro_filter_tau_s: float = 0.020
    random_seed: int = 7

    def __post_init__(self) -> None:
        if self.reel_encoder_counts_per_output_rev < 1:
            raise ValueError("reel encoder counts per output revolution must be positive")
        if self.cable_angle_encoder_counts_per_rev < 1:
            raise ValueError("cable-angle encoder counts per revolution must be positive")
        if self.reel_spool_radius_m <= 0.0:
            raise ValueError("reel spool radius must be positive")
        for name in (
            "reel_encoder_noise_counts_std",
            "cable_angle_noise_rad_std",
            "load_cell_noise_N_std",
            "imu_gyro_noise_rad_s_std",
            "imu_accel_noise_m_s2_std",
            "gyro_bias_learning_rate",
            "accelerometer_trust_band_m_s2",
            "load_cell_filter_tau_s",
            "position_filter_tau_s",
            "velocity_fusion_tau_s",
            "angle_rate_filter_tau_s",
            "reel_rate_filter_tau_s",
            "attitude_filter_tau_s",
            "gyro_filter_tau_s",
        ):
            if float(getattr(self, name)) < 0.0:
                raise ValueError(f"{name} cannot be negative")

    @property
    def reel_line_m_per_count(self) -> float:
        return 2.0 * math.pi * self.reel_spool_radius_m / self.reel_encoder_counts_per_output_rev

    @property
    def cable_angle_rad_per_count(self) -> float:
        return 2.0 * math.pi / self.cable_angle_encoder_counts_per_rev


@dataclass(frozen=True)
class SensorTruth:
    """Ground truth used only by the CoppeliaSim sensor simulator."""

    timestamp_s: float
    reel_length_m: float
    cable_tension_N: float
    anchor_xz_m: Vec2
    cable_mount_xz_m: Vec2
    payload_attitude_rad: float
    payload_angular_rate_rad_s: float
    payload_velocity_xz_m_s: Vec2


@dataclass(frozen=True)
class WallToolSensorSample:
    """Measurements available to the estimator and, later, real drivers."""

    timestamp_s: float
    reel_encoder_count: int
    reel_length_m: float
    cable_angle_encoder_count: int
    cable_angle_rad: float
    cable_tension_N: float
    imu_angular_rate_rad_s: float
    imu_specific_force_body_xz_m_s2: Vec2


@dataclass(frozen=True)
class EstimatedWallToolState:
    timestamp_s: float
    payload_position_xz_m: Vec2
    payload_velocity_xz_m_s: Vec2
    payload_attitude_rad: float
    payload_angular_rate_rad_s: float
    reel_length_m: float
    reel_velocity_m_s: float
    cable_angle_rad: float
    cable_angle_rate_rad_s: float
    cable_tension_N: float
    estimated_geometric_cable_length_m: float
    estimated_gyro_bias_rad_s: float


class WallToolSensorSuite:
    """Generate encoder, load-cell, and IMU readings from plant truth."""

    def __init__(self, config: SensorConfig, gravity_m_s2: float) -> None:
        self.config = config
        self.gravity = float(gravity_m_s2)
        self._random = random.Random(int(config.random_seed))
        self._initial_reel_length_m: float | None = None
        self._previous_timestamp_s: float | None = None
        self._previous_velocity_xz_m_s: Vec2 | None = None
        self._last_sample: WallToolSensorSample | None = None

    def sample(self, truth: SensorTruth) -> WallToolSensorSample:
        if self._last_sample is not None and truth.timestamp_s <= self._last_sample.timestamp_s + 1e-12:
            return self._last_sample
        if self._initial_reel_length_m is None:
            self._initial_reel_length_m = float(truth.reel_length_m)

        line_per_count = self.config.reel_line_m_per_count
        delta_length = float(truth.reel_length_m) - self._initial_reel_length_m
        noisy_reel_count = delta_length / line_per_count + self._noise(self.config.reel_encoder_noise_counts_std)
        reel_count = int(round(noisy_reel_count))
        measured_reel_length = self._initial_reel_length_m + reel_count * line_per_count

        dx = float(truth.cable_mount_xz_m[0]) - float(truth.anchor_xz_m[0])
        dz_down = float(truth.anchor_xz_m[1]) - float(truth.cable_mount_xz_m[1])
        cable_angle = wrap_angle(math.atan2(dx, dz_down) + self._noise(self.config.cable_angle_noise_rad_std))
        angle_per_count = self.config.cable_angle_rad_per_count
        angle_count = int(round(cable_angle / angle_per_count))
        measured_cable_angle = wrap_angle(angle_count * angle_per_count)

        if self._previous_timestamp_s is None or self._previous_velocity_xz_m_s is None:
            acceleration_world = (0.0, 0.0)
        else:
            dt = max(1e-6, float(truth.timestamp_s) - self._previous_timestamp_s)
            acceleration_world = (
                (float(truth.payload_velocity_xz_m_s[0]) - self._previous_velocity_xz_m_s[0]) / dt,
                (float(truth.payload_velocity_xz_m_s[1]) - self._previous_velocity_xz_m_s[1]) / dt,
            )
        # An accelerometer measures specific force f = a - g.  In the x-z
        # plane, gravity is [0, -g], hence f_world = [a_x, a_z + g].
        specific_force_world = (acceleration_world[0], acceleration_world[1] + self.gravity)
        specific_force_body = rotate2(specific_force_world, -float(truth.payload_attitude_rad))

        sample = WallToolSensorSample(
            timestamp_s=float(truth.timestamp_s),
            reel_encoder_count=reel_count,
            reel_length_m=measured_reel_length,
            cable_angle_encoder_count=angle_count,
            cable_angle_rad=measured_cable_angle,
            cable_tension_N=max(0.0, float(truth.cable_tension_N) + self._noise(self.config.load_cell_noise_N_std)),
            imu_angular_rate_rad_s=(
                float(truth.payload_angular_rate_rad_s)
                + self.config.imu_gyro_bias_rad_s
                + self._noise(self.config.imu_gyro_noise_rad_s_std)
            ),
            imu_specific_force_body_xz_m_s2=(
                specific_force_body[0]
                + self.config.imu_accel_bias_body_x_m_s2
                + self._noise(self.config.imu_accel_noise_m_s2_std),
                specific_force_body[1]
                + self.config.imu_accel_bias_body_z_m_s2
                + self._noise(self.config.imu_accel_noise_m_s2_std),
            ),
        )
        self._previous_timestamp_s = float(truth.timestamp_s)
        self._previous_velocity_xz_m_s = (
            float(truth.payload_velocity_xz_m_s[0]),
            float(truth.payload_velocity_xz_m_s[1]),
        )
        self._last_sample = sample
        return sample

    def _noise(self, standard_deviation: float) -> float:
        if standard_deviation <= 0.0:
            return 0.0
        return self._random.gauss(0.0, float(standard_deviation))


class WallToolStateEstimator:
    """Estimate the NMPC state from the proposed physical sensor suite."""

    def __init__(
        self,
        config: SensorConfig,
        anchor_xz_m: Vec2,
        cable_mount_radius_m: float,
        gravity_m_s2: float,
        steel_cable: SteelCableSpec,
    ) -> None:
        self.config = config
        self.anchor = (float(anchor_xz_m[0]), float(anchor_xz_m[1]))
        self.cable_mount_radius = float(cable_mount_radius_m)
        self.gravity = float(gravity_m_s2)
        self.steel_cable = steel_cable
        self._last_sample: WallToolSensorSample | None = None
        self._last_state: EstimatedWallToolState | None = None
        self._kinematic_position: Vec2 | None = None

    def update(self, sample: WallToolSensorSample) -> EstimatedWallToolState:
        if self._last_state is not None and sample.timestamp_s <= self._last_state.timestamp_s + 1e-12:
            return self._last_state

        if self._last_sample is None or self._last_state is None:
            attitude = math.atan2(
                sample.imu_specific_force_body_xz_m_s2[0],
                sample.imu_specific_force_body_xz_m_s2[1],
            )
            gyro_bias = 0.0
            angular_rate = float(sample.imu_angular_rate_rad_s) - gyro_bias
            tension = max(0.0, float(sample.cable_tension_N))
            geometric_length = self._geometric_cable_length(sample.reel_length_m, tension)
            position = self._payload_position(geometric_length, sample.cable_angle_rad, attitude)
            state = EstimatedWallToolState(
                timestamp_s=float(sample.timestamp_s),
                payload_position_xz_m=position,
                payload_velocity_xz_m_s=(0.0, 0.0),
                payload_attitude_rad=attitude,
                payload_angular_rate_rad_s=angular_rate,
                reel_length_m=float(sample.reel_length_m),
                reel_velocity_m_s=0.0,
                cable_angle_rad=float(sample.cable_angle_rad),
                cable_angle_rate_rad_s=0.0,
                cable_tension_N=tension,
                estimated_geometric_cable_length_m=geometric_length,
                estimated_gyro_bias_rad_s=gyro_bias,
            )
            self._last_sample = sample
            self._last_state = state
            self._kinematic_position = position
            return state

        previous = self._last_state
        dt = max(1e-6, float(sample.timestamp_s) - previous.timestamp_s)

        gyro_rate = float(sample.imu_angular_rate_rad_s) - previous.estimated_gyro_bias_rad_s
        predicted_attitude = wrap_angle(previous.payload_attitude_rad + gyro_rate * dt)
        accel_x, accel_z = sample.imu_specific_force_body_xz_m_s2
        accelerometer_attitude = math.atan2(accel_x, accel_z)
        accelerometer_norm = math.hypot(accel_x, accel_z)
        accelerometer_trust = clamp(
            1.0
            - abs(accelerometer_norm - self.gravity)
            / max(self.config.accelerometer_trust_band_m_s2, 1e-9),
            0.0,
            1.0,
        )
        attitude_alpha = filter_alpha(dt, self.config.attitude_filter_tau_s) * accelerometer_trust
        attitude_residual = angle_delta(accelerometer_attitude, predicted_attitude)
        attitude = wrap_angle(predicted_attitude + attitude_alpha * attitude_residual)
        gyro_bias = previous.estimated_gyro_bias_rad_s - (
            self.config.gyro_bias_learning_rate * accelerometer_trust * attitude_residual * dt
        )
        gyro_alpha = filter_alpha(dt, self.config.gyro_filter_tau_s)
        angular_rate = previous.payload_angular_rate_rad_s + gyro_alpha * (
            float(sample.imu_angular_rate_rad_s) - gyro_bias - previous.payload_angular_rate_rad_s
        )
        tension_alpha = filter_alpha(dt, self.config.load_cell_filter_tau_s)
        tension = max(
            0.0,
            previous.cable_tension_N
            + tension_alpha * (float(sample.cable_tension_N) - previous.cable_tension_N),
        )

        reel_velocity_raw = (float(sample.reel_length_m) - previous.reel_length_m) / dt
        reel_rate_alpha = filter_alpha(dt, self.config.reel_rate_filter_tau_s)
        reel_velocity = previous.reel_velocity_m_s + reel_rate_alpha * (
            reel_velocity_raw - previous.reel_velocity_m_s
        )
        angle_rate_raw = angle_delta(sample.cable_angle_rad, previous.cable_angle_rad) / dt
        angle_rate_alpha = filter_alpha(dt, self.config.angle_rate_filter_tau_s)
        cable_angle_rate = previous.cable_angle_rate_rad_s + angle_rate_alpha * (
            angle_rate_raw - previous.cable_angle_rate_rad_s
        )

        geometric_length = self._geometric_cable_length(sample.reel_length_m, tension)
        kinematic_position = self._payload_position(geometric_length, sample.cable_angle_rad, attitude)
        position_alpha = filter_alpha(dt, self.config.position_filter_tau_s)
        position = (
            previous.payload_position_xz_m[0]
            + position_alpha * (kinematic_position[0] - previous.payload_position_xz_m[0]),
            previous.payload_position_xz_m[1]
            + position_alpha * (kinematic_position[1] - previous.payload_position_xz_m[1]),
        )

        prior_kinematic_position = self._kinematic_position or previous.payload_position_xz_m
        kinematic_velocity = (
            (kinematic_position[0] - prior_kinematic_position[0]) / dt,
            (kinematic_position[1] - prior_kinematic_position[1]) / dt,
        )
        specific_force_world = rotate2(sample.imu_specific_force_body_xz_m_s2, attitude)
        acceleration_world = (specific_force_world[0], specific_force_world[1] - self.gravity)
        predicted_velocity = (
            previous.payload_velocity_xz_m_s[0] + acceleration_world[0] * dt,
            previous.payload_velocity_xz_m_s[1] + acceleration_world[1] * dt,
        )
        velocity_observation_alpha = filter_alpha(dt, self.config.velocity_fusion_tau_s)
        velocity = (
            predicted_velocity[0]
            + velocity_observation_alpha * (kinematic_velocity[0] - predicted_velocity[0]),
            predicted_velocity[1]
            + velocity_observation_alpha * (kinematic_velocity[1] - predicted_velocity[1]),
        )

        state = EstimatedWallToolState(
            timestamp_s=float(sample.timestamp_s),
            payload_position_xz_m=position,
            payload_velocity_xz_m_s=velocity,
            payload_attitude_rad=attitude,
            payload_angular_rate_rad_s=angular_rate,
            reel_length_m=float(sample.reel_length_m),
            reel_velocity_m_s=reel_velocity,
            cable_angle_rad=float(sample.cable_angle_rad),
            cable_angle_rate_rad_s=cable_angle_rate,
            cable_tension_N=tension,
            estimated_geometric_cable_length_m=geometric_length,
            estimated_gyro_bias_rad_s=gyro_bias,
        )
        self._last_sample = sample
        self._last_state = state
        self._kinematic_position = kinematic_position
        return state

    def _geometric_cable_length(self, reel_length_m: float, tension_N: float) -> float:
        reel_length = max(1e-6, float(reel_length_m))
        stiffness = self.steel_cable.axial_stiffness_N_m(reel_length)
        elastic_extension = max(0.0, float(tension_N)) / max(stiffness, 1e-9)
        return reel_length + elastic_extension

    def _payload_position(self, cable_distance_m: float, cable_angle_rad: float, attitude_rad: float) -> Vec2:
        mount = (
            self.anchor[0] + float(cable_distance_m) * math.sin(float(cable_angle_rad)),
            self.anchor[1] - float(cable_distance_m) * math.cos(float(cable_angle_rad)),
        )
        mount_offset = rotate2((0.0, self.cable_mount_radius), float(attitude_rad))
        return mount[0] - mount_offset[0], mount[1] - mount_offset[1]


class WallToolSensorPipeline:
    """Convenience wrapper connecting simulated sensors to the estimator."""

    def __init__(
        self,
        config: SensorConfig,
        anchor_xz_m: Vec2,
        cable_mount_radius_m: float,
        gravity_m_s2: float,
        steel_cable: SteelCableSpec,
    ) -> None:
        self.sensors = WallToolSensorSuite(config, gravity_m_s2)
        self.estimator = WallToolStateEstimator(
            config,
            anchor_xz_m,
            cable_mount_radius_m,
            gravity_m_s2,
            steel_cable,
        )
        self.last_sample: WallToolSensorSample | None = None
        self.last_estimate: EstimatedWallToolState | None = None

    def update(self, truth: SensorTruth) -> EstimatedWallToolState:
        self.last_sample = self.sensors.sample(truth)
        self.last_estimate = self.estimator.update(self.last_sample)
        return self.last_estimate
