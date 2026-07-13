"""Hardware-facing 100 Hz sensor model for the vector-thrust plant."""

from __future__ import annotations

import math
import random

from wall_tool_sim.steel_cable import SteelCableSpec
from wall_tool_sim.wall_tool_ui import SimParams, clamp, wrap_angle

from .contracts import PlantTruth, SensorEstimate
from .estimator import SensorFusionEstimator


class VectorToolSensorSuite:
    """Sample, quantize, filter, and reconstruct state without Cartesian truth."""

    def __init__(self, params: SimParams) -> None:
        self.params = params
        self._cable = SteelCableSpec(
            diameter_m=params.steel_cable_diameter_m,
            youngs_modulus_pa=params.steel_cable_youngs_modulus_pa,
            density_kg_m3=params.steel_cable_density_kg_m3,
            structural_compliance_m_N=params.steel_cable_structural_compliance_m_N,
            damping_ratio=params.steel_cable_damping_ratio,
            payload_weight_fraction=params.steel_cable_payload_weight_fraction,
        )
        self._rng = random.Random(params.sensor_random_seed)
        self._next_sample_s = -math.inf
        self._last_sample_s: float | None = None
        self._last_theta = 0.0
        self._load_cell_N = 0.0
        self._last_estimate: SensorEstimate | None = None
        self._fusion = SensorFusionEstimator()

    @staticmethod
    def _quantize(value: float, resolution: float) -> float:
        if not math.isfinite(value) or not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("sensor value and resolution must be finite and positive")
        return round(value / resolution) * resolution

    @property
    def last_estimate(self) -> SensorEstimate:
        if self._last_estimate is None:
            raise RuntimeError("sensor suite has not received its first truth sample")
        return self._last_estimate

    def update(self, truth: PlantTruth, *, force: bool = False) -> SensorEstimate:
        now = float(truth.timestamp_s)
        if self._last_sample_s is not None and now + 1e-12 < self._last_sample_s:
            raise RuntimeError("CoppeliaSim sensor timestamp moved backwards")
        if not force and self._last_estimate is not None and now + 1e-12 < self._next_sample_s:
            return self._last_estimate

        sample_period = self.params.sensor_sample_period_s
        sample_dt = sample_period if self._last_sample_s is None else max(1e-9, now - self._last_sample_s)
        self._last_sample_s = now
        self._next_sample_s = now + sample_period

        anchor_x, _, anchor_z = truth.anchor_world_m
        mount_x, _, mount_z = truth.cable_mount_world_m
        true_theta = math.atan2(mount_x - anchor_x, anchor_z - mount_z)
        theta_resolution = 2.0 * math.pi / self.params.cable_angle_encoder_counts_per_rev
        measured_theta = self._quantize(
            wrap_angle(true_theta + self._rng.gauss(0.0, self.params.cable_angle_noise_std_rad)),
            theta_resolution,
        )

        reel_resolution = (
            2.0 * math.pi * self.params.reel_spool_radius_m
            / self.params.reel_encoder_counts_per_rev
        )
        measured_reel_length = self._quantize(
            truth.measured_reel_length_m + self._rng.gauss(0.0, self.params.reel_length_noise_std_m),
            reel_resolution,
        )
        measured_reel_length = clamp(
            measured_reel_length,
            self.params.min_cable_length,
            self.params.max_cable_length,
        )
        measured_reel_velocity = self._quantize(
            truth.measured_reel_velocity_m_s,
            reel_resolution / sample_period,
        )

        noisy_tension = (
            truth.measured_load_cell_tension_N
            + self._rng.gauss(0.0, self.params.load_cell_noise_std_N)
        )
        load_alpha = clamp(
            sample_dt / max(self.params.load_cell_filter_tau_s + sample_dt, 1e-9),
            0.0,
            1.0,
        )
        if self._last_estimate is None:
            self._load_cell_N = noisy_tension
        else:
            self._load_cell_N += load_alpha * (noisy_tension - self._load_cell_N)
        measured_tension = clamp(self._load_cell_N, 0.0, self.params.max_spool_tension)

        true_attitude = wrap_angle(-truth.orientation_world_rad[1])
        measured_attitude = self._quantize(
            true_attitude + self._rng.gauss(0.0, self.params.imu_angle_noise_std_rad),
            2.0 * math.pi / 65536.0,
        )
        measured_angular_rate = (
            -truth.angular_velocity_world_rad_s[1]
            + self._rng.gauss(0.0, self.params.imu_rate_noise_std_rad_s)
        )

        stiffness = self._cable.axial_stiffness_N_m(max(measured_reel_length, self.params.min_cable_length))
        estimated_stretch = measured_tension / max(stiffness, 1e-9)
        geometric_length = measured_reel_length + estimated_stretch
        cable_out = (math.sin(measured_theta), -math.cos(measured_theta))
        mount_estimate = (
            anchor_x + geometric_length * cable_out[0],
            anchor_z + geometric_length * cable_out[1],
        )
        mount_offset = (
            -self.params.payload_hex_radius * math.sin(measured_attitude),
            self.params.payload_hex_radius * math.cos(measured_attitude),
        )
        position = (
            mount_estimate[0] - mount_offset[0],
            mount_estimate[1] - mount_offset[1],
        )
        position_std = math.sqrt(
            self.params.reel_length_noise_std_m ** 2
            + (self.params.load_cell_noise_std_N / max(stiffness, 1e-9)) ** 2
            + (geometric_length * self.params.cable_angle_noise_std_rad) ** 2
            + (self.params.payload_hex_radius * self.params.imu_angle_noise_std_rad) ** 2
        )
        fusion = self._fusion.update(
            position_xz_m=position,
            attitude_rad=measured_attitude,
            gyro_rate_rad_s=measured_angular_rate,
            dt_s=sample_dt,
            position_std_m=max(position_std, 1e-6),
            attitude_std_rad=max(self.params.imu_angle_noise_std_rad, 1e-6),
            gyro_std_rad_s=max(self.params.imu_rate_noise_std_rad_s, 1e-6),
        )
        theta_rate = (
            0.0
            if self._last_estimate is None
            else wrap_angle(measured_theta - self._last_theta) / sample_dt
        )
        self._last_theta = measured_theta

        self._last_estimate = SensorEstimate(
            timestamp_s=now,
            payload_position_xz_m=fusion.position_xz_m,
            payload_velocity_xz_m_s=fusion.velocity_xz_m_s,
            payload_attitude_rad=fusion.attitude_rad,
            payload_angular_rate_rad_s=fusion.angular_rate_rad_s,
            cable_angle_rad=measured_theta,
            cable_angle_rate_rad_s=theta_rate,
            geometric_cable_length_m=geometric_length,
            reel_length_m=measured_reel_length,
            reel_velocity_m_s=measured_reel_velocity,
            cable_tension_N=measured_tension,
        )
        return self._last_estimate
