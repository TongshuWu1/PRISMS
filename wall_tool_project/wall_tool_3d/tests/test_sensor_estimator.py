from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


WALL_TOOL_PROJECT_ROOT = Path(__file__).resolve().parents[2]
WALL_TOOL_2D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_2d"
WALL_TOOL_3D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_3d"
for path in (WALL_TOOL_3D_ROOT, WALL_TOOL_2D_ROOT, WALL_TOOL_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from coppeliasim_wall_tool.sensor_estimator import (  # noqa: E402
    SensorConfig,
    SensorTruth,
    WallToolSensorPipeline,
    rotate2,
)
from wall_tool_sim.steel_cable import SteelCableSpec  # noqa: E402


class WallToolSensorEstimatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.anchor = (0.0, 6.0)
        self.mount_radius = 0.114
        self.gravity = 9.80665
        self.cable = SteelCableSpec()
        self.config = SensorConfig(
            reel_encoder_counts_per_output_rev=1_000_000_000,
            cable_angle_encoder_counts_per_rev=1_000_000_000,
            reel_encoder_noise_counts_std=0.0,
            cable_angle_noise_rad_std=0.0,
            load_cell_noise_N_std=0.0,
            imu_gyro_noise_rad_s_std=0.0,
            imu_accel_noise_m_s2_std=0.0,
            imu_gyro_bias_rad_s=0.0,
            imu_accel_bias_body_x_m_s2=0.0,
            imu_accel_bias_body_z_m_s2=0.0,
            gyro_bias_learning_rate=0.0,
            load_cell_filter_tau_s=0.0,
            position_filter_tau_s=0.0,
            velocity_fusion_tau_s=0.0,
            angle_rate_filter_tau_s=0.0,
            reel_rate_filter_tau_s=0.0,
            attitude_filter_tau_s=0.0,
            gyro_filter_tau_s=0.0,
        )

    def make_pipeline(self) -> WallToolSensorPipeline:
        return WallToolSensorPipeline(
            self.config,
            self.anchor,
            self.mount_radius,
            self.gravity,
            self.cable,
        )

    def truth_for_payload(
        self,
        timestamp_s: float,
        payload: tuple[float, float],
        velocity: tuple[float, float] = (0.0, 0.0),
        attitude: float = 0.0,
        angular_rate: float = 0.0,
        reel_length: float | None = None,
        tension: float = 0.0,
    ) -> SensorTruth:
        mount_offset = rotate2((0.0, self.mount_radius), attitude)
        mount = (payload[0] + mount_offset[0], payload[1] + mount_offset[1])
        distance = math.hypot(mount[0] - self.anchor[0], mount[1] - self.anchor[1])
        return SensorTruth(
            timestamp_s=timestamp_s,
            reel_length_m=distance if reel_length is None else reel_length,
            cable_tension_N=tension,
            anchor_xz_m=self.anchor,
            cable_mount_xz_m=mount,
            payload_attitude_rad=attitude,
            payload_angular_rate_rad_s=angular_rate,
            payload_velocity_xz_m_s=velocity,
        )

    def test_reconstructs_payload_from_reel_angle_and_imu(self) -> None:
        pipeline = self.make_pipeline()
        payload = (0.73, 2.18)
        estimate = pipeline.update(self.truth_for_payload(0.0, payload, attitude=0.21))
        self.assertAlmostEqual(estimate.payload_position_xz_m[0], payload[0], places=7)
        self.assertAlmostEqual(estimate.payload_position_xz_m[1], payload[1], places=7)
        self.assertAlmostEqual(estimate.payload_attitude_rad, 0.21, places=9)

    def test_load_cell_corrects_elastic_cable_extension(self) -> None:
        pipeline = self.make_pipeline()
        payload = (-0.55, 2.65)
        nominal_truth = self.truth_for_payload(0.0, payload, attitude=-0.13)
        extension = 0.0012
        reel_length = nominal_truth.reel_length_m - extension
        tension = self.cable.axial_stiffness_N_m(reel_length) * extension
        stretched_truth = self.truth_for_payload(
            0.0,
            payload,
            attitude=-0.13,
            reel_length=reel_length,
            tension=tension,
        )
        estimate = pipeline.update(stretched_truth)
        self.assertAlmostEqual(estimate.estimated_geometric_cable_length_m, nominal_truth.reel_length_m, places=7)
        self.assertAlmostEqual(estimate.payload_position_xz_m[0], payload[0], places=7)
        self.assertAlmostEqual(estimate.payload_position_xz_m[1], payload[1], places=7)

    def test_geometry_derivative_recovers_payload_velocity(self) -> None:
        pipeline = self.make_pipeline()
        velocity = (0.30, -0.04)
        initial = (0.10, 2.40)
        pipeline.update(self.truth_for_payload(0.0, initial, velocity=velocity, attitude=0.08))
        dt = 0.01
        moved = (initial[0] + velocity[0] * dt, initial[1] + velocity[1] * dt)
        estimate = pipeline.update(self.truth_for_payload(dt, moved, velocity=velocity, attitude=0.08))
        self.assertAlmostEqual(estimate.payload_velocity_xz_m_s[0], velocity[0], delta=3e-6)
        self.assertAlmostEqual(estimate.payload_velocity_xz_m_s[1], velocity[1], delta=3e-6)

    def test_duplicate_timestamp_does_not_resample_noise(self) -> None:
        noisy_config = SensorConfig(random_seed=19)
        pipeline = WallToolSensorPipeline(
            noisy_config,
            self.anchor,
            self.mount_radius,
            self.gravity,
            self.cable,
        )
        truth = self.truth_for_payload(0.0, (0.2, 2.0))
        first = pipeline.update(truth)
        first_sample = pipeline.last_sample
        second = pipeline.update(truth)
        self.assertIs(first, second)
        self.assertIs(first_sample, pipeline.last_sample)

    def test_default_noisy_sensors_track_smooth_motion(self) -> None:
        pipeline = WallToolSensorPipeline(
            SensorConfig(random_seed=7),
            self.anchor,
            self.mount_radius,
            self.gravity,
            self.cable,
        )
        position_error_squared: list[float] = []
        velocity_error_squared: list[float] = []
        for index in range(1001):
            t = 0.01 * index
            payload = (0.4 * math.sin(0.5 * t), 2.4 + 0.25 * math.sin(0.3 * t))
            velocity = (0.2 * math.cos(0.5 * t), 0.075 * math.cos(0.3 * t))
            attitude = 0.1 * math.sin(0.4 * t)
            angular_rate = 0.04 * math.cos(0.4 * t)
            nominal = self.truth_for_payload(t, payload, velocity, attitude, angular_rate)
            tension = 0.6 + 0.15 * math.sin(0.7 * t)
            reel_length = nominal.reel_length_m - tension / self.cable.axial_stiffness_N_m(nominal.reel_length_m)
            estimate = pipeline.update(
                self.truth_for_payload(
                    t,
                    payload,
                    velocity,
                    attitude,
                    angular_rate,
                    reel_length,
                    tension,
                )
            )
            if t < 1.0:
                continue
            position_error_squared.append(
                (estimate.payload_position_xz_m[0] - payload[0]) ** 2
                + (estimate.payload_position_xz_m[1] - payload[1]) ** 2
            )
            velocity_error_squared.append(
                (estimate.payload_velocity_xz_m_s[0] - velocity[0]) ** 2
                + (estimate.payload_velocity_xz_m_s[1] - velocity[1]) ** 2
            )
        position_rmse = math.sqrt(sum(position_error_squared) / len(position_error_squared))
        velocity_rmse = math.sqrt(sum(velocity_error_squared) / len(velocity_error_squared))
        self.assertLess(position_rmse, 0.006)
        self.assertLess(velocity_rmse, 0.080)

    def test_raw_imu_complementary_filter_rejects_gyro_bias(self) -> None:
        config = SensorConfig(
            reel_encoder_noise_counts_std=0.0,
            cable_angle_noise_rad_std=0.0,
            load_cell_noise_N_std=0.0,
            imu_gyro_noise_rad_s_std=0.0,
            imu_accel_noise_m_s2_std=0.0,
            imu_gyro_bias_rad_s=0.010,
            imu_accel_bias_body_x_m_s2=0.0,
            imu_accel_bias_body_z_m_s2=0.0,
            gyro_bias_learning_rate=0.40,
            attitude_filter_tau_s=0.20,
        )
        pipeline = WallToolSensorPipeline(
            config,
            self.anchor,
            self.mount_radius,
            self.gravity,
            self.cable,
        )
        attitude = 0.20
        estimate = None
        for index in range(1001):
            estimate = pipeline.update(
                self.truth_for_payload(
                    0.01 * index,
                    (0.4, 2.3),
                    attitude=attitude,
                    angular_rate=0.0,
                )
            )
        assert estimate is not None
        self.assertAlmostEqual(estimate.payload_attitude_rad, attitude, delta=0.002)
        self.assertGreater(estimate.estimated_gyro_bias_rad_s, 0.004)
        self.assertLess(abs(estimate.payload_angular_rate_rad_s), 0.006)


if __name__ == "__main__":
    unittest.main()
