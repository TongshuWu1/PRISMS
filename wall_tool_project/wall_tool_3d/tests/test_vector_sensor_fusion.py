from __future__ import annotations

import math
import random
import sys
import unittest
from pathlib import Path


WALL_TOOL_PROJECT_ROOT = Path(__file__).resolve().parents[2]
WALL_TOOL_2D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_2d"
WALL_TOOL_3D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_3d"
for path in (WALL_TOOL_3D_ROOT, WALL_TOOL_2D_ROOT, WALL_TOOL_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from coppeliasim_vector_tool.estimator import SensorFusionEstimator  # noqa: E402


class VectorSensorFusionTests(unittest.TestCase):
    def test_thirty_second_sensor_only_motion_and_gyro_bias_estimation(self) -> None:
        estimator = SensorFusionEstimator()
        rng = random.Random(9041)
        dt = 0.01
        velocity = (0.11, -0.035)
        angular_rate = 0.08
        gyro_bias = 0.018
        estimate = None
        for index in range(3001):
            time_s = index * dt
            true_position = (0.2 + velocity[0] * time_s, 2.3 + velocity[1] * time_s)
            true_attitude = -0.3 + angular_rate * time_s
            estimate = estimator.update(
                position_xz_m=(
                    true_position[0] + rng.gauss(0.0, 0.0015),
                    true_position[1] + rng.gauss(0.0, 0.0015),
                ),
                attitude_rad=true_attitude + rng.gauss(0.0, 0.0025),
                gyro_rate_rad_s=angular_rate + gyro_bias + rng.gauss(0.0, 0.010),
                dt_s=dt,
                position_std_m=0.0015,
                attitude_std_rad=0.0025,
                gyro_std_rad_s=0.010,
            )
        self.assertIsNotNone(estimate)
        assert estimate is not None
        self.assertAlmostEqual(estimate.velocity_xz_m_s[0], velocity[0], delta=0.012)
        self.assertAlmostEqual(estimate.velocity_xz_m_s[1], velocity[1], delta=0.012)
        self.assertAlmostEqual(estimate.angular_rate_rad_s, angular_rate, delta=0.015)
        self.assertAlmostEqual(estimate.gyro_bias_rad_s, gyro_bias, delta=0.015)
        self.assertLess(max(estimate.position_std_m), 0.002)
        self.assertTrue(all(math.isfinite(value) for value in (
            *estimate.position_xz_m,
            *estimate.velocity_xz_m_s,
            estimate.attitude_rad,
            estimate.angular_rate_rad_s,
            estimate.gyro_bias_rad_s,
        )))

    def test_attitude_innovation_wraps_across_pi(self) -> None:
        estimator = SensorFusionEstimator()
        dt = 0.01
        estimate = None
        for index in range(250):
            angle = math.pi - 0.05 + 0.001 * index
            wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
            estimate = estimator.update(
                position_xz_m=(0.0, 2.0),
                attitude_rad=wrapped,
                gyro_rate_rad_s=0.1,
                dt_s=dt,
                position_std_m=0.001,
                attitude_std_rad=0.002,
                gyro_std_rad_s=0.01,
            )
        assert estimate is not None
        expected = (angle + math.pi) % (2.0 * math.pi) - math.pi
        error = (estimate.attitude_rad - expected + math.pi) % (2.0 * math.pi) - math.pi
        self.assertLess(abs(error), 0.015)


if __name__ == "__main__":
    unittest.main()
