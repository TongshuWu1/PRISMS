"""Sensor-only error-state Kalman estimator for the vector-thrust payload."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from wall_tool_sim.wall_tool_ui import wrap_angle


@dataclass(frozen=True)
class FusionEstimate:
    position_xz_m: tuple[float, float]
    velocity_xz_m_s: tuple[float, float]
    attitude_rad: float
    angular_rate_rad_s: float
    gyro_bias_rad_s: float
    position_std_m: tuple[float, float]
    velocity_std_m_s: tuple[float, float]


class SensorFusionEstimator:
    """Seven-state linearized estimator with explicit covariance propagation.

    State is ``[x, z, vx, vz, pitch, pitch_rate, gyro_bias]``.  The position
    observation is reconstructed upstream solely from cable angle, reel
    encoder, load cell, and IMU attitude.  No CoppeliaSim Cartesian state is an
    input to this estimator.
    """

    def __init__(
        self,
        *,
        translational_acceleration_std_m_s2: float = 0.65,
        angular_acceleration_std_rad_s2: float = 1.4,
        gyro_bias_walk_std_rad_s2: float = 0.002,
    ) -> None:
        for name, value in (
            ("translational acceleration noise", translational_acceleration_std_m_s2),
            ("angular acceleration noise", angular_acceleration_std_rad_s2),
            ("gyro bias walk", gyro_bias_walk_std_rad_s2),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self.translational_acceleration_std_m_s2 = float(translational_acceleration_std_m_s2)
        self.angular_acceleration_std_rad_s2 = float(angular_acceleration_std_rad_s2)
        self.gyro_bias_walk_std_rad_s2 = float(gyro_bias_walk_std_rad_s2)
        self._state = np.zeros(7, dtype=float)
        self._covariance = np.eye(7, dtype=float)
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    def initialize(
        self,
        position_xz_m: tuple[float, float],
        attitude_rad: float,
        gyro_rate_rad_s: float,
        position_std_m: float,
        attitude_std_rad: float,
        gyro_std_rad_s: float,
    ) -> FusionEstimate:
        values = (*position_xz_m, attitude_rad, gyro_rate_rad_s, position_std_m, attitude_std_rad, gyro_std_rad_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("fusion initialization values must be finite")
        if min(position_std_m, attitude_std_rad, gyro_std_rad_s) <= 0.0:
            raise ValueError("fusion measurement standard deviations must be positive")
        self._state[:] = (
            position_xz_m[0], position_xz_m[1], 0.0, 0.0,
            wrap_angle(attitude_rad), gyro_rate_rad_s, 0.0,
        )
        self._covariance = np.diag([
            position_std_m ** 2,
            position_std_m ** 2,
            0.08 ** 2,
            0.08 ** 2,
            attitude_std_rad ** 2,
            gyro_std_rad_s ** 2,
            (2.0 * gyro_std_rad_s) ** 2,
        ])
        self._initialized = True
        return self._result()

    def update(
        self,
        *,
        position_xz_m: tuple[float, float],
        attitude_rad: float,
        gyro_rate_rad_s: float,
        dt_s: float,
        position_std_m: float,
        attitude_std_rad: float,
        gyro_std_rad_s: float,
    ) -> FusionEstimate:
        dt = float(dt_s)
        values = (
            *position_xz_m, attitude_rad, gyro_rate_rad_s, dt,
            position_std_m, attitude_std_rad, gyro_std_rad_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("fusion update values must be finite")
        if dt <= 0.0 or min(position_std_m, attitude_std_rad, gyro_std_rad_s) <= 0.0:
            raise ValueError("fusion dt and measurement standard deviations must be positive")
        if not self._initialized:
            return self.initialize(
                position_xz_m,
                attitude_rad,
                gyro_rate_rad_s,
                position_std_m,
                attitude_std_rad,
                gyro_std_rad_s,
            )

        transition = np.eye(7, dtype=float)
        transition[0, 2] = dt
        transition[1, 3] = dt
        transition[4, 5] = dt
        self._state = transition @ self._state
        self._state[4] = wrap_angle(float(self._state[4]))

        q_translation = self.translational_acceleration_std_m_s2 ** 2
        q_rotation = self.angular_acceleration_std_rad_s2 ** 2
        process_covariance = np.zeros((7, 7), dtype=float)
        for position_index, velocity_index in ((0, 2), (1, 3)):
            process_covariance[position_index, position_index] = 0.25 * dt ** 4 * q_translation
            process_covariance[position_index, velocity_index] = 0.5 * dt ** 3 * q_translation
            process_covariance[velocity_index, position_index] = 0.5 * dt ** 3 * q_translation
            process_covariance[velocity_index, velocity_index] = dt ** 2 * q_translation
        process_covariance[4, 4] = 0.25 * dt ** 4 * q_rotation
        process_covariance[4, 5] = 0.5 * dt ** 3 * q_rotation
        process_covariance[5, 4] = 0.5 * dt ** 3 * q_rotation
        process_covariance[5, 5] = dt ** 2 * q_rotation
        process_covariance[6, 6] = dt * self.gyro_bias_walk_std_rad_s2 ** 2
        self._covariance = transition @ self._covariance @ transition.T + process_covariance

        observation = np.array([
            position_xz_m[0], position_xz_m[1], wrap_angle(attitude_rad), gyro_rate_rad_s,
        ], dtype=float)
        measurement_matrix = np.zeros((4, 7), dtype=float)
        measurement_matrix[0, 0] = 1.0
        measurement_matrix[1, 1] = 1.0
        measurement_matrix[2, 4] = 1.0
        measurement_matrix[3, 5] = 1.0
        measurement_matrix[3, 6] = 1.0
        measurement_covariance = np.diag([
            position_std_m ** 2,
            position_std_m ** 2,
            attitude_std_rad ** 2,
            gyro_std_rad_s ** 2,
        ])
        innovation = observation - measurement_matrix @ self._state
        innovation[2] = wrap_angle(float(innovation[2]))
        innovation_covariance = (
            measurement_matrix @ self._covariance @ measurement_matrix.T
            + measurement_covariance
        )
        try:
            gain = np.linalg.solve(
                innovation_covariance,
                measurement_matrix @ self._covariance,
            ).T
        except np.linalg.LinAlgError as exc:
            raise RuntimeError("sensor-fusion innovation covariance is singular") from exc
        self._state += gain @ innovation
        self._state[4] = wrap_angle(float(self._state[4]))
        identity = np.eye(7, dtype=float)
        residual_projection = identity - gain @ measurement_matrix
        # Joseph form retains symmetry/positive semi-definiteness over long runs.
        self._covariance = (
            residual_projection @ self._covariance @ residual_projection.T
            + gain @ measurement_covariance @ gain.T
        )
        self._covariance = 0.5 * (self._covariance + self._covariance.T)
        if not np.all(np.isfinite(self._state)) or not np.all(np.isfinite(self._covariance)):
            raise RuntimeError("sensor-fusion state or covariance became non-finite")
        if np.min(np.linalg.eigvalsh(self._covariance)) < -1e-10:
            raise RuntimeError("sensor-fusion covariance lost positive semi-definiteness")
        return self._result()

    def _result(self) -> FusionEstimate:
        diagonal = np.maximum(np.diag(self._covariance), 0.0)
        return FusionEstimate(
            position_xz_m=(float(self._state[0]), float(self._state[1])),
            velocity_xz_m_s=(float(self._state[2]), float(self._state[3])),
            attitude_rad=float(self._state[4]),
            angular_rate_rad_s=float(self._state[5]),
            gyro_bias_rad_s=float(self._state[6]),
            position_std_m=(math.sqrt(diagonal[0]), math.sqrt(diagonal[1])),
            velocity_std_m_s=(math.sqrt(diagonal[2]), math.sqrt(diagonal[3])),
        )
