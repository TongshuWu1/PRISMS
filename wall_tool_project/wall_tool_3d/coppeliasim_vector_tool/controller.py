"""Standalone sensor-to-actuator controller used by the CoppeliaSim plant."""

from __future__ import annotations

import math
from typing import Sequence

from cable_hybrid_controller.mpc import MPCReferenceHorizon, MPCSolution, WallToolNMPC
from wall_tool_sim.gimbal_servo import GimbalServoSpec
from wall_tool_sim.wall_tool_ui import (
    ReferenceState,
    SimParams,
    WallToolSimulator,
    add2,
    clamp,
    clamp_wall_point_for_params,
    scale2,
    slew_toward,
)

from .contracts import ActuatorCommand, SensorEstimate, Vec2


class ExternalVectorThrustController:
    """Receding-horizon controller with no internal payload-physics step."""

    def __init__(self, params: SimParams) -> None:
        self.params = params
        # The template is used only to construct the shared trajectory and the
        # exact MPC parameter contract. Its internal plant is never stepped.
        template = WallToolSimulator(params)
        self.trajectory = template.trajectory
        self._mpc_config = template._nmpc_config()
        self._nmpc = WallToolNMPC(self._mpc_config)
        self._servo = GimbalServoSpec(
            max_angle_rad=params.gimbal_max_angle_rad,
            max_rate_rad_s=params.gimbal_max_rate_rad_s,
            max_acceleration_rad_s2=params.gimbal_max_acceleration_rad_s2,
            natural_frequency_rad_s=params.gimbal_natural_frequency_rad_s,
            damping_ratio=params.gimbal_damping_ratio,
            command_min_pulse_us=params.gimbal_command_min_pulse_us,
            command_max_pulse_us=params.gimbal_command_max_pulse_us,
            command_resolution_us=params.gimbal_command_resolution_us,
        )
        self._next_solve_s = -math.inf
        self._last_time_s: float | None = None
        self._last_solution: MPCSolution | None = None
        self._hold_integral: Vec2 = (0.0, 0.0)
        self._estimated_left_thrust = 0.0
        self._estimated_right_thrust = 0.0
        self._estimated_left_servo_angle = 0.0
        self._estimated_right_servo_angle = 0.0
        self._estimated_left_servo_rate = 0.0
        self._estimated_right_servo_rate = 0.0
        self._applied_left_thrust = 0.0
        self._applied_right_thrust = 0.0
        self._applied_reel_velocity = 0.0
        self._applied_left_servo_angle = 0.0
        self._applied_right_servo_angle = 0.0
        self._previous_command = (0.0, 0.0, 0.0, 0.0, 0.0)
        self._initialized = False

    def command_corner_smooth_path(
        self,
        start: Vec2,
        goals: Sequence[Vec2],
        corner_speed_m_s: float,
    ) -> None:
        if not goals:
            raise ValueError("external controller path requires at least one goal")
        clamped = [clamp_wall_point_for_params(goal, self.params) for goal in goals]
        self.trajectory.command_corner_smooth_path(
            clamp_wall_point_for_params(start, self.params),
            clamped,
            corner_speed_m_s,
        )

    def command_target(self, start: Vec2, target: Vec2) -> None:
        self.trajectory.command_smooth_path(
            clamp_wall_point_for_params(start, self.params),
            [clamp_wall_point_for_params(target, self.params)],
        )

    def _initialize_actuator_estimates(self, sensor: SensorEstimate) -> None:
        cable_weight = (
            self.params.steel_cable_payload_weight_fraction
            * self.params.steel_cable_density_kg_m3
            * math.pi
            * (0.5 * self.params.steel_cable_diameter_m) ** 2
            * sensor.geometric_cable_length_m
            * self.params.gravity
        )
        hover = clamp(
            0.5 * (
                self.params.total_mass * self.params.gravity
                + cable_weight
                - sensor.cable_tension_N
            ),
            0.0,
            self._mpc_config.max_thrust_per_drone,
        )
        self._estimated_left_thrust = hover
        self._estimated_right_thrust = hover
        self._applied_left_thrust = hover
        self._applied_right_thrust = hover
        self._applied_reel_velocity = sensor.reel_velocity_m_s
        self._previous_command = (hover, hover, sensor.reel_velocity_m_s, 0.0, 0.0)
        self._initialized = True

    def _reference_horizon(self, reference: ReferenceState) -> MPCReferenceHorizon:
        steps = self.params.mpc_horizon_steps
        horizon_dt = self.params.mpc_horizon_dt
        if not reference.active or not self.trajectory.segments:
            return MPCReferenceHorizon(
                positions=tuple(reference.position for _ in range(steps + 1)),
                velocities=tuple((0.0, 0.0) for _ in range(steps + 1)),
            )
        samples = self.trajectory._path_samples()
        progress0 = self.trajectory.geometric_progress_m
        slowdown_distance = self.trajectory._geometric_slowdown_distance(self.params.path_speed)
        positions: list[Vec2] = []
        velocities: list[Vec2] = []
        for index in range(steps + 1):
            progress = progress0 + self.params.path_speed * horizon_dt * index
            point, tangent, remaining = self.trajectory._sample_path_at_progress(samples, progress)
            speed_scale = clamp(remaining / max(slowdown_distance, 1e-9), 0.0, 1.0)
            speed = self.params.path_speed * speed_scale
            if remaining <= self.params.waypoint_tolerance:
                speed = 0.0
            positions.append(point)
            velocities.append(scale2(tangent, speed))
        positions[0] = reference.position
        velocities[0] = reference.velocity
        return MPCReferenceHorizon(tuple(positions), tuple(velocities))

    def _update_hold_integral(self, reference: ReferenceState, sensor: SensorEstimate, dt: float) -> None:
        if reference.active:
            self._hold_integral = (0.0, 0.0)
            return
        error = (
            reference.position[0] - sensor.payload_position_xz_m[0],
            reference.position[1] - sensor.payload_position_xz_m[1],
        )
        limit = self.params.mpc_hold_integral_limit_m_s
        self._hold_integral = (
            clamp(self._hold_integral[0] + error[0] * dt, -limit, limit),
            clamp(self._hold_integral[1] + error[1] * dt, -limit, limit),
        )

    def step(self, sensor: SensorEstimate) -> ActuatorCommand:
        now = sensor.timestamp_s
        dt = self.params.dt if self._last_time_s is None else now - self._last_time_s
        if not math.isfinite(dt) or dt <= 0.0:
            raise RuntimeError(f"controller requires a positive sensor timestamp increment, got {dt}")
        self._last_time_s = now
        if not self._initialized:
            self._initialize_actuator_estimates(sensor)

        reference = self.trajectory.geometric_reference(
            sensor.payload_position_xz_m,
            sensor.payload_velocity_xz_m_s,
            dt,
        )
        reference = ReferenceState(
            position=clamp_wall_point_for_params(reference.position, self.params),
            velocity=reference.velocity,
            acceleration=reference.acceleration,
            final_target=clamp_wall_point_for_params(reference.final_target, self.params),
            active_target=clamp_wall_point_for_params(reference.active_target, self.params),
            active=reference.active,
            waypoint_count=reference.waypoint_count,
        )
        self._update_hold_integral(reference, sensor, dt)

        measured_state = (
            sensor.payload_position_xz_m[0],
            sensor.payload_position_xz_m[1],
            sensor.payload_velocity_xz_m_s[0],
            sensor.payload_velocity_xz_m_s[1],
            sensor.payload_attitude_rad,
            sensor.payload_angular_rate_rad_s,
            clamp(sensor.reel_length_m, self.params.min_cable_length, self.params.max_cable_length),
            clamp(self._estimated_left_thrust, 0.0, self._mpc_config.max_thrust_per_drone),
            clamp(self._estimated_right_thrust, 0.0, self._mpc_config.max_thrust_per_drone),
            clamp(sensor.reel_velocity_m_s, -self.params.max_spool_speed, self.params.max_spool_speed),
            clamp(sensor.cable_tension_N, 0.0, self.params.max_spool_tension),
            clamp(self._estimated_left_servo_angle, -self.params.gimbal_max_angle_rad, self.params.gimbal_max_angle_rad),
            clamp(self._estimated_right_servo_angle, -self.params.gimbal_max_angle_rad, self.params.gimbal_max_angle_rad),
            clamp(self._estimated_left_servo_rate, -self.params.gimbal_max_rate_rad_s, self.params.gimbal_max_rate_rad_s),
            clamp(self._estimated_right_servo_rate, -self.params.gimbal_max_rate_rad_s, self.params.gimbal_max_rate_rad_s),
        )

        if self._last_solution is None or now + 1e-12 >= self._next_solve_s:
            horizon = self._reference_horizon(reference)
            if not reference.active:
                correction = scale2(self._hold_integral, self.params.mpc_hold_integral_gain_s_inv)
                horizon = MPCReferenceHorizon(
                    positions=tuple(
                        clamp_wall_point_for_params(add2(point, correction), self.params)
                        for point in horizon.positions
                    ),
                    velocities=horizon.velocities,
                )
            try:
                self._last_solution = self._nmpc.solve(
                    measured_state=measured_state,
                    reference=horizon,
                    previous_command=self._previous_command,
                )
            except RuntimeError as exc:
                labels = (
                    "x", "z", "vx", "vz", "pitch", "pitch_rate", "reel_length",
                    "left_thrust_est", "right_thrust_est", "reel_speed", "tension",
                    "left_servo_est", "right_servo_est", "left_servo_rate_est",
                    "right_servo_rate_est",
                )
                state_text = ", ".join(
                    f"{label}={value:+.6g}" for label, value in zip(labels, measured_state)
                )
                raise RuntimeError(
                    f"external NMPC failed at sensor t={now:.6f}s; "
                    f"measured_state[{state_text}]; previous_command={self._previous_command}; "
                    f"reference0={horizon.positions[0]}: {exc}"
                ) from exc
            self._next_solve_s = now + self.params.mpc_control_period_s
        solution = self._last_solution
        if solution is None or not solution.success:
            raise RuntimeError("NMPC produced no successful external-plant command")

        thrust_limit = self._mpc_config.max_thrust_per_drone
        left_target = clamp(solution.left_thrust, 0.0, thrust_limit)
        right_target = clamp(solution.right_thrust, 0.0, thrust_limit)
        reel_target = clamp(solution.spool_velocity, -self.params.max_spool_speed, self.params.max_spool_speed)
        left_servo_target = self._servo.realize_pwm_command(solution.left_gimbal_angle)
        right_servo_target = self._servo.realize_pwm_command(solution.right_gimbal_angle)

        self._applied_left_thrust = slew_toward(
            self._applied_left_thrust, left_target, self.params.thrust_command_slew_limit_N_s, dt
        )
        self._applied_right_thrust = slew_toward(
            self._applied_right_thrust, right_target, self.params.thrust_command_slew_limit_N_s, dt
        )
        self._applied_reel_velocity = slew_toward(
            self._applied_reel_velocity, reel_target, self.params.reel_velocity_slew_limit_mps2, dt
        )
        self._applied_left_servo_angle = self._servo.realize_pwm_command(slew_toward(
            self._applied_left_servo_angle,
            left_servo_target,
            self.params.gimbal_command_slew_limit_rad_s,
            dt,
        ))
        self._applied_right_servo_angle = self._servo.realize_pwm_command(slew_toward(
            self._applied_right_servo_angle,
            right_servo_target,
            self.params.gimbal_command_slew_limit_rad_s,
            dt,
        ))

        motor_alpha = clamp(dt / (self.params.motor_thrust_time_constant_s + dt), 0.0, 1.0)
        self._estimated_left_thrust += motor_alpha * (
            self._applied_left_thrust - self._estimated_left_thrust
        )
        self._estimated_right_thrust += motor_alpha * (
            self._applied_right_thrust - self._estimated_right_thrust
        )
        (
            self._estimated_left_servo_angle,
            self._estimated_left_servo_rate,
            _,
            _,
        ) = self._servo.step(
            self._estimated_left_servo_angle,
            self._estimated_left_servo_rate,
            self._applied_left_servo_angle,
            dt,
        )
        (
            self._estimated_right_servo_angle,
            self._estimated_right_servo_rate,
            _,
            _,
        ) = self._servo.step(
            self._estimated_right_servo_angle,
            self._estimated_right_servo_rate,
            self._applied_right_servo_angle,
            dt,
        )
        self._previous_command = (
            self._applied_left_thrust,
            self._applied_right_thrust,
            self._applied_reel_velocity,
            self._applied_left_servo_angle,
            self._applied_right_servo_angle,
        )
        return ActuatorCommand(
            timestamp_s=now,
            left_thrust_N=self._applied_left_thrust,
            right_thrust_N=self._applied_right_thrust,
            reel_velocity_m_s=self._applied_reel_velocity,
            left_servo_angle_rad=self._applied_left_servo_angle,
            right_servo_angle_rad=self._applied_right_servo_angle,
            reference_position_xz_m=reference.position,
            reference_velocity_xz_m_s=reference.velocity,
            solver_status=solution.status,
            solver_time_s=solution.solve_time_s,
        )
