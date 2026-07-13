"""CasADi nonlinear MPC for the cable-supported wall-tool plant.

The optimizer commands only interfaces that exist on the proposed hardware:
left/right thrust, left/right one-axis gimbal angle, and reel line velocity.
Cable tension is not a command. It is predicted from paid-out cable length,
cable geometry, stretch, stretch rate, and the reel actuator state.
"""

from __future__ import annotations

import time

import numpy as np

from .model import MPCConfig, MPCReferenceHorizon, MPCSolution


class WallToolNMPC:
    """Direct-shooting NMPC with actuator and unilateral cable dynamics."""

    NX = 15
    NU = 5

    PX = 0
    PZ = 1
    VX = 2
    VZ = 3
    PHI = 4
    OMEGA = 5
    CABLE_LENGTH = 6
    LEFT_THRUST = 7
    RIGHT_THRUST = 8
    REEL_SPEED = 9
    TENSION = 10
    LEFT_GIMBAL_ANGLE = 11
    RIGHT_GIMBAL_ANGLE = 12
    LEFT_GIMBAL_RATE = 13
    RIGHT_GIMBAL_RATE = 14

    LEFT_THRUST_COMMAND = 0
    RIGHT_THRUST_COMMAND = 1
    REEL_SPEED_COMMAND = 2
    LEFT_GIMBAL_ANGLE_COMMAND = 3
    RIGHT_GIMBAL_ANGLE_COMMAND = 4

    def __init__(self, config: MPCConfig) -> None:
        self.config = config
        self._last_x: np.ndarray | None = None
        self._last_u: np.ndarray | None = None
        self._build_problem()

    def solve(
        self,
        measured_state: tuple[float, ...],
        reference: MPCReferenceHorizon,
        previous_command: tuple[float, ...],
    ) -> MPCSolution:
        if len(measured_state) != self.NX:
            raise ValueError(f"NMPC measured state must contain {self.NX} values")
        if len(reference.positions) != self.config.horizon_steps + 1:
            raise ValueError("MPC reference position horizon length does not match horizon_steps + 1")
        if len(reference.velocities) != self.config.horizon_steps + 1:
            raise ValueError("MPC reference velocity horizon length does not match horizon_steps + 1")

        x0 = np.array(measured_state, dtype=float).reshape(self.NX)
        u_prev = np.array(previous_command, dtype=float).reshape(self.NU)
        p_ref = np.array(reference.positions, dtype=float).T
        v_ref = np.array(reference.velocities, dtype=float).T

        self.opti.set_value(self.x0_param, x0)
        self.opti.set_value(self.u_prev_param, u_prev)
        self.opti.set_value(self.p_ref_param, p_ref)
        self.opti.set_value(self.v_ref_param, v_ref)
        self._set_initial_guess(x0, u_prev, p_ref, v_ref)

        start = time.perf_counter()
        try:
            solution = self.opti.solve_limited()
        except RuntimeError as exc:
            raise RuntimeError(
                "Vector-thrust NMPC failed; no previous command or backup controller was applied. "
                f"Solver detail: {exc}"
            ) from exc
        status = str(self.opti.stats().get("return_status", "unknown"))
        constraint_violation = self._constraint_violation(solution)
        if status not in {"Solve_Succeeded", "Solved_To_Acceptable_Level"}:
            detail = self._largest_constraint_violation(solution)
            raise RuntimeError(
                "Vector-thrust NMPC did not converge and no fallback is permitted: "
                f"IPOPT status={status}, maximum constraint violation={constraint_violation:.3e}; "
                f"{detail}"
            )
        if constraint_violation > max(1.0e-6, 10.0 * self.config.solver_tolerance):
            raise RuntimeError(
                "Vector-thrust NMPC returned a constraint-violating solution: "
                f"maximum violation={constraint_violation:.3e}"
            )
        success = True
        x_value = np.array(solution.value(self.x_expr), dtype=float)
        u_value = np.array(solution.value(self.u_var), dtype=float)
        tension_value = np.asarray(solution.value(self.tension_expr), dtype=float).reshape(-1)
        objective = float(solution.value(self.objective))

        solve_time = time.perf_counter() - start
        self._last_x = x_value
        self._last_u = u_value
        result = self._solution_from_values(
            success,
            status,
            solve_time,
            objective,
            x_value,
            u_value,
            tension_value,
        )
        return result

    def _constraint_violation(self, solution) -> float:
        constraint = np.asarray(solution.value(self.opti.g), dtype=float).reshape(-1)
        lower = np.asarray(solution.value(self.opti.lbg), dtype=float).reshape(-1)
        upper = np.asarray(solution.value(self.opti.ubg), dtype=float).reshape(-1)
        below = np.maximum(lower - constraint, 0.0)
        above = np.maximum(constraint - upper, 0.0)
        return float(max(np.max(below, initial=0.0), np.max(above, initial=0.0)))

    def _largest_constraint_violation(self, solution) -> str:
        constraint = np.asarray(solution.value(self.opti.g), dtype=float).reshape(-1)
        lower = np.asarray(solution.value(self.opti.lbg), dtype=float).reshape(-1)
        upper = np.asarray(solution.value(self.opti.ubg), dtype=float).reshape(-1)
        below = np.maximum(lower - constraint, 0.0)
        above = np.maximum(constraint - upper, 0.0)
        violations = np.maximum(below, above)
        index = int(np.argmax(violations))
        return (
            f"worst constraint index={index}, lower={lower[index]:+.6g}, "
            f"value={constraint[index]:+.6g}, upper={upper[index]:+.6g}"
        )

    def _build_problem(self) -> None:
        try:
            import casadi as ca
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "CasADi is required for tool_head_nmpc. Install it with:\n"
                "  python -m pip install -r requirements.txt"
            ) from exc

        self.ca = ca
        cfg = self.config
        n = cfg.horizon_steps
        self.opti = ca.Opti()
        self.x_var = self.opti.variable(self.NX, n + 1)
        self.u_var = self.opti.variable(self.NU, n)
        self.x0_param = self.opti.parameter(self.NX)
        self.u_prev_param = self.opti.parameter(self.NU)
        self.p_ref_param = self.opti.parameter(2, n + 1)
        self.v_ref_param = self.opti.parameter(2, n + 1)

        self.opti.subject_to(self.x_var[:, 0] == self.x0_param)
        objective = 0.0
        for k in range(n):
            state = self.x_var[:, k]
            control = self.u_var[:, k]
            tension = self._cable_tension(state)
            if k > 0:
                self._add_state_constraints(state, tension)
            self._add_control_constraints(state, control, tension)
            self._add_command_rate_constraints(k)
            objective += self._stage_cost(k, state, control, tension)
            # Implicit midpoint integration is A-stable for the stiff cable
            # and fast actuator states. This preserves the measured hardware
            # time constants without an artificially slow prediction model.
            next_state = self.x_var[:, k + 1]
            midpoint = 0.5 * (state + next_state)
            self.opti.subject_to(
                next_state
                == state + cfg.horizon_dt * self._dynamics(midpoint, control)
            )

        terminal_state = self.x_var[:, n]
        terminal_tension = self._cable_tension(terminal_state)
        self._add_state_constraints(terminal_state, terminal_tension)
        terminal_error = terminal_state[self.PX : self.PZ + 1] - self.p_ref_param[:, n]
        terminal_velocity_error = terminal_state[self.VX : self.VZ + 1] - self.v_ref_param[:, n]
        objective += cfg.terminal_position_weight * ca.dot(terminal_error, terminal_error)
        objective += cfg.terminal_velocity_weight * ca.dot(terminal_velocity_error, terminal_velocity_error)
        objective += cfg.attitude_rate_weight * terminal_state[self.OMEGA] ** 2
        objective += cfg.attitude_weight * (
            terminal_state[self.PHI] - cfg.nominal_attitude_rad
        ) ** 2
        objective += cfg.gimbal_rate_weight * (
            terminal_state[self.LEFT_GIMBAL_RATE] ** 2
            + terminal_state[self.RIGHT_GIMBAL_RATE] ** 2
        )

        self.x_expr = self.x_var
        self.tension_expr = self.x_var[self.TENSION, :].T
        self.objective = objective
        self.opti.minimize(objective)
        self.opti.solver(
            "ipopt",
            {"print_time": False, "expand": True, "error_on_fail": False},
            {
                "print_level": 0,
                "max_iter": cfg.solver_max_iter,
                "tol": cfg.solver_tolerance,
                "acceptable_tol": 10.0 * cfg.solver_tolerance,
                "acceptable_iter": 1,
                "sb": "yes",
            },
        )

    def _set_initial_guess(
        self,
        x0: np.ndarray,
        u_prev: np.ndarray,
        p_ref: np.ndarray,
        v_ref: np.ndarray,
    ) -> None:
        n = self.config.horizon_steps
        if self._last_x is not None and self._last_u is not None:
            # Solves occur every control_period_s, while prediction nodes are
            # horizon_dt apart. Shifting by one whole node when those periods
            # differ advances the warm start to the wrong physical time and
            # is especially harmful at a direction reversal.
            fractional_shift = self.config.control_period_s / self.config.horizon_dt
            x_grid = np.arange(n + 1, dtype=float)
            x_sample = np.clip(x_grid + fractional_shift, 0.0, float(n))
            x_guess = np.vstack([
                np.interp(x_sample, x_grid, self._last_x[row, :])
                for row in range(self.NX)
            ])
            x_guess[:, 0] = x0
            u_grid = np.arange(n, dtype=float)
            u_sample = np.clip(u_grid + fractional_shift, 0.0, float(n - 1))
            u_guess = np.vstack([
                np.interp(u_sample, u_grid, self._last_u[row, :])
                for row in range(self.NU)
            ])
        else:
            x_guess = np.repeat(x0.reshape(self.NX, 1), n + 1, axis=1)
            u_guess = np.repeat(u_prev.reshape(self.NU, 1), n, axis=1)

        # Project the control warm start through the same hard bounds and slew
        # limits used by the NLP. This is only an initial guess; it never
        # bypasses a constraint or supplies a command when optimization fails.
        cfg = self.config
        lower = np.array([
            0.0, 0.0, -cfg.max_spool_speed,
            -cfg.max_gimbal_angle_rad, -cfg.max_gimbal_angle_rad,
        ])
        upper = np.array([
            cfg.max_thrust_per_drone, cfg.max_thrust_per_drone, cfg.max_spool_speed,
            cfg.max_gimbal_angle_rad, cfg.max_gimbal_angle_rad,
        ])
        slew_per_second = np.array([
            cfg.thrust_command_slew_limit_N_s,
            cfg.thrust_command_slew_limit_N_s,
            cfg.reel_velocity_slew_limit_mps2,
            cfg.gimbal_command_slew_limit_rad_s,
            cfg.gimbal_command_slew_limit_rad_s,
        ])
        previous = u_prev.copy()
        for column in range(n):
            interval = cfg.control_period_s if column == 0 else cfg.horizon_dt
            delta = slew_per_second * interval
            u_guess[:, column] = np.clip(u_guess[:, column], lower, upper)
            u_guess[:, column] = np.clip(
                u_guess[:, column],
                previous - delta,
                previous + delta,
            )
            previous = u_guess[:, column]

        self.opti.set_initial(self.x_var, x_guess)
        self.opti.set_initial(self.u_var, u_guess)

    def _stage_cost(self, k: int, state, control, tension):
        cfg = self.config
        ca = self.ca
        position_error = state[self.PX : self.PZ + 1] - self.p_ref_param[:, k]
        velocity_error = state[self.VX : self.VZ + 1] - self.v_ref_param[:, k]
        input_step = control - (self.u_prev_param if k == 0 else self.u_var[:, k - 1])
        normalized_du = ca.vertcat(
            input_step[self.LEFT_THRUST_COMMAND] / max(cfg.max_thrust_per_drone, 1e-9),
            input_step[self.RIGHT_THRUST_COMMAND] / max(cfg.max_thrust_per_drone, 1e-9),
            input_step[self.REEL_SPEED_COMMAND] / max(cfg.max_spool_speed, 1e-9),
            input_step[self.LEFT_GIMBAL_ANGLE_COMMAND] / max(cfg.max_gimbal_angle_rad, 1e-9),
            input_step[self.RIGHT_GIMBAL_ANGLE_COMMAND] / max(cfg.max_gimbal_angle_rad, 1e-9),
        )
        slack = self._smooth_positive(state[self.CABLE_LENGTH] - self._cable_distance(state), 1e-5)
        attitude_error = state[self.PHI] - cfg.nominal_attitude_rad
        _cable_axis_x, cable_axis_z = self._cable_axis(state)
        support_error = (
            tension * cable_axis_z - cfg.desired_cable_support_fraction * cfg.mass * cfg.gravity
        ) / max(cfg.mass * cfg.gravity, 1e-9)
        return (
            cfg.tracking_position_weight * ca.dot(position_error, position_error)
            + cfg.tracking_velocity_weight * ca.dot(velocity_error, velocity_error)
            + cfg.drone_effort_weight
            * (
                (state[self.LEFT_THRUST] / max(cfg.max_thrust_per_drone, 1e-9)) ** 2
                + (state[self.RIGHT_THRUST] / max(cfg.max_thrust_per_drone, 1e-9)) ** 2
            )
            + cfg.cable_effort_weight * (tension / max(cfg.max_cable_tension, 1e-9)) ** 2
            + cfg.reel_speed_weight * (state[self.REEL_SPEED] / max(cfg.max_spool_speed, 1e-9)) ** 2
            + cfg.input_rate_weight * ca.dot(normalized_du, normalized_du)
            + cfg.attitude_rate_weight * state[self.OMEGA] ** 2
            + cfg.attitude_weight * attitude_error * attitude_error
            + cfg.gimbal_angle_weight
            * (
                state[self.LEFT_GIMBAL_ANGLE] ** 2
                + state[self.RIGHT_GIMBAL_ANGLE] ** 2
            )
            + cfg.gimbal_rate_weight
            * (
                state[self.LEFT_GIMBAL_RATE] ** 2
                + state[self.RIGHT_GIMBAL_RATE] ** 2
            )
            + cfg.cable_support_weight * support_error * support_error
            + cfg.slack_weight * slack * slack
        )

    def _add_state_constraints(self, state, tension) -> None:
        cfg = self.config
        self.opti.subject_to(state[self.PX] >= -0.5 * cfg.wall_width + cfg.wall_margin)
        self.opti.subject_to(state[self.PX] <= 0.5 * cfg.wall_width - cfg.wall_margin)
        self.opti.subject_to(state[self.PZ] >= cfg.wall_margin)
        self.opti.subject_to(state[self.PZ] <= cfg.wall_height - cfg.wall_margin)
        self.opti.subject_to(
            state[self.VX] ** 2 + state[self.VZ] ** 2 <= cfg.max_payload_speed_m_s**2
        )
        self.opti.subject_to(state[self.PHI] >= cfg.nominal_attitude_rad - cfg.attitude_limit_rad)
        self.opti.subject_to(state[self.PHI] <= cfg.nominal_attitude_rad + cfg.attitude_limit_rad)
        self.opti.subject_to(state[self.CABLE_LENGTH] >= cfg.min_cable_length)
        self.opti.subject_to(state[self.CABLE_LENGTH] <= cfg.max_cable_length)
        self.opti.subject_to(state[self.LEFT_THRUST] >= 0.0)
        self.opti.subject_to(state[self.LEFT_THRUST] <= cfg.max_thrust_per_drone)
        self.opti.subject_to(state[self.RIGHT_THRUST] >= 0.0)
        self.opti.subject_to(state[self.RIGHT_THRUST] <= cfg.max_thrust_per_drone)
        self.opti.subject_to(state[self.REEL_SPEED] >= -cfg.max_spool_speed)
        self.opti.subject_to(state[self.REEL_SPEED] <= cfg.max_spool_speed)
        self.opti.subject_to(state[self.TENSION] >= cfg.min_tracking_tension)
        self.opti.subject_to(tension <= cfg.max_cable_tension)
        self.opti.subject_to(state[self.LEFT_GIMBAL_ANGLE] >= -cfg.max_gimbal_angle_rad)
        self.opti.subject_to(state[self.LEFT_GIMBAL_ANGLE] <= cfg.max_gimbal_angle_rad)
        self.opti.subject_to(state[self.RIGHT_GIMBAL_ANGLE] >= -cfg.max_gimbal_angle_rad)
        self.opti.subject_to(state[self.RIGHT_GIMBAL_ANGLE] <= cfg.max_gimbal_angle_rad)
        self.opti.subject_to(state[self.LEFT_GIMBAL_RATE] >= -cfg.max_gimbal_rate_rad_s)
        self.opti.subject_to(state[self.LEFT_GIMBAL_RATE] <= cfg.max_gimbal_rate_rad_s)
        self.opti.subject_to(state[self.RIGHT_GIMBAL_RATE] >= -cfg.max_gimbal_rate_rad_s)
        self.opti.subject_to(state[self.RIGHT_GIMBAL_RATE] <= cfg.max_gimbal_rate_rad_s)
        extension = self._cable_distance(state) - state[self.CABLE_LENGTH]
        self.opti.subject_to(extension <= cfg.max_cable_extension_m)
        self.opti.subject_to(extension >= -cfg.slack_limit_m)
        _cable_axis_x, cable_axis_z = self._cable_axis(state)
        self.opti.subject_to(cable_axis_z >= cfg.min_cable_vertical_efficiency)
        # The desired support fraction belongs in the objective, not in the
        # feasible set. A taut cable may transiently carry more than payload
        # weight while the body accelerates or vectoring rotors push downward.
        # The real safety constraints are maximum line tension, cable
        # extension, attitude, and actuator limits, all retained above.

    def _add_control_constraints(self, state, control, tension) -> None:
        cfg = self.config
        self.opti.subject_to(control[self.LEFT_THRUST_COMMAND] >= 0.0)
        self.opti.subject_to(control[self.LEFT_THRUST_COMMAND] <= cfg.max_thrust_per_drone)
        self.opti.subject_to(control[self.RIGHT_THRUST_COMMAND] >= 0.0)
        self.opti.subject_to(control[self.RIGHT_THRUST_COMMAND] <= cfg.max_thrust_per_drone)
        self.opti.subject_to(control[self.LEFT_GIMBAL_ANGLE_COMMAND] >= -cfg.max_gimbal_angle_rad)
        self.opti.subject_to(control[self.LEFT_GIMBAL_ANGLE_COMMAND] <= cfg.max_gimbal_angle_rad)
        self.opti.subject_to(control[self.RIGHT_GIMBAL_ANGLE_COMMAND] >= -cfg.max_gimbal_angle_rad)
        self.opti.subject_to(control[self.RIGHT_GIMBAL_ANGLE_COMMAND] <= cfg.max_gimbal_angle_rad)
        self.opti.subject_to(control[self.REEL_SPEED_COMMAND] <= cfg.max_spool_speed)
        reel_in_fraction = 1.0 - tension / max(cfg.reel_stall_line_force_N, 1e-9)
        self.opti.subject_to(
            control[self.REEL_SPEED_COMMAND] >= -cfg.max_spool_speed * reel_in_fraction
        )

    def _add_command_rate_constraints(self, k: int) -> None:
        cfg = self.config
        if k == 0:
            interval = cfg.control_period_s
            previous = self.u_prev_param
        else:
            interval = cfg.horizon_dt
            previous = self.u_var[:, k - 1]
        reel_delta = self.u_var[self.REEL_SPEED_COMMAND, k] - previous[self.REEL_SPEED_COMMAND]
        max_reel_delta = cfg.reel_velocity_slew_limit_mps2 * interval
        self.opti.subject_to(reel_delta <= max_reel_delta)
        self.opti.subject_to(reel_delta >= -max_reel_delta)
        max_thrust_delta = cfg.thrust_command_slew_limit_N_s * interval
        for command_index in (
            self.LEFT_THRUST_COMMAND,
            self.RIGHT_THRUST_COMMAND,
        ):
            delta = self.u_var[command_index, k] - previous[command_index]
            self.opti.subject_to(delta <= max_thrust_delta)
            self.opti.subject_to(delta >= -max_thrust_delta)
        max_gimbal_delta = cfg.gimbal_command_slew_limit_rad_s * interval
        for command_index in (
            self.LEFT_GIMBAL_ANGLE_COMMAND,
            self.RIGHT_GIMBAL_ANGLE_COMMAND,
        ):
            delta = self.u_var[command_index, k] - previous[command_index]
            self.opti.subject_to(delta <= max_gimbal_delta)
            self.opti.subject_to(delta >= -max_gimbal_delta)

    def _dynamics(self, state, control):
        cfg = self.config
        ca = self.ca
        left_axis_x, left_axis_z, right_axis_x, right_axis_z = self._drone_axes(
            state[self.PHI],
            state[self.LEFT_GIMBAL_ANGLE],
            state[self.RIGHT_GIMBAL_ANGLE],
        )
        cable_axis_x, cable_axis_z = self._cable_axis(state)
        left_arm_x, left_arm_z, right_arm_x, right_arm_z = self._module_arms(state[self.PHI])
        cable_arm_x, cable_arm_z = self._cable_arm(state[self.PHI])
        tension = self._cable_tension(state)

        left_fx = state[self.LEFT_THRUST] * left_axis_x
        left_fz = state[self.LEFT_THRUST] * left_axis_z
        right_fx = state[self.RIGHT_THRUST] * right_axis_x
        right_fz = state[self.RIGHT_THRUST] * right_axis_z
        cable_fx = tension * cable_axis_x
        cable_fz = tension * cable_axis_z
        cable_weight = (
            cfg.cable_payload_weight_fraction
            * cfg.cable_mass_per_length_kg_m
            * self._cable_distance(state)
            * cfg.gravity
        )
        force_x = left_fx + right_fx + cable_fx
        force_z = left_fz + right_fz + cable_fz - cfg.mass * cfg.gravity - cable_weight
        passive_torque = (
            -cfg.passive_attitude_stiffness_Nm_rad
            * ca.sin(state[self.PHI] - cfg.nominal_attitude_rad)
            - cfg.passive_attitude_damping_Nm_s_rad * state[self.OMEGA]
        )
        torque = (
            left_arm_x * left_fz
            - left_arm_z * left_fx
            + right_arm_x * right_fz
            - right_arm_z * right_fx
            + cable_arm_x * cable_fz
            - cable_arm_z * cable_fx
            - cable_arm_x * cable_weight
            - cfg.rotational_damping * state[self.OMEGA]
            + passive_torque
        )
        left_gimbal_acceleration = self._gimbal_acceleration(
            state[self.LEFT_GIMBAL_ANGLE],
            state[self.LEFT_GIMBAL_RATE],
            control[self.LEFT_GIMBAL_ANGLE_COMMAND],
        )
        right_gimbal_acceleration = self._gimbal_acceleration(
            state[self.RIGHT_GIMBAL_ANGLE],
            state[self.RIGHT_GIMBAL_RATE],
            control[self.RIGHT_GIMBAL_ANGLE_COMMAND],
        )
        return ca.vertcat(
            state[self.VX],
            state[self.VZ],
            force_x / cfg.mass,
            force_z / cfg.mass,
            state[self.OMEGA],
            torque / cfg.inertia,
            state[self.REEL_SPEED],
            (control[self.LEFT_THRUST_COMMAND] - state[self.LEFT_THRUST])
            / max(cfg.motor_thrust_time_constant_s, 1e-6),
            (control[self.RIGHT_THRUST_COMMAND] - state[self.RIGHT_THRUST])
            / max(cfg.motor_thrust_time_constant_s, 1e-6),
            (control[self.REEL_SPEED_COMMAND] - state[self.REEL_SPEED])
            / max(cfg.reel_velocity_time_constant_s, 1e-6),
            (self._spring_tension_target(state) - state[self.TENSION])
            / max(cfg.cable_tension_time_constant_s, 1e-6),
            state[self.LEFT_GIMBAL_RATE],
            state[self.RIGHT_GIMBAL_RATE],
            left_gimbal_acceleration,
            right_gimbal_acceleration,
        )

    def _gimbal_acceleration(self, angle, rate, command):
        cfg = self.config
        raw = (
            cfg.gimbal_natural_frequency_rad_s**2 * (command - angle)
            - 2.0 * cfg.gimbal_damping_ratio * cfg.gimbal_natural_frequency_rad_s * rate
        )
        return self.ca.fmin(
            cfg.max_gimbal_acceleration_rad_s2,
            self.ca.fmax(-cfg.max_gimbal_acceleration_rad_s2, raw),
        )

    def _smooth_positive(self, value, epsilon: float):
        return 0.5 * (value + self.ca.sqrt(value * value + epsilon * epsilon))

    def _cable_tension(self, state):
        return state[self.TENSION]

    def _spring_tension_target(self, state):
        cfg = self.config
        extension = self._cable_distance(state) - state[self.CABLE_LENGTH]
        extension_rate = self._cable_distance_rate(state) - state[self.REEL_SPEED]
        positive_extension = self._smooth_positive(extension, 1e-6)
        raw_tension = cfg.cable_stiffness_N_m * positive_extension + cfg.cable_damping_N_s_m * extension_rate
        return self._smooth_positive(raw_tension, 1e-5)

    def _cable_arm(self, attitude):
        radius = self.config.payload_hex_radius
        return -radius * self.ca.sin(attitude), radius * self.ca.cos(attitude)

    def _module_arms(self, attitude):
        c = self.ca.cos(attitude)
        s = self.ca.sin(attitude)
        left_x0, left_z0 = self.config.left_center_offset_zero
        right_x0, right_z0 = self.config.right_center_offset_zero
        return (
            c * left_x0 - s * left_z0,
            s * left_x0 + c * left_z0,
            c * right_x0 - s * right_z0,
            s * right_x0 + c * right_z0,
        )

    def _drone_axes(self, attitude, left_gimbal_angle, right_gimbal_angle):
        left_world_angle = attitude - self.config.nominal_attitude_rad + left_gimbal_angle
        right_world_angle = attitude - self.config.nominal_attitude_rad + right_gimbal_angle
        return (
            self.ca.sin(left_world_angle),
            self.ca.cos(left_world_angle),
            self.ca.sin(right_world_angle),
            self.ca.cos(right_world_angle),
        )

    def _cable_mount(self, state):
        arm_x, arm_z = self._cable_arm(state[self.PHI])
        return state[self.PX] + arm_x, state[self.PZ] + arm_z

    def _cable_axis(self, state):
        mount_x, mount_z = self._cable_mount(state)
        dx = self.config.anchor[0] - mount_x
        dz = self.config.anchor[1] - mount_z
        distance = self.ca.sqrt(dx * dx + dz * dz + 1e-10)
        return dx / distance, dz / distance

    def _cable_axis_z(self, state):
        return self._cable_axis(state)[1]

    def _cable_distance(self, state):
        mount_x, mount_z = self._cable_mount(state)
        dx = self.config.anchor[0] - mount_x
        dz = self.config.anchor[1] - mount_z
        return self.ca.sqrt(dx * dx + dz * dz + 1e-10)

    def _cable_distance_rate(self, state):
        arm_x, arm_z = self._cable_arm(state[self.PHI])
        mount_vx = state[self.VX] - arm_z * state[self.OMEGA]
        mount_vz = state[self.VZ] + arm_x * state[self.OMEGA]
        mount_x, mount_z = self._cable_mount(state)
        out_x = mount_x - self.config.anchor[0]
        out_z = mount_z - self.config.anchor[1]
        distance = self.ca.sqrt(out_x * out_x + out_z * out_z + 1e-10)
        return (out_x * mount_vx + out_z * mount_vz) / distance

    def _solution_from_values(
        self,
        success: bool,
        status: str,
        solve_time_s: float,
        objective: float,
        x_value: np.ndarray,
        u_value: np.ndarray,
        tension_value: np.ndarray,
    ) -> MPCSolution:
        first = u_value[:, 0]
        return MPCSolution(
            success=success,
            status=status,
            solve_time_s=solve_time_s,
            objective=objective,
            left_thrust=float(first[self.LEFT_THRUST_COMMAND]),
            right_thrust=float(first[self.RIGHT_THRUST_COMMAND]),
            left_gimbal_angle=float(first[self.LEFT_GIMBAL_ANGLE_COMMAND]),
            right_gimbal_angle=float(first[self.RIGHT_GIMBAL_ANGLE_COMMAND]),
            cable_tension=float(tension_value[0]),
            spool_velocity=float(first[self.REEL_SPEED_COMMAND]),
            predicted_positions=tuple(
                (float(x_value[self.PX, k]), float(x_value[self.PZ, k]))
                for k in range(x_value.shape[1])
            ),
            predicted_attitudes=tuple(float(x_value[self.PHI, k]) for k in range(x_value.shape[1])),
            predicted_left_gimbal_angles=tuple(
                float(x_value[self.LEFT_GIMBAL_ANGLE, k]) for k in range(x_value.shape[1])
            ),
            predicted_right_gimbal_angles=tuple(
                float(x_value[self.RIGHT_GIMBAL_ANGLE, k]) for k in range(x_value.shape[1])
            ),
            predicted_tensions=tuple(float(value) for value in tension_value[:-1]),
            predicted_spool_speeds=tuple(float(x_value[self.REEL_SPEED, k]) for k in range(x_value.shape[1] - 1)),
        )
