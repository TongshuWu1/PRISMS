"""CasADi nonlinear MPC for the cable-supported wall-tool plant."""

from __future__ import annotations

import math
import time

import numpy as np

from .model import MPCConfig, MPCReferenceHorizon, MPCSolution


class WallToolNMPC:
    """Direct multiple-shooting NMPC with inextensible unilateral cable constraints."""

    NX = 7
    NU = 4
    PX = 0
    PZ = 1
    VX = 2
    VZ = 3
    PHI = 4
    OMEGA = 5
    CABLE_LENGTH = 6
    LEFT_THRUST = 0
    RIGHT_THRUST = 1
    CABLE_TENSION = 2
    SPOOL_SPEED = 3

    def __init__(self, config: MPCConfig) -> None:
        self.config = config
        self._last_x: np.ndarray | None = None
        self._last_u: np.ndarray | None = None
        self._last_solution: MPCSolution | None = None
        self._build_problem()

    def solve(
        self,
        measured_state: tuple[float, float, float, float, float, float, float],
        reference: MPCReferenceHorizon,
        previous_command: tuple[float, float, float, float],
    ) -> MPCSolution:
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
            solution = self.opti.solve()
            status = str(self.opti.stats().get("return_status", "Solve_Succeeded"))
            success = status in {"Solve_Succeeded", "Solved_To_Acceptable_Level"}
            x_value = np.array(solution.value(self.x_var), dtype=float)
            u_value = np.array(solution.value(self.u_var), dtype=float)
            objective = float(solution.value(self.objective))
        except RuntimeError as exc:
            if self._last_solution is None:
                raise RuntimeError(
                    "NMPC failed before any feasible command was available. "
                    "Check target feasibility, cable constraints, and CasADi/IPOPT installation."
                ) from exc
            failed = self._last_solution
            return MPCSolution(
                success=False,
                status=f"held previous feasible command after solver failure: {exc}",
                solve_time_s=time.perf_counter() - start,
                objective=failed.objective,
                left_thrust=failed.left_thrust,
                right_thrust=failed.right_thrust,
                cable_tension=failed.cable_tension,
                spool_velocity=failed.spool_velocity,
                predicted_positions=failed.predicted_positions,
                predicted_attitudes=failed.predicted_attitudes,
                predicted_tensions=failed.predicted_tensions,
                predicted_spool_speeds=failed.predicted_spool_speeds,
            )

        solve_time = time.perf_counter() - start
        self._last_x = x_value
        self._last_u = u_value
        result = self._solution_from_values(success, status, solve_time, objective, x_value, u_value)
        self._last_solution = result
        return result

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
            next_state = self._rk4_step(state, control)
            self.opti.subject_to(self.x_var[:, k + 1] == next_state)
            self._add_state_constraints(state)
            self._add_control_constraints(state, control)
            self._add_reel_rate_constraints(k)
            objective += self._stage_cost(k, state, control)

        terminal_state = self.x_var[:, n]
        self._add_state_constraints(terminal_state)
        terminal_error = terminal_state[self.PX : self.PZ + 1] - self.p_ref_param[:, n]
        terminal_velocity_error = terminal_state[self.VX : self.VZ + 1] - self.v_ref_param[:, n]
        objective += cfg.terminal_position_weight * self.ca.dot(terminal_error, terminal_error)
        objective += cfg.terminal_velocity_weight * self.ca.dot(terminal_velocity_error, terminal_velocity_error)
        objective += cfg.attitude_rate_weight * terminal_state[self.OMEGA] ** 2

        self.objective = objective
        self.opti.minimize(objective)
        self.opti.solver(
            "ipopt",
            {
                "print_time": False,
                "expand": True,
                "error_on_fail": True,
            },
            {
                "print_level": 0,
                "max_iter": cfg.solver_max_iter,
                "tol": cfg.solver_tolerance,
                "acceptable_tol": 10.0 * cfg.solver_tolerance,
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
            x_guess = np.column_stack([self._last_x[:, 1:], self._last_x[:, -1]])
            u_guess = np.column_stack([self._last_u[:, 1:], self._last_u[:, -1]])
            x_guess[:, 0] = x0
        else:
            x_guess = np.zeros((self.NX, n + 1), dtype=float)
            u_guess = np.zeros((self.NU, n), dtype=float)
            hover_each = min(
                self.config.max_thrust_per_drone,
                self.config.mass * self.config.gravity
                / max(2.0 * math.cos(self.config.hex_face_tilt_rad), 1e-9),
            )
            for k in range(n + 1):
                x_guess[self.PX, k] = p_ref[0, k]
                x_guess[self.PZ, k] = p_ref[1, k]
                x_guess[self.VX, k] = v_ref[0, k]
                x_guess[self.VZ, k] = v_ref[1, k]
                x_guess[self.PHI, k] = x0[self.PHI]
                x_guess[self.OMEGA, k] = 0.0
                x_guess[self.CABLE_LENGTH, k] = max(x0[self.CABLE_LENGTH], self._distance_guess(p_ref[:, k], x0[self.PHI]))
            u_guess[self.LEFT_THRUST, :] = hover_each
            u_guess[self.RIGHT_THRUST, :] = hover_each
            u_guess[self.CABLE_TENSION, :] = max(0.0, u_prev[self.CABLE_TENSION])
            u_guess[self.SPOOL_SPEED, :] = u_prev[self.SPOOL_SPEED]

        self.opti.set_initial(self.x_var, x_guess)
        self.opti.set_initial(self.u_var, u_guess)

    def _distance_guess(self, position: np.ndarray, attitude: float) -> float:
        offset_x = -self.config.payload_hex_radius * math.sin(attitude)
        offset_z = self.config.payload_hex_radius * math.cos(attitude)
        dx = self.config.anchor[0] - (float(position[0]) + offset_x)
        dz = self.config.anchor[1] - (float(position[1]) + offset_z)
        return math.hypot(dx, dz)

    def _stage_cost(self, k: int, state, control):
        cfg = self.config
        ca = self.ca
        position_error = state[self.PX : self.PZ + 1] - self.p_ref_param[:, k]
        velocity_error = state[self.VX : self.VZ + 1] - self.v_ref_param[:, k]
        if k == 0:
            input_step = control - self.u_prev_param
        else:
            input_step = control - self.u_var[:, k - 1]
        normalized_du = ca.vertcat(
            input_step[self.LEFT_THRUST] / max(cfg.max_thrust_per_drone, 1e-9),
            input_step[self.RIGHT_THRUST] / max(cfg.max_thrust_per_drone, 1e-9),
            input_step[self.CABLE_TENSION] / max(cfg.max_cable_tension, 1e-9),
            input_step[self.SPOOL_SPEED] / max(cfg.max_spool_speed, 1e-9),
        )
        distance = self._cable_distance(state)
        slack = state[self.CABLE_LENGTH] - distance
        attitude_error = state[self.PHI] - cfg.nominal_attitude_rad
        return (
            cfg.tracking_position_weight * ca.dot(position_error, position_error)
            + cfg.tracking_velocity_weight * ca.dot(velocity_error, velocity_error)
            + cfg.drone_effort_weight
            * (
                (control[self.LEFT_THRUST] / max(cfg.max_thrust_per_drone, 1e-9)) ** 2
                + (control[self.RIGHT_THRUST] / max(cfg.max_thrust_per_drone, 1e-9)) ** 2
            )
            + cfg.cable_effort_weight * (control[self.CABLE_TENSION] / max(cfg.max_cable_tension, 1e-9)) ** 2
            + cfg.reel_speed_weight * (control[self.SPOOL_SPEED] / max(cfg.max_spool_speed, 1e-9)) ** 2
            + cfg.input_rate_weight * ca.dot(normalized_du, normalized_du)
            + cfg.attitude_rate_weight * state[self.OMEGA] ** 2
            + cfg.attitude_weight * attitude_error * attitude_error
            + cfg.slack_weight * slack * slack
        )

    def _add_state_constraints(self, state) -> None:
        cfg = self.config
        distance = self._cable_distance(state)
        self.opti.subject_to(state[self.PX] >= -0.5 * cfg.wall_width + cfg.wall_margin)
        self.opti.subject_to(state[self.PX] <= 0.5 * cfg.wall_width - cfg.wall_margin)
        self.opti.subject_to(state[self.PZ] >= cfg.wall_margin)
        self.opti.subject_to(state[self.PZ] <= cfg.wall_height - cfg.wall_margin)
        self.opti.subject_to(state[self.PHI] >= cfg.nominal_attitude_rad - cfg.attitude_limit_rad)
        self.opti.subject_to(state[self.PHI] <= cfg.nominal_attitude_rad + cfg.attitude_limit_rad)
        self.opti.subject_to(state[self.CABLE_LENGTH] >= cfg.min_cable_length)
        self.opti.subject_to(state[self.CABLE_LENGTH] <= cfg.max_cable_length)
        self.opti.subject_to(state[self.CABLE_LENGTH] - distance >= 0.0)
        self.opti.subject_to(state[self.CABLE_LENGTH] - distance <= cfg.slack_limit_m)

    def _add_control_constraints(self, state, control) -> None:
        cfg = self.config
        cable_axis_z = self._cable_axis_z(state)
        self.opti.subject_to(control[self.LEFT_THRUST] >= 0.0)
        self.opti.subject_to(control[self.LEFT_THRUST] <= cfg.max_thrust_per_drone)
        self.opti.subject_to(control[self.RIGHT_THRUST] >= 0.0)
        self.opti.subject_to(control[self.RIGHT_THRUST] <= cfg.max_thrust_per_drone)
        self.opti.subject_to(control[self.CABLE_TENSION] >= 0.0)
        self.opti.subject_to(control[self.CABLE_TENSION] <= cfg.max_cable_tension)
        self.opti.subject_to(
            control[self.CABLE_TENSION] * cable_axis_z <= cfg.max_cable_support_fraction * cfg.mass * cfg.gravity
        )
        self.opti.subject_to(
            control[self.CABLE_TENSION] * (cfg.min_cable_vertical_efficiency - cable_axis_z) <= 0.0
        )
        self.opti.subject_to(control[self.SPOOL_SPEED] >= -cfg.max_spool_speed)
        self.opti.subject_to(control[self.SPOOL_SPEED] <= cfg.max_spool_speed)

    def _add_reel_rate_constraints(self, k: int) -> None:
        cfg = self.config
        if k == 0:
            max_delta = cfg.spool_accel_limit_mps2 * cfg.control_period_s
            previous_speed = self.u_prev_param[self.SPOOL_SPEED]
        else:
            max_delta = cfg.spool_accel_limit_mps2 * cfg.horizon_dt
            previous_speed = self.u_var[self.SPOOL_SPEED, k - 1]
        delta = self.u_var[self.SPOOL_SPEED, k] - previous_speed
        self.opti.subject_to(delta <= max_delta)
        self.opti.subject_to(delta >= -max_delta)

    def _rk4_step(self, state, control):
        dt = self.config.horizon_dt
        k1 = self._dynamics(state, control)
        k2 = self._dynamics(state + 0.5 * dt * k1, control)
        k3 = self._dynamics(state + 0.5 * dt * k2, control)
        k4 = self._dynamics(state + dt * k3, control)
        return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def _dynamics(self, state, control):
        cfg = self.config
        ca = self.ca
        left_axis_x, left_axis_z, right_axis_x, right_axis_z = self._drone_axes(state[self.PHI])
        cable_axis_x, cable_axis_z = self._cable_axis(state)
        left_arm_x, left_arm_z, right_arm_x, right_arm_z = self._module_arms(state[self.PHI])
        cable_arm_x, cable_arm_z = self._cable_arm(state[self.PHI])

        left_fx = control[self.LEFT_THRUST] * left_axis_x
        left_fz = control[self.LEFT_THRUST] * left_axis_z
        right_fx = control[self.RIGHT_THRUST] * right_axis_x
        right_fz = control[self.RIGHT_THRUST] * right_axis_z
        cable_fx = control[self.CABLE_TENSION] * cable_axis_x
        cable_fz = control[self.CABLE_TENSION] * cable_axis_z
        force_x = left_fx + right_fx + cable_fx
        force_z = left_fz + right_fz + cable_fz - cfg.mass * cfg.gravity
        torque = (
            left_arm_x * left_fz
            - left_arm_z * left_fx
            + right_arm_x * right_fz
            - right_arm_z * right_fx
            + cable_arm_x * cable_fz
            - cable_arm_z * cable_fx
            - cfg.rotational_damping * state[self.OMEGA]
        )
        return ca.vertcat(
            state[self.VX],
            state[self.VZ],
            force_x / cfg.mass,
            force_z / cfg.mass,
            state[self.OMEGA],
            torque / cfg.inertia,
            control[self.SPOOL_SPEED],
        )

    def _cable_arm(self, attitude):
        ca = self.ca
        radius = self.config.payload_hex_radius
        return -radius * ca.sin(attitude), radius * ca.cos(attitude)

    def _module_arms(self, attitude):
        ca = self.ca
        c = ca.cos(attitude)
        s = ca.sin(attitude)
        left_x0, left_z0 = self.config.left_center_offset_zero
        right_x0, right_z0 = self.config.right_center_offset_zero
        left_x = c * left_x0 - s * left_z0
        left_z = s * left_x0 + c * left_z0
        right_x = c * right_x0 - s * right_z0
        right_z = s * right_x0 + c * right_z0
        return left_x, left_z, right_x, right_z

    def _drone_axes(self, attitude):
        ca = self.ca
        local_angle = attitude - self.config.nominal_attitude_rad
        c = ca.cos(local_angle)
        s = ca.sin(local_angle)
        face_sin = math.sin(self.config.hex_face_tilt_rad)
        face_cos = math.cos(self.config.hex_face_tilt_rad)
        left_x = c * face_sin - s * face_cos
        left_z = s * face_sin + c * face_cos
        right_x = c * (-face_sin) - s * face_cos
        right_z = s * (-face_sin) + c * face_cos
        return left_x, left_z, right_x, right_z

    def _cable_mount(self, state):
        arm_x, arm_z = self._cable_arm(state[self.PHI])
        return state[self.PX] + arm_x, state[self.PZ] + arm_z

    def _cable_axis(self, state):
        ca = self.ca
        mount_x, mount_z = self._cable_mount(state)
        dx = self.config.anchor[0] - mount_x
        dz = self.config.anchor[1] - mount_z
        distance = ca.sqrt(dx * dx + dz * dz + 1e-10)
        return dx / distance, dz / distance

    def _cable_axis_z(self, state):
        _axis_x, axis_z = self._cable_axis(state)
        return axis_z

    def _cable_distance(self, state):
        ca = self.ca
        mount_x, mount_z = self._cable_mount(state)
        dx = self.config.anchor[0] - mount_x
        dz = self.config.anchor[1] - mount_z
        return ca.sqrt(dx * dx + dz * dz + 1e-10)

    def _solution_from_values(
        self,
        success: bool,
        status: str,
        solve_time_s: float,
        objective: float,
        x_value: np.ndarray,
        u_value: np.ndarray,
    ) -> MPCSolution:
        first = u_value[:, 0]
        return MPCSolution(
            success=success,
            status=status,
            solve_time_s=solve_time_s,
            objective=objective,
            left_thrust=float(first[self.LEFT_THRUST]),
            right_thrust=float(first[self.RIGHT_THRUST]),
            cable_tension=float(first[self.CABLE_TENSION]),
            spool_velocity=float(first[self.SPOOL_SPEED]),
            predicted_positions=tuple((float(x_value[self.PX, k]), float(x_value[self.PZ, k])) for k in range(x_value.shape[1])),
            predicted_attitudes=tuple(float(x_value[self.PHI, k]) for k in range(x_value.shape[1])),
            predicted_tensions=tuple(float(u_value[self.CABLE_TENSION, k]) for k in range(u_value.shape[1])),
            predicted_spool_speeds=tuple(float(u_value[self.SPOOL_SPEED, k]) for k in range(u_value.shape[1])),
        )
