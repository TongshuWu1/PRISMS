from __future__ import annotations

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WALL_TOOL_2D_ROOT = PROJECT_ROOT / "wall_tool_2d"
for path in (WALL_TOOL_2D_ROOT, PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cable_hybrid_controller.controller import (  # noqa: E402
    best_params,
    command_controller,
    default_scenario,
    make_simulator,
)
from wall_tool_sim.gimbal_servo import GimbalServoSpec  # noqa: E402
from wall_tool_sim.wall_tool_ui import SimParams, integrated_motor_axes  # noqa: E402


class VectorThrustInspectionTests(unittest.TestCase):
    def test_configuration_rejects_hidden_alternatives(self) -> None:
        with self.assertRaisesRegex(ValueError, "no backup controller"):
            SimParams(control_law="sensor_cascade")
        with self.assertRaisesRegex(ValueError, "does not model or command wall-normal contact"):
            SimParams(normal_contact_enabled=True)

    def test_external_plant_fails_until_tilt_servo_channels_exist(self) -> None:
        with self.assertRaisesRegex(NotImplementedError, "tilt-servo channels"):
            make_simulator(external_plant=True)

    def test_270_degree_servo_enforces_travel_and_pwm_resolution(self) -> None:
        servo = GimbalServoSpec()
        self.assertAlmostEqual(math.degrees(2.0 * servo.max_angle_rad), 270.0)
        self.assertAlmostEqual(math.degrees(servo.command_resolution_rad), 0.135)
        with self.assertRaisesRegex(ValueError, "travel limit"):
            servo.validate_command(servo.max_angle_rad + math.radians(1.0))
        requested = math.radians(12.345)
        realized = servo.realize_pwm_command(requested)
        self.assertLessEqual(abs(realized - requested), 0.5 * servo.command_resolution_rad + 1e-12)
        angle, rate, acceleration, saturated = servo.step(0.0, 0.0, math.radians(45.0), 0.005)
        self.assertGreater(angle, 0.0)
        self.assertGreater(rate, 0.0)
        self.assertGreater(acceleration, 0.0)
        self.assertTrue(saturated)

    def test_propeller_axes_follow_independent_gimbals(self) -> None:
        params = best_params()
        left, right = integrated_motor_axes(
            params,
            params.nominal_attitude_rad,
            math.radians(30.0),
            math.radians(-20.0),
        )
        self.assertAlmostEqual(left[0], 0.5, places=7)
        self.assertAlmostEqual(left[1], math.cos(math.radians(30.0)), places=7)
        self.assertAlmostEqual(right[0], math.sin(math.radians(-20.0)), places=7)
        self.assertAlmostEqual(math.hypot(*right), 1.0, places=7)

    def test_sensor_estimate_is_reconstructed_not_ground_truth(self) -> None:
        simulator = make_simulator()
        state = simulator.history[-1]
        self.assertFalse(hasattr(state, "measured_left_gimbal_angle"))
        self.assertFalse(hasattr(state, "measured_right_gimbal_angle"))
        estimation_error = math.dist(state.measured_payload, state.payload)
        self.assertGreater(estimation_error, 0.0)
        self.assertLess(estimation_error, 0.01)

    def test_sensor_sample_and_hold_obeys_configured_period(self) -> None:
        simulator = make_simulator()
        self.assertAlmostEqual(simulator.params.sensor_sample_period_s, 2.0 * simulator.params.dt)
        rng_state = simulator._sensor_rng.getstate()
        simulator.step()
        self.assertEqual(simulator._sensor_rng.getstate(), rng_state)
        simulator.step()
        self.assertNotEqual(simulator._sensor_rng.getstate(), rng_state)

    def test_repeated_turn_path_tracks_without_contact_or_slack(self) -> None:
        scenario = default_scenario()
        params = replace(best_params(scenario.facade_mission), wind_enabled=True)
        simulator = make_simulator(params)
        command_controller(simulator, scenario.targets)
        states = []
        while simulator.t < 12.0:
            states.append(simulator.step())

        self.assertFalse(any(state.contact_force != 0.0 for state in states))
        self.assertFalse(any(state.cable_slack for state in states))
        self.assertGreater(states[-1].active_waypoints, 0)
        self.assertLess(max(state.tool_error for state in states), 0.08)
        self.assertLess(max(abs(state.attitude) for state in states), math.radians(8.0))
        self.assertGreater(
            max(abs(state.left_gimbal_angle) for state in states),
            math.radians(5.0),
        )
        self.assertGreater(
            max(
                abs(state.left_gimbal_angle - state.estimated_left_gimbal_angle)
                for state in states
            ),
            math.radians(0.20),
        )
        self.assertTrue(all(state.mpc_status in {"Solve_Succeeded", "Solved_To_Acceptable_Level"} for state in states))


if __name__ == "__main__":
    unittest.main()
