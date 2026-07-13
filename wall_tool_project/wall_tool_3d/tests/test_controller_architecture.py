from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


WALL_TOOL_PROJECT_ROOT = Path(__file__).resolve().parents[2]
WALL_TOOL_2D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_2d"
for path in (WALL_TOOL_2D_ROOT, WALL_TOOL_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cable_hybrid_controller.controller import make_simulator  # noqa: E402
from cable_hybrid_controller.mpc.solver import WallToolNMPC  # noqa: E402


class ControllerArchitectureTests(unittest.TestCase):
    def test_nmpc_commands_only_physical_actuator_interfaces(self) -> None:
        self.assertEqual(WallToolNMPC.NU, 5)
        self.assertEqual(WallToolNMPC.NX, 15)
        self.assertEqual(WallToolNMPC.LEFT_THRUST_COMMAND, 0)
        self.assertEqual(WallToolNMPC.RIGHT_THRUST_COMMAND, 1)
        self.assertEqual(WallToolNMPC.REEL_SPEED_COMMAND, 2)
        self.assertEqual(WallToolNMPC.LEFT_GIMBAL_ANGLE_COMMAND, 3)
        self.assertEqual(WallToolNMPC.RIGHT_GIMBAL_ANGLE_COMMAND, 4)

    def test_external_controller_fails_until_tilt_servo_channels_are_connected(self) -> None:
        with self.assertRaisesRegex(NotImplementedError, "tilt-servo channels"):
            make_simulator(external_plant=True)

    def test_no_safety_fallback_is_exposed(self) -> None:
        simulator = make_simulator()
        self.assertFalse(hasattr(simulator, "_safe_fallback_command"))

    def test_sixty_second_closed_loop_run_remains_stable(self) -> None:
        simulator = make_simulator()
        simulator.set_target((0.9, 1.5))
        state = simulator.history[-1]
        max_error = 0.0
        max_tension = 0.0
        slack_samples = 0
        for _ in range(int(60.0 / simulator.params.dt)):
            state = simulator.step()
            max_error = max(max_error, state.tool_error)
            max_tension = max(max_tension, state.tension)
            slack_samples += int(state.cable_slack)
        self.assertTrue(all(math.isfinite(value) for value in (*state.payload, *state.payload_velocity)))
        self.assertIn(state.mpc_status, {"Solve_Succeeded", "Solved_To_Acceptable_Level"})
        self.assertLess(state.tool_error, 0.01)
        self.assertLess(max_error, 0.15)
        self.assertLess(max_tension, simulator.params.max_spool_tension)
        self.assertEqual(slack_samples, 0)
        self.assertFalse(simulator.params.payload_pitch_constrained)
        self.assertLess(abs(state.attitude), simulator.params.inspection_attitude_limit_rad)
        self.assertTrue(math.isfinite(state.angular_velocity))
        self.assertLessEqual(max(state.left_thrust, state.right_thrust), simulator.params.max_thrust_per_drone)
        self.assertLessEqual(abs(state.spool_velocity_cmd), simulator.params.max_spool_speed)
        self.assertLessEqual(abs(state.left_gimbal_angle), simulator.params.gimbal_max_angle_rad)
        self.assertLessEqual(abs(state.right_gimbal_angle), simulator.params.gimbal_max_angle_rad)

    def test_fifty_second_multi_corner_path_uses_preview_without_large_turn_error(self) -> None:
        simulator = make_simulator()
        simulator.set_corner_smooth_path(
            ((0.8, 2.0), (0.8, 1.3), (-0.8, 1.3), (-0.8, 2.0), (0.0, 2.0)),
            corner_speed=0.04,
        )
        squared_error_sum = 0.0
        max_error = 0.0
        slack_samples = 0
        speeds = []
        jerks = []
        previous_acceleration = None
        reference_speeds = []
        previous_reference = simulator.history[-1].reference
        step_count = int(50.0 / simulator.params.dt)
        state = simulator.history[-1]
        for _ in range(step_count):
            state = simulator.step()
            squared_error_sum += state.tool_error**2
            max_error = max(max_error, state.tool_error)
            slack_samples += int(state.cable_slack)
            speeds.append(math.hypot(*state.payload_velocity))
            if previous_acceleration is not None:
                jerks.append(math.dist(previous_acceleration, state.payload_acceleration) / simulator.params.dt)
            previous_acceleration = state.payload_acceleration
            reference_speeds.append(
                math.dist(previous_reference, state.reference) / simulator.params.dt
            )
            previous_reference = state.reference
        rms_error = math.sqrt(squared_error_sum / step_count)
        p95_jerk = sorted(jerks)[int(0.95 * (len(jerks) - 1))]
        self.assertLess(rms_error, 0.035)
        self.assertLess(max_error, 0.090)
        self.assertLess(state.tool_error, 0.035)
        self.assertLess(max(speeds), 0.30)
        self.assertLessEqual(max(reference_speeds), 1.01 * simulator.params.path_speed)
        self.assertLess(p95_jerk, 9.0)
        self.assertLess(max(jerks), 16.0)
        self.assertEqual(slack_samples, 0)
        self.assertLess(abs(state.attitude), simulator.params.inspection_attitude_limit_rad)


if __name__ == "__main__":
    unittest.main()
