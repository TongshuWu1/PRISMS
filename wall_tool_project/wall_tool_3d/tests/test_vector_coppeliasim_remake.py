from __future__ import annotations

import dataclasses
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


WALL_TOOL_PROJECT_ROOT = Path(__file__).resolve().parents[2]
WALL_TOOL_2D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_2d"
WALL_TOOL_3D_ROOT = WALL_TOOL_PROJECT_ROOT / "wall_tool_3d"
for path in (WALL_TOOL_3D_ROOT, WALL_TOOL_2D_ROOT, WALL_TOOL_PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cable_hybrid_controller.controller import best_params  # noqa: E402
from coppeliasim_vector_tool.contracts import ActuatorCommand, PlantTruth, SensorEstimate  # noqa: E402
from coppeliasim_vector_tool.controller import ExternalVectorThrustController  # noqa: E402
from coppeliasim_vector_tool.plant import local_vector_to_world, steel_cable_midspan_sag_m  # noqa: E402
from coppeliasim_vector_tool.sensors import VectorToolSensorSuite  # noqa: E402
from coppeliasim_vector_tool.validation_plant import (  # noqa: E402
    IndependentCableModel,
    IndependentReelModel,
    IndependentRotorModel,
    IndependentServoModel,
    datasheet_validation_profile,
    load_calibrated_validation_profile,
)
from wall_tool_sim.steel_cable import SteelCableSpec  # noqa: E402


def initial_truth(params, timestamp_s: float = 0.0) -> PlantTruth:
    anchor = (params.anchor[0], -0.20, params.anchor[1])
    mount = (
        params.initial_payload[0],
        -0.20,
        params.initial_payload[1] + params.payload_hex_radius,
    )
    distance = math.dist(anchor, mount)
    tension = params.desired_cable_support_fraction * params.total_mass * params.gravity
    diameter = params.steel_cable_diameter_m
    area = math.pi * (0.5 * diameter) ** 2
    stiffness = 1.0 / (
        distance / (params.steel_cable_youngs_modulus_pa * area)
        + params.steel_cable_structural_compliance_m_N
    )
    return PlantTruth(
        timestamp_s=timestamp_s,
        position_world_m=(params.initial_payload[0], -0.20, params.initial_payload[1]),
        linear_velocity_world_m_s=(123.0, -456.0, 789.0),
        orientation_world_rad=(0.0, 0.0, 0.0),
        angular_velocity_world_rad_s=(0.0, 0.0, 0.0),
        anchor_world_m=anchor,
        cable_mount_world_m=mount,
        cable_mount_velocity_world_m_s=(0.0, 0.0, 0.0),
        reel_length_m=distance - tension / stiffness,
        reel_velocity_m_s=0.0,
        cable_tension_N=tension,
        left_servo_angle_rad=0.0,
        right_servo_angle_rad=0.0,
        left_thrust_N=0.0,
        right_thrust_N=0.0,
    )


class VectorCoppeliaRemakeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.params = best_params()

    def test_plant_command_contract_has_exactly_five_physical_channels(self) -> None:
        fields = {field.name for field in dataclasses.fields(ActuatorCommand)}
        physical = {
            "left_thrust_N",
            "right_thrust_N",
            "reel_velocity_m_s",
            "left_servo_angle_rad",
            "right_servo_angle_rad",
        }
        self.assertTrue(physical.issubset(fields))
        self.assertFalse(any("contact" in name or "guide" in name for name in fields))

    def test_sensor_contract_exposes_no_servo_encoder_or_cartesian_truth(self) -> None:
        fields = {field.name for field in dataclasses.fields(SensorEstimate)}
        self.assertFalse(any("servo" in name or "truth" in name for name in fields))
        self.assertIn("cable_angle_rad", fields)
        self.assertIn("reel_length_m", fields)
        self.assertIn("cable_tension_N", fields)
        self.assertIn("payload_attitude_rad", fields)

    def test_sensor_is_sample_and_hold_and_does_not_use_truth_velocity(self) -> None:
        sensors = VectorToolSensorSuite(self.params)
        estimate0 = sensors.update(initial_truth(self.params, 0.0), force=True)
        held = sensors.update(initial_truth(self.params, 0.005))
        self.assertIs(held, estimate0)
        estimate1 = sensors.update(initial_truth(self.params, 0.010))
        self.assertIsNot(estimate1, estimate0)
        self.assertLess(math.hypot(*estimate1.payload_velocity_xz_m_s), 0.05)

    def test_sensor_reads_reel_encoder_not_true_paid_out_length(self) -> None:
        truth = dataclasses.replace(
            initial_truth(self.params, 0.0),
            reel_encoder_length_m=initial_truth(self.params, 0.0).reel_length_m + 0.020,
            reel_encoder_velocity_m_s=0.012,
        )
        estimate = VectorToolSensorSuite(self.params).update(truth, force=True)
        self.assertGreater(estimate.reel_length_m - truth.reel_length_m, 0.015)
        self.assertAlmostEqual(estimate.reel_velocity_m_s, 0.012, delta=0.003)

    def test_reel_load_cell_reads_drum_side_of_pulley(self) -> None:
        params = dataclasses.replace(self.params, load_cell_noise_std_N=0.0)
        truth = dataclasses.replace(
            initial_truth(params, 0.0),
            cable_tension_N=1.0,
            drum_tension_N=1.8,
        )
        estimate = VectorToolSensorSuite(params).update(truth, force=True)
        self.assertAlmostEqual(estimate.cable_tension_N, 1.8, places=9)

    def test_external_controller_accepts_only_sensor_estimate_and_outputs_bounded_channels(self) -> None:
        sensors = VectorToolSensorSuite(self.params)
        estimate = sensors.update(initial_truth(self.params), force=True)
        controller = ExternalVectorThrustController(self.params)
        command = controller.step(estimate)
        self.assertIsInstance(command, ActuatorCommand)
        self.assertLessEqual(command.left_thrust_N, self.params.max_thrust_per_drone)
        self.assertLessEqual(command.right_thrust_N, self.params.max_thrust_per_drone)
        self.assertLessEqual(abs(command.reel_velocity_m_s), self.params.max_spool_speed)
        self.assertLessEqual(abs(command.left_servo_angle_rad), self.params.gimbal_max_angle_rad)
        self.assertLessEqual(abs(command.right_servo_angle_rad), self.params.gimbal_max_angle_rad)

    def test_vectoring_axis_uses_actual_servo_angle(self) -> None:
        identity = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        angle = math.radians(31.0)
        axis = local_vector_to_world(identity, [math.sin(angle), 0.0, math.cos(angle)])
        self.assertAlmostEqual(axis[0], math.sin(angle))
        self.assertAlmostEqual(axis[1], 0.0)
        self.assertAlmostEqual(axis[2], math.cos(angle))

    def test_steel_cable_stiffness_damping_and_sag_are_physical(self) -> None:
        cable = SteelCableSpec(
            diameter_m=self.params.steel_cable_diameter_m,
            youngs_modulus_pa=self.params.steel_cable_youngs_modulus_pa,
            density_kg_m3=self.params.steel_cable_density_kg_m3,
            structural_compliance_m_N=self.params.steel_cable_structural_compliance_m_N,
            damping_ratio=self.params.steel_cable_damping_ratio,
            payload_weight_fraction=self.params.steel_cable_payload_weight_fraction,
        )
        length = 4.0
        stiffness = cable.axial_stiffness_N_m(length)
        damping = cable.damping_N_s_m(length, self.params.total_mass)
        self.assertGreater(stiffness, 2500.0)
        self.assertLess(stiffness, 4000.0)
        self.assertGreater(damping, 5.0)
        one_newton_extension = 1.0 / stiffness
        self.assertGreater(one_newton_extension, 0.00025)
        self.assertLess(one_newton_extension, 0.00040)
        low_tension_sag = steel_cable_midspan_sag_m(
            cable, length, 0.5, self.params.gravity, 0.6
        )
        working_tension_sag = steel_cable_midspan_sag_m(
            cable, length, 2.0, self.params.gravity, 0.6
        )
        self.assertGreater(low_tension_sag, working_tension_sag)
        self.assertGreater(working_tension_sag, 0.0)
        self.assertLessEqual(low_tension_sag, cable.max_visual_sag_m)

    def test_calibrated_profile_loader_fails_loudly_on_uncalibrated_template(self) -> None:
        profile = datasheet_validation_profile(self.params)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.json"
            path.write_text(json.dumps(profile.to_json_dict()), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "calibrated=true"):
                load_calibrated_validation_profile(path)

            data = profile.to_json_dict()
            data["calibrated"] = True
            data["profile_name"] = "unit-test-identified-profile"
            data["provenance"] = {
                "hardware_id": "unit-test-fixture",
                "recorded_utc": "2026-07-12T00:00:00Z",
                "raw_data_sha256": "a" * 64,
            }
            path.write_text(json.dumps(data), encoding="utf-8")
            loaded = load_calibrated_validation_profile(path)
            self.assertTrue(loaded.calibrated)
            self.assertEqual(loaded.profile_name, "unit-test-identified-profile")

    def test_independent_models_survive_three_minute_mismatch_run(self) -> None:
        """Exercise slow relaxation, drum layering, play, and cable vibration."""

        profile = datasheet_validation_profile(self.params)
        reel = IndependentReelModel(profile.reel, profile.cable.diameter_m)
        cable = IndependentCableModel(profile.cable)
        rotors = IndependentRotorModel(profile.rotor)
        servos = IndependentServoModel(profile.servo)
        initial_distance = 3.0
        initial_tension = 1.2
        extension = profile.cable.extension_for_tension_m(initial_tension, initial_distance)
        reel.initialize(initial_distance - extension)
        cable.initialize(extension)
        rotors.initialize(0.42, 0.42)

        dt = self.params.dt
        max_encoder_mismatch = 0.0
        max_transverse = 0.0
        for index in range(int(180.0 / dt)):
            now = index * dt
            distance = initial_distance + 0.025 * math.sin(2.0 * math.pi * now / 17.0)
            distance_rate = 0.025 * (2.0 * math.pi / 17.0) * math.cos(2.0 * math.pi * now / 17.0)
            reel_command = 0.018 * math.sin(2.0 * math.pi * now / 9.0)
            reel_state = reel.step(
                reel_command,
                initial_tension,
                dt,
                self.params.min_cable_length,
                self.params.max_cable_length,
            )
            response = cable.step(
                distance,
                reel_state.paid_out_length_m,
                distance_rate,
                reel_state.line_velocity_m_s,
                self.params.total_mass,
                self.params.max_spool_tension,
                self.params.cable_taut_band,
                now,
                dt,
            )
            rotor_state = rotors.step(
                0.42 + 0.08 * math.sin(0.7 * now),
                0.42 - 0.08 * math.sin(0.7 * now),
                dt,
            )
            left_servo, right_servo = servos.step(
                math.radians(12.0) * math.sin(0.31 * now),
                -math.radians(10.0) * math.sin(0.27 * now),
                dt,
            )
            values = (
                reel_state.paid_out_length_m,
                reel_state.encoder_length_m,
                response.payload_tension_N,
                response.drum_tension_N,
                response.tangent_slope,
                rotor_state.left_thrust_N,
                rotor_state.right_thrust_N,
                left_servo.angle_rad,
                right_servo.angle_rad,
            )
            self.assertTrue(all(math.isfinite(value) for value in values))
            self.assertGreaterEqual(response.payload_tension_N, 0.0)
            self.assertLessEqual(response.payload_tension_N, self.params.max_spool_tension)
            max_encoder_mismatch = max(
                max_encoder_mismatch,
                abs(reel_state.encoder_length_m - reel_state.paid_out_length_m),
            )
            max_transverse = max(max_transverse, abs(cable.state.transverse_displacement_m))

        self.assertGreater(max_encoder_mismatch, 5e-5)
        self.assertGreater(max_transverse, 1e-7)
        self.assertLessEqual(max_transverse, profile.cable.max_transverse_displacement_m)


if __name__ == "__main__":
    unittest.main()
