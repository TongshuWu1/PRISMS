#!/usr/bin/env python3
"""Interactive wall-plane PRISMS facade-inspection simulator.

The model is intentionally small, but it is not just a drawing. The suspended
system is integrated as a Cartesian point mass under gravity, finite cable
tension, and two propellers independently rotated by 270-degree position
servos. The controller is a nonlinear MPC with compliant unilateral cable,
reel, motor, and second-order tilt-servo dynamics. Normal contact is
deliberately outside the inspection scope.
Click any point on the wall to command a smooth straight-line move, or enable
append mode to queue a smooth multi-waypoint trajectory.
"""

from __future__ import annotations

import argparse
import bisect
import itertools
import math
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

try:
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle
    from matplotlib.widgets import Button, Slider
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing matplotlib. Install project requirements first:\n"
        "  python -m pip install -r requirements.txt"
    ) from exc

from cable_hybrid_controller.mpc import MPCConfig, MPCReferenceHorizon, MPCSolution, WallToolNMPC
from wall_tool_sim.gimbal_servo import GimbalServoSpec
from wall_tool_sim.reel_motor import ReelMotorSpec
from wall_tool_sim.steel_cable import SteelCableSpec


Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]

DEFAULT_GRAVITY = 9.80665
DEFAULT_REEL_MOTOR = ReelMotorSpec()
DEFAULT_GIMBAL_SERVO = GimbalServoSpec()
PLANNER_DIRECT = "direct"
PLANNER_CENTER_SETUP = "center-setup"
PLANNER_PREDICTIVE = "predictive"
PLANNER_CHOICES = (PLANNER_DIRECT, PLANNER_CENTER_SETUP, PLANNER_PREDICTIVE)

SQRT5 = math.sqrt(5.0)
CAGE_ROT_Y_RAD = math.pi / 4.0
DRONE_RIGHT_HEX = (1, 1, 1)
DRONE_LEFT_HEX = (-1, -1, -1)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def slew_toward(value: float, target: float, max_rate: float, dt: float) -> float:
    """Advance a command toward its target without exceeding a physical rate."""

    if not all(math.isfinite(item) for item in (value, target, max_rate, dt)):
        raise ValueError("command slew inputs must be finite")
    if max_rate <= 0.0 or dt <= 0.0:
        raise ValueError("command slew rate and time step must be positive")
    maximum_step = max_rate * dt
    return value + clamp(target - value, -maximum_step, maximum_step)


def dot2(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1])


def dot3(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def sub3(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2])


def cross3(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    )


def norm3(a: Sequence[float]) -> float:
    return math.sqrt(dot3(a, a))


def normalize3(a: Sequence[float]) -> Vec3:
    length = norm3(a)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return float(a[0]) / length, float(a[1]) / length, float(a[2]) / length


def add2(a: Sequence[float], b: Sequence[float]) -> Vec2:
    return float(a[0]) + float(b[0]), float(a[1]) + float(b[1])


def sub2(a: Sequence[float], b: Sequence[float]) -> Vec2:
    return float(a[0]) - float(b[0]), float(a[1]) - float(b[1])


def scale2(vector: Sequence[float], gain: float) -> Vec2:
    return float(vector[0]) * gain, float(vector[1]) * gain


def rotate2(vector: Sequence[float], angle: float) -> Vec2:
    c = math.cos(angle)
    s = math.sin(angle)
    return c * float(vector[0]) - s * float(vector[1]), s * float(vector[0]) + c * float(vector[1])


def cross2(moment_arm: Sequence[float], force: Sequence[float]) -> float:
    return float(moment_arm[0]) * float(force[1]) - float(moment_arm[1]) * float(force[0])


def normalize2(vector: Sequence[float]) -> Vec2:
    length = math.hypot(float(vector[0]), float(vector[1]))
    if length < 1e-12:
        return (0.0, 0.0)
    return float(vector[0]) / length, float(vector[1]) / length


def limit_norm2(vector: Sequence[float], max_norm: float) -> Vec2:
    length = math.hypot(float(vector[0]), float(vector[1]))
    if length <= max_norm or length < 1e-12:
        return float(vector[0]), float(vector[1])
    scale = max_norm / length
    return float(vector[0]) * scale, float(vector[1]) * scale


def distance2(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Face:
    kind: str
    normal_key: tuple[int, int, int]
    normal: Vec3
    indices: tuple[int, ...]
    center: Vec3


@dataclass(frozen=True)
class TruncatedOctahedronGeometry:
    vertices: tuple[Vec3, ...]
    faces: tuple[Face, ...]
    edges: tuple[tuple[int, int], ...]
    face_by_normal: dict[tuple[int, int, int], Face]


def ordered_face_indices(vertices: Sequence[Vec3], indices: Iterable[int], normal: Vec3) -> tuple[int, ...]:
    face_indices = list(indices)
    center = (
        sum(vertices[index][0] for index in face_indices) / len(face_indices),
        sum(vertices[index][1] for index in face_indices) / len(face_indices),
        sum(vertices[index][2] for index in face_indices) / len(face_indices),
    )
    n = normalize3(normal)
    reference = (0.0, 0.0, 1.0)
    if abs(dot3(n, reference)) > 0.90:
        reference = (0.0, 1.0, 0.0)
    u = normalize3(cross3(n, reference))
    v = normalize3(cross3(n, u))

    def angle(index: int) -> float:
        relative = sub3(vertices[index], center)
        return math.atan2(dot3(relative, v), dot3(relative, u))

    return tuple(sorted(face_indices, key=angle))


def build_truncated_octahedron() -> TruncatedOctahedronGeometry:
    vertices = tuple(
        sorted(
            set(
                itertools.chain.from_iterable(
                    itertools.permutations((0.0, one, two))
                    for one in (-1.0, 1.0)
                    for two in (-2.0, 2.0)
                )
            )
        )
    )
    faces: list[Face] = []

    for axis in range(3):
        for sign in (-1, 1):
            indices = [index for index, point in enumerate(vertices) if point[axis] == 2.0 * sign]
            normal_key = [0, 0, 0]
            normal_key[axis] = sign
            ordered = ordered_face_indices(vertices, indices, tuple(float(value) for value in normal_key))
            center = tuple(sum(vertices[index][dim] for index in ordered) / len(ordered) for dim in range(3))
            faces.append(
                Face(
                    kind="square",
                    normal_key=tuple(normal_key),
                    normal=normalize3(normal_key),
                    indices=ordered,
                    center=center,  # type: ignore[arg-type]
                )
            )

    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                indices = [
                    index
                    for index, point in enumerate(vertices)
                    if sx * point[0] + sy * point[1] + sz * point[2] == 3.0
                ]
                normal_key = (sx, sy, sz)
                ordered = ordered_face_indices(vertices, indices, normal_key)
                center = tuple(sum(vertices[index][dim] for index in ordered) / len(ordered) for dim in range(3))
                faces.append(
                    Face(
                        kind="hex",
                        normal_key=normal_key,
                        normal=normalize3(normal_key),
                        indices=ordered,
                        center=center,  # type: ignore[arg-type]
                    )
                )

    edge_set: set[tuple[int, int]] = set()
    for face in faces:
        for index, vertex_index in enumerate(face.indices):
            next_index = face.indices[(index + 1) % len(face.indices)]
            edge_set.add(tuple(sorted((vertex_index, next_index))))

    return TruncatedOctahedronGeometry(
        vertices=vertices,
        faces=tuple(faces),
        edges=tuple(sorted(edge_set)),
        face_by_normal={face.normal_key: face for face in faces},
    )


GEOMETRY = build_truncated_octahedron()


def rotate_cage(point: Vec3) -> Vec3:
    c = math.cos(CAGE_ROT_Y_RAD)
    s = math.sin(CAGE_ROT_Y_RAD)
    x, y, z = point
    return c * x + s * z, y, -s * x + c * z


def project_local(point: Vec3, radius: float) -> Vec2:
    scale = radius / SQRT5
    rotated = rotate_cage(point)
    return rotated[0] * scale, rotated[2] * scale


def projected_face_offset(radius: float, normal_key: tuple[int, int, int], attitude: float = 0.0) -> Vec2:
    return rotate2(project_local(GEOMETRY.face_by_normal[normal_key].center, radius), attitude)


def oriented_box_polygon(center: Vec2, half_length: float, half_width: float, angle: float) -> list[Vec2]:
    corners = (
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
    )
    return [add2(center, rotate2(corner, angle)) for corner in corners]


def integrated_motor_center_offsets(params: "SimParams", attitude: float) -> tuple[Vec2, Vec2]:
    """Actuator locations for the integrated side-motor body.

    The centers intentionally match the previous two-module moment arms so
    the simplified geometry keeps the active controller physics unchanged.
    """
    radius = params.cage_radius
    left_payload_offset = rotate2((-params.payload_half_length, 0.0), attitude)
    left_motor_offset = projected_face_offset(radius, DRONE_RIGHT_HEX, attitude)
    right_payload_offset = rotate2((params.payload_half_length, 0.0), attitude)
    right_motor_offset = projected_face_offset(radius, DRONE_LEFT_HEX, attitude)
    left_gap = rotate2((-params.module_gap, 0.0), attitude)
    right_gap = rotate2((params.module_gap, 0.0), attitude)
    left_center_offset = sub2(add2(left_payload_offset, left_gap), left_motor_offset)
    right_center_offset = sub2(add2(right_payload_offset, right_gap), right_motor_offset)
    return left_center_offset, right_center_offset


def integrated_motor_centers(params: "SimParams", payload: Vec2, attitude: float) -> tuple[Vec2, Vec2]:
    left_offset, right_offset = integrated_motor_center_offsets(params, attitude)
    return add2(payload, left_offset), add2(payload, right_offset)


def integrated_motor_axes(
    params: "SimParams",
    attitude: float,
    left_gimbal_angle: float,
    right_gimbal_angle: float,
) -> tuple[Vec2, Vec2]:
    """Return world-frame thrust axes for wall-normal gimbal hinges.

    A local gimbal angle of zero points along payload +z. Positive angle
    produces +x thrust. The body attitude is included because each gimbal is
    mounted to the payload rather than inertially stabilized.
    """

    attitude_error = attitude - params.nominal_attitude_rad
    left_world_angle = attitude_error + float(left_gimbal_angle)
    right_world_angle = attitude_error + float(right_gimbal_angle)
    return (
        (math.sin(left_world_angle), math.cos(left_world_angle)),
        (math.sin(right_world_angle), math.cos(right_world_angle)),
    )


@dataclass(frozen=True)
class SimParams:
    wall_width: float = 6.0
    wall_height: float = 6.0
    dt: float = 0.005
    drone_mass: float = 0.050
    payload_tool_mass: float = 0.075
    gravity: float = DEFAULT_GRAVITY
    cage_radius: float = 0.18
    payload_half_length: float = 0.190
    payload_hex_radius: float = 0.114
    module_gap: float = 0.0
    # A passive Y-bridle supplies finite restoring stiffness and damping. An
    # ideal pitch constraint remains available only for isolated plant tests.
    payload_pitch_constrained: bool = False
    passive_attitude_stiffness_Nm_rad: float = 0.20
    passive_attitude_damping_Nm_s_rad: float = 0.055
    max_thrust_per_drone: float = 0.150 * DEFAULT_GRAVITY
    mpc_thrust_command_fraction: float = 0.92

    # Independent 270-degree propeller tilt servos; axes are normal to the wall.
    gimbal_max_angle_rad: float = DEFAULT_GIMBAL_SERVO.max_angle_rad
    gimbal_max_rate_rad_s: float = DEFAULT_GIMBAL_SERVO.max_rate_rad_s
    gimbal_max_acceleration_rad_s2: float = DEFAULT_GIMBAL_SERVO.max_acceleration_rad_s2
    gimbal_command_slew_limit_rad_s: float = math.radians(90.0)
    gimbal_natural_frequency_rad_s: float = DEFAULT_GIMBAL_SERVO.natural_frequency_rad_s
    gimbal_damping_ratio: float = DEFAULT_GIMBAL_SERVO.damping_ratio
    gimbal_command_min_pulse_us: float = DEFAULT_GIMBAL_SERVO.command_min_pulse_us
    gimbal_command_max_pulse_us: float = DEFAULT_GIMBAL_SERVO.command_max_pulse_us
    gimbal_command_resolution_us: float = DEFAULT_GIMBAL_SERVO.command_resolution_us
    gimbal_left_zero_error_rad: float = math.radians(0.35)
    gimbal_right_zero_error_rad: float = math.radians(-0.25)

    # The default hardware-facing controller is a cascaded sensor controller.
    # The experimental NMPC remains selectable for offline comparison.
    control_law: str = "vector_thrust_nmpc"

    # Outer-loop position feedback and bounded force allocation. Attitude
    # feedback is retained for the optional free-pitch model.
    cascade_position_kp: float = 2.5
    cascade_velocity_kd: float = 3.2
    cascade_acceleration_limit_m_s2: float = 0.8
    cascade_attitude_kp: float = 12.0
    cascade_attitude_kd: float = 4.0
    cascade_torque_weight: float = 1.0
    cascade_nominal_tension_N: float = 0.45
    cascade_min_tension_N: float = 0.35
    cascade_max_tension_N: float = 1.20
    cascade_tension_regularization: float = 0.002
    max_control_spool_speed: float = 0.10

    # NMPC horizon, solver, constraints, and objective weights.
    mpc_horizon_steps: int = 18
    mpc_horizon_dt: float = 0.060
    mpc_control_period_s: float = 0.040
    mpc_attitude_limit_rad: float = 1.05
    mpc_slack_limit_m: float = 0.012
    mpc_tracking_position_weight: float = 260.0
    mpc_tracking_velocity_weight: float = 22.0
    mpc_terminal_position_weight: float = 520.0
    mpc_terminal_velocity_weight: float = 36.0
    mpc_drone_effort_weight: float = 0.42
    mpc_cable_effort_weight: float = 0.018
    mpc_reel_speed_weight: float = 0.050
    mpc_input_rate_weight: float = 0.030
    mpc_attitude_rate_weight: float = 0.45
    mpc_attitude_weight: float = 0.025
    mpc_gimbal_angle_weight: float = 0.018
    mpc_gimbal_rate_weight: float = 0.080
    mpc_cable_support_weight: float = 10.0
    mpc_slack_weight: float = 180.0
    mpc_hold_integral_gain_s_inv: float = 0.8
    mpc_hold_integral_limit_m_s: float = 0.060
    mpc_solver_max_iter: int = 90
    mpc_solver_tolerance: float = 1e-5
    mpc_energy_plot_limit_J: float = 0.015

    # Cable and reel limits used by the active compliant unilateral cable plant.
    max_cable_support_fraction: float = 1.0
    desired_cable_support_fraction: float = 0.75
    max_spool_speed: float = DEFAULT_REEL_MOTOR.max_line_speed_m_s
    # Internal NMPC velocity-command slew bound, derived from reel motor response.
    # The physical actuator command remains reel line velocity.
    reel_velocity_slew_limit_mps2: float = (
        DEFAULT_REEL_MOTOR.max_line_speed_m_s / DEFAULT_REEL_MOTOR.velocity_time_constant_s
    )
    reel_motor_voltage_v: float = DEFAULT_REEL_MOTOR.voltage_v
    reel_motor_gear_ratio: float = DEFAULT_REEL_MOTOR.gear_ratio
    reel_motor_no_load_rpm: float = DEFAULT_REEL_MOTOR.no_load_output_rpm
    reel_motor_stall_torque_kg_cm: float = DEFAULT_REEL_MOTOR.stall_torque_kg_cm
    reel_spool_radius_m: float = DEFAULT_REEL_MOTOR.spool_radius_m
    reel_velocity_time_constant_s: float = DEFAULT_REEL_MOTOR.velocity_time_constant_s
    motor_thrust_time_constant_s: float = 0.060
    thrust_command_slew_limit_N_s: float = 2.0
    reel_continuous_torque_fraction: float = DEFAULT_REEL_MOTOR.continuous_torque_fraction
    cable_taut_band: float = 0.006
    cable_stiffness_N_m: float = 750.0
    cable_damping_N_s_m: float = 1.2
    cable_tension_time_constant_s: float = 0.030
    steel_cable_diameter_m: float = 0.0012
    steel_cable_youngs_modulus_pa: float = 200.0e9
    steel_cable_density_kg_m3: float = 7850.0
    steel_cable_structural_compliance_m_N: float = 3.0e-4
    steel_cable_damping_ratio: float = 0.22
    steel_cable_payload_weight_fraction: float = 0.50
    max_spool_tension: float = DEFAULT_REEL_MOTOR.continuous_line_force_N
    min_tracking_tension: float = 0.10
    reel_tension_kp_mps_N: float = 0.055
    reel_tension_ki_mps_Ns: float = 0.010
    reel_tension_integral_limit_Ns: float = 5.0
    load_cell_filter_tau_s: float = 0.018
    cable_tension_rate_limit_N_s: float = 80.0
    max_cable_extension_m: float = 0.009
    min_cable_vertical_efficiency: float = 0.08
    min_control_cable_length: float = 0.62

    # Explicit sensor sample rates, quantization, and noise. Ground-truth
    # payload position/velocity is never passed to the internal controller.
    sensor_random_seed: int = 2804
    sensor_sample_period_s: float = 0.010
    cable_angle_encoder_counts_per_rev: int = 16384
    reel_encoder_counts_per_rev: int = 4096
    cable_angle_noise_std_rad: float = 0.00035
    reel_length_noise_std_m: float = 0.00005
    load_cell_noise_std_N: float = 0.008
    imu_angle_noise_std_rad: float = 0.0025
    imu_rate_noise_std_rad_s: float = 0.010
    velocity_estimator_time_constant_s: float = 0.045

    # Planner cost scales. These are route-selection terms, not a controller.
    max_tangential_accel: float = 2.8

    # Payload attitude dynamics. Attitude is chosen by the NMPC optimizer.
    rotational_damping: float = 0.090
    nominal_attitude_rad: float = 0.0

    # Desired path generation.
    path_speed: float = 0.16
    reference_accel_limit_mps2: float = 0.24
    reference_jerk_limit_mps3: float = 1.2
    reference_min_segment_duration_s: float = 0.90
    reference_preview_time_s: float = 1.2
    reference_preview_min_distance_m: float = 0.18
    reference_turn_lateral_accel_m_s2: float = 0.06
    waypoint_tolerance: float = 0.012

    # Geometry and disturbances.
    min_cable_length: float = 0.10
    max_cable_length: float = 7.0
    initial_payload: Vec2 = (0.0, 2.00)
    wind_enabled: bool = False
    wind_force_x: float = 0.0
    wind_force_z: float = 0.0
    wind_gust_force: float = 0.0
    wind_gust_period_s: float = 11.0
    wind_gust_vertical_fraction: float = 0.35
    edge_wind_gain: float = 0.0

    # Normal-to-wall contact and work-quality checks. The planar controller still
    # tracks the desired x-z path; this state models facade standoff/force.
    normal_contact_enabled: bool = False
    normal_standoff_m: float = 0.10
    normal_initial_gap_m: float = 0.10
    normal_gap_min_m: float = -0.030
    normal_gap_max_m: float = 0.250
    normal_position_kp: float = 70.0
    normal_position_kd: float = 8.0
    normal_air_damping: float = 0.16
    normal_push_force_limit_N: float = 1.35
    normal_retract_force_limit_N: float = 0.85
    normal_contact_stiffness_N_m: float = 160.0
    normal_contact_damping_N_s_m: float = 1.8
    normal_contact_force_limit_N: float = 2.2
    desired_contact_force_N: float = 0.55
    min_contact_force_N: float = 0.25
    max_contact_force_N: float = 0.95
    contact_work_enabled: bool = False
    contact_work_x_min: float = -2.10
    contact_work_x_max: float = 2.10
    contact_work_z_min: float = 1.10
    contact_work_z_max: float = 5.25
    contact_work_margin_m: float = 0.04
    work_contact_speed_limit_mps: float = 0.36
    work_contact_tracking_limit_m: float = 0.12
    work_contact_angular_rate_limit_rad_s: float = 1.5
    inspection_attitude_limit_rad: float = math.radians(8.0)
    normal_wind_force_N: float = 0.0
    normal_wind_gust_force_N: float = 0.0
    normal_wind_gust_period_s: float = 9.5

    def __post_init__(self) -> None:
        if self.control_law != "vector_thrust_nmpc":
            raise ValueError(
                "wall_tool_2d requires control_law='vector_thrust_nmpc'; "
                "no backup controller is configured"
            )
        positive_fields = {
            "physics dt": self.dt,
            "total mass": self.total_mass,
            "gravity": self.gravity,
            "maximum thrust": self.max_thrust_per_drone,
            "gimbal maximum angle": self.gimbal_max_angle_rad,
            "gimbal maximum rate": self.gimbal_max_rate_rad_s,
            "gimbal maximum acceleration": self.gimbal_max_acceleration_rad_s2,
            "gimbal command slew limit": self.gimbal_command_slew_limit_rad_s,
            "gimbal natural frequency": self.gimbal_natural_frequency_rad_s,
            "gimbal damping ratio": self.gimbal_damping_ratio,
            "thrust command slew limit": self.thrust_command_slew_limit_N_s,
            "hold integral gain": self.mpc_hold_integral_gain_s_inv,
            "hold integral limit": self.mpc_hold_integral_limit_m_s,
            "sensor sample period": self.sensor_sample_period_s,
            "velocity estimator time constant": self.velocity_estimator_time_constant_s,
            "minimum tracking tension": self.min_tracking_tension,
            "maximum spool tension": self.max_spool_tension,
        }
        for name, value in positive_fields.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.sensor_sample_period_s < self.dt:
            raise ValueError("sensor sample period cannot be shorter than the physics step")
        if self.cable_angle_encoder_counts_per_rev < 4:
            raise ValueError("cable-angle encoder resolution is invalid")
        if self.reel_encoder_counts_per_rev < 4:
            raise ValueError("reel encoder resolution is invalid")
        if self.gimbal_command_max_pulse_us <= self.gimbal_command_min_pulse_us:
            raise ValueError("tilt-servo command pulse range is invalid")
        if self.gimbal_command_resolution_us <= 0.0:
            raise ValueError("tilt-servo command pulse resolution is invalid")
        if not math.isfinite(self.gimbal_left_zero_error_rad):
            raise ValueError("left tilt-servo zero error must be finite")
        if not math.isfinite(self.gimbal_right_zero_error_rad):
            raise ValueError("right tilt-servo zero error must be finite")
        if not 0.0 < self.desired_cable_support_fraction <= self.max_cable_support_fraction <= 1.0:
            raise ValueError(
                "cable support fractions must satisfy 0 < desired <= maximum <= 1"
            )
        if self.normal_contact_enabled or self.contact_work_enabled:
            raise ValueError(
                "the inspection controller does not model or command wall-normal contact force"
            )

    @property
    def anchor(self) -> Vec2:
        return (0.0, self.wall_height)

    @property
    def total_mass(self) -> float:
        return self.payload_tool_mass + 2.0 * self.drone_mass

    @property
    def hex_face_tilt_rad(self) -> float:
        return math.atan2(1.0, math.sqrt(2.0))

    @property
    def assembly_inertia(self) -> float:
        radius = self.cage_radius
        payload_width = 2.0 * self.payload_half_length
        payload_height = 2.0 * self.payload_hex_radius
        payload_shape_inertia = self.payload_tool_mass * (payload_width * payload_width + payload_height * payload_height) / 12.0
        left_payload_offset = (-self.payload_half_length, 0.0)
        left_drone_offset = projected_face_offset(radius, DRONE_RIGHT_HEX)
        right_payload_offset = (self.payload_half_length, 0.0)
        right_drone_offset = projected_face_offset(radius, DRONE_LEFT_HEX)
        left_distance = distance2(left_payload_offset, left_drone_offset)
        right_distance = distance2(right_payload_offset, right_drone_offset)
        drone_shape_inertia = 0.25 * self.drone_mass * radius * radius
        return (
            payload_shape_inertia
            + drone_shape_inertia
            + self.drone_mass * left_distance * left_distance
            + drone_shape_inertia
            + self.drone_mass * right_distance * right_distance
        )


def clamp_wall_point_for_params(point: Vec2, params: SimParams) -> Vec2:
    margin = max(params.cage_radius, params.payload_half_length, params.payload_hex_radius) * 1.4
    return (
        clamp(point[0], -params.wall_width / 2.0 + margin, params.wall_width / 2.0 - margin),
        clamp(point[1], margin, params.wall_height - margin),
    )


def center_setup_waypoint(target: Vec2, params: SimParams) -> Vec2 | None:
    if target[1] < 2.65 or abs(target[0]) < 0.75:
        return None
    if target[1] > 3.20 and abs(target[0]) > 1.20:
        return clamp_wall_point_for_params((0.0, 1.70), params)
    setup_z = max(1.25, min(2.35, target[1] - 1.05))
    setup_x = 0.25 * target[0]
    return clamp_wall_point_for_params((setup_x, setup_z), params)


def snap_wall_point(point: Vec2) -> Vec2:
    return (round(float(point[0]), 5), round(float(point[1]), 5))


def cable_geometry_proxy(point: Vec2, params: SimParams) -> tuple[float, float, float]:
    dx = float(point[0]) - params.anchor[0]
    dz = params.anchor[1] - float(point[1])
    length = max(1e-9, math.hypot(dx, dz))
    theta = math.atan2(dx, dz)
    vertical_efficiency = dz / length
    return length, theta, vertical_efficiency


def route_length(points: Sequence[Vec2], start: Vec2) -> float:
    total = 0.0
    current = start
    for point in points:
        total += distance2(current, point)
        current = point
    return total


def predictive_setup_candidates(start: Vec2, target: Vec2, params: SimParams) -> tuple[Vec2, ...]:
    candidates: list[Vec2] = []

    def add(point: Vec2 | None) -> None:
        if point is None:
            return
        candidate = snap_wall_point(clamp_wall_point_for_params(point, params))
        if candidate == snap_wall_point(target) or candidate in candidates:
            return
        candidates.append(candidate)

    add(center_setup_waypoint(target, params))
    sign = 1.0 if target[0] >= 0.0 else -1.0
    setup_z_values = (
        1.45,
        1.70,
        1.85,
        2.05,
        max(1.30, min(2.35, target[1] - 1.25)),
        max(1.30, min(2.45, target[1] - 0.95)),
    )
    setup_x_values = (
        0.0,
        0.15 * target[0],
        0.25 * target[0],
        0.40 * target[0],
        sign * 0.22,
    )
    direct = max(distance2(start, target), 1e-6)
    for z in setup_z_values:
        for x in setup_x_values:
            candidate = snap_wall_point(clamp_wall_point_for_params((x, z), params))
            if route_length((candidate, target), start) / direct <= 2.35:
                add(candidate)
    return tuple(candidates)


def predictive_route_cost(start: Vec2, route: Sequence[Vec2], params: SimParams) -> float:
    if not route:
        return math.inf
    target = route[-1]
    direct_length = max(distance2(start, target), 1e-6)
    total_length = route_length(route, start)
    target_length, _target_theta, target_efficiency = cable_geometry_proxy(target, params)
    hard_side_target = target[1] > 2.65 and abs(target[0]) > 0.75
    very_hard_target = target[1] > 3.20 and abs(target[0]) > 1.20

    cost = 0.40 * total_length / direct_length
    current = start
    current_length, current_theta, _current_efficiency = cable_geometry_proxy(current, params)
    for point in route:
        length, theta, efficiency = cable_geometry_proxy(point, params)
        segment_distance = max(distance2(current, point), 1e-6)
        segment_time = max(0.45, segment_distance / max(params.path_speed, 1e-6))
        theta_step = abs(wrap_angle(theta - current_theta))
        length_step = abs(length - current_length)
        angular_accel_proxy = length * theta_step / max(segment_time * segment_time, 1e-6)
        radial_speed_proxy = length_step / segment_time
        shallow_penalty = max(0.0, 0.48 - efficiency)
        cost += (
            0.70 * theta_step * theta_step
            + 0.16 * length_step
            + 4.50 * shallow_penalty * shallow_penalty
            + 0.20 * max(0.0, angular_accel_proxy / max(params.max_tangential_accel, 1e-6) - 1.0) ** 2
            + 0.18 * max(0.0, radial_speed_proxy / max(params.max_spool_speed, 1e-6) - 1.0) ** 2
        )
        if efficiency < params.min_cable_vertical_efficiency or length < params.min_control_cable_length:
            cost += 50.0
        current = point
        current_length = length
        current_theta = theta

    if hard_side_target and len(route) == 1:
        cost += 0.55 + 1.50 * max(0.0, 0.45 - target_efficiency)
        if very_hard_target:
            cost += 1.25 + 0.60 * abs(start[0]) + 0.40 * max(0.0, start[1] - 2.10)
    if hard_side_target and len(route) > 1:
        setup = route[0] if len(route) > 1 else start
        setup_length, _setup_theta, setup_efficiency = cable_geometry_proxy(setup, params)
        preferred_setup_x = 0.0 if very_hard_target else 0.25 * target[0]
        preferred_setup_z = 1.70 if very_hard_target else max(1.55, min(1.85, target[1] - 1.05))
        setup_center_error = abs(setup[0] - preferred_setup_x)
        setup_height_error = setup[1] - preferred_setup_z
        setup_target_clearance = max(0.0, setup[1] - (target[1] - 1.15))
        cost += (
            1.80 * setup_center_error
            + 8.00 * setup_height_error * setup_height_error
            + 2.50 * setup_target_clearance * setup_target_clearance
            + 0.90 * max(0.0, 0.88 - setup_efficiency) ** 2
            + 0.03 * setup_length
        )
        if very_hard_target and setup[1] > 1.90:
            cost += 20.0 * (setup[1] - 1.90) ** 2
    return cost


def predictive_waypoints(start: Vec2, target: Vec2, params: SimParams) -> tuple[Vec2, ...]:
    target = snap_wall_point(clamp_wall_point_for_params(target, params))
    if target[1] <= 2.65 or abs(target[0]) <= 0.75:
        return (target,)
    routes: list[tuple[Vec2, ...]] = [(target,)]
    for candidate in predictive_setup_candidates(start, target, params):
        route = (candidate, target)
        if route not in routes:
            routes.append(route)
    return min(routes, key=lambda route: predictive_route_cost(start, route, params))


@dataclass
class SimState:
    t: float
    theta: float
    theta_dot: float
    length: float
    length_dot: float
    length_ddot: float
    attitude: float
    angular_velocity: float
    angular_acceleration: float
    cable_length: float
    cable_stretch: float
    cable_slack: bool
    cable_tension_saturated: bool
    payload_velocity: Vec2
    payload_acceleration: Vec2
    payload: Vec2
    measured_payload: Vec2
    estimated_payload_velocity: Vec2
    measured_theta: float
    measured_theta_dot: float
    measured_line_length: float
    measured_attitude: float
    measured_angular_velocity: float
    measured_cable_velocity: float
    tool_head: Vec2
    reference: Vec2
    desired_tool_head: Vec2
    reference_velocity: Vec2
    reference_acceleration: Vec2
    target: Vec2
    active_target: Vec2
    measured_tool_error: float
    spool_velocity_cmd: float
    drone_accel_cmd: float
    desired_cable_tension: float
    measured_cable_length: float
    measured_tension: float
    desired_drone_force: Vec2
    drone_force: Vec2
    cable_force: Vec2
    wind_force: Vec2
    normal_gap: float
    normal_velocity: float
    normal_acceleration: float
    normal_actuator_force: float
    normal_wind_force: float
    contact_force: float
    desired_contact_force: float
    contact_valid: bool
    inspection_valid: bool
    work_mode: bool
    desired_attitude_torque: float
    attitude_torque: float
    cable_torque: float
    left_torque: float
    right_torque: float
    left_thrust: float
    right_thrust: float
    left_gimbal_angle: float
    right_gimbal_angle: float
    left_gimbal_rate: float
    right_gimbal_rate: float
    left_gimbal_angle_command: float
    right_gimbal_angle_command: float
    estimated_left_gimbal_angle: float
    estimated_right_gimbal_angle: float
    estimated_left_gimbal_rate: float
    estimated_right_gimbal_rate: float
    tension: float
    tangential_force: float
    desired_tangential_force: float
    allocation_residual: float
    drone_vertical_force: float
    cable_vertical_force: float
    path_error: float
    tool_error: float
    active_waypoints: int
    saturated: bool
    radial_position_error_m: float
    radial_velocity_error_m_s: float
    tangential_position_error_m: float
    tangential_velocity_error_m_s: float
    swing_energy_J: float
    swing_power_W: float
    clf_margin_W: float
    clf_projected_accel_m_s2: float
    mpc_predicted_path: tuple[Vec2, ...] = ()
    mpc_predicted_attitudes: tuple[float, ...] = ()
    mpc_predicted_left_gimbal_angles: tuple[float, ...] = ()
    mpc_predicted_right_gimbal_angles: tuple[float, ...] = ()
    mpc_predicted_tensions: tuple[float, ...] = ()
    mpc_predicted_spool_speeds: tuple[float, ...] = ()
    mpc_status: str = ""
    mpc_solve_time_s: float = 0.0
    mpc_objective: float = 0.0


@dataclass(frozen=True)
class ReferenceState:
    position: Vec2
    velocity: Vec2
    acceleration: Vec2
    final_target: Vec2
    active_target: Vec2
    active: bool
    waypoint_count: int


def solve3(matrix: Sequence[Sequence[float]], rhs: Sequence[float]) -> tuple[float, float, float]:
    rows = [[float(matrix[row][col]) for col in range(3)] + [float(rhs[row])] for row in range(3)]
    for pivot in range(3):
        best = max(range(pivot, 3), key=lambda row: abs(rows[row][pivot]))
        rows[pivot], rows[best] = rows[best], rows[pivot]
        pivot_value = rows[pivot][pivot]
        if abs(pivot_value) < 1e-12:
            raise ValueError("singular trajectory solve")
        for col in range(pivot, 4):
            rows[pivot][col] /= pivot_value
        for row in range(3):
            if row == pivot:
                continue
            scale = rows[row][pivot]
            for col in range(pivot, 4):
                rows[row][col] -= scale * rows[pivot][col]
    return rows[0][3], rows[1][3], rows[2][3]


def quintic_coefficients(
    p0: float,
    v0: float,
    a0: float,
    p1: float,
    v1: float,
    a1: float,
    duration: float,
) -> tuple[float, float, float, float, float, float]:
    t = duration
    c0 = p0
    c1 = v0
    c2 = 0.5 * a0
    matrix = (
        (t**3, t**4, t**5),
        (3.0 * t**2, 4.0 * t**3, 5.0 * t**4),
        (6.0 * t, 12.0 * t**2, 20.0 * t**3),
    )
    rhs = (
        p1 - (c0 + c1 * t + c2 * t**2),
        v1 - (c1 + 2.0 * c2 * t),
        a1 - 2.0 * c2,
    )
    c3, c4, c5 = solve3(matrix, rhs)
    return c0, c1, c2, c3, c4, c5


@dataclass(frozen=True)
class QuinticSegment:
    duration: float
    coeff_x: tuple[float, float, float, float, float, float]
    coeff_z: tuple[float, float, float, float, float, float]
    end: Vec2

    @staticmethod
    def build(
        start: Vec2,
        start_velocity: Vec2,
        start_acceleration: Vec2,
        end: Vec2,
        end_velocity: Vec2,
        end_acceleration: Vec2,
        duration: float,
    ) -> "QuinticSegment":
        duration = max(0.20, duration)
        return QuinticSegment(
            duration=duration,
            coeff_x=quintic_coefficients(
                start[0],
                start_velocity[0],
                start_acceleration[0],
                end[0],
                end_velocity[0],
                end_acceleration[0],
                duration,
            ),
            coeff_z=quintic_coefficients(
                start[1],
                start_velocity[1],
                start_acceleration[1],
                end[1],
                end_velocity[1],
                end_acceleration[1],
                duration,
            ),
            end=end,
        )

    def sample(self, time_s: float) -> ReferenceState:
        t = clamp(time_s, 0.0, self.duration)
        px, vx, ax = self._sample_axis(self.coeff_x, t)
        pz, vz, az = self._sample_axis(self.coeff_z, t)
        return ReferenceState(
            position=(px, pz),
            velocity=(vx, vz),
            acceleration=(ax, az),
            final_target=self.end,
            active_target=self.end,
            active=t < self.duration,
            waypoint_count=1,
        )

    @staticmethod
    def _sample_axis(coefficients: Sequence[float], t: float) -> tuple[float, float, float]:
        c0, c1, c2, c3, c4, c5 = coefficients
        position = c0 + c1 * t + c2 * t**2 + c3 * t**3 + c4 * t**4 + c5 * t**5
        velocity = c1 + 2.0 * c2 * t + 3.0 * c3 * t**2 + 4.0 * c4 * t**3 + 5.0 * c5 * t**4
        acceleration = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t**2 + 20.0 * c5 * t**3
        return position, velocity, acceleration


@dataclass(frozen=True)
class SampledPathSegment:
    samples: tuple[Vec2, ...]
    lengths: tuple[float, ...]
    duration: float
    end: Vec2

    @staticmethod
    def build(points: Sequence[Vec2], speed: float) -> "SampledPathSegment":
        cleaned: list[Vec2] = []
        for point in points:
            if not cleaned or distance2(point, cleaned[-1]) > 1e-6:
                cleaned.append(point)
        if len(cleaned) < 2:
            cleaned = [cleaned[0], cleaned[0]] if cleaned else [(0.0, 0.0), (0.0, 0.0)]

        samples = SampledPathSegment._catmull_rom_samples(cleaned)
        lengths = [0.0]
        for index in range(1, len(samples)):
            lengths.append(lengths[-1] + distance2(samples[index - 1], samples[index]))
        total_length = max(lengths[-1], 1e-9)
        return SampledPathSegment(
            samples=tuple(samples),
            lengths=tuple(lengths),
            duration=max(0.45, total_length / max(speed, 1e-6)),
            end=samples[-1],
        )

    def sample(self, time_s: float) -> ReferenceState:
        if len(self.samples) < 2 or self.duration <= 1e-9:
            point = self.end
            return ReferenceState(point, (0.0, 0.0), (0.0, 0.0), self.end, self.end, False, 1)

        u = clamp(time_s / self.duration, 0.0, 1.0)
        sigma = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        sigma_dot = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / self.duration
        sigma_ddot = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / (self.duration * self.duration)
        total_length = self.lengths[-1]
        distance_along = sigma * total_length
        index = self._length_index(distance_along)
        segment_length = max(self.lengths[index + 1] - self.lengths[index], 1e-9)
        local = (distance_along - self.lengths[index]) / segment_length
        start = self.samples[index]
        end = self.samples[index + 1]
        tangent = scale2(sub2(end, start), 1.0 / segment_length)
        position = add2(start, scale2(sub2(end, start), local))
        velocity = scale2(tangent, sigma_dot * total_length)
        acceleration = scale2(tangent, sigma_ddot * total_length)
        return ReferenceState(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            final_target=self.end,
            active_target=self.end,
            active=time_s < self.duration,
            waypoint_count=1,
        )

    def _length_index(self, distance_along: float) -> int:
        for index in range(len(self.lengths) - 1):
            if distance_along <= self.lengths[index + 1]:
                return index
        return max(0, len(self.lengths) - 2)

    @staticmethod
    def _catmull_rom_samples(points: Sequence[Vec2]) -> list[Vec2]:
        samples: list[Vec2] = []
        for index in range(len(points) - 1):
            p0 = points[max(0, index - 1)]
            p1 = points[index]
            p2 = points[index + 1]
            p3 = points[min(len(points) - 1, index + 2)]
            span = distance2(p1, p2)
            sample_count = max(5, int(span / 0.030))
            for sample_index in range(sample_count):
                u = sample_index / sample_count
                samples.append(SampledPathSegment._catmull_rom_point(p0, p1, p2, p3, u))
        samples.append(points[-1])
        filtered: list[Vec2] = []
        for sample in samples:
            if not filtered or distance2(sample, filtered[-1]) > 1e-5:
                filtered.append(sample)
        return filtered

    @staticmethod
    def _catmull_rom_point(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, u: float) -> Vec2:
        u2 = u * u
        u3 = u2 * u
        x = 0.5 * (
            2.0 * p1[0]
            + (-p0[0] + p2[0]) * u
            + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * u2
            + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * u3
        )
        z = 0.5 * (
            2.0 * p1[1]
            + (-p0[1] + p2[1]) * u
            + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * u2
            + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * u3
        )
        return x, z


class ReferenceTrajectory:
    """Smooth reference generator for straight moves and waypoint curves."""

    def __init__(
        self,
        initial_position: Vec2,
        speed: float,
        tolerance: float,
        accel_limit: float = math.inf,
        jerk_limit: float = math.inf,
        min_segment_duration: float = 0.45,
        preview_time_s: float = 1.0,
        preview_min_distance_m: float = 0.10,
        turn_lateral_accel_m_s2: float = 0.10,
    ) -> None:
        self.speed = speed
        self.tolerance = tolerance
        self.accel_limit = accel_limit
        self.jerk_limit = jerk_limit
        self.min_segment_duration = min_segment_duration
        self.preview_time_s = max(0.05, preview_time_s)
        self.preview_min_distance_m = max(0.0, preview_min_distance_m)
        self.turn_lateral_accel_m_s2 = max(1e-4, turn_lateral_accel_m_s2)
        self.position = initial_position
        self.velocity: Vec2 = (0.0, 0.0)
        self.acceleration: Vec2 = (0.0, 0.0)
        self.goals: list[Vec2] = []
        self.segments: list[QuinticSegment] = []
        self.segment_time = 0.0
        self.geometric_progress_m = 0.0
        self._path_sample_cache: list[Vec2] | None = None
        self._path_cumulative_cache: list[float] | None = None
        self.final_target = initial_position
        self.mode = "hold"

    def _invalidate_path_cache(self) -> None:
        self._path_sample_cache = None
        self._path_cumulative_cache = None

    def reset(self, position: Vec2) -> None:
        self.position = position
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals.clear()
        self.segments.clear()
        self.segment_time = 0.0
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        self.final_target = position
        self.mode = "hold"

    def command_straight(self, start: Vec2, goal: Vec2) -> None:
        self.position = start
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals = [goal]
        self.final_target = goal
        self.mode = "straight"
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        duration = self._segment_duration(start, goal)
        self.segments = [
            QuinticSegment.build(
                start,
                (0.0, 0.0),
                (0.0, 0.0),
                goal,
                (0.0, 0.0),
                (0.0, 0.0),
                duration,
            )
        ]
        self.segment_time = 0.0

    def append_smooth_waypoint(self, start: Vec2, goal: Vec2) -> None:
        if not self.segments:
            self.position = start
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
        self.goals.append(goal)
        self.final_target = goal
        self.mode = "smooth"
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        self._rebuild_smooth_segments()

    def append_stop_waypoint(self, start: Vec2, goal: Vec2) -> None:
        if not self.segments:
            self.position = start
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            segment_start = start
            self.segment_time = 0.0
        else:
            segment_start = self.segments[-1].end
        self.goals.append(goal)
        self.final_target = goal
        self.mode = "stop"
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        duration = self._segment_duration(segment_start, goal)
        self.segments.append(
            QuinticSegment.build(
                segment_start,
                (0.0, 0.0),
                (0.0, 0.0),
                goal,
                (0.0, 0.0),
                (0.0, 0.0),
                duration,
            )
        )

    def command_smooth_path(self, start: Vec2, goals: Sequence[Vec2]) -> None:
        self.position = start
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals = [goals[-1]] if goals else []
        self.final_target = self.goals[-1] if self.goals else start
        self.mode = "draw"
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        points = [start, *goals]
        self.segments = [SampledPathSegment.build(points, self.speed)] if len(points) >= 2 else []
        self.segment_time = 0.0

    def command_corner_smooth_path(self, start: Vec2, goals: Sequence[Vec2], corner_speed: float) -> None:
        self.position = start
        self.velocity = (0.0, 0.0)
        self.acceleration = (0.0, 0.0)
        self.goals = list(goals)
        self.final_target = self.goals[-1] if self.goals else start
        self.mode = "coverage-smooth"
        self.segment_time = 0.0
        self.geometric_progress_m = 0.0
        self._invalidate_path_cache()
        if not goals:
            self.segments = []
            return

        points = [start, *goals]
        velocities = self._corner_smooth_velocities(points, corner_speed)
        self.segments = []
        for index in range(len(points) - 1):
            duration = self._segment_duration(points[index], points[index + 1])
            self.segments.append(
                QuinticSegment.build(
                    points[index],
                    velocities[index],
                    (0.0, 0.0),
                    points[index + 1],
                    velocities[index + 1],
                    (0.0, 0.0),
                    duration,
                )
            )

    def clear(self, hold_position: Vec2) -> None:
        self.reset(hold_position)

    def advance(self, dt: float) -> ReferenceState:
        if not self.segments:
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            self.final_target = self.position
            return self.state()

        remaining_dt = dt
        while self.segments and remaining_dt >= 0.0:
            segment = self.segments[0]
            time_left = segment.duration - self.segment_time
            if remaining_dt < time_left:
                self.segment_time += remaining_dt
                sample = segment.sample(self.segment_time)
                self.position = sample.position
                self.velocity = sample.velocity
                self.acceleration = sample.acceleration
                break

            sample = segment.sample(segment.duration)
            self.position = sample.position
            self.velocity = sample.velocity
            self.acceleration = sample.acceleration
            remaining_dt -= max(0.0, time_left)
            self.segments.pop(0)
            self._invalidate_path_cache()
            if self.goals:
                self.goals.pop(0)
            self.segment_time = 0.0
            if remaining_dt <= 1e-12:
                break

        if not self.segments:
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            self.final_target = self.position
            self.mode = "hold"
        return self.state()

    def geometric_reference(self, tool_position: Vec2, tool_velocity: Vec2, dt: float) -> ReferenceState:
        """Path-following reference from closest tool-head projection, not time advance."""

        if not self.segments:
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            return self.state()

        previous_reference_position = self.position
        samples = self._path_samples()
        projected_point, _tangent, _remaining_length, path_progress = self._project_tool_to_path(tool_position, samples)
        projection_error = distance2(tool_position, projected_point)
        capture_radius = max(0.12, 6.0 * self.tolerance)
        if projection_error <= capture_radius and path_progress > self.geometric_progress_m:
            # Prevent projection from jumping across nearby drawn-path segments.
            # This bound must scale with dt. The previous 4 mm minimum at a
            # 5 ms physics step allowed the geometric reference to advance at
            # 0.8 m/s even when the requested inspection speed was 0.16 m/s.
            # Position progress and the published reference velocity must obey
            # the same speed. Advancing position faster than ``self.speed``
            # gives NMPC a kinematically inconsistent reference at turns.
            max_progress_step = max(1.0e-6, self.speed * max(dt, 0.0))
            self.geometric_progress_m = min(path_progress, self.geometric_progress_m + max_progress_step)
        projection, tangent, remaining_length = self._sample_path_at_progress(samples, self.geometric_progress_m)
        tracking_error = distance2(tool_position, projection)
        speed = self.speed
        slowdown_distance = self._geometric_slowdown_distance(speed)
        speed_scale = clamp(remaining_length / max(slowdown_distance, 1e-9), 0.0, 1.0)
        target_speed = speed * speed_scale
        preview_distance = min(
            remaining_length,
            max(self.preview_min_distance_m, speed * self.preview_time_s),
        )
        preview_position, preview_tangent, preview_remaining = self._sample_path_at_progress(
            samples,
            self.geometric_progress_m + preview_distance,
        )
        del preview_position
        tangent_alignment = clamp(dot2(tangent, preview_tangent), -1.0, 1.0)
        heading_change = math.acos(tangent_alignment)
        curvature = heading_change / max(preview_distance, 1e-6)
        if curvature > 1e-6:
            turn_speed_limit = math.sqrt(self.turn_lateral_accel_m_s2 / curvature)
            target_speed = min(target_speed, turn_speed_limit)
        if tracking_error > capture_radius or remaining_length <= self.tolerance:
            target_speed = 0.0

        self.position = projection
        self.velocity = scale2(tangent, target_speed)
        preview_speed_scale = clamp(
            preview_remaining / max(slowdown_distance, 1e-9),
            0.0,
            1.0,
        )
        preview_speed = min(speed * preview_speed_scale, target_speed)
        preview_velocity = scale2(preview_tangent, preview_speed)
        preview_acceleration = scale2(
            sub2(preview_velocity, self.velocity),
            1.0 / self.preview_time_s,
        )
        self.acceleration = limit_norm2(preview_acceleration, self.accel_limit)

        completion_distance = max(self.tolerance, 0.015)
        completion_progress = max(1.0e-6, self.speed * max(dt, 0.0))
        complete = (
            remaining_length <= completion_distance
            and distance2(previous_reference_position, self.final_target) <= completion_progress
            and distance2(tool_position, self.final_target) <= completion_distance
            and math.hypot(tool_velocity[0], tool_velocity[1]) <= 0.040
        )
        if complete:
            self.position = self.final_target
            self.velocity = (0.0, 0.0)
            self.acceleration = (0.0, 0.0)
            self.goals.clear()
            self.segments.clear()
            self.segment_time = 0.0
            self._invalidate_path_cache()
            self.mode = "hold"
            return self.state()

        return ReferenceState(
            position=self.position,
            velocity=self.velocity,
            acceleration=self.acceleration,
            final_target=self.final_target,
            active_target=self.goals[0] if self.goals else self.final_target,
            active=True,
            waypoint_count=max(1, len(self.goals)),
        )

    def state(self) -> ReferenceState:
        return ReferenceState(
            position=self.position,
            velocity=self.velocity,
            acceleration=self.acceleration,
            final_target=self.final_target,
            active_target=self.goals[0] if self.goals else self.final_target,
            active=bool(self.segments),
            waypoint_count=len(self.goals),
        )

    def pending_path(self) -> list[Vec2]:
        if not self.segments:
            return [self.position]
        points: list[Vec2] = []
        for index, segment in enumerate(self.segments):
            start_time = self.segment_time if index == 0 else 0.0
            sample_count = max(6, int((segment.duration - start_time) / 0.12))
            for sample_index in range(sample_count + 1):
                u = sample_index / sample_count
                t = start_time + u * (segment.duration - start_time)
                points.append(segment.sample(t).position)
        return points

    def _path_samples(self) -> list[Vec2]:
        if not self.segments:
            return [self.position]
        if self._path_sample_cache is not None:
            return self._path_sample_cache

        points: list[Vec2] = []
        for segment in self.segments:
            if isinstance(segment, SampledPathSegment):
                segment_points = list(segment.samples)
            else:
                sample_count = max(12, int(segment.duration / 0.06))
                segment_points = [segment.sample(segment.duration * index / sample_count).position for index in range(sample_count + 1)]
            for point in segment_points:
                if not points or distance2(point, points[-1]) > 1e-6:
                    points.append(point)
        self._path_sample_cache = points if len(points) >= 2 else [self.position, self.final_target]
        cumulative = [0.0]
        for index in range(len(self._path_sample_cache) - 1):
            cumulative.append(
                cumulative[-1]
                + distance2(self._path_sample_cache[index], self._path_sample_cache[index + 1])
            )
        self._path_cumulative_cache = cumulative
        return self._path_sample_cache

    def _path_cumulative_lengths(self, samples: Sequence[Vec2]) -> list[float]:
        if samples is self._path_sample_cache and self._path_cumulative_cache is not None:
            return self._path_cumulative_cache
        cumulative = [0.0]
        for index in range(len(samples) - 1):
            cumulative.append(cumulative[-1] + distance2(samples[index], samples[index + 1]))
        return cumulative

    def _sample_path_at_progress(self, samples: Sequence[Vec2], progress_m: float) -> tuple[Vec2, Vec2, float]:
        if len(samples) < 2:
            return samples[0], (1.0, 0.0), 0.0

        cumulative = self._path_cumulative_lengths(samples)
        total_length = cumulative[-1]
        progress_m = clamp(progress_m, 0.0, total_length)
        index = min(len(samples) - 2, max(0, bisect.bisect_right(cumulative, progress_m) - 1))
        while index < len(samples) - 2 and cumulative[index + 1] - cumulative[index] <= 1e-9:
            index += 1
        start = samples[index]
        end = samples[index + 1]
        segment_length = max(cumulative[index + 1] - cumulative[index], 1e-9)
        local = clamp((progress_m - cumulative[index]) / segment_length, 0.0, 1.0)
        tangent = normalize2(sub2(end, start))
        position = add2(start, scale2(sub2(end, start), local))
        return position, tangent, max(0.0, total_length - progress_m)

    def _project_tool_to_path(self, tool_position: Vec2, samples: Sequence[Vec2]) -> tuple[Vec2, Vec2, float, float]:
        best_projection = samples[0]
        best_tangent = normalize2(sub2(samples[min(1, len(samples) - 1)], samples[0]))
        best_distance_squared = math.inf
        best_remaining_length = 0.0
        best_progress = 0.0
        cumulative = self._path_cumulative_lengths(samples)
        total_length = cumulative[-1]
        search_back_m = 0.25
        search_forward_m = max(0.80, 4.0 * self.speed * self.preview_time_s)
        first_index = max(
            0,
            bisect.bisect_left(cumulative, max(0.0, self.geometric_progress_m - search_back_m)) - 1,
        )
        last_index = min(
            len(samples) - 2,
            bisect.bisect_right(cumulative, self.geometric_progress_m + search_forward_m),
        )
        for index in range(first_index, last_index + 1):
            segment_length = cumulative[index + 1] - cumulative[index]
            if segment_length <= 1e-9:
                continue
            start = samples[index]
            end = samples[index + 1]
            delta = sub2(end, start)
            local = clamp(dot2(sub2(tool_position, start), delta) / (segment_length * segment_length), 0.0, 1.0)
            projection = add2(start, scale2(delta, local))
            error = sub2(tool_position, projection)
            distance_squared = dot2(error, error)
            progress = cumulative[index] + local * segment_length
            if (
                distance_squared < best_distance_squared - 1e-12
                or abs(distance_squared - best_distance_squared) <= 1e-12
                and progress > best_progress
            ):
                best_distance_squared = distance_squared
                best_projection = projection
                best_tangent = scale2(delta, 1.0 / segment_length)
                best_progress = progress
                best_remaining_length = max(0.0, total_length - progress)

        if abs(best_tangent[0]) + abs(best_tangent[1]) <= 1e-9:
            best_tangent = (1.0, 0.0)
        return best_projection, normalize2(best_tangent), best_remaining_length, best_progress

    def _geometric_slowdown_distance(self, speed: float) -> float:
        dynamic_distance = 0.0
        if math.isfinite(self.accel_limit) and self.accel_limit > 1e-9:
            dynamic_distance = speed * speed / (2.0 * self.accel_limit)
        return max(0.12, 1.4 * dynamic_distance, 2.0 * self.tolerance)

    def _rebuild_smooth_segments(self) -> None:
        points = [self.position, *self.goals]
        if len(points) < 2:
            self.segments.clear()
            self.segment_time = 0.0
            return

        velocities = self._waypoint_velocities(points)
        velocities[0] = self.velocity
        velocities[-1] = (0.0, 0.0)
        accelerations = [(0.0, 0.0) for _ in points]
        accelerations[0] = self.acceleration

        self.segments = []
        for index in range(len(points) - 1):
            duration = self._segment_duration(points[index], points[index + 1])
            self.segments.append(
                QuinticSegment.build(
                    points[index],
                    velocities[index],
                    accelerations[index],
                    points[index + 1],
                    velocities[index + 1],
                    accelerations[index + 1],
                    duration,
                )
            )
        self.segment_time = 0.0

    def _waypoint_velocities(self, points: Sequence[Vec2]) -> list[Vec2]:
        velocities = [(0.0, 0.0) for _ in points]
        for index in range(1, len(points) - 1):
            chord = sub2(points[index + 1], points[index - 1])
            direction = normalize2(chord)
            prev_distance = distance2(points[index], points[index - 1])
            next_distance = distance2(points[index + 1], points[index])
            local_speed = min(self.speed, 0.5 * (prev_distance + next_distance) / max(self._segment_duration(points[index - 1], points[index + 1]), 1e-6))
            velocities[index] = scale2(direction, local_speed)
        return velocities

    def _corner_smooth_velocities(self, points: Sequence[Vec2], corner_speed: float) -> list[Vec2]:
        velocities = [(0.0, 0.0) for _ in points]
        bounded_corner_speed = clamp(corner_speed, 0.0, self.speed)
        x_values = [point[0] for point in points]
        z_values = [point[1] for point in points]
        min_x = min(x_values)
        max_x = max(x_values)
        min_z = min(z_values)
        max_z = max(z_values)
        boundary_eps = 1e-6
        for index in range(1, len(points) - 1):
            incoming = normalize2(sub2(points[index], points[index - 1]))
            outgoing = normalize2(sub2(points[index + 1], points[index]))
            turn_alignment = dot2(incoming, outgoing)
            if turn_alignment > 0.95:
                waypoint_speed = self.speed
            elif turn_alignment > 0.20:
                waypoint_speed = min(self.speed, 2.0 * bounded_corner_speed)
            else:
                waypoint_speed = bounded_corner_speed
            velocity = scale2(normalize2(sub2(points[index + 1], points[index - 1])), waypoint_speed)
            x, z = points[index]
            vx, vz = velocity
            if x <= min_x + boundary_eps or x >= max_x - boundary_eps:
                vx = 0.0
            if z <= min_z + boundary_eps or z >= max_z - boundary_eps:
                vz = 0.0
            velocities[index] = (vx, vz)
        return velocities

    def _segment_duration(self, start: Vec2, end: Vec2) -> float:
        distance = distance2(start, end)
        speed_duration = distance / max(self.speed, 1e-6)
        accel_duration = 0.0
        if math.isfinite(self.accel_limit) and self.accel_limit > 1e-9:
            accel_duration = math.sqrt(5.7736 * distance / self.accel_limit)
        jerk_duration = 0.0
        if math.isfinite(self.jerk_limit) and self.jerk_limit > 1e-9:
            jerk_duration = (60.0 * distance / self.jerk_limit) ** (1.0 / 3.0)
        return max(self.min_segment_duration, speed_duration, accel_duration, jerk_duration)


class WallToolSimulator:
    def __init__(self, params: SimParams, *, external_plant: bool = False) -> None:
        self.params = params
        self.external_plant = bool(external_plant)
        if self.external_plant:
            raise NotImplementedError(
                "the CoppeliaSim bridge does not yet drive the two independent tilt-servo channels; "
                "wall_tool_2d refuses to substitute fixed thrust axes"
            )
        self.default_target = params.initial_payload
        self.trajectory = ReferenceTrajectory(
            params.initial_payload,
            speed=params.path_speed,
            tolerance=params.waypoint_tolerance,
            accel_limit=params.reference_accel_limit_mps2,
            jerk_limit=params.reference_jerk_limit_mps3,
            min_segment_duration=params.reference_min_segment_duration_s,
            preview_time_s=params.reference_preview_time_s,
            preview_min_distance_m=params.reference_preview_min_distance_m,
            turn_lateral_accel_m_s2=params.reference_turn_lateral_accel_m_s2,
        )
        self._nmpc: WallToolNMPC | None = None
        self._nmpc_next_solve_t = 0.0
        self._nmpc_last_solution: MPCSolution | None = None
        self._nmpc_previous_command = (0.0, 0.0, 0.0, 0.0, 0.0)
        self._sensor_rng = random.Random(params.sensor_random_seed)
        self.reset()

    def reset(self) -> None:
        self.t = 0.0
        self.position = self.default_target
        self.velocity: Vec2 = (0.0, 0.0)
        self.acceleration: Vec2 = (0.0, 0.0)
        self.theta = 0.0
        self.theta_dot = 0.0
        self.length = 0.0
        self.length_dot = 0.0
        self.length_ddot = 0.0
        self.attitude = self.params.nominal_attitude_rad
        self.angular_velocity = 0.0
        self.angular_acceleration = 0.0
        initial_distance = self._point_to_polar(self.default_target)[1]
        initial_tension = min(
            self.params.max_spool_tension,
            self.params.desired_cable_support_fraction * self.params.total_mass * self.params.gravity,
        )
        initial_cable_weight = self._payload_supported_cable_weight(initial_distance)
        initial_hover_thrust = max(0.0, min(
            self.params.max_thrust_per_drone,
            0.5 * (self.params.total_mass * self.params.gravity + initial_cable_weight - initial_tension),
        ))
        initial_extension = initial_tension / max(self._steel_cable_stiffness(initial_distance), 1e-9)
        self.cable_length = clamp(
            initial_distance - initial_extension,
            self.params.min_cable_length,
            self.params.max_cable_length,
        )
        self.cable_stretch = initial_extension
        self.cable_slack = False
        self.cable_tension_saturated = False
        self.filtered_cable_tension_target = initial_tension
        self.actual_tension = initial_tension
        self.actual_left_thrust = initial_hover_thrust
        self.actual_right_thrust = initial_hover_thrust
        self.estimated_left_thrust = initial_hover_thrust
        self.estimated_right_thrust = initial_hover_thrust
        self.last_left_thrust_command = initial_hover_thrust
        self.last_right_thrust_command = initial_hover_thrust
        self.actual_left_gimbal_angle = 0.0
        self.actual_right_gimbal_angle = 0.0
        self.actual_left_gimbal_rate = 0.0
        self.actual_right_gimbal_rate = 0.0
        self.left_gimbal_acceleration = 0.0
        self.right_gimbal_acceleration = 0.0
        self.left_gimbal_saturated = False
        self.right_gimbal_saturated = False
        self.last_left_gimbal_command = 0.0
        self.last_right_gimbal_command = 0.0
        self.actual_reel_velocity = 0.0
        self.load_cell_tension = initial_tension
        self.reel_tension_error_integral = 0.0
        self.last_spool_velocity_cmd = 0.0
        self._nmpc_next_solve_t = 0.0
        self._nmpc_last_solution = None
        self._nmpc_previous_command = (
            initial_hover_thrust,
            initial_hover_thrust,
            0.0,
            0.0,
            0.0,
        )
        self._hold_position_error_integral: Vec2 = (0.0, 0.0)
        self.measured_payload = self.position
        self.estimated_payload_velocity = (0.0, 0.0)
        self.measured_theta = 0.0
        self.measured_theta_dot = 0.0
        self.measured_line_length = initial_distance
        self.measured_line_velocity = 0.0
        self.measured_cable_stretch = self.cable_stretch
        self.measured_attitude = self.attitude
        self.measured_angular_velocity = 0.0
        self.measured_cable_length = self.cable_length
        self.measured_cable_velocity = 0.0
        self.measured_tension = initial_tension
        # The flight controller does not receive servo shaft-angle feedback.
        # These are command-driven actuator-state estimates, separate from the
        # plant's actual servo angles above.
        self.estimated_left_gimbal_angle = 0.0
        self.estimated_right_gimbal_angle = 0.0
        self.estimated_left_gimbal_rate = 0.0
        self.estimated_right_gimbal_rate = 0.0
        self._previous_sensor_theta = 0.0
        self._previous_sensor_position = self.position
        self._next_sensor_sample_t = -math.inf
        self.normal_gap = clamp(
            self.params.normal_initial_gap_m,
            self.params.normal_gap_min_m,
            self.params.normal_gap_max_m,
        )
        self.normal_velocity = 0.0
        self.normal_acceleration = 0.0
        self.normal_actuator_force = 0.0
        self.normal_wind_force = 0.0
        self.contact_force = 0.0
        self.desired_contact_force = 0.0
        self.contact_work_mode = False
        self._update_cable_coordinates()
        self._update_sensor_estimate()
        self.trajectory.reset(self.default_target)
        self.history: list[SimState] = [
            self.snapshot(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                False,
                self.trajectory.state(),
            )
        ]

    def _clamp_wall_point(self, point: Vec2) -> Vec2:
        return clamp_wall_point_for_params(point, self.params)

    def planned_waypoints(self, point: Vec2, planner: str = PLANNER_DIRECT) -> tuple[Vec2, ...]:
        target = self._clamp_wall_point(point)
        if planner == PLANNER_DIRECT:
            return (snap_wall_point(target),)
        if planner == PLANNER_CENTER_SETUP:
            setup = center_setup_waypoint(target, self.params)
            return (snap_wall_point(setup), snap_wall_point(target)) if setup is not None else (snap_wall_point(target),)
        if planner == PLANNER_PREDICTIVE:
            return predictive_waypoints(self._payload_from_state(), target, self.params)
        raise ValueError(f"Unknown planner '{planner}'. Choose one of: {', '.join(PLANNER_CHOICES)}")

    def set_target(self, point: Vec2, planner: str = PLANNER_DIRECT) -> None:
        start = self._payload_from_state()
        waypoints = self.planned_waypoints(point, planner)
        # Point clicks use the same geometric path representation as drag paths.
        # With the direct planner this is a single segment to the clicked point,
        # not a setup waypoint or dt-advanced target.
        self.trajectory.command_smooth_path(start, waypoints)

    def append_target(self, point: Vec2, planner: str = PLANNER_DIRECT) -> None:
        for waypoint in self.planned_waypoints(point, planner):
            self.trajectory.append_smooth_waypoint(self._payload_from_state(), waypoint)

    def append_stop_target(self, point: Vec2, planner: str = PLANNER_DIRECT) -> None:
        for waypoint in self.planned_waypoints(point, planner):
            self.trajectory.append_stop_waypoint(self._payload_from_state(), waypoint)

    def set_smooth_path(self, points: Sequence[Vec2]) -> None:
        clamped_points = [self._clamp_wall_point(point) for point in points]
        if not clamped_points:
            return
        self.trajectory.command_smooth_path(self._payload_from_state(), clamped_points)

    def set_corner_smooth_path(self, points: Sequence[Vec2], corner_speed: float) -> None:
        clamped_points = [self._clamp_wall_point(point) for point in points]
        if not clamped_points:
            return
        self.trajectory.command_corner_smooth_path(self._payload_from_state(), clamped_points, corner_speed)

    def clear_trajectory(self) -> None:
        self.trajectory.clear(self._payload_from_state())
        if self.history:
            last = self.history[-1]
            self.history[-1] = self.snapshot(
                last.left_thrust,
                last.right_thrust,
                last.tension,
                last.tangential_force,
                last.spool_velocity_cmd,
                last.drone_accel_cmd,
                last.desired_cable_tension,
                last.desired_drone_force,
                last.drone_force,
                last.cable_force,
                last.wind_force,
                last.saturated,
                self.trajectory.state(),
                desired_tangential_force=last.desired_tangential_force,
            )

    def _cable_mount_offset(self, attitude: float) -> Vec2:
        return rotate2((0.0, self.params.payload_hex_radius), attitude)

    def _cable_mount_position(self, payload: Vec2, attitude: float) -> Vec2:
        return add2(payload, self._cable_mount_offset(attitude))

    def _module_center_offsets(self, attitude: float) -> tuple[Vec2, Vec2]:
        return integrated_motor_center_offsets(self.params, attitude)

    def _drone_axes(
        self,
        attitude: float,
        left_gimbal_angle: float,
        right_gimbal_angle: float,
    ) -> tuple[Vec2, Vec2]:
        return integrated_motor_axes(
            self.params,
            attitude,
            left_gimbal_angle,
            right_gimbal_angle,
        )

    def _point_to_polar(self, point: Vec2) -> tuple[float, float]:
        attach_point = self._cable_mount_position(point, self.params.nominal_attitude_rad)
        dx = attach_point[0] - self.params.anchor[0]
        dz_down = self.params.anchor[1] - attach_point[1]
        length = clamp(math.hypot(dx, dz_down), self.params.min_cable_length, self.params.max_cable_length)
        theta = math.atan2(dx, dz_down)
        return theta, length

    def _reference_to_polar(self, reference: ReferenceState) -> tuple[float, float, float, float]:
        theta, length = self._point_to_polar(reference.position)
        e_r = (math.sin(theta), -math.cos(theta))
        e_theta = (math.cos(theta), math.sin(theta))
        length_dot = dot2(reference.velocity, e_r)
        theta_dot = dot2(reference.velocity, e_theta) / max(length, 1e-6)
        return theta, length, theta_dot, length_dot

    def _payload_from_state(self) -> Vec2:
        return self.position

    def _payload_velocity_from_state(self) -> Vec2:
        return self.velocity

    @staticmethod
    def _quantize(value: float, resolution: float) -> float:
        if not math.isfinite(value) or not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("sensor value and resolution must be finite; resolution must be positive")
        return round(value / resolution) * resolution

    def _update_sensor_estimate(self, *, force: bool = False) -> None:
        """Sample explicit encoders, load cell, and IMU and reconstruct x-z state."""

        params = self.params
        if not force and self.t + 1.0e-12 < self._next_sensor_sample_t:
            return
        sample_dt = params.sensor_sample_period_s
        self._next_sensor_sample_t = self.t + sample_dt

        cable_angle_resolution = 2.0 * math.pi / params.cable_angle_encoder_counts_per_rev
        reel_length_resolution = (
            2.0 * math.pi * params.reel_spool_radius_m / params.reel_encoder_counts_per_rev
        )

        noisy_theta = self.theta + self._sensor_rng.gauss(0.0, params.cable_angle_noise_std_rad)
        measured_theta = self._quantize(wrap_angle(noisy_theta), cable_angle_resolution)
        measured_cable_length = self._quantize(
            self.cable_length + self._sensor_rng.gauss(0.0, params.reel_length_noise_std_m),
            reel_length_resolution,
        )
        measured_cable_length = clamp(
            measured_cable_length,
            params.min_cable_length,
            params.max_cable_length,
        )

        load_alpha = clamp(
            sample_dt / max(params.load_cell_filter_tau_s + sample_dt, 1e-9),
            0.0,
            1.0,
        )
        noisy_tension = self.actual_tension + self._sensor_rng.gauss(0.0, params.load_cell_noise_std_N)
        self.load_cell_tension += load_alpha * (noisy_tension - self.load_cell_tension)
        measured_tension = clamp(self.load_cell_tension, 0.0, params.max_spool_tension)

        measured_attitude = self._quantize(
            self.attitude + self._sensor_rng.gauss(0.0, params.imu_angle_noise_std_rad),
            2.0 * math.pi / 65536.0,
        )
        measured_angular_velocity = self.angular_velocity + self._sensor_rng.gauss(
            0.0, params.imu_rate_noise_std_rad_s
        )

        stiffness = self._steel_cable_stiffness(max(measured_cable_length, params.min_cable_length))
        estimated_stretch = measured_tension / max(stiffness, 1e-9)
        measured_line_length = measured_cable_length + estimated_stretch
        cable_out = (math.sin(measured_theta), -math.cos(measured_theta))
        estimated_mount = add2(params.anchor, scale2(cable_out, measured_line_length))
        estimated_payload = sub2(estimated_mount, self._cable_mount_offset(measured_attitude))

        velocity_alpha = clamp(
            sample_dt / max(params.velocity_estimator_time_constant_s + sample_dt, 1e-9),
            0.0,
            1.0,
        )
        raw_velocity = scale2(sub2(estimated_payload, self._previous_sensor_position), 1.0 / sample_dt)
        estimated_velocity = add2(
            self.estimated_payload_velocity,
            scale2(sub2(raw_velocity, self.estimated_payload_velocity), velocity_alpha),
        )
        measured_theta_dot = wrap_angle(measured_theta - self._previous_sensor_theta) / sample_dt
        measured_line_velocity = (measured_line_length - self.measured_line_length) / sample_dt

        self.measured_theta = measured_theta
        self.measured_theta_dot = measured_theta_dot
        self.measured_cable_length = measured_cable_length
        self.measured_cable_velocity = self._quantize(
            self.actual_reel_velocity,
            reel_length_resolution / sample_dt,
        )
        self.measured_tension = measured_tension
        self.measured_attitude = measured_attitude
        self.measured_angular_velocity = measured_angular_velocity
        self.measured_cable_stretch = estimated_stretch
        self.measured_line_velocity = measured_line_velocity
        self.measured_line_length = measured_line_length
        self.measured_payload = estimated_payload
        self.estimated_payload_velocity = estimated_velocity

        self._previous_sensor_theta = measured_theta
        self._previous_sensor_position = estimated_payload

    def _safe_reference(self, reference: ReferenceState) -> ReferenceState:
        return ReferenceState(
            position=self._clamp_wall_point(reference.position),
            velocity=reference.velocity,
            acceleration=reference.acceleration,
            final_target=self._clamp_wall_point(reference.final_target),
            active_target=self._clamp_wall_point(reference.active_target),
            active=reference.active,
            waypoint_count=reference.waypoint_count,
        )

    def _wind_force(self) -> Vec2:
        params = self.params
        if not params.wind_enabled:
            return (0.0, 0.0)
        phase = 2.0 * math.pi * self.t / max(params.wind_gust_period_s, 1e-6)
        gust = params.wind_gust_force * (0.55 * math.sin(phase) + 0.25 * math.sin(2.7 * phase + 0.6))
        edge_ratio = abs(self.position[0]) / max(params.wall_width / 2.0, 1e-6)
        edge_gain = 1.0 + params.edge_wind_gain * edge_ratio * edge_ratio
        return (
            edge_gain * (params.wind_force_x + gust),
            params.wind_force_z + params.wind_gust_vertical_fraction * gust * math.sin(0.43 * phase + 1.2),
        )

    def _normal_wind_force(self) -> float:
        params = self.params
        if not params.normal_contact_enabled:
            return 0.0
        phase = 2.0 * math.pi * self.t / max(params.normal_wind_gust_period_s, 1e-6)
        gust = params.normal_wind_gust_force_N * (0.55 * math.sin(phase + 0.4) + 0.22 * math.sin(2.1 * phase))
        return params.normal_wind_force_N + gust

    @staticmethod
    def _cable_support_tension_limit(cable_axis: Vec2, params: SimParams) -> float:
        vertical_efficiency = max(0.0, cable_axis[1])
        if vertical_efficiency <= 1e-9:
            return 0.0
        support_limited_tension = (
            params.max_cable_support_fraction * params.total_mass * params.gravity / vertical_efficiency
        )
        return clamp(support_limited_tension, 0.0, params.max_spool_tension)

    def _steel_cable_spec(self) -> SteelCableSpec:
        params = self.params
        return SteelCableSpec(
            diameter_m=params.steel_cable_diameter_m,
            youngs_modulus_pa=params.steel_cable_youngs_modulus_pa,
            density_kg_m3=params.steel_cable_density_kg_m3,
            structural_compliance_m_N=params.steel_cable_structural_compliance_m_N,
            damping_ratio=params.steel_cable_damping_ratio,
            payload_weight_fraction=params.steel_cable_payload_weight_fraction,
        )

    def _reel_motor_spec(self) -> ReelMotorSpec:
        params = self.params
        return ReelMotorSpec(
            voltage_v=params.reel_motor_voltage_v,
            gear_ratio=params.reel_motor_gear_ratio,
            no_load_output_rpm=params.reel_motor_no_load_rpm,
            stall_torque_kg_cm=params.reel_motor_stall_torque_kg_cm,
            spool_radius_m=params.reel_spool_radius_m,
            velocity_time_constant_s=params.reel_velocity_time_constant_s,
            continuous_torque_fraction=params.reel_continuous_torque_fraction,
        )

    def _steel_cable_stiffness(self, length_m: float) -> float:
        return self._steel_cable_spec().axial_stiffness_N_m(length_m)

    def _steel_cable_damping(self, length_m: float) -> float:
        spec = self._steel_cable_spec()
        effective_mass = self.params.total_mass + spec.mass_kg(length_m) / 3.0
        return spec.damping_N_s_m(length_m, effective_mass)

    def _payload_supported_cable_weight(self, length_m: float) -> float:
        spec = self._steel_cable_spec()
        return spec.payload_weight_fraction * spec.weight_N(length_m, self.params.gravity)

    def _in_contact_work_region(self, point: Vec2) -> bool:
        params = self.params
        margin = params.contact_work_margin_m
        return (
            params.contact_work_enabled
            and params.contact_work_x_min - margin <= point[0] <= params.contact_work_x_max + margin
            and params.contact_work_z_min - margin <= point[1] <= params.contact_work_z_max + margin
        )

    def _contact_work_mode_for_reference(self, reference: ReferenceState) -> bool:
        return self._in_contact_work_region(reference.position) or self._in_contact_work_region(self.position)

    def _surface_contact_force(self, gap: float, velocity: float) -> float:
        params = self.params
        penetration = max(0.0, -gap)
        contact = params.normal_contact_stiffness_N_m * penetration - params.normal_contact_damping_N_s_m * velocity
        return clamp(contact, 0.0, params.normal_contact_force_limit_N)

    def _update_normal_contact(self, reference: ReferenceState) -> None:
        params = self.params
        if not params.normal_contact_enabled:
            self.normal_gap = params.normal_standoff_m
            self.normal_velocity = 0.0
            self.normal_acceleration = 0.0
            self.normal_actuator_force = 0.0
            self.normal_wind_force = 0.0
            self.contact_force = 0.0
            self.desired_contact_force = 0.0
            self.contact_work_mode = False
            return

        self.contact_work_mode = self._contact_work_mode_for_reference(reference)
        self.desired_contact_force = params.desired_contact_force_N if self.contact_work_mode else 0.0
        guide_free_gap = params.normal_standoff_m
        if self.contact_work_mode:
            desired_gap = -self.desired_contact_force / max(params.normal_contact_stiffness_N_m, 1e-9)
            guide_free_gap = desired_gap - self.desired_contact_force / max(params.normal_position_kp, 1e-9)

        contact_before = self._surface_contact_force(self.normal_gap, self.normal_velocity)
        guide_deflection = self.normal_gap - guide_free_gap
        actuator_force = (
            params.normal_position_kp * guide_deflection
            + params.normal_position_kd * self.normal_velocity
        )

    def _gimbal_servo_spec(self) -> GimbalServoSpec:
        params = self.params
        return GimbalServoSpec(
            max_angle_rad=params.gimbal_max_angle_rad,
            max_rate_rad_s=params.gimbal_max_rate_rad_s,
            max_acceleration_rad_s2=params.gimbal_max_acceleration_rad_s2,
            natural_frequency_rad_s=params.gimbal_natural_frequency_rad_s,
            damping_ratio=params.gimbal_damping_ratio,
            command_min_pulse_us=params.gimbal_command_min_pulse_us,
            command_max_pulse_us=params.gimbal_command_max_pulse_us,
            command_resolution_us=params.gimbal_command_resolution_us,
        )
        actuator_force = clamp(
            actuator_force,
            -params.normal_retract_force_limit_N,
            params.normal_push_force_limit_N,
        )
        normal_wind = self._normal_wind_force()
        normal_damping = params.normal_air_damping * self.normal_velocity
        self.normal_acceleration = (
            contact_before + normal_wind - actuator_force - normal_damping
        ) / max(params.total_mass, 1e-9)
        self.normal_velocity += self.normal_acceleration * params.dt
        self.normal_gap += self.normal_velocity * params.dt
        if self.normal_gap < params.normal_gap_min_m:
            self.normal_gap = params.normal_gap_min_m
            self.normal_velocity = max(0.0, self.normal_velocity)
        elif self.normal_gap > params.normal_gap_max_m:
            self.normal_gap = params.normal_gap_max_m
            self.normal_velocity = min(0.0, self.normal_velocity)
        self.normal_actuator_force = actuator_force
        self.normal_wind_force = normal_wind
        self.contact_force = self._surface_contact_force(self.normal_gap, self.normal_velocity)

    def _contact_valid_for_reference(self, reference: ReferenceState) -> bool:
        params = self.params
        if not (params.normal_contact_enabled and self.contact_work_mode):
            return False
        speed = math.hypot(self.velocity[0], self.velocity[1])
        tracking_error = distance2(self._payload_from_state(), reference.position)
        return (
            params.min_contact_force_N <= self.contact_force <= params.max_contact_force_N
            and speed <= params.work_contact_speed_limit_mps
            and tracking_error <= params.work_contact_tracking_limit_m
            and abs(self.angular_velocity) <= params.work_contact_angular_rate_limit_rad_s
        )

    def _update_cable_coordinates(self) -> None:
        mount_offset = self._cable_mount_offset(self.attitude)
        mount_position = add2(self.position, mount_offset)
        mount_velocity = add2(self.velocity, scale2((-mount_offset[1], mount_offset[0]), self.angular_velocity))
        anchor_to_mount = sub2(mount_position, self.params.anchor)
        distance = max(1e-9, math.hypot(anchor_to_mount[0], anchor_to_mount[1]))
        e_out = (anchor_to_mount[0] / distance, anchor_to_mount[1] / distance)
        e_theta = (-e_out[1], e_out[0])
        self.length = distance
        self.theta = math.atan2(mount_position[0] - self.params.anchor[0], self.params.anchor[1] - mount_position[1])
        self.length_dot = dot2(mount_velocity, e_out)
        self.theta_dot = dot2(mount_velocity, e_theta) / distance
        self.length_ddot = dot2(self.acceleration, e_out)

    def _nmpc_config(self) -> MPCConfig:
        params = self.params
        left_offset_zero, right_offset_zero = self._module_center_offsets(0.0)
        wall_margin = max(params.cage_radius, params.payload_half_length, params.payload_hex_radius) * 1.4
        return MPCConfig(
            horizon_steps=max(2, params.mpc_horizon_steps),
            horizon_dt=max(params.dt, params.mpc_horizon_dt),
            control_period_s=max(params.dt, params.mpc_control_period_s),
            mass=params.total_mass,
            inertia=params.assembly_inertia,
            gravity=params.gravity,
            anchor=params.anchor,
            wall_width=params.wall_width,
            wall_height=params.wall_height,
            wall_margin=wall_margin,
            max_payload_speed_m_s=params.work_contact_speed_limit_mps,
            payload_hex_radius=params.payload_hex_radius,
            payload_half_length=params.payload_half_length,
            module_gap=params.module_gap,
            left_center_offset_zero=left_offset_zero,
            right_center_offset_zero=right_offset_zero,
            hex_face_tilt_rad=params.hex_face_tilt_rad,
            nominal_attitude_rad=params.nominal_attitude_rad,
            rotational_damping=params.rotational_damping,
            passive_attitude_stiffness_Nm_rad=params.passive_attitude_stiffness_Nm_rad,
            passive_attitude_damping_Nm_s_rad=params.passive_attitude_damping_Nm_s_rad,
            motor_thrust_time_constant_s=params.motor_thrust_time_constant_s,
            max_thrust_per_drone=params.max_thrust_per_drone * params.mpc_thrust_command_fraction,
            thrust_command_slew_limit_N_s=params.thrust_command_slew_limit_N_s,
            max_gimbal_angle_rad=params.gimbal_max_angle_rad,
            max_gimbal_rate_rad_s=params.gimbal_max_rate_rad_s,
            gimbal_command_slew_limit_rad_s=params.gimbal_command_slew_limit_rad_s,
            max_gimbal_acceleration_rad_s2=params.gimbal_max_acceleration_rad_s2,
            gimbal_natural_frequency_rad_s=params.gimbal_natural_frequency_rad_s,
            gimbal_damping_ratio=params.gimbal_damping_ratio,
            max_cable_tension=params.max_spool_tension,
            min_tracking_tension=params.min_tracking_tension,
            max_cable_support_fraction=params.max_cable_support_fraction,
            desired_cable_support_fraction=params.desired_cable_support_fraction,
            min_cable_vertical_efficiency=params.min_cable_vertical_efficiency,
            min_cable_length=params.min_cable_length,
            max_cable_length=params.max_cable_length,
            max_spool_speed=params.max_spool_speed,
            reel_velocity_time_constant_s=params.reel_velocity_time_constant_s,
            reel_stall_line_force_N=self._reel_motor_spec().stall_line_force_N,
            reel_velocity_slew_limit_mps2=params.reel_velocity_slew_limit_mps2,
            cable_tension_rate_limit_N_s=params.cable_tension_rate_limit_N_s,
            cable_stiffness_N_m=params.cable_stiffness_N_m,
            cable_damping_N_s_m=params.cable_damping_N_s_m,
            cable_tension_time_constant_s=params.cable_tension_time_constant_s,
            max_cable_extension_m=params.max_cable_extension_m,
            cable_mass_per_length_kg_m=self._steel_cable_spec().mass_per_length_kg_m,
            cable_payload_weight_fraction=params.steel_cable_payload_weight_fraction,
            attitude_limit_rad=params.mpc_attitude_limit_rad,
            slack_limit_m=params.mpc_slack_limit_m,
            tracking_position_weight=params.mpc_tracking_position_weight,
            tracking_velocity_weight=params.mpc_tracking_velocity_weight,
            terminal_position_weight=params.mpc_terminal_position_weight,
            terminal_velocity_weight=params.mpc_terminal_velocity_weight,
            drone_effort_weight=params.mpc_drone_effort_weight,
            cable_effort_weight=params.mpc_cable_effort_weight,
            reel_speed_weight=params.mpc_reel_speed_weight,
            input_rate_weight=params.mpc_input_rate_weight,
            attitude_rate_weight=params.mpc_attitude_rate_weight,
            attitude_weight=params.mpc_attitude_weight,
            gimbal_angle_weight=params.mpc_gimbal_angle_weight,
            gimbal_rate_weight=params.mpc_gimbal_rate_weight,
            cable_support_weight=params.mpc_cable_support_weight,
            slack_weight=params.mpc_slack_weight,
            solver_max_iter=max(20, params.mpc_solver_max_iter),
            solver_tolerance=max(1e-8, params.mpc_solver_tolerance),
        )

    def _ensure_nmpc(self) -> WallToolNMPC:
        if self._nmpc is None:
            self._nmpc = WallToolNMPC(self._nmpc_config())
        return self._nmpc

    @staticmethod
    def _solve_small_least_squares(
        matrix: Sequence[Sequence[float]],
        rhs: Sequence[float],
    ) -> tuple[float, ...]:
        """Solve a one-, two-, or three-variable symmetric linear system."""
        size = len(rhs)
        if size == 1:
            if abs(float(matrix[0][0])) < 1e-12:
                raise ValueError("singular allocation system")
            return (float(rhs[0]) / float(matrix[0][0]),)
        if size == 2:
            a, b = float(matrix[0][0]), float(matrix[0][1])
            c, d = float(matrix[1][0]), float(matrix[1][1])
            determinant = a * d - b * c
            if abs(determinant) < 1e-12:
                raise ValueError("singular allocation system")
            return (
                (float(rhs[0]) * d - b * float(rhs[1])) / determinant,
                (a * float(rhs[1]) - float(rhs[0]) * c) / determinant,
            )
        if size == 3:
            return solve3(matrix, rhs)
        if size == 0:
            return ()
        raise ValueError("allocation system must contain at most three variables")

    def _bounded_cascade_allocation(
        self,
        matrix: Sequence[Sequence[float]],
        desired_wrench: Sequence[float],
    ) -> tuple[float, float, float]:
        """Active-set allocation for left thrust, right thrust, and tension."""
        params = self.params
        thrust_limit = params.max_thrust_per_drone * params.mpc_thrust_command_fraction
        lower = (0.0, 0.0, max(0.0, params.cascade_min_tension_N))
        upper = (
            thrust_limit,
            thrust_limit,
            min(params.max_spool_tension, params.cascade_max_tension_N),
        )
        weights = (1.0, 1.0, max(1e-6, params.cascade_torque_weight))
        tension_regularization = max(0.0, params.cascade_tension_regularization)
        nominal_tension = clamp(params.cascade_nominal_tension_N, lower[2], upper[2])
        best: tuple[float, tuple[float, float, float]] | None = None

        # -1 fixes a variable at its lower bound, 0 leaves it free, and +1
        # fixes it at its upper bound. With only three actuators, exhaustive
        # active-set enumeration is deterministic and inexpensive.
        for modes in itertools.product((-1, 0, 1), repeat=3):
            command = [
                lower[index] if mode < 0 else upper[index] if mode > 0 else 0.0
                for index, mode in enumerate(modes)
            ]
            free = [index for index, mode in enumerate(modes) if mode == 0]
            residual_rhs = [
                float(desired_wrench[row])
                - sum(
                    float(matrix[row][column]) * command[column]
                    for column in range(3)
                    if column not in free
                )
                for row in range(3)
            ]
            if free:
                normal_matrix = [
                    [
                        sum(
                            weights[row]
                            * float(matrix[row][column])
                            * float(matrix[row][other])
                            for row in range(3)
                        )
                        + (tension_regularization if column == other == 2 else 0.0)
                        for other in free
                    ]
                    for column in free
                ]
                normal_rhs = [
                    sum(
                        weights[row] * float(matrix[row][column]) * residual_rhs[row]
                        for row in range(3)
                    )
                    + (tension_regularization * nominal_tension if column == 2 else 0.0)
                    for column in free
                ]
                try:
                    free_solution = self._solve_small_least_squares(normal_matrix, normal_rhs)
                except ValueError:
                    continue
                for column, value in zip(free, free_solution):
                    command[column] = float(value)

            if any(
                command[index] < lower[index] - 1e-9
                or command[index] > upper[index] + 1e-9
                for index in range(3)
            ):
                continue
            wrench_error = [
                sum(float(matrix[row][column]) * command[column] for column in range(3))
                - float(desired_wrench[row])
                for row in range(3)
            ]
            cost = sum(weights[row] * wrench_error[row] ** 2 for row in range(3))
            cost += tension_regularization * (command[2] - nominal_tension) ** 2
            candidate = (cost, (command[0], command[1], command[2]))
            if best is None or candidate[0] < best[0]:
                best = candidate

        if best is None:
            hover = min(
                thrust_limit,
                params.total_mass * params.gravity
                / max(2.0 * math.cos(params.hex_face_tilt_rad), 1e-9),
            )
            return hover, hover, nominal_tension
        return best[1]

    def _sensor_cascade_command(
        self,
        reference: ReferenceState,
    ) -> tuple[float, float, float, float]:
        """Compute a command using only encoder, load-cell, and IMU estimates.

        With the default passive pitch guide, the outer loop requests only x-z
        force and uses a constant cable-tension setpoint for smooth allocation.
        The optional free-pitch model also requests pitch torque and uses the
        bounded three-actuator allocator.
        The inner load-cell PI loop converts that setpoint into reel velocity;
        the reel encoder closes the velocity loop in the motor drive.
        """
        params = self.params
        position_error = sub2(reference.position, self.measured_payload)
        velocity_error = sub2(reference.velocity, self.estimated_payload_velocity)
        desired_acceleration = limit_norm2(
            (
                reference.acceleration[0]
                + params.cascade_position_kp * position_error[0]
                + params.cascade_velocity_kd * velocity_error[0],
                reference.acceleration[1]
                + params.cascade_position_kp * position_error[1]
                + params.cascade_velocity_kd * velocity_error[1],
            ),
            params.cascade_acceleration_limit_m_s2,
        )

        cable_mount = self._cable_mount_position(self.measured_payload, self.measured_attitude)
        anchor_to_mount = sub2(cable_mount, params.anchor)
        distance = max(1e-6, math.hypot(anchor_to_mount[0], anchor_to_mount[1]))
        cable_out = (anchor_to_mount[0] / distance, anchor_to_mount[1] / distance)
        cable_axis = (-cable_out[0], -cable_out[1])
        allocation_attitude = (
            params.nominal_attitude_rad
            if params.payload_pitch_constrained
            else self.measured_attitude
        )
        left_axis, right_axis = self._drone_axes(
            allocation_attitude,
            self.estimated_left_gimbal_angle,
            self.estimated_right_gimbal_angle,
        )
        left_arm, right_arm = self._module_center_offsets(allocation_attitude)
        cable_arm = self._cable_mount_offset(self.measured_attitude)
        cable_weight_force = (0.0, -self._payload_supported_cable_weight(distance))
        desired_force = (
            params.total_mass * desired_acceleration[0] - cable_weight_force[0],
            params.total_mass * (desired_acceleration[1] + params.gravity)
            - cable_weight_force[1],
        )
        if params.payload_pitch_constrained:
            desired_tension = clamp(
                params.cascade_nominal_tension_N,
                params.cascade_min_tension_N,
                min(params.cascade_max_tension_N, params.max_spool_tension),
            )
            required_drone_force = (
                desired_force[0] - desired_tension * cable_axis[0],
                desired_force[1] - desired_tension * cable_axis[1],
            )
            determinant = left_axis[0] * right_axis[1] - right_axis[0] * left_axis[1]
            if abs(determinant) < 1e-9:
                hover = params.total_mass * params.gravity / max(
                    2.0 * math.cos(params.hex_face_tilt_rad), 1e-9
                )
                left_command = hover
                right_command = hover
            else:
                left_command = (
                    required_drone_force[0] * right_axis[1]
                    - required_drone_force[1] * right_axis[0]
                ) / determinant
                right_command = (
                    left_axis[0] * required_drone_force[1]
                    - left_axis[1] * required_drone_force[0]
                ) / determinant
            thrust_limit = params.max_thrust_per_drone * params.mpc_thrust_command_fraction
            left_command = clamp(left_command, 0.0, thrust_limit)
            right_command = clamp(right_command, 0.0, thrust_limit)
        else:
            cable_weight_torque = cross2(cable_arm, cable_weight_force)
            desired_angular_acceleration = (
                -params.cascade_attitude_kp
                * wrap_angle(self.measured_attitude - params.nominal_attitude_rad)
                - params.cascade_attitude_kd * self.measured_angular_velocity
            )
            desired_wrench = (
                desired_force[0],
                desired_force[1],
                params.assembly_inertia * desired_angular_acceleration
                + params.rotational_damping * self.measured_angular_velocity
                - cable_weight_torque,
            )
            allocation_matrix = (
                (left_axis[0], right_axis[0], cable_axis[0]),
                (left_axis[1], right_axis[1], cable_axis[1]),
                (
                    cross2(left_arm, left_axis),
                    cross2(right_arm, right_axis),
                    cross2(cable_arm, cable_axis),
                ),
            )
            left_command, right_command, desired_tension = self._bounded_cascade_allocation(
                allocation_matrix,
                desired_wrench,
            )

        mount_offset = cable_arm
        mount_velocity = add2(
            self.estimated_payload_velocity,
            scale2((-mount_offset[1], mount_offset[0]), self.measured_angular_velocity),
        )
        distance_rate = dot2(cable_out, mount_velocity)
        tension_error = self.measured_tension - desired_tension
        unsaturated_reel_command = (
            distance_rate
            + params.reel_tension_kp_mps_N * tension_error
            + params.reel_tension_ki_mps_Ns * self.reel_tension_error_integral
        )
        reel_limit = min(params.max_spool_speed, params.max_control_spool_speed)
        reel_command = clamp(unsaturated_reel_command, -reel_limit, reel_limit)
        # Conditional integration prevents the load-cell loop from winding up
        # while the reel is at its operational speed bound.
        driving_out_of_saturation = (
            abs(unsaturated_reel_command - reel_command) <= 1e-12
            or (reel_command >= reel_limit and tension_error < 0.0)
            or (reel_command <= -reel_limit and tension_error > 0.0)
        )
        if driving_out_of_saturation:
            self.reel_tension_error_integral = clamp(
                self.reel_tension_error_integral + tension_error * params.dt,
                -params.reel_tension_integral_limit_Ns,
                params.reel_tension_integral_limit_Ns,
            )
        return left_command, right_command, desired_tension, reel_command

    def _nmpc_reference_horizon(self, reference: ReferenceState) -> MPCReferenceHorizon:
        params = self.params
        steps = max(2, params.mpc_horizon_steps)
        dt = max(params.dt, params.mpc_horizon_dt)
        if not reference.active or not self.trajectory.segments:
            positions = tuple(reference.position for _ in range(steps + 1))
            velocities = tuple((0.0, 0.0) for _ in range(steps + 1))
            return MPCReferenceHorizon(positions=positions, velocities=velocities)

        samples = self.trajectory._path_samples()
        progress0 = self.trajectory.geometric_progress_m
        speed = params.path_speed
        slowdown_distance = self.trajectory._geometric_slowdown_distance(speed)
        positions: list[Vec2] = []
        velocities: list[Vec2] = []
        for index in range(steps + 1):
            progress = progress0 + speed * dt * index
            point, tangent, remaining = self.trajectory._sample_path_at_progress(samples, progress)
            speed_scale = clamp(remaining / max(slowdown_distance, 1e-9), 0.0, 1.0)
            target_speed = speed * speed_scale
            if remaining <= params.waypoint_tolerance:
                target_speed = 0.0
            positions.append(point)
            velocities.append(scale2(tangent, target_speed))
        positions[0] = reference.position
        velocities[0] = reference.velocity
        return MPCReferenceHorizon(positions=tuple(positions), velocities=tuple(velocities))

    def _step_tool_head_nmpc(self) -> SimState:
        params = self.params
        mass = params.total_mass
        self._update_cable_coordinates()
        self._update_sensor_estimate()
        reference = self._safe_reference(
            self.trajectory.geometric_reference(self.measured_payload, self.estimated_payload_velocity, params.dt)
        )
        if reference.active:
            self._hold_position_error_integral = (0.0, 0.0)
        else:
            hold_error = sub2(reference.position, self.measured_payload)
            self._hold_position_error_integral = (
                clamp(
                    self._hold_position_error_integral[0] + hold_error[0] * params.dt,
                    -params.mpc_hold_integral_limit_m_s,
                    params.mpc_hold_integral_limit_m_s,
                ),
                clamp(
                    self._hold_position_error_integral[1] + hold_error[1] * params.dt,
                    -params.mpc_hold_integral_limit_m_s,
                    params.mpc_hold_integral_limit_m_s,
                ),
            )
        if self.external_plant:
            self.contact_work_mode = self._contact_work_mode_for_reference(reference)
            self.desired_contact_force = params.desired_contact_force_N if self.contact_work_mode else 0.0
        else:
            self._update_normal_contact(reference)

        true_mount = self._cable_mount_position(self.position, self.attitude)
        true_distance = distance2(params.anchor, true_mount)
        measured_state = (
            self.measured_payload[0],
            self.measured_payload[1],
            self.estimated_payload_velocity[0],
            self.estimated_payload_velocity[1],
            self.measured_attitude,
            self.measured_angular_velocity,
            clamp(self.measured_cable_length, params.min_cable_length, params.max_cable_length),
            clamp(self.estimated_left_thrust, 0.0, params.max_thrust_per_drone),
            clamp(self.estimated_right_thrust, 0.0, params.max_thrust_per_drone),
            clamp(self.measured_cable_velocity, -params.max_spool_speed, params.max_spool_speed),
            clamp(self.measured_tension, 0.0, params.max_spool_tension),
            clamp(
                self.estimated_left_gimbal_angle,
                -params.gimbal_max_angle_rad,
                params.gimbal_max_angle_rad,
            ),
            clamp(
                self.estimated_right_gimbal_angle,
                -params.gimbal_max_angle_rad,
                params.gimbal_max_angle_rad,
            ),
            clamp(
                self.estimated_left_gimbal_rate,
                -params.gimbal_max_rate_rad_s,
                params.gimbal_max_rate_rad_s,
            ),
            clamp(
                self.estimated_right_gimbal_rate,
                -params.gimbal_max_rate_rad_s,
                params.gimbal_max_rate_rad_s,
            ),
        )

        if params.control_law != "vector_thrust_nmpc":
            raise RuntimeError(f"unsupported controller '{params.control_law}'")
        if self._nmpc_last_solution is None or self.t + 1e-12 >= self._nmpc_next_solve_t:
            horizon = self._nmpc_reference_horizon(reference)
            if not reference.active:
                hold_correction = scale2(
                    self._hold_position_error_integral,
                    params.mpc_hold_integral_gain_s_inv,
                )
                horizon = MPCReferenceHorizon(
                    positions=tuple(
                        self._clamp_wall_point(add2(position, hold_correction))
                        for position in horizon.positions
                    ),
                    velocities=horizon.velocities,
                )
            solution = self._ensure_nmpc().solve(
                measured_state=measured_state,
                reference=horizon,
                previous_command=self._nmpc_previous_command,
            )
            self._nmpc_last_solution = solution
            self._nmpc_next_solve_t = self.t + max(params.dt, params.mpc_control_period_s)
        else:
            solution = self._nmpc_last_solution

        if not solution.success:
            raise RuntimeError(
                f"vector-thrust NMPC returned an unsuccessful solution: {solution.status}"
            )

        command_values = (
            solution.left_thrust,
            solution.right_thrust,
            solution.cable_tension,
            solution.spool_velocity,
            solution.left_gimbal_angle,
            solution.right_gimbal_angle,
        )
        if not all(math.isfinite(value) for value in command_values):
            raise RuntimeError(f"NMPC produced a non-finite actuator command: {command_values!r}")
        thrust_limit = params.max_thrust_per_drone * params.mpc_thrust_command_fraction
        numerical_tolerance = 1.0e-6
        if not -numerical_tolerance <= solution.left_thrust <= thrust_limit + numerical_tolerance:
            raise RuntimeError(f"left thrust command violates hardware limit: {solution.left_thrust:.6f} N")
        if not -numerical_tolerance <= solution.right_thrust <= thrust_limit + numerical_tolerance:
            raise RuntimeError(f"right thrust command violates hardware limit: {solution.right_thrust:.6f} N")
        if abs(solution.spool_velocity) > params.max_spool_speed + numerical_tolerance:
            raise RuntimeError(
                f"reel velocity command violates hardware limit: {solution.spool_velocity:.6f} m/s"
            )
        if not -numerical_tolerance <= solution.cable_tension <= params.max_spool_tension + numerical_tolerance:
            raise RuntimeError(
                f"predicted cable tension is outside the load-cell range: {solution.cable_tension:.6f} N"
            )
        left_thrust_target = clamp(solution.left_thrust, 0.0, thrust_limit)
        right_thrust_target = clamp(solution.right_thrust, 0.0, thrust_limit)
        left_thrust_command = slew_toward(
            self.last_left_thrust_command,
            left_thrust_target,
            params.thrust_command_slew_limit_N_s,
            params.dt,
        )
        right_thrust_command = slew_toward(
            self.last_right_thrust_command,
            right_thrust_target,
            params.thrust_command_slew_limit_N_s,
            params.dt,
        )
        desired_cable_tension = clamp(solution.cable_tension, 0.0, params.max_spool_tension)
        spool_velocity_target = clamp(
            solution.spool_velocity,
            -params.max_spool_speed,
            params.max_spool_speed,
        )
        spool_velocity_command = slew_toward(
            self.last_spool_velocity_cmd,
            spool_velocity_target,
            params.reel_velocity_slew_limit_mps2,
            params.dt,
        )
        gimbal_servo = self._gimbal_servo_spec()
        left_gimbal_target = gimbal_servo.realize_pwm_command(solution.left_gimbal_angle)
        right_gimbal_target = gimbal_servo.realize_pwm_command(solution.right_gimbal_angle)
        left_gimbal_command = gimbal_servo.realize_pwm_command(slew_toward(
            self.last_left_gimbal_command,
            left_gimbal_target,
            params.gimbal_command_slew_limit_rad_s,
            params.dt,
        ))
        right_gimbal_command = gimbal_servo.realize_pwm_command(slew_toward(
            self.last_right_gimbal_command,
            right_gimbal_target,
            params.gimbal_command_slew_limit_rad_s,
            params.dt,
        ))
        actual_left_gimbal_command = clamp(
            left_gimbal_command + params.gimbal_left_zero_error_rad,
            -params.gimbal_max_angle_rad,
            params.gimbal_max_angle_rad,
        )
        actual_right_gimbal_command = clamp(
            right_gimbal_command + params.gimbal_right_zero_error_rad,
            -params.gimbal_max_angle_rad,
            params.gimbal_max_angle_rad,
        )

        reel_motor = self._reel_motor_spec()
        motor_alpha = clamp(
            params.dt / max(params.motor_thrust_time_constant_s + params.dt, 1e-9),
            0.0,
            1.0,
        )
        self.actual_left_thrust += motor_alpha * (left_thrust_command - self.actual_left_thrust)
        self.actual_right_thrust += motor_alpha * (right_thrust_command - self.actual_right_thrust)
        self.estimated_left_thrust += motor_alpha * (left_thrust_command - self.estimated_left_thrust)
        self.estimated_right_thrust += motor_alpha * (right_thrust_command - self.estimated_right_thrust)
        (
            self.actual_left_gimbal_angle,
            self.actual_left_gimbal_rate,
            self.left_gimbal_acceleration,
            self.left_gimbal_saturated,
        ) = gimbal_servo.step(
            self.actual_left_gimbal_angle,
            self.actual_left_gimbal_rate,
            actual_left_gimbal_command,
            params.dt,
        )
        (
            self.estimated_left_gimbal_angle,
            self.estimated_left_gimbal_rate,
            _,
            _,
        ) = gimbal_servo.step(
            self.estimated_left_gimbal_angle,
            self.estimated_left_gimbal_rate,
            left_gimbal_command,
            params.dt,
        )
        (
            self.estimated_right_gimbal_angle,
            self.estimated_right_gimbal_rate,
            _,
            _,
        ) = gimbal_servo.step(
            self.estimated_right_gimbal_angle,
            self.estimated_right_gimbal_rate,
            right_gimbal_command,
            params.dt,
        )
        (
            self.actual_right_gimbal_angle,
            self.actual_right_gimbal_rate,
            self.right_gimbal_acceleration,
            self.right_gimbal_saturated,
        ) = gimbal_servo.step(
            self.actual_right_gimbal_angle,
            self.actual_right_gimbal_rate,
            actual_right_gimbal_command,
            params.dt,
        )
        self.actual_reel_velocity = reel_motor.velocity_step(
            self.actual_reel_velocity,
            spool_velocity_command,
            self.measured_tension,
            params.dt,
        )
        self.actual_reel_velocity = clamp(
            self.actual_reel_velocity,
            -min(params.max_spool_speed, reel_motor.max_line_speed_m_s),
            min(params.max_spool_speed, reel_motor.max_line_speed_m_s),
        )
        previous_cable_length = self.cable_length
        self.cable_length = clamp(
            self.cable_length + self.actual_reel_velocity * params.dt,
            params.min_cable_length,
            params.max_cable_length,
        )
        self.actual_reel_velocity = (self.cable_length - previous_cable_length) / max(params.dt, 1e-9)
        left_thrust = self.actual_left_thrust
        right_thrust = self.actual_right_thrust
        spool_velocity = self.actual_reel_velocity
        self.last_left_thrust_command = left_thrust_command
        self.last_right_thrust_command = right_thrust_command
        self.last_left_gimbal_command = left_gimbal_command
        self.last_right_gimbal_command = right_gimbal_command
        self.last_spool_velocity_cmd = spool_velocity_command

        true_cable_arm = self._cable_mount_offset(self.attitude)
        true_mount_position = add2(self.position, true_cable_arm)
        true_mount_velocity = add2(self.velocity, scale2((-true_cable_arm[1], true_cable_arm[0]), self.angular_velocity))
        anchor_to_mount = sub2(true_mount_position, params.anchor)
        true_distance = max(1e-9, math.hypot(anchor_to_mount[0], anchor_to_mount[1]))
        true_cable_out = (anchor_to_mount[0] / true_distance, anchor_to_mount[1] / true_distance)
        true_cable_axis = (-true_cable_out[0], -true_cable_out[1])
        radial_mount_velocity = dot2(true_cable_out, true_mount_velocity)
        cable_extension = true_distance - self.cable_length
        cable_extension_rate = radial_mount_velocity - spool_velocity
        cable_stiffness = self._steel_cable_stiffness(true_distance)
        cable_damping = self._steel_cable_damping(true_distance)
        raw_tension = cable_stiffness * max(0.0, cable_extension) + cable_damping * cable_extension_rate
        actual_tension_limit = params.max_spool_tension
        self.cable_tension_saturated = raw_tension > actual_tension_limit
        if cable_extension >= -params.cable_taut_band:
            tension = clamp(raw_tension, 0.0, actual_tension_limit)
        else:
            tension = 0.0
        self.actual_tension = tension
        self.cable_stretch = max(0.0, cable_extension)
        self.cable_slack = cable_extension < -params.cable_taut_band or tension <= 1e-9
        cable_force = scale2(true_cable_axis, tension)

        true_left_axis, true_right_axis = self._drone_axes(
            self.attitude,
            self.actual_left_gimbal_angle,
            self.actual_right_gimbal_angle,
        )
        true_left_arm, true_right_arm = self._module_center_offsets(self.attitude)
        left_force = scale2(true_left_axis, left_thrust)
        right_force = scale2(true_right_axis, right_thrust)
        drone_force = add2(left_force, right_force)
        cable_torque = cross2(true_cable_arm, cable_force)
        left_torque = cross2(true_left_arm, left_force)
        right_torque = cross2(true_right_arm, right_force)
        cable_weight_force = (0.0, -self._payload_supported_cable_weight(true_distance))
        cable_weight_torque = cross2(true_cable_arm, cable_weight_force)
        passive_attitude_torque = (
            -params.passive_attitude_stiffness_Nm_rad
            * math.sin(self.attitude - params.nominal_attitude_rad)
            - params.passive_attitude_damping_Nm_s_rad * self.angular_velocity
        )
        net_attitude_torque = (
            cable_torque
            + left_torque
            + right_torque
            + cable_weight_torque
            - params.rotational_damping * self.angular_velocity
            + passive_attitude_torque
        )
        gravity_force = (0.0, -mass * params.gravity)
        wind_force = self._wind_force()
        net_force = add2(add2(add2(add2(drone_force, cable_force), gravity_force), wind_force), cable_weight_force)
        self.acceleration = scale2(net_force, 1.0 / mass)
        self.velocity = add2(self.velocity, scale2(self.acceleration, params.dt))
        self.position = add2(self.position, scale2(self.velocity, params.dt))
        if params.payload_pitch_constrained:
            raise RuntimeError("ideal payload pitch constraints are disabled for the inspection model")
        self.angular_acceleration = net_attitude_torque / max(params.assembly_inertia, 1e-9)
        self.angular_velocity += self.angular_acceleration * params.dt
        self.attitude = wrap_angle(self.attitude + self.angular_velocity * params.dt)
        self.t += params.dt
        self._update_cable_coordinates()
        self._update_sensor_estimate()
        self._nmpc_previous_command = (
            left_thrust_command,
            right_thrust_command,
            spool_velocity_command,
            left_gimbal_command,
            right_gimbal_command,
        )

        control_tangential_axis = (math.cos(self.theta), math.sin(self.theta))
        tangential_force = dot2(drone_force, control_tangential_axis)
        desired_left_axis, desired_right_axis = self._drone_axes(
            self.attitude,
            left_gimbal_command,
            right_gimbal_command,
        )
        desired_left_force = scale2(desired_left_axis, left_thrust_command)
        desired_right_force = scale2(desired_right_axis, right_thrust_command)
        desired_drone_force = add2(desired_left_force, desired_right_force)
        speed_error = sub2(reference.velocity, self.estimated_payload_velocity)
        position_error = sub2(reference.position, self.measured_payload)
        swing_energy = 0.5 * mass * (
            dot2(speed_error, speed_error)
            + dot2(position_error, position_error)
        )
        state = self.snapshot(
            left_thrust,
            right_thrust,
            tension,
            tangential_force,
            spool_velocity_command,
            math.hypot(desired_drone_force[0], desired_drone_force[1]) / max(mass, 1e-9),
            desired_cable_tension,
            desired_drone_force,
            drone_force,
            cable_force,
            wind_force,
            self.cable_tension_saturated
            or self.left_gimbal_saturated
            or self.right_gimbal_saturated,
            reference,
            desired_tangential_force=tangential_force,
            desired_attitude_torque=0.0,
            attitude_torque=net_attitude_torque,
            cable_torque=cable_torque,
            left_torque=left_torque,
            right_torque=right_torque,
            allocation_residual=0.0,
            radial_position_error_m=position_error[1],
            radial_velocity_error_m_s=speed_error[1],
            tangential_position_error_m=position_error[0],
            tangential_velocity_error_m_s=speed_error[0],
            swing_energy_J=swing_energy,
            mpc_solution=solution,
        )
        self.history.append(state)
        if len(self.history) > 6000:
            self.history = self.history[-6000:]
        return state

    def step(self) -> SimState:
        return self._step_tool_head_nmpc()

    def snapshot(
        self,
        left_thrust: float,
        right_thrust: float,
        tension: float,
        tangential_force: float,
        spool_velocity_cmd: float,
        drone_accel_cmd: float,
        desired_cable_tension: float,
        desired_drone_force: Vec2,
        drone_force: Vec2,
        cable_force: Vec2,
        wind_force: Vec2,
        saturated: bool,
        reference: ReferenceState,
        desired_tangential_force: float = 0.0,
        desired_attitude_torque: float = 0.0,
        attitude_torque: float = 0.0,
        cable_torque: float = 0.0,
        left_torque: float = 0.0,
        right_torque: float = 0.0,
        allocation_residual: float | None = None,
        radial_position_error_m: float = 0.0,
        radial_velocity_error_m_s: float = 0.0,
        tangential_position_error_m: float = 0.0,
        tangential_velocity_error_m_s: float = 0.0,
        swing_energy_J: float = 0.0,
        swing_power_W: float = 0.0,
        clf_margin_W: float = 0.0,
        clf_projected_accel_m_s2: float = 0.0,
        mpc_solution: MPCSolution | None = None,
    ) -> SimState:
        payload = self._payload_from_state()
        tool_head = payload
        desired_tool_head = reference.position
        return SimState(
            t=self.t,
            theta=self.theta,
            theta_dot=self.theta_dot,
            length=self.length,
            length_dot=self.length_dot,
            length_ddot=self.length_ddot,
            attitude=self.attitude,
            angular_velocity=self.angular_velocity,
            angular_acceleration=self.angular_acceleration,
            cable_length=self.cable_length,
            cable_stretch=self.cable_stretch,
            cable_slack=self.cable_slack,
            cable_tension_saturated=self.cable_tension_saturated,
            payload_velocity=self.velocity,
            payload_acceleration=self.acceleration,
            payload=payload,
            measured_payload=self.measured_payload,
            estimated_payload_velocity=self.estimated_payload_velocity,
            measured_theta=self.measured_theta,
            measured_theta_dot=self.measured_theta_dot,
            measured_line_length=self.measured_line_length,
            measured_attitude=self.measured_attitude,
            measured_angular_velocity=self.measured_angular_velocity,
            measured_cable_velocity=self.measured_cable_velocity,
            tool_head=tool_head,
            reference=reference.position,
            desired_tool_head=desired_tool_head,
            reference_velocity=reference.velocity,
            reference_acceleration=reference.acceleration,
            target=reference.final_target,
            active_target=reference.active_target,
            measured_tool_error=distance2(self.measured_payload, desired_tool_head),
            spool_velocity_cmd=spool_velocity_cmd,
            drone_accel_cmd=drone_accel_cmd,
            desired_cable_tension=desired_cable_tension,
            measured_cable_length=self.measured_cable_length,
            measured_tension=self.measured_tension,
            desired_drone_force=desired_drone_force,
            drone_force=drone_force,
            cable_force=cable_force,
            wind_force=wind_force,
            normal_gap=self.normal_gap,
            normal_velocity=self.normal_velocity,
            normal_acceleration=self.normal_acceleration,
            normal_actuator_force=self.normal_actuator_force,
            normal_wind_force=self.normal_wind_force,
            contact_force=self.contact_force,
            desired_contact_force=self.desired_contact_force,
            contact_valid=False,
            inspection_valid=(
                distance2(payload, desired_tool_head) <= self.params.work_contact_tracking_limit_m
                and math.hypot(self.velocity[0], self.velocity[1])
                <= self.params.work_contact_speed_limit_mps
                and abs(self.attitude - self.params.nominal_attitude_rad)
                <= self.params.inspection_attitude_limit_rad
                and abs(self.angular_velocity)
                <= self.params.work_contact_angular_rate_limit_rad_s
            ),
            work_mode=self.contact_work_mode,
            desired_attitude_torque=desired_attitude_torque,
            attitude_torque=attitude_torque,
            cable_torque=cable_torque,
            left_torque=left_torque,
            right_torque=right_torque,
            left_thrust=left_thrust,
            right_thrust=right_thrust,
            left_gimbal_angle=self.actual_left_gimbal_angle,
            right_gimbal_angle=self.actual_right_gimbal_angle,
            left_gimbal_rate=self.actual_left_gimbal_rate,
            right_gimbal_rate=self.actual_right_gimbal_rate,
            left_gimbal_angle_command=self.last_left_gimbal_command,
            right_gimbal_angle_command=self.last_right_gimbal_command,
            estimated_left_gimbal_angle=self.estimated_left_gimbal_angle,
            estimated_right_gimbal_angle=self.estimated_right_gimbal_angle,
            estimated_left_gimbal_rate=self.estimated_left_gimbal_rate,
            estimated_right_gimbal_rate=self.estimated_right_gimbal_rate,
            tension=tension,
            tangential_force=tangential_force,
            desired_tangential_force=desired_tangential_force,
            allocation_residual=distance2(drone_force, desired_drone_force)
            if allocation_residual is None
            else allocation_residual,
            drone_vertical_force=max(0.0, drone_force[1]),
            cable_vertical_force=max(0.0, cable_force[1]),
            path_error=distance2(tool_head, desired_tool_head),
            tool_error=distance2(tool_head, desired_tool_head),
            active_waypoints=reference.waypoint_count,
            saturated=saturated,
            radial_position_error_m=radial_position_error_m,
            radial_velocity_error_m_s=radial_velocity_error_m_s,
            tangential_position_error_m=tangential_position_error_m,
            tangential_velocity_error_m_s=tangential_velocity_error_m_s,
            swing_energy_J=swing_energy_J,
            swing_power_W=swing_power_W,
            clf_margin_W=clf_margin_W,
            clf_projected_accel_m_s2=clf_projected_accel_m_s2,
            mpc_predicted_path=mpc_solution.predicted_positions if mpc_solution is not None else (),
            mpc_predicted_attitudes=mpc_solution.predicted_attitudes if mpc_solution is not None else (),
            mpc_predicted_left_gimbal_angles=(
                mpc_solution.predicted_left_gimbal_angles if mpc_solution is not None else ()
            ),
            mpc_predicted_right_gimbal_angles=(
                mpc_solution.predicted_right_gimbal_angles if mpc_solution is not None else ()
            ),
            mpc_predicted_tensions=mpc_solution.predicted_tensions if mpc_solution is not None else (),
            mpc_predicted_spool_speeds=mpc_solution.predicted_spool_speeds if mpc_solution is not None else (),
            mpc_status=mpc_solution.status if mpc_solution is not None else "",
            mpc_solve_time_s=mpc_solution.solve_time_s if mpc_solution is not None else 0.0,
            mpc_objective=mpc_solution.objective if mpc_solution is not None else 0.0,
        )


class IntegratedToolArtist:
    def __init__(self, ax, params: SimParams, zorder: int) -> None:
        self.params = params
        self.body = Polygon(
            [(0.0, 0.0)] * 4,
            closed=True,
            facecolor="#f2cc60",
            edgecolor="#5c4512",
            linewidth=1.5,
            alpha=1.0,
            zorder=zorder,
        )
        self.left_motor = Polygon(
            [(0.0, 0.0)] * 4,
            closed=True,
            facecolor="#f7f7f7",
            edgecolor="#111111",
            linewidth=1.3,
            alpha=1.0,
            zorder=zorder + 2,
        )
        self.right_motor = Polygon(
            [(0.0, 0.0)] * 4,
            closed=True,
            facecolor="#f7f7f7",
            edgecolor="#111111",
            linewidth=1.3,
            alpha=1.0,
            zorder=zorder + 2,
        )
        self.tool_pad = Circle(
            (0.0, 0.0),
            params.payload_hex_radius * 0.34,
            facecolor="#8a4f00",
            edgecolor="#4b2f05",
            linewidth=1.1,
            zorder=zorder + 4,
        )
        self.cable_tab = Polygon(
            [(0.0, 0.0)] * 4,
            closed=True,
            facecolor="#d8b247",
            edgecolor="#5c4512",
            linewidth=1.0,
            zorder=zorder + 3,
        )
        self.left_nozzle, = ax.plot([], [], color="#111111", linewidth=1.8, solid_capstyle="round", zorder=zorder + 4)
        self.right_nozzle, = ax.plot([], [], color="#111111", linewidth=1.8, solid_capstyle="round", zorder=zorder + 4)
        for patch in (self.body, self.left_motor, self.right_motor, self.tool_pad, self.cable_tab):
            ax.add_patch(patch)

    def update(
        self,
        center: Vec2,
        attitude: float,
        left_gimbal_angle: float,
        right_gimbal_angle: float,
        left_motor_center: Vec2 | None = None,
        right_motor_center: Vec2 | None = None,
    ) -> None:
        params = self.params
        if left_motor_center is None or right_motor_center is None:
            left_motor_center, right_motor_center = integrated_motor_centers(params, center, attitude)
        left_axis, right_axis = integrated_motor_axes(
            params,
            attitude,
            left_gimbal_angle,
            right_gimbal_angle,
        )
        body_half_length = max(
            params.payload_half_length,
            distance2(center, left_motor_center) + 0.09,
            distance2(center, right_motor_center) + 0.09,
        )
        body_half_width = max(params.payload_hex_radius * 0.72, params.cage_radius * 0.33)
        self.body.set_xy(oriented_box_polygon(center, body_half_length, body_half_width, attitude))
        self.left_motor.set_xy(oriented_box_polygon(left_motor_center, params.cage_radius * 0.42, params.cage_radius * 0.22, math.atan2(left_axis[1], left_axis[0])))
        self.right_motor.set_xy(oriented_box_polygon(right_motor_center, params.cage_radius * 0.42, params.cage_radius * 0.22, math.atan2(right_axis[1], right_axis[0])))
        self.tool_pad.center = center
        self.cable_tab.set_xy(
            oriented_box_polygon(
                add2(center, rotate2((0.0, params.payload_hex_radius * 0.78), attitude)),
                params.payload_hex_radius * 0.28,
                params.payload_hex_radius * 0.15,
                attitude,
            )
        )
        left_tip = add2(left_motor_center, scale2(left_axis, params.cage_radius * 0.50))
        right_tip = add2(right_motor_center, scale2(right_axis, params.cage_radius * 0.50))
        self.left_nozzle.set_data([left_motor_center[0], left_tip[0]], [left_motor_center[1], left_tip[1]])
        self.right_nozzle.set_data([right_motor_center[0], right_tip[0]], [right_motor_center[1], right_tip[1]])


class WallToolApp:
    def __init__(self, simulator: WallToolSimulator, planner: str = PLANNER_DIRECT) -> None:
        self.sim = simulator
        self.params = simulator.params
        self.planner = planner
        self.playing = True
        self.show_trace = True
        self.show_target = True
        self.show_path = True
        self.show_forces = True
        self.append_mode = False
        self.draw_mode = False
        self.is_drawing = False
        self.draw_points: list[Vec2] = []
        self.draw_min_spacing = 0.055
        self.draw_max_points = 28
        self.live_window_s = 16.0
        self._last_frame_wall_time = time.perf_counter()

        self.fig = plt.figure(figsize=(15.0, 9.2), constrained_layout=False)
        grid = self.fig.add_gridspec(
            2,
            2,
            width_ratios=[1.0, 0.54],
            height_ratios=[1.0, 0.18],
            left=0.055,
            right=0.975,
            bottom=0.08,
            top=0.92,
            wspace=0.08,
            hspace=0.18,
        )
        self.ax = self.fig.add_subplot(grid[0, 0])
        panel_grid = grid[0, 1].subgridspec(
            5,
            1,
            height_ratios=[0.38, 0.22, 0.22, 0.22, 0.22],
            hspace=0.42,
        )
        self.panel_ax = self.fig.add_subplot(panel_grid[0])
        self.task_ax = self.fig.add_subplot(panel_grid[1])
        self.smooth_ax = self.fig.add_subplot(panel_grid[2])
        self.cable_ax = self.fig.add_subplot(panel_grid[3])
        self.reel_ax = self.fig.add_subplot(panel_grid[4])
        self.control_ax = self.fig.add_subplot(grid[1, :])
        self.control_ax.axis("off")
        self.panel_ax.axis("off")
        self.fig.suptitle("PRISMS Cable-Suspended Wall Tool Simulator", fontsize=14)

        self._build_scene()
        self._build_panel()
        self._build_live_plots()
        self._build_controls()
        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.draw()

    def _build_scene(self) -> None:
        params = self.params
        self.ax.set_aspect("equal", adjustable="box")
        margin = 0.35
        self.ax.set_xlim(-params.wall_width / 2.0 - margin, params.wall_width / 2.0 + margin)
        self.ax.set_ylim(-0.10, params.wall_height + 0.35)
        self.ax.set_xlabel("wall x [m]")
        self.ax.set_ylabel("wall z [m]")

        self.wall = Rectangle(
            (-params.wall_width / 2.0, 0.0),
            params.wall_width,
            params.wall_height,
            facecolor="#f3f1ea",
            edgecolor="#6d6a62",
            linewidth=2.0,
        )
        self.ax.add_patch(self.wall)
        if params.contact_work_enabled:
            self.work_region_patch = Rectangle(
                (params.contact_work_x_min, params.contact_work_z_min),
                params.contact_work_x_max - params.contact_work_x_min,
                params.contact_work_z_max - params.contact_work_z_min,
                facecolor="#ffffff",
                edgecolor="#2f6f4e",
                linewidth=1.8,
                linestyle="-",
                alpha=0.30,
                zorder=1,
            )
            self.ax.add_patch(self.work_region_patch)
            self.ax.text(
                params.contact_work_x_min,
                params.contact_work_z_max + 0.06,
                "inspection bay",
                color="#2f6f4e",
                fontsize=8.5,
                va="bottom",
            )
        self.ax.grid(True, color="#d8d4c9", linewidth=0.8)

        self.spool = Circle(params.anchor, 0.075, facecolor="#444444", edgecolor="black", zorder=5)
        self.ax.add_patch(self.spool)
        self.ax.text(params.anchor[0], params.anchor[1] + 0.13, "anchor + spool", ha="center", fontsize=9)

        self.cable_line, = self.ax.plot([], [], color="#222222", linewidth=2.0, zorder=3)
        self.trace_line, = self.ax.plot([], [], color="#2b7a78", linewidth=2.0, alpha=0.80, zorder=2)
        self.desired_trace_line, = self.ax.plot([], [], color="#8a5b22", linewidth=1.8, linestyle=":", alpha=0.90, zorder=2)
        self.path_line, = self.ax.plot([], [], color="#555555", linewidth=1.5, linestyle="--", alpha=0.72, zorder=4)
        self.mpc_prediction_line, = self.ax.plot([], [], color="#6b46c1", linewidth=1.7, linestyle="-.", alpha=0.90, zorder=6)
        self.draw_preview_line, = self.ax.plot([], [], color="#f39c12", linewidth=2.0, alpha=0.85, zorder=8)
        self.structure_line, = self.ax.plot([], [], color="#4a4a4a", linewidth=2.2, alpha=0.55, zorder=5)
        self.cable_mount_point, = self.ax.plot([], [], marker="o", linestyle="none", color="#111111", markersize=4.2, zorder=13)
        self.attitude_line, = self.ax.plot([], [], color="#111111", linewidth=1.6, alpha=0.85, zorder=13)
        self.reference_point, = self.ax.plot([], [], marker="o", color="#1f77b4", markersize=5.0, zorder=9, visible=False)
        self.waypoint_points, = self.ax.plot([], [], marker="o", linestyle="none", color="#8a5b22", markersize=3.0, alpha=0.45, zorder=9)
        self.target_point, = self.ax.plot(
            [],
            [],
            marker="o",
            linestyle="none",
            markerfacecolor="none",
            markeredgecolor="#8a5b22",
            markersize=8.0,
            mew=1.8,
            zorder=9,
        )
        self.tool_line, = self.ax.plot([], [], marker="o", linestyle="none", color="#8a4f00", markersize=6.0, zorder=13)

        self.tool_artist = IntegratedToolArtist(self.ax, params, 6)

        self.left_axis_guide, = self.ax.plot([], [], color="#777777", linestyle="--", linewidth=1.0, zorder=10)
        self.right_axis_guide, = self.ax.plot([], [], color="#777777", linestyle="--", linewidth=1.0, zorder=10)
        self.left_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=14, color="#1f77b4", zorder=12)
        self.right_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=14, color="#1f77b4", zorder=12)
        self.gravity_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=15, color="#333333", zorder=12)
        self.tension_arrow = FancyArrowPatch((0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=14, color="#6a3d9a", zorder=12)
        for arrow in (self.left_arrow, self.right_arrow, self.gravity_arrow, self.tension_arrow):
            self.ax.add_patch(arrow)

    def _build_panel(self) -> None:
        self.panel_ax.text(0.0, 0.98, "Mission Monitor", fontsize=12, fontweight="bold", va="top")
        self.state_text = self.panel_ax.text(
            0.0,
            0.88,
            "",
            fontsize=7.0,
            family="monospace",
            va="top",
            linespacing=1.10,
            clip_on=True,
        )

    def _build_live_plots(self) -> None:
        self.task_error_line, = self.task_ax.plot([], [], color="#111111", linewidth=1.5, label="error [m]")
        self._format_live_axis(self.task_ax, "Tracking Error", "m")
        self.task_ax.tick_params(labelbottom=False)

        self.smooth_body_line, = self.smooth_ax.plot([], [], color="#c05621", linewidth=1.25, label="body angle [deg]")
        self.smooth_cable_rate_line, = self.smooth_ax.plot([], [], color="#2b6cb0", linewidth=1.1, label="cable angle [deg]")
        self._format_live_axis(self.smooth_ax, "Body And Cable Angle", "deg")
        self.smooth_ax.tick_params(labelbottom=False)

        self.support_line, = self.cable_ax.plot([], [], color="#2f855a", linewidth=1.35, label="tension [N]")
        self.power_line, = self.cable_ax.plot([], [], color="#111111", linewidth=1.0, linestyle="--", label="desired [N]")
        self.thrust_fraction_line, = self.cable_ax.plot([], [], color="#6b46c1", linewidth=1.2, label="vertical support [N]")
        self._format_live_axis(self.cable_ax, "Cable Tension", "N")
        self.cable_ax.tick_params(labelbottom=False)

        self.spool_velocity_ratio_line, = self.reel_ax.plot([], [], color="#2b6cb0", linewidth=1.2, label="reel velocity [m/s]")
        self._format_live_axis(self.reel_ax, "Reel Velocity", "m/s")
        self.reel_ax.set_xlabel("time [s]", fontsize=7.8)

    @staticmethod
    def _format_live_axis(ax, title: str, ylabel: str) -> None:
        ax.set_title(title, fontsize=9.2)
        ax.set_ylabel(ylabel, fontsize=7.8)
        ax.grid(True, color="#dddddd", linewidth=0.7)
        ax.legend(loc="upper right", fontsize=5.9, framealpha=0.90, ncol=2)
        ax.tick_params(axis="both", labelsize=7.2)

    def _build_controls(self) -> None:
        self.play_ax = self.fig.add_axes([0.055, 0.026, 0.080, 0.038])
        self.reset_ax = self.fig.add_axes([0.145, 0.026, 0.070, 0.038])
        self.clear_ax = self.fig.add_axes([0.225, 0.026, 0.070, 0.038])
        self.append_ax = self.fig.add_axes([0.305, 0.026, 0.105, 0.038])
        self.draw_ax = self.fig.add_axes([0.420, 0.026, 0.090, 0.038])
        self.speed_ax = self.fig.add_axes([0.565, 0.035, 0.175, 0.024])
        self.trace_ax = self.fig.add_axes([0.760, 0.052, 0.078, 0.034])
        self.target_ax = self.fig.add_axes([0.844, 0.052, 0.078, 0.034])
        self.path_ax = self.fig.add_axes([0.760, 0.012, 0.078, 0.034])
        self.forces_ax = self.fig.add_axes([0.844, 0.012, 0.078, 0.034])

        self.play_button = Button(self.play_ax, "Pause")
        self.reset_button = Button(self.reset_ax, "Reset")
        self.clear_button = Button(self.clear_ax, "Clear")
        self.append_button = Button(self.append_ax, "Append Off")
        self.draw_button = Button(self.draw_ax, "Draw Off")
        self.speed_slider = Slider(self.speed_ax, "speed", 0.25, 4.0, valinit=1.0)
        self.trace_button = Button(self.trace_ax, "Trace On")
        self.target_button = Button(self.target_ax, "Target On")
        self.path_button = Button(self.path_ax, "Path On")
        self.forces_button = Button(self.forces_ax, "Forces On")

        self.play_button.on_clicked(self.toggle_play)
        self.reset_button.on_clicked(self.reset)
        self.clear_button.on_clicked(self.clear_trace)
        self.append_button.on_clicked(self.toggle_append)
        self.draw_button.on_clicked(self.toggle_draw)
        self.trace_button.on_clicked(lambda _event: self.toggle_layer("trace"))
        self.target_button.on_clicked(lambda _event: self.toggle_layer("target"))
        self.path_button.on_clicked(lambda _event: self.toggle_layer("path"))
        self.forces_button.on_clicked(lambda _event: self.toggle_layer("forces"))

    def module_centers(self, payload: Vec2, attitude: float) -> tuple[Vec2, Vec2]:
        return integrated_motor_centers(self.params, payload, attitude)

    def draw(self) -> None:
        state = self.sim.history[-1]
        params = self.params
        x, z = state.payload
        radius = params.cage_radius

        if self.show_trace:
            self.trace_line.set_data(
                [sample.tool_head[0] for sample in self.sim.history],
                [sample.tool_head[1] for sample in self.sim.history],
            )
            self.desired_trace_line.set_data(
                [sample.desired_tool_head[0] for sample in self.sim.history],
                [sample.desired_tool_head[1] for sample in self.sim.history],
            )
        else:
            self.trace_line.set_data([], [])
            self.desired_trace_line.set_data([], [])
        pending_path = self.sim.trajectory.pending_path()
        if self.show_path and len(pending_path) >= 1:
            self.path_line.set_data([point[0] for point in pending_path], [point[1] for point in pending_path])
            self.reference_point.set_data([], [])
            self.waypoint_points.set_data([], [])
        else:
            self.path_line.set_data([], [])
            self.reference_point.set_data([], [])
            self.waypoint_points.set_data([], [])
        if state.mpc_predicted_path:
            self.mpc_prediction_line.set_data(
                [point[0] for point in state.mpc_predicted_path],
                [point[1] for point in state.mpc_predicted_path],
            )
        else:
            self.mpc_prediction_line.set_data([], [])
        if self.show_target:
            self.target_point.set_data([state.target[0]], [state.target[1]])
        else:
            self.target_point.set_data([], [])
        if self.draw_points:
            self.draw_preview_line.set_data([point[0] for point in self.draw_points], [point[1] for point in self.draw_points])
        else:
            self.draw_preview_line.set_data([], [])

        attitude = state.attitude
        left_center, right_center = self.module_centers(state.payload, attitude)
        self.tool_artist.update(
            state.payload,
            attitude,
            state.left_gimbal_angle,
            state.right_gimbal_angle,
            left_center,
            right_center,
        )
        self.structure_line.set_data(
            [left_center[0], state.payload[0], right_center[0]],
            [left_center[1], state.payload[1], right_center[1]],
        )

        cable_mount = add2(state.payload, rotate2((0.0, params.payload_hex_radius), attitude))
        self.cable_line.set_data([params.anchor[0], cable_mount[0]], [params.anchor[1], cable_mount[1]])
        self.cable_line.set_linestyle("--" if state.cable_slack else "-")
        self.cable_mount_point.set_data([cable_mount[0]], [cable_mount[1]])
        attitude_tip = add2(state.payload, rotate2((0.0, params.payload_hex_radius * 1.35), attitude))
        self.attitude_line.set_data([state.payload[0], attitude_tip[0]], [state.payload[1], attitude_tip[1]])
        self.tool_line.set_data([state.tool_head[0]], [state.tool_head[1]])

        left_axis, right_axis = self.sim._drone_axes(
            attitude,
            state.left_gimbal_angle,
            state.right_gimbal_angle,
        )
        guide_length = 0.30
        self.left_axis_guide.set_data(
            [left_center[0], left_center[0] + left_axis[0] * guide_length],
            [left_center[1], left_center[1] + left_axis[1] * guide_length],
        )
        self.right_axis_guide.set_data(
            [right_center[0], right_center[0] + right_axis[0] * guide_length],
            [right_center[1], right_center[1] + right_axis[1] * guide_length],
        )

        force_scale = 0.25 / params.max_thrust_per_drone
        self._set_arrow(self.left_arrow, left_center, add2(left_center, scale2(left_axis, 0.035 + force_scale * state.left_thrust)))
        self._set_arrow(self.right_arrow, right_center, add2(right_center, scale2(right_axis, 0.035 + force_scale * state.right_thrust)))
        self._set_arrow(self.gravity_arrow, state.payload, (x, z - 0.20))
        cable_direction = normalize2((params.anchor[0] - x, params.anchor[1] - z))
        self._set_arrow(self.tension_arrow, state.payload, add2(state.payload, scale2(cable_direction, 0.18)))
        for arrow in (self.left_arrow, self.right_arrow, self.gravity_arrow, self.tension_arrow):
            arrow.set_visible(self.show_forces)
        self.left_axis_guide.set_visible(self.show_forces)
        self.right_axis_guide.set_visible(self.show_forces)

        self.state_text.set_text(self._efficiency_text(state))
        self._update_live_plots()

    def _efficiency_text(self, state: SimState) -> str:
        params = self.params
        weight = max(params.total_mass * params.gravity, 1e-9)
        speed = math.hypot(state.payload_velocity[0], state.payload_velocity[1])
        acceleration = math.hypot(state.payload_acceleration[0], state.payload_acceleration[1])
        cable_support = state.cable_vertical_force / weight
        controller_state = "OK"
        if state.cable_slack:
            controller_state = "SLACK"
        elif state.saturated:
            controller_state = "LIMIT"
        window_text = self._window_metrics_text()
        return (
            f"t {state.t:6.1f}s  wp {state.active_waypoints:2d}  {controller_state}\n"
            f"tracking {state.tool_error:5.3f}m  speed {speed:4.2f}m/s  accel {acceleration:4.2f}m/s^2\n"
            f"gimbals {math.degrees(state.left_gimbal_angle):+5.1f}/{math.degrees(state.right_gimbal_angle):+5.1f}deg  "
            f"tension {state.measured_tension:4.2f}N  support {100.0 * cable_support:4.0f}%\n"
            f"inputs L {state.left_thrust:4.2f}N  R {state.right_thrust:4.2f}N  reel {state.spool_velocity_cmd:+5.3f}m/s\n"
            f"mpc {1000.0 * state.mpc_solve_time_s:4.1f}ms  {state.mpc_status[:22]}\n"
            f"{window_text}"
        )

    def _window_metrics_text(self) -> str:
        if not self.sim.history:
            return ""
        latest_t = self.sim.history[-1].t
        start_t = max(0.0, latest_t - self.live_window_s)
        samples = [sample for sample in self.sim.history if sample.t >= start_t]
        if not samples:
            return ""
        params = self.params
        weight = max(params.total_mass * params.gravity, 1e-9)
        no_cable_hover_each = weight / max(2.0 * math.cos(params.hex_face_tilt_rad), 1e-9)
        no_cable_power_index = max(2.0 * no_cable_hover_each**1.5, 1e-9)
        rms_error = math.sqrt(sum(sample.tool_error * sample.tool_error for sample in samples) / len(samples))
        cable_support = sum(sample.cable_vertical_force / weight for sample in samples) / len(samples)
        drone_power = sum((sample.left_thrust**1.5 + sample.right_thrust**1.5) / no_cable_power_index for sample in samples) / len(samples)
        max_thrust = max(max(sample.left_thrust, sample.right_thrust) / max(params.max_thrust_per_drone, 1e-9) for sample in samples)
        body_rates = sorted(abs(sample.angular_velocity) for sample in samples)
        p95_body = body_rates[int(0.95 * (len(body_rates) - 1))] if body_rates else 0.0
        jerks: list[float] = []
        for index in range(1, len(samples)):
            dt = max(samples[index].t - samples[index - 1].t, 1e-9)
            da = sub2(samples[index].payload_acceleration, samples[index - 1].payload_acceleration)
            jerks.append(math.hypot(da[0], da[1]) / dt)
        sorted_jerks = sorted(jerks)
        p95_jerk = sorted_jerks[int(0.95 * (len(sorted_jerks) - 1))] if sorted_jerks else 0.0
        return (
            f"last {self.live_window_s:.0f}s rms {rms_error:5.3f}m\n"
            f"p95 body {p95_body:4.2f}rad/s  jerk {p95_jerk:4.1f}\n"
            f"avg sup {100.0 * cable_support:4.0f}%  power {100.0 * drone_power:4.0f}%  peak {100.0 * max_thrust:4.0f}%"
        )

    def _update_live_plots(self) -> None:
        if not self.sim.history:
            return
        latest_t = self.sim.history[-1].t
        start_t = max(0.0, latest_t - self.live_window_s)
        samples = [sample for sample in self.sim.history if sample.t >= start_t]
        times = [sample.t for sample in samples]

        tracking_error = [sample.tool_error for sample in samples]
        self.task_error_line.set_data(times, tracking_error)

        body_angle = [math.degrees(sample.attitude) for sample in samples]
        cable_angle = [math.degrees(sample.theta) for sample in samples]
        self.smooth_body_line.set_data(times, body_angle)
        self.smooth_cable_rate_line.set_data(times, cable_angle)

        tension = [sample.measured_tension for sample in samples]
        desired_tension = [sample.desired_cable_tension for sample in samples]
        vertical_support = [sample.cable_vertical_force for sample in samples]
        self.support_line.set_data(times, tension)
        self.power_line.set_data(times, desired_tension)
        self.thrust_fraction_line.set_data(times, vertical_support)

        spool_velocity = [sample.spool_velocity_cmd for sample in samples]
        self.spool_velocity_ratio_line.set_data(times, spool_velocity)

        x_right = max(self.live_window_s, latest_t)
        x_left = max(0.0, x_right - self.live_window_s)
        plot_groups = (
            (self.task_ax, tracking_error),
            (self.smooth_ax, body_angle + cable_angle),
            (self.cable_ax, tension + desired_tension + vertical_support),
            (self.reel_ax, spool_velocity),
        )
        for axis, values in plot_groups:
            axis.set_xlim(x_left, x_right)
            if values:
                ymin = min(values)
                ymax = max(values)
                margin = max(0.02, 0.10 * max(1e-9, ymax - ymin))
                axis.set_ylim(ymin - margin, ymax + margin)

    @staticmethod
    def _set_arrow(arrow: FancyArrowPatch, start: Vec2, end: Vec2) -> None:
        arrow.set_positions(start, end)

    def input_mode_label(self) -> str:
        if self.draw_mode:
            return "draw"
        if self.append_mode:
            return "append"
        return "single"

    def animate(self, _frame: int):
        now = time.perf_counter()
        wall_dt = clamp(now - self._last_frame_wall_time, 0.0, 0.12)
        self._last_frame_wall_time = now
        if self.playing:
            speed = float(self.speed_slider.val)
            sim_dt = speed * wall_dt
            steps = max(1, int(round(sim_dt / self.params.dt))) if sim_dt > 0.0 else 0
            for _ in range(steps):
                self.sim.step()
            self.draw()
        return []

    def on_press(self, event) -> None:
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        clicked_point = self.sim._clamp_wall_point((float(event.xdata), float(event.ydata)))
        if self.draw_mode:
            self.is_drawing = True
            self.draw_points = [clicked_point]
            self.playing = False
            self._last_frame_wall_time = time.perf_counter()
            self.play_button.label.set_text("Play")
            self.draw()
            self.fig.canvas.draw_idle()
            return
        if self.append_mode:
            self.sim.append_target(clicked_point, planner=self.planner)
        else:
            self.sim.set_target(clicked_point, planner=self.planner)
        self.playing = True
        self._last_frame_wall_time = time.perf_counter()
        self.play_button.label.set_text("Pause")
        self.draw()
        self.fig.canvas.draw_idle()

    def on_motion(self, event) -> None:
        if not self.draw_mode or not self.is_drawing:
            return
        if event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        point = self.sim._clamp_wall_point((float(event.xdata), float(event.ydata)))
        if self._append_draw_point(point):
            self.draw_preview_line.set_data(
                [draw_point[0] for draw_point in self.draw_points],
                [draw_point[1] for draw_point in self.draw_points],
            )
            self.fig.canvas.draw_idle()

    def on_release(self, event) -> None:
        if not self.draw_mode or not self.is_drawing:
            return
        if event.inaxes is self.ax and event.xdata is not None and event.ydata is not None:
            self._append_draw_point(self.sim._clamp_wall_point((float(event.xdata), float(event.ydata))))
        self.is_drawing = False
        smooth_path = self._simplify_draw_points(self.draw_points)
        self.draw_points = []
        if smooth_path:
            self.sim.set_smooth_path(smooth_path)
            self.playing = True
            self._last_frame_wall_time = time.perf_counter()
            self.play_button.label.set_text("Pause")
        self.draw()
        self.fig.canvas.draw_idle()

    def _append_draw_point(self, point: Vec2) -> bool:
        if not self.draw_points or distance2(point, self.draw_points[-1]) >= self.draw_min_spacing:
            self.draw_points.append(point)
            return True
        return False

    def _simplify_draw_points(self, points: Sequence[Vec2]) -> list[Vec2]:
        if not points:
            return []
        filtered = [points[0]]
        for point in points[1:]:
            if distance2(point, filtered[-1]) >= self.draw_min_spacing:
                filtered.append(point)
        if distance2(points[-1], filtered[-1]) >= 1e-6:
            filtered.append(points[-1])
        if len(filtered) <= self.draw_max_points:
            return filtered
        keep: list[Vec2] = []
        last_index = len(filtered) - 1
        for sample_index in range(self.draw_max_points):
            source_index = round(sample_index * last_index / (self.draw_max_points - 1))
            point = filtered[source_index]
            if not keep or distance2(point, keep[-1]) >= 1e-6:
                keep.append(point)
        return keep

    def toggle_play(self, _event) -> None:
        self.playing = not self.playing
        self._last_frame_wall_time = time.perf_counter()
        self.play_button.label.set_text("Pause" if self.playing else "Play")

    def reset(self, _event) -> None:
        self.sim.reset()
        self.playing = False
        self._last_frame_wall_time = time.perf_counter()
        self.append_mode = False
        self.draw_mode = False
        self.is_drawing = False
        self.draw_points = []
        self.play_button.label.set_text("Play")
        self.append_button.label.set_text("Append Off")
        self.draw_button.label.set_text("Draw Off")
        self.draw()
        self.fig.canvas.draw_idle()

    def clear_trace(self, _event) -> None:
        self.sim.clear_trajectory()
        self.sim.history = self.sim.history[-1:]
        self.draw_points = []
        self.draw()
        self.fig.canvas.draw_idle()

    def toggle_append(self, _event) -> None:
        self.append_mode = not self.append_mode
        if self.append_mode:
            self.draw_mode = False
            self.is_drawing = False
            self.draw_points = []
            self.draw_button.label.set_text("Draw Off")
        self.append_button.label.set_text("Append On" if self.append_mode else "Append Off")

    def toggle_draw(self, _event) -> None:
        self.draw_mode = not self.draw_mode
        self.is_drawing = False
        self.draw_points = []
        if self.draw_mode:
            self.append_mode = False
            self.append_button.label.set_text("Append Off")
        self.draw_button.label.set_text("Draw On" if self.draw_mode else "Draw Off")
        self.draw()
        self.fig.canvas.draw_idle()

    def toggle_layer(self, label: str) -> None:
        if label == "trace":
            self.show_trace = not self.show_trace
        elif label == "target":
            self.show_target = not self.show_target
        elif label == "path":
            self.show_path = not self.show_path
        elif label == "forces":
            self.show_forces = not self.show_forces
        self._update_layer_button_labels()
        self.draw()
        self.fig.canvas.draw_idle()

    def _update_layer_button_labels(self) -> None:
        self.trace_button.label.set_text("Trace On" if self.show_trace else "Trace Off")
        self.target_button.label.set_text("Target On" if self.show_target else "Target Off")
        self.path_button.label.set_text("Path On" if self.show_path else "Path Off")
        self.forces_button.label.set_text("Forces On" if self.show_forces else "Forces Off")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive 2.5D PRISMS wall-tool simulator.")
    parser.add_argument("--duration", type=float, default=8.0, help="Batch-simulation duration for --save-fig.")
    parser.add_argument("--dt", type=float, default=SimParams.dt)
    parser.add_argument(
        "--planner",
        choices=PLANNER_CHOICES,
        default=PLANNER_DIRECT,
        help="Reference planner used for click targets and batch export.",
    )
    parser.add_argument(
        "--save-fig",
        default="",
        help="Optional PNG path for the current/final frame. Use with --no-show for batch export.",
    )
    parser.add_argument("--no-show", action="store_true", help="Run and save without opening a window.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    params = SimParams(dt=float(args.dt))
    simulator = WallToolSimulator(params)
    app = WallToolApp(simulator, planner=str(args.planner))

    if args.save_fig:
        simulator.set_target((0.65, 1.15), planner=str(args.planner))
        for _ in range(max(0, int(float(args.duration) / params.dt))):
            simulator.step()
        app.draw()
        output = Path(args.save_fig)
        if not output.is_absolute():
            output = PROJECT_ROOT / output
        output.parent.mkdir(parents=True, exist_ok=True)
        app.fig.savefig(output, dpi=180)
        print(f"Saved frame: {output}")
    if not args.no_show:
        ani = animation.FuncAnimation(app.fig, app.animate, interval=40, blit=False)
        app.fig._prisms_animation = ani
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
