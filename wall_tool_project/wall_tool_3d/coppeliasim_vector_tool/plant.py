"""CoppeliaSim-owned rigid-body plant with explicit cable and actuator dynamics."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

from wall_tool_sim.steel_cable import SteelCableSpec
from wall_tool_sim.wall_tool_ui import SimParams, clamp, integrated_motor_center_offsets, wrap_angle

from .contracts import ActuatorCommand, PlantTruth
from .scene import CABLE_SEGMENTS, CABLE_STRANDS, SceneHandles
from .validation_plant import (
    IndependentCableModel,
    IndependentReelModel,
    IndependentRotorModel,
    IndependentServoModel,
    ValidationPlantProfile,
    datasheet_validation_profile,
)


def add3(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [float(a[i]) + float(b[i]) for i in range(3)]


def sub3(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [float(a[i]) - float(b[i]) for i in range(3)]


def scale3(a: Sequence[float], scale: float) -> list[float]:
    return [float(value) * float(scale) for value in a]


def dot3(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(a[i]) * float(b[i]) for i in range(3))


def cross3(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    ]


def norm3(a: Sequence[float]) -> float:
    return math.sqrt(dot3(a, a))


def normalize3(a: Sequence[float]) -> list[float]:
    length = norm3(a)
    if length <= 1e-12:
        raise RuntimeError("cannot normalize a zero-length plant vector")
    return scale3(a, 1.0 / length)


def local_vector_to_world(matrix: Sequence[float], local: Sequence[float]) -> list[float]:
    return [
        float(matrix[0]) * float(local[0]) + float(matrix[1]) * float(local[1]) + float(matrix[2]) * float(local[2]),
        float(matrix[4]) * float(local[0]) + float(matrix[5]) * float(local[1]) + float(matrix[6]) * float(local[2]),
        float(matrix[8]) * float(local[0]) + float(matrix[9]) * float(local[1]) + float(matrix[10]) * float(local[2]),
    ]


def local_point_to_world(matrix: Sequence[float], local: Sequence[float]) -> list[float]:
    return add3([float(matrix[3]), float(matrix[7]), float(matrix[11])], local_vector_to_world(matrix, local))


def matrix_from_z_axis(origin: Sequence[float], z_axis_input: Sequence[float]) -> list[float]:
    z_axis = normalize3(z_axis_input)
    reference = [0.0, 1.0, 0.0]
    if abs(dot3(reference, z_axis)) > 0.95:
        reference = [1.0, 0.0, 0.0]
    x_axis = normalize3(cross3(reference, z_axis))
    y_axis = cross3(z_axis, x_axis)
    return [
        x_axis[0], y_axis[0], z_axis[0], float(origin[0]),
        x_axis[1], y_axis[1], z_axis[1], float(origin[1]),
        x_axis[2], y_axis[2], z_axis[2], float(origin[2]),
    ]


def steel_cable_midspan_sag_m(
    cable: SteelCableSpec,
    chord_length_m: float,
    tension_N: float,
    gravity_m_s2: float,
    transverse_gravity_fraction: float,
) -> float:
    """Small-sag wire-rope approximation under distributed self weight."""

    length = max(float(chord_length_m), 0.0)
    fraction = clamp(float(transverse_gravity_fraction), 0.0, 1.0)
    transverse_weight_N_m = cable.mass_per_length_kg_m * max(float(gravity_m_s2), 0.0) * fraction
    sag = (
        transverse_weight_N_m * length * length
        / (8.0 * max(float(tension_N), cable.min_visual_tension_N))
    )
    return clamp(sag, 0.0, cable.max_visual_sag_m)


def _add_wrench_at_point(
    net_force: list[float],
    net_torque: list[float],
    force: Sequence[float],
    point: Sequence[float],
    center: Sequence[float],
) -> None:
    torque = cross3(sub3(point, center), force)
    for i in range(3):
        net_force[i] += float(force[i])
        net_torque[i] += float(torque[i])


@dataclass
class VectorPlantState:
    reel_length_m: float | None = None
    reel_velocity_m_s: float = 0.0
    cable_tension_N: float = 0.0
    drum_tension_N: float = 0.0
    left_thrust_N: float = 0.0
    right_thrust_N: float = 0.0
    left_servo_angle_rad: float = 0.0
    right_servo_angle_rad: float = 0.0
    left_servo_rate_rad_s: float = 0.0
    right_servo_rate_rad_s: float = 0.0
    last_time_s: float | None = None
    last_visual_time_s: float = -math.inf
    cable_segment_lengths: list[float] = field(default_factory=list)


class CoppeliaVectorPlant:
    """Apply only modeled physical wrenches to a free CoppeliaSim rigid body."""

    def __init__(
        self,
        sim,
        handles: SceneHandles,
        params: SimParams,
        validation_profile: ValidationPlantProfile | None = None,
    ) -> None:
        self.sim = sim
        self.handles = handles
        self.params = params
        self.validation_profile = validation_profile or datasheet_validation_profile(params)
        self.state = VectorPlantState()
        # SteelCableSpec is retained only for visual diameter/sag rendering.
        # All forces come from the independent validation models below.
        cable_profile = self.validation_profile.cable
        self._visual_cable = SteelCableSpec(
            diameter_m=cable_profile.diameter_m,
            youngs_modulus_pa=cable_profile.axial_rigidity_N
            / (math.pi * (0.5 * cable_profile.diameter_m) ** 2),
            density_kg_m3=cable_profile.mass_per_length_kg_m
            / (math.pi * (0.5 * cable_profile.diameter_m) ** 2),
            structural_compliance_m_N=cable_profile.termination_compliance_m_N,
            damping_ratio=cable_profile.damping_ratio,
            payload_weight_fraction=cable_profile.payload_weight_fraction,
        )
        self._cable_model = IndependentCableModel(cable_profile)
        self._reel_model = IndependentReelModel(
            self.validation_profile.reel,
            cable_profile.diameter_m,
        )
        self._rotor_model = IndependentRotorModel(self.validation_profile.rotor)
        self._servo_model = IndependentServoModel(self.validation_profile.servo)
        self._left_offset, self._right_offset = integrated_motor_center_offsets(params, 0.0)

    def _read_body(self):
        matrix = [float(value) for value in self.sim.getObjectMatrix(self.handles.payload, -1)]
        position = [matrix[3], matrix[7], matrix[11]]
        orientation = [float(value) for value in self.sim.getObjectOrientation(self.handles.payload, -1)]
        linear_velocity, angular_velocity = self.sim.getVelocity(self.handles.payload)
        return matrix, position, orientation, [float(v) for v in linear_velocity], [float(v) for v in angular_velocity]

    def truth(self, timestamp_s: float | None = None) -> PlantTruth:
        matrix, position, orientation, linear_velocity, angular_velocity = self._read_body()
        anchor = [float(v) for v in self.sim.getObjectPosition(self.handles.anchor, -1)]
        mount = local_point_to_world(matrix, [0.0, 0.0, self.params.payload_hex_radius])
        mount_arm = sub3(mount, position)
        mount_velocity = add3(linear_velocity, cross3(angular_velocity, mount_arm))
        now = float(self.sim.getSimulationTime()) if timestamp_s is None else float(timestamp_s)
        if self.state.reel_length_m is None:
            cable_profile = self.validation_profile.cable
            distance = norm3(sub3(mount, anchor))
            initial_tension = min(
                self.params.max_spool_tension,
                self.params.desired_cable_support_fraction * self.params.total_mass * self.params.gravity,
            )
            initial_extension = cable_profile.extension_for_tension_m(initial_tension, distance)
            self.state.reel_length_m = clamp(
                distance - initial_extension,
                self.params.min_cable_length,
                self.params.max_cable_length,
            )
            self.state.cable_tension_N = initial_tension
            self.state.drum_tension_N = initial_tension
            self._reel_model.initialize(self.state.reel_length_m)
            self._cable_model.initialize(initial_extension)
            # The experiment begins from a physically trimmed, pre-armed
            # condition: the cable is preloaded by the reel and the rotors are
            # already spinning before the payload is released.  Starting the
            # dynamic body with zero rotor thrust while the controller assumes
            # trim injects an artificial cable-spring transient.
            cable_weight = (
                cable_profile.payload_weight_fraction
                * cable_profile.mass_per_length_kg_m
                * distance
                * self.params.gravity
            )
            initial_thrust = clamp(
                0.5 * (
                    self.params.total_mass * self.params.gravity
                    + cable_weight
                    - initial_tension
                ),
                0.0,
                self.params.max_thrust_per_drone,
            )
            self.state.left_thrust_N = initial_thrust
            self.state.right_thrust_N = initial_thrust
            self._rotor_model.initialize(initial_thrust, initial_thrust)
        reel_state = self._reel_model.state
        return PlantTruth(
            timestamp_s=now,
            position_world_m=tuple(position),
            linear_velocity_world_m_s=tuple(linear_velocity),
            orientation_world_rad=tuple(orientation),
            angular_velocity_world_rad_s=tuple(angular_velocity),
            anchor_world_m=tuple(anchor),
            cable_mount_world_m=tuple(mount),
            cable_mount_velocity_world_m_s=tuple(mount_velocity),
            reel_length_m=float(self.state.reel_length_m),
            reel_velocity_m_s=self.state.reel_velocity_m_s,
            cable_tension_N=self.state.cable_tension_N,
            left_servo_angle_rad=self.state.left_servo_angle_rad,
            right_servo_angle_rad=self.state.right_servo_angle_rad,
            left_thrust_N=self.state.left_thrust_N,
            right_thrust_N=self.state.right_thrust_N,
            reel_encoder_length_m=reel_state.encoder_length_m,
            reel_encoder_velocity_m_s=reel_state.encoder_velocity_m_s,
            drum_tension_N=self.state.drum_tension_N,
        )

    def _wind_force(self, now: float, x: float) -> list[float]:
        if not self.params.wind_enabled:
            return [0.0, 0.0, 0.0]
        period = max(self.params.wind_gust_period_s, 1e-6)
        phase = 2.0 * math.pi * now / period
        gust = self.params.wind_gust_force * (
            0.55 * math.sin(phase) + 0.25 * math.sin(2.7 * phase + 0.6)
        )
        half_width = max(0.5 * self.params.wall_width, 1e-6)
        edge_ratio = clamp(abs(x) / half_width, 0.0, 1.0)
        edge_gain = 1.0 + self.params.edge_wind_gain * edge_ratio * edge_ratio
        return [
            edge_gain * (self.params.wind_force_x + gust),
            0.0,
            self.params.wind_force_z
            + self.params.wind_gust_vertical_fraction * gust * math.sin(0.43 * phase + 1.2),
        ]

    def _update_visuals(self, truth: PlantTruth) -> None:
        now = truth.timestamp_s
        if now - self.state.last_visual_time_s < 0.10:
            return
        self.state.last_visual_time_s = now
        for handle, offset, angle in (
            (self.handles.left_servo, self._left_offset, self.state.left_servo_angle_rad),
            (self.handles.right_servo, self._right_offset, self.state.right_servo_angle_rad),
        ):
            axis = [math.sin(angle), 0.0, math.cos(angle)]
            self.sim.setObjectMatrix(handle, self.handles.payload, matrix_from_z_axis([offset[0], 0.0, offset[1]], axis))

        start = list(truth.anchor_world_m)
        end = list(truth.cable_mount_world_m)
        delta = sub3(end, start)
        chord_length = max(norm3(delta), 1e-9)
        chord_axis = scale3(delta, 1.0 / chord_length)
        gravity_axis = [0.0, 0.0, -1.0]
        gravity_parallel = scale3(chord_axis, dot3(gravity_axis, chord_axis))
        transverse_gravity = sub3(gravity_axis, gravity_parallel)
        transverse_fraction = norm3(transverse_gravity)
        sag_direction = (
            scale3(transverse_gravity, 1.0 / transverse_fraction)
            if transverse_fraction > 1e-9
            else [0.0, 0.0, 0.0]
        )
        midspan_sag = steel_cable_midspan_sag_m(
            self._visual_cable,
            chord_length,
            truth.cable_tension_N,
            self.params.gravity,
            transverse_fraction,
        )

        centerline: list[list[float]] = []
        for point_index in range(CABLE_SEGMENTS + 1):
            u = point_index / CABLE_SEGMENTS
            straight = add3(start, scale3(delta, u))
            parabola = 4.0 * midspan_sag * u * (1.0 - u)
            centerline.append(add3(straight, scale3(sag_direction, parabola)))

        expected_handles = CABLE_SEGMENTS * CABLE_STRANDS
        if len(self.handles.cable_segments) != expected_handles:
            raise RuntimeError(
                f"steel cable visual requires {expected_handles} strand segments, "
                f"found {len(self.handles.cable_segments)}"
            )
        if not self.state.cable_segment_lengths:
            self.state.cable_segment_lengths = [chord_length / CABLE_SEGMENTS] * expected_handles
        strand_orbit_radius = 0.26 * self._visual_cable.diameter_m
        rendered_lay_turns = 6.0
        for segment_index in range(CABLE_SEGMENTS):
            base0 = centerline[segment_index]
            base1 = centerline[segment_index + 1]
            segment_axis = normalize3(sub3(base1, base0))
            reference = [0.0, 1.0, 0.0]
            if abs(dot3(reference, segment_axis)) > 0.95:
                reference = [1.0, 0.0, 0.0]
            orbit_x = normalize3(cross3(reference, segment_axis))
            orbit_y = cross3(segment_axis, orbit_x)
            u0 = segment_index / CABLE_SEGMENTS
            u1 = (segment_index + 1) / CABLE_SEGMENTS
            for strand_index in range(CABLE_STRANDS):
                phase_offset = 2.0 * math.pi * strand_index / CABLE_STRANDS
                phase0 = 2.0 * math.pi * rendered_lay_turns * u0 + phase_offset
                phase1 = 2.0 * math.pi * rendered_lay_turns * u1 + phase_offset
                offset0 = add3(
                    scale3(orbit_x, strand_orbit_radius * math.cos(phase0)),
                    scale3(orbit_y, strand_orbit_radius * math.sin(phase0)),
                )
                offset1 = add3(
                    scale3(orbit_x, strand_orbit_radius * math.cos(phase1)),
                    scale3(orbit_y, strand_orbit_radius * math.sin(phase1)),
                )
                p0 = add3(base0, offset0)
                p1 = add3(base1, offset1)
                segment = sub3(p1, p0)
                length = max(norm3(segment), 1e-6)
                center = scale3(add3(p0, p1), 0.5)
                handle_index = segment_index * CABLE_STRANDS + strand_index
                handle = self.handles.cable_segments[handle_index]
                previous = self.state.cable_segment_lengths[handle_index]
                self.sim.scaleObject(handle, 1.0, 1.0, length / max(previous, 1e-9), 0)
                self.sim.setObjectMatrix(handle, -1, matrix_from_z_axis(center, segment))
                self.state.cable_segment_lengths[handle_index] = length

    def apply(self, command: ActuatorCommand, truth: PlantTruth) -> PlantTruth:
        now = truth.timestamp_s
        dt = self.params.dt if self.state.last_time_s is None else now - self.state.last_time_s
        if not math.isfinite(dt) or dt <= 0.0:
            raise RuntimeError(f"plant requires a positive time step, got {dt}")
        self.state.last_time_s = now

        rotor_state = self._rotor_model.step(command.left_thrust_N, command.right_thrust_N, dt)
        self.state.left_thrust_N = rotor_state.left_thrust_N
        self.state.right_thrust_N = rotor_state.right_thrust_N
        left_servo, right_servo = self._servo_model.step(
            command.left_servo_angle_rad,
            command.right_servo_angle_rad,
            dt,
        )
        self.state.left_servo_angle_rad = left_servo.angle_rad
        self.state.left_servo_rate_rad_s = left_servo.rate_rad_s
        self.state.right_servo_angle_rad = right_servo.angle_rad
        self.state.right_servo_rate_rad_s = right_servo.rate_rad_s
        reel_state = self._reel_model.step(
            command.reel_velocity_m_s,
            self.state.drum_tension_N,
            dt,
            self.params.min_cable_length,
            self.params.max_cable_length,
        )
        self.state.reel_length_m = reel_state.paid_out_length_m
        self.state.reel_velocity_m_s = reel_state.line_velocity_m_s

        matrix, center, orientation, linear_velocity, angular_velocity = self._read_body()
        anchor = list(truth.anchor_world_m)
        mount = local_point_to_world(matrix, [0.0, 0.0, self.params.payload_hex_radius])
        mount_velocity = add3(linear_velocity, cross3(angular_velocity, sub3(mount, center)))
        anchor_to_mount = sub3(mount, anchor)
        distance = max(norm3(anchor_to_mount), 1e-9)
        outward = scale3(anchor_to_mount, 1.0 / distance)
        distance_rate = dot3(mount_velocity, outward)
        cable_response = self._cable_model.step(
            distance_m=distance,
            paid_out_length_m=float(self.state.reel_length_m),
            distance_rate_m_s=distance_rate,
            reel_line_velocity_m_s=self.state.reel_velocity_m_s,
            effective_mass_kg=self.params.total_mass,
            maximum_tension_N=self.params.max_spool_tension,
            taut_band_m=self.params.cable_taut_band,
            timestamp_s=now,
            dt_s=dt,
        )
        self.state.cable_tension_N = cable_response.payload_tension_N
        self.state.drum_tension_N = cable_response.drum_tension_N

        net_force = [0.0, 0.0, 0.0]
        net_torque = [0.0, 0.0, 0.0]
        for offset, angle, thrust in (
            (self._left_offset, self.state.left_servo_angle_rad, self.state.left_thrust_N),
            (self._right_offset, self.state.right_servo_angle_rad, self.state.right_thrust_N),
        ):
            local_axis = [math.sin(angle), 0.0, math.cos(angle)]
            axis_world = normalize3(local_vector_to_world(matrix, local_axis))
            point_world = local_point_to_world(matrix, [offset[0], 0.0, offset[1]])
            _add_wrench_at_point(net_force, net_torque, scale3(axis_world, thrust), point_world, center)

        cable_transverse = [-outward[2], 0.0, outward[0]]
        cable_tangent = normalize3(add3(
            outward,
            scale3(cable_transverse, cable_response.tangent_slope),
        ))
        cable_force = scale3(cable_tangent, -self.state.cable_tension_N)
        _add_wrench_at_point(net_force, net_torque, cable_force, mount, center)
        cable_weight = (
            self.validation_profile.cable.payload_weight_fraction
            * self.validation_profile.cable.mass_per_length_kg_m
            * distance
            * self.params.gravity
        )
        net_force[2] -= cable_weight
        pitch = wrap_angle(-float(orientation[1]))
        pitch_rate = -float(angular_velocity[1])
        passive_planar_torque = (
            -self.params.passive_attitude_stiffness_Nm_rad * math.sin(pitch - self.params.nominal_attitude_rad)
            - self.params.passive_attitude_damping_Nm_s_rad * pitch_rate
            - self.params.rotational_damping * pitch_rate
        )
        net_torque[1] -= passive_planar_torque
        wind = self._wind_force(now, center[0])
        for i in range(3):
            net_force[i] += wind[i]
        self.sim.addForceAndTorque(self.handles.payload, net_force, net_torque)

        updated = self.truth(now)
        self._update_visuals(updated)
        return updated
