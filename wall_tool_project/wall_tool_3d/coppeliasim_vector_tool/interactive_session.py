"""Thread-safe interactive CoppeliaSim session for the trajectory UI."""

from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from cable_hybrid_controller.controller import best_params
from wall_tool_sim.wall_tool_ui import SimParams, clamp_wall_point_for_params

from . import remote, scene
from .controller import ExternalVectorThrustController
from .plant import CoppeliaVectorPlant
from .sensors import VectorToolSensorSuite
from .validation_plant import datasheet_validation_profile


Vec2 = tuple[float, float]


@dataclass(frozen=True)
class SessionConfig:
    path: tuple[Vec2, ...]
    path_speed_m_s: float
    corner_speed_m_s: float
    camera: str = "overview"
    host: str = "localhost"
    port: int = 23000
    executable: Path = remote.DEFAULT_EXE
    regenerate_scene: bool = True

    def __post_init__(self) -> None:
        if not self.path:
            raise ValueError("trajectory requires at least one target point")
        if len(self.path) > 48:
            raise ValueError("trajectory cannot contain more than 48 points")
        if not 0.02 <= self.path_speed_m_s <= 0.25:
            raise ValueError("path speed must be within [0.02, 0.25] m/s")
        if not 0.0 <= self.corner_speed_m_s <= self.path_speed_m_s:
            raise ValueError("corner speed must be non-negative and no greater than path speed")
        if self.camera not in {"overview", "payload", "winch"}:
            raise ValueError(f"unknown camera view: {self.camera}")
        if not all(math.isfinite(value) for point in self.path for value in point):
            raise ValueError("trajectory points must be finite")


@dataclass(frozen=True)
class SessionTelemetry:
    timestamp_s: float
    payload_xz_m: Vec2
    estimated_xz_m: Vec2
    reference_xz_m: Vec2
    payload_velocity_xz_m_s: Vec2
    estimated_velocity_xz_m_s: Vec2
    orientation_rpy_rad: tuple[float, float, float]
    angular_velocity_rpy_rad_s: tuple[float, float, float]
    wall_normal_drift_m: float
    error_m: float
    speed_m_s: float
    pitch_rad: float
    pitch_rate_rad_s: float
    cable_angle_rad: float
    cable_angle_rate_rad_s: float
    tension_N: float
    reel_length_m: float
    reel_velocity_m_s: float
    left_thrust_N: float
    right_thrust_N: float
    left_servo_rad: float
    right_servo_rad: float
    solver_status: str
    solver_time_s: float
    path_active: bool
    waypoints_remaining: int


@dataclass(frozen=True)
class SessionEvent:
    kind: str
    message: str


class InteractiveCoppeliaSession:
    """Own CoppeliaSim in one worker thread; expose only events and telemetry."""

    def __init__(self) -> None:
        self.telemetry: queue.Queue[SessionTelemetry] = queue.Queue(maxsize=4)
        self.events: queue.Queue[SessionEvent] = queue.Queue()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._pause = threading.Event()
        self._lock = threading.Lock()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def paused(self) -> bool:
        return self._pause.is_set()

    def start(self, config: SessionConfig) -> None:
        with self._lock:
            if self.running:
                raise RuntimeError("a CoppeliaSim trajectory is already running")
            self._stop.clear()
            self._pause.clear()
            self._drain(self.telemetry)
            self._thread = threading.Thread(
                target=self._run,
                args=(config,),
                name="coppeliasim-interactive-session",
                daemon=True,
            )
            self._thread.start()

    def pause(self) -> None:
        if not self.running:
            raise RuntimeError("cannot pause because no trajectory is running")
        self._pause.set()
        self._event("paused", "Simulation paused; physics and controller time are frozen.")

    def resume(self) -> None:
        if not self.running:
            raise RuntimeError("cannot resume because no trajectory is running")
        self._pause.clear()
        self._event("running", "Simulation resumed.")

    def stop(self) -> None:
        self._stop.set()
        self._pause.clear()

    def wait(self, timeout_s: float = 10.0) -> bool:
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout=max(0.0, timeout_s))
        return not thread.is_alive()

    @staticmethod
    def _drain(target: queue.Queue) -> None:
        while True:
            try:
                target.get_nowait()
            except queue.Empty:
                return

    def _event(self, kind: str, message: str) -> None:
        self.events.put(SessionEvent(kind, message))

    def _publish(self, sample: SessionTelemetry) -> None:
        try:
            self.telemetry.put_nowait(sample)
        except queue.Full:
            try:
                self.telemetry.get_nowait()
            except queue.Empty:
                pass
            self.telemetry.put_nowait(sample)

    def _run(self, config: SessionConfig) -> None:
        client = None
        sim = None
        try:
            base = best_params()
            params = SimParams(**{**base.__dict__, "path_speed": config.path_speed_m_s})
            path = tuple(clamp_wall_point_for_params(point, params) for point in config.path)
            self._event("connecting", "Connecting to CoppeliaSim and building the physical scene...")
            client, sim, _process = remote.connect_or_launch(
                config.host,
                config.port,
                30.0,
                launch=True,
                executable=config.executable,
                headless=False,
            )
            self._event("connecting", "Connected. Preparing the detailed scene...")
            remote.stop_if_running(sim)
            if config.regenerate_scene or not scene.SCENE_PATH.exists():
                handles = scene.build_scene(sim, params)
                scene.save_scene(sim, handles)
                self._event("connecting", "Detailed scene rebuilt and saved.")
            else:
                sim.loadScene(str(scene.SCENE_PATH))
                handles = scene.resolve_handles(sim)
                self._event("connecting", "Validated detailed scene loaded.")
            scene.configure_camera(sim, params, config.camera)
            sim.setFloatParam(sim.floatparam_simulation_time_step, params.dt)

            validation_profile = datasheet_validation_profile(params)
            plant = CoppeliaVectorPlant(sim, handles, params, validation_profile)
            self._event(
                "connecting",
                f"Independent validation plant loaded: {validation_profile.profile_name} "
                "(uncalibrated datasheet profile).",
            )
            initial_truth = plant.truth(0.0)
            sensors = VectorToolSensorSuite(params)
            initial_estimate = sensors.update(initial_truth, force=True)
            controller = ExternalVectorThrustController(params)
            controller.command_corner_smooth_path(
                initial_estimate.payload_position_xz_m,
                path,
                config.corner_speed_m_s,
            )
            self._event("connecting", "Sensors and NMPC initialized from the physical trim state.")
            scene.create_planned_path_visual(
                sim,
                (initial_estimate.payload_position_xz_m, *path),
            )
            self._event("connecting", "Planned path rendered; starting synchronous physics...")

            client.setStepping(True)
            sim.startSimulation()
            self._event("running", f"Running {len(path)}-point trajectory at {params.path_speed:.3f} m/s.")
            command = None
            last_controller_sample_s = -math.inf
            next_telemetry_s = 0.0
            start_y = initial_truth.position_world_m[1]
            while not self._stop.is_set():
                if self._pause.is_set():
                    time.sleep(0.02)
                    continue
                truth = plant.truth()
                estimate = sensors.update(truth)
                if command is None or estimate.timestamp_s > last_controller_sample_s + 1e-12:
                    command = controller.step(estimate)
                    last_controller_sample_s = estimate.timestamp_s
                sim.setObjectPosition(
                    handles.target,
                    -1,
                    [command.reference_position_xz_m[0], -0.038, command.reference_position_xz_m[1]],
                )
                plant.apply(command, truth)
                error = math.hypot(
                    truth.position_world_m[0] - command.reference_position_xz_m[0],
                    truth.position_world_m[2] - command.reference_position_xz_m[1],
                )
                self._assert_physical_limits(truth, error, start_y, params)
                if truth.timestamp_s + 1e-12 >= next_telemetry_s:
                    self._publish(SessionTelemetry(
                        timestamp_s=truth.timestamp_s,
                        payload_xz_m=(truth.position_world_m[0], truth.position_world_m[2]),
                        estimated_xz_m=estimate.payload_position_xz_m,
                        reference_xz_m=command.reference_position_xz_m,
                        payload_velocity_xz_m_s=(
                            truth.linear_velocity_world_m_s[0],
                            truth.linear_velocity_world_m_s[2],
                        ),
                        estimated_velocity_xz_m_s=estimate.payload_velocity_xz_m_s,
                        orientation_rpy_rad=(
                            truth.orientation_world_rad[0],
                            -truth.orientation_world_rad[1],
                            truth.orientation_world_rad[2],
                        ),
                        angular_velocity_rpy_rad_s=(
                            truth.angular_velocity_world_rad_s[0],
                            -truth.angular_velocity_world_rad_s[1],
                            truth.angular_velocity_world_rad_s[2],
                        ),
                        wall_normal_drift_m=truth.position_world_m[1] - start_y,
                        error_m=error,
                        speed_m_s=math.hypot(
                            truth.linear_velocity_world_m_s[0],
                            truth.linear_velocity_world_m_s[2],
                        ),
                        pitch_rad=-truth.orientation_world_rad[1],
                        pitch_rate_rad_s=estimate.payload_angular_rate_rad_s,
                        cable_angle_rad=estimate.cable_angle_rad,
                        cable_angle_rate_rad_s=estimate.cable_angle_rate_rad_s,
                        tension_N=truth.cable_tension_N,
                        reel_length_m=truth.reel_length_m,
                        reel_velocity_m_s=truth.reel_velocity_m_s,
                        left_thrust_N=truth.left_thrust_N,
                        right_thrust_N=truth.right_thrust_N,
                        left_servo_rad=truth.left_servo_angle_rad,
                        right_servo_rad=truth.right_servo_angle_rad,
                        solver_status=command.solver_status,
                        solver_time_s=command.solver_time_s,
                        path_active=bool(controller.trajectory.segments),
                        waypoints_remaining=len(controller.trajectory.goals),
                    ))
                    next_telemetry_s += 0.05
                client.step()
            self._event("stopping", "Stopping CoppeliaSim trajectory...")
        except Exception as exc:
            self._event("error", f"{type(exc).__name__}: {exc}")
        finally:
            if sim is not None:
                try:
                    remote.stop_if_running(sim)
                except Exception as exc:
                    self._event("error", f"CoppeliaSim stop failed: {exc}")
            if client is not None:
                try:
                    client.setStepping(False)
                except Exception:
                    pass
            self._event("stopped", "Simulation stopped. The CoppeliaSim scene remains open.")

    @staticmethod
    def _assert_physical_limits(truth, error: float, start_y: float, params: SimParams) -> None:
        if abs(truth.position_world_m[1] - start_y) > 0.025:
            raise RuntimeError("wall-normal drift exceeded 25 mm")
        if max(abs(truth.orientation_world_rad[0]), abs(truth.orientation_world_rad[2])) > math.radians(5.0):
            raise RuntimeError("payload roll/yaw exceeded 5 degrees")
        if abs(truth.orientation_world_rad[1]) > params.mpc_attitude_limit_rad:
            raise RuntimeError("payload pitch exceeded the NMPC attitude limit")
        if truth.cable_tension_N < 0.05:
            raise RuntimeError("steel cable became slack")
        if truth.cable_tension_N > params.max_spool_tension + 1e-6:
            raise RuntimeError("steel cable tension exceeded the load-cell/reel limit")
        if error > 0.12:
            raise RuntimeError(f"trajectory error exceeded 120 mm ({1000.0 * error:.1f} mm)")
