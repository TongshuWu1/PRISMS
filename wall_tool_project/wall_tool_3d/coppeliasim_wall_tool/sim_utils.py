"""Shared CoppeliaSim remote-API helpers for the wall-tool scene."""

from __future__ import annotations

import math
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterable, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
LOCAL_COPPELIASIM_CLIENT = Path(
    r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu"
    r"\programming\zmqRemoteApi\clients\python\src"
)
DEFAULT_COPPELIASIM_EXE = Path(r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe")
if LOCAL_COPPELIASIM_CLIENT.exists():
    sys.path.insert(0, str(LOCAL_COPPELIASIM_CLIENT))

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ModuleNotFoundError as exc:  # pragma: no cover - depends on local install.
    raise SystemExit(
        "Missing CoppeliaSim ZMQ remote API client. Install requirements first:\n"
        "  python -m pip install -r requirements.txt"
    ) from exc


GENERATED_PREFIXES = (
    "/wall_tool_",
    "/facade_",
    "/anchor_",
    "/reel_",
    "/pen_",
    "/ink_",
    "/inspection_",
)


def connect_client(host: str = "localhost", port: int = 23000, connect_timeout: int = 20):
    """Connect and return both the ZMQ client and the CoppeliaSim module."""
    print(f"Connecting to CoppeliaSim ZMQ remote API at {host}:{port} ...", flush=True)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(2.0)
        if sock.connect_ex((host, port)) != 0:
            raise RuntimeError(
                f"CoppeliaSim is not listening at {host}:{port}. "
                "Start CoppeliaSim and make sure the ZMQ remote API server is running."
            )
    client = RemoteAPIClient(host=host, port=port)
    client.initialTimeout = int(connect_timeout)
    return client, client.require("sim")


def connect(host: str = "localhost", port: int = 23000, connect_timeout: int = 20):
    """Connect to a running CoppeliaSim ZMQ remote API server."""
    _client, sim = connect_client(host, port, connect_timeout)
    return sim


def server_is_listening(host: str = "localhost", port: int = 23000) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(1.0)
        return sock.connect_ex((host, port)) == 0


def launch_coppeliasim(exe_path: Path = DEFAULT_COPPELIASIM_EXE) -> subprocess.Popen:
    if not exe_path.exists():
        raise FileNotFoundError(
            f"CoppeliaSim executable not found: {exe_path}\n"
            "Pass --coppeliasim-exe with your local CoppeliaSim executable path."
        )
    print(f"Launching CoppeliaSim: {exe_path}", flush=True)
    return subprocess.Popen([str(exe_path)], cwd=str(exe_path.parent))


def connect_or_launch(
    host: str = "localhost",
    port: int = 23000,
    connect_timeout: int = 20,
    *,
    launch: bool = True,
    exe_path: Path = DEFAULT_COPPELIASIM_EXE,
):
    if not server_is_listening(host, port) and launch:
        launch_coppeliasim(exe_path)
        deadline = time.perf_counter() + max(5.0, float(connect_timeout))
        while time.perf_counter() < deadline:
            if server_is_listening(host, port):
                break
            time.sleep(0.5)
    return connect(host, port, connect_timeout)


def connect_or_launch_client(
    host: str = "localhost",
    port: int = 23000,
    connect_timeout: int = 20,
    *,
    launch: bool = True,
    exe_path: Path = DEFAULT_COPPELIASIM_EXE,
):
    if not server_is_listening(host, port) and launch:
        launch_coppeliasim(exe_path)
        deadline = time.perf_counter() + max(5.0, float(connect_timeout))
        while time.perf_counter() < deadline:
            if server_is_listening(host, port):
                break
            time.sleep(0.5)
    return connect_client(host, port, connect_timeout)


def object_alias(sim, handle: int) -> str:
    return str(sim.getObjectAlias(handle, 1))


def all_scene_objects(sim) -> list[int]:
    handles: list[int] = []
    index = 0
    while True:
        handle = sim.getObjects(index, sim.handle_all)
        if handle < 0:
            break
        handles.append(handle)
        index += 1
    return handles


def stop_if_running(sim) -> None:
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation(True)
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)


def remove_generated(sim, prefixes: Iterable[str] = GENERATED_PREFIXES) -> None:
    active_prefixes = tuple(prefixes)
    to_remove = [handle for handle in all_scene_objects(sim) if object_alias(sim, handle).startswith(active_prefixes)]
    if to_remove:
        sim.removeObjects(to_remove, False)
        print(f"Removed {len(to_remove)} previous generated wall-tool objects.", flush=True)


def set_static(sim, handle: int, enabled: bool) -> None:
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1 if enabled else 0)


def set_respondable(sim, handle: int, enabled: bool) -> None:
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1 if enabled else 0)


def set_visible(sim, handle: int, enabled: bool) -> None:
    sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, 1 if enabled else 0)


def color_shape(sim, handle: int, color: Sequence[float]) -> None:
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, list(color))


def create_shape(
    sim,
    primitive: int,
    size: Sequence[float],
    alias: str,
    position: Sequence[float],
    color: Sequence[float],
    *,
    orientation: Sequence[float] = (0.0, 0.0, 0.0),
    static: bool = True,
    respondable: bool = False,
    parent: int | None = None,
) -> int:
    handle = sim.createPrimitiveShape(primitive, list(size), 0)
    sim.setObjectAlias(handle, alias, 1)
    sim.setObjectPosition(handle, parent if parent is not None else -1, list(position))
    sim.setObjectOrientation(handle, parent if parent is not None else -1, list(orientation))
    color_shape(sim, handle, color)
    set_static(sim, handle, static)
    set_respondable(sim, handle, respondable)
    if parent is not None:
        sim.setObjectParent(handle, parent, True)
    return int(handle)


def normalize(vector: Sequence[float]) -> list[float]:
    norm = math.sqrt(sum(float(value) * float(value) for value in vector))
    if norm < 1e-12:
        return [0.0, 0.0, 1.0]
    return [float(value) / norm for value in vector]


def cross(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    ]


def matrix_from_z_axis(center: Sequence[float], direction: Sequence[float]) -> list[float]:
    z_axis = normalize(direction)
    reference = [0.0, 0.0, 1.0]
    if abs(sum(reference[index] * z_axis[index] for index in range(3))) > 0.95:
        reference = [0.0, 1.0, 0.0]
    x_axis = normalize(cross(reference, z_axis))
    y_axis = cross(z_axis, x_axis)
    return [
        x_axis[0], y_axis[0], z_axis[0], float(center[0]),
        x_axis[1], y_axis[1], z_axis[1], float(center[1]),
        x_axis[2], y_axis[2], z_axis[2], float(center[2]),
    ]


def update_cylinder_between(
    sim,
    handle: int,
    start: Sequence[float],
    end: Sequence[float],
    previous_length: float,
) -> float:
    delta = [float(end[index]) - float(start[index]) for index in range(3)]
    length = max(1e-6, math.sqrt(sum(value * value for value in delta)))
    center = [(float(start[index]) + float(end[index])) * 0.5 for index in range(3)]
    if previous_length > 1e-9:
        sim.scaleObject(handle, 1.0, 1.0, length / previous_length, 0)
    sim.setObjectMatrix(handle, -1, matrix_from_z_axis(center, delta))
    return length


def wall_point_to_world(x: float, z: float, wall_y: float = 0.0) -> list[float]:
    return [float(x), float(wall_y), float(z)]


def payload_pose_to_world(x: float, z: float, standoff: float, attitude_rad: float) -> tuple[list[float], list[float]]:
    return [float(x), -abs(float(standoff)), float(z)], [0.0, float(attitude_rad), 0.0]
