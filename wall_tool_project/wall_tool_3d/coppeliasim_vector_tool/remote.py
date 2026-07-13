"""Minimal CoppeliaSim 4.1 ZMQ connection and scene helpers."""

from __future__ import annotations

import math
import socket
import subprocess
import time
from pathlib import Path
from typing import Sequence

import zmq
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


DEFAULT_EXE = Path(r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe")


def _client(host: str, port: int, timeout_s: float) -> RemoteAPIClient:
    client = RemoteAPIClient(host=host, port=port)
    timeout_ms = int(1000.0 * max(1.0, float(timeout_s)))
    client.timeout = max(1, int(math.ceil(float(timeout_s))))
    client.socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    client.socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
    client.socket.setsockopt(zmq.LINGER, 0)
    return client


def server_is_listening(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(1.0)
        return sock.connect_ex((host, port)) == 0


def connect_or_launch(
    host: str = "localhost",
    port: int = 23000,
    timeout_s: float = 30.0,
    *,
    launch: bool = True,
    executable: Path = DEFAULT_EXE,
    headless: bool = False,
):
    process: subprocess.Popen | None = None
    if server_is_listening(host, port):
        existing_client = _client(host, port, timeout_s)
        existing_sim = existing_client.require("sim")
        existing_is_headless = bool(
            existing_sim.getBoolParam(existing_sim.boolparam_headless)
        )
        if existing_is_headless and not headless:
            if not launch:
                raise RuntimeError(
                    f"CoppeliaSim at {host}:{port} is headless; a visible run was requested"
                )
            # A stale batch-validation server must never make a normal launch
            # appear to do nothing. Stop it cleanly, close it, then start the
            # requested visible application on the same API port.
            stop_if_running(existing_sim)
            try:
                existing_sim.quitSimulator()
            except Exception:
                # The remote call commonly loses its reply because the server
                # closes the socket while servicing quitSimulator.
                pass
            deadline = time.perf_counter() + 10.0
            while server_is_listening(host, port) and time.perf_counter() < deadline:
                time.sleep(0.05)
            if server_is_listening(host, port):
                raise RuntimeError(
                    "the existing headless CoppeliaSim instance did not release "
                    f"{host}:{port}; close it before starting a visible run"
                )
        else:
            return existing_client, existing_sim, process

    if not server_is_listening(host, port):
        if not launch:
            raise RuntimeError(f"CoppeliaSim is not listening at {host}:{port}")
        if not executable.exists():
            raise FileNotFoundError(f"CoppeliaSim executable not found: {executable}")
        args = [str(executable)]
        if headless:
            args.append("-h")
        process = subprocess.Popen(args, cwd=str(executable.parent))
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline and not server_is_listening(host, port):
            time.sleep(0.25)
        if not server_is_listening(host, port):
            if process.poll() is None:
                process.terminate()
            raise RuntimeError("CoppeliaSim launched but its ZMQ server did not become available")
    client = _client(host, port, timeout_s)
    return client, client.require("sim"), process


def stop_if_running(sim) -> None:
    if sim.getSimulationState() == sim.simulation_stopped:
        return
    sim.stopSimulation(True)
    deadline = time.perf_counter() + 10.0
    while sim.getSimulationState() != sim.simulation_stopped:
        if time.perf_counter() > deadline:
            raise RuntimeError("CoppeliaSim did not stop within 10 seconds")
        time.sleep(0.02)


def set_static(sim, handle: int, enabled: bool) -> None:
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, int(enabled))


def set_respondable(sim, handle: int, enabled: bool) -> None:
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, int(enabled))


def set_visible(sim, handle: int, enabled: bool) -> None:
    sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, 1 if enabled else 0)


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
    parent: int = -1,
    visible: bool = True,
    specular: Sequence[float] = (0.18, 0.18, 0.18),
    emission: Sequence[float] | None = None,
    transparency: float = 0.0,
    smooth: bool = True,
) -> int:
    handle = int(sim.createPrimitiveShape(primitive, [float(v) for v in size], 0))
    sim.setObjectAlias(handle, alias, 1)
    # Establish the hierarchy before applying a parent-relative transform.
    # Parenting with ``keepInPlace=True`` after setting a relative pose silently
    # reinterprets that pose as a world pose and displaces child geometry.
    if parent != -1:
        sim.setObjectParent(handle, parent, False)
    sim.setObjectPosition(handle, parent, [float(v) for v in position])
    sim.setObjectOrientation(handle, parent, [float(v) for v in orientation])
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, [float(v) for v in color])
    sim.setShapeColor(handle, None, sim.colorcomponent_specular, [float(v) for v in specular])
    if emission is not None:
        sim.setShapeColor(handle, None, sim.colorcomponent_emission, [float(v) for v in emission])
    if transparency > 0.0:
        sim.setShapeColor(
            handle,
            None,
            sim.colorcomponent_transparency,
            [max(0.0, min(1.0, float(transparency)))],
        )
    if smooth:
        try:
            sim.setObjectFloatParam(handle, sim.shapefloatparam_shading_angle, math.radians(45.0))
        except Exception:
            pass
    set_static(sim, handle, static)
    set_respondable(sim, handle, respondable)
    set_visible(sim, handle, visible)
    return handle


def object_handle(sim, alias: str) -> int:
    try:
        return int(sim.getObject(f"/{alias}"))
    except Exception as exc:
        raise RuntimeError(f"required CoppeliaSim object '/{alias}' is missing") from exc
