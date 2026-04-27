"""Manage the splatsim Docker container lifecycle."""

from __future__ import annotations

import logging
import time
from pathlib import Path

import docker
from docker.types import DeviceRequest
import grpc

logger = logging.getLogger(__name__)


class SplatSimDockerManager:
    """Start / stop the ``splatsim:latest`` Docker container.

    The container is launched with ``--network=host`` so that DDS
    multicast discovery works between the container and the host, and
    with full GPU access via the NVIDIA Container Toolkit.
    """

    def __init__(
        self,
        image: str = "splatsim:latest",
        grpc_port: int = 50051,
    ) -> None:
        self._image = image
        self._grpc_port = grpc_port
        self._client = docker.from_env()
        self._container = None

    @property
    def grpc_address(self) -> str:
        return f"localhost:{self._grpc_port}"

    def start(self, tileset_host_path: str) -> str:
        """Start the container and return the gRPC address.

        Parameters
        ----------
        tileset_host_path:
            Absolute path to the tileset directory on the host.
            Mounted as ``/data`` inside the container.
        """
        tileset_dir = str(Path(tileset_host_path).resolve().parent)

        logger.info(
            "Starting splatsim container (image=%s, mount=%s → /data)",
            self._image,
            tileset_dir,
        )
        self._container = self._client.containers.run(
            self._image,
            detach=True,
            network_mode="host",
            device_requests=[DeviceRequest(count=-1, capabilities=[["gpu"]])],
            volumes={tileset_dir: {"bind": "/data", "mode": "ro"}},
            auto_remove=True,
        )
        logger.info("Container started: %s", self._container.short_id)
        return self.grpc_address

    def wait_for_ready(self, timeout: float = 60.0) -> None:
        """Block until the gRPC server is reachable or *timeout* expires."""
        deadline = time.monotonic() + timeout
        address = self.grpc_address
        while time.monotonic() < deadline:
            try:
                channel = grpc.insecure_channel(address)
                grpc.channel_ready_future(channel).result(timeout=2.0)
                channel.close()
                logger.info("splatsim gRPC server is ready at %s", address)
                return
            except grpc.FutureTimeoutError:
                pass
            except Exception:
                pass
            time.sleep(1.0)
        raise TimeoutError(
            f"splatsim gRPC server at {address} not ready within {timeout}s"
        )

    def stop(self) -> None:
        """Stop and remove the container (idempotent)."""
        if self._container is not None:
            try:
                self._container.stop(timeout=10)
                logger.info("Container stopped: %s", self._container.short_id)
            except Exception as exc:
                logger.warning("Error stopping container: %s", exc)
            self._container = None
