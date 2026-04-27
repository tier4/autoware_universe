"""gRPC client wrapper for the splatsim RenderingService."""

from __future__ import annotations

import logging
import queue
import threading
from typing import Iterator

import grpc

from autoware_carla_interface.splatsim.proto import (
    rendering_service_pb2 as pb2,
    rendering_service_pb2_grpc as pb2_grpc,
)

logger = logging.getLogger(__name__)

_SENTINEL = None  # pushed to signal stream end


class SplatSimGrpcClient:
    """Thread-safe gRPC client for the splatsim ``RenderingService``."""

    def __init__(self, address: str = "localhost:50051") -> None:
        self._address = address
        self._channel = grpc.insecure_channel(address)
        self._stub = pb2_grpc.RenderingServiceStub(self._channel)

        self._pose_queue: queue.Queue[pb2.CameraData | None] = queue.Queue(
            maxsize=128
        )
        self._stream_thread: threading.Thread | None = None
        self._stream_result: pb2.StreamSummary | None = None
        self._stream_error: Exception | None = None

    def initialize(self, request: pb2.InitializeRequest) -> pb2.InitializeResponse:
        """Send ``Initialize`` RPC.  Blocks until the server finishes loading."""
        logger.info("Sending Initialize to %s ...", self._address)
        response = self._stub.Initialize(request)
        if response.success:
            logger.info("Initialize succeeded")
        else:
            logger.error("Initialize failed: %s", response.message)
        return response

    # ── streaming ─────────────────────────────────────────────────────

    def start_stream(self) -> None:
        """Open the ``StreamCameraData`` client-streaming RPC in a background thread."""
        self._stream_thread = threading.Thread(
            target=self._stream_worker, daemon=True
        )
        self._stream_thread.start()

    def send_camera_data(
        self,
        sec: int,
        nanosec: int,
        position: tuple[float, float, float],
        rotation_wxyz: tuple[float, float, float, float],
    ) -> None:
        """Enqueue a ``CameraData`` message for the background stream."""
        msg = pb2.CameraData(
            stamp=pb2.Timestamp(sec=sec, nanosec=nanosec),
            pose=pb2.Pose(
                position=pb2.Vector3(x=position[0], y=position[1], z=position[2]),
                rotation=pb2.Quaternion(
                    w=rotation_wxyz[0],
                    x=rotation_wxyz[1],
                    y=rotation_wxyz[2],
                    z=rotation_wxyz[3],
                ),
            ),
        )
        try:
            self._pose_queue.put_nowait(msg)
        except queue.Full:
            logger.warning("Pose queue full — dropping oldest frame")
            try:
                self._pose_queue.get_nowait()
            except queue.Empty:
                pass
            self._pose_queue.put_nowait(msg)

    def close_stream(self) -> pb2.StreamSummary | None:
        """Signal end-of-stream and wait for the background thread."""
        self._pose_queue.put(_SENTINEL)
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=10.0)
        if self._stream_error is not None:
            logger.error("Stream ended with error: %s", self._stream_error)
        return self._stream_result

    def close(self) -> None:
        """Close the gRPC channel."""
        self._channel.close()

    # ── internals ─────────────────────────────────────────────────────

    def _pose_generator(self) -> Iterator[pb2.CameraData]:
        """Yield CameraData from the queue until _SENTINEL is received."""
        while True:
            msg = self._pose_queue.get()
            if msg is _SENTINEL:
                return
            yield msg

    def _stream_worker(self) -> None:
        """Background thread that drives the StreamCameraData RPC."""
        try:
            self._stream_result = self._stub.StreamCameraData(
                self._pose_generator()
            )
            logger.info(
                "Stream finished: rendered=%d, received=%d",
                self._stream_result.frames_rendered,
                self._stream_result.poses_received,
            )
        except Exception as exc:
            self._stream_error = exc
            logger.exception("StreamCameraData failed")
