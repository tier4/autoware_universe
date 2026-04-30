"""SplatSimRGBCamera -- one Docker container + gRPC stream per RGB camera."""

from __future__ import annotations

import logging
import math
from pathlib import Path

import carla
import numpy as np

from autoware_carla_interface.splatsim.docker_manager import SplatSimDockerManager
from autoware_carla_interface.splatsim.grpc_client import SplatSimGrpcClient
from autoware_carla_interface.splatsim.coordinate_transformer import (
    CoordinateTransformer,
    parse_tileset_transform,
    _rotation_matrix_to_quaternion_wxyz,
)
from autoware_carla_interface.splatsim.proto import rendering_service_pb2 as pb2

logger = logging.getLogger(__name__)

# CARLA → ENU: flip y-axis (CARLA South → ENU North)
_S_CARLA_TO_ENU = np.diag([1.0, -1.0, 1.0])


def _fov_to_intrinsics(
    width: int, height: int, fov_deg: float,
) -> tuple[float, float, float, float]:
    """Convert horizontal FOV + resolution to ``(fx, fy, cx, cy)``."""
    fov_rad = math.radians(fov_deg)
    fx = width / (2.0 * math.tan(fov_rad / 2.0))
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


class SplatSimRGBCamera:
    """Manages one splatsim Docker container for a single RGB camera.

    Replaces a CARLA ``sensor.camera.rgb`` sensor with Gaussian Splatting
    rendering.  The Docker container publishes ``Image`` and ``CameraInfo``
    messages directly via CycloneDDS.
    """

    def __init__(
        self,
        sensor_spec: dict,
        *,
        tileset_path: str,
        splatsim_image: str = "splatsim:latest",
        grpc_port: int = 50051,
        use_sh: bool = True,
        frame_rate: float = 20.0,
        image_topic: str = "/splatsim/image_raw",
        camera_info_topic: str = "/splatsim/camera_info",
        frame_id: str = "splatsim_camera",
        near_plane: float = 0.01,
        far_plane: float = 1000.0,
        device: str = "cuda:0",
        restart_container: bool = False,
    ) -> None:
        self._sensor_id = sensor_spec["id"]

        # Camera intrinsics from objects.json
        cam_w = sensor_spec["image_size_x"]
        cam_h = sensor_spec["image_size_y"]
        cam_fov = sensor_spec["fov"]

        # Camera extrinsic (actor -> camera) as CARLA Transform
        sp = sensor_spec["spawn_point"]
        self._sensor_transform = carla.Transform(
            carla.Location(x=sp["x"], y=sp["y"], z=sp["z"]),
            carla.Rotation(roll=sp["roll"], pitch=sp["pitch"], yaw=sp["yaw"]),
        )

        # ── Docker container ──
        container_name = f"splatsim_{self._sensor_id}"
        self._docker = SplatSimDockerManager(
            image=splatsim_image, grpc_port=grpc_port,
            container_name=container_name,
            force_restart=restart_container,
        )
        self._docker.start(tileset_path)
        self._docker.wait_for_ready(timeout=120.0)

        # ── gRPC Initialize ──
        fx, fy, cx, cy = _fov_to_intrinsics(cam_w, cam_h, cam_fov)
        container_tileset = f"/data/{Path(tileset_path).name}"

        self._grpc = SplatSimGrpcClient(address=self._docker.grpc_address)
        init_request = pb2.InitializeRequest(
            tileset_path=container_tileset,
            use_sh=use_sh,
            intrinsics=pb2.CameraIntrinsics(
                fx=fx, fy=fy, cx=cx, cy=cy, width=cam_w, height=cam_h,
            ),
            frame_rate=frame_rate,
            image_topic=image_topic,
            camera_info_topic=camera_info_topic,
            frame_id=frame_id,
            near_plane=near_plane,
            far_plane=far_plane,
            device=device,
            background_color=pb2.Vector3(x=0.0, y=0.0, z=0.0),
        )
        resp = self._grpc.initialize(init_request)
        if not resp.success:
            raise RuntimeError(f"splatsim Initialize failed: {resp.message}")

        # ── Coordinate transformer ──
        ecef_rot, ecef_trans = parse_tileset_transform(tileset_path)
        scene_origin = np.array(
            [resp.scene_origin.x, resp.scene_origin.y, resp.scene_origin.z],
            dtype=np.float64,
        )
        self._transformer = CoordinateTransformer(
            ecef_rotation=ecef_rot,
            ecef_translation=ecef_trans,
            scene_origin=scene_origin,
        )

        # ── Start streaming ──
        self._grpc.start_stream()
        logger.info("SplatSimRGBCamera '%s' initialized", self._sensor_id)

    def update(
        self,
        actor_matrix_4x4: list[list[float]],
        stamp_sec: int,
        stamp_nanosec: int,
    ) -> None:
        """Compute camera pose in tile-local coordinates and send to splatsim.

        Parameters
        ----------
        actor_matrix_4x4 : list[list[float]]
            Raw 4x4 world-to-actor matrix from ``carla.Transform.get_matrix()``.
        """
        T_world_actor = np.array(actor_matrix_4x4, dtype=np.float64)
        T_actor_camera = np.asarray(
            self._sensor_transform.get_matrix(), dtype=np.float64,
        )
        T_carla_cam = T_world_actor @ T_actor_camera

        carla_pos = T_carla_cam[:3, 3]
        R_carla = T_carla_cam[:3, :3]

        # Position: CARLA → ENU (flip y) → tile-local
        enu_pos = _S_CARLA_TO_ENU @ carla_pos
        tile_pos = self._transformer.enu_position_to_tile_local(
            enu_pos[0], enu_pos[1], enu_pos[2],
        )

        # Rotation: CARLA → ENU (flip y) → tile-local
        R_enu = _S_CARLA_TO_ENU @ R_carla
        R_tile = self._transformer.enu_rotation_to_tile_local(R_enu)
        quat_wxyz = _rotation_matrix_to_quaternion_wxyz(R_tile)

        self._grpc.send_camera_data(
            sec=stamp_sec,
            nanosec=stamp_nanosec,
            position=(float(tile_pos[0]), float(tile_pos[1]), float(tile_pos[2])),
            rotation_wxyz=quat_wxyz,
        )

    def shutdown(self) -> None:
        """Close gRPC stream and stop Docker container."""
        if self._grpc is None:
            return
        logger.info("Shutting down SplatSimRGBCamera '%s'", self._sensor_id)
        self._grpc.close_stream()
        self._grpc.close()
        self._grpc = None
        self._docker.stop()
        self._docker = None

    def __del__(self) -> None:
        self.shutdown()
