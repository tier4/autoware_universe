"""SplatSimRGBCamera -- one Docker container + gRPC stream per RGB camera."""

from __future__ import annotations

import logging
import math
from pathlib import Path

import numpy as np

from autoware_carla_interface.splatsim.docker_manager import SplatSimDockerManager
from autoware_carla_interface.splatsim.grpc_client import SplatSimGrpcClient
from autoware_carla_interface.splatsim.coordinate_transformer import (
    CoordinateTransformer,
    parse_tileset_transform,
    _quaternion_xyzw_to_rotation_matrix,
    _rotation_matrix_to_quaternion_wxyz,
)
from autoware_carla_interface.splatsim.proto import rendering_service_pb2 as pb2

logger = logging.getLogger(__name__)


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


def _build_extrinsic_matrix(
    x: float, y: float, z: float,
    roll: float, pitch: float, yaw: float,
) -> np.ndarray:
    """Build a 4x4 homogeneous transform from translation + Euler (radians)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy_, sy = math.cos(yaw), math.sin(yaw)
    R = np.array(
        [
            [cy_ * cp, cy_ * sp * sr - sy * cr, cy_ * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy_ * cr, sy * sp * cr - cy_ * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


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
        proj_origin_lat: float = 0.0,
        proj_origin_lon: float = 0.0,
    ) -> None:
        self._sensor_id = sensor_spec["id"]

        # Camera intrinsics from objects.json
        cam_w = sensor_spec["image_size_x"]
        cam_h = sensor_spec["image_size_y"]
        cam_fov = sensor_spec["fov"]

        # Camera extrinsic (base_link -> camera) from objects.json spawn_point
        sp = sensor_spec["spawn_point"]
        self._T_ego_camera = _build_extrinsic_matrix(
            sp["x"], sp["y"], sp["z"],
            math.radians(sp["roll"]),
            math.radians(sp["pitch"]),
            math.radians(sp["yaw"]),
        )

        # ── Docker container ──
        self._docker = SplatSimDockerManager(
            image=splatsim_image, grpc_port=grpc_port,
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
            proj_origin=(proj_origin_lat, proj_origin_lon),
            ecef_rotation=ecef_rot,
            ecef_translation=ecef_trans,
            scene_origin=scene_origin,
        )

        # ── Start streaming ──
        self._grpc.start_stream()
        logger.info("SplatSimRGBCamera '%s' initialized", self._sensor_id)

    def update(
        self,
        ego_position: tuple[float, float, float],
        ego_quaternion_xyzw: tuple[float, float, float, float],
        stamp_sec: int,
        stamp_nanosec: int,
    ) -> None:
        """Compute camera pose in tile-local coordinates and send to splatsim."""
        R_ego = _quaternion_xyzw_to_rotation_matrix(*ego_quaternion_xyzw)
        T_enu_ego = np.eye(4, dtype=np.float64)
        T_enu_ego[:3, :3] = R_ego
        T_enu_ego[:3, 3] = ego_position

        T_enu_camera = T_enu_ego @ self._T_ego_camera
        cam_pos = T_enu_camera[:3, 3]
        R_cam_enu = T_enu_camera[:3, :3]

        tile_pos = self._transformer.enu_position_to_tile_local(
            cam_pos[0], cam_pos[1], cam_pos[2],
        )
        R_tile = self._transformer.enu_rotation_to_tile_local(R_cam_enu)
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
