"""ROS 2 node that bridges CARLA ego pose to splatsim via gRPC.

Responsibilities:
  1. Start the ``splatsim:latest`` Docker container
  2. Initialize the gRPC RenderingService (scene, camera, DDS topics)
  3. Subscribe to CARLA ground-truth pose and stream camera poses
  4. Shut down the container on exit
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock

from autoware_carla_interface.splatsim.docker_manager import SplatSimDockerManager
from autoware_carla_interface.splatsim.grpc_client import SplatSimGrpcClient
from autoware_carla_interface.splatsim.coordinate_transformer import (
    CoordinateTransformer,
    parse_tileset_transform,
    _quaternion_xyzw_to_rotation_matrix,
)
from autoware_carla_interface.splatsim.proto import rendering_service_pb2 as pb2


def _fov_to_intrinsics(
    width: int, height: int, fov_deg: float,
) -> tuple[float, float, float, float]:
    """Convert horizontal FOV + resolution to ``(fx, fy, cx, cy)``."""
    fov_rad = math.radians(fov_deg)
    fx = width / (2.0 * math.tan(fov_rad / 2.0))
    fy = fx  # square pixels
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
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


class SplatSimBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("splatsim_bridge")

        # ── Parameters ──
        self.declare_parameter("tileset_path", "")
        self.declare_parameter("splatsim_image", "splatsim:latest")
        self.declare_parameter("grpc_port", 50051)
        self.declare_parameter("use_sh", True)
        self.declare_parameter("camera_width", 1920)
        self.declare_parameter("camera_height", 1080)
        self.declare_parameter("camera_fov", 90.0)
        self.declare_parameter("camera_extrinsic_x", 0.7)
        self.declare_parameter("camera_extrinsic_y", 0.0)
        self.declare_parameter("camera_extrinsic_z", 1.6)
        self.declare_parameter("camera_extrinsic_roll", 0.0)
        self.declare_parameter("camera_extrinsic_pitch", 0.0)
        self.declare_parameter("camera_extrinsic_yaw", 0.0)
        self.declare_parameter("frame_rate", 20.0)
        self.declare_parameter("image_topic", "/splatsim/image_raw")
        self.declare_parameter("camera_info_topic", "/splatsim/camera_info")
        self.declare_parameter("frame_id", "splatsim_camera")
        self.declare_parameter("near_plane", 0.01)
        self.declare_parameter("far_plane", 1000.0)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("background_color", [0.0, 0.0, 0.0])
        # Geo-reference from CARLA xodr
        self.declare_parameter("proj_origin_lat", 0.0)
        self.declare_parameter("proj_origin_lon", 0.0)

        tileset_path = self.get_parameter("tileset_path").value
        if not tileset_path:
            self.get_logger().fatal("tileset_path is required")
            raise RuntimeError("tileset_path is required")

        image = self.get_parameter("splatsim_image").value
        grpc_port = self.get_parameter("grpc_port").value
        cam_w = self.get_parameter("camera_width").value
        cam_h = self.get_parameter("camera_height").value
        cam_fov = self.get_parameter("camera_fov").value
        frame_rate = self.get_parameter("frame_rate").value
        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        frame_id = self.get_parameter("frame_id").value

        # Camera extrinsic (base_link → camera)
        ex = self.get_parameter("camera_extrinsic_x").value
        ey = self.get_parameter("camera_extrinsic_y").value
        ez = self.get_parameter("camera_extrinsic_z").value
        eroll = math.radians(self.get_parameter("camera_extrinsic_roll").value)
        epitch = math.radians(self.get_parameter("camera_extrinsic_pitch").value)
        eyaw = math.radians(self.get_parameter("camera_extrinsic_yaw").value)
        self._T_ego_camera = _build_extrinsic_matrix(ex, ey, ez, eroll, epitch, eyaw)

        # ── Docker ──
        self._docker = SplatSimDockerManager(image=image, grpc_port=grpc_port)
        self._docker.start(tileset_path)
        self._docker.wait_for_ready(timeout=120.0)

        # ── gRPC Initialize ──
        fx, fy, cx, cy = _fov_to_intrinsics(cam_w, cam_h, cam_fov)
        bg = self.get_parameter("background_color").value

        self._grpc = SplatSimGrpcClient(address=self._docker.grpc_address)

        # Remap tileset_path to container mount point
        from pathlib import Path
        tileset_filename = Path(tileset_path).name
        container_tileset = f"/data/{tileset_filename}"

        init_request = pb2.InitializeRequest(
            tileset_path=container_tileset,
            use_sh=self.get_parameter("use_sh").value,
            intrinsics=pb2.CameraIntrinsics(
                fx=fx, fy=fy, cx=cx, cy=cy, width=cam_w, height=cam_h,
            ),
            frame_rate=frame_rate,
            image_topic=image_topic,
            camera_info_topic=camera_info_topic,
            frame_id=frame_id,
            near_plane=self.get_parameter("near_plane").value,
            far_plane=self.get_parameter("far_plane").value,
            device=self.get_parameter("device").value,
            background_color=pb2.Vector3(x=bg[0], y=bg[1], z=bg[2]),
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
        lat_0 = self.get_parameter("proj_origin_lat").value
        lon_0 = self.get_parameter("proj_origin_lon").value
        self._transformer = CoordinateTransformer(
            proj_origin=(lat_0, lon_0),
            ecef_rotation=ecef_rot,
            ecef_translation=ecef_trans,
            scene_origin=scene_origin,
        )

        # ── Start streaming ──
        self._grpc.start_stream()

        # ── Subscribers ──
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            self._pose_callback,
            10,
        )

        self.get_logger().info("SplatSim bridge node initialized")

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Convert ego pose to camera tile-local pose and stream to splatsim."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Build ego pose matrix in ENU
        R_ego = _quaternion_xyzw_to_rotation_matrix(q.x, q.y, q.z, q.w)
        T_enu_ego = np.eye(4, dtype=np.float64)
        T_enu_ego[:3, :3] = R_ego
        T_enu_ego[:3, 3] = [p.x, p.y, p.z]

        # Apply camera extrinsic offset
        T_enu_camera = T_enu_ego @ self._T_ego_camera
        cam_pos = T_enu_camera[:3, 3]
        R_cam_enu = T_enu_camera[:3, :3]

        # Convert to tile-local
        tile_pos = self._transformer.enu_position_to_tile_local(
            cam_pos[0], cam_pos[1], cam_pos[2],
        )
        R_tile = self._transformer.enu_rotation_to_tile_local(R_cam_enu)

        # Rotation matrix → wxyz quaternion
        from autoware_carla_interface.splatsim.coordinate_transformer import (
            _rotation_matrix_to_quaternion_wxyz,
        )
        quat_wxyz = _rotation_matrix_to_quaternion_wxyz(R_tile)

        stamp = msg.header.stamp
        self._grpc.send_camera_data(
            sec=stamp.sec,
            nanosec=stamp.nanosec,
            position=(float(tile_pos[0]), float(tile_pos[1]), float(tile_pos[2])),
            rotation_wxyz=quat_wxyz,
        )

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down splatsim bridge ...")
        self._grpc.close_stream()
        self._grpc.close()
        self._docker.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SplatSimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
