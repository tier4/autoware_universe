# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.sr/bin/env python

import json
import math
import re
import threading
import xml.etree.ElementTree as ET

from autoware_perception_msgs.msg import PredictedObjects
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import VelocityReport
from builtin_interfaces.msg import Time
import carla
from cv_bridge import CvBridge
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from transforms3d.euler import euler2quat

from .modules.carla_data_provider import CarlaDataProvider
from .modules.carla_data_provider import GameTime
from .modules.carla_data_provider import datetime
from .modules.carla_utils import carla_location_to_ros_point
from .modules.carla_utils import carla_rotation_to_ros_quaternion
from .modules.carla_utils import create_cloud
from .modules.carla_utils import ros_pose_to_carla_transform
from .modules.carla_wrapper import SensorInterface


def _parse_geo_reference(xodr_xml: str):
    """Extract ``(lat_0, lon_0)`` from the OpenDRIVE ``<geoReference>`` PROJ string.

    Ported from ``splatsim.carla_integration.geo_transform.parse_geo_reference``.
    """
    match = re.search(
        r"<geoReference>\s*<!\[CDATA\[(.*?)\]\]>\s*</geoReference>",
        xodr_xml,
        re.DOTALL,
    )
    if match:
        proj_string = match.group(1).strip()
    else:
        root = ET.fromstring(xodr_xml)
        geo_ref = root.find(".//geoReference")
        if geo_ref is not None and geo_ref.text:
            proj_string = geo_ref.text.strip()
        else:
            raise ValueError("No <geoReference> found in OpenDRIVE XML")

    lat_match = re.search(r"\+lat_0=([0-9eE.+-]+)", proj_string)
    lon_match = re.search(r"\+lon_0=([0-9eE.+-]+)", proj_string)
    if lat_match is None or lon_match is None:
        raise ValueError(
            f"Cannot extract +lat_0/+lon_0 from GeoReference: {proj_string}"
        )
    return float(lat_match.group(1)), float(lon_match.group(1))



class carla_ros2_interface(object):
    def __init__(self):
        self.sensor_interface = SensorInterface()
        self.timestamp = None
        self.ego_actor = None
        self.physics_control = None
        self.channels = 0
        self.id_to_sensor_type_map = {}
        self.id_to_camera_info_map = {}
        self.cv_bridge = CvBridge()
        self.first_ = True
        self.pub_lidar = {}
        self.sensor_frequencies = {
            "top": 11,
            "left": 11,
            "right": 11,
            "camera": 11,
            "imu": 50,
            "status": 50,
            "pose": 2,
        }
        self.publish_prev_times = {
            sensor: datetime.datetime.now() for sensor in self.sensor_frequencies
        }

        # initialize ros2 node
        rclpy.init(args=None)
        self.ros2_node = rclpy.create_node("carla_ros2_interface")
        self.parameters = {
            "host": rclpy.Parameter.Type.STRING,
            "port": rclpy.Parameter.Type.INTEGER,
            "sync_mode": rclpy.Parameter.Type.BOOL,
            "timeout": rclpy.Parameter.Type.INTEGER,
            "fixed_delta_seconds": rclpy.Parameter.Type.DOUBLE,
            "carla_map": rclpy.Parameter.Type.STRING,
            "ego_vehicle_role_name": rclpy.Parameter.Type.STRING,
            "spawn_point": rclpy.Parameter.Type.STRING,
            "vehicle_type": rclpy.Parameter.Type.STRING,
            "objects_definition_file": rclpy.Parameter.Type.STRING,
            "use_traffic_manager": rclpy.Parameter.Type.BOOL,
            "max_real_delta_seconds": rclpy.Parameter.Type.DOUBLE,
            "render_with_splatsim": rclpy.Parameter.Type.BOOL,
            "splatsim_tileset_path": rclpy.Parameter.Type.STRING,
            "splatsim_image": rclpy.Parameter.Type.STRING,
            "splatsim_grpc_port": rclpy.Parameter.Type.INTEGER,
            "splatsim_use_sh": rclpy.Parameter.Type.BOOL,
            "splatsim_frame_rate": rclpy.Parameter.Type.DOUBLE,
            "splatsim_image_topic": rclpy.Parameter.Type.STRING,
            "splatsim_camera_info_topic": rclpy.Parameter.Type.STRING,
            "splatsim_frame_id": rclpy.Parameter.Type.STRING,
            "splatsim_near_plane": rclpy.Parameter.Type.DOUBLE,
            "splatsim_far_plane": rclpy.Parameter.Type.DOUBLE,
            "splatsim_device": rclpy.Parameter.Type.STRING,
        }
        self.param_values = {}
        for param_name, param_type in self.parameters.items():
            self.ros2_node.declare_parameter(param_name, param_type)
            self.param_values[param_name] = self.ros2_node.get_parameter(param_name).value

        self.render_with_splatsim = bool(self.param_values.get("render_with_splatsim", False))
        self._splatsim_cameras = []
        self._mgrs_offset_x = 0.0
        self._mgrs_offset_y = 0.0

        # Publish clock
        self.clock_publisher = self.ros2_node.create_publisher(Clock, "/clock", 10)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=0)
        self.clock_publisher.publish(obj_clock)

        # Sensor Config (Edit your sensor here)
        self.sensors = json.load(open(self.param_values["objects_definition_file"]))

        # Subscribing Autoware Control messages and converting to CARLA control
        self.sub_control = self.ros2_node.create_subscription(
            ActuationCommandStamped, "/control/command/actuation_cmd", self.control_callback, 1
        )

        self.sub_vehicle_initialpose = self.ros2_node.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.initialpose_callback, 1
        )

        self.current_control = carla.VehicleControl()

        # Direct data publishing from CARLA for Autoware
        self.pub_pose_with_cov = self.ros2_node.create_publisher(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", 1
        )
        self.pub_vel_state = self.ros2_node.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 1
        )
        self.pub_steering_state = self.ros2_node.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 1
        )
        self.pub_ctrl_mode = self.ros2_node.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 1
        )
        self.pub_gear_state = self.ros2_node.create_publisher(
            GearReport, "/vehicle/status/gear_status", 1
        )
        self.pub_actuation_status = self.ros2_node.create_publisher(
            ActuationStatusStamped, "/vehicle/status/actuation_status", 1
        )

        # Create Publisher for each Physical Sensors
        self._splatsim_camera_specs = []
        for sensor in self.sensors["sensors"]:
            # When splatsim is active, skip camera/lidar (not spawned in CARLA)
            if self.render_with_splatsim and sensor["type"].startswith(
                ("sensor.camera", "sensor.lidar")
            ):
                if sensor["type"] == "sensor.camera.rgb":
                    self._splatsim_camera_specs.append(sensor)
                continue

            self.id_to_sensor_type_map[sensor["id"]] = sensor["type"]
            if sensor["type"] == "sensor.camera.rgb":
                self.pub_camera = self.ros2_node.create_publisher(
                    Image, "/sensing/camera/traffic_light/image_raw", 1
                )
                self.pub_camera_info = self.ros2_node.create_publisher(
                    CameraInfo, "/sensing/camera/traffic_light/camera_info", 1
                )
            elif sensor["type"] == "sensor.lidar.ray_cast":
                if sensor["id"] in self.sensor_frequencies:
                    self.pub_lidar[sensor["id"]] = self.ros2_node.create_publisher(
                        PointCloud2, f'/sensing/lidar/{sensor["id"]}/pointcloud_before_sync', 10
                    )
                else:
                    self.ros2_node.get_logger().info(
                        "Please use Top, Right, or Left as the LIDAR ID"
                    )
            elif sensor["type"] == "sensor.other.imu":
                self.pub_imu = self.ros2_node.create_publisher(
                    Imu, "/sensing/imu/tamagawa/imu_raw", 1
                )
            else:
                self.ros2_node.get_logger().info(f'No Publisher for {sensor["type"]} Sensor')
                pass

        # When splatsim is active, publish dummy perception and localization data
        if self.render_with_splatsim:
            self.pub_empty_objects = self.ros2_node.create_publisher(
                PredictedObjects, "/perception/object_recognition/objects", 1
            )
            self.pub_empty_pointcloud = self.ros2_node.create_publisher(
                PointCloud2, "/perception/obstacle_segmentation/pointcloud", 1
            )
            # Localization publishers
            self.pub_tf = self.ros2_node.create_publisher(TFMessage, "/tf", 10)
            self.pub_localization_odom = self.ros2_node.create_publisher(
                Odometry, "/localization/kinematic_state", 10
            )
            self.pub_localization_pose = self.ros2_node.create_publisher(
                PoseWithCovarianceStamped,
                "/localization/pose_estimator/pose_with_covariance",
                10,
            )
            self.pub_localization_accel = self.ros2_node.create_publisher(
                AccelWithCovarianceStamped, "/localization/acceleration", 10
            )
            self._latest_imu_accel = None

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros2_node,))
        self.spin_thread.start()

    def __call__(self):
        input_data = self.sensor_interface.get_data()
        timestamp = GameTime.get_time()
        control = self.run_step(input_data, timestamp)
        return control

    def get_param(self):
        return self.param_values

    def checkFrequency(self, sensor):
        time_delta = (
            datetime.datetime.now() - self.publish_prev_times[sensor]
        ).microseconds / 1000000.0
        if 1.0 / time_delta >= self.sensor_frequencies[sensor]:
            return True
        return False

    def get_msg_header(self, frame_id):
        """Obtain and modify ROS message header."""
        header = Header()
        header.frame_id = frame_id
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header

    def lidar(self, carla_lidar_measurement, id_):
        """Transform the received lidar measurement into a ROS point cloud message."""
        if self.checkFrequency(id_):
            return
        self.publish_prev_times[id_] = datetime.datetime.now()

        header = self.get_msg_header(frame_id="velodyne_top_changed")
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
        ]

        lidar_data = numpy.frombuffer(
            carla_lidar_measurement.raw_data, dtype=numpy.float32
        ).reshape(-1, 4)
        intensity = lidar_data[:, 3]
        intensity = (
            numpy.clip(intensity, 0, 1) * 255
        )  # CARLA lidar intensity values are between 0 and 1
        intensity = intensity.astype(numpy.uint8).reshape(-1, 1)

        return_type = numpy.zeros((lidar_data.shape[0], 1), dtype=numpy.uint8)
        channel = numpy.empty((0, 1), dtype=numpy.uint16)
        self.channels = self.sensors["sensors"]

        for i in range(self.channels[1]["channels"]):
            current_ring_points_count = carla_lidar_measurement.get_point_count(i)
            channel = numpy.vstack(
                (channel, numpy.full((current_ring_points_count, 1), i, dtype=numpy.uint16))
            )

        lidar_data = numpy.hstack((lidar_data[:, :3], intensity, return_type, channel))
        lidar_data[:, 1] *= -1

        dtype = [
            ("x", "f4"),
            ("y", "f4"),
            ("z", "f4"),
            ("intensity", "u1"),
            ("return_type", "u1"),
            ("channel", "u2"),
        ]

        structured_lidar_data = numpy.zeros(lidar_data.shape[0], dtype=dtype)
        structured_lidar_data["x"] = lidar_data[:, 0]
        structured_lidar_data["y"] = lidar_data[:, 1]
        structured_lidar_data["z"] = lidar_data[:, 2]
        structured_lidar_data["intensity"] = lidar_data[:, 3].astype(numpy.uint8)
        structured_lidar_data["return_type"] = lidar_data[:, 4].astype(numpy.uint8)
        structured_lidar_data["channel"] = lidar_data[:, 5].astype(numpy.uint16)

        point_cloud_msg = create_cloud(header, fields, structured_lidar_data)
        self.pub_lidar[id_].publish(point_cloud_msg)

    def initialpose_callback(self, data):
        """Transform RVIZ initial pose to CARLA."""
        pose = data.pose.pose
        pose.position.z += 2.0
        carla_pose_transform = ros_pose_to_carla_transform(pose)
        if self.ego_actor is not None:
            self.ego_actor.set_transform(carla_pose_transform)
        else:
            print("Can't find Ego Vehicle")

    def pose(self):
        """Transform odometry data to Pose and publish Pose with Covariance message."""
        if self.checkFrequency("pose"):
            return
        self.publish_prev_times["pose"] = datetime.datetime.now()

        header = self.get_msg_header(frame_id="map")
        out_pose_with_cov = PoseWithCovarianceStamped()
        pose_carla = Pose()
        pose_carla.position = carla_location_to_ros_point(self.ego_actor.get_transform().location)
        pose_carla.orientation = carla_rotation_to_ros_quaternion(
            self.ego_actor.get_transform().rotation
        )
        out_pose_with_cov.header = header
        out_pose_with_cov.pose.pose = pose_carla
        out_pose_with_cov.pose.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.pub_pose_with_cov.publish(out_pose_with_cov)

    def _build_camera_info(self, camera_actor):
        """Build camera info."""
        camera_info = CameraInfo()
        camera_info.width = camera_actor.width
        camera_info.height = camera_actor.height
        camera_info.distortion_model = "plumb_bob"
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (2.0 * math.tan(camera_actor.fov * math.pi / 360.0))
        fy = fx
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._camera_info = camera_info

    def camera(self, carla_camera_data):
        """Transform the received carla camera data into a ROS image and info message and publish."""
        while self.first_:
            self._camera_info_ = self._build_camera_info(carla_camera_data)
            self.first_ = False

        if self.checkFrequency("camera"):
            return
        self.publish_prev_times["camera"] = datetime.datetime.now()

        image_data_array = numpy.ndarray(
            shape=(carla_camera_data.height, carla_camera_data.width, 4),
            dtype=numpy.uint8,
            buffer=carla_camera_data.raw_data,
        )
        # cspell:ignore interp bgra
        img_msg = self.cv_bridge.cv2_to_imgmsg(image_data_array, encoding="bgra8")
        img_msg.header = self.get_msg_header(
            frame_id="traffic_light_left_camera/camera_optical_link"
        )
        cam_info = self._camera_info
        cam_info.header = img_msg.header
        self.pub_camera_info.publish(cam_info)
        self.pub_camera.publish(img_msg)

    def imu(self, carla_imu_measurement):
        """Transform a received imu measurement into a ROS Imu message and publish Imu message."""
        if self.checkFrequency("imu"):
            return
        self.publish_prev_times["imu"] = datetime.datetime.now()

        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(frame_id="tamagawa/imu_link_changed")
        imu_msg.angular_velocity.x = -carla_imu_measurement.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.angular_velocity.z = -carla_imu_measurement.gyroscope.z

        imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
        imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        roll = math.radians(carla_imu_measurement.transform.rotation.roll)
        pitch = -math.radians(carla_imu_measurement.transform.rotation.pitch)
        yaw = -math.radians(carla_imu_measurement.transform.rotation.yaw)

        quat = euler2quat(roll, pitch, yaw)
        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        self.pub_imu.publish(imu_msg)

        # Cache acceleration for localization publishing
        if self.render_with_splatsim:
            self._latest_imu_accel = (
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z,
            )

    def control_callback(self, in_cmd):
        """Convert and publish CARLA Ego Vehicle Control to AUTOWARE."""
        out_cmd = carla.VehicleControl()
        out_cmd.throttle = in_cmd.actuation.accel_cmd
        # convert base on steer curve of the vehicle
        steer_curve = self.physics_control.steering_curve
        current_vel = self.ego_actor.get_velocity()
        max_steer_ratio = numpy.interp(
            abs(current_vel.x), [v.x for v in steer_curve], [v.y for v in steer_curve]
        )
        out_cmd.steer = (
            -in_cmd.actuation.steer_cmd
            * max_steer_ratio
            * math.radians(self.physics_control.wheels[0].max_steer_angle)
        )
        out_cmd.brake = in_cmd.actuation.brake_cmd
        self.current_control = out_cmd

    def ego_status(self):
        """Publish ego vehicle status."""
        if self.checkFrequency("status"):
            return

        self.publish_prev_times["status"] = datetime.datetime.now()

        # convert velocity from cartesian to ego frame
        trans_mat = numpy.array(self.ego_actor.get_transform().get_matrix()).reshape(4, 4)
        rot_mat = trans_mat[0:3, 0:3]
        inv_rot_mat = rot_mat.T
        vel_vec = numpy.array(
            [
                self.ego_actor.get_velocity().x,
                self.ego_actor.get_velocity().y,
                self.ego_actor.get_velocity().z,
            ]
        ).reshape(3, 1)
        ego_velocity = (inv_rot_mat @ vel_vec).T[0]

        out_vel_state = VelocityReport()
        out_steering_state = SteeringReport()
        out_ctrl_mode = ControlModeReport()
        out_gear_state = GearReport()
        out_actuation_status = ActuationStatusStamped()

        out_vel_state.header = self.get_msg_header(frame_id="base_link")
        out_vel_state.longitudinal_velocity = ego_velocity[0]
        out_vel_state.lateral_velocity = -ego_velocity[1]  # CARLA right+ → ROS left+
        # CARLA get_angular_velocity() returns deg/s; Autoware expects rad/s.
        # Negate for left-handed (CW+) → right-handed (CCW+) convention.
        out_vel_state.heading_rate = -math.radians(
            self.ego_actor.get_angular_velocity().z
        )

        out_steering_state.stamp = out_vel_state.header.stamp
        out_steering_state.steering_tire_angle = -math.radians(
            self.ego_actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        )

        out_gear_state.stamp = out_vel_state.header.stamp
        out_gear_state.report = GearReport.DRIVE

        out_ctrl_mode.stamp = out_vel_state.header.stamp
        out_ctrl_mode.mode = ControlModeReport.AUTONOMOUS

        control = self.ego_actor.get_control()
        out_actuation_status.header = self.get_msg_header(frame_id="base_link")
        out_actuation_status.status.accel_status = control.throttle
        out_actuation_status.status.brake_status = control.brake
        out_actuation_status.status.steer_status = -control.steer

        self.pub_actuation_status.publish(out_actuation_status)
        self.pub_vel_state.publish(out_vel_state)
        self.pub_steering_state.publish(out_steering_state)
        self.pub_ctrl_mode.publish(out_ctrl_mode)
        self.pub_gear_state.publish(out_gear_state)

    def _init_geo_transform(self):
        """Compute MGRS offset from CARLA OpenDRIVE GeoReference.

        The xodr coordinate origin corresponds to the geographic point
        (lat_0, lon_0) in the GeoReference.  The MGRS offset is the
        projected coordinate of this origin in the lanelet2 MGRSProjector,
        so that::

            autoware_map_x = carla_x + mgrs_offset_x
            autoware_map_y = -carla_y + mgrs_offset_y

        Ported from ``splatsim.carla_integration.geo_transform.GeoTransform``.
        """
        import lanelet2.core
        import lanelet2.io
        from autoware_lanelet2_extension_python.projection import MGRSProjector

        world = CarlaDataProvider.get_world()
        xodr_xml = world.get_map().to_opendrive()
        lat_0, lon_0 = _parse_geo_reference(xodr_xml)

        projector = MGRSProjector(lanelet2.io.Origin(lat_0, lon_0))
        origin_gps = lanelet2.core.GPSPoint(lat_0, lon_0, 0.0)
        origin_local = projector.forward(origin_gps)
        self._mgrs_offset_x = origin_local.x
        self._mgrs_offset_y = origin_local.y

        self.ros2_node.get_logger().info(
            f"GeoTransform initialized: lat_0={lat_0:.8f}, lon_0={lon_0:.8f}, "
            f"mgrs_offset=({self._mgrs_offset_x:.1f}, {self._mgrs_offset_y:.1f})"
        )

    def init_splatsim_cameras(self):
        """Create SplatSimRGBCamera instances for each camera sensor spec.

        Must be called after the CARLA world is loaded (ego_actor is set).
        """
        if not self.render_with_splatsim or not self._splatsim_camera_specs:
            return
        if not self.param_values.get("splatsim_tileset_path"):
            self.ros2_node.get_logger().warn(
                "render_with_splatsim is true but splatsim_tileset_path is empty; "
                "skipping splatsim camera initialization"
            )
            return

        self._init_geo_transform()

        from .splatsim.splatsim_camera import SplatSimRGBCamera

        p = self.param_values
        for spec in self._splatsim_camera_specs:
            cam = SplatSimRGBCamera(
                spec,
                tileset_path=p["splatsim_tileset_path"],
                splatsim_image=p["splatsim_image"],
                grpc_port=p["splatsim_grpc_port"],
                use_sh=p["splatsim_use_sh"],
                frame_rate=p["splatsim_frame_rate"],
                image_topic=p["splatsim_image_topic"],
                camera_info_topic=p["splatsim_camera_info_topic"],
                frame_id=p["splatsim_frame_id"],
                near_plane=p["splatsim_near_plane"],
                far_plane=p["splatsim_far_plane"],
                device=p["splatsim_device"],
            )
            self._splatsim_cameras.append(cam)
            self.ros2_node.get_logger().info(
                f"SplatSimRGBCamera created for sensor '{spec['id']}'"
            )

    def _publish_localization(self):
        """Publish localization data from CARLA ground truth when splatsim is active.

        Uses lanelet2 coordinate system (2D pose) instead of converting
        CARLA's 3D rotation matrix.  Matches the approach in
        ``splatsim.carla_integration.autoware_bridge.publish_initialpose``.

        Position::
            CARLA (x=East, y=South, z=Up)
              → xodr  (flip y: y_xodr = −y_carla)
              → MGRS  (add MGRS offset from lanelet2 projector)

        Orientation (2D yaw only, roll=pitch=0)::
            CARLA yaw (degrees, CW-positive)
              → ROS yaw (radians, CCW-positive): yaw_ros = −deg2rad(yaw_carla)
        """
        header = self.get_msg_header(frame_id="map")
        carla_tf = self.ego_actor.get_transform()

        # --- Position: CARLA → xodr (flip y) → MGRS absolute (add offset) ---
        pose = Pose()
        pose.position.x = carla_tf.location.x + self._mgrs_offset_x
        pose.position.y = -carla_tf.location.y + self._mgrs_offset_y
        pose.position.z = carla_tf.location.z

        # --- Orientation: 2D yaw only (same as autoware_bridge.publish_initialpose) ---
        # CARLA yaw: degrees, 0=East, positive=clockwise (right)
        # ROS yaw:   radians, 0=East, positive=counter-clockwise (left)
        yaw = -math.radians(carla_tf.rotation.yaw)
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        # --- Direct /tf publish (map → base_link) ---
        # Bypasses EKF/NDT — splatsim provides ground-truth localization.
        tf_stamped = TransformStamped()
        tf_stamped.header = header
        tf_stamped.child_frame_id = "base_link"
        tf_stamped.transform.translation.x = pose.position.x
        tf_stamped.transform.translation.y = pose.position.y
        tf_stamped.transform.translation.z = pose.position.z
        tf_stamped.transform.rotation = pose.orientation
        tf_msg = TFMessage(transforms=[tf_stamped])
        self.pub_tf.publish(tf_msg)

        # --- Direct /localization/kinematic_state (Odometry) ---
        odom = Odometry()
        odom.header = header
        odom.child_frame_id = "base_link"
        odom.pose.pose = pose
        # Velocity in base_link frame (reuse ego_status logic)
        trans_mat = numpy.array(
            self.ego_actor.get_transform().get_matrix()
        ).reshape(4, 4)
        rot_mat = trans_mat[0:3, 0:3]
        inv_rot_mat = rot_mat.T
        vel_vec = numpy.array([
            self.ego_actor.get_velocity().x,
            self.ego_actor.get_velocity().y,
            self.ego_actor.get_velocity().z,
        ]).reshape(3, 1)
        ego_velocity = (inv_rot_mat @ vel_vec).T[0]
        odom.twist.twist.linear.x = float(ego_velocity[0])
        odom.twist.twist.linear.y = float(-ego_velocity[1])  # CARLA right+ → ROS left+
        odom.twist.twist.linear.z = float(ego_velocity[2])
        odom.twist.twist.angular.z = -math.radians(
            self.ego_actor.get_angular_velocity().z
        )
        self.pub_localization_odom.publish(odom)

        # --- Pose with covariance (for pose_estimator topic) ---
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = header
        pose_cov.pose.pose = pose
        pcov = pose_cov.pose.covariance
        pcov[0] = 0.0001
        pcov[7] = 0.0001
        pcov[14] = 0.0001
        pcov[21] = 0.0001
        pcov[28] = 0.0001
        pcov[35] = 0.0001
        self.pub_localization_pose.publish(pose_cov)

        # AccelWithCovarianceStamped
        accel_msg = AccelWithCovarianceStamped()
        accel_msg.header = self.get_msg_header(frame_id="base_link")
        if self._latest_imu_accel is not None:
            accel_msg.accel.accel.linear.x = self._latest_imu_accel[0]
            accel_msg.accel.accel.linear.y = self._latest_imu_accel[1]
            accel_msg.accel.accel.linear.z = self._latest_imu_accel[2]
        self.pub_localization_accel.publish(accel_msg)

    def run_step(self, input_data, timestamp):
        self.timestamp = timestamp
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock_publisher.publish(obj_clock)

        # publish data of all sensors
        for key, data in input_data.items():
            sensor_type = self.id_to_sensor_type_map[key]
            if sensor_type == "sensor.camera.rgb":
                self.camera(data[1])
            elif sensor_type == "sensor.other.gnss":
                # Skip GNSS pose when splatsim provides localization directly;
                # the GNSS pose lacks the MGRS offset and would conflict with
                # the corrected localization published by _publish_localization.
                if not self.render_with_splatsim:
                    self.pose()
            elif sensor_type == "sensor.lidar.ray_cast":
                self.lidar(data[1], key)
            elif sensor_type == "sensor.other.imu":
                self.imu(data[1])
            else:
                self.ros2_node.get_logger().info("No Publisher for [{key}] Sensor")

        # Send ego pose to splatsim cameras
        if self._splatsim_cameras and self.ego_actor is not None:
            pos = carla_location_to_ros_point(self.ego_actor.get_transform().location)
            quat = carla_rotation_to_ros_quaternion(
                self.ego_actor.get_transform().rotation
            )
            for cam in self._splatsim_cameras:
                cam.update(
                    ego_position=(pos.x, pos.y, pos.z),
                    ego_quaternion_xyzw=(quat.x, quat.y, quat.z, quat.w),
                    stamp_sec=seconds,
                    stamp_nanosec=nanoseconds,
                )

        # Publish dummy perception and localization data when splatsim is active
        if self.render_with_splatsim:
            header = self.get_msg_header(frame_id="map")
            empty_objects = PredictedObjects()
            empty_objects.header = header
            self.pub_empty_objects.publish(empty_objects)

            empty_pc = PointCloud2()
            empty_pc.header = self.get_msg_header(frame_id="base_link")
            self.pub_empty_pointcloud.publish(empty_pc)

            if self.ego_actor is not None:
                self._publish_localization()

        # Publish ego vehicle status
        self.ego_status()
        return self.current_control

    def shutdown(self):
        for cam in self._splatsim_cameras:
            cam.shutdown()
        self._splatsim_cameras.clear()
        self.ros2_node.destroy_node()
