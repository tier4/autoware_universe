#!/usr/bin/env python3
"""
ROS2 Inference Node for naiveTF Planning Model

This node subscribes to Autoware topics, processes the data using the same
extraction logic as the bag extraction tool, runs inference with the naiveTF model,
and publishes predicted trajectories in the global frame.
"""

import os
import time
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS2 message types
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray
from autoware_planning_msgs.msg import LaneletRoute, Path, PathPoint
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

# Import from naiveTF and extract_bags
from autoware_naive_tf_planner.model import PlanningModel
from autoware_naive_tf_planner.maps.map_manager import MapManager
from autoware_naive_tf_planner.object_tracker.base_object_tracker import CurrentFrameObject, BaseTracker
from collections import deque
import scipy.spatial
from scipy.spatial import ConvexHull


LANELET_TYPE_MAPPING = {
    "road": 0,
    "private": 1,
    "highway": 2,
    "play_street": 3,
    "emergency_lane": 4,
    "bus_lane": 5,
    "bicycle_lane": 6,
    "exit": 7,
    "walkway": 8,
    "shared_walkway": 9,
    "crosswalk": 10,
    "stairs": 11,
    "road_shoulder": 12,
    "pedestrian_lane": 13,
    "bicycle_lane": 14,
    "none": 15,
}

LANELET_LOCATION_MAPPING = {"urban": 0, "nonurban": 1, "private": 2, "none": 3}

LANELET_TURN_DIRECTION_MAPPING = {"straight": 0, "left": 1, "right": 2}


def attribute_or(lanelet, key, default):
    if key in lanelet.attributes:
        return lanelet.attributes[key]
    return default


def fit_rotated_bbox(points: np.ndarray) -> np.ndarray:
    """
    Fit a minimum-area rotated bounding box around a set of points.
    
    Args:
        points: numpy array of shape (N, 2) or (N, 3) containing coordinates
    
    Returns:
        corners: numpy array of shape (4, 3) containing the corners of the rotated bounding box
    """
    # Convert to numpy if tensor
    if isinstance(points, torch.Tensor):
        points = points.numpy()
    
    # Store Z coordinates if they exist
    has_z = points.shape[1] > 2
    if has_z:
        z_min = np.min(points[:, 2])
        z_max = np.max(points[:, 2])
        z_val = z_min  # We'll use min z for the bbox corners
        points_2d = points[:, :2]
    else:
        z_val = 0
        points_2d = points
    
    # Remove duplicate 2D points
    points_2d = np.unique(points_2d, axis=0)
    
    # If we have less than 3 points, create a simple box
    if len(points_2d) < 3:
        min_coords = np.min(points_2d, axis=0)
        max_coords = np.max(points_2d, axis=0)
        corners = np.array([
            [min_coords[0], min_coords[1], z_val],
            [max_coords[0], min_coords[1], z_val],
            [max_coords[0], max_coords[1], z_val],
            [min_coords[0], max_coords[1], z_val]
        ])
        return torch.tensor(corners, dtype=torch.float32)
    
    try:
        # Try normal convex hull first
        hull = ConvexHull(points_2d)
        hull_points = points_2d[hull.vertices]
    except scipy.spatial._qhull.QhullError:
        # If that fails, try with jitter
        jitter = np.random.normal(0, 1e-8, points_2d.shape)
        points_jittered = points_2d + jitter
        try:
            hull = ConvexHull(points_jittered)
            hull_points = points_2d[hull.vertices]
        except scipy.spatial._qhull.QhullError:
            # If still fails, fall back to axis-aligned bounding box
            min_coords = np.min(points_2d, axis=0)
            max_coords = np.max(points_2d, axis=0)
            corners = np.array([
                [min_coords[0], min_coords[1], z_val],
                [max_coords[0], min_coords[1], z_val],
                [max_coords[0], max_coords[1], z_val],
                [min_coords[0], max_coords[1], z_val]
            ])
            return torch.tensor(corners, dtype=torch.float32)
    
    # Get the edges of the convex hull
    edges = np.roll(hull_points, -1, axis=0) - hull_points
    
    # Normalize the edges
    angles = np.arctan2(edges[:, 1], edges[:, 0])
    
    # Try all unique angles to find minimum area
    min_area = float('inf')
    best_corners = None
    
    for angle in angles:
        # Rotate points
        rotation = np.array([[np.cos(angle), -np.sin(angle)],
                           [np.sin(angle), np.cos(angle)]])
        rotated_points = np.dot(points_2d, rotation)
        
        # Get axis-aligned bounding box
        min_x, min_y = np.min(rotated_points, axis=0)
        max_x, max_y = np.max(rotated_points, axis=0)
        
        # Calculate area
        area = (max_x - min_x) * (max_y - min_y)
        
        if area < min_area:
            min_area = area
            
            # Generate corners of the bounding box
            corners_2d = np.array([
                [min_x, min_y],  # bottom-left
                [max_x, min_y],  # bottom-right
                [max_x, max_y],  # top-right
                [min_x, max_y],  # top-left
            ])
            
            # Rotate corners back
            rotation_inv = np.array([[np.cos(-angle), -np.sin(-angle)],
                                   [np.sin(-angle), np.cos(-angle)]])
            best_corners = np.dot(corners_2d, rotation_inv)
    
    # Add Z coordinate
    best_corners_3d = np.column_stack([best_corners, np.full(4, z_val)])
    
    return torch.tensor(best_corners_3d, dtype=torch.float32)

def identify_current_light_status(
    turn_direction: int, traffic_light_elements: list
) -> int:
    """
    Identify the current traffic light status based on turn direction and traffic light elements.

    Args:
        turn_direction: Integer representing the turn direction (0=straight, 1=left, 2=right)
        traffic_light_elements: List of dictionaries containing traffic light information
                               (color, shape, status, confidence)

    Returns:
        int: The color of the relevant traffic light (0=UNKNOWN, 1=RED, 2=AMBER, 3=GREEN, 4=WHITE)
    """
    # Filter out ineffective elements (color == 0)
    effective_elements = [
        element for element in traffic_light_elements if element["color"] != 0
    ]

    # If no effective elements, return UNKNOWN (0)
    if not effective_elements:
        return 0

    # If only one effective element, return its color
    if len(effective_elements) == 1:
        return effective_elements[0]["color"]

    # For multiple elements, find the one that matches the turn direction
    # Map turn direction to corresponding arrow shape
    direction_to_shape_map = {
        0: 4,  # straight -> UP_ARROW
        1: 2,  # left -> LEFT_ARROW
        2: 3,  # right -> RIGHT_ARROW
    }

    target_shape = direction_to_shape_map.get(turn_direction, 0)

    # First priority: Find elements with exactly matching direction
    matching_elements = [
        element for element in effective_elements if element["shape"] == target_shape
    ]
    if matching_elements:
        # If multiple matching elements, take the one with highest confidence
        return max(matching_elements, key=lambda x: x["confidence"])["color"]

    # Second priority: Find circle elements
    circle_elements = [
        element for element in effective_elements if element["shape"] == 1
    ]  # CIRCLE
    if circle_elements:
        # If multiple circle elements, take the one with highest confidence
        return max(circle_elements, key=lambda x: x["confidence"])["color"]

    # If no matching direction or circle, return the element with highest confidence
    return max(effective_elements, key=lambda x: x["confidence"])["color"]


def get_current_traffic_light(lanelet, current_traffic_light_status: dict):
    traffic_light_ids_connected_to_lanelet = lanelet.trafficLights()
    if len(traffic_light_ids_connected_to_lanelet) == 0:
        return 0  # unknown, empty

    main_traffic_light_id = str(traffic_light_ids_connected_to_lanelet[0].id)
    if main_traffic_light_id in current_traffic_light_status:
        current_traffic_group = current_traffic_light_status[main_traffic_light_id]
        turn_direction = LANELET_TURN_DIRECTION_MAPPING[
            attribute_or(lanelet, "turn_direction", "straight")
        ]
        light = identify_current_light_status(turn_direction, current_traffic_group)
        return light
    else:
        return 0 # the traffic light not detected


class NaiveTFInferenceNode(Node):
    def __init__(self):
        super().__init__('naive_tf_inference_node')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('map_path', '')
        self.declare_parameter('trace_back_step', 10)
        self.declare_parameter('look_ahead_steps', 30)
        self.declare_parameter('prediction_horizon', 3.0)  # seconds
        self.declare_parameter('prediction_dt', 0.1)       # seconds
        self.declare_parameter('local_map_range', 80.0)    # meters
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')

        self.declare_parameter('wheel_base', 0.0)
        self.declare_parameter('max_steer_angle', 0.0)
        self.declare_parameter('front_overhang', 0.0)
        self.declare_parameter('rear_overhang', 0.0)
        self.declare_parameter('left_overhang', 0.0)
        self.declare_parameter('right_overhang', 0.0)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        map_path = self.get_parameter('map_path').value
        self.trace_back_step = self.get_parameter('trace_back_step').value
        self.look_ahead_steps = self.get_parameter('look_ahead_steps').value
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.prediction_dt = self.get_parameter('prediction_dt').value
        local_map_range = self.get_parameter('local_map_range').value
        device = self.get_parameter('device').value

        self.vehicle_params = {}
        self.vehicle_params["wheel_base"] = self.get_parameter("wheel_base").value
        self.vehicle_params["max_steer_angle"] = self.get_parameter("max_steer_angle").value
        self.vehicle_params["front_overhang"] = self.get_parameter("front_overhang").value
        self.vehicle_params["rear_overhang"] = self.get_parameter("rear_overhang").value
        self.vehicle_params["left_overhang"] = self.get_parameter("left_overhang").value
        self.vehicle_params["right_overhang"] = self.get_parameter("right_overhang").value

        self.max_objects = 80
        self.max_map_element_num = 80
        self.max_route_element_num = 10
        self.lane_point_number=20
        
        # Check if required parameters are provided
        if not model_path or not map_path:
            self.get_logger().error('model_path and map_path parameters are required')
            return
        
        # Initialize model
        self.device = torch.device(device)
        self.model = self.load_model(model_path)
        self.model.eval()

        self.current_objects = {"objects": []}
        self.object_received = False
        
        # Initialize map manager
        self.map_manager = MapManager(map_path, local_map_range)
        
        # Initialize object tracker
        self.object_tracker = BaseTracker(
            trace_back_step=self.trace_back_step,
        )
        
        # Initialize buffers for ego history
        self.ego_history = deque(maxlen=self.trace_back_step)
        self.current_step = 0
        self.has_route = False
        self.last_inference_time = 0.0
        self.inference_interval = 0.1  # seconds
        self.traffic_light_data = {}
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        route_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Create subscribers
        self.objects_sub = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self.objects_callback,
            sensor_qos
        )
        
        self.ego_pose_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.ego_pose_callback,
            sensor_qos
        )
        
        self.route_sub = self.create_subscription(
            LaneletRoute,
            '/planning/mission_planning/route',
            self.route_callback,
            route_qos
        )

        self.traffic_light_sub = self.create_subscription(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/traffic_signals",
            self.traffic_light_callback,
            sensor_qos
        )
        
        # Create publishers
        self.path_pub = self.create_publisher(
            Path,
            # "/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path",
            '/planning/naive_tf/predicted_path',
            10
        )

        self.trajectory_pub = self.create_publisher(
            Trajectory,
            # '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory',
            '/planning/naive_tf/predicted_trajectory',
            10
        )
        
        self.viz_pub = self.create_publisher(
            MarkerArray,
            '/planning/naive_tf/visualization',
            10
        )
        
        self.get_logger().info('NaiveTF inference node initialized')
        
    def load_model(self, model_path):
        """Load the trained naiveTF model"""
        self.get_logger().info(f'Loading model from {model_path}')
        model = PlanningModel().to(self.device)
        
        checkpoint = torch.load(model_path, map_location=self.device)
        if 'model_state_dict' in checkpoint:
            model.load_state_dict(checkpoint['model_state_dict'])
            self.get_logger().info(f"Loaded model from epoch {checkpoint.get('epoch', 'unknown')}")
        else:
            model.load_state_dict(checkpoint)
            self.get_logger().info("Loaded model weights")
        
        model.eval()
        return model
    
    def traffic_light_callback(self, msg):
        data = dict()
        for traffic_light_group in msg.traffic_light_groups:
            group_id = traffic_light_group.traffic_light_group_id
            elements = traffic_light_group.elements
            data[group_id] = []
            for i, element in enumerate(elements):
                data[group_id].append({
                    "color": element.color,
                    "shape": element.shape,
                    "status": element.status,
                    "confidence": element.confidence
                })
        self.traffic_light_data = data
    
    def route_callback(self, msg):
        """Process route message"""
        self.get_logger().info('Received route')
        self.map_manager.set_global_path(msg)
        self.has_route = True
    
    def objects_callback(self, msg):
        """Process tracked objects message"""
        if len(self.ego_history) == 0:
            return

        current_objects = [CurrentFrameObject(obj) for obj in msg.objects]

        register_objects = []
        for obj in current_objects:
            ego_transforms = list(self.object_tracker.ego_transforms.queue)
            ego_xyz = (ego_transforms[-1] @ np.array([0, 0, 0, 1]))[:3]
            distance_ego_object = np.linalg.norm(ego_xyz - obj.transform[:3, 3])
            if distance_ego_object > 80:
                continue
            register_objects.append({
                "id": obj.uuid,
                "type": obj.object_type,
                "transform": obj.transform.tolist(),
                "velocity": obj.velocity.tolist(),
                "footprint": obj.footprint.tolist(),
                "global_footprint": obj.global_footprint.tolist()
            })
    
        self.current_objects = {"objects": register_objects}
        self.object_received = True

    
    def ego_pose_callback(self, msg):
        """Process ego vehicle pose message"""
        self.object_tracker.step_ego_pose(msg)
        self.ego_history.append(msg)
        
        # Only run inference if we have enough history and a route
        if (len(self.ego_history) >= self.trace_back_step) and self.has_route and self.object_received:
            # Limit inference rate
            current_time = time.time()
            if current_time - self.last_inference_time >= self.inference_interval:
                self.last_inference_time = current_time
                self.run_inference(msg)
    
    def run_inference(self, current_odom):
        """Run model inference and publish results"""
        # Prepare data for the model
        data = self.prepare_model_input(current_odom)
        if data is None:
            return
        
        # Run inference
        with torch.no_grad():
            start_time = time.time()
            predictions = self.model(data)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            inference_time = (time.time() - start_time) * 1000  # ms
        
        self.get_logger().debug(f'Inference time: {inference_time:.2f} ms')
        
        # Process and publish predictions
        self.process_and_publish_predictions(predictions, current_odom)
        
    
    def prepare_model_input(self, current_odom):
        """Prepare input data for the model"""
            # Get current pose in map frame
        pose = current_odom.pose.pose
        
        # Get map data
        map_data = self.map_manager.fetch_local_map(pose)
        
        # Get object tracker data (similar to activate_key_frame in BaseStepEngine)
        tracker_outputs = self.object_tracker.step_key_frame(self.current_step - self.look_ahead_steps)
        
        # Combine data
        data_dict = {
            "objects": self.current_objects["objects"],
            "history_trajectories": tracker_outputs["ego_history"],
            "history_trajectories_transform_list": [
                tracker_outputs["ego_history"]["transforms"][i].tolist()
                for i in range(len(tracker_outputs["ego_history"]["transforms"]))
            ],
            "history_trajectories_speed_list": [
                tracker_outputs["ego_history"]["velocities"][i][0]
                for i in range(len(tracker_outputs["ego_history"]["velocities"]))
            ]
        }
        
        # Add map data
        data_dict.update(map_data)
        
        # Process data for model input (similar to CacheDataset.__getitem__)
        model_input = self.process_data_for_model(data_dict)
        
        return model_input
        
    
    def process_data_for_model(self, frame):
        """Process raw data into model input format (similar to CacheDataset.__getitem__)"""
        data = dict()
        data["vehicle_parameters"] = torch.tensor(
            [
                self.vehicle_params["wheel_base"], self.vehicle_params['max_steer_angle'],
                self.vehicle_params['front_overhang'], self.vehicle_params['rear_overhang'],
                self.vehicle_params['left_overhang'], self.vehicle_params['right_overhang']
            ], dtype=torch.float32
        ) # [6]
        data["objects_types"] = torch.zeros([self.max_objects], dtype=torch.long)
        data["objects_mask"] = torch.zeros([self.max_objects], dtype=torch.bool)
        data["objects_transform"] = torch.zeros(
            [self.max_objects, 4, 4], dtype=torch.float32
        )
        data["objects_velocity"] = torch.zeros(
            [self.max_objects, 3], dtype=torch.float32
        )
        data["objects_footprint"] = torch.zeros(
            [self.max_objects, 4, 3], dtype=torch.float32
        )  # [x, y, z]

        actual_objects_num = len(frame["objects"])
        if actual_objects_num > self.max_objects:
            frame["objects"] = frame["objects"][: self.max_objects]
            actual_objects_num = self.max_objects

        if actual_objects_num > 0:
            data["objects_types"][:actual_objects_num] = torch.tensor(
                [object["type"] for object in frame["objects"]], dtype=torch.long
            )
            data["objects_mask"][:actual_objects_num] = torch.tensor(
                [True] * actual_objects_num, dtype=torch.bool
            )
            data["objects_transform"][:actual_objects_num] = torch.tensor(
                [object["transform"] for object in frame["objects"]],
                dtype=torch.float32,
            )
            data["objects_velocity"][:actual_objects_num] = torch.tensor(
                [object["velocity"] for object in frame["objects"]], dtype=torch.float32
            )

            # Process footprints
            # If the footprint is not a 4-point bbox, fit a rotated bbox to it
            processed_footprints = []
            for obj in frame["objects"][:actual_objects_num]:
                footprint = obj["global_footprint"]  # Shape could be (N, 2)
                if len(footprint) != 4:  # If not already a 4-point bbox
                    bbox = fit_rotated_bbox(np.array(footprint))
                    processed_footprints.append(bbox)
                else:
                    processed_footprints.append(
                        torch.tensor(footprint, dtype=torch.float32)
                    )

            data["objects_footprint"][:actual_objects_num] = torch.stack(
                processed_footprints
            )

        data["history_trajectories_transform"] = torch.tensor(
            frame["history_trajectories_transform_list"], dtype=torch.float32
        )  # [N_h, 4, 4]
        data["history_trajectories_speed"] = torch.tensor(
            frame["history_trajectories_speed_list"], dtype=torch.float32
        )  # [N_h, 3]

        nearby_road_ids = [
            lanelet_idx
            for lanelet_idx in frame["nearby_lanelets_ids"]
            if int(lanelet_idx) in self.map_manager.resampled_lanelets
        ]
        is_road_in_route = [
            bool(lanelet_idx in frame["routes"]) for lanelet_idx in nearby_road_ids
        ]
        if len(nearby_road_ids) > self.max_map_element_num:
            nearby_road_ids = nearby_road_ids[: self.max_map_element_num]
            is_road_in_route = is_road_in_route[: self.max_map_element_num]

        nearby_lanelets = [
            self.map_manager.map_object.laneletLayer.get(lanelet_idx)
            for lanelet_idx in nearby_road_ids
        ]

        data["boundary_left_boundaries"] = torch.zeros(
            [self.max_map_element_num, self.lane_point_number, 3], dtype=torch.float32
        )
        data["boundary_right_boundaries"] = torch.zeros(
            [self.max_map_element_num, self.lane_point_number, 3], dtype=torch.float32
        )
        data["boundary_in_route"] = torch.zeros(
            [self.max_map_element_num], dtype=torch.bool
        )
        data["boundary_mask"] = torch.zeros(
            [self.max_map_element_num, self.lane_point_number], dtype=torch.bool
        )
        data["lanelet_subtypes"] = torch.zeros(
            [self.max_map_element_num], dtype=torch.long
        )
        data["lanelet_locations"] = torch.zeros(
            [self.max_map_element_num], dtype=torch.long
        )
        data["lanelet_turn_directions"] = torch.zeros(
            [self.max_map_element_num], dtype=torch.long
        )
        data["lanelet_speed_limit"] = torch.zeros(
            [self.max_map_element_num], dtype=torch.float32
        )
        data["lanelet_traffic_lights"] = torch.zeros(
            [self.max_map_element_num], dtype=torch.long
        )

        data["boundary_mask"][0 : len(nearby_road_ids)] = True

        data["boundary_in_route"][0 : len(nearby_road_ids)] = torch.tensor(
            is_road_in_route, dtype=torch.bool
        )
        data["lanelet_subtypes"][0 : len(nearby_road_ids)] = torch.tensor(
            [
                LANELET_TYPE_MAPPING[attribute_or(lanelet, "subtype", "road")]
                for lanelet in nearby_lanelets
            ],
            dtype=torch.long,
        )
        data["lanelet_locations"][0 : len(nearby_road_ids)] = torch.tensor(
            [
                LANELET_LOCATION_MAPPING[attribute_or(lanelet, "location", "urban")]
                for lanelet in nearby_lanelets
            ],
            dtype=torch.long,
        )
        data["lanelet_turn_directions"][0 : len(nearby_road_ids)] = torch.tensor(
            [
                LANELET_TURN_DIRECTION_MAPPING[
                    attribute_or(lanelet, "turn_direction", "straight")
                ]
                for lanelet in nearby_lanelets
            ],
            dtype=torch.long,
        )
        data["lanelet_speed_limit"][0 : len(nearby_road_ids)] = torch.tensor(
            [
                float(attribute_or(lanelet, "speed_limit", "50"))
                / 100.0  # normalized to [0, 1]
                for lanelet in nearby_lanelets
            ],
            dtype=torch.float32,
        )
        data["lanelet_traffic_lights"][0 : len(nearby_road_ids)] = torch.tensor(
            [
                get_current_traffic_light(
                    lanelet, self.traffic_light_data
                )
                for lanelet in nearby_lanelets
            ],
            dtype=torch.long,
        )
        data["boundary_left_boundaries"][0 : len(nearby_road_ids)] = torch.tensor(
            [
                [
                    (p.x, p.y, p.z)
                    for p in self.map_manager.resampled_lanelets[int(lanelet_idx)]["left_bound"]
                ]
                for lanelet_idx in nearby_road_ids
            ]
        )
        data["boundary_right_boundaries"][0 : len(nearby_road_ids)] = torch.tensor(
            [
                [
                    (p.x, p.y, p.z)
                    for p in self.map_manager.resampled_lanelets[int(lanelet_idx)]["right_bound"]
                ]
                for lanelet_idx in nearby_road_ids
            ]
        )

        ego_transform = data["history_trajectories_transform"][-1]  # [4, 4]
        ## normalize and transform the ego_trajectory, velocity, and neighbouring
        ego_transform = torch.inverse(ego_transform)
        ## ego speed is originally not in global frame, so we don't need to transform it
        data["history_trajectories_transform"] = torch.matmul(
            ego_transform,
            data["history_trajectories_transform"],
        )  # [N_h, 4, 4]
        data["objects_transform"] = torch.matmul(
            ego_transform, data["objects_transform"]
        )  # [N_o, 4, 4]

        # Transform velocities - only need rotation part of transformation
        rotation_matrix = ego_transform[:3, :3]  # [3, 3]
        data["objects_velocity"] = torch.matmul(
            rotation_matrix, data["objects_velocity"][..., None]
        )[
            ..., 0
        ]  # [N_o, 3]

        # Transform footprints from global to ego frame
        # First convert 2D points to homogeneous coordinates
        footprints_homogeneous = torch.ones(
            data["objects_footprint"].shape[0],
            data["objects_footprint"].shape[1],
            4,
            dtype=torch.float32,
        )  # [N_o, 4, 4]
        footprints_homogeneous[..., :3] = data[
            "objects_footprint"
        ]  # Set x,y coordinates
        footprints_homogeneous = footprints_homogeneous.transpose(-2, -1)  # Trans
        footprints_homogeneous = torch.matmul(
            ego_transform, footprints_homogeneous
        )  # [N_o, 4, 4]
        footprints_homogeneous = footprints_homogeneous.transpose(-2, -1)  # Trans
        data["objects_footprint"] = footprints_homogeneous[..., :2]  # [N_o, 4, 2]
        # print(data["objects_footprint"][data["objects_mask"]], ego_transform)

        # Transform map elements
        # Transform left boundaries
        left_boundaries_homogeneous = torch.ones(
            data["boundary_left_boundaries"].shape[0],
            data["boundary_left_boundaries"].shape[1],
            4,
            dtype=torch.float32,
        )
        left_boundaries_homogeneous[..., :3] = data["boundary_left_boundaries"]
        left_boundaries_homogeneous = left_boundaries_homogeneous.transpose(-2, -1)
        left_boundaries_homogeneous = torch.matmul(
            ego_transform, left_boundaries_homogeneous
        )
        data["boundary_left_boundaries"] = left_boundaries_homogeneous[
            :, :2, :
        ].transpose(-2, -1)[..., :2] # the output should still be in 2d

        # Transform right boundaries
        right_boundaries_homogeneous = torch.ones(
            data["boundary_right_boundaries"].shape[0],
            data["boundary_right_boundaries"].shape[1],
            4,
            dtype=torch.float32,
        )
        right_boundaries_homogeneous[..., :3] = data["boundary_right_boundaries"]
        right_boundaries_homogeneous = right_boundaries_homogeneous.transpose(-2, -1)
        right_boundaries_homogeneous = torch.matmul(
            ego_transform, right_boundaries_homogeneous
        )
        data["boundary_right_boundaries"] = right_boundaries_homogeneous[
            :, :2, :
        ].transpose(-2, -1)[..., :2] # the output should still be in 2d

        data = {k: v.unsqueeze(0).contiguous().to(self.device) if isinstance(v, torch.Tensor) else v 
               for k, v in data.items()}
        data["original_ego_transform"] = ego_transform
        return data  # Return data instead of frame
    
    def get_object_type_id(self, object_type):
        """Convert object type string to ID"""
        type_mapping = {
            "UNKNOWN": 0,
            "CAR": 1,
            "TRUCK": 2,
            "BUS": 3,
            "TRAILER": 4,
            "MOTORCYCLE": 5,
            "BICYCLE": 6,
            "PEDESTRIAN": 7
        }
        return type_mapping.get(object_type, 0)
    
    def process_and_publish_predictions(self, predictions, current_odom):
        """Process model predictions and publish them"""
        # Get best trajectory based on highest probability
        probabilities = predictions['probability'][0]
        best_mode = probabilities.argmax().item()
        best_trajectory = predictions['trajectory'][0, best_mode].cpu().numpy()  # [steps, 4]
        
        # Get original ego transform to convert back to global frame
        ego_transform = torch.tensor(self.object_tracker.ego_transforms.queue[-1])
        
        # Create path message
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = current_odom.header.stamp

        # Create trajectory message
        trajectory_msg = Trajectory()
        trajectory_msg.header.frame_id = "map"
        trajectory_msg.header.stamp = current_odom.header.stamp
        
        # Get current ego pose and velocity
        current_pose = current_odom.pose.pose
        current_twist = current_odom.twist.twist
        
        # Convert trajectory points to global frame and add to path and trajectory
        global_positions = []
        global_orientations = []
        
        for i, point in enumerate(best_trajectory):
            # Extract position (x, y) and direction (dx, dy)
            ## Notice dx, dy is not usable because they are not trained
            x, y = point[0], point[1]
            if i == 0:
                dx = 1
                dy = 0
                # so that local orientation is unity rotation
            else:
                dx = point[0] - best_trajectory[i-i][0]
                dy = point[1] - best_trajectory[i-1][1]
            
            # Create homogeneous point
            local_point = np.array([x, y, 0, 1])
            
            # Transform to global frame
            global_point = ego_transform @ local_point
            
            # Calculate orientation from direction vector
            yaw = np.arctan2(dy, dx)
            
            # Transform yaw to global frame (add ego orientation)
            from scipy.spatial.transform import Rotation
            q = [
                current_odom.pose.pose.orientation.x,
                current_odom.pose.pose.orientation.y,
                current_odom.pose.pose.orientation.z,
                current_odom.pose.pose.orientation.w
            ]
            r = Rotation.from_quat(q)
            # global_yaw = yaw + r.as_euler('xyz')[2]
            r_local_yaw = Rotation.from_euler('z', yaw)
            
            # Convert yaw to quaternion
            r_yaw = r * r_local_yaw
            #q_yaw = Rotation.from_euler('z', global_yaw).as_quat()
            q_yaw = r_yaw.as_quat()
            
            # Store global positions and orientations for later calculations
            global_positions.append([global_point[0].item(), global_point[1].item(), global_point[2].item()])
            global_orientations.append(q_yaw)
            
            # Create path point (for backward compatibility)
            path_point = PathPoint()
            path_point.pose.position.x = global_point[0].item()
            path_point.pose.position.y = global_point[1].item()
            path_point.pose.position.z = global_point[2].item()
            path_point.pose.orientation.x = q_yaw[0]
            path_point.pose.orientation.y = q_yaw[1]
            path_point.pose.orientation.z = q_yaw[2]
            path_point.pose.orientation.w = q_yaw[3]
            
            # Add to path
            path_msg.points.append(path_point)
        
        # Calculate velocities and accelerations for trajectory points
        for i in range(len(best_trajectory)):
            trajectory_point = TrajectoryPoint()
            
            # Set pose
            trajectory_point.pose.position.x = global_positions[i][0]
            trajectory_point.pose.position.y = global_positions[i][1]
            trajectory_point.pose.position.z = global_positions[i][2]
            trajectory_point.pose.orientation.x = global_orientations[i][0]
            trajectory_point.pose.orientation.y = global_orientations[i][1]
            trajectory_point.pose.orientation.z = global_orientations[i][2]
            trajectory_point.pose.orientation.w = global_orientations[i][3]
            
            # Set time from start
            trajectory_point.time_from_start.sec = int(i * self.prediction_dt)
            trajectory_point.time_from_start.nanosec = int((i * self.prediction_dt % 1) * 1e9)
            
            # Calculate velocities and accelerations
            if i == 0:
                # For the first point, use current vehicle velocity
                trajectory_point.longitudinal_velocity_mps = current_twist.linear.x
                trajectory_point.lateral_velocity_mps = current_twist.linear.y
                trajectory_point.acceleration_mps2 = 0.0  # We don't have acceleration data
            else:
                # Calculate velocity from position difference
                dt = self.prediction_dt
                dx = global_positions[i][0] - global_positions[i-1][0]
                dy = global_positions[i][1] - global_positions[i-1][1]
                
                # Get orientation at previous point to project velocity
                prev_quat = global_orientations[i-1]
                r_prev = Rotation.from_quat(prev_quat)
                heading_prev = r_prev.as_euler('xyz')[2]
                
                # Project velocity into longitudinal and lateral components
                speed = np.sqrt(dx*dx + dy*dy) / dt
                # direction = np.arctan2(dy, dx)
                
                # Longitudinal velocity is along the heading direction
                trajectory_point.longitudinal_velocity_mps = speed
                trajectory_point.lateral_velocity_mps = 0.0
                
                # Calculate acceleration if we have at least 2 previous points
                if i >= 2:
                    prev_speed = np.sqrt(
                        (global_positions[i-1][0] - global_positions[i-2][0])**2 + 
                        (global_positions[i-1][1] - global_positions[i-2][1])**2
                    ) / dt
                    
                    # Acceleration is change in speed over time
                    trajectory_point.acceleration_mps2 = (speed - prev_speed) / dt
                else:
                    trajectory_point.acceleration_mps2 = 0.0
            
            # Calculate heading rate (yaw rate)
            if i > 0:
                # Get current and previous orientation
                curr_quat = global_orientations[i]
                prev_quat = global_orientations[i-1]
                
                r_curr = Rotation.from_quat(curr_quat)
                r_prev = Rotation.from_quat(prev_quat)
                
                # Extract yaw (heading) from quaternions
                heading_curr = r_curr.as_euler('xyz')[2]
                heading_prev = r_prev.as_euler('xyz')[2]
                
                # Calculate heading rate (handle wrap-around)
                heading_diff = heading_curr - heading_prev
                if heading_diff > np.pi:
                    heading_diff -= 2 * np.pi
                elif heading_diff < -np.pi:
                    heading_diff += 2 * np.pi
                
                trajectory_point.heading_rate_rps = heading_diff / self.prediction_dt
            else:
                trajectory_point.heading_rate_rps = current_twist.angular.z
            
            # We don't have wheel angle data from the prediction, so set to 0
            trajectory_point.front_wheel_angle_rad = 0.0
            trajectory_point.rear_wheel_angle_rad = 0.0
            
            # Add to trajectory
            trajectory_msg.points.append(trajectory_point)
        
        # Publish path (for backward compatibility)
        self.path_pub.publish(path_msg)
        
        # Publish trajectory
        self.trajectory_pub.publish(trajectory_msg)
        
        # Create and publish visualization markers
        self.publish_visualization_markers(predictions, ego_transform, current_odom.header.stamp)
    
    def publish_visualization_markers(self, predictions, ego_transform, stamp):
        """Publish visualization markers for all predicted trajectories"""
        marker_array = MarkerArray()
        
        # Get all trajectories and probabilities
        trajectories = predictions['trajectory'][0].cpu().numpy()  # [num_modes, steps, 4]
        probabilities = predictions['probability'][0].cpu().numpy()  # [num_modes]
        
        # Create markers for each trajectory mode
        for mode_idx, (trajectory, probability) in enumerate(zip(trajectories, probabilities)):
            # Create line strip marker for trajectory
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "predicted_trajectories"
            marker.id = mode_idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Set line properties based on probability
            marker.scale.x = 0.1 if mode_idx == probabilities.argmax() else 0.05
            
            # Color based on probability (best mode is green, others are blue with alpha based on probability)
            color = ColorRGBA()
            if mode_idx == probabilities.argmax():
                color.r = 0.0
                color.g = 1.0
                color.b = 0.0
                color.a = 1.0
            else:
                color.r = 0.0
                color.g = 0.0
                color.b = 1.0
                color.a = max(0.2, probability / probabilities.max().item())
            
            marker.color = color
            
            # Set lifetime
            marker.lifetime = Duration(sec=1, nanosec=0)
            
            # Add points to line strip
            for point in trajectory:
                # Create homogeneous point in local frame
                local_point = np.array([point[0], point[1], 0, 1])
                
                # Transform to global frame
                global_point = ego_transform.numpy() @ local_point
                
                # Add to marker
                p = PoseStamped()
                p.pose.position.x = global_point[0]
                p.pose.position.y = global_point[1]
                p.pose.position.z = global_point[2]
                marker.points.append(p.pose.position)
            
            marker_array.markers.append(marker)
            
            # Add text marker with probability
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = stamp
            text_marker.ns = "trajectory_probabilities"
            text_marker.id = mode_idx
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position text at the end of trajectory
            if len(trajectory) > 0:
                end_point = trajectory[-1]
                local_point = np.array([end_point[0], end_point[1], 0, 1])
                global_point = ego_transform.numpy() @ local_point
                
                text_marker.pose.position.x = global_point[0]
                text_marker.pose.position.y = global_point[1]
                text_marker.pose.position.z = global_point[2] + 1.0  # Slightly above trajectory
            
            text_marker.text = f"Mode {mode_idx}: {probability:.2f}"
            text_marker.scale.z = 0.5  # Text size
            text_marker.color = color
            text_marker.lifetime = Duration(sec=1, nanosec=0)
            
            marker_array.markers.append(text_marker)
        
        # Publish markers
        self.viz_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = NaiveTFInferenceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()