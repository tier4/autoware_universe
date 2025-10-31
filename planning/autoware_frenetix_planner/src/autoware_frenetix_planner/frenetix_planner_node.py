
import sys
import lanelet2  # noqa: F401 # isort: skip
from autoware_lanelet2_extension_python.utility.utilities import fromBinMsg
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import Path, LaneletRoute
from autoware_planning_msgs.msg import Trajectory
from autoware_perception_msgs.msg import PredictedObjects
from autoware_control_msgs.msg import Control
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped, Pose, Point, Quaternion
from autoware_planning_msgs.msg import TrajectoryPoint
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import rclpy.qos
from autoware_frenetix_planner.frenetix_motion_planner import FrenetixMotionPlanner
from autoware_frenetix_planner.config import FrenetixPlannerParams, CostWeightsParams
from tf_transformations import quaternion_from_euler
from builtin_interfaces.msg import Duration

# Main planner node class
class FrenetixPlanner(Node):

    def __init__(self):
        super().__init__("frenetix_planner")
        self.get_logger().info("Node initializing...")

        # Declare and get parameters from the YAML file.
        # The node will fail to start if the YAML is missing or incomplete.
        self.params = self._declare_and_load_params()

        # Initialize subscribers and publishers
        self._init_subscriber()
        self._init_publisher()
        self.get_logger().info("Subscribers and publishers initialized.")

        # Instantiate the planner with the loaded parameters
        self.planner = FrenetixMotionPlanner(logger=self.get_logger(), params=self.params)
        self.get_logger().info("FrenetixMotionPlanner initialized.")
        
        # Set up a callback for runtime parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.car_altitude = 0.0
        self.objects = None
        self.route_processed = False
    
    @property
    def route(self):
        return self.planner.route

    @property
    def lanelet_map(self):
        return self.planner.lanelet_map
    
    @property
    def logger(self):
        return self.get_logger()

    def _declare_and_load_params(self) -> FrenetixPlannerParams:
        """
        Declares all required ROS 2 parameters with their explicit types and directly 
        constructs the typed parameter dataclass object.
        """

        self.get_logger().info("Declaring parameters and building param object...")

        # Helper function to declare a parameter with its type and get its value
        def declare_and_get(name, param_type):
            return self.declare_parameter(name, param_type).value

        # Build the nested CostWeightsParams object first
        cost_weights_params = CostWeightsParams(
            acceleration=declare_and_get("cost_weights.acceleration", Parameter.Type.DOUBLE),
            jerk=declare_and_get("cost_weights.jerk", Parameter.Type.DOUBLE),
            lateral_jerk=declare_and_get("cost_weights.lateral_jerk", Parameter.Type.DOUBLE),
            longitudinal_jerk=declare_and_get("cost_weights.longitudinal_jerk", Parameter.Type.DOUBLE),
            orientation_offset=declare_and_get("cost_weights.orientation_offset", Parameter.Type.DOUBLE),
            lane_center_offset=declare_and_get("cost_weights.lane_center_offset", Parameter.Type.DOUBLE),
            distance_to_reference_path=declare_and_get("cost_weights.distance_to_reference_path", Parameter.Type.DOUBLE),
            prediction=declare_and_get("cost_weights.prediction", Parameter.Type.DOUBLE),
            distance_to_obstacles=declare_and_get("cost_weights.distance_to_obstacles", Parameter.Type.DOUBLE),
            velocity_offset=declare_and_get("cost_weights.velocity_offset", Parameter.Type.DOUBLE),
            positive_velocity_offset=declare_and_get("cost_weights.positive_velocity_offset", Parameter.Type.DOUBLE),
            negative_velocity_offset=declare_and_get("cost_weights.negative_velocity_offset", Parameter.Type.DOUBLE),
            cartesian_lateral_acceleration=declare_and_get("cost_weights.cartesian_lateral_acceleration", Parameter.Type.DOUBLE)
        )

        # Now build the main Parameters object
        params = FrenetixPlannerParams(
            # Double (float) parameters
            planning_horizon=declare_and_get("planning_horizon", Parameter.Type.DOUBLE),
            sampling_dt=declare_and_get("sampling_dt", Parameter.Type.DOUBLE),
            t_min=declare_and_get("t_min", Parameter.Type.DOUBLE),
            d_min=declare_and_get("d_min", Parameter.Type.DOUBLE),
            d_max=declare_and_get("d_max", Parameter.Type.DOUBLE),
            velocity_limit=declare_and_get("velocity_limit", Parameter.Type.DOUBLE),
            desired_velocity=declare_and_get("desired_velocity", Parameter.Type.DOUBLE),
            delta_max=declare_and_get("delta_max", Parameter.Type.DOUBLE),
            wheelbase=declare_and_get("wheelbase", Parameter.Type.DOUBLE),
            v_switch=declare_and_get("v_switch", Parameter.Type.DOUBLE),
            a_max=declare_and_get("a_max", Parameter.Type.DOUBLE),
            v_delta_max=declare_and_get("v_delta_max", Parameter.Type.DOUBLE),
            length=declare_and_get("length", Parameter.Type.DOUBLE),
            width=declare_and_get("width", Parameter.Type.DOUBLE),
            wb_rear_axle=declare_and_get("wb_rear_axle", Parameter.Type.DOUBLE),
            v_max=declare_and_get("v_max", Parameter.Type.DOUBLE),
            
            # Integer parameters
            sampling_min=declare_and_get("sampling_min", Parameter.Type.INTEGER),
            sampling_max=declare_and_get("sampling_max", Parameter.Type.INTEGER),
            
            # The nested parameter object
            cost_weights=cost_weights_params
        )

        return params
    
    def parameters_callback(self, params) -> SetParametersResult:
        """
        This callback is triggered when a parameter is changed at runtime.
        It dynamically updates the fields in the existing params object and
        passes the updated parameters to the core planner.
        """
        self.get_logger().info("Parameter update request received...")

        # Iterate through all changed parameters
        for param in params:
            keys = param.name.split('.')
            
            # Check if it is a nested parameter (e.g., 'cost_weights.prediction')
            if len(keys) == 2:
                parent_key, child_key = keys
                # Get the nested param object (e.g., self.params.cost_weights)
                nested_param_object = getattr(self.params, parent_key)
                # Set the attribute on the nested object
                setattr(nested_param_object, child_key, param.value)
            else:
                # It's a top-level parameter, set it directly on the main params object
                setattr(self.params, param.name, param.value)

        # Pass the entire, newly modified params object to the planner's update method
        self.planner.update_params(self.params)
        self.get_logger().info("Planner parameters updated successfully.")

        # Signal that the parameter changes were successfully applied
        return SetParametersResult(successful=True)

    # Callback for reference path messages
    def reference_path_callback(self, reference_trajectory_msg):
        self.get_logger().debug(f"Received reference path message with {len(reference_trajectory_msg.points)} points.")
        
        # convert points to numpy array [[x, y], ...]
        reference_path = np.array([[p.pose.position.x, p.pose.position.y] for p in reference_trajectory_msg.points])

        # set/update reference path in planner
        if self.planner.set_reference_path(reference_path):
          self.get_logger().info("Reference path set/updated.")

    # Callback for objects messages
    def objects_callback(self, objects_msg):
        self.objects = objects_msg

    # Callback for synchronized odometry and acceleration messages
    def kinematic_state_callback(self, odometry_msg, accel_msg, control_msg=None):
        
        # Extract position and orientation from odometry
        position = odometry_msg.pose.pose.position
        orientation = odometry_msg.pose.pose.orientation
        self.car_altitude = odometry_msg.pose.pose.position.z

        # Extract velocity from odometry
        velocity = odometry_msg.twist.twist.linear.x
        
        # Extract acceleration from accel_msg
        acceleration = accel_msg.accel.accel.linear.x

        # Extract control command (steering angle) if available
        steering_angle = control_msg.lateral.steering_tire_angle if control_msg else 0.0

        # Set/update ego state in planner
        if self.planner.set_cartesian_state(position, orientation, 
                                            velocity=velocity, 
                                            acceleration=acceleration, 
                                            steering_angle=steering_angle,
                                            timestamp=odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nanosec * 1e-9):
            self.get_logger().debug("Synchronized kinematic state set/updated.")

    # Callback for vector map messages
    def vector_map_callback(self, map_msg: LaneletMapBin):
        self.get_logger().info(f"Received vector map message: {getattr(map_msg, 'name_map', None)}")
        # Conversion from binary map message to lanelet map would happen here
        lanelet_map = fromBinMsg(map_msg)
        self.planner.set_lanelet_map(lanelet_map)
        self.get_logger().info("Lanelet map converted.")

        if self.route and not self.route_processed:
            self.planner.process_route()
            self.route_processed = True

    # Route callback - retrieves route information (lanelet IDs) from mission planner
    def route_callback(self, route_msg):
        """
        Called when a new route is received.
        Only processes the route if the lanelet IDs have changed.
        """
        
        self.logger.debug("New route callback received.")

        # Extract the list of IDs from the segments
        try:
            new_route = [seg.preferred_primitive.id for seg in route_msg.segments]
        except AttributeError:
            self.logger.error("Error parsing route message. 'segments' or 'preferred_primitive' not found.")
            return
        
        if not new_route:
            self.logger.warn("Received route, but it contains no lanelet segments.")
            return
        
        # new route is different from current route
        if not self.route or self.route != new_route:
            self.logger.info(f"New route received! {len(new_route)} lanelets.")
            self.logger.info(f"Route lanelet IDs: {new_route}")

            # Update route in planner
            self.planner.set_route(new_route)

            # process route if map is ready
            if self.lanelet_map:
                self.planner.process_route()
                self.route_processed = True

        # IDs are identical, no action needed
        else:
            self.logger.debug("Route received, but IDs are identical. No action required.")

    # Timer callback to publish trajectory
    def timer_callback_publish_trajectory(self):
        
        # Convert the latest objects to Frenetix format and set in planner
        if self.planner.set_objects(self.objects):
            self.get_logger().debug("Objects set/updated.")
        
        # Generate trajectory using the planner
        frenetix_trajectory = self.planner.cyclic_plan()
        trajectory_msg = self.convert_frenetix_to_autoware_trajectory(frenetix_trajectory, car_altitude=self.car_altitude)
        if trajectory_msg:
            self.trajectory_pub.publish(trajectory_msg)

    # Set up all topic subscribers
    def _init_subscriber(self):
        
        # reference path
        self.reference_trajectory_sub_ = self.create_subscription(
            Path, "/input/reference_path", self.reference_path_callback, 1
        )

        # predicted objects
        self.predicted_objects_sub_ = self.create_subscription(
            PredictedObjects, "/input/objects", self.objects_callback, 1
        )

        route_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # route subscriber
        self.route_sub_ = self.create_subscription(
            LaneletRoute, "/input/route", self.route_callback, route_qos
        )

        # Use message_filters for synchronized kinematic state
        self.odom_sub = Subscriber(self, Odometry, "/input/odometry")
        self.accel_sub = Subscriber(self, AccelWithCovarianceStamped, "/input/acceleration")
        # self.control_sub = Subscriber(self, Control, "/input/control_cmd")
        self.ksts = ApproximateTimeSynchronizer([self.odom_sub, self.accel_sub], queue_size=10, slop=0.1, allow_headerless=True)
        self.ksts.registerCallback(self.kinematic_state_callback)

        map_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub_ = self.create_subscription(
            LaneletMapBin, "/input/vector_map", self.vector_map_callback, map_qos
        )
        self.get_logger().info("All subscribers set up.")

    # Set up all topic publishers
    def _init_publisher(self):
        self.trajectory_pub = self.create_publisher(Trajectory, "/output/trajectory", 1)
        self.timer_trajectory_pub = self.create_timer(0.1, self.timer_callback_publish_trajectory)
        self.get_logger().info("Trajectory publisher set up.")

    # convert frenetix trajectory to autoware trajectory message
    def convert_frenetix_to_autoware_trajectory(self, opt_trajectory, car_altitude=0.0):
        """
        Converts a Frenetix optimal trajectory to an Autoware Trajectory message.
        """
        if opt_trajectory is None:
            self.get_logger().debug("No optimal trajectory to convert.")
            return None

        trajectory_x = opt_trajectory.cartesian.x
        trajectory_y = opt_trajectory.cartesian.y
        trajectory_theta = opt_trajectory.cartesian.theta
        trajectory_v = opt_trajectory.cartesian.v
        trajectory_a = opt_trajectory.cartesian.a

        num_points = len(trajectory_x)
        trajectory_msg = Trajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = "map"

        trajectory_msg.points = [
            TrajectoryPoint(
                pose=Pose(
                    position=Point(),
                    orientation=Quaternion()
                ),
                longitudinal_velocity_mps=0.0,
                lateral_velocity_mps=0.0,
                acceleration_mps2=0.0,
                heading_rate_rps=0.0,
                front_wheel_angle_rad=0.0,
                rear_wheel_angle_rad=0.0,
                time_from_start=Duration(sec=0, nanosec=0)
            ) for _ in range(num_points)
        ]

        for i in range(num_points):
          quat = quaternion_from_euler(0, 0, trajectory_theta[i])
          traj_point = trajectory_msg.points[i]
          traj_point.pose.position.x = trajectory_x[i]
          traj_point.pose.position.y = trajectory_y[i]
          traj_point.pose.position.z = car_altitude

          traj_point.pose.orientation.x = quat[0]
          traj_point.pose.orientation.y = quat[1]
          traj_point.pose.orientation.z = quat[2]
          traj_point.pose.orientation.w = quat[3]

          traj_point.longitudinal_velocity_mps = trajectory_v[i]
          traj_point.acceleration_mps2 = trajectory_a[i]
          traj_point.lateral_velocity_mps = 0.0
          traj_point.heading_rate_rps = 0.0
          traj_point.front_wheel_angle_rad = 0.0
          traj_point.rear_wheel_angle_rad = 0.0
          traj_point.time_from_start = Duration(sec=0, nanosec=0)

          # Debug message for each trajectory point
          self.get_logger().debug(
          f"Trajectory point {i}: x={traj_point.pose.position.x:.2f}, y={traj_point.pose.position.y:.2f}, "
          f"theta={trajectory_theta[i]:.2f}, v={traj_point.longitudinal_velocity_mps:.2f}, "
          f"a={traj_point.acceleration_mps2:.2f}"
          )

        return trajectory_msg

# Main entry point for the node
def main():
    rclpy.init(args=sys.argv)
    node = FrenetixPlanner()
    rclpy.spin(node)
    # catch errors during initialization, e.g., missing parameters
    rclpy.logging.get_logger("frenetix_planner_main").fatal(f"Node crashed: {e}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
