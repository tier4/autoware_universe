
import sys
import lanelet2  # noqa: F401 # isort: skip
from autoware_lanelet2_extension_python.utility.utilities import fromBinMsg
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import Path, LaneletRoute
from autoware_planning_msgs.msg import Trajectory
from autoware_perception_msgs.msg import PredictedObjects
from autoware_control_msgs.msg import Control
from visualization_msgs.msg import MarkerArray
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
from autoware_frenetix_planner.config import FrenetixPlannerParams, CostWeightsParams, EvasiveParams
from autoware_frenetix_planner.stop_check import compute_min_stop_distance
from tf_transformations import quaternion_from_euler
from builtin_interfaces.msg import Duration
from std_srvs.srv import SetBool

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
        self._init_services()
        self.get_logger().info("Subscribers and publishers initialized.")

        # Instantiate the planner with the loaded parameters
        self.planner = FrenetixMotionPlanner(logger=self.get_logger(), params=self.params)
        self.get_logger().info("FrenetixMotionPlanner initialized.")
        
        # Set up a callback for runtime parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.car_altitude = 0.0
        self.objects = None
        self.route_processed = False

        # Safety filter buffers
        self.latest_autoware_traj = None
        self.stop_wall_xy = None
        self.last_selected_source = None
        # Evasive mode state (latched when True until release condition is met)
        self.evasive_active = False
    
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

        # Build the nested CostWeightsParams object first
        evasive_params = EvasiveParams(
            acc_min=declare_and_get("evasive.acc_min", Parameter.Type.DOUBLE),
            jerk_acc=declare_and_get("evasive.jerk_acc", Parameter.Type.DOUBLE),
            jerk_dec=declare_and_get("evasive.jerk_dec", Parameter.Type.DOUBLE),
            stop_buffer=declare_and_get("evasive.stop_buffer", Parameter.Type.DOUBLE),
            # Trigger conditions
            velocity_threshold=declare_and_get("evasive.velocity_threshold", Parameter.Type.DOUBLE),
            # Release condition thresholds
            release_longitudinal_back=declare_and_get("evasive.release_back", Parameter.Type.DOUBLE),
            release_lateral_center=declare_and_get("evasive.release_lat", Parameter.Type.DOUBLE),
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
            cost_weights=cost_weights_params,

            # The nested evasive parameters
            evasive = evasive_params
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
                if parent_key not in self.params:
                    self.get_logger().error(f"Unknown parameter group: {parent_key}")
                    return SetParametersResult(successful=False)
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
        # when we are in evasive mode, ignore reference path updates
        # if self.evasive_active:
        #     self.get_logger().warn("------- [Evasive] Ignoring reference path update -------")
        #     return
        
        self.get_logger().debug(f"Received reference path message with {len(reference_trajectory_msg.points)} points.")
        
        # convert points to numpy array [[x, y], ...]
        reference_path = np.array([[p.pose.position.x, p.pose.position.y] for p in reference_trajectory_msg.points])

        # set/update reference path in planner
        if self.planner.set_reference_path(reference_path):
          self.get_logger().debug("Reference path set/updated.")

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

    # Autoware trajectory callback
    def autoware_trajectory_callback(self, traj_msg: Trajectory):
        self.latest_autoware_traj = traj_msg

    def stop_wall_callback(self, marker_array_msg: MarkerArray):
        # marker cannot change if evasive mode is active
        if marker_array_msg.markers is not None and len(marker_array_msg.markers) > 0 and not self.evasive_active:
            position = marker_array_msg.markers[0].pose.position

            if position.x == 0.0 and position.y == 0.0:
                self.stop_wall_xy = None
            else:
                self.stop_wall_xy = [position.x, position.y]

    def timer_callback_publish_trajectory(self):
        # Update objects in planner
        if self.planner.set_objects(self.objects):
            self.get_logger().debug("Objects set/updated.")

        # Generate Frenetix trajectory candidate
        frenetix_trajectory = self.planner.cyclic_plan()
        frenetix_msg = self.convert_frenetix_to_autoware_trajectory(
            frenetix_trajectory, car_altitude=self.car_altitude
        )

        # Update evasive mode based on latest Autoware trajectory and state
        if self.latest_autoware_traj:
            self.check_evasive_mode(self.latest_autoware_traj)
        else:
            # If no Autoware trajectory at all, prefer evasive mode
            self.get_logger().warn("No Autoware trajectory available! Waiting for trajectory ...")

        # Select output strictly by mode
        if self.evasive_active:
            selected_msg = frenetix_msg
            src = "frenetix"
        else:
            selected_msg = self.latest_autoware_traj
            src = "autoware"

        # Publish if available
        if selected_msg:
            selected_msg.header.stamp = self.get_clock().now().to_msg()
            if src != self.last_selected_source:
                self.get_logger().info(f"Evasive maneuver filter selected: {src} trajectory")
                self.last_selected_source = src
            self.trajectory_pub.publish(selected_msg)


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

        # subscribe to autoware trajectory for safety filtering
        self.autoware_traj_sub_ = self.create_subscription(
            Trajectory,
            "/input/autoware_trajectory",
            self.autoware_trajectory_callback,
            1,
        )

        # subscribe to stop wall for safety filtering
        self.stop_wall_sub_ = self.create_subscription(
            MarkerArray,
            "/input/stop_wall",
            self.stop_wall_callback,
            1,
        )

        self.get_logger().info("All subscribers set up.")

    # Set up all topic publishers
    def _init_publisher(self):
        self.trajectory_pub = self.create_publisher(Trajectory, "/output/trajectory", 1)
        self.timer_trajectory_pub = self.create_timer(0.1, self.timer_callback_publish_trajectory)
        self.get_logger().info("Trajectory publisher set up.")

    # Set up services (if any)
    def _init_services(self):
        self.set_evasive_mode_service = self.create_service(
            srv_type=SetBool,
            srv_name='set_evasive_mode',
            callback=self.set_evasive_mode_callback
        )
        self.get_logger().info("Services set up.")

    def set_evasive_mode_callback(self, request, response):
        """
        Service callback to reset the evasive mode state.
        """
        if request.data:
            self.evasive_active = True
            self.get_logger().info("Evasive mode activated via service call.")
        else:
            self.evasive_active = False
            self.stop_wall_xy = None
            self.get_logger().info("Evasive mode deactivated via service call.")
        
        response.success = True
        return response

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
    
    def check_evasive_mode(self, traj_msg: Trajectory) -> None:
        """
        Evasive mode state machine.
        - If evasive is active: check only the release condition (obstacle behind AND back on lane).
        - If evasive is inactive: check only the trigger condition (cannot stop comfortably).
        Side effects:
          - Updates `self.evasive_active` in place.
          - Clears `self.stop_wall_xy` only on release.
        """

        log = self.get_logger()
        ev = self.params.evasive

        v0 = float(self.planner.cartesian_state.velocity)
        a0 = float(self.planner.cartesian_state.acceleration)

        # Distance to stop point (positive: ahead; negative: behind)
        dist_to_stop, ego_d = self.planner.get_longitudinal_distance_to_point(self.stop_wall_xy)

        # ---------------------------
        # 1) ACTIVE --> release only
        # ---------------------------
        if self.evasive_active:
            log.warn("------------- [Evasive] Mode active -------------")

            # Release the evasive mode if velocity is already low.
            if v0 < 1.5 and abs(ego_d) < 0.5:
                log.warn(
                    f"[Evasive] Mode released: speed {v0:.2f} m/s < 1 m/s."
                )
                self.evasive_active = False
                self.stop_wall_xy = None  # cleared only when mode ends
                log.warn("------------- [Evasive] Mode deactivated -------------")
                return

            # The stop wall is set externally and must not be cleared while active.
            # Compute longitudinal distance to stop point and lateral offset to lane center.
            dist_to_stop, ego_d = self.planner.get_longitudinal_distance_to_point(self.stop_wall_xy)
            log.warn(
                f"[Evasive] Status: lateral ego offset={ego_d:.2f} m, "
                f"distance to stop point={dist_to_stop:.2f} m"
            )

            # Release condition: obstacle sufficiently behind AND ego re-centered on lane.
            if (dist_to_stop <= -ev.release_longitudinal_back           # assume that we have passed the obstacle
                    and abs(ego_d) <= abs(ev.release_lateral_center)):  # ego is back near lane center
                self.evasive_active = False
                self.stop_wall_xy = None  # cleared only when mode ends
                log.warn("[Evasive] Mode released: obstacle passed and ego re-centered.")
                log.warn("------------- [Evasive] Mode deactivated -------------")
            return  # stay latched otherwise

        # --------------------------------
        # 2) INACTIVE --> trigger only
        # --------------------------------
        # If no stop wall is tracked, nothing to evaluate --> stay with Autoware.
        if self.stop_wall_xy is None:
            return

        # Ego kinematics required for stopping distance estimation.
        if self.planner.cartesian_state is None:
            return

        # Do not trigger at low speed: Autoware can brake, and evasive would not help.
        if v0 < float(ev.velocity_threshold):
            log.info(
                f"[Evasive] No trigger: speed {v0:.2f} m/s < min_trigger_speed {float(ev.velocity_threshold):.2f} m/s."
            )
            return

        # Compute jerk/acc constrained minimal stopping distance (+ buffer).
        try:
            d_stop = compute_min_stop_distance(
                v0=v0,
                a0=a0,
                acc_min=float(ev.acc_min),
                jerk_acc=float(ev.jerk_acc),
                jerk_dec=float(ev.jerk_dec),
            )
            if d_stop is not None:
                d_stop += float(ev.stop_buffer)
        except Exception as e:
            # If computation fails, do not trigger aggressively; keep Autoware.
            log.warn(f"[Evasive] Stop distance computation failed: {e}")
            return

        if d_stop is None:
            log.info("[Evasive] No trigger: stopping distance estimator returned None.")
            return

        log.info(
            f"[Evasive] Trigger check: distance_to_stop={dist_to_stop:.2f} m, "
            f"min_stop_distance+buffer={d_stop:.2f} m, v0={v0:.2f} m/s, a0={a0:.2f} m/s^2"
        )

        # Trigger condition: cannot stop smoothly before the stop point.
        if dist_to_stop < d_stop:
            self.evasive_active = True
            log.warn("------------- [Evasive] Mode activated -------------")
            log.warn(
                "[Evasive] Triggered: distance to stop point is shorter than required stopping distance."
            )
        # else: remain in Autoware mode


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
