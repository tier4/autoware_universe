import lanelet2  # noqa: F401 # isort: skip
# from autoware_lanelet2_extension_python.utility.utilities import fromBinMsg
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import Path
from autoware_planning_msgs.msg import Trajectory
from autoware_control_msgs.msg import Control
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped, Pose, Point, Quaternion
from autoware_planning_msgs.msg import TrajectoryPoint
import numpy as np
import rclpy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.node import Node
import rclpy.qos
from autoware_frenetix_planner.frenetix_motion_planner import FrenetixMotionPlanner
from tf_transformations import quaternion_from_euler
from builtin_interfaces.msg import Duration

# Main planner node class
class FrenetixPlanner(Node):

    def __init__(self):
        super().__init__("frenetix_planner")
        self.get_logger().info("Node initialized.")
        self.lanelet_map_ = None  # Stores the received lanelet map
        self._init_subscriber()   # Set up all topic subscribers
        self._init_publisher()    # Set up all topic publishers
        self.get_logger().info("Subscribers and publishers initialized.")
        self.planner = FrenetixMotionPlanner(logger=self.get_logger())
        self.get_logger().info("FrenetixMotionPlanner initialized.")
        self.car_altitude = 0.0  # Default altitude for trajectory points

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
        self.get_logger().info(f"Received objects message with {len(objects_msg.points)} points.")

    # Callback for synchronized odometry and acceleration messages
    def kinematic_state_callback(self, odometry_msg, accel_msg, control_msg):
        
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
        # self.lanelet_map_ = fromBinMsg(map_msg)
        # self.get_logger().info("Lanelet map converted.")

    # Timer callback to publish trajectory
    def timer_callback_publish_trajectory(self):
        frenetix_trajectory = self.planner.cyclic_plan()
        trajectory_msg = self.convert_frenetix_to_autoware_trajectory(frenetix_trajectory, car_altitude=self.car_altitude)
        if trajectory_msg:
            self.trajectory_pub.publish(trajectory_msg)

    # Set up all topic subscribers
    def _init_subscriber(self):
        self.reference_trajectory_sub_ = self.create_subscription(
            Path, "~/input/reference_path", self.reference_path_callback, 1
        )
        self.predicted_objects_sub_ = self.create_subscription(
            Trajectory, "~/input/objects", self.objects_callback, 1
        )

        # Use message_filters for synchronized kinematic state
        self.odom_sub = Subscriber(self, Odometry, "~/input/odometry")
        self.accel_sub = Subscriber(self, AccelWithCovarianceStamped, "~/input/acceleration")
        self.control_sub = Subscriber(self, Control, "~/input/control_cmd")
        self.ksts = ApproximateTimeSynchronizer([self.odom_sub, self.accel_sub, self.control_sub], queue_size=10, slop=0.05, allow_headerless=True)
        self.ksts.registerCallback(self.kinematic_state_callback)

        map_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub_ = self.create_subscription(
            LaneletMapBin, "~/input/vector_map", self.vector_map_callback, map_qos
        )
        self.get_logger().info("All subscribers set up.")

    # Set up all topic publishers
    def _init_publisher(self):
        self.trajectory_pub = self.create_publisher(Trajectory, "~/output/trajectory", 1)
        self.timer_trajectory_pub = self.create_timer(0.1, self.timer_callback_publish_trajectory)
        self.get_logger().info("Trajectory publisher set up.")

    # convert frenetix trajecory to autoware trajectory message

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
    rclpy.init()
    node = FrenetixPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
