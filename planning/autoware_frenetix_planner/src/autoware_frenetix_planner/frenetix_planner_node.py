import lanelet2  # noqa: F401 # isort: skip
# from autoware_lanelet2_extension_python.utility.utilities import fromBinMsg
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import Path
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
import numpy as np
import rclpy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.node import Node
import rclpy.qos
from autoware_frenetix_planner.frenetix_motion_planner import FrenetixMotionPlanner

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
    def kinematic_state_callback(self, odometry_msg, accel_msg):
        # Extract position and orientation from odometry
        position = odometry_msg.pose.pose.position
        orientation = odometry_msg.pose.pose.orientation
        # Extract velocity from odometry
        velocity = odometry_msg.twist.twist.linear.x
        # Extract acceleration from accel_msg
        acceleration = accel_msg.accel.accel.linear.x

        # Set/update ego state in planner
        if self.planner.set_kinematic_state(position, orientation, velocity=velocity, acceleration=acceleration):
            self.get_logger().debug("Synchronized kinematic state set/updated.")

    # Callback for vector map messages
    def vector_map_callback(self, map_msg: LaneletMapBin):
        self.get_logger().info(f"Received vector map message: {getattr(map_msg, 'name_map', None)}")
        # Conversion from binary map message to lanelet map would happen here
        # self.lanelet_map_ = fromBinMsg(map_msg)
        # self.get_logger().info("Lanelet map converted.")

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
        self.ksts = ApproximateTimeSynchronizer([self.odom_sub, self.accel_sub], queue_size=10, slop=0.05)
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
        self.trajectory_pub_ = self.create_publisher(Trajectory, "~/output/trajectory", 1)
        self.get_logger().info("Trajectory publisher set up.")

# Main entry point for the node
def main():
    rclpy.init()
    node = FrenetixPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
