import lanelet2  # noqa: F401 # isort: skip
from autoware_lanelet2_extension_python.utility.utilities import fromBinMsg
from autoware_map_msgs.msg import LaneletMapBin
from autoware_planning_msgs.msg import Path
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import rclpy.qos


class FrenetixPlanner(Node):

    def __init__(self):
        super().__init__("frenetix_planner")
        self.reference_trajectory_sub_ = self.create_subscription(
            Path, "~/input/reference_path", self.reference_path_callback, 1
        )
        self.predicted_objects_sub_ = self.create_subscription(
            Trajectory, "~/input/objects", self.objects_callback, 1
        )
        self.odometry_sub_ = self.create_subscription(
            Odometry, "~/input/odometry", self.odometry_callback, 1
        )
        map_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub_ = self.create_subscription(
            LaneletMapBin, "~/input/vector_map", self.vector_map_callback, map_qos
        )
        self.trajectory_pub_ = self.create_publisher(Trajectory, "~/output/trajectory", 1)

    def reference_path_callback(self, reference_trajectory_msg):
        print("TODO")

    def objects_callback(self, objects_msg):
        print("TODO")

    def odometry_callback(self, odometry_msg):
        print("TODO")

    def vector_map_callback(self, map_msg: LaneletMapBin):
        print("got map", map_msg.name_map)
        self.lanelet_map_ = fromBinMsg(map_msg)


def main():
    rclpy.init()

    node = FrenetixPlanner()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
