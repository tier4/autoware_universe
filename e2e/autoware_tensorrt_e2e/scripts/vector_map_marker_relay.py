#!/usr/bin/env python3
"""Republish the lanelet2 marker array so RViz can actually draw it.

Two things stop the map from appearing in RViz when this node is exercised
against a rosbag, and neither is visible as an error:

1. `autoware_lanelet2_map_visualizer` emits duplicate `(ns, id)` marker keys —
   1808 of them on the tadaiba map, `traffic_light_triangle` worst at 25x. RViz
   refuses the array on its duplicate check and draws nothing at all. The ids
   are renumbered here; geometry is untouched.
2. A looping `ros2 bag play --clock` jumps simulated time backwards once per
   cycle, RViz resets and discards everything it holds, and a latched
   (transient-local) message is delivered once per subscription and never
   again. A slow heartbeat re-publishes the map so the view heals itself.

Run it beside the map loader; point RViz at the output topic.

    ros2 run autoware_tensorrt_e2e vector_map_marker_relay.py
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray

HEARTBEAT_SECONDS = 5.0


class VectorMapMarkerRelay(Node):
    def __init__(self) -> None:
        super().__init__("vector_map_marker_relay")
        input_topic = self.declare_parameter("input_topic", "/map/vector_map_marker").value
        output_topic = self.declare_parameter(
            "output_topic", "/map/vector_map_marker_clean").value
        heartbeat = self.declare_parameter("heartbeat_seconds", HEARTBEAT_SECONDS).value

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._markers: MarkerArray | None = None
        self._pub = self.create_publisher(MarkerArray, output_topic, qos)
        self.create_subscription(MarkerArray, input_topic, self._on_markers, qos)
        if heartbeat > 0.0:
            self.create_timer(heartbeat, self._heartbeat)

    def _on_markers(self, msg: MarkerArray) -> None:
        seen: set[tuple[str, int]] = set()
        next_id: dict[str, int] = {}
        duplicates = 0
        for marker in msg.markers:
            key = (marker.ns, marker.id)
            if key in seen:
                duplicates += 1
                candidate = next_id.get(marker.ns, 1_000_000)
                while (marker.ns, candidate) in seen:
                    candidate += 1
                marker.id = candidate
                next_id[marker.ns] = candidate + 1
                key = (marker.ns, marker.id)
            seen.add(key)
        self._markers = msg
        self._pub.publish(msg)
        self.get_logger().info(
            f"republished {len(msg.markers)} markers ({duplicates} ids renumbered)")

    def _heartbeat(self) -> None:
        if self._markers is not None:
            self._pub.publish(self._markers)


def main() -> None:
    rclpy.init()
    node = VectorMapMarkerRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
