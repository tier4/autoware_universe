// Copyright 2026 TIER IV, Inc.

#include "autoware/static_freespace_planner/debug_publisher.hpp"

#include <geometry_msgs/msg/point.hpp>

namespace autoware::static_freespace_planner
{
DebugPublisher::DebugPublisher(
  rclcpp::Node * node, rclcpp::Publisher<Marker>::SharedPtr marker_pub,
  rclcpp::Publisher<String>::SharedPtr route_name_pub)
: node_(node), marker_pub_(marker_pub), route_name_pub_(route_name_pub)
{
}

void DebugPublisher::publishPathMarker(
  const std::vector<WaypointLoader::Waypoint> & waypoints, const std::string & frame_id)
{
  Marker marker = createLineStripMarker(waypoints);
  marker.header.frame_id = frame_id;
  marker.header.stamp = node_->get_clock()->now();
  marker_pub_->publish(marker);
}

void DebugPublisher::publishRouteName(const std::string & route_name)
{
  String msg;
  msg.data = route_name;
  route_name_pub_->publish(msg);
}

Marker DebugPublisher::createLineStripMarker(
  const std::vector<WaypointLoader::Waypoint> & waypoints)
{
  Marker marker;
  marker.ns = "staticfreespace_planner";
  marker.id = 0;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.scale.x = 0.1;  // line width
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  for (const auto & waypoint : waypoints) {
    geometry_msgs::msg::Point p;
    p.x = waypoint.x;
    p.y = waypoint.y;
    p.z = waypoint.z;
    marker.points.push_back(p);
  }

  return marker;
}
}  // namespace autoware::static_freespace_planner
