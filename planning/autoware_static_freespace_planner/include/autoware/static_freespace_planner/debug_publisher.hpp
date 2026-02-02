// Copyright 2026 TIER IV, Inc.

#ifndef AUTOWARE_STATIC_FREESPACE_PLANNER_DEBUG_PUBLISHER_HPP_
#define AUTOWARE_STATIC_FREESPACE_PLANNER_DEBUG_PUBLISHER_HPP_

#include "autoware/static_freespace_planner/waypoint_loader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>

class TestDebugPublisher;

namespace autoware::static_freespace_planner
{
using visualization_msgs::msg::Marker;
using std_msgs::msg::String;
class DebugPublisher
{
public:
  DebugPublisher(
  rclcpp::Node* node,
  rclcpp::Publisher<Marker>::SharedPtr marker_pub,
  rclcpp::Publisher<String>::SharedPtr route_name_pub);

  void publishPathMarker(
    const std::vector<WaypointLoader::Waypoint>& waypoints,
    const std::string& frame_id);

  void publishRouteName(const std::string& route_name);

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<String>::SharedPtr route_name_pub_;

  Marker createLineStripMarker(
    const std::vector<WaypointLoader::Waypoint>& waypoints);

  friend class ::TestDebugPublisher;
};
} // namespace autoware::static_freespace_planner
#endif // AUTOWARE_STATIC_FREESPACE_PLANNER_DEBUG_PUBLISHER_HPP_