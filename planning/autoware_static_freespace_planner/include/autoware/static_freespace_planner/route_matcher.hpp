// Copyright 2026 TIER IV, Inc.

#ifndef AUTOWARE__STATIC_FREESPACE_PLANNER__ROUTE_MATCHER_HPP_
#define AUTOWARE__STATIC_FREESPACE_PLANNER__ROUTE_MATCHER_HPP_

#include "autoware/static_freespace_planner/route_index_loader.hpp"
#include "autoware/static_freespace_planner/waypoint_loader.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <string>
#include <vector>

class TestRouteMatcher;

namespace autoware::static_freespace_planner
{
using geometry_msgs::msg::Pose;
class RouteMatcher
{
public:
  RouteMatcher(
    const std::vector<RouteIndexLoader::RouteDefinition> & routes, double search_radius_m,
    double yaw_threshold_rad);

  std::optional<RouteIndexLoader::RouteDefinition> findMatchingRoute(
    const Pose & start_pose, const Pose & goal_pose);

private:
  std::vector<RouteIndexLoader::RouteDefinition> routes_;
  double search_radius_m_;
  double yaw_threshold_rad_;
  WaypointLoader waypoint_loader_;

  double calcDistance2D(const Pose & p1, const Pose & p2);
  double calcYawDifference(const Pose & p1, const Pose & p2);

  friend class ::TestRouteMatcher;
};
}  // namespace autoware::static_freespace_planner
#endif  // AUTOWARE__STATIC_FREESPACE_PLANNER__ROUTE_MATCHER_HPP_
