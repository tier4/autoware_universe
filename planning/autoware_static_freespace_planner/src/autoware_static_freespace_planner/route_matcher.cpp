// Copyright 2026 TIER IV, Inc.

#include "autoware/static_freespace_planner/route_matcher.hpp"
#include "autoware/static_freespace_planner/route_index_loader.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <autoware_utils/geometry/geometry.hpp>

namespace autoware::static_freespace_planner
{
RouteMatcher::RouteMatcher(const std::vector<RouteIndexLoader::RouteDefinition>& routes, double search_radius_m, double yaw_threshold_rad)
: routes_(routes), search_radius_m_(search_radius_m), yaw_threshold_rad_(yaw_threshold_rad)
{}

std::optional<RouteIndexLoader::RouteDefinition> RouteMatcher::findMatchingRoute(
  const Pose& start_pose,
  const Pose& goal_pose)
{
  if (routes_.empty()) {
    return std::nullopt;
  }

  RouteIndexLoader::RouteDefinition candidate_route;
  double candidate_total_distance = std::numeric_limits<double>::max();

  for (const auto& route_definition : routes_) {
    // Load waypoints from the route definition
    const auto& route_start_wp = waypoint_loader_.loadFirstWaypoint(route_definition.csv_path);
    const auto& route_goal_wp = waypoint_loader_.loadLastWaypoint(route_definition.csv_path);

    Pose route_start_pose;
    route_start_pose.position.x = route_start_wp.x;
    route_start_pose.position.y = route_start_wp.y;
    route_start_pose.position.z = route_start_wp.z;
    route_start_pose.orientation.x = route_start_wp.qx;
    route_start_pose.orientation.y = route_start_wp.qy;
    route_start_pose.orientation.z = route_start_wp.qz;
    route_start_pose.orientation.w = route_start_wp.qw;

    Pose route_goal_pose;
    route_goal_pose.position.x = route_goal_wp.x;
    route_goal_pose.position.y = route_goal_wp.y;
    route_goal_pose.position.z = route_goal_wp.z;
    route_goal_pose.orientation.x = route_goal_wp.qx;
    route_goal_pose.orientation.y = route_goal_wp.qy;
    route_goal_pose.orientation.z = route_goal_wp.qz;
    route_goal_pose.orientation.w = route_goal_wp.qw;

    // Calculate distance and orientation differences between start and goal poses
    const double start_distance = calcDistance2D(start_pose, route_start_pose);
    const double goal_distance = calcDistance2D(goal_pose, route_goal_pose);
    const double start_yaw_diff = calcYawDifference(start_pose, route_start_pose);
    const double goal_yaw_diff = calcYawDifference(goal_pose, route_goal_pose);

    // If all thresholds are met, matching is successful
    if (start_distance <= search_radius_m_ &&
        goal_distance <= search_radius_m_ &&
        start_yaw_diff <= yaw_threshold_rad_ &&
        goal_yaw_diff <= yaw_threshold_rad_) {
      const double total_distance = start_distance + goal_distance;
      // Select the route with the smallest total distance
      if (candidate_total_distance > total_distance) {
        candidate_total_distance = total_distance;
        candidate_route = route_definition;
      }
    }
  }

  // Return the best matching route or nullopt if none found
  if (candidate_total_distance == std::numeric_limits<double>::max()) {
    return std::nullopt;
  }

  return candidate_route;
}

double RouteMatcher::calcDistance2D(const Pose& p1, const Pose& p2)
{
  const double dx = p1.position.x - p2.position.x;
  const double dy = p1.position.y - p2.position.y;
  return std::sqrt(dx * dx + dy * dy);
}
double RouteMatcher::calcYawDifference(const Pose& p1, const Pose& p2)
{
  // Extract yaw angle from quaternion  
  tf2::Quaternion q1(p1.orientation.x, p1.orientation.y,
  p1.orientation.z, p1.orientation.w);
  tf2::Quaternion q2(p2.orientation.x, p2.orientation.y,
  p2.orientation.z, p2.orientation.w);

  tf2::Matrix3x3 mat1(q1);
  tf2::Matrix3x3 mat2(q2);

  double roll1, pitch1, yaw1;
  double roll2, pitch2, yaw2;

  mat1.getRPY(roll1, pitch1, yaw1);
  mat2.getRPY(roll2, pitch2, yaw2);

  // Calculate angle difference (-π ~ π normalization)
  double diff = yaw2 - yaw1;
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;

  // Return absolute value in radians
  return std::abs(diff);

}
} // namespace autoware::static_freespace_planner
