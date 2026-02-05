// Copyright 2026 TIER IV, Inc.

#include "autoware/static_freespace_planner/trajectory_generator.hpp"

#include "autoware/static_freespace_planner/waypoint_loader.hpp"

#include <vector>

namespace autoware::static_freespace_planner
{
using autoware_planning_msgs::msg::TrajectoryPoint;

Trajectory TrajectoryGenerator::createTrajectoryForSeq(
  const std::vector<WaypointLoader::Waypoint> & all_waypoints, int target_seq,
  const PoseStamped & current_pose)
{
  Trajectory traj;

  for (const auto & waypoint : all_waypoints) {
    if (waypoint.seq != target_seq) {
      continue;
    }

    TrajectoryPoint point;
    point.pose = waypointToPose(waypoint);
    point.pose.position.z = current_pose.pose.position.z;  // Set height to current z value

    // Use the specified speed in the waypoint (m/s)
    // Positive: forward, Negative: backward
    point.longitudinal_velocity_mps = waypoint.mps;

    traj.points.push_back(point);
  }

  return traj;
}

Trajectory TrajectoryGenerator::createStopTrajectory(
  const PoseStamped & current_pose, const rclcpp::Clock::SharedPtr clock)
{
  Trajectory traj;
  traj.header.stamp = clock->now();
  traj.header.frame_id = current_pose.header.frame_id;

  TrajectoryPoint point;
  point.pose = current_pose.pose;
  point.longitudinal_velocity_mps = 0.0;

  traj.points.push_back(point);
  return traj;
}

Trajectory TrajectoryGenerator::createTrajectory(
  const std::vector<WaypointLoader::Waypoint> & waypoints, const PoseStamped & current_pose)
{
  Trajectory traj;
  for (const auto & waypoint : waypoints) {
    TrajectoryPoint point;
    point.pose = waypointToPose(waypoint);
    point.pose.position.z = current_pose.pose.position.z;  // Set height to current z value

    // Use the specified speed in the waypoint (m/s)
    // Positive: forward, Negative: backward
    point.longitudinal_velocity_mps = waypoint.mps;

    traj.points.push_back(point);
  }

  return traj;
}

Pose TrajectoryGenerator::waypointToPose(const WaypointLoader::Waypoint & waypoint)
{
  Pose pose;
  pose.position.x = waypoint.x;
  pose.position.y = waypoint.y;
  pose.position.z = waypoint.z;
  pose.orientation.x = waypoint.qx;
  pose.orientation.y = waypoint.qy;
  pose.orientation.z = waypoint.qz;
  pose.orientation.w = waypoint.qw;
  return pose;
}
}  // namespace autoware::static_freespace_planner
