// Copyright 2026 TIER IV, Inc.

#ifndef AUTOWARE_STATIC_FREESPACE_PLANNER_TRAJECTORY_GENERATOR_HPP_
#define AUTOWARE_STATIC_FREESPACE_PLANNER_TRAJECTORY_GENERATOR_HPP_

#include "autoware/static_freespace_planner/waypoint_loader.hpp"
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

class TestTrajectoryGenerator;

namespace autoware::static_freespace_planner
{
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;

class TrajectoryGenerator
{
public:
  Trajectory createTrajectory(
    const std::vector<WaypointLoader::Waypoint>& waypoints,
    const PoseStamped& current_pose);

  Trajectory createStopTrajectory(
    const PoseStamped& current_pose,
    const rclcpp::Clock::SharedPtr clock);

  // Generate trajectory filtered by Seq
  Trajectory createTrajectoryForSeq(
    const std::vector<WaypointLoader::Waypoint>& all_waypoints,
    int target_seq,
    const PoseStamped& current_pose);

private:
  Pose waypointToPose(const WaypointLoader::Waypoint& waypoint);
  friend class ::TestTrajectoryGenerator;
};
} // namespace autoware::static_freespace_planner
#endif // AUTOWARE_STATIC_FREESPACE_PLANNER_TRAJECTORY_GENERATOR_HPP_