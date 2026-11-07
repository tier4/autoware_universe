// Copyright 2025 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/trajectory_time_overlap/footprints.hpp"
#include "autoware/trajectory_time_overlap/overlap.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::trajectory_time_overlap
{
namespace
{
using autoware_planning_msgs::msg::TrajectoryPoint;

autoware::vehicle_info_utils::VehicleInfo make_vehicle()
{
  autoware::vehicle_info_utils::VehicleInfo info;
  info.wheel_base_m = 2.5;
  info.front_overhang_m = 1.0;
  info.rear_overhang_m = 1.0;
  info.wheel_tread_m = 1.5;
  info.left_overhang_m = 0.25;
  info.right_overhang_m = 0.25;
  info.max_longitudinal_offset_m = info.front_overhang_m + info.wheel_base_m;
  info.min_longitudinal_offset_m = -info.rear_overhang_m;
  return info;
}

std::vector<TrajectoryPoint> straight_trajectory()
{
  std::vector<TrajectoryPoint> points;
  constexpr double speed = 5.0;
  for (double s = 0.0; s <= 40.0 + 1e-6; s += 1.0) {
    TrajectoryPoint point;
    point.pose.position.x = s;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(speed);
    point.time_from_start = rclcpp::Duration::from_seconds(s / speed);
    points.push_back(point);
  }
  return points;
}

geometry_msgs::msg::Pose pose_at(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

std::vector<geometry_msgs::msg::Pose> path_along_y(
  const double x, const double y0, const double y1, const int steps)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  for (int i = 0; i <= steps; ++i) {
    const auto ratio = static_cast<double>(i) / static_cast<double>(steps);
    poses.push_back(pose_at(x, y0 + (y1 - y0) * ratio));
  }
  return poses;
}

const std::vector<universe_utils::Point2d> k_rectangle = {
  {0.2, 0.2}, {0.2, -0.2}, {-0.2, -0.2}, {-0.2, 0.2}};
const std::vector<universe_utils::Point2d> k_triangle = {{0.3, 0.0}, {-0.2, 0.25}, {-0.2, -0.25}};

std::vector<TimeOverlapIntervalPair> overlaps_for(
  const TrajectoryCornerFootprint & ego, const std::vector<geometry_msgs::msg::Pose> & poses,
  const double time_step, const std::vector<universe_utils::Point2d> & outline)
{
  const auto object_footprint = build_object_path_footprint(poses, time_step, outline);
  return combine_time_overlap_intervals(
    calculate_ego_and_object_time_overlap_intervals(ego, object_footprint), 0.0);
}

TrajectoryCornerFootprint make_ego_footprint()
{
  return calculate_trajectory_corner_footprint(straight_trajectory(), make_vehicle(), 0.0, 0.0);
}
}  // namespace

TEST(TrajectoryTimeOverlap, EmptyIntervalsAreNotCleared)
{
  EXPECT_FALSE(cleared_before_ego({}, 0.5));
}

TEST(TrajectoryTimeOverlap, SinglePoseProducesNoIntervals)
{
  const auto ego = make_ego_footprint();
  const auto intervals = overlaps_for(ego, {pose_at(20.0, 0.0)}, 0.5, k_rectangle);
  EXPECT_TRUE(intervals.empty());
  EXPECT_FALSE(cleared_before_ego(intervals, 0.5));
}

TEST(TrajectoryTimeOverlap, FastCrosserClearsBeforeEgo)
{
  const auto ego = make_ego_footprint();
  // y = 4 -> -4 over 2s. Ego's front reaches x=20 at about 3.3s.
  const auto intervals = overlaps_for(ego, path_along_y(20.0, 4.0, -4.0, 8), 0.25, k_rectangle);
  ASSERT_FALSE(intervals.empty());
  EXPECT_TRUE(cleared_before_ego(intervals, 0.5));
  for (const auto & interval : intervals) {
    EXPECT_TRUE(interval.exited);
    EXPECT_LT(interval.object.to + 0.5, interval.ego.from);
  }
}

TEST(TrajectoryTimeOverlap, SlowCrosserStillInsideWhenEgoArrives)
{
  const auto ego = make_ego_footprint();
  // Same crossing stretched over 8s, so the object is still in the corridor at ego's arrival.
  const auto intervals = overlaps_for(ego, path_along_y(20.0, 4.0, -4.0, 16), 0.5, k_rectangle);
  ASSERT_FALSE(intervals.empty());
  EXPECT_FALSE(cleared_before_ego(intervals, 0.5));
}

TEST(TrajectoryTimeOverlap, HorizonCutoffWhileStillInsideIsNotAnExit)
{
  const auto ego = make_ego_footprint();
  std::vector<geometry_msgs::msg::Pose> poses;
  for (int i = 0; i < 3; ++i) {
    poses.push_back(pose_at(20.0, 0.0));
  }
  const auto intervals = overlaps_for(ego, poses, 0.5, k_rectangle);
  ASSERT_FALSE(intervals.empty());
  EXPECT_FALSE(cleared_before_ego(intervals, 0.5));
  for (const auto & interval : intervals) {
    EXPECT_FALSE(interval.exited);
  }
}

TEST(TrajectoryTimeOverlap, ReentryThatMeetsEgoIsNotCleared)
{
  const auto ego = make_ego_footprint();
  std::vector<geometry_msgs::msg::Pose> poses;
  const std::vector<double> ys = {0.0, 2.0, 4.0, 4.0, 4.0, 4.0, 2.0, 0.0, 0.0, 0.0, 0.0};
  for (const auto y : ys) {
    poses.push_back(pose_at(20.0, y));
  }
  const auto intervals = overlaps_for(ego, poses, 0.5, k_rectangle);
  ASSERT_GE(intervals.size(), 1UL);
  EXPECT_FALSE(cleared_before_ego(intervals, 0.5));
}

TEST(TrajectoryTimeOverlap, RectangleAndTriangleBothCrossTheCorridor)
{
  const auto ego = make_ego_footprint();
  const auto poses = path_along_y(20.0, 4.0, -4.0, 8);
  const auto rectangle_intervals = overlaps_for(ego, poses, 0.25, k_rectangle);
  const auto triangle_intervals = overlaps_for(ego, poses, 0.25, k_triangle);
  EXPECT_FALSE(rectangle_intervals.empty());
  EXPECT_FALSE(triangle_intervals.empty());
}
}  // namespace autoware::trajectory_time_overlap
