// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "autoware/trajectory_modifier/trajectory_modifier_utils/detection_area_utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{
using autoware::trajectory_modifier::utils::detection_area::TargetFiltering;
using autoware::trajectory_modifier::utils::detection_area::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint make_point(const double x, const double y = 0.0)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = 5.0F;
  return point;
}

Trajectory make_path(const double end_x)
{
  std::vector<TrajectoryPoint> points;
  for (double x = 0.0; x <= end_x; x += 1.0) points.push_back(make_point(x));
  return *Trajectory::Builder{}.build(points);
}

lanelet::LineString3d make_line(const double x)
{
  return lanelet::LineString3d(
    lanelet::utils::getId(),
    {lanelet::Point3d(lanelet::utils::getId(), x, -2.0, 0.0),
     lanelet::Point3d(lanelet::utils::getId(), x, 2.0, 0.0)});
}

lanelet::ConstPolygons3d make_area()
{
  lanelet::Polygon3d area;
  area.push_back(lanelet::Point3d(lanelet::utils::getId(), 1.0, -1.0, 0.0));
  area.push_back(lanelet::Point3d(lanelet::utils::getId(), 1.0, 1.0, 0.0));
  area.push_back(lanelet::Point3d(lanelet::utils::getId(), 3.0, 1.0, 0.0));
  area.push_back(lanelet::Point3d(lanelet::utils::getId(), 3.0, -1.0, 0.0));
  return {area};
}
}  // namespace

TEST(DetectionAreaUtils, StopPointUsesIntersectionMarginAndVehicleOffset)
{
  const auto path = make_path(20.0);
  const auto line = make_line(10.0);
  const auto stop_point =
    autoware::trajectory_modifier::utils::detection_area::get_stop_point(path, line, 1.0, 2.0);
  ASSERT_TRUE(stop_point);
  EXPECT_NEAR(*stop_point, 7.0, 1e-6);
}

TEST(DetectionAreaUtils, StopPointReturnsEmptyWhenTrajectoryDoesNotCrossLine)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_point(0.0, 10.0));
  points.push_back(make_point(20.0, 10.0));
  const auto path = *Trajectory::Builder{}.build(points);
  EXPECT_FALSE(
    autoware::trajectory_modifier::utils::detection_area::get_stop_point(
      path, make_line(10.0), 0.0, 0.0));
}

TEST(DetectionAreaUtils, PointCloudDetectionUsesPolygonInterior)
{
  autoware::trajectory_modifier::utils::detection_area::PointCloud points;
  points.emplace_back(0.0F, 0.0F, 0.0F);
  points.emplace_back(2.0F, 0.0F, 0.0F);
  const auto obstacles =
    autoware::trajectory_modifier::utils::detection_area::get_obstacle_points(make_area(), points);
  ASSERT_EQ(obstacles.size(), 1U);
  EXPECT_DOUBLE_EQ(obstacles.front().x, 2.0);
}

TEST(DetectionAreaUtils, TargetFilteringUsesHighestProbabilityClassification)
{
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  TargetFiltering filtering;
  filtering.car = true;

  Classification low_probability_pedestrian;
  low_probability_pedestrian.label = Classification::PEDESTRIAN;
  low_probability_pedestrian.probability = 0.1F;
  Classification high_probability_car;
  high_probability_car.label = Classification::CAR;
  high_probability_car.probability = 0.9F;

  EXPECT_TRUE(
    autoware::trajectory_modifier::utils::detection_area::is_target_object(
      {low_probability_pedestrian, high_probability_car}, filtering));
  EXPECT_EQ(
    autoware::trajectory_modifier::utils::detection_area::object_label_to_string(
      Classification::CAR),
    "car");
}

TEST(DetectionAreaUtils, StopStateClearsOnlyAfterConfiguredDuration)
{
  const std::optional<rclcpp::Time> last_obstacle{rclcpp::Time(10, 0)};
  EXPECT_FALSE(
    autoware::trajectory_modifier::utils::detection_area::can_clear_stop_state(
      last_obstacle, rclcpp::Time(11, 0), 2.0));
  EXPECT_TRUE(
    autoware::trajectory_modifier::utils::detection_area::can_clear_stop_state(
      last_obstacle, rclcpp::Time(12, 0), 2.0));
  EXPECT_TRUE(
    autoware::trajectory_modifier::utils::detection_area::can_clear_stop_state(
      std::nullopt, rclcpp::Time(0, 0), 2.0));
}

TEST(DetectionAreaUtils, FeasibleStopDistanceIsSafeForInvalidAcceleration)
{
  EXPECT_DOUBLE_EQ(
    autoware::trajectory_modifier::utils::detection_area::feasible_stop_distance_by_max_acceleration(
      5.0, 0.0),
    0.0);
  EXPECT_DOUBLE_EQ(
    autoware::trajectory_modifier::utils::detection_area::feasible_stop_distance_by_max_acceleration(
      5.0, 2.0),
    6.25);
}
