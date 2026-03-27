// Copyright 2026 TIER IV, Inc.
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

#include "../../src/filters/safety/collision_check_filter.cpp"

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <cmath>
#include <iterator>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{

geometry_msgs::msg::Pose create_pose(const double x, const double y, const double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  return pose;
}

geometry_msgs::msg::Twist create_twist(
  const double linear_x, const double angular_z = 0.0, const double linear_y = 0.0)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_x;
  twist.linear.y = linear_y;
  twist.angular.z = angular_z;
  return twist;
}

TrajectoryPoints create_straight_trajectory_points(const std::vector<double> & xs)
{
  TrajectoryPoints traj_points;
  traj_points.reserve(xs.size());
  for (const auto x : xs) {
    TrajectoryPoint point;
    point.pose = create_pose(x, 0.0, 0.0);
    traj_points.push_back(point);
  }
  return traj_points;
}

autoware_perception_msgs::msg::Shape create_bounding_box_shape(
  const double length = 4.0, const double width = 2.0)
{
  autoware_perception_msgs::msg::Shape shape;
  shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  shape.dimensions.x = length;
  shape.dimensions.y = width;
  shape.dimensions.z = 1.5;
  return shape;
}

autoware_perception_msgs::msg::PredictedPath create_straight_predicted_path(
  const double y, const double confidence, const std::vector<double> & xs)
{
  autoware_perception_msgs::msg::PredictedPath predicted_path;
  predicted_path.confidence = confidence;
  predicted_path.time_step = rclcpp::Duration::from_seconds(TIME_RESOLUTION);
  predicted_path.path.reserve(xs.size());
  for (const auto x : xs) {
    predicted_path.path.push_back(create_pose(x, y, 0.0));
  }
  return predicted_path;
}

autoware_perception_msgs::msg::PredictedObject create_predicted_object(
  const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Twist & initial_twist,
  const autoware_perception_msgs::msg::Shape & shape,
  const std::vector<autoware_perception_msgs::msg::PredictedPath> & predicted_paths)
{
  autoware_perception_msgs::msg::PredictedObject object;
  object.object_id = autoware_utils_uuid::generate_uuid();
  object.kinematics.initial_pose_with_covariance.pose = initial_pose;
  object.kinematics.initial_twist_with_covariance.twist = initial_twist;
  object.kinematics.predicted_paths = predicted_paths;
  object.shape = shape;
  return object;
}

VehicleInfo create_vehicle_info()
{
  VehicleInfo vehicle_info;
  vehicle_info.max_longitudinal_offset_m = 4.0;
  vehicle_info.min_longitudinal_offset_m = -1.0;
  vehicle_info.vehicle_width_m = 2.0;
  return vehicle_info;
}

void expect_same_polygon(const Polygon2d & actual, const Polygon2d & expected)
{
  ASSERT_EQ(actual.outer().size(), expected.outer().size());
  for (size_t i = 0; i < actual.outer().size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.outer().at(i).x(), expected.outer().at(i).x());
    EXPECT_DOUBLE_EQ(actual.outer().at(i).y(), expected.outer().at(i).y());
  }
}

}  // namespace

TEST(TrajectoryUtilitiesTest, ComputePoseTrajectoryInterpolatesAndClamps)
{
  const auto traj_points = create_straight_trajectory_points({0.0, 10.0, 20.0});
  const TravelDistanceTrajectory distances = {0.0, 5.0, 15.0, 25.0};

  const auto poses = trajectory::pose::compute_pose_trajectory(traj_points, distances);

  ASSERT_EQ(poses.size(), distances.size());
  EXPECT_DOUBLE_EQ(poses.at(0).position.x, 0.0);
  EXPECT_DOUBLE_EQ(poses.at(1).position.x, 5.0);
  EXPECT_DOUBLE_EQ(poses.at(2).position.x, 15.0);
  EXPECT_DOUBLE_EQ(poses.at(3).position.x, 20.0);
  EXPECT_DOUBLE_EQ(poses.at(3).position.y, 0.0);
}

TEST(TrajectoryUtilitiesTest, ComputePoseTrajectoryInterpolatesOrientationSpherically)
{
  TrajectoryPoint start_point;
  start_point.pose = create_pose(0.0, 0.0, 0.0);

  TrajectoryPoint end_point;
  end_point.pose = create_pose(10.0, 0.0, M_PI / 3.0);

  const TrajectoryPoints traj_points = {start_point, end_point};
  const TravelDistanceTrajectory distances = {5.0};

  const auto poses = trajectory::pose::compute_pose_trajectory(traj_points, distances);

  ASSERT_EQ(poses.size(), 1u);
  EXPECT_DOUBLE_EQ(poses.at(0).position.x, 5.0);
  EXPECT_DOUBLE_EQ(poses.at(0).position.y, 0.0);
  EXPECT_NEAR(tf2::getYaw(poses.at(0).orientation), M_PI / 6.0, 1e-6);
}

TEST(TrajectoryUtilitiesTest, ComputeFootprintTrajectoryForObjectShapeMatchesUtility)
{
  const PoseTrajectory poses = {create_pose(1.0, 2.0, 0.0)};
  const auto shape = create_bounding_box_shape(4.0, 2.0);

  const auto footprints = trajectory::footprint::compute_footprint_trajectory(poses, shape);

  ASSERT_EQ(footprints.size(), 1u);
  expect_same_polygon(
    footprints.front(), autoware_utils_geometry::to_polygon2d(poses.front(), shape));
}

TEST(TrajectoryUtilitiesTest, ComputeFootprintTrajectoryForVehicleMatchesUtility)
{
  const PoseTrajectory poses = {create_pose(1.0, 2.0, 0.0)};
  const auto vehicle_info = create_vehicle_info();

  const auto footprints = trajectory::footprint::compute_footprint_trajectory(poses, vehicle_info);

  ASSERT_EQ(footprints.size(), 1u);
  expect_same_polygon(
    footprints.front(), autoware_utils_geometry::to_footprint(
                          poses.front(), vehicle_info.max_longitudinal_offset_m,
                          -vehicle_info.min_longitudinal_offset_m, vehicle_info.vehicle_width_m));
}

TEST(TrajectoryUtilitiesTest, GenerateEgoTrajectoryBuildsConsistentTrajectoryData)
{
  auto vehicle_info = create_vehicle_info();
  const auto traj_points = create_straight_trajectory_points({0.0, 10.0, 20.0});
  const auto initial_twist = create_twist(2.0);

  const auto trajectory_data =
    trajectory::generate_ego_trajectory(initial_twist, 0.0, 0.0, 1.05, traj_points, vehicle_info);

  ASSERT_EQ(trajectory_data.getId(), "ego");
  ASSERT_EQ(trajectory_data.size(), 11u);
  EXPECT_DOUBLE_EQ(trajectory_data.getTimes().front(), 0.0);
  EXPECT_NEAR(trajectory_data.getTimes().back(), 1.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().back(), 2.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().back().position.x, 2.0, 1e-6);
  expect_same_polygon(
    trajectory_data.getFootprints().front(),
    autoware_utils_geometry::to_footprint(
      trajectory_data.getPoses().front(), vehicle_info.max_longitudinal_offset_m,
      -vehicle_info.min_longitudinal_offset_m, vehicle_info.vehicle_width_m));
}

TEST(TrajectoryUtilitiesTest, GeneratePredictedPathTrajectoryUsesHighestConfidencePath)
{
  const auto shape = create_bounding_box_shape(4.0, 2.0);
  const auto initial_pose = create_pose(0.0, 0.0, 0.0);
  const auto initial_twist = create_twist(1.0);
  const std::vector<autoware_perception_msgs::msg::PredictedPath> predicted_paths = {
    create_straight_predicted_path(10.0, 0.1, {0.0, 1.0, 2.0, 3.0, 4.0}),
    create_straight_predicted_path(0.0, 0.9, {0.0, 1.0, 2.0, 3.0, 4.0})};
  const auto object = create_predicted_object(initial_pose, initial_twist, shape, predicted_paths);

  const auto trajectory_data = trajectory::generate_predicted_path_trajectory(
    object, 0.0, 0.0, rclcpp::Duration::from_seconds(0.1), 0.35);

  ASSERT_EQ(trajectory_data.size(), 3u);
  EXPECT_EQ(trajectory_data.getId().find("_predicted_path"), trajectory_data.getId().size() - 15);
  EXPECT_NEAR(trajectory_data.getTimes().front(), 0.1, 1e-6);
  EXPECT_NEAR(trajectory_data.getTimes().back(), 0.3, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(0).position.x, 0.1, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(1).position.x, 0.2, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(2).position.x, 0.3, 1e-6);
  EXPECT_DOUBLE_EQ(trajectory_data.getPoses().at(0).position.y, 0.0);
}

TEST(TrajectoryUtilitiesTest, ComputeLongitudinalVelocityUsesPathYawForPathLongerThanEpsilon)
{
  // The path length is intentionally between 1e-3 and 1e3 to catch threshold typos.
  const PoseTrajectory points = {create_pose(0.0, 0.0, M_PI_2), create_pose(1.0, 0.0, M_PI_2)};
  const auto object = create_predicted_object(
    create_pose(0.5, 0.0, 0.0), create_twist(2.0), create_bounding_box_shape(), {});

  const auto longitudinal_velocity =
    rss_deceleration::compute_longitudinal_velocity(points, object);

  EXPECT_NEAR(longitudinal_velocity, 2.0, 1e-6);
}

TEST(TrajectoryUtilitiesTest, ComputeLongitudinalVelocityFallsBackToFrontPoseYawForDegeneratePath)
{
  const PoseTrajectory points = {
    create_pose(1.0, 2.0, M_PI / 3.0), create_pose(1.0, 2.0, -M_PI / 2.0)};
  const auto object = create_predicted_object(
    create_pose(1.0, 2.0, M_PI / 6.0), create_twist(2.0), create_bounding_box_shape(), {});

  const auto longitudinal_velocity =
    rss_deceleration::compute_longitudinal_velocity(points, object);

  EXPECT_NEAR(longitudinal_velocity, 2.0 * std::cos(M_PI / 6.0), 1e-6);
}

TEST(TrajectoryUtilitiesTest, ComputeLongitudinalVelocityThrowsOnEmptyPoints)
{
  const PoseTrajectory points;
  const auto object = create_predicted_object(
    create_pose(0.0, 0.0, 0.0), create_twist(2.0), create_bounding_box_shape(), {});

  EXPECT_THROW(
    rss_deceleration::compute_longitudinal_velocity(points, object), std::invalid_argument);
}

TEST(TrajectoryUtilitiesTest, GenerateConstantCurvaturePathTrajectoryMatchesPredictor)
{
  const auto shape = create_bounding_box_shape(4.0, 2.0);
  const auto initial_pose = create_pose(1.0, 2.0, 0.0);
  const auto initial_twist = create_twist(1.0, 1.0);
  const auto object = create_predicted_object(initial_pose, initial_twist, shape, {});

  const auto trajectory_data = trajectory::generate_constant_curvature_path_trajectory(
    object, 0.0, 0.0, rclcpp::Duration::from_seconds(0.0), 0.25);
  const auto [expected_times, expected_distances] =
    trajectory::time_distance::compute_motion_profile_1d(initial_twist, 0.0, 0.0, 0.0, 0.25);
  const auto expected_poses = trajectory::pose::constant_curvature_predictor::compute(
    initial_pose, initial_twist, expected_distances);

  ASSERT_EQ(trajectory_data.size(), expected_times.size());
  EXPECT_EQ(
    trajectory_data.getId().find("_constant_curvature_path"), trajectory_data.getId().size() - 24);
  for (size_t i = 0; i < expected_times.size(); ++i) {
    EXPECT_NEAR(trajectory_data.getTimes().at(i), expected_times.at(i), 1e-6);
    EXPECT_NEAR(trajectory_data.getPoses().at(i).position.x, expected_poses.at(i).position.x, 1e-6);
    EXPECT_NEAR(trajectory_data.getPoses().at(i).position.y, expected_poses.at(i).position.y, 1e-6);
    EXPECT_NEAR(
      tf2::getYaw(trajectory_data.getPoses().at(i).orientation),
      tf2::getYaw(expected_poses.at(i).orientation), 1e-6);
  }
}

TEST(TrajectoryUtilitiesTest, TrajectoryDataReturnsFootprintsInNearestTimeRange)
{
  const TimeTrajectory times = {0.0, 0.1, 0.2};
  const TravelDistanceTrajectory distances = {0.0, 1.0, 2.0};
  const PoseTrajectory poses = {
    create_pose(0.0, 0.0), create_pose(1.0, 0.0), create_pose(2.0, 0.0)};
  const auto shape = create_bounding_box_shape(2.0, 1.0);
  const FootprintTrajectory footprints = {
    autoware_utils_geometry::to_polygon2d(poses.at(0), shape),
    autoware_utils_geometry::to_polygon2d(poses.at(1), shape),
    autoware_utils_geometry::to_polygon2d(poses.at(2), shape)};
  const TrajectoryData trajectory_data("sample", times, distances, poses, footprints);

  const auto range = trajectory_data.getFootprintsInTimeRange(0.05, 0.15);
  const auto empty_range = trajectory_data.getFootprintsInTimeRange(0.3, 0.2);

  ASSERT_EQ(std::distance(range.begin(), range.end()), 2);
  EXPECT_DOUBLE_EQ(range.begin()->outer().front().x(), footprints.at(0).outer().front().x());
  EXPECT_DOUBLE_EQ(
    std::next(range.begin())->outer().front().x(), footprints.at(1).outer().front().x());
  EXPECT_EQ(std::distance(empty_range.begin(), empty_range.end()), 0);
}

}  // namespace autoware::trajectory_validator::plugin::safety
