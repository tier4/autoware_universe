// Copyright 2024 TIER IV, Inc.
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

#include "autoware/predicted_trajectory_validator/predicted_trajectory_validator.hpp"
#include "autoware/predicted_trajectory_validator/uncrossable_bound_departure_validator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cmath>
#include <memory>

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

namespace autoware::predicted_trajectory_validator
{
class UncrossableBoundDepartureValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_predicted_trajectory_validator") +
         "/config/predicted_trajectory_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<PredictedTrajectoryValidator>(options);
    ::predicted_trajectory_validator::ParamListener param_listener(
      node_->get_node_parameters_interface());
    params_ = param_listener.get_params();

    // Vehicle with a large left overhang so the left edge sits well left of the vehicle center,
    // matching the boundary departure checker's own test scenario.
    // Arguments: wheel_radius, wheel_width, wheel_base, wheel_tread, front_overhang, rear_overhang,
    //            left_overhang, right_overhang, vehicle_height, max_steer_angle.
    vehicle_info_ = autoware::vehicle_info_utils::createVehicleInfo(
      0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 2.5, 0.128, 0.128, 0.70);
  }
  void TearDown() override { rclcpp::shutdown(); }

  // A straight road border along y = 2.0.
  static lanelet::LaneletMapPtr make_road_border_map()
  {
    auto map = std::make_shared<lanelet::LaneletMap>();
    lanelet::Point3d p1(lanelet::utils::getId(), -100.0, 2.0, 0.0);
    lanelet::Point3d p2(lanelet::utils::getId(), 100.0, 2.0, 0.0);
    lanelet::LineString3d boundary(lanelet::utils::getId(), {p1, p2});
    boundary.attributes()[lanelet::AttributeName::Type] = "road_border";
    map->add(boundary);
    return map;
  }

  static Trajectory make_trajectory(double start_x, double start_y, double velocity, double yaw)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);

    Trajectory trajectory;
    for (int i = 0; i < 5; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = start_x + i * 5.0 * std::cos(yaw);
      point.pose.position.y = start_y + i * 5.0 * std::sin(yaw);
      point.pose.orientation = tf2::toMsg(quaternion);
      point.longitudinal_velocity_mps = static_cast<float>(velocity);
      trajectory.points.emplace_back(point);
    }
    return trajectory;
  }

  static nav_msgs::msg::Odometry make_odometry(
    const Trajectory & trajectory, double velocity, double time_s)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = rclcpp::Time(static_cast<int64_t>(time_s * 1e9));
    if (!trajectory.points.empty()) {
      odometry.pose.pose = trajectory.points.front().pose;
    }
    odometry.twist.twist.linear.x = velocity;
    return odometry;
  }

  std::shared_ptr<PredictedTrajectoryValidator> node_;
  ::predicted_trajectory_validator::Params params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
};

TEST_F(UncrossableBoundDepartureValidatorTest, ValidWhenMapIsUnavailable)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  const auto odometry = make_odometry(trajectory, 10.0, 0.0);
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;

  ValidationResult res;
  HazardWarningSources hazard_sources;
  validator.validate(
    res, trajectory, odometry, acceleration, nullptr, vehicle_info_, markers, hazard_sources);

  EXPECT_FALSE(res.is_valid);
}

TEST_F(UncrossableBoundDepartureValidatorTest, ValidWhenTrajectoryTooShort)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  Trajectory trajectory;
  trajectory.points.emplace_back(TrajectoryPoint{});
  const auto odometry = make_odometry(trajectory, 10.0, 0.0);
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;

  for (const bool initial : {true, false}) {
    ValidationResult res;
    res.is_valid = initial;
    HazardWarningSources hazard_sources;
    validator.validate(
      res, trajectory, odometry, acceleration, make_road_border_map(), vehicle_info_, markers,
      hazard_sources);

    EXPECT_EQ(res.is_valid, initial);
  }
}

TEST_F(UncrossableBoundDepartureValidatorTest, ValidWhenTrajectoryParallelToBoundary)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto map = make_road_border_map();
  const auto trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;

  ValidationResult res;
  HazardWarningSources hazard_sources;
  validator.validate(
    res, trajectory, make_odometry(trajectory, 10.0, 0.0), acceleration, map, vehicle_info_,
    markers, hazard_sources);

  EXPECT_FALSE(res.is_valid);
}

TEST_F(UncrossableBoundDepartureValidatorTest, InvalidWhenTrajectoryDepartsBoundary)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto map = make_road_border_map();
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;
  ValidationResult res;
  HazardWarningSources hazard_sources;

  // Start safe, then steer toward the boundary and keep violating past the ON-time buffer so the
  // hysteresis promotes the departure to CRITICAL.
  const auto safe_trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  validator.validate(
    res, safe_trajectory, make_odometry(safe_trajectory, 10.0, 0.0), acceleration, map,
    vehicle_info_, markers, hazard_sources);
  EXPECT_FALSE(res.is_valid);

  const auto danger_trajectory = make_trajectory(0.0, -2.5, 10.0, 0.2);
  for (double time_s = 0.1; time_s <= 0.6; time_s += 0.1) {
    validator.validate(
      res, danger_trajectory, make_odometry(danger_trajectory, 10.0, time_s), acceleration, map,
      vehicle_info_, markers, hazard_sources);
  }

  EXPECT_TRUE(res.is_valid);
  EXPECT_FALSE(markers.markers.empty());
}

TEST_F(
  UncrossableBoundDepartureValidatorTest, DepartureOnlyWithinTheWarningGateRaisesOnlyTheWarning)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto map = make_road_border_map();
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;

  const double velocity = 2.0;
  auto trajectory = make_trajectory(0.0, -2.5, velocity, 0.2);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points[i].time_from_start =
      rclcpp::Duration::from_seconds(static_cast<double>(i) * 5.0 / velocity);
  }

  ValidationResult res;
  HazardWarningSources sources;
  for (double time_s = 0.1; time_s <= 0.6; time_s += 0.1) {
    res = ValidationResult{};
    sources = HazardWarningSources{};
    validator.validate(
      res, trajectory, make_odometry(trajectory, velocity, time_s), acceleration, map,
      vehicle_info_, markers, sources);
  }

  EXPECT_TRUE(res.is_valid);
  ASSERT_TRUE(sources.departure.has_value());
  EXPECT_FALSE(sources.departure->empty());
}

TEST_F(UncrossableBoundDepartureValidatorTest, DepartureIsNotHandedOverWhenDisabledOrSkipped)
{
  const auto map = make_road_border_map();
  const auto trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  const auto is_handed_over = [&](
                                const ::predicted_trajectory_validator::Params & params,
                                const lanelet::LaneletMapPtr & lanelet_map,
                                const Trajectory & predicted_trajectory) {
    UncrossableBoundDepartureValidator validator(node_->get_logger(), params);
    geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
    visualization_msgs::msg::MarkerArray markers;
    ValidationResult res;
    HazardWarningSources sources;
    validator.validate(
      res, predicted_trajectory, make_odometry(trajectory, 10.0, 0.0), acceleration, lanelet_map,
      vehicle_info_, markers, sources);
    return sources.departure.has_value();
  };

  auto no_warning = params_;
  no_warning.uncrossable_bound_departure_validator.enable_warning = false;

  EXPECT_TRUE(is_handed_over(params_, map, trajectory));
  EXPECT_FALSE(is_handed_over(no_warning, map, trajectory));
  EXPECT_FALSE(is_handed_over(params_, nullptr, trajectory));
  EXPECT_FALSE(is_handed_over(params_, map, Trajectory{}));
}

TEST_F(UncrossableBoundDepartureValidatorTest, WarningIsRaisedWhileTheMrmCheckIsDisabled)
{
  auto params = params_;
  params.uncrossable_bound_departure_validator.enable = false;
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params);
  const auto map = make_road_border_map();
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;

  const auto trajectory = make_trajectory(0.0, -2.5, 10.0, 0.2);
  ValidationResult res;
  HazardWarningSources sources;
  for (double time_s = 0.1; time_s <= 0.6; time_s += 0.1) {
    res = ValidationResult{};
    sources = HazardWarningSources{};
    validator.validate(
      res, trajectory, make_odometry(trajectory, 10.0, time_s), acceleration, map, vehicle_info_,
      markers, sources);
  }

  EXPECT_TRUE(res.is_valid);
  ASSERT_TRUE(sources.departure.has_value());
  EXPECT_FALSE(sources.departure->empty());
}
}  // namespace autoware::predicted_trajectory_validator
