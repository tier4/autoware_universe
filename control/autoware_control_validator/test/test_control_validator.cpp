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

#include "autoware/control_validator/control_validator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cmath>
#include <memory>
#include <tuple>

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

Trajectory make_linear_trajectory(
  const TrajectoryPoint & start, const TrajectoryPoint & end, size_t num_points, double velocity)
{
  auto create_quaternion = [](double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  };

  double yaw = std::atan2(
    end.pose.position.y - start.pose.position.y, end.pose.position.x - start.pose.position.x);
  yaw += (velocity < 0) ? M_PI : 0;

  Trajectory trajectory;
  trajectory.points.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    double ratio = static_cast<double>(i) / static_cast<double>(num_points - 1);

    TrajectoryPoint point;
    point.pose.position.x =
      start.pose.position.x + ratio * (end.pose.position.x - start.pose.position.x);
    point.pose.position.y =
      start.pose.position.y + ratio * (end.pose.position.y - start.pose.position.y);
    point.pose.orientation = create_quaternion(yaw);
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.lateral_velocity_mps = 0.0;

    trajectory.points.emplace_back(point);
  }

  return trajectory;
}

TrajectoryPoint make_trajectory_point(double x, double y)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  return point;
}

namespace autoware::control_validator
{
class TrajectoryValidatorTest
: public ::testing::TestWithParam<std::tuple<Trajectory, Trajectory, double, bool>>
{
public:
  void validate(
    ControlValidatorStatus & res, const Trajectory & predicted_trajectory,
    const Trajectory & reference_trajectory)
  {
    return trajectory_validator_->validate(res, predicted_trajectory, reference_trajectory);
  }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_control_validator") +
         "/config/control_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<ControlValidator>(options);
    ::control_validator::ParamListener param_listener(node_->get_node_parameters_interface());
    trajectory_validator_ = std::make_shared<TrajectoryValidator>(param_listener.get_params());
  }
  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<TrajectoryValidator> trajectory_validator_;
};

TEST_P(TrajectoryValidatorTest, test_calc_lateral_deviation_status)
{
  auto [reference_trajectory, predicted_trajectory, expected_deviation, expected_condition] =
    GetParam();
  ControlValidatorStatus res;
  validate(res, predicted_trajectory, reference_trajectory);

  EXPECT_EQ(res.is_valid_max_distance_deviation, expected_condition);
  EXPECT_NEAR(res.max_distance_deviation, expected_deviation, 1e-5);
}

INSTANTIATE_TEST_SUITE_P(
  TrajectoryDeviationTests, TrajectoryValidatorTest,
  ::testing::Values(

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0.99), 11, 1.0),
      0.99, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 1.0), 11, 1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 1.01), 11, 1.0),
      1.01, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, -1.0),
      make_linear_trajectory(
        make_trajectory_point(0, 0), make_trajectory_point(10, 0.99), 11, -1.0),
      0.99, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, -1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 1.0), 11, -1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, -1.0),
      make_linear_trajectory(
        make_trajectory_point(0, 0), make_trajectory_point(10, 1.01), 11, -1.0),
      1.01, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(11, 0), make_trajectory_point(20, 0.0), 11, 1.0),
      0.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(11, 0), make_trajectory_point(20, 0.0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      0.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(1, 0), make_trajectory_point(10, 1.0), 11, 1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(-1, 0), make_trajectory_point(10, 1.0), 11, 1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(20, 2.0), 21, 1.0),
      1.0, true))

);

class AccelerationValidatorTest : public ::testing::TestWithParam<std::tuple<bool, double, double>>
{
public:
  bool is_in_error_range() { return acceleration_validator_->is_in_error_range(); }
  void set_desired(double x) { acceleration_validator_->desired_acc_lpf.reset(x); }
  void set_measured(double x) { acceleration_validator_->measured_acc_lpf.reset(x); }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_control_validator") +
         "/config/control_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<ControlValidator>(options);
    ::control_validator::ParamListener param_listener(node_->get_node_parameters_interface());
    acceleration_validator_ = std::make_shared<AccelerationValidator>(param_listener.get_params());
  }
  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<AccelerationValidator> acceleration_validator_;
};

TEST_P(AccelerationValidatorTest, test_is_in_error_range)
{
  auto [expected, des, mes] = GetParam();
  set_desired(des);
  set_measured(mes);

  ASSERT_EQ(expected, is_in_error_range());
};

INSTANTIATE_TEST_SUITE_P(
  AccelerationValidatorTests, AccelerationValidatorTest,
  ::testing::Values(
    std::make_tuple(true, 0.0, 0.0), std::make_tuple(false, 0.0, 5.0),
    std::make_tuple(false, 0.0, -5.0), std::make_tuple(true, 1.0, 1.0),
    std::make_tuple(false, 1.0, 5.0), std::make_tuple(false, 1.0, -5.0),
    std::make_tuple(true, -1.0, -1.0), std::make_tuple(false, -1.0, -5.0),
    std::make_tuple(false, -1.0, 5.0)));

class UncrossableBoundDepartureValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_control_validator") +
         "/config/control_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<ControlValidator>(options);
    ::control_validator::ParamListener param_listener(node_->get_node_parameters_interface());
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

  std::shared_ptr<ControlValidator> node_;
  ::control_validator::Params params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
};

TEST_F(UncrossableBoundDepartureValidatorTest, ValidWhenMapIsUnavailable)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  const auto odometry = make_odometry(trajectory, 10.0, 0.0);
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;

  ControlValidatorStatus res;
  validator.validate(res, trajectory, odometry, acceleration, nullptr, vehicle_info_, markers);

  EXPECT_TRUE(res.is_valid_boundary_departure);
}

TEST_F(UncrossableBoundDepartureValidatorTest, ValidWhenTrajectoryTooShort)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  Trajectory trajectory;
  trajectory.points.emplace_back(TrajectoryPoint{});
  const auto odometry = make_odometry(trajectory, 10.0, 0.0);
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;

  ControlValidatorStatus res;
  validator.validate(
    res, trajectory, odometry, acceleration, make_road_border_map(), vehicle_info_, markers);

  EXPECT_TRUE(res.is_valid_boundary_departure);
}

TEST_F(UncrossableBoundDepartureValidatorTest, ValidWhenTrajectoryParallelToBoundary)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto map = make_road_border_map();
  const auto trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;

  ControlValidatorStatus res;
  validator.validate(
    res, trajectory, make_odometry(trajectory, 10.0, 0.0), acceleration, map, vehicle_info_,
    markers);

  EXPECT_TRUE(res.is_valid_boundary_departure);
}

TEST_F(UncrossableBoundDepartureValidatorTest, InvalidWhenTrajectoryDepartsBoundary)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto map = make_road_border_map();
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;
  ControlValidatorStatus res;

  // Start safe, then steer toward the boundary and keep violating past the ON-time buffer so the
  // hysteresis promotes the departure to CRITICAL.
  const auto safe_trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  validator.validate(
    res, safe_trajectory, make_odometry(safe_trajectory, 10.0, 0.0), acceleration, map,
    vehicle_info_, markers);
  EXPECT_TRUE(res.is_valid_boundary_departure);

  const auto danger_trajectory = make_trajectory(0.0, -2.5, 10.0, 0.2);
  for (double time_s = 0.1; time_s <= 0.6; time_s += 0.1) {
    validator.validate(
      res, danger_trajectory, make_odometry(danger_trajectory, 10.0, time_s), acceleration, map,
      vehicle_info_, markers);
  }

  EXPECT_FALSE(res.is_valid_boundary_departure);
  EXPECT_FALSE(markers.markers.empty());
}

}  // namespace autoware::control_validator
