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
#include <autoware_control_validator/msg/control_validator_status.hpp>
#include <autoware_control_validator/msg/hazard_alert.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

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

Odometry make_odometry(double longitudinal_velocity)
{
  Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = longitudinal_velocity;
  return odometry;
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
  hazard::HazardWarningSources hazard_sources;
  validator.validate(
    res, trajectory, odometry, acceleration, nullptr, vehicle_info_, markers, hazard_sources);

  EXPECT_TRUE(res.will_cross_uncrossable_bound);
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
  hazard::HazardWarningSources hazard_sources;
  validator.validate(
    res, trajectory, odometry, acceleration, make_road_border_map(), vehicle_info_, markers,
    hazard_sources);

  EXPECT_TRUE(res.will_cross_uncrossable_bound);
}

TEST_F(UncrossableBoundDepartureValidatorTest, ValidWhenTrajectoryParallelToBoundary)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto map = make_road_border_map();
  const auto trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;

  ControlValidatorStatus res;
  hazard::HazardWarningSources hazard_sources;
  validator.validate(
    res, trajectory, make_odometry(trajectory, 10.0, 0.0), acceleration, map, vehicle_info_,
    markers, hazard_sources);

  EXPECT_TRUE(res.will_cross_uncrossable_bound);
}

TEST_F(UncrossableBoundDepartureValidatorTest, InvalidWhenTrajectoryDepartsBoundary)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto map = make_road_border_map();
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;
  ControlValidatorStatus res;
  hazard::HazardWarningSources hazard_sources;

  // Start safe, then steer toward the boundary and keep violating past the ON-time buffer so the
  // hysteresis promotes the departure to CRITICAL.
  const auto safe_trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  validator.validate(
    res, safe_trajectory, make_odometry(safe_trajectory, 10.0, 0.0), acceleration, map,
    vehicle_info_, markers, hazard_sources);
  EXPECT_TRUE(res.will_cross_uncrossable_bound);

  const auto danger_trajectory = make_trajectory(0.0, -2.5, 10.0, 0.2);
  for (double time_s = 0.1; time_s <= 0.6; time_s += 0.1) {
    validator.validate(
      res, danger_trajectory, make_odometry(danger_trajectory, 10.0, time_s), acceleration, map,
      vehicle_info_, markers, hazard_sources);
  }

  EXPECT_FALSE(res.will_cross_uncrossable_bound);
  EXPECT_FALSE(markers.markers.empty());
}

TEST_F(UncrossableBoundDepartureValidatorTest, CautionDeparturePositionsAreHandedOverInBaseLink)
{
  UncrossableBoundDepartureValidator validator(node_->get_logger(), params_);
  const auto map = make_road_border_map();
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  visualization_msgs::msg::MarkerArray markers;
  ControlValidatorStatus res;

  const auto safe_trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  hazard::HazardWarningSources safe_sources;
  validator.validate(
    res, safe_trajectory, make_odometry(safe_trajectory, 10.0, 0.0), acceleration, map,
    vehicle_info_, markers, safe_sources);
  ASSERT_TRUE(safe_sources.departure.has_value());
  EXPECT_TRUE(safe_sources.departure->empty());
  EXPECT_FALSE(safe_sources.collision.has_value());

  const double yaw = 0.2;
  const auto danger_trajectory = make_trajectory(0.0, -2.5, 10.0, yaw);
  hazard::HazardWarningSources sources;
  for (double time_s = 0.1; time_s <= 0.6; time_s += 0.1) {
    sources = hazard::HazardWarningSources{};
    validator.validate(
      res, danger_trajectory, make_odometry(danger_trajectory, 10.0, time_s), acceleration, map,
      vehicle_info_, markers, sources);
  }

  ASSERT_TRUE(sources.departure.has_value());
  ASSERT_FALSE(sources.departure->empty());
  const auto & ego = danger_trajectory.points.front().pose.position;
  for (const auto & source : *sources.departure) {
    EXPECT_EQ(
      source.risk_level, params_.uncrossable_bound_departure_validator.departure_risk_level);
    ASSERT_EQ(source.shape.outer().size(), 1U);
    const auto & p = source.shape.outer().front();
    EXPECT_NEAR(ego.y + std::sin(yaw) * p.x() + std::cos(yaw) * p.y(), 2.0, 1e-6);
  }
}

TEST_F(UncrossableBoundDepartureValidatorTest, DepartureIsNotHandedOverWhenDisabledOrSkipped)
{
  const auto map = make_road_border_map();
  const auto trajectory = make_trajectory(0.0, -2.5, 10.0, 0.0);
  const auto is_handed_over = [&](
                                const ::control_validator::Params & params,
                                const lanelet::LaneletMapPtr & lanelet_map,
                                const Trajectory & predicted_trajectory) {
    UncrossableBoundDepartureValidator validator(node_->get_logger(), params);
    geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
    visualization_msgs::msg::MarkerArray markers;
    ControlValidatorStatus res;
    hazard::HazardWarningSources sources;
    validator.validate(
      res, predicted_trajectory, make_odometry(trajectory, 10.0, 0.0), acceleration, lanelet_map,
      vehicle_info_, markers, sources);
    return sources.departure.has_value();
  };

  auto no_road_border = params_;
  no_road_border.uncrossable_bound_departure_validator.enable_road_border = false;
  auto level_zero = params_;
  level_zero.uncrossable_bound_departure_validator.departure_risk_level = 0;
  auto level_four = params_;
  level_four.uncrossable_bound_departure_validator.departure_risk_level = 4;
  auto disabled = params_;
  disabled.uncrossable_bound_departure_validator.enable = false;

  EXPECT_TRUE(is_handed_over(params_, map, trajectory));
  EXPECT_FALSE(is_handed_over(no_road_border, map, trajectory));
  EXPECT_FALSE(is_handed_over(level_zero, map, trajectory));
  EXPECT_FALSE(is_handed_over(level_four, map, trajectory));
  EXPECT_FALSE(is_handed_over(disabled, map, trajectory));
  EXPECT_FALSE(is_handed_over(params_, nullptr, trajectory));
  EXPECT_FALSE(is_handed_over(params_, map, Trajectory{}));
}

class VelocityValidatorTest
: public ::testing::TestWithParam<std::tuple<double, double, bool, bool>>
{
public:
  void validate(
    ControlValidatorStatus & res, const Trajectory & reference_trajectory,
    const Odometry & kinematics)
  {
    velocity_validator_->validate(res, reference_trajectory, kinematics);
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
    velocity_validator_ = std::make_shared<VelocityValidator>(param_listener.get_params());
  }
  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<VelocityValidator> velocity_validator_;
};

TEST_P(VelocityValidatorTest, test_velocity_validation)
{
  const auto [target_vel, vehicle_vel, expected_is_rolling_back, expected_is_over_velocity] =
    GetParam();

  const auto reference_trajectory = make_linear_trajectory(
    make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, target_vel);
  const auto kinematics = make_odometry(vehicle_vel);

  ControlValidatorStatus res;
  res.is_rolling_back = false;
  res.is_over_velocity = false;
  validate(res, reference_trajectory, kinematics);

  EXPECT_EQ(res.is_rolling_back, expected_is_rolling_back);
  EXPECT_EQ(res.is_over_velocity, expected_is_over_velocity);
}

INSTANTIATE_TEST_SUITE_P(
  VelocityValidatorTests, VelocityValidatorTest,
  ::testing::Values(
    // 1. non-zero target velocity with no rollback (same sign)
    std::make_tuple(1.0, 1.0, false, false),
    // 2. zero target velocity with no rollback (forward motion while stopped target)
    std::make_tuple(0.0, 1.0, false, false),
    // 3. non-zero target velocity with rollback (opposite signs)
    std::make_tuple(1.0, -1.0, true, false),
    // 4. zero target velocity with rollback (backward motion while stopped target)
    std::make_tuple(0.0, -1.0, true, false),
    // over velocity: vehicle much faster than target
    std::make_tuple(1.0, 5.0, false, true),
    // reverse driving with matching signs is not rollback
    std::make_tuple(-1.0, -1.0, false, false)));

// ==============================================================================
// Hazard warning: node level
// ==============================================================================

struct HazardCycleResult
{
  std::optional<ControlValidatorStatus> status;
  std::optional<HazardAlert> alert;
  std::optional<double> hazard_processing_ms;
};

class HazardWarningNodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  static std::shared_ptr<ControlValidator> make_node(
    const bool hazard_enable, const bool thresholds_decided)
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_control_validator") +
         "/config/control_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});
    std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("hazard_warning.enable", hazard_enable),
      rclcpp::Parameter("hazard_warning.debug.publish_markers", false)};
    if (thresholds_decided) {
      overrides.emplace_back(
        "hazard_warning.braking.reaction_times", std::vector<double>{2.0, 1.5, 1.0});
    } else {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      overrides.emplace_back(
        "hazard_warning.braking.reaction_times", std::vector<double>{nan, nan, nan});
    }
    options.parameter_overrides(overrides);
    return std::make_shared<ControlValidator>(options);
  }

  static Trajectory make_timed_trajectory(
    const rclcpp::Time & stamp, const double velocity, const double duration)
  {
    Trajectory trajectory;
    trajectory.header.stamp = stamp;
    trajectory.header.frame_id = "map";
    const auto n = static_cast<size_t>(std::round(duration / 0.1)) + 1;
    for (size_t i = 0; i < n; ++i) {
      const double t = static_cast<double>(i) * 0.1;
      TrajectoryPoint point;
      point.pose.position.x = velocity * t;
      point.pose.orientation.w = 1.0;
      point.longitudinal_velocity_mps = static_cast<float>(velocity);
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      trajectory.points.push_back(point);
    }
    return trajectory;
  }

  static sensor_msgs::msg::PointCloud2 make_cloud(const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "base_link";
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    // 25 points at x = 40 m: inside the ROI, far enough not to raise with a 3 s threshold.
    modifier.resize(25);
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 5; ++j) {
        *it_x = 40.0F + 0.1F * i;
        *it_y = -0.2F + 0.1F * j;
        *it_z = 1.0F;
        ++it_x;
        ++it_y;
        ++it_z;
      }
    }
    cloud.height = 1;
    cloud.width = 25;
    return cloud;
  }

  // Publishes one set of inputs (a stationary car 15 m ahead, ego at 10 m/s), triggers one
  // control_cmd callback and collects the outputs.
  static HazardCycleResult run_cycle(
    const std::shared_ptr<ControlValidator> & node, const bool autonomous)
  {
    using autoware_adapi_v1_msgs::msg::OperationModeState;
    using autoware_perception_msgs::msg::ObjectClassification;
    using autoware_perception_msgs::msg::PredictedObject;
    using autoware_perception_msgs::msg::PredictedObjects;
    using autoware_perception_msgs::msg::Shape;

    auto helper = std::make_shared<rclcpp::Node>("hazard_test_helper");
    const std::string ns = "/control_validator";
    auto pub_cmd = helper->create_publisher<Control>(ns + "/input/control_cmd", 1);
    auto pub_kin = helper->create_publisher<Odometry>(ns + "/input/kinematics", 1);
    auto pub_ref = helper->create_publisher<Trajectory>(ns + "/input/reference_trajectory", 1);
    auto pub_pred = helper->create_publisher<Trajectory>(ns + "/input/predicted_trajectory", 1);
    auto pub_acc = helper->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      ns + "/input/measured_acceleration", 1);
    auto pub_mode =
      helper->create_publisher<OperationModeState>(ns + "/input/operational_mode_state", 1);
    auto pub_objects = helper->create_publisher<PredictedObjects>(ns + "/input/objects", 1);
    auto pub_cloud =
      helper->create_publisher<sensor_msgs::msg::PointCloud2>(ns + "/input/pointcloud", 1);

    HazardCycleResult result;
    auto sub_status = helper->create_subscription<ControlValidatorStatus>(
      ns + "/output/validation_status", 1,
      [&](const ControlValidatorStatus::ConstSharedPtr msg) { result.status = *msg; });
    auto sub_alert = helper->create_subscription<HazardAlert>(
      ns + "/output/hazard_alert", 1,
      [&](const HazardAlert::ConstSharedPtr msg) { result.alert = *msg; });
    auto sub_time = helper->create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
      ns + "/debug/hazard/processing_time_ms", 1,
      [&](const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg) {
        result.hazard_processing_ms = msg->data;
      });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(helper);

    const auto now = node->now();
    const auto trajectory = make_timed_trajectory(now, 10.0, 5.0);
    Odometry kinematics;
    kinematics.header.stamp = now;
    kinematics.header.frame_id = "map";
    kinematics.pose.pose.orientation.w = 1.0;
    kinematics.twist.twist.linear.x = 10.0;
    geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
    acceleration.header.stamp = now;
    OperationModeState mode;
    mode.stamp = now;
    mode.mode = autonomous ? OperationModeState::AUTONOMOUS : OperationModeState::STOP;
    mode.is_autoware_control_enabled = autonomous;
    PredictedObjects objects;
    objects.header.stamp = now;
    objects.header.frame_id = "map";
    PredictedObject car;
    ObjectClassification classification;
    classification.label = ObjectClassification::CAR;
    classification.probability = 1.0;
    car.classification.push_back(classification);
    car.kinematics.initial_pose_with_covariance.pose.position.x = 15.0;
    car.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    car.shape.type = Shape::BOUNDING_BOX;
    car.shape.dimensions.x = 1.0;
    car.shape.dimensions.y = 1.0;
    car.shape.dimensions.z = 1.5;
    objects.objects.push_back(car);

    // Inputs first, then wait until the polling subscribers can take them.
    for (int i = 0; i < 20; ++i) {
      pub_kin->publish(kinematics);
      pub_ref->publish(trajectory);
      pub_pred->publish(trajectory);
      pub_acc->publish(acceleration);
      pub_mode->publish(mode);
      pub_objects->publish(objects);
      pub_cloud->publish(make_cloud(now));
      executor.spin_some(std::chrono::milliseconds(50));
    }

    Control control_cmd;
    control_cmd.stamp = node->now();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!result.status && std::chrono::steady_clock::now() < deadline) {
      pub_cmd->publish(control_cmd);
      executor.spin_some(std::chrono::milliseconds(100));
    }
    // Give the alert of the same cycle time to arrive.
    for (int i = 0; i < 5 && !result.alert; ++i) {
      executor.spin_some(std::chrono::milliseconds(50));
    }
    executor.remove_node(helper);
    executor.remove_node(node);
    return result;
  }
};

TEST_F(HazardWarningNodeTest, ExistingStatusIsUnchangedByTheHazardWarning)
{
  // 1-line summary: ControlValidatorStatus is identical with the warning enabled and disabled.
  HazardCycleResult with_hazard;
  {
    auto node = make_node(true, true);
    with_hazard = run_cycle(node, true);
  }
  HazardCycleResult without_hazard;
  {
    auto node = make_node(false, true);
    without_hazard = run_cycle(node, true);
  }
  ASSERT_TRUE(with_hazard.status.has_value());
  ASSERT_TRUE(without_hazard.status.has_value());
  EXPECT_TRUE(with_hazard.alert.has_value());
  EXPECT_FALSE(without_hazard.alert.has_value());

  const auto & a = *with_hazard.status;
  const auto & b = *without_hazard.status;
  EXPECT_EQ(a.is_valid_max_distance_deviation, b.is_valid_max_distance_deviation);
  EXPECT_EQ(a.is_valid_acc, b.is_valid_acc);
  EXPECT_EQ(a.is_rolling_back, b.is_rolling_back);
  EXPECT_EQ(a.is_over_velocity, b.is_over_velocity);
  EXPECT_EQ(a.is_valid_lateral_jerk, b.is_valid_lateral_jerk);
  EXPECT_EQ(a.has_overrun_stop_point, b.has_overrun_stop_point);
  EXPECT_EQ(a.will_overrun_stop_point, b.will_overrun_stop_point);
  EXPECT_EQ(a.is_valid_yaw, b.is_valid_yaw);
  EXPECT_EQ(a.is_warn_yaw, b.is_warn_yaw);
  EXPECT_EQ(a.will_cross_uncrossable_bound, b.will_cross_uncrossable_bound);
  EXPECT_EQ(a.invalid_count, b.invalid_count);

  if (with_hazard.hazard_processing_ms) {
    std::cout << "[ hazard   ] processing time: " << *with_hazard.hazard_processing_ms << " ms"
              << std::endl;
  }
}

TEST_F(HazardWarningNodeTest, AlertIsPublishedWhileManualDrivingSuppressesTheDiagnostics)
{
  // 1-line summary: A stationary car 15 m ahead at 10 m/s (TTC ~1.1 s) raises FRONT even when
  // validation_filtering() overrides the diagnostics.
  auto node = make_node(true, true);
  const auto result = run_cycle(node, false);

  ASSERT_TRUE(result.status.has_value());
  EXPECT_TRUE(result.status->is_valid_max_distance_deviation);
  EXPECT_FALSE(result.status->will_cross_uncrossable_bound);

  ASSERT_TRUE(result.alert.has_value());
  EXPECT_TRUE(result.alert->sectors[HazardAlert::FRONT].has_object);
  EXPECT_EQ(result.alert->sectors[HazardAlert::FRONT].risk_level, 3);
  EXPECT_FALSE(result.alert->sectors[HazardAlert::REAR].has_object);
  EXPECT_EQ(result.alert->sectors[HazardAlert::REAR].risk_level, 0);
}

TEST_F(HazardWarningNodeTest, UndecidedThresholdsDisableTheWarningButKeepTheValidator)
{
  // 1-line summary: With the shipped .nan thresholds the node starts, validates, and publishes
  // no alert.
  auto node = make_node(true, false);
  const auto result = run_cycle(node, true);

  ASSERT_TRUE(result.status.has_value());
  EXPECT_FALSE(result.alert.has_value());
}

}  // namespace autoware::control_validator
