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

#include "autoware/predicted_trajectory_validator/predicted_trajectory_validator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_predicted_trajectory_validator/msg/predicted_trajectory_validator_status.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_control_msgs/msg/hazard_alert.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

namespace autoware::predicted_trajectory_validator
{
class HazardDepartureValidatorTest : public ::testing::Test
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

TEST_F(HazardDepartureValidatorTest, DepartureIsNotHandedOverWhenDisabledOrSkipped)
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

TEST_F(HazardDepartureValidatorTest, WarningIsRaisedWhileTheMrmCheckIsDisabled)
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

struct HazardCycleResult
{
  std::optional<PredictedTrajectoryValidatorStatus> status;
  std::optional<HazardAlert> alert;
  std::optional<double> hazard_processing_ms;
};

class HazardWarningNodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  static std::shared_ptr<PredictedTrajectoryValidator> make_node(
    const bool collision_warning, const bool departure_warning = true,
    const std::vector<rclcpp::Parameter> & extra_overrides = {})
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_predicted_trajectory_validator") +
         "/config/predicted_trajectory_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});
    std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("collision_validator.enable_warning", collision_warning),
      rclcpp::Parameter("uncrossable_bound_departure_validator.enable_warning", departure_warning)};
    overrides.insert(overrides.end(), extra_overrides.begin(), extra_overrides.end());
    options.parameter_overrides(overrides);
    return std::make_shared<PredictedTrajectoryValidator>(options);
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
    // 25 points at x = 40 m: inside the ROI, beyond where braking after the longest reaction
    // time stops.
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
  // predicted_trajectory callback and collects the outputs.
  static HazardCycleResult run_cycle(
    const std::shared_ptr<PredictedTrajectoryValidator> & node, const bool autonomous)
  {
    using autoware_adapi_v1_msgs::msg::OperationModeState;
    using autoware_perception_msgs::msg::ObjectClassification;
    using autoware_perception_msgs::msg::PredictedObject;
    using autoware_perception_msgs::msg::PredictedObjects;
    using autoware_perception_msgs::msg::Shape;

    auto helper = std::make_shared<rclcpp::Node>("hazard_test_helper");
    const std::string ns = "/predicted_trajectory_validator";
    auto pub_kin = helper->create_publisher<Odometry>(ns + "/input/kinematics", 1);
    auto pub_pred = helper->create_publisher<Trajectory>(ns + "/input/predicted_trajectory", 1);
    auto pub_acc = helper->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      ns + "/input/measured_acceleration", 1);
    auto pub_mode =
      helper->create_publisher<OperationModeState>(ns + "/input/operational_mode_state", 1);
    auto pub_objects = helper->create_publisher<PredictedObjects>(ns + "/input/objects", 1);
    auto pub_cloud =
      helper->create_publisher<sensor_msgs::msg::PointCloud2>(ns + "/input/pointcloud", 1);

    HazardCycleResult result;
    auto sub_status = helper->create_subscription<PredictedTrajectoryValidatorStatus>(
      ns + "/output/validation_status", 1,
      [&](const PredictedTrajectoryValidatorStatus::ConstSharedPtr msg) { result.status = *msg; });
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
      pub_acc->publish(acceleration);
      pub_mode->publish(mode);
      pub_objects->publish(objects);
      pub_cloud->publish(make_cloud(now));
      executor.spin_some(std::chrono::milliseconds(50));
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!result.status && std::chrono::steady_clock::now() < deadline) {
      pub_pred->publish(trajectory);
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
  // PredictedTrajectoryValidatorStatus is identical with the warning enabled and disabled.
  HazardCycleResult with_hazard;
  {
    auto node = make_node(true);
    with_hazard = run_cycle(node, true);
  }
  HazardCycleResult without_hazard;
  {
    auto node = make_node(false, false);
    without_hazard = run_cycle(node, true);
  }
  ASSERT_TRUE(with_hazard.status.has_value());
  ASSERT_TRUE(without_hazard.status.has_value());
  EXPECT_TRUE(with_hazard.alert.has_value());
  EXPECT_FALSE(without_hazard.alert.has_value());

  const auto & a = *with_hazard.status;
  const auto & b = *without_hazard.status;
  EXPECT_EQ(
    a.uncrossable_bound_departure_result.is_valid, b.uncrossable_bound_departure_result.is_valid);
  EXPECT_EQ(
    a.uncrossable_bound_departure_result.invalid_count,
    b.uncrossable_bound_departure_result.invalid_count);

  if (with_hazard.hazard_processing_ms) {
    std::cout << "[ hazard   ] processing time: " << *with_hazard.hazard_processing_ms << " ms"
              << std::endl;
  }
}

TEST_F(HazardWarningNodeTest, AlertIsPublishedWhileManualDrivingSuppressesTheDiagnostics)
{
  // A stationary car 15 m ahead at 10 m/s, not avoidable even with the shortest reaction time,
  // raises FRONT even when the manual driving overrides the diagnostics.
  auto node = make_node(true);
  const auto result = run_cycle(node, false);

  ASSERT_TRUE(result.status.has_value());
  EXPECT_TRUE(result.status->uncrossable_bound_departure_result.is_valid);

  ASSERT_TRUE(result.alert.has_value());
  EXPECT_EQ(result.alert->sectors[0].risk_level, HazardSector::HIGH);
  EXPECT_EQ(result.alert->sectors[4].risk_level, HazardSector::NONE);
}

TEST_F(HazardWarningNodeTest, DisabledWarningKeepsTheValidator)
{
  // With both enable_warning false the node starts, validates, and
  // publishes no alert.
  auto node = make_node(false, false);
  const auto result = run_cycle(node, true);

  ASSERT_TRUE(result.status.has_value());
  EXPECT_FALSE(result.alert.has_value());
}

TEST_F(HazardWarningNodeTest, OutOfRangeParametersAreRejected)
{
  // Values outside the schema validation fail the parameter loading, and a runtime change to
  // such a value is rejected and keeps the previous value.
  const std::vector<rclcpp::Parameter> invalid = {
    rclcpp::Parameter("collision_validator.braking.reaction_times", std::vector<double>{2.0, 1.0}),
    rclcpp::Parameter(
      "collision_validator.braking.reaction_times", std::vector<double>{2.0, 1.5, -1.0}),
    rclcpp::Parameter("collision_validator.braking.deceleration", 0.0),
    rclcpp::Parameter("collision_validator.collision_cos_threshold", 1.5),
    rclcpp::Parameter(
      "collision_validator.enabled_object_classes", std::vector<std::string>{"car", "spaceship"}),
    rclcpp::Parameter("collision_validator.proximity_distance", 0.0),
    rclcpp::Parameter("uncrossable_bound_departure_validator.departure_risk_level", 0),
    rclcpp::Parameter("uncrossable_bound_departure_validator.departure_risk_level", 4)};
  for (const auto & parameter : invalid) {
    EXPECT_ANY_THROW(make_node(true, true, {parameter})) << parameter.get_name();
  }

  auto node = make_node(true);
  const auto result = node->set_parameter(
    rclcpp::Parameter("uncrossable_bound_departure_validator.departure_risk_level", 4));
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(
    node->get_parameter("uncrossable_bound_departure_validator.departure_risk_level").as_int(), 2);
}
}  // namespace autoware::predicted_trajectory_validator
