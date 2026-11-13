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
#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/node_options.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_control_msgs/msg/hazard_alert.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

namespace autoware::predicted_trajectory_validator
{
namespace
{
constexpr char kNs[] = "/predicted_trajectory_validator";

// A straight road border along y = 2.0.
autoware_map_msgs::msg::LaneletMapBin make_road_border_map()
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Point3d p1(lanelet::utils::getId(), -100.0, 2.0, 0.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 100.0, 2.0, 0.0);
  lanelet::LineString3d boundary(lanelet::utils::getId(), {p1, p2});
  boundary.attributes()[lanelet::AttributeName::Type] = "road_border";
  map->add(boundary);
  return autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
}

Trajectory make_trajectory(const double yaw, const size_t num_points = 5)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  for (size_t i = 0; i < num_points; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i) * 5.0 * std::cos(yaw);
    point.pose.position.y = -2.5 + static_cast<double>(i) * 5.0 * std::sin(yaw);
    point.pose.orientation = tf2::toMsg(quaternion);
    point.longitudinal_velocity_mps = 10.0F;
    trajectory.points.emplace_back(point);
  }
  return trajectory;
}

bool all_sectors_none(const HazardAlert & alert)
{
  for (const auto & sector : alert.sectors) {
    if (sector.risk_level != HazardSector::NONE) {
      return false;
    }
  }
  return true;
}
}  // namespace

class PredictedTrajectoryValidatorNodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override
  {
    if (executor_) {
      executor_->remove_node(helper_);
      executor_->remove_node(node_);
    }
    executor_.reset();
    helper_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  // Vehicle with a large left overhang, the same as the boundary departure checker's test scenario.
  void start()
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_predicted_trajectory_validator") +
         "/config/predicted_trajectory_validator.param.yaml"});
    options.parameter_overrides(
      {rclcpp::Parameter("wheel_radius", 0.383), rclcpp::Parameter("wheel_width", 0.235),
       rclcpp::Parameter("wheel_base", 2.79), rclcpp::Parameter("wheel_tread", 1.64),
       rclcpp::Parameter("front_overhang", 1.0), rclcpp::Parameter("rear_overhang", 1.1),
       rclcpp::Parameter("left_overhang", 2.5), rclcpp::Parameter("right_overhang", 0.128),
       rclcpp::Parameter("vehicle_height", 0.128), rclcpp::Parameter("max_steer_angle", 0.70),
       rclcpp::Parameter("collision_validator.enable_warning", true),
       rclcpp::Parameter("uncrossable_bound_departure_validator.enable_warning", true)});
    node_ = std::make_shared<PredictedTrajectoryValidator>(options);
    node_->on_lanelet_map(
      std::make_shared<autoware_map_msgs::msg::LaneletMapBin>(make_road_border_map()));

    helper_ = std::make_shared<rclcpp::Node>("predicted_trajectory_validator_test_helper");
    pub_pred_ =
      helper_->create_publisher<Trajectory>(std::string(kNs) + "/input/predicted_trajectory", 1);
    pub_kin_ = helper_->create_publisher<Odometry>(std::string(kNs) + "/input/kinematics", 1);
    pub_acc_ = helper_->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      std::string(kNs) + "/input/measured_acceleration", 1);
    pub_mode_ = helper_->create_publisher<OperationModeState>(
      std::string(kNs) + "/input/operational_mode_state", 1);
    sub_status_ = helper_->create_subscription<PredictedTrajectoryValidatorStatus>(
      std::string(kNs) + "/output/validation_status", 10,
      [this](const PredictedTrajectoryValidatorStatus::ConstSharedPtr msg) {
        statuses_.push_back(*msg);
      });
    sub_alert_ = helper_->create_subscription<HazardAlert>(
      std::string(kNs) + "/output/hazard_alert", 10,
      [this](const HazardAlert::ConstSharedPtr msg) { alerts_.push_back(*msg); });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(helper_);
    spin_for(std::chrono::milliseconds(300));
  }

  void spin_for(const std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
  }

  void publish_inputs(
    const Trajectory & trajectory, const double time_s, const bool autonomous,
    const bool with_kinematics = true, const bool with_acceleration = true)
  {
    if (with_kinematics) {
      Odometry kinematics;
      kinematics.header.stamp = rclcpp::Time(static_cast<int64_t>(time_s * 1e9));
      kinematics.header.frame_id = "map";
      kinematics.pose.pose = trajectory.points.front().pose;
      kinematics.twist.twist.linear.x = 10.0;
      pub_kin_->publish(kinematics);
    }
    if (with_acceleration) {
      pub_acc_->publish(geometry_msgs::msg::AccelWithCovarianceStamped{});
    }
    OperationModeState mode;
    mode.mode = autonomous ? OperationModeState::AUTONOMOUS : OperationModeState::STOP;
    mode.is_autoware_control_enabled = autonomous;
    pub_mode_->publish(mode);
    spin_for(std::chrono::milliseconds(50));
  }

  // Publishes the predicted trajectory and waits for the status of that cycle.
  bool trigger(const Trajectory & trajectory, const std::chrono::milliseconds timeout)
  {
    const auto before = statuses_.size();
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (statuses_.size() == before && std::chrono::steady_clock::now() < deadline) {
      pub_pred_->publish(trajectory);
      spin_for(std::chrono::milliseconds(50));
    }
    return statuses_.size() > before;
  }

  // One cycle with the kinematics placed at the front of the given trajectory.
  ValidationResult cycle(
    const Trajectory & trajectory, const double time_s, const bool autonomous,
    const Trajectory & pose_source)
  {
    publish_inputs(pose_source, time_s, autonomous);
    EXPECT_TRUE(trigger(trajectory, std::chrono::seconds(2)));
    return statuses_.empty() ? ValidationResult{}
                             : statuses_.back().uncrossable_bound_departure_result;
  }

  // Safe at t = 0, then the trajectory heading toward the border until the departure is CRITICAL.
  std::vector<ValidationResult> drive_toward_border(const bool autonomous, double & time_s)
  {
    const auto safe = make_trajectory(0.0);
    const auto danger = make_trajectory(0.2);
    std::vector<ValidationResult> results;
    results.push_back(cycle(safe, time_s, autonomous, safe));
    for (int i = 0; i < 6; ++i) {
      time_s += 0.1;
      results.push_back(cycle(danger, time_s, autonomous, danger));
    }
    return results;
  }

  std::shared_ptr<PredictedTrajectoryValidator> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_pred_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_kin_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_mode_;
  rclcpp::Subscription<PredictedTrajectoryValidatorStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<HazardAlert>::SharedPtr sub_alert_;
  std::vector<PredictedTrajectoryValidatorStatus> statuses_;
  std::vector<HazardAlert> alerts_;
};

TEST_F(PredictedTrajectoryValidatorNodeTest, NothingIsValidatedWithoutKinematics)
{
  start();
  const auto trajectory = make_trajectory(0.0);
  publish_inputs(trajectory, 0.0, true, false, true);
  EXPECT_FALSE(trigger(trajectory, std::chrono::seconds(1)));
  EXPECT_TRUE(alerts_.empty());
}

TEST_F(PredictedTrajectoryValidatorNodeTest, NothingIsValidatedWithoutAcceleration)
{
  start();
  const auto trajectory = make_trajectory(0.0);
  publish_inputs(trajectory, 0.0, true, true, false);
  EXPECT_FALSE(trigger(trajectory, std::chrono::seconds(1)));
  EXPECT_TRUE(alerts_.empty());
}

TEST_F(PredictedTrajectoryValidatorNodeTest, ReferenceTrajectoryIsNotRequired)
{
  start();
  EXPECT_EQ(node_->count_subscribers(std::string(kNs) + "/input/reference_trajectory"), 0U);
  const auto trajectory = make_trajectory(0.0);
  publish_inputs(trajectory, 0.0, true);
  EXPECT_TRUE(trigger(trajectory, std::chrono::seconds(2)));
}

TEST_F(PredictedTrajectoryValidatorNodeTest, ShortTrajectoryPublishesNoAlert)
{
  start();
  const auto trajectory = make_trajectory(0.0, 1);
  publish_inputs(trajectory, 0.0, true);
  ASSERT_TRUE(trigger(trajectory, std::chrono::seconds(2)));
  spin_for(std::chrono::milliseconds(100));

  EXPECT_TRUE(alerts_.empty());
  EXPECT_TRUE(statuses_.back().uncrossable_bound_departure_result.is_valid);
}

TEST_F(PredictedTrajectoryValidatorNodeTest, ShortTrajectoryKeepsThePreviousResult)
{
  start();
  double time_s = 0.0;
  const auto results = drive_toward_border(true, time_s);
  ASSERT_FALSE(results.back().is_valid);

  time_s += 0.1;
  const auto danger = make_trajectory(0.2);
  const auto result = cycle(make_trajectory(0.2, 1), time_s, true, danger);
  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.invalid_count, results.back().invalid_count + 1);
}

TEST_F(PredictedTrajectoryValidatorNodeTest, ManualDrivingOverwritesTheResultButNotTheAlert)
{
  start();
  double time_s = 0.0;
  const auto results = drive_toward_border(false, time_s);
  for (const auto & result : results) {
    EXPECT_TRUE(result.is_valid);
    EXPECT_EQ(result.invalid_count, 0);
  }
  spin_for(std::chrono::milliseconds(100));

  ASSERT_FALSE(alerts_.empty());
  EXPECT_FALSE(all_sectors_none(alerts_.back()));
}

TEST_F(PredictedTrajectoryValidatorNodeTest, InvalidCountIsCountedPerResultAndResetWhenValid)
{
  start();
  double time_s = 0.0;
  const auto results = drive_toward_border(true, time_s);
  int64_t expected_count = 0;
  for (const auto & result : results) {
    expected_count = result.is_valid ? 0 : expected_count + 1;
    EXPECT_EQ(result.invalid_count, expected_count);
  }
  ASSERT_GT(expected_count, 1);

  const auto safe = make_trajectory(0.0);
  ValidationResult result;
  for (int i = 0; i < 10 && !result.is_valid; ++i) {
    time_s += 0.1;
    result = cycle(safe, time_s, true, safe);
  }
  EXPECT_TRUE(result.is_valid);
  EXPECT_EQ(result.invalid_count, 0);
}
}  // namespace autoware::predicted_trajectory_validator
