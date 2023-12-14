// Copyright 2023 Tier IV, Inc.
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

#include "redundant_autoware_state_checker.hpp"

namespace redundant_autoware_state_checker
{

RedundantAutowareStateChecker::RedundantAutowareStateChecker()
: Node("RedundantAutowareStateChecker")
{
  using std::placeholders::_1;

  // Params
  states_equality_timeout = this->declare_parameter("states_equality_timeout", 1.0);  // 1.0s
  update_rate_hz = this->declare_parameter<double>("update_rate_hz", 10.0);           // 10hz
  pose_distance_threshold =
    this->declare_parameter<double>("pose_distance_threshold", 1.0);  // 1.0m

  // Publishers
  pub_diagnostic = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics/supervisor", rclcpp::QoS(1));

  // Subscribers
  sub_overall_mrm_state = create_subscription<autoware_adapi_v1_msgs::msg::MrmState>(
    "/supervisor/fail_safe/overall/mrm_state", rclcpp::QoS(1),
    std::bind(&RedundantAutowareStateChecker::on_mrm_state, this, _1));
  sub_main_pose_with_covariance =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/main/localization/pose_with_covariance", rclcpp::QoS(1),
      std::bind(&RedundantAutowareStateChecker::on_main_pose_with_covariance, this, _1));
  sub_sub_pose_with_covariance = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/sub/localization/pose_with_covariance", rclcpp::QoS(1),
    std::bind(&RedundantAutowareStateChecker::on_sub_pose_with_covariance, this, _1));
  sub_main_operation_mode_state =
    create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "/main/api/operation_mode/state", rclcpp::QoS(1),
      std::bind(&RedundantAutowareStateChecker::on_main_operation_mode_state, this, _1));
  sub_sub_operation_mode_state =
    create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "/sub/api/operation_mode/state", rclcpp::QoS(1),
      std::bind(&RedundantAutowareStateChecker::on_sub_operation_mode_state, this, _1));
  sub_main_localization_initialization_state =
    create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
      "/main/api/localization/initialization_state", rclcpp::QoS(1),
      std::bind(
        &RedundantAutowareStateChecker::on_main_localization_initialization_state, this, _1));
  sub_sub_localization_initialization_state =
    create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
      "/sub/api/localization/initialization_state", rclcpp::QoS(1),
      std::bind(
        &RedundantAutowareStateChecker::on_sub_localization_initialization_state, this, _1));
  sub_main_route_state = create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/main/api/routing/state", rclcpp::QoS(1),
    std::bind(&RedundantAutowareStateChecker::on_main_route_state, this, _1));
  sub_sub_route_state = create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/sub/api/routing/state", rclcpp::QoS(1),
    std::bind(&RedundantAutowareStateChecker::on_sub_route_state, this, _1));
  sub_main_route = create_subscription<autoware_adapi_v1_msgs::msg::Route>(
    "/main/api/routing/route", rclcpp::QoS(1),
    std::bind(&RedundantAutowareStateChecker::on_main_route, this, _1));
  sub_sub_route = create_subscription<autoware_adapi_v1_msgs::msg::Route>(
    "/sub/api/routing/route", rclcpp::QoS(1),
    std::bind(&RedundantAutowareStateChecker::on_sub_route, this, _1));

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&RedundantAutowareStateChecker::onTimer, this));

  // Variables
  is_autonomous = true;
  last_time_pose_with_covariance_is_equal = this->now();
  last_time_operation_mode_state_is_equal = this->now();
  last_time_localization_initialization_state_is_equal = this->now();
  last_time_route_state_is_equal = this->now();
  last_time_route_is_equal = this->now();
}

void RedundantAutowareStateChecker::on_mrm_state(
  const autoware_adapi_v1_msgs::msg::MrmState::ConstSharedPtr msg)
{
  is_autonomous =
    (msg->state == autoware_adapi_v1_msgs::msg::MrmState::NORMAL &&
     msg->behavior == autoware_adapi_v1_msgs::msg::MrmState::NONE);
}

void RedundantAutowareStateChecker::on_main_pose_with_covariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  has_subscribed_main_pose_with_covariance = true;
  main_pose_with_covariance = msg;
}

void RedundantAutowareStateChecker::on_sub_pose_with_covariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  has_subscribed_sub_pose_with_covariance = true;
  sub_pose_with_covariance = msg;
}

void RedundantAutowareStateChecker::on_main_operation_mode_state(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  has_subscribed_main_operation_mode_state = true;
  main_operation_mode_state = msg;
}

void RedundantAutowareStateChecker::on_sub_operation_mode_state(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  has_subscribed_sub_operation_mode_state = true;
  sub_operation_mode_state = msg;
}

void RedundantAutowareStateChecker::on_main_localization_initialization_state(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg)
{
  has_subscribed_main_localization_initialization_state = true;
  main_localization_initialization_state = msg;
}

void RedundantAutowareStateChecker::on_sub_localization_initialization_state(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg)
{
  has_subscribed_sub_localization_initialization_state = true;
  sub_localization_initialization_state = msg;
}

void RedundantAutowareStateChecker::on_main_route_state(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  has_subscribed_main_route_state = true;
  main_route_state = msg;
}

void RedundantAutowareStateChecker::on_sub_route_state(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  has_subscribed_sub_route_state = true;
  sub_route_state = msg;
}

void RedundantAutowareStateChecker::on_main_route(
  const autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr msg)
{
  has_subscribed_main_route = true;
  main_route = msg;
}

void RedundantAutowareStateChecker::on_sub_route(
  const autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr msg)
{
  has_subscribed_sub_route = true;
  sub_route = msg;
}

void RedundantAutowareStateChecker::onTimer()
{
  if (!is_autonomous) return;

  const auto now = this->now();

  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = now;

  diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
  diag_status_msg.name = "redundant_autoware_state_checker";
  diag_status_msg.hardware_id = "";
  diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diag_status_msg.message = "";

  // Check the pose with covariance equality
  // Note that only this check raises WARN, not ERROR
  if (is_equal_pose_with_covariance()) {
    last_time_pose_with_covariance_is_equal = now;
  } else {
    if ((now - last_time_pose_with_covariance_is_equal).seconds() > states_equality_timeout) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "Main and Sub ECUs' pose with covariance are different.";
    }
  }

  // Check the operation mode state equality
  if (is_equal_operation_mode_stete()) {
    last_time_operation_mode_state_is_equal = now;
  } else {
    if ((now - last_time_operation_mode_state_is_equal).seconds() > states_equality_timeout) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message += "Main and Sub ECUs' operation mode states are different.";
    }
  }

  // Check the localization initialization state equality
  if (is_equal_localization_initialization_state()) {
    last_time_localization_initialization_state_is_equal = now;
  } else {
    if (
      (now - last_time_localization_initialization_state_is_equal).seconds() >
      states_equality_timeout) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message +=
        "Main and Sub ECUs' localization initialization states are different.";
    }
  }

  // Check the route state equality
  if (is_equal_route_stete()) {
    last_time_route_state_is_equal = now;
  } else {
    if ((now - last_time_route_state_is_equal).seconds() > states_equality_timeout) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message += "Main and Sub ECUs' route states are different.";
    }
  }

  // Check the route equality
  if (is_equal_route()) {
    last_time_route_is_equal = now;
  } else {
    if ((now - last_time_route_is_equal).seconds() > states_equality_timeout) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message += "Main and Sub ECUs' routes are different.";
    }
  }

  diag_msg.status.push_back(diag_status_msg);
  pub_diagnostic->publish(diag_msg);
}

bool RedundantAutowareStateChecker::is_equal_pose_with_covariance()
{
  // If ether state is not subscribed, then assume the states are equal
  if (!has_subscribed_main_pose_with_covariance) return true;
  if (!has_subscribed_sub_pose_with_covariance) return true;

  // Currently, Main and Sub ECUs poses are used to check the equality
  const auto main_pose = main_pose_with_covariance->pose.pose;
  const auto sub_pose = sub_pose_with_covariance->pose.pose;
  const auto squared_distance =
    (main_pose.position.x - sub_pose.position.x) * (main_pose.position.x - sub_pose.position.x) +
    (main_pose.position.y - sub_pose.position.y) * (main_pose.position.y - sub_pose.position.y);
  if (squared_distance > pose_distance_threshold * pose_distance_threshold) return false;

  return true;
}

bool RedundantAutowareStateChecker::is_equal_operation_mode_stete()
{
  // If ether state is not subscribed, then assume the states are equal
  if (!has_subscribed_main_operation_mode_state) return true;
  if (!has_subscribed_sub_operation_mode_state) return true;

  // Currently, following menbers are used to check the equality
  if (main_operation_mode_state->mode != sub_operation_mode_state->mode) return false;
  if (
    main_operation_mode_state->is_autoware_control_enabled !=
    sub_operation_mode_state->is_autoware_control_enabled)
    return false;

  return true;
}

bool RedundantAutowareStateChecker::is_equal_localization_initialization_state()
{
  // If ether state is not subscribed, then assume the states are equal
  if (!has_subscribed_main_localization_initialization_state) return true;
  if (!has_subscribed_sub_localization_initialization_state) return true;

  // Currently, following menbers are used to check the equality
  if (
    main_localization_initialization_state->state != main_localization_initialization_state->state)
    return false;

  return true;
}

bool RedundantAutowareStateChecker::is_equal_route_stete()
{
  // If ether state is not subscribed, then assume the states are equal
  if (!has_subscribed_main_route_state) return true;
  if (!has_subscribed_sub_route_state) return true;

  // Currently, following menbers are used to check the equality
  if (main_route_state->state != sub_route_state->state) return false;

  return true;
}

bool RedundantAutowareStateChecker::is_equal_route()
{
  // If ether state is not subscribed, then assume the states are equal
  if (!has_subscribed_main_route) return true;
  if (!has_subscribed_sub_route) return true;

  // Currently, following menbers are used to check the equality
  if (main_route->data != sub_route->data) return false;

  return true;
}

}  // namespace redundant_autoware_state_checker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<redundant_autoware_state_checker::RedundantAutowareStateChecker>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
