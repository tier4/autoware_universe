// Copyright 2023 TIER IV, Inc.
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

#ifndef REDUNDANT_AUTOWARE_STATE_CHECKER_HPP_
#define REDUNDANT_AUTOWARE_STATE_CHECKER_HPP_

#include "rclcpp/rclcpp.hpp"

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace redundant_autoware_state_checker
{

class RedundantAutowareStateChecker : public rclcpp::Node
{
public:
  explicit RedundantAutowareStateChecker();

private:
  // Publishers
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostic;

  // Subscribers
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MrmState>::SharedPtr sub_overall_mrm_state;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_main_pose_with_covariance;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_sub_pose_with_covariance;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    sub_main_operation_mode_state;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    sub_sub_operation_mode_state;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr
    sub_main_localization_initialization_state;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr
    sub_sub_localization_initialization_state;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_main_route_state;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_sub_route_state;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr sub_main_route;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr sub_sub_route;

  // Callbacks
  void onTimer();
  void on_main_pose_with_covariance(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_sub_pose_with_covariance(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_mrm_state(const autoware_adapi_v1_msgs::msg::MrmState::ConstSharedPtr msg);
  void on_main_operation_mode_state(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);
  void on_sub_operation_mode_state(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);
  void on_main_localization_initialization_state(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg);
  void on_sub_localization_initialization_state(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg);
  void on_main_route_state(const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg);
  void on_sub_route_state(const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg);
  void on_main_route(const autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr msg);
  void on_sub_route(const autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr msg);

  // Variables
  bool is_autonomous;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_subscribed_main_pose_with_covariance = false;
  bool has_subscribed_sub_pose_with_covariance = false;
  bool has_subscribed_main_operation_mode_state = false;
  bool has_subscribed_sub_operation_mode_state = false;
  bool has_subscribed_main_localization_initialization_state = false;
  bool has_subscribed_sub_localization_initialization_state = false;
  bool has_subscribed_main_route_state = false;
  bool has_subscribed_sub_route_state = false;
  bool has_subscribed_main_route = false;
  bool has_subscribed_sub_route = false;

  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr main_pose_with_covariance;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr sub_pose_with_covariance;
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr main_operation_mode_state;
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr sub_operation_mode_state;
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr
    main_localization_initialization_state;
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr
    sub_localization_initialization_state;
  autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr main_route_state;
  autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr sub_route_state;
  autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr main_route;
  autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr sub_route;

  rclcpp::Time last_time_pose_with_covariance_is_equal;
  rclcpp::Time last_time_operation_mode_state_is_equal;
  rclcpp::Time last_time_localization_initialization_state_is_equal;
  rclcpp::Time last_time_route_state_is_equal;
  rclcpp::Time last_time_route_is_equal;

  // Params
  double states_equality_timeout;
  double update_rate_hz;
  double pose_distance_threshold;

  // Functions
  bool is_equal_pose_with_covariance();
  bool is_equal_operation_mode_stete();
  bool is_equal_localization_initialization_state();
  bool is_equal_route_stete();
  bool is_equal_route();
};

}  // namespace redundant_autoware_state_checker

#endif  // REDUNDANT_AUTOWARE_STATE_CHECKER_HPP_
