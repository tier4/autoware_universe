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

#ifndef AUTOWARE__STACK_EVALUATOR__STACK_EVALUATOR_NODE_HPP_
#define AUTOWARE__STACK_EVALUATOR__STACK_EVALUATOR_NODE_HPP_

#include "autoware/stack_evaluator/evaluator_plugin_base.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::stack_evaluator
{

class StackEvaluatorNode : public rclcpp::Node
{
public:
  explicit StackEvaluatorNode(const rclcpp::NodeOptions & node_options);

private:
  void set_up_params();
  void initialize_plugins();
  void load_plugin(const std::string & plugin_name);
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);
  void onTimer();

  void fetchSharedData(plugin::EvaluatorData & data);

  pluginlib::ClassLoader<plugin::EvaluatorPluginBase> plugin_loader_;
  std::vector<std::shared_ptr<plugin::EvaluatorPluginBase>> plugins_;
  bool initialized_plugins_{false};
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  bool use_control_evaluator_{true};
  bool use_planning_evaluator_{true};

  autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> odometry_sub_{
    this, "~/input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<geometry_msgs::msg::AccelWithCovarianceStamped>
    accel_sub_{this, "~/input/acceleration"};
  autoware_utils::InterProcessPollingSubscriber<autoware_planning_msgs::msg::Trajectory> traj_sub_{
    this, "~/input/trajectory"};
  autoware_utils::InterProcessPollingSubscriber<
    autoware_planning_msgs::msg::LaneletRoute, autoware_utils::polling_policy::Newest>
    route_sub_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    autoware_map_msgs::msg::LaneletMapBin, autoware_utils::polling_policy::Newest>
    lanelet_map_bin_sub_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    autoware_internal_planning_msgs::msg::PathWithLaneId>
    behavior_path_sub_{this, "~/input/behavior_path"};
  autoware_utils::InterProcessPollingSubscriber<
    autoware_vehicle_msgs::msg::SteeringReport>
    steering_sub_{this, "~/input/steering_status"};
  autoware_utils::InterProcessPollingSubscriber<autoware_perception_msgs::msg::PredictedObjects>
    objects_sub_{this, "~/input/objects"};
  autoware_utils::InterProcessPollingSubscriber<
    autoware_vehicle_msgs::msg::TurnIndicatorsReport>
    blinker_sub_{this, "~/input/turn_indicators_status"};

  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    processing_time_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::stack_evaluator

#endif  // AUTOWARE__STACK_EVALUATOR__STACK_EVALUATOR_NODE_HPP_
