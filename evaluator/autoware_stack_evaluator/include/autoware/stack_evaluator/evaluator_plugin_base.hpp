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

#ifndef AUTOWARE__STACK_EVALUATOR__EVALUATOR_PLUGIN_BASE_HPP_
#define AUTOWARE__STACK_EVALUATOR__EVALUATOR_PLUGIN_BASE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>

namespace autoware::stack_evaluator::plugin
{

using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

struct EvaluatorData
{
  Odometry::ConstSharedPtr odometry;
  AccelWithCovarianceStamped::ConstSharedPtr acceleration;
  Trajectory::ConstSharedPtr trajectory;
  PredictedObjects::ConstSharedPtr objects;
  LaneletRoute::ConstSharedPtr route;
  LaneletMapBin::ConstSharedPtr lanelet_map_bin;
  PathWithLaneId::ConstSharedPtr behavior_path;
  SteeringReport::ConstSharedPtr steering;
  TurnIndicatorsReport::ConstSharedPtr blinker;
};

class EvaluatorPluginBase
{
public:
  EvaluatorPluginBase() = default;
  virtual ~EvaluatorPluginBase() = default;

  virtual void evaluate(EvaluatorData & data) = 0;

  virtual void set_up_params() = 0;

  virtual rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) = 0;

  virtual void initialize(const std::string & name, rclcpp::Node * node_ptr)
  {
    name_ = name;
    node_ptr_ = node_ptr;
    set_up_params();
  }

  std::string get_name() const { return name_; }

protected:
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }

private:
  std::string name_{"unnamed_plugin"};
  rclcpp::Node * node_ptr_{nullptr};
};

}  // namespace autoware::stack_evaluator::plugin

#endif  // AUTOWARE__STACK_EVALUATOR__EVALUATOR_PLUGIN_BASE_HPP_
