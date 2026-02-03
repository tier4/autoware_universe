// Copyright 2025 TIER IV, Inc.
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

#include "obstacle_stop_modifier.hpp"

#include <autoware_utils/ros/update_param.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{

ObstacleStopModifier::ObstacleStopModifier(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,
  const TrajectoryModifierParams & params)
: TrajectoryModifierPluginBase(name, node_ptr, time_keeper, params)
{
  set_up_params();
}

bool ObstacleStopModifier::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, const TrajectoryModifierParams & /*params*/,
  const TrajectoryModifierData & /*data*/) const
{
  if (traj_points.empty()) {
    return false;
  }

  if (!predicted_objects_) {
    return false;
  }

  // TODO(odashima): Implement actual obstacle detection logic
  return false;
}

void ObstacleStopModifier::modify_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
  const TrajectoryModifierData & data)
{
  if (!is_trajectory_modification_required(traj_points, params, data)) {
    return;
  }

  // TODO(odashima): Implement actual obstacle stop logic
  // 1. Find obstacles on the trajectory path
  // 2. Calculate stop point before the obstacle
  // 3. Insert stop point into trajectory
}

void ObstacleStopModifier::set_up_params()
{
  auto * node = get_node_ptr();

  rcl_interfaces::msg::ParameterDescriptor stop_margin_desc;
  stop_margin_desc.description = "Stop margin before obstacle [m]";
  params_.stop_margin_m =
    node->declare_parameter<double>("obstacle_stop.stop_margin_m", 5.0, stop_margin_desc);

  rcl_interfaces::msg::ParameterDescriptor detection_range_desc;
  detection_range_desc.description = "Detection range for obstacles [m]";
  params_.detection_range_m =
    node->declare_parameter<double>("obstacle_stop.detection_range_m", 50.0, detection_range_desc);
}

rcl_interfaces::msg::SetParametersResult ObstacleStopModifier::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    update_param<double>(parameters, "obstacle_stop.stop_margin_m", params_.stop_margin_m);
    update_param<double>(parameters, "obstacle_stop.detection_range_m", params_.detection_range_m);
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void ObstacleStopModifier::set_predicted_objects(const PredictedObjects::ConstSharedPtr & objects)
{
  predicted_objects_ = objects;
}

}  // namespace autoware::minimum_rule_based_planner::plugin
