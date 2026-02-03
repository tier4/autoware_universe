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

#ifndef PLUGINS__OBSTACLE_STOP_MODIFIER_HPP_
#define PLUGINS__OBSTACLE_STOP_MODIFIER_HPP_

#include <autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{

using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::TrajectoryModifierParams;
using autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_perception_msgs::msg::PredictedObjects;

class ObstacleStopModifier : public TrajectoryModifierPluginBase
{
public:
  ObstacleStopModifier(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,
    const TrajectoryModifierParams & params);

  void modify_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
    const TrajectoryModifierData & data) override;

  void set_up_params() override;

  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

  bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
    const TrajectoryModifierData & data) const override;

  // Set predicted objects for obstacle detection
  void set_predicted_objects(const PredictedObjects::ConstSharedPtr & objects);

private:
  struct Parameters
  {
    double stop_margin_m{5.0};
    double detection_range_m{50.0};
  };

  Parameters params_;
  PredictedObjects::ConstSharedPtr predicted_objects_;
};

}  // namespace autoware::minimum_rule_based_planner::plugin

#endif  // PLUGINS__OBSTACLE_STOP_MODIFIER_HPP_
