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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/traffic_light_stop.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <memory>
#include <string>

namespace autoware::trajectory_modifier::plugin
{

void TrafficLightStop::on_initialize([[maybe_unused]] const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "modifier_traffic_light_stop");
}

void TrafficLightStop::update_params([[maybe_unused]] const TrajectoryModifierParams & params)
{
}

bool TrafficLightStop::is_trajectory_modification_required(
  [[maybe_unused]] const TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  return false;
}

bool TrafficLightStop::modify_trajectory(
  [[maybe_unused]] TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  return false;
}

void TrafficLightStop::check_traffic_lights(
  [[maybe_unused]] const TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
}

bool TrafficLightStop::set_stop_point(
  [[maybe_unused]] TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  return false;
}

bool TrafficLightStop::apply_stopping(
  [[maybe_unused]] TrajectoryPoints & traj_points,
  [[maybe_unused]] const double target_stop_point_arc_length) const
{
  return false;
}

void TrafficLightStop::publish_debug_string([[maybe_unused]] bool is_safe) const
{
}

void TrafficLightStop::publish_debug_data([[maybe_unused]] const std::string & ns) const
{
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::TrafficLightStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
