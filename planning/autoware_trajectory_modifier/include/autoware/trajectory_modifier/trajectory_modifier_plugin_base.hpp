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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGIN_BASE_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGIN_BASE_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"

#include <utility>

namespace autoware::trajectory_modifier
{
// The hotfix pipeline uses InputData for every plugin. Keep the velocity-limit implementation's
// data name local to its adapter instead of changing the other plugins' interface.
using TrajectoryModifierData = plugin::InputData;
using TrajectoryModifierParams = trajectory_modifier_params::Params;
}  // namespace autoware::trajectory_modifier

namespace autoware::trajectory_modifier::plugin
{
enum class ProcessingResult { Unchanged, Modified };

class VelocityLimitPluginBase : public TrajectoryModifierPluginBase
{
public:
  bool modify_trajectory(TrajectoryPoints & points, const InputData & input) final
  {
    return process(points, input) == ProcessingResult::Modified;
  }

  bool is_trajectory_modification_required(
    [[maybe_unused]] const TrajectoryPoints & points,
    [[maybe_unused]] const InputData & input) override
  {
    return true;
  }

  virtual ProcessingResult process(TrajectoryPoints & points, const TrajectoryModifierData & input) =
    0;

protected:
  void publish_selected_debug_trajectory(
    const rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr & publisher,
    const TrajectoryPoints & points, const TrajectoryModifierData & data) const
  {
    if (
      data.candidate_index != 0U ||
      (publisher->get_subscription_count() == 0U &&
       publisher->get_intra_process_subscription_count() == 0U)) {
      return;
    }
    autoware_planning_msgs::msg::Trajectory trajectory;
    trajectory.header = data.candidate_header;
    trajectory.points = points;
    publisher->publish(std::move(trajectory));
  }
};
}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGIN_BASE_HPP_
