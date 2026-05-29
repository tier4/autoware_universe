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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__TRAFFIC_LIGHT_STOP_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__TRAFFIC_LIGHT_STOP_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

namespace autoware::trajectory_modifier::plugin
{
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using visualization_msgs::msg::MarkerArray;

class TrafficLightStop : public TrajectoryModifierPluginBase
{
public:
  TrafficLightStop() = default;

  bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const InputData & input) override;

  void update_params(const TrajectoryModifierParams & params) override;

  void publish_debug_data([[maybe_unused]] const std::string & ns) const override;

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  TrajectoryModifierParams::StoppingConstraints stopping_params_;

  SafetyFactorArray safety_factors_;

  MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_text_;

  void check_traffic_lights(const TrajectoryPoints & traj_points, const InputData & input);

  bool set_stop_point(TrajectoryPoints & traj_points, const InputData & input);

  bool apply_stopping(
    TrajectoryPoints & traj_points, const double target_stop_point_arc_length) const;

  void publish_debug_string(bool is_safe) const;
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__TRAFFIC_LIGHT_STOP_HPP_
