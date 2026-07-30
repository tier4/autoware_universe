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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VIRTUAL_TRAFFIC_LIGHT_STOP_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VIRTUAL_TRAFFIC_LIGHT_STOP_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp>

#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
class VirtualTrafficLightStop : public TrajectoryModifierPluginBase
{
public:
  using State = tier4_v2x_msgs::msg::VirtualTrafficLightState;

  enum class ModuleState : uint8_t {
    NONE = 0,
    REQUESTING = 1,
    PASSING = 2,
    FINALIZING = 3,
    FINALIZED = 4,
  };

  void begin_cycle(const InputData & input) override;
  void end_cycle() override;

  bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const InputData & input) override;

  void update_params(const TrajectoryModifierParams & params) override;

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  struct PlannerParam
  {
    double max_delay_sec{3.0};
    double near_line_distance{1.0};
    double dead_line_margin{1.0};
    double max_yaw_deviation_rad{1.5707963267948966};
    bool check_timeout_after_stop_line{true};
    double hold_stop_margin_distance{0.0};
  };

  struct Module
  {
    lanelet::Id lane_id{};
    std::shared_ptr<const lanelet::autoware::VirtualTrafficLight> regulatory_element;
    lanelet::ConstLanelet lane;
    std::string instrument_type;
    std::string instrument_id;
    std::vector<tier4_v2x_msgs::msg::KeyValue> custom_tags;
    ModuleState state{ModuleState::NONE};
    std::optional<State> virtual_traffic_light_state;
    std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> infrastructure_command;
  };

  PlannerParam planner_param_;
  bool enabled_{false};
  bool first_candidate_in_cycle_{true};
  std::vector<lanelet::Id> route_lanelet_ids_;
  std::shared_ptr<lanelet::LaneletMap> last_lanelet_map_;
  std::vector<Module> modules_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr
    pub_infrastructure_commands_;

  void rebuild_modules(const InputData & input);
  void update_module_states(const InputData & input);
  bool process_trajectory(
    TrajectoryPoints & traj_points, const InputData & input, const bool update_state,
    const bool apply_modification);
  bool process_module(
    Module & module, TrajectoryPoints & traj_points, const InputData & input,
    const bool update_state, const bool apply_modification);
  bool insert_stop_velocity(
    TrajectoryPoints & traj_points, const TrajectoryPoints & path_points,
    const std::optional<double> & collision_s, const InputData & input, Module & module,
    const bool stop_line);

  void update_command(Module & module);
  void set_state(Module & module, ModuleState state, std::optional<lanelet::Id> end_line_id = {});
  bool is_state_timeout(const Module & module) const;
  bool has_right_of_way(const Module & module) const;

  static std::string module_key(lanelet::Id lane_id, lanelet::Id regulatory_element_id);
  static std::string state_to_string(ModuleState state);
};
}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VIRTUAL_TRAFFIC_LIGHT_STOP_HPP_
