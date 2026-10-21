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

#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

  ProcessingResult process(TrajectoryPoints & points, TrajectoryModifierData & data) override;

  void update_params(const TrajectoryModifierParams & params) override;

  void publish_debug_data(const std::string & ns) const override;

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  using VirtualTrafficLightStateArray = tier4_v2x_msgs::msg::VirtualTrafficLightStateArray;

  enum class Decision : uint8_t {
    NONE = 0,
    STOP = 1,
  };

  enum class StopTarget : uint8_t {
    NONE = 0,
    STOP_LINE = 1,
    END_LINE = 2,
  };

  enum class StopReason : uint8_t {
    NONE = 0,
    NO_STATE = 1,
    NO_RIGHT_OF_WAY = 2,
    STATE_TIMEOUT_BEFORE_STOP_LINE = 3,
    STATE_TIMEOUT_AFTER_STOP_LINE = 4,
    WAITING_FINALIZATION = 5,
    INVALID_END_LINE = 6,
  };

  struct PlannerParam
  {
    double max_delay_sec{3.0};
    double near_line_distance{1.0};
    double dead_line_margin{1.0};
    double max_yaw_deviation_rad{1.5707963267948966};
    bool check_timeout_after_stop_line{true};
    double min_hold_trajectory_length{5.0};
  };

  struct DebugData
  {
    Decision decision{Decision::NONE};
    StopTarget stop_target{StopTarget::NONE};
    StopReason stop_reason{StopReason::NONE};
    bool modified{false};
    bool has_vtl_state{false};
    bool approval{false};
    bool finalized{false};
    bool timeout{false};
    bool ego_is_in_module_lane{false};
    bool stop_line_relevant{false};
    std::optional<double> message_age_sec;
    std::optional<double> start_arc_path;
    std::optional<double> start_arc_centerline;
    std::optional<double> stop_arc_path;
    std::optional<double> stop_arc_centerline;
    std::optional<double> end_arc_path;
    std::optional<double> end_arc_centerline;
    std::optional<double> stop_distance;
  };

  struct ActiveEndLine
  {
    lanelet::Id id{};
    lanelet::ConstLineString3d line;
    double centerline_arc{};
  };

  struct Module
  {
    lanelet::Id lane_id{};
    std::shared_ptr<const lanelet::autoware::VirtualTrafficLight> regulatory_element;
    lanelet::ConstLanelet lane;
    std::string instrument_type;
    std::string instrument_id;
    std::vector<tier4_v2x_msgs::msg::KeyValue> custom_tags;
    geometry_msgs::msg::Point instrument_center;
    ModuleState state{ModuleState::NONE};
    std::optional<State> virtual_traffic_light_state;
    std::optional<ActiveEndLine> active_end_line;
    bool end_hold_active{false};
    std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> infrastructure_command;
    DebugData debug_data;
  };

  using Trajectory =
    autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

  PlannerParam planner_param_;
  TrajectoryModifierParams::StoppingConstraints stopping_params_;
  bool enabled_{false};
  std::vector<lanelet::Id> route_lanelet_ids_;
  std::shared_ptr<lanelet::LaneletMap> last_lanelet_map_;
  std::vector<Module> modules_;
  bool cycle_initialized_{false};
  rclcpp::Time cycle_time_;
  std::optional<nav_msgs::msg::Odometry> cycle_odometry_;
  double cycle_acceleration_{0.0};
  std::unique_ptr<
    autoware_utils_rclcpp::InterProcessPollingSubscriber<VirtualTrafficLightStateArray>>
    virtual_traffic_light_states_sub_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr
    pub_infrastructure_commands_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::StringStamped>::SharedPtr debug_text_pub_;

  void prepare_cycle(const TrajectoryModifierData & input);
  void publish_infrastructure_commands();
  bool modify_trajectory(TrajectoryPoints & traj_points);
  void rebuild_modules(const TrajectoryModifierData & input);
  void update_module_states(const VirtualTrafficLightStateArray::ConstSharedPtr & states);
  void update_module_lifecycle();
  bool process_trajectory(TrajectoryPoints & traj_points);
  bool process_module(
    Module & module, TrajectoryPoints & traj_points, const Trajectory & path, double ego_s);
  bool insert_stop_velocity(
    TrajectoryPoints & traj_points, const Trajectory & path, double ego_s,
    const std::optional<double> & collision_s, Module & module, StopReason reason,
    StopTarget target);

  void update_command(Module & module);
  void set_state(Module & module, ModuleState state, std::optional<lanelet::Id> end_line_id = {});
  bool is_state_timeout(const Module & module) const;
  bool has_right_of_way(const Module & module) const;

  void publish_debug_string(const std::string & ns) const;

  static std::string module_key(lanelet::Id lane_id, lanelet::Id regulatory_element_id);
  static std::string state_to_string(ModuleState state);
  static std::string decision_to_string(Decision decision);
  static std::string stop_target_to_string(StopTarget target);
  static std::string stop_reason_to_string(StopReason reason);
};
}  // namespace autoware::trajectory_modifier::plugin

#endif  // VIRTUAL_TRAFFIC_LIGHT_STOP_HPP_
