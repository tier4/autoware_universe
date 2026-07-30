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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__DETECTION_AREA_STOP_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__DETECTION_AREA_STOP_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/detection_area_utils.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
class DetectionAreaStop : public TrajectoryModifierPluginBase
{
public:
  enum class State { GO, STOP };

  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using StringStamped = autoware_internal_debug_msgs::msg::StringStamped;

  void begin_cycle(const InputData & input) override;
  void end_cycle() override;

  bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const InputData & input) override;

  void update_params(const TrajectoryModifierParams & params) override;

  void publish_debug_data(const std::string & ns) const override;

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  using Trajectory = utils::detection_area::Trajectory;
  using PointCloud = utils::detection_area::PointCloud;
  using TargetFiltering = utils::detection_area::TargetFiltering;
  using DetectionArea = lanelet::autoware::DetectionArea;

  struct PlannerParam
  {
    double stop_margin{0.0};
    bool use_dead_line{false};
    double dead_line_margin{5.0};
    double state_clear_time{2.0};
    double hold_stop_margin_distance{0.0};
    double distance_to_judge_over_stop_line{0.5};
    bool suppress_pass_judge_when_stopping{false};
    bool enable_detected_obstacle_logging{false};
    std::string unstoppable_policy{"stop_after_stopline"};
    double max_deceleration{1.0};
    double delay_response_time{0.5};
    TargetFiltering target_filtering;
  };

  struct Module
  {
    lanelet::Id lane_id{};
    std::shared_ptr<const DetectionArea> regulatory_element;
    State state{State::GO};
    std::optional<rclcpp::Time> last_obstacle_found_time;
    double forward_offset_to_stop_line{0.0};
    bool has_obstacle{false};
    std::string detection_source;
    std::vector<geometry_msgs::msg::Point> obstacle_points;
    std::vector<std::vector<geometry_msgs::msg::Point>> object_polygons;
    std::optional<geometry_msgs::msg::Pose> stop_pose;
    std::optional<geometry_msgs::msg::Pose> dead_line_pose;
    double stop_point_arc_length{0.0};
    bool dead_line_passed{false};
    bool candidate_modified{false};
    std::string candidate_policy;
  };

  PlannerParam planner_param_;
  bool first_candidate_in_cycle_{true};
  std::vector<lanelet::Id> route_lanelet_ids_;
  std::shared_ptr<lanelet::LaneletMap> last_lanelet_map_;
  std::vector<Module> modules_;
  std::shared_ptr<const PointCloud> cycle_pointcloud_;
  std::string debug_status_;
  bool last_candidate_modified_{false};
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_text_;

  void rebuild_modules(const InputData & input);
  void update_cycle_observations(const InputData & input);
  std::shared_ptr<const PointCloud> make_map_pointcloud(const InputData & input) const;

  bool process_trajectory(
    TrajectoryPoints & traj_points, const InputData & input, bool update_state,
    bool apply_modification);
  bool process_module(
    Module & module, TrajectoryPoints & traj_points, const InputData & input, bool update_state,
    bool apply_modification);
  bool insert_stop_velocity(
    Trajectory & path, double stop_point_s, double modified_stop_point_s,
    const geometry_msgs::msg::Pose & self_pose, Module & module);

  void update_params(const TrajectoryModifierParams::DetectionArea & params);
  void set_state(Module & module, State state);
  void reset_candidate_debug();
  static std::string module_key(lanelet::Id lane_id, lanelet::Id regulatory_element_id);
};
}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__DETECTION_AREA_STOP_HPP_
