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

  DetectionAreaStop() = default;

  void begin_cycle(const InputData & input) override;

  bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const InputData & input) override;

  void update_params(const TrajectoryModifierParams & params) override;

  void publish_debug_data(const std::string & ns) const override;

  const TrajectoryModifierParams::DetectionArea & get_params() const { return params_; }

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  using Trajectory = utils::detection_area::Trajectory;
  using PointCloud = utils::detection_area::PointCloud;
  using DetectionArea = lanelet::autoware::DetectionArea;

  struct Module
  {
    lanelet::Id lane_id{};
    std::shared_ptr<const DetectionArea> regulatory_element;
    State state{State::GO};
    std::optional<rclcpp::Time> last_obstacle_found_time;
    bool has_obstacle{false};
    std::string detection_source;
    std::vector<geometry_msgs::msg::Point> obstacle_points;
    std::vector<std::vector<geometry_msgs::msg::Point>> object_polygons;
    std::optional<geometry_msgs::msg::Pose> stop_pose;
    std::optional<geometry_msgs::msg::Pose> dead_line_pose;
    double stop_point_arc_length{0.0};
    double forward_offset_to_stop_line{0.0};
    bool dead_line_passed{false};
    bool candidate_modified{false};
    std::string candidate_policy;
  };

  struct StopDecision
  {
    Module * module{nullptr};
    double stop_point_arc_length{0.0};
    std::string policy;
  };

  TrajectoryModifierParams::DetectionArea params_;
  TrajectoryModifierParams::StoppingConstraints stopping_params_;

  std::vector<lanelet::Id> route_lanelet_ids_;
  std::shared_ptr<lanelet::LaneletMap> last_lanelet_map_;
  std::vector<Module> modules_;
  std::shared_ptr<const PointCloud> cycle_pointcloud_;
  std::string debug_status_;
  bool last_candidate_modified_{false};
  bool pending_trajectory_release_{false};
  std::optional<float> last_reference_velocity_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_text_;

  void rebuild_modules(const InputData & input);
  void update_cycle_observations(const InputData & input);
  std::shared_ptr<const PointCloud> make_map_pointcloud(const InputData & input) const;

  [[nodiscard]] bool check_inputs(const InputData & input) const;
  std::optional<StopDecision> find_stop_decision(
    const TrajectoryPoints & traj_points, const InputData & input);
  std::optional<double> evaluate_module(
    Module & module, const TrajectoryPoints & traj_points, const InputData & input);
  bool set_stop_point(
    TrajectoryPoints & traj_points, const InputData & input, StopDecision & decision);
  [[nodiscard]] bool should_hold_stop_at_ego(
    const TrajectoryPoints & traj_points, const InputData & input) const;
  bool hold_stop_at_ego(TrajectoryPoints & traj_points, const InputData & input);
  [[nodiscard]] bool should_release_trajectory_at_ego(
    const TrajectoryPoints & traj_points, const InputData & input) const;
  bool release_stopped_trajectory(TrajectoryPoints & traj_points, const InputData & input);
  [[nodiscard]] bool candidate_relates_to_active_stop(
    const TrajectoryPoints & traj_points, const InputData & input, const Module & module) const;

  void reset_candidate_debug();
  void set_state(Module & module, State state);
  void publish_debug_string() const;
  static std::string module_key(lanelet::Id lane_id, lanelet::Id regulatory_element_id);
};
}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__DETECTION_AREA_STOP_HPP_
