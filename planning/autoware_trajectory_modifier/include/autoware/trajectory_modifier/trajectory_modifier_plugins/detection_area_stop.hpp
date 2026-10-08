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

#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"
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

  ProcessingResult process(TrajectoryPoints & points, TrajectoryModifierData & data) override;

  void update_params(const TrajectoryModifierParams & params) override;

  void publish_debug_data(const std::string & ns) const override;

  const TrajectoryModifierParams::DetectionAreaStop & get_params() const { return params_; }

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
    // Physical observations are computed once per cycle, independently of candidates.
    std::optional<double> physical_stop_distance;
    bool physical_deadline_passed{false};
    bool has_obstacle{false};
    bool force_stop_required{false};
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

  struct StopDecision
  {
    size_t module_index{0};
    std::optional<geometry_msgs::msg::Pose> stop_pose;
    std::optional<geometry_msgs::msg::Pose> dead_line_pose;
    double stop_point_arc_length{0.0};
    std::string policy;
  };

  TrajectoryModifierParams::DetectionAreaStop params_;
  TrajectoryModifierParams::StoppingConstraints stopping_params_;

  std::vector<lanelet::Id> route_lanelet_ids_;
  std::shared_ptr<lanelet::LaneletMap> last_lanelet_map_;
  std::vector<Module> modules_;
  std::shared_ptr<const PointCloud> cycle_pointcloud_;
  std::string debug_status_;
  bool last_candidate_modified_{false};
  rclcpp::Time cycle_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry::ConstSharedPtr cycle_odometry_;
  double cycle_acceleration_{0.0};
  bool cycle_initialized_{false};
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_text_;

  void prepare_cycle(const TrajectoryModifierData & input);
  bool modify_trajectory(TrajectoryPoints & traj_points, const TrajectoryModifierData & input);
  void rebuild_modules(const TrajectoryModifierData & input);
  void update_cycle_observations(const TrajectoryModifierData & input);
  void update_physical_stop_state(const TrajectoryModifierData & input);
  std::shared_ptr<const PointCloud> make_map_pointcloud(const TrajectoryModifierData & input) const;

  [[nodiscard]] bool check_inputs(const TrajectoryModifierData & input) const;
  std::optional<StopDecision> find_stop_decision(
    const TrajectoryPoints & traj_points, const TrajectoryModifierData & input) const;
  std::optional<StopDecision> evaluate_module(
    const Module & module, const TrajectoryPoints & traj_points,
    const TrajectoryModifierData & input) const;
  bool set_stop_point(
    TrajectoryPoints & traj_points, const TrajectoryModifierData & input, StopDecision & decision);
  [[nodiscard]] bool should_hold_stop_at_ego(
    const TrajectoryPoints & traj_points, const TrajectoryModifierData & input) const;
  bool hold_stop_at_ego(TrajectoryPoints & traj_points, const TrajectoryModifierData & input);
  [[nodiscard]] bool candidate_relates_to_active_stop(
    const TrajectoryPoints & traj_points, const TrajectoryModifierData & input,
    const Module & module) const;

  void reset_candidate_debug();
  void set_state(Module & module, State state);
  void publish_debug_string() const;
  static std::string module_key(lanelet::Id lane_id, lanelet::Id regulatory_element_id);
};
}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__DETECTION_AREA_STOP_HPP_
