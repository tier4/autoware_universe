// Copyright 2023 TIER IV, Inc.
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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include "util.hpp"

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

class NoDrivableLaneModule : public SceneModuleInterface
{
public:
  enum class State { INIT, APPROACHING, INSIDE_NO_DRIVABLE_LANE, STOPPED };

  struct SegmentIndexWithPose
  {
    size_t index;
    geometry_msgs::msg::Pose pose;
  };

  struct DebugData
  {
    double base_link2front;
    std::optional<geometry_msgs::msg::Point> first_intersection_point;
    std::optional<geometry_msgs::msg::Point> second_intersection_point;
    std::vector<geometry_msgs::msg::Point> no_drivable_lane_polygon;
    geometry_msgs::msg::Pose stop_pose;
  };

  struct PlannerParam
  {
    double stop_margin;
    bool print_debug_info;
  };

  NoDrivableLaneModule(
    const int64_t module_id, const int64_t lane_id, const PlannerParam & planner_param,
    const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(PathWithLaneId * _path) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  const int64_t lane_id_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  // State machine
  State state_;

  void handle_init_state(
    const double ego_front_s,
    const PathWithNoDrivableLanePolygonIntersection & path_polygon_intersection);
  void handle_approaching_state(
    experimental::trajectory::Trajectory<PathPointWithLaneId> & path, const double ego_front_s,
    const PathWithNoDrivableLanePolygonIntersection & path_polygon_intersection);
  void handle_inside_no_drivable_lane_state(
    experimental::trajectory::Trajectory<PathPointWithLaneId> & path, const double ego_front_s);
  void handle_stopped_state(
    experimental::trajectory::Trajectory<PathPointWithLaneId> & path, const double ego_front_s);
  void initialize_debug_data(
    const experimental::trajectory::Trajectory<PathPointWithLaneId> & path,
    const lanelet::Lanelet & no_drivable_lane, const geometry_msgs::msg::Point & ego_pos,
    const PathWithNoDrivableLanePolygonIntersection & path_polygon_intersection);
};
}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
