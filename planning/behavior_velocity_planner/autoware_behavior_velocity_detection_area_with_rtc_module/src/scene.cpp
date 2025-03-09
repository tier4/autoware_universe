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

#include "scene.hpp"

#include <autoware/behavior_velocity_detection_area_module/utils.hpp>

#include <memory>
#include <utility>

namespace autoware::behavior_velocity_planner
{
using autoware::motion_utils::calcLongitudinalOffsetPose;
using autoware::motion_utils::calcSignedArcLength;

DetectionAreaWithRTCModule::DetectionAreaWithRTCModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::DetectionArea & detection_area_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: DetectionAreaModule(
    module_id, lane_id, detection_area_reg_elem, planner_param, logger, clock, time_keeper,
    planning_factor_interface),
  SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface)
{
}

bool DetectionAreaWithRTCModule::modifyPathVelocity(PathWithLaneId * path)
{
  DetectionAreaModule::setPlannerData(SceneModuleInterfaceWithRTC::planner_data_);

  const auto condition = [this](
                           const std::shared_ptr<const rclcpp::Time> & last_obstacle_found_time,
                           const rclcpp::Time & now, const double state_clear_time) {
    const bool is_safe =
      detection_area::can_clear_stop_state(last_obstacle_found_time, now, state_clear_time);
    return std::make_pair(is_safe, isActivated());
  };

  const auto result = DetectionAreaModule::modify_path_velocity(path, condition);
  if (!result.has_value()) {
    return false;
  }

  setSafe(result.value().first);
  setDistance(result.value().second);

  return true;
}
}  // namespace autoware::behavior_velocity_planner
