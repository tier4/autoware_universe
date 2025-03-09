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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <autoware/behavior_velocity_detection_area_module/scene.hpp>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::behavior_velocity_planner
{
class DetectionAreaWithRTCModule : public DetectionAreaModule, public SceneModuleInterfaceWithRTC
{
public:
  DetectionAreaWithRTCModule(
    const int64_t module_id, const int64_t lane_id,
    const lanelet::autoware::DetectionArea & detection_area_reg_elem,
    const PlannerParam & planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(PathWithLaneId * path) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override
  {
    return DetectionAreaModule::createDebugMarkerArray();
  }
  autoware::motion_utils::VirtualWalls createVirtualWalls() override
  {
    return DetectionAreaModule::createVirtualWalls();
  }

private:
};
}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
