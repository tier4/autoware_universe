// Copyright 2020 Tier IV, Inc.
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

#include <lanelet2_extension/utility/query.hpp>
#include <scene_module/bus_stop/manager.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bus_stop
{
using lanelet::autoware::BusStop;

BusStopModuleManager::BusStopModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(node, getModuleName()), node_(node)
{
  const std::string ns(getModuleName());
  planner_param_.buffer_size = node.declare_parameter<int64_t>(ns + ".buffer_size");
  planner_param_.lpf_gain = node.declare_parameter<double>(ns + ".lpf_gain");
  planner_param_.state_param.turn_signal_blinking_duration =
    node.declare_parameter<double>(ns + ".turn_signal_blinking_duration");
  planner_param_.safe_obstacle_vel_threshold_kmph =
    node.declare_parameter<double>(ns + ".safe_obstacle_vel_threshold_kmph");
  planner_param_.num_safe_vel_threshold =
    node.declare_parameter<int64_t>(ns + ".num_safe_vel_threshold");
  planner_param_.stop_margin_from_stop_line =
    node.declare_parameter<double>(ns + ".stop_margin_from_stop_line");
}

void BusStopModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & bus_stop_with_lane_id : planning_utils::getRegElemMapOnPath<BusStop>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_pose.pose)) {
    // Use lanelet_id to unregister module when the route is changed
    const auto lane_id = bus_stop_with_lane_id.second.id();
    const auto module_id = bus_stop_with_lane_id.first->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<BusStopModule>(
        module_id, lane_id, *bus_stop_with_lane_id.first, planner_param_, node_,
        logger_.get_child("bus_stop_module"), clock_));
      generateUUID(module_id);
      updateRTCStatus(
        getUUID(module_id), true, std::numeric_limits<double>::lowest(), path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
BusStopModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto bus_stop_id_set = planning_utils::getRegElemIdSetOnPath<BusStop>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_pose.pose);

  return [bus_stop_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return bus_stop_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace bus_stop
}  // namespace behavior_velocity_planner
