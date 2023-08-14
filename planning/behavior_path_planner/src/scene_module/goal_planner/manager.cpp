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

#include "behavior_path_planner/scene_module/goal_planner/manager.hpp"

#include "behavior_path_planner/utils/goal_planner/util.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

GoalPlannerModuleManager::GoalPlannerModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
  const std::shared_ptr<GoalPlannerParameters> & parameters)
: SceneModuleManagerInterface(node, name, config, {""}), parameters_{parameters}
{
  left_side_parking_ = parameters_->parking_policy == ParkingPolicy::LEFT_SIDE;
}

void GoalPlannerModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto & p = parameters_;

  [[maybe_unused]] std::string ns = name_ + ".";

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m->updateModuleParams(p);
  });
}

// enable SimultaneousExecutable whenever goal modification is not allowed
// because only minor path refinements are made for fixed goals
bool GoalPlannerModuleManager::isSimultaneousExecutableAsApprovedModule() const
{
  if (observers_.empty()) {
    if (!goal_planner_utils::isAllowedGoalModificaition(
          planner_data_->route_handler, left_side_parking_)) {
      return true;
    }

    return enable_simultaneous_execution_as_approved_module_;
  }

  const auto checker = [this](const SceneModuleObserver & observer) {
    if (observer.expired()) {
      return enable_simultaneous_execution_as_approved_module_;
    }

    const auto goal_planner_module = std::dynamic_pointer_cast<GoalPlannerModule>(observer.lock());
    if (!goal_planner_utils::isAllowedGoalModificaition(
          planner_data_->route_handler, left_side_parking_)) {
      return true;
    }

    return enable_simultaneous_execution_as_approved_module_;
  };

  return std::all_of(observers_.begin(), observers_.end(), checker);
}

// enable SimultaneousExecutable whenever goal modification is not allowed
// because only minor path refinements are made for fixed goals
bool GoalPlannerModuleManager::isSimultaneousExecutableAsCandidateModule() const
{
  if (observers_.empty()) {
    if (!goal_planner_utils::isAllowedGoalModificaition(
          planner_data_->route_handler, left_side_parking_)) {
      return true;
    }

    return enable_simultaneous_execution_as_candidate_module_;
  }

  const auto checker = [this](const SceneModuleObserver & observer) {
    if (observer.expired()) {
      return enable_simultaneous_execution_as_candidate_module_;
    }

    if (!goal_planner_utils::isAllowedGoalModificaition(
          planner_data_->route_handler, left_side_parking_)) {
      return true;
    }

    return enable_simultaneous_execution_as_candidate_module_;
  };

  return std::all_of(observers_.begin(), observers_.end(), checker);
}

}  // namespace behavior_path_planner
