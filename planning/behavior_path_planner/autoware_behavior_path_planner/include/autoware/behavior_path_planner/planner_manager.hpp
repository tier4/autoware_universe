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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"
#include "autoware/universe_utils/ros/debug_publisher.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <autoware/universe_utils/system/time_keeper.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

using autoware::universe_utils::StopWatch;
using tier4_planning_msgs::msg::PathWithLaneId;
using tier4_planning_msgs::msg::StopReasonArray;
using SceneModulePtr = std::shared_ptr<SceneModuleInterface>;
using SceneModuleManagerPtr = std::shared_ptr<SceneModuleManagerInterface>;
using DebugPublisher = autoware::universe_utils::DebugPublisher;
using DebugDoubleMsg = tier4_debug_msgs::msg::Float64Stamped;
using DebugStringMsg = tier4_debug_msgs::msg::StringStamped;

enum Action {
  ADD = 0,
  DELETE,
  MOVE,
};

struct ModuleUpdateInfo
{
  explicit ModuleUpdateInfo(
    const SceneModulePtr & module_ptr, const Action & action, const std::string & description)
  : status(module_ptr->getCurrentStatus()),
    action(action),
    module_name(module_ptr->name()),
    description(description)
  {
  }

  explicit ModuleUpdateInfo(
    const std::string & name, const Action & action, const ModuleStatus & status,
    const std::string & description)
  : status(status), action(action), module_name(name), description(description)
  {
  }

  ModuleStatus status;

  Action action;

  std::string module_name;

  std::string description;
};

struct SceneModuleStatus
{
  explicit SceneModuleStatus(const std::string & n) : module_name(n) {}

  std::string module_name;

  bool is_execution_ready{false};
  bool is_waiting_approval{false};

  ModuleStatus status{ModuleStatus::SUCCESS};
};

class PlannerManager
{
public:
  explicit PlannerManager(rclcpp::Node & node);

  /**
   * @brief run all candidate and approved modules.
   * @param planner data.
   */
  BehaviorModuleOutput run(const std::shared_ptr<PlannerData> & data);

  /**
   * @brief register managers.
   * @param node.
   * @param plugin name.
   */
  void launchScenePlugin(rclcpp::Node & node, const std::string & name);

  /**
   * @brief calculate max iteration numbers.
   * Let N be the number of scene modules. The maximum number of iterations executed in a loop is N,
   * but after that, if there are any modules that have succeeded or failed, the approve_modules of
   * all modules are cleared, and the loop is executed for N-1 modules. As this process repeats, it
   * becomes N + (N-1) + (N-2) + … + 1, therefore the maximum number of iterations is N(N+1)/2.
   * @param number of scene module
   *
   */
  void calculateMaxIterationNum(const size_t scene_module_num);

  /**
   * @brief unregister managers.
   * @param node.
   * @param plugin name.
   */
  void removeScenePlugin(rclcpp::Node & node, const std::string & name);

  /**
   * @brief update module parameters. used for dynamic reconfigure.
   * @param parameters.
   */
  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
  {
    std::for_each(manager_ptrs_.begin(), manager_ptrs_.end(), [&parameters](const auto & m) {
      m->updateModuleParams(parameters);
    });
  }

  /**
   * @brief reset all member variables.
   */
  void reset()
  {
    std::for_each(manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->reset(); });
    approved_module_ptrs_.clear();
    candidate_module_ptrs_.clear();
    current_route_lanelet_ = std::nullopt;
    resetProcessingTime();
  }

  /**
   * @brief publish all registered modules' markers.
   */
  void publishMarker() const
  {
    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->publishMarker(); });
  }

  /**
   * @brief publish all registered modules' virtual wall.
   */
  void publishVirtualWall() const
  {
    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->publishVirtualWall(); });
  }

  /**
   * @brief get manager pointers.
   * @return manager pointers.
   */
  std::vector<SceneModuleManagerPtr> getSceneModuleManagers() const { return manager_ptrs_; }

  /**
   * @brief get all scene modules status (is execution request, is ready and status).
   * @return statuses
   */
  std::vector<std::shared_ptr<SceneModuleStatus>> getSceneModuleStatus() const
  {
    std::vector<std::shared_ptr<SceneModuleStatus>> ret;

    const auto size = approved_module_ptrs_.size() + candidate_module_ptrs_.size();

    ret.reserve(size);

    for (const auto & m : approved_module_ptrs_) {
      auto s = std::make_shared<SceneModuleStatus>(m->name());
      s->is_waiting_approval = m->isWaitingApproval();
      s->status = m->getCurrentStatus();
      ret.push_back(s);
    }

    for (const auto & m : candidate_module_ptrs_) {
      auto s = std::make_shared<SceneModuleStatus>(m->name());
      s->is_waiting_approval = m->isWaitingApproval();
      s->status = m->getCurrentStatus();
      ret.push_back(s);
    }

    return ret;
  }

  /**
   * @brief aggregate launched module's stop reasons.
   * @return stop reason array
   */
  StopReasonArray getStopReasons() const
  {
    StopReasonArray stop_reason_array;
    stop_reason_array.header.frame_id = "map";
    stop_reason_array.header.stamp = clock_.now();

    std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
      const auto reason = m->getStopReason();
      if (reason.reason != "") {
        stop_reason_array.stop_reasons.push_back(m->getStopReason());
      }
    });

    std::for_each(
      candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), [&](const auto & m) {
        const auto reason = m->getStopReason();
        if (reason.reason != "") {
          stop_reason_array.stop_reasons.push_back(m->getStopReason());
        }
      });

    return stop_reason_array;
  }

  /**
   * @brief check if there are approved modules.
   */
  bool hasApprovedModules() const { return !approved_module_ptrs_.empty(); }

  bool hasNonAlwaysExecutableApprovedModules() const
  {
    return std::any_of(
      approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
      [this](const auto & m) { return !getManager(m)->isAlwaysExecutableModule(); });
  }

  /**
   * @brief check if there are candidate modules.
   */
  bool hasCandidateModules() const { return !candidate_module_ptrs_.empty(); }

  /**
   * @brief show planner manager internal condition.
   */
  void print() const;

  /**
   * @brief publish processing time of each module.
   */
  void publishProcessingTime() const;

  /**
   * @brief visit each module and get debug information.
   */
  std::shared_ptr<SceneModuleVisitor> getDebugMsg();

  /**
   * @brief reset the current route lanelet to be the closest lanelet within the route
   * @param planner data.
   */
  void resetCurrentRouteLanelet(const std::shared_ptr<PlannerData> & data)
  {
    lanelet::ConstLanelet ret{};
    data->route_handler->getClosestLaneletWithinRoute(data->self_odometry->pose.pose, &ret);
    RCLCPP_DEBUG(logger_, "update current route lanelet. id:%ld", ret.id());
    current_route_lanelet_ = ret;
  }

private:
  /**
   * @brief run the module and publish RTC status.
   * @param module.
   * @param planner data.
   * @return planning result.
   */
  BehaviorModuleOutput run(
    const SceneModulePtr & module_ptr, const std::shared_ptr<PlannerData> & planner_data,
    const BehaviorModuleOutput & previous_module_output) const
  {
    universe_utils::ScopedTimeTrack st(module_ptr->name() + "=>run", *module_ptr->getTimeKeeper());
    stop_watch_.tic(module_ptr->name());

    module_ptr->setData(planner_data);
    module_ptr->setPreviousModuleOutput(previous_module_output);

    module_ptr->lockRTCCommand();
    const auto result = module_ptr->run();
    module_ptr->unlockRTCCommand();

    module_ptr->postProcess();

    module_ptr->updateCurrentState();

    module_ptr->publishSteeringFactor();

    module_ptr->publishObjectsOfInterestMarker();

    processing_time_.at(module_ptr->name()) += stop_watch_.toc(module_ptr->name(), true);

    return result;
  }

  /**
   * @brief find and set the closest lanelet within the route to current route lanelet
   * @param planner data.
   */
  void updateCurrentRouteLanelet(const std::shared_ptr<PlannerData> & data);

  void generateCombinedDrivableArea(
    BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & data) const;

  /**
   * @brief get reference path from current_route_lanelet_ centerline.
   * @param planner data.
   * @return reference path.
   */
  BehaviorModuleOutput getReferencePath(const std::shared_ptr<PlannerData> & data) const;

  /**
   * @brief publish the root reference path and current route lanelet
   */
  void publishDebugRootReferencePath(const BehaviorModuleOutput & reference_path) const;

  /**
   * @brief stop and unregister the module from manager.
   * @param module.
   */
  void deleteExpiredModules(SceneModulePtr & module_ptr) const
  {
    module_ptr->onExit();
    module_ptr->publishObjectsOfInterestMarker();
    module_ptr.reset();
  }

  /**
   * @brief swap the modules order based on it's priority.
   * @param modules.
   * @details for now, the priority is decided in config file and doesn't change runtime.
   */
  void sortByPriority(std::vector<SceneModulePtr> & modules) const
  {
    // TODO(someone) enhance this priority decision method.
    std::sort(modules.begin(), modules.end(), [this](auto a, auto b) {
      return getManager(a)->getPriority() < getManager(b)->getPriority();
    });
  }

  /**
   * @brief push back to approved_module_ptrs_.
   * @param approved module pointer.
   */
  void addApprovedModule(const SceneModulePtr & module_ptr)
  {
    approved_module_ptrs_.push_back(module_ptr);
  }

  /**
   * @brief reset processing time.
   */
  void resetProcessingTime()
  {
    for (auto & t : processing_time_) {
      t.second = 0.0;
    }
  }

  /**
   * @brief stop and remove all modules in candidate_module_ptrs_.
   */
  void clearCandidateModules()
  {
    std::for_each(candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), [this](auto & m) {
      deleteExpiredModules(m);
    });
    candidate_module_ptrs_.clear();
  }

  /**
   * @brief get manager pointer from module name.
   * @param module pointer.
   * @return manager pointer.
   */
  SceneModuleManagerPtr getManager(const SceneModulePtr & module_ptr) const
  {
    const auto itr = std::find_if(
      manager_ptrs_.begin(), manager_ptrs_.end(),
      [&module_ptr](const auto & m) { return m->name() == module_ptr->name(); });

    if (itr == manager_ptrs_.end()) {
      throw std::domain_error("unknown manager name.");
    }

    return *itr;
  }

  /**
   * @brief run all modules in approved_module_ptrs_ and get a planning result as
   * approved_modules_output.
   * @param planner data.
   * @param deleted modules.
   * @return valid planning result.
   * @details in this function, expired modules (ModuleStatus::FAILURE or ModuleStatus::SUCCESS) are
   * removed from approved_module_ptrs_ and added to deleted_modules.
   */
  BehaviorModuleOutput runApprovedModules(
    const std::shared_ptr<PlannerData> & data, std::vector<SceneModulePtr> & deleted_modules);

  /**
   * @brief select a module that should be execute at first.
   * @param modules that make execution request.
   * @return the highest priority module.
   */
  SceneModulePtr selectHighestPriorityModule(std::vector<SceneModulePtr> & request_modules) const;

  /**
   * @brief update candidate_module_ptrs_ based on latest request modules.
   * @param the highest priority module in latest request modules.
   */
  void updateCandidateModules(
    const std::vector<SceneModulePtr> & request_modules,
    const SceneModulePtr & highest_priority_module);

  /**
   * @brief get all modules that make execution request.
   * @param decided (=approved) path.
   * @param deleted modules.
   * @return request modules.
   */
  std::vector<SceneModulePtr> getRequestModules(
    const BehaviorModuleOutput & previous_module_output,
    const std::vector<SceneModulePtr> & deleted_modules) const;

  /**
   * @brief checks whether a path of trajectory has forward driving direction
   * @param modules that make execution request.
   * @param planner data.
   * @param decided (=approved) path.
   * @return the highest priority module in request modules, and it's planning result.
   */
  std::pair<SceneModulePtr, BehaviorModuleOutput> runRequestModules(
    const std::vector<SceneModulePtr> & request_modules, const std::shared_ptr<PlannerData> & data,
    const BehaviorModuleOutput & previous_module_output);

  /**
   * @brief run keep last approved modules
   * @param planner data.
   * @param previous module output.
   * @return planning result.
   */
  BehaviorModuleOutput runKeepLastModules(
    const std::shared_ptr<PlannerData> & data, const BehaviorModuleOutput & previous_output) const;

  static std::string getNames(const std::vector<SceneModulePtr> & modules);

  std::optional<lanelet::ConstLanelet> current_route_lanelet_{std::nullopt};

  std::vector<SceneModuleManagerPtr> manager_ptrs_;

  std::vector<SceneModulePtr> approved_module_ptrs_;

  std::vector<SceneModulePtr> candidate_module_ptrs_;

  std::unique_ptr<DebugPublisher> debug_publisher_ptr_;

  std::unique_ptr<DebugPublisher> state_publisher_ptr_;

  pluginlib::ClassLoader<SceneModuleManagerInterface> plugin_loader_;

  mutable rclcpp::Logger logger_;

  mutable rclcpp::Clock clock_;

  mutable StopWatch<std::chrono::milliseconds> stop_watch_;

  mutable std::unordered_map<std::string, double> processing_time_;

  mutable std::vector<ModuleUpdateInfo> debug_info_;

  mutable std::shared_ptr<SceneModuleVisitor> debug_msg_ptr_;

  size_t max_iteration_num_{100};
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER__PLANNER_MANAGER_HPP_
