// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"

#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/planning_factor_interface/planning_factor_interface.hpp"

#include <magic_enum.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::objects_of_interest_marker_interface::ColorName;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;
using autoware::planning_factor_interface::PlanningFactorInterface;
using autoware::rtc_interface::RTCInterface;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using autoware_utils::calc_offset_pose;
using autoware_utils::generate_uuid;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_rtc_msgs::msg::State;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::MarkerArray;
using PlanResult = PathWithLaneId::SharedPtr;

SceneModuleInterface::SceneModuleInterface(
  const std::string & name, rclcpp::Node & node,
  std::unordered_map<std::string, std::shared_ptr<autoware::rtc_interface::RTCInterface>>
    rtc_interface_ptr_map,
  std::unordered_map<
    std::string,
    std::shared_ptr<
      autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>>
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: name_{name},
  logger_{node.get_logger().get_child(name)},
  clock_{node.get_clock()},
  rtc_interface_ptr_map_(std::move(rtc_interface_ptr_map)),
  objects_of_interest_marker_interface_ptr_map_(
    std::move(objects_of_interest_marker_interface_ptr_map)),
  planning_factor_interface_{planning_factor_interface},
  time_keeper_(std::make_shared<autoware_utils::TimeKeeper>())
{
  for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
    uuid_map_.emplace(module_name, generate_uuid());
  }
}

/**
 * @brief Execute module. Once this function is executed,
 *        it will continue to run as long as it is in the RUNNING state.
 */
BehaviorModuleOutput SceneModuleInterface::run()
{
  updateData();
  const auto output = isWaitingApproval() ? planWaitingApproval() : plan();
  try {
    autoware::motion_utils::validateNonEmpty(output.path.points);
  } catch (const std::exception & ex) {
    throw std::invalid_argument("[" + name_ + "]" + ex.what());
  }
  return output;
}

/**
 * @brief Set the current_state_ based on updateState output.
 */
void SceneModuleInterface::updateCurrentState()
{
  const auto print = [this](const auto & from, const auto & to) {
    RCLCPP_DEBUG(getLogger(), "[%s] Transit from %s to %s.", name_.c_str(), from.data(), to.data());
  };

  const auto & from = current_state_;
  current_state_ = updateState();
  print(magic_enum::enum_name(from), magic_enum::enum_name(current_state_));
}

/**
 * @brief Called when the module exit from RUNNING.
 */
void SceneModuleInterface::onExit()
{
  RCLCPP_DEBUG(getLogger(), "%s %s", name_.c_str(), __func__);

  if (getCurrentStatus() == ModuleStatus::SUCCESS) {
    updateRTCStatusForSuccess();
  }

  clearWaitingApproval();
  unlockNewModuleLaunch();
  unlockOutputPath();

  processOnExit();
}

void SceneModuleInterface::publishObjectsOfInterestMarker()
{
  for (const auto & [module_name, ptr] : objects_of_interest_marker_interface_ptr_map_) {
    if (ptr) {
      ptr->publishMarkerArray();
    }
  }
}

void SceneModuleInterface::lockRTCCommand()
{
  for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
    if (ptr) {
      ptr->lockCommandUpdate();
    }
  }
}

void SceneModuleInterface::unlockRTCCommand()
{
  for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
    if (ptr) {
      ptr->unlockCommandUpdate();
    }
  }
}

/**
 * @brief set previous module's output as input for this module
 */
void SceneModuleInterface::setPreviousModuleOutput(
  const BehaviorModuleOutput & previous_module_output)
{
  previous_module_output_ = previous_module_output;
}

std::shared_ptr<autoware_utils::TimeKeeper> SceneModuleInterface::getTimeKeeper() const
{
  return time_keeper_;
}

/**
 * @brief set planner data
 */
void SceneModuleInterface::setData(const std::shared_ptr<const PlannerData> & data)
{
  planner_data_ = data;
}

void SceneModuleInterface::lockOutputPath()
{
  is_locked_output_path_ = true;
}

void SceneModuleInterface::unlockOutputPath()
{
  is_locked_output_path_ = false;
}

bool SceneModuleInterface::isWaitingApproval() const
{
  return is_waiting_approval_ || current_state_ == ModuleStatus::WAITING_APPROVAL;
}

bool SceneModuleInterface::isCurrentRouteLaneletToBeReset() const
{
  return false;
}

bool SceneModuleInterface::isLockedNewModuleLaunch() const
{
  return is_locked_new_module_launch_;
}

PlanResult SceneModuleInterface::getPathCandidate() const
{
  return path_candidate_;
}

PlanResult SceneModuleInterface::getPathReference() const
{
  return path_reference_;
}

MarkerArray SceneModuleInterface::getInfoMarkers() const
{
  return info_marker_;
}

MarkerArray SceneModuleInterface::getDebugMarkers() const
{
  return debug_marker_;
}

MarkerArray SceneModuleInterface::getDrivableLanesMarkers() const
{
  return drivable_lanes_marker_;
}

MarkerArray SceneModuleInterface::getModuleVirtualWall()
{
  return visualization_msgs::msg::MarkerArray();
}

ModuleStatus SceneModuleInterface::getCurrentStatus() const
{
  return current_state_;
}

std::string SceneModuleInterface::name() const
{
  return name_;
}

PoseWithDetailOpt SceneModuleInterface::getStopPose() const
{
  if (!stop_pose_) {
    return {};
  }

  const auto & base_link2front = planner_data_->parameters.base_link2front;
  return PoseWithDetail(
    calc_offset_pose(stop_pose_.value().pose, base_link2front, 0.0, 0.0),
    stop_pose_.value().detail);
}

PoseWithDetailOpt SceneModuleInterface::getSlowPose() const
{
  if (!slow_pose_) {
    return {};
  }

  const auto & base_link2front = planner_data_->parameters.base_link2front;
  return PoseWithDetail(
    calc_offset_pose(slow_pose_.value().pose, base_link2front, 0.0, 0.0),
    slow_pose_.value().detail);
}

PoseWithDetailOpt SceneModuleInterface::getDeadPose() const
{
  if (!dead_pose_) {
    return {};
  }

  const auto & base_link2front = planner_data_->parameters.base_link2front;
  return PoseWithDetail(
    calc_offset_pose(dead_pose_.value().pose, base_link2front, 0.0, 0.0),
    dead_pose_.value().detail);
}

void SceneModuleInterface::resetWallPoses() const
{
  stop_pose_ = std::nullopt;
  slow_pose_ = std::nullopt;
  dead_pose_ = std::nullopt;
}

rclcpp::Logger SceneModuleInterface::getLogger() const
{
  return logger_;
}

bool SceneModuleInterface::existRegisteredRequest() const
{
  return std::any_of(
    rtc_interface_ptr_map_.begin(), rtc_interface_ptr_map_.end(),
    [&](const auto & rtc) { return rtc.second->isRegistered(uuid_map_.at(rtc.first)); });
}

bool SceneModuleInterface::existApprovedRequest() const
{
  return std::any_of(
    rtc_interface_ptr_map_.begin(), rtc_interface_ptr_map_.end(), [&](const auto & rtc) {
      if (!rtc.second->isRegistered(uuid_map_.at(rtc.first))) {
        return false;
      }

      if (rtc.second->isTerminated(uuid_map_.at(rtc.first))) {
        return false;
      }

      return rtc.second->isActivated(uuid_map_.at(rtc.first));
    });
}

bool SceneModuleInterface::existNotApprovedRequest() const
{
  return std::any_of(
    rtc_interface_ptr_map_.begin(), rtc_interface_ptr_map_.end(), [&](const auto & rtc) {
      return rtc.second->isRegistered(uuid_map_.at(rtc.first)) &&
             !rtc.second->isActivated(uuid_map_.at(rtc.first)) &&
             !rtc.second->isTerminated(uuid_map_.at(rtc.first));
    });
}

bool SceneModuleInterface::canTransitWaitingApprovalState() const
{
  if (!existRegisteredRequest()) {
    return false;
  }
  return existNotApprovedRequest();
}

bool SceneModuleInterface::canTransitWaitingApprovalToRunningState() const
{
  if (!existRegisteredRequest()) {
    return true;
  }
  return existApprovedRequest();
}

/**
 * @brief Return SUCCESS if plan is not needed or plan is successfully finished,
 *        FAILURE if plan has failed, RUNNING if plan is on going.
 *        These condition is to be implemented in each modules.
 */
ModuleStatus SceneModuleInterface::updateState()
{
  auto log_debug_throttled = [&](std::string_view message) -> void {
    RCLCPP_DEBUG(getLogger(), "%s", message.data());
  };
  if (current_state_ == ModuleStatus::IDLE) {
    auto init_state = setInitState();
    RCLCPP_DEBUG(
      getLogger(), "transiting from IDLE to %s", magic_enum::enum_name(init_state).data());
    return init_state;
  }

  if (current_state_ == ModuleStatus::RUNNING) {
    if (canTransitSuccessState()) {
      log_debug_throttled("transiting from RUNNING to SUCCESS");
      return ModuleStatus::SUCCESS;
    }

    if (canTransitFailureState()) {
      log_debug_throttled("transiting from RUNNING to FAILURE");
      return ModuleStatus::FAILURE;
    }

    if (canTransitWaitingApprovalState()) {
      log_debug_throttled("transiting from RUNNING to WAITING_APPROVAL");
      return ModuleStatus::WAITING_APPROVAL;
    }

    log_debug_throttled("transiting from RUNNING to RUNNING");
    return ModuleStatus::RUNNING;
  }

  if (current_state_ == ModuleStatus::WAITING_APPROVAL) {
    if (canTransitSuccessState()) {
      log_debug_throttled("transiting from WAITING_APPROVAL to SUCCESS");
      return ModuleStatus::SUCCESS;
    }

    if (canTransitFailureState()) {
      log_debug_throttled("transiting from WAITING_APPROVAL to FAILURE");
      return ModuleStatus::FAILURE;
    }

    if (canTransitWaitingApprovalToRunningState()) {
      log_debug_throttled("transiting from WAITING_APPROVAL to RUNNING");
      return ModuleStatus::RUNNING;
    }

    log_debug_throttled("transiting from WAITING_APPROVAL to WAITING APPROVAL");
    return ModuleStatus::WAITING_APPROVAL;
  }

  if (current_state_ == ModuleStatus::SUCCESS) {
    log_debug_throttled("already SUCCESS");
    return ModuleStatus::SUCCESS;
  }

  if (current_state_ == ModuleStatus::FAILURE) {
    log_debug_throttled("already FAILURE");
    return ModuleStatus::FAILURE;
  }

  log_debug_throttled("already IDLE");
  return ModuleStatus::IDLE;
}

/**
 * @brief Explicitly set the initial state
 */
ModuleStatus SceneModuleInterface::setInitState() const
{
  return ModuleStatus::RUNNING;
}

BehaviorModuleOutput SceneModuleInterface::planWaitingApproval()
{
  path_candidate_ = std::make_shared<PathWithLaneId>(planCandidate().path_candidate);
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  return getPreviousModuleOutput();
}
void SceneModuleInterface::updateRTCStatus(
  const double start_distance, const double finish_distance)
{
  for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
    if (ptr) {
      const auto state = !ptr->isRegistered(uuid_map_.at(module_name)) || isWaitingApproval()
                           ? State::WAITING_FOR_EXECUTION
                           : State::RUNNING;
      ptr->updateCooperateStatus(
        uuid_map_.at(module_name), isExecutionReady(), state, start_distance, finish_distance,
        clock_->now());
    }
  }
}

void SceneModuleInterface::updateRTCStatusForSuccess()
{
  for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
    if (ptr) {
      if (ptr->isRegistered(uuid_map_.at(module_name))) {
        ptr->updateCooperateStatus(
          uuid_map_.at(module_name), true, State::SUCCEEDED, std::numeric_limits<double>::lowest(),
          std::numeric_limits<double>::lowest(), clock_->now());
      }
    }
  }
}

void SceneModuleInterface::setObjectsOfInterestData(
  const geometry_msgs::msg::Pose & obj_pose, const autoware_perception_msgs::msg::Shape & obj_shape,
  const ColorName & color_name)
{
  for (const auto & [module_name, ptr] : objects_of_interest_marker_interface_ptr_map_) {
    if (ptr) {
      ptr->insertObjectData(obj_pose, obj_shape, color_name);
    }
  }
}

/**
 * @brief Return true if the activation command is received from the RTC interface.
 *        If no RTC interface is registered, return true.
 */
bool SceneModuleInterface::isActivated() const
{
  if (rtc_interface_ptr_map_.empty()) {
    return true;
  }

  if (!existRegisteredRequest()) {
    return false;
  }
  return existApprovedRequest();
}

void SceneModuleInterface::removeRTCStatus()
{
  for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
    if (ptr) {
      ptr->clearCooperateStatus();
    }
  }
}

void SceneModuleInterface::set_longitudinal_planning_factor(const PathWithLaneId & path)
{
  if (stop_pose_.has_value()) {
    planning_factor_interface_->add(
      path.points, getEgoPose(), stop_pose_.value().pose,
      autoware::planning_factor_interface::PlanningFactor::STOP, SafetyFactorArray{}, true, 0.0,
      0.0, stop_pose_.value().detail);
    return;
  }

  if (!slow_pose_.has_value()) {
    return;
  }

  planning_factor_interface_->add(
    path.points, getEgoPose(), slow_pose_.value().pose,
    autoware::planning_factor_interface::PlanningFactor::SLOW_DOWN, SafetyFactorArray{}, true, 0.0,
    0.0, slow_pose_.value().detail);
}

void SceneModuleInterface::setDrivableLanes(const std::vector<DrivableLanes> & drivable_lanes)
{
  drivable_lanes_marker_ =
    marker_utils::createDrivableLanesMarkerArray(drivable_lanes, "drivable_lanes");
}

void SceneModuleInterface::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "%s %s", name_.c_str(), __func__);

  processOnEntry();
}
}  // namespace autoware::behavior_path_planner
