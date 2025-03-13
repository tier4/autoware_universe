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

#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"

#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware_utils/ros/marker_helper.hpp"
#include "autoware_utils/ros/parameter.hpp"

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
namespace autoware::behavior_path_planner
{

using autoware::motion_utils::createDeadLineVirtualWallMarker;
using autoware::motion_utils::createSlowDownVirtualWallMarker;
using autoware::motion_utils::createStopVirtualWallMarker;
using unique_identifier_msgs::msg::UUID;
using SceneModulePtr = std::shared_ptr<SceneModuleInterface>;
using SceneModuleObserver = std::weak_ptr<SceneModuleInterface>;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;
using autoware::planning_factor_interface::PlanningFactorInterface;
using autoware::rtc_interface::RTCInterface;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using tier4_rtc_msgs::msg::State;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::MarkerArray;
using PlanResult = PathWithLaneId::SharedPtr;

bool SceneModuleManagerInterface::isExecutionRequested(
  const BehaviorModuleOutput & previous_module_output) const
{
  idle_module_ptr_->setData(planner_data_);
  idle_module_ptr_->setPreviousModuleOutput(previous_module_output);
  idle_module_ptr_->updateData();

  return idle_module_ptr_->isExecutionRequested();
}

void SceneModuleManagerInterface::registerNewModule(
  const SceneModuleObserver & observer, const BehaviorModuleOutput & previous_module_output)
{
  if (observer.expired()) {
    return;
  }

  observer.lock()->setData(planner_data_);
  observer.lock()->setPreviousModuleOutput(previous_module_output);
  observer.lock()->getTimeKeeper()->add_reporter(this->pub_processing_time_);
  observer.lock()->onEntry();

  observers_.push_back(observer);
}

void SceneModuleManagerInterface::updateObserver()
{
  const auto itr = std::remove_if(
    observers_.begin(), observers_.end(), [](const auto & observer) { return observer.expired(); });

  observers_.erase(itr, observers_.end());
}

void SceneModuleManagerInterface::publishRTCStatus()
{
  for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
    if (ptr) {
      ptr->removeExpiredCooperateStatus();
      ptr->publishCooperateStatus(rclcpp::Clock(RCL_ROS_TIME).now());
    }
  }
}

void SceneModuleManagerInterface::publish_planning_factors()
{
  planning_factor_interface_->publish();
}

void SceneModuleManagerInterface::publishVirtualWall() const
{
  using autoware_utils::append_marker_array;

  visualization_msgs::msg::MarkerArray markers{};

  const auto marker_offset = std::numeric_limits<uint8_t>::max();

  uint32_t marker_id = marker_offset;
  for (const auto & m : observers_) {
    if (m.expired()) {
      continue;
    }

    const std::vector<std::pair<
      PoseWithDetailOpt, std::function<MarkerArray(
                           const geometry_msgs::msg::Pose &, const std::string &, rclcpp::Time,
                           uint32_t, double, const std::string &, bool)>>>
      pose_and_func_vec = {
        {m.lock()->getStopPose(), createStopVirtualWallMarker},
        {m.lock()->getSlowPose(), createSlowDownVirtualWallMarker},
        {m.lock()->getDeadPose(), createDeadLineVirtualWallMarker}};

    for (const auto & [opt_pose, create_virtual_wall] : pose_and_func_vec) {
      if (!!opt_pose) {
        const auto detail = opt_pose.value().detail;
        const auto text = m.lock()->name() + (detail.empty() ? "" : " (" + detail + ")");
        const auto virtual_wall = create_virtual_wall(
          opt_pose.value().pose, text, rclcpp::Clock().now(), marker_id, 0.0, "", true);
        append_marker_array(virtual_wall, &markers);
      }
    }

    const auto module_specific_wall = m.lock()->getModuleVirtualWall();
    append_marker_array(module_specific_wall, &markers);

    m.lock()->resetWallPoses();
  }

  pub_virtual_wall_->publish(markers);
}

void SceneModuleManagerInterface::publishMarker() const
{
  using autoware_utils::append_marker_array;

  visualization_msgs::msg::MarkerArray info_markers{};
  visualization_msgs::msg::MarkerArray debug_markers{};
  visualization_msgs::msg::MarkerArray drivable_lanes_markers{};

  const auto marker_offset = std::numeric_limits<uint8_t>::max();

  uint32_t marker_id = marker_offset;
  for (const auto & m : observers_) {
    if (m.expired()) {
      continue;
    }

    for (auto & marker : m.lock()->getInfoMarkers().markers) {
      marker.id += marker_id;
      info_markers.markers.push_back(marker);
    }

    for (auto & marker : m.lock()->getDebugMarkers().markers) {
      marker.id += marker_id;
      debug_markers.markers.push_back(marker);
    }

    for (auto & marker : m.lock()->getDrivableLanesMarkers().markers) {
      marker.id += marker_id;
      drivable_lanes_markers.markers.push_back(marker);
    }

    marker_id += marker_offset;
  }

  if (observers_.empty() && idle_module_ptr_ != nullptr) {
    append_marker_array(idle_module_ptr_->getInfoMarkers(), &info_markers);
    append_marker_array(idle_module_ptr_->getDebugMarkers(), &debug_markers);
    append_marker_array(idle_module_ptr_->getDrivableLanesMarkers(), &drivable_lanes_markers);
  }

  pub_info_marker_->publish(info_markers);
  pub_debug_marker_->publish(debug_markers);
  pub_drivable_lanes_->publish(drivable_lanes_markers);
}

bool SceneModuleManagerInterface::exist(const SceneModulePtr & module_ptr) const
{
  return std::any_of(observers_.begin(), observers_.end(), [&](const auto & observer) {
    return !observer.expired() && observer.lock() == module_ptr;
  });
}

/**
 * Determine if a new module can be launched. It ensures that only one instance of a particular
 * scene module type is registered at a time.
 *
 * When this returns true:
 * - A new instance of the scene module can be launched.
 * - No other instance of the same name of scene module is currently active or registered.
 *
 */
bool SceneModuleManagerInterface::canLaunchNewModule() const
{
  return observers_.empty();
}

bool SceneModuleManagerInterface::isSimultaneousExecutableAsApprovedModule() const
{
  return config_.enable_simultaneous_execution_as_approved_module;
}

bool SceneModuleManagerInterface::isSimultaneousExecutableAsCandidateModule() const
{
  return config_.enable_simultaneous_execution_as_candidate_module;
}

void SceneModuleManagerInterface::setData(const std::shared_ptr<PlannerData> & planner_data)
{
  planner_data_ = planner_data;
}

void SceneModuleManagerInterface::reset()
{
  std::for_each(observers_.begin(), observers_.end(), [](const auto & observer) {
    if (!observer.expired()) {
      observer.lock()->onExit();
    }
  });

  observers_.clear();

  if (idle_module_ptr_ != nullptr) {
    idle_module_ptr_->onExit();
    idle_module_ptr_.reset();
  }

  pub_debug_marker_->publish(MarkerArray{});
}

std::string SceneModuleManagerInterface::name() const
{
  return name_;
}

std::vector<SceneModuleObserver> SceneModuleManagerInterface::getSceneModuleObservers()
{
  return observers_;
}

std::shared_ptr<SceneModuleInterface> SceneModuleManagerInterface::getIdleModule()
{
  return std::move(idle_module_ptr_);
}

void SceneModuleManagerInterface::initInterface(
  rclcpp::Node * node, const std::vector<std::string> & rtc_types)
{
  using autoware_utils::get_or_declare_parameter;

  // init manager configuration
  {
    std::string ns = name_ + ".";
    try {
      config_.enable_rtc = get_or_declare_parameter<bool>(*node, "enable_all_modules_auto_mode")
                             ? false
                             : get_or_declare_parameter<bool>(*node, ns + "enable_rtc");
    } catch (const std::exception & e) {
      config_.enable_rtc = get_or_declare_parameter<bool>(*node, ns + "enable_rtc");
    }

    config_.enable_simultaneous_execution_as_approved_module = get_or_declare_parameter<bool>(
      *node, ns + "enable_simultaneous_execution_as_approved_module");
    config_.enable_simultaneous_execution_as_candidate_module = get_or_declare_parameter<bool>(
      *node, ns + "enable_simultaneous_execution_as_candidate_module");
  }

  // init rtc configuration
  for (const auto & rtc_type : rtc_types) {
    const auto snake_case_name = utils::convertToSnakeCase(name_);
    const auto rtc_interface_name =
      rtc_type.empty() ? snake_case_name : snake_case_name + "_" + rtc_type;
    rtc_interface_ptr_map_.emplace(
      rtc_type, std::make_shared<autoware::rtc_interface::RTCInterface>(
                  node, rtc_interface_name, config_.enable_rtc));
    objects_of_interest_marker_interface_ptr_map_.emplace(
      rtc_type, std::make_shared<
                  autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
                  node, rtc_interface_name));
  }

  // init publisher
  {
    pub_info_marker_ = node->create_publisher<MarkerArray>("~/info/" + name_, 20);
    pub_debug_marker_ = node->create_publisher<MarkerArray>("~/debug/" + name_, 20);
    pub_virtual_wall_ = node->create_publisher<MarkerArray>("~/virtual_wall/" + name_, 20);
    pub_drivable_lanes_ = node->create_publisher<MarkerArray>("~/drivable_lanes/" + name_, 20);
    pub_processing_time_ = node->create_publisher<autoware_utils::ProcessingTimeDetail>(
      "~/processing_time/" + name_, 20);
  }

  // planning factor
  {
    planning_factor_interface_ =
      std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(node, name_);
  }

  // misc
  {
    node_ = node;
  }
}

void SceneModuleManagerInterface::updateIdleModuleInstance()
{
  if (idle_module_ptr_) {
    idle_module_ptr_->onEntry();
    return;
  }

  idle_module_ptr_ = createNewSceneModuleInstance();
}
}  // namespace autoware::behavior_path_planner
