
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_MANAGER_INTERFACE_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/publisher.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(const SceneModuleManagerInterface &) = delete;
  SceneModuleManagerInterface(SceneModuleManagerInterface &&) = delete;
  SceneModuleManagerInterface & operator=(const SceneModuleManagerInterface &) = delete;
  SceneModuleManagerInterface & operator=(SceneModuleManagerInterface &&) = delete;
  explicit SceneModuleManagerInterface(std::string name) : name_{std::move(name)} {}

  virtual ~SceneModuleManagerInterface() = default;

  virtual void init(rclcpp::Node * node) = 0;

  void updateIdleModuleInstance();

  bool isExecutionRequested(const BehaviorModuleOutput & previous_module_output) const;

  void registerNewModule(
    const std::weak_ptr<SceneModuleInterface> & observer,
    const BehaviorModuleOutput & previous_module_output);

  void updateObserver();

  void publishRTCStatus();

  void publish_planning_factors();

  void publishVirtualWall() const;

  void publishMarker() const;

  bool exist(const std::shared_ptr<SceneModuleInterface> & module_ptr) const;

  /**
   * Determine if a new module can be launched. It ensures that only one instance of a particular
   * scene module type is registered at a time.
   *
   * When this returns true:
   * - A new instance of the scene module can be launched.
   * - No other instance of the same name of scene module is currently active or registered.
   *
   */
  bool canLaunchNewModule() const;

  virtual bool isSimultaneousExecutableAsApprovedModule() const;

  virtual bool isSimultaneousExecutableAsCandidateModule() const;

  void setData(const std::shared_ptr<PlannerData> & planner_data);

  void reset();

  std::string name() const;

  std::vector<std::weak_ptr<SceneModuleInterface>> getSceneModuleObservers();

  std::shared_ptr<SceneModuleInterface> getIdleModule();

  virtual void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) = 0;

  void initInterface(rclcpp::Node * node, const std::vector<std::string> & rtc_types);

protected:
  virtual std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() = 0;

  rclcpp::Node * node_ = nullptr;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_info_marker_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_virtual_wall_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_drivable_lanes_;

  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr pub_processing_time_;

  std::string name_;

  std::shared_ptr<PlannerData> planner_data_;

  std::vector<std::weak_ptr<SceneModuleInterface>> observers_;

  std::unique_ptr<SceneModuleInterface> idle_module_ptr_;

  std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;

  std::unordered_map<std::string, std::shared_ptr<autoware::rtc_interface::RTCInterface>>
    rtc_interface_ptr_map_{};

  std::unordered_map<
    std::string,
    std::shared_ptr<
      autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>>
    objects_of_interest_marker_interface_ptr_map_{};

  ModuleConfigParameters config_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_MANAGER_INTERFACE_HPP_
