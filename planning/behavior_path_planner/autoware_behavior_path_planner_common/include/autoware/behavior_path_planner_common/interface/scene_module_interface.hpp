// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_INTERFACE_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"
#include "autoware/behavior_path_planner_common/turn_signal_decider.hpp"
#include "autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp"
#include "autoware/planning_factor_interface/planning_factor_interface.hpp"
#include "autoware/rtc_interface/rtc_interface.hpp"
#include "autoware_utils/system/time_keeper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_rtc_msgs/msg/state.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include <any>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{

enum class ModuleStatus {
  IDLE = 0,
  RUNNING = 1,
  WAITING_APPROVAL = 2,
  SUCCESS = 3,
  FAILURE = 4,
};

class SceneModuleInterface
{
public:
  SceneModuleInterface(
    const std::string & name, rclcpp::Node & node,
    std::unordered_map<std::string, std::shared_ptr<autoware::rtc_interface::RTCInterface>>
      rtc_interface_ptr_map,
    std::unordered_map<
      std::string,
      std::shared_ptr<
        autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>>
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  SceneModuleInterface(const SceneModuleInterface &) = delete;
  SceneModuleInterface(SceneModuleInterface &&) = delete;
  SceneModuleInterface & operator=(const SceneModuleInterface &) = delete;
  SceneModuleInterface & operator=(SceneModuleInterface &&) = delete;
  virtual ~SceneModuleInterface() = default;

  virtual void updateModuleParams(const std::any & parameters) = 0;

  virtual void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const = 0;

  /**
   * @brief Return true if the module has request for execution (not necessarily feasible)
   */
  virtual bool isExecutionRequested() const = 0;

  /**
   * @brief Return true if the execution is available (e.g. safety check for lane change)
   */
  virtual bool isExecutionReady() const = 0;

  /**
   * @brief update data for planning. Note that the call of this function does not mean
   *        that the module executed. It should only updates the data necessary for
   *        planCandidate (e.g., resampling of path).
   */
  virtual void updateData() {}

  /**
   * @brief After executing run(), update the module-specific status and/or data used for internal
   *        processing that are not defined in ModuleStatus.
   */
  virtual void postProcess() {}

  /**
   * @brief Execute module. Once this function is executed,
   *        it will continue to run as long as it is in the RUNNING state.
   */
  virtual BehaviorModuleOutput run();

  /**
   * @brief Set the current_state_ based on updateState output.
   */
  void updateCurrentState();

  /**
   * @brief Called on the first time when the module goes into RUNNING.
   */
  void onEntry();

  /**
   * @brief Called when the module exit from RUNNING.
   */
  void onExit();

  void publishObjectsOfInterestMarker();

  void lockRTCCommand();

  void unlockRTCCommand();

  /**
   * @brief set previous module's output as input for this module
   */
  void setPreviousModuleOutput(const BehaviorModuleOutput & previous_module_output);

  std::shared_ptr<autoware_utils::TimeKeeper> getTimeKeeper() const;

  /**
   * @brief set planner data
   */
  virtual void setData(const std::shared_ptr<const PlannerData> & data);

  void lockOutputPath();

  void unlockOutputPath();

  bool isWaitingApproval() const;

  virtual bool isCurrentRouteLaneletToBeReset() const;

  bool isLockedNewModuleLaunch() const;

  PathWithLaneId::SharedPtr getPathCandidate() const;

  PathWithLaneId::SharedPtr getPathReference() const;

  visualization_msgs::msg::MarkerArray getInfoMarkers() const;

  visualization_msgs::msg::MarkerArray getDebugMarkers() const;

  visualization_msgs::msg::MarkerArray getDrivableLanesMarkers() const;

  virtual visualization_msgs::msg::MarkerArray getModuleVirtualWall();

  ModuleStatus getCurrentStatus() const;

  std::string name() const;

  PoseWithDetailOpt getStopPose() const;

  PoseWithDetailOpt getSlowPose() const;

  PoseWithDetailOpt getDeadPose() const;

  void resetWallPoses() const;

  rclcpp::Logger getLogger() const;

private:
  bool existRegisteredRequest() const;

  bool existApprovedRequest() const;

  bool existNotApprovedRequest() const;

  bool canTransitWaitingApprovalState() const;

  bool canTransitWaitingApprovalToRunningState() const;

  /**
   * @brief Return SUCCESS if plan is not needed or plan is successfully finished,
   *        FAILURE if plan has failed, RUNNING if plan is on going.
   *        These condition is to be implemented in each modules.
   */
  ModuleStatus updateState();

  std::string name_;

  ModuleStatus current_state_{ModuleStatus::IDLE};

  BehaviorModuleOutput previous_module_output_;

  bool is_locked_new_module_launch_{false};

  bool is_locked_output_path_{false};

protected:
  /**
   * @brief State transition condition ANY -> SUCCESS
   */
  virtual bool canTransitSuccessState() = 0;

  /**
   * @brief State transition condition ANY -> FAILURE
   */
  virtual bool canTransitFailureState() = 0;

  /**
   * @brief Explicitly set the initial state
   */
  virtual ModuleStatus setInitState() const;

  /**
   * @brief Get candidate path. This information is used for external judgement.
   */
  virtual CandidateOutput planCandidate() const = 0;

  /**
   * @brief Calculate path. This function is called with the plan is approved.
   */
  virtual BehaviorModuleOutput plan() = 0;

  /**
   * @brief Calculate path under waiting_approval condition.
   *        The default implementation is just to return the reference path.
   */
  virtual BehaviorModuleOutput planWaitingApproval();

  /**
   * @brief Module unique entry process.
   */
  virtual void processOnEntry() {}

  /**
   * @brief Module unique exit process.
   */
  virtual void processOnExit() {}

  virtual void updateRTCStatus(const double start_distance, const double finish_distance);

  void updateRTCStatusForSuccess();

  void setObjectsOfInterestData(
    const geometry_msgs::msg::Pose & obj_pose,
    const autoware_perception_msgs::msg::Shape & obj_shape,
    const autoware::objects_of_interest_marker_interface::ColorName & color_name);

  /**
   * @brief Return true if the activation command is received from the RTC interface.
   *        If no RTC interface is registered, return true.
   */
  bool isActivated() const;

  void removeRTCStatus();

  void set_longitudinal_planning_factor(const PathWithLaneId & path);

  void setDrivableLanes(const std::vector<DrivableLanes> & drivable_lanes);

  BehaviorModuleOutput getPreviousModuleOutput() const { return previous_module_output_; }

  bool isOutputPathLocked() const { return is_locked_output_path_; }

  void lockNewModuleLaunch() { is_locked_new_module_launch_ = true; }

  void unlockNewModuleLaunch() { is_locked_new_module_launch_ = false; }

  void waitApproval() { is_waiting_approval_ = true; }

  void clearWaitingApproval() { is_waiting_approval_ = false; }

  void resetPathCandidate() { path_candidate_.reset(); }

  void resetPathReference() { path_reference_.reset(); }

  geometry_msgs::msg::Point getEgoPosition() const
  {
    return planner_data_->self_odometry->pose.pose.position;
  }

  geometry_msgs::msg::Pose getEgoPose() const { return planner_data_->self_odometry->pose.pose; }

  geometry_msgs::msg::Twist getEgoTwist() const
  {
    return planner_data_->self_odometry->twist.twist;
  }

  double getEgoSpeed() const
  {
    return std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  }

  rclcpp::Logger logger_;

  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<const PlannerData> planner_data_;

  bool is_waiting_approval_{false};

  std::unordered_map<std::string, UUID> uuid_map_;

  PathWithLaneId::SharedPtr path_candidate_;
  PathWithLaneId::SharedPtr path_reference_;

  std::unordered_map<std::string, std::shared_ptr<autoware::rtc_interface::RTCInterface>>
    rtc_interface_ptr_map_;

  std::unordered_map<
    std::string,
    std::shared_ptr<
      autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>>
    objects_of_interest_marker_interface_ptr_map_;

  mutable std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;

  mutable PoseWithDetailOpt stop_pose_{std::nullopt};

  mutable PoseWithDetailOpt slow_pose_{std::nullopt};

  mutable PoseWithDetailOpt dead_pose_{std::nullopt};

  mutable visualization_msgs::msg::MarkerArray info_marker_;

  mutable visualization_msgs::msg::MarkerArray debug_marker_;

  mutable visualization_msgs::msg::MarkerArray drivable_lanes_marker_;

  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_INTERFACE_HPP_
