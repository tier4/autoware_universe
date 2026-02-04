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

#ifndef AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__SCENE_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_direction_change_module/data_structs.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

// State for cusp-based reverse lane following
enum class CuspReverseLaneFollowingStatus {
  IDLE = 0,           // Module is idle/inactive
  FORWARD_FOLLOWING,  // Following path in forward direction
  APPROACHING_CUSP,   // Approaching a cusp point (decelerating)
  AT_CUSP,            // At a cusp point (stopped, preparing for reverse)
  REVERSE_FOLLOWING,  // Following path in reverse direction
  COMPLETED           // Direction change completed
};

// Helper function to convert status to string for debugging
inline std::string toString(CuspReverseLaneFollowingStatus status) {
  switch (status) {
    case CuspReverseLaneFollowingStatus::IDLE: return "IDLE";
    case CuspReverseLaneFollowingStatus::FORWARD_FOLLOWING: return "FORWARD_FOLLOWING";
    case CuspReverseLaneFollowingStatus::APPROACHING_CUSP: return "APPROACHING_CUSP";
    case CuspReverseLaneFollowingStatus::AT_CUSP: return "AT_CUSP";
    case CuspReverseLaneFollowingStatus::REVERSE_FOLLOWING: return "REVERSE_FOLLOWING";
    case CuspReverseLaneFollowingStatus::COMPLETED: return "COMPLETED";
    default: return "UNKNOWN";
  }
}

class DirectionChangeModule : public SceneModuleInterface
{
public:
  DirectionChangeModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<DirectionChangeParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> planning_factor_interface);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  bool isReadyForNextRequest(
    const double & min_request_time_sec, bool override_requests = false) const noexcept;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;

  void setParameters(const std::shared_ptr<DirectionChangeParameters> & parameters);

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<DirectionChangeParameters>>(parameters);
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  bool canTransitSuccessState() override;
  bool canTransitFailureState() override { return false; }

  void initVariables();

  // Helper functions
  std::vector<size_t> findCuspPoints() const;

  /**
   * @brief Evaluate path and determine if module should activate
   *
   * Following start_planner_module pattern: performs sequential evaluation
   * and records each step result in conditions_evaluation.
   *
   * Evaluation steps:
   * 1. Check path existence
   * 2. Check direction_change_area tag
   * 3. Detect cusp points
   * 4. Check lane continuity safety
   *
   * @param[out] evaluation_data Debug data to record evaluation results
   * @return true if module should activate, false otherwise
   */
  bool evaluatePath(PathEvaluationDebugData & evaluation_data) const;

  /**
   * @brief Check if path has direction_change_area tag in any lanelet
   * @param[out] evaluation_data Debug data to record evaluation step
   * @return true if tag found, false otherwise
   */
  bool checkDirectionChangeAreaTag(PathEvaluationDebugData & evaluation_data) const;

  /**
   * @brief Detect and validate cusp points in path
   * @param[out] evaluation_data Debug data to record evaluation step
   * @return true if valid cusp points found (or none needed), false on error
   */
  bool detectAndValidateCuspPoints(PathEvaluationDebugData & evaluation_data);

  /**
   * @brief Check lane continuity safety for direction change
   * @param[out] evaluation_data Debug data to record evaluation step
   * @return true if safety check passed, false otherwise
   */
  bool checkLaneContinuitySafetyWithEvaluation(PathEvaluationDebugData & evaluation_data) const;

  // Legacy function (kept for compatibility, delegates to evaluatePath)
  bool shouldActivateModule() const;

  // Member variables
  PathWithLaneId reference_path_{};
  PathWithLaneId modified_path_{};
  std::shared_ptr<DirectionChangeParameters> parameters_;

  // Direction change data
  std::vector<size_t> cusp_point_indices_{};

  // State machine for cusp-based reverse lane following
  CuspReverseLaneFollowingStatus status_{CuspReverseLaneFollowingStatus::IDLE};
  size_t first_cusp_index_{0};  // Store first cusp index for path splitting
  
  // Store the actual cusp point position (not index) for reliable distance calculation
  // This is necessary because reference_path_ is updated every cycle with ego-relative indices
  std::optional<geometry_msgs::msg::Point> first_cusp_position_{std::nullopt};
  
  // Parameters for state transitions
  static constexpr double kApproachingCuspDistance = 5.0;  // [m] Distance to start APPROACHING_CUSP
  static constexpr double kAtCuspDistance = 0.3;           // [m] Distance to transition to AT_CUSP (now vehicle creeps closer)
  static constexpr double kStoppedVelocityThreshold = 0.1; // [m/s] Velocity threshold for "stopped"

  // Debug data
  mutable DirectionChangeDebugData debug_data_;
  void setDebugMarkersVisualization() const;

  // Publisher for processed path with reversed orientations
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr path_publisher_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__SCENE_HPP_

