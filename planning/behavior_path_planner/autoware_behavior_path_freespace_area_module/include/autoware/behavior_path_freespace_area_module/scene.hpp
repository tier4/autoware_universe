// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_FREESPACE_AREA_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_FREESPACE_AREA_MODULE__SCENE_HPP_

#include "autoware/behavior_path_freespace_area_module/data_structs.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

// Asynchronous A*/RRT* worker. Mirrors goal_planner's FreespaceParkingPlanner pattern: a dedicated
// timer on its own callback group, request/response guarded by one mutex, and an is-running atomic.
// Unlike goal_planner it shares state with the module through a shared_ptr-owned context instead of
// raw references, so a callback that is still in flight when the module instance is destroyed can
// never dereference freed memory.
class FreespaceAreaPlanner
{
public:
  FreespaceAreaPlanner(
    std::shared_ptr<FreespaceAreaWorkerContext> context, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock,
    std::shared_ptr<autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm> algo,
    const bool replan_when_obstacle_found)
  : ctx_(std::move(context)),
    logger_(logger),
    clock_(clock),
    algo_(std::move(algo)),
    replan_when_obstacle_found_(replan_when_obstacle_found)
  {
  }
  void onTimer();

private:
  std::shared_ptr<FreespaceAreaWorkerContext> ctx_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm> algo_;
  bool replan_when_obstacle_found_;
};

enum class FreespaceAreaState { IDLE = 0, PLANNING, LATCHED, COMPLETED };
enum class FreespaceAreaMode { NONE = 0, TRANSIT, TERMINAL };

class FreespaceAreaModule : public SceneModuleInterface
{
public:
  FreespaceAreaModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<FreespaceAreaParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> planning_factor_interface);

  ~FreespaceAreaModule() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<FreespaceAreaParameters>>(parameters);
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  bool canTransitSuccessState() override;
  bool canTransitFailureState() override { return false; }

  void initVariables();

  // Activation helpers.
  struct ActivationContext
  {
    bool active{false};
    FreespaceAreaMode mode{FreespaceAreaMode::NONE};
    lanelet::ConstArea area;
    lanelet::ConstLanelets entry_lanelets;
    lanelet::ConstLanelets exit_lanelets;
    bool ego_inside_area{false};
  };
  std::optional<ActivationContext> evaluateActivation() const;

  // A* start / goal computation.
  geometry_msgs::msg::Pose computeStartPose(const ActivationContext & ctx) const;
  geometry_msgs::msg::Pose computeGoalPose(const ActivationContext & ctx) const;

  // Compose the final path from the latched waypoints.
  BehaviorModuleOutput composeOutput();

  // Latch / replan bookkeeping.
  bool isLatchInvalid(const geometry_msgs::msg::Pose & start_pose);
  void requestPlan(
    const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose);

  std::shared_ptr<FreespaceAreaParameters> parameters_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // Async worker plumbing. worker_ctx_ is shared with the worker (see FreespaceAreaWorkerContext).
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  std::shared_ptr<FreespaceAreaWorkerContext> worker_ctx_;
  std::shared_ptr<autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm> algo_;

  // State.
  FreespaceAreaState state_{FreespaceAreaState::IDLE};
  FreespaceAreaMode mode_{FreespaceAreaMode::NONE};
  ActivationContext current_ctx_;

  // Latched result.
  PathWithLaneId composed_path_{};
  geometry_msgs::msg::Pose latched_start_pose_{};
  geometry_msgs::msg::Pose latched_goal_pose_{};
  rclcpp::Time latched_response_stamp_{0, 0, RCL_ROS_TIME};

  // Stuck detection.
  std::optional<rclcpp::Time> stopped_since_{};

  // Route change detection.
  std::optional<std::array<uint8_t, 16>> route_uuid_{};
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_FREESPACE_AREA_MODULE__SCENE_HPP_
