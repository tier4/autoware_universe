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

#include "autoware/behavior_path_direction_change_module/scene.hpp"

#include "autoware/behavior_path_direction_change_module/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/Forward.h>

#include <tf2/utils.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestIndex;
using autoware_utils::calc_distance2d;

DirectionChangeModule::DirectionChangeModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<DirectionChangeParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface{
    name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface},
  parameters_{parameters}
{
  // Create publisher for processed path with reversed orientations
  path_publisher_ = node.create_publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>(
    "~/output/direction_change/path", 1);
}

void DirectionChangeModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  modified_path_ = PathWithLaneId();
  cusp_point_indices_.clear();
  resetPathCandidate();
  resetPathReference();
}

void DirectionChangeModule::processOnEntry()
{
  // std::cout << "[MY_DEBUG] [DirectionChange] Module entry - initializing variables" << std::endl;
  initVariables();
  updateData();
}

void DirectionChangeModule::processOnExit()
{
  // std::cout << "[MY_DEBUG] [DirectionChange] Module exit - resetting variables" << std::endl;
  initVariables();
}

void DirectionChangeModule::setParameters(
  const std::shared_ptr<DirectionChangeParameters> & parameters)
{
  parameters_ = parameters;
}

bool DirectionChangeModule::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }

  if (!parameters_->enable_cusp_detection && !parameters_->enable_reverse_following) {
    return false;
  }

  return shouldActivateModule();
}

bool DirectionChangeModule::isExecutionReady() const
{
  return true;
}

bool DirectionChangeModule::isReadyForNextRequest(
  const double & min_request_time_sec, bool override_requests) const noexcept
{
  (void)min_request_time_sec;
  (void)override_requests;
  return true;
}

void DirectionChangeModule::updateData()
{
  // Lightweight data update: Only update reference path from previous module output
  // Following standard BPP module pattern: updateData() should be lightweight,
  // expensive computations (like cusp detection) are done in plan()
  // For the first module, this is the reference path generated from route handler
  // (route comes from /planning/mission_planning/route topic)
  // For subsequent modules, this is the output from the previous module
  // Path points should already have lane_ids assigned and be on centerline
  const auto previous_output = getPreviousModuleOutput();
  if (previous_output.path.points.empty()) {
    // std::cout << "[MY_DEBUG] [DirectionChange] Previous module output path is empty. Cannot update data." << std::endl;
    return;
  }
  
  reference_path_ = previous_output.path;
  // Note: Cusp detection is done in plan() when actually planning, not here
}

std::vector<size_t> DirectionChangeModule::findCuspPoints() const
{
  if (!parameters_->enable_cusp_detection) {
    return {};
  }

  return detectCuspPoints(reference_path_, parameters_->cusp_detection_angle_threshold_deg);
}

bool DirectionChangeModule::shouldActivateModule() const
{
  if (reference_path_.points.empty()) {
    // std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: Path empty, module inactive" << std::endl;
    return false;
  }

  // Check direction_change_area tag for each lanelet in the path
  // Decision: Module activates if direction_change_area tag is present (regardless of cusp points)
  // If no tag → module inactive
  // If tag exists → module active (will handle cusps if they exist, or just forward path if not)

  // Check lanelets from path points
  if (planner_data_->route_handler) {
    std::set<int64_t> checked_lane_ids;
    for (const auto & path_point : reference_path_.points) {
      for (const auto & lane_id : path_point.lane_ids) {
        if (checked_lane_ids.find(lane_id) != checked_lane_ids.end()) {
          continue;  // Already checked this lanelet
        }
        checked_lane_ids.insert(lane_id);

        try {
          const auto lanelet = planner_data_->route_handler->getLaneletsFromId(lane_id);
          if (hasDirectionChangeAreaTag(lanelet)) {
            // std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: direction_change_area tag found in lane_id=" << lane_id << ", module ACTIVE" << std::endl;
            return true;  // Tag found, activate module
          }
        } catch (...) {
          // Lanelet not found, skip
          continue;
        }
      }
    }
  }

  // No direction_change_area tag found, module inactive
  // std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: No direction_change_area tag found, module INACTIVE" << std::endl;
  return false;
}


BehaviorModuleOutput DirectionChangeModule::plan()
{
  // Note: updateData() is already called by SceneModuleInterface::run() before plan()
  // Following standard BPP module pattern: expensive computations happen in plan()
  
  // Detect cusp points when actually planning (expensive work done here, not in updateData())
  cusp_point_indices_ = findCuspPoints();
  
  BehaviorModuleOutput output;
  
  // std::cout << "[MY_DEBUG] [DirectionChange] plan() called: Path points=" << reference_path_.points.size() << ", Cusp points detected=" << cusp_point_indices_.size() << std::endl;
  if (!cusp_point_indices_.empty()) {
    std::stringstream ss;
    ss << "[MY_DEBUG] [DirectionChange] Cusp indices: ";
    for (size_t i = 0; i < cusp_point_indices_.size() && i < 10; ++i) {
      ss << cusp_point_indices_[i];
      if (i < cusp_point_indices_.size() - 1 && i < 9) ss << ", ";
    }
    if (cusp_point_indices_.size() > 10) ss << "... (total: " << cusp_point_indices_.size() << ")";
    // std::cout << ss.str() << std::endl;
  }

  // Final implementation: overwrite path in-place
  output.path = reference_path_;

  // Reverse orientations (yaw) at cusp points to indicate reverse direction segments
  // If cusps exist → reverse orientations after odd-numbered cusps
  // If no cusps → path remains unchanged (original orientations)
  if (!cusp_point_indices_.empty()) {
    // std::cout << "[MY_DEBUG] [DirectionChange] Cusp points detected (" << cusp_point_indices_.size() << "), performing safety check and orientation reversal" << std::endl;
    
    // Critical Safety Check: Lane Continuity with Reverse Exit
    // Check if odd number of cusps (reverse exit) conflicts with next lane without direction_change_area tag
    // This prevents fatal condition where vehicle exits with reversed orientation but next lane expects forward
    const bool safety_check_passed = checkLaneContinuitySafety(
      reference_path_, cusp_point_indices_, planner_data_->route_handler);
    
    // std::cout << "[MY_DEBUG] [DirectionChange] Safety check result: " << (safety_check_passed ? "PASSED" : "FAILED") << std::endl;
    
    if (!safety_check_passed) {
      // std::cout << "[MY_DEBUG] [DirectionChange] FATAL: Lane continuity safety check failed. Odd number of cusps with reverse exit but next lane doesn't have direction_change_area tag. Returning path without modification." << std::endl;
      // Return path unchanged (original orientation) to prevent fatal condition
      output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
      return output;
    }

    reverseOrientationAtCusps(&output.path, cusp_point_indices_);
    
    // std::cout << "[MY_DEBUG] [DirectionChange] Orientation reversal completed for " << cusp_point_indices_.size() << " cusp points" << std::endl;
  } else {
    // std::cout << "[MY_DEBUG] [DirectionChange] No cusp points detected, returning path unchanged (forward segment only)" << std::endl;
  }
  // If no cusp points but tag exists, just return the path as-is (forward segment only)

  // Store modified path for debug visualization
  modified_path_ = output.path;

  // Publish processed path with reversed orientations to topic
  if (path_publisher_ && !modified_path_.points.empty()) {
    autoware_internal_planning_msgs::msg::PathWithLaneId path_msg;
    path_msg.header.stamp = clock_->now();
    
    // Get frame_id from reference path or planner_data
    if (!reference_path_.points.empty() && planner_data_ && planner_data_->route_handler) {
      path_msg.header.frame_id = planner_data_->route_handler->getRouteHeader().frame_id;
    } else {
      path_msg.header.frame_id = "map";  // Default fallback
    }
    
    path_msg.points.reserve(modified_path_.points.size());
    for (const auto & point : modified_path_.points) {
      autoware_internal_planning_msgs::msg::PathPointWithLaneId path_point;
      path_point.point = point.point;
      path_point.lane_ids = point.lane_ids;
      path_msg.points.push_back(path_point);
    }
    
    path_publisher_->publish(path_msg);
  }

  output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
  
  // Preserve drivable area information from previous module
  // This is critical: PlannerManager's generateCombinedDrivableArea() needs drivable_area_info
  // (specifically drivable_lanes) to compute left_bound and right_bound. Since we only modify
  // path point orientations (yaw), not geometry, the lane information remains valid and should
  // be preserved. Without this, generateDrivableArea() fails to compute bounds and throws error:
  // "The right or left bound of drivable area is empty"
  output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;

  return output;
}

BehaviorModuleOutput DirectionChangeModule::planWaitingApproval()
{
  return plan();
}

CandidateOutput DirectionChangeModule::planCandidate() const
{
  CandidateOutput output;
  output.path_candidate = reference_path_;
  return output;
}

// Removed: planForwardFollowing(), planReverseFollowing(), planAtCusp()
// These methods are no longer needed with simplified design
// Velocity signs are applied directly in plan() via applySignedVelocityToPath()

bool DirectionChangeModule::canTransitSuccessState()
{
  // Simplified: No state machine, so always return true when module completes processing
  // Module completes when path is processed and signed velocities are applied
  return true;
}

void DirectionChangeModule::setDebugMarkersVisualization() const
{
  debug_data_.cusp_points.clear();
  debug_data_.forward_path = reference_path_;
  debug_data_.reverse_path = modified_path_;

  for (const auto & cusp_idx : cusp_point_indices_) {
    if (cusp_idx < reference_path_.points.size()) {
      debug_data_.cusp_points.push_back(reference_path_.points[cusp_idx].point.pose.position);
    }
  }
}

}  // namespace autoware::behavior_path_planner

