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
  // The ~ expands to the node's namespace (behavior_path_planner)
  // Full topic will be: /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/direction_change/path
  path_publisher_ = node.create_publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>(
    "~/output/direction_change/path", 1);
  std::cout << "[MY_DEBUG] [DirectionChange] Constructor: Created path publisher at topic: " 
            << path_publisher_->get_topic_name() 
            << " (publisher valid: " << (path_publisher_ ? "yes" : "no") << ")" << std::endl;
}

void DirectionChangeModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  modified_path_ = PathWithLaneId();
  cusp_point_indices_.clear();
  current_segment_state_ = PathSegmentState::FORWARD_ONLY;
  first_cusp_index_ = 0;
  resetPathCandidate();
  resetPathReference();
}

void DirectionChangeModule::processOnEntry()
{
  std::cout << "[MY_DEBUG] [DirectionChange] Module entry - initializing variables" << std::endl;
  initVariables();
  updateData();
}

void DirectionChangeModule::processOnExit()
{
  std::cout << "[MY_DEBUG] [DirectionChange] Module exit - resetting variables" << std::endl;
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
    std::cout << "[MY_DEBUG] [DirectionChange] Previous module output path is empty. Cannot update data." << std::endl;
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
    std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: Path empty, module inactive" << std::endl;
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
            std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: direction_change_area tag found in lane_id=" << lane_id << ", module ACTIVE" << std::endl;
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
  std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: No direction_change_area tag found, module INACTIVE" << std::endl;
  return false;
}


BehaviorModuleOutput DirectionChangeModule::plan()
{
  // Note: updateData() is already called by SceneModuleInterface::run() before plan()
  // Following standard BPP module pattern: expensive computations happen in plan()
  
  // Detect cusp points when actually planning (expensive work done here, not in updateData())
  cusp_point_indices_ = findCuspPoints();
  
  BehaviorModuleOutput output;
  
  std::cout << "[MY_DEBUG] [DirectionChange] plan() called: Path points=" << reference_path_.points.size() << ", Cusp points detected=" << cusp_point_indices_.size() << std::endl;
  if (!cusp_point_indices_.empty()) {
    std::stringstream ss;
    ss << "[MY_DEBUG] [DirectionChange] Cusp indices: ";
    for (size_t i = 0; i < cusp_point_indices_.size() && i < 10; ++i) {
      ss << cusp_point_indices_[i];
      if (i < cusp_point_indices_.size() - 1 && i < 9) ss << ", ";
    }
    if (cusp_point_indices_.size() > 10) ss << "... (total: " << cusp_point_indices_.size() << ")";
    std::cout << ss.str() << std::endl;
  }

  // Strategy: Separate forward/backward path publishing
  // - If no cusps: return full path as forward segment
  // - If cusps exist: split at first cusp and publish forward/backward segments separately
  //   This prevents downstream modules from seeing mixed orientations
  
  if (cusp_point_indices_.empty()) {
    // No cusps detected in current reference_path
    // IMPORTANT: If we're already in BACKWARD_ONLY state, maintain that state
    // This prevents oscillation when cusp detection becomes unstable after passing cusp
    if (current_segment_state_ == PathSegmentState::BACKWARD_ONLY) {
      std::cout << "[MY_DEBUG] [DirectionChange] No cusp detected but already in BACKWARD_ONLY state, maintaining backward path" << std::endl;
      // Continue publishing backward path: reverse orientations and velocities
      output.path = reference_path_;
      for (auto & p : output.path.points) {
        double yaw = tf2::getYaw(p.point.pose.orientation);
        yaw = autoware_utils::normalize_radian(yaw + M_PI);
        p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
        p.point.longitudinal_velocity_mps = -std::abs(p.point.longitudinal_velocity_mps);
      }
      modified_path_ = output.path;
    } else {
      // Not in backward state yet - return full path as forward segment
      std::cout << "[MY_DEBUG] [DirectionChange] No cusp points detected, returning full path as forward segment" << std::endl;
      output.path = reference_path_;
      modified_path_ = output.path;
      current_segment_state_ = PathSegmentState::FORWARD_ONLY;
    }
  } else {
    // Cusps detected: implement separate forward/backward publishing strategy
    first_cusp_index_ = cusp_point_indices_[0];
    
    std::cout << "[MY_DEBUG] [DirectionChange] First cusp at index: " << first_cusp_index_ << std::endl;
    
    // Critical Safety Check: Lane Continuity with Reverse Exit
    const bool safety_check_passed = checkLaneContinuitySafety(
      reference_path_, cusp_point_indices_, planner_data_->route_handler);
    
    std::cout << "[MY_DEBUG] [DirectionChange] Safety check result: " << (safety_check_passed ? "PASSED" : "FAILED") << std::endl;
    
    if (!safety_check_passed) {
      std::cout << "[MY_DEBUG] [DirectionChange] FATAL: Lane continuity safety check failed. Returning path without modification." << std::endl;
      output.path = reference_path_;
      output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
      output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
      return output;
    }
    
    // Get ego's current position
    if (!planner_data_ || !planner_data_->self_odometry) {
      std::cout << "[MY_DEBUG] [DirectionChange] WARNING: No ego odometry available, defaulting to forward segment" << std::endl;
      current_segment_state_ = PathSegmentState::FORWARD_ONLY;
    } else {
      const auto & ego_pose = planner_data_->self_odometry->pose.pose;
      
      // Find ego's nearest point on the reference path
      const auto ego_nearest_idx_opt = findNearestIndex(reference_path_.points, ego_pose);
      
      if (!ego_nearest_idx_opt) {
        std::cout << "[MY_DEBUG] [DirectionChange] WARNING: Could not find nearest index for ego pose, defaulting to forward segment" << std::endl;
        current_segment_state_ = PathSegmentState::FORWARD_ONLY;
      } else {
        const size_t ego_nearest_idx = *ego_nearest_idx_opt;
        
        // Calculate signed arc length from ego to first cusp point
        // Negative = ego is before cusp, Positive = ego is after cusp
        const double distance_to_cusp = calcSignedArcLength(
          reference_path_.points, ego_nearest_idx, first_cusp_index_);
      
      // Threshold for "reached cusp" (2.0 meters before cusp)
      // Using negative threshold because calcSignedArcLength returns negative when ego is before target
      const double cusp_arrival_threshold = 2.0;
      
      auto stateToString = [](const PathSegmentState & s) {
        switch (s) {
          case PathSegmentState::FORWARD_ONLY:   return "FORWARD_ONLY";
          case PathSegmentState::BACKWARD_ONLY:  return "BACKWARD_ONLY";
          default:                               return "UNKNOWN";
        }
      };
      std::cout << "[MY_DEBUG] [DirectionChange] "
                << "state=" << stateToString(current_segment_state_)
                << ", ego_nearest_idx=" << ego_nearest_idx
                << ", first_cusp_index_=" << first_cusp_index_
                << ", distance_to_cusp=" << distance_to_cusp
                << ", threshold=" << cusp_arrival_threshold
                << std::endl;
        
        // Determine which segment to publish based on ego position
        if (distance_to_cusp < cusp_arrival_threshold) {
          // Ego has reached/near cusp → switch to backward segment
          if (current_segment_state_ != PathSegmentState::BACKWARD_ONLY) {
            std::cout << "[MY_DEBUG] [DirectionChange] Ego reached cusp, switching to BACKWARD_ONLY state" << std::endl;
          }
          current_segment_state_ = PathSegmentState::BACKWARD_ONLY;
        } else {
          // Ego still on forward segment → publish forward segment only
          if (current_segment_state_ != PathSegmentState::FORWARD_ONLY) {
            std::cout << "[MY_DEBUG] [DirectionChange] Ego before cusp, switching to FORWARD_ONLY state" << std::endl;
          }
//          current_segment_state_ = PathSegmentState::FORWARD_ONLY;
        }
      }
    }
    
    // Split path and publish appropriate segment
    if (current_segment_state_ == PathSegmentState::FORWARD_ONLY) {
      // Extract forward segment: [0, first_cusp_index] (inclusive)
      if (first_cusp_index_ < reference_path_.points.size()) {
        output.path.points.assign(
          reference_path_.points.begin(),
          reference_path_.points.begin() + first_cusp_index_ + 1);
        std::cout << "[MY_DEBUG] [DirectionChange] Publishing FORWARD segment: " 
                  << output.path.points.size() << " points (indices 0-" << first_cusp_index_ << ")" << std::endl;
      } else {
        std::cout << "[MY_DEBUG] [DirectionChange] ERROR: First cusp index out of bounds, using full path" << std::endl;
        output.path = reference_path_;
      }
    } else {
      // Extract backward segment: [first_cusp_index, end] (inclusive)
      if (first_cusp_index_ < reference_path_.points.size()) {
        output.path.points.assign(
          reference_path_.points.begin() + first_cusp_index_,
          reference_path_.points.end());

      // 全点を reverse として扱う（CUSP ロジック不要）
      for (auto & p : output.path.points) {
        double yaw = tf2::getYaw(p.point.pose.orientation);
        yaw = autoware_utils::normalize_radian(yaw + M_PI);
        p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
        p.point.longitudinal_velocity_mps = -std::abs(p.point.longitudinal_velocity_mps);
      }          

        // Reverse orientations for backward segment
        // Create a vector with single cusp at index 0 (start of backward segment)
        std::vector<size_t> backward_cusp_indices = {0};
        //reverseOrientationAtCusps(&output.path, backward_cusp_indices);
        
        std::cout << "[MY_DEBUG] [DirectionChange] Publishing BACKWARD segment: " 
                  << output.path.points.size() << " points (indices " << first_cusp_index_ 
                  << "-" << (reference_path_.points.size() - 1) << ") with reversed orientations" << std::endl;
      } else {
        std::cout << "[MY_DEBUG] [DirectionChange] ERROR: First cusp index out of bounds, using full path" << std::endl;
        output.path = reference_path_;
      }
    }
    
    modified_path_ = output.path;
    if (!output.path.points.empty()) {
      const size_t N = std::min<size_t>(5, output.path.points.size());
      std::cout << "[MY_DEBUG] [DirectionChange] Output path head " << N
                << " points (yaw [deg], v [m/s]):" << std::endl;
      for (size_t i = 0; i < N; ++i) {
        const auto & pt = output.path.points[i].point;
        const double yaw = tf2::getYaw(pt.pose.orientation);  // [rad]
        const double yaw_deg = yaw * 180.0 / M_PI;
        const double v = pt.longitudinal_velocity_mps;
        std::cout << "  idx=" << i
                  << ", yaw=" << yaw_deg
                  << " deg, v=" << v << " m/s"
                  << std::endl;
      }
    }  
  }

  // Publish processed path to topic for debugging
  if (path_publisher_ && !output.path.points.empty()) {
    autoware_internal_planning_msgs::msg::PathWithLaneId path_msg;
    path_msg.header.stamp = clock_->now();
    
    // Get frame_id from reference path or planner_data
    if (!reference_path_.points.empty() && planner_data_ && planner_data_->route_handler) {
      path_msg.header.frame_id = planner_data_->route_handler->getRouteHeader().frame_id;
    } else {
      path_msg.header.frame_id = "map";  // Default fallback
    }
    
    path_msg.points.reserve(output.path.points.size());
    for (const auto & point : output.path.points) {
      autoware_internal_planning_msgs::msg::PathPointWithLaneId path_point;
      path_point.point = point.point;
      path_point.lane_ids = point.lane_ids;
      path_msg.points.push_back(path_point);
    }
    
    path_publisher_->publish(path_msg);
    std::cout << "[MY_DEBUG] [DirectionChange] Published " 
              << (current_segment_state_ == PathSegmentState::FORWARD_ONLY ? "FORWARD" : "BACKWARD")
              << " segment to topic: " << path_publisher_->get_topic_name() 
              << " with " << path_msg.points.size() << " points" << std::endl;
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
  // Module can complete only after backward segment is published and ego has completed it
  if (!cusp_point_indices_.empty() && 
      current_segment_state_ == PathSegmentState::BACKWARD_ONLY) {
    // Check if ego has completed backward segment
    if (planner_data_ && planner_data_->self_odometry && !modified_path_.points.empty()) {
      const auto & ego_pose = planner_data_->self_odometry->pose.pose;
      const auto ego_nearest_idx_opt = findNearestIndex(modified_path_.points, ego_pose);
      
      if (ego_nearest_idx_opt && *ego_nearest_idx_opt < modified_path_.points.size()) {
        const size_t ego_nearest_idx = *ego_nearest_idx_opt;
        const double remaining_distance = calcSignedArcLength(
          modified_path_.points, ego_nearest_idx, modified_path_.points.size() - 1);
        
        // Complete if ego is near end of backward segment (within 1 meter)
        const double completion_threshold = 1.0;
        if (remaining_distance < completion_threshold) {
          std::cout << "[MY_DEBUG] [DirectionChange] Ego completed backward segment, module can transit to SUCCESS" << std::endl;
          return true;
        }
      }
    }
    // Still in backward segment
    return false;
  }
  
  // No cusps or still in forward segment - module stays active
  return false;
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

