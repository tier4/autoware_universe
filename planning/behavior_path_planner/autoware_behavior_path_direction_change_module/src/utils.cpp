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

#include "autoware/behavior_path_direction_change_module/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <autoware/route_handler/route_handler.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/Forward.h>

#include <rclcpp/rclcpp.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <vector>

// Debug flag - comment out to disable all debug prints
#define DEBUG_DIRECTION_CHANGE_MODULE

namespace autoware::behavior_path_planner
{

std::vector<size_t> detectCuspPoints(
  const PathWithLaneId & path, const double angle_threshold_deg)
{
  std::vector<size_t> cusp_indices;
  if (path.points.size() < 2) {
    #ifdef DEBUG_DIRECTION_CHANGE_MODULE
    // Note: No logger available in this utility function, so debug prints are minimal
    #endif
    return cusp_indices;
  }

  const double angle_threshold_rad = autoware_utils::deg2rad(angle_threshold_deg);

  // Detect cusp points by comparing yaw angles between consecutive path points
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & prev_point = path.points[i - 1].point.pose;
    const auto & curr_point = path.points[i].point.pose;

    // Extract yaw angles from path point orientations
    const double yaw_prev = tf2::getYaw(prev_point.orientation);
    const double yaw_curr = tf2::getYaw(curr_point.orientation);

    // Calculate normalized angle difference
    const double angle_diff = autoware_utils::normalize_radian(yaw_curr - yaw_prev);

    // If angle change exceeds threshold, mark as cusp point
    if (std::abs(angle_diff) > angle_threshold_rad) {
      cusp_indices.push_back(i);
      #ifdef DEBUG_DIRECTION_CHANGE_MODULE
      // Note: No logger available in this utility function
      #endif
    }
  }

  #ifdef DEBUG_DIRECTION_CHANGE_MODULE
  // Note: No logger available in this utility function, debug info logged in calling function
  #endif

  return cusp_indices;
}

void reverseOrientationAtCusps(PathWithLaneId * path, const std::vector<size_t> & cusp_indices)
{
  if (path->points.empty() || cusp_indices.empty()) {
    #ifdef DEBUG_DIRECTION_CHANGE_MODULE
    // Note: No logger available in this utility function
    #endif
    return;
  }

  // Optimized: Single pass through path points with cusp index tracking
  // Complexity: O(n + m) where n = path points, m = cusp indices
  // Pattern: Before first cusp: original, After first cusp: reversed, After second: original, etc.
  // Toggle orientation at each cusp: original → reversed → original → reversed → ...
  size_t cusp_idx = 0;  // Current cusp index pointer
  bool is_reversed = false;  // Current reversal state
  size_t reversed_point_count = 0;  // Count points with reversed orientation

  for (size_t i = 0; i < path->points.size(); ++i) {
    // Check if we've passed any new cusp points
    while (cusp_idx < cusp_indices.size() && i > cusp_indices[cusp_idx]) {
        is_reversed = !is_reversed;  // Toggle at each cusp passed
      ++cusp_idx;
      #ifdef DEBUG_DIRECTION_CHANGE_MODULE
      // Note: No logger available in this utility function, debug info logged in calling function
      #endif
    }

    if (is_reversed) {
      // Reverse orientation: add π radians to yaw
      double yaw = tf2::getYaw(path->points[i].point.pose.orientation);
      yaw = autoware_utils::normalize_radian(yaw + M_PI);
      path->points[i].point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
      ++reversed_point_count;
      #ifdef DEBUG_DIRECTION_CHANGE_MODULE
      // Note: No logger available in this utility function, detailed debug info logged in calling function
      #endif
    }
  }
  
  #ifdef DEBUG_DIRECTION_CHANGE_MODULE
  // Note: No logger available in this utility function, summary logged in calling function
  // Reversed point count: reversed_point_count
  #endif

  // TODO: Future enhancement - velocity reversal for full compatibility with downstream modules
  //       Some downstream modules (e.g., motion_velocity_planner) use isDrivingForwardWithTwist()
  //       which checks velocity signs first. For full compatibility, consider also reversing
  //       velocity signs (negative for reverse segments) in addition to orientation reversal.
  //       This would ensure compatibility with both geometry-based and velocity-based direction detection.
  //       
  //       Implementation suggestion:
  //       void applyVelocityReversal(PathWithLaneId * path, const std::vector<size_t> & cusp_indices)
  //       {
  //         for (size_t i = 0; i < path->points.size(); ++i) {
  //           bool is_reversed = false;
  //           for (const auto & cusp_idx : cusp_indices) {
  //             if (i > cusp_idx) { is_reversed = !is_reversed; }
  //           }
  //           if (is_reversed) {
  //             path->points[i].point.longitudinal_velocity_mps = 
  //               -std::abs(path->points[i].point.longitudinal_velocity_mps);
  //           }
  //         }
  //       }
}

std::vector<size_t> detectLaneBoundaries(const PathWithLaneId & path)
{
  std::vector<size_t> lane_boundary_indices;
  
  // TODO: Implementation postponed
  // This method should detect lane transitions by:
  // 1. Tracking lane_ids in path points
  // 2. Finding indices where lane_id changes
  // 3. Returning vector of transition point indices
  // 
  // For now, return empty vector (placeholder)
  
  if (path.points.size() < 2) {
    return lane_boundary_indices;
  }

  // Placeholder: Check for lane_id changes
  // Actual implementation will be added in future
  // Optimized: Use set for O(1) lookup instead of nested loops
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & prev_lane_ids = path.points[i - 1].lane_ids;
    const auto & curr_lane_ids = path.points[i].lane_ids;
    
    // Quick check: if sizes differ, definitely a boundary
    if (prev_lane_ids.size() != curr_lane_ids.size()) {
      lane_boundary_indices.push_back(i);
      continue;
    }
    
    // Use set for efficient lookup (O(n) instead of O(n²))
    std::set<int64_t> prev_set(prev_lane_ids.begin(), prev_lane_ids.end());
    std::set<int64_t> curr_set(curr_lane_ids.begin(), curr_lane_ids.end());
    
    // If sets differ, it's a lane boundary
    if (prev_set != curr_set) {
      lane_boundary_indices.push_back(i);
    }
  }

  return lane_boundary_indices;
}

bool hasDirectionChangeAreaTag(const lanelet::ConstLanelet & lanelet)
{
  const std::string direction_change_area = lanelet.attributeOr("direction_change_area", "none");
  return direction_change_area != "none";
}

bool checkLaneContinuitySafety(
  const PathWithLaneId & path, const std::vector<size_t> & cusp_indices,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  // TODO: Implementation placeholder for critical safety check
  // 
  // Safety Check: Lane Continuity with Reverse Exit
  // 
  // Problem Scenario:
  // - Odd number of cusps means vehicle exits current lane in REVERSE mode
  // - If next lane doesn't have turn_area tag, Autoware defaults to FORWARD following
  // - This creates a FATAL condition: vehicle reversing but system expects forward motion
  // 
  // Algorithm:
  // 1. Count number of cusps in current lane (before lane boundary)
  // 2. If odd number of cusps:
  //    a. Find next lane (after lane boundary)
  //    b. Check if next lane has turn_area tag
  //    c. If next lane doesn't have tag → FATAL CONDITION (return false)
  //    d. If next lane has tag → Safe (can continue reverse, return true)
  // 3. If even number of cusps:
  //    a. Vehicle exits in FORWARD mode → Safe (return true)
  // 
  // Implementation Steps:
  // - Use detectLaneBoundaries() to find lane transition points
  // - Count cusps before each lane boundary
  // - Check turn_area tag for next lane after boundary
  // - Return false if fatal condition detected, true otherwise
  // 
  // For now, return true (assume safe) until full implementation
  // This should be implemented before production use
  
  if (!route_handler || path.points.empty() || cusp_indices.empty()) {
    #ifdef DEBUG_DIRECTION_CHANGE_MODULE
    // Note: No logger available in this utility function
    #endif
    return true;  // No cusps or no route handler, assume safe
  }

  // Placeholder: Full implementation needed
  // This is a critical safety check and must be implemented
  // Note: Logger warning removed as this function doesn't have access to logger
  // Warning will be logged in plan() if check fails

  #ifdef DEBUG_DIRECTION_CHANGE_MODULE
  // Note: No logger available in this utility function
  // Safety check: cusp_count=%zu (odd=%s), result=SAFE (placeholder)
  #endif

  return true;  // Placeholder: assume safe for now
}

}  // namespace autoware::behavior_path_planner

