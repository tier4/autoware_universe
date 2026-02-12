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

namespace
{
void logDirectionChangeDebugInfo(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & current_reference_path,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & output_path,
  const geometry_msgs::msg::Pose & ego_pose)
{
  using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
  auto print_path = [](const std::string & label,
                       const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
  {
    std::cout << "[MY_DEBUG] [DirectionChange] " << label
              << " size=" << path.points.size() << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto & pt = path.points[i].point;
      const double x   = pt.pose.position.x;
      const double y   = pt.pose.position.y;
      const double yaw = tf2::getYaw(pt.pose.orientation);            // [rad]
      const double yaw_deg = yaw * 180.0 / M_PI;                      // [deg]
      const double v   = pt.longitudinal_velocity_mps;                // [m/s]
      std::cout << "  idx=" << i
                << ", x="   << x
                << ", y="   << y
                << ", yaw=" << yaw_deg << " deg"
                << ", v="   << v << " m/s"
                << std::endl;
    }
  };
  // ① current_reference_path
  print_path("Current reference path (input)", current_reference_path);
  // ② output.path
  print_path("Output path (DirectionChange output)", output_path);
  // ③ Ego
  {
    std::cout << "[MY_DEBUG] [DirectionChange] Ego state:" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    const double ex   = ego_pose.position.x;
    const double ey   = ego_pose.position.y;
    const double eyaw = tf2::getYaw(ego_pose.orientation);
    const double eyaw_deg = eyaw * 180.0 / M_PI;
    std::cout << "  x="   << ex
              << ", y="   << ey
              << ", yaw=" << eyaw_deg << " deg"
              << std::endl;
  }
}
}  // namespace

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
  current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
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
  BehaviorModuleOutput output;
  
  // Copy reference_path_ to local variable for stability
  const auto current_reference_path = reference_path_;

  // Detect cusp points using current_reference_path
  cusp_point_indices_ = detectCuspPoints(current_reference_path, parameters_->cusp_detection_angle_threshold_deg);

  
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
    // IMPORTANT: If we're already in reverse states, maintain that state
    // This prevents oscillation when cusp detection becomes unstable after passing cusp
    if (current_segment_state_ == PathSegmentState::AT_CUSP ||
        current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING) {
      std::cout << "[MY_DEBUG] [DirectionChange] No cusp detected but already in reverse state, maintaining backward path" << std::endl;
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
      current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
    }
  } else {
    // Cusps detected: implement separate forward/backward publishing strategy
    first_cusp_index_ = cusp_point_indices_[0];
    std::cout << "[MY_DEBUG] [DirectionChange] First cusp at index: " << first_cusp_index_ << std::endl;

      // Store cusp point position in global coordinates
    if (first_cusp_index_ < current_reference_path.points.size()) {
      first_cusp_position_ = current_reference_path.points[first_cusp_index_].point.pose.position;
      has_valid_cusp_ = true;
    }

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
      current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
    } else {
      const auto & ego_pose = planner_data_->self_odometry->pose.pose;
      
      // Split path at CUSP point to avoid incorrect nearest index detection in U-turn/folded paths
      // Strategy: Search only in the relevant segment based on current state
      size_t ego_nearest_idx = 0;
      double distance_to_cusp = 0.0;
      bool found_nearest = false;
      
      // Dynamic search range based on current state
      switch (current_segment_state_) {
        case PathSegmentState::APPROACHING_CUSP: {
          // APPROACHING_CUSP: Search only in the forward segment to avoid detecting points behind cusp
          std::vector<PathPointWithLaneId> forward_segment(
            current_reference_path.points.begin(),
            current_reference_path.points.begin() + first_cusp_index_ + 1
          );

          const auto ego_nearest_idx_opt = findNearestIndex(forward_segment, ego_pose);
          if (ego_nearest_idx_opt) {
            ego_nearest_idx = *ego_nearest_idx_opt;
            distance_to_cusp = calcSignedArcLength(
              forward_segment, ego_nearest_idx, first_cusp_index_);
            found_nearest = true;
          }
          break;
        }

        case PathSegmentState::FORWARD_FOLLOWING: {
          // FORWARD_FOLLOWING: Search only in the forward segment [0, first_cusp_index_]
          std::vector<PathPointWithLaneId> forward_segment(
            current_reference_path.points.begin(),
            current_reference_path.points.begin() + first_cusp_index_ + 1
          );

          const auto ego_nearest_idx_opt = findNearestIndex(forward_segment, ego_pose);
          if (ego_nearest_idx_opt) {
            ego_nearest_idx = *ego_nearest_idx_opt;
            distance_to_cusp = calcSignedArcLength(
              forward_segment, ego_nearest_idx, first_cusp_index_);
            found_nearest = true;
          }
          break;
        }

        case PathSegmentState::AT_CUSP: {
          // AT_CUSP: Search only in the forward segment (same as FORWARD_FOLLOWING)
          std::vector<PathPointWithLaneId> forward_segment(
            current_reference_path.points.begin(),
            current_reference_path.points.begin() + first_cusp_index_ + 1
          );

          const auto ego_nearest_idx_opt = findNearestIndex(forward_segment, ego_pose);
          if (ego_nearest_idx_opt) {
            ego_nearest_idx = *ego_nearest_idx_opt;
            distance_to_cusp = calcSignedArcLength(
              forward_segment, ego_nearest_idx, first_cusp_index_);
            found_nearest = true;
          }
          break;
        }

        case PathSegmentState::REVERSE_FOLLOWING: {
          // REVERSE_FOLLOWING: Search only in the backward segment [first_cusp_index_, end]
          std::vector<PathPointWithLaneId> backward_segment(
            current_reference_path.points.begin() + first_cusp_index_,
            current_reference_path.points.end()
          );

          const auto ego_nearest_idx_opt = findNearestIndex(backward_segment, ego_pose);
          if (ego_nearest_idx_opt) {
            // Adjust index to global path coordinate
            ego_nearest_idx = *ego_nearest_idx_opt + first_cusp_index_;
            // Distance from cusp (negative since ego is after cusp in backward segment)
            distance_to_cusp = calcSignedArcLength(
              current_reference_path.points, ego_nearest_idx, first_cusp_index_);
            found_nearest = true;
          }
          break;
        }

        default: {
          // Unknown state, default to forward segment search
          std::vector<PathPointWithLaneId> forward_segment(
            current_reference_path.points.begin(),
            current_reference_path.points.begin() + first_cusp_index_ + 1
          );

          const auto ego_nearest_idx_opt = findNearestIndex(forward_segment, ego_pose);
          if (ego_nearest_idx_opt) {
            ego_nearest_idx = *ego_nearest_idx_opt;
            distance_to_cusp = calcSignedArcLength(
              forward_segment, ego_nearest_idx, first_cusp_index_);
            found_nearest = true;
          }
          break;
        }
      }
      
      if (!found_nearest) {
        std::cout << "[MY_DEBUG] [DirectionChange] WARNING: Could not find nearest index for ego pose, defaulting to forward segment" << std::endl;
        current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
      } else {
        // State transition thresholds are defined in parameters_->cusp_detection_distance_start_approaching and cusp_detection_distance_threshold
        
        auto stateToString = [](const PathSegmentState & s) {
          switch (s) {
            case PathSegmentState::IDLE:               return "IDLE";
            case PathSegmentState::FORWARD_FOLLOWING:  return "FORWARD_FOLLOWING";
            case PathSegmentState::APPROACHING_CUSP:   return "APPROACHING_CUSP";
            case PathSegmentState::AT_CUSP:            return "AT_CUSP";
            case PathSegmentState::REVERSE_FOLLOWING:  return "REVERSE_FOLLOWING";
            case PathSegmentState::COMPLETED:          return "COMPLETED";
            default:                                   return "UNKNOWN";
          }
        };
        std::cout << "[MY_DEBUG] [DirectionChange] "
                  << "state=" << stateToString(current_segment_state_)
                  << ", ego_nearest_idx=" << ego_nearest_idx
                  << ", first_cusp_index_=" << first_cusp_index_
                  << ", distance_to_cusp=" << distance_to_cusp
                  << ", start_approaching_threshold=" << parameters_->cusp_detection_distance_start_approaching
                  << ", at_cusp_threshold=" << parameters_->cusp_detection_distance_threshold
                  << std::endl;

        // Determine state transition based on current state and distance to cusp
        // Allow only one state transition per cycle for robustness
        PathSegmentState new_state = current_segment_state_;

        switch (current_segment_state_) {
          case PathSegmentState::IDLE:
            // IDLE状態からの遷移は外部トリガーに依存
            break;

          case PathSegmentState::FORWARD_FOLLOWING:
            if (distance_to_cusp <= parameters_->cusp_detection_distance_start_approaching) {
              new_state = PathSegmentState::APPROACHING_CUSP;
            }
            break;

          case PathSegmentState::APPROACHING_CUSP:
            // Only allow forward transition to AT_CUSP (no backward transition)
            if (distance_to_cusp <= parameters_->cusp_detection_distance_threshold) {
              new_state = PathSegmentState::AT_CUSP;
            }
            break;

          case PathSegmentState::AT_CUSP:
            // Unconditionally transition to REVERSE_FOLLOWING when AT_CUSP is reached
            new_state = PathSegmentState::REVERSE_FOLLOWING;
            break;

          case PathSegmentState::REVERSE_FOLLOWING:
            // REVERSE_FOLLOWING状態は基本的に維持
            // 必要に応じてCOMPLETEDへの遷移を追加可能
            break;

          case PathSegmentState::COMPLETED:
            // COMPLETED状態は最終状態
            break;

          default:
            // Unknown state, reset to FORWARD_FOLLOWING
            new_state = PathSegmentState::FORWARD_FOLLOWING;
            break;
        }

        // State transition logging
        if (new_state != current_segment_state_) {
          std::cout << "[MY_DEBUG] [DirectionChange] State transition: "
                    << stateToString(current_segment_state_) << " -> "
                    << stateToString(new_state) << std::endl;
          current_segment_state_ = new_state;
        }
      }
    }
    
    // Split path and publish appropriate segment based on state
    if (current_segment_state_ == PathSegmentState::FORWARD_FOLLOWING ||
        current_segment_state_ == PathSegmentState::APPROACHING_CUSP) {
      // Extract forward segment: [0, first_cusp_index] (inclusive)
      if (first_cusp_index_ < current_reference_path.points.size()) {
        output.path.points.assign(
          current_reference_path.points.begin(),
          current_reference_path.points.begin() + first_cusp_index_ + 1);
        std::cout << "[MY_DEBUG] [DirectionChange] Publishing FORWARD segment: "
                  << output.path.points.size() << " points (indices 0-" << first_cusp_index_ << ")" << std::endl;
      } else {
        std::cout << "[MY_DEBUG] [DirectionChange] ERROR: First cusp index out of bounds, using full path" << std::endl;
        output.path = current_reference_path;
      }
    } else if (current_segment_state_ == PathSegmentState::AT_CUSP ||
               current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING) {
      // Extract backward segment: [first_cusp_index, end] (inclusive)
      if (first_cusp_index_ < current_reference_path.points.size()) {
        output.path.points.assign(
          current_reference_path.points.begin() + first_cusp_index_,
          current_reference_path.points.end());

      // ★ ここで幾何だけ densify（yaw と速度をいじる前）
      const double max_yaw_step_rad = autoware_utils::deg2rad(5.0);  // 調整パラメータ
      const double max_dist_step    = 0.5;                            // 調整パラメータ
      densifyPathByYawAndDistance(output.path.points, max_yaw_step_rad, max_dist_step);
      
          // Calculate speed limits based on reference path and parameters
          const double reference_speed_limit = [&]() -> double {
            // Use the speed from the first cusp point as reference, or a default if not available
            if (first_cusp_index_ < current_reference_path.points.size()) {
              return std::abs(static_cast<double>(current_reference_path.points[first_cusp_index_].point.longitudinal_velocity_mps));
            }
            return 2.0;  // Default fallback speed [m/s]
          }();

          // Determine effective speed limits with safety checks
          const double effective_cusp_speed = std::min(parameters_->reverse_initial_speed, parameters_->reverse_speed_limit);
          const double effective_reverse_speed = parameters_->reverse_speed_limit;

          // Calculate backward speeds with reference path consideration
          const double backward_cruise_speed = std::min(reference_speed_limit, effective_reverse_speed);
          const double backward_slow_speed = std::min(reference_speed_limit, effective_cusp_speed);

          for (size_t i = 0; i < output.path.points.size(); ++i) {
            auto & p = output.path.points[i];
            // yaw を π だけ反転
            double yaw = tf2::getYaw(p.point.pose.orientation);
            yaw = autoware_utils::normalize_radian(yaw + M_PI);
            p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
            // 速度プロファイルを上書き
            if (i == 0) {
              // CUSP直後：超低速（yaw大変化対策）
              p.point.longitudinal_velocity_mps = -backward_slow_speed;
            } else {
              // それ以降：参照パス速度にreverse_speed_limitを上限とした巡航速度
              p.point.longitudinal_velocity_mps = -backward_cruise_speed;
            }
        
        // Filter lane_ids: Keep only the maximum ID (backward lanelet)
        // OSM creation rule: forward lanelet ID < backward lanelet ID
        if (!p.lane_ids.empty() && p.lane_ids.size() > 1) {
          std::vector<int64_t> original_ids = p.lane_ids;
          int64_t max_lane_id = *std::max_element(p.lane_ids.begin(), p.lane_ids.end());
          p.lane_ids = {max_lane_id};
          
          std::cout << "[MY_DEBUG] [DirectionChange] Filtered lane_ids: [";
          for (size_t i = 0; i < original_ids.size(); ++i) {
            std::cout << original_ids[i];
            if (i < original_ids.size() - 1) std::cout << ", ";
          }
          std::cout << "] -> [" << max_lane_id << "]" << std::endl;
        }
      }          

        // Reverse orientations for backward segment
        // Create a vector with single cusp at index 0 (start of backward segment)
        std::vector<size_t> backward_cusp_indices = {0};
        //reverseOrientationAtCusps(&output.path, backward_cusp_indices);
        
        std::cout << "[MY_DEBUG] [DirectionChange] Publishing BACKWARD segment: " 
                  << output.path.points.size() << " points (indices " << first_cusp_index_ 
                  << "-" << (current_reference_path.points.size() - 1) << ") with reversed orientations" << std::endl;
      } else {
        std::cout << "[MY_DEBUG] [DirectionChange] ERROR: First cusp index out of bounds, using full path" << std::endl;
        output.path = current_reference_path;
      }
    }
    
    modified_path_ = output.path;
//    if (!output.path.points.empty()) {
//      const size_t N = std::min<size_t>(5, output.path.points.size());
//      std::cout << "[MY_DEBUG] [DirectionChange] Output path head " << N
//                << " points (yaw [deg], v [m/s]):" << std::endl;
//     for (size_t i = 0; i < N; ++i) {
//        const auto & pt = output.path.points[i].point;
//        const double yaw = tf2::getYaw(pt.pose.orientation);  // [rad]
//        const double yaw_deg = yaw * 180.0 / M_PI;
//        const double v = pt.longitudinal_velocity_mps;
//        std::cout << "  idx=" << i
//                  << ", yaw=" << yaw_deg
//                  << " deg, v=" << v << " m/s"
//                  << std::endl;
//      }
//    }  
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
    std::string segment_type;
    if (current_segment_state_ == PathSegmentState::FORWARD_FOLLOWING ||
        current_segment_state_ == PathSegmentState::APPROACHING_CUSP) {
      segment_type = "FORWARD";
    } else if (current_segment_state_ == PathSegmentState::AT_CUSP ||
               current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING) {
      segment_type = "BACKWARD";
    } else {
      segment_type = "UNKNOWN";
    }

    std::cout << "[MY_DEBUG] [DirectionChange] Published "
              << segment_type
              << " segment to topic: " << path_publisher_->get_topic_name()
              << " with " << path_msg.points.size() << " points" << std::endl;
  }

  output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
  
  // Handle drivable area information based on segment state
  // For backward segment, filter drivable_lanes to match the actual path lane_ids
  // to avoid mismatch between path (Lanelet1498) and drivable_area (Lanelet1483)
  if ((current_segment_state_ == PathSegmentState::AT_CUSP ||
       current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING) &&
      !output.path.points.empty()) {
    // Collect lane_ids from backward segment path
    std::set<int64_t> backward_lane_ids;
    for (const auto & point : output.path.points) {
      backward_lane_ids.insert(point.lane_ids.begin(), point.lane_ids.end());
    }
    
    std::cout << "[MY_DEBUG] [DirectionChange] Backward segment lane_ids: ";
    for (const auto & id : backward_lane_ids) {
      std::cout << id << " ";
    }
    std::cout << std::endl;
    
    // Filter drivable_lanes to only include lanes present in backward segment
    auto prev_drivable_info = getPreviousModuleOutput().drivable_area_info;
    output.drivable_area_info = prev_drivable_info;  // Copy structure
    output.drivable_area_info.drivable_lanes.clear();  // Clear lanes for filtering
    
    for (const auto & drivable_lane : prev_drivable_info.drivable_lanes) {
      // Check if any lane in this DrivableLanes is in backward_lane_ids
      bool contains_backward_lane = false;
      std::vector<int64_t> drivable_lane_ids;
      
      // Collect all lane IDs from this DrivableLanes
      drivable_lane_ids.push_back(drivable_lane.right_lane.id());
      drivable_lane_ids.push_back(drivable_lane.left_lane.id());
      for (const auto & middle_lane : drivable_lane.middle_lanes) {
        drivable_lane_ids.push_back(middle_lane.id());
      }
      
      // Check if any of these IDs match backward segment
      for (const auto & id : drivable_lane_ids) {
        if (backward_lane_ids.count(id) > 0) {
          contains_backward_lane = true;
          break;
        }
      }
      
      if (contains_backward_lane) {
        output.drivable_area_info.drivable_lanes.push_back(drivable_lane);
        std::cout << "[MY_DEBUG] [DirectionChange] Keeping drivable_lane with ids: ";
        for (const auto & id : drivable_lane_ids) {
          std::cout << id << " ";
        }
        std::cout << std::endl;
      } else {
        std::cout << "[MY_DEBUG] [DirectionChange] Filtering out drivable_lane with ids: ";
        for (const auto & id : drivable_lane_ids) {
          std::cout << id << " ";
        }
        std::cout << std::endl;
      }
    }
    
    std::cout << "[MY_DEBUG] [DirectionChange] Filtered drivable_lanes count: " 
              << output.drivable_area_info.drivable_lanes.size() 
              << " (original: " << prev_drivable_info.drivable_lanes.size() << ")" << std::endl;
  } else {
    // Forward segment or no cusps: preserve drivable area information from previous module
    output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
  }

  // debug infoを出力
  if (planner_data_ && planner_data_->self_odometry) {
    const auto & ego_pose = planner_data_->self_odometry->pose.pose;

  logDirectionChangeDebugInfo(
    current_reference_path,   // ① input
    output.path,              // ② output
    ego_pose);                // ③ ego pose
  }

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
      current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING) {
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

