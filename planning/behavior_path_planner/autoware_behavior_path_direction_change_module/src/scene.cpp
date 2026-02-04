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
  status_ = CuspReverseLaneFollowingStatus::IDLE;
  first_cusp_index_ = 0;
  first_cusp_position_ = std::nullopt;
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

// ============================================================================
// Path Evaluation Functions (following start_planner_module pattern)
// ============================================================================

bool DirectionChangeModule::checkDirectionChangeAreaTag(
  PathEvaluationDebugData & evaluation_data) const
{
  if (!planner_data_->route_handler) {
    evaluation_data.conditions_evaluation.emplace_back("route_handler not available");
    evaluation_data.has_direction_change_area_tag = false;
    return false;
  }

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
          evaluation_data.conditions_evaluation.emplace_back(
            "direction_change_area tag found (lane_id=" + std::to_string(lane_id) + ")");
          evaluation_data.has_direction_change_area_tag = true;
          return true;
        }
      } catch (...) {
        // Lanelet not found, skip
        continue;
      }
    }
  }

  evaluation_data.conditions_evaluation.emplace_back("no direction_change_area tag found");
  evaluation_data.has_direction_change_area_tag = false;
  return false;
}

bool DirectionChangeModule::detectAndValidateCuspPoints(PathEvaluationDebugData & evaluation_data)
{
  if (!parameters_->enable_cusp_detection) {
    evaluation_data.conditions_evaluation.emplace_back("cusp detection disabled");
    evaluation_data.num_cusp_points = 0;
    return true;  // Not an error, just disabled
  }

  cusp_point_indices_ = detectCuspPoints(reference_path_, parameters_->cusp_detection_angle_threshold_deg);
  evaluation_data.num_cusp_points = cusp_point_indices_.size();

  if (cusp_point_indices_.empty()) {
    evaluation_data.conditions_evaluation.emplace_back("no cusp points detected");
    return true;  // Not an error, module can still operate in forward-only mode
  }

  evaluation_data.first_cusp_index = cusp_point_indices_[0];

  std::ostringstream oss;
  oss << "cusp points detected: " << cusp_point_indices_.size() << " (indices: ";
  for (size_t i = 0; i < std::min<size_t>(cusp_point_indices_.size(), 5); ++i) {
    if (i > 0) oss << ", ";
    oss << cusp_point_indices_[i];
  }
  if (cusp_point_indices_.size() > 5) oss << ", ...";
  oss << ")";
  evaluation_data.conditions_evaluation.emplace_back(oss.str());

  return true;
}

bool DirectionChangeModule::checkLaneContinuitySafetyWithEvaluation(
  PathEvaluationDebugData & evaluation_data) const
{
  if (cusp_point_indices_.empty()) {
    evaluation_data.conditions_evaluation.emplace_back("lane continuity check: skipped (no cusps)");
    evaluation_data.lane_continuity_check_passed = true;
    return true;
  }

  const bool safety_check_passed =
    checkLaneContinuitySafety(reference_path_, cusp_point_indices_, planner_data_->route_handler);

  if (safety_check_passed) {
    evaluation_data.conditions_evaluation.emplace_back("lane continuity check: passed");
    evaluation_data.lane_continuity_check_passed = true;
  } else {
    evaluation_data.conditions_evaluation.emplace_back("lane continuity check: FAILED");
    evaluation_data.lane_continuity_check_passed = false;
  }

  return safety_check_passed;
}

bool DirectionChangeModule::evaluatePath(PathEvaluationDebugData & evaluation_data) const
{
  // Clear previous evaluation data
  evaluation_data.clear();

  // Step 1: Check path existence
  if (reference_path_.points.empty()) {
    evaluation_data.conditions_evaluation.emplace_back("path is empty");
    std::cout << "[MY_DEBUG] [DirectionChange] evaluatePath: " << evaluation_data.to_evaluation_table() << std::endl;
    return false;
  }
  evaluation_data.conditions_evaluation.emplace_back(
    "path exists (" + std::to_string(reference_path_.points.size()) + " points)");

  // Step 2: Check direction_change_area tag
  if (!checkDirectionChangeAreaTag(evaluation_data)) {
    std::cout << "[MY_DEBUG] [DirectionChange] evaluatePath: " << evaluation_data.to_evaluation_table() << std::endl;
    return false;
  }

  // Note: Cusp detection and lane continuity checks are done in plan() for expensive operations
  // evaluatePath() determines if module should activate based on map tags

  evaluation_data.conditions_evaluation.emplace_back("module activation: approved");
  std::cout << "[MY_DEBUG] [DirectionChange] evaluatePath: " << evaluation_data.to_evaluation_table() << std::endl;
  return true;
}

bool DirectionChangeModule::shouldActivateModule() const
{
  // Delegate to evaluatePath with mutable debug_data_
  PathEvaluationDebugData temp_evaluation_data;
  const bool result = evaluatePath(temp_evaluation_data);

  // Store evaluation results in debug_data_ for visualization
  debug_data_.evaluation_data = temp_evaluation_data;

  return result;
}


BehaviorModuleOutput DirectionChangeModule::plan()
{
  // Note: updateData() is already called by SceneModuleInterface::run() before plan()
  // Following standard BPP module pattern: expensive computations happen in plan()

  BehaviorModuleOutput output;

  // Clear previous evaluation data and start fresh evaluation for plan()
  debug_data_.evaluation_data.clear();
  debug_data_.evaluation_data.conditions_evaluation.emplace_back("plan() started");

  // Step 1: Detect and validate cusp points (expensive work done here, not in updateData())
  if (!detectAndValidateCuspPoints(debug_data_.evaluation_data)) {
    std::cout << "[MY_DEBUG] [DirectionChange] plan() cusp detection failed: "
              << debug_data_.evaluation_data.to_evaluation_table() << std::endl;
    output.path = reference_path_;
    output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
    output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
    return output;
  }

  std::cout << "[MY_DEBUG] [DirectionChange] plan() called: Path points="
            << reference_path_.points.size()
            << ", Cusp points detected=" << cusp_point_indices_.size() << std::endl;

  // Strategy: Separate forward/backward path publishing
  // - If no cusps: return full path as forward segment
  // - If cusps exist: split at first cusp and publish forward/backward segments separately
  //   This prevents downstream modules from seeing mixed orientations
  
  if (cusp_point_indices_.empty()) {
    // No cusps detected in current reference_path
    // IMPORTANT: If we're already in REVERSE_FOLLOWING state, maintain that state
    // This prevents oscillation when cusp detection becomes unstable after passing cusp
    if (status_ == CuspReverseLaneFollowingStatus::REVERSE_FOLLOWING) {
      std::cout << "[MY_DEBUG] [DirectionChange] No cusp detected but already in REVERSE_FOLLOWING state, maintaining backward path" << std::endl;
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
      status_ = CuspReverseLaneFollowingStatus::FORWARD_FOLLOWING;
    }
  } else {
    // Cusps detected: implement separate forward/backward publishing strategy
    first_cusp_index_ = cusp_point_indices_[0];
    debug_data_.evaluation_data.first_cusp_index = first_cusp_index_;
    
    // Store the actual cusp point position (world coordinates) on first detection
    // This is critical because reference_path_ indices change every cycle as ego moves
    if (!first_cusp_position_.has_value() && first_cusp_index_ < reference_path_.points.size()) {
      first_cusp_position_ = reference_path_.points[first_cusp_index_].point.pose.position;
      std::cout << "[MY_DEBUG] [DirectionChange] Stored cusp position: ("
                << first_cusp_position_->x << ", " << first_cusp_position_->y << ")" << std::endl;
    }

    std::cout << "[MY_DEBUG] [DirectionChange] First cusp at index: " << first_cusp_index_
              << std::endl;

    // Step 2: Critical Safety Check - Lane Continuity with Reverse Exit
    // (following start_planner pattern: record evaluation result)
    if (!checkLaneContinuitySafetyWithEvaluation(debug_data_.evaluation_data)) {
      std::cout << "[MY_DEBUG] [DirectionChange] FATAL: Lane continuity safety check failed.\n"
                << debug_data_.evaluation_data.to_evaluation_table() << std::endl;
      debug_data_.evaluation_data.conditions_evaluation.emplace_back(
        "plan() aborted: safety check failed");
      output.path = reference_path_;
      output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
      output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
      return output;
    }
    
    // ========== STATE MACHINE FOR CUSP-BASED REVERSE LANE FOLLOWING ==========
    // Get ego's current state
    double distance_to_cusp = std::numeric_limits<double>::max();
    double ego_velocity = 0.0;
    geometry_msgs::msg::Pose ego_pose;
    
    if (planner_data_ && planner_data_->self_odometry) {
      ego_pose = planner_data_->self_odometry->pose.pose;
      ego_velocity = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
      
      if (first_cusp_position_.has_value()) {
        distance_to_cusp = calc_distance2d(ego_pose.position, *first_cusp_position_);
      }
    }
    
    // Initialize status if IDLE
    if (status_ == CuspReverseLaneFollowingStatus::IDLE) {
      status_ = CuspReverseLaneFollowingStatus::FORWARD_FOLLOWING;
    }
    
    // State transition logic
    CuspReverseLaneFollowingStatus prev_status = status_;
    
    switch (status_) {
      case CuspReverseLaneFollowingStatus::FORWARD_FOLLOWING:
        // Transition to APPROACHING_CUSP when within approach distance
        if (distance_to_cusp <= kApproachingCuspDistance) {
          status_ = CuspReverseLaneFollowingStatus::APPROACHING_CUSP;
        }
        break;
        
      case CuspReverseLaneFollowingStatus::APPROACHING_CUSP:
        // Transition to AT_CUSP when close enough AND stopped
        {
          const bool distance_ok = (distance_to_cusp <= kAtCuspDistance);
          const bool velocity_ok = (ego_velocity < kStoppedVelocityThreshold);
          std::cout << "[MY_DEBUG] [DirectionChange] APPROACHING_CUSP check: "
                    << "distance_ok=" << (distance_ok ? "true" : "false")
                    << " (" << distance_to_cusp << " <= " << kAtCuspDistance << ")"
                    << ", velocity_ok=" << (velocity_ok ? "true" : "false")
                    << " (" << ego_velocity << " < " << kStoppedVelocityThreshold << ")"
                    << std::endl;
          if (distance_ok && velocity_ok) {
            status_ = CuspReverseLaneFollowingStatus::AT_CUSP;
          }
        }
        break;
        
      case CuspReverseLaneFollowingStatus::AT_CUSP:
        // Immediately transition to REVERSE_FOLLOWING
        status_ = CuspReverseLaneFollowingStatus::REVERSE_FOLLOWING;
        break;
        
      case CuspReverseLaneFollowingStatus::REVERSE_FOLLOWING:
        // Stay in REVERSE_FOLLOWING until completed
        // (Completion logic can be added later)
        break;
        
      case CuspReverseLaneFollowingStatus::COMPLETED:
        // Stay completed
        break;
        
      default:
        break;
    }
    
    // Log state transition
    if (prev_status != status_) {
      std::cout << "[MY_DEBUG] [DirectionChange] State transition: " 
                << toString(prev_status) << " -> " << toString(status_) << std::endl;
    }
    
    std::cout << "[MY_DEBUG] [DirectionChange] "
              << "status=" << toString(status_)
              << ", ego_pos=(" << ego_pose.position.x << ", " << ego_pose.position.y << ")"
              << ", ego_vel=" << ego_velocity << "m/s"
              << ", cusp_pos=(" << (first_cusp_position_.has_value() ? 
                 std::to_string(first_cusp_position_->x) + ", " + std::to_string(first_cusp_position_->y) : "N/A") << ")"
              << ", distance_to_cusp=" << distance_to_cusp << "m"
              << std::endl;
    
    // ========== PATH GENERATION BASED ON STATUS ==========
    switch (status_) {
      case CuspReverseLaneFollowingStatus::FORWARD_FOLLOWING:
      case CuspReverseLaneFollowingStatus::APPROACHING_CUSP:
      {
        // Generate forward path to CUSP point
        if (first_cusp_index_ < reference_path_.points.size() && planner_data_ && planner_data_->self_odometry) {
          // Create forward segment [0, first_cusp_index]
          PathWithLaneId forward_segment;
          forward_segment.points.assign(
            reference_path_.points.begin(),
            reference_path_.points.begin() + first_cusp_index_ + 1);
          
          // Find ego's nearest point on the forward segment
          const auto ego_nearest_idx_opt = findNearestIndex(forward_segment.points, ego_pose);
          size_t start_idx = 0;
          if (ego_nearest_idx_opt) {
            start_idx = *ego_nearest_idx_opt;
          }
          
          // Build output path from start_idx to cusp
          output.path.points.clear();
          for (size_t i = start_idx; i < forward_segment.points.size(); ++i) {
            output.path.points.push_back(forward_segment.points[i]);
          }
          
          // Set velocity based on status
          // Strategy: Keep low velocity at CUSP point, set stop point AFTER cusp
          // This allows the vehicle to reach closer to the actual CUSP point
          const double forward_velocity = parameters_->cusp_approach_speed;
          const double cusp_creep_velocity = 0.3;  // [m/s] Low velocity to creep towards cusp
          const double stop_distance_after_cusp = 0.5;  // [m] Distance after cusp to place stop point
          
          for (size_t i = 0; i < output.path.points.size(); ++i) {
            // All points including CUSP get low velocity (not zero)
            output.path.points[i].point.longitudinal_velocity_mps = 
              (i < output.path.points.size() - 1) ? forward_velocity : cusp_creep_velocity;
          }
          
          // Calculate path direction at cusp for extension
          double current_length = 0.0;
          for (size_t i = 1; i < output.path.points.size(); ++i) {
            current_length += autoware_utils::calc_distance2d(
              output.path.points[i-1].point.pose.position,
              output.path.points[i].point.pose.position);
          }
          
          if (output.path.points.size() >= 2) {
            const auto & cusp_point = output.path.points.back();
            const auto & prev_point = output.path.points[output.path.points.size() - 2];
            
            const double dx_raw = cusp_point.point.pose.position.x - prev_point.point.pose.position.x;
            const double dy_raw = cusp_point.point.pose.position.y - prev_point.point.pose.position.y;
            const double dist = std::hypot(dx_raw, dy_raw);
            const double dx = (dist > 0.01) ? dx_raw / dist : 1.0;
            const double dy = (dist > 0.01) ? dy_raw / dist : 0.0;
            
            // Add stop point after cusp (this is where motion planner will try to stop)
            PathPointWithLaneId stop_point = cusp_point;
            stop_point.point.pose.position.x += dx * stop_distance_after_cusp;
            stop_point.point.pose.position.y += dy * stop_distance_after_cusp;
            stop_point.point.longitudinal_velocity_mps = 0.0;
            output.path.points.push_back(stop_point);
            
            // Add extra points to meet minimum path length
            const double min_path_length = 10.0;
            const double total_length = current_length + stop_distance_after_cusp;
            if (total_length < min_path_length) {
              const int num_extra_points = static_cast<int>((min_path_length - total_length) / 1.0) + 1;
              for (int i = 1; i <= num_extra_points; ++i) {
                PathPointWithLaneId extra_point = stop_point;
                extra_point.point.pose.position.x += dx * i * 1.0;
                extra_point.point.pose.position.y += dy * i * 1.0;
                extra_point.point.longitudinal_velocity_mps = 0.0;
                output.path.points.push_back(extra_point);
              }
            }
          }
          
          std::cout << "[MY_DEBUG] [DirectionChange] Publishing FORWARD path: " 
                    << output.path.points.size() << " points (start_idx=" << start_idx
                    << ", cusp_idx=" << first_cusp_index_ << ", path_length=" << current_length << "m)" << std::endl;
        } else {
          output.path = reference_path_;
        }
        break;
      }
      
      case CuspReverseLaneFollowingStatus::AT_CUSP:
      {
        // Generate stop path at CUSP (all velocities = 0)
        if (first_cusp_index_ < reference_path_.points.size()) {
          // Just output a single point at cusp with velocity 0
          output.path.points.clear();
          output.path.points.push_back(reference_path_.points[first_cusp_index_]);
          output.path.points.back().point.longitudinal_velocity_mps = 0.0;
          
          std::cout << "[MY_DEBUG] [DirectionChange] AT_CUSP: Outputting stop path" << std::endl;
        }
        break;
      }
      
      case CuspReverseLaneFollowingStatus::REVERSE_FOLLOWING:
      {
        // Generate backward path from CUSP point
        if (first_cusp_index_ < reference_path_.points.size() && planner_data_ && planner_data_->self_odometry) {
          // Extract backward segment: [first_cusp_index, end]
          PathWithLaneId backward_segment;
          backward_segment.points.assign(
            reference_path_.points.begin() + first_cusp_index_,
            reference_path_.points.end());
          
          // Find ego's nearest point on the backward segment
          const auto ego_nearest_idx_opt = findNearestIndex(backward_segment.points, ego_pose);
          size_t start_idx = 0;
          if (ego_nearest_idx_opt) {
            start_idx = *ego_nearest_idx_opt;
          }
          
          // Build output path from start_idx to end
          output.path.points.clear();
          for (size_t i = start_idx; i < backward_segment.points.size(); ++i) {
            output.path.points.push_back(backward_segment.points[i]);
          }
          
          // Reverse orientations and set negative velocity for backward driving
          const double reverse_velocity = parameters_->reverse_speed_limit;  // e.g., -2.0 m/s
          for (auto & p : output.path.points) {
            double yaw = tf2::getYaw(p.point.pose.orientation);
            yaw = autoware_utils::normalize_radian(yaw + M_PI);
            p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
            p.point.longitudinal_velocity_mps = -std::abs(reverse_velocity);
          }
          
          std::cout << "[MY_DEBUG] [DirectionChange] Publishing BACKWARD path: " 
                    << output.path.points.size() << " points (start_idx=" << start_idx << ")" << std::endl;
        } else {
          output.path = reference_path_;
        }
        break;
      }
      
      case CuspReverseLaneFollowingStatus::COMPLETED:
      case CuspReverseLaneFollowingStatus::IDLE:
      default:
        output.path = reference_path_;
        break;
    }
    
    modified_path_ = output.path;
    if (!output.path.points.empty() && planner_data_ && planner_data_->self_odometry) {
      // Calculate total path length
      double total_length = 0.0;
      for (size_t i = 1; i < output.path.points.size(); ++i) {
        total_length += calc_distance2d(
          output.path.points[i-1].point.pose.position,
          output.path.points[i].point.pose.position);
      }
      
      // Show first point, last point, and path length
      const auto & ego_pos = planner_data_->self_odometry->pose.pose.position;
      const auto & first_pt = output.path.points.front().point;
      const auto & last_pt = output.path.points.back().point;
      const double dist_ego_to_first = calc_distance2d(ego_pos, first_pt.pose.position);
      
      std::cout << "[MY_DEBUG] [DirectionChange] Output path: " 
                << output.path.points.size() << " points, total_length=" << total_length << "m" << std::endl;
      std::cout << "  ego:   (" << ego_pos.x << ", " << ego_pos.y << ")" << std::endl;
      std::cout << "  first: (" << first_pt.pose.position.x << ", " << first_pt.pose.position.y 
                << "), v=" << first_pt.longitudinal_velocity_mps << " m/s, dist_from_ego=" << dist_ego_to_first << "m" << std::endl;
      std::cout << "  last:  (" << last_pt.pose.position.x << ", " << last_pt.pose.position.y 
                << "), v=" << last_pt.longitudinal_velocity_mps << " m/s" << std::endl;
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
              << toString(status_)
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

  // Final evaluation record
  debug_data_.evaluation_data.conditions_evaluation.emplace_back(
    "plan() completed: " +
    toString(status_) +
    " segment with " + std::to_string(output.path.points.size()) + " points");

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
      status_ == CuspReverseLaneFollowingStatus::REVERSE_FOLLOWING) {
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

  // Note: evaluation_data is already updated during evaluatePath() and plan()
  // Output evaluation table for debugging if debug marker publishing is enabled
  if (parameters_->publish_debug_marker && !debug_data_.evaluation_data.conditions_evaluation.empty()) {
    std::cout << "[MY_DEBUG] [DirectionChange] Evaluation Summary:\n"
              << debug_data_.evaluation_data.to_evaluation_table() << std::endl;
  }
}

}  // namespace autoware::behavior_path_planner

