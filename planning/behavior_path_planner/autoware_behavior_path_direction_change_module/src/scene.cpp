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

#include "autoware/behavior_path_direction_change_module/scene.hpp"

#include "autoware/behavior_path_direction_change_module/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <rclcpp/logging.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
void logDirectionChangeDebugInfo(
  const rclcpp::Logger & logger, bool condition,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & current_reference_path,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & output_path,
  const geometry_msgs::msg::Pose & ego_pose)
{
  if (!condition) {
    return;
  }
  using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
  std::stringstream ss;

  auto print_path = [&ss](
                      const std::string & label,
                      const autoware_internal_planning_msgs::msg::PathWithLaneId & path) {
    ss << "[DirectionChange] " << label << " size=" << path.points.size() << "\n";
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto & pt = path.points[i].point;
      const double x = pt.pose.position.x;
      const double y = pt.pose.position.y;
      const double yaw = tf2::getYaw(pt.pose.orientation);
      const double yaw_deg = yaw * 180.0 / M_PI;
      const double v = pt.longitudinal_velocity_mps;
      ss << "  idx=" << i << ", x=" << x << ", y=" << y << ", yaw=" << yaw_deg << " deg"
         << ", v=" << v << " m/s\n";
    }
  };

  print_path("Current reference path (input)", current_reference_path);
  print_path("Output path (DirectionChange output)", output_path);

  ss << "[DirectionChange] Ego state:\n";
  const double ego_x = ego_pose.position.x;
  const double ego_y = ego_pose.position.y;
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  const double ego_yaw_deg = ego_yaw * 180.0 / M_PI;
  ss << "  x=" << ego_x << ", y=" << ego_y << ", yaw=" << ego_yaw_deg << " deg\n";

  RCLCPP_INFO_STREAM(logger, ss.str());
}

std::string pathSegmentStateToString(const autoware::behavior_path_planner::PathSegmentState & s)
{
  switch (s) {
    case autoware::behavior_path_planner::PathSegmentState::IDLE:
      return "IDLE";
    case autoware::behavior_path_planner::PathSegmentState::FORWARD_FOLLOWING:
      return "FORWARD_FOLLOWING";
    case autoware::behavior_path_planner::PathSegmentState::APPROACHING_CUSP:
      return "APPROACHING_CUSP";
    case autoware::behavior_path_planner::PathSegmentState::AT_CUSP:
      return "AT_CUSP";
    case autoware::behavior_path_planner::PathSegmentState::REVERSE_FOLLOWING:
      return "REVERSE_FOLLOWING";
    case autoware::behavior_path_planner::PathSegmentState::COMPLETED:
      return "COMPLETED";
    default:
      return "UNKNOWN";
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
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},  // NOLINT
  parameters_{parameters}
{
  // Create publisher for processed path with reversed orientations
  // The ~ expands to the node's namespace (behavior_path_planner)
  // Full topic will be:
  // /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/direction_change/path
  path_publisher_ = node.create_publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>(
    "~/output/direction_change/path", 1);
  RCLCPP_DEBUG(getLogger(), "Created path publisher: %s", path_publisher_->get_topic_name());
}

void DirectionChangeModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  modified_path_ = PathWithLaneId();
  cusp_point_indices_.clear();
  previous_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
  current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
  current_segment_index_ = 0;
  odometry_buffer_direction_switch_.clear();
  cusp_stopped_since_.reset();
  resetPathCandidate();
  resetPathReference();
}

void DirectionChangeModule::processOnEntry()
{
  RCLCPP_DEBUG(getLogger(), "Module entry - initializing variables");
  initVariables();
  updateData();
}

void DirectionChangeModule::processOnExit()
{
  RCLCPP_DEBUG(getLogger(), "Module exit - resetting variables");
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
  const auto previous_output = getPreviousModuleOutput();
  if (previous_output.path.points.empty()) {
    RCLCPP_WARN(getLogger(), "Previous module output path is empty. Cannot update data.");
    return;
  }

  if (planner_data_ && planner_data_->route_handler) {
    const auto centerline_path = getReferencePathFromDirectionChangeLanelets(
      previous_output.path, planner_data_->route_handler);
    if (!centerline_path.points.empty()) {
      reference_path_ = centerline_path;
      RCLCPP_DEBUG(
        getLogger(), "Using centerline from direction_change lanelets (%zu points)",
        reference_path_.points.size());
      return;
    }
  }

  // Fallback: use previous module output when not in direction_change area, or when
  // centerline from lanelets is unavailable (no route_handler, no tagged lane_ids, or empty path).
  reference_path_ = previous_output.path;
}

bool DirectionChangeModule::shouldActivateModule() const
{
  if (reference_path_.points.empty()) {
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "shouldActivateModule: Path empty, module inactive");
    return false;
  }

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
            // Activate only when in tag area and away from goal (avoid re-entry after completion).
            // Goal planner and start planner also follow similar checks for goal arrival
            if (planner_data_->self_odometry) {
              try {
                const auto goal_pose = planner_data_->route_handler->getGoalPose();
                const double dist_to_goal = autoware_utils::calc_distance2d(
                  planner_data_->self_odometry->pose.pose.position, goal_pose.position);
                if (dist_to_goal < parameters_->th_arrived_distance) {
                  RCLCPP_WARN(
                    getLogger(), "shouldActivateModule: at goal (dist=%.2f m), module INACTIVE",
                    dist_to_goal);
                  return false;
                }
              } catch (...) {
                // No goal or getGoalPose failed; allow activation
              }
            }
            RCLCPP_DEBUG_EXPRESSION(
              getLogger(), parameters_->print_debug_info,
              "shouldActivateModule: direction_change_lane tag found in lane_id=%ld, module ACTIVE",
              static_cast<int64_t>(lane_id));
            return true;  // Tag found and away from goal, activate module
          }
        } catch (...) {
          // Lanelet not found, skip
          continue;
        }
      }
    }
  }

  // No direction_change_lane tag found, module inactive
  RCLCPP_DEBUG_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "shouldActivateModule: No direction_change_lane tag found, module INACTIVE");
  return false;
}

bool DirectionChangeModule::isSustainedStoppedForDirectionSwitch()
{
  if (!planner_data_ || !planner_data_->self_odometry || !clock_) {
    return false;
  }
  const rclcpp::Time now = clock_->now();
  const double v = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  odometry_buffer_direction_switch_.emplace_back(now, v);

  const rclcpp::Duration window = rclcpp::Duration::from_seconds(parameters_->th_stopped_time);
  const rclcpp::Time cutoff = now - window;
  while (!odometry_buffer_direction_switch_.empty() &&
         rclcpp::Time(odometry_buffer_direction_switch_.front().first) < cutoff) {
    odometry_buffer_direction_switch_.pop_front();
  }

  if (odometry_buffer_direction_switch_.size() < 2u) {
    return false;
  }
  const double span_sec = (rclcpp::Time(odometry_buffer_direction_switch_.back().first) -
                           rclcpp::Time(odometry_buffer_direction_switch_.front().first))
                            .seconds();
  if (span_sec < parameters_->th_stopped_time) {
    return false;
  }
  for (const auto & entry : odometry_buffer_direction_switch_) {
    if (entry.second >= parameters_->stop_velocity_threshold) {
      return false;
    }
  }
  return true;
}

BehaviorModuleOutput DirectionChangeModule::plan()
{
  // ----- 1. Initialization -----
  // Note: updateData() is already called by SceneModuleInterface::run() before plan()
  BehaviorModuleOutput output;

  // Copy reference_path_ to local variable for stability
  const auto current_reference_path = reference_path_;

  if (!planner_data_ || !planner_data_->self_odometry) {
    RCLCPP_WARN(getLogger(), "No ego odometry available, transitioning to COMPLETED and passing through");
    current_segment_state_ = PathSegmentState::COMPLETED;
    return getPreviousModuleOutput();
  }

  // ----- 2. Cusp point detection -----
  // Detect cusp points using current_reference_path
  cusp_point_indices_ =
    detectCuspPoints(current_reference_path, parameters_->cusp_detection_angle_threshold_deg);

  RCLCPP_INFO_EXPRESSION(
    getLogger(), parameters_->print_debug_info, "plan(): path_points=%zu, cusp_points=%zu",
    reference_path_.points.size(), cusp_point_indices_.size());
  if (!cusp_point_indices_.empty() && parameters_->print_debug_info) {
    std::stringstream ss;
    ss << "Cusp indices: ";
    for (size_t i = 0; i < cusp_point_indices_.size() && i < 10; ++i) {
      ss << cusp_point_indices_[i];
      if (i < cusp_point_indices_.size() - 1 && i < 9) ss << ", ";
    }
    if (cusp_point_indices_.size() > 10) ss << "... (total: " << cusp_point_indices_.size() << ")";
    RCLCPP_DEBUG_STREAM(getLogger(), ss.str());
  }

  // Segment bounds and ego-related values for state transition (used in Section 3).
  // When cusps exist: segment bounds, nearest index, distance_to_cusp, vehicle_velocity, goal info.
  size_t c_start = 0;
  size_t c_end =
    current_reference_path.points.empty() ? 0 : current_reference_path.points.size() - 1;
  bool is_last_segment = false;
  double distance_to_cusp = 0.0;
  double vehicle_velocity = 0.0;
  bool found_nearest = false;
  double distance_to_goal = std::numeric_limits<double>::max();
  bool goal_available = false;

  if (cusp_point_indices_.empty()) {
    has_valid_cusp_ = false;
  } else {
    // Cusps present: compute segment bounds, ego nearest, distance to cusp, etc.
    const size_t num_segments = cusp_point_indices_.size() + 1u;
    if (current_segment_index_ >= num_segments) {
      current_segment_index_ = num_segments - 1u;
    }
    c_start = (current_segment_index_ == 0u) ? 0u : cusp_point_indices_[current_segment_index_ - 1u];
    c_end = (current_segment_index_ < cusp_point_indices_.size())
              ? cusp_point_indices_[current_segment_index_]
              : (current_reference_path.points.size() - 1u);
    is_last_segment = (current_segment_index_ >= cusp_point_indices_.size());

    if (c_end < current_reference_path.points.size()) {
      first_cusp_position_ = current_reference_path.points[c_end].point.pose.position;
      has_valid_cusp_ = true;
    } else {
      has_valid_cusp_ = false;
    }
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "segment_index=%zu, c_start=%zu, c_end=%zu, is_last_segment=%d", current_segment_index_,
      c_start, c_end, static_cast<int>(is_last_segment));

    if (planner_data_ && planner_data_->self_odometry) {
      const auto & ego_pose = planner_data_->self_odometry->pose.pose;
      std::vector<PathPointWithLaneId> current_segment(
        current_reference_path.points.begin() + static_cast<std::ptrdiff_t>(c_start),
        current_reference_path.points.begin() + static_cast<std::ptrdiff_t>(c_end) + 1);
      const auto ego_nearest_idx_opt = findNearestIndex(current_segment, ego_pose);
      size_t ego_nearest_idx = 0;
      if (ego_nearest_idx_opt) {
        ego_nearest_idx = *ego_nearest_idx_opt;
        if (!is_last_segment && current_segment.size() > 0) {
          const size_t segment_c_end_local = current_segment.size() - 1u;
          distance_to_cusp =
            calcSignedArcLength(current_segment, ego_nearest_idx, segment_c_end_local);
        }
        found_nearest = true;
      }
      vehicle_velocity =
        std::abs(planner_data_->self_odometry->twist.twist.linear.x);

      if (planner_data_->route_handler) {
        try {
          const auto goal_pose = planner_data_->route_handler->getGoalPose();
          distance_to_goal =
            autoware_utils::calc_distance2d(ego_pose.position, goal_pose.position);
          goal_available = true;
        } catch (...) {
          // Goal not available, continue with normal transition
        }
      }

      if (found_nearest && parameters_->print_debug_info) {
        RCLCPP_INFO_EXPRESSION(
          getLogger(), parameters_->print_debug_info,
          "state=%s, ego_nearest_idx=%zu, c_start=%zu, c_end=%zu, distance_to_cusp=%.2f, "
          "vehicle_velocity=%.2f m/s",
          pathSegmentStateToString(current_segment_state_).c_str(), ego_nearest_idx, c_start,
          c_end, distance_to_cusp, vehicle_velocity);
      }
    }
  }

  if (!cusp_point_indices_.empty() && !found_nearest) {
    RCLCPP_WARN(
      getLogger(),
      "Could not find nearest index for ego pose, transitioning to COMPLETED and passing through");
    current_segment_state_ = PathSegmentState::COMPLETED;
    return getPreviousModuleOutput();
  }
    /* Critical Safety Check: Lane Continuity with Reverse Exit
  const bool safety_check_passed = checkLaneContinuitySafety(
    reference_path_, cusp_point_indices_, planner_data_->route_handler);

  RCLCPP_DEBUG_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "Safety check result: %s", safety_check_passed ? "PASSED" : "FAILED");
  if (!safety_check_passed) {
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "FATAL: Lane continuity safety check failed. Returning path without modification.");
    output.path = reference_path_;
    output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
    output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
    return output;
  }*/

  // ----- 3. State transition -----
  previous_segment_state_ = current_segment_state_;

  {
    auto apply_no_cusp_forward = [&]() {
      current_segment_index_ = 0;
      has_valid_cusp_ = false;
      current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
    };
    auto apply_no_cusp_keep = [&]() {
      current_segment_index_ = 0;
      has_valid_cusp_ = false;
    };
    auto set_last_segment_state = [&]() {
      current_segment_state_ = (current_segment_index_ % 2 == 0)
                                 ? PathSegmentState::FORWARD_FOLLOWING
                                 : PathSegmentState::REVERSE_FOLLOWING;
    };

    switch (previous_segment_state_) {
      case PathSegmentState::IDLE:
        break;
      case PathSegmentState::FORWARD_FOLLOWING:
        if (cusp_point_indices_.empty()) {
          apply_no_cusp_forward();
          break;
        }
        if (is_last_segment) {
          set_last_segment_state();
        } else if (
          distance_to_cusp <= parameters_->cusp_detection_distance_start_approaching) {
          current_segment_state_ = PathSegmentState::APPROACHING_CUSP;
        }
        break;
      case PathSegmentState::APPROACHING_CUSP:
        if (cusp_point_indices_.empty()) {
          apply_no_cusp_forward();
          break;
        }
        if (is_last_segment) {
          set_last_segment_state();
        } else if (
          distance_to_cusp <= parameters_->cusp_detection_distance_threshold) {
          current_segment_state_ = PathSegmentState::AT_CUSP;
        }
        break;
      case PathSegmentState::AT_CUSP:
        if (cusp_point_indices_.empty()) {
          apply_no_cusp_keep();
          break;
        }
        if (is_last_segment) {
          set_last_segment_state();
        } else {
          if (vehicle_velocity < parameters_->stop_velocity_threshold) {
            if (!cusp_stopped_since_.has_value()) {
              cusp_stopped_since_ = clock_->now();
            }
            const double stopped_duration =
              (clock_->now() - cusp_stopped_since_.value()).seconds();
            if (stopped_duration >= parameters_->th_stopped_time) {
              cusp_stopped_since_.reset();
              const bool is_next_cusp_available =
                (current_segment_index_ < cusp_point_indices_.size());
              if (
                !is_next_cusp_available && goal_available &&
                distance_to_goal < parameters_->th_arrived_distance) {
                current_segment_state_ = PathSegmentState::COMPLETED;
                RCLCPP_DEBUG(getLogger(), "Transition to COMPLETED");
              } else {
                current_segment_index_++;
                set_last_segment_state();
              }
            }
          } else {
            if (cusp_stopped_since_.has_value()) {
              cusp_stopped_since_.reset();
            }
          }
        }
        break;
      case PathSegmentState::REVERSE_FOLLOWING:
        if (cusp_point_indices_.empty()) {
          apply_no_cusp_keep();
          break;
        }
        if (is_last_segment) {
          set_last_segment_state();
        } else if (
          distance_to_cusp <= parameters_->cusp_detection_distance_start_approaching) {
          current_segment_state_ = PathSegmentState::APPROACHING_CUSP;
        }
        break;
      case PathSegmentState::COMPLETED:
        break;
      default:
        RCLCPP_WARN(
          getLogger(),
          "Unexpected segment state, transitioning to COMPLETED and passing through");
        current_segment_state_ = PathSegmentState::COMPLETED;
        return getPreviousModuleOutput();
    }
  }

  if (current_segment_state_ != previous_segment_state_) {
    if (current_segment_state_ == PathSegmentState::AT_CUSP) {
      cusp_stopped_since_.reset();
    }
    c_start = (current_segment_index_ == 0u) ? 0u : cusp_point_indices_[current_segment_index_ - 1u];
    c_end = (current_segment_index_ < cusp_point_indices_.size())
              ? cusp_point_indices_[current_segment_index_]
              : (current_reference_path.points.size() - 1u);

    RCLCPP_INFO(
      getLogger(), "State transition: %s -> %s, segment_index=%zu",
      pathSegmentStateToString(previous_segment_state_).c_str(),
      pathSegmentStateToString(current_segment_state_).c_str(), current_segment_index_);
  }

  // ----- 4. Output setting per state -----
  // Output current segment [c_start, c_end]
  if (c_end >= current_reference_path.points.size()) {
    output.path = current_reference_path;
  } else {
    output.path.points.assign(
      current_reference_path.points.begin() + static_cast<std::ptrdiff_t>(c_start),
      current_reference_path.points.begin() + static_cast<std::ptrdiff_t>(c_end) + 1);
  }

  bool reversed = false;

  // TODO:do_reversalは別メソッドにする
  auto do_reversal = [&]() -> bool {

    const double max_yaw_step_rad =
      autoware_utils::deg2rad(parameters_->reverse_path_densify_max_yaw_step_deg);
    const double max_dist_step = parameters_->reverse_path_densify_max_distance_step;
    densifyPathByYawAndDistance(output.path.points, max_yaw_step_rad, max_dist_step);

    // Determines the maximum reverse speed and the low-speed start speed
    // 1. Derive the maximum allowed speed from the original forward path
    const double path_reference_speed_limit = [&]() {
      if (output.path.points.empty()) {
        // If the path is empty, fall back to the reverse speed limit parameter
        return parameters_->reverse_speed_limit;
      }
      const auto & p0 = output.path.points[0];  // Use the first point as reference
      return std::abs(static_cast<double>(p0.point.longitudinal_velocity_mps));
    }();

    // 2. Compute reverse-direction speed caps from parameters
    const double reverse_max_speed_param = parameters_->reverse_speed_limit;
    const double reverse_initial_speed_param = parameters_->reverse_initial_speed;

    // Upper bound for low-speed behavior around the cusp / reverse start
    const double reverse_cusp_speed_cap =
      std::min(reverse_initial_speed_param, reverse_max_speed_param);

    // 3. Final backward speeds, clipped by both path-based and parameter-based limits
    const double backward_cruise_speed =
      std::min(path_reference_speed_limit, reverse_max_speed_param);  // for normal reverse cruising
    const double backward_slow_speed =
      std::min(path_reference_speed_limit, reverse_cusp_speed_cap);  // for cusp / initial reverse

    for (size_t i = 0; i < output.path.points.size(); ++i) {
      auto & p = output.path.points[i];
      // Rotate yaw by π so that the pose faces backward.
      double yaw = tf2::getYaw(p.point.pose.orientation);
      yaw = autoware_utils::normalize_radian(yaw + M_PI);
      p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
      // Set a low reverse speed (negative) for the first point,
      // and a cruising reverse speed (negative) for the others.
      p.point.longitudinal_velocity_mps = (i == 0) ? -backward_slow_speed : -backward_cruise_speed;
      // If a point has multiple lane_ids, keep only the maximum lane ID.
      if (!p.lane_ids.empty() && p.lane_ids.size() > 1) {
        int64_t max_lane_id = *std::max_element(p.lane_ids.begin(), p.lane_ids.end());
        p.lane_ids = {max_lane_id};
      }
      // Set the last point's speed to 0 to make it a stop point.
      output.path.points.back().point.longitudinal_velocity_mps = 0.0;
    }
    return true;
  };

  switch (current_segment_state_) {
    case PathSegmentState::IDLE:
    case PathSegmentState::FORWARD_FOLLOWING:
      break;
    case PathSegmentState::APPROACHING_CUSP:
      if (current_segment_index_ % 2 == 1) {
        reversed = do_reversal();
      }
      break;
    case PathSegmentState::AT_CUSP:
      // When in AT_CUSP we must command stop at segment end so the vehicle stops before direction
      // switch. Otherwise the reference path velocities (e.g. from centerline) keep the vehicle
      // creeping and sustained stop is never satisfied, leading to lane departure.
      if (!output.path.points.empty()) {
        output.path.points.back().point.longitudinal_velocity_mps = 0.0;
      }
      if ((current_segment_index_ % 2 == 1) || cusp_point_indices_.empty()) {
        reversed = do_reversal();
      }
      break;
    case PathSegmentState::REVERSE_FOLLOWING:
      reversed = do_reversal();
      break;
    case PathSegmentState::COMPLETED:
      break;
    default:
      break;
  }

  RCLCPP_DEBUG_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "Publishing %s segment: %zu points (indices %zu-%zu)",
    reversed ? "REVERSE" : "FORWARD", output.path.points.size(), c_start, c_end);

  modified_path_ = output.path;

  // ----- 5. Output processing -----
  output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;

  // Preserve drivable area information from previous module
  // This is critical: PlannerManager's generateCombinedDrivableArea() needs drivable_area_info
  // (specifically drivable_lanes) to compute left_bound and right_bound. Since we only modify
  // path point orientations (yaw), not geometry, the lane information remains valid and should
  // be preserved. Without this, generateDrivableArea() fails to compute bounds and throws error:
  // "The right or left bound of drivable area is empty"
  output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;

  // ----- 6. Debug output -----
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
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info, "Published %s segment to %s with %zu points",
      reversed ? "REVERSE" : "FORWARD", path_publisher_->get_topic_name(), path_msg.points.size());

    // Debug logging for stop point analysis
    bool has_stop_point = false;
    double first_point_vel = 0.0;
    double last_point_vel = 0.0;

    size_t stop_point_index = 0;
    if (!path_msg.points.empty()) {
      first_point_vel = path_msg.points.front().point.longitudinal_velocity_mps;
      last_point_vel = path_msg.points.back().point.longitudinal_velocity_mps;

      for (size_t i = 0; i < path_msg.points.size(); ++i) {
        if (std::abs(path_msg.points[i].point.longitudinal_velocity_mps) < 1e-3) {
          has_stop_point = true;
          stop_point_index = i;
          break;
        }
      }
    }

    if (parameters_->print_debug_info) {
      std::ostringstream ss;
      ss << "Path analysis: state=" << pathSegmentStateToString(current_segment_state_)
         << " path_points=" << path_msg.points.size()
         << " has_stop_point=" << (has_stop_point ? "YES" : "NO")
         << " first_vel=" << first_point_vel << " last_vel=" << last_point_vel;
      if (has_stop_point) {
        const auto & stop_point = path_msg.points[stop_point_index].point.pose.position;
        ss << " stop_idx=" << stop_point_index << " x=" << stop_point.x << " y=" << stop_point.y;
      }
      RCLCPP_DEBUG_STREAM(getLogger(), ss.str());
    }
  }

  // Debug path/ego info
  if (planner_data_ && planner_data_->self_odometry) {
    logDirectionChangeDebugInfo(
      getLogger(), parameters_->print_debug_info, reference_path_, output.path,
      planner_data_->self_odometry->pose.pose);
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

bool DirectionChangeModule::canTransitSuccessState()
{
  // Only complete when we're on the LAST segment and ego has reached its end.
  // With multiple cusps we have multiple segments; we must not exit after the first reverse
  // segment.
  const bool is_last_segment = (current_segment_index_ >= cusp_point_indices_.size());
  if (!is_last_segment) {
    return false;
  }
  if (
    current_segment_state_ != PathSegmentState::REVERSE_FOLLOWING &&
    current_segment_state_ != PathSegmentState::FORWARD_FOLLOWING) {
    return false;
  }
  if (!planner_data_ || !planner_data_->self_odometry || modified_path_.points.empty()) {
    return false;
  }
  const auto & ego_pose = planner_data_->self_odometry->pose.pose;
  const auto ego_nearest_idx_opt = findNearestIndex(modified_path_.points, ego_pose);
  if (!ego_nearest_idx_opt || *ego_nearest_idx_opt >= modified_path_.points.size()) {
    return false;
  }
  const size_t ego_nearest_idx = *ego_nearest_idx_opt;
  const double remaining_distance =
    calcSignedArcLength(modified_path_.points, ego_nearest_idx, modified_path_.points.size() - 1);

  if (remaining_distance < parameters_->th_arrived_distance) {
    current_segment_state_ = PathSegmentState::COMPLETED;
    RCLCPP_DEBUG(
      getLogger(), "Ego completed last segment, state=COMPLETED, module can transit to SUCCESS");
    return true;
  }
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
