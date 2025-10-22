// Copyright 2020 Tier IV, Inc.
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

#include "scene.hpp"

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <cstring>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware::motion_utils::calcLongitudinalOffsetPose;
using autoware::motion_utils::calcSignedArcLength;

DetectionAreaModule::DetectionAreaModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::DetectionArea & detection_area_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  detection_area_reg_elem_(detection_area_reg_elem),
  state_(State::GO),
  planner_param_(planner_param),
  debug_data_()
{
}

void DetectionAreaModule::print_detected_obstacle(
  const std::vector<geometry_msgs::msg::Point> & obstacle_points,
  const geometry_msgs::msg::Pose & self_pose) const
{
  std::stringstream obstacles_ss;
  if (last_obstacle_found_time_) {
    rclcpp::Duration time_from_last_detection = clock_->now() - *last_obstacle_found_time_;
    constexpr double obstacle_disappear_time_threshold = 0.1;
    if (time_from_last_detection.seconds() > obstacle_disappear_time_threshold) {
      obstacles_ss << std::fixed << std::setprecision(2) << time_from_last_detection.seconds()
                   << "[s] since last obstacle cleared";
    }
  }

  if (!obstacle_points.empty()) {
    const auto p = obstacle_points[0];
    const auto p_from_ego = autoware_utils::inverse_transform_point(p, self_pose);
    obstacles_ss << "obstacle: map_coord(" << std::fixed << std::setprecision(2) << p.x << ", "
                 << p.y << ", " << p.z << "), ego_coord(" << p_from_ego.x << ", " << p_from_ego.y
                 << ", " << p_from_ego.z << ")";
  }

  logInfoThrottle(
    1000, "ego: map_coord(%.2f, %.2f, %.2f), %s", self_pose.position.x, self_pose.position.y,
    self_pose.position.z, obstacles_ss.str().c_str());
}

bool DetectionAreaModule::modifyPathVelocity(PathWithLaneId * _path)
{
  auto path = experimental::trajectory::pretty_build(_path->points);
  if (!path) {
    return false;
  }
  const auto & left_bound = _path->left_bound;
  const auto & right_bound = _path->right_bound;

  // Store original path
  const auto original_path = *path;

  // Reset data
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  // Find obstacles in detection area
  const auto obstacle_points = detection_area::get_obstacle_points(
    detection_area_reg_elem_.detectionAreas(), *planner_data_->no_ground_pointcloud);
  debug_data_.obstacle_points = obstacle_points;
  if (!obstacle_points.empty()) {
    last_obstacle_found_time_ = std::make_shared<const rclcpp::Time>(clock_->now());
  }

  // Get stop line geometry
  const auto stop_line =
    detection_area::get_stop_line(detection_area_reg_elem_, left_bound, right_bound);

  // Get self pose
  const auto & self_pose = planner_data_->current_odometry->pose;
  const auto self_s = experimental::trajectory::find_first_nearest_index(
    *path, self_pose, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);
  if (!self_s) {
    return false;
  }

  // Get current lanelet and connected lanelets
  const auto connected_lane_ids =
    planning_utils::collectConnectedLaneIds(lane_id_, planner_data_->route_handler_);
  // Get stop point
  const auto stop_point_s = detection_area::get_stop_point(
    original_path, stop_line, planner_param_.stop_margin,
    planner_data_->vehicle_info_.max_longitudinal_offset_m - forward_offset_to_stop_line_,
    connected_lane_ids);
  if (!stop_point_s) {
    return true;
  }

  auto modified_stop_point_s = *stop_point_s;

  const auto is_stopped = planner_data_->isVehicleStopped(0.0);
  const auto dist_from_self_to_stop = *stop_point_s - *self_s;

  // Don't re-approach when the ego stops closer to the stop point than hold_stop_margin_distance
  if (is_stopped && dist_from_self_to_stop < planner_param_.hold_stop_margin_distance) {
    modified_stop_point_s = *self_s;
  }

  setDistance(dist_from_self_to_stop);

  // Check state
  const auto prev_state = state_;  // used to log the state change

  setSafe(
    detection_area::can_clear_stop_state(
      last_obstacle_found_time_, clock_->now(), planner_param_.state_clear_time));
  if (isActivated()) {
    last_obstacle_found_time_ = {};
    if (!planner_param_.suppress_pass_judge_when_stopping || !is_stopped) {
      state_ = State::GO;
      if (prev_state != State::GO) {
        logInfo("state changed: STOP -> GO");
      }
    }
    return true;
  }

  // Force ignore objects after dead_line
  if (planner_param_.use_dead_line) {
    // Use '-' for margin because it's the backward distance from stop line
    const auto dead_line_point_s = detection_area::get_stop_point(
      original_path, stop_line, -planner_param_.dead_line_margin,
      planner_data_->vehicle_info_.max_longitudinal_offset_m, connected_lane_ids);

    if (dead_line_point_s) {
      debug_data_.dead_line_poses.push_back(path->compute(*dead_line_point_s).point.pose);

      const double dist_from_self_to_dead_line = *dead_line_point_s - *self_s;
      if (dist_from_self_to_dead_line < 0.0) {
        logWarn("vehicle is over dead line");
        setSafe(true);
        return true;
      }
    }
  }

  // Ignore objects detected after stop_line if not in STOP state
  if (
    state_ != State::STOP &&
    dist_from_self_to_stop < -planner_param_.distance_to_judge_over_stop_line) {
    setSafe(true);
    return true;
  }

  // Ignore objects if braking distance is not enough
  if (planner_param_.use_pass_judge_line) {
    const auto current_velocity = planner_data_->current_velocity->twist.linear.x;
    const double pass_judge_line_distance = planning_utils::calcJudgeLineDistWithAccLimit(
      current_velocity, planner_data_->max_stop_acceleration_threshold,
      planner_data_->delay_response_time);
    if (
      state_ != State::STOP &&
      !detection_area::has_enough_braking_distance(
        *self_s, *stop_point_s, pass_judge_line_distance, current_velocity)) {
      logWarnThrottle(1000, "[detection_area] vehicle is over stop border");
      setSafe(true);
      return true;
    }
  }

  // Insert stop point
  state_ = State::STOP;
  if (prev_state != State::STOP) {
    if (planner_param_.use_max_acceleration) {
      forward_offset_to_stop_line_ = std::max(
        detection_area::feasible_stop_distance_by_max_acceleration(
          planner_data_->current_velocity->twist.linear.x, planner_param_.max_acceleration) -
          dist_from_self_to_stop,
        0.0);

      modified_stop_point_s += forward_offset_to_stop_line_;
    }
    logInfo("state changed: GO -> STOP");
  }

  if (state_ == State::STOP && planner_param_.enable_detected_obstacle_logging) {
    print_detected_obstacle(obstacle_points, self_pose);
  }

  path->longitudinal_velocity_mps().range(modified_stop_point_s, path->length()).set(0.0);

  // For virtual wall
  const auto stop_pose = path->compute(*stop_point_s).point.pose;
  debug_data_.stop_poses.push_back(stop_pose);

  // Create StopReason
  {
    planning_factor_interface_->add(
      path->restore(), planner_data_->current_odometry->pose, stop_pose,
      autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
      0.0 /*shift distance*/, "");
  }

  return true;
}
}  // namespace autoware::behavior_velocity_planner
