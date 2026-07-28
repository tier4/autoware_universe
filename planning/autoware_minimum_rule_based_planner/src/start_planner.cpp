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

#include "start_planner.hpp"

#include "path_planner.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/distance.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

StartPlanner::StartPlanner(const rclcpp::Logger & logger, const VehicleInfo & vehicle_info)
: logger_(logger), vehicle_info_(vehicle_info)
{
}

StartPlannerResult StartPlanner::plan(
  const Trajectory & reference_trajectory, const geometry_msgs::msg::Pose & ego_pose,
  const double ego_velocity, const PredictedObjects::ConstSharedPtr & objects,
  const ShiftCandidateGenerator & generate_candidate, const StartPlannerParams & params)
{
  StartPlannerResult result;

  if (reference_trajectory.points.size() < 2) {
    pull_out_active_ = false;
    return result;
  }

  const double lateral_offset =
    autoware::motion_utils::calcLateralOffset(reference_trajectory.points, ego_pose.position);
  const double abs_offset = std::abs(lateral_offset);

  if (pull_out_active_) {
    if (abs_offset < params.finish_lateral_offset) {
      pull_out_active_ = false;  // merged back onto the reference path
      return result;
    }
  } else {
    const bool is_stopped = std::abs(ego_velocity) < params.ego_stopped_velocity;
    if (!is_stopped || abs_offset < params.activation_lateral_offset) {
      return result;
    }
    pull_out_active_ = true;
    RCLCPP_INFO(
      logger_, "Start planner activated: lateral offset %.2f m, ego velocity %.2f m/s",
      lateral_offset, ego_velocity);
  }

  // ego is left of the reference path -> shifting right, and vice versa
  const uint8_t turn_signal =
    lateral_offset > 0.0 ? TurnIndicatorsCommand::ENABLE_RIGHT : TurnIndicatorsCommand::ENABLE_LEFT;
  result.turn_indicators_command = turn_signal;

  const auto object_polygons = objects
                                 ? start_planner_utils::get_static_object_polygons(
                                     *objects, ego_pose.position, params.object_velocity_threshold,
                                     params.object_search_radius)
                                 : std::vector<autoware_utils::Polygon2d>{};

  const int sampling_num = std::max(params.lateral_accel_sampling_num, 1);
  const double accel_resolution = std::max(
    (params.maximum_lateral_accel - params.minimum_lateral_accel) / sampling_num,
    std::numeric_limits<double>::epsilon());
  const double clamped_velocity =
    std::max(std::max(0.0, ego_velocity), params.min_speed_for_curvature);

  for (const double margin : params.collision_check_margins) {
    for (double lateral_accel = params.minimum_lateral_accel;
         lateral_accel <= params.maximum_lateral_accel + std::numeric_limits<double>::epsilon();
         lateral_accel += accel_resolution) {
      const auto candidate = generate_candidate(lateral_accel);
      if (!candidate || candidate->points.size() < 2) {
        continue;
      }

      // check the shift section (same length formula as the candidate generation) plus some extra
      const double check_length =
        utils::compute_shift_length_from_lateral_accel(
          abs_offset, clamped_velocity, lateral_accel, params.minimum_shift_distance) +
        params.collision_check_extra_length;

      if (start_planner_utils::has_collision(
            candidate->points, check_length, vehicle_info_, object_polygons, margin)) {
        continue;
      }

      result.status = StartPlannerResult::Status::PLANNED;
      result.trajectory = *candidate;
      result.selected_lateral_accel = lateral_accel;
      result.selected_margin = margin;
      return result;
    }
  }

  RCLCPP_WARN_THROTTLE(
    logger_, clock_, 5000,
    "Start planner: no collision-free pull out candidate found, keeping stopped.");
  result.status = StartPlannerResult::Status::BLOCKED;
  return result;
}

namespace start_planner_utils
{

std::vector<autoware_utils::Polygon2d> get_static_object_polygons(
  const PredictedObjects & objects, const geometry_msgs::msg::Point & ego_position,
  const double velocity_threshold, const double search_radius)
{
  std::vector<autoware_utils::Polygon2d> polygons;
  for (const auto & object : objects.objects) {
    const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
    const double speed = std::hypot(twist.linear.x, twist.linear.y);
    if (speed > velocity_threshold) {
      continue;
    }
    const auto & position = object.kinematics.initial_pose_with_covariance.pose.position;
    if (autoware_utils::calc_distance2d(position, ego_position) > search_radius) {
      continue;
    }
    polygons.push_back(autoware_utils::to_polygon2d(object));
  }
  return polygons;
}

bool has_collision(
  const TrajectoryPoints & points, const double check_length, const VehicleInfo & vehicle_info,
  const std::vector<autoware_utils::Polygon2d> & object_polygons, const double margin)
{
  return has_collision(points, 0.0, check_length, vehicle_info, object_polygons, margin);
}

bool has_collision(
  const TrajectoryPoints & points, const double check_start_length, const double check_end_length,
  const VehicleInfo & vehicle_info, const std::vector<autoware_utils::Polygon2d> & object_polygons,
  const double margin)
{
  if (object_polygons.empty()) {
    return false;
  }

  double accumulated_length = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      accumulated_length +=
        autoware_utils::calc_distance2d(points.at(i - 1).pose.position, points.at(i).pose.position);
      if (accumulated_length > check_end_length) {
        break;
      }
    }
    if (accumulated_length < check_start_length) {
      continue;
    }

    const auto footprint = autoware_utils::to_footprint(
      points.at(i).pose, vehicle_info.max_longitudinal_offset_m, vehicle_info.rear_overhang_m,
      vehicle_info.vehicle_width_m);
    for (const auto & object_polygon : object_polygons) {
      if (boost::geometry::distance(footprint, object_polygon) < margin) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace start_planner_utils
}  // namespace autoware::minimum_rule_based_planner
