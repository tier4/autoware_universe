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

#include "goal_planner.hpp"

#include "path_planner.hpp"
#include "start_planner.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

namespace
{

// [m] a goal projecting farther beyond the trajectory end is not on this trajectory
constexpr double goal_beyond_end_tolerance = 2.0;

// Quintic smoothstep lateral profile for the pull over: y(t) rises 0 -> 1 with
// y'(0) = y''(0) = y'(1) = y''(1) = 0 (same polynomial family as the ego shift).
double smoothstep5(const double t)
{
  return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t);
}

double smoothstep5_derivative(const double t)
{
  return 30.0 * t * t * (1.0 - 2.0 * t + t * t);
}

double total_arc_length(const TrajectoryPoints & points)
{
  double length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    length +=
      autoware_utils::calc_distance2d(points.at(i - 1).pose.position, points.at(i).pose.position);
  }
  return length;
}

}  // namespace

GoalPlanner::GoalPlanner(const rclcpp::Logger & logger, const VehicleInfo & vehicle_info)
: logger_(logger), vehicle_info_(vehicle_info)
{
}

GoalPlannerResult GoalPlanner::plan(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Pose & ego_pose, const PredictedObjects::ConstSharedPtr & objects,
  const GoalPlannerParams & params) const
{
  GoalPlannerResult result;

  const auto & points = trajectory.points;
  if (points.size() < 2) {
    return result;
  }

  // A goal projecting beyond the trajectory end (e.g. more than the remaining route) is not
  // handled by this planner; the trajectory simply continues and the goal is approached later.
  const size_t last_seg = points.size() - 2;
  const double beyond_end =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(points, last_seg, goal_pose.position) -
    autoware_utils::calc_distance2d(
      points.at(last_seg).pose.position, points.at(last_seg + 1).pose.position);
  if (beyond_end > goal_beyond_end_tolerance) {
    return result;
  }

  const double d_goal = autoware::motion_utils::calcLateralOffset(points, goal_pose.position);
  if (std::abs(d_goal) < params.activation_lateral_offset) {
    return result;  // goal is (nearly) on the path: normal arrival, early stop fallback (~0 m)
  }

  const double s_goal = std::clamp(
    autoware::motion_utils::calcSignedArcLength(
      points, points.front().pose.position, goal_pose.position),
    0.0, total_arc_length(points));

  const auto object_polygons = objects
                                 ? start_planner_utils::get_static_object_polygons(
                                     *objects, goal_pose.position, params.object_velocity_threshold,
                                     params.object_search_radius)
                                 : std::vector<autoware_utils::Polygon2d>{};

  const double s_ego = autoware::motion_utils::calcSignedArcLength(
    points, points.front().pose.position, ego_pose.position);

  const int sampling_num = std::max(params.lateral_accel_sampling_num, 1);
  const double accel_resolution = std::max(
    (params.maximum_lateral_accel - params.minimum_lateral_accel) / sampling_num,
    std::numeric_limits<double>::epsilon());

  for (const double margin : params.collision_check_margins) {
    for (double lateral_accel = params.minimum_lateral_accel;
         lateral_accel <= params.maximum_lateral_accel + std::numeric_limits<double>::epsilon();
         lateral_accel += accel_resolution) {
      const double shift_length = utils::compute_shift_length_from_lateral_accel(
        std::abs(d_goal), params.expected_parking_speed, lateral_accel,
        params.minimum_shift_distance);

      const auto candidate =
        goal_planner_utils::make_pull_over_candidate(trajectory, goal_pose, shift_length);
      if (!candidate || candidate->points.size() < 2) {
        continue;
      }

      const double s_shift_start = std::max(0.0, s_goal - shift_length);
      if (start_planner_utils::has_collision(
            candidate->points, s_shift_start, std::numeric_limits<double>::max(), vehicle_info_,
            object_polygons, margin)) {
        continue;
      }

      result.status = GoalPlannerResult::Status::PLANNED;
      result.trajectory = *candidate;
      result.selected_lateral_accel = lateral_accel;
      result.selected_margin = margin;
      result.in_pull_over_approach = s_ego >= s_shift_start - params.turn_signal_distance;
      if (result.in_pull_over_approach) {
        result.turn_indicators_command =
          d_goal > 0.0 ? TurnIndicatorsCommand::ENABLE_LEFT : TurnIndicatorsCommand::ENABLE_RIGHT;
      }
      return result;
    }
  }

  RCLCPP_WARN_THROTTLE(
    logger_, clock_, 5000,
    "Goal planner: no collision-free pull over candidate found, falling back to early stop.");
  result.status = GoalPlannerResult::Status::BLOCKED;
  return result;
}

namespace goal_planner_utils
{

std::optional<Trajectory> make_pull_over_candidate(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & goal_pose,
  const double shift_length)
{
  const auto & points = trajectory.points;
  if (points.size() < 2) {
    return std::nullopt;
  }

  const double total_length = total_arc_length(points);
  const double s_goal = std::clamp(
    autoware::motion_utils::calcSignedArcLength(
      points, points.front().pose.position, goal_pose.position),
    0.0, total_length);
  if (s_goal < 0.5) {
    return std::nullopt;
  }

  const double d_goal = autoware::motion_utils::calcLateralOffset(points, goal_pose.position);
  const double L = std::min(shift_length, s_goal);
  const double s_start = s_goal - L;

  // On curved paths the per-point lateral offset does not land exactly on the goal (curvature x
  // offset error), so snapping the end point onto the goal would kink the final segment and
  // destabilize the yaw there. Blend the last few meters onto the goal axis (the straight line
  // through the goal pose) so the approach is collinear with the goal heading.
  const double goal_yaw = tf2::getYaw(goal_pose.orientation);
  const double blend_length = std::min(3.0, L / 2.0);

  Trajectory candidate;
  candidate.header = trajectory.header;

  double s = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      s +=
        autoware_utils::calc_distance2d(points.at(i - 1).pose.position, points.at(i).pose.position);
    }
    if (s <= s_start) {
      candidate.points.push_back(points.at(i));
      continue;
    }
    if (s >= s_goal) {
      break;
    }
    const double t = (s - s_start) / L;
    const double y = d_goal * smoothstep5(t);
    const double yp = d_goal * smoothstep5_derivative(t) / L;
    const double base_yaw = tf2::getYaw(points.at(i).pose.orientation);

    TrajectoryPoint pt = points.at(i);
    pt.pose.position.x += y * (-std::sin(base_yaw));
    pt.pose.position.y += y * std::cos(base_yaw);
    double yaw = base_yaw + std::atan2(yp, 1.0);

    const double remaining = s_goal - s;
    if (blend_length > 1e-3 && remaining < blend_length) {
      const double w = smoothstep5(1.0 - remaining / blend_length);
      const double target_x = goal_pose.position.x - remaining * std::cos(goal_yaw);
      const double target_y = goal_pose.position.y - remaining * std::sin(goal_yaw);
      pt.pose.position.x = (1.0 - w) * pt.pose.position.x + w * target_x;
      pt.pose.position.y = (1.0 - w) * pt.pose.position.y + w * target_y;
      yaw += w * autoware_utils::normalize_radian(goal_yaw - yaw);
    }
    pt.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    candidate.points.push_back(pt);
  }

  // End exactly at the goal pose with a non-degenerate final segment
  constexpr double min_final_segment_length = 0.3;  // [m]
  pin_trajectory_end(candidate, goal_pose, min_final_segment_length);

  return candidate;
}

void pin_trajectory_end(
  Trajectory & trajectory, const geometry_msgs::msg::Pose & end_pose,
  const double min_final_segment_length)
{
  if (trajectory.points.empty()) {
    return;
  }

  // Drop trailing points too close to end_pose: a near-degenerate final segment makes the end
  // orientation unstable across cycles (smoothing/resampling stages recompute yaw from the
  // segment direction), which can trip the trajectory deviation check on arrival.
  const float end_velocity = trajectory.points.back().longitudinal_velocity_mps;
  while (trajectory.points.size() > 1 &&
         autoware_utils::calc_distance2d(
           trajectory.points.back().pose.position, end_pose.position) < min_final_segment_length) {
    trajectory.points.pop_back();
  }

  TrajectoryPoint end_pt = trajectory.points.back();
  end_pt.pose = end_pose;
  end_pt.longitudinal_velocity_mps = end_velocity;
  trajectory.points.push_back(end_pt);
}

Trajectory crop_trajectory_end(const Trajectory & trajectory, const double crop_length)
{
  if (crop_length <= 1e-3 || trajectory.points.size() < 2) {
    return trajectory;
  }

  const auto & points = trajectory.points;
  const double target_length = std::max(0.0, total_arc_length(points) - crop_length);

  Trajectory cropped;
  cropped.header = trajectory.header;
  cropped.points.push_back(points.front());

  double s = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    const double segment_length =
      autoware_utils::calc_distance2d(points.at(i - 1).pose.position, points.at(i).pose.position);
    if (s + segment_length >= target_length) {
      const double remain = target_length - s;
      if (remain > 1e-3 && segment_length > 1e-6) {
        const double ratio = remain / segment_length;
        TrajectoryPoint end_pt = points.at(i - 1);
        end_pt.pose.position.x +=
          ratio * (points.at(i).pose.position.x - points.at(i - 1).pose.position.x);
        end_pt.pose.position.y +=
          ratio * (points.at(i).pose.position.y - points.at(i - 1).pose.position.y);
        end_pt.pose.position.z +=
          ratio * (points.at(i).pose.position.z - points.at(i - 1).pose.position.z);
        cropped.points.push_back(end_pt);
      }
      break;
    }
    s += segment_length;
    cropped.points.push_back(points.at(i));
  }

  return cropped;
}

}  // namespace goal_planner_utils
}  // namespace autoware::minimum_rule_based_planner
