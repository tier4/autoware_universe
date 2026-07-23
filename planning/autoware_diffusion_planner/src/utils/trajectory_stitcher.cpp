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

#include "autoware/diffusion_planner/utils/trajectory_stitcher.hpp"

#include "autoware/diffusion_planner/constants.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace autoware::diffusion_planner
{
namespace
{

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

TrajectoryStitcher::TrajectoryStitcher(const TrajectoryStitcherParams & params) : params_(params)
{
}

void TrajectoryStitcher::update_params(const TrajectoryStitcherParams & params)
{
  params_ = params;
}

StitchingStatus TrajectoryStitcher::compute_planning_origin(
  const rclcpp::Time & frame_time, const nav_msgs::msg::Odometry & ego_odometry,
  const unique_identifier_msgs::msg::UUID & route_uuid, const bool mppi_overwrite_active)
{
  StitchingStatus status;
  status.planning_origin = ego_odometry.pose.pose;

  const auto reset_with = [&](const std::string & reason) {
    status.stitched = false;
    status.reset_reason = reason;
    active_ = false;
    filtered_origin_.reset();
    last_ego_pose_.reset();
    return status;
  };

  if (!params_.enable) {
    return reset_with("disabled");
  }
  if (mppi_overwrite_active) {
    return reset_with("mppi_active");
  }
  if (!prev_trajectory_) {
    return reset_with("no_previous_trajectory");
  }

  const double age_s = (frame_time - rclcpp::Time(prev_trajectory_->header.stamp)).seconds();
  if (age_s < 0.0 || age_s > params_.max_trajectory_age_s) {
    clear_previous_trajectory();
    return reset_with("stale_trajectory");
  }
  if (route_uuid != prev_route_uuid_) {
    clear_previous_trajectory();
    return reset_with("route_changed");
  }
  if (ego_odometry.twist.twist.linear.x < constants::MOVING_VELOCITY_THRESHOLD_MPS) {
    return reset_with("low_speed");
  }

  const auto points = autoware::motion_utils::removeOverlapPoints(prev_trajectory_->points);
  if (points.size() < 2) {
    return reset_with("degenerate_trajectory");
  }

  const auto & ego_position = ego_odometry.pose.pose.position;
  const size_t segment_index =
    autoware::motion_utils::findNearestSegmentIndex(points, ego_position);
  const double ego_arc =
    autoware::motion_utils::calcSignedArcLength(points, 0, segment_index) +
    autoware::motion_utils::calcLongitudinalOffsetToSegment(points, segment_index, ego_position);
  const double lateral_deviation = autoware::motion_utils::calcLateralOffset(points, ego_position);
  if (!std::isfinite(ego_arc) || !std::isfinite(lateral_deviation)) {
    return reset_with("degenerate_trajectory");
  }

  const double lead_arc = ego_odometry.twist.twist.linear.x * params_.time_offset_s;
  const double target_arc = ego_arc + lead_arc;
  const double total_arc = autoware::motion_utils::calcArcLength(points);
  if (target_arc < -params_.longitudinal_deviation_threshold_m || target_arc > total_arc) {
    return reset_with("beyond_horizon");
  }

  status.lateral_deviation_m = lateral_deviation;
  status.longitudinal_deviation_m = lead_arc;

  const double abs_lat = std::abs(lateral_deviation);
  if (active_) {
    if (abs_lat > params_.lateral_deviation_threshold_m) {
      rearm_allowed_time_ = frame_time + rclcpp::Duration::from_seconds(params_.rearm_cooldown_s);
      return reset_with("deviation_exceeded");
    }
  } else {
    if (rearm_allowed_time_ && frame_time < *rearm_allowed_time_) {
      return reset_with("waiting_rearm");
    }
    if (abs_lat >= params_.lateral_rearm_threshold_m) {
      return reset_with("waiting_rearm");
    }
    active_ = true;
  }

  // The stored trajectory has no point at the plan origin (the first point is the model's
  // first prediction step), so positions behind it are extrapolated along the first segment.
  // Orientation is the yaw of the path tangent measured over a long baseline: the
  // model-predicted headings and short-baseline tangents are too noisy to anchor the
  // planning frame, and any pitch would foreshorten the planar model inputs.
  geometry_msgs::msg::Pose projection;
  if (target_arc < 0.0) {
    const auto & first = points[0].pose;
    const auto & second = points[1].pose;
    const double dx = second.position.x - first.position.x;
    const double dy = second.position.y - first.position.y;
    const double segment_length = std::hypot(dx, dy);
    projection = first;
    projection.position.x += target_arc * dx / segment_length;
    projection.position.y += target_arc * dy / segment_length;
    projection.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(dy, dx));
  } else {
    projection = autoware::motion_utils::calcInterpolatedPose(points, target_arc, false);
    constexpr double tangent_half_baseline = 2.5;
    const double back_arc = std::max(target_arc - tangent_half_baseline, 0.0);
    const double front_arc = std::min(target_arc + tangent_half_baseline, total_arc);
    const auto back_pose = autoware::motion_utils::calcInterpolatedPose(points, back_arc, false);
    const auto front_pose = autoware::motion_utils::calcInterpolatedPose(points, front_arc, false);
    const double dx = front_pose.position.x - back_pose.position.x;
    const double dy = front_pose.position.y - back_pose.position.y;
    if (std::hypot(dx, dy) > 1e-3) {
      projection.orientation =
        autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(dy, dx));
    }
  }

  // Complementary filter: the anchor advances by the measured ego motion increment
  // (localization-grade, carries no accumulated tracking error) and is pulled toward the
  // path projection by path_correction_gain, which attenuates the projection noise
  // inherited from the model output points.
  geometry_msgs::msg::Pose origin = projection;
  if (filtered_origin_ && last_ego_pose_) {
    const double prev_ego_yaw = yaw_of(last_ego_pose_->orientation);
    const double ego_dx = ego_position.x - last_ego_pose_->position.x;
    const double ego_dy = ego_position.y - last_ego_pose_->position.y;
    const double body_dx = std::cos(prev_ego_yaw) * ego_dx + std::sin(prev_ego_yaw) * ego_dy;
    const double body_dy = -std::sin(prev_ego_yaw) * ego_dx + std::cos(prev_ego_yaw) * ego_dy;
    const double body_dyaw = wrap_angle(yaw_of(ego_odometry.pose.pose.orientation) - prev_ego_yaw);

    const double anchor_yaw = yaw_of(filtered_origin_->orientation);
    const double predicted_x = filtered_origin_->position.x + std::cos(anchor_yaw) * body_dx -
                               std::sin(anchor_yaw) * body_dy;
    const double predicted_y = filtered_origin_->position.y + std::sin(anchor_yaw) * body_dx +
                               std::cos(anchor_yaw) * body_dy;
    const double predicted_yaw = wrap_angle(anchor_yaw + body_dyaw);

    const double gain = params_.path_correction_gain;
    origin.position.x = predicted_x + gain * (projection.position.x - predicted_x);
    origin.position.y = predicted_y + gain * (projection.position.y - predicted_y);
    origin.position.z = projection.position.z;
    origin.orientation = autoware_utils_geometry::create_quaternion_from_yaw(wrap_angle(
      predicted_yaw + gain * wrap_angle(yaw_of(projection.orientation) - predicted_yaw)));
  }
  filtered_origin_ = origin;
  last_ego_pose_ = ego_odometry.pose.pose;

  status.planning_origin = origin;
  status.stitched = true;
  return status;
}

void TrajectoryStitcher::set_previous_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const unique_identifier_msgs::msg::UUID & route_uuid)
{
  prev_trajectory_ = trajectory;
  prev_route_uuid_ = route_uuid;
}

void TrajectoryStitcher::push_planning_origin_history(
  const rclcpp::Time & frame_time, const geometry_msgs::msg::Pose & origin_pose,
  const size_t max_size)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = frame_time;
  odometry.header.frame_id = "map";
  odometry.pose.pose = origin_pose;
  planning_origin_history_.push_back(odometry);
  while (planning_origin_history_.size() > max_size) {
    planning_origin_history_.pop_front();
  }
}

void TrajectoryStitcher::reset()
{
  clear_previous_trajectory();
  planning_origin_history_.clear();
  rearm_allowed_time_.reset();
  filtered_origin_.reset();
  last_ego_pose_.reset();
  active_ = false;
}

void TrajectoryStitcher::clear_previous_trajectory()
{
  prev_trajectory_.reset();
}

}  // namespace autoware::diffusion_planner
