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

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <cmath>
#include <string>

namespace autoware::diffusion_planner
{

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

  const double elapsed_s = age_s + params_.time_offset_s;
  const auto virtual_pose = interpolate_pose_at(elapsed_s);
  if (!virtual_pose) {
    return reset_with("elapsed_beyond_horizon");
  }

  if (!compute_deviation(ego_odometry.pose.pose.position, virtual_pose->position, status)) {
    return reset_with("degenerate_trajectory");
  }

  const double abs_lat = std::abs(status.lateral_deviation_m);
  const double abs_lon = std::abs(status.longitudinal_deviation_m);
  if (active_) {
    if (
      abs_lat > params_.lateral_deviation_threshold_m ||
      abs_lon > params_.longitudinal_deviation_threshold_m) {
      return reset_with("deviation_exceeded");
    }
  } else {
    if (
      abs_lat >= params_.lateral_rearm_threshold_m ||
      abs_lon >= params_.longitudinal_deviation_threshold_m) {
      return reset_with("waiting_rearm");
    }
    active_ = true;
  }

  status.planning_origin = *virtual_pose;
  status.stitched = true;
  return status;
}

void TrajectoryStitcher::set_previous_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & planning_origin,
  const unique_identifier_msgs::msg::UUID & route_uuid)
{
  prev_trajectory_ = trajectory;
  prev_origin_ = planning_origin;
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
  active_ = false;
}

std::optional<geometry_msgs::msg::Pose> TrajectoryStitcher::interpolate_pose_at(
  const double elapsed_s) const
{
  const auto & points = prev_trajectory_->points;
  if (points.empty()) {
    return std::nullopt;
  }

  const auto time_of = [](const autoware_planning_msgs::msg::TrajectoryPoint & point) {
    return rclcpp::Duration(point.time_from_start).seconds();
  };

  if (elapsed_s > time_of(points.back())) {
    return std::nullopt;
  }

  geometry_msgs::msg::Pose previous_pose = prev_origin_;
  double previous_time = 0.0;
  for (const auto & point : points) {
    const double point_time = time_of(point);
    if (elapsed_s <= point_time) {
      const double ratio = (point_time > previous_time)
                             ? (elapsed_s - previous_time) / (point_time - previous_time)
                             : 0.0;
      return autoware_utils_geometry::calc_interpolated_pose(
        previous_pose, point.pose, ratio, false);
    }
    previous_time = point_time;
    previous_pose = point.pose;
  }
  return std::nullopt;
}

bool TrajectoryStitcher::compute_deviation(
  const geometry_msgs::msg::Point & real_position,
  const geometry_msgs::msg::Point & virtual_position, StitchingStatus & status) const
{
  const auto points = autoware::motion_utils::removeOverlapPoints(prev_trajectory_->points);
  if (points.size() < 2) {
    return false;
  }

  const double lateral_deviation = autoware::motion_utils::calcLateralOffset(points, real_position);
  const double longitudinal_deviation =
    autoware::motion_utils::calcSignedArcLength(points, real_position, virtual_position);
  if (!std::isfinite(lateral_deviation) || !std::isfinite(longitudinal_deviation)) {
    return false;
  }

  status.lateral_deviation_m = lateral_deviation;
  status.longitudinal_deviation_m = longitudinal_deviation;
  return true;
}

void TrajectoryStitcher::clear_previous_trajectory()
{
  prev_trajectory_.reset();
}

}  // namespace autoware::diffusion_planner
