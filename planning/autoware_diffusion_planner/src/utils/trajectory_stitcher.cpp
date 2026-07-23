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

#include <algorithm>
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

  const geometry_msgs::msg::Pose projected_pose =
    autoware::motion_utils::calcInterpolatedPose(points, std::max(target_arc, 0.0), false);

  geometry_msgs::msg::Pose origin = projected_pose;
  const double gain = params_.correction_gain;
  origin.position.x += gain * (ego_position.x - projected_pose.position.x);
  origin.position.y += gain * (ego_position.y - projected_pose.position.y);
  origin.position.z += gain * (ego_position.z - projected_pose.position.z);

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
  active_ = false;
}

void TrajectoryStitcher::clear_previous_trajectory()
{
  prev_trajectory_.reset();
}

}  // namespace autoware::diffusion_planner
