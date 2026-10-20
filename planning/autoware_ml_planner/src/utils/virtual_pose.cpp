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

#include "autoware/ml_planner/utils/virtual_pose.hpp"

#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/closest.hpp"

#include <Eigen/Geometry>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>
#include <vector>

namespace autoware::ml_planner::utils
{
namespace
{
using autoware::experimental::trajectory::interpolator::CubicSpline;
using geometry_msgs::msg::Pose;
using PoseTrajectory = autoware::experimental::trajectory::Trajectory<Pose>;

// Leading segments of the published trajectory searched for the closest point: a cycle advances
// the vehicle by about one segment, and a far part (a U-turn's return leg) must not be matched.
constexpr size_t SEARCH_SEGMENT_COUNT = 5;
// Earlier virtual poses, at least MIN_PREFIX_SPACING_M apart, put before the published trajectory
// so the spline also extends behind the vehicle.
constexpr size_t PREFIX_COUNT = 10;
constexpr double MIN_PREFIX_SPACING_M = 0.05;
// The virtual heading is the mean spline tangent over this arc length on each side of the closest
// point; below MIN_YAW_WINDOW_M of window the measured heading is used.
constexpr double YAW_HALF_WINDOW_M = 1.0;
constexpr double MIN_YAW_WINDOW_M = 0.2;
// The planning start counts as unchanged within these.
constexpr double SAME_POSITION_M = 0.01;
constexpr double SAME_YAW_DEG = 0.1;

// Distances are planar: the closest point search and the arc length would otherwise include the
// height of a trajectory on a slope.
Pose planar(Pose pose)
{
  pose.position.z = 0.0;
  return pose;
}

struct PoseOffset
{
  double longitudinal_m;
  double lateral_m;
  double distance_m;
  double yaw_deg;
};

// Offset of the measured pose from a reference position, split along and across the reference
// heading, and the heading difference.
PoseOffset pose_offset(
  const Pose & measured, const Eigen::Vector2d & reference_position, const double reference_yaw)
{
  const Eigen::Vector2d offset =
    Eigen::Vector2d(measured.position.x, measured.position.y) - reference_position;
  const double measured_yaw = autoware_utils_geometry::get_rpy(measured.orientation).z;
  return PoseOffset{
    std::abs(std::cos(reference_yaw) * offset.x() + std::sin(reference_yaw) * offset.y()),
    std::abs(-std::sin(reference_yaw) * offset.x() + std::cos(reference_yaw) * offset.y()),
    offset.norm(),
    autoware_utils::rad2deg(
      std::abs(autoware_utils::normalize_radian(reference_yaw - measured_yaw)))};
}

PoseOffset pose_offset(const Pose & measured, const Pose & reference)
{
  return pose_offset(
    measured, Eigen::Vector2d(reference.position.x, reference.position.y),
    autoware_utils_geometry::get_rpy(reference.orientation).z);
}

bool exceeds_reset_limits(const PoseOffset & offset, const VirtualPoseParams & params)
{
  return offset.longitudinal_m > params.max_longitudinal_error_m ||
         offset.lateral_m > params.max_lateral_error_m || offset.yaw_deg > params.max_yaw_error_deg;
}

// Circular mean of the spline tangent over a window kept symmetric around s (shrunk at either end,
// so the mean is not biased toward the path ahead); nullopt when the window is too short.
std::optional<double> mean_tangent_yaw(const PoseTrajectory & trajectory, const double s)
{
  const double half = std::min({YAW_HALF_WINDOW_M, s, trajectory.length() - s});
  if (2.0 * half < MIN_YAW_WINDOW_M) {
    return std::nullopt;
  }
  constexpr double SAMPLE_TICK_M = 0.05;
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  for (const double yaw :
       trajectory.azimuth(trajectory.base_arange({s - half, s + half}, SAMPLE_TICK_M))) {
    sum_cos += std::cos(yaw);
    sum_sin += std::sin(yaw);
  }
  return std::atan2(sum_sin, sum_cos);
}

struct ClosestPoint
{
  Eigen::Vector2d position;
  std::optional<double> tangent_yaw;
};

// Closest point to the query among the leading segments after the prefix of the reference.
std::optional<ClosestPoint> find_closest_point(
  const geometry_msgs::msg::Point & query, std::vector<Pose> reference, const size_t prefix_count)
{
  // A stop at the end of a trajectory repeats its last point, which would give the spline
  // zero-length bases: keep the poses up to the first repetition.
  const auto repeated =
    std::adjacent_find(reference.begin(), reference.end(), [](const Pose & lhs, const Pose & rhs) {
      return autoware::experimental::trajectory::is_almost_same(lhs.position, rhs.position);
    });
  if (repeated != reference.end()) {
    reference.erase(std::next(repeated), reference.end());
  }
  if (reference.size() < prefix_count + 2) {
    return std::nullopt;
  }
  const auto trajectory =
    PoseTrajectory::Builder{}.set_xy_interpolator<CubicSpline>().build(reference);
  if (!trajectory) {
    return std::nullopt;
  }
  const std::vector<double> bases = trajectory->get_underlying_bases();
  const double search_start_s = bases[prefix_count];
  const double search_end_s =
    bases[std::min(prefix_count + SEARCH_SEGMENT_COUNT, bases.size() - 1)];
  geometry_msgs::msg::Point planar_query;
  planar_query.x = query.x;
  planar_query.y = query.y;
  const auto s = autoware::experimental::trajectory::closest_with_constraint(
    *trajectory, planar_query, [search_start_s, search_end_s](const double & arc) {
      return search_start_s <= arc && arc <= search_end_s;
    });
  if (!s) {
    return std::nullopt;
  }
  const Pose closest = trajectory->compute(*s);
  return ClosestPoint{
    Eigen::Vector2d(closest.position.x, closest.position.y), mean_tangent_yaw(*trajectory, *s)};
}

// The measured pose moved onto the closest point and turned to the virtual heading about the map
// z axis, so the vehicle's height, roll and pitch are kept (the trajectory carries none).
Pose make_virtual_pose(const Pose & measured, const Eigen::Vector2d & position, const double yaw)
{
  const double measured_yaw = autoware_utils_geometry::get_rpy(measured.orientation).z;
  const Eigen::Quaterniond measured_q(
    measured.orientation.w, measured.orientation.x, measured.orientation.y, measured.orientation.z);
  const Eigen::Quaterniond virtual_q =
    (Eigen::Quaterniond(Eigen::AngleAxisd(
       autoware_utils::normalize_radian(yaw - measured_yaw), Eigen::Vector3d::UnitZ())) *
     measured_q)
      .normalized();
  Pose pose = measured;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.orientation.x = virtual_q.x();
  pose.orientation.y = virtual_q.y();
  pose.orientation.z = virtual_q.z();
  pose.orientation.w = virtual_q.w();
  return pose;
}
}  // namespace

VirtualPose::VirtualPose(const double history_window_s)
: history_(history_window_s, [](const Odometry & msg) { return rclcpp::Time(msg.header.stamp); })
{
}

void VirtualPose::update(
  const TimedBuffer<Odometry> & measured_history, const VirtualPoseParams & params)
{
  const Odometry & measured = measured_history.back();
  if (history_.empty()) {
    restart_from(measured_history);
  }
  const auto set_planning_start = [&](const Pose & pose) {
    Odometry frame = measured;
    frame.pose.pose = pose;
    // Replaces the measured entry seeded with the same stamp, and the planning start of a cycle
    // without new odometry, so the past always ends at the pose the output is decoded in.
    history_.push_or_replace_back(frame);
  };

  // At standstill keep the planning start: snapping again onto each new plan would only move it
  // with the plan's jitter and the localization drift. The reset limits still apply.
  if (hold_anchor_) {
    const auto anchor_offset = pose_offset(measured.pose.pose, *hold_anchor_);
    const Pose held = history_.back().pose.pose;
    if (
      anchor_offset.distance_m < params.hold_position_tolerance_m &&
      anchor_offset.yaw_deg < params.hold_yaw_tolerance_deg &&
      !exceeds_reset_limits(pose_offset(measured.pose.pose, held), params)) {
      set_planning_start(held);
      return;
    }
  }
  hold_anchor_.reset();
  if (std::abs(measured.twist.twist.linear.x) < params.hold_max_speed_mps) {
    hold_anchor_ = measured.pose.pose;
  }

  if (!published_start_) {
    set_planning_start(measured.pose.pose);
    return;
  }
  const Reference reference = build_reference();
  const auto closest =
    find_closest_point(measured.pose.pose.position, reference.poses, reference.prefix_count);
  if (closest) {
    const double yaw = closest->tangent_yaw.value_or(
      autoware_utils_geometry::get_rpy(measured.pose.pose.orientation).z);
    if (!exceeds_reset_limits(pose_offset(measured.pose.pose, closest->position, yaw), params)) {
      set_planning_start(make_virtual_pose(measured.pose.pose, closest->position, yaw));
      return;
    }
  }

  // Back onto the vehicle: beyond the reset limits, or no closest point while the planning start
  // was elsewhere.
  const auto start_offset = pose_offset(measured.pose.pose, history_.back().pose.pose);
  if (closest || start_offset.distance_m > SAME_POSITION_M || start_offset.yaw_deg > SAME_YAW_DEG) {
    // A past of earlier virtual poses would end in the jump, which the model reads as a sudden
    // lateral motion: restart it from the measured poses. The trajectory just left must not be
    // snapped onto again if this cycle's planning fails.
    restart_from(measured_history);
    return;
  }
  set_planning_start(measured.pose.pose);
}

void VirtualPose::set_published_trajectory(const Trajectory & trajectory)
{
  if (history_.empty() || trajectory.points.empty()) {
    return;
  }
  // The published points start one time step after the planning start, so the planning start
  // begins the search: from the first point, the virtual pose would advance by a step every cycle
  // even while the vehicle lags behind it.
  published_start_ = planar(history_.back().pose.pose);
  published_poses_.clear();
  for (const auto & point : trajectory.points) {
    published_poses_.push_back(planar(point.pose));
  }
}

void VirtualPose::reset()
{
  history_.clear();
  published_start_.reset();
  published_poses_.clear();
  hold_anchor_.reset();
  restart_ = true;
}

bool VirtualPose::take_restart()
{
  return std::exchange(restart_, false);
}

void VirtualPose::restart_from(const TimedBuffer<Odometry> & measured_history)
{
  history_.clear();
  for (const auto & msg : measured_history.msgs()) {
    history_.push_back(msg);
  }
  published_start_.reset();
  published_poses_.clear();
  restart_ = true;
}

VirtualPose::Reference VirtualPose::build_reference() const
{
  // Earlier planning starts, newest first, excluding the one the trajectory was published from
  // (the newest history entry).
  std::vector<Pose> prefix;
  Eigen::Vector2d successor(published_start_->position.x, published_start_->position.y);
  const auto & past = history_.msgs();
  for (auto it = past.rbegin(); it != past.rend() && prefix.size() < PREFIX_COUNT; ++it) {
    const Eigen::Vector2d position(it->pose.pose.position.x, it->pose.pose.position.y);
    if ((position - successor).norm() < MIN_PREFIX_SPACING_M) {
      continue;
    }
    prefix.push_back(planar(it->pose.pose));
    successor = position;
  }
  Reference reference{std::vector<Pose>(prefix.rbegin(), prefix.rend()), prefix.size()};
  reference.poses.push_back(*published_start_);
  reference.poses.insert(reference.poses.end(), published_poses_.begin(), published_poses_.end());
  return reference;
}

}  // namespace autoware::ml_planner::utils
