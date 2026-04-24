// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory_validator/filters/safety/collision_check_filter.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <autoware_internal_planning_msgs/msg/control_point.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <any>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
struct EvaluationArtifacts
{
  bool is_feasible{true};
  std::string error_msg{};
  std::vector<MetricReport> metrics{};
  autoware_internal_planning_msgs::msg::PlanningFactorArray planning_factors{};
};
}  // namespace

PetCollisionParams::PetCollisionParams(
  const validator::Params::CollisionCheck::PetCollision & pet, const std::string & key)
{
  enable_assessment = extract_labeled_param<bool>(pet.enable_assessment, key);
  predicted_path_trajectory = extract_labeled_param<bool>(pet.predicted_path_trajectory, key);
  constant_curvature_trajectory =
    extract_labeled_param<bool>(pet.constant_curvature_trajectory, key);
  diffusion_based_trajectory = extract_labeled_param<bool>(pet.diffusion_based_trajectory, key);
  ego_braking_delay = extract_labeled_param<double>(pet.ego_braking_delay, key);
  ego_assumed_acceleration = extract_labeled_param<double>(pet.ego_assumed_acceleration, key);
  collision_time_threshold = extract_labeled_param<double>(pet.collision_time_threshold, key);
}

RssParams::RssParams(const validator::Params::CollisionCheck::Rss & rss, const std::string & key)
{
  enable_assessment = extract_labeled_param<bool>(rss.enable_assessment, key);
  ego_deceleration_threshold = extract_labeled_param<double>(rss.ego_deceleration_threshold, key);
  object_acceleration = extract_labeled_param<double>(rss.object_acceleration, key);
  stop_margin = extract_labeled_param<double>(rss.stop_margin, key);
  ego_reaction_time = extract_labeled_param<double>(rss.ego_reaction_time, key);
}

DracParams::DracParams(
  const validator::Params::CollisionCheck::Drac & drac, const std::string & key)
{
  enable_assessment = extract_labeled_param<bool>(drac.enable_assessment, key);
  predicted_path_trajectory = extract_labeled_param<bool>(drac.predicted_path_trajectory, key);
  constant_curvature_trajectory =
    extract_labeled_param<bool>(drac.constant_curvature_trajectory, key);
  diffusion_based_trajectory = extract_labeled_param<bool>(drac.diffusion_based_trajectory, key);
}

template <typename OutT, typename ParamStruct>
OutT extract_labeled_param(const ParamStruct & params_struct, const std::string & key)
{
  if constexpr (std::is_aggregate_v<ParamStruct>) {
    if (key == "base") {
      return static_cast<OutT>(params_struct.base);
    }

    using MemberPtr = OutT ParamStruct::*;

    static const std::unordered_map<std::string, MemberPtr> mappings = {
      {"car", &ParamStruct::car},
      {"truck", &ParamStruct::truck},
      {"bus", &ParamStruct::bus},
      {"trailer", &ParamStruct::trailer},
      {"motorcycle", &ParamStruct::motorcycle},
      {"bicycle", &ParamStruct::bicycle},
      {"pedestrian", &ParamStruct::pedestrian},
      {"animal", &ParamStruct::animal},
      {"hazard", &ParamStruct::hazard},
      {"over_drivable", &ParamStruct::over_drivable},
      {"under_drivable", &ParamStruct::under_drivable},
      {"unknown", &ParamStruct::unknown}};

    auto it = mappings.find(key);
    if (it == mappings.end()) {
      throw std::invalid_argument("Unknown label key: " + key);
    }

    auto label_value = params_struct.*(it->second);
    if constexpr (std::is_floating_point_v<OutT>) {
      return static_cast<OutT>(std::isnan(label_value) ? params_struct.base : label_value);
    } else if constexpr (std::is_same_v<OutT, std::string>) {
      return static_cast<OutT>(label_value.empty() ? params_struct.base : label_value);
    } else {
      return static_cast<OutT>(label_value);
    }

  } else {
    return static_cast<OutT>(params_struct);
  }
}

// Trajectory generation helpers.
namespace trajectory::time_distance
{
std::pair<TimeTrajectory, TravelDistanceTrajectory> compute_motion_profile_1d(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double start_time, double end_time, double time_resolution)
{
  const double initial_velocity = std::hypot(initial_twist.linear.x, initial_twist.linear.y);
  if (initial_velocity <= 0.0 || start_time >= end_time) {
    return {{start_time}, {0.0}};
  }

  struct StopProfile
  {
    double stop_time;
    double stop_position;
  };

  const auto stop_profile = [&]() -> std::optional<StopProfile> {
    if (assumed_acceleration >= 0.0) return std::nullopt;
    const double time_to_stop = initial_velocity / -assumed_acceleration;
    const double stop_time = braking_lag + time_to_stop;
    const double stop_position = initial_velocity * (braking_lag + 0.5 * time_to_stop);
    return StopProfile{stop_time, stop_position};
  }();

  TimeTrajectory times;
  TravelDistanceTrajectory distances;
  times.reserve(static_cast<size_t>((end_time - start_time) / time_resolution) + 4U);
  distances.reserve(times.capacity());

  auto distance = [&](double t) {
    if (t < braking_lag) {
      return initial_velocity * t;
    } else if (stop_profile.has_value() && t >= stop_profile.value().stop_time) {
      return stop_profile.value().stop_position;
    } else {
      const double time_after_lag = t - braking_lag;
      return initial_velocity * t + (0.5 * assumed_acceleration * time_after_lag * time_after_lag);
    }
  };

  constexpr double epsilon = 1e-3;
  auto append_sample = [&](const double t) {
    if (t < start_time || t > end_time) return;
    if (!times.empty() && t < times.back() + epsilon) return;
    times.push_back(t);
    distances.push_back(distance(t));
  };

  append_sample(start_time);
  for (int64_t tick = static_cast<int64_t>(std::floor(start_time / time_resolution)) + 1;; ++tick) {
    const double tick_time = static_cast<double>(tick) * time_resolution;
    if (
      stop_profile.has_value() && times.back() < stop_profile.value().stop_time &&
      tick_time > stop_profile.value().stop_time) {
      // todo(takagi): Investigate if it's necessary to add the stop time to `times`.
      append_sample(stop_profile.value().stop_time);
      break;
    }

    if (tick_time >= end_time) {
      break;
    }
    append_sample(tick_time);
  }
  append_sample(end_time);

  return {times, distances};
}
}  // namespace trajectory::time_distance

namespace trajectory::pose
{
geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose,
  const double ratio)
{
  geometry_msgs::msg::Pose interpolated_pose;
  interpolated_pose.position.x =
    interpolation::lerp(start_pose.position.x, end_pose.position.x, ratio);
  interpolated_pose.position.y =
    interpolation::lerp(start_pose.position.y, end_pose.position.y, ratio);
  interpolated_pose.position.z =
    interpolation::lerp(start_pose.position.z, end_pose.position.z, ratio);

  tf2::Quaternion start_q;
  tf2::Quaternion end_q;
  tf2::fromMsg(start_pose.orientation, start_q);
  tf2::fromMsg(end_pose.orientation, end_q);
  interpolated_pose.orientation = tf2::toMsg(start_q.slerp(end_q, ratio));

  return interpolated_pose;
}

namespace constant_curvature_predictor
{
struct TwistPerDistance
{
  Eigen::Vector2d linear;
  double angular;
};

namespace detail
{

Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
  iso.translation() << pose.position.x, pose.position.y;
  iso.linear() = Eigen::Rotation2Dd(tf2::getYaw(pose.orientation)).toRotationMatrix();
  return iso;
}

TwistPerDistance compute_twist_per_distance(const geometry_msgs::msg::Twist & twist)
{
  const Eigen::Vector2d vel(twist.linear.x, twist.linear.y);
  const double linear_speed = vel.norm();
  const double inv_speed = (linear_speed > 1e-6) ? (1.0 / linear_speed) : 0.0;

  return {vel * inv_speed, twist.angular.z * inv_speed};
}

Eigen::Isometry2d compute_delta_isometry(const TwistPerDistance & twist_per_dist, double distance)
{
  Eigen::Isometry2d delta_iso = Eigen::Isometry2d::Identity();
  const double theta = twist_per_dist.angular * distance;

  delta_iso.rotate(Eigen::Rotation2Dd(theta));

  double a, b;
  if (std::abs(theta) < 1e-4) {
    const double theta_sq = theta * theta;
    a = 1.0 - theta_sq / 6.0;
    b = theta / 2.0 - (theta_sq * theta) / 24.0;
  } else {
    a = std::sin(theta) / theta;
    b = (1.0 - std::cos(theta)) / theta;
  }

  Eigen::Matrix2d V;
  V << a, -b, b, a;
  delta_iso.translation() = V * (twist_per_dist.linear * distance);

  return delta_iso;
}

geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry2d & iso, double initial_z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = iso.translation().x();
  pose.position.y = iso.translation().y();
  pose.position.z = initial_z;

  const double yaw = Eigen::Rotation2Dd(iso.linear()).angle();
  pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  return pose;
}

}  // namespace detail

PoseTrajectory compute(
  const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Twist & initial_twist,
  const TravelDistanceTrajectory & distance_trajectory)
{
  const Eigen::Isometry2d start_iso = detail::pose_to_isometry(initial_pose);
  const TwistPerDistance twist_per_dist = detail::compute_twist_per_distance(initial_twist);

  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(distance_trajectory.size());

  for (const auto & distance : distance_trajectory) {
    const Eigen::Isometry2d delta_iso = detail::compute_delta_isometry(twist_per_dist, distance);
    const Eigen::Isometry2d target_iso = start_iso * delta_iso;
    pose_trajectory.push_back(detail::isometry_to_pose(target_iso, initial_pose.position.z));
  }

  return pose_trajectory;
}

}  // namespace constant_curvature_predictor

template <class T>
PoseTrajectory compute_pose_trajectory(
  const T & traj_points, const TravelDistanceTrajectory & distance_trajectory)
{
  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(distance_trajectory.size());
  for (const auto & distance : distance_trajectory) {
    pose_trajectory.push_back(
      autoware::motion_utils::calcInterpolatedPose(traj_points, distance, false));
  }
  return pose_trajectory;
}

PoseTrajectory compute_pose_trajectory_from_time(
  const TrajectoryPoints & traj_points, const TimeTrajectory & time_trajectory)
{
  if (traj_points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  std::vector<double> point_times;
  point_times.reserve(traj_points.size());
  for (const auto & point : traj_points) {
    point_times.push_back(rclcpp::Duration(point.time_from_start).seconds());
  }

  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(time_trajectory.size());

  for (const auto target_time : time_trajectory) {
    size_t lower_idx = 0;
    size_t upper_idx = 0;
    if (traj_points.size() == 1) {
      pose_trajectory.push_back(traj_points.front().pose);
      continue;
    } else if (target_time <= point_times.front()) {
      lower_idx = 0;
      upper_idx = 1;
    } else if (target_time >= point_times.back()) {
      lower_idx = traj_points.size() - 2;
      upper_idx = traj_points.size() - 1;
    } else {
      const auto upper_it = std::lower_bound(point_times.begin(), point_times.end(), target_time);
      upper_idx = static_cast<size_t>(std::distance(point_times.begin(), upper_it));
      lower_idx = upper_idx - 1;
    }

    const double lower_time = point_times.at(lower_idx);
    const double upper_time = point_times.at(upper_idx);
    const double denom = upper_time - lower_time;
    const double ratio = denom > 1e-6 ? (target_time - lower_time) / denom : 0.0;

    pose_trajectory.push_back(
      interpolate_pose(traj_points.at(lower_idx).pose, traj_points.at(upper_idx).pose, ratio));
  }

  return pose_trajectory;
}

}  // namespace trajectory::pose

namespace trajectory::footprint
{
FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const autoware_perception_msgs::msg::Shape & object_shape)
{
  FootprintTrajectory footprint_trajectory;
  footprint_trajectory.reserve(pose_trajectory.size());

  for (const auto & pose : pose_trajectory) {
    footprint_trajectory.push_back(geometry::to_polygon2d(pose, object_shape));
  }
  return footprint_trajectory;
}

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const VehicleInfo & vehicle_info)
{
  FootprintTrajectory footprint_trajectory;
  footprint_trajectory.reserve(pose_trajectory.size());

  for (const auto & pose : pose_trajectory) {
    footprint_trajectory.push_back(autoware_utils_geometry::to_footprint(
      pose, vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
      vehicle_info.vehicle_width_m));
  }
  return footprint_trajectory;
}
}  // namespace trajectory::footprint

namespace trajectory
{

namespace detail
{
double to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return rclcpp::Duration(duration).seconds();
}

double project_current_pose_on_trajectory(
  const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & current_pose)
{
  if (traj_points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  if (traj_points.size() == 1) {
    return to_seconds(traj_points.front().time_from_start);
  }

  const auto project_on_segment = [&](const size_t start_idx, const size_t end_idx) {
    const auto & start_pose = traj_points.at(start_idx).pose;
    const auto & end_pose = traj_points.at(end_idx).pose;
    const double current_x = current_pose.position.x;
    const double current_y = current_pose.position.y;

    const double dx = end_pose.position.x - start_pose.position.x;
    const double dy = end_pose.position.y - start_pose.position.y;
    const double segment_length_sq = dx * dx + dy * dy;

    double ratio = 0.0;
    if (segment_length_sq > 1e-6) {
      ratio =
        ((current_x - start_pose.position.x) * dx + (current_y - start_pose.position.y) * dy) /
        segment_length_sq;
    }

    return interpolation::lerp(
      to_seconds(traj_points.at(start_idx).time_from_start),
      to_seconds(traj_points.at(end_idx).time_from_start), ratio);
  };

  const size_t nearest_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, current_pose.position);
  return project_on_segment(nearest_segment_idx, nearest_segment_idx + 1);
}

TravelDistanceTrajectory compute_cumulative_distances(const PoseTrajectory & pose_trajectory)
{
  TravelDistanceTrajectory distances;
  distances.reserve(pose_trajectory.size());

  double cumulative_distance = 0.0;
  for (size_t i = 0; i < pose_trajectory.size(); ++i) {
    if (i > 0) {
      cumulative_distance += autoware_utils_geometry::calc_distance2d(
        pose_trajectory.at(i - 1).position, pose_trajectory.at(i).position);
    }
    distances.push_back(cumulative_distance);
  }

  return distances;
}

TimeTrajectory compute_sample_times(double start_time, double end_time, double time_resolution)
{
  TimeTrajectory times;
  times.reserve(static_cast<size_t>((end_time - start_time) / time_resolution) + 2U);

  constexpr double epsilon = 1e-3;
  auto append_sample = [&](const double t) {
    if (t < start_time || t > end_time) return;
    if (!times.empty() && t < times.back() + epsilon) return;
    times.push_back(t);
  };

  append_sample(start_time);
  for (int64_t tick = static_cast<int64_t>(std::floor(start_time / time_resolution)) + 1;; ++tick) {
    const double tick_time = static_cast<double>(tick) * time_resolution;
    if (tick_time >= end_time) {
      break;
    }
    append_sample(tick_time);
  }
  append_sample(end_time);

  return times;
}

geometry_msgs::msg::Pose interpolate_predicted_path_pose(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path, double query_time,
  double path_start_time)
{
  if (predicted_path.path.empty()) {
    throw std::invalid_argument("predicted path must not be empty");
  }

  const double path_time_step = rclcpp::Duration(predicted_path.time_step).seconds();
  if (predicted_path.path.size() == 1 || path_time_step <= 0.0) {
    return predicted_path.path.front();
  }

  const double clamped_query_time = std::clamp(
    query_time, path_start_time,
    path_start_time + path_time_step * static_cast<double>(predicted_path.path.size() - 1));
  const double shifted_query_time = clamped_query_time - path_start_time;
  const size_t index = static_cast<size_t>(std::floor(shifted_query_time / path_time_step));
  const size_t next_index = std::min(index + 1, predicted_path.path.size() - 1);
  const double ratio =
    (shifted_query_time - static_cast<double>(index) * path_time_step) / path_time_step;
  return autoware::universe_utils::calcInterpolatedPose(
    predicted_path.path.at(index), predicted_path.path.at(next_index), ratio, false);
}
}  // namespace detail

TrajectoryData generate_ego_trajectory(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double max_time, double time_resolution, const TrajectoryPoints & traj_points,
  VehicleInfo & vehicle_info)
{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    initial_twist, braking_lag, assumed_acceleration, 0.0, max_time, time_resolution);

  // todo(takagi): use initial pose from context instead of traj_points.front()
  // check https://star4.slack.com/archives/C03QW0GU6P7/p1773898469086129
  double distance_offset =
    autoware::motion_utils::calcSignedArcLength(traj_points, traj_points.front().pose.position, 0);
  for (double & val : distances) {
    val += distance_offset;
  }
  auto poses = pose::compute_pose_trajectory(traj_points, distances);

  auto footprints = footprint::compute_footprint_trajectory(poses, vehicle_info);

  return TrajectoryData(
    TrajectoryIdentification{"EGO"}, std::move(times), std::move(distances), std::move(poses),
    std::move(footprints));
}

TrajectoryData generate_ego_trajectory(
  const TrajectoryPoints & traj_points, const FilterContext & context, double max_time,
  double time_resolution, VehicleInfo & vehicle_info)
{
  if (traj_points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  const double start_time =
    detail::project_current_pose_on_trajectory(traj_points, context.odometry->pose.pose);
  const double end_time =
    std::min(detail::to_seconds(traj_points.back().time_from_start), start_time + max_time);

  TimeTrajectory relative_times{0.0};
  TimeTrajectory absolute_times{start_time};
  for (double sample_time = time_resolution; start_time + sample_time < end_time;
       sample_time =
         std::floor((sample_time + time_resolution + 1e-6) / time_resolution) * time_resolution) {
    relative_times.push_back(sample_time);
    absolute_times.push_back(start_time + sample_time);
  }

  auto poses = pose::compute_pose_trajectory_from_time(traj_points, absolute_times);
  auto distances = detail::compute_cumulative_distances(poses);
  auto footprints = footprint::compute_footprint_trajectory(poses, vehicle_info);

  return TrajectoryData(
    TrajectoryIdentification{"EGO"}, std::move(relative_times), std::move(distances),
    std::move(poses), std::move(footprints));
}

TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time,
  const builtin_interfaces::msg::Time & stamp, double time_resolution)
{
  // todo(takagi): use all or appropriate predicted paths
  const auto most_confident_path_it = std::max_element(
    predicted_object.kinematics.predicted_paths.begin(),
    predicted_object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, braking_lag,
    assumed_acceleration, start_time.seconds(),
    std::min(
      max_time, most_confident_path_it->path.size() *
                  rclcpp::Duration(most_confident_path_it->time_step).seconds()),
    time_resolution);

  auto poses = pose::compute_pose_trajectory(most_confident_path_it->path, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    TrajectoryIdentification{
      predicted_object, stamp, "map_based_predicted_path", assumed_acceleration},
    std::move(times), std::move(distances), std::move(poses), std::move(footprints));
}

TrajectoryData generate_diffusion_based_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  rclcpp::Duration start_time, double max_time, const builtin_interfaces::msg::Time & stamp,
  double time_resolution)
{
  const auto most_confident_path_it = std::max_element(
    predicted_object.kinematics.predicted_paths.begin(),
    predicted_object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  const auto & predicted_path = *most_confident_path_it;
  const double prediction_horizon =
    start_time.seconds() + static_cast<double>(predicted_path.path.size() - 1) *
                             rclcpp::Duration(predicted_path.time_step).seconds();
  auto times = detail::compute_sample_times(
    start_time.seconds(), std::min(max_time, prediction_horizon), time_resolution);
  PoseTrajectory poses;
  poses.reserve(times.size());
  for (const auto & time : times) {
    poses.push_back(
      detail::interpolate_predicted_path_pose(predicted_path, time, start_time.seconds()));
  }

  TravelDistanceTrajectory distances;
  distances.reserve(poses.size());
  distances.push_back(0.0);
  for (size_t i = 1; i < poses.size(); ++i) {
    distances.push_back(
      distances.back() +
      autoware_utils_geometry::calc_distance2d(poses.at(i - 1).position, poses.at(i).position));
  }

  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    TrajectoryIdentification{predicted_object, stamp, "diffusion_based_trajectory", 0.0},
    std::move(times), std::move(distances), std::move(poses), std::move(footprints));
}

TrajectoryData generate_constant_curvature_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time,
  const builtin_interfaces::msg::Time & stamp, double time_resolution)
{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, braking_lag,
    assumed_acceleration, start_time.seconds(), max_time, time_resolution);

  auto poses = pose::constant_curvature_predictor::compute(
    predicted_object.kinematics.initial_pose_with_covariance.pose,
    predicted_object.kinematics.initial_twist_with_covariance.twist, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    TrajectoryIdentification{
      predicted_object, stamp, "constant_curvature_path", assumed_acceleration},
    std::move(times), std::move(distances), std::move(poses), std::move(footprints));
}

// todo: refactor with generate_object_trajectories().
TrajectoryData generate_object_trajectory(
  const FilterContext & context, const unique_identifier_msgs::msg::UUID object_id,
  const std::string & traj_type_str, const double acc, const double time_resolution,
  const double time_horizon)
{
  // todo: remove this lamda from generate_object_trajectory()
  const auto find_predicted_object =
    [&object_id](
      const auto & predicted_objects) -> const autoware_perception_msgs::msg::PredictedObject & {
    auto it = std::find_if(
      predicted_objects.begin(), predicted_objects.end(),
      [&object_id](const auto & object) { return object.object_id == object_id; });
    assert(it != predicted_objects.end());
    return *it;
  };

  if (traj_type_str.find("map_based_predicted_path") != std::string::npos) {
    assert(context.predicted_objects);
    const auto & predicted_object = find_predicted_object(context.predicted_objects->objects);
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    return trajectory::generate_predicted_path_trajectory(
      predicted_object, 0.0, acc, objects_reference_time, time_horizon,
      context.predicted_objects->header.stamp, time_resolution);
  }

  if (traj_type_str.find("constant_curvature_path") != std::string::npos) {
    assert(context.predicted_objects);
    const auto & predicted_object = find_predicted_object(context.predicted_objects->objects);
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    return trajectory::generate_constant_curvature_trajectory(
      predicted_object, 0.0, acc, objects_reference_time, time_horizon,
      context.predicted_objects->header.stamp, time_resolution);
  }

  if (traj_type_str.find("diffusion_based") != std::string::npos) {
    assert(context.neural_network_predicted_objects);
    const auto & predicted_object =
      find_predicted_object(context.neural_network_predicted_objects->objects);
    // todo(takagi):
    // ここでのobjects_reference_timeの使用は不適当。perceptionから出力された時刻と位置で整形すること
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.neural_network_predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    // ここでのgenerate_predicted_path_trajectoryの呼び出しは意図したもの。pathとして解釈して、速度プロファイルは上書きしたい。
    // todo: trajectoryのラベルstringがdiffusionではなく、map_baseadになる課題がある
    return trajectory::generate_predicted_path_trajectory(
      predicted_object, 0.0, acc, objects_reference_time, time_horizon,
      context.neural_network_predicted_objects->header.stamp, time_resolution);
  }

  assert(false);
  throw std::logic_error("Unsupported trajectory type in DRAC assessment: " + traj_type_str);
}

}  // namespace trajectory

// Geometry helpers for overlap checks.
namespace geometry
{

namespace detail
{

template <typename Points>
std::pair<double, double> project_points(const Points & ring, double axis_x, double axis_y)
{
  if (ring.empty()) {
    return {0.0, 0.0};
  }

  const auto project = [&](const auto & point) {
    return boost::geometry::get<0>(point) * axis_x + boost::geometry::get<1>(point) * axis_y;
  };

  auto point_it = ring.begin();
  double min_projection = project(*point_it);
  double max_projection = min_projection;
  for (++point_it; point_it != ring.end(); ++point_it) {
    const double projection = project(*point_it);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
  }

  return {min_projection, max_projection};
}

template <typename ClosedRing>
bool has_separating_axis(
  const ClosedRing & candidate_axes, const ClosedRing & ring_a, const ClosedRing & ring_b)
{
  auto previous_it = candidate_axes.begin();
  for (auto current_it = std::next(candidate_axes.begin()); current_it != candidate_axes.end();
       ++current_it) {
    const double edge_x =
      boost::geometry::get<0>(*current_it) - boost::geometry::get<0>(*previous_it);
    const double edge_y =
      boost::geometry::get<1>(*current_it) - boost::geometry::get<1>(*previous_it);
    const auto [min_a, max_a] = project_points(ring_a, -edge_y, edge_x);
    const auto [min_b, max_b] = project_points(ring_b, -edge_y, edge_x);
    if (max_a < min_b || max_b < min_a) {
      return true;
    }
    previous_it = current_it;
  }

  return false;
}

}  // namespace detail

// todo(takagi): review by myself and consider moving to a more common place if necessary. see
// https://en.wikipedia.org/wiki/Hyperplane_separation_theorem.
// SAT-based intersection check for convex boost::geometry polygons.
template <typename ConvexPolygon>
bool intersects_sat(const ConvexPolygon & poly_a, const ConvexPolygon & poly_b)
{
  const auto & ring_a = poly_a.outer();
  const auto & ring_b = poly_b.outer();

  constexpr size_t minimum_closed_convex_ring_size = 3U;
  if (
    ring_a.size() < minimum_closed_convex_ring_size ||
    ring_b.size() < minimum_closed_convex_ring_size) {
    return false;
  }

  return !detail::has_separating_axis(ring_a, ring_a, ring_b) &&
         !detail::has_separating_axis(ring_b, ring_a, ring_b);
}

// todo(takagi): relace autoware_utils_geometry::to_polygon2d with this function. autoware_utils' s
// version takes more malloc cost.
Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape)
{
  Polygon2d polygon;

  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
  transform.linear() = Eigen::Rotation2Dd(tf2::getYaw(pose.orientation)).toRotationMatrix();
  transform.translation() = Eigen::Vector2d{pose.position.x, pose.position.y};

  auto append_transformed_point =
    [&](const Eigen::Isometry2d & transform, const double x, const double y) {
      const Eigen::Vector2d transformed = transform * Eigen::Vector2d{x, y};
      polygon.outer().push_back(Point2d{transformed.x(), transformed.y()});
    };

  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const double half_x = shape.dimensions.x / 2.0;
    const double half_y = shape.dimensions.y / 2.0;

    polygon.outer().reserve(5);
    append_transformed_point(transform, half_x, half_y);
    append_transformed_point(transform, half_x, -half_y);
    append_transformed_point(transform, -half_x, -half_y);
    append_transformed_point(transform, -half_x, half_y);
    polygon.outer().push_back(polygon.outer().front());
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    const double radius = shape.dimensions.x / 2.0;
    constexpr int circle_discrete_num = 6;

    polygon.outer().reserve(circle_discrete_num + 1);
    for (int i = 0; i < circle_discrete_num; ++i) {
      const double theta =
        -1.0 * (static_cast<double>(i) / static_cast<double>(circle_discrete_num)) * 2.0 * M_PI;
      append_transformed_point(transform, std::cos(theta) * radius, std::sin(theta) * radius);
    }
    polygon.outer().push_back(polygon.outer().front());
  } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    polygon.outer().reserve(shape.footprint.points.size() + 1);
    for (const auto & point : shape.footprint.points) {
      append_transformed_point(transform, point.x, point.y);
    }
    if (!polygon.outer().empty()) {
      polygon.outer().push_back(polygon.outer().front());
    }
  } else {
    throw std::logic_error("The shape type is not supported in autoware_utils.");
  }

  return polygon;
}

}  // namespace geometry

// RSS-based required deceleration assessment.
namespace rss_deceleration
{
struct Assessment
{
  TrajectoryIdentification object;
  double required_deceleration;
};

struct Result  // todo(takagi): should be designed to appropriately represent the assessment.
{
  std::optional<Assessment> worst_assessment;
  bool has_violation{false};
  std::vector<Assessment> violations;
};

template <typename PosePoints, typename Object>
double compute_longitudinal_velocity(const PosePoints & points, const Object & object)
{
  if (points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  constexpr double min_path_end_to_end_distance = 1e-3;

  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const bool use_path_yaw =
    points.size() >= 2 && autoware_utils_geometry::calc_distance2d(points.front(), points.back()) >=
                            min_path_end_to_end_distance;
  const double object_yaw_relative_to_points =
    use_path_yaw ? autoware::motion_utils::calcYawDeviation(points, object_pose, true)
                 : autoware::universe_utils::calcYawDeviation(points.front(), object_pose);
  const Eigen::Rotation2Dd object_to_points_rotation(object_yaw_relative_to_points);

  const auto & object_twist = object.kinematics.initial_twist_with_covariance.twist;
  const Eigen::Vector2d object_velocity_in_object_frame(
    object_twist.linear.x, object_twist.linear.y);
  const Eigen::Vector2d object_velocity_in_points_frame =
    object_to_points_rotation * object_velocity_in_object_frame;

  return object_velocity_in_points_frame.x();
}

std::optional<double> compute_distance_to_collision(
  const TrajectoryData & ego_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto object_footprint =
    geometry::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape);
  const auto object_envelope = boost::geometry::return_envelope<Box2d>(object_footprint);

  if (!boost::geometry::intersects(
        ego_trajectory.get_or_compute_overall_envelope(), object_envelope)) {
    return std::nullopt;
  }

  for (size_t i = 0; i < ego_trajectory.size(); ++i) {
    const auto prev_i = (i == 0) ? 0 : (i - 1);
    const auto & ego_footprint = ego_trajectory.get_or_compute_convex(IndexRange{prev_i, i});
    if (geometry::intersects_sat(ego_footprint, object_footprint)) {
      // todo(takagi): for precise calculation, intersection length should be considered.
      return ego_trajectory.getDistances().at(i);
    }
  }

  return std::nullopt;
}

TrajectoryData generate_rss_ego_trajectory(
  const TrajectoryPoints & traj_points, const FilterContext & context, double time_resolution,
  VehicleInfo & vehicle_info)
{
  const double ego_time_horizon_for_rss =
    rclcpp::Duration(traj_points.back().time_from_start).seconds();

  return trajectory::generate_ego_trajectory(
    traj_points, context, ego_time_horizon_for_rss, time_resolution, vehicle_info);
}

Assessment assess_required_deceleration(
  const TrajectoryData & ego_trajectory, const geometry_msgs::msg::Twist & ego_twist,
  const autoware_perception_msgs::msg::PredictedObject & object, const RssParams & rss_params,
  const builtin_interfaces::msg::Time & stamp)
{
  const auto ego_long_vel = ego_twist.linear.x;
  if (ego_long_vel <= 0.0) {
    return Assessment{TrajectoryIdentification{object, stamp}, 0.0};
  }

  // compute current distance
  const auto distance_to_collision =
    rss_deceleration::compute_distance_to_collision(ego_trajectory, object);
  if (!distance_to_collision.has_value()) {
    return Assessment{TrajectoryIdentification{object, stamp}, 0.0};
  }

  // compute safe distance
  const double obj_long_vel = std::clamp(
    rss_deceleration::compute_longitudinal_velocity(ego_trajectory.getPoses(), object), 0.0, 30.0);
  const double safe_distance =
    distance_to_collision.value() - rss_params.stop_distance_margin +
    obj_long_vel * obj_long_vel * 0.5 / -rss_params.object_assumed_acceleration -
    ego_long_vel * rss_params.ego_total_braking_delay;

  // compute required deceleration
  const double required_deceleration = safe_distance <= 0.0
                                         ? std::numeric_limits<double>::infinity()
                                         : ego_long_vel * ego_long_vel * 0.5 / safe_distance;

  return Assessment{TrajectoryIdentification{object, stamp}, required_deceleration};
}

Result assess(
  const TrajectoryPoints & traj_points, const FilterContext & context, const RssParams & rss_params,
  double time_resolution, VehicleInfo & vehicle_info)

{
  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    return {};
  }

  const auto ego_trajectory =
    generate_rss_ego_trajectory(traj_points, context, time_resolution, vehicle_info);

  Result result{};
  result.violations.reserve(context.predicted_objects->objects.size());

  for (const auto & object : context.predicted_objects->objects) {
    const auto assessment = assess_required_deceleration(
      ego_trajectory, context.odometry->twist.twist, object, rss_params,
      context.predicted_objects->header.stamp);

    if (
      !result.worst_assessment.has_value() ||
      assessment.required_deceleration > result.worst_assessment->required_deceleration) {
      result.worst_assessment = assessment;
    }

    if (assessment.required_deceleration > -rss_params.error_threshold.ego_acceleration) {
      result.has_violation = true;
      result.violations.push_back(assessment);
    }
  }

  return result;
}

}  // namespace rss_deceleration

// Collision timing assessment.
namespace collision_timing_assessment
{

struct Finding
{
  TrajectoryIdentification object_identification;
  double pet;
  double ttc;
  PoseTrajectory ego_trajectory;
  PoseTrajectory object_trajectory;
  Polygon2d ego_hull;
  Polygon2d object_hull;
};

struct Result
{
  std::vector<Finding> planned_speed_findings;
  std::optional<double> drac{0.0};
  std::vector<Finding> drac_findings;  // Last evaluated PET findings during DRAC search.
};

struct ObjectTrajectoryGenerationOptions
{
  bool predicted_path_trajectory{false};
  bool constant_curvature_trajectory{false};
  bool diffusion_based_trajectory{false};

  ObjectTrajectoryGenerationOptions() = default;

  template <typename ParamsT>
  explicit ObjectTrajectoryGenerationOptions(const ParamsT & params)
  {
    predicted_path_trajectory = params.assessment_trajectories.map_based;
    constant_curvature_trajectory = params.assessment_trajectories.constant_curvature;
    diffusion_based_trajectory = params.assessment_trajectories.diffusion_based;
  }

  void merge_with(const ObjectTrajectoryGenerationOptions & other)
  {
    predicted_path_trajectory = predicted_path_trajectory || other.predicted_path_trajectory;
    constant_curvature_trajectory =
      constant_curvature_trajectory || other.constant_curvature_trajectory;
    diffusion_based_trajectory = diffusion_based_trajectory || other.diffusion_based_trajectory;
  }
};

bool is_target_trajectory_type(
  const ObjectTrajectoryGenerationOptions & options, const std::string & trajectory_type)
{
  // Match by trajectory family so this stays robust across small label variants.
  if (trajectory_type.find("diffusion_based_trajectory") != std::string::npos) {
    return options.diffusion_based_trajectory;
  }
  if (trajectory_type.find("constant_curvature_path") != std::string::npos) {
    return options.constant_curvature_trajectory;
  }
  if (trajectory_type.find("map_based_predicted_path") != std::string::npos) {
    return options.predicted_path_trajectory;
  }
  return false;
}

struct DracAssessment
{
  std::optional<double> drac{0.0};
  std::vector<Finding> findings;
};

// todo: remove for loop
std::vector<TrajectoryData> generate_object_trajectories(
  const FilterContext & context, double required_time_horizon, double object_assumed_acceleration,
  double time_resolution, const ObjectTrajectoryGenerationOptions & options)
{
  std::vector<TrajectoryData> object_trajectories{};

  if (context.predicted_objects) {
    const auto trajectory_num_per_object =
      static_cast<size_t>(options.predicted_path_trajectory) +
      static_cast<size_t>(options.constant_curvature_trajectory);
    object_trajectories.reserve(
      object_trajectories.size() +
      context.predicted_objects->objects.size() * trajectory_num_per_object);
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    for (const auto & object : context.predicted_objects->objects) {
      if (options.predicted_path_trajectory && !object.kinematics.predicted_paths.empty()) {
        object_trajectories.push_back(trajectory::generate_predicted_path_trajectory(
          object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
          context.predicted_objects->header.stamp, time_resolution));
      }

      if (options.constant_curvature_trajectory) {
        object_trajectories.push_back(trajectory::generate_constant_curvature_trajectory(
          object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
          context.predicted_objects->header.stamp, time_resolution));
      }
    }
  }

  if (options.diffusion_based_trajectory && context.neural_network_predicted_objects) {
    object_trajectories.reserve(
      object_trajectories.size() + context.neural_network_predicted_objects->objects.size());
    const rclcpp::Duration neural_network_objects_reference_time =
      rclcpp::Time(context.neural_network_predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    for (const auto & object : context.neural_network_predicted_objects->objects) {
      if (object.kinematics.predicted_paths.empty()) {
        continue;
      }
      object_trajectories.push_back(trajectory::generate_diffusion_based_trajectory(
        object, neural_network_objects_reference_time, required_time_horizon,
        context.neural_network_predicted_objects->header.stamp, time_resolution));
    }
  }
  return object_trajectories;
}

std::optional<Finding> find_collision_timing(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  double positive_pet_threshold, double negative_pet_threshold, double time_resolution)
{
  const double before_pet_threshold = std::abs(negative_pet_threshold);
  const double after_pet_threshold = std::max(0.0, positive_pet_threshold);
  const double max_pet_threshold = std::max(before_pet_threshold, after_pet_threshold);

  if (!boost::geometry::intersects(
        ref_trajectory.get_or_compute_overall_envelope(),
        test_trajectory.get_or_compute_envelope(TimeRange{
          ref_trajectory.getTimes().front() - before_pet_threshold,
          ref_trajectory.getTimes().back() + after_pet_threshold}))) {
    return std::nullopt;
  }

  struct CandidateFinding
  {
    double ttc;
    double pet;
    IndexRange ref_index_range;
    TimeRange test_time_range;
  };

  // todo: return only trajectory identification and index/time ranges from here,
  // and move add_debug_markers() generation to assessment to avoid carrying debug-only copies.
  const auto make_finding = [&](const CandidateFinding & candidate) -> Finding {
    const auto & object_identification = test_trajectory.getObjectIdentification();
    return Finding{
      object_identification,
      candidate.pet,
      candidate.ttc,
      ref_trajectory.getPoses(),
      test_trajectory.getPoses(),
      ref_trajectory.get_or_compute_convex(candidate.ref_index_range),
      test_trajectory.get_or_compute_convex(candidate.test_time_range)};
  };

  std::optional<CandidateFinding> candidate_finding{};
  for (size_t i = 0; i < ref_trajectory.size(); ++i) {
    size_t prev_i = (i == 0) ? 0 : i - 1;
    const double ref_start_time = ref_trajectory.getTimes().at(prev_i);
    const double ref_end_time = ref_trajectory.getTimes().at(i);

    const IndexRange ref_index_range{prev_i, i};
    const Box2d & ref_envelope = ref_trajectory.get_or_compute_envelope(ref_index_range);
    const Polygon2d & ref_convex = ref_trajectory.get_or_compute_convex(ref_index_range);

    const double current_pet_limit =
      candidate_finding.has_value() ? std::abs(candidate_finding->pet) : max_pet_threshold;

    if (!boost::geometry::intersects(
          ref_envelope, test_trajectory.get_or_compute_envelope(TimeRange{
                          ref_start_time - current_pet_limit,
                          ref_trajectory.getTimes().back() + current_pet_limit}))) {
      continue;
    }

    const auto has_intersects = [&](const TimeRange & time_range) -> bool {
      if (!boost::geometry::intersects(
            ref_envelope, test_trajectory.get_or_compute_envelope(time_range))) {
        return false;
      }

      return geometry::intersects_sat(
        ref_convex, test_trajectory.get_or_compute_convex(time_range));
    };

    for (double pet_range = 0.0; pet_range < current_pet_limit; pet_range += time_resolution) {
      const TimeRange test_time_range_before{ref_start_time - pet_range, ref_end_time - pet_range};
      const bool has_intersects_before = has_intersects(test_time_range_before);

      const TimeRange test_time_range_after{ref_start_time + pet_range, ref_end_time + pet_range};
      const bool has_intersects_after = has_intersects(test_time_range_after);

      if (!has_intersects_before && !has_intersects_after) {
        continue;
      }

      const double pet = has_intersects_before ? -pet_range : pet_range;
      const TimeRange test_time_range =
        has_intersects_before ? test_time_range_before : test_time_range_after;

      candidate_finding = CandidateFinding{ref_start_time, pet, ref_index_range, test_time_range};
      break;
    }
    if (candidate_finding.has_value() && candidate_finding->pet == 0.0) {
      return make_finding(candidate_finding.value());
    }
  }

  if (!candidate_finding.has_value()) {
    return std::nullopt;
  }

  return make_finding(candidate_finding.value());
}

std::vector<Finding> assess_planned_speed_collision_timing(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const validator::Params::CollisionCheck::PetCollision & pet_collision_params,
  double time_resolution, VehicleInfo & vehicle_info,
  const std::vector<TrajectoryData> & object_trajectories)
{
  const double ego_time_horizon_for_pet = std::abs(context.odometry->twist.twist.linear.x) * 0.5 /
                                            -pet_collision_params.ego_assumed_acceleration +
                                          pet_collision_params.ego_total_braking_delay;
  auto ego_trajectory = trajectory::generate_ego_trajectory(
    traj_points, context, ego_time_horizon_for_pet, time_resolution, vehicle_info);

  std::vector<Finding> findings{};
  findings.reserve(object_trajectories.size());

  for (const auto & object_trajectory : object_trajectories) {
    if (!is_target_trajectory_type(
          ObjectTrajectoryGenerationOptions{pet_collision_params},
          object_trajectory.getObjectIdentification().trajectory_type)) {
      continue;
    }

    auto finding = find_collision_timing(
      ego_trajectory, object_trajectory,
      pet_collision_params.warn_threshold.ego_first_passing_time_gap,
      -pet_collision_params.warn_threshold.object_first_passing_time_gap, time_resolution);
    if (finding.has_value()) {
      findings.push_back(std::move(finding.value()));
    }
  }

  return findings;
}

DracAssessment assess_drac(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const validator::Params::CollisionCheck::Drac & drac_params, VehicleInfo & vehicle_info,
  const std::vector<TrajectoryData> & object_trajectories,
  const validator::Params::CollisionCheck::GlobalSetting & global_setting)
{
  const double ego_time_horizon = rclcpp::Duration(traj_points.back().time_from_start).seconds();

  constexpr double DEFAULT_EGO_DECELERATION_STEP = 1.0;
  constexpr double DEFAULT_MAX_EGO_DECELERATION = 6.0;
  std::vector<Finding> last_findings{};

  for (double ego_dec = 0.0; ego_dec < DEFAULT_MAX_EGO_DECELERATION + 1e-3;
       ego_dec += DEFAULT_EGO_DECELERATION_STEP) {
    const auto ego_deceleration_trajectory = [&]() {
      if (ego_dec == 0.0) {
        return trajectory::generate_ego_trajectory(
          traj_points, context, ego_time_horizon, global_setting.time_resolution, vehicle_info);
      } else if (ego_dec > DEFAULT_MAX_EGO_DECELERATION - 1e-3) {
        return trajectory::generate_ego_trajectory(
          context.odometry->twist.twist, 0.0, -ego_dec, ego_time_horizon,
          global_setting.time_resolution, traj_points, vehicle_info);
      }
      return trajectory::generate_ego_trajectory(
        context.odometry->twist.twist, drac_params.ego_total_braking_delay, -ego_dec,
        ego_time_horizon, global_setting.time_resolution, traj_points, vehicle_info);
    }();

    std::vector<Finding> findings{};
    findings.reserve(object_trajectories.size());
    for (const auto & object_trajectory : object_trajectories) {
      if (!is_target_trajectory_type(
            ObjectTrajectoryGenerationOptions{drac_params},
            object_trajectory.getObjectIdentification().trajectory_type)) {
        continue;
      }
      // todo: prepare parameter
      constexpr double drac_params_collision_time_threshold = 1.0;
      auto finding_nominal_object_motion = find_collision_timing(
        ego_deceleration_trajectory, object_trajectory, drac_params_collision_time_threshold,
        -drac_params_collision_time_threshold, global_setting.time_resolution);
      if (!finding_nominal_object_motion.has_value()) {
        continue;
      }

      const auto & traj_type_str = object_trajectory.getObjectIdentification().trajectory_type;
      const auto & object_id = object_trajectory.getObjectIdentification().uuid;

      const auto object_deceleration_trajectory = trajectory::generate_object_trajectory(
        context, object_id, traj_type_str, -ego_dec, global_setting.time_resolution,
        ego_time_horizon + drac_params_collision_time_threshold);

      auto finding_dec_object_motion = find_collision_timing(
        ego_deceleration_trajectory, object_deceleration_trajectory,
        drac_params_collision_time_threshold, -drac_params_collision_time_threshold,
        global_setting.time_resolution);
      if (!finding_dec_object_motion.has_value()) {
        continue;
      }

      findings.push_back(std::move(finding_nominal_object_motion.value()));
      findings.push_back(std::move(finding_dec_object_motion.value()));
    }
    if (findings.empty()) {
      return DracAssessment{ego_dec, std::move(last_findings)};
    }

    last_findings = std::move(findings);
  }

  return DracAssessment{std::nullopt, std::move(last_findings)};
}

Result assess(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const PetCollisionParams & pet_collision_params, const DracParams & drac_params,
  const validator::Params::CollisionCheck::GlobalSetting & global_setting,
  VehicleInfo & vehicle_info)
{
  ObjectTrajectoryGenerationOptions required_trajectory_types;
  if (pet_collision_params.enable_assessment) {
    required_trajectory_types.merge_with(ObjectTrajectoryGenerationOptions{pet_collision_params});
  }
  if (drac_params.enable_assessment) {
    required_trajectory_types.merge_with(ObjectTrajectoryGenerationOptions{drac_params});
  }

  // todo: std::max(pet_collision_params.collision_time_threshold,
  // drac_params.collision_time_threshold)
  const double required_time_horizon =
    rclcpp::Duration(traj_points.back().time_from_start).seconds() +
    pet_collision_params.warn_threshold.ego_first_passing_time_gap;
  const auto nominal_speed_object_trajectories = generate_object_trajectories(
    context, required_time_horizon, -1.0, global_setting.time_resolution,
    required_trajectory_types);

  Result result{};
  if (!pet_collision_params.enable_assessment) {
    result.planned_speed_findings = {};
  } else {
    result.planned_speed_findings = assess_planned_speed_collision_timing(
      traj_points, context, pet_collision_params, global_setting.time_resolution, vehicle_info,
      nominal_speed_object_trajectories);
  }

  if (!drac_params.enable_assessment) {
    DracAssessment drac_assessment{0.0, {}};  // dummy
    result.drac_findings = drac_assessment.findings;
    result.drac = drac_assessment.drac;
  } else {
    const auto drac_assessment = assess_drac(
      traj_points, context, drac_params, vehicle_info, nominal_speed_object_trajectories,
      global_setting);
    result.drac_findings = drac_assessment.findings;
    result.drac = drac_assessment.drac;
  }
  return result;
}

}  // namespace collision_timing_assessment

void CollisionCheckFilter::update_parameters(const validator::Params & params)
{
  create_param_maps(params);

  pet_collision_params_ = pet_collision_param_map_.at("base");
  rss_params_ = rss_param_map_.at("base");
  drac_params_ = drac_param_map_.at("base");

  global_setting_ = params.collision_check.global_setting;
}

autoware_internal_planning_msgs::msg::SafetyFactorArray make_safety_factor_array(
  const builtin_interfaces::msg::Time & stamp, const collision_timing_assessment::Finding & finding,
  const std::string & collision_type, double time_resolution)
{
  using autoware_internal_planning_msgs::msg::SafetyFactor;
  using autoware_internal_planning_msgs::msg::SafetyFactorArray;

  SafetyFactor safety_factor;
  safety_factor.type = SafetyFactor::OBJECT;
  safety_factor.object_id = finding.object_identification.uuid;
  safety_factor.ttc_begin = static_cast<float>(finding.ttc);
  safety_factor.ttc_end = static_cast<float>(finding.ttc + time_resolution);
  safety_factor.is_safe = false;
  if (!finding.object_trajectory.empty()) {
    safety_factor.points.push_back(finding.object_trajectory.front().position);
  }

  SafetyFactorArray safety_factors;
  safety_factors.header.stamp = stamp;
  safety_factors.header.frame_id = "map";
  safety_factors.factors.push_back(std::move(safety_factor));
  safety_factors.is_safe = false;
  safety_factors.detail = collision_type;
  return safety_factors;
}

void CollisionCheckFilter::create_param_maps(const validator::Params & params)
{
  pet_collision_param_map_.clear();
  rss_param_map_.clear();
  drac_param_map_.clear();

  const validator::Params::CollisionCheck::PetCollision & pet =
    params.collision_check.pet_collision;
  const validator::Params::CollisionCheck::Rss & rss = params.collision_check.rss;
  const validator::Params::CollisionCheck::Drac & drac = params.collision_check.drac;

  static constexpr const char * k_base = "base";
  // Class labels: keep in sync with parameter_struct.yaml and extract_labeled_param().
  static constexpr std::array<const char *, 12> k_object_class_keys{
    "car",        "truck",  "bus",    "trailer",       "motorcycle",     "bicycle",
    "pedestrian", "animal", "hazard", "over_drivable", "under_drivable", "unknown",
  };

  pet_collision_param_map_[k_base] = PetCollisionParams(pet, k_base);
  rss_param_map_[k_base] = RssParams(rss, k_base);
  drac_param_map_[k_base] = DracParams(drac, k_base);

  for (const char * class_key : k_object_class_keys) {
    pet_collision_param_map_[class_key] = PetCollisionParams(pet, class_key);
    rss_param_map_[class_key] = RssParams(rss, class_key);
    drac_param_map_[class_key] = DracParams(drac, class_key);
  }
}

void CollisionCheckFilter::add_debug_markers(
  const rclcpp::Time & stamp, const std::string & ns, const std::string & trajectory_id,
  const PoseTrajectory & ego_trajectory, const PoseTrajectory & object_trajectory,
  const Polygon2d & ego_hull, const Polygon2d & object_hull)
{
  int id = debug_markers_.markers.empty() ? 0 : debug_markers_.markers.back().id + 1;

  struct Color
  {
    float r;
    float g;
    float b;
  };
  const auto resolve_trajectory_color = [&](const std::string & id_str) {
    if (id_str.find("_diffusion_based_trajectory") != std::string::npos) {
      return Color{1.0F, 0.55F, 0.0F};
    }
    if (id_str.find("_constant_curvature_path") != std::string::npos) {
      return Color{0.0F, 0.75F, 1.0F};
    }
    return Color{0.2F, 1.0F, 0.2F};
  };
  const auto trajectory_color = resolve_trajectory_color(trajectory_id);

  auto add_poly_marker =
    [&](const Polygon2d & poly, const std::string & local_namespace, float r, float g, float b) {
      if (poly.outer().empty()) return;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = ns + "/" + local_namespace;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.05;  // line width
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 0.9;

      for (const auto & p : poly.outer()) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = 0.0;
        m.points.push_back(pt);
      }
      // close the polygon by adding the first point at the end
      geometry_msgs::msg::Point pt_first;
      pt_first.x = poly.outer().front().x();
      pt_first.y = poly.outer().front().y();
      pt_first.z = 0.0;
      m.points.push_back(pt_first);

      debug_markers_.markers.push_back(std::move(m));
    };

  auto add_trajectory_marker = [&](
                                 const PoseTrajectory & trajectory,
                                 const std::string & local_namespace, float r, float g, float b,
                                 float alpha) {
    if (trajectory.empty()) return;

    for (const auto & pose : trajectory) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = ns + "/" + local_namespace;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = pose;
      m.scale.x = 0.3;
      m.scale.y = 0.18;
      m.scale.z = 0.18;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = alpha;
      debug_markers_.markers.push_back(std::move(m));
    }
  };

  add_poly_marker(ego_hull, "ego_worst_pet", 0.0, 0.0, 1.0);
  add_poly_marker(object_hull, "obj_worst_pet", 1.0, 0.0, 0.0);
  add_trajectory_marker(ego_trajectory, "ego_trajectory", 1.0F, 1.0F, 1.0F, 0.9F);
  add_trajectory_marker(
    object_trajectory, "object_trajectory", trajectory_color.r, trajectory_color.g,
    trajectory_color.b, 0.95F);
}

void CollisionCheckFilter::add_error_text_marker(
  const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & ego_pose,
  const std::string & error_msg)
{
  int id = debug_markers_.markers.empty() ? 0 : debug_markers_.markers.back().id + 1;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = stamp;
  m.ns = "collision_check_error";
  m.id = id++;
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.z = 0.6;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 1.0;
  m.color.a = 0.95;
  m.pose = ego_pose;
  m.pose.position.z += 1.0;
  m.text = error_msg;
  debug_markers_.markers.push_back(std::move(m));
}

void CollisionCheckFilter::clear_detection_times()
{
  pet_continuous_times_.clear();
  rss_continuous_times_.clear();
  drac_continuous_times_.clear();
}

void log_collision_messages(const uint8_t level, const std::string & messages)
{
  if (messages.empty()) {
    return;
  }
  if (level == MetricReport::ERROR) {
    RCLCPP_ERROR(rclcpp::get_logger("CollisionCheckFilter"), "Not feasible: %s", messages.c_str());
    return;
  }
  RCLCPP_WARN(rclcpp::get_logger("CollisionCheckFilter"), "Warning: %s", messages.c_str());
}

void append_text_marker_message(std::string & text, const std::string & message)
{
  if (!message.empty()) {
    text += message + "\n";
  }
}

using AddDebugMarkers = std::function<void(
  const rclcpp::Time &, const std::string &, const std::string &, const PoseTrajectory &,
  const PoseTrajectory &, const Polygon2d &, const Polygon2d &)>;

using AddPlanningFactor = std::function<void(
  const builtin_interfaces::msg::Time &, const geometry_msgs::msg::Pose &,
  const collision_timing_assessment::Finding &, const std::string &,
  autoware_internal_planning_msgs::msg::PlanningFactorArray &)>;

void add_collision_planning_factor(
  const double time_resolution, const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & ego_pose, const collision_timing_assessment::Finding & finding,
  const std::string & collision_type,
  autoware_internal_planning_msgs::msg::PlanningFactorArray & planning_factors)
{
  const auto safety_factors =
    make_safety_factor_array(stamp, finding, collision_type, time_resolution);
  const auto control_point =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::ControlPoint>()
      .pose(ego_pose)
      .velocity(0.0)
      .shift_length(0.0)
      .distance(0.0);
  auto factor =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::PlanningFactor>()
      .module("")
      .is_driving_forward(true)
      .control_points({control_point})
      .behavior(autoware_internal_planning_msgs::msg::PlanningFactor::STOP)
      .detail(collision_type)
      .safety_factors(safety_factors);
  planning_factors.factors.push_back(std::move(factor));
}

void process_pet_findings(
  const std::string & validator_name, const std::string & validator_category,
  const validator::Params::CollisionCheck::PetCollision & pet_collision_params,
  ContinuousDetectionTimes & pet_continuous_times, const rclcpp::Time & current_time,
  const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<collision_timing_assessment::Finding> & findings,
  EvaluationArtifacts & artifacts, const AddDebugMarkers & add_debug_markers,
  const AddPlanningFactor & add_planning_factor)
{
  pet_continuous_times.update(current_time, findings, [](const auto & finding) {
    return finding.object_identification.trajectory_id_string();
  });

  std::string log_messages{};
  std::string marker_messages{};
  uint8_t log_level = MetricReport::WARN;
  for (const auto & finding : findings) {
    const auto & obj_id = finding.object_identification;
    const bool is_error =
      finding.pet <= pet_collision_params.error_threshold.ego_first_passing_time_gap &&
      finding.pet >= -pet_collision_params.error_threshold.object_first_passing_time_gap;
    const uint8_t metric_level = is_error ? MetricReport::ERROR : MetricReport::WARN;
    if (is_error) {
      artifacts.is_feasible = false;
      log_level = MetricReport::ERROR;
    }

    artifacts.metrics.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(validator_name)
        .validator_category(validator_category)
        .metric_name(
          fmt::format("check_PET_{}_{}", obj_id.trajectory_id_string(), obj_id.classification))
        .metric_value(finding.pet)
        .level(metric_level));
    artifacts.metrics.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(validator_name)
        .validator_category(validator_category)
        .metric_name(
          fmt::format("check_TTC_{}_{}", obj_id.trajectory_id_string(), obj_id.classification))
        .metric_value(finding.ttc)
        .level(metric_level));

    const auto finding_msg = fmt::format(
      "PET collision, classification: {}, ID: {}, PET: {}, TTC: {}, duration: {}, stamp: {}.{};",
      obj_id.classification, obj_id.trajectory_id_string(), finding.pet, finding.ttc,
      pet_continuous_times.get_time(obj_id.trajectory_id_string()), obj_id.stamp.sec,
      obj_id.stamp.nanosec);
    log_messages += finding_msg;
    append_text_marker_message(marker_messages, finding_msg);
    add_debug_markers(
      stamp, "planned_speed_collision", obj_id.trajectory_id_string(), finding.ego_trajectory,
      finding.object_trajectory, finding.ego_hull, finding.object_hull);
    if (is_error) {
      add_planning_factor(stamp, ego_pose, finding, "PET", artifacts.planning_factors);
    }
  }

  artifacts.error_msg += marker_messages;
  log_collision_messages(log_level, log_messages);
}

void process_drac_findings(
  const std::string & validator_name, const std::string & validator_category,
  const validator::Params::CollisionCheck::Drac & drac_params,
  ContinuousDetectionTimes & drac_continuous_times, const rclcpp::Time & current_time,
  const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Pose & ego_pose,
  const collision_timing_assessment::Result & collision_timing_result,
  EvaluationArtifacts & artifacts, const AddDebugMarkers & add_debug_markers,
  const AddPlanningFactor & add_planning_factor)
{
  drac_continuous_times.update(
    current_time, collision_timing_result.drac_findings,
    [](const auto & finding) { return finding.object_identification.trajectory_id_string(); });

  const bool is_warn =
    collision_timing_result.drac == std::nullopt ||
    collision_timing_result.drac.value() >= -drac_params.warn_threshold.ego_acceleration;
  const bool is_error =
    collision_timing_result.drac == std::nullopt ||
    collision_timing_result.drac.value() >= -drac_params.error_threshold.ego_acceleration;
  if (!is_warn) {
    return;
  }

  std::string log_messages{};
  std::string marker_messages{};
  const uint8_t metric_level = is_error ? MetricReport::ERROR : MetricReport::WARN;
  for (const auto & finding : collision_timing_result.drac_findings) {
    const auto & obj_id = finding.object_identification;
    if (is_error) {
      artifacts.is_feasible = false;
    }

    artifacts.metrics.push_back(autoware_trajectory_validator::build<MetricReport>()
                                  .validator_name(validator_name)
                                  .validator_category(validator_category)
                                  .metric_name(fmt::format(
                                    "check_DRAC_{}_{}_{}", obj_id.trajectory_id_string(),
                                    obj_id.classification, obj_id.trajectory_type))
                                  .metric_value(collision_timing_result.drac.value_or(0.0))
                                  .level(metric_level));

    const auto finding_msg = fmt::format(
      "DRAC collision, ID: {}, PET: {}, TTC: {}, DRAC: {}, stamp: {}.{};",
      obj_id.trajectory_id_string(), finding.pet, finding.ttc,
      collision_timing_result.drac.has_value()
        ? std::to_string(collision_timing_result.drac.value())
        : "Cant be avoided",
      obj_id.stamp.sec, obj_id.stamp.nanosec);
    log_messages += finding_msg;
    append_text_marker_message(marker_messages, finding_msg);
    add_debug_markers(
      stamp, "drac_collision", obj_id.trajectory_id_string(), finding.ego_trajectory,
      finding.object_trajectory, finding.ego_hull, finding.object_hull);
    if (is_error) {
      add_planning_factor(stamp, ego_pose, finding, "DRAC", artifacts.planning_factors);
    }
  }

  artifacts.error_msg += marker_messages;
  log_collision_messages(metric_level, log_messages);
}

void process_rss_violations(
  const std::string & validator_name, const std::string & validator_category,
  const validator::Params::CollisionCheck::GlobalSetting & global_setting,
  const validator::Params::CollisionCheck::Rss & rss_params, const TrajectoryPoints & traj_points,
  const FilterContext & context, VehicleInfo & vehicle_info,
  ContinuousDetectionTimes & rss_continuous_times, const rclcpp::Time & current_time,
  EvaluationArtifacts & artifacts)
{
  if (!rss_params.enable_assessment) {
    return;
  }

  const auto rss_result = rss_deceleration::assess(
    traj_points, context, rss_params, global_setting.time_resolution, vehicle_info);
  rss_continuous_times.update(current_time, rss_result.violations, [](const auto & violation) {
    return violation.object.object_id_string();
  });

  std::string log_messages{};
  std::string marker_messages{};
  for (const auto & violation : rss_result.violations) {
    const auto object_id = violation.object.object_id_string();
    artifacts.is_feasible = false;
    artifacts.metrics.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(validator_name)
        .validator_category(validator_category)
        .metric_name(fmt::format("check_RSS_{}_{}", violation.object.classification, object_id))
        .metric_value(violation.required_deceleration)
        .level(MetricReport::ERROR));

    const auto finding_msg = fmt::format(
      "RSS collision, classification: {}, ID: {}, duration: {}, required deceleration: {}, "
      "stamp: {}.{};",
      violation.object.classification, object_id, rss_continuous_times.get_time(object_id),
      violation.required_deceleration, violation.object.stamp.sec, violation.object.stamp.nanosec);
    log_messages += finding_msg;
    append_text_marker_message(marker_messages, finding_msg);
  }

  artifacts.error_msg += marker_messages;
  log_collision_messages(MetricReport::ERROR, log_messages);
}

CollisionCheckFilter::result_t CollisionCheckFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (
    (!context.predicted_objects || context.predicted_objects->objects.empty()) &&
    (!context.neural_network_predicted_objects ||
     context.neural_network_predicted_objects->objects.empty())) {
    clear_detection_times();
    return {};  // No objects to check collision with
  }

  if (traj_points.empty()) {
    clear_detection_times();
    return {};  // No trajectory to check
  }

  EvaluationArtifacts artifacts{};
  const rclcpp::Time current_time = context.odometry->header.stamp;
  const auto collision_timing_result = collision_timing_assessment::assess(
    traj_points, context, pet_collision_params_, drac_params_, global_setting_, *vehicle_info_ptr_);
  const auto add_debug_markers_cb =
    [this](
      const rclcpp::Time & stamp, const std::string & ns, const std::string & trajectory_id,
      const PoseTrajectory & ego_trajectory, const PoseTrajectory & object_trajectory,
      const Polygon2d & ego_hull, const Polygon2d & object_hull) {
      add_debug_markers(
        stamp, ns, trajectory_id, ego_trajectory, object_trajectory, ego_hull, object_hull);
    };
  const auto add_planning_factor_cb =
    [this](
      const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Pose & ego_pose,
      const collision_timing_assessment::Finding & finding, const std::string & collision_type,
      autoware_internal_planning_msgs::msg::PlanningFactorArray & planning_factors) {
      add_collision_planning_factor(
        global_setting_.time_resolution, stamp, ego_pose, finding, collision_type,
        planning_factors);
    };
  process_pet_findings(
    get_name(), category(), pet_collision_params_, pet_continuous_times_, current_time,
    context.odometry->header.stamp, context.odometry->pose.pose,
    collision_timing_result.planned_speed_findings, artifacts, add_debug_markers_cb,
    add_planning_factor_cb);
  process_drac_findings(
    get_name(), category(), drac_params_, drac_continuous_times_, current_time,
    context.odometry->header.stamp, context.odometry->pose.pose, collision_timing_result, artifacts,
    add_debug_markers_cb, add_planning_factor_cb);
  process_rss_violations(
    get_name(), category(), global_setting_, rss_params_, traj_points, context, *vehicle_info_ptr_,
    rss_continuous_times_, current_time, artifacts);
  if (!artifacts.error_msg.empty()) {
    add_error_text_marker(
      context.odometry->header.stamp, context.odometry->pose.pose, artifacts.error_msg);
  }

  return ValidationResult{
    artifacts.is_feasible, std::move(artifacts.metrics), std::move(artifacts.planning_factors)};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::CollisionCheckFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
