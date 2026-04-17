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
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <any>
#include <array>
#include <cmath>
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
using autoware::object_recognition_utils::convertLabelToString;
using autoware::object_recognition_utils::getHighestProbLabel;

ObjectIdentification make_object_identification(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  return {
    convertLabelToString(getHighestProbLabel(object.classification)),
    autoware_utils_uuid::to_hex_string(object.object_id)};
}

ObjectIdentification make_trajectory_identification(
  const autoware_perception_msgs::msg::PredictedObject & object, const std::string & suffix)
{
  return {
    convertLabelToString(getHighestProbLabel(object.classification)),
    autoware_utils_uuid::to_hex_string(object.object_id) + suffix};
}

// Trajectory generation helpers.
namespace trajectory::time_distance
{
std::pair<TimeTrajectory, TravelDistanceTrajectory> compute_motion_profile_1d(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double start_time, double end_time)
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
  times.reserve(static_cast<size_t>((end_time - start_time) / TIME_RESOLUTION) + 4U);
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
  for (int64_t tick = static_cast<int64_t>(std::floor(start_time / TIME_RESOLUTION)) + 1;; ++tick) {
    const double tick_time = static_cast<double>(tick) * TIME_RESOLUTION;
    if (
      stop_profile.has_value() && times.back() < stop_profile.value().stop_time &&
      tick_time > stop_profile.value().stop_time) {
      // todo(takagi): Investigate if it's necessary to add the stop time to `times`.
      append_sample(stop_profile.value().stop_time);
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
    footprint_trajectory.push_back(autoware_utils_geometry::to_polygon2d(pose, object_shape));
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
}  // namespace detail

TrajectoryData generate_ego_trajectory(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double max_time, const TrajectoryPoints & traj_points, VehicleInfo & vehicle_info)
{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    initial_twist, braking_lag, assumed_acceleration, 0.0, max_time);

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
    ObjectIdentification{"EGO", ""}, std::move(times), std::move(distances), std::move(poses),
    std::move(footprints));
}

TrajectoryData generate_ego_trajectory(
  const TrajectoryPoints & traj_points, const FilterContext & context, double max_time,
  VehicleInfo & vehicle_info)
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
  for (double sample_time = TIME_RESOLUTION; start_time + sample_time < end_time;
       sample_time =
         std::floor((sample_time + TIME_RESOLUTION + 1e-6) / TIME_RESOLUTION) * TIME_RESOLUTION) {
    relative_times.push_back(sample_time);
    absolute_times.push_back(start_time + sample_time);
  }

  auto poses = pose::compute_pose_trajectory_from_time(traj_points, absolute_times);
  auto distances = detail::compute_cumulative_distances(poses);
  auto footprints = footprint::compute_footprint_trajectory(poses, vehicle_info);

  return TrajectoryData(
    ObjectIdentification{"EGO", ""}, std::move(relative_times), std::move(distances),
    std::move(poses), std::move(footprints));
}

TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time)
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
                  rclcpp::Duration(most_confident_path_it->time_step).seconds()));

  auto poses = pose::compute_pose_trajectory(most_confident_path_it->path, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    make_trajectory_identification(predicted_object, "_predicted_path"), std::move(times),
    std::move(distances), std::move(poses), std::move(footprints));
}

TrajectoryData generate_constant_curvature_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time)
{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, braking_lag,
    assumed_acceleration, start_time.seconds(), max_time);

  auto poses = pose::constant_curvature_predictor::compute(
    predicted_object.kinematics.initial_pose_with_covariance.pose,
    predicted_object.kinematics.initial_twist_with_covariance.twist, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    make_trajectory_identification(predicted_object, "_constant_curvature_path"), std::move(times),
    std::move(distances), std::move(poses), std::move(footprints));
}
}  // namespace trajectory

// Geometry helpers for overlap checks.
namespace geometry
{
template <typename Range>
Box2d compute_overall_envelope(const Range & polygons)
{
  Box2d overall_box;
  boost::geometry::assign_inverse(overall_box);

  for (const auto & poly : polygons) {
    for (const auto & p : poly.outer()) {
      boost::geometry::expand(overall_box, p);
    }
  }

  return overall_box;
}

template <typename Range>
Polygon2d compute_overall_convex_hull(const Range & polygons)
{
  MultiPoint2d all_points;

  all_points.reserve(std::distance(polygons.begin(), polygons.end()) * 4);  // heuristic reserve
  for (const auto & poly : polygons) {
    for (const auto & pt : poly.outer()) {
      all_points.push_back(pt);
    }
  }

  Polygon2d hull;
  boost::geometry::convex_hull(all_points, hull);

  return hull;
}

template <typename Range1, typename Range2>
bool has_overall_convex_hull_overlap(const Range1 & footprints1, const Range2 & footprints2)
{
  const auto overall_box1 = compute_overall_envelope(footprints1);
  const auto overall_box2 = compute_overall_envelope(footprints2);

  if (!boost::geometry::intersects(overall_box1, overall_box2)) {
    return false;
  }

  const auto overall_convex1 = compute_overall_convex_hull(footprints1);
  const auto overall_convex2 = compute_overall_convex_hull(footprints2);
  if (!boost::geometry::intersects(overall_convex1, overall_convex2)) {
    return false;
  }

  return true;
}
}  // namespace geometry

// RSS-based required deceleration assessment.
namespace rss_deceleration
{
struct Assessment
{
  ObjectIdentification object;
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
  const auto object_footprint = autoware_utils_geometry::to_polygon2d(
    object.kinematics.initial_pose_with_covariance.pose, object.shape);
  const auto object_footprint_range =
    boost::make_iterator_range(&object_footprint, &object_footprint + 1);

  if (!geometry::has_overall_convex_hull_overlap(
        ego_trajectory.getFootprints(), object_footprint_range)) {
    return std::nullopt;
  }

  for (size_t i = 0; i < ego_trajectory.size(); ++i) {
    const auto & ego_footprint = ego_trajectory.getFootprints().at(i);
    // todo(takagi): should be check in range of &ego_footprint-1, &ego_footprint+1.
    const auto ego_footprint_range = boost::make_iterator_range(&ego_footprint, &ego_footprint + 1);
    if (geometry::has_overall_convex_hull_overlap(ego_footprint_range, object_footprint_range)) {
      // todo(takagi): for precise calculation, intersection length should be considered.
      return ego_trajectory.getDistances().at(i);
    }
  }

  return std::nullopt;
}

TrajectoryData generate_rss_ego_trajectory(
  const TrajectoryPoints & traj_points, const FilterContext & context, VehicleInfo & vehicle_info)
{
  const double ego_time_horizon_for_rss =
    rclcpp::Duration(traj_points.back().time_from_start).seconds();

  return trajectory::generate_ego_trajectory(
    traj_points, context, ego_time_horizon_for_rss, vehicle_info);
}

Assessment assess_required_deceleration(
  const TrajectoryData & ego_trajectory, const geometry_msgs::msg::Twist & ego_twist,
  const autoware_perception_msgs::msg::PredictedObject & object,
  const validator::Params::CollisionCheck::Rss & rss_params)
{
  const auto ego_long_vel = ego_twist.linear.x;
  if (ego_long_vel <= 0.0) {
    return Assessment{make_object_identification(object), 0.0};
  }

  // compute current distance
  const auto distance_to_collision =
    rss_deceleration::compute_distance_to_collision(ego_trajectory, object);
  if (!distance_to_collision.has_value()) {
    return Assessment{make_object_identification(object), 0.0};
  }

  // compute safe distance
  const double obj_long_vel = std::clamp(
    rss_deceleration::compute_longitudinal_velocity(ego_trajectory.getPoses(), object), 0.0, 30.0);
  const double safe_distance = distance_to_collision.value() - rss_params.stop_margin +
                               obj_long_vel * obj_long_vel * 0.5 / -rss_params.object_acceleration.base -
                               ego_long_vel * rss_params.ego_reaction_time.base;

  // compute required deceleration
  const double required_deceleration = safe_distance <= 0.0
                                         ? std::numeric_limits<double>::infinity()
                                         : ego_long_vel * ego_long_vel * 0.5 / safe_distance;

  return Assessment{make_object_identification(object), required_deceleration};
}

Result assess(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const validator::Params::CollisionCheck::Rss & rss_params, VehicleInfo & vehicle_info)
{
  const auto ego_trajectory = generate_rss_ego_trajectory(traj_points, context, vehicle_info);

  Result result{};
  result.violations.reserve(context.predicted_objects->objects.size());

  for (const auto & object : context.predicted_objects->objects) {
    const auto assessment = assess_required_deceleration(
      ego_trajectory, context.odometry->twist.twist, object, rss_params);

    if (
      !result.worst_assessment.has_value() ||
      assessment.required_deceleration > result.worst_assessment->required_deceleration) {
      result.worst_assessment = assessment;
    }

    if (assessment.required_deceleration > rss_params.ego_deceleration_threshold.base) {
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
  std::string trajectory_id;
  ObjectIdentification object;
  double pet;
  double ttc;
  Polygon2d ego_hull;
  Polygon2d object_hull;
};

struct Result
{
  std::vector<Finding> planned_speed_findings;
  std::optional<double> drac{0.0};
  std::vector<Finding> drac_findings;  // Last evaluated PET findings during DRAC search.
};

struct DracAssessment
{
  std::optional<double> drac{0.0};
  std::vector<Finding> findings;
};

std::vector<TrajectoryData> generate_object_trajectories(
  const FilterContext & context, double required_time_horizon, double object_assumed_acceleration)
{
  const rclcpp::Duration objects_reference_time =
    rclcpp::Time(context.predicted_objects->header.stamp) -
    rclcpp::Time(context.odometry->header.stamp);
  std::vector<TrajectoryData> object_trajectories{};
  object_trajectories.reserve(context.predicted_objects->objects.size() * 2);
  for (const auto & object : context.predicted_objects->objects) {
    object_trajectories.push_back(trajectory::generate_predicted_path_trajectory(
      object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon));

    object_trajectories.push_back(trajectory::generate_constant_curvature_trajectory(
      object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon));
  }

  return object_trajectories;
}

// todo(takagi): should be designed to TTC definition condition, currently minimum PET detected time
// is returned as ttc.
std::optional<Finding> find_collision_timing(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  double pet_threshold)
{
  if (!geometry::has_overall_convex_hull_overlap(
        ref_trajectory.getFootprints(), test_trajectory.getFootprintsInTimeRange(
                                          0.0, ref_trajectory.getTimes().back() + pet_threshold))) {
    return std::nullopt;
  }

  std::optional<Finding> candidate_finding{};
  for (size_t i = 0; i < ref_trajectory.size(); ++i) {
    const double ref_start_time = ref_trajectory.getTimes().at(i);
    const double ref_end_time = ref_start_time + TIME_RESOLUTION;
    const auto ref_poly = ref_trajectory.getFootprintsInTimeRange(ref_start_time, ref_end_time);
    auto check_slice_collision = [&](double start, double end) {
      const auto slice_poly = test_trajectory.getFootprintsInTimeRange(start, end);
      return geometry::has_overall_convex_hull_overlap(ref_poly, slice_poly);
    };

    const double current_pet_limit =
      candidate_finding.has_value() ? candidate_finding->pet : pet_threshold;
    const double test_start_time = ref_start_time - current_pet_limit;
    const double test_end_time = ref_end_time + current_pet_limit;
    if (!check_slice_collision(test_start_time, test_end_time)) {
      continue;
    }

    // todo(takagi): If we only want to know if the value is below the threshold, not the exact
    // value, we can skip this for loop.

    // todo(takagi): return signed PET instead of absolute value.
    for (double pet_range = 0.0; pet_range <= current_pet_limit; pet_range += TIME_RESOLUTION) {
      const double test_start_time_before = ref_start_time - pet_range;
      const double test_end_time_before = test_start_time_before + TIME_RESOLUTION;

      const double test_start_time_after = ref_end_time + pet_range - TIME_RESOLUTION;
      const double test_end_time_after = ref_end_time + pet_range;

      if (
        check_slice_collision(test_start_time_before, test_end_time_before) ||
        check_slice_collision(test_start_time_after, test_end_time_after)) {
        Finding finding;
        finding.trajectory_id = test_trajectory.getObjectIdentification().id;
        finding.object = test_trajectory.getObjectIdentification();
        finding.pet = pet_range;
        finding.ttc = ref_start_time;
        finding.ego_hull = geometry::compute_overall_convex_hull(
          ref_trajectory.getFootprintsInTimeRange(ref_start_time, ref_end_time));
        finding.object_hull = geometry::compute_overall_convex_hull(
          test_trajectory.getFootprintsInTimeRange(test_start_time_before, test_end_time_after));
        candidate_finding = std::move(finding);

        break;
      }
    }
    if (candidate_finding.has_value() && candidate_finding->pet == 0.0) {
      return candidate_finding;
    }
  }

  return candidate_finding;
}

std::vector<Finding> assess_collision_timing(
  const TrajectoryData & ego_trajectory, const std::vector<TrajectoryData> & object_trajectories,
  const validator::Params::CollisionCheck::PetCollision & pet_collision_params)
{
  std::vector<Finding> findings{};
  findings.reserve(object_trajectories.size());

  for (const auto & object_trajectory : object_trajectories) {
    auto finding = find_collision_timing(
      ego_trajectory, object_trajectory, pet_collision_params.collision_time_threshold.base);
    if (finding.has_value()) {
      findings.push_back(std::move(finding.value()));
    }
  }

  return findings;
}

std::vector<Finding> assess_planned_speed_collision_timing(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const validator::Params::CollisionCheck::PetCollision & pet_collision_params,
  VehicleInfo & vehicle_info)
{
  const double ego_time_horizon_for_pet = std::abs(context.odometry->twist.twist.linear.x) * 0.5 /
                                            -pet_collision_params.ego_assumed_acceleration.base +
                                          pet_collision_params.ego_braking_delay;

  // todo: use planned trajectory instead of constant speed assumption to fit the requirements.
  auto ego_trajectory = trajectory::generate_ego_trajectory(
    context.odometry->twist.twist, 0.0, 0.0, ego_time_horizon_for_pet, traj_points, vehicle_info);

  auto object_trajectories = generate_object_trajectories(
    context,
    ego_time_horizon_for_pet + pet_collision_params.collision_time_threshold.base, 0.0);
  return assess_collision_timing(ego_trajectory, object_trajectories, pet_collision_params);
}

DracAssessment assess_drac(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const validator::Params::CollisionCheck::PetCollision & pet_collision_params,
  VehicleInfo & vehicle_info)
{
  const double ego_time_horizon = rclcpp::Duration(traj_points.back().time_from_start).seconds();

  // todo(takagi): reuse the object trajectories for pet assessment for computational efficiency.

  // Currently, the memory allocation in `generate_object_trajectories()` accounts for about half of
  // the total computation time of `is_feasible()`.
  const auto constant_speed_objects_trajectory = generate_object_trajectories(
    context, ego_time_horizon + pet_collision_params.collision_time_threshold.base, 0.0);

  constexpr double DEFAULT_EGO_DECELERATION_STEP = 1.0;
  constexpr double DEFAULT_MAX_EGO_DECELERATION = 6.0;
  std::vector<Finding> last_findings;

  for (double ego_dec = 0.0; ego_dec <= DEFAULT_MAX_EGO_DECELERATION;
       ego_dec += DEFAULT_EGO_DECELERATION_STEP) {
    const auto ego_deceleration_trajectory = [&]() {
      if (ego_dec == 0.0) {
        // todo(takagi): return planned_trajectory();
      } else if (ego_dec > DEFAULT_MAX_EGO_DECELERATION - 1e-3) {
        return trajectory::generate_ego_trajectory(
          context.odometry->twist.twist, 0.0, -ego_dec, ego_time_horizon, traj_points,
          vehicle_info);
      }
      return trajectory::generate_ego_trajectory(
        context.odometry->twist.twist, pet_collision_params.ego_braking_delay, -ego_dec,
        ego_time_horizon, traj_points, vehicle_info);
    }();

    auto findings = assess_collision_timing(
      ego_deceleration_trajectory, constant_speed_objects_trajectory, pet_collision_params);
    if (findings.empty()) {
      return DracAssessment{ego_dec, std::move(last_findings)};
    }

    last_findings = std::move(findings);
  }

  return DracAssessment{std::nullopt, std::move(last_findings)};
}

Result assess(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const validator::Params::CollisionCheck::PetCollision & pet_collision_params,
  VehicleInfo & vehicle_info)
{
  Result result{};
  result.planned_speed_findings =
    assess_planned_speed_collision_timing(traj_points, context, pet_collision_params, vehicle_info);
  // const auto drac_assessment =
  //   assess_drac(traj_points, context, pet_collision_params, vehicle_info);
  DracAssessment drac_assessment{0.0, {}};  // dummy
  result.drac_findings = drac_assessment.findings;
  result.drac = drac_assessment.drac;

  return result;
}

}  // namespace collision_timing_assessment


void CollisionCheckFilter::update_parameters(const validator::Params & params)
{
  pet_collision_params_ = params.collision_check.pet_collision;
  rss_params_ = params.collision_check.rss;
  try {
    create_collision_check_params_map(params);
  } catch (const std::exception &) {
    collision_check_params_map_.clear();
    throw;
  }
}

void CollisionCheckFilter::create_collision_check_params_map(const validator::Params & params)
{
  collision_check_params_map_.clear();

  const validator::Params::CollisionCheck::PetCollision & pet = params.collision_check.pet_collision;
  const validator::Params::CollisionCheck::Rss & rss = params.collision_check.rss;

  const std::function<CollisionCheckParams(const std::string &)> fill_entry =
    [&](const std::string & key) {
      CollisionCheckParams entry{};
      entry.pet_collision_params.ego_braking_delay = 
        try_labeled_double_param(pet.ego_braking_delay, key);
      entry.pet_collision_params.ego_assumed_acceleration = 
        try_labeled_double_param(pet.ego_assumed_acceleration, key);
      entry.pet_collision_params.collision_time_threshold =
        try_labeled_double_param(pet.collision_time_threshold, key);

      entry.rss_params.ego_reaction_time = 
        try_labeled_double_param(rss.ego_reaction_time, key);
      entry.rss_params.ego_deceleration_threshold = 
        try_labeled_double_param(rss.ego_deceleration_threshold, key);
      entry.rss_params.object_acceleration = 
        try_labeled_double_param(rss.object_acceleration, key);
      entry.rss_params.stop_margin = 
        try_labeled_double_param(rss.stop_margin, key);
      return entry;
    };

  static std::vector<std::string> all_class_keys = {
    "car", "truck", "bus", "trailer", "motorcycle", "bicycle", "pedestrian", "animal",
    "hazard", "over_drivable", "under_drivable", "unknown"
  };

  static constexpr const char * k_base = "base";
  collision_check_params_map_[k_base] = fill_entry(k_base);

  for (const std::string & key : all_class_keys) {
    collision_check_params_map_[key] = fill_entry(key);
  }
}

template <typename ParamStruct>
double CollisionCheckFilter::try_labeled_double_param(const ParamStruct & params_struct, const std::string & key)
{
  static constexpr const char k_base_key[] = "base";
  if (key == k_base_key) {
    return params_struct.base;
  }

  double target_params = std::numeric_limits<double>::quiet_NaN();

  if (key == "car") {
    target_params =  params_struct.car;  // fallback to base if "car" specific value is not set
  } else if (key == "truck") {
    target_params =  params_struct.truck;  // fallback to base if "truck" specific value is not set
  } else if (key == "bus") {
    target_params =  params_struct.bus;  // fallback to base if "bus" specific value is not set
  } else if (key == "trailer") {
    target_params =  params_struct.trailer;  // fallback to base if "trailer" specific value is not set
  } else if (key == "motorcycle") {
    target_params =  params_struct.motorcycle;  // fallback to base if "motorcycle" specific value is not set
  } else if (key == "bicycle") {
    target_params =  params_struct.bicycle;  // fallback to base if "bicycle" specific value is not set
  } else if (key == "pedestrian") {
    target_params =  params_struct.pedestrian;
  } else if (key == "animal") {
    target_params =  params_struct.animal;
  } else if (key == "hazard") {
    target_params =  params_struct.hazard;
  } else if (key == "over_drivable") {
    target_params =  params_struct.over_drivable;
  } else if (key == "under_drivable") {
    target_params =  params_struct.under_drivable;
  } else if (key == "unknown") {
    target_params =  params_struct.unknown;
  } else {
    throw std::invalid_argument("Unknown label key: " + key);
  }

  return std::isnan(target_params) ? params_struct.base : target_params;
}

double CollisionCheckFilter::try_labeled_double_param(double shared_value, const std::string & /*key*/)
{
  // Scalar parameters are not split per label; key is ignored.
  return shared_value;
}

void CollisionCheckFilter::add_debug_markers(
  const rclcpp::Time & stamp, const std::string & ns, const Polygon2d & ego_hull,
  const Polygon2d & object_hull)
{
  int id = debug_markers_.markers.empty() ? 0 : debug_markers_.markers.back().id + 1;

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

  add_poly_marker(ego_hull, "ego_worst_pet", 0.0, 0.0, 1.0);
  add_poly_marker(object_hull, "obj_worst_pet", 1.0, 0.0, 0.0);
}

CollisionCheckFilter::result_t CollisionCheckFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  // autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  // stopwatch.tic();

  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    pet_continuous_times_.clear();
    rss_continuous_times_.clear();
    drac_continuous_times_.clear();
    return {};  // No objects to check collision with
  }

  if (traj_points.empty()) {
    pet_continuous_times_.clear();
    rss_continuous_times_.clear();
    drac_continuous_times_.clear();
    return {};  // No trajectory to check
  }

  bool is_feasible = true;
  std::vector<MetricReport> metrics;

  const rclcpp::Time current_time = context.odometry->header.stamp;

  const auto collision_timing_result = collision_timing_assessment::assess(
    traj_points, context, pet_collision_params_, *vehicle_info_ptr_);

  pet_continuous_times_.update(
    current_time, collision_timing_result.planned_speed_findings,
    [](const auto & finding) { return finding.trajectory_id; });
  for (const auto & finding : collision_timing_result.planned_speed_findings) {
    // Mark as infeasible if any finding exists
    is_feasible = false;

    // Record metrics for PET and TTC for each finding
    metrics.push_back(autoware_trajectory_validator::build<MetricReport>()
                        .validator_name(get_name())
                        .validator_category(category())
                        .metric_name(fmt::format(
                          "check_PET_{}_{}_{}", finding.trajectory_id,
                          finding.object.classification, finding.object.id))
                        .metric_value(finding.pet)
                        .level(MetricReport::ERROR));
    metrics.push_back(autoware_trajectory_validator::build<MetricReport>()
                        .validator_name(get_name())
                        .validator_category(category())
                        .metric_name(fmt::format(
                          "check_TTC_{}_{}_{}", finding.trajectory_id,
                          finding.object.classification, finding.object.id))
                        .metric_value(finding.ttc)
                        .level(MetricReport::ERROR));

    add_debug_markers(
      context.odometry->header.stamp, "planned_speed_collision", finding.ego_hull,
      finding.object_hull);
  }

  drac_continuous_times_.update(
    current_time, collision_timing_result.drac_findings,
    [](const auto & finding) { return finding.trajectory_id; });
  if (
    collision_timing_result.drac == std::nullopt ||
    collision_timing_result.drac.value() >=
    -pet_collision_params_.ego_assumed_acceleration.base) {
    for (const auto & finding : collision_timing_result.drac_findings) {
      is_feasible = false;
  
      metrics.push_back(autoware_trajectory_validator::build<MetricReport>()
                          .validator_name(get_name())
                          .validator_category(category())
                          .metric_name(fmt::format(
                            "check_DRAC_{}_{}_{}", finding.trajectory_id, finding.object.classification,
                            finding.object.id))
                          .metric_value(collision_timing_result.drac.value_or(0.0))
                          .level(MetricReport::ERROR));
      add_debug_markers(
        context.odometry->header.stamp, "drac_collision", finding.ego_hull, finding.object_hull);
    }
  }

  const auto rss_result =
    rss_deceleration::assess(traj_points, context, rss_params_, *vehicle_info_ptr_);
  rss_continuous_times_.update(current_time, rss_result.violations, [](const auto & violation) {
    return violation.object.id;
  });
  for (const auto & violation : rss_result.violations) {
    // Mark as infeasible if any RSS violation exists
    is_feasible = false;

    // Record metrics for each RSS violation
    metrics.push_back(autoware_trajectory_validator::build<MetricReport>()
                        .validator_name(get_name())
                        .validator_category(category())
                        .metric_name(fmt::format(
                          "check_RSS_{}_{}", violation.object.classification, violation.object.id))
                        .metric_value(violation.required_deceleration)
                        .level(MetricReport::ERROR));
  }

  return ValidationResult{is_feasible, std::move(metrics)};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::CollisionCheckFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
