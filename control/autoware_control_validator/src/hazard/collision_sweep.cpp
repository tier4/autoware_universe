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

// Ported from autoware_trajectory_validator (autoware.universe 3eb3290355). See the header.

#include "hazard/collision_sweep.hpp"

#include "hazard/collision_target.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::control_validator::hazard
{
namespace time_distance
{
// trajectory_utils.cpp compute_motion_profile_1d() with braking_lag = 0 and
// assumed_acceleration = 0 (constant velocity).
std::pair<TimeTrajectory, TravelDistanceTrajectory> compute_motion_profile_1d(
  const geometry_msgs::msg::Twist & initial_twist, double start_time, double end_time,
  double time_resolution)
{
  const double initial_velocity = std::hypot(initial_twist.linear.x, initial_twist.linear.y);
  if (initial_velocity <= 0.0 || start_time >= end_time) {
    return {{start_time}, {0.0}};
  }

  TimeTrajectory times;
  TravelDistanceTrajectory distances;
  times.reserve(static_cast<size_t>((end_time - start_time) / time_resolution) + 4U);
  distances.reserve(times.capacity());

  constexpr double epsilon = 1e-3;
  auto append_sample = [&](const double t) {
    if (t < start_time || t > end_time) return;
    if (!times.empty() && t < times.back() + epsilon) return;
    times.push_back(t);
    distances.push_back(initial_velocity * t);
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

  return {times, distances};
}
}  // namespace time_distance

namespace pose
{
geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose,
  const double ratio)
{
  geometry_msgs::msg::Pose interpolated_pose;
  interpolated_pose.position.x =
    start_pose.position.x + ratio * (end_pose.position.x - start_pose.position.x);
  interpolated_pose.position.y =
    start_pose.position.y + ratio * (end_pose.position.y - start_pose.position.y);
  interpolated_pose.position.z =
    start_pose.position.z + ratio * (end_pose.position.z - start_pose.position.z);

  const Eigen::Quaterniond start_q(
    start_pose.orientation.w, start_pose.orientation.x, start_pose.orientation.y,
    start_pose.orientation.z);
  const Eigen::Quaterniond end_q(
    end_pose.orientation.w, end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z);
  const Eigen::Quaterniond q = start_q.slerp(ratio, end_q);
  interpolated_pose.orientation.w = q.w();
  interpolated_pose.orientation.x = q.x();
  interpolated_pose.orientation.y = q.y();
  interpolated_pose.orientation.z = q.z();

  return interpolated_pose;
}

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

void correct_vehicle_yaw(
  PoseTrajectory & poses, const double initial_yaw, const double rear_axle_length)
{
  if (poses.empty()) {
    return;
  }
  double yaw = initial_yaw;
  double rear_x = poses.front().position.x - rear_axle_length * std::cos(yaw);
  double rear_y = poses.front().position.y - rear_axle_length * std::sin(yaw);
  poses.front().orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  for (size_t k = 1; k < poses.size(); ++k) {
    const double dx = poses[k].position.x - rear_x;
    const double dy = poses[k].position.y - rear_y;
    if (std::hypot(dx, dy) >= 1e-6) {
      yaw = std::atan2(dy, dx);
    }
    rear_x = poses[k].position.x - rear_axle_length * std::cos(yaw);
    rear_y = poses[k].position.y - rear_axle_length * std::sin(yaw);
    poses[k].orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  }
}
}  // namespace pose

namespace geometry
{
Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
  const double yaw = autoware_utils_geometry::get_rpy(pose).z;
  transform.linear() = Eigen::Rotation2Dd(yaw).toRotationMatrix();
  transform.translation() = Eigen::Vector2d{pose.position.x, pose.position.y};
  return transform;
}

Point2d transform_point(const Eigen::Isometry2d & transform, const Point2d & point)
{
  const Eigen::Vector2d transformed = transform * Eigen::Vector2d{point.x(), point.y()};
  return Point2d{transformed.x(), transformed.y()};
}

std::vector<Point2d> create_base_polygon(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const double half_x = shape.dimensions.x / 2.0;
    const double half_y = shape.dimensions.y / 2.0;

    return {
      Point2d{half_x, half_y}, Point2d{half_x, -half_y}, Point2d{-half_x, -half_y},
      Point2d{-half_x, half_y}};
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    const double radius = shape.dimensions.x / 2.0;

    return {
      Point2d{radius, radius}, Point2d{radius, -radius}, Point2d{-radius, -radius},
      Point2d{-radius, radius}};
  } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    std::vector<Point2d> polygon;
    polygon.reserve(shape.footprint.points.size());
    for (const auto & point : shape.footprint.points) {
      polygon.push_back(Point2d{point.x, point.y});
    }

    return polygon;
  }

  throw std::logic_error("The shape type is not supported.");
}

std::vector<Point2d> create_base_polygon(const trajectory::footprint::EgoDimensions & ego)
{
  const double half_width = ego.vehicle_width / 2.0;
  return {
    Point2d{ego.front_offset, half_width}, Point2d{ego.front_offset, -half_width},
    Point2d{-ego.rear_overhang, -half_width}, Point2d{-ego.rear_overhang, half_width}};
}

Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape)
{
  const auto iso = pose_to_isometry(pose);
  const auto points = create_base_polygon(shape);

  Polygon2d polygon;
  polygon.outer().reserve(points.size() + 1U);
  for (const auto & point : points) {
    polygon.outer().push_back(transform_point(iso, point));
  }
  if (!polygon.outer().empty()) {
    polygon.outer().push_back(polygon.outer().front());
  }
  boost::geometry::correct(polygon);
  return polygon;
}

Polygon2d to_polygon2d(const std::array<Point2d, 4> & quad)
{
  Polygon2d polygon;
  polygon.outer().assign(quad.begin(), quad.end());
  polygon.outer().push_back(quad.front());
  boost::geometry::correct(polygon);
  return polygon;
}

Polygon2d to_polygon2d(const std::vector<Point2d> & ring)
{
  Polygon2d polygon;
  polygon.outer().assign(ring.begin(), ring.end());
  if (!ring.empty()) {
    polygon.outer().push_back(ring.front());
  }
  boost::geometry::correct(polygon);
  return polygon;
}
}  // namespace geometry

namespace trajectory
{
namespace footprint
{
FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const std::vector<Point2d> & base_poly)
{
  if (base_poly.size() == 4) {
    QuadTrajectory trajectory;
    trajectory.reserve(pose_trajectory.size());

    for (const auto & pose : pose_trajectory) {
      const auto iso = geometry::pose_to_isometry(pose);

      trajectory.push_back(
        std::array<Point2d, 4>{
          geometry::transform_point(iso, base_poly[0]),
          geometry::transform_point(iso, base_poly[1]),
          geometry::transform_point(iso, base_poly[2]),
          geometry::transform_point(iso, base_poly[3])});
    }
    return trajectory;
  } else {
    NgonTrajectory trajectory;
    trajectory.reserve(pose_trajectory.size());

    for (const auto & pose : pose_trajectory) {
      const auto iso = geometry::pose_to_isometry(pose);

      std::vector<Point2d> transformed_poly;
      transformed_poly.reserve(base_poly.size());

      for (const auto & pt : base_poly) {
        transformed_poly.push_back(geometry::transform_point(iso, pt));
      }

      trajectory.push_back(std::move(transformed_poly));
    }
    return trajectory;
  }
}

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const autoware_perception_msgs::msg::Shape & object_shape)
{
  return compute_footprint_trajectory(pose_trajectory, geometry::create_base_polygon(object_shape));
}

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const EgoDimensions & ego_dimensions)
{
  return compute_footprint_trajectory(
    pose_trajectory, geometry::create_base_polygon(ego_dimensions));
}
}  // namespace footprint

std::tuple<size_t, size_t, double> resolve_interpolation(
  const std::vector<double> & values, const double target_value)
{
  if (values.size() < 2U) {
    throw std::invalid_argument("values must contain at least two elements");
  }

  if (!std::isfinite(target_value)) {
    throw std::invalid_argument("target_value must be finite");
  }

  size_t lower_idx = 0;
  size_t upper_idx = 1;
  double ratio = 0.0;

  if (target_value <= values.front()) {
    lower_idx = 0;
    upper_idx = 1;
  } else if (target_value >= values.back()) {
    lower_idx = values.size() - 2;
    upper_idx = values.size() - 1;
  } else {
    const auto upper_it = std::lower_bound(values.begin(), values.end(), target_value);
    upper_idx = static_cast<size_t>(std::distance(values.begin(), upper_it));
    lower_idx = upper_idx - 1;
  }

  const double denom = values.at(upper_idx) - values.at(lower_idx);
  ratio = denom > 1e-6 ? (target_value - values.at(lower_idx)) / denom : 0.0;

  return {lower_idx, upper_idx, ratio};
}

TimeTrajectory serialize_times(const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  std::vector<double> times;
  times.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    times.push_back(rclcpp::Duration(point.time_from_start).seconds());
  }
  return times;
}

TravelDistanceTrajectory serialize_distances(
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  const auto & points = trajectory.points;
  std::vector<double> distances;
  distances.reserve(points.size());
  double cumulative_distance = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      cumulative_distance += autoware_utils_geometry::calc_distance2d(
        points.at(i - 1).pose.position, points.at(i).pose.position);
    }
    distances.push_back(cumulative_distance);
  }

  return distances;
}

TrajectoryInterpolator::TrajectoryInterpolator(
  const autoware_planning_msgs::msg::Trajectory & trajectory)
: trajectory_points_(trajectory.points),
  reference_time_(rclcpp::Time(trajectory.header.stamp)),
  time_from_refs_(serialize_times(trajectory)),
  dist_from_fronts_(serialize_distances(trajectory))
{
  if (trajectory_points_.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  const auto is_finite = [](const double value) { return std::isfinite(value); };
  if (!std::all_of(time_from_refs_.begin(), time_from_refs_.end(), is_finite)) {
    throw std::invalid_argument("time_from_refs must contain only finite values");
  }
  if (!std::all_of(dist_from_fronts_.begin(), dist_from_fronts_.end(), is_finite)) {
    throw std::invalid_argument("dist_from_fronts must contain only finite values");
  }

  if (!std::is_sorted(time_from_refs_.begin(), time_from_refs_.end())) {
    throw std::invalid_argument("time_from_refs must be in non-decreasing order");
  }
  if (!std::is_sorted(dist_from_fronts_.begin(), dist_from_fronts_.end())) {
    throw std::invalid_argument("dist_from_fronts must be in non-decreasing order");
  }
}

InterpolatedState TrajectoryInterpolator::interpolate_state_from_time(
  const rclcpp::Time & target_time) const
{
  if (trajectory_points_.size() == 1) {
    const auto & point = trajectory_points_.front();
    return InterpolatedState{0.0, point.pose, point.longitudinal_velocity_mps};
  }

  const double target_time_from_ref = (target_time - reference_time_).seconds();
  const auto [lower_idx, upper_idx, ratio] =
    resolve_interpolation(time_from_refs_, target_time_from_ref);

  const auto lerp = [](const double a, const double b, const double r) { return a + r * (b - a); };
  const auto pose = pose::interpolate_pose(
    trajectory_points_.at(lower_idx).pose, trajectory_points_.at(upper_idx).pose, ratio);

  return InterpolatedState{
    lerp(dist_from_fronts_.at(lower_idx), dist_from_fronts_.at(upper_idx), ratio), pose,
    lerp(
      static_cast<double>(trajectory_points_.at(lower_idx).longitudinal_velocity_mps),
      static_cast<double>(trajectory_points_.at(upper_idx).longitudinal_velocity_mps), ratio)};
}

TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const autoware_perception_msgs::msg::PredictedPath & predicted_path, const double max_time,
  const double time_offset, const double time_resolution,
  const std::optional<double> rear_axle_length)
{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, 0.0,
    std::min(
      max_time, predicted_path.path.size() * rclcpp::Duration(predicted_path.time_step).seconds()),
    time_resolution);

  for (auto & t : times) {
    t += time_offset;
  }

  auto poses = pose::compute_pose_trajectory(predicted_path.path, distances);
  if (rear_axle_length) {
    pose::correct_vehicle_yaw(
      poses,
      autoware_utils_geometry::get_rpy(
        predicted_object.kinematics.initial_pose_with_covariance.pose)
        .z,
      *rear_axle_length);
  }
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(std::move(times), std::move(poses), std::move(footprints));
}

TrajectoryData generate_static_trajectory(const Polygon2d & polygon, const TimeTrajectory & times)
{
  Point2d centroid;
  boost::geometry::centroid(polygon, centroid);
  geometry_msgs::msg::Pose pose;
  pose.position.x = centroid.x();
  pose.position.y = centroid.y();
  pose.orientation.w = 1.0;

  std::vector<Point2d> ring(polygon.outer().begin(), polygon.outer().end());
  if (ring.size() > 1 && boost::geometry::equals(ring.front(), ring.back())) {
    ring.pop_back();
  }

  PoseTrajectory poses(times.size(), pose);
  NgonTrajectory footprints(times.size(), ring);
  return TrajectoryData(times, std::move(poses), std::move(footprints));
}
}  // namespace trajectory

namespace collision_timing_assessment
{
std::optional<CollisionTiming> find_collision_timing(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  const std::vector<bool> & ref_stopped)
{
  const auto overall_test_index_range = test_trajectory.resolve_covering_index_range(
    {ref_trajectory.getTimes().front(), ref_trajectory.getTimes().back()});
  if (
    !overall_test_index_range ||
    !boost::geometry::intersects(
      ref_trajectory.get_or_compute_overall_envelope(),
      test_trajectory.get_or_compute_envelope(*overall_test_index_range))) {
    return std::nullopt;
  }

  const auto has_intersects =
    [](
      const TrajectoryData & first_trajectory, const IndexRange & first_index_range,
      const TrajectoryData & second_trajectory, const IndexRange & second_index_range) {
      return (
        boost::geometry::intersects(
          first_trajectory.get_or_compute_envelope(first_index_range),
          second_trajectory.get_or_compute_envelope(second_index_range)) &&
        geometry::intersects_sat(
          first_trajectory.get_or_compute_convex(first_index_range),
          second_trajectory.get_or_compute_convex(second_index_range)));
    };

  for (size_t k = 1; k < ref_trajectory.size(); ++k) {
    if (k < ref_stopped.size() && ref_stopped[k]) {
      continue;
    }
    const TimeRange window{ref_trajectory.getTimes().at(k - 1), ref_trajectory.getTimes().at(k)};
    const IndexRange ref_index_range{k - 1, k};
    const auto test_index_range = test_trajectory.resolve_covering_index_range(window);
    if (!test_index_range) {
      continue;
    }
    if (has_intersects(ref_trajectory, ref_index_range, test_trajectory, *test_index_range)) {
      return CollisionTiming{window.second};
    }
  }

  return std::nullopt;
}
}  // namespace collision_timing_assessment

CollisionSweep::CollisionSweep(
  const vehicle_info_utils::VehicleInfo & vehicle_info, const double time_resolution)
: ego_dimensions_{
    vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
    vehicle_info.vehicle_width_m},
  time_resolution_(time_resolution)
{
  set_time_resolution(time_resolution);
}

void CollisionSweep::set_time_resolution(const double time_resolution)
{
  if (!(time_resolution > 0.0)) {
    throw std::invalid_argument("time_resolution must be positive");
  }
  time_resolution_ = time_resolution;
}

Polygon2d CollisionSweep::ego_polygon(const geometry_msgs::msg::Pose & pose) const
{
  const auto iso = geometry::pose_to_isometry(pose);
  const auto base = geometry::create_base_polygon(ego_dimensions_);
  std::vector<Point2d> ring;
  ring.reserve(base.size());
  for (const auto & pt : base) {
    ring.push_back(geometry::transform_point(iso, pt));
  }
  return geometry::to_polygon2d(ring);
}

double CollisionSweep::distance_now(const Polygon2d & ego_now, const Polygon2d & candidate)
{
  return boost::geometry::distance(ego_now, candidate);
}

double compute_cos_collision(
  const geometry_msgs::msg::Pose & ego_pose_at_hit, const bool forward, const Point2d & hit_point)
{
  const double yaw = autoware_utils_geometry::get_rpy(ego_pose_at_hit).z;
  const double sign = forward ? 1.0 : -1.0;
  const Eigen::Vector2d heading(sign * std::cos(yaw), sign * std::sin(yaw));
  const Eigen::Vector2d to_hit(
    hit_point.x() - ego_pose_at_hit.position.x, hit_point.y() - ego_pose_at_hit.position.y);
  const double distance = to_hit.norm();
  if (distance < 1e-3) {
    return 1.0;
  }
  return std::clamp(heading.dot(to_hit) / distance, -1.0, 1.0);
}

namespace
{
double sweep_horizon(
  const autoware_planning_msgs::msg::Trajectory & predicted, const SweepLimits & limits)
{
  return std::min(
    limits.max_time, rclcpp::Duration(predicted.points.back().time_from_start).seconds());
}

std::optional<double> vehicle_rear_axle_length(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_perception_msgs::msg::PredictedPath & path, const double rear_axle_ratio)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_perception_msgs::msg::Shape;
  uint8_t label = ObjectClassification::UNKNOWN;
  float best_probability = -1.0F;
  for (const auto & classification : object.classification) {
    if (classification.probability > best_probability) {
      best_probability = classification.probability;
      label = classification.label;
    }
  }
  if (
    label != ObjectClassification::CAR && label != ObjectClassification::TRUCK &&
    label != ObjectClassification::BUS) {
    return std::nullopt;
  }
  if (object.shape.type != Shape::BOUNDING_BOX) {
    return std::nullopt;
  }
  if (path.path.empty()) {
    return std::nullopt;
  }
  const auto & start = path.path.front().position;
  const auto first_away = std::find_if(path.path.begin(), path.path.end(), [&](const auto & pose) {
    return std::hypot(pose.position.x - start.x, pose.position.y - start.y) >= 0.05;
  });
  if (first_away == path.path.end()) {
    return std::nullopt;
  }
  const double path_yaw =
    std::atan2(first_away->position.y - start.y, first_away->position.x - start.x);
  const double initial_yaw =
    autoware_utils_geometry::get_rpy(object.kinematics.initial_pose_with_covariance.pose).z;
  const double diff =
    std::atan2(std::sin(path_yaw - initial_yaw), std::cos(path_yaw - initial_yaw));
  if (std::abs(diff) > M_PI / 2.0) {
    return std::nullopt;
  }
  return rear_axle_ratio * object.shape.dimensions.x;
}
}  // namespace

EgoSweep CollisionSweep::build_ego_sweep(
  const autoware_planning_msgs::msg::Trajectory & predicted, const nav_msgs::msg::Odometry & odom,
  const SweepLimits & limits) const
{
  // Ego: resample the predicted trajectory at t_k = k * dt up to min(max_time, last point).
  const trajectory::TrajectoryInterpolator interpolator(predicted);
  const double horizon = std::min(limits.max_time, interpolator.time_from_refs_.back());
  const auto num_steps =
    static_cast<size_t>(std::floor(horizon / time_resolution_ + TIME_INDEX_EPSILON));

  TimeTrajectory ego_times;
  PoseTrajectory ego_poses;
  std::vector<bool> ego_stopped;
  ego_times.reserve(num_steps + 1);
  ego_poses.reserve(num_steps + 1);
  ego_stopped.reserve(num_steps + 1);
  for (size_t k = 0; k <= num_steps; ++k) {
    const double t = static_cast<double>(k) * time_resolution_;
    const auto state = interpolator.interpolate_state_from_time(
      interpolator.reference_time_ + rclcpp::Duration::from_seconds(t));
    ego_times.push_back(t);
    ego_poses.push_back(state.pose);
    ego_stopped.push_back(std::abs(state.longitudinal_velocity) < limits.stop_velocity);
  }
  const auto ego_footprints =
    trajectory::footprint::compute_footprint_trajectory(ego_poses, ego_dimensions_);
  return EgoSweep(
    std::move(ego_times), std::move(ego_poses), ego_footprints, std::move(ego_stopped),
    odom.twist.twist.linear.x >= 0.0);
}

std::shared_ptr<const ObjectMotion> CollisionSweep::make_object_motion(
  const autoware_perception_msgs::msg::PredictedObject & object, const double time_offset,
  const double horizon, const TimeTrajectory & static_times, const double rear_axle_ratio) const
{
  const auto & paths = object.kinematics.predicted_paths;
  const auto best_path = std::max_element(
    paths.begin(), paths.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.confidence < rhs.confidence; });
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  const bool is_moving = std::hypot(twist.linear.x, twist.linear.y) > 1e-3;
  const auto & initial_pose = object.kinematics.initial_pose_with_covariance.pose;

  if (best_path != paths.end() && !best_path->path.empty() && is_moving) {
    return std::make_shared<const ObjectMotion>(
      trajectory::generate_predicted_path_trajectory(
        object, *best_path, horizon + std::abs(time_offset), time_offset, time_resolution_,
        vehicle_rear_axle_length(object, *best_path, rear_axle_ratio)),
      false, initial_pose);
  }
  return std::make_shared<const ObjectMotion>(
    trajectory::generate_static_trajectory(
      geometry::to_polygon2d(initial_pose, object.shape), static_times),
    true, initial_pose);
}

std::vector<std::shared_ptr<const ObjectMotion>> CollisionSweep::build_object_motions(
  const autoware_planning_msgs::msg::Trajectory & predicted,
  const autoware_perception_msgs::msg::PredictedObjects & objects, const SweepLimits & limits) const
{
  std::vector<std::shared_ptr<const ObjectMotion>> motions;
  if (predicted.points.size() < 2) {
    return motions;
  }
  const double horizon = sweep_horizon(predicted, limits);
  const auto num_steps =
    static_cast<size_t>(std::floor(horizon / time_resolution_ + TIME_INDEX_EPSILON));
  TimeTrajectory static_times;
  static_times.reserve(num_steps + 1);
  for (size_t k = 0; k <= num_steps; ++k) {
    static_times.push_back(static_cast<double>(k) * time_resolution_);
  }
  const double time_offset =
    (rclcpp::Time(objects.header.stamp) - rclcpp::Time(predicted.header.stamp)).seconds();
  motions.reserve(objects.objects.size());
  for (const auto & object : objects.objects) {
    motions.push_back(
      make_object_motion(object, time_offset, horizon, static_times, limits.rear_axle_ratio));
  }
  return motions;
}

std::vector<SweepCandidate> CollisionSweep::evaluate(
  const autoware_planning_msgs::msg::Trajectory & predicted, const nav_msgs::msg::Odometry & odom,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const std::vector<PointCluster> & point_clusters,
  const builtin_interfaces::msg::Time & pointcloud_stamp, const SweepLimits & limits,
  const ClusterAttachments & attachments) const
{
  std::vector<SweepCandidate> candidates;
  if (predicted.points.size() < 2) {
    return candidates;
  }

  const EgoSweep ego = build_ego_sweep(predicted, odom, limits);
  const double horizon = sweep_horizon(predicted, limits);
  const Polygon2d & ego_now = ego.now();
  std::vector<std::unique_ptr<CollisionTarget>> targets;

  // Objects: closest first, enabled labels only, not touching the current footprint, at most
  // n_eval_object.
  const double objects_time_offset =
    (rclcpp::Time(objects.header.stamp) - rclcpp::Time(predicted.header.stamp)).seconds();
  struct ObjectEntry
  {
    const autoware_perception_msgs::msg::PredictedObject * object;
    SweepCandidate candidate;
  };
  std::vector<ObjectEntry> object_entries;
  if (std::abs(objects_time_offset) <= limits.max_objects_stamp_gap_s) {
    for (const auto & object : objects.objects) {
      uint8_t label = 0;
      float best_probability = -1.0F;
      for (const auto & classification : object.classification) {
        if (classification.probability > best_probability) {
          best_probability = classification.probability;
          label = classification.label;
        }
      }
      const bool enabled =
        std::find(limits.enabled_labels.begin(), limits.enabled_labels.end(), label) !=
        limits.enabled_labels.end();
      if (!enabled) {
        continue;
      }
      SweepCandidate candidate;
      candidate.kind = SweepCandidate::Kind::OBJECT;
      candidate.label = label;
      candidate.source_stamp = objects.header.stamp;
      candidate.polygon_now =
        geometry::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape);
      if (boost::geometry::intersects(ego_now, candidate.polygon_now)) {
        continue;
      }
      candidate.distance_now = distance_now(ego_now, candidate.polygon_now);
      object_entries.push_back(ObjectEntry{&object, std::move(candidate)});
    }
  }
  std::stable_sort(
    object_entries.begin(), object_entries.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.candidate.distance_now < rhs.candidate.distance_now;
    });
  if (object_entries.size() > limits.n_eval_object) {
    object_entries.resize(limits.n_eval_object);
  }

  std::vector<std::array<uint8_t, 16>> selected_ids;
  selected_ids.reserve(object_entries.size());
  for (auto & entry : object_entries) {
    targets.push_back(
      std::make_unique<ObjectTarget>(
        std::move(entry.candidate),
        make_object_motion(
          *entry.object, objects_time_offset, horizon, ego.times, limits.rear_axle_ratio),
        time_resolution_));
    selected_ids.push_back(entry.object->object_id.uuid);
  }

  // Pointcloud: clusters attached to a selected object move with it; the closest
  // n_eval_pointcloud of the other clusters (by centroid) are static. A point collides at the
  // first t_k whose ego footprint contains it.
  if (limits.enable_pointcloud) {
    const double t_cloud =
      (rclcpp::Time(pointcloud_stamp) - rclcpp::Time(predicted.header.stamp)).seconds();
    std::vector<std::pair<double, const PointCluster *>> ordered;
    ordered.reserve(point_clusters.size());
    for (size_t i = 0; i < point_clusters.size(); ++i) {
      const auto & cluster = point_clusters[i];
      if (cluster.points.empty()) {
        continue;
      }
      const auto attached_to = attachments.objects_at_cloud && i < attachments.object_index.size()
                                 ? attachments.object_index[i]
                                 : std::nullopt;
      if (!attached_to) {
        ordered.emplace_back(boost::geometry::distance(ego_now, cluster.centroid), &cluster);
        continue;
      }
      if (
        *attached_to >= attachments.objects_at_cloud->objects.size() ||
        *attached_to >= attachments.motions.size() || !attachments.motions[*attached_to]) {
        continue;
      }
      const auto & id = attachments.objects_at_cloud->objects[*attached_to].object_id.uuid;
      if (std::find(selected_ids.begin(), selected_ids.end(), id) == selected_ids.end()) {
        continue;
      }
      const auto & motion = attachments.motions[*attached_to];
      if (motion->is_static()) {
        targets.push_back(
          std::make_unique<PointClusterTarget>(cluster.points, nullptr, pointcloud_stamp));
        continue;
      }
      const auto map_to_object = geometry::pose_to_isometry(motion->pose_at(t_cloud)).inverse();
      std::vector<Point2d> local_points;
      local_points.reserve(cluster.points.size());
      for (const auto & point : cluster.points) {
        local_points.push_back(geometry::transform_point(map_to_object, point));
      }
      targets.push_back(
        std::make_unique<PointClusterTarget>(std::move(local_points), motion, pointcloud_stamp));
    }
    std::stable_sort(ordered.begin(), ordered.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.first < rhs.first;
    });
    if (ordered.size() > limits.n_eval_pointcloud) {
      ordered.resize(limits.n_eval_pointcloud);
    }
    for (const auto & [centroid_distance, cluster] : ordered) {
      targets.push_back(
        std::make_unique<PointClusterTarget>(cluster->points, nullptr, pointcloud_stamp));
    }
  }

  for (const auto & target : targets) {
    target->evaluate(ego, candidates);
  }
  return candidates;
}
}  // namespace autoware::control_validator::hazard
