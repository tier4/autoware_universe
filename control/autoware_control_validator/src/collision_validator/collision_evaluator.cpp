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

#include "autoware/control_validator/collision_validator/collision_evaluator.hpp"

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

namespace autoware::control_validator::collision_validator
{
namespace detail
{
namespace
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
}  // namespace

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

namespace
{
std::vector<Point2d> create_base_polygon(const EgoDimensions & ego)
{
  const double half_width = ego.vehicle_width / 2.0;
  return {
    Point2d{ego.front_offset, half_width}, Point2d{ego.front_offset, -half_width},
    Point2d{-ego.rear_overhang, -half_width}, Point2d{-ego.rear_overhang, half_width}};
}
}  // namespace

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

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const std::vector<Point2d> & base_poly)
{
  if (base_poly.size() == 4) {
    QuadTrajectory trajectory;
    trajectory.reserve(pose_trajectory.size());

    for (const auto & pose : pose_trajectory) {
      const auto iso = pose_to_isometry(pose);

      trajectory.push_back(
        std::array<Point2d, 4>{
          transform_point(iso, base_poly[0]), transform_point(iso, base_poly[1]),
          transform_point(iso, base_poly[2]), transform_point(iso, base_poly[3])});
    }
    return trajectory;
  } else {
    NgonTrajectory trajectory;
    trajectory.reserve(pose_trajectory.size());

    for (const auto & pose : pose_trajectory) {
      const auto iso = pose_to_isometry(pose);

      std::vector<Point2d> transformed_poly;
      transformed_poly.reserve(base_poly.size());

      for (const auto & pt : base_poly) {
        transformed_poly.push_back(transform_point(iso, pt));
      }

      trajectory.push_back(std::move(transformed_poly));
    }
    return trajectory;
  }
}

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const autoware_perception_msgs::msg::Shape & object_shape)
{
  return compute_footprint_trajectory(pose_trajectory, create_base_polygon(object_shape));
}

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const EgoDimensions & ego_dimensions)
{
  return compute_footprint_trajectory(pose_trajectory, create_base_polygon(ego_dimensions));
}

namespace
{
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
}  // namespace

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
  const auto pose = interpolate_pose(
    trajectory_points_.at(lower_idx).pose, trajectory_points_.at(upper_idx).pose, ratio);

  return InterpolatedState{
    lerp(dist_from_fronts_.at(lower_idx), dist_from_fronts_.at(upper_idx), ratio), pose,
    lerp(
      static_cast<double>(trajectory_points_.at(lower_idx).longitudinal_velocity_mps),
      static_cast<double>(trajectory_points_.at(upper_idx).longitudinal_velocity_mps), ratio)};
}

InterpolatedState TrajectoryInterpolator::interpolate_state_from_dist(
  const double target_dist) const
{
  if (trajectory_points_.size() == 1) {
    const auto & point = trajectory_points_.front();
    return InterpolatedState{0.0, point.pose, point.longitudinal_velocity_mps};
  }

  const auto [lower_idx, upper_idx, ratio] = resolve_interpolation(dist_from_fronts_, target_dist);

  const auto lerp = [](const double a, const double b, const double r) { return a + r * (b - a); };
  const auto pose = interpolate_pose(
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
  auto [times, distances] = compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, 0.0,
    std::min(
      max_time, predicted_path.path.size() * rclcpp::Duration(predicted_path.time_step).seconds()),
    time_resolution);

  for (auto & t : times) {
    t += time_offset;
  }

  auto poses = compute_pose_trajectory(predicted_path.path, distances);
  if (rear_axle_length) {
    correct_vehicle_yaw(
      poses,
      autoware_utils_geometry::get_rpy(
        predicted_object.kinematics.initial_pose_with_covariance.pose)
        .z,
      *rear_axle_length);
  }
  auto footprints = compute_footprint_trajectory(poses, predicted_object.shape);

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

std::optional<CollisionStep> find_first_collision(
  const TrajectoryData & ego, const TrajectoryData & target, const std::vector<bool> & ego_stopped)
{
  const auto overall_target_index_range =
    target.resolve_covering_index_range({ego.getTimes().front(), ego.getTimes().back()});
  if (
    !overall_target_index_range || !boost::geometry::intersects(
                                     ego.get_or_compute_overall_envelope(),
                                     target.get_or_compute_envelope(*overall_target_index_range))) {
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
        intersects_sat(
          first_trajectory.get_or_compute_convex(first_index_range),
          second_trajectory.get_or_compute_convex(second_index_range)));
    };

  for (size_t k = 1; k < ego.size(); ++k) {
    if (k < ego_stopped.size() && ego_stopped[k]) {
      continue;
    }
    const TimeRange window{ego.getTimes().at(k - 1), ego.getTimes().at(k)};
    const IndexRange ego_index_range{k - 1, k};
    const auto target_index_range = target.resolve_covering_index_range(window);
    if (!target_index_range) {
      continue;
    }
    if (has_intersects(ego, ego_index_range, target, *target_index_range)) {
      return CollisionStep{k, *target_index_range};
    }
  }

  return std::nullopt;
}

std::optional<size_t> find_first_collision(
  const TrajectoryData & ego, const Point2d & point, const std::vector<bool> & ego_stopped)
{
  if (!boost::geometry::within(point, ego.get_or_compute_overall_envelope())) {
    return std::nullopt;
  }
  for (size_t k = 1; k < ego.size(); ++k) {
    if (k < ego_stopped.size() && ego_stopped[k]) {
      continue;
    }
    const IndexRange window{k - 1, k};
    if (
      boost::geometry::within(point, ego.get_or_compute_envelope(window)) &&
      boost::geometry::within(point, ego.get_or_compute_convex(window))) {
      return k;
    }
  }
  return std::nullopt;
}
}  // namespace detail

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
using detail::find_first_collision;
using detail::FootprintTrajectory;
using detail::PoseTrajectory;
using detail::TIME_INDEX_EPSILON;
using detail::TimeTrajectory;
using detail::TrajectoryData;

double sweep_horizon(
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
  const CollisionEvaluatorParam & param)
{
  return std::min(
    param.max_time, rclcpp::Duration(predicted_trajectory.points.back().time_from_start).seconds());
}

uint8_t most_probable_label(const autoware_perception_msgs::msg::PredictedObject & object)
{
  uint8_t label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  float best_probability = -1.0F;
  for (const auto & classification : object.classification) {
    if (classification.probability > best_probability) {
      best_probability = classification.probability;
      label = classification.label;
    }
  }
  return label;
}

std::optional<double> vehicle_rear_axle_length(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_perception_msgs::msg::PredictedPath & path, const double rear_axle_ratio)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_perception_msgs::msg::Shape;
  const auto label = most_probable_label(object);
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

// Midpoint of the closest pair of points between the boundaries of two polygons, used when the
// footprints touch only along an edge and the intersection area is empty.
Point2d closest_boundary_midpoint(const Polygon2d & a, const Polygon2d & b)
{
  double best = std::numeric_limits<double>::infinity();
  Point2d best_a;
  Point2d best_b;
  const auto update = [&](const Point2d & pa, const Point2d & pb) {
    const double d = boost::geometry::distance(pa, pb);
    if (d < best) {
      best = d;
      best_a = pa;
      best_b = pb;
    }
  };
  const auto project = [](const Point2d & p, const Point2d & s0, const Point2d & s1) {
    const Eigen::Vector2d d = s1 - s0;
    const double len2 = d.squaredNorm();
    if (len2 < 1e-12) return s0;
    const double t = std::clamp(static_cast<double>((p - s0).dot(d)) / len2, 0.0, 1.0);
    return Point2d{s0.x() + t * d.x(), s0.y() + t * d.y()};
  };
  const auto & ra = a.outer();
  const auto & rb = b.outer();
  for (size_t i = 0; i + 1 < rb.size(); ++i) {
    for (const auto & pa : ra) {
      update(pa, project(pa, rb[i], rb[i + 1]));
    }
  }
  for (size_t i = 0; i + 1 < ra.size(); ++i) {
    for (const auto & pb : rb) {
      update(project(pb, ra[i], ra[i + 1]), pb);
    }
  }
  return Point2d{0.5 * (best_a.x() + best_b.x()), 0.5 * (best_a.y() + best_b.y())};
}

// Collision point of an object: centroid of ego_footprint(t_k) ∩ object_footprint(t_k).
Point2d object_hit_point(const Polygon2d & ego_footprint, const Polygon2d & object_footprint)
{
  boost::geometry::model::multi_polygon<Polygon2d> overlap;
  boost::geometry::intersection(ego_footprint, object_footprint, overlap);
  if (!overlap.empty() && boost::geometry::area(overlap) > 1e-9) {
    Point2d centroid;
    boost::geometry::centroid(overlap, centroid);
    return centroid;
  }
  return closest_boundary_midpoint(ego_footprint, object_footprint);
}

Polygon2d point_polygon(const Point2d & point)
{
  Polygon2d polygon;
  polygon.outer().push_back(point);
  polygon.outer().push_back(point);
  return polygon;
}

CollisionCandidate make_point_candidate(
  const EgoSweep & ego, const builtin_interfaces::msg::Time & stamp, const Point2d & point_now,
  const size_t k, const Point2d & hit_point)
{
  CollisionCandidate candidate;
  candidate.kind = CollisionCandidate::Kind::POINTCLOUD;
  candidate.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  candidate.source_stamp = stamp;
  candidate.polygon_now = point_polygon(point_now);
  candidate.distance_now = boost::geometry::distance(ego.now(), point_now);
  candidate.ttc = ego.times[k];
  candidate.cos_collision = compute_cos_collision(ego.poses[k], ego.forward, hit_point);
  return candidate;
}

template <typename CollidesWhenBraking>
uint8_t brake_level_of(const CollidesWhenBraking & collides_when_braking)
{
  uint8_t level = 0;
  for (size_t i = 1; i <= 3; ++i) {
    if (!collides_when_braking(i)) {
      break;
    }
    level = static_cast<uint8_t>(i);
  }
  return level;
}

bool passes_cos_threshold(const std::optional<double> & cos_collision, const double cos_threshold)
{
  return cos_collision.has_value() && *cos_collision >= cos_threshold;
}

struct EgoInObjectFrame
{
  TrajectoryData trajectory;
  std::vector<Eigen::Isometry2d> object_to_map;
};

EgoInObjectFrame ego_trajectory_in_object_frame(const EgoSweep & ego, const ObjectMotion & motion)
{
  const size_t num_steps = ego.times.size();
  std::vector<Eigen::Isometry2d> object_to_map(num_steps);
  std::vector<Eigen::Isometry2d> map_to_object(num_steps);
  PoseTrajectory poses_local;
  poses_local.reserve(num_steps);
  for (size_t k = 0; k < num_steps; ++k) {
    object_to_map[k] = detail::pose_to_isometry(motion.pose_at(ego.times[k]));
    map_to_object[k] = object_to_map[k].inverse();
    const auto local = map_to_object[k] * detail::pose_to_isometry(ego.poses[k]);
    geometry_msgs::msg::Pose pose;
    pose.position.x = local.translation().x();
    pose.position.y = local.translation().y();
    pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
      std::atan2(local.linear()(1, 0), local.linear()(0, 0)));
    poses_local.push_back(pose);
  }
  auto footprints_local = std::visit(
    [&](const auto & footprints) -> FootprintTrajectory {
      auto transformed = footprints;
      for (size_t k = 0; k < transformed.size(); ++k) {
        for (auto & vertex : transformed[k]) {
          vertex = detail::transform_point(map_to_object[k], vertex);
        }
      }
      return transformed;
    },
    ego.trajectory.getFootprints());
  return EgoInObjectFrame{
    TrajectoryData(ego.times, std::move(poses_local), std::move(footprints_local)),
    std::move(object_to_map)};
}
}  // namespace

class SweepsWithBraking
{
public:
  SweepsWithBraking(
    const CollisionEvaluator & evaluator,
    const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
    const nav_msgs::msg::Odometry & odom, const vehicle_info_utils::VehicleInfo & vehicle_info)
  : evaluator_(evaluator),
    predicted_trajectory_(predicted_trajectory),
    odom_(odom),
    vehicle_info_(vehicle_info)
  {
  }

  const EgoSweep & at(const size_t level) const
  {
    auto & sweep = sweeps_.at(level - 1);
    if (!sweep) {
      sweep.emplace(evaluator_.build_ego_sweep(
        predicted_trajectory_, odom_, vehicle_info_,
        BrakingParam{
          evaluator_.param_.reaction_times.at(level - 1), evaluator_.param_.braking_deceleration}));
    }
    return *sweep;
  }

private:
  const CollisionEvaluator & evaluator_;
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory_;
  const nav_msgs::msg::Odometry & odom_;
  const vehicle_info_utils::VehicleInfo & vehicle_info_;
  mutable std::array<std::optional<EgoSweep>, 3> sweeps_;
};

CollisionEvaluator::CollisionEvaluator(const CollisionEvaluatorParam & param) : param_(param)
{
}

void CollisionEvaluator::update_parameters(const CollisionEvaluatorParam & param)
{
  param_ = param;
}

double CollisionEvaluator::distance_now(const Polygon2d & ego_now, const Polygon2d & candidate)
{
  return boost::geometry::distance(ego_now, candidate);
}

EgoSweep CollisionEvaluator::build_ego_sweep(
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
  const nav_msgs::msg::Odometry & odom, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::optional<BrakingParam> & braking) const
{
  // Ego: resample the predicted trajectory at t_k = k * dt up to min(max_time, last point).
  const detail::TrajectoryInterpolator interpolator(predicted_trajectory);
  const double horizon = std::min(param_.max_time, interpolator.time_from_refs_.back());
  const auto num_steps =
    static_cast<size_t>(std::floor(horizon / param_.time_resolution + TIME_INDEX_EPSILON));
  const auto state_at = [&](const double t) {
    return interpolator.interpolate_state_from_time(
      interpolator.reference_time_ + rclcpp::Duration::from_seconds(t));
  };

  std::optional<detail::InterpolatedState> braking_start;
  double braking_velocity = 0.0;
  if (braking) {
    braking_start = state_at(braking->reaction_time);
    braking_velocity = std::abs(braking_start->longitudinal_velocity);
  }

  TimeTrajectory ego_times;
  PoseTrajectory ego_poses;
  std::vector<bool> ego_stopped;
  ego_times.reserve(num_steps + 1);
  ego_poses.reserve(num_steps + 1);
  ego_stopped.reserve(num_steps + 1);
  for (size_t k = 0; k <= num_steps; ++k) {
    const double t = static_cast<double>(k) * param_.time_resolution;
    ego_times.push_back(t);
    if (braking && t >= braking->reaction_time) {
      const double elapsed =
        std::min(t - braking->reaction_time, braking_velocity / braking->deceleration);
      const double distance = braking_start->distance + braking_velocity * elapsed -
                              0.5 * braking->deceleration * elapsed * elapsed;
      ego_poses.push_back(interpolator.interpolate_state_from_dist(distance).pose);
      ego_stopped.push_back(false);
      continue;
    }
    const auto state = state_at(t);
    ego_poses.push_back(state.pose);
    ego_stopped.push_back(!braking && std::abs(state.longitudinal_velocity) < param_.stop_velocity);
  }
  const detail::EgoDimensions ego_dimensions{
    vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
    vehicle_info.vehicle_width_m};
  const auto ego_footprints = detail::compute_footprint_trajectory(ego_poses, ego_dimensions);
  return EgoSweep(
    std::move(ego_times), std::move(ego_poses), ego_footprints, std::move(ego_stopped),
    odom.twist.twist.linear.x >= 0.0);
}

std::shared_ptr<const ObjectMotion> CollisionEvaluator::make_object_motion(
  const autoware_perception_msgs::msg::PredictedObject & object, const double time_offset,
  const double horizon, const TimeTrajectory & static_times) const
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
      detail::generate_predicted_path_trajectory(
        object, *best_path, horizon + std::abs(time_offset), time_offset, param_.time_resolution,
        vehicle_rear_axle_length(object, *best_path, param_.rear_axle_ratio)),
      false, initial_pose);
  }
  return std::make_shared<const ObjectMotion>(
    detail::generate_static_trajectory(
      detail::to_polygon2d(initial_pose, object.shape), static_times),
    true, initial_pose);
}

std::vector<std::shared_ptr<const ObjectMotion>> CollisionEvaluator::build_object_motions(
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
  const autoware_perception_msgs::msg::PredictedObjects & objects_at_pointcloud_stamp) const
{
  std::vector<std::shared_ptr<const ObjectMotion>> motions;
  if (predicted_trajectory.points.size() < 2) {
    return motions;
  }
  const double horizon = sweep_horizon(predicted_trajectory, param_);
  const auto num_steps =
    static_cast<size_t>(std::floor(horizon / param_.time_resolution + TIME_INDEX_EPSILON));
  TimeTrajectory static_times;
  static_times.reserve(num_steps + 1);
  for (size_t k = 0; k <= num_steps; ++k) {
    static_times.push_back(static_cast<double>(k) * param_.time_resolution);
  }
  const double time_offset = (rclcpp::Time(objects_at_pointcloud_stamp.header.stamp) -
                              rclcpp::Time(predicted_trajectory.header.stamp))
                               .seconds();
  motions.reserve(objects_at_pointcloud_stamp.objects.size());
  for (const auto & object : objects_at_pointcloud_stamp.objects) {
    motions.push_back(make_object_motion(object, time_offset, horizon, static_times));
  }
  return motions;
}

std::vector<std::optional<size_t>> CollisionEvaluator::attach_clusters_to_objects(
  const std::vector<PointCluster> & clusters,
  const autoware_perception_msgs::msg::PredictedObjects & objects_at_pointcloud_stamp,
  const std::vector<std::shared_ptr<const ObjectMotion>> & motions, const double t_pointcloud) const
{
  using autoware_perception_msgs::msg::Shape;
  struct ObjectArea
  {
    size_t index;
    Polygon2d polygon;
    Point2d origin;
  };
  const auto & margin = param_.object_margin;
  std::vector<ObjectArea> areas;
  areas.reserve(objects_at_pointcloud_stamp.objects.size());
  for (size_t i = 0; i < objects_at_pointcloud_stamp.objects.size(); ++i) {
    const auto & object = objects_at_pointcloud_stamp.objects[i];
    if (i >= motions.size() || !motions[i]) {
      continue;
    }
    const auto label = most_probable_label(object);
    const bool enabled =
      std::find(param_.enabled_labels.begin(), param_.enabled_labels.end(), label) !=
      param_.enabled_labels.end();
    if (!enabled) {
      continue;
    }
    const auto pose = motions[i]->pose_at(t_pointcloud);
    const double yaw = autoware_utils_geometry::get_rpy(pose).z;

    const auto & shape = object.shape;
    double x_min = 0.0;
    double x_max = 0.0;
    double y_min = 0.0;
    double y_max = 0.0;
    if (shape.type == Shape::BOUNDING_BOX) {
      x_max = 0.5 * shape.dimensions.x;
      x_min = -x_max;
      y_max = 0.5 * shape.dimensions.y;
      y_min = -y_max;
    } else if (shape.type == Shape::CYLINDER) {
      x_max = y_max = 0.5 * shape.dimensions.x;
      x_min = y_min = -x_max;
    } else {
      if (shape.footprint.points.empty()) {
        continue;
      }
      x_min = y_min = std::numeric_limits<double>::infinity();
      x_max = y_max = -std::numeric_limits<double>::infinity();
      for (const auto & p : shape.footprint.points) {
        x_min = std::min(x_min, static_cast<double>(p.x));
        x_max = std::max(x_max, static_cast<double>(p.x));
        y_min = std::min(y_min, static_cast<double>(p.y));
        y_max = std::max(y_max, static_cast<double>(p.y));
      }
    }
    x_max += margin.front;
    x_min -= margin.rear;
    y_max += margin.left;
    y_min -= margin.right;

    Polygon2d polygon;
    for (const auto & [lx, ly] :
         {std::pair{x_max, y_max}, std::pair{x_min, y_max}, std::pair{x_min, y_min},
          std::pair{x_max, y_min}}) {
      polygon.outer().emplace_back(
        pose.position.x + std::cos(yaw) * lx - std::sin(yaw) * ly,
        pose.position.y + std::sin(yaw) * lx + std::cos(yaw) * ly);
    }
    boost::geometry::correct(polygon);
    areas.push_back(ObjectArea{i, std::move(polygon), Point2d{pose.position.x, pose.position.y}});
  }

  std::vector<std::optional<size_t>> attached_to;
  attached_to.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    std::optional<size_t> best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto & area : areas) {
      if (!boost::geometry::within(cluster.centroid, area.polygon)) {
        continue;
      }
      const double distance = boost::geometry::distance(cluster.centroid, area.origin);
      if (distance < best_distance) {
        best_distance = distance;
        best = area.index;
      }
    }
    attached_to.push_back(best);
  }
  return attached_to;
}

CollisionCandidate CollisionEvaluator::evaluate_object(
  const EgoSweep & ego, SweepsWithBraking & braked, CollisionCandidate candidate,
  const ObjectMotion & motion) const
{
  const auto & object_trajectory = motion.trajectory();
  const auto step = find_first_collision(ego.trajectory, object_trajectory, ego.stopped);
  if (!step) {
    return candidate;
  }
  const size_t k = step->ego_index;
  candidate.ttc = ego.times[k];
  const auto hit = object_hit_point(
    ego.polygons[k], object_trajectory.get_or_compute_convex(step->target_index_range));
  candidate.cos_collision = compute_cos_collision(ego.poses[k], ego.forward, hit);
  if (!passes_cos_threshold(candidate.cos_collision, param_.cos_threshold)) {
    return candidate;
  }
  candidate.brake_level = brake_level_of([&](const size_t level) {
    const auto & braked_ego = braked.at(level);
    return find_first_collision(braked_ego.trajectory, object_trajectory, braked_ego.stopped)
      .has_value();
  });
  return candidate;
}

void CollisionEvaluator::evaluate_static_points(
  const EgoSweep & ego, SweepsWithBraking & braked, const std::vector<Point2d> & points,
  const builtin_interfaces::msg::Time & stamp, std::vector<CollisionCandidate> & out) const
{
  for (const auto & point : points) {
    const auto k = find_first_collision(ego.trajectory, point, ego.stopped);
    if (!k) {
      continue;
    }
    auto candidate = make_point_candidate(ego, stamp, point, *k, point);
    candidate.brake_level = brake_level_of([&](const size_t level) {
      const auto & braked_ego = braked.at(level);
      return find_first_collision(braked_ego.trajectory, point, braked_ego.stopped).has_value();
    });
    out.push_back(std::move(candidate));
  }
}

void CollisionEvaluator::evaluate_attached_points(
  const EgoSweep & ego, SweepsWithBraking & braked, const std::vector<Point2d> & points_local,
  const ObjectMotion & motion, const builtin_interfaces::msg::Time & stamp,
  std::vector<CollisionCandidate> & out) const
{
  const auto ego_local = ego_trajectory_in_object_frame(ego, motion);
  std::array<std::optional<EgoInObjectFrame>, 3> braked_local;
  const auto braked_local_at = [&](const size_t level) -> const EgoInObjectFrame & {
    auto & sweep = braked_local.at(level - 1);
    if (!sweep) {
      sweep.emplace(ego_trajectory_in_object_frame(braked.at(level), motion));
    }
    return *sweep;
  };

  for (const auto & point : points_local) {
    const auto k = find_first_collision(ego_local.trajectory, point, ego.stopped);
    if (!k) {
      continue;
    }
    const auto point_now = detail::transform_point(ego_local.object_to_map.front(), point);
    const auto hit_point = detail::transform_point(ego_local.object_to_map[*k], point);
    auto candidate = make_point_candidate(ego, stamp, point_now, *k, hit_point);
    if (passes_cos_threshold(candidate.cos_collision, param_.cos_threshold)) {
      candidate.brake_level = brake_level_of([&](const size_t level) {
        return find_first_collision(
                 braked_local_at(level).trajectory, point, braked.at(level).stopped)
          .has_value();
      });
    }
    out.push_back(std::move(candidate));
  }
}

std::vector<CollisionCandidate> CollisionEvaluator::evaluate(
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
  const nav_msgs::msg::Odometry & odom, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const autoware_perception_msgs::msg::PredictedObjects * objects_at_pointcloud_stamp,
  const std::vector<PointCluster> & point_clusters,
  const builtin_interfaces::msg::Time & pointcloud_stamp) const
{
  std::vector<CollisionCandidate> candidates;
  if (predicted_trajectory.points.size() < 2 || !(param_.time_resolution > 0.0)) {
    return candidates;
  }

  const EgoSweep ego = build_ego_sweep(predicted_trajectory, odom, vehicle_info);
  const double horizon = sweep_horizon(predicted_trajectory, param_);
  const Polygon2d & ego_now = ego.now();
  SweepsWithBraking braked(*this, predicted_trajectory, odom, vehicle_info);

  const double t_pointcloud =
    (rclcpp::Time(pointcloud_stamp) - rclcpp::Time(predicted_trajectory.header.stamp)).seconds();
  std::vector<std::shared_ptr<const ObjectMotion>> attach_motions;
  std::vector<std::optional<size_t>> attached_to;
  if (param_.enable_pointcloud && objects_at_pointcloud_stamp) {
    attach_motions = build_object_motions(predicted_trajectory, *objects_at_pointcloud_stamp);
    attached_to = attach_clusters_to_objects(
      point_clusters, *objects_at_pointcloud_stamp, attach_motions, t_pointcloud);
  }

  // Objects: closest first, enabled labels only, not touching the current footprint, at most
  // n_eval_object.
  const double objects_time_offset =
    (rclcpp::Time(objects.header.stamp) - rclcpp::Time(predicted_trajectory.header.stamp))
      .seconds();
  struct ObjectEntry
  {
    const autoware_perception_msgs::msg::PredictedObject * object;
    CollisionCandidate candidate;
  };
  std::vector<ObjectEntry> object_entries;
  if (std::abs(objects_time_offset) <= param_.max_objects_stamp_gap_s) {
    for (const auto & object : objects.objects) {
      const auto label = most_probable_label(object);
      const bool enabled =
        std::find(param_.enabled_labels.begin(), param_.enabled_labels.end(), label) !=
        param_.enabled_labels.end();
      if (!enabled) {
        continue;
      }
      CollisionCandidate candidate;
      candidate.kind = CollisionCandidate::Kind::OBJECT;
      candidate.label = label;
      candidate.source_stamp = objects.header.stamp;
      candidate.polygon_now =
        detail::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape);
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
  if (object_entries.size() > param_.n_eval_object) {
    object_entries.resize(param_.n_eval_object);
  }

  std::vector<std::array<uint8_t, 16>> selected_ids;
  selected_ids.reserve(object_entries.size());
  for (auto & entry : object_entries) {
    const auto motion = make_object_motion(*entry.object, objects_time_offset, horizon, ego.times);
    candidates.push_back(evaluate_object(ego, braked, std::move(entry.candidate), *motion));
    selected_ids.push_back(entry.object->object_id.uuid);
  }

  if (!param_.enable_pointcloud) {
    return candidates;
  }

  // Pointcloud: clusters attached to a selected object move with it; the closest
  // n_eval_pointcloud of the other clusters (by centroid) are static.
  const auto evaluate_cluster_points = [&](const size_t index, const auto & evaluate_points) {
    const size_t begin = candidates.size();
    evaluate_points();
    for (size_t i = begin; i < candidates.size(); ++i) {
      candidates[i].cluster_index = index;
    }
  };
  std::vector<std::pair<double, size_t>> ordered;
  ordered.reserve(point_clusters.size());
  for (size_t i = 0; i < point_clusters.size(); ++i) {
    const auto & cluster = point_clusters[i];
    if (cluster.points.empty()) {
      continue;
    }
    const auto attached = i < attached_to.size() ? attached_to[i] : std::nullopt;
    if (!attached) {
      ordered.emplace_back(boost::geometry::distance(ego_now, cluster.centroid), i);
      continue;
    }
    const auto & id = objects_at_pointcloud_stamp->objects[*attached].object_id.uuid;
    if (std::find(selected_ids.begin(), selected_ids.end(), id) == selected_ids.end()) {
      continue;
    }
    const auto & motion = *attach_motions[*attached];
    if (motion.is_static()) {
      evaluate_cluster_points(i, [&] {
        evaluate_static_points(ego, braked, cluster.points, pointcloud_stamp, candidates);
      });
      continue;
    }
    const auto map_to_object = detail::pose_to_isometry(motion.pose_at(t_pointcloud)).inverse();
    std::vector<Point2d> points_local;
    points_local.reserve(cluster.points.size());
    for (const auto & point : cluster.points) {
      points_local.push_back(detail::transform_point(map_to_object, point));
    }
    evaluate_cluster_points(i, [&] {
      evaluate_attached_points(ego, braked, points_local, motion, pointcloud_stamp, candidates);
    });
  }
  std::stable_sort(ordered.begin(), ordered.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.first < rhs.first;
  });
  if (ordered.size() > param_.n_eval_pointcloud) {
    ordered.resize(param_.n_eval_pointcloud);
  }
  for (const auto & [centroid_distance, index] : ordered) {
    evaluate_cluster_points(index, [&] {
      evaluate_static_points(
        ego, braked, point_clusters[index].points, pointcloud_stamp, candidates);
    });
  }

  return candidates;
}
}  // namespace autoware::control_validator::collision_validator
