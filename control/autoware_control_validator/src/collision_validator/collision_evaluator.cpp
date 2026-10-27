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
#include <type_traits>
#include <utility>
#include <variant>
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
  const TrajectoryData & ref, const TrajectoryData & test, const std::vector<bool> & ref_stopped)
{
  const auto overall_test_index_range =
    test.resolve_covering_index_range({ref.getTimes().front(), ref.getTimes().back()});
  if (
    !overall_test_index_range || !boost::geometry::intersects(
                                   ref.get_or_compute_overall_envelope(),
                                   test.get_or_compute_envelope(*overall_test_index_range))) {
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

  for (size_t k = 1; k < ref.size(); ++k) {
    if (k < ref_stopped.size() && ref_stopped[k]) {
      continue;
    }
    const TimeRange window{ref.getTimes().at(k - 1), ref.getTimes().at(k)};
    const IndexRange ref_index_range{k - 1, k};
    const auto test_index_range = test.resolve_covering_index_range(window);
    if (!test_index_range) {
      continue;
    }
    if (has_intersects(ref, ref_index_range, test, *test_index_range)) {
      return CollisionStep{k, *test_index_range};
    }
  }

  return std::nullopt;
}

std::optional<size_t> find_first_collision(
  const TrajectoryData & ref, const Point2d & point, const std::vector<bool> & ref_stopped)
{
  if (!boost::geometry::within(point, ref.get_or_compute_overall_envelope())) {
    return std::nullopt;
  }
  for (size_t k = 1; k < ref.size(); ++k) {
    if (k < ref_stopped.size() && ref_stopped[k]) {
      continue;
    }
    const IndexRange window{k - 1, k};
    if (
      boost::geometry::within(point, ref.get_or_compute_envelope(window)) &&
      boost::geometry::within(point, ref.get_or_compute_convex(window))) {
      return k;
    }
  }
  return std::nullopt;
}
}  // namespace detail

EgoSweep::EgoSweep(
  detail::TimeTrajectory times_in, detail::PoseTrajectory poses_in,
  const detail::FootprintTrajectory & footprints, std::vector<bool> stopped_in,
  const bool forward_in)
: times(std::move(times_in)),
  poses(std::move(poses_in)),
  stopped(std::move(stopped_in)),
  trajectory(times, poses, footprints),
  forward(forward_in)
{
  polygons.reserve(times.size());
  for (size_t k = 0; k < times.size(); ++k) {
    polygons.push_back(
      std::visit(
        [k](const auto & footprint) { return detail::to_polygon2d(footprint.at(k)); }, footprints));
  }
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
using detail::find_first_collision;
using detail::FootprintTrajectory;
using detail::PoseTrajectory;
using detail::TIME_INDEX_EPSILON;
using detail::TimeTrajectory;
using detail::TrajectoryData;

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
  double best_distance = std::numeric_limits<double>::infinity();
  Point2d best_a;
  Point2d best_b;
  const auto update = [&](const Point2d & point_a, const Point2d & point_b) {
    const double distance = boost::geometry::distance(point_a, point_b);
    if (distance < best_distance) {
      best_distance = distance;
      best_a = point_a;
      best_b = point_b;
    }
  };
  const auto project = [](const Point2d & p, const Point2d & s0, const Point2d & s1) {
    const Eigen::Vector2d segment = s1 - s0;
    const double len2 = segment.squaredNorm();
    if (len2 < 1e-12) return s0;
    const double ratio = std::clamp(static_cast<double>((p - s0).dot(segment)) / len2, 0.0, 1.0);
    return Point2d{s0.x() + ratio * segment.x(), s0.y() + ratio * segment.y()};
  };
  const auto & ring_a = a.outer();
  const auto & ring_b = b.outer();
  for (size_t i = 0; i + 1 < ring_b.size(); ++i) {
    for (const auto & point_a : ring_a) {
      update(point_a, project(point_a, ring_b[i], ring_b[i + 1]));
    }
  }
  for (size_t i = 0; i + 1 < ring_a.size(); ++i) {
    for (const auto & point_b : ring_b) {
      update(project(point_b, ring_a[i], ring_a[i + 1]), point_b);
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
  return polygon;
}

CollisionCandidate make_point_candidate(const EgoSweep & ego_sweep, const Point2d & point)
{
  CollisionCandidate candidate;
  candidate.polygon_now = point_polygon(point);
  candidate.distance_now = boost::geometry::distance(ego_sweep.now(), point);
  return candidate;
}

bool is_all_stopped(const EgoSweep & ego_sweep)
{
  return std::all_of(
    ego_sweep.stopped.begin(), ego_sweep.stopped.end(), [](const bool stopped) { return stopped; });
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

  double stop_time(const size_t level) const
  {
    auto & time = stop_times_.at(level - 1);
    if (!time) {
      const double reaction_time = evaluator_.param_.reaction_times.at(level - 1);
      const detail::TrajectoryInterpolator interpolator(predicted_trajectory_);
      const auto state = interpolator.interpolate_state_from_time(
        interpolator.reference_time_ + rclcpp::Duration::from_seconds(reaction_time));
      time = reaction_time +
             std::abs(state.longitudinal_velocity) / evaluator_.param_.braking_deceleration;
    }
    return *time;
  }

private:
  const CollisionEvaluator & evaluator_;
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory_;
  const nav_msgs::msg::Odometry & odom_;
  const vehicle_info_utils::VehicleInfo & vehicle_info_;
  mutable std::array<std::optional<EgoSweep>, 3> sweeps_;
  mutable std::array<std::optional<double>, 3> stop_times_;
};

CollisionEvaluator::CollisionEvaluator(const CollisionEvaluatorParam & param) : param_(param)
{
}

void CollisionEvaluator::update_parameters(const CollisionEvaluatorParam & param)
{
  param_ = param;
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

TrajectoryData CollisionEvaluator::make_object_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & object, const double time_offset,
  const TimeTrajectory & static_times) const
{
  const auto & paths = object.kinematics.predicted_paths;
  const auto best_path = std::max_element(
    paths.begin(), paths.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.confidence < rhs.confidence; });
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  const bool is_moving = std::hypot(twist.linear.x, twist.linear.y) > 1e-3;
  const auto & initial_pose = object.kinematics.initial_pose_with_covariance.pose;

  if (best_path != paths.end() && !best_path->path.empty() && is_moving) {
    const double path_length_time = static_cast<double>(best_path->path.size()) *
                                    rclcpp::Duration(best_path->time_step).seconds();
    return detail::generate_predicted_path_trajectory(
      object, *best_path, path_length_time, time_offset, param_.time_resolution,
      vehicle_rear_axle_length(object, *best_path, param_.rear_axle_ratio));
  }
  return detail::generate_static_trajectory(
    detail::to_polygon2d(initial_pose, object.shape), static_times);
}

CollisionCandidate CollisionEvaluator::evaluate_object(
  const EgoSweep & ego_sweep, SweepsWithBraking & braked_sweeps, CollisionCandidate candidate,
  const TrajectoryData & object_trajectory) const
{
  const auto step =
    find_first_collision(ego_sweep.trajectory, object_trajectory, ego_sweep.stopped);
  if (!step) {
    return candidate;
  }
  const size_t k = step->ref_index;
  const auto hit_point = object_hit_point(
    ego_sweep.polygons[k], object_trajectory.get_or_compute_convex(step->test_index_range));
  if (
    compute_cos_collision(ego_sweep.poses[k], ego_sweep.forward, hit_point) <
    param_.cos_threshold) {
    return candidate;
  }
  std::optional<bool> reaches_ego_now;
  candidate.brake_level = brake_level_of([&](const size_t level) {
    const auto & braked_sweep = braked_sweeps.at(level);
    const auto braked_step =
      find_first_collision(braked_sweep.trajectory, object_trajectory, braked_sweep.stopped);
    if (!braked_step) {
      return false;
    }
    if (braked_sweep.times[braked_step->ref_index] < braked_sweeps.stop_time(level)) {
      return true;
    }
    if (!reaches_ego_now) {
      reaches_ego_now =
        find_first_collision(
          object_trajectory,
          detail::generate_static_trajectory(ego_sweep.now(), object_trajectory.getTimes()), {})
          .has_value();
    }
    return !*reaches_ego_now;
  });
  return candidate;
}

void CollisionEvaluator::evaluate_points(
  const EgoSweep & ego_sweep, SweepsWithBraking & braked_sweeps,
  const std::vector<Point2d> & points, std::vector<CollisionCandidate> & out) const
{
  const bool all_stopped = is_all_stopped(ego_sweep);
  for (const auto & point : points) {
    auto candidate = make_point_candidate(ego_sweep, point);
    candidate.near = candidate.distance_now <= param_.proximity_distance && !all_stopped;
    if (find_first_collision(ego_sweep.trajectory, point, ego_sweep.stopped)) {
      candidate.brake_level = brake_level_of([&](const size_t level) {
        const auto & braked_sweep = braked_sweeps.at(level);
        return find_first_collision(braked_sweep.trajectory, point, braked_sweep.stopped)
          .has_value();
      });
    }
    if (candidate.near || candidate.brake_level >= 1) {
      out.push_back(std::move(candidate));
    }
  }
}

std::vector<CollisionCandidate> CollisionEvaluator::evaluate(
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
  const nav_msgs::msg::Odometry & odom, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const autoware_perception_msgs::msg::PredictedObjects * objects,
  const PointCloudClusters * point_clusters) const
{
  std::vector<CollisionCandidate> candidates;
  if (predicted_trajectory.points.size() < 2 || !(param_.time_resolution > 0.0)) {
    return candidates;
  }

  const EgoSweep ego_sweep = build_ego_sweep(predicted_trajectory, odom, vehicle_info);
  const Polygon2d & ego_now = ego_sweep.now();
  SweepsWithBraking braked_sweeps(*this, predicted_trajectory, odom, vehicle_info);

  // Objects: closest first, enabled labels only, not touching the current footprint, at most
  // n_eval_object.
  const double objects_time_offset =
    objects
      ? (rclcpp::Time(objects->header.stamp) - rclcpp::Time(predicted_trajectory.header.stamp))
          .seconds()
      : 0.0;
  struct ObjectEntry
  {
    const autoware_perception_msgs::msg::PredictedObject * object;
    CollisionCandidate candidate;
  };
  std::vector<ObjectEntry> object_entries;
  if (objects && std::abs(objects_time_offset) <= param_.max_objects_stamp_gap_s) {
    for (const auto & object : objects->objects) {
      const auto label = most_probable_label(object);
      const bool enabled =
        std::find(param_.enabled_labels.begin(), param_.enabled_labels.end(), label) !=
        param_.enabled_labels.end();
      if (!enabled) {
        continue;
      }
      CollisionCandidate candidate;
      candidate.polygon_now =
        detail::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape);
      if (boost::geometry::intersects(ego_now, candidate.polygon_now)) {
        continue;
      }
      candidate.distance_now = boost::geometry::distance(ego_now, candidate.polygon_now);
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

  const bool all_stopped = is_all_stopped(ego_sweep);
  for (auto & entry : object_entries) {
    entry.candidate.near =
      entry.candidate.distance_now <= param_.proximity_distance && !all_stopped;
    const auto object_trajectory =
      make_object_trajectory(*entry.object, objects_time_offset, ego_sweep.times);
    auto candidate =
      evaluate_object(ego_sweep, braked_sweeps, std::move(entry.candidate), object_trajectory);
    if (candidate.near || candidate.brake_level >= 1) {
      candidates.push_back(std::move(candidate));
    }
  }

  if (!point_clusters) {
    return candidates;
  }

  // Pointcloud: the closest n_eval_pointcloud clusters (by centroid), as static points.
  const auto & clusters = point_clusters->clusters;
  std::vector<std::pair<double, size_t>> selected_clusters;
  selected_clusters.reserve(clusters.size());
  for (size_t i = 0; i < clusters.size(); ++i) {
    selected_clusters.emplace_back(boost::geometry::distance(ego_now, clusters[i].centroid), i);
  }
  std::stable_sort(
    selected_clusters.begin(), selected_clusters.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });
  if (selected_clusters.size() > param_.n_eval_pointcloud) {
    selected_clusters.resize(param_.n_eval_pointcloud);
  }
  for (const auto & [centroid_distance, index] : selected_clusters) {
    evaluate_points(ego_sweep, braked_sweeps, clusters[index].points, candidates);
  }

  return candidates;
}
}  // namespace autoware::control_validator::collision_validator
