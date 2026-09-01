// Copyright 2026 Autoware Foundation
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

#include "autoware/avoidance_target_detector/object_filtering.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{

namespace
{

using autoware_planning_msgs::msg::TrajectoryPoint;

constexpr double k_s_position_epsilon_m = 1e-3;

constexpr double k_high_likelihood = 0.95;
constexpr double k_low_likelihood = 0.05;
constexpr double k_neutral_likelihood = 0.5;
constexpr double k_motion_epsilon = 1e-6;

// Kinematics accessors abstracting the field differences between object message types.
const geometry_msgs::msg::Pose & get_object_pose(const PredictedObject & object)
{
  return object.kinematics.initial_pose_with_covariance.pose;
}

const geometry_msgs::msg::Pose & get_object_pose(const TrackedObject & object)
{
  return object.kinematics.pose_with_covariance.pose;
}

const geometry_msgs::msg::Twist & get_object_twist(const PredictedObject & object)
{
  return object.kinematics.initial_twist_with_covariance.twist;
}

const geometry_msgs::msg::Twist & get_object_twist(const TrackedObject & object)
{
  return object.kinematics.twist_with_covariance.twist;
}

template <typename ObjectT>
bool is_object_of_interest(const ObjectT & object)
{
  constexpr float probability_threshold = 0.1f;
  return std::any_of(
    object.classification.begin(), object.classification.end(),
    [](const ObjectClassification & classification) {
      if (classification.probability <= probability_threshold) {
        return false;
      }
      return std::find(
               labels_of_interest.begin(), labels_of_interest.end(), classification.label) !=
             labels_of_interest.end();
    });
}

std::optional<autoware_utils_geometry::Polygon2d> build_near_segment_polygon(
  const lanelet::BasicPolygon2d & near_segment_polygon)
{
  if (near_segment_polygon.size() < 3) {
    return std::nullopt;
  }

  autoware_utils_geometry::Polygon2d segment_polygon;
  auto & outer = segment_polygon.outer();
  outer.reserve(near_segment_polygon.size());
  for (const auto & point : near_segment_polygon) {
    outer.emplace_back(point.x(), point.y());
  }
  boost::geometry::correct(segment_polygon);
  return segment_polygon;
}

template <typename ObjectT>
bool overlaps_near_segment_polygon(
  const ObjectT & object, const autoware_utils_geometry::Polygon2d & near_segment_polygon)
{
  const auto object_polygon = autoware_utils_geometry::to_polygon2d(object);
  return boost::geometry::intersects(object_polygon, near_segment_polygon);
}

std::vector<lanelet::ConstLanelet> get_nearest_lanelets(
  const lanelet::LaneletMap & route_map, const geometry_msgs::msg::Point & point)
{
  constexpr size_t k_max_nearest_lanelets = 5;
  const lanelet::BasicPoint2d search_point{point.x, point.y};
  std::vector<lanelet::ConstLanelet> nearest_lanelets;
  nearest_lanelets.reserve(k_max_nearest_lanelets);

  route_map.laneletLayer.nearestUntil(
    search_point, [&](const lanelet::BoundingBox2d & bbox, const lanelet::ConstLanelet & lanelet) {
      if (lanelet::geometry::inside(lanelet, search_point)) {
        nearest_lanelets.push_back(lanelet);
        return nearest_lanelets.size() >= k_max_nearest_lanelets;
      }
      constexpr double k_bbox_touch_epsilon_m = 1e-3;
      return lanelet::geometry::distance2d(bbox, search_point) > k_bbox_touch_epsilon_m;
    });

  return nearest_lanelets;
}

bool has_shortest_path_without_lane_change(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::ConstLanelet & from,
  const lanelet::ConstLanelet & to)
{
  constexpr lanelet::routing::RoutingCostId k_default_routing_cost_id{0};
  constexpr bool k_with_lane_changes{false};
  return static_cast<bool>(
    routing_graph.shortestPath(from, to, k_default_routing_cost_id, k_with_lane_changes));
}

/** Reuses bidirectional routing results for objects assigned to the same lanelet pair. */
template <typename ObjectT>
bool is_routably_connected_to_ego_without_lane_change(
  const FrameEvaluationContext<ObjectT> & context, const lanelet::ConstLanelet & ego_lanelet,
  const lanelet::ConstLanelet & object_lanelet)
{
  const LaneletPairKey key{ego_lanelet.id(), object_lanelet.id()};
  if (const auto iter = context.routability_cache.find(key);
      iter != context.routability_cache.end()) {
    return iter->second;
  }

  const bool is_connected =
    has_shortest_path_without_lane_change(*context.routing_graph, object_lanelet, ego_lanelet) ||
    has_shortest_path_without_lane_change(*context.routing_graph, ego_lanelet, object_lanelet);
  context.routability_cache.emplace(key, is_connected);
  return is_connected;
}

template <typename ObjectT>
double linear_velocity_norm(const ObjectT & object)
{
  const auto & linear = get_object_twist(object).linear;
  return std::hypot(linear.x, linear.y, linear.z);
}

double trajectory_horizon_seconds(const Trajectory & trajectory)
{
  double horizon = 0.0;
  for (const auto & point : trajectory.points) {
    const double seconds = static_cast<double>(point.time_from_start.sec) +
                           static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
    horizon = std::max(horizon, seconds);
  }
  return horizon;
}

template <typename ObjectT>
double circular_footprint_radius(const ObjectT & object)
{
  const auto center = get_object_pose(object).position;
  const auto footprint = autoware_utils_geometry::to_polygon2d(object);
  double radius = 0.0;
  for (const auto & point : footprint.outer()) {
    radius = std::max(radius, std::hypot(point.x() - center.x, point.y() - center.y));
  }
  return radius;
}

double point_to_segment_distance(
  const autoware_utils_geometry::Point2d & point, const autoware_utils_geometry::Point2d & start,
  const autoware_utils_geometry::Point2d & end)
{
  const double vx = end.x() - start.x();
  const double vy = end.y() - start.y();
  const double length_squared = vx * vx + vy * vy;
  if (length_squared <= k_motion_epsilon) {
    return std::hypot(point.x() - start.x(), point.y() - start.y());
  }
  const double projection = std::clamp(
    ((point.x() - start.x()) * vx + (point.y() - start.y()) * vy) / length_squared, 0.0, 1.0);
  return std::hypot(
    point.x() - (start.x() + projection * vx), point.y() - (start.y() + projection * vy));
}

double projected_path_to_trajectory_distance(
  const autoware_utils_geometry::Point2d & motion_start,
  const autoware_utils_geometry::Point2d & motion_end, const Trajectory & trajectory)
{
  if (trajectory.points.size() == 1U) {
    const auto & point = trajectory.points.front().pose.position;
    return point_to_segment_distance(
      autoware_utils_geometry::Point2d{point.x, point.y}, motion_start, motion_end);
  }

  const bool is_stationary =
    std::hypot(motion_end.x() - motion_start.x(), motion_end.y() - motion_start.y()) <=
    k_motion_epsilon;
  const autoware_utils_geometry::Segment2d motion_segment{motion_start, motion_end};
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
    const auto & start = trajectory.points[index - 1].pose.position;
    const auto & end = trajectory.points[index].pose.position;
    const autoware_utils_geometry::Point2d trajectory_start{start.x, start.y};
    const autoware_utils_geometry::Point2d trajectory_end{end.x, end.y};
    if (is_stationary) {
      distance = std::min(
        distance, point_to_segment_distance(motion_start, trajectory_start, trajectory_end));
    } else {
      const autoware_utils_geometry::Segment2d trajectory_segment{trajectory_start, trajectory_end};
      distance = std::min(distance, boost::geometry::distance(motion_segment, trajectory_segment));
    }
  }
  return distance;
}

template <typename ObjectsT>
ObjectsT filter_objects_in_range_impl(
  const ObjectsT & objects, const Trajectory & trajectory, const double margin,
  const double additional_prediction_horizon)
{
  if (!std::isfinite(margin) || margin < 0.0) {
    throw std::invalid_argument("The object filter margin must be finite and nonnegative");
  }
  if (!std::isfinite(additional_prediction_horizon) || additional_prediction_horizon < 0.0) {
    throw std::invalid_argument(
      "The additional object prediction horizon must be finite and nonnegative");
  }
  if (trajectory.points.empty()) {
    return objects;
  }

  const double horizon = trajectory_horizon_seconds(trajectory) + additional_prediction_horizon;
  ObjectsT filtered;
  filtered.header = objects.header;
  filtered.objects.reserve(objects.objects.size());
  for (const auto & object : objects.objects) {
    const auto & pose = get_object_pose(object);
    const double speed = get_object_twist(object).linear.x;
    if (horizon <= k_motion_epsilon && std::abs(speed) > k_motion_epsilon) {
      filtered.objects.push_back(object);
      continue;
    }

    const auto & orientation = pose.orientation;
    const double yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    const autoware_utils_geometry::Point2d motion_start{pose.position.x, pose.position.y};
    const autoware_utils_geometry::Point2d motion_end{
      pose.position.x + speed * std::cos(yaw) * horizon,
      pose.position.y + speed * std::sin(yaw) * horizon};
    const double object_radius = circular_footprint_radius(object);
    const double distance =
      projected_path_to_trajectory_distance(motion_start, motion_end, trajectory);
    if (distance <= margin + object_radius) {
      filtered.objects.push_back(object);
    }
  }
  return filtered;
}

/**
 * @brief Build an interpolated trajectory from a trajectory message.
 * @param trajectory_msg Input trajectory message.
 * @return Interpolated trajectory, or std::nullopt if build fails.
 */
std::optional<aw_trajectory::Trajectory<TrajectoryPoint>> build_trajectory(
  const Trajectory & trajectory_msg)
{
  if (trajectory_msg.points.size() < 2) {
    return std::nullopt;
  }

  const auto trajectory =
    aw_trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(trajectory_msg.points);
  if (!trajectory) {
    return std::nullopt;
  }

  return *trajectory;
}

/** Builds the trajectory and immutable lookup data shared by one frame's object evaluations. */
template <typename ObjectT>
FrameEvaluationContext<ObjectT> make_frame_evaluation_context(
  const rclcpp::Time & current_time, const Trajectory & trajectory)
{
  FrameEvaluationContext<ObjectT> context{
    current_time, build_trajectory(trajectory), {}, {}, std::nullopt, nullptr, nullptr, {}, {}};
  if (context.built_trajectory) {
    context.trajectory_bases = context.built_trajectory->get_underlying_bases();
    context.trajectory_base_points.reserve(context.trajectory_bases.size());
    for (const double s : context.trajectory_bases) {
      context.trajectory_base_points.push_back(context.built_trajectory->compute(s).pose.position);
    }
  }
  return context;
}

/**
 * @brief Get the reference point used for d-coordinate validation.
 * @param object Object.
 * @return Object center position.
 */
template <typename ObjectT>
geometry_msgs::msg::Point get_object_reference_point(const ObjectT & object)
{
  return get_object_pose(object).position;
}

double max_interpolator_safe_s(const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory)
{
  const auto bases = trajectory.get_underlying_bases();
  if (bases.empty()) {
    return trajectory.length();
  }
  return std::min(trajectory.length(), bases.back());
}

double max_interpolator_safe_s(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::vector<double> & trajectory_bases)
{
  if (trajectory_bases.empty()) {
    return trajectory.length();
  }
  return std::min(trajectory.length(), trajectory_bases.back());
}

double clamp_trajectory_s(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const double s)
{
  return std::clamp(s, 0.0, max_interpolator_safe_s(trajectory));
}

double clamp_trajectory_s(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::vector<double> & trajectory_bases, const double s)
{
  return std::clamp(s, 0.0, max_interpolator_safe_s(trajectory, trajectory_bases));
}

double closest_trajectory_s(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & point)
{
  const auto s =
    aw_trajectory::closest_with_constraint(trajectory, point, [](const double &) { return true; });
  if (!s.has_value()) {
    return clamp_trajectory_s(trajectory, 0.0);
  }
  return clamp_trajectory_s(trajectory, *s);
}

double closest_trajectory_s(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::vector<double> & trajectory_bases, const geometry_msgs::msg::Point & point)
{
  const auto s =
    aw_trajectory::closest_with_constraint(trajectory, point, [](const double &) { return true; });
  if (!s.has_value()) {
    return clamp_trajectory_s(trajectory, trajectory_bases, 0.0);
  }
  return clamp_trajectory_s(trajectory, trajectory_bases, *s);
}

/** Cached geometry and arc-length bounds for one trajectory segment. */
struct TrajectorySegment
{
  double s_start;
  double s_end;
  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;
};

/** Closest-point result for one trajectory segment. */
struct SegmentProjection
{
  double s;
  double distance;
};

/** Minimum and maximum trajectory arc lengths covered by an object footprint. */
struct FootprintSRange
{
  double min;
  double max;
};

/** Projects a point onto a segment using the same endpoint rules as the full trajectory lookup. */
SegmentProjection project_onto_segment(
  const TrajectorySegment & segment, const geometry_msgs::msg::Point & point)
{
  const double vx = segment.end.x - segment.start.x;
  const double vy = segment.end.y - segment.start.y;
  const double vz = segment.end.z - segment.start.z;
  const double wx = point.x - segment.start.x;
  const double wy = point.y - segment.start.y;
  const double wz = point.z - segment.start.z;
  const double c1 = wx * vx + wy * vy + wz * vz;
  const double c2 = vx * vx + vy * vy + vz * vz;

  double ratio = 0.0;
  double s = segment.s_start;
  if (c2 > std::numeric_limits<double>::epsilon()) {
    if (c2 <= c1) {
      ratio = 1.0;
      s = segment.s_end;
    } else if (c1 > 0.0) {
      ratio = c1 / c2;
      s = segment.s_start + ratio * (segment.s_end - segment.s_start);
    }
  }

  const double dx = point.x - (segment.start.x + ratio * vx);
  const double dy = point.y - (segment.start.y + ratio * vy);
  const double dz = point.z - (segment.start.z + ratio * vz);
  return {s, std::sqrt(dx * dx + dy * dy + dz * dz)};
}

/** Returns a lightweight segment view backed by frame-cached trajectory data. */
TrajectorySegment get_trajectory_segment(
  const std::vector<double> & bases,
  const std::vector<geometry_msgs::msg::Point> & trajectory_base_points,
  const std::size_t end_index)
{
  return {
    bases[end_index - 1], bases[end_index], trajectory_base_points[end_index - 1],
    trajectory_base_points[end_index]};
}

/** Finds the closest projection among segments intersecting the requested arc-length window. */
std::optional<SegmentProjection> closest_projection_in_window(
  const std::vector<double> & bases,
  const std::vector<geometry_msgs::msg::Point> & trajectory_base_points,
  const geometry_msgs::msg::Point & point, const double s_lo, const double s_hi)
{
  std::optional<SegmentProjection> closest;
  if (bases.size() < 2 || trajectory_base_points.size() != bases.size()) {
    return closest;
  }

  const auto first_end = std::lower_bound(std::next(bases.begin()), bases.end(), s_lo);
  for (auto end = first_end; end != bases.end(); ++end) {
    const std::size_t end_index = static_cast<std::size_t>(std::distance(bases.begin(), end));
    const auto segment = get_trajectory_segment(bases, trajectory_base_points, end_index);
    if (segment.s_start > s_hi) {
      break;
    }
    const auto projection = project_onto_segment(segment, point);
    if (!closest || projection.distance < closest->distance) {
      closest = projection;
    }
  }
  return closest;
}

/** Finds the center-point distance bound for every segment excluded from the local window. */
double closest_distance_outside_window(
  const std::vector<double> & bases,
  const std::vector<geometry_msgs::msg::Point> & trajectory_base_points,
  const geometry_msgs::msg::Point & point, const double s_lo, const double s_hi)
{
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < bases.size(); ++i) {
    const auto segment = get_trajectory_segment(bases, trajectory_base_points, i);
    if (segment.s_end >= s_lo && segment.s_start <= s_hi) {
      continue;
    }
    closest_distance = std::min(closest_distance, project_onto_segment(segment, point).distance);
  }
  return closest_distance;
}

/** Computes the 3D Euclidean distance used by the local-search correctness bound. */
double point_distance(const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief Compute the arc-length range of an object footprint along the trajectory.
 * @param trajectory Interpolated reference trajectory.
 * @param trajectory_bases Cached trajectory arc-length bases.
 * @param trajectory_base_points Cached positions at the trajectory bases.
 * @param object Object.
 * @return Minimum and maximum footprint s [m].
 */
template <typename ObjectT>
FootprintSRange get_footprint_s_range(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::vector<double> & trajectory_bases,
  const std::vector<geometry_msgs::msg::Point> & trajectory_base_points, const ObjectT & object,
  const autoware_utils_geometry::Polygon2d & footprint)
{
  FootprintSRange range{std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
  const auto reference_point = get_object_reference_point(object);
  const double s_reference = closest_trajectory_s(trajectory, trajectory_bases, reference_point);
  constexpr double k_local_s_window_m = 15.0;
  const double s_lo = std::max(0.0, s_reference - k_local_s_window_m);
  const double s_hi = std::min(
    max_interpolator_safe_s(trajectory, trajectory_bases), s_reference + k_local_s_window_m);
  const double closest_outside_distance = closest_distance_outside_window(
    trajectory_bases, trajectory_base_points, reference_point, s_lo, s_hi);

  const auto update_s_range = [&](const geometry_msgs::msg::Point & point) {
    const auto local_projection =
      closest_projection_in_window(trajectory_bases, trajectory_base_points, point, s_lo, s_hi);
    const double reference_to_point_distance = point_distance(reference_point, point);

    // The triangle inequality proves the local result is globally closest. On looping or
    // self-near trajectories where that cannot be proven, preserve behavior with the full search.
    const bool local_is_globally_closest =
      local_projection &&
      (std::isinf(closest_outside_distance) ||
       local_projection->distance < closest_outside_distance - reference_to_point_distance);
    const double s = local_is_globally_closest
                       ? clamp_trajectory_s(trajectory, trajectory_bases, local_projection->s)
                       : closest_trajectory_s(trajectory, trajectory_bases, point);
    range.min = std::min(range.min, s);
    range.max = std::max(range.max, s);
  };

  if (footprint.outer().empty()) {
    update_s_range(reference_point);
    return range;
  }

  for (const auto & footprint_point : footprint.outer()) {
    update_s_range(
      autoware_utils_geometry::create_point(footprint_point.x(), footprint_point.y(), 0.0));
  }
  return range;
}

/**
 * @brief Check whether the object footprint lies beyond the trajectory end in s.
 * @param trajectory Interpolated reference trajectory.
 * @param range Cached footprint s range.
 * @return True if the minimum footprint s exceeds trajectory.length().
 */
bool is_beyond_trajectory_end(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const FootprintSRange & range)
{
  return range.min > trajectory.length() + k_s_position_epsilon_m;
}

/**
 * @brief Check whether the object footprint lies within the trajectory s-range.
 * @param trajectory Interpolated reference trajectory.
 * @param range Cached footprint s range.
 * @return True if footprint s is within [0, trajectory.length()].
 */
bool is_within_trajectory_s_range(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const FootprintSRange & range)
{
  return range.min >= -k_s_position_epsilon_m &&
         range.max <= trajectory.length() + k_s_position_epsilon_m;
}

/**
 * @brief Get the last M arc-length samples from the trajectory.
 * @param trajectory_bases Cached trajectory arc-length bases.
 * @return Vector of s values [m] (up to OnTrajectoryDValidationParams::sample_count_m).
 */
std::vector<double> get_last_m_s_samples(const std::vector<double> & trajectory_bases)
{
  if (trajectory_bases.empty()) {
    return {};
  }

  const std::size_t sample_count =
    std::min(OnTrajectoryDValidationParams::sample_count_m, trajectory_bases.size());
  return {
    trajectory_bases.end() - static_cast<std::ptrdiff_t>(sample_count), trajectory_bases.end()};
}

/**
 * @brief Get trajectory s samples near the object footprint within S meters.
 * @param trajectory Interpolated reference trajectory.
 * @param trajectory_bases Cached trajectory arc-length bases.
 * @param range Cached footprint s range.
 * @return Vector of s values [m] in [s_min - S, s_max + S].
 */
std::vector<double> get_s_samples_near_object(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::vector<double> & trajectory_bases, const FootprintSRange & range)
{
  const double s_lo = std::max(0.0, range.min - OnTrajectoryDValidationParams::near_s_range_m);
  const double s_hi = std::min(
    max_interpolator_safe_s(trajectory, trajectory_bases),
    range.max + OnTrajectoryDValidationParams::near_s_range_m);

  if (s_hi < s_lo) {
    return {};
  }

  const auto first = std::lower_bound(trajectory_bases.begin(), trajectory_bases.end(), s_lo);
  const auto last = std::upper_bound(first, trajectory_bases.end(), s_hi);
  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(std::distance(first, last)));
  samples.insert(samples.end(), first, last);

  if (samples.size() < 2) {
    samples.clear();
    const auto fallback_sample_count =
      static_cast<std::size_t>(
        std::floor((s_hi - s_lo) / OnTrajectoryDValidationParams::s_sample_interval_m + 0.5)) +
      1U;
    samples.reserve(fallback_sample_count);
    for (double s = s_lo; s <= s_hi + OnTrajectoryDValidationParams::s_sample_interval_m * 0.5;
         s += OnTrajectoryDValidationParams::s_sample_interval_m) {
      samples.push_back(std::min(s, s_hi));
    }
  }

  return samples;
}

/**
 * @brief Get rear-left and rear-right corners of a box centered on the object pose.
 * @param object Object supplying the pose the box is centered on and aligned with.
 * @param length Box extent along the object x axis [m].
 * @param width Box extent along the object y axis [m].
 * @return Rear edge corner points in map frame.
 */
template <typename ObjectT>
std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> get_box_rear_edge_points(
  const ObjectT & object, const double length, const double width)
{
  const auto & object_pose = get_object_pose(object);
  const double rear_longitudinal_offset = -0.5 * length;
  const double half_width = 0.5 * width;

  const auto rear_left = autoware_utils_geometry::calc_offset_pose(
                           object_pose, rear_longitudinal_offset, half_width, 0.0)
                           .position;
  const auto rear_right = autoware_utils_geometry::calc_offset_pose(
                            object_pose, rear_longitudinal_offset, -half_width, 0.0)
                            .position;
  return {rear_left, rear_right};
}

/**
 * @brief Check whether shape.dimensions defines a usable bounding box.
 * @param object Object.
 * @return True if both of dimensions x and y are non-zero.
 * @details The rear edge is a planar quantity, so only the length and width are required. A
 *          flat box with dimensions.z of zero still yields valid rear corners.
 */
template <typename ObjectT>
bool has_valid_bounding_box_dimensions(const ObjectT & object)
{
  constexpr double k_dimension_epsilon_m = 1e-6;
  const auto & dimensions = object.shape.dimensions;
  return std::abs(dimensions.x) > k_dimension_epsilon_m &&
         std::abs(dimensions.y) > k_dimension_epsilon_m;
}

/**
 * @brief Get rear-left and rear-right corners of the object footprint.
 * @param object Object.
 * @param footprint Object footprint in map frame; used only when no bounding box is available.
 * @return Rear edge corner points in map frame.
 * @details Resolved in three steps. A CYLINDER only carries a diameter in dimensions.x, so it is
 *          bounded by a square of that diameter. Otherwise, a non-zero length and width define a
 *          bounding box and are preferred whatever the shape type is, so a POLYGON that also
 *          reports dimensions uses the box. Only when neither applies is the rearmost edge
 *          scanned out of the footprint polygon.
 */
template <typename ObjectT>
std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> get_object_rear_edge_points(
  const ObjectT & object, const autoware_utils_geometry::Polygon2d & footprint)
{
  if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    const double diameter = object.shape.dimensions.x;
    return get_box_rear_edge_points(object, diameter, diameter);
  }

  if (has_valid_bounding_box_dimensions(object)) {
    return get_box_rear_edge_points(object, object.shape.dimensions.x, object.shape.dimensions.y);
  }

  const auto & object_pose = get_object_pose(object);

  if (footprint.outer().empty()) {
    const auto reference_point = get_object_reference_point(object);
    return {reference_point, reference_point};
  }

  double rear_longitudinal = std::numeric_limits<double>::max();
  for (const auto & footprint_point : footprint.outer()) {
    const auto point =
      autoware_utils_geometry::create_point(footprint_point.x(), footprint_point.y(), 0.0);
    rear_longitudinal = std::min(
      rear_longitudinal, autoware_utils_geometry::calc_longitudinal_deviation(object_pose, point));
  }

  constexpr double k_rear_edge_tolerance_m = 0.01;
  geometry_msgs::msg::Point rear_left;
  geometry_msgs::msg::Point rear_right;
  double rear_left_lateral = std::numeric_limits<double>::lowest();
  double rear_right_lateral = std::numeric_limits<double>::max();
  bool has_rear_left = false;
  bool has_rear_right = false;

  for (const auto & footprint_point : footprint.outer()) {
    const auto point =
      autoware_utils_geometry::create_point(footprint_point.x(), footprint_point.y(), 0.0);
    const double longitudinal =
      autoware_utils_geometry::calc_longitudinal_deviation(object_pose, point);
    if (rear_longitudinal + k_rear_edge_tolerance_m < longitudinal) {
      continue;
    }

    const double lateral = autoware_utils_geometry::calc_lateral_deviation(object_pose, point);
    if (lateral > rear_left_lateral) {
      rear_left_lateral = lateral;
      rear_left = point;
      has_rear_left = true;
    }
    if (lateral < rear_right_lateral) {
      rear_right_lateral = lateral;
      rear_right = point;
      has_rear_right = true;
    }
  }

  if (!has_rear_left || !has_rear_right) {
    const auto reference_point = get_object_reference_point(object);
    return {reference_point, reference_point};
  }

  return {rear_left, rear_right};
}

/**
 * @brief Compute |d| at a given trajectory s from the object rear edge.
 * @details When the rear edge straddles the trajectory (d_rear_left > 0 and d_rear_right < 0),
 *          |d| is 0. Otherwise |d| is taken from the rear corner closest to the trajectory.
 */
double calc_rear_edge_d_magnitude_at_s(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::vector<double> & trajectory_bases, const double s,
  const geometry_msgs::msg::Point & rear_left, const geometry_msgs::msg::Point & rear_right)
{
  const auto trajectory_point =
    trajectory.compute(clamp_trajectory_s(trajectory, trajectory_bases, s));
  const double d_rear_left =
    autoware_utils_geometry::calc_lateral_deviation(trajectory_point.pose, rear_left);
  const double d_rear_right =
    autoware_utils_geometry::calc_lateral_deviation(trajectory_point.pose, rear_right);

  if (d_rear_left > 0.0 && d_rear_right < 0.0) {
    return 0.0;
  }
  if (d_rear_left > 0.0 && d_rear_right > 0.0) {
    return std::abs(d_rear_right);
  }
  if (d_rear_left < 0.0 && d_rear_right < 0.0) {
    return std::abs(d_rear_left);
  }

  return std::max(std::abs(d_rear_left), std::abs(d_rear_right));
}

/**
 * @brief Check whether |d(k)| and |d(k) - d(k-1)| are consistently small over s samples.
 * @param trajectory Interpolated reference trajectory.
 * @param trajectory_bases Cached trajectory arc-length bases.
 * @param rear_left Rear-left corner of the object footprint.
 * @param rear_right Rear-right corner of the object footprint.
 * @param s_samples Arc-length samples to evaluate.
 * @return True if all magnitudes and consecutive deviations are below thresholds.
 */
bool matches_small_d_pattern(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::vector<double> & trajectory_bases, const geometry_msgs::msg::Point & rear_left,
  const geometry_msgs::msg::Point & rear_right, const std::vector<double> & s_samples)
{
  if (s_samples.size() < 2) {
    return false;
  }

  std::optional<double> previous_d_magnitude;
  for (const double s : s_samples) {
    const double d_magnitude =
      calc_rear_edge_d_magnitude_at_s(trajectory, trajectory_bases, s, rear_left, rear_right);
    if (d_magnitude >= OnTrajectoryDValidationParams::magnitude_threshold_m) {
      return false;
    }
    if (
      previous_d_magnitude && std::abs(d_magnitude - *previous_d_magnitude) >=
                                OnTrajectoryDValidationParams::deviation_threshold_m) {
      return false;
    }
    previous_d_magnitude = d_magnitude;
  }

  return true;
}

template <typename ObjectT>
std::vector<geometry_msgs::msg::Point> get_object_footprint_points(
  const ObjectT & object, const autoware_utils_geometry::Polygon2d & footprint)
{
  std::vector<geometry_msgs::msg::Point> footprint_points;
  footprint_points.reserve(footprint.outer().size());

  for (const auto & footprint_point : footprint.outer()) {
    footprint_points.push_back(
      autoware_utils_geometry::create_point(footprint_point.x(), footprint_point.y(), 0.0));
  }

  if (footprint_points.empty()) {
    footprint_points.push_back(get_object_reference_point(object));
  }

  return footprint_points;
}

geometry_msgs::msg::Point get_closest_point_on_polyline(
  const geometry_msgs::msg::Point & query, const std::vector<geometry_msgs::msg::Point> & polyline)
{
  if (polyline.empty()) {
    return query;
  }
  if (polyline.size() == 1) {
    return polyline.front();
  }

  geometry_msgs::msg::Point closest_point = polyline.front();
  double min_dist_sq = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    const auto & seg_start = polyline[i];
    const auto & seg_end = polyline[i + 1];

    const double dx = seg_end.x - seg_start.x;
    const double dy = seg_end.y - seg_start.y;
    const double len_sq = dx * dx + dy * dy;

    geometry_msgs::msg::Point projected_point = seg_start;
    if (len_sq > 1e-12) {
      const double t = std::clamp(
        ((query.x - seg_start.x) * dx + (query.y - seg_start.y) * dy) / len_sq, 0.0, 1.0);
      projected_point.x = seg_start.x + t * dx;
      projected_point.y = seg_start.y + t * dy;
      projected_point.z = seg_start.z + t * (seg_end.z - seg_start.z);
    }

    const double dist_sq = (query.x - projected_point.x) * (query.x - projected_point.x) +
                           (query.y - projected_point.y) * (query.y - projected_point.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_point = projected_point;
    }
  }

  return closest_point;
}

bool is_footprint_point_inside_drivable_area(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & footprint_point,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const double tolerance_m)
{
  const double s = closest_trajectory_s(trajectory, footprint_point);
  const auto ref_pose = trajectory.compute(s).pose;

  const double lateral_offset =
    autoware_utils_geometry::calc_lateral_deviation(ref_pose, footprint_point);
  const auto left_closest = get_closest_point_on_polyline(ref_pose.position, left_bound);
  const auto right_closest = get_closest_point_on_polyline(ref_pose.position, right_bound);
  const double left_bound_offset =
    autoware_utils_geometry::calc_lateral_deviation(ref_pose, left_closest);
  const double right_bound_offset =
    autoware_utils_geometry::calc_lateral_deviation(ref_pose, right_closest);

  const double left_limit = std::max(left_bound_offset, right_bound_offset);
  const double right_limit = std::min(left_bound_offset, right_bound_offset);

  return (right_limit - tolerance_m) <= lateral_offset &&
         lateral_offset <= (left_limit + tolerance_m);
}

std::vector<geometry_msgs::msg::Point> to_geometry_points(const lanelet::LineString2d & linestring)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(linestring.size());
  for (const auto & point : linestring) {
    geometry_msgs::msg::Point geometry_point;
    geometry_point.x = point.x();
    geometry_point.y = point.y();
    geometry_point.z = 0.0;
    points.push_back(geometry_point);
  }
  return points;
}

template <typename ObjectT>
bool is_object_beyond_trajectory_end(
  const FrameEvaluationContext<ObjectT> & context, const ObjectT & object)
{
  if (!context.built_trajectory) {
    return false;
  }

  const auto footprint = autoware_utils_geometry::to_polygon2d(object);
  const auto range = get_footprint_s_range(
    *context.built_trajectory, context.trajectory_bases, context.trajectory_base_points, object,
    footprint);
  return is_beyond_trajectory_end(*context.built_trajectory, range);
}

template <typename ObjectT>
bool should_filter_out_on_trajectory_object(
  const FrameEvaluationContext<ObjectT> & context, const ObjectT & object)
{
  if (!context.built_trajectory) {
    return false;
  }

  const auto & built_trajectory = *context.built_trajectory;
  const auto footprint = autoware_utils_geometry::to_polygon2d(object);
  const auto [rear_left, rear_right] = get_object_rear_edge_points(object, footprint);
  const auto range = get_footprint_s_range(
    built_trajectory, context.trajectory_bases, context.trajectory_base_points, object, footprint);

  if (is_beyond_trajectory_end(built_trajectory, range)) {
    return matches_small_d_pattern(
      built_trajectory, context.trajectory_bases, rear_left, rear_right,
      get_last_m_s_samples(context.trajectory_bases));
  }

  if (is_within_trajectory_s_range(built_trajectory, range)) {
    return matches_small_d_pattern(
      built_trajectory, context.trajectory_bases, rear_left, rear_right,
      get_s_samples_near_object(built_trajectory, context.trajectory_bases, range));
  }

  return false;
}

template <typename ObjectT>
bool should_filter_out_by_longitudinal_distance(
  const FrameEvaluationContext<ObjectT> & context, const ObjectT & object,
  const autoware_utils_geometry::Polygon2d & footprint,
  const LongitudinalDistanceFilterParams & params)
{
  if (!context.built_trajectory) {
    return false;
  }

  const auto & built_trajectory = *context.built_trajectory;
  const auto start_pose = built_trajectory.compute(0.0).pose;
  const auto end_pose = built_trajectory.compute(max_interpolator_safe_s(built_trajectory)).pose;
  const auto footprint_points = get_object_footprint_points(object, footprint);

  const bool all_before_start = std::all_of(
    footprint_points.begin(), footprint_points.end(),
    [&](const geometry_msgs::msg::Point & footprint_point) {
      return autoware_utils_geometry::calc_longitudinal_deviation(start_pose, footprint_point) <
             -params.tolerance_m;
    });

  const bool all_after_end = std::all_of(
    footprint_points.begin(), footprint_points.end(),
    [&](const geometry_msgs::msg::Point & footprint_point) {
      return autoware_utils_geometry::calc_longitudinal_deviation(end_pose, footprint_point) >
             params.tolerance_m;
    });

  return all_before_start || all_after_end;
}

template <typename ObjectT>
bool should_filter_out_by_lateral_distance(
  const RouteBounds & route_bounds, const FrameEvaluationContext<ObjectT> & context,
  const ObjectT & object, const autoware_utils_geometry::Polygon2d & footprint,
  const LateralDistanceFilterParams & params)
{
  const auto left_bound = to_geometry_points(route_bounds.first);
  const auto right_bound = to_geometry_points(route_bounds.second);
  if (left_bound.size() < 2 || right_bound.size() < 2) {
    return false;
  }

  if (!context.built_trajectory) {
    return false;
  }

  const auto footprint_points = get_object_footprint_points(object, footprint);
  return std::all_of(
    footprint_points.begin(), footprint_points.end(),
    [&](const geometry_msgs::msg::Point & footprint_point) {
      return !is_footprint_point_inside_drivable_area(
        *context.built_trajectory, footprint_point, left_bound, right_bound, params.tolerance_m);
    });
}

}  // namespace

PredictedObjects filter_objects_in_range(
  const PredictedObjects & objects, const Trajectory & trajectory, const double margin,
  const double additional_prediction_horizon)
{
  return filter_objects_in_range_impl(objects, trajectory, margin, additional_prediction_horizon);
}

TrackedObjects filter_objects_in_range(
  const TrackedObjects & objects, const Trajectory & trajectory, const double margin,
  const double additional_prediction_horizon)
{
  return filter_objects_in_range_impl(objects, trajectory, margin, additional_prediction_horizon);
}

template <typename ObjectT>
bool is_object_beyond_trajectory_end(const Trajectory & trajectory_msg, const ObjectT & object)
{
  const rclcpp::Time evaluation_time{0, 0, RCL_ROS_TIME};
  const auto context = make_frame_evaluation_context<ObjectT>(evaluation_time, trajectory_msg);
  return is_object_beyond_trajectory_end(context, object);
}

template <typename ObjectT>
bool should_filter_out_on_trajectory_object(
  const Trajectory & trajectory_msg, const ObjectT & object)
{
  const rclcpp::Time evaluation_time{0, 0, RCL_ROS_TIME};
  const auto context = make_frame_evaluation_context<ObjectT>(evaluation_time, trajectory_msg);
  return should_filter_out_on_trajectory_object(context, object);
}

template <typename ObjectT>
bool should_filter_out_by_longitudinal_distance(
  const Trajectory & trajectory_msg, const ObjectT & object,
  const LongitudinalDistanceFilterParams & params)
{
  const rclcpp::Time evaluation_time{0, 0, RCL_ROS_TIME};
  const auto context = make_frame_evaluation_context<ObjectT>(evaluation_time, trajectory_msg);
  const auto footprint = autoware_utils_geometry::to_polygon2d(object);
  return should_filter_out_by_longitudinal_distance(context, object, footprint, params);
}

template <typename ObjectT>
bool should_filter_out_by_lateral_distance(
  const RouteBounds & route_bounds, const Trajectory & trajectory_msg, const ObjectT & object,
  const LateralDistanceFilterParams & params)
{
  const rclcpp::Time evaluation_time{0, 0, RCL_ROS_TIME};
  const auto context = make_frame_evaluation_context<ObjectT>(evaluation_time, trajectory_msg);
  const auto footprint = autoware_utils_geometry::to_polygon2d(object);
  return should_filter_out_by_lateral_distance(route_bounds, context, object, footprint, params);
}

template <typename ObjectT>
TwoClassFilter<ObjectT>::TwoClassFilter(
  [[maybe_unused]] const ObjectT & object, const rclcpp::Time & last_update_time)
: prior_{k_neutral_likelihood},
  posterior_{k_neutral_likelihood},
  is_initialized_{false},
  last_update_time_{last_update_time}
{
}

template <typename ObjectT>
void TwoClassFilter<ObjectT>::observe_and_update(
  const rclcpp::Time & current_time, const ObjectT & object, const Trajectory & trajectory)
{
  const auto context = make_frame_evaluation_context<ObjectT>(current_time, trajectory);
  observe_and_update(context, object);
}

template <typename ObjectT>
void TwoClassFilter<ObjectT>::observe_and_update(
  const FrameEvaluationContext<ObjectT> & context, const ObjectT & object)
{
  calculate_likelihood(context, object);

  if (!is_initialized_) {
    posterior_ = target_likelihood_;
    prior_ = posterior_;
    target_likelihood_ = 1.0;
    non_target_likelihood_ = 1.0;
    is_initialized_ = true;
    last_update_time_ = context.current_time;
    return;
  }

  apply_transition_to_prior(context, object);
  apply_bayesian_update();
  last_update_time_ = context.current_time;

  target_likelihood_ = 1.0;
  non_target_likelihood_ = 1.0;
  prior_ = posterior_;
}

template <typename ObjectT>
void TwoClassFilter<ObjectT>::apply_transition_to_prior(
  const FrameEvaluationContext<ObjectT> & context, const ObjectT & object)
{
  const auto transition = transition_matrix(context, object);
  const double non_target_prior = 1.0 - prior_;
  prior_ = transition[0][0] * prior_ + transition[1][0] * non_target_prior;
}

template <typename ObjectT>
void TwoClassFilter<ObjectT>::apply_bayesian_update()
{
  const double numerator = prior_ * target_likelihood_;
  const double denominator = numerator + (1.0 - prior_) * non_target_likelihood_;
  posterior_ = denominator > 0.0 ? numerator / denominator : prior_;
}

/** TargetFilter implementation */

template <typename ObjectT>
void TargetFilter<ObjectT>::calculate_likelihood(
  [[maybe_unused]] const FrameEvaluationContext<ObjectT> & context, const ObjectT & object)
{
  this->target_likelihood_ = is_object_of_interest(object) ? k_high_likelihood : k_low_likelihood;
  this->non_target_likelihood_ = 1.0 - this->target_likelihood_;
}

template <typename ObjectT>
typename TwoClassFilter<ObjectT>::Matrix2x2 TargetFilter<ObjectT>::transition_matrix(
  [[maybe_unused]] const FrameEvaluationContext<ObjectT> & context,
  [[maybe_unused]] const ObjectT & object) const
{
  constexpr double state_persistence = 0.95;
  constexpr double switch_probability = 1.0 - state_persistence;
  return {{
    {state_persistence, switch_probability},
    {switch_probability, state_persistence},
  }};
}

/** StationaryFilter implementation */

template <typename ObjectT>
void StationaryFilter<ObjectT>::calculate_likelihood(
  [[maybe_unused]] const FrameEvaluationContext<ObjectT> & context, const ObjectT & object)
{
  const double pre_clamp_probability =
    -(linear_velocity_norm(object) - MovingObjectFilterParams::max_linear_velocity_mps) /
    (MovingObjectFilterParams::max_linear_velocity_mps -
     MovingObjectFilterParams::promising_stop_velocity_mps);
  this->target_likelihood_ = std::clamp(pre_clamp_probability, 0.1, 0.99);
  this->non_target_likelihood_ = 1.0 - this->target_likelihood_;
}

template <typename ObjectT>
typename TwoClassFilter<ObjectT>::Matrix2x2 StationaryFilter<ObjectT>::transition_matrix(
  [[maybe_unused]] const FrameEvaluationContext<ObjectT> & context, const ObjectT & object) const
{
  const double velocity_norm = linear_velocity_norm(object);

  if (velocity_norm < MovingObjectFilterParams::max_linear_velocity_mps) {
    return {{
      {0.9, 0.1},
      {0.5, 0.5},
    }};
  }

  return {{
    {0.05, 0.95},
    {0.95, 0.05},
  }};
}

/** DeviationFilter implementation */

template <typename ObjectT>
void DeviationFilter<ObjectT>::calculate_likelihood(
  const FrameEvaluationContext<ObjectT> & context, const ObjectT & object)
{
  this->target_likelihood_ =
    should_filter_out_on_trajectory_object(context, object) ? k_low_likelihood : k_high_likelihood;
  this->non_target_likelihood_ = 1.0 - this->target_likelihood_;
}

template <typename ObjectT>
typename TwoClassFilter<ObjectT>::Matrix2x2 DeviationFilter<ObjectT>::transition_matrix(
  const FrameEvaluationContext<ObjectT> & context, const ObjectT & object) const
{
  if (is_object_beyond_trajectory_end(context, object)) {
    return {{
      {0.8, 0.2},
      {0.4, 0.6},
    }};
  }

  return {{
    {0.95, 0.05},
    {0.2, 0.8},
  }};
}

template <typename ObjectT>
AvoidanceTargetDetectorBase<ObjectT>::AvoidanceTargetDetectorBase(
  const ObjectT & object, const rclcpp::Time & last_update_time)
: target_filter_{std::make_unique<TargetFilter<ObjectT>>(object, last_update_time)},
  stationary_filter_{std::make_unique<StationaryFilter<ObjectT>>(object, last_update_time)},
  deviation_filter_{std::make_unique<DeviationFilter<ObjectT>>(object, last_update_time)},
  is_stationary_avoidance_target_stamped_{last_update_time, false},
  is_moving_vehicle_stamped_{last_update_time, false},
  stale_check_time_{last_update_time}
{
}

template <typename ObjectT>
void AvoidanceTargetDetectorBase<ObjectT>::observe_and_update_all(
  const rclcpp::Time & current_time, const ObjectT & object, const Trajectory & trajectory,
  const lanelet::LaneletMapPtr & route_map,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph,
  const lanelet::BasicPolygon2d & near_segment_polygon,
  const std::vector<lanelet::ConstLanelet> & ego_lanelets)
{
  auto context = make_frame_evaluation_context<ObjectT>(current_time, trajectory);
  context.route_map = route_map.get();
  context.routing_graph = routing_graph.get();
  context.near_segment_polygon = build_near_segment_polygon(near_segment_polygon);
  context.ego_lanelets = ego_lanelets;
  observe_and_update_all(context, object);
}

template <typename ObjectT>
void AvoidanceTargetDetectorBase<ObjectT>::observe_and_update_all(
  const FrameEvaluationContext<ObjectT> & context, const ObjectT & object)
{
  stale_check_time_ = context.current_time;

  target_filter_->observe_and_update(context, object);
  stationary_filter_->observe_and_update(context, object);
  deviation_filter_->observe_and_update(context, object);

  is_driving_along_candidate_now_ = false;
  is_on_polygon_now_ = false;
  if (!is_object_of_interest() || is_stationary()) {
    return;
  }

  is_on_polygon_now_ = context.near_segment_polygon &&
                       overlaps_near_segment_polygon(object, *context.near_segment_polygon);
  if (!is_on_polygon_now_) {
    return;
  }

  if (!context.route_map) {
    return;
  }

  const auto object_lanelets =
    get_nearest_lanelets(*context.route_map, get_object_pose(object).position);
  if (object_lanelets.empty()) {
    return;
  }

  if (!context.routing_graph || context.ego_lanelets.empty()) {
    return;
  }

  const bool is_routably_connected = std::any_of(
    context.ego_lanelets.begin(), context.ego_lanelets.end(),
    [&](const lanelet::ConstLanelet & ego_lanelet) {
      return std::any_of(
        object_lanelets.begin(), object_lanelets.end(),
        [&](const lanelet::ConstLanelet & object_lanelet) {
          return is_routably_connected_to_ego_without_lane_change(
            context, ego_lanelet, object_lanelet);
        });
    });
  is_driving_along_candidate_now_ = !is_routably_connected;
}

template <typename ObjectT>
void AvoidanceTargetDetectorBase<ObjectT>::track_avoidance_targets()
{
  const auto & current_time = stale_check_time_;
  const bool is_target_now = is_object_of_interest() && is_stationary() && is_deviated();

  if (!is_avoidance_tracking_initialized_) {
    is_stationary_avoidance_target_stamped_.first = current_time;
    is_stationary_avoidance_target_stamped_.second = is_target_now;
    is_avoidance_tracking_initialized_ = true;
    return;
  }

  avoidance_state_change_count_ = (is_target_now != is_stationary_avoidance_target_stamped_.second)
                                    ? avoidance_state_change_count_ + 1
                                    : 0;

  if (
    rclcpp::Time(current_time) - rclcpp::Time(is_stationary_avoidance_target_stamped_.first) <
    rclcpp::Duration::from_seconds(FilterManagerParams::static_hysteresis_seconds)) {
    return;
  }

  if (avoidance_state_change_count_ < FilterManagerParams::count_threshold) {
    return;
  }

  is_stationary_avoidance_target_stamped_.first = current_time;
  is_stationary_avoidance_target_stamped_.second = is_target_now;
}

template <typename ObjectT>
void AvoidanceTargetDetectorBase<ObjectT>::track_driving_along_vehicles()
{
  const auto & current_time = stale_check_time_;
  const bool is_moving_vehicle_now = is_driving_along_candidate_now_;

  if (!is_moving_vehicle_tracking_initialized_) {
    is_moving_vehicle_stamped_.first = current_time;
    is_moving_vehicle_stamped_.second = is_moving_vehicle_now;
    is_moving_vehicle_tracking_initialized_ = true;
    return;
  }

  moving_vehicle_state_change_count_ = (is_moving_vehicle_now != is_moving_vehicle_stamped_.second)
                                         ? moving_vehicle_state_change_count_ + 1
                                         : 0;

  // Keep stamped moving-vehicle state when a previously tracked vehicle leaves the near-segment
  // polygon.
  if (is_moving_vehicle_stamped_.second && !is_on_polygon_now_) {
    is_moving_vehicle_stamped_.first = current_time;
    is_moving_vehicle_stamped_.second = true;
    return;
  }

  if (
    is_moving_vehicle_stamped_.second &&
    rclcpp::Time(current_time) - rclcpp::Time(is_moving_vehicle_stamped_.first) <
      rclcpp::Duration::from_seconds(FilterManagerParams::moving_hysteresis_seconds)) {
    return;
  }

  if (moving_vehicle_state_change_count_ < FilterManagerParams::count_threshold) {
    return;
  }

  is_moving_vehicle_stamped_.first = current_time;
  is_moving_vehicle_stamped_.second = is_moving_vehicle_now;
}

template <typename ObjectT>
void ObjectSelectorBase<ObjectT>::track_avoidance_targets()
{
  for (auto & [object_id, detector] : object_filters_) {
    (void)object_id;
    detector.track_avoidance_targets();
  }
}

template <typename ObjectT>
void ObjectSelectorBase<ObjectT>::track_driving_along_vehicles()
{
  for (auto & [object_id, detector] : object_filters_) {
    (void)object_id;
    detector.track_driving_along_vehicles();
  }
}

template <typename ObjectT>
void ObjectSelectorBase<ObjectT>::update_objects(
  const rclcpp::Time & current_time, const Objects & objects, const Trajectory & trajectory,
  const ExtendedRouteHandler & extended_route_handler)
{
  const auto route_map = extended_route_handler.getRouteMap();
  const auto routing_graph = extended_route_handler.getRouteMapRoutingGraph();
  auto context = make_frame_evaluation_context<ObjectT>(current_time, trajectory);
  context.route_map = route_map.get();
  context.routing_graph = routing_graph.get();

  if (!trajectory.points.empty()) {
    context.near_segment_polygon =
      build_near_segment_polygon(extended_route_handler.get_near_segment_polygon(
        trajectory.points.front().pose.position, trajectory.points.back().pose.position));
    if (route_map) {
      context.ego_lanelets =
        get_nearest_lanelets(*route_map, trajectory.points.back().pose.position);
      constexpr std::size_t k_max_object_lanelets = 5;
      context.routability_cache.reserve(context.ego_lanelets.size() * k_max_object_lanelets);
    }
  }

  for (const auto & object : objects.objects) {
    if (!is_object_of_interest(object)) {
      continue;
    }
    const auto it = object_filters_.try_emplace(object.object_id.uuid, object, current_time).first;
    it->second.observe_and_update_all(context, object);
  }

  for (auto it = object_filters_.begin(); it != object_filters_.end();) {
    if (it->second.is_stale(current_time)) {
      it = object_filters_.erase(it);
    } else {
      ++it;
    }
  }
}

template <typename ObjectT>
typename ObjectSelectorBase<ObjectT>::Objects ObjectSelectorBase<ObjectT>::get_avoidance_targets(
  const Objects & objects, const Trajectory & trajectory, const RouteBounds & route_bounds)
{
  track_avoidance_targets();

  Objects avoidance_targets = objects;
  avoidance_targets.objects.erase(
    std::remove_if(
      avoidance_targets.objects.begin(), avoidance_targets.objects.end(),
      [&](const ObjectT & object) {
        const auto it = object_filters_.find(object.object_id.uuid);
        return it == object_filters_.end() || !it->second.is_stationary_avoidance_target();
      }),
    avoidance_targets.objects.end());

  if (avoidance_targets.objects.empty()) {
    return avoidance_targets;
  }

  const rclcpp::Time evaluation_time{0, 0, RCL_ROS_TIME};
  const auto context = make_frame_evaluation_context<ObjectT>(evaluation_time, trajectory);
  avoidance_targets.objects.erase(
    std::remove_if(
      avoidance_targets.objects.begin(), avoidance_targets.objects.end(),
      [&](const ObjectT & object) {
        const auto footprint = autoware_utils_geometry::to_polygon2d(object);
        if (should_filter_out_by_longitudinal_distance(
              context, object, footprint, LongitudinalDistanceFilterParams{})) {
          return true;
        }
        if (should_filter_out_by_lateral_distance(
              route_bounds, context, object, footprint, LateralDistanceFilterParams{})) {
          return true;
        }
        return false;
      }),
    avoidance_targets.objects.end());

  return avoidance_targets;
}

template <typename ObjectT>
typename ObjectSelectorBase<ObjectT>::Objects
ObjectSelectorBase<ObjectT>::get_driving_along_vehicles(const Objects & objects)
{
  track_driving_along_vehicles();

  Objects driving_along_vehicles = objects;
  driving_along_vehicles.objects.erase(
    std::remove_if(
      driving_along_vehicles.objects.begin(), driving_along_vehicles.objects.end(),
      [&](const ObjectT & object) {
        const auto it = object_filters_.find(object.object_id.uuid);
        return it == object_filters_.end() || !it->second.is_moving_vehicle() ||
               it->second.is_stationary_avoidance_target();
      }),
    driving_along_vehicles.objects.end());

  return driving_along_vehicles;
}

// Explicit instantiations for the supported object message types.
template class TwoClassFilter<PredictedObject>;
template class TwoClassFilter<TrackedObject>;
template class TargetFilter<PredictedObject>;
template class TargetFilter<TrackedObject>;
template class StationaryFilter<PredictedObject>;
template class StationaryFilter<TrackedObject>;
template class DeviationFilter<PredictedObject>;
template class DeviationFilter<TrackedObject>;
template class AvoidanceTargetDetectorBase<PredictedObject>;
template class AvoidanceTargetDetectorBase<TrackedObject>;
template class ObjectSelectorBase<PredictedObject>;
template class ObjectSelectorBase<TrackedObject>;

template bool is_object_beyond_trajectory_end<PredictedObject>(
  const Trajectory &, const PredictedObject &);
template bool is_object_beyond_trajectory_end<TrackedObject>(
  const Trajectory &, const TrackedObject &);
template bool should_filter_out_on_trajectory_object<PredictedObject>(
  const Trajectory &, const PredictedObject &);
template bool should_filter_out_on_trajectory_object<TrackedObject>(
  const Trajectory &, const TrackedObject &);
template bool should_filter_out_by_longitudinal_distance<PredictedObject>(
  const Trajectory &, const PredictedObject &, const LongitudinalDistanceFilterParams &);
template bool should_filter_out_by_longitudinal_distance<TrackedObject>(
  const Trajectory &, const TrackedObject &, const LongitudinalDistanceFilterParams &);
template bool should_filter_out_by_lateral_distance<PredictedObject>(
  const RouteBounds &, const Trajectory &, const PredictedObject &,
  const LateralDistanceFilterParams &);
template bool should_filter_out_by_lateral_distance<TrackedObject>(
  const RouteBounds &, const Trajectory &, const TrackedObject &,
  const LateralDistanceFilterParams &);

}  // namespace autoware::avoidance_target_detector
