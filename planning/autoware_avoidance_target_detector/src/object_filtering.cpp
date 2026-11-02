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

#include <autoware/trajectory/interpolator/linear.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
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
#include <string>
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
      if (
        !std::isfinite(classification.probability) ||
        classification.probability <= probability_threshold || classification.probability > 1.0F) {
        return false;
      }
      return std::find(
               labels_of_interest.begin(), labels_of_interest.end(), classification.label) !=
             labels_of_interest.end();
    });
}

template <typename ObjectT>
bool has_valid_object_geometry(const ObjectT & object)
{
  const auto & pose = get_object_pose(object);
  const auto & q = pose.orientation;
  const auto & v = get_object_twist(object).linear;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (
    !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(norm) ||
    std::abs(norm - 1.0) > 1e-3 || !std::isfinite(v.x) || !std::isfinite(v.y) ||
    !std::isfinite(v.z))
    return false;
  const auto & shape = object.shape;
  if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    if (shape.footprint.points.size() < 3) return false;
    for (const auto & p : shape.footprint.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
    }
  } else if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    if (
      !std::isfinite(shape.dimensions.x) || shape.dimensions.x <= 0.0 ||
      !std::isfinite(shape.dimensions.y) || shape.dimensions.y <= 0.0)
      return false;
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    if (!std::isfinite(shape.dimensions.x) || shape.dimensions.x <= 0.0) return false;
  } else {
    return false;
  }
  return boost::geometry::is_valid(autoware_utils_geometry::to_polygon2d(object));
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
  if (!boost::geometry::is_valid(segment_polygon)) return std::nullopt;
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
    if (!has_valid_object_geometry(object)) {
      filtered.objects.push_back(object);  // Keep invalid objects visible to selector diagnostics.
      continue;
    }
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

  // All selector geometry is planar. Preserve the original messages for consumers.
  std::vector<TrajectoryPoint> points;
  points.reserve(trajectory_msg.points.size());
  for (const auto & original : trajectory_msg.points) {
    const auto & p = original.pose.position;
    const auto & q = original.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) || norm < 1e-6) {
      return std::nullopt;
    }
    auto point = original;
    point.pose.position.z = 0.0;
    const double yaw =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), norm * norm - 2.0 * (q.y * q.y + q.z * q.z));
    point.pose.orientation = geometry_msgs::msg::Quaternion{};
    point.pose.orientation.z = std::sin(yaw * 0.5);
    point.pose.orientation.w = std::cos(yaw * 0.5);
    if (
      !points.empty() &&
      std::hypot(p.x - points.back().pose.position.x, p.y - points.back().pose.position.y) < 1e-6) {
      continue;
    }
    points.push_back(point);
  }
  if (points.size() < 2) return std::nullopt;
  using aw_trajectory::interpolator::Linear;
  const auto trajectory = aw_trajectory::Trajectory<TrajectoryPoint>::Builder{}
                            .set_xy_interpolator<Linear>()
                            .set_z_interpolator<Linear>()
                            .build(points);
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
  context.prediction_horizon_s = trajectory_horizon_seconds(trajectory);
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
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::vector<double> & trajectory_bases, const double s)
{
  return std::clamp(s, 0.0, max_interpolator_safe_s(trajectory, trajectory_bases));
}

/** XY projection with a separate, unclamped endpoint coordinate.
 * Extrapolate only when the globally closest segment is a terminal segment. A point on a
 * returning part of the reference therefore cannot be rejected by the start tangent plane.
 */
struct PlanarProjection
{
  double s{0.0};
  double extended_s{0.0};
  double distance{std::numeric_limits<double>::infinity()};
};

PlanarProjection project_planar(
  const std::vector<double> & bases, const std::vector<geometry_msgs::msg::Point> & points,
  const geometry_msgs::msg::Point & point)
{
  PlanarProjection best;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    const double squared_length = dx * dx + dy * dy;
    if (squared_length < 1e-12) continue;
    const double ratio =
      ((point.x - points[i - 1].x) * dx + (point.y - points[i - 1].y) * dy) / squared_length;
    const double clamped = std::clamp(ratio, 0.0, 1.0);
    const double distance = std::hypot(
      point.x - points[i - 1].x - clamped * dx, point.y - points[i - 1].y - clamped * dy);
    if (distance >= best.distance) continue;
    const double ds = bases[i] - bases[i - 1];
    best = {bases[i - 1] + clamped * ds, bases[i - 1] + clamped * ds, distance};
    if ((i == 1 && ratio < 0.0) || (i + 1 == points.size() && ratio > 1.0)) {
      best.extended_s = bases[i - 1] + ratio * ds;
    }
  }
  return best;
}

struct FootprintSRange
{
  double min;
  double max;
};

template <typename ObjectT>
FootprintSRange get_footprint_s_range(
  [[maybe_unused]] const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const std::vector<double> & bases, const std::vector<geometry_msgs::msg::Point> & points,
  const ObjectT & object, const autoware_utils_geometry::Polygon2d & footprint)
{
  FootprintSRange range{std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
  const auto update = [&](const geometry_msgs::msg::Point & point) {
    const auto projection = project_planar(bases, points, point);
    range.min = std::min(range.min, projection.extended_s);
    range.max = std::max(range.max, projection.extended_s);
  };
  if (footprint.outer().empty()) update(get_object_reference_point(object));
  for (const auto & point : footprint.outer()) {
    update(autoware_utils_geometry::create_point(point.x(), point.y(), 0.0));
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
  return range.max >= -k_s_position_epsilon_m &&
         range.min <= trajectory.length() + k_s_position_epsilon_m;
}

std::vector<double> uniform_s_samples(const double start, const double end)
{
  std::vector<double> samples;
  if (end < start) return samples;
  for (double s = start; s < end; s += OnTrajectoryDValidationParams::s_sample_interval_m) {
    samples.push_back(s);
  }
  samples.push_back(end);
  return samples;
}

std::vector<double> get_last_m_s_samples(const std::vector<double> & bases)
{
  if (bases.empty()) return {};
  return uniform_s_samples(
    std::max(
      0.0, bases.back() - static_cast<double>(OnTrajectoryDValidationParams::sample_count_m)),
    bases.back());
}

std::vector<double> get_s_samples_near_object(
  const aw_trajectory::Trajectory<TrajectoryPoint> & trajectory, const std::vector<double> & bases,
  const FootprintSRange & range)
{
  const double end = max_interpolator_safe_s(trajectory, bases);
  return uniform_s_samples(
    std::clamp(range.min - OnTrajectoryDValidationParams::near_s_range_m, 0.0, end),
    std::clamp(range.max + OnTrajectoryDValidationParams::near_s_range_m, 0.0, end));
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
  return std::isfinite(dimensions.x) && std::isfinite(dimensions.y) &&
         dimensions.x > k_dimension_epsilon_m && dimensions.y > k_dimension_epsilon_m;
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
 * @details When the rear edge straddles the trajectory, |d| is 0 regardless of corner ordering.
 *          Otherwise |d| is taken from the rear corner closest to the trajectory.
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

  const auto [lo, hi] = std::minmax(d_rear_left, d_rear_right);
  if (lo <= 0.0 && hi >= 0.0) return 0.0;
  return std::min(std::abs(lo), std::abs(hi));
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
  double previous_s = 0.0;
  for (const double s : s_samples) {
    const double d_magnitude =
      calc_rear_edge_d_magnitude_at_s(trajectory, trajectory_bases, s, rear_left, rear_right);
    if (d_magnitude >= OnTrajectoryDValidationParams::magnitude_threshold_m) {
      return false;
    }
    if (
      previous_d_magnitude &&
      std::abs(d_magnitude - *previous_d_magnitude) >
        OnTrajectoryDValidationParams::deviation_threshold_m * (s - previous_s)) {
      return false;
    }
    previous_d_magnitude = d_magnitude;
    previous_s = s;
  }

  return true;
}

std::optional<autoware_utils_geometry::Polygon2d> make_route_polygon(const RouteBounds & bounds)
{
  if (bounds.first.size() < 2 || bounds.second.size() < 2) return std::nullopt;
  autoware_utils_geometry::Polygon2d polygon;
  for (const auto & p : bounds.first) polygon.outer().emplace_back(p.x(), p.y());
  for (const auto & p : bounds.second.invert()) {
    polygon.outer().emplace_back(p.x(), p.y());
  }
  boost::geometry::correct(polygon);
  if (!boost::geometry::is_valid(polygon)) return std::nullopt;
  return polygon;
}

template <typename ObjectT>
autoware_utils_geometry::LineString2d reference_polyline(
  const FrameEvaluationContext<ObjectT> & context)
{
  autoware_utils_geometry::LineString2d line;
  for (const auto & p : context.trajectory_base_points) line.emplace_back(p.x, p.y);
  return line;
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

  if (boost::geometry::intersects(reference_polyline(context), footprint)) return false;
  const auto range = get_footprint_s_range(
    *context.built_trajectory, context.trajectory_bases, context.trajectory_base_points, object,
    footprint);
  return range.max < -params.tolerance_m ||
         range.min > context.built_trajectory->length() + params.tolerance_m;
}

template <typename ObjectT>
bool should_filter_out_by_lateral_distance(
  const RouteBounds & route_bounds, const FrameEvaluationContext<ObjectT> & context,
  const ObjectT & object, const autoware_utils_geometry::Polygon2d & footprint,
  const LateralDistanceFilterParams & params)
{
  (void)context;
  (void)object;
  const auto polygon = make_route_polygon(route_bounds);
  if (!polygon) return false;  // evaluate() reports invalid nonempty bounds separately.
  return boost::geometry::disjoint(footprint, *polygon) &&
         boost::geometry::distance(footprint, *polygon) > params.tolerance_m;
}

/** Offset from the nearest traffic lane, independent of candidate trajectory points.
 * Associate by lane polygon distance before measuring centerline offset. Considering containing
 * lanes prevents adjacent-lane queues from looking parked, while excluding distant tangent
 * extensions prevents an unrelated crossing lane from hiding a curbside parked vehicle.
 */
std::optional<double> lane_center_offset(
  const lanelet::LaneletMap * map, const geometry_msgs::msg::Point & position)
{
  if (!map) return std::nullopt;
  const lanelet::BasicPoint2d search_point{position.x, position.y};
  constexpr double association_tolerance_m = 1e-3;
  double nearest_lane_distance = std::numeric_limits<double>::infinity();
  std::optional<double> offset;
  for (const auto & lane : map->laneletLayer.nearest(search_point, 10)) {
    if (lane.attributeOr("subtype", std::string{"road"}) != "road") continue;
    const double lane_distance = lanelet::geometry::distance2d(lane, search_point);
    if (lane_distance > nearest_lane_distance + association_tolerance_m) continue;
    const auto center = lane.centerline2d();
    double closest = std::numeric_limits<double>::infinity();
    double lateral = 0.0;
    for (std::size_t i = 1; i < center.size(); ++i) {
      const double dx = center[i].x() - center[i - 1].x();
      const double dy = center[i].y() - center[i - 1].y();
      const double length = std::hypot(dx, dy);
      if (length < 1e-6) continue;
      const double wx = position.x - center[i - 1].x();
      const double wy = position.y - center[i - 1].y();
      const double ratio = std::clamp((wx * dx + wy * dy) / (length * length), 0.0, 1.0);
      const double distance = std::hypot(wx - ratio * dx, wy - ratio * dy);
      if (distance < closest) {
        closest = distance;
        lateral = std::abs(wx * dy - wy * dx) / length;
      }
    }
    // Limit lane association to the local scene, including curbside objects outside lane polygons.
    if (closest > 10.0) continue;
    if (lane_distance < nearest_lane_distance - association_tolerance_m) offset.reset();
    nearest_lane_distance = std::min(nearest_lane_distance, lane_distance);
    if (!offset || lateral < *offset) offset = lateral;
  }
  return offset;
}

void update_temporal_state(
  const rclcpp::Time & time, const bool candidate, const double dwell_seconds,
  std::pair<rclcpp::Time, bool> & state, bool & initialized,
  std::optional<rclcpp::Time> & pending_since, std::size_t & count)
{
  if (!initialized) {
    state = {time, candidate};
    initialized = true;
    return;
  }
  if (candidate == state.second) {
    pending_since.reset();
    count = 0;
    return;
  }
  if (!pending_since) pending_since = time;
  count = std::min(count + 1, static_cast<std::size_t>(FilterManagerParams::count_threshold));
  if (
    count >= FilterManagerParams::count_threshold &&
    (time - *pending_since).seconds() >= dwell_seconds) {
    state = {time, candidate};
    pending_since.reset();
    count = 0;
  }
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
  if (is_initialized_ && context.current_time <= last_update_time_) return;
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
    !context.built_trajectory
      ? k_neutral_likelihood
      : (should_filter_out_on_trajectory_object(context, object) ? k_low_likelihood
                                                                 : k_high_likelihood);
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
  if (target_filter_->is_initialized() && context.current_time <= stale_check_time_) return;
  stale_check_time_ = context.current_time;
  target_filter_->observe_and_update(context, object);
  stationary_filter_->observe_and_update(context, object);
  deviation_filter_->observe_and_update(context, object);
  if (linear_velocity_norm(object) >= MovingObjectFilterParams::max_linear_velocity_mps) {
    last_moving_observation_ = context.current_time;
  }
  evaluate_geometry(context, object);
  track_avoidance_targets();
  track_driving_along_vehicles();
}

template <typename ObjectT>
void AvoidanceTargetDetectorBase<ObjectT>::evaluate_geometry(
  const FrameEvaluationContext<ObjectT> & context, const ObjectT & object)
{
  lane_center_offset_m_ = lane_center_offset(context.route_map, get_object_pose(object).position);
  const bool recently_moving =
    last_moving_observation_ && (context.current_time - *last_moving_observation_).seconds() <
                                  ParkedObjectPolicyParams::recently_moving_seconds;
  parked_candidate_now_ =
    lane_center_offset_m_ &&
    *lane_center_offset_m_ >= ParkedObjectPolicyParams::minimum_lane_center_offset_m &&
    !recently_moving;

  is_driving_along_candidate_now_ = false;
  is_on_polygon_now_ = context.near_segment_polygon &&
                       overlaps_near_segment_polygon(object, *context.near_segment_polygon);
  if (
    !is_object_of_interest() || is_stationary() || !is_on_polygon_now_ || !context.route_map ||
    !context.routing_graph || context.ego_lanelets.empty())
    return;
  const auto object_lanelets =
    get_nearest_lanelets(*context.route_map, get_object_pose(object).position);
  if (object_lanelets.empty()) return;
  const bool connected = std::any_of(
    context.ego_lanelets.begin(), context.ego_lanelets.end(), [&](const auto & ego_lanelet) {
      return std::any_of(object_lanelets.begin(), object_lanelets.end(), [&](const auto & lanelet) {
        return is_routably_connected_to_ego_without_lane_change(context, ego_lanelet, lanelet);
      });
    });
  is_driving_along_candidate_now_ = !connected;
}

template <typename ObjectT>
void AvoidanceTargetDetectorBase<ObjectT>::track_avoidance_targets()
{
  update_temporal_state(
    stale_check_time_, is_object_of_interest() && is_stationary() && parked_candidate_now_,
    FilterManagerParams::static_hysteresis_seconds, is_stationary_avoidance_target_stamped_,
    is_avoidance_tracking_initialized_, avoidance_pending_since_, avoidance_state_change_count_);
}

template <typename ObjectT>
void AvoidanceTargetDetectorBase<ObjectT>::track_driving_along_vehicles()
{
  if (is_stationary() || !is_object_of_interest()) {
    is_moving_vehicle_stamped_ = {stale_check_time_, false};
    is_moving_vehicle_tracking_initialized_ = true;
    moving_pending_since_.reset();
    moving_vehicle_state_change_count_ = 0;
    return;
  }
  update_temporal_state(
    stale_check_time_, is_driving_along_candidate_now_,
    is_moving_vehicle_stamped_.second ? FilterManagerParams::moving_hysteresis_seconds
                                      : FilterManagerParams::static_hysteresis_seconds,
    is_moving_vehicle_stamped_, is_moving_vehicle_tracking_initialized_, moving_pending_since_,
    moving_vehicle_state_change_count_);
}

template <typename ObjectT>
void ObjectSelectorBase<ObjectT>::update_objects(
  const rclcpp::Time & current_time, const Objects & objects, const Trajectory & trajectory,
  const ExtendedRouteHandler & extended_route_handler)
{
  if (last_evaluation_time_ && current_time < *last_evaluation_time_) {
    object_filters_.clear();
    last_measurement_time_.reset();
  }
  last_evaluation_time_ = current_time;
  measurement_header_ = objects.header;
  const rclcpp::Time measurement_time{objects.header.stamp, current_time.get_clock_type()};
  measurement_age_s_ = (current_time - measurement_time).seconds();
  const bool new_measurement =
    !last_measurement_time_ || measurement_time > *last_measurement_time_;

  // Prune before observing: a reappearing UUID must not revive expired history.
  for (auto it = object_filters_.begin(); it != object_filters_.end();) {
    if (it->second.is_stale(current_time))
      it = object_filters_.erase(it);
    else
      ++it;
  }
  if (measurement_age_s_ > FilterManagerParams::stale_threshold_seconds) {
    status_ = SelectionStatus::stale_objects;
    context_.reset();
    return;
  }
  if (measurement_age_s_ < -FilterManagerParams::future_stamp_tolerance_seconds) {
    status_ = SelectionStatus::future_objects;
    context_.reset();
    return;
  }
  if (last_measurement_time_ && measurement_time < *last_measurement_time_) {
    status_ = SelectionStatus::out_of_order_objects;
    context_.reset();
    return;
  }
  if (objects.header.frame_id.empty() || objects.header.frame_id != trajectory.header.frame_id) {
    status_ = SelectionStatus::frame_mismatch;
    context_.reset();
    return;
  }

  const auto route_map = extended_route_handler.getRouteMap();
  const auto routing_graph = extended_route_handler.getRouteMapRoutingGraph();
  auto context = make_frame_evaluation_context<ObjectT>(measurement_time, trajectory);
  context.route_map = route_map.get();
  context.routing_graph = routing_graph.get();
  if (context.built_trajectory) {
    context.near_segment_polygon =
      build_near_segment_polygon(extended_route_handler.get_near_segment_polygon(
        trajectory.points.front().pose.position, trajectory.points.back().pose.position));
    if (route_map) {
      context.ego_lanelets =
        get_nearest_lanelets(*route_map, trajectory.points.front().pose.position);
    }
  }
  for (const auto & object : objects.objects) {
    if (!has_valid_object_geometry(object)) continue;
    auto it = object_filters_.find(object.object_id.uuid);
    if (new_measurement) {
      // Existing tracks receive negative classification evidence as well.
      if (it == object_filters_.end() && !is_object_of_interest(object)) continue;
      if (it == object_filters_.end()) {
        it = object_filters_.try_emplace(object.object_id.uuid, object, measurement_time).first;
      }
      it->second.observe_and_update_all(context, object);
    } else if (it != object_filters_.end()) {
      it->second.evaluate_geometry(context, object);
    }
  }
  if (new_measurement) last_measurement_time_ = measurement_time;
  status_ = context.built_trajectory ? SelectionStatus::ready : SelectionStatus::invalid_reference;
  context.current_time = current_time;
  context_.emplace(std::move(context));
}

template <typename ObjectT>
ObjectSelectionResult<typename ObjectSelectorBase<ObjectT>::Objects>
ObjectSelectorBase<ObjectT>::evaluate(
  const Objects & objects, const RouteBounds & route_bounds,
  const SelectionGeometryParams & geometry) const
{
  ObjectSelectionResult<Objects> result;
  result.status = status_;
  result.avoidance_targets.header = objects.header;
  result.driving_along_vehicles.header = objects.header;
  result.yield_targets.header = objects.header;
  if (objects.header != measurement_header_) result.status = SelectionStatus::out_of_order_objects;
  if (
    geometry.trajectory_envelope_radius_m &&
    (!std::isfinite(*geometry.trajectory_envelope_radius_m) ||
     *geometry.trajectory_envelope_radius_m < 0.0)) {
    throw std::invalid_argument("Selection envelope radius must be finite and nonnegative");
  }
  if (!std::isfinite(geometry.prediction_extension_s) || geometry.prediction_extension_s < 0.0) {
    throw std::invalid_argument("Selection prediction extension must be finite and nonnegative");
  }
  const auto reference_line =
    context_ ? reference_polyline(*context_) : autoware_utils_geometry::LineString2d{};
  const auto route_polygon = geometry.trajectory_envelope_radius_m
                               ? std::optional<autoware_utils_geometry::Polygon2d>{}
                               : make_route_polygon(route_bounds);
  if (
    result.status == SelectionStatus::ready && !geometry.trajectory_envelope_radius_m &&
    (!route_bounds.first.empty() || !route_bounds.second.empty()) && !route_polygon) {
    result.status = SelectionStatus::invalid_route_bounds;
  }
  for (const auto & object : objects.objects) {
    ObjectSelectionDecision decision;
    decision.object_id = object.object_id.uuid;
    decision.measurement_age_s = measurement_age_s_;
    if (result.status != SelectionStatus::ready || !context_) {
      decision.reason = to_string(result.status);
      result.decisions.push_back(std::move(decision));
      continue;
    }
    if (!has_valid_object_geometry(object)) {
      decision.reason = "invalid_object_geometry";
      result.decisions.push_back(std::move(decision));
      continue;
    }
    const auto it = object_filters_.find(object.object_id.uuid);
    if (it == object_filters_.end()) {
      decision.disposition = ObjectDisposition::ignored;
      decision.reason = "no_eligible_track";
      result.decisions.push_back(std::move(decision));
      continue;
    }
    const auto & detector = it->second;
    decision.interest_probability = detector.get_is_target_probability();
    decision.stationary_probability = detector.get_is_stationary_probability();
    decision.deviation_probability = detector.get_is_deviated_probability();
    decision.lane_center_offset_m = detector.lane_center_offset_m_;
    const auto footprint = autoware_utils_geometry::to_polygon2d(object);
    decision.trajectory_distance_m = boost::geometry::distance(reference_line, footprint);

    if (!is_object_of_interest(object) || !detector.is_object_of_interest()) {
      decision.disposition = ObjectDisposition::ignored;
      decision.reason = "classification_not_eligible";
    } else if (
      detector.is_stationary() &&
      linear_velocity_norm(object) < MovingObjectFilterParams::max_linear_velocity_mps) {
      if (!detector.lane_center_offset_m_) {
        decision.reason = "lane_context_unavailable";
      } else if (!detector.parked_candidate_now_) {
        decision.disposition = ObjectDisposition::yield;
        decision.reason = "traffic_lane_or_recent_motion";
      } else if (!detector.is_stationary_avoidance_target()) {
        decision.disposition = ObjectDisposition::yield;
        decision.reason = "awaiting_stationary_avoidance_confirmation";
      } else {
        decision.disposition = ObjectDisposition::avoidance;
        decision.reason = "parked_off_lane_center";
      }
    } else if (detector.is_moving_vehicle() && !detector.is_stationary()) {
      decision.disposition = ObjectDisposition::driving_along;
      decision.reason = "moving_disconnected_lane";
    } else {
      decision.disposition = ObjectDisposition::yield;
      decision.reason = "moving_or_uncertain_vehicle";
    }

    if (decision.disposition == ObjectDisposition::avoidance) {
      if (geometry.trajectory_envelope_radius_m) {
        double envelope_distance = *decision.trajectory_distance_m;
        const auto & pose = get_object_pose(object);
        const auto & q = pose.orientation;
        const double yaw =
          std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        const double travel = get_object_twist(object).linear.x *
                              (context_->prediction_horizon_s + geometry.prediction_extension_s);
        if (std::abs(travel) > k_motion_epsilon) {
          const autoware_utils_geometry::Segment2d motion{
            {pose.position.x, pose.position.y},
            {pose.position.x + std::cos(yaw) * travel, pose.position.y + std::sin(yaw) * travel}};
          envelope_distance = std::max(
            0.0,
            boost::geometry::distance(reference_line, motion) - circular_footprint_radius(object));
        }
        const bool predicted_path_reaches =
          std::find(
            geometry.predicted_path_target_ids.begin(), geometry.predicted_path_target_ids.end(),
            object.object_id.uuid) != geometry.predicted_path_target_ids.end();
        if (!predicted_path_reaches && envelope_distance > *geometry.trajectory_envelope_radius_m) {
          decision.disposition = ObjectDisposition::ignored;
          decision.reason = "outside_trajectory_envelope";
        }
      } else if (should_filter_out_by_longitudinal_distance(
                   *context_, object, footprint, LongitudinalDistanceFilterParams{})) {
        decision.disposition = ObjectDisposition::ignored;
        decision.reason = "outside_longitudinal_extent";
      } else if (route_polygon && boost::geometry::disjoint(footprint, *route_polygon)) {
        decision.disposition = ObjectDisposition::ignored;
        decision.reason = "outside_route_corridor";
      }
    }
    if (decision.disposition == ObjectDisposition::avoidance) {
      result.avoidance_targets.objects.push_back(object);
    } else if (decision.disposition == ObjectDisposition::driving_along) {
      result.driving_along_vehicles.objects.push_back(object);
    } else if (decision.disposition == ObjectDisposition::yield) {
      result.yield_targets.objects.push_back(object);
    }
    result.decisions.push_back(std::move(decision));
  }
  if (
    result.status == SelectionStatus::ready &&
    std::any_of(result.decisions.begin(), result.decisions.end(), [](const auto & decision) {
      return decision.reason == "invalid_object_geometry";
    })) {
    result.status = SelectionStatus::invalid_objects;
    result.avoidance_targets.objects.clear();
    result.driving_along_vehicles.objects.clear();
    result.yield_targets.objects.clear();
    for (auto & decision : result.decisions) {
      if (decision.reason != "invalid_object_geometry") {
        decision.reason = "selection_unavailable_invalid_objects";
        decision.disposition = ObjectDisposition::unknown;
      }
    }
  }
  return result;
}

template <typename ObjectT>
typename ObjectSelectorBase<ObjectT>::Objects ObjectSelectorBase<ObjectT>::get_avoidance_targets(
  const Objects & objects, [[maybe_unused]] const Trajectory & trajectory,
  const RouteBounds & route_bounds) const
{
  return evaluate(objects, route_bounds).avoidance_targets;
}

template <typename ObjectT>
typename ObjectSelectorBase<ObjectT>::Objects
ObjectSelectorBase<ObjectT>::get_driving_along_vehicles(const Objects & objects) const
{
  return evaluate(objects, {}).driving_along_vehicles;
}

const char * to_string(const SelectionStatus status)
{
  switch (status) {
    case SelectionStatus::ready:
      return "ready";
    case SelectionStatus::stale_objects:
      return "stale_objects";
    case SelectionStatus::future_objects:
      return "future_objects";
    case SelectionStatus::out_of_order_objects:
      return "out_of_order_objects";
    case SelectionStatus::invalid_reference:
      return "invalid_reference";
    case SelectionStatus::frame_mismatch:
      return "frame_mismatch";
    case SelectionStatus::invalid_route_bounds:
      return "invalid_route_bounds";
    case SelectionStatus::invalid_objects:
      return "invalid_objects";
  }
  return "unknown";
}

const char * to_string(const ObjectDisposition disposition)
{
  switch (disposition) {
    case ObjectDisposition::ignored:
      return "ignored";
    case ObjectDisposition::avoidance:
      return "avoidance";
    case ObjectDisposition::yield:
      return "yield";
    case ObjectDisposition::driving_along:
      return "driving_along";
    case ObjectDisposition::unknown:
      return "unknown";
  }
  return "unknown";
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
