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

#include "autoware/trajectory_modifier/time_sequence_raw/road_border_avoidance.hpp"

#include <Eigen/Core>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::time_sequence_raw
{
namespace bg = boost::geometry;
using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::Point2d;

namespace
{
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

LinearRing2d place_footprint(
  const LinearRing2d & base_footprint, const double x, const double y, const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  LinearRing2d placed;
  placed.reserve(base_footprint.size());
  for (const auto & p : base_footprint) {
    placed.emplace_back(c * p.x() - s * p.y() + x, s * p.x() + c * p.y() + y);
  }
  return placed;
}

Eigen::Vector2d point_xy(const geometry_msgs::msg::Point & point)
{
  return {point.x, point.y};
}

/// Polyline tangent. Falls back to the pose yaw when the segment length vanishes.
Eigen::Vector2d path_tangent(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const size_t index,
  const Eigen::Vector2d & yaw_heading)
{
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  if (index + 1 < points.size()) {
    direction += point_xy(points[index + 1].pose.position) - point_xy(points[index].pose.position);
  }
  if (index > 0) {
    direction += point_xy(points[index].pose.position) - point_xy(points[index - 1].pose.position);
  }
  if (direction.norm() < 1e-4) {
    return yaw_heading;
  }
  return direction.normalized();
}

/// Signed curvature [1/m], positive when the path turns left.
double path_curvature(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const size_t index)
{
  if (index == 0 || index + 1 >= points.size()) {
    return 0.0;
  }
  const Eigen::Vector2d incoming =
    point_xy(points[index].pose.position) - point_xy(points[index - 1].pose.position);
  const Eigen::Vector2d outgoing =
    point_xy(points[index + 1].pose.position) - point_xy(points[index].pose.position);
  const double incoming_length = incoming.norm();
  const double outgoing_length = outgoing.norm();
  if (incoming_length < 1e-4 || outgoing_length < 1e-4) {
    return 0.0;
  }
  const double turning =
    std::atan2(incoming.x() * outgoing.y() - incoming.y() * outgoing.x(), incoming.dot(outgoing));
  return turning / (0.5 * (incoming_length + outgoing_length));
}

double distance_to_nearest_border(
  const std::vector<const LineString2d *> & borders, const Point2d & position)
{
  double min_distance = std::numeric_limits<double>::max();
  for (const LineString2d * border : borders) {
    min_distance = std::min(min_distance, bg::distance(position, *border));
  }
  return min_distance;
}

constexpr double k_boundary_eps_m = 1e-3;
constexpr double k_inward_curvature_margin = 0.05;
constexpr double k_segment_heading_rad = 0.2;

struct OffsetProjection
{
  double offset_m{0.0};
  bool resolved{false};
};

/// Project `preferred` onto the feasible path-normal offsets inside ±max_shift.
/// Feasible means the footprint (and, on a bend, the segment midpoint) is clear, and the
/// inward offset has not collapsed the local radius. The result is the feasible value
/// closest to `preferred`. An empty set keeps the in-range offset farthest from the borders.
template <typename CollidingFn, typename DistanceFn>
OffsetProjection project_offset(
  const double preferred, const double step, const double max_shift, const double curvature,
  const CollidingFn & colliding, const DistanceFn & border_distance)
{
  const auto in_range = [max_shift](const double offset) {
    return std::abs(offset) <= max_shift + 1e-9;
  };
  const auto radius_ok = [curvature](const double offset) {
    return !(std::abs(curvature) > 1e-6 && offset * curvature >= 1.0 - k_inward_curvature_margin);
  };
  const auto feasible = [&](const double offset) {
    return in_range(offset) && radius_ok(offset) && !colliding(offset);
  };

  if (feasible(preferred)) {
    return {preferred, true};
  }

  std::vector<double> samples;
  samples.reserve(static_cast<size_t>(2.0 * max_shift / std::max(step, 1e-3)) + 3U);
  for (double offset = -max_shift; offset <= max_shift + 1e-9; offset += step) {
    samples.push_back(offset);
  }
  if (samples.empty() || samples.back() < max_shift - 1e-9) {
    samples.push_back(max_shift);
  }

  std::optional<double> nearest_feasible;
  for (const double offset : samples) {
    if (!feasible(offset)) {
      continue;
    }
    if (
      !nearest_feasible ||
      std::abs(offset - preferred) < std::abs(*nearest_feasible - preferred) - 1e-12) {
      nearest_feasible = offset;
    }
  }

  if (!nearest_feasible) {
    double best = std::clamp(preferred, -max_shift, max_shift);
    double best_distance = border_distance(best);
    for (const double offset : samples) {
      if (!in_range(offset) || !radius_ok(offset)) {
        continue;
      }
      const double distance = border_distance(offset);
      const bool farther = distance > best_distance + 1e-9;
      const bool same_distance_closer = std::abs(distance - best_distance) <= 1e-9 &&
                                        std::abs(offset - preferred) < std::abs(best - preferred);
      if (farther || same_distance_closer) {
        best = offset;
        best_distance = distance;
      }
    }
    return {best, false};
  }

  double colliding_side = std::clamp(preferred, -max_shift, max_shift);
  double clear_side = *nearest_feasible;
  if (feasible(colliding_side)) {
    return {colliding_side, true};
  }
  for (int iteration = 0; iteration < 24; ++iteration) {
    if (std::abs(clear_side - colliding_side) <= k_boundary_eps_m) {
      break;
    }
    const double mid = 0.5 * (colliding_side + clear_side);
    if (feasible(mid)) {
      clear_side = mid;
    } else {
      colliding_side = mid;
    }
  }
  return {clear_side, true};
}
}  // namespace

RoadBorderAvoidance::RoadBorderAvoidance(
  const RoadBorderAvoidanceParams & params,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
: params_(params), base_footprint_(vehicle_info.createFootprint(params.footprint_margin_m))
{
}

void RoadBorderAvoidance::set_map(const lanelet::LaneletMap & lanelet_map)
{
  std::vector<LineString2d> road_borders;
  for (const auto & line_string : lanelet_map.lineStringLayer) {
    const std::string line_string_type = line_string.attributeOr("type", "");
    if (line_string_type != "road_border" || line_string.size() < 2) {
      continue;
    }
    LineString2d border;
    border.reserve(line_string.size());
    for (const auto & point : line_string) {
      border.emplace_back(point.x(), point.y());
    }
    road_borders.push_back(std::move(border));
  }
  set_road_borders(std::move(road_borders));
}

void RoadBorderAvoidance::set_road_borders(std::vector<LineString2d> road_borders)
{
  road_borders_ = std::move(road_borders);
}

RoadBorderAvoidanceResult RoadBorderAvoidance::adjust(
  const Trajectory & raw_trajectory, const geometry_msgs::msg::Pose & ego_pose) const
{
  RoadBorderAvoidanceResult result;
  result.trajectory = raw_trajectory;
  if (road_borders_.empty() || raw_trajectory.points.empty()) {
    return result;
  }

  const Point2d ego_point(ego_pose.position.x, ego_pose.position.y);
  std::vector<const LineString2d *> nearby_borders;
  for (const auto & border : road_borders_) {
    if (bg::distance(ego_point, border) <= params_.search_radius_m) {
      nearby_borders.push_back(&border);
    }
  }
  if (nearby_borders.empty()) {
    return result;
  }

  const auto intersects_any = [&nearby_borders](const LinearRing2d & footprint) {
    return std::any_of(
      nearby_borders.begin(), nearby_borders.end(),
      [&footprint](const LineString2d * border) { return bg::intersects(footprint, *border); });
  };

  // Tangents come from the raw polyline. Later points must not see already-shifted neighbors.
  const auto raw_points = result.trajectory.points;
  std::vector<Eigen::Vector2d> tangents(raw_points.size());
  std::vector<double> curvatures(raw_points.size(), 0.0);
  for (size_t index = 0; index < raw_points.size(); ++index) {
    const double yaw = yaw_from_quaternion(raw_points[index].pose.orientation);
    tangents[index] =
      path_tangent(raw_points, index, Eigen::Vector2d(std::cos(yaw), std::sin(yaw)));
    curvatures[index] = path_curvature(raw_points, index);
  }

  // Signed offset along the raw-path left normal. Positive is to the left of the polyline.
  double carried_offset_m = 0.0;

  for (size_t index = 0; index < raw_points.size(); ++index) {
    auto & point = result.trajectory.points[index];
    const double raw_x = raw_points[index].pose.position.x;
    const double raw_y = raw_points[index].pose.position.y;
    const double yaw = yaw_from_quaternion(raw_points[index].pose.orientation);
    const Eigen::Vector2d & tangent = tangents[index];
    const Eigen::Vector2d normal_left(-tangent.y(), tangent.x());
    const double curvature = curvatures[index];

    const double preferred = params_.propagate_shift ? carried_offset_m : 0.0;
    bool check_segment = false;
    Eigen::Vector2d next_xy(raw_x, raw_y);
    double next_yaw = yaw;
    if (index + 1 < raw_points.size()) {
      const Eigen::Vector2d & next_tangent = tangents[index + 1];
      const double heading_change = std::atan2(
        tangent.x() * next_tangent.y() - tangent.y() * next_tangent.x(), tangent.dot(next_tangent));
      check_segment = std::abs(heading_change) >= k_segment_heading_rad;
      next_xy = point_xy(raw_points[index + 1].pose.position);
      next_yaw = yaw_from_quaternion(raw_points[index + 1].pose.orientation);
    }

    const auto footprint_at = [&](const double offset) {
      return place_footprint(
        base_footprint_, raw_x + normal_left.x() * offset, raw_y + normal_left.y() * offset, yaw);
    };
    const auto colliding = [&](const double offset) {
      if (intersects_any(footprint_at(offset))) {
        return true;
      }
      if (!check_segment) {
        return false;
      }
      const double mid_x = 0.5 * (raw_x + next_xy.x()) + normal_left.x() * offset;
      const double mid_y = 0.5 * (raw_y + next_xy.y()) + normal_left.y() * offset;
      const double mid_yaw =
        std::atan2(std::sin(yaw) + std::sin(next_yaw), std::cos(yaw) + std::cos(next_yaw));
      return intersects_any(place_footprint(base_footprint_, mid_x, mid_y, mid_yaw));
    };
    const auto border_distance = [&](const double offset) {
      return distance_to_nearest_border(
        nearby_borders,
        Point2d(raw_x + normal_left.x() * offset, raw_y + normal_left.y() * offset));
    };

    const OffsetProjection projection = project_offset(
      preferred, params_.shift_step_m, params_.max_lateral_shift_m, curvature, colliding,
      border_distance);

    if (std::abs(projection.offset_m) > 1e-12) {
      point.pose.position.x = raw_x + normal_left.x() * projection.offset_m;
      point.pose.position.y = raw_y + normal_left.y() * projection.offset_m;
    }
    carried_offset_m = projection.offset_m;
    if (std::abs(projection.offset_m) <= 1e-12) {
      continue;
    }
    if (projection.resolved) {
      ++result.num_shifted_points;
    } else {
      ++result.num_unresolved_points;
    }
  }

  return result;
}

}  // namespace autoware::trajectory_modifier::time_sequence_raw
