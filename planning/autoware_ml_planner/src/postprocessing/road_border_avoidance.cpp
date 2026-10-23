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

#include "autoware/ml_planner/postprocessing/road_border_avoidance.hpp"

#include <Eigen/Core>
#include <rclcpp/duration.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ml_planner::postprocess
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

struct NearestPoint
{
  Point2d point;
  double z{0.0};
  double sq_dist{std::numeric_limits<double>::max()};
};

NearestPoint nearest_point_on_border(const RoadBorder & border, const Point2d & point)
{
  const LineString2d & line = border.line;
  NearestPoint nearest{line.front(), border.z.front()};
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    const Eigen::Vector2d & a = line[i];
    const Eigen::Vector2d & b = line[i + 1];
    const Eigen::Vector2d ab = b - a;
    const double ab_sq = ab.squaredNorm();
    const double t = (ab_sq > 0.0) ? std::clamp((point - a).dot(ab) / ab_sq, 0.0, 1.0) : 0.0;
    const Eigen::Vector2d candidate = a + t * ab;
    const double sq_dist = (point - candidate).squaredNorm();
    if (sq_dist < nearest.sq_dist) {
      nearest.point = Point2d(candidate.x(), candidate.y());
      nearest.z = border.z[i] + t * (border.z[i + 1] - border.z[i]);
      nearest.sq_dist = sq_dist;
    }
  }
  return nearest;
}

const RoadBorder * find_nearest_overlapping_border(
  const std::vector<const RoadBorder *> & borders, const LinearRing2d & footprint,
  const Point2d & position)
{
  const RoadBorder * nearest_border = nullptr;
  double min_sq_dist = std::numeric_limits<double>::max();
  for (const RoadBorder * border : borders) {
    if (!bg::intersects(footprint, border->line)) {
      continue;
    }
    const double sq_dist = bg::comparable_distance(position, border->line);
    if (sq_dist < min_sq_dist) {
      min_sq_dist = sq_dist;
      nearest_border = border;
    }
  }
  return nearest_border;
}

constexpr int k_linear_shift_steps = 3;
// Borders whose height differs more than this from the trajectory point are on another level
// (overpass / underpass). Well below the minimum vertical clearance of grade-separated roads.
constexpr double k_max_height_difference_m = 2.5;
// The reference height follows the nearest same-height border within this distance.
constexpr double k_height_tracking_distance_m = 10.0;
constexpr double k_bisection_eps_m = 1e-3;

/// Clear a colliding pose: up to 3 `step` probes, then bisection to `max_shift`.
/// If the cap is still colliding, finish with linear steps so a clear window
/// between the last probe and an opposite curb is not skipped.
template <typename CollidingFn>
bool find_clear_offset(
  double & offset, const double step, const double max_shift, const CollidingFn & colliding)
{
  const auto within_max = [max_shift](const double candidate) {
    return std::abs(candidate) <= max_shift + 1e-9;
  };

  int linear_steps = 0;
  while (linear_steps < k_linear_shift_steps && within_max(offset + step)) {
    offset += step;
    ++linear_steps;
    if (!colliding(offset)) {
      return true;
    }
  }

  const double hi = std::copysign(max_shift, step);
  if (!within_max(hi) || std::abs(hi - offset) <= k_bisection_eps_m) {
    return false;
  }

  if (colliding(hi)) {
    while (within_max(offset + step)) {
      offset += step;
      if (!colliding(offset)) {
        return true;
      }
    }
    return false;
  }

  double lo = offset;
  double clear = hi;
  while (std::abs(clear - lo) > k_bisection_eps_m) {
    const double mid = 0.5 * (lo + clear);
    if (colliding(mid)) {
      lo = mid;
    } else {
      clear = mid;
    }
  }
  offset = clear;
  return true;
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
  std::vector<RoadBorder> road_borders;
  for (const auto & line_string : lanelet_map.lineStringLayer) {
    const std::string line_string_type = line_string.attributeOr("type", "");
    if (line_string_type != "road_border" || line_string.size() < 2) {
      continue;
    }
    RoadBorder border;
    border.line.reserve(line_string.size());
    border.z.reserve(line_string.size());
    for (const auto & point : line_string) {
      border.line.emplace_back(point.x(), point.y());
      border.z.push_back(point.z());
    }
    road_borders.push_back(std::move(border));
  }
  set_road_borders(std::move(road_borders));
}

void RoadBorderAvoidance::set_road_borders(std::vector<RoadBorder> road_borders)
{
  // Split into two-point segments so that the height check applies to each part of a border:
  // a single line string may run from the ego road up onto an overpass.
  road_borders_.clear();
  for (const auto & border : road_borders) {
    for (size_t i = 0; i + 1 < border.line.size(); ++i) {
      road_borders_.push_back(
        RoadBorder{
          LineString2d{border.line[i], border.line[i + 1]}, {border.z[i], border.z[i + 1]}});
    }
  }
}

void RoadBorderAvoidance::set_road_borders(const std::vector<LineString2d> & road_borders)
{
  std::vector<RoadBorder> borders;
  borders.reserve(road_borders.size());
  for (const auto & line : road_borders) {
    borders.push_back(RoadBorder{line, std::vector<double>(line.size(), 0.0)});
  }
  set_road_borders(std::move(borders));
}

RoadBorderAvoidanceResult RoadBorderAvoidance::adjust(
  const Trajectory & raw_trajectory, const geometry_msgs::msg::Pose & ego_pose) const
{
  RoadBorderAvoidanceResult result;
  result.trajectory = raw_trajectory;
  if (road_borders_.empty() || raw_trajectory.points.empty()) {
    return result;
  }

  // Pre-filter borders reachable within the horizon.
  const Point2d ego_point(ego_pose.position.x, ego_pose.position.y);
  std::vector<const RoadBorder *> nearby_borders;
  for (const auto & border : road_borders_) {
    if (bg::distance(ego_point, border.line) <= params_.search_radius_m) {
      nearby_borders.push_back(&border);
    }
  }
  if (nearby_borders.empty()) {
    return result;
  }

  // Borders at the height of the current trajectory point (updated per point).
  std::vector<const RoadBorder *> same_level_borders;
  double reference_z = ego_pose.position.z;
  const auto intersects_any = [&same_level_borders](const LinearRing2d & footprint) {
    return std::any_of(
      same_level_borders.begin(), same_level_borders.end(),
      [&footprint](const RoadBorder * border) { return bg::intersects(footprint, border->line); });
  };

  // Signed lateral offset from the raw position (positive = left of the heading). With
  // propagate_shift it is carried over to subsequent points along their own lateral
  // direction; otherwise every point starts from the raw position again.
  double carried_offset_m = 0.0;

  for (auto & point : result.trajectory.points) {
    const double raw_x = point.pose.position.x;
    const double raw_y = point.pose.position.y;

    // Keep only borders on the same level and let the reference height follow the nearest one.
    same_level_borders.clear();
    NearestPoint nearest_same_level;
    for (const RoadBorder * border : nearby_borders) {
      const NearestPoint nearest = nearest_point_on_border(*border, Point2d(raw_x, raw_y));
      if (std::abs(nearest.z - reference_z) > k_max_height_difference_m) {
        continue;
      }
      same_level_borders.push_back(border);
      if (nearest.sq_dist < nearest_same_level.sq_dist) {
        nearest_same_level = nearest;
      }
    }
    if (nearest_same_level.sq_dist <= k_height_tracking_distance_m * k_height_tracking_distance_m) {
      reference_z = nearest_same_level.z;
    }

    if (rclcpp::Duration(point.time_from_start).seconds() < params_.start_time_s) {
      continue;
    }

    const double yaw = yaw_from_quaternion(point.pose.orientation);
    const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d lateral_left(-heading.y(), heading.x());

    double offset = params_.propagate_shift ? carried_offset_m : 0.0;
    const auto footprint_at = [&](const double off) {
      return place_footprint(
        base_footprint_, raw_x + lateral_left.x() * off, raw_y + lateral_left.y() * off, yaw);
    };
    const auto apply_offset = [&](const double off) {
      point.pose.position.x = raw_x + lateral_left.x() * off;
      point.pose.position.y = raw_y + lateral_left.y() * off;
    };

    const LinearRing2d footprint = footprint_at(offset);
    const Point2d position(raw_x + lateral_left.x() * offset, raw_y + lateral_left.y() * offset);

    // The nearest overlapping border (if any) decides the shift direction.
    const RoadBorder * offending_border =
      find_nearest_overlapping_border(same_level_borders, footprint, position);
    if (offending_border == nullptr) {
      if (offset != 0.0) {
        apply_offset(offset);
        ++result.num_shifted_points;
      }
      continue;
    }

    // Probe then bisect the offset perpendicular to the heading, away from the border.
    const Point2d border_point = nearest_point_on_border(*offending_border, position).point;
    const Eigen::Vector2d to_border = border_point - position;
    const double cross = heading.x() * to_border.y() - heading.y() * to_border.x();
    const double step = (cross > 0.0) ? -params_.shift_step_m : params_.shift_step_m;
    const bool resolved = find_clear_offset(
      offset, step, params_.max_lateral_shift_m,
      [&](const double off) { return intersects_any(footprint_at(off)); });

    // Apply the (possibly capped) offset; moving away is better than staying overlapped.
    apply_offset(offset);
    carried_offset_m = offset;
    if (resolved) {
      ++result.num_shifted_points;
    } else {
      ++result.num_unresolved_points;
    }
  }

  return result;
}

}  // namespace autoware::ml_planner::postprocess
