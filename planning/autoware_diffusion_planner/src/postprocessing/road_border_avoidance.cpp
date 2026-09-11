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

#include "autoware/diffusion_planner/postprocessing/road_border_avoidance.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::postprocess
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

Point2d nearest_point_on_linestring(const LineString2d & line, const Point2d & point)
{
  Point2d nearest = line.front();
  double min_sq_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    const Eigen::Vector2d & a = line[i];
    const Eigen::Vector2d & b = line[i + 1];
    const Eigen::Vector2d ab = b - a;
    const double ab_sq = ab.squaredNorm();
    const double t = (ab_sq > 0.0) ? std::clamp((point - a).dot(ab) / ab_sq, 0.0, 1.0) : 0.0;
    const Eigen::Vector2d candidate = a + t * ab;
    const double sq_dist = (point - candidate).squaredNorm();
    if (sq_dist < min_sq_dist) {
      min_sq_dist = sq_dist;
      nearest = Point2d(candidate.x(), candidate.y());
    }
  }
  return nearest;
}

const LineString2d * find_nearest_overlapping_border(
  const std::vector<const LineString2d *> & borders, const LinearRing2d & footprint,
  const Point2d & position)
{
  const LineString2d * nearest_border = nullptr;
  double min_sq_dist = std::numeric_limits<double>::max();
  for (const LineString2d * border : borders) {
    if (!bg::intersects(footprint, *border)) {
      continue;
    }
    const double sq_dist = bg::comparable_distance(position, *border);
    if (sq_dist < min_sq_dist) {
      min_sq_dist = sq_dist;
      nearest_border = border;
    }
  }
  return nearest_border;
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

  // Pre-filter borders reachable within the horizon.
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

  // Signed lateral offset from the raw position (positive = left of the heading). With
  // propagate_shift it is carried over to subsequent points along their own lateral
  // direction; otherwise every point starts from the raw position again.
  double carried_offset_m = 0.0;

  for (auto & point : result.trajectory.points) {
    const double raw_x = point.pose.position.x;
    const double raw_y = point.pose.position.y;
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
    const LineString2d * offending_border =
      find_nearest_overlapping_border(nearby_borders, footprint, position);
    if (offending_border == nullptr) {
      if (offset != 0.0) {
        apply_offset(offset);
        ++result.num_shifted_points;
      }
      continue;
    }

    // Step the offset perpendicular to the heading, away from the border side.
    const Point2d border_point = nearest_point_on_linestring(*offending_border, position);
    const Eigen::Vector2d to_border = border_point - position;
    const double cross = heading.x() * to_border.y() - heading.y() * to_border.x();
    const double step = (cross > 0.0) ? -params_.shift_step_m : params_.shift_step_m;

    bool resolved = false;
    while (std::abs(offset + step) <= params_.max_lateral_shift_m + 1e-9) {
      offset += step;
      if (!intersects_any(footprint_at(offset))) {
        resolved = true;
        break;
      }
    }

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

}  // namespace autoware::diffusion_planner::postprocess
