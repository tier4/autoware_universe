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

#include <Eigen/Geometry>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
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
constexpr double k_bisection_eps_m = 1e-3;
// Match autoware_ml_planner (#3659 + #3691) chunking and ego-level Z-filtering constants.
constexpr double k_max_same_level_height_diff_m = 4.0;
constexpr double k_same_level_height_diff_per_distance = 0.05;
constexpr size_t k_points_per_road_border = 20;
constexpr double k_road_border_max_step_m = 2.5;
constexpr double k_border_point_epsilon = 1e-6;

double safe_segment_length(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  const double dx = b.x() - a.x();
  const double dy = b.y() - a.y();
  const double dz = b.z() - a.z();
  const double sq = dx * dx + dy * dy + (std::isfinite(dz) ? dz * dz : 0.0);
  return std::sqrt(sq);
}

Eigen::Isometry3d make_map_to_ego_transform(const geometry_msgs::msg::Pose & ego_pose)
{
  Eigen::Quaterniond q(
    ego_pose.orientation.w, ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z);
  if (q.squaredNorm() > 1e-12) {
    q.normalize();
  } else {
    q = Eigen::Quaterniond::Identity();
  }
  Eigen::Isometry3d ego_to_map = Eigen::Isometry3d::Identity();
  ego_to_map.translation() =
    Eigen::Vector3d(ego_pose.position.x, ego_pose.position.y, ego_pose.position.z);
  ego_to_map.linear() = q.toRotationMatrix();
  return ego_to_map.inverse();
}

bool is_on_ego_level(
  const RoadBorder & border, const Eigen::Isometry3d & map_to_ego, const double fallback_map_z)
{
  if (border.line.empty() || border.line.size() != border.z.size()) {
    return false;
  }
  double closest_sq_dist = std::numeric_limits<double>::max();
  size_t closest_index = border.line.size();
  double closest_z_ego = std::numeric_limits<double>::quiet_NaN();
  for (size_t i = 0; i < border.line.size(); ++i) {
    // Avoid 0.0 * NaN poisoning horizontal XY selection when a vertex has non-finite Z.
    const double z_for_xy = std::isfinite(border.z[i]) ? border.z[i] : fallback_map_z;
    const Eigen::Vector3d p_ego_xy =
      map_to_ego * Eigen::Vector3d(border.line[i].x(), border.line[i].y(), z_for_xy);
    const double sq_dist = p_ego_xy.x() * p_ego_xy.x() + p_ego_xy.y() * p_ego_xy.y();
    if (sq_dist < closest_sq_dist) {
      closest_sq_dist = sq_dist;
      closest_index = i;
      closest_z_ego =
        std::isfinite(border.z[i])
          ? (map_to_ego * Eigen::Vector3d(border.line[i].x(), border.line[i].y(), border.z[i])).z()
          : border.z[i];
    }
  }
  if (closest_index == border.line.size() || !std::isfinite(closest_z_ego)) {
    return false;
  }
  const double distance = std::sqrt(closest_sq_dist);
  return std::abs(closest_z_ego) <=
         k_max_same_level_height_diff_m + k_same_level_height_diff_per_distance * distance;
}

RoadBorder to_road_border(const std::vector<Eigen::Vector3d> & points)
{
  RoadBorder border;
  border.line.reserve(points.size());
  border.z.reserve(points.size());
  for (const auto & p : points) {
    border.line.emplace_back(p.x(), p.y());
    border.z.push_back(p.z());
  }
  return border;
}

/// Split a polyline into chunks of `num_points` vertices with step <= `max_step_m`, keeping
/// every original vertex (matching autoware_ml_planner::split_line_string).
std::vector<RoadBorder> split_road_border_chunks(
  const std::vector<Eigen::Vector3d> & raw_points, const size_t num_points, const double max_step_m)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(raw_points.size());
  for (const auto & p : raw_points) {
    if (!std::isfinite(p.x()) || !std::isfinite(p.y())) {
      continue;
    }
    if (points.empty() || safe_segment_length(p, points.back()) > k_border_point_epsilon) {
      points.push_back(p);
    }
  }
  if (points.empty() || num_points < 2) {
    return {};
  }
  if (points.size() == 1) {
    return {to_road_border(std::vector<Eigen::Vector3d>(num_points, points.front()))};
  }

  struct Part
  {
    Eigen::Vector3d from;
    Eigen::Vector3d to;
    size_t n_steps;

    double step_length() const
    {
      return safe_segment_length(from, to) / static_cast<double>(n_steps);
    }
    void append_to(std::vector<Eigen::Vector3d> & out) const
    {
      for (size_t step = 1; step < n_steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(n_steps);
        const double x = from.x() + t * (to.x() - from.x());
        const double y = from.y() + t * (to.y() - from.y());
        const double z = (std::isfinite(from.z()) && std::isfinite(to.z()))
                           ? from.z() + t * (to.z() - from.z())
                           : std::numeric_limits<double>::quiet_NaN();
        out.emplace_back(x, y, z);
      }
      out.push_back(to);
    }
  };

  const double safe_max_step_m = std::max(max_step_m, k_border_point_epsilon);
  std::vector<Eigen::Vector3d> dense{points.front()};
  std::vector<size_t> edge_of_step;
  for (size_t i = 1; i < points.size(); ++i) {
    const Part edge{
      points[i - 1], points[i],
      static_cast<size_t>(
        std::max(1.0, std::ceil(safe_segment_length(points[i - 1], points[i]) / safe_max_step_m)))};
    edge.append_to(dense);
    edge_of_step.insert(edge_of_step.end(), edge.n_steps, i - 1);
  }

  const size_t n_steps = edge_of_step.size();
  const size_t n_pieces = (n_steps + num_points - 2) / (num_points - 1);
  std::vector<RoadBorder> chunks;
  chunks.reserve(n_pieces);
  for (size_t piece_index = 0; piece_index < n_pieces; ++piece_index) {
    const size_t begin = piece_index * n_steps / n_pieces;
    const size_t end = (piece_index + 1) * n_steps / n_pieces;
    std::vector<Part> parts;
    for (size_t step = begin; step < end; ++step) {
      if (!parts.empty() && edge_of_step[step] == edge_of_step[step - 1]) {
        parts.back().to = dense[step + 1];
        ++parts.back().n_steps;
      } else {
        parts.push_back(Part{dense[step], dense[step + 1], 1});
      }
    }

    for (size_t step_count = end - begin; step_count < num_points - 1; ++step_count) {
      const auto longest = std::max_element(
        parts.begin(), parts.end(),
        [](const Part & a, const Part & b) { return a.step_length() < b.step_length(); });
      ++longest->n_steps;
    }

    std::vector<Eigen::Vector3d> piece{parts.front().from};
    piece.reserve(num_points);
    for (const auto & part : parts) {
      part.append_to(piece);
    }
    chunks.push_back(to_road_border(piece));
  }
  return chunks;
}

/// Join map road-border line strings sharing an end point ID (valence == 2) so chunk boundaries do
/// not depend on how the map splits a border.
std::vector<std::vector<Eigen::Vector3d>> join_lanelet_road_borders(
  const std::vector<lanelet::ConstLineString3d> & line_strings)
{
  using End = std::pair<size_t, bool>;
  const auto end_point_id = [&line_strings](const End & end) {
    const auto & ls = line_strings[end.first];
    return end.second ? ls.front().id() : ls.back().id();
  };

  std::unordered_map<lanelet::Id, std::vector<End>> ends_at_point;
  for (size_t i = 0; i < line_strings.size(); ++i) {
    if (line_strings[i].empty()) {
      continue;
    }
    ends_at_point[end_point_id({i, true})].emplace_back(i, true);
    ends_at_point[end_point_id({i, false})].emplace_back(i, false);
  }
  const auto is_joint = [&](const End & end) {
    return ends_at_point.at(end_point_id(end)).size() == 2;
  };

  std::vector<bool> used(line_strings.size(), false);
  std::vector<std::vector<Eigen::Vector3d>> chains;
  const auto follow_chain = [&](size_t index, bool reversed) {
    std::vector<Eigen::Vector3d> chain;
    while (true) {
      used[index] = true;
      std::vector<Eigen::Vector3d> points;
      points.reserve(line_strings[index].size());
      for (const auto & p : line_strings[index]) {
        points.emplace_back(p.x(), p.y(), p.z());
      }
      if (reversed) {
        std::reverse(points.begin(), points.end());
      }
      chain.insert(chain.end(), points.begin() + (chain.empty() ? 0 : 1), points.end());

      const End exit_end{index, reversed};
      if (!is_joint(exit_end)) {
        break;
      }
      const auto & meeting = ends_at_point.at(end_point_id(exit_end));
      const End & next = meeting[0] == exit_end ? meeting[1] : meeting[0];
      if (used[next.first]) {
        break;
      }
      index = next.first;
      reversed = !next.second;
    }
    chains.push_back(std::move(chain));
  };

  for (size_t i = 0; i < line_strings.size(); ++i) {
    if (used[i] || line_strings[i].empty()) {
      continue;
    }
    if (!is_joint({i, true})) {
      follow_chain(i, false);
    } else if (!is_joint({i, false})) {
      follow_chain(i, true);
    }
  }
  for (size_t i = 0; i < line_strings.size(); ++i) {
    if (!used[i] && !line_strings[i].empty()) {
      follow_chain(i, false);
    }
  }
  return chains;
}

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
  road_borders_.clear();
  std::vector<lanelet::ConstLineString3d> road_border_line_strings;
  for (const auto & line_string : lanelet_map.lineStringLayer) {
    const std::string line_string_type = line_string.attributeOr("type", "");
    if (line_string_type == "road_border" && line_string.size() >= 2) {
      road_border_line_strings.push_back(line_string);
    }
  }
  for (const auto & chain : join_lanelet_road_borders(road_border_line_strings)) {
    if (chain.size() < 2) {
      continue;
    }
    auto chunks =
      split_road_border_chunks(chain, k_points_per_road_border, k_road_border_max_step_m);
    road_borders_.insert(
      road_borders_.end(), std::make_move_iterator(chunks.begin()),
      std::make_move_iterator(chunks.end()));
  }
}

void RoadBorderAvoidance::set_road_borders(std::vector<RoadBorder> road_borders)
{
  road_borders_.clear();
  for (const auto & border : road_borders) {
    if (border.line.size() != border.z.size() || border.line.size() < 2) {
      continue;
    }
    std::vector<Eigen::Vector3d> points;
    points.reserve(border.line.size());
    for (size_t i = 0; i < border.line.size(); ++i) {
      points.emplace_back(border.line[i].x(), border.line[i].y(), border.z[i]);
    }
    auto chunks =
      split_road_border_chunks(points, k_points_per_road_border, k_road_border_max_step_m);
    road_borders_.insert(
      road_borders_.end(), std::make_move_iterator(chunks.begin()),
      std::make_move_iterator(chunks.end()));
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

  const Eigen::Isometry3d map_to_ego = make_map_to_ego_transform(ego_pose);
  const Point2d ego_point(ego_pose.position.x, ego_pose.position.y);
  std::vector<const RoadBorder *> same_level_borders;
  for (const auto & border : road_borders_) {
    if (
      bg::distance(ego_point, border.line) <= params_.search_radius_m &&
      is_on_ego_level(border, map_to_ego, ego_pose.position.z)) {
      same_level_borders.push_back(&border);
    }
  }
  if (same_level_borders.empty()) {
    return result;
  }

  const auto intersects_any = [&same_level_borders](const LinearRing2d & footprint) {
    return std::any_of(
      same_level_borders.begin(), same_level_borders.end(),
      [&footprint](const RoadBorder * border) { return bg::intersects(footprint, border->line); });
  };

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

    const RoadBorder * offending_border =
      find_nearest_overlapping_border(same_level_borders, footprint, position);
    if (offending_border == nullptr) {
      if (offset != 0.0) {
        apply_offset(offset);
        ++result.num_shifted_points;
      }
      continue;
    }

    const Point2d border_point = nearest_point_on_border(*offending_border, position).point;
    const Eigen::Vector2d to_border = border_point - position;
    const double cross = heading.x() * to_border.y() - heading.y() * to_border.x();
    const double step = (cross > 0.0) ? -params_.shift_step_m : params_.shift_step_m;
    const bool resolved = find_clear_offset(
      offset, step, params_.max_lateral_shift_m,
      [&](const double off) { return intersects_any(footprint_at(off)); });

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

}  // namespace autoware::trajectory_modifier::time_sequence_raw
