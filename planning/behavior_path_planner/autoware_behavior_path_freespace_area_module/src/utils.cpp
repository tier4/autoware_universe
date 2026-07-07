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

#include "autoware/behavior_path_freespace_area_module/utils.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::freespace_area_utils
{

void encodeTravelDirectionOrientation(PathWithLaneId & path)
{
  for (auto & p : path.points) {
    if (p.point.longitudinal_velocity_mps < 0.0) {
      double yaw = tf2::getYaw(p.point.pose.orientation);
      yaw = autoware_utils::normalize_radian(yaw + M_PI);
      p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    }
    p.point.longitudinal_velocity_mps = std::abs(p.point.longitudinal_velocity_mps);
  }
}

std::vector<size_t> detectCuspIndices(const PathWithLaneId & path, const double angle_threshold_deg)
{
  std::vector<size_t> cusp_indices;
  if (path.points.size() < 2) {
    return cusp_indices;
  }
  const double angle_threshold_rad = autoware_utils::deg2rad(angle_threshold_deg);
  for (size_t i = 1; i < path.points.size(); ++i) {
    const double yaw_prev = tf2::getYaw(path.points[i - 1].point.pose.orientation);
    const double yaw_curr = tf2::getYaw(path.points[i].point.pose.orientation);
    const double angle_diff = autoware_utils::normalize_radian(yaw_curr - yaw_prev);
    if (std::abs(angle_diff) > angle_threshold_rad) {
      cusp_indices.push_back(i);
    }
  }
  return cusp_indices;
}

geometry_msgs::msg::PoseArray toPoseArray(const PathWithLaneId & path)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header = path.header;
  pose_array.poses.reserve(path.points.size());
  for (const auto & p : path.points) {
    pose_array.poses.push_back(p.point.pose);
  }
  return pose_array;
}

namespace
{
double signedLateralOffset(const geometry_msgs::msg::Point & query, const PathWithLaneId & path)
{
  if (path.points.size() < 2) {
    return 0.0;
  }
  // find nearest path point
  size_t nearest = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & pp = path.points[i].point.pose.position;
    const double d = std::hypot(query.x - pp.x, query.y - pp.y);
    if (d < min_dist) {
      min_dist = d;
      nearest = i;
    }
  }
  const size_t i0 = (nearest + 1 < path.points.size()) ? nearest : nearest - 1;
  const auto & a = path.points[i0].point.pose.position;
  const auto & b = path.points[i0 + 1].point.pose.position;
  const double heading = std::atan2(b.y - a.y, b.x - a.x);
  const double dx = query.x - a.x;
  const double dy = query.y - a.y;
  // cross product of heading unit vector with (query - a): positive => left side
  return -std::sin(heading) * dx + std::cos(heading) * dy;
}
}  // namespace

std::pair<std::vector<geometry_msgs::msg::Point>, std::vector<geometry_msgs::msg::Point>>
generateBoundsFromAreaPolygon(const PathWithLaneId & path, const lanelet::ConstArea & area)
{
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;

  // Extract ordered polygon vertices.
  std::vector<geometry_msgs::msg::Point> vertices;
  for (const auto & p : area.outerBoundPolygon()) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    vertices.push_back(pt);
  }
  const size_t n = vertices.size();
  if (n < 3 || path.points.size() < 2) {
    return {left_bound, right_bound};
  }

  // Cumulative arc length of the path (longitudinal coordinate reference).
  std::vector<double> cum_len(path.points.size(), 0.0);
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & a = path.points[i - 1].point.pose.position;
    const auto & b = path.points[i].point.pose.position;
    cum_len[i] = cum_len[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }

  auto longitudinal_position = [&path, &cum_len](const geometry_msgs::msg::Point & q) {
    size_t nearest = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto & pp = path.points[i].point.pose.position;
      const double d = std::hypot(q.x - pp.x, q.y - pp.y);
      if (d < min_dist) {
        min_dist = d;
        nearest = i;
      }
    }
    return cum_len[nearest];
  };

  // Classify each vertex by its signed lateral offset from the path, then order each side by
  // longitudinal position so the bounds run in the path direction. This is robust for
  // convex-ish areas; strongly non-convex outlines may be misordered (documented limitation).
  std::vector<std::pair<double, geometry_msgs::msg::Point>> left_tagged;
  std::vector<std::pair<double, geometry_msgs::msg::Point>> right_tagged;
  for (const auto & v : vertices) {
    const double offset = signedLateralOffset(v, path);
    const double s = longitudinal_position(v);
    if (offset > 0.0) {
      left_tagged.emplace_back(s, v);
    } else {
      right_tagged.emplace_back(s, v);
    }
  }

  auto by_s = [](const auto & a, const auto & b) { return a.first < b.first; };
  std::sort(left_tagged.begin(), left_tagged.end(), by_s);
  std::sort(right_tagged.begin(), right_tagged.end(), by_s);

  left_bound.reserve(left_tagged.size());
  for (const auto & [s, v] : left_tagged) {
    left_bound.push_back(v);
  }
  right_bound.reserve(right_tagged.size());
  for (const auto & [s, v] : right_tagged) {
    right_bound.push_back(v);
  }

  return {left_bound, right_bound};
}

nav_msgs::msg::OccupancyGrid cropCostmapAroundArea(
  const nav_msgs::msg::OccupancyGrid & src, const lanelet::ConstArea & area,
  const std::vector<geometry_msgs::msg::Pose> & keep_poses, const double margin)
{
  const auto & q = src.info.origin.orientation;
  const bool identity_orientation =
    std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 && std::abs(q.z) < 1e-6;
  if (!identity_orientation || src.info.resolution <= 0.0) {
    return src;
  }

  // bounding box over the area outline and the poses to keep (start/goal)
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  const auto expand = [&](const double x, const double y) {
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
  };
  for (const auto & p : area.outerBoundPolygon()) {
    expand(p.x(), p.y());
  }
  for (const auto & pose : keep_poses) {
    expand(pose.position.x, pose.position.y);
  }
  if (min_x > max_x || min_y > max_y) {
    return src;
  }
  min_x -= margin;
  min_y -= margin;
  max_x += margin;
  max_y += margin;

  const double res = src.info.resolution;
  const double src_ox = src.info.origin.position.x;
  const double src_oy = src.info.origin.position.y;

  // snap the crop window to the source grid cells
  const auto to_cell_x = [&](const double x) {
    return static_cast<int64_t>(std::floor((x - src_ox) / res));
  };
  const auto to_cell_y = [&](const double y) {
    return static_cast<int64_t>(std::floor((y - src_oy) / res));
  };
  const int64_t cx0 = to_cell_x(min_x);
  const int64_t cy0 = to_cell_y(min_y);
  const int64_t cx1 = to_cell_x(max_x) + 1;
  const int64_t cy1 = to_cell_y(max_y) + 1;
  const int64_t out_w = cx1 - cx0;
  const int64_t out_h = cy1 - cy0;
  if (out_w <= 0 || out_h <= 0) {
    return src;
  }

  nav_msgs::msg::OccupancyGrid out;
  out.header = src.header;
  out.info = src.info;
  out.info.width = static_cast<uint32_t>(out_w);
  out.info.height = static_cast<uint32_t>(out_h);
  out.info.origin.position.x = src_ox + static_cast<double>(cx0) * res;
  out.info.origin.position.y = src_oy + static_cast<double>(cy0) * res;
  // cells with no source data (outside the published grid) are treated as occupied
  out.data.assign(static_cast<size_t>(out_w * out_h), static_cast<int8_t>(100));

  const int64_t src_w = src.info.width;
  const int64_t src_h = src.info.height;
  const int64_t copy_x0 = std::max<int64_t>(cx0, 0);
  const int64_t copy_x1 = std::min<int64_t>(cx1, src_w);
  const int64_t copy_y0 = std::max<int64_t>(cy0, 0);
  const int64_t copy_y1 = std::min<int64_t>(cy1, src_h);
  for (int64_t sy = copy_y0; sy < copy_y1; ++sy) {
    const int64_t oy = sy - cy0;
    for (int64_t sx = copy_x0; sx < copy_x1; ++sx) {
      const int64_t ox = sx - cx0;
      out.data[static_cast<size_t>(oy * out_w + ox)] =
        src.data[static_cast<size_t>(sy * src_w + sx)];
    }
  }
  return out;
}

}  // namespace autoware::behavior_path_planner::freespace_area_utils
