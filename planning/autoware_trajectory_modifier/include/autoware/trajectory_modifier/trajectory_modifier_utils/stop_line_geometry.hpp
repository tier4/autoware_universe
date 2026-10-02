// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__STOP_LINE_GEOMETRY_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__STOP_LINE_GEOMETRY_HPP_

#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/intersection.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::trajectory_modifier::utils
{
// Extend only the checking geometry: a short base_link trajectory may still carry
// the vehicle front over a line. Output trajectory geometry and velocities are untouched.
template <class Trajectory, class LineString>
std::vector<double> crossed_with_front_offset(
  const Trajectory & path, const LineString & line, const double front_offset)
{
  auto collisions = autoware::experimental::trajectory::crossed(path, line);
  if (front_offset <= 0.0 || line.size() < 2) return collisions;

  const auto pose = path.compute(path.length()).pose;
  const auto front = autoware_utils::calc_offset_pose(pose, front_offset, 0.0, 0.0);
  const autoware_utils::LineString2d extension{
    {pose.position.x, pose.position.y}, {front.position.x, front.position.y}};
  autoware_utils::LineString2d stop_line;
  for (const auto & p : line) stop_line.emplace_back(p.x(), p.y());
  std::vector<autoware_utils::Point2d> intersections;
  boost::geometry::intersection(extension, stop_line, intersections);
  for (const auto & p : intersections) {
    collisions.push_back(
      path.length() + std::hypot(p.x() - pose.position.x, p.y() - pose.position.y));
  }
  std::sort(collisions.begin(), collisions.end());
  collisions.erase(
    std::unique(
      collisions.begin(), collisions.end(),
      [](const double a, const double b) { return std::abs(a - b) < 1e-6; }),
    collisions.end());
  return collisions;
}
}  // namespace autoware::trajectory_modifier::utils

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__STOP_LINE_GEOMETRY_HPP_
