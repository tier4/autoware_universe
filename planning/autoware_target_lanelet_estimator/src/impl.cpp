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

#include "autoware/target_lanelet_estimator/impl.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BoundingBox.h>

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace autoware::target_lanelet_estimator
{
namespace
{
lanelet::ConstLanelets extract_route_lanelets(
  const LaneletRoute & route, const lanelet::LaneletMapConstPtr & lanelet_map)
{
  lanelet::ConstLanelets route_lanelets;
  for (const auto & segment : route.segments) {
    for (const auto & primitive : segment.primitives) {
      route_lanelets.push_back(lanelet_map->laneletLayer.get(primitive.id));
    }
  }
  return route_lanelets;
}

std::vector<lanelet::BasicPolygon2d> compute_trajectory_footprints(
  const Trajectory & trajectory, const VehicleInfo & vehicle_info)
{
  const auto base_footprint = vehicle_info.createFootprint();

  std::vector<lanelet::BasicPolygon2d> footprints;
  footprints.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    const auto & pose = point.pose;
    const double yaw = tf2::getYaw(pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    lanelet::BasicPolygon2d footprint;
    footprint.reserve(base_footprint.size());
    for (const auto & p : base_footprint) {
      footprint.emplace_back(
        pose.position.x + cos_yaw * p.x() - sin_yaw * p.y(),
        pose.position.y + sin_yaw * p.x() + cos_yaw * p.y());
    }
    footprints.push_back(std::move(footprint));
  }
  return footprints;
}

size_t count_overlapping_footprints(
  const lanelet::ConstLanelet & lanelet, const std::vector<lanelet::BasicPolygon2d> & footprints)
{
  const auto lanelet_polygon = lanelet.polygon2d().basicPolygon();
  size_t count = 0;
  for (const auto & footprint : footprints) {
    if (!boost::geometry::disjoint(footprint, lanelet_polygon)) {
      ++count;
    }
  }
  return count;
}

bool is_on_any_lanelet(
  const lanelet::BasicPolygon2d & footprint, const lanelet::LaneletMapConstPtr & lanelet_map)
{
  lanelet::BoundingBox2d bbox;
  for (const auto & p : footprint) {
    bbox.extend(p);
  }
  for (const auto & lanelet : lanelet_map->laneletLayer.search(bbox)) {
    if (!boost::geometry::disjoint(footprint, lanelet.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}
}  // namespace

TargetLaneletsResult get_target_lanelets(
  const LaneletRoute & route, const Trajectory & trajectory,
  const lanelet::LaneletMapConstPtr & lanelet_map, const VehicleInfo & vehicle_info)
{
  const auto footprints = compute_trajectory_footprints(trajectory, vehicle_info);

  TargetLaneletsResult result;
  for (const auto & lanelet : extract_route_lanelets(route, lanelet_map)) {
    const auto overlap_count = count_overlapping_footprints(lanelet, footprints);
    if (overlap_count > 0) {
      result.lanelets.push_back({lanelet.id(), 100.0});
    }
  }

  // out_of_lanelet: a footprint overlaps no lanelet at all
  for (const auto & footprint : footprints) {
    if (!is_on_any_lanelet(footprint, lanelet_map)) {
      result.out_of_lanelet = true;
      break;
    }
  }
  return result;
}

}  // namespace autoware::target_lanelet_estimator
