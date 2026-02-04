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

#include "polygon_utils.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/perimeter.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <cmath>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner::polygon_utils
{

using autoware_utils_geometry::Point2d;

std::vector<Polygon2d> create_one_step_polygons(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info,
  const double lat_margin)
{
  std::vector<Polygon2d> polygons;
  polygons.reserve(traj_points.size());
  for (const auto & point : traj_points) {
    auto polygon = autoware_utils_geometry::to_footprint(
      point.pose, vehicle_info.max_longitudinal_offset_m, vehicle_info.rear_overhang_m,
      vehicle_info.vehicle_width_m + 2.0 * lat_margin);
    boost::geometry::correct(polygon);
    polygons.push_back(polygon);
  }
  return polygons;
}

std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const std::vector<double> & decimated_s_values, const geometry_msgs::msg::Point & obj_position,
  const Polygon2d & obj_polygon, const double x_offset_to_bumper)
{
  if (
    traj_points.size() != traj_polygons.size() || traj_points.size() != decimated_s_values.size()) {
    return std::nullopt;
  }

  for (size_t i = 0; i < traj_points.size(); ++i) {
    // Rough distance check
    const double rough_dist =
      autoware_utils::calc_distance2d(traj_points[i].pose.position, obj_position);
    const double perimeter_half = boost::geometry::perimeter(traj_polygons[i]) * 0.5;
    if (rough_dist > perimeter_half) {
      continue;
    }

    // Precise intersection check
    if (!boost::geometry::intersects(obj_polygon, traj_polygons[i])) {
      continue;
    }

    // Find the collision point (farthest penetration from bumper)
    const auto bumper_pose =
      autoware_utils::calc_offset_pose(traj_points[i].pose, x_offset_to_bumper, 0.0, 0.0);

    double max_dist = 0.0;
    geometry_msgs::msg::Point collision_point = obj_position;

    for (const auto & obj_p : obj_polygon.outer()) {
      Point2d p2d{obj_p.x(), obj_p.y()};
      if (boost::geometry::within(p2d, traj_polygons[i])) {
        geometry_msgs::msg::Point geom_p;
        geom_p.x = obj_p.x();
        geom_p.y = obj_p.y();
        geom_p.z = obj_position.z;
        const double dist =
          std::abs(autoware_utils::inverse_transform_point(geom_p, bumper_pose).x);
        if (dist > max_dist) {
          max_dist = dist;
          collision_point = geom_p;
        }
      }
    }

    // Use pre-computed arc length on the main trajectory
    const double arc_length = decimated_s_values[i];
    return std::make_pair(collision_point, arc_length - max_dist);
  }

  return std::nullopt;
}

}  // namespace autoware::minimum_rule_based_planner::polygon_utils
