// Copyright 2024 Tier IV, Inc.
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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Point.h>

#include <memory>
#include <sstream>
#include <utility>
#include <vector>

namespace
{
// calc smallest enclosing circle with average O(N) algorithm
// reference:
// https://erickimphotography.com/blog/wp-content/uploads/2018/09/Computational-Geometry-Algorithms-and-Applications-3rd-Ed.pdf
std::pair<lanelet::BasicPoint2d, double> get_smallest_enclosing_circle(
  const lanelet::ConstPolygon2d & poly)
{
  // The `eps` is used to avoid precision bugs in circle inclusion checks.
  // If the value of `eps` is too small, this function doesn't work well. More than 1e-10 is
  // recommended.
  const double eps = 1e-5;
  lanelet::BasicPoint2d center(0.0, 0.0);
  double radius_squared = 0.0;

  auto cross = [](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> double {
    return p1.x() * p2.y() - p1.y() * p2.x();
  };

  auto make_circle_3 = [&](
                         const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2,
                         const lanelet::BasicPoint2d & p3) -> void {
    // reference for circumcenter vector https://en.wikipedia.org/wiki/Circumscribed_circle
    const double a = (p2 - p3).squaredNorm();
    const double b = (p3 - p1).squaredNorm();
    const double c = (p1 - p2).squaredNorm();
    const double s = cross(p2 - p1, p3 - p1);
    if (std::abs(s) < eps) return;
    center = (a * (b + c - a) * p1 + b * (c + a - b) * p2 + c * (a + b - c) * p3) / (4 * s * s);
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto make_circle_2 =
    [&](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> void {
    center = (p1 + p2) * 0.5;
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto in_circle = [&](const lanelet::BasicPoint2d & p) -> bool {
    return (center - p).squaredNorm() <= radius_squared;
  };

  // mini disc
  for (size_t i = 1; i < poly.size(); i++) {
    const auto p1 = poly[i].basicPoint2d();
    if (in_circle(p1)) continue;

    // mini disc with point
    const auto p0 = poly[0].basicPoint2d();
    make_circle_2(p0, p1);
    for (size_t j = 0; j < i; j++) {
      const auto p2 = poly[j].basicPoint2d();
      if (in_circle(p2)) continue;

      // mini disc with two points
      make_circle_2(p1, p2);
      for (size_t k = 0; k < j; k++) {
        const auto p3 = poly[k].basicPoint2d();
        if (in_circle(p3)) continue;

        // mini disc with tree points
        make_circle_3(p1, p2, p3);
      }
    }
  }

  return std::make_pair(center, radius_squared);
}
}  // namespace

namespace autoware::behavior_velocity_planner::detection_area
{
autoware_utils::LineString2d get_stop_line(
  const lanelet::autoware::DetectionArea & detection_area,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  const auto stop_line = detection_area.stopLine();
  return planning_utils::extendSegmentToBounds(
    lanelet::utils::to2D(stop_line).basicLineString(), left_bound, right_bound);
}

std::optional<double> get_stop_point(
  const Trajectory & path, const autoware_utils::LineString2d & stop_line, const double margin,
  const double vehicle_offset, const lanelet::Ids & lane_ids)
{
  const auto collision_points = experimental::trajectory::crossed_with_constraint(
    path, stop_line, [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p) {
      return lane_ids.empty() ||
             std::any_of(p.lane_ids.begin(), p.lane_ids.end(), [&](const lanelet::Id id) {
               return std::find(lane_ids.begin(), lane_ids.end(), id) != lane_ids.end();
             });
    });

  if (collision_points.empty()) {
    return std::nullopt;
  }

  return collision_points.front() - margin - vehicle_offset;
}

std::vector<geometry_msgs::msg::Point> get_obstacle_points(
  const lanelet::ConstPolygons3d & detection_areas, const pcl::PointCloud<pcl::PointXYZ> & points)
{
  std::vector<geometry_msgs::msg::Point> obstacle_points;
  for (const auto & detection_area : detection_areas) {
    const auto poly = lanelet::utils::to2D(detection_area);
    const auto circle = get_smallest_enclosing_circle(poly);
    for (const auto p : points) {
      const double squared_dist = (circle.first.x() - p.x) * (circle.first.x() - p.x) +
                                  (circle.first.y() - p.y) * (circle.first.y() - p.y);
      if (squared_dist <= circle.second) {
        if (boost::geometry::within(Point2d{p.x, p.y}, poly.basicPolygon())) {
          obstacle_points.push_back(autoware_utils::create_point(p.x, p.y, p.z));
          // get all obstacle point becomes high computation cost so skip if any point is found
          break;
        }
      }
    }
  }
  return obstacle_points;
}

bool can_clear_stop_state(
  const std::shared_ptr<const rclcpp::Time> & last_obstacle_found_time, const rclcpp::Time & now,
  const double state_clear_time)
{
  // vehicle can clear stop state if the obstacle has never appeared in detection area
  if (!last_obstacle_found_time) {
    return true;
  }

  // vehicle can clear stop state if the certain time has passed since the obstacle disappeared
  const auto elapsed_time = now - *last_obstacle_found_time;
  if (elapsed_time.seconds() >= state_clear_time) {
    return true;
  }

  // rollback in simulation mode
  if (elapsed_time.seconds() < 0.0) {
    return true;
  }

  return false;
}

bool has_enough_braking_distance(
  const double self_s, const double line_point_s, const double pass_judge_line_distance,
  const double current_velocity)
{
  // prevent from being judged as not having enough distance when the current velocity is zero
  // and the vehicle crosses the stop line
  if (current_velocity < 1e-3) {
    return true;
  }

  return line_point_s - self_s > pass_judge_line_distance;
}

double feasible_stop_distance_by_max_acceleration(
  const double current_velocity, const double max_acceleration)
{
  return current_velocity * current_velocity / (2.0 * max_acceleration);
}

}  // namespace autoware::behavior_velocity_planner::detection_area
