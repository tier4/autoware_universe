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

#include "autoware/trajectory_modifier/trajectory_modifier_utils/detection_area_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>

namespace
{
std::pair<lanelet::BasicPoint2d, double> get_smallest_enclosing_circle(
  const lanelet::ConstPolygon2d & polygon)
{
  constexpr double epsilon = 1e-5;
  lanelet::BasicPoint2d center{0.0, 0.0};
  double radius_squared = 0.0;

  const auto cross = [](const lanelet::BasicPoint2d & lhs, const lanelet::BasicPoint2d & rhs) {
    return lhs.x() * rhs.y() - lhs.y() * rhs.x();
  };
  const auto make_circle_two = [&](const lanelet::BasicPoint2d & lhs,
                                   const lanelet::BasicPoint2d & rhs) {
    center = (lhs + rhs) * 0.5;
    radius_squared = (center - lhs).squaredNorm() + epsilon;
  };
  const auto in_circle = [&](const lanelet::BasicPoint2d & point) {
    return (center - point).squaredNorm() <= radius_squared;
  };

  for (size_t i = 1; i < polygon.size(); ++i) {
    const auto point_i = polygon[i].basicPoint2d();
    if (in_circle(point_i)) continue;
    make_circle_two(polygon[0].basicPoint2d(), point_i);
    for (size_t j = 0; j < i; ++j) {
      const auto point_j = polygon[j].basicPoint2d();
      if (in_circle(point_j)) continue;
      make_circle_two(point_i, point_j);
      for (size_t k = 0; k < j; ++k) {
        const auto point_k = polygon[k].basicPoint2d();
        if (in_circle(point_k)) continue;

        const double a = (point_i - point_j).squaredNorm();
        const double b = (point_j - point_k).squaredNorm();
        const double c = (point_k - point_i).squaredNorm();
        const double twice_area = cross(point_i - point_k, point_j - point_k);
        if (std::abs(twice_area) < epsilon) continue;
        center =
          (a * (b + c - a) * point_i + b * (c + a - b) * point_j +
           c * (a + b - c) * point_k) /
          (4.0 * twice_area * twice_area);
        radius_squared = (center - point_i).squaredNorm() + epsilon;
      }
    }
  }
  return {center, radius_squared};
}
}  // namespace

namespace autoware::trajectory_modifier::utils::detection_area
{
std::optional<double> get_stop_point(
  const Trajectory & path, const lanelet::ConstLineString3d & stop_line, const double margin,
  const double vehicle_offset)
{
  const auto collisions = autoware::experimental::trajectory::crossed(path, stop_line);
  if (collisions.empty()) return std::nullopt;
  return collisions.front() - margin - vehicle_offset;
}

std::vector<geometry_msgs::msg::Point> get_obstacle_points(
  const lanelet::ConstPolygons3d & detection_areas, const PointCloud & points)
{
  std::vector<geometry_msgs::msg::Point> obstacle_points;
  for (const auto & detection_area : detection_areas) {
    const auto polygon = lanelet::utils::to2D(detection_area);
    const auto circle = get_smallest_enclosing_circle(polygon);
    for (const auto & point : points) {
      const double dx = circle.first.x() - point.x;
      const double dy = circle.first.y() - point.y;
      if (dx * dx + dy * dy > circle.second) continue;
      if (lanelet::geometry::within(
            lanelet::BasicPoint2d{point.x, point.y}, polygon.basicPolygon())) {
        obstacle_points.push_back(autoware_utils::create_point(point.x, point.y, point.z));
        break;
      }
    }
  }
  return obstacle_points;
}

std::optional<autoware_perception_msgs::msg::PredictedObject> get_detected_object(
  const lanelet::ConstPolygons3d & detection_areas,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  const TargetFiltering & target_filtering)
{
  for (const auto & object : predicted_objects.objects) {
    if (!is_target_object(object.classification, target_filtering)) continue;
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto object_polygon = autoware_utils::to_polygon2d(pose, object.shape);
    for (const auto & detection_area : detection_areas) {
      const auto detection_polygon = lanelet::utils::to2D(detection_area).basicPolygon();
      if (boost::geometry::intersects(object_polygon, detection_polygon)) return object;
    }
  }
  return std::nullopt;
}

bool is_target_object(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classifications,
  const TargetFiltering & target_filtering)
{
  using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;
  if (classifications.empty()) return false;
  const auto label = autoware::object_recognition_utils::getHighestProbLabel(classifications);
  switch (label) {
    case ObjectClassification::UNKNOWN:
      return target_filtering.unknown;
    case ObjectClassification::CAR:
      return target_filtering.car;
    case ObjectClassification::TRUCK:
      return target_filtering.truck;
    case ObjectClassification::BUS:
      return target_filtering.bus;
    case ObjectClassification::TRAILER:
      return target_filtering.trailer;
    case ObjectClassification::MOTORCYCLE:
      return target_filtering.motorcycle;
    case ObjectClassification::BICYCLE:
      return target_filtering.bicycle;
    case ObjectClassification::PEDESTRIAN:
      return target_filtering.pedestrian;
    case ObjectClassification::ANIMAL:
      return target_filtering.animal;
    case ObjectClassification::HAZARD:
      return target_filtering.hazard;
    case ObjectClassification::OVER_DRIVABLE:
      return target_filtering.over_drivable;
    case ObjectClassification::UNDER_DRIVABLE:
      return target_filtering.under_drivable;
    default:
      return false;
  }
}

std::string object_label_to_string(const uint8_t label)
{
  using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;
  switch (label) {
    case ObjectClassification::UNKNOWN:
      return "unknown";
    case ObjectClassification::CAR:
      return "car";
    case ObjectClassification::TRUCK:
      return "truck";
    case ObjectClassification::BUS:
      return "bus";
    case ObjectClassification::TRAILER:
      return "trailer";
    case ObjectClassification::MOTORCYCLE:
      return "motorcycle";
    case ObjectClassification::BICYCLE:
      return "bicycle";
    case ObjectClassification::PEDESTRIAN:
      return "pedestrian";
    case ObjectClassification::ANIMAL:
      return "animal";
    case ObjectClassification::HAZARD:
      return "hazard";
    case ObjectClassification::OVER_DRIVABLE:
      return "over_drivable";
    case ObjectClassification::UNDER_DRIVABLE:
      return "under_drivable";
    default:
      return "unrecognized";
  }
}

bool can_clear_stop_state(
  const std::optional<rclcpp::Time> & last_obstacle_found_time, const rclcpp::Time & now,
  const double state_clear_time)
{
  if (!last_obstacle_found_time) return true;
  const auto elapsed = (now - *last_obstacle_found_time).seconds();
  return elapsed >= state_clear_time || elapsed < 0.0;
}

double feasible_stop_distance_by_max_acceleration(
  const double current_velocity, const double max_acceleration)
{
  if (max_acceleration <= 0.0) return 0.0;
  return current_velocity * current_velocity / (2.0 * max_acceleration);
}
}  // namespace autoware::trajectory_modifier::utils::detection_area
