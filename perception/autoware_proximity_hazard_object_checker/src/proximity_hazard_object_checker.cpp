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

#include "autoware/proximity_hazard_object_checker/proximity_hazard_object_checker.hpp"

#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_math/unit_conversion.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace autoware::proximity_hazard_object_checker
{

namespace
{
struct PointXY
{
  double x;
  double y;
};

// Closest point on the polygon's outer ring to specific coordinates.
// Avoids boost::geometry::closest_points, which is Boost 1.81+ only.
PointXY closest_polygon_point_to_coordinates(
  const Polygon2d & polygon, const double rx, const double ry)
{
  PointXY best = {0.0, 0.0};
  double min_d2 = std::numeric_limits<double>::infinity();

  const auto & ring = polygon.outer();
  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    const double x1 = ring[i].x();
    const double y1 = ring[i].y();
    const double x2 = ring[i + 1].x();
    const double y2 = ring[i + 1].y();
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double len2 = dx * dx + dy * dy;

    double t{0.0};
    if (len2 >= std::numeric_limits<double>::epsilon()) {
      t = -((x1 - rx) * dx + (y1 - ry) * dy) / len2;
      t = std::clamp(t, 0.0, 1.0);
    }

    const double px = x1 + t * dx;
    const double py = y1 + t * dy;
    const double d2 = (px - rx) * (px - rx) + (py - ry) * (py - ry);
    if (d2 < min_d2) {
      min_d2 = d2;
      best = {px, py};
    }
  }
  return best;
}

// Closest point on a linear ring to specific coordinates.
PointXY closest_ring_point_to_coordinates(
  const LinearRing2d & ring, const double rx, const double ry)
{
  PointXY best = {0.0, 0.0};
  double min_d2 = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    const double x1 = ring[i].x();
    const double y1 = ring[i].y();
    const double x2 = ring[i + 1].x();
    const double y2 = ring[i + 1].y();
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double len2 = dx * dx + dy * dy;

    double t{0.0};
    if (len2 >= std::numeric_limits<double>::epsilon()) {
      t = -((x1 - rx) * dx + (y1 - ry) * dy) / len2;
      t = std::clamp(t, 0.0, 1.0);
    }

    const double px = x1 + t * dx;
    const double py = y1 + t * dy;
    const double d2 = (px - rx) * (px - rx) + (py - ry) * (py - ry);
    if (d2 < min_d2) {
      min_d2 = d2;
      best = {px, py};
    }
  }
  return best;
}
}  // namespace

ProximityHazardObjectChecker::ProximityHazardObjectChecker(
  proximity_hazard_object::Params params, LinearRing2d vehicle_footprint)
: params_(std::move(params)),
  vehicle_footprint_(std::move(vehicle_footprint)),
  vehicle_circumradius_(0.0),
  max_detection_range_squared_(0.0)
{
  boost::geometry::centroid(vehicle_footprint_, ego_center_);

  max_detection_range_squared_ = params_.max_detection_range_m * params_.max_detection_range_m;

  double max_r2 = 0.0;
  for (const auto & p : vehicle_footprint_) {
    max_r2 = std::max(max_r2, p.x() * p.x() + p.y() * p.y());
  }
  vehicle_circumradius_ = std::sqrt(max_r2);
}

ProximityHazardObjects ProximityHazardObjectChecker::process(
  const PredictedObjects & input, const geometry_msgs::msg::TransformStamped & to_base_link) const
{
  ProximityHazardObjects out;
  out.header = input.header;

  constexpr int num_sectors = 8;
  std::array<double, num_sectors> closest_cd{};
  closest_cd.fill(std::numeric_limits<double>::infinity());

  for (const auto & object : input.objects) {
    // Classification filter.
    if (!params_.include_unknown_objects) {
      const bool is_unknown =
        std::any_of(object.classification.begin(), object.classification.end(), [](const auto & c) {
          return c.label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
        });
      if (is_unknown) {
        continue;
      }
    }

    geometry_msgs::msg::Pose pose_in_base_link;
    tf2::doTransform(
      object.kinematics.initial_pose_with_covariance.pose, pose_in_base_link, to_base_link);

    const double object_radius = compute_circumradius(object.shape);
    const double center_d2 = pose_in_base_link.position.x * pose_in_base_link.position.x +
                             pose_in_base_link.position.y * pose_in_base_link.position.y;
    const double reject_radius =
      vehicle_circumradius_ + object_radius + params_.max_detection_range_m;
    if (center_d2 > reject_radius * reject_radius) {
      continue;
    }

    const auto object_polygon =
      autoware_utils_geometry::to_polygon2d(pose_in_base_link, object.shape);

    const double cd = boost::geometry::comparable_distance(vehicle_footprint_, object_polygon);
    if (cd > max_detection_range_squared_) {
      continue;
    }

    // Find the point on object_polygon closest to vehicle_footprint_ (the exact point of approach)
    PointXY object_closest = {0.0, 0.0};
    double min_poly_d2 = std::numeric_limits<double>::infinity();

    // 1. Check all vertices of vehicle_footprint_ against object_polygon
    for (const auto & p_ego : vehicle_footprint_) {
      PointXY p_obj = closest_polygon_point_to_coordinates(object_polygon, p_ego.x(), p_ego.y());
      double d2 = (p_obj.x - p_ego.x()) * (p_obj.x - p_ego.x()) +
                  (p_obj.y - p_ego.y()) * (p_obj.y - p_ego.y());
      if (d2 < min_poly_d2) {
        min_poly_d2 = d2;
        object_closest = p_obj;
      }
    }

    // 2. Check all vertices of object_polygon against vehicle_footprint_
    const auto & obj_ring = object_polygon.outer();
    for (const auto & p_obj : obj_ring) {
      PointXY p_ego = closest_ring_point_to_coordinates(vehicle_footprint_, p_obj.x(), p_obj.y());
      double d2 = (p_obj.x() - p_ego.x) * (p_obj.x() - p_ego.x) +
                  (p_obj.y() - p_ego.y) * (p_obj.y() - p_ego.y);
      if (d2 < min_poly_d2) {
        min_poly_d2 = d2;
        object_closest = {p_obj.x(), p_obj.y()};
      }
    }

    // Bearing from the ego footprint centroid (ego_center_) to the true closest point on the object
    // polygon.
    const double bearing =
      std::atan2(object_closest.y - ego_center_.y(), object_closest.x - ego_center_.x());
    const auto sector_opt = bearing_to_sector(bearing);
    if (!sector_opt) {
      continue;
    }
    const uint8_t sector = *sector_opt;

    if (cd < closest_cd[sector]) {
      closest_cd[sector] = cd;
      auto & slot = out.sectors[sector];
      slot.has_object = true;
      slot.distance_m = static_cast<float>(std::sqrt(cd));
      slot.predicted_object = object;
    }
  }

  return out;
}

std::optional<uint8_t> ProximityHazardObjectChecker::bearing_to_sector(double bearing_rad) const
{
  double b = std::fmod(bearing_rad + M_PI, 2.0 * M_PI);
  if (b < 0.0) {
    b += 2.0 * M_PI;
  }
  b -= M_PI;

  auto sector = [](const auto & range) -> std::pair<double, double> {
    const auto start = autoware_utils_math::deg2rad(range.front());
    const auto end = autoware_utils_math::deg2rad(range.back());
    return std::make_pair(start, end);
  };

  auto in_range = [b](double start, double end) {
    return (end > start) ? (b >= start && b < end) : (b >= start || b < end);
  };

  const auto [front_start, front_end] = sector(params_.sector_range.front);
  if (in_range(front_start, front_end)) return ProximityHazardObjects::FRONT;

  const auto [front_left_start, front_left_end] = sector(params_.sector_range.front_left);
  if (in_range(front_left_start, front_left_end)) return ProximityHazardObjects::FRONT_LEFT;

  const auto [left_start, left_end] = sector(params_.sector_range.left);
  if (in_range(left_start, left_end)) return ProximityHazardObjects::LEFT;

  const auto [rear_left_start, rear_left_end] = sector(params_.sector_range.rear_left);
  if (in_range(rear_left_start, rear_left_end)) return ProximityHazardObjects::REAR_LEFT;

  const auto [rear_start, rear_end] = sector(params_.sector_range.rear);
  if (in_range(rear_start, rear_end)) return ProximityHazardObjects::REAR;

  const auto [rear_right_start, rear_right_end] = sector(params_.sector_range.rear_right);
  if (in_range(rear_right_start, rear_right_end)) return ProximityHazardObjects::REAR_RIGHT;

  const auto [right_start, right_end] = sector(params_.sector_range.right);
  if (in_range(right_start, right_end)) return ProximityHazardObjects::RIGHT;

  const auto [front_right_start, front_right_end] = sector(params_.sector_range.front_right);
  if (in_range(front_right_start, front_right_end)) return ProximityHazardObjects::FRONT_RIGHT;

  return std::nullopt;
}

double ProximityHazardObjectChecker::compute_circumradius(
  const autoware_perception_msgs::msg::Shape & shape)
{
  switch (shape.type) {
    case autoware_perception_msgs::msg::Shape::BOUNDING_BOX:
      return 0.5 * std::hypot(shape.dimensions.x, shape.dimensions.y);
    case autoware_perception_msgs::msg::Shape::CYLINDER:
      return 0.5 * shape.dimensions.x;
    case autoware_perception_msgs::msg::Shape::POLYGON: {
      double max_r2 = 0.0;
      for (const auto & p : shape.footprint.points) {
        max_r2 = std::max(max_r2, static_cast<double>(p.x * p.x + p.y * p.y));
      }
      return std::sqrt(max_r2);
    }
    default:
      return 0.0;
  }
}

}  // namespace autoware::proximity_hazard_object_checker
