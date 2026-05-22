// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

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
constexpr std::size_t kNumSectors = 6;

// Closest point on a 2D polygon's outer ring to (0, 0). Walks each segment,
// projects origin onto it (clamped to [0, 1]), keeps the minimum-distance hit.
// Avoids boost::geometry::closest_points, which is Boost 1.81+ only.
struct PointXY
{
  double x;
  double y;
};
PointXY closest_polygon_point_to_origin(const autoware_utils_geometry::Polygon2d & poly)
{
  double min_d2 = std::numeric_limits<double>::infinity();
  PointXY best{0.0, 0.0};
  const auto & ring = poly.outer();
  if (ring.size() < 2) {
    return best;
  }
  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    const double x1 = ring[i].x();
    const double y1 = ring[i].y();
    const double x2 = ring[i + 1].x();
    const double y2 = ring[i + 1].y();
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-12) {
      t = -(x1 * dx + y1 * dy) / len2;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double px = x1 + t * dx;
    const double py = y1 + t * dy;
    const double d2 = px * px + py * py;
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
  max_detection_range_squared_ = params_.max_detection_range_m * params_.max_detection_range_m;

  // base_link is not necessarily at the vehicle's geometric center, so derive the
  // circumradius from the footprint itself rather than length/width.
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
  out.header = input.header;  // propagate the source perception header unchanged
  // out.sectors is fixed-size 6; all slots default to has_object == false.

  std::array<double, kNumSectors> closest_cd;
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

    // Transform pose into base_link; shape is object-local and unchanged.
    geometry_msgs::msg::Pose pose_in_base_link;
    tf2::doTransform(
      object.kinematics.initial_pose_with_covariance.pose, pose_in_base_link, to_base_link);

    // Bounding-circle pre-filter, in squared space (no sqrt).
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

    // Comparable (squared) polygon-to-polygon distance — same ordering as distance,
    // no sqrt. Cartesian comparable_distance returns the squared distance.
    const double cd = boost::geometry::comparable_distance(vehicle_footprint_, object_polygon);
    if (cd > max_detection_range_squared_) {
      continue;
    }

    // Bearing from base_link origin to the closest point on the object polygon.
    const auto object_closest = closest_polygon_point_to_origin(object_polygon);
    const double bearing = std::atan2(object_closest.y, object_closest.x);
    const auto sector_opt = bearing_to_sector(bearing);
    if (!sector_opt) {
      // Bearing fell outside all configured sector ranges — misconfiguration.
      continue;
    }
    const uint8_t sector = *sector_opt;

    // Keep only the closest per sector; defer sqrt until we know we're updating.
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
  // Normalize to [-pi, pi).
  double b = std::fmod(bearing_rad + M_PI, 2.0 * M_PI);
  if (b < 0.0) {
    b += 2.0 * M_PI;
  }
  b -= M_PI;

  // REP-103: +x forward, +y left. Bearing 0 = directly ahead. Sectors are half-open [a, b).
  auto sector = [](const auto & range) -> std::pair<double, double> {
    const auto start = autoware_utils_math::deg2rad(range.front());
    const auto end = autoware_utils_math::deg2rad(range.back());
    return std::make_pair(start, end);
  };

  // Handles wraparound: if end < start, the range crosses +/-pi (e.g. REAR).
  auto in_range = [b](double start, double end) {
    return (end > start) ? (b >= start && b < end) : (b >= start || b < end);
  };

  const auto [front_start, front_end] = sector(params_.sector_range.front);
  if (in_range(front_start, front_end)) return ProximityHazardObjects::FRONT;

  const auto [front_left_start, front_left_end] = sector(params_.sector_range.front_left);
  if (in_range(front_left_start, front_left_end)) return ProximityHazardObjects::FRONT_LEFT;

  const auto [rear_left_start, rear_left_end] = sector(params_.sector_range.rear_left);
  if (in_range(rear_left_start, rear_left_end)) return ProximityHazardObjects::REAR_LEFT;

  const auto [rear_start, rear_end] = sector(params_.sector_range.rear);
  if (in_range(rear_start, rear_end)) return ProximityHazardObjects::REAR;

  const auto [rear_right_start, rear_right_end] = sector(params_.sector_range.rear_right);
  if (in_range(rear_right_start, rear_right_end)) return ProximityHazardObjects::REAR_RIGHT;

  const auto [front_right_start, front_right_end] = sector(params_.sector_range.front_right);
  if (in_range(front_right_start, front_right_end)) return ProximityHazardObjects::FRONT_RIGHT;

  return std::nullopt;
}

double ProximityHazardObjectChecker::compute_circumradius(
  const autoware_perception_msgs::msg::Shape & shape)
{
  using ShapeMsg = autoware_perception_msgs::msg::Shape;
  switch (shape.type) {
    case ShapeMsg::BOUNDING_BOX:
      return 0.5 * std::hypot(shape.dimensions.x, shape.dimensions.y);
    case ShapeMsg::CYLINDER:
      return 0.5 * shape.dimensions.x;
    case ShapeMsg::POLYGON: {
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
