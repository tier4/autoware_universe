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

#include "autoware/control_validator/hazard_warning/hazard_warning.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_validator::hazard
{
namespace
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kWedgeArcStepDeg = 10.0;

double deg2rad(const double deg)
{
  return deg * M_PI / 180.0;
}

std::optional<HazardWarningParam> to_hazard_warning_param(
  const ::control_validator::Params & params)
{
  const auto & p = params.hazard_warning;
  HazardWarningParam out;
  out.enable = p.enable;
  out.min_alert_duration = p.min_alert_duration;
  out.sector_radius_m = p.sector.radius_m;
  out.sector_boundary_tolerance_deg = p.sector.boundary_tolerance_deg;
  const std::array<const std::vector<double> *, kNumSectors> ranges{
    &p.sector.range.front,      &p.sector.range.front_right, &p.sector.range.right,
    &p.sector.range.rear_right, &p.sector.range.rear,        &p.sector.range.rear_left,
    &p.sector.range.left,       &p.sector.range.front_left};
  for (size_t s = 0; s < kNumSectors; ++s) {
    if (ranges[s]->size() != 2) {
      return std::nullopt;
    }
    out.sector_ranges[s] = {(*ranges[s])[0], (*ranges[s])[1]};
  }
  return out;
}

std_msgs::msg::ColorRGBA make_color(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::msg::ColorRGBA level_color(const uint8_t level)
{
  switch (level) {
    case 0:
      return make_color(0.3F, 0.8F, 0.3F, 0.8F);
    case 1:
      return make_color(1.0F, 1.0F, 0.0F, 0.9F);
    case 2:
      return make_color(1.0F, 0.5F, 0.0F, 0.9F);
    default:
      return make_color(1.0F, 0.0F, 0.0F, 0.9F);
  }
}

Marker make_polygon_marker(
  const Polygon2d & polygon, const std::string & ns, const int32_t id,
  const std_msgs::msg::ColorRGBA & color, const builtin_interfaces::msg::Time & stamp)
{
  Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color = color;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  marker.frame_locked = true;
  if (polygon.outer().size() == 1) {
    marker.type = Marker::SPHERE;
    marker.pose.position.x = polygon.outer().front().x();
    marker.pose.position.y = polygon.outer().front().y();
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    return marker;
  }
  marker.type = Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  for (const auto & pt : polygon.outer()) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    marker.points.push_back(p);
  }
  return marker;
}

Marker make_risk_level_marker(
  const HazardSource & source, const int32_t id, const builtin_interfaces::msg::Time & stamp)
{
  Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = stamp;
  marker.ns = "hazard_risk_level";
  marker.id = id;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.action = Marker::ADD;
  Point2d position = source.shape.outer().front();
  if (source.shape.outer().size() > 1) {
    boost::geometry::centroid(source.shape, position);
  }
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.5;
  marker.color = level_color(source.risk_level);
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  marker.frame_locked = true;
  marker.text = "risk_level:" + std::to_string(source.risk_level);
  return marker;
}
}  // namespace

HazardWarning::HazardWarning(
  const rclcpp::Logger & logger, const ::control_validator::Params & params, rclcpp::Node * node,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: logger_(logger), time_keeper_(std::move(time_keeper))
{
  update_parameters(params);
  if (node) {
    pub_markers_ = node->create_publisher<MarkerArray>("~/debug/hazard/markers", 1);
  }
}

void HazardWarning::update_parameters(const ::control_validator::Params & params)
{
  publish_markers_ = params.hazard_warning.debug.publish_markers;
  const auto param = to_hazard_warning_param(params);
  if (!param) {
    RCLCPP_ERROR(
      logger_, "hazard_warning.sector.range entries must have 2 elements. Warning disabled.");
    enable_ = false;
    return;
  }
  param_ = *param;
  enable_ = validate_parameters();
  if (wedge_apex_) {
    build_wedges(*wedge_apex_);
  }
}

bool HazardWarning::validate_parameters() const
{
  if (!std::isfinite(param_.min_alert_duration) || param_.min_alert_duration < 0.0) {
    RCLCPP_ERROR(logger_, "hazard_warning.min_alert_duration must be >= 0. Warning disabled.");
    return false;
  }
  if (!std::isfinite(param_.sector_radius_m) || param_.sector_radius_m <= 0.0) {
    RCLCPP_ERROR(logger_, "hazard_warning.sector.radius_m must be > 0. Warning disabled.");
    return false;
  }
  if (!std::isfinite(param_.sector_boundary_tolerance_deg)) {
    RCLCPP_ERROR(
      logger_, "hazard_warning.sector.boundary_tolerance_deg must be finite. Warning disabled.");
    return false;
  }
  for (const auto & range : param_.sector_ranges) {
    if (!std::isfinite(range[0]) || !std::isfinite(range[1])) {
      RCLCPP_ERROR(logger_, "hazard_warning.sector.range must be finite. Warning disabled.");
      return false;
    }
  }
  return true;
}

void HazardWarning::build_wedges(const Point2d & apex)
{
  // Same construction as autoware_proximity_hazard_checker: the wedge apex is the centroid of the
  // vehicle footprint (base_link), the angular range is widened by the boundary tolerance. The arc
  // is polygonised so that the whole radius is covered for wide sectors.
  wedge_apex_ = apex;
  const double radius = param_.sector_radius_m;
  for (size_t s = 0; s < kNumSectors; ++s) {
    const double from_deg = param_.sector_ranges[s][0] - param_.sector_boundary_tolerance_deg;
    double to_deg = param_.sector_ranges[s][1] + param_.sector_boundary_tolerance_deg;
    while (to_deg <= from_deg) {
      to_deg += 360.0;
    }
    const auto num_arc_steps =
      std::max<size_t>(1, static_cast<size_t>(std::ceil((to_deg - from_deg) / kWedgeArcStepDeg)));

    Polygon2d wedge;
    boost::geometry::append(wedge, apex);
    for (size_t i = 0; i <= num_arc_steps; ++i) {
      const double angle =
        deg2rad(from_deg + (to_deg - from_deg) * static_cast<double>(i) / num_arc_steps);
      boost::geometry::append(
        wedge, Point2d{apex.x() + radius * std::cos(angle), apex.y() + radius * std::sin(angle)});
    }
    boost::geometry::append(wedge, apex);
    boost::geometry::correct(wedge);
    wedges_[s] = wedge;
  }
}

void HazardWarning::assign_sectors(
  const HazardWarningSources & hazard_sources, std::array<uint8_t, kNumSectors> & max_level) const
{
  max_level.fill(0);
  for (const auto * sources : {&hazard_sources.collision, &hazard_sources.departure}) {
    if (!sources->has_value()) {
      continue;
    }
    for (const auto & source : **sources) {
      if (source.shape.outer().empty()) {
        continue;
      }
      const bool is_point = source.shape.outer().size() == 1;
      for (size_t s = 0; s < kNumSectors; ++s) {
        const bool in_sector = is_point
                                 ? boost::geometry::within(source.shape.outer().front(), wedges_[s])
                                 : boost::geometry::intersects(source.shape, wedges_[s]);
        if (in_sector) {
          max_level[s] = std::max(max_level[s], source.risk_level);
        }
      }
    }
  }
}

std::optional<HazardAlert> HazardWarning::make_alert(
  const HazardWarningSources & hazard_sources, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const rclcpp::Time & now)
{
  if (
    !param_.enable || !enable_ ||
    (!hazard_sources.collision.has_value() && !hazard_sources.departure.has_value())) {
    return std::nullopt;
  }

  const Point2d apex{
    0.5 * (vehicle_info.max_longitudinal_offset_m + vehicle_info.min_longitudinal_offset_m),
    0.5 * (vehicle_info.left_overhang_m - vehicle_info.right_overhang_m)};
  if (!wedge_apex_ || !boost::geometry::equals(*wedge_apex_, apex)) {
    build_wedges(apex);
  }

  std::array<uint8_t, kNumSectors> max_level{};
  assign_sectors(hazard_sources, max_level);

  HazardAlert alert;
  alert.header.stamp = now;
  alert.header.frame_id = "base_link";
  for (size_t s = 0; s < kNumSectors; ++s) {
    const bool raw = max_level[s] >= 1;
    const bool on = sector_latches_[s].update(raw, now, param_.min_alert_duration);
    if (raw) {
      last_level_[s] = max_level[s];
    } else if (!on) {
      last_level_[s] = 0;
    }
    alert.sectors[s].has_object = on;
    alert.sectors[s].risk_level = on ? last_level_[s] : 0;
  }

  time_keeper_->start_track("hazard_warning_publish_debug");
  publish_debug(hazard_sources, alert);
  time_keeper_->end_track("hazard_warning_publish_debug");
  return alert;
}

void HazardWarning::publish_debug(
  const HazardWarningSources & hazard_sources, const HazardAlert & alert) const
{
  if (!publish_markers_ || !pub_markers_ || pub_markers_->get_subscription_count() == 0) {
    return;
  }
  MarkerArray markers;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  int32_t id = 0;
  for (const auto & [sources, ns] :
       {std::pair{&hazard_sources.collision, "hazard_collision_sources"},
        std::pair{&hazard_sources.departure, "hazard_departure_sources"}}) {
    if (!sources->has_value()) {
      continue;
    }
    for (const auto & source : **sources) {
      if (!source.shape.outer().empty()) {
        markers.markers.push_back(make_risk_level_marker(source, id, hazard_sources.stamp));
      }
      markers.markers.push_back(make_polygon_marker(
        source.shape, ns, id++, level_color(source.risk_level), hazard_sources.stamp));
    }
  }
  for (size_t s = 0; s < kNumSectors; ++s) {
    if (!alert.sectors[s].has_object) {
      continue;
    }
    markers.markers.push_back(make_polygon_marker(
      wedges_[s], "hazard_sectors", static_cast<int32_t>(s),
      level_color(alert.sectors[s].risk_level), hazard_sources.stamp));
  }
  pub_markers_->publish(markers);
}
}  // namespace autoware::control_validator::hazard
