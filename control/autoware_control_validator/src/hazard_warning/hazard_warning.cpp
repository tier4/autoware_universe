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

namespace autoware::control_validator
{
namespace
{
using autoware_control_validator::msg::HazardAlert;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;
using hazard_warning::HazardWarningParam;
using hazard_warning::kNumSectors;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kWedgeArcStepDeg = 10.0;
constexpr double kWedgeRadius = 60.0;
constexpr double kSectorBoundaryToleranceDeg = 1.0;
constexpr std::array<std::array<double, 2>, kNumSectors> kSectorRanges{{
  {-10.0, 10.0},
  {-30.0, -10.0},
  {-150.0, -30.0},
  {-170.0, -150.0},
  {170.0, -170.0},
  {150.0, 170.0},
  {30.0, 150.0},
  {10.0, 30.0},
}};

double deg2rad(const double deg)
{
  return deg * M_PI / 180.0;
}

double normalize_deg(const double deg)
{
  const double normalized = std::fmod(deg, 360.0);
  return normalized < 0.0 ? normalized + 360.0 : normalized;
}

struct BearingArc
{
  double start_deg;
  double length_deg;
};

bool overlaps(const BearingArc & a, const BearingArc & b)
{
  return normalize_deg(b.start_deg - a.start_deg) <= a.length_deg ||
         normalize_deg(a.start_deg - b.start_deg) <= b.length_deg;
}

std::optional<BearingArc> bearing_arc(const Polygon2d & shape, const Point2d & apex)
{
  const auto & ring = shape.outer();
  if (ring.size() >= 3 && boost::geometry::covered_by(apex, shape)) {
    return std::nullopt;
  }
  std::vector<double> bearings;
  bearings.reserve(ring.size());
  for (const auto & vertex : ring) {
    bearings.push_back(
      normalize_deg(std::atan2(vertex.y() - apex.y(), vertex.x() - apex.x()) * 180.0 / M_PI));
  }
  std::sort(bearings.begin(), bearings.end());
  double max_gap = 360.0 - (bearings.back() - bearings.front());
  size_t start_index = 0;
  for (size_t i = 1; i < bearings.size(); ++i) {
    const double gap = bearings[i] - bearings[i - 1];
    if (gap > max_gap) {
      max_gap = gap;
      start_index = i;
    }
  }
  return BearingArc{bearings[start_index], 360.0 - max_gap};
}

BearingArc sector_arc(const size_t sector)
{
  const double from_deg = kSectorRanges[sector][0] - kSectorBoundaryToleranceDeg;
  double to_deg = kSectorRanges[sector][1] + kSectorBoundaryToleranceDeg;
  while (to_deg <= from_deg) {
    to_deg += 360.0;
  }
  return BearingArc{normalize_deg(from_deg), std::min(to_deg - from_deg, 360.0)};
}

HazardWarningParam to_hazard_warning_param(
  const ::control_validator::Params::HazardWarning & params)
{
  HazardWarningParam out;
  out.enable = params.enable;
  out.min_alert_duration = params.min_alert_duration;
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
      return make_color(0.0F, 0.8F, 1.0F, 0.9F);
    case 2:
      return make_color(1.0F, 1.0F, 0.0F, 0.9F);
    case 3:
      return make_color(1.0F, 0.5F, 0.0F, 0.9F);
    default:
      return make_color(1.0F, 0.0F, 0.0F, 0.9F);
  }
}

Marker make_polygon_marker(
  const Polygon2d & polygon, const std::string & ns, const int32_t id,
  const std_msgs::msg::ColorRGBA & color, const builtin_interfaces::msg::Time & stamp,
  const bool frame_locked)
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
  marker.frame_locked = frame_locked;
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
  const HazardSource & source, const std::string & ns, const int32_t id,
  const builtin_interfaces::msg::Time & stamp, const bool frame_locked)
{
  Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = stamp;
  marker.ns = ns;
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
  marker.frame_locked = frame_locked;
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
  param_ = to_hazard_warning_param(params.hazard_warning);
}

void HazardWarning::build_wedges(const Point2d & apex)
{
  // The wedge apex is the centroid of the vehicle footprint (base_link), the angular range is
  // widened by the boundary tolerance. The arc is polygonised so that the whole radius is covered
  // for wide sectors.
  wedge_apex_ = apex;
  for (size_t sector = 0; sector < kNumSectors; ++sector) {
    const auto arc = sector_arc(sector);
    const double from_deg = arc.start_deg;
    const double to_deg = from_deg + arc.length_deg;
    const auto num_arc_steps =
      std::max<size_t>(1, static_cast<size_t>(std::ceil((to_deg - from_deg) / kWedgeArcStepDeg)));

    Polygon2d wedge;
    boost::geometry::append(wedge, apex);
    for (size_t i = 0; i <= num_arc_steps; ++i) {
      const double angle =
        deg2rad(from_deg + (to_deg - from_deg) * static_cast<double>(i) / num_arc_steps);
      boost::geometry::append(
        wedge,
        Point2d{
          apex.x() + kWedgeRadius * std::cos(angle), apex.y() + kWedgeRadius * std::sin(angle)});
    }
    boost::geometry::append(wedge, apex);
    boost::geometry::correct(wedge);
    wedges_[sector] = wedge;
  }
}

void HazardWarning::assign_sectors(
  const HazardWarningSources & hazard_sources, const Point2d & apex,
  std::array<uint8_t, kNumSectors> & max_level,
  std::array<HazardWarningSources, kNumSectors> & sector_sources) const
{
  max_level.fill(0);
  const auto assign = [&](
                        const std::optional<std::vector<HazardSource>> & sources,
                        std::optional<std::vector<HazardSource>> HazardWarningSources::* member) {
    if (!sources) {
      return;
    }
    for (const auto & source : *sources) {
      if (source.shape.outer().empty()) {
        continue;
      }
      const auto arc = bearing_arc(source.shape, apex);
      for (size_t sector = 0; sector < kNumSectors; ++sector) {
        if (!arc || overlaps(*arc, sector_arc(sector))) {
          max_level[sector] = std::max(max_level[sector], source.risk_level);
          auto & assigned = sector_sources[sector].*member;
          if (!assigned) {
            assigned.emplace();
          }
          assigned->push_back(source);
        }
      }
    }
  };
  assign(hazard_sources.collision, &HazardWarningSources::collision);
  assign(hazard_sources.departure, &HazardWarningSources::departure);
}

std::optional<HazardAlert> HazardWarning::make_alert(
  const HazardWarningSources & hazard_sources, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const rclcpp::Time & now)
{
  if (
    !param_.enable ||
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
  std::array<HazardWarningSources, kNumSectors> sector_sources;
  assign_sectors(hazard_sources, apex, max_level, sector_sources);

  HazardAlert alert;
  alert.header.stamp = now;
  alert.header.frame_id = "base_link";
  for (size_t sector = 0; sector < kNumSectors; ++sector) {
    const auto level =
      sector_latches_[sector].update(max_level[sector], now, param_.min_alert_duration);
    alert.sectors[sector].risk_level = level;
    if (max_level[sector] >= 1) {
      last_on_sources_[sector] = std::move(sector_sources[sector]);
    } else if (level == 0) {
      last_on_sources_[sector].collision.reset();
      last_on_sources_[sector].departure.reset();
    }
  }

  time_keeper_->start_track("hazard_warning_publish_debug");
  publish_debug(alert);
  time_keeper_->end_track("hazard_warning_publish_debug");
  return alert;
}

void HazardWarning::publish_debug(const HazardAlert & alert) const
{
  if (!pub_markers_ || pub_markers_->get_subscription_count() == 0) {
    return;
  }
  MarkerArray markers;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  int32_t id = 0;
  const auto draw_sources = [&](
                              const std::optional<std::vector<HazardSource>> & sources,
                              const std::string & sources_ns, const std::string & risk_level_ns) {
    if (!sources) {
      return;
    }
    for (const auto & source : *sources) {
      if (!source.shape.outer().empty()) {
        markers.markers.push_back(
          make_risk_level_marker(source, risk_level_ns, id, source.stamp, false));
      }
      markers.markers.push_back(make_polygon_marker(
        source.shape, sources_ns, id++, level_color(source.risk_level), source.stamp, false));
    }
  };
  for (size_t sector = 0; sector < kNumSectors; ++sector) {
    if (alert.sectors[sector].risk_level < 1) {
      continue;
    }
    draw_sources(last_on_sources_[sector].collision, "collision_sources", "collision_risk_level");
    draw_sources(last_on_sources_[sector].departure, "departure_sources", "departure_risk_level");
    markers.markers.push_back(make_polygon_marker(
      wedges_[sector], "hazard_sectors", static_cast<int32_t>(sector),
      level_color(alert.sectors[sector].risk_level), alert.header.stamp, true));
  }
  pub_markers_->publish(markers);
}
}  // namespace autoware::control_validator
