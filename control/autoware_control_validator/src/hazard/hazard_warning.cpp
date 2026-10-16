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

#include "hazard/hazard_warning.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_validator::hazard
{
namespace
{
constexpr double kWedgeArcStepDeg = 10.0;

double deg2rad(const double deg)
{
  return deg * M_PI / 180.0;
}
}  // namespace

HazardWarning::HazardWarning(
  const rclcpp::Logger & logger, const HazardWarningParam & param,
  const vehicle_info_utils::VehicleInfo & vehicle_info)
: logger_(logger), param_(param), vehicle_info_(vehicle_info)
{
  update_parameters(param);
}

void HazardWarning::update_parameters(const HazardWarningParam & param)
{
  param_ = param;
  enabled_ = validate_parameters();
  build_wedges();
}

bool HazardWarning::validate_parameters() const
{
  const auto & p = param_;
  if (!std::isfinite(p.ttc_threshold_on) || p.ttc_threshold_on <= 0.0) {
    RCLCPP_ERROR(
      logger_, "hazard_warning.ttc_threshold_on is undecided or invalid (%f). Warning disabled.",
      p.ttc_threshold_on);
    return false;
  }
  if (p.ttc_thresholds.empty()) {
    RCLCPP_ERROR(logger_, "hazard_warning.ttc_thresholds is empty. Warning disabled.");
    return false;
  }
  for (const auto th : p.ttc_thresholds) {
    if (!std::isfinite(th) || th <= 0.0) {
      RCLCPP_ERROR(
        logger_,
        "hazard_warning.ttc_thresholds contains an undecided or invalid value (%f). "
        "Warning disabled.",
        th);
      return false;
    }
  }
  if (std::abs(p.ttc_thresholds.front() - p.ttc_threshold_on) > 1e-9) {
    RCLCPP_ERROR(
      logger_,
      "hazard_warning.ttc_thresholds[0] (%f) must equal ttc_threshold_on (%f). "
      "Warning disabled.",
      p.ttc_thresholds.front(), p.ttc_threshold_on);
    return false;
  }
  for (size_t i = 1; i < p.ttc_thresholds.size(); ++i) {
    if (p.ttc_thresholds[i] >= p.ttc_thresholds[i - 1]) {
      RCLCPP_ERROR(
        logger_, "hazard_warning.ttc_thresholds must be strictly descending. Warning disabled.");
      return false;
    }
  }
  if (
    !std::isfinite(p.collision_cos_threshold) || p.collision_cos_threshold < -1.0 ||
    p.collision_cos_threshold > 1.0) {
    RCLCPP_ERROR(
      logger_, "hazard_warning.collision_cos_threshold must be in [-1, 1] (%f). Warning disabled.",
      p.collision_cos_threshold);
    return false;
  }
  if (!std::isfinite(p.min_alert_duration) || p.min_alert_duration < 0.0) {
    RCLCPP_ERROR(logger_, "hazard_warning.min_alert_duration must be >= 0. Warning disabled.");
    return false;
  }
  if (!std::isfinite(p.rear_axle_ratio) || p.rear_axle_ratio < 0.0 || p.rear_axle_ratio > 0.5) {
    RCLCPP_ERROR(
      logger_,
      "hazard_warning.vehicle_yaw.rear_axle_ratio must be in [0, 0.5] (%f). Warning disabled.",
      p.rear_axle_ratio);
    return false;
  }
  if (!std::isfinite(p.sector_radius_m) || p.sector_radius_m <= 0.0) {
    RCLCPP_ERROR(logger_, "hazard_warning.sector.radius_m must be > 0. Warning disabled.");
    return false;
  }
  return true;
}

void HazardWarning::build_wedges()
{
  // Same construction as autoware_proximity_hazard_checker: the wedge apex is the centroid of the
  // vehicle footprint (base_link), the angular range is widened by the boundary tolerance. The arc
  // is polygonised so that the whole radius is covered for wide sectors.
  const double center_x =
    0.5 * (vehicle_info_.max_longitudinal_offset_m + vehicle_info_.min_longitudinal_offset_m);
  const double center_y = 0.5 * (vehicle_info_.left_overhang_m - vehicle_info_.right_overhang_m);
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
    boost::geometry::append(wedge, Point2d{center_x, center_y});
    for (size_t i = 0; i <= num_arc_steps; ++i) {
      const double angle =
        deg2rad(from_deg + (to_deg - from_deg) * static_cast<double>(i) / num_arc_steps);
      boost::geometry::append(
        wedge, Point2d{center_x + radius * std::cos(angle), center_y + radius * std::sin(angle)});
    }
    boost::geometry::append(wedge, Point2d{center_x, center_y});
    boost::geometry::correct(wedge);
    wedges_[s] = wedge;
  }
}

uint8_t HazardWarning::level_of(const double ttc) const
{
  const auto count = std::count_if(
    param_.ttc_thresholds.begin(), param_.ttc_thresholds.end(),
    [ttc](const double th) { return ttc < th; });
  return static_cast<uint8_t>(std::min<std::ptrdiff_t>(count, 255));
}

Polygon2d HazardWarning::to_base_link(
  const Polygon2d & polygon_map, const geometry_msgs::msg::Pose & ego_pose)
{
  Polygon2d polygon_base;
  polygon_base.outer().reserve(polygon_map.outer().size());
  for (const auto & pt : polygon_map.outer()) {
    const auto local = autoware_utils_geometry::inverse_transform_point(
      Eigen::Vector3d{pt.x(), pt.y(), 0.0}, ego_pose);
    polygon_base.outer().emplace_back(local.x(), local.y());
  }
  boost::geometry::correct(polygon_base);
  return polygon_base;
}

bool HazardWarning::passes_thresholds(const SweepCandidate & candidate) const
{
  if (!candidate.ttc.has_value() || !(*candidate.ttc < param_.ttc_threshold_on)) {
    return false;
  }
  if (param_.collision_cos_threshold <= -1.0) {
    return true;
  }
  return candidate.cos_collision.has_value() &&
         *candidate.cos_collision >= param_.collision_cos_threshold;
}

HazardAlert HazardWarning::run(
  const CollisionEvaluation & evaluation,
  const std::optional<boundary_departure_checker::DepartureResult> & departure,
  const nav_msgs::msg::Odometry & odom, const rclcpp::Time & now,
  std::vector<CandidateJudgement> * judgements)
{
  HazardAlert alert;
  alert.header.stamp = now;
  alert.header.frame_id = "base_link";

  std::array<bool, kNumSectors> raw{};
  std::array<double, kNumSectors> ttc_min{};
  ttc_min.fill(std::numeric_limits<double>::infinity());

  if (judgements) {
    judgements->assign(evaluation.candidates.size(), CandidateJudgement{});
  }
  if (enabled_) {
    for (size_t i = 0; i < evaluation.candidates.size(); ++i) {
      const auto & candidate = evaluation.candidates[i];
      if (!passes_thresholds(candidate)) {
        continue;
      }
      const auto polygon_base = to_base_link(candidate.polygon_now, odom.pose.pose);
      const bool is_point = candidate.kind == SweepCandidate::Kind::POINTCLOUD;
      for (size_t s = 0; s < kNumSectors; ++s) {
        const bool in_sector = is_point
                                 ? boost::geometry::within(polygon_base.outer().front(), wedges_[s])
                                 : boost::geometry::intersects(polygon_base, wedges_[s]);
        if (!in_sector) {
          continue;
        }
        raw[s] = true;
        ttc_min[s] = std::min(ttc_min[s], *candidate.ttc);
        if (judgements) {
          (*judgements)[i].raised = true;
          (*judgements)[i].sectors.push_back(static_cast<uint8_t>(s));
          (*judgements)[i].level = level_of(*candidate.ttc);
        }
      }
    }
  }

  for (size_t s = 0; s < kNumSectors; ++s) {
    const bool on = sector_latches_[s].update(raw[s], now, param_.min_alert_duration);
    alert.has_object[s] = on;
    if (raw[s]) {
      alert.ttc_s[s] = static_cast<float>(ttc_min[s]);
      last_level_[s] = level_of(ttc_min[s]);
    } else {
      alert.ttc_s[s] = std::numeric_limits<float>::quiet_NaN();
      if (!on) {
        last_level_[s] = 0;
      }
    }
    alert.level[s] = last_level_[s];
  }

  const bool raw_departure =
    enabled_ && param_.enable_road_border && departure.has_value() &&
    (departure->status == boundary_departure_checker::DepartureType::CAUTION ||
     departure->status == boundary_departure_checker::DepartureType::CRITICAL);
  alert.is_boundary_departure =
    departure_latch_.update(raw_departure, now, param_.min_alert_duration);
  alert.boundary_lateral_distance_m =
    departure.has_value() ? static_cast<float>(departure->lat_dist_to_uncrossable_bound)
                          : std::numeric_limits<float>::quiet_NaN();

  return alert;
}

namespace
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

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
  const Polygon2d & polygon, const std::string & ns, const int32_t id, const double z,
  const std_msgs::msg::ColorRGBA & color, const builtin_interfaces::msg::Time & stamp)
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.color = color;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  for (const auto & pt : polygon.outer()) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = z;
    marker.points.push_back(p);
  }
  return marker;
}

Polygon2d wedge_to_map(const Polygon2d & wedge, const geometry_msgs::msg::Pose & ego_pose)
{
  Polygon2d polygon_map;
  polygon_map.outer().reserve(wedge.outer().size());
  for (const auto & pt : wedge.outer()) {
    const auto global =
      autoware_utils_geometry::transform_point(Eigen::Vector3d{pt.x(), pt.y(), 0.0}, ego_pose);
    polygon_map.outer().emplace_back(global.x(), global.y());
  }
  return polygon_map;
}
}  // namespace

HazardCandidateArray create_candidate_array(
  const CollisionEvaluation & evaluation, const std::vector<CandidateJudgement> & judgements,
  const rclcpp::Time & now)
{
  HazardCandidateArray array;
  array.header.stamp = now;
  array.header.frame_id = "map";
  array.candidates.reserve(evaluation.candidates.size());
  for (size_t i = 0; i < evaluation.candidates.size(); ++i) {
    const auto & candidate = evaluation.candidates[i];
    HazardCandidate out;
    out.source_stamp = candidate.source_stamp;
    out.kind = candidate.kind == SweepCandidate::Kind::OBJECT ? HazardCandidate::KIND_OBJECT
                                                              : HazardCandidate::KIND_POINTCLOUD;
    out.label = candidate.label;
    out.ttc_s =
      candidate.ttc ? static_cast<float>(*candidate.ttc) : std::numeric_limits<float>::quiet_NaN();
    out.cos_collision = candidate.cos_collision ? static_cast<float>(*candidate.cos_collision)
                                                : std::numeric_limits<float>::quiet_NaN();
    if (i < judgements.size()) {
      out.sectors = judgements[i].sectors;
      out.level = judgements[i].level;
    }
    array.candidates.push_back(std::move(out));
  }
  return array;
}

MarkerArray create_hazard_markers(
  const CollisionEvaluation & evaluation, const std::vector<CandidateJudgement> & judgements,
  const HazardAlert & alert, const std::array<Polygon2d, kNumSectors> & wedges,
  const nav_msgs::msg::Odometry & odom)
{
  MarkerArray markers;
  const auto & now = alert.header.stamp;
  const double z = odom.pose.pose.position.z;

  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  std::array<std::optional<size_t>, kNumSectors> min_ttc_point{};
  for (size_t i = 0; i < evaluation.candidates.size(); ++i) {
    const auto & candidate = evaluation.candidates[i];
    if (candidate.kind != SweepCandidate::Kind::POINTCLOUD || !candidate.ttc.has_value()) {
      continue;
    }
    const auto point_base =
      HazardWarning::to_base_link(candidate.polygon_now, odom.pose.pose).outer().front();
    for (size_t s = 0; s < kNumSectors; ++s) {
      if (!boost::geometry::within(point_base, wedges[s])) {
        continue;
      }
      auto & selected = min_ttc_point[s];
      if (!selected || *candidate.ttc < *evaluation.candidates[*selected].ttc) {
        selected = i;
      }
    }
  }
  std::vector<bool> draw_point(evaluation.candidates.size(), false);
  for (const auto & selected : min_ttc_point) {
    if (selected) {
      draw_point[*selected] = true;
    }
  }

  int32_t id = 0;
  for (size_t i = 0; i < evaluation.candidates.size(); ++i) {
    const auto & candidate = evaluation.candidates[i];
    const uint8_t level = i < judgements.size() ? judgements[i].level : 0;
    const auto color = level_color(level);
    const auto & stamp = candidate.source_stamp;
    const bool is_point = candidate.kind == SweepCandidate::Kind::POINTCLOUD;

    if (is_point && !draw_point[i]) {
      ++id;
      continue;
    }

    if (is_point) {
      Marker sphere;
      sphere.header.frame_id = "map";
      sphere.header.stamp = stamp;
      sphere.ns = "hazard_hit_points";
      sphere.id = id;
      sphere.type = Marker::SPHERE;
      sphere.action = Marker::ADD;
      sphere.pose.position.x = candidate.polygon_now.outer().front().x();
      sphere.pose.position.y = candidate.polygon_now.outer().front().y();
      sphere.pose.position.z = z + 0.1;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.3;
      sphere.scale.y = 0.3;
      sphere.scale.z = 0.3;
      sphere.color = color;
      sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
      markers.markers.push_back(sphere);
    } else {
      markers.markers.push_back(
        make_polygon_marker(candidate.polygon_now, "hazard_objects", id, z + 0.1, color, stamp));
    }

    if (candidate.ttc.has_value()) {
      Marker text;
      text.header.frame_id = "map";
      text.header.stamp = stamp;
      text.ns = "hazard_ttc";
      text.id = id;
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      Point2d centroid = candidate.polygon_now.outer().front();
      if (!is_point) {
        boost::geometry::centroid(candidate.polygon_now, centroid);
      }
      text.pose.position.x = centroid.x();
      text.pose.position.y = centroid.y();
      text.pose.position.z = z + 1.0;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.5;
      text.color = color;
      text.lifetime = rclcpp::Duration::from_seconds(0.5);
      char buffer[48];
      std::snprintf(
        buffer, sizeof(buffer), "ttc %.1f cos %.2f", *candidate.ttc,
        candidate.cos_collision.value_or(std::numeric_limits<double>::quiet_NaN()));
      text.text = buffer;
      markers.markers.push_back(text);
    }
    ++id;
  }

  for (size_t s = 0; s < kNumSectors; ++s) {
    if (!alert.has_object[s]) {
      continue;
    }
    markers.markers.push_back(make_polygon_marker(
      wedge_to_map(wedges[s], odom.pose.pose), "hazard_sectors", static_cast<int32_t>(s), z + 0.2,
      level_color(alert.level[s]), now));
  }

  return markers;
}
}  // namespace autoware::control_validator::hazard
