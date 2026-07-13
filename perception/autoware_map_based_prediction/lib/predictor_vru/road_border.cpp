// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/predictor_vru/road_border.hpp"

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
// A crossing point together with its planar distance from the segment start, so the caller can pick
// the earliest crossing without recomputing the distance.
struct SegmentCrossing
{
  double distance_from_start{0.0};
  geometry_msgs::msg::Point point{};
};

std::optional<SegmentCrossing> nearest_segment_crossing(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b,
  const lanelet::ConstLineString3d & border)
{
  std::optional<SegmentCrossing> nearest;
  for (size_t j = 1; j < border.size(); ++j) {
    const auto c1 = lanelet::utils::conversion::toGeomMsgPt(border[j - 1]);
    const auto c2 = lanelet::utils::conversion::toGeomMsgPt(border[j]);
    if (const auto crossing = autoware_utils::intersect(a, b, c1, c2)) {
      const double d = std::hypot(crossing->x - a.x, crossing->y - a.y);
      if (!nearest || d < nearest->distance_from_start) {
        nearest = SegmentCrossing{d, *crossing};
      }
    }
  }
  return nearest;
}

bool point_near_crosswalk(const lanelet::LaneletMap & map, const geometry_msgs::msg::Point & p)
{
  // Tolerance so a crossing that lands just outside a crosswalk polygon still counts as inside it:
  // the road_border linestring and the crosswalk boundary rarely share exact vertices.
  constexpr double margin = 1.0;
  const lanelet::BasicPoint2d point2d{p.x, p.y};
  const lanelet::BoundingBox2d search_box{
    lanelet::BasicPoint2d{p.x - margin, p.y - margin},
    lanelet::BasicPoint2d{p.x + margin, p.y + margin}};
  const auto candidates = map.laneletLayer.search(search_box);
  for (const auto & candidate : candidates) {
    const std::string subtype = candidate.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (
      subtype != lanelet::AttributeValueString::Crosswalk &&
      subtype != lanelet::AttributeValueString::Walkway) {
      continue;
    }
    if (
      lanelet::geometry::inside(candidate, point2d) ||
      boost::geometry::distance(point2d, candidate.polygon2d()) <= margin) {
      return true;
    }
  }
  return false;
}

double object_speed(const autoware_perception_msgs::msg::TrackedObject & object)
{
  const auto & v = object.kinematics.twist_with_covariance.twist.linear;
  return std::hypot(v.x, v.y);
}

lanelet::BasicLineString2d to_basic_path_2d(const PredictedPath & path)
{
  lanelet::BasicLineString2d ls;
  ls.reserve(path.path.size());
  for (const auto & pose : path.path) {
    ls.emplace_back(pose.position.x, pose.position.y);
  }
  return ls;
}
}  // namespace

void RoadBorderModule::build_from_map(
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
  const std::vector<std::string> & boundary_types)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  road_border_layer_ = nullptr;
  if (!lanelet_map_ptr_) {
    return;
  }

  lanelet::LineStrings3d borders;
  for (const auto & linestring : lanelet_map_ptr_->lineStringLayer) {
    const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "none");
    if (std::find(boundary_types.begin(), boundary_types.end(), type) != boundary_types.end()) {
      borders.emplace_back(
        std::const_pointer_cast<lanelet::LineStringData>(linestring.constData()));
    }
  }
  if (borders.empty()) {
    return;
  }
  road_border_layer_ = lanelet::utils::createMap(borders);
}

PredictedPath RoadBorderModule::cut_path_at_road_border(
  const PredictedPath & predicted_path,
  const autoware_perception_msgs::msg::TrackedObject & object,
  const path_cut::MaxDecelerationParams & max_decel_params) const
{
  if (!road_border_layer_ || predicted_path.path.size() < 2) {
    return predicted_path;
  }

  const auto path_ls = to_basic_path_2d(predicted_path);
  const auto candidates =
    road_border_layer_->lineStringLayer.search(lanelet::geometry::boundingBox2d(path_ls));

  std::optional<size_t> cut_index;
  double distance_to_line = 0.0;
  geometry_msgs::msg::Point crossing_point;
  double arc_length = 0.0;
  for (size_t seg = 0; seg + 1 < predicted_path.path.size() && !cut_index; ++seg) {
    const auto & a = predicted_path.path.at(seg).position;
    const auto & b = predicted_path.path.at(seg + 1).position;
    std::optional<SegmentCrossing> earliest;
    for (const auto & border : candidates) {
      const auto crossing = nearest_segment_crossing(a, b, border);
      if (crossing && (!earliest || crossing->distance_from_start < earliest->distance_from_start)) {
        earliest = crossing;
      }
    }
    if (earliest) {
      cut_index = seg;
      distance_to_line = std::max(arc_length + earliest->distance_from_start, 0.0);
      crossing_point = earliest->point;
    }
    arc_length += std::hypot(b.x - a.x, b.y - a.y);
  }

  if (!cut_index) {
    return predicted_path;
  }
  if (lanelet_map_ptr_ && point_near_crosswalk(*lanelet_map_ptr_, crossing_point)) {
    return predicted_path;
  }

  const double speed = object_speed(object);
  const uint8_t label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  const double max_deceleration = path_cut::max_deceleration_for_label(max_decel_params, label);
  if (!path_cut::can_stop_before_the_line(distance_to_line, speed, max_deceleration)) {
    return predicted_path;
  }

  return path_cut::force_cut_at_index(predicted_path, cut_index.value());
}

}  // namespace autoware::map_based_prediction
