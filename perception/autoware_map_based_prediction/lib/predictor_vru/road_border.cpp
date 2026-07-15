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

#include "autoware/map_based_prediction/path_cut/footprint_path_cut.hpp"
#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
std::vector<autoware_utils_geometry::LineString2d> collect_candidate_road_border_linestrings(
  const lanelet::LaneletMap & road_border_layer, const PredictedPath & predicted_path,
  const autoware_perception_msgs::msg::Shape & object_shape)
{
  const auto candidates = road_border_layer.lineStringLayer.search(
    path_cut::get_bbox_contain_path_with_footprint(predicted_path, object_shape));
  std::vector<autoware_utils_geometry::LineString2d> linestrings_2d;
  linestrings_2d.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    autoware_utils_geometry::LineString2d linestring;
    boost::geometry::convert(lanelet::utils::to2D(candidate).basicLineString(), linestring);
    linestrings_2d.push_back(linestring);
  }
  return linestrings_2d;
}

double arc_length_to_index(const PredictedPath & path, const size_t index)
{
  const auto & poses = path.path;
  double length = 0.0;
  for (size_t i = 0; i + 1 < poses.size() && i < index; ++i) {
    const double dx = poses.at(i + 1).position.x - poses.at(i).position.x;
    const double dy = poses.at(i + 1).position.y - poses.at(i).position.y;
    length += std::hypot(dx, dy);
  }
  return length;
}

std::optional<lanelet::BasicPoint2d> get_first_border_crossing_point(
  const PredictedPath & predicted_path,
  const std::vector<autoware_utils_geometry::LineString2d> & borders)
{
  const auto & poses = predicted_path.path;
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const autoware_utils_geometry::LineString2d segment{
      {poses.at(i).position.x, poses.at(i).position.y},
      {poses.at(i + 1).position.x, poses.at(i + 1).position.y}};
    for (const auto & border : borders) {
      std::vector<autoware_utils_geometry::Point2d> intersections;
      boost::geometry::intersection(segment, border, intersections);
      if (!intersections.empty()) {
        const auto & p = intersections.front();
        return lanelet::BasicPoint2d(p.x(), p.y());
      }
    }
  }
  return std::nullopt;
}

bool is_point_inside_crosswalk(
  const lanelet::LaneletMap & map, const std::optional<lanelet::BasicPoint2d> & point)
{
  if (!point) {
    return false;
  }
  const lanelet::BoundingBox2d search_box{*point, *point};
  for (const auto & candidate : map.laneletLayer.search(search_box)) {
    const std::string subtype = candidate.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (
      subtype != lanelet::AttributeValueString::Crosswalk &&
      subtype != lanelet::AttributeValueString::Walkway) {
      continue;
    }
    if (lanelet::geometry::inside(candidate, *point)) {
      return true;
    }
  }
  return false;
}
}  // namespace

void RoadBorderModule::build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  road_border_layer_ = nullptr;
  if (!lanelet_map_ptr_) {
    return;
  }

  lanelet::LineStrings3d borders;
  for (const auto & linestring : lanelet_map_ptr_->lineStringLayer) {
    const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "none");
    if (type == "road_border") {
      borders.emplace_back(
        std::const_pointer_cast<lanelet::LineStringData>(linestring.constData()));
    }
  }
  if (!borders.empty()) {
    road_border_layer_ = lanelet::utils::createMap(borders);
  }
}

PredictedPath RoadBorderModule::cut_path_at_road_border(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::TrackedObject & object,
  const path_cut::MaxDecelerationParams & max_decel_params) const
{
  const auto & poses = predicted_path.path;
  if (!road_border_layer_ || poses.size() < 2 || !path_cut::shape_has_footprint(object.shape)) {
    return predicted_path;
  }

  const autoware_perception_msgs::msg::Shape & object_shape = object.shape;
  const std::vector<autoware_utils_geometry::LineString2d> candidates =
    collect_candidate_road_border_linestrings(*road_border_layer_, predicted_path, object_shape);

  const std::optional<size_t> road_border_crossing_index =
    path_cut::find_footprint_crossing_index(predicted_path, object_shape, candidates);
  if (!road_border_crossing_index) {
    return predicted_path;
  }

  const std::optional<lanelet::BasicPoint2d> road_border_crossing_point =
    get_first_border_crossing_point(predicted_path, candidates);
  if (is_point_inside_crosswalk(*lanelet_map_ptr_, road_border_crossing_point)) {
    return predicted_path;
  }

  const auto & v = object.kinematics.twist_with_covariance.twist.linear;
  const double max_deceleration = path_cut::max_deceleration_for_label(
    max_decel_params,
    autoware::object_recognition_utils::getHighestProbLabel(object.classification));
  const double distance_to_cross = arc_length_to_index(predicted_path, *road_border_crossing_index);
  if (!path_cut::can_stop_before_the_line(
        distance_to_cross, std::hypot(v.x, v.y), max_deceleration)) {
    return predicted_path;
  }

  return path_cut::force_cut_at_index(
    predicted_path, std::max<size_t>(*road_border_crossing_index, 1UL) - 1UL);
}

}  // namespace autoware::map_based_prediction
