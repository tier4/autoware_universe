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
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/BoundingBox.h>
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
double calc_footprint_search_margin(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto hx = shape.dimensions.x * 0.5;
    const auto hy = shape.dimensions.y * 0.5;
    return std::hypot(hx, hy);
  }
  if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return shape.dimensions.x * 0.5;
  }
  return std::max(shape.dimensions.x, shape.dimensions.y) * 0.5;
}

std::vector<autoware_utils_geometry::LineString2d> collect_candidate_road_border_linestrings(
  const lanelet::LaneletMap & road_border_layer, const PredictedPath & predicted_path,
  const autoware_perception_msgs::msg::Shape & object_shape)
{
  lanelet::BoundingBox2d search_bbox;
  const auto search_margin = calc_footprint_search_margin(object_shape);
  const lanelet::BasicPoint2d offset(search_margin, search_margin);
  for (const auto & pose : predicted_path.path) {
    const lanelet::BasicPoint2d center(pose.position.x, pose.position.y);
    search_bbox.extend(center - offset);
    search_bbox.extend(center + offset);
  }

  const auto candidates = road_border_layer.lineStringLayer.search(search_bbox);
  std::vector<autoware_utils_geometry::LineString2d> linestrings_2d;
  linestrings_2d.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    autoware_utils_geometry::LineString2d linestring;
    boost::geometry::convert(lanelet::utils::to2D(candidate).basicLineString(), linestring);
    linestrings_2d.push_back(linestring);
  }
  return linestrings_2d;
}

bool point_near_crosswalk(const lanelet::LaneletMap & map, const lanelet::BasicPoint2d & point)
{
  // Tolerance so a crossing that lands just outside a crosswalk polygon still counts as inside it:
  // the road_border linestring and the crosswalk boundary rarely share exact vertices.
  constexpr double margin = 1.0;
  const lanelet::BoundingBox2d search_box{
    point - lanelet::BasicPoint2d{margin, margin}, point + lanelet::BasicPoint2d{margin, margin}};
  for (const auto & candidate : map.laneletLayer.search(search_box)) {
    const std::string subtype = candidate.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (
      subtype != lanelet::AttributeValueString::Crosswalk &&
      subtype != lanelet::AttributeValueString::Walkway) {
      continue;
    }
    if (
      lanelet::geometry::inside(candidate, point) ||
      boost::geometry::distance(point, candidate.polygon2d()) <= margin) {
      return true;
    }
  }
  return false;
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
  if (!borders.empty()) {
    road_border_layer_ = lanelet::utils::createMap(borders);
  }
}

PredictedPath RoadBorderModule::cut_path_at_road_border(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::TrackedObject & object,
  const path_cut::MaxDecelerationParams & max_decel_params) const
{
  const auto & poses = predicted_path.path;
  if (!road_border_layer_ || poses.size() < 2) {
    return predicted_path;
  }

  const autoware_perception_msgs::msg::Shape & object_shape = object.shape;
  const std::vector<autoware_utils_geometry::LineString2d> candidates =
    collect_candidate_road_border_linestrings(*road_border_layer_, predicted_path, object_shape);
  if (candidates.empty()) {
    return predicted_path;
  }

  double distance_to_cross = 0.0;  // arc length up to poses.at(i), where the footprint first crosses
  for (size_t i = 0; i < poses.size(); ++i) {
    const autoware_utils_geometry::Polygon2d footprint =
      autoware_utils_geometry::to_polygon2d(poses.at(i), object_shape);
    const bool crosses =
      std::any_of(candidates.begin(), candidates.end(), [&](const auto & border) {
        return boost::geometry::intersects(footprint, border);
      });

    if (crosses) {
      const lanelet::BasicPoint2d crossing(poses.at(i).position.x, poses.at(i).position.y);
      if (lanelet_map_ptr_ && point_near_crosswalk(*lanelet_map_ptr_, crossing)) {
        return predicted_path;
      }
      const auto & v = object.kinematics.twist_with_covariance.twist.linear;
      const double max_deceleration = path_cut::max_deceleration_for_label(
        max_decel_params,
        autoware::object_recognition_utils::getHighestProbLabel(object.classification));
      if (!path_cut::can_stop_before_the_line(
            distance_to_cross, std::hypot(v.x, v.y), max_deceleration)) {
        return predicted_path;
      }
      // Drop the crossing pose and beyond, keeping at least one pose.
      return path_cut::force_cut_at_index(predicted_path, std::max<size_t>(i, 1UL) - 1UL);
    }
    if (i + 1 < poses.size()) {
      const lanelet::BasicPoint2d here(poses.at(i).position.x, poses.at(i).position.y);
      const lanelet::BasicPoint2d next(poses.at(i + 1).position.x, poses.at(i + 1).position.y);
      distance_to_cross += (next - here).norm();
    }
  }
  return predicted_path;
}

}  // namespace autoware::map_based_prediction
