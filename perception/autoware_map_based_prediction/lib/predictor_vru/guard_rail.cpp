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

#include "autoware/map_based_prediction/predictor_vru/guard_rail.hpp"

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cmath>
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

bool has_required_info(const autoware_perception_msgs::msg::PredictedObject & predicted_object)
{
  using autoware_perception_msgs::msg::Shape;
  if (predicted_object.kinematics.predicted_paths.empty()) {
    return false;
  }
  const auto & shape = predicted_object.shape;
  switch (shape.type) {
    case Shape::BOUNDING_BOX:
      return shape.dimensions.x > 0.0 && shape.dimensions.y > 0.0;
    case Shape::CYLINDER:
      return shape.dimensions.x > 0.0;
    case Shape::POLYGON:
      return !shape.footprint.points.empty();
    default:
      return false;
  }
}

std::vector<autoware_utils_geometry::LineString2d> collect_candidate_guard_rail_linestrings(
  const lanelet::LaneletMap & guard_rail_layer, const std::vector<PredictedPath> & predicted_paths,
  const autoware_perception_msgs::msg::Shape & object_shape)
{
  lanelet::BoundingBox2d search_bbox;
  const auto search_margin = calc_footprint_search_margin(object_shape);
  const lanelet::BasicPoint2d offset(search_margin, search_margin);
  for (const auto & predicted_path : predicted_paths) {
    for (const auto & pose : predicted_path.path) {
      const lanelet::BasicPoint2d center(pose.position.x, pose.position.y);
      search_bbox.extend(center - offset);
      search_bbox.extend(center + offset);
    }
  }

  const auto candidates = guard_rail_layer.lineStringLayer.search(search_bbox);
  std::vector<autoware_utils_geometry::LineString2d> guard_rail_linestrings_2d;
  guard_rail_linestrings_2d.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    autoware_utils_geometry::LineString2d linestring;
    boost::geometry::convert(lanelet::utils::to2D(candidate).basicLineString(), linestring);
    guard_rail_linestrings_2d.push_back(linestring);
  }
  return guard_rail_linestrings_2d;
}

std::optional<size_t> find_guard_rail_crossing_index(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape,
  const std::vector<autoware_utils_geometry::LineString2d> & guard_rail_linestrings_2d)
{
  if (guard_rail_linestrings_2d.empty()) {
    return std::nullopt;
  }
  for (auto i = 0UL; i < predicted_path.path.size(); ++i) {
    const autoware_utils_geometry::Polygon2d footprint =
      autoware_utils_geometry::to_polygon2d(predicted_path.path.at(i), object_shape);
    for (const auto & guard_rail_linestring : guard_rail_linestrings_2d) {
      if (boost::geometry::intersects(footprint, guard_rail_linestring)) {
        return i;
      }
    }
  }
  return std::nullopt;
}
}  // namespace

void GuardRailModule::build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  if (!lanelet_map_ptr) {
    guard_rail_layer_ = nullptr;
    return;
  }

  lanelet::LineStrings3d guard_rails;
  for (const auto & linestring : lanelet_map_ptr->lineStringLayer) {
    const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "none");
    if (type == "guard_rail") {
      guard_rails.emplace_back(
        std::const_pointer_cast<lanelet::LineStringData>(linestring.constData()));
    }
  }
  guard_rail_layer_ = lanelet::utils::createMap(guard_rails);
}

std::vector<PredictedPath> GuardRailModule::cut_paths_crossing_guard_rail(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object) const
{
  std::vector<PredictedPath> cut_paths = predicted_object.kinematics.predicted_paths;
  if (!has_required_info(predicted_object) || !guard_rail_layer_) {
    return cut_paths;
  }

  const autoware_perception_msgs::msg::Shape & object_shape = predicted_object.shape;
  const std::vector<autoware_utils_geometry::LineString2d> candidate_linestrings =
    collect_candidate_guard_rail_linestrings(*guard_rail_layer_, cut_paths, object_shape);

  for (PredictedPath & predicted_path : cut_paths) {
    const std::optional<size_t> crossing_index =
      find_guard_rail_crossing_index(predicted_path, object_shape, candidate_linestrings);
    if (crossing_index) {
      // Drop the crossing pose and beyond, keeping at least one pose.
      const size_t last_kept_index = std::max<size_t>(*crossing_index, 1UL) - 1UL;
      predicted_path = path_cut::force_cut_at_index(predicted_path, last_kept_index);
    }
  }
  return cut_paths;
}
}  // namespace autoware::map_based_prediction
