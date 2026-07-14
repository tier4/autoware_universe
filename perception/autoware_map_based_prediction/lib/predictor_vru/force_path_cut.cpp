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

#include "autoware/map_based_prediction/predictor_vru/force_path_cut.hpp"

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Polygon.h>

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

lanelet::BasicLineString2d to_basic_line_string(const std::vector<geometry_msgs::msg::Pose> & poses)
{
  lanelet::BasicLineString2d path_ls;
  path_ls.reserve(poses.size());
  for (const auto & pose : poses) {
    path_ls.emplace_back(pose.position.x, pose.position.y);
  }
  return path_ls;
}

std::vector<autoware_utils_geometry::Polygon2d> collect_candidate_polygons(
  const lanelet::LaneletMap & polygon_layer, const std::vector<PredictedPath> & predicted_paths,
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

  const auto candidates = polygon_layer.polygonLayer.search(search_bbox);
  std::vector<autoware_utils_geometry::Polygon2d> polygons_2d;
  polygons_2d.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    autoware_utils_geometry::Polygon2d polygon;
    boost::geometry::convert(lanelet::utils::to2D(candidate.basicPolygon()), polygon);
    boost::geometry::correct(polygon);
    polygons_2d.push_back(polygon);
  }
  return polygons_2d;
}

// Index of the last pose to keep when the footprint first enters a boundary polygon.
std::optional<size_t> find_polygon_last_kept_index(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape,
  const std::vector<autoware_utils_geometry::Polygon2d> & polygons_2d)
{
  if (polygons_2d.empty()) {
    return std::nullopt;
  }
  for (auto i = 0UL; i < predicted_path.path.size(); ++i) {
    const autoware_utils_geometry::Polygon2d footprint =
      autoware_utils_geometry::to_polygon2d(predicted_path.path.at(i), object_shape);
    for (const auto & polygon : polygons_2d) {
      // NOTE: intersects_convex (GJK) treats both polygons as convex. A non-convex boundary area
      // is evaluated as its convex hull, but this works effectively.
      if (autoware_utils_geometry::intersects_convex(footprint, polygon)) {
        // Drop the entering pose and beyond, keeping at least one pose.
        return std::max<size_t>(i, 1UL) - 1UL;
      }
    }
  }
  return std::nullopt;
}

// Index of the last pose to keep when the path first crosses a boundary linestring.
std::optional<size_t> find_linestring_last_kept_index(
  const lanelet::BasicLineString2d & path_ls, const lanelet::LaneletMap & linestring_layer)
{
  if (path_ls.size() < 2) {
    return std::nullopt;
  }
  const auto candidates =
    linestring_layer.lineStringLayer.search(lanelet::geometry::boundingBox2d(path_ls));
  for (size_t seg = 0; seg + 1 < path_ls.size(); ++seg) {
    const lanelet::BasicLineString2d segment(
      lanelet::BasicPoints2d{path_ls[seg], path_ls[seg + 1]});
    const bool crosses = std::any_of(candidates.begin(), candidates.end(), [&](const auto & border) {
      return boost::geometry::intersects(segment, lanelet::utils::to2D(border).basicLineString());
    });
    if (crosses) {
      return seg;
    }
  }
  return std::nullopt;
}
}  // namespace

void ForcePathCutModule::build_from_map(
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
  const std::vector<std::string> & boundary_types)
{
  polygon_layer_ = nullptr;
  linestring_layer_ = nullptr;
  if (!lanelet_map_ptr) {
    return;
  }

  const auto is_boundary_type = [&boundary_types](const std::string & value) {
    return std::find(boundary_types.begin(), boundary_types.end(), value) != boundary_types.end();
  };

  lanelet::Polygons3d polygons;
  for (const auto & polygon : lanelet_map_ptr->polygonLayer) {
    const std::string subtype = polygon.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (is_boundary_type(subtype)) {
      polygons.emplace_back(std::const_pointer_cast<lanelet::LineStringData>(polygon.constData()));
    }
  }
  if (!polygons.empty()) {
    polygon_layer_ = lanelet::utils::createMap(polygons);
  }

  lanelet::LineStrings3d linestrings;
  for (const auto & linestring : lanelet_map_ptr->lineStringLayer) {
    const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "none");
    if (is_boundary_type(type)) {
      linestrings.emplace_back(
        std::const_pointer_cast<lanelet::LineStringData>(linestring.constData()));
    }
  }
  if (!linestrings.empty()) {
    linestring_layer_ = lanelet::utils::createMap(linestrings);
  }
}

std::vector<PredictedPath> ForcePathCutModule::cut_paths_crossing_boundary(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object) const
{
  std::vector<PredictedPath> cut_paths = predicted_object.kinematics.predicted_paths;
  if (!has_required_info(predicted_object) || (!polygon_layer_ && !linestring_layer_)) {
    return cut_paths;
  }

  const autoware_perception_msgs::msg::Shape & object_shape = predicted_object.shape;
  const std::vector<autoware_utils_geometry::Polygon2d> candidate_polygons =
    polygon_layer_ ? collect_candidate_polygons(*polygon_layer_, cut_paths, object_shape)
                   : std::vector<autoware_utils_geometry::Polygon2d>{};

  for (PredictedPath & predicted_path : cut_paths) {
    std::optional<size_t> last_kept_index =
      find_polygon_last_kept_index(predicted_path, object_shape, candidate_polygons);
    if (linestring_layer_) {
      const std::optional<size_t> linestring_index =
        find_linestring_last_kept_index(to_basic_line_string(predicted_path.path), *linestring_layer_);
      if (linestring_index && (!last_kept_index || *linestring_index < *last_kept_index)) {
        last_kept_index = linestring_index;
      }
    }
    if (last_kept_index) {
      predicted_path = path_cut::force_cut_at_index(predicted_path, *last_kept_index);
    }
  }
  return cut_paths;
}
}  // namespace autoware::map_based_prediction
