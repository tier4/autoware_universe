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

#include "autoware/map_based_prediction/path_cut/footprint_path_cut.hpp"

#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::map_based_prediction::path_cut
{

namespace
{
void extend_bbox_by_footprint(
  lanelet::BoundingBox2d & search_bbox, const PredictedPath & predicted_path, const double margin)
{
  const lanelet::BasicPoint2d offset(margin, margin);
  for (const auto & pose : predicted_path.path) {
    const lanelet::BasicPoint2d center(pose.position.x, pose.position.y);
    search_bbox.extend(center - offset);
    search_bbox.extend(center + offset);
  }
}
}  // namespace

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

bool shape_has_footprint(const autoware_perception_msgs::msg::Shape & shape)
{
  using autoware_perception_msgs::msg::Shape;
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

bool has_required_info(const autoware_perception_msgs::msg::PredictedObject & predicted_object)
{
  return !predicted_object.kinematics.predicted_paths.empty() &&
         shape_has_footprint(predicted_object.shape);
}

lanelet::BoundingBox2d footprint_search_bbox(
  const std::vector<PredictedPath> & predicted_paths,
  const autoware_perception_msgs::msg::Shape & object_shape)
{
  lanelet::BoundingBox2d search_bbox;
  const auto margin = calc_footprint_search_margin(object_shape);
  for (const auto & predicted_path : predicted_paths) {
    extend_bbox_by_footprint(search_bbox, predicted_path, margin);
  }
  return search_bbox;
}

lanelet::BoundingBox2d footprint_search_bbox(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape)
{
  lanelet::BoundingBox2d search_bbox;
  extend_bbox_by_footprint(search_bbox, predicted_path, calc_footprint_search_margin(object_shape));
  return search_bbox;
}

std::optional<size_t> find_footprint_crossing_index(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape,
  const std::vector<autoware_utils_geometry::LineString2d> & linestrings_2d)
{
  if (linestrings_2d.empty()) {
    return std::nullopt;
  }
  for (auto i = 0UL; i < predicted_path.path.size(); ++i) {
    const autoware_utils_geometry::Polygon2d footprint =
      autoware_utils_geometry::to_polygon2d(predicted_path.path.at(i), object_shape);
    for (const auto & linestring : linestrings_2d) {
      if (boost::geometry::intersects(footprint, linestring)) {
        return i;
      }
    }
  }
  return std::nullopt;
}

std::optional<size_t> find_footprint_crossing_index(
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
        return i;
      }
    }
  }
  return std::nullopt;
}

}  // namespace autoware::map_based_prediction::path_cut
