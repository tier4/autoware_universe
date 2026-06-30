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

#include "autoware/map_based_prediction/predictor_vru/vegetation.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
double calcFootprintSearchMargin(const autoware_perception_msgs::msg::Shape & shape)
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

std::vector<autoware_utils_geometry::Polygon2d> collectCandidateVegetationPolygons(
  const lanelet::LaneletMap & vegetation_layer, const std::vector<PredictedPath> & predicted_paths,
  const autoware_perception_msgs::msg::Shape & shape)
{
  lanelet::BoundingBox2d search_bbox;
  const auto search_margin = calcFootprintSearchMargin(shape);
  const lanelet::BasicPoint2d offset(search_margin, search_margin);
  for (const auto & predicted_path : predicted_paths) {
    for (const auto & pose : predicted_path.path) {
      const lanelet::BasicPoint2d center(pose.position.x, pose.position.y);
      search_bbox.extend(center - offset);
      search_bbox.extend(center + offset);
    }
  }

  const auto candidates = vegetation_layer.polygonLayer.search(search_bbox);
  std::vector<autoware_utils_geometry::Polygon2d> vegetation_polygons_2d;
  vegetation_polygons_2d.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    autoware_utils_geometry::Polygon2d polygon;
    boost::geometry::convert(lanelet::utils::to2D(candidate.basicPolygon()), polygon);
    boost::geometry::correct(polygon);
    vegetation_polygons_2d.push_back(polygon);
  }
  return vegetation_polygons_2d;
}

std::optional<size_t> findVegetationCrossingIndex(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & shape,
  const std::vector<autoware_utils_geometry::Polygon2d> & vegetation_polygons_2d)
{
  if (vegetation_polygons_2d.empty()) {
    return std::nullopt;
  }
  for (auto i = 0UL; i < predicted_path.path.size(); ++i) {
    const auto footprint = autoware_utils_geometry::to_polygon2d(predicted_path.path[i], shape);
    for (const auto & vegetation_polygon : vegetation_polygons_2d) {
      // NOTE: intersects_convex (GJK) treats both polygons as convex. A non-convex vegetation area
      // is evaluated as its convex hull, but this works effectively.
      if (autoware_utils_geometry::intersects_convex(footprint, vegetation_polygon)) {
        return i;
      }
    }
  }
  return std::nullopt;
}
}  // namespace

void VegetationModule::buildFromMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  if (!lanelet_map_ptr) {
    vegetation_layer_ = nullptr;
    return;
  }

  lanelet::Polygons3d vegetations;
  for (const auto & polygon : lanelet_map_ptr->polygonLayer) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    const std::string subtype = polygon.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (type == "area" && subtype == "vegetation") {
      vegetations.emplace_back(
        std::const_pointer_cast<lanelet::LineStringData>(polygon.constData()));
    }
  }
  vegetation_layer_ = lanelet::utils::createMap(vegetations);
}

void VegetationModule::cutPathsCrossingVegetation(
  std::vector<PredictedPath> & predicted_paths,
  const autoware_perception_msgs::msg::TrackedObject & object,
  visualization_msgs::msg::MarkerArray * debug_markers, const rclcpp::Time & stamp)
{
  std::vector<autoware_utils_geometry::Polygon2d> candidate_polygons;
  if (vegetation_layer_ && !predicted_paths.empty()) {
    candidate_polygons =
      collectCandidateVegetationPolygons(*vegetation_layer_, predicted_paths, object.shape);
  }

  std::unordered_set<std::string> boxed_objects;
  for (auto & predicted_path : predicted_paths) {
    PredictedPath original_path;
    if (debug_markers) {
      original_path = predicted_path;
    }

    const auto crossing_index =
      findVegetationCrossingIndex(predicted_path, object.shape, candidate_polygons);
    if (crossing_index) {
      predicted_path.path.resize(std::max<size_t>(*crossing_index, 1UL));
    }

    if (debug_markers) {
      const auto event = debug::createVegetationPathEvent(original_path, predicted_path, object);
      debug::appendVegetationEventMarkers(*debug_markers, event, stamp, boxed_objects);
    }
  }
}
}  // namespace autoware::map_based_prediction
