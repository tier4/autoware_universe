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

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
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

PredictedPath VegetationModule::cutPathCrossingVegetation(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & shape) const
{
  const auto crossing_index = getVegetationCrossingIndex(predicted_path, shape);
  if (!crossing_index) {
    return predicted_path;
  }
  auto trimmed_path = predicted_path;
  trimmed_path.path.resize(*crossing_index);
  return trimmed_path;
}

std::optional<size_t> VegetationModule::getVegetationCrossingIndex(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & shape) const
{
  if (!vegetation_layer_ || predicted_path.path.empty()) {
    return std::nullopt;
  }

  // Bounding box over the whole path with footprint margin to avoid candidate misses.
  lanelet::BoundingBox2d search_bbox;
  const auto search_margin = calcFootprintSearchMargin(shape);
  for (const auto & pose : predicted_path.path) {
    const lanelet::BasicPoint2d center(pose.position.x, pose.position.y);
    const lanelet::BasicPoint2d offset(search_margin, search_margin);
    search_bbox.extend(center - offset);
    search_bbox.extend(center + offset);
  }
  const auto candidate_vegetation_polygons = vegetation_layer_->polygonLayer.search(search_bbox);
  if (candidate_vegetation_polygons.empty()) {
    return std::nullopt;
  }

  // convert candidate vegetation areas to 2d polygons once
  std::vector<autoware_utils_geometry::Polygon2d> vegetation_polygons_2d;
  vegetation_polygons_2d.reserve(candidate_vegetation_polygons.size());

  for (const auto & candidate : candidate_vegetation_polygons) {
    autoware_utils_geometry::LinearRing2d ring;
    boost::geometry::convert(lanelet::utils::to2D(candidate.basicPolygon()), ring);
    autoware_utils_geometry::Polygon2d polygon;
    polygon.outer() = ring;
    boost::geometry::correct(polygon);
    vegetation_polygons_2d.push_back(polygon);
  }

  // check the object footprint at each path point against the vegetation areas
  for (auto i = 0UL; i < predicted_path.path.size(); ++i) {
    const auto footprint = autoware_utils_geometry::to_polygon2d(predicted_path.path[i], shape);

    for (const auto & vegetation_polygon : vegetation_polygons_2d) {
      // NOTE: intersects_convex (GJK) treats both polygons as convex. A non-convex vegetation area
      // is evaluated as its convex hull, but this work effectively.
      if (autoware_utils_geometry::intersects_convex(footprint, vegetation_polygon)) {
        return i;
      }
    }
  }
  return std::nullopt;
}

void VegetationModule::recordVegetationPathCutEvent(
  const PredictedPath & predicted_path, const PredictedPath & cut_path,
  const autoware_perception_msgs::msg::TrackedObject & object)
{
  debug_recorder_.recordEvent(predicted_path, cut_path, object);
}
}  // namespace autoware::map_based_prediction
