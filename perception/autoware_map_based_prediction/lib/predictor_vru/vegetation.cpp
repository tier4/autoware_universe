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

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
using Point2d = autoware_utils_geometry::Point2d;
using Polygon2d = autoware_utils_geometry::Polygon2d;
using Box2d = autoware_utils_geometry::Box2d;

Polygon2d footprint_polygon_at_pose(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape)
{
  auto polygon = autoware_utils::to_polygon2d(pose, shape);
  boost::geometry::correct(polygon);
  return polygon;
}

void extend_bbox_with_polygon(lanelet::BoundingBox2d & bbox, const Polygon2d & polygon)
{
  for (const auto & point : polygon.outer()) {
    bbox.extend(lanelet::BasicPoint2d(point.x(), point.y()));
  }
}

Polygon2d convex_polygon_covering_segment_footprints(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose,
  const autoware_perception_msgs::msg::Shape & shape)
{
  const auto start_polygon = footprint_polygon_at_pose(start_pose, shape);
  const auto end_polygon = footprint_polygon_at_pose(end_pose, shape);

  boost::geometry::model::multi_point<Point2d> points;
  for (const auto & point : start_polygon.outer()) {
    points.push_back(point);
  }
  for (const auto & point : end_polygon.outer()) {
    points.push_back(point);
  }

  Polygon2d hull;
  boost::geometry::convex_hull(points, hull);
  boost::geometry::correct(hull);
  return hull;
}

std::vector<Polygon2d> collect_candidate_vegetation_polygons(
  const lanelet::LaneletMap & vegetation_layer, const std::vector<PredictedPath> & predicted_paths,
  const autoware_perception_msgs::msg::Shape & object_shape)
{
  lanelet::BoundingBox2d search_bbox;
  for (const auto & predicted_path : predicted_paths) {
    for (const auto & pose : predicted_path.path) {
      extend_bbox_with_polygon(search_bbox, footprint_polygon_at_pose(pose, object_shape));
    }
  }

  const auto candidates = vegetation_layer.polygonLayer.search(search_bbox);
  std::vector<Polygon2d> vegetation_polygons_2d;
  vegetation_polygons_2d.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    autoware_utils_geometry::Polygon2d polygon;
    boost::geometry::convert(lanelet::utils::to2D(candidate.basicPolygon()), polygon);
    boost::geometry::correct(polygon);
    vegetation_polygons_2d.push_back(polygon);
  }
  return vegetation_polygons_2d;
}

std::optional<size_t> find_vegetation_crossing_index(
  const PredictedPath & predicted_path, const std::vector<Polygon2d> & vegetation_polygons,
  const autoware_perception_msgs::msg::Shape & object_shape)
{
  const auto & path = predicted_path.path;
  if (path.size() < 2 || vegetation_polygons.empty()) {
    return std::nullopt;
  }

  boost::geometry::model::multi_point<Point2d> path_footprint_points;
  for (const auto & pose : path) {
    for (const auto & point : footprint_polygon_at_pose(pose, object_shape).outer()) {
      path_footprint_points.push_back(point);
    }
  }
  const auto path_bbox = boost::geometry::return_envelope<Box2d>(path_footprint_points);

  for (const auto & vegetation_polygon : vegetation_polygons) {
    if (!boost::geometry::intersects(path_bbox, vegetation_polygon)) {
      continue;
    }
    for (auto i = 0UL; i + 1 < path.size(); ++i) {
      const auto swept_polygon =
        convex_polygon_covering_segment_footprints(path.at(i), path.at(i + 1), object_shape);
      if (boost::geometry::intersects(swept_polygon, vegetation_polygon)) {
        return i;
      }
    }
  }
  return std::nullopt;
}
}  // namespace

void VegetationModule::build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
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

std::vector<PredictedPath> VegetationModule::cut_paths_crossing_vegetation(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object) const
{
  std::vector<PredictedPath> cut_paths = predicted_object.kinematics.predicted_paths;
  if (cut_paths.empty() || !vegetation_layer_) {
    return cut_paths;
  }

  const std::vector<Polygon2d> candidate_polygons =
    collect_candidate_vegetation_polygons(*vegetation_layer_, cut_paths, predicted_object.shape);

  for (PredictedPath & predicted_path : cut_paths) {
    const std::optional<size_t> crossing_index =
      find_vegetation_crossing_index(predicted_path, candidate_polygons, predicted_object.shape);
    if (crossing_index) {
      // Keep the last pose before the segment enters the vegetation area.
      predicted_path = path_cut::force_cut_at_index(predicted_path, crossing_index.value());
    }
  }
  return cut_paths;
}
}  // namespace autoware::map_based_prediction
