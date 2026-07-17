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

#include "autoware/map_based_prediction/predictor_vru/road_boundary.hpp"

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

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
bool does_path_cross_boundary(
  const lanelet::BasicLineString2d & predicted_path,
  const lanelet::ConstLineString3d & boundary_line)
{
  return boost::geometry::intersects(
    predicted_path, lanelet::utils::to2D(boundary_line.basicLineString()));
}

bool is_point_within_crosswalk(
  const lanelet::LaneletMap * crosswalk_layer, const lanelet::BasicPoint2d & point)
{
  if (!crosswalk_layer) {
    return false;
  }
  const lanelet::BoundingBox2d query(point, point);
  for (const auto & crosswalk : crosswalk_layer->laneletLayer.search(query)) {
    if (boost::geometry::within(point, crosswalk.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}

// A crossing on this segment counts as a cut point unless every intersection with the boundary
// falls inside a crosswalk / walkway (i.e. the object is predicted to cross at a crosswalk).
bool segment_has_cuttable_crossing(
  const lanelet::BasicLineString2d & path_segment, const lanelet::ConstLineString3d & boundary,
  const lanelet::LaneletMap * crosswalk_layer)
{
  const auto boundary_2d = lanelet::utils::to2D(boundary).basicLineString();
  if (!boost::geometry::intersects(path_segment, boundary_2d)) {
    return false;
  }
  lanelet::BasicPoints2d intersections;
  boost::geometry::intersection(path_segment, boundary_2d, intersections);
  // Fall back to cutting when the intersection points cannot be resolved (e.g. collinear overlap).
  if (intersections.empty()) {
    return true;
  }
  for (const auto & intersection : intersections) {
    if (!is_point_within_crosswalk(crosswalk_layer, intersection)) {
      return true;
    }
  }
  return false;
}

std::optional<size_t> find_road_boundary_crossing_index(
  const lanelet::LaneletMap & road_boundary_layer, const lanelet::LaneletMap * crosswalk_layer,
  const PredictedPath & predicted_path)
{
  const auto & path = predicted_path.path;
  if (path.size() < 2) {
    return std::nullopt;
  }

  lanelet::BasicLineString2d predicted_path_ls;
  predicted_path_ls.reserve(path.size());
  for (const auto & pt : path) {
    predicted_path_ls.emplace_back(pt.position.x, pt.position.y);
  }

  const auto candidates =
    road_boundary_layer.lineStringLayer.search(lanelet::geometry::boundingBox2d(predicted_path_ls));
  std::vector<lanelet::ConstLineString3d> crossed_boundaries{};
  for (const auto & candidate : candidates) {
    if (does_path_cross_boundary(predicted_path_ls, candidate)) {
      crossed_boundaries.push_back(candidate);
    }
  }
  if (crossed_boundaries.empty()) {
    return std::nullopt;
  }

  for (auto i = 0UL; i + 1 < predicted_path_ls.size(); ++i) {
    const lanelet::BasicLineString2d path_segment(
      lanelet::BasicPoints2d{predicted_path_ls.at(i), predicted_path_ls.at(i + 1)});
    for (const auto & boundary : crossed_boundaries) {
      if (segment_has_cuttable_crossing(path_segment, boundary, crosswalk_layer)) {
        return i;
      }
    }
  }
  return std::nullopt;
}
}  // namespace

void RoadBoundaryModule::build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  if (!lanelet_map_ptr) {
    road_boundary_layer_ = nullptr;
    crosswalk_layer_ = nullptr;
    return;
  }

  lanelet::LineStrings3d boundaries;
  std::unordered_set<lanelet::Id> added_ids;
  lanelet::ConstLanelets crosswalks;
  for (const auto & lanelet : lanelet_map_ptr->laneletLayer) {
    const std::string subtype = lanelet.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (
      subtype == lanelet::AttributeValueString::Crosswalk ||
      subtype == lanelet::AttributeValueString::Walkway) {
      crosswalks.push_back(lanelet);
      continue;
    }
    if (subtype != lanelet::AttributeValueString::Road && subtype != "road_shoulder") {
      continue;
    }
    for (const auto & bound : {lanelet.leftBound(), lanelet.rightBound()}) {
      if (added_ids.insert(bound.id()).second) {
        boundaries.emplace_back(
          std::const_pointer_cast<lanelet::LineStringData>(bound.constData()));
      }
    }
  }
  road_boundary_layer_ = lanelet::utils::createMap(boundaries);
  crosswalk_layer_ = lanelet::utils::createConstMap(crosswalks, {});
}

std::vector<PredictedPath> RoadBoundaryModule::cut_paths_crossing_road_boundary(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const bool object_within_road) const
{
  std::vector<PredictedPath> cut_paths = predicted_object.kinematics.predicted_paths;
  // Objects already inside a road lanelet are left untouched so that a path exiting the road
  // (i.e. its first boundary crossing) is not mistaken for a jump-out and cut.
  if (!road_boundary_layer_ || object_within_road) {
    return cut_paths;
  }

  const auto & linear_velocity =
    predicted_object.kinematics.initial_twist_with_covariance.twist.linear;
  const double object_speed = std::hypot(linear_velocity.x, linear_velocity.y);
  const double max_deceleration = path_cut::max_deceleration_for_label(
    max_deceleration_params_,
    autoware::object_recognition_utils::getHighestProbLabel(predicted_object.classification));

  for (PredictedPath & predicted_path : cut_paths) {
    const std::optional<size_t> crossing_index =
      find_road_boundary_crossing_index(*road_boundary_layer_, crosswalk_layer_.get(), predicted_path);
    if (!crossing_index) {
      continue;
    }
    // Only cut when the object can decelerate to a stop before the boundary. If it cannot stop in
    // time, the jump-out onto the road is unavoidable and the full path is kept.
    const double distance_to_cross = autoware::motion_utils::calcSignedArcLength(
      predicted_path.path, 0, crossing_index.value());
    if (!path_cut::can_stop_before_the_line(distance_to_cross, object_speed, max_deceleration)) {
      continue;
    }
    // Keep the last pose before the path crosses into the road.
    predicted_path = path_cut::force_cut_at_index(predicted_path, crossing_index.value());
  }
  return cut_paths;
}

}  // namespace autoware::map_based_prediction
