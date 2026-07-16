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
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::MultiLineString2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

constexpr double crosswalk_corridor_extend_margin = 1.0;

Box2d to_box2d(const lanelet::BoundingBox2d & box)
{
  return {Point2d(box.min().x(), box.min().y()), Point2d(box.max().x(), box.max().y())};
}

lanelet::BoundingBox2d to_lanelet_box(const Box2d & box)
{
  return {
    lanelet::BasicPoint2d(box.min_corner().x(), box.min_corner().y()),
    lanelet::BasicPoint2d(box.max_corner().x(), box.max_corner().y())};
}

Point2d extend_outward(const Point2d & end, const Point2d & inner, const double margin)
{
  const double dx = end.x() - inner.x();
  const double dy = end.y() - inner.y();
  const double len = std::hypot(dx, dy);
  if (len < 1e-6) {
    return end;
  }
  return {end.x() + dx / len * margin, end.y() + dy / len * margin};
}

std::vector<Point2d> to_extended_bound_points(
  const lanelet::ConstLineString3d & bound, const double extend_margin)
{
  std::vector<Point2d> points;
  points.reserve(bound.size());
  for (const auto & p : bound) {
    points.emplace_back(p.x(), p.y());
  }
  if (points.size() >= 2) {
    points.front() = extend_outward(points.front(), points.at(1), extend_margin);
    points.back() = extend_outward(points.back(), points.at(points.size() - 2), extend_margin);
  }
  return points;
}

Polygon2d build_extended_crosswalk_polygon(
  const lanelet::ConstLanelet & crosswalk, const double extend_margin)
{
  const auto left = to_extended_bound_points(crosswalk.leftBound(), extend_margin);
  const auto right = to_extended_bound_points(crosswalk.rightBound(), extend_margin);

  Polygon2d corridor;
  auto & outer = corridor.outer();
  outer.insert(outer.end(), left.begin(), left.end());
  outer.insert(outer.end(), right.rbegin(), right.rend());
  boost::geometry::correct(corridor);
  return corridor;
}

lanelet::BoundingBox2d expand_box(const lanelet::BoundingBox2d & box, const double margin)
{
  const lanelet::BasicPoint2d offset(margin, margin);
  return {box.min() - offset, box.max() + offset};
}

std::vector<Polygon2d> collect_crosswalk_polygons(
  const lanelet::LaneletMap & map, const lanelet::BoundingBox2d & search_box,
  const double extend_margin)
{
  std::vector<Polygon2d> corridors;
  for (const auto & candidate : map.laneletLayer.search(expand_box(search_box, extend_margin))) {
    const std::string subtype = candidate.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (
      subtype != lanelet::AttributeValueString::Crosswalk &&
      subtype != lanelet::AttributeValueString::Walkway) {
      continue;
    }
    Polygon2d corridor = build_extended_crosswalk_polygon(candidate, extend_margin);
    if (!boost::geometry::is_valid(corridor)) {
      continue;
    }
    corridors.push_back(std::move(corridor));
  }
  return corridors;
}

std::vector<LineString2d> clip_out_with_polygons(
  const LineString2d & candidate, const std::vector<Polygon2d> & corridors_polygons)
{
  std::vector<LineString2d> pieces{candidate};
  for (const auto & corridor : corridors_polygons) {
    std::vector<LineString2d> next;
    for (const auto & piece : pieces) {
      MultiLineString2d outside;
      boost::geometry::difference(piece, corridor, outside);
      for (auto & part : outside) {
        if (part.size() >= 2) {
          next.push_back(std::move(part));
        }
      }
    }
    pieces = std::move(next);
    if (pieces.empty()) {
      break;
    }
  }
  return pieces;
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
}  // namespace

void RoadBorderModule::build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  road_border_rtree_.clear();
  if (!lanelet_map_ptr) {
    return;
  }

  std::vector<RoadBorderNode> nodes;
  for (const auto & linestring : lanelet_map_ptr->lineStringLayer) {
    const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "none");
    if (type != "road_border") {
      continue;
    }

    LineString2d border;
    boost::geometry::convert(lanelet::utils::to2D(linestring).basicLineString(), border);
    if (border.size() < 2) {
      continue;
    }

    Box2d border_box;
    boost::geometry::envelope(border, border_box);
    const std::vector<Polygon2d> crosswalk_corridors = collect_crosswalk_polygons(
      *lanelet_map_ptr, to_lanelet_box(border_box), crosswalk_corridor_extend_margin);

    for (auto & piece : clip_out_with_polygons(border, crosswalk_corridors)) {
      Box2d piece_box;
      boost::geometry::envelope(piece, piece_box);
      nodes.emplace_back(piece_box, std::move(piece));
    }
  }
  road_border_rtree_ = RoadBorderRtree(nodes);
}

PredictedPath RoadBorderModule::cut_path_at_road_border(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::TrackedObject & object,
  const path_cut::MaxDecelerationParams & max_decel_params) const
{
  const auto & poses = predicted_path.path;
  if (
    road_border_rtree_.empty() || poses.size() < 2 ||
    !path_cut::shape_has_footprint(object.shape)) {
    return predicted_path;
  }

  const autoware_perception_msgs::msg::Shape & object_shape = object.shape;
  const lanelet::BoundingBox2d search_box =
    path_cut::get_bbox_contain_path_with_footprint(predicted_path, object_shape);

  std::vector<autoware_utils_geometry::LineString2d> candidates;
  for (auto it =
         road_border_rtree_.qbegin(boost::geometry::index::intersects(to_box2d(search_box)));
       it != road_border_rtree_.qend(); ++it) {
    candidates.push_back(it->second);
  }

  const std::optional<size_t> road_border_crossing_index =
    path_cut::find_footprint_crossing_index(predicted_path, object_shape, candidates);
  if (!road_border_crossing_index) {
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
