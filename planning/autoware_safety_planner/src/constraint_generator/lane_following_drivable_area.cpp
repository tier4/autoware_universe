// Copyright 2026 TIER IV, Inc.
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

#include "lane_following_drivable_area.hpp"

#include "../utils/frenet_utils.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <iterator>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

//! [m] spacing of the samples taken along the reference_path when searching road_borders
constexpr double SAMPLE_STEP_M = 5.0;
//! [m] spacing of the samples taken along the reference_path when locating a walkway crossing
constexpr double WALKWAY_SAMPLE_STEP_M = 0.5;

//! Point of line closest to p
lanelet::BasicPoint2d closest_point_on_polyline(
  const lanelet::BasicPoint2d & p, const lanelet::ConstLineString3d & line)
{
  double best = std::numeric_limits<double>::max();
  lanelet::BasicPoint2d best_point{line[0].x(), line[0].y()};
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    const lanelet::BasicPoint2d a{line[i].x(), line[i].y()};
    const lanelet::BasicPoint2d b{line[i + 1].x(), line[i + 1].y()};
    const auto ab = b - a;
    const double len2 = ab.squaredNorm();
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    const lanelet::BasicPoint2d q = a + ab * t;
    const double dist = (p - q).norm();
    if (dist < best) {
      best = dist;
      best_point = q;
    }
  }
  return best_point;
}

//! Moves every vertex by margin_m towards its foot on the inner polyline (the centerline for a lane
//! bound, the reference_path for a road_border). The IR carries no margin, so the clearance is
//! baked into the geometry here. A vertex lying on the inner polyline has no direction and stays
//! put.
std::vector<Point2d> offset_towards(
  const std::vector<Point2d> & polyline, const lanelet::ConstLineString3d & inner,
  const double margin_m)
{
  if (!(margin_m > 0.0)) {
    return polyline;
  }
  std::vector<Point2d> offset;
  offset.reserve(polyline.size());
  for (const auto & vertex : polyline) {
    const lanelet::BasicPoint2d q{vertex.x(), vertex.y()};
    const auto to_inner = closest_point_on_polyline(q, inner) - q;
    const double dist = to_inner.norm();
    if (dist <= margin_m) {
      offset.push_back(vertex);
      continue;
    }
    const auto moved = q + to_inner * (margin_m / dist);
    offset.emplace_back(moved.x(), moved.y());
  }
  return offset;
}

//! Builds one Boundary constraint. The order of the vertices does not matter: the side the
//! boundary forbids is decided by the consumer, from where the polyline falls relative to its
//! reference path.
Constraint make_boundary_constraint(
  const std::vector<Point2d> & polyline, const Hardness hardness, const std::string & plugin_name,
  const std::string & target_id, const std::string & detail)
{
  Constraint constraint;
  constraint.certainty = Certainty::DEFINITE;  // the map is a settled premise
  constraint.hardness = hardness;
  Boundary boundary;
  boundary.polyline.assign(polyline.begin(), polyline.end());
  constraint.payload = std::move(boundary);
  constraint.source = Source{plugin_name, target_id, detail};
  return constraint;
}

//! Both bounds of every lanelet, always soft whether or not there is a lane next to them: the
//! road_borders keep the ego on the road, so the lane bounds only need to be a preference
std::vector<Constraint> make_lane_bound_constraints(
  const lanelet::ConstLanelets & lanelets, const double margin_m, const std::string & plugin_name)
{
  std::vector<Constraint> constraints;
  for (const auto & lanelet : lanelets) {
    for (const auto side_left : {true, false}) {
      const auto & bound = side_left ? lanelet.leftBound() : lanelet.rightBound();
      // The map is an external resource: a bound with fewer than two vertices is not a polyline
      if (bound.size() < 2) {
        continue;
      }
      std::vector<Point2d> polyline;
      polyline.reserve(bound.size());
      for (const auto & point : bound) {
        polyline.emplace_back(point.x(), point.y());
      }
      constraints.push_back(make_boundary_constraint(
        offset_towards(polyline, lanelet.centerline(), margin_m), Hardness::SOFT, plugin_name,
        std::to_string(lanelet.id()), side_left ? "left_bound" : "right_bound"));
    }
  }
  return constraints;
}

struct NearbyMapElements
{
  std::set<lanelet::Id> road_border_ids;
  std::set<lanelet::Id> walkway_ids;
};

//! The road_borders, and the walkways when with_walkways, within distance_m of the reference_path.
//! Both come from one pass since they share the bbox searches
NearbyMapElements find_nearby_map_elements(
  const PathPointTrajectory & reference_path, const lanelet::LaneletMapConstPtr & lanelet_map,
  const double distance_m, const bool with_walkways)
{
  NearbyMapElements elements;
  for (double s = 0.0; s <= reference_path.length(); s += SAMPLE_STEP_M) {
    const auto position = reference_path.compute(s).point.pose.position;
    const lanelet::BoundingBox2d search_box{
      lanelet::BasicPoint2d{position.x - distance_m, position.y - distance_m},
      lanelet::BasicPoint2d{position.x + distance_m, position.y + distance_m}};
    for (const auto & linestring : lanelet_map->lineStringLayer.search(search_box)) {
      if (
        linestring.attributeOr(lanelet::AttributeName::Type, "") == std::string("road_border") &&
        linestring.size() >= 2) {
        elements.road_border_ids.insert(linestring.id());
      }
    }
    if (with_walkways) {
      for (const auto & walkway : lanelet_map->laneletLayer.search(search_box)) {
        if (walkway.attributeOr(lanelet::AttributeName::Subtype, "") == std::string("walkway")) {
          elements.walkway_ids.insert(walkway.id());
        }
      }
    }
  }
  return elements;
}

//! Every road_border becomes hard, on either side and regardless of the lanes in between. Which
//! side it forbids is left to the consumer. A single linestring can wrap hundreds of meters of
//! road, so only the runs of vertices within distance_m of the reference_path are emitted; a
//! distant run projects to a huge l and rejects every candidate
std::vector<Constraint> make_road_border_constraints(
  const std::set<lanelet::Id> & road_border_ids, const lanelet::LaneletMapConstPtr & lanelet_map,
  const lanelet::ConstLineString3d & reference_line, const double distance_m, const double margin_m,
  const std::string & plugin_name)
{
  std::vector<Constraint> constraints;
  for (const auto border_id : road_border_ids) {
    const auto border = lanelet_map->lineStringLayer.get(border_id);
    std::vector<Point2d> run;
    const auto flush = [&]() {
      if (run.size() >= 2) {
        constraints.push_back(make_boundary_constraint(
          offset_towards(run, reference_line, margin_m), Hardness::HARD, plugin_name,
          std::to_string(border_id), "road_border"));
      }
      run.clear();
    };
    for (const auto & point : border) {
      const lanelet::BasicPoint2d q{point.x(), point.y()};
      if ((closest_point_on_polyline(q, reference_line) - q).norm() <= distance_m) {
        run.emplace_back(q.x(), q.y());
      } else {
        flush();
      }
    }
    flush();
  }
  return constraints;
}

//! Where the reference_path crosses a sidewalk (a driveway into a parking lot), the road_border
//! stops at both edges of the sidewalk and leaves the side open. On each side the gap is closed by
//! a hard segment between the road_borders nearest to where the path enters and leaves it
std::vector<Constraint> make_walkway_gap_constraints(
  const std::set<lanelet::Id> & walkway_ids, const std::set<lanelet::Id> & road_border_ids,
  const lanelet::LaneletMapConstPtr & lanelet_map, const PathPointTrajectory & reference_path,
  const lanelet::ConstLineString3d & reference_line, const double distance_m, const double margin_m,
  const std::string & plugin_name)
{
  // Nearest road_border point on the side_sign side of the path, measured from the path at s_ref
  const auto nearest_border = [&](const double s_ref, const double side_sign) {
    const auto p = reference_path.compute(s_ref).point.pose.position;
    const lanelet::BasicPoint2d p_ref{p.x, p.y};
    double best_dist = distance_m;
    std::optional<std::pair<lanelet::Id, lanelet::BasicPoint2d>> best;
    for (const auto border_id : road_border_ids) {
      const auto q = closest_point_on_polyline(p_ref, lanelet_map->lineStringLayer.get(border_id));
      const double dist = (q - p_ref).norm();
      if (
        dist < best_dist &&
        lateral_offset_at(reference_path, s_ref, Point2d{q.x(), q.y()}) * side_sign > 0.0) {
        best_dist = dist;
        best = std::make_pair(border_id, q);
      }
    }
    return best;
  };

  std::vector<Constraint> constraints;
  for (const auto walkway_id : walkway_ids) {
    const auto walkway = lanelet_map->laneletLayer.get(walkway_id);
    // Arc length where the path entered the walkway; valid while in_walkway
    double s_in = 0.0;
    bool in_walkway = false;
    for (double s = 0.0; s <= reference_path.length(); s += WALKWAY_SAMPLE_STEP_M) {
      const auto position = reference_path.compute(s).point.pose.position;
      const bool inside =
        lanelet::geometry::inside(walkway, lanelet::BasicPoint2d{position.x, position.y});
      if (inside && !in_walkway) {
        s_in = s;
        in_walkway = true;
      }
      if (!in_walkway || (inside && s + WALKWAY_SAMPLE_STEP_M <= reference_path.length())) {
        continue;
      }
      const double s_out = s;
      for (const auto side_sign : {1.0, -1.0}) {
        const auto border_in = nearest_border(s_in, side_sign);
        const auto border_out = nearest_border(s_out, side_sign);
        // The same road_border on both ends runs along the sidewalk, so there is no gap to close
        if (!border_in || !border_out || border_in->first == border_out->first) {
          continue;
        }
        const std::vector<Point2d> segment{
          {border_in->second.x(), border_in->second.y()},
          {border_out->second.x(), border_out->second.y()}};
        constraints.push_back(make_boundary_constraint(
          offset_towards(segment, reference_line, margin_m), Hardness::HARD, plugin_name,
          std::to_string(walkway_id), side_sign > 0.0 ? "left_walkway_gap" : "right_walkway_gap"));
      }
      in_walkway = false;
    }
  }
  return constraints;
}

}  // namespace

ConstraintGeneratorOutput LaneFollowingDrivableAreaConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  ConstraintGeneratorOutput output;

  // Without a route there is nothing to constrain; an empty output keeps the pipeline running
  if (!context.route_manager) {
    return output;
  }

  // Cover the same window of lanes as the reference_path, sharing its parameters
  const auto & route_manager = *context.route_manager;
  const auto lanelets =
    route_manager
      .get_lanelet_sequence_on_route(
        params_.reference_path.forward_length_m, params_.reference_path.backward_length_m)
      .as_lanelets();
  const auto & lanelet_map = route_manager.lanelet_map_ptr();
  const auto & drivable_area_params = params_.lane_following_drivable_area;
  const double margin_m = drivable_area_params.margin_m;
  const double distance_m = drivable_area_params.road_border_distance_m;

  const auto & reference_path = context.reference_path;
  lanelet::LineString3d reference_line;
  for (const auto & point : reference_path.restore()) {
    const auto & position = point.point.pose.position;
    reference_line.push_back(lanelet::Point3d(lanelet::InvalId, position.x, position.y, 0.0));
  }
  const auto nearby = find_nearby_map_elements(
    reference_path, lanelet_map, distance_m, drivable_area_params.close_walkway_gap);

  const auto append = [&](std::vector<Constraint> && constraints) {
    output.constraints.insert(
      output.constraints.end(), std::make_move_iterator(constraints.begin()),
      std::make_move_iterator(constraints.end()));
  };
  append(make_lane_bound_constraints(lanelets, margin_m, get_name()));
  append(make_road_border_constraints(
    nearby.road_border_ids, lanelet_map, reference_line, distance_m, margin_m, get_name()));
  append(make_walkway_gap_constraints(
    nearby.walkway_ids, nearby.road_border_ids, lanelet_map, reference_path, reference_line,
    distance_m, margin_m, get_name()));

  return output;
}

}  // namespace autoware::safety_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::LaneFollowingDrivableAreaConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
