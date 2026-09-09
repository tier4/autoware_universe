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

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner::experiment
{

namespace
{

//! [m] spacing of the samples taken along a bound
constexpr double SAMPLE_STEP_M = 5.0;
//! [m] a parallel lane is looked for this far outside the bound; keep it below half a lane width
constexpr double ADJACENT_OFFSET_M = 1.5;
//! [rad] heading difference still counted as parallel (an oncoming lane is compared flipped). It
//! keeps the crossing roads of an intersection out
constexpr double PARALLEL_YAW_THRESHOLD = M_PI / 4.0;
//! fraction of the samples that must hit a parallel lane for the side to count as having one; it
//! absorbs the samples lost at the seams
constexpr double ADJACENT_FRACTION = 0.5;
//! [m] search radius for a road_border
constexpr double BORDER_SEARCH_RADIUS_M = 15.0;

//! Samples a polyline every step of arc length, as (point, heading) pairs
std::vector<std::pair<lanelet::BasicPoint2d, double>> sample_polyline_with_yaw(
  const lanelet::ConstLineString3d & line, const double step)
{
  std::vector<std::pair<lanelet::BasicPoint2d, double>> samples;
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    const lanelet::BasicPoint2d p0{line[i].x(), line[i].y()};
    const lanelet::BasicPoint2d p1{line[i + 1].x(), line[i + 1].y()};
    const double yaw = std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
    const double length = (p1 - p0).norm();
    for (double s = 0.0; s < length; s += step) {
      samples.emplace_back(p0 + (p1 - p0) * (s / length), yaw);
    }
  }
  return samples;
}

double point_segment_distance(
  const lanelet::BasicPoint2d & p, const lanelet::BasicPoint2d & a, const lanelet::BasicPoint2d & b)
{
  const auto ab = b - a;
  const double len2 = ab.squaredNorm();
  const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
  return (p - (a + ab * t)).norm();
}

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

//! Heading of the segment of line closest to point
double nearest_segment_yaw(
  const lanelet::ConstLineString3d & line, const lanelet::BasicPoint2d & point)
{
  double best_dist = std::numeric_limits<double>::max();
  double best_yaw = 0.0;
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    const lanelet::BasicPoint2d p0{line[i].x(), line[i].y()};
    const lanelet::BasicPoint2d p1{line[i + 1].x(), line[i + 1].y()};
    const double dist = point_segment_distance(point, p0, p1);
    if (dist < best_dist) {
      best_dist = dist;
      best_yaw = std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
    }
  }
  return best_yaw;
}

//! Whether a parallel lane (oncoming included) covers p_out
bool has_parallel_road_lanelet_at(
  const lanelet::LaneletMapConstPtr & lanelet_map, const lanelet::BasicPoint2d & p_out,
  const double yaw, const lanelet::Id self_id)
{
  for (const auto & candidate : autoware::experimental::lanelet2_utils::get_road_lanelets_at(
         lanelet_map, p_out.x(), p_out.y())) {
    if (candidate.id() == self_id) {
      continue;
    }
    const double dyaw = std::abs(
      std::remainder(nearest_segment_yaw(candidate.centerline(), p_out) - yaw, 2.0 * M_PI));
    if (dyaw < PARALLEL_YAW_THRESHOLD || dyaw > M_PI - PARALLEL_YAW_THRESHOLD) {
      return true;
    }
  }
  return false;
}

//! Builds one Boundary constraint. The order of the vertices does not matter: the side the
//! boundary forbids is decided by the consumer, from where the polyline falls relative to its
//! reference path.
Constraint make_boundary_constraint(
  const std::vector<Point2d> & polyline, const double margin_m, const Hardness hardness,
  const double slack_weight, const std::string & plugin_name, const std::string & target_id,
  const std::string & detail)
{
  Constraint constraint;
  constraint.certainty = Certainty::DEFINITE;  // the map is a settled premise
  constraint.hardness = hardness;
  constraint.slack_weight = hardness == Hardness::SOFT ? slack_weight : 0.0;
  Boundary boundary;
  boundary.polyline.assign(polyline.begin(), polyline.end());
  boundary.margin = margin_m;
  constraint.payload = std::move(boundary);
  constraint.source = Source{plugin_name, Category::SAFETY, target_id, detail};
  return constraint;
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
  const double margin_m = params_.lane_following_drivable_area.margin_m;
  const double bound_slack_weight = params_.lane_following_drivable_area.bound_slack_weight;

  for (const auto & lanelet : lanelets) {
    for (const auto side_left : {true, false}) {
      const auto & bound = side_left ? lanelet.leftBound() : lanelet.rightBound();
      // The map is an external resource: a bound with fewer than two vertices is not a polyline
      if (bound.size() < 2) {
        continue;
      }
      const double side_sign = side_left ? 1.0 : -1.0;
      const auto samples = sample_polyline_with_yaw(bound, SAMPLE_STEP_M);
      if (samples.empty()) {
        continue;
      }

      // One pass decides whether there is a parallel lane and collects the nearest road_border of
      // the stretches without one
      std::size_t adjacent_hits = 0;
      std::set<lanelet::Id> border_ids;
      for (const auto & [point, yaw] : samples) {
        // Outside of the left bound is to the left of the driving direction, and vice versa
        const lanelet::BasicPoint2d p_out{
          point.x() - std::sin(yaw) * ADJACENT_OFFSET_M * side_sign,
          point.y() + std::cos(yaw) * ADJACENT_OFFSET_M * side_sign};

        if (has_parallel_road_lanelet_at(lanelet_map, p_out, yaw, lanelet.id())) {
          ++adjacent_hits;
          continue;
        }

        const lanelet::BoundingBox2d search_box{
          lanelet::BasicPoint2d{
            p_out.x() - BORDER_SEARCH_RADIUS_M, p_out.y() - BORDER_SEARCH_RADIUS_M},
          lanelet::BasicPoint2d{
            p_out.x() + BORDER_SEARCH_RADIUS_M, p_out.y() + BORDER_SEARCH_RADIUS_M}};
        double best_dist = std::numeric_limits<double>::max();
        std::optional<lanelet::Id> best_id;
        for (const auto & linestring : lanelet_map->lineStringLayer.search(search_box)) {
          const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "");
          if (type != "road_border" || linestring.size() < 2) {
            continue;
          }
          // Never take the border of the other side: emitting the left border for a road without
          // one on the right would close the drivable area. Anything on the negative side of the
          // outward normal of the bound is not a candidate
          const auto q = closest_point_on_polyline(p_out, linestring);
          const double outward =
            (-std::sin(yaw) * (q.x() - point.x()) + std::cos(yaw) * (q.y() - point.y())) *
            side_sign;
          if (outward <= 0.0) {
            continue;
          }
          // The bbox search hits the bounding rectangle of the linestring, so a long border comes
          // back even when its nearest vertex is hundreds of meters away. Take only the borders
          // that really come within the radius; a distant one projects to a huge l on the other
          // side and rejects every candidate
          const double dist = (p_out - q).norm();
          if (dist > BORDER_SEARCH_RADIUS_M) {
            continue;
          }
          if (dist < best_dist) {
            best_dist = dist;
            best_id = linestring.id();
          }
        }
        if (best_id) {
          border_ids.insert(*best_id);
        }
      }

      const bool adjacent =
        static_cast<double>(adjacent_hits) / static_cast<double>(samples.size()) >=
        ADJACENT_FRACTION;

      if (adjacent || border_ids.empty()) {
        // With a parallel lane the bound of the own lane becomes soft, and no road_border is
        // emitted on this side. Where there is neither a parallel lane nor a road_border (the edge
        // of the map drawn as a line type only, ...) it becomes hard instead: emitting nothing
        // would leave the side unbounded and let a candidate aiming at the lateral goal position
        // leave the lane
        std::vector<Point2d> polyline;
        polyline.reserve(bound.size());
        for (const auto & point : bound) {
          polyline.emplace_back(point.x(), point.y());
        }
        const auto hardness = adjacent ? Hardness::SOFT : Hardness::HARD;
        output.constraints.push_back(make_boundary_constraint(
          polyline, margin_m, hardness, bound_slack_weight, get_name(),
          std::to_string(lanelet.id()), side_left ? "left_bound" : "right_bound"));
        continue;
      }

      // Without a parallel lane the road_border becomes the hard boundary, which keeps the
      // shoulder inside the drivable area. A single linestring can wrap hundreds of meters of road,
      // so only the runs of vertices within BORDER_SEARCH_RADIUS_M of the own bound are emitted; a
      // distant run projects to a huge l on the other side and rejects every candidate
      for (const auto border_id : border_ids) {
        const auto border = lanelet_map->lineStringLayer.get(border_id);
        std::vector<Point2d> polyline;
        polyline.reserve(border.size());
        for (const auto & point : border) {
          polyline.emplace_back(point.x(), point.y());
        }
        std::vector<Point2d> run;
        const auto flush = [&]() {
          if (run.size() >= 2) {
            output.constraints.push_back(make_boundary_constraint(
              run, margin_m, Hardness::HARD, 0.0, get_name(), std::to_string(border_id),
              side_left ? "left_road_border" : "right_road_border"));
          }
          run.clear();
        };
        for (const auto & vertex : polyline) {
          const lanelet::BasicPoint2d q{vertex.x(), vertex.y()};
          // The same border can wrap around the road, as in a rotary, and come back on the other
          // side, so keep only the vertices outside the bound on this side
          const auto foot = closest_point_on_polyline(q, bound);
          const double bound_yaw = nearest_segment_yaw(bound, q);
          const double outward =
            (-std::sin(bound_yaw) * (q.x() - foot.x()) + std::cos(bound_yaw) * (q.y() - foot.y())) *
            side_sign;
          if ((q - foot).norm() <= BORDER_SEARCH_RADIUS_M && outward > 0.0) {
            run.push_back(vertex);
          } else {
            flush();
          }
        }
        flush();
      }
    }
  }

  // --- debug marker: the boundary polylines that were emitted ---
  {
    using autoware_utils_visualization::create_default_marker;
    using autoware_utils_visualization::create_marker_color;
    using autoware_utils_visualization::create_marker_scale;

    auto bound_marker = create_default_marker(
      "map", rclcpp::Time(0, 0, RCL_ROS_TIME), "lane_following_drivable_area_bounds", 0,
      Marker::LINE_LIST, create_marker_scale(0.15, 0.0, 0.0),
      create_marker_color(1.0, 0.6, 0.0, 0.9));
    for (const auto & constraint : output.constraints) {
      const auto & boundary = std::get<Boundary>(constraint.payload);
      for (std::size_t i = 0; i + 1 < boundary.polyline.size(); ++i) {
        for (const auto & p : {boundary.polyline[i], boundary.polyline[i + 1]}) {
          geometry_msgs::msg::Point q;
          q.x = p.x();
          q.y = p.y();
          q.z = context.odometry.pose.pose.position.z;
          bound_marker.points.push_back(q);
        }
      }
    }
    if (!bound_marker.points.empty()) {
      MarkerArray marker_array;
      marker_array.markers.push_back(std::move(bound_marker));
      output.debug_markers = std::move(marker_array);
    }
  }

  return output;
}

}  // namespace autoware::safety_planner::experiment

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experiment::LaneFollowingDrivableAreaConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
