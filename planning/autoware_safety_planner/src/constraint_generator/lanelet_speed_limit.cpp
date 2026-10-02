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

#include "lanelet_speed_limit.hpp"

#include <boost/geometry/algorithms/correct.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner::experimental
{

namespace
{

constexpr double sample_interval_m = 1.0;  // resolution of the position a limit takes effect at
constexpr double half_width_m = 5.0;       // half width of the band a limit is applied to

// Band of half_width_m around the reference_path from s_begin to s_end
Polygon2d make_band_polygon(
  const PathPointTrajectory & path, const double s_begin, const double s_end)
{
  const auto num_division = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil((s_end - s_begin) / sample_interval_m)));
  std::vector<Pose2d> centerline;
  centerline.reserve(num_division + 1);
  for (std::size_t i = 0; i <= num_division; ++i) {
    const double s = std::min(s_begin + static_cast<double>(i) * sample_interval_m, s_end);
    const auto position = path.compute(s).point.pose.position;
    centerline.push_back(Pose2d{Point2d{position.x, position.y}, path.azimuth(s)});
  }

  Polygon2d polygon;
  auto & ring = polygon.outer();
  ring.reserve(2 * centerline.size() + 1);
  for (const auto & pose : centerline) {
    ring.emplace_back(
      pose.position.x() - half_width_m * std::sin(pose.yaw),
      pose.position.y() + half_width_m * std::cos(pose.yaw));
  }
  for (auto it = centerline.rbegin(); it != centerline.rend(); ++it) {
    ring.emplace_back(
      it->position.x() + half_width_m * std::sin(it->yaw),
      it->position.y() - half_width_m * std::cos(it->yaw));
  }
  boost::geometry::correct(polygon);
  return polygon;
}

std::optional<std::size_t> match_belonging_lanelet(
  const lanelet::ConstLanelets & lanelets, const std::size_t from,
  const lanelet::BasicPoint2d & point)
{
  for (std::size_t i = from; i < lanelets.size(); ++i) {
    if (boost::geometry::within(point, lanelets[i].polygon2d().basicPolygon())) {
      return i;
    }
  }
  return std::nullopt;
}

}  // namespace

ConstraintGeneratorOutput LaneletSpeedLimitConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  ConstraintGeneratorOutput output;

  // Without a route there is no lane sequence to read the limits from; an empty output keeps the
  // pipeline running
  if (!context.route_manager) {
    return output;
  }

  const auto & route_manager = *context.route_manager;
  const auto lanelets =
    route_manager.get_lanelet_sequence_on_route(params_.reference_path.forward_length_m, 0.0)
      .as_lanelets();
  const auto & path = context.reference_path;
  const double length = path.length();
  if (lanelets.empty() || length <= 0.0) {
    return output;
  }
  const auto traffic_rules = route_manager.traffic_rules_ptr();
  const auto current_id = route_manager.current_lanelet().id();
  const double v_ego = std::max(0.0, context.odometry.twist.twist.linear.x);

  struct LaneletSpan  // <'a>
  {
    std::size_t lanelet_index;  // <'a> of lanelets
    double s_begin;
    double s_end;
  };
  std::vector<LaneletSpan> spans;
  const auto num_division = static_cast<std::size_t>(std::ceil(length / sample_interval_m));
  std::size_t index = 0;
  for (std::size_t i = 0; i <= num_division; ++i) {
    const double s = std::min(static_cast<double>(i) * sample_interval_m, length);
    const auto position = path.compute(s).point.pose.position;
    // NOTE(soblin): safety_planner never executes lane change, so the reference path points are
    // always on the route lanelets
    index = match_belonging_lanelet(lanelets, index, lanelet::BasicPoint2d(position.x, position.y))
              .value_or(index);
    if (spans.empty() || spans.back().lanelet_index != index) {
      if (!spans.empty()) {
        spans.back().s_end = s;
      }
      spans.push_back(LaneletSpan{index, spans.empty() ? 0.0 : s, length});
    }
  }

  for (const auto & span : spans) {
    const auto & lanelet = lanelets[span.lanelet_index];
    const double v_limit =
      static_cast<double>(traffic_rules->speedLimit(lanelet).speedLimit.value());

    auto region = make_band_polygon(path, span.s_begin, span.s_end);
    // The path is built from the map, but a degenerate one is no region
    if (region.outer().size() < 4) {
      continue;
    }

    Constraint constraint;
    constraint.payload = SpeedLimitZone{
      std::move(region), lanelet.id() == current_id ? std::max(v_limit, v_ego) : v_limit};
    constraint.source = Source{get_name(), std::to_string(lanelet.id()), "speed_limit"};
    output.constraints.push_back(std::move(constraint));
  }

  return output;
}

}  // namespace autoware::safety_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experimental::LaneletSpeedLimitConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
