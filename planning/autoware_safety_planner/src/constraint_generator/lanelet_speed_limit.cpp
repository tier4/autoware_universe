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

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <algorithm>
#include <string>
#include <utility>

namespace autoware::safety_planner::experiment
{

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

  // Cover the same window of lanes as the reference_path, sharing its parameters
  const auto & route_manager = *context.route_manager;
  const auto lanelets =
    route_manager
      .get_lanelet_sequence_on_route(
        params_.reference_path.forward_length_m, params_.reference_path.backward_length_m)
      .as_lanelets();
  const auto traffic_rules = route_manager.traffic_rules_ptr();
  const auto current_id = route_manager.current_lanelet().id();
  const double v_ego = std::max(0.0, context.odometry.twist.twist.linear.x);

  for (const auto & lanelet : lanelets) {
    const double v_limit =
      static_cast<double>(traffic_rules->speedLimit(lanelet).speedLimit.value());

    Polygon2d region;
    auto & ring = region.outer();
    const auto polygon = lanelet.polygon2d().basicPolygon();
    ring.reserve(polygon.size() + 1);
    for (const auto & point : polygon) {
      ring.emplace_back(point.x(), point.y());
    }
    boost::geometry::correct(region);
    // The map is an external resource: a lanelet whose bounds degenerate is no region
    if (ring.size() < 4) {
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

}  // namespace autoware::safety_planner::experiment

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experiment::LaneletSpeedLimitConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
