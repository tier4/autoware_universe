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

#include "next_lanelet_speed_limit.hpp"

#include <boost/geometry/algorithms/correct.hpp>

#include <string>
#include <utility>

namespace autoware::safety_planner::experiment
{

ConstraintGeneratorOutput NextLaneletSpeedLimitConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  ConstraintGeneratorOutput output;
  if (!context.route_manager) {
    return output;
  }

  const auto & route_manager = *context.route_manager;
  const auto lanelets =
    route_manager.get_lanelet_sequence_on_route(params_.reference_path.forward_length_m, 0.0)
      .as_lanelets();
  const auto current_id = route_manager.current_lanelet().id();
  for (std::size_t i = 0; i + 1 < lanelets.size(); ++i) {
    if (lanelets[i].id() != current_id) {
      continue;
    }
    const auto & next = lanelets[i + 1];
    Polygon2d region;
    for (const auto & p : next.polygon2d().basicPolygon()) {
      region.outer().emplace_back(p.x(), p.y());
    }
    boost::geometry::correct(region);

    Constraint constraint;
    constraint.payload = ScalarBound{
      BoundedQuantity::VELOCITY, 0.0, params_.next_lanelet_speed_limit.velocity_mps,
      std::move(region)};
    constraint.source = Source{get_name(), std::to_string(next.id()), "speed_limit"};
    output.constraints.push_back(std::move(constraint));
    break;
  }
  return output;
}

}  // namespace autoware::safety_planner::experiment

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experiment::NextLaneletSpeedLimitConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
