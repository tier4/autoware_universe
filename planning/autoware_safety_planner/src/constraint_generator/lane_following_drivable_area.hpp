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

#ifndef AUTOWARE__SAFETY_PLANNER__CONSTRAINT_GENERATOR__LANE_FOLLOWING_DRIVABLE_AREA_HPP_
#define AUTOWARE__SAFETY_PLANNER__CONSTRAINT_GENERATOR__LANE_FOLLOWING_DRIVABLE_AREA_HPP_

#include "constraint_generator_interface.hpp"

#include <string>

namespace autoware::safety_planner
{

//! Builds the drivable area constraints (Boundary) from the map. For every lanelet of the lane
//! sequence on the route, on each side:
//! - if there is a parallel lane next to it (oncoming included; road subtype, heading within
//!   +-45 deg), the **bound of the own lane** becomes the boundary, forbidding a lane change
//! - otherwise the nearest **road_border** (the physical edge of the road) becomes the boundary, so
//!   that shoulders and zebras stay inside the drivable area
//!
//! Why not extend over the shoulder lanelets (left/right_shoulder_lanelet): the adjacency lookup
//! assumes the linestring objects are shared, and never fires on a map that duplicates the
//! linestring per lanelet. For the same reason the presence of a parallel lane is decided
//! geometrically, from sampled points, rather than through the routing graph.
class LaneFollowingDrivableAreaConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "lane_following_drivable_area"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__CONSTRAINT_GENERATOR__LANE_FOLLOWING_DRIVABLE_AREA_HPP_
