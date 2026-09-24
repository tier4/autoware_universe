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

#ifndef CONSTRAINT_GENERATOR__LANE_FOLLOWING_DRIVABLE_AREA_HPP_
#define CONSTRAINT_GENERATOR__LANE_FOLLOWING_DRIVABLE_AREA_HPP_

#include "constraint_generator_interface.hpp"

#include <string>

namespace autoware::safety_planner::experimental
{

//! Builds the drivable area constraints (Boundary) from the map:
//! - both bounds of every lanelet of the lane sequence on the route become **soft**, so that the
//!   ego keeps to its own lane unless something forces it out
//! - every **road_border** (the physical edge of the road) within road_border_distance_m of the
//!   reference_path becomes **hard**, on either side and regardless of the lanes in between
//! - optionally (close_walkway_gap), where the reference_path crosses a walkway lanelet, the gap
//! the
//!   sidewalk leaves between the road_borders is closed by a **hard** segment on each side
class LaneFollowingDrivableAreaConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "lane_following_drivable_area"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

}  // namespace autoware::safety_planner::experimental

#endif  // CONSTRAINT_GENERATOR__LANE_FOLLOWING_DRIVABLE_AREA_HPP_
