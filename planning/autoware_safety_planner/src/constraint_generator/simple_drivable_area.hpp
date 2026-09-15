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

#ifndef CONSTRAINT_GENERATOR__SIMPLE_DRIVABLE_AREA_HPP_
#define CONSTRAINT_GENERATOR__SIMPLE_DRIVABLE_AREA_HPP_

#include "constraint_generator_interface.hpp"

#include <string>
#include <vector>

namespace autoware::safety_planner::experiment
{

//! Takes the reference_path widened by a constant half width and extended by a constant length at
//! both ends as the drivable area. A stand-in that does not read the map; the real area comes from
//! the lane bounds and road_border of the lanelets.
//!
//! The constraint IR has no "stay inside this polygon" payload, so the polygon is emitted as its
//! left and right side, one Boundary each. The ends of those polylines are the ends of the area.
//! The polygon itself is published as a debug marker.
class SimpleDrivableAreaConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "simple_drivable_area"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

//! Builds the two offset polylines and the closed polygon from centerline samples (pose + yaw).
//! The polygon is returned clockwise and closed (bg::correct applied).
struct DrivableAreaShape
{
  std::vector<Point2d> left;   //!< in driving direction
  std::vector<Point2d> right;  //!< in driving direction
  Polygon2d polygon{};
};

DrivableAreaShape make_drivable_area_shape(
  const std::vector<Pose2d> & centerline, double half_width_m, double forward_extension_m,
  double backward_extension_m);

}  // namespace autoware::safety_planner::experiment

#endif  // CONSTRAINT_GENERATOR__SIMPLE_DRIVABLE_AREA_HPP_
