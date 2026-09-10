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
#ifndef UTILS__FRENET_UTILS_HPP_
#define UTILS__FRENET_UTILS_HPP_

#include "../constraint.hpp"
#include "../context.hpp"
#include "../type_alias.hpp"

namespace autoware::safety_planner
{

//! (s, l) on the reference_path of this cycle: s = 0 at its rear end, l positive to the left
struct EgoFrenetState
{
  double s{0.0};
  double l{0.0};
};

EgoFrenetState compute_ego_frenet_state(const PlannerContext & context);

//! Lateral offset of the world point q from the centerline point at arc length s
double lateral_offset_at(const PathPointTrajectory & path, double s, const Point2d & q);

//! The yaw is the centerline tangent
Pose2d to_world_pose(const PathPointTrajectory & path, double s, double l);

//! Footprint bounding box in (s, l), with the heading taken as the one of the centerline; the
//! growth from the heading deviation is not compensated
struct SlBox
{
  double s_min{0.0};
  double s_max{0.0};
  double l_min{0.0};
  double l_max{0.0};
};

//! Footprint with the rear axle at (s, l)
SlBox footprint_sl_box(const VehicleInfo & vehicle_info, double s, double l);

//! Footprint swept while the rear axle moves inside reference_box
SlBox footprint_sl_box(const VehicleInfo & vehicle_info, const SlBox & reference_box);

}  // namespace autoware::safety_planner

#endif  // UTILS__FRENET_UTILS_HPP_
