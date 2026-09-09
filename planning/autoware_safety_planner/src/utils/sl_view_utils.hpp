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

#ifndef UTILS__SL_VIEW_UTILS_HPP_
#define UTILS__SL_VIEW_UTILS_HPP_

// Helpers shared by everything that reads the projected views of CompiledConstraints, i.e. the
// (s, l) space. As stated in constraints_compiler.hpp, s is always measured on the reference_path
// of the current cycle.

#include "../context.hpp"
#include "../type_alias.hpp"
#include "constraints_compiler.hpp"

#include <vector>

namespace autoware::safety_planner
{

// ---------------------------------------------------------------------------------------------
// Kinematic limits of the vehicle, read from the IR
// ---------------------------------------------------------------------------------------------

//! The ScalarBound constraints that hold everywhere and at all times, collected from the IR. They
//! come from the vehicle_kinematics plugin, whose ROS parameters are the source of truth. Only the
//! hard limits (v_hard, a_hard_*) are read from the IR; the nominal values keep the defaults below.
//! Make them configurable through the parameters rather than through the IR.
struct KinematicLimits
{
  double v_hard{16.7};      //!< [m/s] hard speed limit
  double a_hard_min{-6.0};  //!< [m/s^2] hardest deceleration
  double a_hard_max{6.0};   //!< [m/s^2] hardest acceleration
  double v_nom{13.88};      //!< [m/s] cruising speed
  double a_nom_min{-1.0};   //!< [m/s^2] comfortable deceleration
  double a_nom_max{1.0};    //!< [m/s^2] comfortable acceleration
  double a_lat_nom{2.0};    //!< [m/s^2] lateral acceleration the corner deceleration aims at
};

KinematicLimits collect_kinematic_limits(const CompiledConstraints & compiled_constraints);

// ---------------------------------------------------------------------------------------------
// Frenet coordinates on the reference_path
// ---------------------------------------------------------------------------------------------

//! Arc length and lateral offset (positive to the left) on the reference_path
struct EgoFrenetState
{
  double s{0.0};
  double l{0.0};
};

//! Projects the ego position onto the reference_path
EgoFrenetState compute_ego_frenet_state(const PlannerContext & context);

//! Lateral offset of the world point q, seen from the centerline point at arc length s
double lateral_offset_at(const PathPointTrajectory & path, double s, const Point2d & q);

//! Converts (s, l) back to world coordinates. The yaw is the centerline tangent.
Pose2d to_world_pose(const PathPointTrajectory & path, double s, double l);

// ---------------------------------------------------------------------------------------------
// Bounding box of the footprint in (s, l)
// ---------------------------------------------------------------------------------------------

//! Box bounding the footprint in (s, l): the conservative approximation the projected views are
//! evaluated with. The heading is taken as the one of the centerline, and the growth from the
//! heading deviation is absorbed by the margins.
struct SlBox
{
  double s_min{0.0};
  double s_max{0.0};
  double l_min{0.0};
  double l_max{0.0};
};

//! Bounding box of the footprint with the reference point (the rear axle) at (s, l)
SlBox footprint_sl_box(const VehicleInfo & vehicle_info, double s, double l);

//! Bounding box of the footprint swept while the reference point moves inside the given box
SlBox footprint_sl_box(const VehicleInfo & vehicle_info, const SlBox & reference_box);

// ---------------------------------------------------------------------------------------------
// Evaluation of the projected views
// ---------------------------------------------------------------------------------------------

//! Interpolates l of a boundary polyline (ascending in s) linearly at the arc length s
double interpolate_boundary_l(const std::vector<SlPoint> & polyline, double s);

//! Extremum of the boundary l over the arc length interval [s_lo, s_hi]: min(l) when the boundary
//! forbids its left, max(l) when it forbids its right, i.e. the tightest value of the interval.
//! Returns false without touching extreme_l when the interval misses the polyline.
bool lateral_bound_extreme_l(
  const LateralBoundEntry & bound, double s_lo, double s_hi, double & extreme_l);

//! Whether the footprint box reaches into the forbidden side of the boundary, margin included
bool violates_lateral_bound(const LateralBoundEntry & bound, const SlBox & box);

//! Whether the footprint box overlaps an occupancy slab during [t0, t1], inflated by
//! KeepOut::margin_m
bool violates_occupancy(
  const OccupancyEntry & occupancy, const CompiledConstraints & compiled_constraints,
  const SlBox & box, double t0, double t1);

//! Whether the front of the footprint passes a stop line that is active during [t0, t1]
bool violates_stop_bar(const StopBarEntry & stop_bar, const SlBox & box, double t0, double t1);

}  // namespace autoware::safety_planner

#endif  // UTILS__SL_VIEW_UTILS_HPP_
