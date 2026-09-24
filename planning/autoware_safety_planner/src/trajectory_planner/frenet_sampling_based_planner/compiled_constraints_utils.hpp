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
#ifndef TRAJECTORY_PLANNER__FRENET_SAMPLING_BASED_PLANNER__COMPILED_CONSTRAINTS_UTILS_HPP_
#define TRAJECTORY_PLANNER__FRENET_SAMPLING_BASED_PLANNER__COMPILED_CONSTRAINTS_UTILS_HPP_

#include "../../utils/frenet_utils.hpp"
#include "constraints_compiler.hpp"

#include <vector>

namespace autoware::safety_planner
{

//! Only the hard limits are read from the IR (the global ScalarBounds of vehicle_kinematics); the
//! nominal values keep the defaults below until they get parameters of their own
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

//! The lateral envelope a Boundary imposes, as a polyline ascending in s: at each s, the l of the
//! boundary nearest to the reference path on the forbidden side, i.e. where a ray cast from the
//! centerline towards that side first hits the boundary. pieces are runs of consecutive projected
//! vertices, in polyline order; the polyline is broken where a vertex could not be taken
std::vector<SlPoint> make_lateral_envelope(
  const std::vector<std::vector<SlPoint>> & pieces, Side forbidden_side);

//! The lateral bounds around one point of the reference path, in the Cartesian frame there (x
//! along the tangent, y to the left), binned along x: per bin, the smallest y that a boundary
//! forbidding its left reaches and the largest y of one forbidding its right. The footprint is
//! checked here as a rigid rectangle: as a box in (s, l) its corners are off by about
//! x^2 k / 2 + x sin(theta), over a meter at the front of a bus on the outside of a tight curve
struct BoundaryProfile
{
  Pose2d frame{};
  double x0{0.0};             //!< [m] start of the first bin
  double bin{1.0};            //!< [m] bin width
  std::vector<double> left;   //!< +INF where no boundary reaches
  std::vector<double> right;  //!< -INF where no boundary reaches
  double left_min{INF};
  double right_max{-INF};
};

//! Bins the pieces of bounds over [x_min, x_max] of frame. Only the segments with a vertex within
//! [s_min, s_max] are taken, which keeps the other leg of a hairpin out
BoundaryProfile make_boundary_profile(
  const std::vector<const LateralBoundEntry *> & bounds, const Pose2d & frame, double x_min,
  double x_max, double s_min, double s_max, double bin);

//! Whether the footprint with the rear axle at rear_axle (world coordinates), stretched by
//! longitudinal_margin at both ends, reaches past a boundary of profile. The part of the footprint
//! outside the bins of profile is not checked
bool footprint_hits_boundary(
  const BoundaryProfile & profile, const VehicleInfo & vehicle_info, const Pose2d & rear_axle,
  double longitudinal_margin);

//! Linear in s, clamped outside the polyline
double interpolate_boundary_l(const std::vector<SlPoint> & polyline, double s);

//! Tightest l of the boundary over [s_lo, s_hi]: min(l) when it forbids its left, max(l) when
//! its right. Returns false when the interval misses the polyline
bool lateral_bound_extreme_l(
  const LateralBoundEntry & bound, double s_lo, double s_hi, double & extreme_l);

bool violates_lateral_bound(const LateralBoundEntry & bound, const SlBox & box);

//! [t0, t1] is the time the footprint box is occupied by ego
bool violates_occupancy(const OccupancyEntry & occupancy, const SlBox & box, double t0, double t1);

//! Evaluated on the front of the footprint box
bool violates_stop_bar(const StopBarEntry & stop_bar, const SlBox & box, double t0, double t1);

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__FRENET_SAMPLING_BASED_PLANNER__COMPILED_CONSTRAINTS_UTILS_HPP_
