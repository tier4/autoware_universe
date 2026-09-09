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

#ifndef UTILS__CONSTRAINTS_COMPILER_HPP_
#define UTILS__CONSTRAINTS_COMPILER_HPP_

// Constraint compilation: turns the generator output (a list of Constraint in world coordinates)
// into the IR the consumers read, CompiledConstraints.
//
// The IR has two layers:
// - raw: every Constraint, flattened, still in world coordinates. **This is the single source of
//   truth**
// - projected views (scalar_bounds / lateral_bounds / stop_bars / occupancies): raw projected onto
//   context.reference_path, for the coarse consumers (the sampling planner, ...). Each entry
//   points back to raw through raw_index
//
// Rules of the projection:
// - the arc length s is measured on **the reference_path of this cycle only**. s = 0 is the rear
//   end of the reference path (backward_length_m behind ego) and it ends at the goal. Consumers do
//   not build their own route-based frame
// - a view may round conservatively, up to about the grid resolution of the DP. Passing in the view
//   and failing on raw is allowed (the outer iteration and the fallback handle it), but rounding so
//   conservatively that it removes solutions is not done silently
// - the Frenet projection is unique only within the curvature radius of the centerline. Geometry
//   that projects ambiguously is dropped from the view and listed in unprojected (it still applies
//   in the exact evaluation because it stays in raw)
// - a point is projected onto its **nearest foot**. At a kink of the centerline the perpendicular
//   bands of the two neighboring segments do not meet, leaving a wedge of width |l*dtheta| on the
//   outside of the turn; a point falling in it is still taken, with its foot clamped to the kink.
//   Dropping it would fail open: for a generator whose boundary vertices are aligned with the
//   centerline samples (simple_drivable_area) the outer vertices would disappear entirely and the
//   lateral constraint over that s range would vanish
// - the IR is rebuilt every cycle. s carries no meaning across cycles (targets are matched across
//   cycles through the id in Source)

#include "../constraint.hpp"
#include "../context.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::safety_planner
{

//! Point in (s, l). s is the arc length [m] along reference_path, l the lateral offset [m] from
//! the centerline (positive to the left).
struct SlPoint
{
  double s{0.0};
  double l{0.0};
};

//! Projection of a ScalarBound. With a region the arc length interval comes from intersecting the
//! region with the centerline (one entry per interval); without one it covers everything.
struct ScalarBoundEntry
{
  BoundedQuantity quantity{BoundedQuantity::VELOCITY};
  double s0{-INF};  //!< [m] closed arc length interval in which the bound holds
  double s1{+INF};
  double min{-INF};
  double max{+INF};
  std::size_t raw_index{0};
};

//! Side of the reference path that a projected boundary forbids. Boundary itself carries no side;
//! it is decided here, from where the polyline falls relative to the reference path.
enum class Side : std::uint8_t { LEFT, RIGHT };

//! Projection of a Boundary: the polyline sampled in (s, l), ascending in s. Combining the entries
//! into the envelopes l_min(s) / l_max(s) is the consumer's job, so one boundary stays one entry.
struct LateralBoundEntry
{
  std::vector<SlPoint> polyline;    //!< ascending in s
  Side forbidden_side{Side::LEFT};  //!< the side of the reference path the boundary closes off
  std::size_t raw_index{0};
};

//! Projection of a Gate: the arc length where the segment crosses the centerline. A gate that does
//! not cross it is absent from the view (it goes to unprojected).
struct StopBarEntry
{
  double s_stop{0.0};  //!< [m] entry is forbidden beyond this s (evaluated on the footprint front)
  TimeWindow time{};   //!< copy of Gate::time
  std::size_t raw_index{0};
};

//! The occupancy of a KeepOut, rounded conservatively into an (s, l) range per time slab: the box
//! bounding the union of the shapes interpolated at both ends of the slab.
struct OccupancySlab
{
  double t0{0.0};  //!< [s] time span of the slab
  double t1{0.0};
  double s0{0.0};  //!< [m] arc length range of the occupancy
  double s1{0.0};
  double l0{0.0};  //!< [m] lateral offset range of the occupancy
  double l1{0.0};
};

struct OccupancyEntry
{
  std::vector<OccupancySlab> slabs;  //!< ascending in t
  std::size_t raw_index{0};
};

//! The compiled constraints (IR). raw is the source of truth, the views are derived from it.
struct CompiledConstraints
{
  //! Every constraint, flattened, in world coordinates. The exact evaluation (carving the
  //! corridor, validation) reads these.
  std::vector<Constraint> raw_constraints;

  // ---- projected views, for the coarse consumers; each entry references raw by raw_index ----
  std::vector<ScalarBoundEntry> scalar_bounds;
  std::vector<LateralBoundEntry> lateral_bounds;
  std::vector<StopBarEntry> stop_bars;
  std::vector<OccupancyEntry> occupancies;

  //! Indices of the raw constraints that could not be projected (a gate that misses the
  //! centerline, ambiguous geometry, malformed IR). Reported in the diagnostics and the debug
  //! markers; they stay in raw, so they still apply in the exact evaluation.
  std::vector<std::size_t> unprojected;
};

CompiledConstraints compile_constraint_list(
  const PlannerContext & context, const std::vector<Constraint> & constraints);

}  // namespace autoware::safety_planner

#endif  // UTILS__CONSTRAINTS_COMPILER_HPP_
