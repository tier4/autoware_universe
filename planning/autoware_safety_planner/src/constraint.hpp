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

#ifndef CONSTRAINT_HPP_
#define CONSTRAINT_HPP_

#include "type_alias.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace autoware::safety_planner
{

inline constexpr double INF = std::numeric_limits<double>::infinity();

//! How certain the premise of a constraint is. It selects the constraint set the constraint goes
//! into: normal = DEFINITE only, cautious = DEFINITE + POSSIBLE. A generator only declares the
//! certainty of what it emits and does not know how many trajectories are planned.
enum class Certainty : std::uint8_t {
  DEFINITE,
  POSSIBLE,
};

//! Origin of a constraint. Used only to group entries in the report; the pipeline ignores it.
enum class Category : std::uint8_t {
  SAFETY,
  TRAFFIC,
};

//! HARD must be satisfied (no slack). SOFT may be violated with a slack variable, penalized by
//! Constraint::slack_weight.
enum class Hardness : std::uint8_t {
  HARD,
  SOFT,
};

struct TimeWindow
{
  double t0{0.0};
  double t1{INF};
};

struct ArcRange
{
  double s0{-INF};  //!< [m] relative to the projection of ego
  double s1{INF};
};

struct Domain
{
  TimeWindow time{};
  ArcRange arc{};
};

//! Identifies the emitter, for reports, diagnostics, markers and PlanningFactor.
struct Source
{
  std::string plugin_name;
  Category category{Category::SAFETY};
  //! Identifies the entity the constraint is about, and must be **stable across cycles**
  //! (perception UUID, lanelet id, ...); never an arc length or an array index. Empty means the
  //! constraint has no specific target (vehicle kinematics, ...). Two consumers use it:
  //! - key of the discrete decisions, matching the decision on the same target across cycles
  //! - object_id of the SafetyFactor, so the validation layer can tell which object caused a stop
  // TODO(odashima): change to object?
  std::string target_id;

  // TODO(odashima): follow PlanningFactor

  std::string detail;  //!< kind of the constraint, e.g. "stop_line", "dynamic_obstacle"
};

struct Pose2d
{
  Point2d position{0.0, 0.0};
  double yaw{0.0};
};

//! Pose with a time stamp. t is seconds relative to the planning reference time.
struct TimedPose
{
  double t{0.0};
  Pose2d pose{};
};

//! Polygon with a time stamp. t is seconds relative to the planning reference time.
struct TimedPolygon
{
  double t{0.0};
  Polygon2d polygon{};
};

// ---------------------------------------------------------------------------------------------
// payloads
// ---------------------------------------------------------------------------------------------

enum class BoundedQuantity : std::uint8_t {
  VELOCITY,     //!< v      [m/s]
  LON_ACCEL,    //!< a      [m/s^2] the only quantity for which both bounds are meaningful
  LON_JERK,     //!< j      [m/s^3] bound on |j|
  LAT_ACCEL,    //!< v^2|k| [m/s^2]
  CURVATURE,    //!< |k|    [1/m]
  STEER_ANGLE,  //!< |d|    [rad]
  STEER_RATE,   //!< |d'|   [rad/s]
};

//! (i) Scalar box. Without a region it holds everywhere (vehicle kinematics); with one it holds
//! while base_link is inside the region. An emitter that wants to be conservative inflates the
//! region itself.
struct ScalarBound
{
  BoundedQuantity quantity;
  double min{-INF};  //!< unused (left at -INF) for quantities bounded in absolute value
  double max{+INF};
  std::optional<Polygon2d> region{};  //!< nullopt = everywhere
};

//! (ii) Do not cross. A drivable area boundary polyline; the forbidden side is not tagged here,
//! the consumer decides it from where the polyline lies relative to its own reference path.
//! Splitting, face selection and arc length assignment are the consumer's job, so the polyline may
//! stay as the raw map vertices. Static within a cycle and valid for all times.
struct Boundary
{
  LineString2d polyline{};  //!< two vertices or more
  double margin{0.0};       //!< [m] >= 0, the forbidden side is inflated by this much
};

//! Rigid occupancy: a body-local shape moving along a predicted pose sequence. A single waypoint
//! means a static object. Poses are interpolated linearly between waypoints (yaw along the shortest
//! angle); outside the covered time range the occupancy is undefined.
struct RigidBody
{
  Polygon2d shape{};                 //!< in body frame
  std::vector<TimedPose> waypoints;  //!< ascending in t, at least one
};

//! Occupancy given as a sequence of time-varying polygons, for regions that grow or deform (the
//! reachable set of a possible cut-in, ...). Interpolation between polygons is conservative (the
//! union of the two neighbors may be used); outside the covered time range it is undefined.
struct TimedPolygonSequence
{
  std::vector<TimedPolygon> polygons;  //!< ascending in t, at least one
};

//! (iii) Do not occupy. The footprint, inflated by the margin, must not intersect the occupancy.
struct KeepOut
{
  std::variant<RigidBody, TimedPolygonSequence> occupancy{};
  double margin_m{0.0};  //!< [m] >= 0, the consumer inflates the footprint by this much
};

//! (iv) Gate that must not be passed. While the constraint is active (Constraint::domain), the
//! footprint must not cross the segment towards the forbidden side, which is the left of first ->
//! second. A gate can be driven around, so it has to be emitted long enough to cover the width it
//! is meant to block.
struct Gate
{
  Segment2d line{};    //!< directed segment
  double margin{0.0};  //!< [m] >= 0, extra clearance kept in front of the line
};

using ConstraintPayload = std::variant<ScalarBound, Boundary, KeepOut, Gate>;

// ---------------------------------------------------------------------------------------------
// Constraint
// ---------------------------------------------------------------------------------------------

struct Constraint
{
  Certainty certainty{Certainty::DEFINITE};
  Hardness hardness{Hardness::HARD};
  double slack_weight{0.0};  //!< [-] penalty on the slack, only used when hardness is SOFT
  Domain domain{};
  ConstraintPayload payload{};
  Source source{};
};

}  // namespace autoware::safety_planner

#endif  // CONSTRAINT_HPP_
