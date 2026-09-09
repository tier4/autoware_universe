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
//! into: normal = DEFINITE only, cautious = DEFINITE + POSSIBLE.
enum class Certainty : std::uint8_t {
  DEFINITE,
  POSSIBLE,
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

struct Source
{
  std::string plugin_name;
  //! Identifies the entity the constraint is about, and must be stable across cycles
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

struct TimedPose
{
  double t{0.0};
  Pose2d pose{};
};

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

struct ScalarBound
{
  BoundedQuantity quantity;
  double min{-INF};  //!< unused (left at -INF) for quantities bounded in absolute value
  double max{+INF};

  // TODO(odashima): need region for lateral range constraint?
  std::optional<Polygon2d> region{};  //!< nullopt = everywhere
};

struct Boundary
{
  LineString2d polyline{};  //!< two vertices or more
};

struct RigidBody
{
  Polygon2d shape{};                 //!< in body frame
  std::vector<TimedPose> waypoints;  //!< ascending in t, at least one
};

struct TimedPolygonSequence
{
  std::vector<TimedPolygon> polygons;  //!< ascending in t, at least one
};

struct KeepOut
{
  std::variant<RigidBody, TimedPolygonSequence> occupancy{};
};

struct Gate
{
  Segment2d line{};
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
