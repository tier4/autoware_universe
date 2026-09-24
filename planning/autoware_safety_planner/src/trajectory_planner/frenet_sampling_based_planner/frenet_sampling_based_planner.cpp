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

#include "frenet_sampling_based_planner.hpp"

#include "../../utils/frenet_utils.hpp"

#include <autoware_frenet_planner/polynomials.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::safety_planner::experimental
{

namespace
{

//! Linear interpolation of samples spaced res apart from s0, clamped at both ends
double interpolate_uniform(
  const std::vector<double> & values, const double s0, const double res, const double s)
{
  if (values.size() < 2) {
    return values.front();
  }
  const double u = std::clamp((s - s0) / res, 0.0, static_cast<double>(values.size() - 1));
  const auto i = std::min(static_cast<std::size_t>(u), values.size() - 2);
  const double r = u - static_cast<double>(i);
  return values[i] * (1.0 - r) + values[i + 1] * r;
}

//! Linear interpolation of headings, wrapping at 2 pi
double interpolate_uniform_angle(
  const std::vector<double> & yaws, const double s0, const double res, const double s)
{
  if (yaws.size() < 2) {
    return yaws.front();
  }
  const double u = std::clamp((s - s0) / res, 0.0, static_cast<double>(yaws.size() - 1));
  const auto i = std::min(static_cast<std::size_t>(u), yaws.size() - 2);
  const double r = u - static_cast<double>(i);
  const double d = autoware_utils_math::normalize_radian(yaws[i + 1] - yaws[i]);
  return autoware_utils_math::normalize_radian(yaws[i] + d * r);
}

//! Curvature of the offset path r = p_ref + l n at a point where the centerline has curvature
//! k (and derivative dk), from theta = psi_ref + atan(l' / a) differentiated by the path arc
//! length: kappa = (k + (a l'' + l' (dk l + k l')) / m^2) / m, a = 1 - k l, m = |r'| = hypot(a, l')
double offset_path_curvature(
  const double k, const double dk, const double l, const double dl, const double d2l)
{
  const double a = 1.0 - k * l;
  // m vanishes at the center of curvature of the centerline; floored as in
  // compute_ego_frenet_state
  const double m = std::max(std::hypot(a, dl), 0.2);
  return (k + (a * d2l + dl * (dk * l + k * dl)) / (m * m)) / m;
}

//! l'' that gives the path the curvature kappa: offset_path_curvature solved for d2l
double offset_path_d2l(
  const double k, const double dk, const double l, const double dl, const double kappa)
{
  const double a = 1.0 - k * l;
  const double m = std::max(std::hypot(a, dl), 0.2);
  return ((kappa * m - k) * m * m - dl * (dk * l + k * dl)) / (std::abs(a) < 0.2 ? 0.2 : a);
}

//! The tightest boundary l over the s window that the footprint covers from anywhere within a
//! cell, one entry per cell: min(l) for a boundary that forbids its left, max(l) for one that
//! forbids its right, and +-INF where the boundary does not reach the window. Same as
//! lateral_bound_extreme_l, except that the window is rounded out to whole cells, which can only
//! tighten the bound
std::vector<double> tabulate_lateral_bound(
  const LateralBoundEntry & bound, const VehicleInfo & vehicle_info, const double res,
  const std::size_t cells)
{
  const bool left = bound.forbidden_side == Side::LEFT;
  const double none = left ? INF : -INF;
  const auto tighter = [left](const double a, const double b) {
    return left ? std::min(a, b) : std::max(a, b);
  };
  const auto & polyline = bound.polyline;
  std::vector<double> window(cells, none);
  if (polyline.size() < 2) {
    return window;  // as in lateral_bound_extreme_l, such a boundary applies nowhere
  }

  // The extreme within each cell, walking the polyline once: s only grows from cell to cell, so
  // both the segment holding it and the vertices inside it are found by advancing an index
  std::vector<double> in_cell(cells, none);
  std::size_t seg = 0;
  std::size_t vertex = 0;
  const auto l_at = [&](const double s) {
    while (seg + 2 < polyline.size() && polyline[seg + 1].s < s) {
      ++seg;
    }
    const auto & p0 = polyline[seg];
    const auto & p1 = polyline[seg + 1];
    const double r = p1.s > p0.s ? std::clamp((s - p0.s) / (p1.s - p0.s), 0.0, 1.0) : 0.0;
    return p0.l * (1.0 - r) + p1.l * r;
  };
  for (std::size_t i = 0; i < cells; ++i) {
    const double s_lo = std::max(static_cast<double>(i) * res, polyline.front().s);
    const double s_hi = std::min(static_cast<double>(i + 1) * res, polyline.back().s);
    if (s_hi < s_lo) {
      continue;
    }
    double value = tighter(l_at(s_lo), l_at(s_hi));
    while (vertex < polyline.size() && polyline[vertex].s <= s_lo) {
      ++vertex;
    }
    for (; vertex < polyline.size() && polyline[vertex].s < s_hi; ++vertex) {
      value = tighter(value, polyline[vertex].l);
    }
    in_cell[i] = value;
  }

  const auto count = static_cast<std::ptrdiff_t>(cells);
  const auto behind =
    static_cast<std::ptrdiff_t>(std::floor(vehicle_info.min_longitudinal_offset_m / res));
  const auto ahead =
    static_cast<std::ptrdiff_t>(std::floor(vehicle_info.max_longitudinal_offset_m / res)) + 1;
  for (std::ptrdiff_t i = 0; i < count; ++i) {
    double value = none;
    for (std::ptrdiff_t j = std::max<std::ptrdiff_t>(i + behind, 0);
         j <= std::min<std::ptrdiff_t>(i + ahead, count - 1); ++j) {
      value = tighter(value, in_cell[j]);
    }
    window[i] = value;
  }
  return window;
}

}  // namespace

FrenetSamplingBasedPlanner::ConstraintTables::ConstraintTables(
  const PlannerContext & context, const ReferenceGrid & grid,
  const CompiledConstraints & compiled_constraints, const double resolution)
: res(resolution), limits(collect_kinematic_limits(compiled_constraints))
{
  cells = static_cast<std::size_t>(std::ceil(context.reference_path.length() / res)) + 1;

  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (!(bound.s0 == -INF && bound.s1 == INF)) {
      continue;  // a bound limited to an interval is read per cell, from v_max
    }
    switch (bound.quantity) {
      case BoundedQuantity::LAT_ACCEL:
        bounds.lat_accel = std::min(bounds.lat_accel, bound.max);
        break;
      case BoundedQuantity::LON_JERK:
        bounds.lon_jerk = std::min(bounds.lon_jerk, bound.max);
        break;
      case BoundedQuantity::STEER_ANGLE:
        bounds.steer_angle = std::min(bounds.steer_angle, bound.max);
        break;
      case BoundedQuantity::STEER_RATE:
        bounds.steer_rate = std::min(bounds.steer_rate, bound.max);
        break;
      default:
        break;  // VELOCITY and LON_ACCEL belong to KinematicLimits
    }
  }

  // The global VELOCITY bounds are already in v_hard; the regional ones (a speed limit zone) are
  // applied to every cell they touch, so that a zone starting inside a cell is not missed
  v_max.assign(cells, limits.v_hard);
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (bound.quantity != BoundedQuantity::VELOCITY || (bound.s0 == -INF && bound.s1 == INF)) {
      continue;
    }
    for (std::size_t i = cell(bound.s0); i <= cell(bound.s1); ++i) {
      v_max[i] = std::min(v_max[i], bound.max);
    }
  }

  lateral_is_hard.reserve(compiled_constraints.lateral_bounds.size());
  lateral_extreme_l.reserve(compiled_constraints.lateral_bounds.size());
  std::vector<const LateralBoundEntry *> hard_bounds;
  for (const auto & bound : compiled_constraints.lateral_bounds) {
    const bool hard =
      compiled_constraints.raw_constraints[bound.raw_index].hardness == Hardness::HARD;
    lateral_is_hard.push_back(hard);
    lateral_extreme_l.push_back(
      hard ? std::vector<double>{}
           : tabulate_lateral_bound(bound, context.vehicle_info, res, cells));
    if (hard) {
      hard_bounds.push_back(&bound);
    }
  }

  // The bins cover the footprint with the rear axle anywhere in the cell, at any heading. The
  // margin holds the stretch of footprint_hits_boundary (half a time step of travel, below 1 m up
  // to 60 km/h) and the lateral offset of the rear axle along the turning frame
  constexpr double BIN_M = 0.25;
  constexpr double X_MARGIN_M = 2.0;
  const auto & vehicle_info = context.vehicle_info;
  const double half_width =
    std::max(vehicle_info.max_lateral_offset_m, -vehicle_info.min_lateral_offset_m);
  const double reach = std::hypot(
    std::max(vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m),
    half_width);
  const double x_min = -reach - X_MARGIN_M;
  const double x_max = res + reach + X_MARGIN_M;
  // A boundary point is taken by the arc length of its foot, and on the inside of a curve that
  // runs ahead of x by 1 / (1 - k l); twice the reach covers |k l| up to 0.5, i.e. an inner curb
  // at 3 m on the tightest curve of a bus (R 6.4 m). The window only keeps the other leg of a
  // hairpin out, so it is not tightened further
  boundary_profiles.reserve(cells);
  const double length = context.reference_path.length();
  for (std::size_t i = 0; i < cells; ++i) {
    const double s = std::min(static_cast<double>(i) * res, length);
    const Pose2d frame{grid.position(s, 0.0), grid.azimuth(s)};
    boundary_profiles.push_back(make_boundary_profile(
      hard_bounds, frame, x_min, x_max, s + 2.0 * x_min, s + 2.0 * x_max, BIN_M));
  }
}

std::size_t FrenetSamplingBasedPlanner::ConstraintTables::cell(const double s) const
{
  return static_cast<std::size_t>(std::clamp(s / res, 0.0, static_cast<double>(cells - 1)));
}

FrenetSamplingBasedPlanner::ReferenceGrid::ReferenceGrid(
  const PathPointTrajectory & path, const double resolution)
{
  const double s_max = path.length();
  // The spacing is shrunk so that an integer number of intervals covers the path exactly: the
  // lookups below take a uniform grid, and a shorter last interval would shift everything queried
  // within it
  const auto intervals = static_cast<std::size_t>(std::ceil(s_max / resolution));
  res_ = s_max / static_cast<double>(intervals);
  const std::size_t size = intervals + 1;
  for (auto * v : {&x_, &y_, &z_, &yaw_, &cos_yaw_, &sin_yaw_, &curvature_, &dkappa_}) {
    v->reserve(size);
  }
  for (std::size_t i = 0; i < size; ++i) {
    const double s = std::min(static_cast<double>(i) * res_, s_max);
    const auto position = path.compute(s).point.pose.position;
    const double yaw = path.azimuth(s);
    x_.push_back(position.x);
    y_.push_back(position.y);
    z_.push_back(position.z);
    yaw_.push_back(yaw);
    cos_yaw_.push_back(std::cos(yaw));
    sin_yaw_.push_back(std::sin(yaw));
    curvature_.push_back(path.curvature(s));
  }
  for (std::size_t i = 0; i < size; ++i) {
    const std::size_t i0 = i > 0 ? i - 1 : i;
    const std::size_t i1 = std::min(i + 1, size - 1);
    dkappa_.push_back((curvature_[i1] - curvature_[i0]) / (static_cast<double>(i1 - i0) * res_));
  }
}

double FrenetSamplingBasedPlanner::ReferenceGrid::curvature(const double s) const
{
  return interpolate_uniform(curvature_, 0.0, res_, s);
}

double FrenetSamplingBasedPlanner::ReferenceGrid::dkappa(const double s) const
{
  return interpolate_uniform(dkappa_, 0.0, res_, s);
}

double FrenetSamplingBasedPlanner::ReferenceGrid::azimuth(const double s) const
{
  return interpolate_uniform_angle(yaw_, 0.0, res_, s);
}

double FrenetSamplingBasedPlanner::ReferenceGrid::z(const double s) const
{
  return interpolate_uniform(z_, 0.0, res_, s);
}

Point2d FrenetSamplingBasedPlanner::ReferenceGrid::position(const double s, const double l) const
{
  const double u = std::clamp(s / res_, 0.0, static_cast<double>(x_.size() - 1));
  const auto i = std::min(static_cast<std::size_t>(u), x_.size() - 2);
  const double r = u - static_cast<double>(i);
  // The tangent is interpolated as (cos, sin) rather than as the angle: this runs once per point
  // of every candidate, and over one grid interval the two differ by less than a millimeter
  const auto blend = [&](const std::vector<double> & v) { return v[i] * (1.0 - r) + v[i + 1] * r; };
  return Point2d{blend(x_) - blend(sin_yaw_) * l, blend(y_) + blend(cos_yaw_) * l};
}

FrenetSamplingBasedPlanner::PreviousLateral::PreviousLateral(
  const PathPointTrajectory & reference_path, const Trajectory & previous)
{
  const PathProjector projector(reference_path);
  s.reserve(previous.points.size());
  l.reserve(previous.points.size());
  for (const auto & point : previous.points) {
    const double projected_s = projector.closest(point.pose.position);
    // The projection is not monotonic where the previous trajectory barely moves: a stopped ego
    // puts all its points within a few millimeters of the same s, in no particular order. Only the
    // increasing ones are kept, which is what at() searches on
    if (!s.empty() && projected_s <= s.back()) {
      continue;
    }
    s.push_back(projected_s);
    l.push_back(lateral_offset_at(
      reference_path, projected_s, Point2d{point.pose.position.x, point.pose.position.y}));
  }
}

std::optional<double> FrenetSamplingBasedPlanner::PreviousLateral::at(const double query_s) const
{
  if (s.size() < 2) {
    return std::nullopt;
  }
  const auto it = std::lower_bound(s.begin(), s.end(), query_s);
  if (it == s.begin()) {
    return l.front();
  }
  if (it == s.end()) {
    return l.back();
  }
  const auto i = static_cast<std::size_t>(std::distance(s.begin(), it));
  const double r = (query_s - s[i - 1]) / (s[i] - s[i - 1]);
  return l[i - 1] * (1.0 - r) + l[i] * r;
}

void FrenetSamplingBasedPlanner::on_initialize(
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper, const Params & params)
{
  TrajectoryPlannerInterface::on_initialize(time_keeper, params);
  const TurnSignalParams turn_signal_params{
    params.turn_signal.search_distance, params.turn_signal.min_blink_duration,
    params.turn_signal.stopped_velocity_threshold, params.turn_signal.heading_align_threshold};
  normal_turn_indicator_decider_.update_params(turn_signal_params);
  cautious_turn_indicator_decider_.update_params(turn_signal_params);
  constexpr std::size_t kBoundaryCacheSize = 256;
  soft_boundary_simplifier_ = std::make_unique<BoundarySimplifier>(
    params.frenet_sampling_based_planner.boundary.soft_simplify_tolerance_m, kBoundaryCacheSize);
  hard_boundary_simplifier_ = std::make_unique<BoundarySimplifier>(
    params.frenet_sampling_based_planner.boundary.hard_simplify_tolerance_m, kBoundaryCacheSize);
}

TrajectoryPlannerResult FrenetSamplingBasedPlanner::plan_trajectories(
  const TrajectoryPlannerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  auto grid_st =
    std::make_unique<autoware_utils_debug::ScopedTimeTrack>("build_reference_grid", *time_keeper_);
  const ReferenceGrid grid(
    input.context.reference_path, params_.frenet_sampling_based_planner.path_resolution_m);
  grid_st.reset();

  TrajectoryPlannerResult result;
  {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_normal", *time_keeper_);
    if (
      auto trajectory = plan_one_side(
        input.context, grid, input.normal_constraints, normal_previous_trajectory_,
        result.normal_debug)) {
      auto turn_st = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
        "decide_turn_indicators", *time_keeper_);
      const auto turn_indicators =
        normal_turn_indicator_decider_.decide(input.context, *trajectory);
      turn_st.reset();
      normal_previous_trajectory_ = *trajectory;
      result.normal_trajectory = PlannedTrajectory{std::move(*trajectory), turn_indicators};
    }
  }
  const bool cautious_differs = std::any_of(
    input.cautious_constraints.begin(), input.cautious_constraints.end(),
    [](const Constraint & constraint) { return constraint.certainty == Certainty::POSSIBLE; });
  if (cautious_differs) {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_cautious", *time_keeper_);
    if (
      auto trajectory = plan_one_side(
        input.context, grid, input.cautious_constraints, cautious_previous_trajectory_,
        result.cautious_debug)) {
      auto turn_st = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
        "decide_turn_indicators", *time_keeper_);
      const auto turn_indicators =
        cautious_turn_indicator_decider_.decide(input.context, *trajectory);
      turn_st.reset();
      cautious_previous_trajectory_ = *trajectory;
      result.cautious_trajectory = PlannedTrajectory{std::move(*trajectory), turn_indicators};
    }
  } else {
    // Copied rather than solved again: the sampling is deterministic and holds no state across
    // calls, so the same constraint set gives back the same trajectory. Not left empty either,
    // since the node publishes the cautious candidate every cycle
    result.cautious_trajectory = result.normal_trajectory;
    result.cautious_debug = result.normal_debug;
    cautious_previous_trajectory_ = normal_previous_trajectory_;
  }
  return result;
}

std::optional<Trajectory> FrenetSamplingBasedPlanner::plan_one_side(
  const PlannerContext & context, const ReferenceGrid & grid,
  const std::vector<Constraint> & constraints,
  const std::optional<Trajectory> & previous_trajectory, TrajectoryPlannerDebug & debug)
{
  // The phases below are timed one after another rather than through nested scopes, so that the
  // tree published on ~/debug/processing_time_detail_ms lists them as siblings under
  // plan_normal / plan_cautious
  auto phase =
    std::make_unique<autoware_utils_debug::ScopedTimeTrack>("simplify_boundaries", *time_keeper_);
  // The boundaries are thinned out before they are compiled: their vertices are walked by the
  // compiler and again by ConstraintTables. The gain is on the boundaries whose vertices sit
  // closer than the sampling interval of the compiler, which re-densifies to that interval anyway
  auto simplified_constraints = constraints;
  for (auto & constraint : simplified_constraints) {
    if (auto * boundary = std::get_if<Boundary>(&constraint.payload)) {
      auto & simplifier = constraint.hardness == Hardness::HARD ? *hard_boundary_simplifier_
                                                                : *soft_boundary_simplifier_;
      boundary->polyline = simplifier.simplify(boundary->polyline);
    }
  }

  phase.reset();
  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
    "compile_constraint_list", *time_keeper_);
  const auto compiled_constraints = compile_constraint_list(context, simplified_constraints);

  phase.reset();
  phase =
    std::make_unique<autoware_utils_debug::ScopedTimeTrack>("tabulate_constraints", *time_keeper_);
  const ConstraintTables tables(
    context, grid, compiled_constraints, params_.frenet_sampling_based_planner.path_resolution_m);

  phase.reset();
  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
    "make_lateral_bounds_markers", *time_keeper_);
  debug.markers["lateral_bounds"] =
    make_lateral_bounds_markers(context, grid, compiled_constraints);

  phase.reset();
  phase =
    std::make_unique<autoware_utils_debug::ScopedTimeTrack>("compute_initial_state", *time_keeper_);
  const auto initial_state = compute_initial_state(context, grid);

  phase.reset();
  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>("generate_paths", *time_keeper_);
  auto paths = generate_paths(context, grid, initial_state);
  // A path is followed to its end once here, shared by all the candidates built on it
  {
    const auto & p = params_.frenet_sampling_based_planner;
    const double wheel_base_m = context.vehicle_info.wheel_base_m;
    for (auto & path : paths) {
      for (std::size_t i = 0; i < path.s.size(); ++i) {
        const auto cell = tables.cell(path.s[i]);
        // A soft boundary, the own lane bound towards a parallel lane, costs the squared amount by
        // which it is exceeded, integrated along the path. Not over time within each candidate:
        // there a profile that stops short of where the path leaves the lane pays less than one
        // that drives on, and inside a corner that already overlaps the lane bound the cheapest
        // candidate stands still. Where the boundary does not reach, extreme_l is the infinity
        // the table is filled with and the violation comes out negative
        const auto box = footprint_sl_box(context.vehicle_info, path.s[i], path.l[i]);
        for (std::size_t b = 0; b < compiled_constraints.lateral_bounds.size(); ++b) {
          if (tables.lateral_is_hard[b]) {
            continue;
          }
          const double extreme_l = tables.lateral_extreme_l[b][cell];
          const bool forbids_left =
            compiled_constraints.lateral_bounds[b].forbidden_side == Side::LEFT;
          const double violation = forbids_left ? box.l_max - extreme_l : extreme_l - box.l_min;
          if (violation > 0.0) {
            path.soft_bound_cost +=
              p.weights.soft_bound * violation * violation * p.path_resolution_m;
          }
        }
        // Without this the choice within the time horizon heads for the outside of a corner that
        // is only cleared by cutting inside it, and stalls there. A penalty and not a rejection:
        // far from such a corner every path still runs into it
        if (p.weights.path_infeasible > 0.0 && path.feasible_to_end) {
          const Pose2d rear_axle{grid.position(path.s[i], path.l[i]), path.yaw[i]};
          path.feasible_to_end =
            std::abs(std::atan(path.kappa[i] * wheel_base_m)) <= tables.bounds.steer_angle &&
            !footprint_hits_boundary(
              tables.boundary_profiles[cell], context.vehicle_info, rear_axle, 0.0);
        }
      }
    }
  }

  phase.reset();
  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
    "generate_velocity_profiles", *time_keeper_);
  const auto profiles =
    generate_velocity_profiles(context, grid, tables, initial_state, compiled_constraints);

  phase.reset();
  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
    "project_previous_trajectory", *time_keeper_);
  const auto previous_lateral = previous_trajectory
                                  ? PreviousLateral{context.reference_path, *previous_trajectory}
                                  : PreviousLateral{};

  // Combining and evaluating are two passes and not one, so that the time keeper can tell the two
  // apart; they are independent per candidate, so the result is the same either way. Not tracked
  // per candidate: the grid holds about a thousand of them and each track is a node of the
  // published tree
  phase.reset();
  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>("combine", *time_keeper_);
  std::vector<Candidate> candidates;
  candidates.reserve(paths.size() * profiles.size());
  for (const auto & path : paths) {
    for (const auto & profile : profiles) {
      candidates.push_back(combine(path, profile));
    }
  }

  phase.reset();
  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>("evaluate", *time_keeper_);
  time_keeper_->comment(std::to_string(candidates.size()) + " candidates");
  for (auto & candidate : candidates) {
    evaluate(
      context, grid, compiled_constraints, tables, initial_state.l_goal, previous_lateral,
      candidate);
  }

  phase.reset();
  // Off by default: the grid holds about a thousand candidates of a hundred points each, which is
  // a few MB of markers per cycle
  if (params_.frenet_sampling_based_planner.debug.publish_candidate_markers) {
    phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
      "append_debug_markers", *time_keeper_);
    append_debug_markers(context, grid, candidates, debug.markers["candidates"]);
    phase.reset();
  }

  const Candidate * best = nullptr;
  for (const auto & candidate : candidates) {
    if (candidate.valid && (!best || candidate.cost < best->cost)) {
      best = &candidate;
    }
  }

  if (!best) {
    std::map<std::string, int> reasons;
    for (const auto & candidate : candidates) {
      const auto pos = candidate.tag.rfind('[');
      reasons[pos == std::string::npos ? "?" : candidate.tag.substr(pos)]++;
    }
    std::stringstream ss;
    for (const auto & [reason, count] : reasons) {
      ss << " " << reason << "x" << count;
    }
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("safety_planner"), steady_clock, 5000,
      "[frenet_sampling_based_planner] no valid candidate (%zu sampled:%s). Falling back to the "
      "stop trajectory.",
      candidates.size(), ss.str().c_str());
    // The stop trajectory holds the ego steer while braking at the hardest deceleration. Not the
    // quintic back to l0 of the candidates: from a heading well off the centerline it bends past
    // the steer limit within a few meters, and the fallback is not checked
    const auto hold = hold_steer_path(context, grid, initial_state);
    const auto stop = make_stop_profile(initial_state, tables.limits);
    return to_trajectory_msg(context, combine(hold, stop));
  }
  return to_trajectory_msg(context, *best);
}

FrenetSamplingBasedPlanner::InitialState FrenetSamplingBasedPlanner::compute_initial_state(
  const PlannerContext & context, const ReferenceGrid & grid) const
{
  const auto ego = compute_ego_frenet_state(context);
  const auto & path = context.reference_path;
  const double ego_yaw = autoware_utils_geometry::get_rpy(context.odometry.pose.pose).z;
  const double frenet_yaw = autoware_utils_math::normalize_radian(ego_yaw - grid.azimuth(ego.s));
  const double v = context.odometry.twist.twist.linear.x;

  InitialState state;
  state.s = ego.s;
  state.l = ego.l;
  // tan diverges as the heading approaches +-90 deg against the centerline, so the slope is cut
  // off at the equivalent of +-60 deg. The factor (1 - k_ref*l) inverts the heading expression of
  // sample_path; without it yaw[0] does not match the ego heading
  state.dl_ds = (1.0 - grid.curvature(ego.s) * ego.l) *
                std::tan(std::clamp(frenet_yaw, -M_PI / 3.0, M_PI / 3.0));
  // Pinning l''(0) to 0 would flatten the start of the lateral motion at every replan and barely
  // approach a lateral target 40 m ahead, the restart problem of a receding horizon. The initial
  // curvature comes from the measured steer angle instead. Exact rather than the small angle
  // k_ego - k_ref: with the ego at full steer the approximation puts kappa[0] over the steer limit
  // and every candidate, the stop fallback included, is rejected on its first point
  const double kappa_ego =
    std::tan(context.steering.steering_tire_angle) / context.vehicle_info.wheel_base_m;
  state.d2l_ds2 =
    offset_path_d2l(grid.curvature(ego.s), grid.dkappa(ego.s), ego.l, state.dl_ds, kappa_ego);
  // Floored as in compute_ego_frenet_state: 1 - k l vanishes at the center of curvature
  const double metric0 = std::max(1.0 - grid.curvature(ego.s) * ego.l, 0.2);
  state.v = std::max(0.0, v * std::cos(frenet_yaw)) / metric0;
  // Not taken from the measurement below the engage velocity: a negative one (rolling back on a
  // slope, the brake) starts every s(t) backwards, so all candidates fail the reverse check and
  // only the stop fallback is left, and the ego chatters between the two. While moving it is also
  // what the longitudinal controller feeds forward, so a slowdown would feed back into the command
  const double a = context.acceleration.accel.accel.linear.x;
  state.a = (state.v < params_.engage_velocity.velocity_hard_mps ? std::max(0.0, a) : a) / metric0;
  state.l_goal = lateral_offset_at(
    path, path.length(), Point2d{context.goal_pose.position.x, context.goal_pose.position.y});
  return state;
}

FrenetSamplingBasedPlanner::PathCandidate FrenetSamplingBasedPlanner::sample_path(
  const PlannerContext & context, const ReferenceGrid & grid, const InitialState & initial_state,
  const double length, const double l_target, const double return_length) const
{
  using autoware::frenet_planner::Polynomial;

  const auto & ref = context.reference_path;
  const double res = params_.frenet_sampling_based_planner.path_resolution_m;
  const double s0 = initial_state.s;
  const double s_max = ref.length();

  // l(s) joins the initial (l0, l'0, l''0) to the terminal (l_T, 0, 0) over the arc length L and
  // holds l_T beyond it. A terminal state past the end of the path is cut at the end, but not
  // below the shortest target length: a shift squeezed into the last meter before the goal is
  // sampled at only one or two points, which hides its curvature from the checks in evaluate()
  const auto & target_lengths = params_.frenet_sampling_based_planner.target_lengths_m;
  const double L = std::max(
    *std::min_element(target_lengths.begin(), target_lengths.end()), std::min(length, s_max - s0));
  // The way back to the centerline is also cut at the end of the path
  const bool returns = return_length > 0.0;
  const double L_back = std::max(res, std::min(return_length, s_max - s0 - L));
  // At the turning point of a way out and back, l'' takes that of a cosine bump of the same
  // widths. With l'' = 0 there the path follows the reference at an offset, which on the inside
  // of a curve is tighter than the reference itself; the bump is what eases the curvature
  const double d2l_turn = returns ? -l_target * M_PI * M_PI / (L * L + L_back * L_back) : 0.0;
  const Polynomial lat(
    initial_state.l, initial_state.dl_ds, initial_state.d2l_ds2, l_target, 0.0, d2l_turn, L);
  const Polynomial back(l_target, 0.0, d2l_turn, 0.0, 0.0, 0.0, L_back);

  PathCandidate path;
  for (double s = s0; s <= s_max + 1e-9; s += res) {
    const double u = s - s0;
    double l = returns ? 0.0 : l_target;
    double dl_ds = 0.0;
    double d2l_ds2 = 0.0;
    if (u <= L) {
      l = lat.position(u);
      dl_ds = lat.velocity(u);
      d2l_ds2 = lat.acceleration(u);
    } else if (returns && u <= L + L_back) {
      l = back.position(u - L);
      dl_ds = back.velocity(u - L);
      d2l_ds2 = back.acceleration(u - L);
    }
    const double s_ref = std::clamp(s, 0.0, s_max);
    const double k_ref = grid.curvature(s_ref);
    path.s.push_back(s);
    path.l.push_back(l);
    // The heading comes from the analytic Frenet expression psi = psi_ref + atan(l' /
    // (1 - k_ref*l)). Taking it from the chord between world positions would put the first heading
    // off the ego heading by k*res/2, which in closed loop drifts the ego heading a little every
    // cycle until it hits the steer rate limit after a few dozen of them
    path.yaw.push_back(
      autoware_utils_math::normalize_radian(
        grid.azimuth(s_ref) + std::atan2(dl_ds, 1.0 - k_ref * l)));
    path.metric.push_back(std::hypot(1.0 - k_ref * l, dl_ds));
    // Analytic, and per arc length of the path rather than of the reference: the two differ by the
    // metric, which reaches 2 for an ego 4 m outside a lane of R 4 m, and with the difference of
    // the headings over the reference arc length every candidate there failed the steer angle
    // check. Since l'(0) and l''(0) come from the ego, the first point matches the ego heading and
    // steer, so a candidate leaving in another direction needs no separate rejection here
    path.kappa.push_back(offset_path_curvature(k_ref, grid.dkappa(s_ref), l, dl_ds, d2l_ds2));
  }
  std::stringstream ss;
  ss << "L=" << L << " l=" << l_target;
  if (returns) {
    ss << " back=" << L_back;
  }
  path.tag = ss.str();
  return path;
}

FrenetSamplingBasedPlanner::PathCandidate FrenetSamplingBasedPlanner::hold_steer_path(
  const PlannerContext & context, const ReferenceGrid & grid,
  const InitialState & initial_state) const
{
  const double res = params_.frenet_sampling_based_planner.path_resolution_m;
  const double s_max = context.reference_path.length();
  const double kappa = offset_path_curvature(
    grid.curvature(initial_state.s), grid.dkappa(initial_state.s), initial_state.l,
    initial_state.dl_ds, initial_state.d2l_ds2);

  // l(s) of the circle of curvature kappa, integrated over the samples with l'' from the
  // curvature at each of them
  PathCandidate path;
  double l = initial_state.l;
  double dl_ds = initial_state.dl_ds;
  for (double s = initial_state.s; s <= s_max + 1e-9; s += res) {
    const double s_ref = std::clamp(s, 0.0, s_max);
    const double k_ref = grid.curvature(s_ref);
    const double d2l_ds2 = offset_path_d2l(k_ref, grid.dkappa(s_ref), l, dl_ds, kappa);
    path.s.push_back(s);
    path.l.push_back(l);
    path.yaw.push_back(
      autoware_utils_math::normalize_radian(
        grid.azimuth(s_ref) + std::atan2(dl_ds, 1.0 - k_ref * l)));
    path.metric.push_back(std::hypot(1.0 - k_ref * l, dl_ds));
    path.kappa.push_back(kappa);
    l += dl_ds * res + 0.5 * d2l_ds2 * res * res;
    dl_ds += d2l_ds2 * res;
  }
  path.tag = "hold_steer";
  return path;
}

std::vector<FrenetSamplingBasedPlanner::PathCandidate> FrenetSamplingBasedPlanner::generate_paths(
  const PlannerContext & context, const ReferenceGrid & grid,
  const InitialState & initial_state) const
{
  const auto & p = params_.frenet_sampling_based_planner;
  // The lateral position of the goal joins the terminal candidates, so that a goal off the grid
  // (on the shoulder, ...) can still be reached
  auto lateral_targets = p.target_lateral_positions_m;
  const bool on_grid = std::any_of(
    lateral_targets.begin(), lateral_targets.end(),
    [&](const double l) { return std::abs(l - initial_state.l_goal) < 0.05; });
  if (!on_grid) {
    lateral_targets.push_back(initial_state.l_goal);
  }
  std::vector<PathCandidate> paths;
  for (const double length : p.target_lengths_m) {
    for (const double l_target : lateral_targets) {
      paths.push_back(sample_path(context, grid, initial_state, length, l_target));
    }
  }
  // Apex-aligned: on a corner too tight for the footprint to follow the reference_path, the way
  // through cuts inside it and comes back, and it has to be timed on the corner. So the offset is
  // reached at the curvature peak, whatever the distance from the ego; sampled on the ego like the
  // others, such a path lines up with the corner only from a few positions
  if (!p.apex_lateral_offsets_m.empty()) {
    const double res = p.path_resolution_m;
    const double s_end = std::min(
      context.reference_path.length(),
      initial_state.s + *std::max_element(p.target_lengths_m.begin(), p.target_lengths_m.end()));
    for (double s = initial_state.s + res; s + res <= s_end; s += res) {
      const double k = grid.curvature(s);
      if (
        std::abs(k) < p.apex_min_curvature || std::abs(k) < std::abs(grid.curvature(s - res)) ||
        std::abs(k) < std::abs(grid.curvature(s + res))) {
        continue;
      }
      // The inside of the curve is towards its center: left (positive l) for a left turn
      const double inside = k > 0.0 ? 1.0 : -1.0;
      for (const double offset : p.apex_lateral_offsets_m) {
        for (const double return_length : p.apex_return_lengths_m) {
          paths.push_back(sample_path(
            context, grid, initial_state, s - initial_state.s, inside * offset, return_length));
        }
      }
    }
  }
  return paths;
}

std::vector<FrenetSamplingBasedPlanner::VelocityProfile>
FrenetSamplingBasedPlanner::generate_velocity_profiles(
  const PlannerContext & context, const ReferenceGrid & grid, const ConstraintTables & tables,
  const InitialState & initial_state, const CompiledConstraints & compiled_constraints) const
{
  using autoware::frenet_planner::Polynomial;

  const auto & p = params_.frenet_sampling_based_planner;
  const double dt = p.time_step_s;
  const double horizon = params_.trajectory_horizon_s;
  const double s_max = context.reference_path.length();
  double v_limit = tables.v_max[tables.cell(initial_state.s)];
  {
    // The terminal speeds are sampled against the speed at which the path curvature within the
    // horizon can still be taken under the lateral acceleration and steer rate limits. Sampling
    // ratios of the global limit (tens of km/h) instead leaves no candidate at an intermediate
    // speed in front of a curve, so only the stop profile survives and the ego crawls to a halt.
    // The limit at a point ahead is converted into the speed from which that point is reachable by
    // braking; taking the curvature ahead as the limit here would slow the whole horizon down
    // because of the curvature spike at the end of the path, where it joins the goal. The
    // deceleration only sets the sampling reference, so a fixed value is enough; whether a
    // candidate really works is decided in evaluate()
    constexpr double SAMPLING_DECEL_MPS2 = 1.0;
    const auto & bounds = tables.bounds;
    const double wheel_base_m = context.vehicle_info.wheel_base_m;
    const double res = p.path_resolution_m;
    const double s_end = std::min(s_max, initial_state.s + v_limit * horizon);
    double prev_steer = std::atan(grid.curvature(initial_state.s) * wheel_base_m);
    for (double s = initial_state.s + res; s <= s_end; s += res) {
      const double kappa = grid.curvature(s);
      const double steer = std::atan(kappa * wheel_base_m);
      // A regional velocity bound ahead (a speed limit zone) has to be reached by braking too,
      // otherwise every candidate above it is rejected inside the region and only the stop
      // profile survives
      double v_cap = tables.v_max[tables.cell(s)];
      if (std::abs(kappa) > 1e-6) {
        v_cap = std::min(v_cap, std::sqrt(bounds.lat_accel / std::abs(kappa)));
      }
      const double steer_grad = std::abs(steer - prev_steer) / res;  // [rad/m]
      if (steer_grad > 1e-6) {
        v_cap = std::min(v_cap, bounds.steer_rate / steer_grad);
      }
      prev_steer = steer;
      if (std::isfinite(v_cap)) {
        v_limit = std::min(
          v_limit, std::sqrt(v_cap * v_cap + 2.0 * SAMPLING_DECEL_MPS2 * (s - initial_state.s)));
      }
    }
  }

  std::vector<VelocityProfile> profiles;
  const auto sample = [&](const double duration, const double v_target, const double s_target) {
    const Polynomial lon(
      initial_state.s, initial_state.v, initial_state.a, s_target, v_target, 0.0, duration);
    VelocityProfile profile;
    for (double t = 0.0; t <= horizon + 1e-9; t += dt) {
      if (t <= duration) {
        profile.s.push_back(lon.position(t));
        profile.v.push_back(lon.velocity(t));
        profile.a.push_back(lon.acceleration(t));
      } else {
        // Hold the terminal state at a constant speed until the end of the horizon
        profile.s.push_back(s_target + v_target * (t - duration));
        profile.v.push_back(v_target);
        profile.a.push_back(0.0);
      }
    }
    std::stringstream ss;
    ss << "T=" << duration << " v=" << v_target;
    profile.tag = ss.str();
    profiles.push_back(std::move(profile));
  };

  for (const double duration : p.target_durations_s) {
    for (const double v_ratio : p.target_velocity_ratios) {
      const double v_target = v_ratio * v_limit;
      // The terminal s is the distance covered at the average speed, as in velocity keeping, and
      // never reaches beyond the goal at the end of the path
      const double s_target =
        std::min(s_max, initial_state.s + 0.5 * (initial_state.v + v_target) * duration);
      sample(duration, v_target, s_target);
    }
  }

  // Profiles that stop at the goal: T is derived from the remaining distance, as the time to
  // cover it at the average of the initial speed. With a fixed set of T the only candidates left a
  // few meters before the goal either cannot cover the distance within T, i.e. drive backwards, or
  // overshoot it, and all of them are rejected
  {
    // Stop at the goal, or at the nearest stop bar (Gate) ahead if that comes first. Without this
    // the only candidate that respects a stop bar is standstill (every profile that moves reaches
    // the bar within the horizon), so the ego would never approach it. The target is the
    // base_link position whose footprint front just touches the bar (violates_stop_bar)
    double s_stop_target = s_max;
    for (const auto & stop_bar : compiled_constraints.stop_bars) {
      if (stop_bar.time.t1 < 0.0 || stop_bar.time.t0 > horizon) {
        continue;
      }
      s_stop_target =
        std::min(s_stop_target, stop_bar.s_stop - context.vehicle_info.max_longitudinal_offset_m);
    }
    s_stop_target = std::max(s_stop_target, initial_state.s);
    const double remaining = s_stop_target - initial_state.s;
    const double duration = std::clamp(
      2.0 * remaining / std::max(initial_state.v, 0.1), p.target_durations_s.front(), horizon);
    sample(duration, 0.0, s_stop_target);
  }
  return profiles;
}

FrenetSamplingBasedPlanner::VelocityProfile FrenetSamplingBasedPlanner::make_stop_profile(
  const InitialState & initial_state, const KinematicLimits & limits) const
{
  const auto & p = params_.frenet_sampling_based_planner;
  const double dt = p.time_step_s;
  const double decel = std::abs(limits.a_hard_min);

  VelocityProfile profile;
  profile.tag = "stop";
  double s = initial_state.s;
  double v = initial_state.v;
  for (double t = 0.0; t <= params_.trajectory_horizon_s + 1e-9; t += dt) {
    profile.s.push_back(s);
    profile.v.push_back(v);
    profile.a.push_back(v > 0.0 ? -decel : 0.0);
    const double v_next = std::max(0.0, v - decel * dt);
    s += 0.5 * (v + v_next) * dt;
    v = v_next;
  }
  return profile;
}

FrenetSamplingBasedPlanner::Candidate FrenetSamplingBasedPlanner::combine(
  const PathCandidate & path, const VelocityProfile & profile) const
{
  const double res = params_.frenet_sampling_based_planner.path_resolution_m;
  const double s0 = path.s.front();

  Candidate candidate;
  candidate.path = &path;
  candidate.tag = path.tag + " " + profile.tag;
  candidate.s = profile.s;
  candidate.l.reserve(profile.s.size());
  candidate.kappa.reserve(profile.s.size());
  candidate.v.reserve(profile.s.size());
  candidate.a.reserve(profile.s.size());
  for (std::size_t k = 0; k < profile.s.size(); ++k) {
    const double s = profile.s[k];
    // The change of the metric along the path is left out of the acceleration
    const double metric = interpolate_uniform(path.metric, s0, res, s);
    candidate.l.push_back(interpolate_uniform(path.l, s0, res, s));
    candidate.kappa.push_back(interpolate_uniform(path.kappa, s0, res, s));
    candidate.v.push_back(profile.v[k] * metric);
    candidate.a.push_back(profile.a[k] * metric);
  }
  return candidate;
}

void FrenetSamplingBasedPlanner::evaluate(
  const PlannerContext & context, const ReferenceGrid & grid,
  const CompiledConstraints & compiled_constraints, const ConstraintTables & tables,
  const double l_goal, const PreviousLateral & previous_lateral, Candidate & candidate) const
{
  const auto & p = params_.frenet_sampling_based_planner;
  const double s_max = context.reference_path.length();
  const double blend_length =
    *std::max_element(p.target_lengths_m.begin(), p.target_lengths_m.end());
  const double wheel_base_m = context.vehicle_info.wheel_base_m;
  const auto & bounds = tables.bounds;
  const double dt = p.time_step_s;
  const auto reject = [&](const char * reason) {
    candidate.valid = false;
    candidate.tag += std::string(" [") + reason + "]";
  };

  double cost = 0.0;
  for (std::size_t k = 0; k < candidate.s.size(); ++k) {
    const double s = candidate.s[k];
    const double l = candidate.l[k];
    const double kappa = candidate.kappa[k];
    const double v = candidate.v[k];
    const double a = candidate.a[k];

    // A candidate that passes the goal at the end of the path, or drives backwards, is invalid
    if (s > s_max + 1e-3) {
      return reject("beyond_goal");
    }
    if (v < -1e-3) {
      return reject("reverse");
    }
    // --- vehicle kinematics (the ScalarBound constraints of VehicleKinematics) ---
    const auto cell = tables.cell(s);
    const double v_max = tables.v_max[cell];
    if (v > v_max + 1e-6) {
      return reject("velocity");
    }
    if (a < tables.limits.a_hard_min - 1e-6 || a > tables.limits.a_hard_max + 1e-6) {
      return reject("lon_accel");
    }
    const double steer = std::atan(kappa * wheel_base_m);
    if (std::abs(steer) > bounds.steer_angle) {
      return reject("steer_angle");
    }
    if (std::abs(v * v * kappa) > bounds.lat_accel) {
      return reject("lat_accel");
    }
    if (k + 1 < candidate.s.size()) {
      const double next_a = candidate.a[k + 1];
      if (std::abs((next_a - a) / dt) > bounds.lon_jerk) {
        return reject("lon_jerk");
      }
      const double next_steer = std::atan(candidate.kappa[k + 1] * wheel_base_m);
      if (std::abs((next_steer - steer) / dt) > bounds.steer_rate) {
        return reject("steer_rate");
      }
    }

    // --- geometric constraints, on the projected views ---
    const auto box = footprint_sl_box(context.vehicle_info, s, l);
    const double t0 = static_cast<double>(k) * dt;
    const double t1 = (k + 1 < candidate.s.size()) ? t0 + dt : t0;
    // Stretched by half the travel of a time step at both ends, so that the footprints of
    // consecutive points leave no gap between them
    const Pose2d rear_axle{
      grid.position(s, l),
      interpolate_uniform_angle(
        candidate.path->yaw, candidate.path->s.front(), p.path_resolution_m, s)};
    if (footprint_hits_boundary(
          tables.boundary_profiles[cell], context.vehicle_info, rear_axle,
          0.5 * std::abs(v) * dt)) {
      return reject("lateral_bound");
    }
    for (const auto & occupancy : compiled_constraints.occupancies) {
      if (violates_occupancy(occupancy, box, t0, t1)) {
        return reject("occupancy");
      }
    }
    for (const auto & stop_bar : compiled_constraints.stop_bars) {
      if (violates_stop_bar(stop_bar, box, t0, t1)) {
        return reject("stop_bar");
      }
    }

    // Soft cost, integrated over time. The lateral reference is the centerline, except that over
    // the last 2B to B before the goal (B being the longest lateral travel) it blends linearly into
    // the lateral position of the goal, and stays there for the remaining B. Starting to approach
    // it only B ahead would need an ever larger curvature as the distance shrinks, and the
    // candidates would be rejected by the steer rate and end up near the centerline
    const double l_ref =
      l_goal * std::clamp((2.0 * blend_length - (s_max - s)) / blend_length, 0.0, 1.0);
    // Continuity with the previous output. Without it the cheapest candidate of the grid is taken
    // anew every cycle, and since the lateral targets are spaced far closer than the differences
    // they make to the other terms, the winner flips between neighboring targets on the noise of
    // the ego state alone
    if (const auto l_previous = previous_lateral.at(s)) {
      const double dl = l - *l_previous;
      cost += p.weights.previous_lateral * dl * dl * dt;
    }
    const double dv = v_max - v;
    cost += p.weights.lateral * (l - l_ref) * (l - l_ref) * dt;
    cost += p.weights.velocity * dv * dv * dt;
    cost += p.weights.curvature * kappa * kappa * dt;
    if (k + 1 < candidate.s.size()) {
      const double lon_jerk = (candidate.a[k + 1] - a) / dt;
      cost += p.weights.lon_jerk * lon_jerk * lon_jerk * dt;
    }
  }
  cost += candidate.path->soft_bound_cost;
  if (!candidate.path->feasible_to_end) {
    cost += p.weights.path_infeasible;
  }
  candidate.cost = cost;
}

Trajectory FrenetSamplingBasedPlanner::to_trajectory_msg(
  const PlannerContext & context, const Candidate & candidate) const
{
  const auto & ref = context.reference_path;
  const double res = params_.frenet_sampling_based_planner.path_resolution_m;
  const double dt = params_.frenet_sampling_based_planner.time_step_s;
  const double s0 = candidate.path->s.front();
  const double s_max = ref.length();
  const double wheel_base_m = context.vehicle_info.wheel_base_m;

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = context.odometry.header.stamp;
  trajectory.points.reserve(candidate.s.size());
  // The spline is evaluated here rather than read off the ReferenceGrid: only the winner reaches
  // this point, so the cost is one candidate worth of it, and the chord of the grid cuts a few
  // millimeters off the inside of a tight curve
  for (std::size_t k = 0; k < candidate.s.size(); ++k) {
    const double s_ref = std::clamp(candidate.s[k], 0.0, s_max);
    const auto ref_position = ref.compute(s_ref).point.pose.position;
    const double ref_yaw = ref.azimuth(s_ref);
    const double l = candidate.l[k];
    const double kappa = candidate.kappa[k];
    TrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(k) * dt);
    point.pose.position.x = ref_position.x - std::sin(ref_yaw) * l;
    point.pose.position.y = ref_position.y + std::cos(ref_yaw) * l;
    // The road z, not the ego z: the longitudinal controller reads the slope it compensates from
    // the z of the trajectory, and a flat trajectory leaves an uphill start uncompensated
    point.pose.position.z = ref_position.z;
    point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
      interpolate_uniform_angle(candidate.path->yaw, s0, res, candidate.s[k]));
    point.longitudinal_velocity_mps = static_cast<float>(candidate.v[k]);
    point.acceleration_mps2 = static_cast<float>(candidate.a[k]);
    point.heading_rate_rps = static_cast<float>(candidate.v[k] * kappa);
    point.front_wheel_angle_rad = static_cast<float>(std::atan(kappa * wheel_base_m));
    trajectory.points.push_back(point);
  }
  return trajectory;
}

void FrenetSamplingBasedPlanner::append_debug_markers(
  const PlannerContext & context, const ReferenceGrid & grid,
  const std::vector<Candidate> & candidates, MarkerArray & debug_markers) const
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const auto stamp = context.odometry.header.stamp;

  auto valid_marker = create_default_marker(
    "map", stamp, "candidates_valid", 0, Marker::LINE_LIST, create_marker_scale(0.05, 0.0, 0.0),
    create_marker_color(0.0, 1.0, 0.0, 0.5));
  auto invalid_marker = create_default_marker(
    "map", stamp, "candidates_invalid", 0, Marker::LINE_LIST, create_marker_scale(0.03, 0.0, 0.0),
    create_marker_color(1.0, 0.0, 0.0, 0.2));

  // The world points of one candidate, reused by the next one
  std::vector<geometry_msgs::msg::Point> world;
  for (const auto & candidate : candidates) {
    world.clear();
    for (std::size_t k = 0; k < candidate.s.size(); ++k) {
      const auto position = grid.position(candidate.s[k], candidate.l[k]);
      geometry_msgs::msg::Point q;
      q.x = position.x();
      q.y = position.y();
      q.z = grid.z(candidate.s[k]);
      world.push_back(q);
    }
    auto & marker = candidate.valid ? valid_marker : invalid_marker;
    for (std::size_t k = 0; k + 1 < world.size(); ++k) {
      marker.points.push_back(world[k]);
      marker.points.push_back(world[k + 1]);
    }
  }
  if (!valid_marker.points.empty()) {
    debug_markers.markers.push_back(valid_marker);
  }
  if (!invalid_marker.points.empty()) {
    debug_markers.markers.push_back(invalid_marker);
  }
}

MarkerArray FrenetSamplingBasedPlanner::make_lateral_bounds_markers(
  const PlannerContext & context, const ReferenceGrid & grid,
  const CompiledConstraints & compiled_constraints) const
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  constexpr double INTERVAL_M = 2.0;
  const auto now = context.odometry.header.stamp;
  auto hard_marker = create_default_marker(
    "map", now, "lateral_bounds_hard", 0, Marker::LINE_LIST, create_marker_scale(0.05, 0.0, 0.0),
    create_marker_color(1.0, 0.2, 0.0, 0.8));
  auto soft_marker = create_default_marker(
    "map", now, "lateral_bounds_soft", 0, Marker::LINE_LIST, create_marker_scale(0.05, 0.0, 0.0),
    create_marker_color(1.0, 0.8, 0.0, 0.5));
  const auto & reference_path = context.reference_path;
  const double z = context.odometry.pose.pose.position.z;
  for (double s = 0.0; s <= reference_path.length(); s += INTERVAL_M) {
    // Only the nearest bound per side and hardness is drawn, as that is the one in effect; drawing
    // every bound would let a farther one cover the nearer one and look like it breaks through
    std::map<std::pair<bool, Side>, double> nearest;
    for (const auto & bound : compiled_constraints.lateral_bounds) {
      if (
        bound.polyline.size() < 2 || s < bound.polyline.front().s || s > bound.polyline.back().s) {
        continue;
      }
      const double l_bound = interpolate_boundary_l(bound.polyline, s);
      const bool hard =
        compiled_constraints.raw_constraints[bound.raw_index].hardness == Hardness::HARD;
      const auto [it, inserted] =
        nearest.emplace(std::make_pair(hard, bound.forbidden_side), l_bound);
      if (!inserted) {
        it->second = bound.forbidden_side == Side::LEFT ? std::min(it->second, l_bound)
                                                        : std::max(it->second, l_bound);
      }
    }
    for (const auto & [key, l_bound] : nearest) {
      auto & marker = key.first ? hard_marker : soft_marker;
      for (const double l : {0.0, l_bound}) {
        const auto position = grid.position(s, l);
        geometry_msgs::msg::Point q;
        q.x = position.x();
        q.y = position.y();
        q.z = z;
        marker.points.push_back(q);
      }
    }
  }
  MarkerArray marker_array;
  for (auto & marker : {hard_marker, soft_marker}) {
    if (!marker.points.empty()) {
      marker_array.markers.push_back(marker);
    }
  }
  return marker_array;
}

}  // namespace autoware::safety_planner::experimental

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experimental::FrenetSamplingBasedPlanner,
  autoware::safety_planner::TrajectoryPlannerInterface)
