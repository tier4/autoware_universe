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

#include "rough_planner.hpp"

#include "../../utils/sl_view_utils.hpp"

#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

// =============================================================================================
// constants
// =============================================================================================

// The grid, the weights and the output grid live in RoughPlannerParams (rough_planner.hpp); what
// stays here are the numerical tolerances
constexpr double CURVATURE_EPS = 1e-6;
constexpr double EPS = 1e-9;

// =============================================================================================
// internal types of the DP
// =============================================================================================

//! Geometry of a grid cell (s_i, l_j), independent of the layer and the speed
struct DpNodeGeometry
{
  Pose2d pose{};          //!< world coordinates; the yaw is the centerline tangent, the dl/ds
                          //!< correction being applied by the transitions
  double curvature{0.0};  //!< [1/m] centerline curvature at s_i, without the l correction
};

//! One node (t_k, s_i, l_j, v_m) of the DP grid. The speed is an axis of the grid, so the geometry
//! sits in DpNodeGeometry and a node carries only its validity and its search state.
struct DpNode
{
  // --- validity, written by the rejection stage of build_dp_grid ---
  //! false when the node is outside the reachable band in s or v, rejected by the geometry, or
  //! above the speed limit. Such a node is excluded from the search
  bool valid{true};

  // --- search state, written by search_dp_candidates ---
  double cost{INF};  //!< cost to come; INF means unreached
  int parent_s{-1};  //!< s index of the best parent, for the backtracking (-1 = no parent)
  int parent_l{-1};  //!< l index of the best parent
  int parent_v{-1};  //!< v index of the best parent
};

//! The four-dimensional (s, l, t, v) grid of the DP, rebuilt every cycle
struct DpGrid
{
  std::vector<double> s_values;  //!< [m] arc length from the ego position, ascending
  std::vector<double> l_values;  //!< [m] lateral offset, positive to the left, ascending
  std::vector<double> t_values;  //!< [s] time of a layer; layer 0 is the actual ego state at t = 0
  std::vector<double> v_values;  //!< [m/s] speed, from 0 upwards

  std::vector<DpNodeGeometry> geometries;  //!< size = s * l, one per (s_i, l_j)
  std::vector<DpNode> nodes;               //!< size = t * s * l * v, t-major

  //! Target speed per s, the reference of the cost, filled in by mark_invalid_nodes
  std::vector<double> v_target;

  //! Whether every l of s_i in layer k is blocked by the geometry or a stop line; size = t * s,
  //! t-major. It is all the wall of the braking condition looks at: being outside the reachable
  //! band or above the speed limit only means "not there yet", not that something is in the way
  std::vector<std::uint8_t> b_blocked;

  double s_ego{0.0};  //!< [m] arc length of the ego, the origin of the grid
  double l_ego{0.0};  //!< [m] lateral offset of the ego, kept exact rather than snapped

  std::size_t geometry_index(const int s_index, const int l_index) const
  {
    return static_cast<std::size_t>(s_index) * l_values.size() + l_index;
  }
  DpNodeGeometry & geometry(const int s_index, const int l_index)
  {
    return geometries[geometry_index(s_index, l_index)];
  }
  const DpNodeGeometry & geometry(const int s_index, const int l_index) const
  {
    return geometries[geometry_index(s_index, l_index)];
  }

  std::size_t node_index(
    const int t_index, const int s_index, const int l_index, const int v_index) const
  {
    return ((static_cast<std::size_t>(t_index) * s_values.size() + s_index) * l_values.size() +
            l_index) *
             v_values.size() +
           v_index;
  }
  DpNode & node(const int t_index, const int s_index, const int l_index, const int v_index)
  {
    return nodes[node_index(t_index, s_index, l_index, v_index)];
  }
  const DpNode & node(
    const int t_index, const int s_index, const int l_index, const int v_index) const
  {
    return nodes[node_index(t_index, s_index, l_index, v_index)];
  }
};

//! One point of the rough path from the backtracking, one per layer
struct DpPathPoint
{
  double t{0.0};  //!< [s]
  double s{0.0};  //!< [m]
  double l{0.0};  //!< [m]
  double v{0.0};  //!< [m/s] the speed reached through the best parent
};

//! A candidate path of the DP. cost is the total cost to come, which orders the candidates.
struct DpPath
{
  std::vector<DpPathPoint> points;
  double cost{0.0};
};

// =============================================================================================
// the stages of the DP
// =============================================================================================

//! [DP 1a] Spans the geometry of the grid: [0, params.dp.s_max_m] ahead of the ego along the
//! reference_path becomes the (s, l) grid and [0, params.dp.horizon_s] the layers, and every node
//! gets its world pose and the curvature of the centerline.
DpGrid build_grid_geometry(
  const RoughPlannerParams & params, const PlannerContext & context, const KinematicLimits & limits)
{
  DpGrid grid;
  const auto & path = context.reference_path;

  // Arc length and lateral offset of the ego; l_ego is kept exact rather than snapped
  const EgoFrenetState ego = compute_ego_frenet_state(context);
  grid.s_ego = ego.s;
  grid.l_ego = ego.l;

  // s axis: from the ego, in steps of s_step_m, to the end of the path or s_max_m
  const double s_end = std::min(path.length(), grid.s_ego + params.dp.s_max_m);
  const int num_s =
    std::max(static_cast<int>(std::floor((s_end - grid.s_ego) / params.dp.s_step_m)), 0) + 1;
  for (int i = 0; i < num_s; ++i) {
    grid.s_values.push_back(grid.s_ego + i * params.dp.s_step_m);
  }

  // l axis: [-l_range_m, +l_range_m] in steps of l_step_m, positive to the left
  const int num_l_half = static_cast<int>(std::round(params.dp.l_range_m / params.dp.l_step_m));
  for (int j = -num_l_half; j <= num_l_half; ++j) {
    grid.l_values.push_back(j * params.dp.l_step_m);
  }

  // t axis: [0, horizon_s] in steps of t_step_s; layer 0 is the actual ego state
  const int num_t = static_cast<int>(std::round(params.dp.horizon_s / params.dp.t_step_s)) + 1;
  for (int k = 0; k < num_t; ++k) {
    grid.t_values.push_back(k * params.dp.t_step_s);
  }

  // v axis: [0, v_hard] in steps of v_step_mps
  const int num_v = static_cast<int>(std::floor(limits.v_hard / params.dp.v_step_mps)) + 1;
  for (int m = 0; m < num_v; ++m) {
    grid.v_values.push_back(m * params.dp.v_step_mps);
  }

  // Geometry per (s, l), shared by every layer and speed
  grid.geometries.resize(grid.s_values.size() * grid.l_values.size());
  for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
    const double s = grid.s_values[i];
    const auto ref_position = path.compute(s).point.pose.position;
    const double ref_yaw = path.azimuth(s);
    const double ref_curvature = path.curvature(s);
    const double normal_x = -std::sin(ref_yaw);  // left normal, the positive direction of l
    const double normal_y = std::cos(ref_yaw);
    for (std::size_t j = 0; j < grid.l_values.size(); ++j) {
      const double l = grid.l_values[j];
      DpNodeGeometry & geometry = grid.geometry(i, j);
      geometry.pose.position =
        Point2d{ref_position.x + normal_x * l, ref_position.y + normal_y * l};
      geometry.pose.yaw = ref_yaw;  // the dl/ds correction is applied by the transitions
      geometry.curvature = ref_curvature;
    }
  }

  grid.nodes.resize(
    grid.t_values.size() * grid.s_values.size() * grid.l_values.size() * grid.v_values.size());
  return grid;
}

//! Lower end of the reachable band: the distance covered while braking at a_hard_min throughout
double min_reachable_distance(const double v0, const double t, const KinematicLimits & limits)
{
  const double decel = std::abs(limits.a_hard_min);
  const double t_stop = v0 / decel;
  if (t >= t_stop) {
    return v0 * v0 / (2.0 * decel);
  }
  return v0 * t - 0.5 * decel * t * t;
}

//! Upper end of the reachable band: the distance covered at a_hard_max, saturating at v_hard
double max_reachable_distance(const double v0, const double t, const KinematicLimits & limits)
{
  const double t_saturate = std::max((limits.v_hard - v0) / limits.a_hard_max, 0.0);
  if (t <= t_saturate) {
    return v0 * t + 0.5 * limits.a_hard_max * t * t;
  }
  const double distance_to_saturate =
    v0 * t_saturate + 0.5 * limits.a_hard_max * t_saturate * t_saturate;
  return distance_to_saturate + limits.v_hard * (t - t_saturate);
}

//! Per s, the speed limit v_upper = min(v_hard, v_legal, v_curve), which is hard, and the target
//! speed v_target = min(v_nom, v_upper), which the cost refers to. When the path is connected to
//! the goal, the envelope of stopping there at a_nom applies to both.
struct SpeedLimits
{
  std::vector<double> v_upper;
  std::vector<double> v_target;
};

SpeedLimits compute_speed_limits(
  const DpGrid & grid, const CompiledConstraints & compiled_constraints,
  const PlannerContext & context, const KinematicLimits & kinematic_limits)
{
  const bool stop_at_path_end = context.is_reference_path_connected_to_goal_pose();
  const double s_path_end = context.reference_path.length();

  SpeedLimits limits;
  limits.v_upper.reserve(grid.s_values.size());
  limits.v_target.reserve(grid.s_values.size());
  for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
    const double s = grid.s_values[i];
    const double curvature = std::abs(grid.geometry(i, 0).curvature);
    const double v_curve =
      std::sqrt(kinematic_limits.a_lat_nom / std::max(curvature, CURVATURE_EPS));

    double v_upper = std::min(kinematic_limits.v_hard, v_curve);
    double v_target = std::min(kinematic_limits.v_nom, v_curve);
    for (const auto & bound : compiled_constraints.scalar_bounds) {
      if (bound.quantity != BoundedQuantity::VELOCITY || s < bound.s0 || s > bound.s1) {
        continue;
      }
      // Every interval bound counts as a legal speed limit
      v_target = std::min(v_target, bound.max);
      v_upper = std::min(v_upper, bound.max);
    }
    if (stop_at_path_end) {
      // The envelope of decelerating comfortably to v = 0 at the goal
      const double v_stop =
        std::sqrt(2.0 * std::abs(kinematic_limits.a_nom_min) * std::max(s_path_end - s, 0.0));
      v_upper = std::min(v_upper, v_stop);
      v_target = std::min(v_target, v_stop);
    }
    limits.v_upper.push_back(v_upper);
    limits.v_target.push_back(std::min(v_target, v_upper));
  }
  return limits;
}

//! [DP 1b] Clears the valid flag of the nodes the constraints reject:
//! - the reachable band: everything outside the s(t_k) envelope between hardest braking and
//!   hardest acceleration
//! - static geometry: the lateral bounds do not depend on the layer, so they are evaluated once
//!   per (s, l)
//! - dynamic geometry: the occupancies, over the layer time plus and minus half a window
//! - stop lines: a node whose front passes a stop bar whose time window covers the layer
//! Passing here and failing on raw is allowed; the exact evaluation happens downstream.
DpGrid mark_invalid_nodes(
  const RoughPlannerParams & params, DpGrid grid, const CompiledConstraints & compiled_constraints,
  const PlannerContext & context, const KinematicLimits & limits)
{
  //! Half window around the layer time over which the dynamic constraints are evaluated, so that
  //! the gap between two layers is covered conservatively
  const double half_window = 0.5 * params.dp.t_step_s;
  const double v0 = std::max(context.odometry.twist.twist.linear.x, 0.0);
  const double band_tolerance = 0.5 * params.dp.s_step_m;  // slack for the grid rounding

  // The speed limit is a property of a node: v_m > v_upper(s_i) makes it invalid. v_target rides
  // on the grid into the search, as the reference of the cost
  const SpeedLimits speed_limits =
    compute_speed_limits(grid, compiled_constraints, context, limits);
  grid.v_target = speed_limits.v_target;

  // The boundaries depend on neither the layer nor the speed, so evaluate them once per (s, l)
  std::vector<std::uint8_t> static_valid(grid.s_values.size() * grid.l_values.size(), 1);
  for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
    for (std::size_t j = 0; j < grid.l_values.size(); ++j) {
      const SlBox box = footprint_sl_box(context.vehicle_info, grid.s_values[i], grid.l_values[j]);
      for (const auto & bound : compiled_constraints.lateral_bounds) {
        if (violates_lateral_bound(bound, box)) {
          static_valid[i * grid.l_values.size() + j] = 0;
          break;
        }
      }
    }
  }

  // Per layer: the reachable band in s and v, the occupancies and stop lines, the speed limit
  grid.b_blocked.assign(grid.t_values.size() * grid.s_values.size(), 0);
  for (std::size_t k = 0; k < grid.t_values.size(); ++k) {
    const double t = grid.t_values[k];
    const double s_band_min = grid.s_ego + min_reachable_distance(v0, t, limits) - band_tolerance;
    const double s_band_max = grid.s_ego + max_reachable_distance(v0, t, limits) + band_tolerance;
    // Reachable band in v: what the hardest acceleration and braking reach from v0
    const double v_band_min = v0 + limits.a_hard_min * t - 0.5 * params.dp.v_step_mps;
    const double v_band_max = v0 + limits.a_hard_max * t + 0.5 * params.dp.v_step_mps;
    for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
      const double s = grid.s_values[i];
      const bool in_band = s >= s_band_min && s <= s_band_max;
      bool all_l_b_blocked = true;
      for (std::size_t j = 0; j < grid.l_values.size(); ++j) {
        // The rejection is decided once at the (s, l, t) level and applied to every v. What the
        // geometry rejects is counted separately, as the material of the wall test
        bool b_valid = static_valid[i * grid.l_values.size() + j] != 0;
        if (b_valid) {
          const SlBox box = footprint_sl_box(context.vehicle_info, s, grid.l_values[j]);
          for (const auto & occupancy : compiled_constraints.occupancies) {
            if (violates_occupancy(
                  occupancy, compiled_constraints, box, t - half_window, t + half_window)) {
              b_valid = false;
              break;
            }
          }
          if (b_valid) {
            for (const auto & stop_bar : compiled_constraints.stop_bars) {
              if (violates_stop_bar(stop_bar, box, t - half_window, t + half_window)) {
                b_valid = false;
                break;
              }
            }
          }
        }
        all_l_b_blocked = all_l_b_blocked && !b_valid;
        const bool base_valid = in_band && b_valid;
        for (std::size_t m = 0; m < grid.v_values.size(); ++m) {
          const double v = grid.v_values[m];
          const bool v_valid =
            v <= speed_limits.v_upper[i] + EPS && v >= v_band_min - EPS && v <= v_band_max + EPS;
          grid.node(k, i, j, m).valid = base_valid && v_valid;
        }
      }
      grid.b_blocked[k * grid.s_values.size() + i] = all_l_b_blocked ? 1 : 0;
    }
  }
  return grid;
}

//! [DP 1] Builds the searchable grid: the geometry plus the validity
DpGrid build_dp_grid(
  const RoughPlannerParams & params, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints, const KinematicLimits & limits)
{
  return mark_invalid_nodes(
    params, build_grid_geometry(params, context, limits), compiled_constraints, context, limits);
}

//! Result of the search. An empty candidates means no path satisfies the geometry, and the caller
//! falls back to the stop plan; rejected carries the reasons, for debug.rejected.
struct DpSearchResult
{
  std::vector<DpPath> candidates;  //!< ascending in cost
  std::vector<std::string> rejected;
};

//! [DP 3] Sweeps the cost to come forward and backtracks from the terminal node to the candidates.
//! prev_decisions carries in the hysteresis barrier: a surcharge on a path that flips a side, and
//! the rejection of a hasty STOP -> GO or FOLLOW -> LEAD.
DpSearchResult search_dp_candidates(
  const RoughPlannerParams & params, DpGrid grid, const CompiledConstraints & compiled_constraints,
  const PlannerContext & context, const Decisions & prev_decisions,
  const KinematicLimits & kinematic_limits)
{
  // The grid is taken by value: the sweep writes the cost and the parents into its nodes and
  // throws it away afterwards
  // TODO(odashima): carry in the hysteresis barrier from prev_decisions once derive_decisions() is
  // implemented
  (void)prev_decisions;
  (void)compiled_constraints;

  DpSearchResult result;
  const std::size_t num_t = grid.t_values.size();
  const std::size_t num_s = grid.s_values.size();
  const std::size_t num_l = grid.l_values.size();
  const std::size_t num_v = grid.v_values.size();
  if (num_t < 2 || num_s == 0 || num_l == 0 || num_v == 0) {
    result.rejected.push_back("dp: degenerate grid");
    return result;
  }

  const double dt = params.dp.t_step_s;
  const double decel = std::abs(kinematic_limits.a_hard_min);
  const double v0 = std::clamp(context.odometry.twist.twist.linear.x, 0.0, kinematic_limits.v_hard);

  // Transition cost. v_target is the one mark_invalid_nodes put into the grid
  const auto edge_cost = [&](
                           const double ds, const double l_prev, const double l_next,
                           const double v_next, const double accel, const std::size_t i_next) {
    const double lateral_rate = (l_next - l_prev) / dt;
    const double v_error = v_next - grid.v_target[i_next];
    const double accel_over = std::max(std::abs(accel) - kinematic_limits.a_nom_max, 0.0);
    return -params.dp.weights.progress * ds + params.dp.weights.lateral * l_next * l_next * dt +
           params.dp.weights.lateral_rate * lateral_rate * lateral_rate * dt +
           params.dp.weights.velocity * v_error * v_error * dt +
           params.dp.weights.accel * accel * accel * dt +
           params.dp.weights.accel_nominal * accel_over * accel_over * dt;
  };

  //! A state the search expands from. s_index < 0 marks the actual ego state, off the grid
  struct SourceState
  {
    double s{0.0};
    double l{0.0};
    double v{0.0};
    double cost{0.0};
    int s_index{-1};
    int l_index{-1};
    int v_index{-1};
  };

  const auto to_s_index = [&](const double s_query) {
    return (s_query - grid.s_ego) / params.dp.s_step_m;
  };
  const auto to_l_index = [&](const double l_query) {
    return (l_query + params.dp.l_range_m) / params.dp.l_step_m;
  };
  const auto to_v_index = [&](const double v_query) { return v_query / params.dp.v_step_mps; };

  const auto relax = [&](
                       const std::size_t layer_next, const std::size_t i_next,
                       const std::size_t j_next, const std::size_t m_next, const double cost_next,
                       const SourceState & src) {
    DpNode & node = grid.node(layer_next, i_next, j_next, m_next);
    if (!node.valid || cost_next >= node.cost) {
      return;
    }
    node.cost = cost_next;
    node.parent_s = src.s_index;
    node.parent_l = src.l_index;
    node.parent_v = src.v_index;
  };

  // The end of the path is a wall only when it is connected to the goal, see distance_to_wall
  const double goal_wall_s =
    context.is_reference_path_connected_to_goal_pose() ? context.reference_path.length() : INF;

  // Forward sweep from layer k to k + 1
  for (std::size_t k = 0; k + 1 < num_t; ++k) {
    const std::size_t layer_next = k + 1;

    // Braking feasibility up to the wall, the conservative condition: an s of layer k + 1 where
    // the geometry or a stop line blocks every l is a wall, and only a transition slow enough to
    // stop within the remaining distance at the hardest braking is allowed.
    // A node outside the reachable band or above the speed limit is not a wall: the front of the
    // band only means "not there yet at that time", and taking it for a wall would raise a phantom
    // one right in front of layer 1 and cap the speed. Nor is the front of the grid, where the path
    // is merely cut off by s_max_m or the forward length and nothing beyond is known. Only a goal
    // the path is connected to becomes a wall
    std::vector<double> distance_to_wall(num_s, INF);
    {
      double wall_s = goal_wall_s;
      for (int i = static_cast<int>(num_s) - 1; i >= 0; --i) {
        if (grid.b_blocked[layer_next * num_s + i] != 0) {
          wall_s = grid.s_values[i];
        }
        distance_to_wall[i] = std::isinf(wall_s) ? INF : std::max(wall_s - grid.s_values[i], 0.0);
      }
    }

    const auto expand = [&](const SourceState & src) {
      const bool from_ego = src.s_index < 0;
      // The ego is not on the grid, so the first layer gets half a cell of slack on the lateral
      // condition
      const double snap_slack = from_ego ? 0.5 * params.dp.l_step_m : 0.0;

      // v' is enumerated along its axis, within the acceleration box, and ds = (v + v')/2 * dt
      // follows from assuming a constant acceleration. s' is snapped to the nearest cell and the
      // cost uses the real distance between the cells. v' = 0 is part of the enumeration, so a
      // stopping transition needs no case of its own
      const int m_lo = std::max(
        static_cast<int>(std::ceil(to_v_index(src.v + kinematic_limits.a_hard_min * dt) - EPS)), 0);
      const int m_hi = std::min(
        static_cast<int>(std::floor(to_v_index(src.v + kinematic_limits.a_hard_max * dt) + EPS)),
        static_cast<int>(num_v) - 1);
      for (int m_next = m_lo; m_next <= m_hi; ++m_next) {
        const double v_next = grid.v_values[m_next];
        const double ds_ideal = 0.5 * (src.v + v_next) * dt;
        const int i_next = static_cast<int>(std::round(to_s_index(src.s + ds_ideal)));
        if (i_next < 0 || i_next >= static_cast<int>(num_s)) {
          continue;
        }
        const double ds = grid.s_values[i_next] - src.s;
        if (ds < -EPS) {
          continue;
        }
        if (v_next * v_next > 2.0 * decel * distance_to_wall[i_next] + EPS) {
          continue;  // too fast to stop in front of the wall
        }
        const double accel = (v_next - src.v) / dt;

        // Lateral condition; nothing moves sideways while standing still (ds = 0)
        const double dl_max =
          (ds < EPS)
            ? 0.0
            : std::min(params.dp.lateral_slope_max * ds, params.dp.lateral_rate_max_mps * dt) +
                snap_slack;
        const int j_lo = std::max(static_cast<int>(std::ceil(to_l_index(src.l - dl_max) - EPS)), 0);
        const int j_hi = std::min(
          static_cast<int>(std::floor(to_l_index(src.l + dl_max) + EPS)),
          static_cast<int>(num_l) - 1);
        for (int j_next = j_lo; j_next <= j_hi; ++j_next) {
          const double l_next = grid.l_values[j_next];
          const double cost = src.cost + edge_cost(ds, src.l, l_next, v_next, accel, i_next);
          relax(layer_next, i_next, j_next, m_next, cost, src);
        }
      }
    };

    if (k == 0) {
      // Layer 0 is the single, unsnapped actual ego state
      SourceState ego;
      ego.s = grid.s_ego;
      ego.l = grid.l_ego;
      ego.v = v0;
      ego.cost = 0.0;
      expand(ego);
    } else {
      for (std::size_t i = 0; i < num_s; ++i) {
        for (std::size_t j = 0; j < num_l; ++j) {
          for (std::size_t m = 0; m < num_v; ++m) {
            const DpNode & node = grid.node(k, i, j, m);
            if (!node.valid || std::isinf(node.cost)) {
              continue;
            }
            SourceState src;
            src.s = grid.s_values[i];
            src.l = grid.l_values[j];
            src.v = grid.v_values[m];
            src.cost = node.cost;
            src.s_index = static_cast<int>(i);
            src.l_index = static_cast<int>(j);
            src.v_index = static_cast<int>(m);
            expand(src);
          }
        }
      }
    }
  }

  // The terminal node is the one of the last layer with the smallest cost to come. No terminal
  // term is needed, the reward for progress already being part of the edges
  const std::size_t last_layer = num_t - 1;
  int best_i = -1;
  int best_j = -1;
  int best_m = -1;
  double best_cost = INF;
  for (std::size_t i = 0; i < num_s; ++i) {
    for (std::size_t j = 0; j < num_l; ++j) {
      for (std::size_t m = 0; m < num_v; ++m) {
        const DpNode & node = grid.node(last_layer, i, j, m);
        if (node.valid && node.cost < best_cost) {
          best_cost = node.cost;
          best_i = static_cast<int>(i);
          best_j = static_cast<int>(j);
          best_m = static_cast<int>(m);
        }
      }
    }
  }
  if (best_i < 0) {
    result.rejected.push_back("dp: no feasible path to horizon");
    return result;
  }

  // Backtracking; only the best path for now
  DpPath path;
  path.cost = best_cost;
  std::vector<DpPathPoint> reversed_points;
  int i_trace = best_i;
  int j_trace = best_j;
  int m_trace = best_m;
  for (int k = static_cast<int>(last_layer); k >= 1; --k) {
    if (i_trace < 0 || j_trace < 0 || m_trace < 0) {
      result.rejected.push_back("dp: broken parent chain");
      return result;
    }
    const DpNode & node = grid.node(k, i_trace, j_trace, m_trace);
    reversed_points.push_back(
      {grid.t_values[k], grid.s_values[i_trace], grid.l_values[j_trace], grid.v_values[m_trace]});
    i_trace = node.parent_s;
    j_trace = node.parent_l;
    m_trace = node.parent_v;
  }
  path.points.push_back({0.0, grid.s_ego, grid.l_ego, v0});  // layer 0, the actual ego state
  path.points.insert(path.points.end(), reversed_points.rbegin(), reversed_points.rend());
  result.candidates.push_back(std::move(path));
  return result;
}

//! Cubic Hermite interpolation of l(s); a linear one would spike the curvature at every knot. The
//! knots are strictly increasing in s, a standstill collapsing into one, and the slopes are central
//! differences, 0 at the ends.
class LateralOffsetSpline
{
public:
  explicit LateralOffsetSpline(const std::vector<DpPathPoint> & dp_points)
  {
    for (const auto & point : dp_points) {
      if (knot_s_.empty() || point.s > knot_s_.back() + EPS) {
        knot_s_.push_back(point.s);
        knot_l_.push_back(point.l);
      }
    }
    knot_slope_.assign(knot_s_.size(), 0.0);  // the slope is 0 at both ends
    for (std::size_t i = 1; i + 1 < knot_s_.size(); ++i) {
      knot_slope_[i] = (knot_l_[i + 1] - knot_l_[i - 1]) / (knot_s_[i + 1] - knot_s_[i - 1]);
    }
  }

  double evaluate(const double s) const
  {
    if (knot_s_.empty()) {
      return 0.0;
    }
    if (s <= knot_s_.front()) {
      return knot_l_.front();
    }
    if (s >= knot_s_.back()) {
      return knot_l_.back();
    }
    std::size_t seg = 0;
    while (seg + 2 < knot_s_.size() && s > knot_s_[seg + 1]) {
      ++seg;
    }
    const double h = knot_s_[seg + 1] - knot_s_[seg];
    const double u = (s - knot_s_[seg]) / h;
    const double u2 = u * u;
    const double u3 = u2 * u;
    return (2.0 * u3 - 3.0 * u2 + 1.0) * knot_l_[seg] + (u3 - 2.0 * u2 + u) * h * knot_slope_[seg] +
           (-2.0 * u3 + 3.0 * u2) * knot_l_[seg + 1] + (u3 - u2) * h * knot_slope_[seg + 1];
  }

private:
  std::vector<double> knot_s_;
  std::vector<double> knot_l_;
  std::vector<double> knot_slope_;
};

//! Turns (s, l) into a RoughPlanPoint in world coordinates, with the centerline tangent as yaw
RoughPlanPoint to_rough_plan_point(
  const PathPointTrajectory & path, const double t, const double s, const double l, const double v,
  const double a)
{
  const auto ref_position = path.compute(s).point.pose.position;
  const double ref_yaw = path.azimuth(s);
  RoughPlanPoint point;
  point.t = t;
  point.pose.position =
    Point2d{ref_position.x - std::sin(ref_yaw) * l, ref_position.y + std::cos(ref_yaw) * l};
  point.pose.yaw = ref_yaw;
  point.kappa = path.curvature(s);
  point.v = v;
  point.a = a;
  return point;
}

//! [DP 4] Lifts the rough path, sampled at the DP layers, onto the stages of the optimizer
//! (params.num_points points spaced params.time_step_s apart).
//! TODO(odashima): the jerk limit of step 5.5 is missing
RoughPlan lift_to_stage_grid(
  const RoughPlannerParams & params, const DpPath & dp_path,
  const CompiledConstraints & compiled_constraints, const PlannerContext & context,
  const KinematicLimits & kinematic_limits)
{
  RoughPlan plan;
  plan.source = RoughPlanSource::SPATIOTEMPORAL_DP;
  if (dp_path.points.size() < 2) {
    return plan;
  }
  const auto & path = context.reference_path;
  const double s_path_end = path.length();

  // --- steps 1-2: expand v(t) and s(t) to the stage spacing, at a constant acceleration per
  //     interval ---
  std::vector<double> v_fine(params.num_points);
  std::vector<double> s_fine(params.num_points);
  for (int k = 0; k < params.num_points; ++k) {
    const double t = k * params.time_step_s;
    const std::size_t seg =
      std::min(static_cast<std::size_t>(t / params.dp.t_step_s), dp_path.points.size() - 2);
    const auto & p0 = dp_path.points[seg];
    const auto & p1 = dp_path.points[seg + 1];
    const double tau = t - p0.t;
    const double accel = (p1.v - p0.v) / params.dp.t_step_s;
    v_fine[k] = std::max(p0.v + accel * tau, 0.0);
    s_fine[k] = std::min(p0.s + p0.v * tau + 0.5 * accel * tau * tau, s_path_end);
  }

  // --- step 3: cap the speed, reapplying the curvature, the speed limits and the stopping
  //     envelope at the stage resolution, which catches the sharp curvature the 2 m grid of the DP
  //     cannot see ---
  const bool stop_at_path_end = context.is_reference_path_connected_to_goal_pose();
  const auto v_cap_at = [&](const double s) {
    const double curvature = std::abs(path.curvature(s));
    double cap = std::min(
      kinematic_limits.v_nom,
      std::sqrt(kinematic_limits.a_lat_nom / std::max(curvature, CURVATURE_EPS)));
    for (const auto & bound : compiled_constraints.scalar_bounds) {
      if (bound.quantity == BoundedQuantity::VELOCITY && s >= bound.s0 && s <= bound.s1) {
        cap = std::min(cap, bound.max);
      }
    }
    if (stop_at_path_end) {
      cap = std::min(
        cap, std::sqrt(2.0 * std::abs(kinematic_limits.a_nom_min) * std::max(s_path_end - s, 0.0)));
    }
    return cap;
  };
  for (int k = 0; k < params.num_points; ++k) {
    v_fine[k] = std::min(v_fine[k], v_cap_at(s_fine[k]));
  }

  // --- steps 4-5: the forward and backward pass at a_nom, starting from the measured ego
  //     speed ---
  v_fine[0] = std::clamp(context.odometry.twist.twist.linear.x, 0.0, v_fine[0]);
  for (int k = 0; k + 1 < params.num_points; ++k) {
    v_fine[k + 1] =
      std::min(v_fine[k + 1], v_fine[k] + kinematic_limits.a_nom_max * params.time_step_s);
  }
  for (int k = params.num_points - 2; k >= 0; --k) {
    v_fine[k] = std::min(
      v_fine[k], v_fine[k + 1] + std::abs(kinematic_limits.a_nom_min) * params.time_step_s);
  }

  // --- step 6: integrate s(t) again from the smoothed v, once ---
  s_fine[0] = dp_path.points.front().s;
  for (int k = 0; k + 1 < params.num_points; ++k) {
    s_fine[k + 1] =
      std::min(s_fine[k] + 0.5 * (v_fine[k] + v_fine[k + 1]) * params.time_step_s, s_path_end);
    if (s_fine[k + 1] >= s_path_end - EPS && stop_at_path_end) {
      v_fine[k + 1] = 0.0;
    }
  }

  // --- step 7: interpolate l(s) and fill in the curvature and the acceleration ---
  const LateralOffsetSpline lateral_spline(dp_path.points);
  plan.points.reserve(params.num_points);
  plan.s.reserve(params.num_points);
  for (int k = 0; k < params.num_points; ++k) {
    const double a =
      (k + 1 < params.num_points) ? (v_fine[k + 1] - v_fine[k]) / params.time_step_s : 0.0;
    plan.points.push_back(to_rough_plan_point(
      path, k * params.time_step_s, s_fine[k], lateral_spline.evaluate(s_fine[k]), v_fine[k], a));
    plan.s.push_back(s_fine[k]);
  }
  return plan;
}

// =============================================================================================
// the other candidates
// =============================================================================================

//! Outcome of trying to reuse the previous solution; a nullopt plan means it does not hold, and
//! rejected says why
struct PreviousSolutionResult
{
  std::optional<RoughPlan> plan;
  std::vector<std::string> rejected;
};

//! [candidate 1] Reuses the solution of the previous cycle, returning it through the same speed
//! smoothing as the DP path.
//! ([[maybe_unused]] only while this is disabled to exercise the DP; drop it when it comes back)
[[maybe_unused]] PreviousSolutionResult try_previous_solution(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const PreviousPlanningResult & prev_planning_result)
{
  // TODO(odashima): the six checks (quality, deviation of the initial state, geometry, stop
  // lines, speed limits, and an object new enough that no decision covers it)
  (void)context;
  (void)compiled_constraints;
  (void)prev_planning_result;
  return {};
}

//! [candidate 3] The stop plan, the last resort and always feasible: hold the current lateral
//! offset, run parallel to the reference_path and stop at a comfortable deceleration.
//! TODO(odashima): choose the deceleration (the smallest one that still stops in front of the
//! obstacle), connect it under the jerk limit, and decide the blocked flag
RoughPlan make_stop_plan(
  const RoughPlannerParams & params, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints, const KinematicLimits & kinematic_limits)
{
  (void)compiled_constraints;

  RoughPlan plan;
  plan.source = RoughPlanSource::STOP;
  const auto & path = context.reference_path;
  const double s_path_end = path.length();
  const EgoFrenetState ego = compute_ego_frenet_state(context);
  const double v0 = std::clamp(context.odometry.twist.twist.linear.x, 0.0, kinematic_limits.v_hard);

  plan.points.reserve(params.num_points);
  plan.s.reserve(params.num_points);
  double s = ego.s;
  for (int k = 0; k < params.num_points; ++k) {
    const double t = k * params.time_step_s;
    const double v = std::max(v0 + kinematic_limits.a_nom_min * t, 0.0);
    const double v_next = std::max(v0 + kinematic_limits.a_nom_min * (t + params.time_step_s), 0.0);
    const double a = (v > 0.0) ? kinematic_limits.a_nom_min : 0.0;
    plan.points.push_back(to_rough_plan_point(path, t, s, ego.l, v, a));
    plan.s.push_back(s);
    s = std::min(s + 0.5 * (v + v_next) * params.time_step_s, s_path_end);
  }
  return plan;
}

// =============================================================================================
// debug markers; nothing here changes the behavior
// =============================================================================================

//! The DP grid: the cells in world coordinates, with the layer as the height at 1 s = 1 m, drawn
//! as small spheres, green where valid and red where not
MarkerArray make_grid_markers(const DpGrid & grid, const double z_base)
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const auto stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  auto valid_marker = create_default_marker(
    "map", stamp, "grid_valid", 0, Marker::SPHERE_LIST, create_marker_scale(0.1, 0.1, 0.1),
    create_marker_color(0.0, 1.0, 0.0, 0.8));
  auto invalid_marker = create_default_marker(
    "map", stamp, "grid_invalid", 0, Marker::SPHERE_LIST, create_marker_scale(0.1, 0.1, 0.1),
    create_marker_color(1.0, 0.0, 0.0, 0.8));

  for (std::size_t k = 0; k < grid.t_values.size(); ++k) {
    for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
      for (std::size_t j = 0; j < grid.l_values.size(); ++j) {
        // The v axis is collapsed for the display: green as soon as one v is valid
        bool any_v_valid = false;
        for (std::size_t m = 0; m < grid.v_values.size(); ++m) {
          if (grid.node(k, i, j, m).valid) {
            any_v_valid = true;
            break;
          }
        }
        geometry_msgs::msg::Point point;
        point.x = grid.geometry(i, j).pose.position.x();
        point.y = grid.geometry(i, j).pose.position.y();
        point.z = z_base + grid.t_values[k];  // 1 s = 1 m
        (any_v_valid ? valid_marker : invalid_marker).points.push_back(point);
      }
    }
  }

  MarkerArray marker_array;
  if (!valid_marker.points.empty()) {
    marker_array.markers.push_back(valid_marker);
  }
  if (!invalid_marker.points.empty()) {
    marker_array.markers.push_back(invalid_marker);
  }
  return marker_array;
}

//! The candidates: one orange line strip each, at the same time scale as the grid, in the
//! namespace "candidate_<rank>"
MarkerArray make_candidate_markers(const std::vector<RoughPlan> & plans, const double z_base)
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  MarkerArray marker_array;
  const auto stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  for (std::size_t index = 0; index < plans.size(); ++index) {
    auto marker = create_default_marker(
      "map", stamp, "candidate_" + std::to_string(index), 0, Marker::LINE_STRIP,
      create_marker_scale(0.1, 0.0, 0.0), create_marker_color(1.0, 0.65, 0.0, 0.9));
    for (const auto & plan_point : plans[index].points) {
      geometry_msgs::msg::Point point;
      point.x = plan_point.pose.position.x();
      point.y = plan_point.pose.position.y();
      point.z = z_base + plan_point.t;  // 1 s = 1 m
      marker.points.push_back(point);
    }
    if (marker.points.size() >= 2) {
      marker_array.markers.push_back(marker);
    }
  }
  return marker_array;
}

//! Concatenates two MarkerArrays
MarkerArray merge_marker_arrays(MarkerArray first, const MarkerArray & second)
{
  first.markers.insert(first.markers.end(), second.markers.begin(), second.markers.end());
  return first;
}

// =============================================================================================
// assembling the result
// =============================================================================================

//! Completes a plan by deriving its decisions from the geometry of the trajectory
RoughPlan finalize_plan(
  RoughPlan plan, const CompiledConstraints & compiled_constraints,
  const Decisions & prev_decisions)
{
  plan.decisions = derive_decisions(plan, compiled_constraints, prev_decisions);
  return plan;
}

//! Tries the previous solution, then the space-time DP, then the stop plan, in that order, and
//! returns the candidates of the first stage that holds. The stop plan is always feasible, so
//! there is always at least one.
RoughPlanResult make_plan_candidates(
  const RoughPlannerParams & params, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints,
  const PreviousPlanningResult & prev_planning_result)
{
  RoughPlanResult result;
  const Decisions prev_decisions =
    prev_planning_result.plan ? prev_planning_result.plan->decisions : Decisions{};

  // [1] Reuse of the previous solution, the first layer of the hysteresis: as long as it holds,
  // the decisions do not change. Disabled for now, to exercise the DP
  // const auto previous = try_previous_solution(context, compiled_constraints,
  //                                             prev_planning_result);
  // result.debug.rejected = previous.rejected;
  // if (previous.plan) {
  //   result.plans.push_back(finalize_plan(*previous.plan, compiled_constraints, prev_decisions));
  //   return result;
  // }

  // Collect the kinematic limits from the IR
  const KinematicLimits kinematic_limits = collect_kinematic_limits(compiled_constraints);

  // [2] The space-time DP: build the grid, search it, lift the result onto the stages
  DpGrid grid = build_dp_grid(params, context, compiled_constraints, kinematic_limits);
  result.debug.debug_markers = make_grid_markers(grid, context.odometry.pose.pose.position.z);
  const DpSearchResult dp_result = search_dp_candidates(
    params, std::move(grid), compiled_constraints, context, prev_decisions, kinematic_limits);
  result.debug.rejected.insert(
    result.debug.rejected.end(), dp_result.rejected.begin(), dp_result.rejected.end());
  for (const DpPath & candidate : dp_result.candidates) {
    result.plans.push_back(finalize_plan(
      lift_to_stage_grid(params, candidate, compiled_constraints, context, kinematic_limits),
      compiled_constraints, prev_decisions));
  }
  if (!result.plans.empty()) {
    return result;
  }

  // [3] The stop plan, always feasible
  result.plans.push_back(finalize_plan(
    make_stop_plan(params, context, compiled_constraints, kinematic_limits), compiled_constraints,
    prev_decisions));
  return result;
}

}  // namespace

RoughPlanner::RoughPlanner(const RoughPlannerParams & params) : params_(params)
{
}

RoughPlanResult RoughPlanner::plan_rough_trajectories(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const PreviousPlanningResult & prev_planning_result) const
{
  const auto start_time = std::chrono::steady_clock::now();

  RoughPlanResult result =
    make_plan_candidates(params_, context, compiled_constraints, prev_planning_result);
  result.debug.debug_markers = merge_marker_arrays(
    std::move(result.debug.debug_markers),
    make_candidate_markers(result.plans, context.odometry.pose.pose.position.z));

  result.debug.elapsed_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
      .count();
  return result;
}

Decisions derive_decisions(
  const RoughPlan & plan, const CompiledConstraints & compiled_constraints,
  const Decisions & prev_decisions)
{
  // TODO(odashima): derive the decisions from the geometry of the trajectory
  // - side: for a static object whose s range overlaps the trajectory, which side the closest
  //   point passes on
  // - lead_lag: whether the ego clears the conflicting s range first or last
  // - stop_go: whether s_stop is passed within the time window
  (void)plan;
  (void)compiled_constraints;
  (void)prev_decisions;
  return {};
}

}  // namespace autoware::safety_planner
