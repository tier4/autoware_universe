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

#include "reference_path_smoother.hpp"

#include "frenet_utils.hpp"

#include <Eigen/Core>
#include <autoware/osqp_interface/osqp_interface.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

std::pair<double, double> lateral_room(
  const geometry_msgs::msg::Point & position, const lanelet::ConstLanelets & lanelets,
  const double vehicle_half_width_m)
{
  //! [m] clearance kept between the footprint and the lane bound
  constexpr double LANE_MARGIN_M = 0.3;

  const lanelet::BasicPoint2d p{position.x, position.y};
  const lanelet::ConstLanelet * nearest = nullptr;
  double nearest_dist = std::numeric_limits<double>::max();
  for (const auto & lanelet : lanelets) {
    const double dist = lanelet::geometry::distance2d(lanelet.polygon2d(), p);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest = &lanelet;
    }
  }
  if (!nearest || nearest_dist > 0.0) {
    return {0.0, 0.0};
  }
  const double left = lanelet::geometry::distance2d(nearest->leftBound2d(), p);
  const double right = lanelet::geometry::distance2d(nearest->rightBound2d(), p);
  return {
    std::max(0.0, right - vehicle_half_width_m - LANE_MARGIN_M),
    std::max(0.0, left - vehicle_half_width_m - LANE_MARGIN_M)};
}

std::optional<PathPointTrajectory> smooth_reference_path(
  const PathPointTrajectory & path, const lanelet::ConstLanelets & lanelets,
  const double vehicle_half_width_m, const double clearance_m)
{
  //! [m] resampling interval, 1 m as in the EB; an uneven spacing makes the QP unstable
  constexpr double RESAMPLE_INTERVAL_M = 1.0;
  //! weight on the deviation from the original position (lat_error_weight of the EB); against a
  //! smoothing term of 1 it acts as a regularizer
  constexpr double LAT_ERROR_WEIGHT = 0.001;
  //! number of trailing points held fixed, to keep the position and heading of the goal
  constexpr std::size_t NUM_FIXED_TAIL = 2;
  constexpr double OSQP_EPS_ABS = 1.0e-6;

  // Resample every 1 m, with the last sample pinned to the end of the path, i.e. the goal
  std::vector<double> ss;
  for (double s = 0.0; s < path.length() - 0.5 * RESAMPLE_INTERVAL_M; s += RESAMPLE_INTERVAL_M) {
    ss.push_back(s);
  }
  ss.push_back(path.length());
  const auto n = ss.size();
  if (n < 5) {
    return std::nullopt;  // too short for a second difference
  }
  auto points = path.restore();  // used as the carrier of the type and the lane_ids only
  points.clear();
  points.reserve(n);
  for (const double s : ss) {
    points.push_back(path.compute(s));
  }

  // The variables are the normal offsets lambda_i of the points; as in the EB nothing moves
  // longitudinally. With p = p_ref + N lambda the objective |D2 p|^2 + w|lambda|^2 becomes
  // 0.5 lambda' P lambda + q' lambda, P = 2(N'D2'D2 N + w I), q = 2 N'D2'D2 p_ref
  const int ni = static_cast<int>(n);
  Eigen::MatrixXd d2 = Eigen::MatrixXd::Zero(2 * (ni - 2), 2 * ni);
  for (int i = 0; i + 2 < ni; ++i) {
    for (const int dim : {0, 1}) {
      d2(i + dim * (ni - 2), i + dim * ni) = 1.0;
      d2(i + dim * (ni - 2), i + 1 + dim * ni) = -2.0;
      d2(i + dim * (ni - 2), i + 2 + dim * ni) = 1.0;
    }
  }
  Eigen::VectorXd p_ref(2 * ni);
  Eigen::MatrixXd n_matrix = Eigen::MatrixXd::Zero(2 * ni, ni);
  for (int i = 0; i < ni; ++i) {
    const auto & pos = points[static_cast<std::size_t>(i)].point.pose.position;
    const double yaw = path.azimuth(ss[static_cast<std::size_t>(i)]);
    p_ref(i) = pos.x;
    p_ref(ni + i) = pos.y;
    n_matrix(i, i) = -std::sin(yaw);
    n_matrix(ni + i, i) = std::cos(yaw);
  }
  const Eigen::MatrixXd dn = d2 * n_matrix;
  const Eigen::MatrixXd p_matrix =
    2.0 * (dn.transpose() * dn + LAT_ERROR_WEIGHT * Eigen::MatrixXd::Identity(ni, ni));
  const Eigen::VectorXd q_eigen = 2.0 * dn.transpose() * (d2 * p_ref);
  const std::vector<double> q_vector(q_eigen.data(), q_eigen.data() + ni);

  std::vector<double> lower(n);
  std::vector<double> upper(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (i + NUM_FIXED_TAIL >= n) {
      lower[i] = 0.0;
      upper[i] = 0.0;
      continue;
    }
    const auto [room_right, room_left] =
      lateral_room(points[i].point.pose.position, lanelets, vehicle_half_width_m);
    lower[i] = -std::min(clearance_m, room_right);
    upper[i] = std::min(clearance_m, room_left);
  }
  const Eigen::MatrixXd a_matrix = Eigen::MatrixXd::Identity(ni, ni);

  osqp_interface::OSQPInterface solver(OSQP_EPS_ABS, true);
  const auto result = solver.optimize(p_matrix, a_matrix, q_vector, lower, upper);
  if (
    result.solution_status != OSQP_SOLVED ||
    static_cast<int>(result.primal_solution.size()) != ni) {
    return std::nullopt;
  }
  for (const double value : result.primal_solution) {
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
  }

  for (int i = 0; i < ni; ++i) {
    auto & pos = points[static_cast<std::size_t>(i)].point.pose.position;
    pos.x += n_matrix(i, i) * result.primal_solution[static_cast<std::size_t>(i)];
    pos.y += n_matrix(ni + i, i) * result.primal_solution[static_cast<std::size_t>(i)];
  }
  // Recompute the headings from the neighboring points, so that the orientation returned by
  // compute() does not describe the positions from before the smoothing
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p0 = points[i == 0 ? i : i - 1].point.pose.position;
    const auto & p1 = points[i + 1 < n ? i + 1 : i].point.pose.position;
    points[i].point.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(p1.y - p0.y, p1.x - p0.x));
  }
  return experimental::trajectory::pretty_build(points);
}

std::optional<std::vector<double>> optimize_goal_shift(
  const std::vector<double> & nominal, const std::vector<double> & centerline_curvature,
  const double ds, const double max_curvature, const std::vector<double> & lower_bound,
  const std::vector<double> & upper_bound, const double deviation_weight, const bool hold_end,
  const std::vector<double> & previous, const double previous_weight,
  const std::optional<double> start_curvature, const double max_curvature_rate)
{
  //! The curvature of the offset path is nonlinear in l through 1 - k_ref l and l'; both are
  //! frozen at the profile of the previous pass and the QP re-run. k_ref * l reaches ~1 on the
  //! tight lanelets this is for, so linearizing those terms away is not enough
  constexpr int NUM_PASSES = 3;
  constexpr double CONVERGED_M = 1.0e-3;
  constexpr double OSQP_EPS_ABS = 1.0e-4;
  constexpr double OSQP_EPS_REL = 1.0e-3;

  const auto n = nominal.size();
  if (n < 4) {
    return std::nullopt;  // the end conditions leave nothing to optimize
  }
  const int ni = static_cast<int>(n);

  Eigen::MatrixXd d2 = Eigen::MatrixXd::Zero(ni - 2, ni);
  for (int i = 0; i + 2 < ni; ++i) {
    d2(i, i) = 1.0;
    d2(i, i + 1) = -2.0;
    d2(i, i + 2) = 1.0;
  }
  // deviation_weight also keeps P positive definite; the second difference alone is singular on
  // constants and ramps
  Eigen::MatrixXd p_matrix =
    2.0 * (d2.transpose() * d2 + deviation_weight * Eigen::MatrixXd::Identity(ni, ni));
  std::vector<double> q_vector(n, 0.0);
  for (std::size_t i = 0; i < previous.size(); ++i) {
    if (std::isfinite(previous[i])) {
      p_matrix(static_cast<int>(i), static_cast<int>(i)) += 2.0 * previous_weight;
      q_vector[i] = -2.0 * previous_weight * previous[i];
    }
  }

  // Rows: the value and the slope at each end, the offset of every point, the curvature of every
  // interior point, then the change of the curvature between consecutive interior points when it
  // is limited. The second difference at the ends is left free on purpose: pinning it at the start
  // of the ramp takes away the freedom the tight lanelets need
  constexpr int NUM_END_CONDITIONS = 4;
  const bool limit_rate = std::isfinite(max_curvature_rate);
  const int num_rows = NUM_END_CONDITIONS + ni + ni - 2 + (limit_rate ? ni - 3 : 0);
  const int curvature_row = NUM_END_CONDITIONS + ni;
  const int rate_row = curvature_row + ni - 2;
  Eigen::MatrixXd a_matrix = Eigen::MatrixXd::Zero(num_rows, ni);
  a_matrix(0, 0) = 1.0;
  a_matrix(1, 0) = -1.0;
  a_matrix(1, 1) = 1.0;
  a_matrix(2, ni - 1) = 1.0;
  a_matrix(3, ni - 2) = -1.0;
  a_matrix(3, ni - 1) = 1.0;
  a_matrix.block(NUM_END_CONDITIONS, 0, ni, ni) = Eigen::MatrixXd::Identity(ni, ni);
  std::vector<double> lower(static_cast<std::size_t>(num_rows));
  std::vector<double> upper(static_cast<std::size_t>(num_rows));
  const std::array<double, NUM_END_CONDITIONS> end_conditions{
    nominal.front(), nominal[1] - nominal.front(), nominal.back(), nominal.back() - nominal[n - 2]};
  for (std::size_t row = 0; row < NUM_END_CONDITIONS; ++row) {
    const bool free = !hold_end && row >= 2;
    lower[row] = free ? -std::numeric_limits<double>::infinity() : end_conditions[row];
    upper[row] = free ? std::numeric_limits<double>::infinity() : end_conditions[row];
  }
  for (std::size_t i = 0; i < n; ++i) {
    lower[NUM_END_CONDITIONS + i] = lower_bound[i];
    upper[NUM_END_CONDITIONS + i] = upper_bound[i];
  }

  osqp_interface::OSQPInterface solver(OSQP_EPS_ABS, true);
  // The default 1e-4 takes thousands of iterations when the curvature rate is tight (the steer
  // rate at speed), past the iteration limit; 1e-3 is well under a millimeter on the offsets
  solver.updateEpsRel(OSQP_EPS_REL);
  std::vector<double> profile = nominal;
  // The previous output is the linearization point where it is known: the nominal falls off the
  // ego to the centerline in one step, and the curvature rows linearized on that step are
  // infeasible with the ego meters off a tight centerline
  for (std::size_t i = 2; i < previous.size() && i + 2 < n; ++i) {
    if (std::isfinite(previous[i])) {
      profile[i] = previous[i];
    }
  }
  std::vector<double> dual;
  for (int pass = 0; pass < NUM_PASSES; ++pass) {
    // The exact curvature of the offset path, with a = 1 - k_ref l and l' from the previous pass:
    // (a^2 k_ref + a l'' + 2 k_ref l'^2) / (a^2 + l'^2)^(3/2). Only l'' is a variable, so the row
    // carries the a and the rest goes into the bounds. The start curvature is held at the second
    // sample, the first one with a second difference
    std::vector<double> coefficient(n, 0.0);  // curvature_i = coefficient_i * l''_i + offset_i
    std::vector<double> offset(n, 0.0);
    for (int i = 1; i + 1 < ni; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      const double k_ref = centerline_curvature[idx];
      const double a = 1.0 - k_ref * profile[idx];
      const double dl = (profile[idx + 1] - profile[idx - 1]) / (2.0 * ds);
      const double denominator = std::pow(a * a + dl * dl, 1.5);
      const double constant = a * a * k_ref + 2.0 * k_ref * dl * dl;
      coefficient[idx] = a / (denominator * ds * ds);
      offset[idx] = constant / denominator;
      const int row = curvature_row + i - 1;
      a_matrix(row, i - 1) = a / (ds * ds);
      a_matrix(row, i) = -2.0 * a / (ds * ds);
      a_matrix(row, i + 1) = a / (ds * ds);
      const double limit = i == 1 && start_curvature
                             ? std::clamp(*start_curvature, -max_curvature, max_curvature)
                             : max_curvature;
      lower[static_cast<std::size_t>(row)] =
        (i == 1 && start_curvature ? limit : -limit) * denominator - constant;
      upper[static_cast<std::size_t>(row)] = limit * denominator - constant;
    }
    for (int i = 1; limit_rate && i + 2 < ni; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      const int row = rate_row + i - 1;
      a_matrix(row, i - 1) = -coefficient[idx];
      a_matrix(row, i) = coefficient[idx + 1] + 2.0 * coefficient[idx];
      a_matrix(row, i + 1) = -2.0 * coefficient[idx + 1] - coefficient[idx];
      a_matrix(row, i + 2) = coefficient[idx + 1];
      const double constant = offset[idx + 1] - offset[idx];
      lower[static_cast<std::size_t>(row)] = -max_curvature_rate * ds - constant;
      upper[static_cast<std::size_t>(row)] = max_curvature_rate * ds - constant;
    }

    // Started from the previous pass, not cold: the passes differ by the linearization only, and
    // from zero each takes thousands of iterations at a corner beyond the steering at speed
    solver.initializeProblem(p_matrix, a_matrix, q_vector, lower, upper);
    if (pass > 0) {
      solver.setWarmStart(profile, dual);
    }
    const auto result = solver.optimize();
    if (
      result.solution_status != OSQP_SOLVED ||
      static_cast<int>(result.primal_solution.size()) != ni) {
      return std::nullopt;
    }
    double change = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      if (!std::isfinite(result.primal_solution[i])) {
        return std::nullopt;
      }
      change = std::max(change, std::abs(result.primal_solution[i] - profile[i]));
    }
    profile = result.primal_solution;
    dual = result.lagrange_multipliers;
    if (change < CONVERGED_M) {
      break;
    }
  }
  return profile;
}

std::vector<PathPointWithLaneId> shift_path_points(
  const PathPointTrajectory & path, const std::vector<double> & arc_lengths,
  const std::vector<double> & offsets)
{
  const auto original = path.restore();
  const auto bases = path.get_underlying_bases();
  std::vector<PathPointWithLaneId> points;
  for (std::size_t i = 0; i < original.size() && bases[i] < arc_lengths.front(); ++i) {
    points.push_back(original[i]);
  }
  // arc_lengths.front() itself is resampled rather than taken from the original points: it may
  // already carry a lateral offset
  const std::size_t first_shifted = points.size();
  for (std::size_t i = 0; i < arc_lengths.size(); ++i) {
    const auto shifted = to_world_pose(path, arc_lengths[i], offsets[i]);
    auto point = path.compute(arc_lengths[i]);
    point.point.pose.position.x = shifted.position.x();
    point.point.pose.position.y = shifted.position.y();
    points.push_back(point);
  }
  // The headings have to describe the shifted positions, not the centerline they came from
  for (std::size_t i = std::max<std::size_t>(first_shifted, 1); i < points.size(); ++i) {
    const auto & p0 = points[i - 1].point.pose.position;
    const auto & p1 = points[i + 1 < points.size() ? i + 1 : i].point.pose.position;
    points[i].point.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(p1.y - p0.y, p1.x - p0.x));
  }
  return points;
}

}  // namespace autoware::safety_planner
