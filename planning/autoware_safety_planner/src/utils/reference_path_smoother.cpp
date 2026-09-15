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

#include <Eigen/Core>
#include <autoware/osqp_interface/osqp_interface.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

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
  //! [m] clearance kept between the footprint and the lane bound
  constexpr double LANE_MARGIN_M = 0.3;
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

  // How far a point may travel: the distance to the bounds of the nearest lanelet, less the
  // footprint and the margin. A point outside every lanelet, as around the goal, is held fixed
  const auto lateral_room = [&](const geometry_msgs::msg::Point & position) {
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
      return std::pair<double, double>{0.0, 0.0};
    }
    const double left = lanelet::geometry::distance2d(nearest->leftBound2d(), p);
    const double right = lanelet::geometry::distance2d(nearest->rightBound2d(), p);
    return std::pair<double, double>{
      std::max(0.0, right - vehicle_half_width_m - LANE_MARGIN_M),
      std::max(0.0, left - vehicle_half_width_m - LANE_MARGIN_M)};
  };

  std::vector<double> lower(n);
  std::vector<double> upper(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (i + NUM_FIXED_TAIL >= n) {
      lower[i] = 0.0;
      upper[i] = 0.0;
      continue;
    }
    const auto [room_right, room_left] = lateral_room(points[i].point.pose.position);
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

}  // namespace autoware::safety_planner
