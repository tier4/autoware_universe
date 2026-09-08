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
  //! [m] 再サンプル間隔。EB と同じ 1 m (間隔が不揃いだと最適化が不安定になる)
  constexpr double RESAMPLE_INTERVAL_M = 1.0;
  //! 元位置からのずれの重み (EB の lat_error_weight)。平滑化項 1 に対して正則化程度
  constexpr double LAT_ERROR_WEIGHT = 0.001;
  //! 終端 goal の位置と向きを保つために固定する点数
  constexpr std::size_t NUM_FIXED_TAIL = 2;
  //! [m] レーン境界に対して footprint の外側に残す余裕
  constexpr double LANE_MARGIN_M = 0.3;
  constexpr double OSQP_EPS_ABS = 1.0e-6;

  // 1 m で再サンプル (終端は経路終端 = goal に合わせる)
  std::vector<double> ss;
  for (double s = 0.0; s < path.length() - 0.5 * RESAMPLE_INTERVAL_M; s += RESAMPLE_INTERVAL_M) {
    ss.push_back(s);
  }
  ss.push_back(path.length());
  const auto n = ss.size();
  if (n < 5) {
    return std::nullopt;  // 2 階差分が組めない
  }
  auto points = path.restore();  // 型と lane_ids の器として使い、中身は再サンプルで置き換える
  points.clear();
  points.reserve(n);
  for (const double s : ss) {
    points.push_back(path.compute(s));
  }

  // 変数は各点の法線方向オフセット λ_i (EB と同じく縦方向には動かさない)。p = p_ref + N λ、
  // 目的 ‖D₂ p‖² + w‖λ‖² を λ で書くと 0.5 λᵀPλ + qᵀλ、P = 2(NᵀD₂ᵀD₂N + wI)、q = 2 NᵀD₂ᵀD₂ p_ref
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

  // 各点の可動範囲: 最寄り lanelet の左右 bound までの距離から footprint とマージンを引いた分
  // (goal 接続部のように lanelet の外に居る点は動かさない)
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
  // 向きは隣接点の差分から取り直す (Trajectory の azimuth() は x,y の微分なので必須では無いが、
  // compute() の orientation が古い位置のものにならないようにする)
  for (std::size_t i = 0; i < n; ++i) {
    const auto & p0 = points[i == 0 ? i : i - 1].point.pose.position;
    const auto & p1 = points[i + 1 < n ? i + 1 : i].point.pose.position;
    points[i].point.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(p1.y - p0.y, p1.x - p0.x));
  }
  return experimental::trajectory::pretty_build(points);
}

}  // namespace autoware::safety_planner
