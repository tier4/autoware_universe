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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__TRAJECTORY_OPTIMIZER_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__TRAJECTORY_OPTIMIZER_HPP_

#include "autoware/trajectory_processor/time_sequence_raw/acados_solver_wrapper.hpp"
#include "autoware/trajectory_processor/time_sequence_raw/optimizer_params.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::trajectory_processor::time_sequence_raw
{
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

struct OptimizationResult
{
  Trajectory trajectory;
  bool optimized{false};
  int solver_status{0};
  double solve_time_ms{0.0};
};

/// Tracks a pose-only time-indexed trajectory with a kinematic bicycle OCP.
/// Initial state is ego odometry + measured steering; incoming trajectory speed is ignored.
class TrajectoryOptimizer
{
public:
  TrajectoryOptimizer(
    const TrajectoryOptimizationParams & params,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, size_t batch_size);

  OptimizationResult optimize(
    const Trajectory & raw_trajectory, const Odometry & ego_odometry,
    const std::optional<double> & current_steering_angle_rad, size_t batch_index);

private:
  TrajectoryOptimizationParams params_;
  double wheelbase_m_;
  double max_steering_angle_rad_;
  std::unique_ptr<AcadosSolverWrapper> solver_;

  struct PreviousSolution
  {
    SolverSolution solution;
    rclcpp::Time stamp;
  };
  std::vector<std::optional<PreviousSolution>> previous_solutions_;
};

}  // namespace autoware::trajectory_processor::time_sequence_raw

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__TRAJECTORY_OPTIMIZER_HPP_
