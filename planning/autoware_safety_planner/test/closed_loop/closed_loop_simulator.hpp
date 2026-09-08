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

#ifndef CLOSED_LOOP__CLOSED_LOOP_SIMULATOR_HPP_
#define CLOSED_LOOP__CLOSED_LOOP_SIMULATOR_HPP_

// Lightweight closed-loop simulation without a ROS node.
// Builds SafetyPlannerInput the same way SafetyPlannerNode::on_timer does, advances the ego
// along the output trajectory by dt (perfect tracking; no controller or vehicle model), and
// repeats until the goal is reached / the ego stalls / the step limit is hit.
// Every output trajectory goes through a validity check.

#include "context.hpp"
#include "safety_planner.hpp"
#include "type_alias.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner::testing
{

struct ClosedLoopConfig
{
  double dt_s{0.1};  //!< one cycle (matches the node's planning_frequency_hz = 10 Hz)
  size_t max_steps{1200};
  double goal_distance_threshold_m{1.0};
  double goal_velocity_threshold_mps{0.1};
  //! Stalled if the goal distance does not shrink by stall_progress_m within stall_window_steps
  size_t stall_window_steps{100};
  double stall_progress_m{0.1};
};

//! Ego state at the start of a cycle and the trajectory planned in that cycle
struct StepRecord
{
  Odometry odometry;
  AccelWithCovarianceStamped acceleration;
  SteeringReport steering;
  Trajectory trajectory;  //!< normal_trajectory
  //! reference_path sampled at 1 m (x, y); kept as points so the plot does not need the
  //! trajectory class
  std::vector<std::pair<double, double>> reference_path_xy;
};

struct ClosedLoopResult
{
  bool goal_reached{false};
  std::string termination_reason;
  std::vector<StepRecord> steps;
  //! Ego state at termination (after the last advance). Its trajectory is empty
  StepRecord final_state;
  //! "step N: <reason>" entries; empty if every trajectory was valid
  std::vector<std::string> violations;
};

//! Validity check of one output trajectory. Returns the list of violations (empty if valid)
std::vector<std::string> validate_trajectory(
  const Trajectory & trajectory, const Pose & ego_pose, const VehicleInfo & vehicle_info);

//! Writes <path_prefix>_ego.csv (one row per step plus the final state as the last row) and
//! <path_prefix>_trajectories.csv (one row per trajectory point of every step) for offline analysis
void write_result_csv(const ClosedLoopResult & result, const std::string & path_prefix);

class ClosedLoopSimulator
{
public:
  //! predicted_objects are held fixed over the whole run (only the stamp follows the ego clock)
  ClosedLoopSimulator(
    const Params & params, const VehicleInfo & vehicle_info, const LaneletMapBin & map_bin,
    const LaneletRoute & route, const PredictedObjects & predicted_objects,
    const ClosedLoopConfig & config);

  ClosedLoopResult run();

  //! For visualization (the lanelet map is reachable through route_manager)
  const SafetyPlannerInput & input() const { return input_; }

private:
  //! Moves the ego to the point dt seconds ahead on the trajectory and updates
  //! odometry / acceleration / steering
  void advance_ego(const Trajectory & trajectory);
  bool update_route_manager();

  Params params_;
  ClosedLoopConfig config_;
  LaneletMapBin map_bin_;
  LaneletRoute route_;
  PredictedObjects predicted_objects_;
  SafetyPlanner planner_;
  SafetyPlannerInput input_;
  size_t step_{0};
};

}  // namespace autoware::safety_planner::testing

#endif  // CLOSED_LOOP__CLOSED_LOOP_SIMULATOR_HPP_
