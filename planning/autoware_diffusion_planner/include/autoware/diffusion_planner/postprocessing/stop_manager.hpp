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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__STOP_MANAGER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__STOP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::diffusion_planner::postprocess
{

/**
 * @brief Decides when a predicted "stop then depart" velocity plan should be trusted.
 *
 * The ML planner can keep predicting a trajectory that stops and then departs after a fixed
 * duration (e.g. "stop for 1 second, then go") for longer than that duration actually elapses in
 * reality, because the re-predicted plan looks similar frame after frame while the ego remains
 * stopped. Naively forcing a hard stop once any near-zero velocity sample is seen in the plan
 * (see `apply_stopping_threshold`'s `enable_force_stop` latch) would otherwise suppress the
 * eventual, legitimate departure forever.
 *
 * This manager tracks, while the ego is actually stopped, the longest departure time the plan has
 * ever predicted since the stop began. Once the ego has actually been stopped longer than that
 * maximum, the predicted departure is considered overdue/trustworthy and force-stop latching is
 * released so the plan's predicted motion is allowed through.
 */
class StopManager
{
public:
  StopManager() = default;

  /**
   * @brief Updates the stop-tracking state for the current frame and decides whether force-stop
   * latching should be enabled.
   *
   * The ego's "current" velocity is taken to be the first element of the *previous* call's
   * `planned_velocity_profile` (i.e. what the model predicted for this instant one inference
   * cycle ago), rather than the real odometry velocity. This keeps the stop/departure judgement
   * self-consistent with the model's own velocity scale/bias instead of mixing it with a
   * differently-noised external signal.
   *
   * @param stamp Current frame timestamp.
   * @param planned_velocity_profile The predicted velocity profile for the current frame,
   * starting at the current time step (e.g. the batch-0 `ego_velocity_future` slice).
   * @param stopping_threshold Velocity threshold below which the ego/plan is considered
   * stopped [m/s].
   * @param dt Time step between consecutive elements of `planned_velocity_profile` [s].
   * @return true if force-stop latching should be enabled for this frame; false if the ego is
   * currently moving, or if the actual stopped duration has exceeded the longest departure time
   * the plan has predicted since the stop began.
   */
  bool update(
    const rclcpp::Time & stamp, const std::vector<float> & planned_velocity_profile,
    double stopping_threshold, double dt);

  /**
   * @brief Clears the tracked stop state (as if the ego had just started moving).
   */
  void reset();

private:
  bool is_tracking_{false};
  rclcpp::Time stop_start_time_;
  double max_departure_time_s_{0.0};
  float previous_plan_first_velocity_mps_{0.0f};
};

}  // namespace autoware::diffusion_planner::postprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__STOP_MANAGER_HPP_
