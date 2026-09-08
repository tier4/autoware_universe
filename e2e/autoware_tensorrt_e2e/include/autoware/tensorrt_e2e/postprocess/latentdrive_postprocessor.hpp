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

#ifndef AUTOWARE__TENSORRT_E2E__POSTPROCESS__LATENTDRIVE_POSTPROCESSOR_HPP_
#define AUTOWARE__TENSORRT_E2E__POSTPROCESS__LATENTDRIVE_POSTPROCESSOR_HPP_

#include "autoware/tensorrt_e2e/postprocess/trajectory_postprocessor.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>

#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace latentdrive
{

/// One waypoint in the ego frame: x forward [m], y left [m], yaw [rad], 0 = straight ahead.
struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A plan in the ego frame at inference time; waypoint i is at (i + 1) * time_step ahead.
using Plan = std::vector<Waypoint>;

struct SmoothingParams
{
  bool enable{false};
  //! Weight of the fresh plan in the blend; 1.0 passes it through untouched.
  double alpha{0.35};
  //! A plan whose end moves further than this from the carried-forward one is a new decision,
  //! not jitter, and replaces the filter state [m].
  double reset_jump_m{8.0};
  //! A gap between ticks longer than this (or a step back in time) drops the state [s].
  double max_gap_seconds{1.0};
};

/// Wrap an angle into (-pi, pi].
double wrap_pi(double rad);

/**
 * @brief Sample a plan in waypoint-index space: u = -1 is the ego origin the plan starts
 * from, u = i is waypoint i, fractional u interpolates, and beyond the last waypoint the final
 * segment is extended.
 */
Waypoint sample_at(const Plan & plan, double u);

/// Express `point` in the frame of `origin` (both given in the same frame).
Waypoint to_local(const Waypoint & point, const Waypoint & origin);

/**
 * @brief Re-anchor the previous plan on the current ego pose and the current time grid.
 *
 * `previous` is expressed in the previous ego frame; `ego_now` is the current ego pose in that
 * same frame (measured, from odometry, so the result does not assume the vehicle followed the
 * plan); `elapsed_s` is the time between the two plans. Waypoint i of the result is where the
 * previous plan said the vehicle would be at `(i + 1) * time_step` after now, in the current
 * ego frame.
 */
Plan carry_forward(
  const Plan & previous, const Waypoint & ego_now, double elapsed_s, double time_step);

/**
 * @class PlanSmoother
 * @brief Temporal filter over consecutive plans, ported from LatentDrive-TRT's display
 * smoother and re-anchored on measured ego motion.
 *
 * The engine is deterministic per frame, but consecutive frames disagree on how far the plan
 * reaches (about a metre at the 4 s end), which reads as a tail pulsing in and out and, fed
 * to a controller, as a jittery reference. Each update carries the previous output forward
 * by the ego's actual motion and blends the fresh plan into it on the shared time axis.
 */
class PlanSmoother
{
public:
  explicit PlanSmoother(const SmoothingParams & params) : params_(params) {}

  /**
   * @brief Blend `plan` into the filter state and return the result.
   * @param ego_now Current ego pose in the previous plan's ego frame (ignored without state).
   * @param elapsed_s Time since the previous update; the state is dropped when it is not in
   *        (0, max_gap_seconds].
   */
  Plan update(const Plan & plan, const Waypoint & ego_now, double elapsed_s, double time_step);

  void reset() { previous_.clear(); }
  bool has_state() const { return !previous_.empty(); }

private:
  SmoothingParams params_;
  Plan previous_;
};

}  // namespace latentdrive

/**
 * @class LatentDrivePostprocessor
 * @brief The common postprocessor with an optional temporal smoothing of the LatentDrive plan.
 *
 * When `latentdrive.smoothing.enable` is set, the `(x, y, yaw)` plan is filtered in the ego
 * frame before the base class turns it into messages, and the unfiltered plan is published as
 * an extra candidate trajectory whose generator name ends in `_raw`, so evaluations can read
 * either. On by default in the deployment configuration: the filter trades about
 * `(1 - alpha) / alpha` ticks of lag for a steady reference, and open-loop accuracy figures
 * belong to the raw output.
 *
 * The plan that goes into the trajectory is also published as a `nav_msgs/Path` in `base_link`
 * on `~/debug/latentdrive/plan`, the way the LatentDrive-TRT replay node published it. RViz
 * draws a base_link path attached to the ego model, whereas the map-frame trajectory is drawn
 * where the ego was at the tick and the model moves on at 50 Hz between 10 Hz ticks, which
 * reads as the start point jittering back and forth.
 */
class LatentDrivePostprocessor : public TrajectoryPostprocessor
{
public:
  LatentDrivePostprocessor(rclcpp::Node & node, const PostprocessParams & params);

  void validate_output_specs(const std::vector<TensorSpec> & output_specs) override;

  Output process(
    const TensorMap & outputs, const EgoFrame & ego,
    const std::vector<autoware::diffusion_planner::AgentHistory> * neighbor_histories,
    const rclcpp::Time & stamp, const unique_identifier_msgs::msg::UUID & generator_uuid) override;

  const latentdrive::SmoothingParams & smoothing_params() const { return smoothing_; }

private:
  latentdrive::Plan read_plan(const Tensor & tensor) const;
  void write_plan(const latentdrive::Plan & plan, Tensor & tensor) const;

  void publish_plan(const latentdrive::Plan & plan, const rclcpp::Time & stamp) const;

  rclcpp::Node & node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_plan_;
  latentdrive::SmoothingParams smoothing_;
  latentdrive::PlanSmoother smoother_;
  std::optional<Eigen::Matrix4d> previous_map_to_ego_;
  std::optional<rclcpp::Time> previous_stamp_;
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__POSTPROCESS__LATENTDRIVE_POSTPROCESSOR_HPP_
