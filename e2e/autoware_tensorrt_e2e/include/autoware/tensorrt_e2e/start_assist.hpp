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


#ifndef AUTOWARE__TENSORRT_E2E__START_ASSIST_HPP_
#define AUTOWARE__TENSORRT_E2E__START_ASSIST_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <vector>

namespace autoware::tensorrt_e2e
{

struct StartAssistParams
{
  bool enable{false};
  //! The assist trajectory reaches distance_m, duration_s after the engagement: constant
  //! acceleration 2 * distance_m / duration_s^2, capped at max_speed_mps.
  double distance_m{5.0};
  double duration_s{4.0};
  double max_speed_mps{2.5};
  //! Measured speed at which the model takes over.
  double handover_speed_mps{0.5};
  //! The model takes over after this long even if the vehicle has not reached handover speed.
  double timeout_s{4.0};
  //! The assist only starts when the vehicle is engaged while slower than this.
  double standstill_speed_mps{0.3};
};

/**
 * @brief Gets the vehicle moving after it is switched to autonomous at standstill, where the
 * model tends to keep planning a stop: until the vehicle reaches handover speed, the published
 * trajectory is a straight run along the current heading with a fixed acceleration ramp that
 * starts at the engagement, then the model's plan is published again. Engaging while already
 * moving, and stops later in the same autonomous session, leave the model in charge.
 */
class StartAssist
{
public:
  enum class Event { kNone, kStarted, kHandedOver, kTimedOut, kDisengaged };

  explicit StartAssist(const StartAssistParams & params);

  /**
   * @brief Advance once per tick.
   * @param engaged Whether the vehicle is under autonomous control.
   * @param speed Measured longitudinal speed [m/s].
   * @param now Tick time [s].
   */
  Event update(bool engaged, double speed, double now);

  bool active() const { return active_; }
  double acceleration() const;

  /**
   * @brief The assist trajectory from `pose`: points `time_step` apart over `horizon` seconds,
   * the first one `time_step` ahead (like the model's), speed and spacing following the ramp
   * that started at the engagement.
   */
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> make_points(
    const geometry_msgs::msg::Pose & pose, double now, double time_step, double horizon) const;

private:
  double distance_at(double t) const;  //!< Distance along the ramp t seconds after engagement.
  double speed_at(double t) const;

  StartAssistParams params_;
  std::optional<bool> was_engaged_;  //!< nullopt until the first observation (no edge then).
  bool active_{false};
  double engage_time_{0.0};
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__START_ASSIST_HPP_
