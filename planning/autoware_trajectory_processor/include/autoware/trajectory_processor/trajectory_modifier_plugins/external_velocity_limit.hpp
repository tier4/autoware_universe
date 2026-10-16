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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__EXTERNAL_VELOCITY_LIMIT_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__EXTERNAL_VELOCITY_LIMIT_HPP_

#include "autoware/trajectory_processor/trajectory_modifier_plugins/velocity_limits.hpp"
#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <cstddef>
#include <memory>

namespace autoware::trajectory_processor::plugin
{

namespace detail
{
/// @brief Deceleration magnitude from the message constraints when provided and non-zero,
/// otherwise the default.
double get_external_velocity_limit_deceleration(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit,
  double default_deceleration);
/// @brief Jerk magnitude from the message constraints when provided and non-zero, otherwise the
/// default.
double get_external_velocity_limit_min_jerk(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit, double default_jerk);
}  // namespace detail

class ExternalVelocityLimit : public TrajectoryProcessorPluginBase
{
public:
  ProcessingResult process(
    TrajectoryPoints & traj_points, TrajectoryProcessorData & input) override;

  void update_params(const TrajectoryProcessorParams & params) override;

  /// @brief How the profile of the latest processed candidate was started (Measured when no
  /// profile was generated).
  [[nodiscard]] detail::ProfileStart last_profile_start() const { return last_profile_start_; }

protected:
  void on_initialize(const TrajectoryProcessorParams & params) override;

private:
  PublisherHandle<autoware_internal_debug_msgs::msg::StringStamped> profile_reset_pub_;
  PublisherHandle<autoware_planning_msgs::msg::Trajectory> debug_trajectory_pub_;
  detail::VelocityLimitConstraints constraints_;
  detail::FixedProfileMemory fixed_profile_;
  detail::ProfileStart last_profile_start_{detail::ProfileStart::Measured};
  std::size_t last_clear_revision_{0U};
};

}  // namespace autoware::trajectory_processor::plugin

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__EXTERNAL_VELOCITY_LIMIT_HPP_
