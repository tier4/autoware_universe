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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__MAP_VELOCITY_LIMITS_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__MAP_VELOCITY_LIMITS_HPP_

#include "autoware/trajectory_processor/trajectory_modifier_utils/utils.hpp"
#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

#include <autoware/avoidance_target_detector/boundary.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::trajectory_processor::plugin
{
using autoware::trajectory_processor::TrajectoryProcessorData;
using autoware::trajectory_processor::TrajectoryProcessorParams;
using autoware::trajectory_processor::plugin::ProcessingResult;
using autoware::trajectory_processor::plugin::TrajectoryPoints;
using autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase;
using ModifierParams = trajectory_processor_params::Params;

class MapVelocityLimits : public TrajectoryProcessorPluginBase
{
public:
  MapVelocityLimits() = default;

  ProcessingResult process(
    TrajectoryPoints & traj_points, TrajectoryProcessorData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const TrajectoryProcessorData & input);

  void update_params(const TrajectoryProcessorParams & params) override;

protected:
  autoware_planning_msgs::msg::LaneletRoute::_uuid_type previous_route_uuid_;
  std::shared_ptr<autoware::avoidance_target_detector::ExtendedRouteHandler>
    extended_route_handler_;
  autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides
    limit_overrides_;
  double constant_deceleration_;
  bool enable_smoothing_{true};

  void on_initialize(const TrajectoryProcessorParams & params) override;
};

}  // namespace autoware::trajectory_processor::plugin

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__MAP_VELOCITY_LIMITS_HPP_
