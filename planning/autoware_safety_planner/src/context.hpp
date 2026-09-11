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

#ifndef CONTEXT_HPP_
#define CONTEXT_HPP_

#include "type_alias.hpp"

#include <optional>
#include <utility>

namespace autoware::safety_planner
{

struct SafetyPlannerInput
{
  VehicleInfo vehicle_info;
  std::optional<RouteManager> route_manager;  // TODO(odashima): check API
  Odometry odometry;
  AccelWithCovarianceStamped acceleration;
  SteeringReport steering;
  Pose goal_pose;
  PredictedObjects::ConstSharedPtr predicted_objects;
};

struct PlannerContext
{
  explicit PlannerContext(const SafetyPlannerInput & input, PathPointTrajectory reference_path)
  : vehicle_info(input.vehicle_info),
    route_manager(input.route_manager),
    odometry(input.odometry),
    acceleration(input.acceleration),
    steering(input.steering),
    goal_pose(input.goal_pose),
    predicted_objects(input.predicted_objects),
    reference_path(std::move(reference_path))
  {
  }

  const VehicleInfo & vehicle_info;
  const std::optional<RouteManager> & route_manager;
  const Odometry & odometry;
  const AccelWithCovarianceStamped & acceleration;
  const SteeringReport & steering;
  const Pose & goal_pose;
  const PredictedObjects::ConstSharedPtr & predicted_objects;
  PathPointTrajectory reference_path;
};

}  // namespace autoware::safety_planner

#endif  // CONTEXT_HPP_
