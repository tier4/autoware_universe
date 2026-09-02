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

namespace autoware::safety_planner
{

struct PlannerContext
{
  VehicleInfo vehicle_info;
  std::optional<RouteManager> route_manager;  // TODO(odashima): check API
  Odometry odometry;
  AccelWithCovarianceStamped acceleration;
  SteeringReport steering;
  Pose goal_pose;
  PredictedObjects::ConstSharedPtr predicted_objects;

  // Reference path is setted via safety_planner first step
  PathPointTrajectory reference_path;

  //! goal_pose の reference_path 上の弧長 [m]。接続していない周期 (前方打ち切りで
  //! 経路が goal に届いていない等) は nullopt。判定は下の接続判定と同一
  std::optional<double> goal_arc_length() const;

  //! goal_pose が reference_path の終端に対して縦距離 0.1 m 以内である場合は接続しているとみなす。
  //! 横距離は路肩駐車のような場合だとつながらないので縦距離のみで判定する
  bool is_reference_path_connected_to_goal_pose() const;
};

}  // namespace autoware::safety_planner

#endif  // CONTEXT_HPP_
