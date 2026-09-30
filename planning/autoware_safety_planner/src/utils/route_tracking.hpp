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

#ifndef UTILS__ROUTE_TRACKING_HPP_
#define UTILS__ROUTE_TRACKING_HPP_

#include "../type_alias.hpp"

#include <optional>

namespace autoware::safety_planner
{

//! Follows the ego along the route. The safety planner itself never changes lanes, but the
//! planner running in parallel does; once the ego is fully inside another route lanelet the
//! current lanelet is switched to it so that the reference path continues on the new lane.
std::optional<RouteManager> track_current_lanelet(
  RouteManager && route_manager, const Pose & current_pose, const double dist_threshold_m,
  const double yaw_threshold_rad);

}  // namespace autoware::safety_planner

#endif  // UTILS__ROUTE_TRACKING_HPP_
