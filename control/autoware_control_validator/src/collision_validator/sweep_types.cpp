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

#include "autoware/control_validator/collision_validator/sweep_types.hpp"

#include <utility>
#include <vector>

namespace autoware::control_validator::collision_validator
{
EgoSweep::EgoSweep(
  detail::TimeTrajectory times_in, detail::PoseTrajectory poses_in,
  const detail::FootprintTrajectory & footprints, std::vector<bool> stopped_in,
  const bool forward_in)
: times(std::move(times_in)),
  poses(std::move(poses_in)),
  stopped(std::move(stopped_in)),
  trajectory(times, poses, footprints),
  forward(forward_in)
{
  polygons.reserve(times.size());
  for (size_t k = 0; k < times.size(); ++k) {
    polygons.push_back(
      std::visit(
        [k](const auto & footprint) { return detail::to_polygon2d(footprint.at(k)); }, footprints));
  }
}
}  // namespace autoware::control_validator::collision_validator
