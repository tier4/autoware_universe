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

#ifndef AUTOWARE__DIFFUSION_PLANNER__MPPI_UTILS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__MPPI_UTILS_HPP_

#include <autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <vector>

namespace autoware::diffusion_planner
{

template <class Segment>
inline std::vector<autoware::mppi_optimizer::Segment> to_mppi_segments(
  const std::vector<Segment> & segments)
{
  std::vector<autoware::mppi_optimizer::Segment> result;
  result.reserve(segments.size());
  for (const auto & segment : segments) {
    result.push_back(
      {static_cast<float>(boost::geometry::get<0, 0>(segment)),
       static_cast<float>(boost::geometry::get<0, 1>(segment)),
       static_cast<float>(boost::geometry::get<1, 0>(segment)),
       static_cast<float>(boost::geometry::get<1, 1>(segment))});
  }
  return result;
}

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__MPPI_UTILS_HPP_
