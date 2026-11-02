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

#ifndef AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__HAZARD_WARNING_SOURCES_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__HAZARD_WARNING_SOURCES_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace autoware::control_validator
{
struct HazardSource
{
  autoware_utils_geometry::Polygon2d shape;  // base_link of the current ego; a point has 1 vertex
  uint8_t risk_level{0};
  builtin_interfaces::msg::Time stamp{};  // odometry stamp used to express the shape in base_link
};

struct HazardWarningSources
{
  std::optional<std::vector<HazardSource>> collision;  // nullopt when not evaluated this cycle
  std::optional<std::vector<HazardSource>> departure;
};
}  // namespace autoware::control_validator

#endif  // AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__HAZARD_WARNING_SOURCES_HPP_
