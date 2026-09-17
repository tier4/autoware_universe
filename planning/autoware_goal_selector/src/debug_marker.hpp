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

#ifndef DEBUG_MARKER_HPP_
#define DEBUG_MARKER_HPP_

#include "types.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <string>
#include <vector>

namespace autoware::goal_selector
{

/// @brief Create markers of the trigger areas and of the goal candidates colored by the vacancy
/// judged for each goal name.
visualization_msgs::msg::MarkerArray create_debug_marker_array(
  const std::vector<Trigger> & triggers, const std::map<std::string, bool> & vacancies,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double margin,
  const rclcpp::Time & stamp);

}  // namespace autoware::goal_selector

#endif  // DEBUG_MARKER_HPP_
