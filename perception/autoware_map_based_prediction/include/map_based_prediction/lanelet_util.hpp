// Copyright 2026 TIER IV, inc.
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

#ifndef MAP_BASED_PREDICTION__LANELET_UTIL_HPP_
#define MAP_BASED_PREDICTION__LANELET_UTIL_HPP_

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <unordered_map>

namespace autoware::map_based_prediction::lanelet_util
{
using autoware_perception_msgs::msg::TrafficLightGroup;

/// Id of the TrafficLight regulatory element of @p way_lanelet, if any.
std::optional<lanelet::Id> getTrafficSignalId(const lanelet::ConstLanelet & way_lanelet);

/// Latest observation in @p signal_id_map for the traffic light of @p lanelet.
std::optional<TrafficLightGroup> getSignalForLanelet(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & signal_id_map,
  const lanelet::ConstLanelet & lanelet);

}  // namespace autoware::map_based_prediction::lanelet_util

#endif  // MAP_BASED_PREDICTION__LANELET_UTIL_HPP_
