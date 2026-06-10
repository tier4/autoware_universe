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

#include "map_based_prediction/lanelet_util.hpp"

#include <rclcpp/logging.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <unordered_map>

namespace autoware::map_based_prediction::lanelet_util
{

std::optional<lanelet::Id> getTrafficSignalId(const lanelet::ConstLanelet & way_lanelet)
{
  const auto traffic_light_reg_elems =
    way_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
  if (traffic_light_reg_elems.empty()) {
    return std::nullopt;
  }
  if (traffic_light_reg_elems.size() > 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("map_based_prediction"),
      "[Map Based Prediction]: Multiple regulatory elements as TrafficLight are defined to one "
      "lanelet object.");
  }
  return traffic_light_reg_elems.front()->id();
}

std::optional<TrafficLightGroup> getSignalForLanelet(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & signal_id_map,
  const lanelet::ConstLanelet & lanelet)
{
  const auto signal_id = getTrafficSignalId(lanelet);
  if (!signal_id) {
    return std::nullopt;
  }
  const auto it = signal_id_map.find(*signal_id);
  if (it == signal_id_map.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace autoware::map_based_prediction::lanelet_util
