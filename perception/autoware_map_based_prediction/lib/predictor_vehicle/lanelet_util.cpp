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

#include "autoware/map_based_prediction/predictor_vehicle/lanelet_util.hpp"

#include <rclcpp/logging.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <unordered_map>

namespace autoware::map_based_prediction::lanelet_util
{

bool hasTrafficLight(const lanelet::ConstLanelet & lanelet)
{
  return !lanelet.regulatoryElementsAs<lanelet::TrafficLight>().empty();
}

std::optional<lanelet::ConstLineString3d> getStopLine(const lanelet::ConstLanelet & lanelet)
{
  for (const auto & traffic_light : lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
    if (const auto stop_line = traffic_light->stopLine()) {
      return *stop_line;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLineString3d> getStopLineOrEntryEdge(
  const lanelet::ConstLanelet & lanelet)
{
  if (const auto stop_line = getStopLine(lanelet)) {
    return stop_line;
  }
  const auto & left = lanelet.leftBound();
  const auto & right = lanelet.rightBound();
  if (left.empty() || right.empty()) {
    return std::nullopt;
  }
  const auto lp = left.front();
  const auto rp = right.front();
  return lanelet::ConstLineString3d(
    lanelet::LineString3d(
      lanelet::utils::getId(),
      {lanelet::Point3d(lanelet::utils::getId(), lp.x(), lp.y(), lp.z()),
       lanelet::Point3d(lanelet::utils::getId(), rp.x(), rp.y(), rp.z())}));
}

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
