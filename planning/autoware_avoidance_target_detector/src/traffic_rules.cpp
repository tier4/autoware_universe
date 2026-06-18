// Copyright 2026 Autoware Foundation
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

#include "autoware/avoidance_target_detector/traffic_rules.hpp"

#include <lanelet2_core/Attribute.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace autoware::avoidance_target_detector::traffic_rules
{

namespace
{
lanelet::traffic_rules::RegisterTrafficRules<GoalPurposeRules> register_goal_purpose_rule(
  k_goal_purpose_location, lanelet::Participants::Vehicle);
}  // namespace

lanelet::Optional<bool> GoalPurposeRules::canPass(
  const lanelet::RegulatoryElementConstPtrs & /*reg_elems*/) const
{
  return {};
}

lanelet::Optional<bool> GoalPurposeRules::canPass(
  const std::string & type, const std::string & /*location*/) const
{
  using ParticipantsMap = std::map<std::string, std::vector<std::string>>;
  using Value = lanelet::AttributeValueString;

  static const ParticipantsMap participant_map{
    {"", {lanelet::Participants::Vehicle}},
    {Value::Road, {lanelet::Participants::Vehicle, lanelet::Participants::Bicycle}},
    {"road_shoulder",
     {lanelet::Participants::Vehicle, lanelet::Participants::Bicycle,
      lanelet::Participants::Pedestrian}},
    {"pedestrian_lane",
     {lanelet::Participants::Vehicle, lanelet::Participants::Bicycle,
      lanelet::Participants::Pedestrian}},
    {Value::Highway, {lanelet::Participants::Vehicle}},
    {Value::BicycleLane, {lanelet::Participants::Vehicle, lanelet::Participants::Bicycle}},
    {Value::PlayStreet,
     {lanelet::Participants::Pedestrian, lanelet::Participants::Bicycle,
      lanelet::Participants::Vehicle}},
    {Value::BusLane,
     {lanelet::Participants::VehicleBus, lanelet::Participants::VehicleEmergency,
      lanelet::Participants::VehicleTaxi}},
    {Value::EmergencyLane, {lanelet::Participants::VehicleEmergency}},
    {Value::Exit,
     {lanelet::Participants::Pedestrian, lanelet::Participants::Bicycle,
      lanelet::Participants::Vehicle}},
    {Value::Walkway, {lanelet::Participants::Pedestrian}},
    {Value::Crosswalk, {lanelet::Participants::Pedestrian}},
    {Value::Stairs, {lanelet::Participants::Pedestrian}},
    {Value::SharedWalkway, {lanelet::Participants::Pedestrian, lanelet::Participants::Bicycle}}};

  const auto participants = participant_map.find(type);
  if (participants == participant_map.end()) {
    return {};
  }

  return std::any_of(
    participants->second.begin(), participants->second.end(),
    [this](const std::string & allowed_participant) {
      return this->participant().compare(0, allowed_participant.size(), allowed_participant) == 0;
    });
}

const lanelet::traffic_rules::CountrySpeedLimits & GoalPurposeRules::countrySpeedLimits() const
{
  return speed_limits_;
}

lanelet::Optional<lanelet::traffic_rules::SpeedLimitInformation> GoalPurposeRules::speedLimit(
  const lanelet::RegulatoryElementConstPtrs & /*reg_elems*/) const
{
  return {};
}

lanelet::routing::RoutingGraphConstPtr create_goal_purpose_routing_graph(
  const lanelet::LaneletMap & lanelet_map)
{
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    k_goal_purpose_location, lanelet::Participants::Vehicle);
  return lanelet::routing::RoutingGraph::build(lanelet_map, *traffic_rules);
}

std::optional<lanelet::ConstLanelet> get_left_lanelet(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::ConstLanelet & lanelet)
{
  if (const auto left_lanelet = routing_graph.left(lanelet)) {
    return *left_lanelet;
  }
  if (const auto adjacent_left_lanelet = routing_graph.adjacentLeft(lanelet)) {
    return *adjacent_left_lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> get_right_lanelet(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::ConstLanelet & lanelet)
{
  if (const auto right_lanelet = routing_graph.right(lanelet)) {
    return *right_lanelet;
  }
  if (const auto adjacent_right_lanelet = routing_graph.adjacentRight(lanelet)) {
    return *adjacent_right_lanelet;
  }
  return std::nullopt;
}

}  // namespace autoware::avoidance_target_detector::traffic_rules
