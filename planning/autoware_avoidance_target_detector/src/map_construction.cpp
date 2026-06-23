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

#include "autoware/avoidance_target_detector/map_construction.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::avoidance_target_detector
{

namespace traffic_rules
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

}  // namespace traffic_rules

namespace
{

// Temporary debug code. Must be removed before release.
void export_debug_map(const lanelet::LaneletMapPtr & map, const LaneletRoute & route)
{
  const auto package_share_directory =
    ament_index_cpp::get_package_share_directory("autoware_avoidance_target_detector");
  const auto debug_map_path_str = package_share_directory + "/debug_map.osm";

  lanelet::LaneletMapPtr debug_map = std::make_shared<lanelet::LaneletMap>();
  const auto routing_graph = traffic_rules::create_goal_purpose_routing_graph(*map);

  autoware_planning_msgs::msg::LaneletRoute enhanced_route;
  enhanced_route.header = route.header;
  enhanced_route.start_pose = route.start_pose;
  enhanced_route.goal_pose = route.goal_pose;

  std::set<lanelet::Id> lanelet_ids;
  for (const auto & segment : route.segments) {
    autoware_planning_msgs::msg::LaneletSegment enhanced_segment;
    enhanced_segment.preferred_primitive = segment.preferred_primitive;
    for (const auto & primitive : segment.primitives) {
      if (!map->laneletLayer.exists(primitive.id)) {
        continue;
      }
      autoware_planning_msgs::msg::LaneletPrimitive prim;
      prim.id = primitive.id;
      prim.primitive_type = "lane";
      lanelet::Lanelet lanelet = map->laneletLayer.get(primitive.id);
      enhanced_segment.primitives.push_back(prim);

      if (const auto left_lanelet = traffic_rules::get_left_lanelet(*routing_graph, lanelet)) {
        autoware_planning_msgs::msg::LaneletPrimitive prim;
        prim.id = left_lanelet->id();
        prim.primitive_type = "lane";
        enhanced_segment.primitives.push_back(prim);
        lanelet_ids.insert(left_lanelet->id());
      }
      if (const auto right_lanelet = traffic_rules::get_right_lanelet(*routing_graph, lanelet)) {
        autoware_planning_msgs::msg::LaneletPrimitive prim;
        prim.id = right_lanelet->id();
        prim.primitive_type = "lane";
        enhanced_segment.primitives.push_back(prim);
        lanelet_ids.insert(right_lanelet->id());
      }
      lanelet_ids.insert(primitive.id);
    }
    enhanced_route.segments.push_back(enhanced_segment);
  }

  // Get siblings
  for (size_t i = 0; i + 2 < enhanced_route.segments.size(); ++i) {
    auto & segment = enhanced_route.segments[i + 1];

    std::set<lanelet::Id> next_ids;
    for (const auto & prim : enhanced_route.segments[i + 2].primitives) {
      next_ids.insert(prim.id);
    }

    std::set<lanelet::Id> current_ids;
    for (const auto & prim : segment.primitives) {
      current_ids.insert(prim.id);
    }

    for (const auto & prev_prim : enhanced_route.segments[i].primitives) {
      if (!map->laneletLayer.exists(prev_prim.id)) {
        continue;
      }
      const auto lanelet = map->laneletLayer.get(prev_prim.id);
      for (const auto & candidate : routing_graph->following(lanelet)) {
        if (current_ids.count(candidate.id()) > 0) {
          continue;
        }
        const auto next_lanelets = routing_graph->following(candidate);
        const bool connects_to_next_segment = std::any_of(
          next_lanelets.begin(), next_lanelets.end(),
          [&](const auto & next) { return next_ids.count(next.id()) > 0; });
        if (!connects_to_next_segment) {
          continue;
        }

        autoware_planning_msgs::msg::LaneletPrimitive sibling_prim;
        sibling_prim.id = candidate.id();
        sibling_prim.primitive_type = "lane";
        segment.primitives.push_back(sibling_prim);
        current_ids.insert(candidate.id());
        lanelet_ids.insert(candidate.id());
      }
    }
  }

  for (const auto lanelet_id : lanelet_ids) {
    if (!map->laneletLayer.exists(lanelet_id)) {
      continue;
    }
    lanelet::Lanelet lanelet = map->laneletLayer.get(lanelet_id);
    const auto reg_elements = lanelet.regulatoryElements();
    for (const auto & elem : reg_elements) {
      lanelet.removeRegulatoryElement(elem);
    }
    debug_map->add(lanelet);
  }

  constexpr double k_road_border_near_distance_m = 1.5;
  std::set<lanelet::Id> road_border_ids;
  for (const auto lanelet_id : lanelet_ids) {
    if (!map->laneletLayer.exists(lanelet_id)) {
      continue;
    }
    const auto lanelet = map->laneletLayer.get(lanelet_id);
    const auto nearby_linestrings =
      lanelet::geometry::findWithin2d(map->lineStringLayer, lanelet, k_road_border_near_distance_m);
    for (const auto & nearby_linestring : nearby_linestrings) {
      const auto & linestring = nearby_linestring.second;
      if (road_border_ids.count(linestring.id()) > 0) {
        continue;
      }
      constexpr auto no_type = "none";
      const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, no_type);
      if (type != "road_border") {
        continue;
      }
      road_border_ids.insert(linestring.id());
      debug_map->add(linestring);
    }
  }

  // get map origin

  constexpr double origin_lat = 35.22312494103;
  constexpr double origin_lon = 138.80245834626;
  lanelet::Origin origin({origin_lat, origin_lon});
  lanelet::projection::MGRSProjector projector(origin);
  projector.setMGRSCode(lanelet::GPSPoint{origin_lat, origin_lon, 0.0});

  lanelet::write(debug_map_path_str, *debug_map, projector);
  RCLCPP_INFO(
    rclcpp::get_logger("autoware_avoidance_target_detector"), "Exported debug map to %s",
    debug_map_path_str.c_str());
}

}  // namespace

void create_map(
  const LaneletMapBin & map_bin, const LaneletRoute & route,
  std::shared_ptr<RouteHandler> & route_handler,
  lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  route_handler = std::make_shared<RouteHandler>();
  route_handler->setMap(map_bin);
  route_handler->setRoute(route);

  routing_graph =
    traffic_rules::create_goal_purpose_routing_graph(*route_handler->getLaneletMapPtr());

  export_debug_map(route_handler->getLaneletMapPtr(), route);
}

}  // namespace autoware::avoidance_target_detector
