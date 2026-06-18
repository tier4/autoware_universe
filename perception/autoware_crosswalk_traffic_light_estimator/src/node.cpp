// Copyright 2022-2025 UCI SORA Lab, TIER IV, Inc.
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
#include "autoware_crosswalk_traffic_light_estimator/node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>

#include <lanelet2_core/Exceptions.h>

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::crosswalk_traffic_light_estimator
{
namespace
{

bool hasMergeLane(
  const lanelet::ConstLanelet & lanelet_1, const lanelet::ConstLanelet & lanelet_2,
  const lanelet::routing::RoutingGraphPtr & routing_graph_ptr)
{
  const auto next_lanelets_1 = routing_graph_ptr->following(lanelet_1);
  const auto next_lanelets_2 = routing_graph_ptr->following(lanelet_2);

  for (const auto & next_lanelet_1 : next_lanelets_1) {
    for (const auto & next_lanelet_2 : next_lanelets_2) {
      if (next_lanelet_1.id() == next_lanelet_2.id()) {
        return true;
      }
    }
  }

  return false;
}

bool hasMergeLane(
  const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphPtr & routing_graph_ptr)
{
  for (size_t i = 0; i < lanelets.size(); ++i) {
    for (size_t j = i + 1; j < lanelets.size(); ++j) {
      const auto lanelet_1 = lanelets.at(i);
      const auto lanelet_2 = lanelets.at(j);

      if (lanelet_1.id() == lanelet_2.id()) {
        continue;
      }

      const std::string turn_direction_1 = lanelet_1.attributeOr("turn_direction", "none");
      const std::string turn_direction_2 = lanelet_2.attributeOr("turn_direction", "none");
      if (turn_direction_1 == turn_direction_2) {
        continue;
      }

      if (!hasMergeLane(lanelet_1, lanelet_2, routing_graph_ptr)) {
        continue;
      }

      return true;
    }
  }

  return false;
}

/// @brief convert a string to the corresponding traffic signal color
std::optional<uint8_t> str_to_color(std::string_view str)
{
  if (str == "red") {
    return TrafficSignalElement::RED;
  }
  if (str == "green") {
    return TrafficSignalElement::GREEN;
  }
  if (str == "amber") {
    return TrafficSignalElement::AMBER;
  }
  if (str == "white") {
    return TrafficSignalElement::WHITE;
  }
  return std::nullopt;
}

/// @brief parse the input string and extract a rule to estimate a traffic signal
/// @details the string is expected to have format "signal_color_relation:color1:color2"
std::optional<std::pair<uint8_t, uint8_t>> parse_signal_estimation_rules(std::string_view input)
{
  constexpr auto delimiter = ':';
  constexpr std::string_view prefix = "signal_color_relation:";
  if (input.size() < prefix.size() || input.substr(0, prefix.length()) != prefix) {
    return std::nullopt;
  }
  input.remove_prefix(prefix.length());

  // extract the color mapping
  const auto delimiter_pos = input.find(delimiter);
  if (delimiter_pos == std::string_view::npos) {
    return std::nullopt;
  }

  std::string_view from_str = input.substr(0, delimiter_pos);
  std::string_view to_str = input.substr(delimiter_pos + 1);

  if (const auto from_color = str_to_color(from_str)) {
    if (const auto to_color = str_to_color(to_str)) {
      return std::make_pair(*from_color, *to_color);
    }
  }
  return std::nullopt;
}

/// @brief extract ids from the input string
/// @details the string is expected to have format "id1,id2,...", without any space
lanelet::Ids parse_ids(std::string_view input)
{
  lanelet::Ids ids;
  if (input.empty()) {
    return ids;
  }

  constexpr auto delimiter = ',';
  size_t start = 0;
  size_t end = 0;
  lanelet::Id id{};
  while ((end = input.find(delimiter, start)) != std::string_view::npos) {
    const auto [_, err] = std::from_chars(input.data() + start, input.data() + end, id);
    if (err == std::errc()) {
      ids.push_back(id);
    }
    start = end + 1;
  }
  const auto [_, err] = std::from_chars(input.data() + start, input.data() + input.size(), id);
  if (err == std::errc()) {
    ids.push_back(id);
  }
  return ids;
}

// Sorted ids of a reg_elem's physical faces (role=refers linestrings).
std::vector<lanelet::Id> collect_face_ids(const lanelet::TrafficLight & traffic_light)
{
  std::vector<lanelet::Id> face_ids;
  for (const auto & signal_face : traffic_light.trafficLights()) {
    if (!signal_face.isLineString()) {
      continue;
    }
    face_ids.push_back(static_cast<lanelet::ConstLineString3d>(signal_face).id());
  }
  std::sort(face_ids.begin(), face_ids.end());
  return face_ids;
}

TrafficSignalElement make_solid_circle(const uint8_t color)
{
  TrafficSignalElement element;
  element.color = color;
  element.shape = TrafficSignalElement::CIRCLE;
  element.confidence = 1.0;
  return element;
}

template <typename ElementPredicate>
bool has_element_matching(
  const lanelet::Id & reg_elem_id, const TrafficLightIdMap & traffic_light_id_map,
  const ElementPredicate & is_match)
{
  const auto signal_entry = traffic_light_id_map.find(reg_elem_id);
  if (signal_entry == traffic_light_id_map.end()) {
    return false;
  }
  const auto & elements = signal_entry->second.first.elements;
  return std::any_of(elements.begin(), elements.end(), is_match);
}

boost::optional<uint8_t> highest_confidence_circle_color(
  const lanelet::Id & reg_elem_id, const TrafficLightIdMap & traffic_light_id_map)
{
  const auto signal_entry = traffic_light_id_map.find(reg_elem_id);
  if (signal_entry == traffic_light_id_map.end()) {
    return boost::none;
  }
  boost::optional<uint8_t> circle_color{boost::none};
  double highest_confidence = 0.0;
  for (const auto & element : signal_entry->second.first.elements) {
    if (element.shape != TrafficSignalElement::CIRCLE) {
      continue;
    }
    if (element.confidence < highest_confidence) {
      continue;
    }
    highest_confidence = element.confidence;
    circle_color = element.color;
  }
  return circle_color;
}

bool is_shows_green_arrow(
  const lanelet::Id & reg_elem_id, const TrafficLightIdMap & traffic_light_id_map)
{
  return has_element_matching(
    reg_elem_id, traffic_light_id_map, [](const TrafficSignalElement & element) {
      return element.color == TrafficSignalElement::GREEN &&
             element.shape != TrafficSignalElement::CIRCLE &&
             element.shape != TrafficSignalElement::CROSS;
    });
}

bool is_shows_straight_arrow(
  const lanelet::Id & reg_elem_id, const TrafficLightIdMap & traffic_light_id_map)
{
  return has_element_matching(
    reg_elem_id, traffic_light_id_map, [](const TrafficSignalElement & element) {
      return element.color == TrafficSignalElement::GREEN &&
             (element.shape == TrafficSignalElement::UP_ARROW ||
              element.shape == TrafficSignalElement::UP_LEFT_ARROW ||
              element.shape == TrafficSignalElement::UP_RIGHT_ARROW);
    });
}
}  // namespace

CrosswalkTrafficLightEstimatorNode::CrosswalkTrafficLightEstimatorNode(
  const rclcpp::NodeOptions & options)
: Node("crosswalk_traffic_light_estimator", options)
{
  using std::placeholders::_1;

  use_last_detect_color_ = declare_parameter<bool>("use_last_detect_color");
  use_pedestrian_signal_detect_ = declare_parameter<bool>("use_pedestrian_signal_detect");
  last_detect_color_hold_time_ = declare_parameter<double>("last_detect_color_hold_time");
  last_colors_hold_time_ = declare_parameter<double>("last_colors_hold_time");

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&CrosswalkTrafficLightEstimatorNode::onMap, this, _1));
  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&CrosswalkTrafficLightEstimatorNode::onRoute, this, _1));
  sub_traffic_light_array_ = create_subscription<TrafficSignalArray>(
    "~/input/classified/traffic_signals", rclcpp::QoS{1},
    std::bind(&CrosswalkTrafficLightEstimatorNode::onTrafficLightArray, this, _1));

  pub_traffic_light_array_ =
    this->create_publisher<TrafficSignalArray>("~/output/traffic_signals", rclcpp::QoS{1});
  pub_processing_time_ = std::make_shared<DebugPublisher>(this, "~/debug");
}

void CrosswalkTrafficLightEstimatorNode::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "[CrosswalkTrafficLightEstimatorNode]: Start loading lanelet");
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));

  auto routing_graph_and_traffic_rules =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr_);

  routing_graph_ptr_ =
    autoware::experimental::lanelet2_utils::remove_const(routing_graph_and_traffic_rules.first);
  traffic_rules_ptr_ = routing_graph_and_traffic_rules.second;

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);
  buildIntersectionSignalIndex();
  RCLCPP_DEBUG(get_logger(), "[CrosswalkTrafficLightEstimatorNode]: Map is loaded");
}

void CrosswalkTrafficLightEstimatorNode::buildIntersectionSignalIndex()
{
  all_intersection_ids_.clear();
  signals_by_intersection_.clear();
  intersection_by_signal_.clear();
  std::unordered_set<std::string> seen_intersection;
  for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
    const std::string intersection_id = lanelet.attributeOr("intersection_area", "none");
    if (intersection_id == "none") {
      continue;
    }
    if (seen_intersection.insert(intersection_id).second) {
      all_intersection_ids_.push_back(intersection_id);
    }

    // for no route case
    for (const auto & traffic_light : lanelet.regulatoryElementsAs<const lanelet::TrafficLight>()) {
      const auto reg_elem_id = traffic_light->id();
      auto & reg_elem_ids = signals_by_intersection_[intersection_id];
      if (std::find(reg_elem_ids.begin(), reg_elem_ids.end(), reg_elem_id) == reg_elem_ids.end()) {
        reg_elem_ids.push_back(reg_elem_id);
      }
      intersection_by_signal_[reg_elem_id] = intersection_id;
    }
  }
}

void CrosswalkTrafficLightEstimatorNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  if (lanelet_map_ptr_ == nullptr) {
    RCLCPP_WARN(get_logger(), "cannot set traffic light in route because don't receive map");
    return;
  }
  route_received_ = true;

  lanelet::ConstLanelets route_lanelets;
  for (const auto & segment : msg->segments) {
    for (const auto & primitive : segment.primitives) {
      try {
        route_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(primitive.id));
      } catch (const lanelet::NoSuchPrimitiveError & ex) {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
        return;
      }
    }
  }

  conflicting_crosswalks_.clear();

  for (const auto & route_lanelet : route_lanelets) {
    constexpr int PEDESTRIAN_GRAPH_ID = 1;
    const auto conflict_lls =
      overall_graphs_ptr_->conflictingInGraph(route_lanelet, PEDESTRIAN_GRAPH_ID);
    for (const auto & lanelet : conflict_lls) {
      conflicting_crosswalks_.push_back(lanelet);
    }
  }

  std::unordered_set<std::string> route_intersections;
  for (const auto & route_lanelet : route_lanelets) {
    const std::string intersection_id = route_lanelet.attributeOr("intersection_area", "none");
    if (intersection_id != "none") {
      route_intersections.insert(intersection_id);
    }
  }
  target_intersection_ids_.clear();
  for (const auto & intersection_id : all_intersection_ids_) {
    if (route_intersections.count(intersection_id) > 0) {
      target_intersection_ids_.push_back(intersection_id);
    }
  }
}

void CrosswalkTrafficLightEstimatorNode::update_crosswalk_overrides_from_map(
  std::unordered_map<lanelet::Id, uint8_t> & crosswalk_traffic_signal_overrides,
  const lanelet::Id traffic_light_group_id, const TrafficLightIdMap & traffic_light_id_map)
{
  const auto traffic_light_it =
    lanelet_map_ptr_->regulatoryElementLayer.find(traffic_light_group_id);
  if (traffic_light_it == lanelet_map_ptr_->regulatoryElementLayer.end()) {
    RCLCPP_WARN(
      get_logger(), "Traffic light group ID %ld not found in regulatory element layer",
      traffic_light_group_id);
    return;
  }
  const auto & traffic_light = *traffic_light_it;
  const auto current_vehicle_traffic_light_color =
    getHighestConfidenceTrafficSignal(traffic_light->id(), traffic_light_id_map);

  for (const auto & attribute : traffic_light->attributes()) {
    const auto & color_mapping = parse_signal_estimation_rules(attribute.first);
    if (!color_mapping) {
      continue;
    }
    const auto & [from_color, to_color] = *color_mapping;
    if (from_color != current_vehicle_traffic_light_color) {
      continue;
    }
    for (const auto id : parse_ids(attribute.second.value())) {
      crosswalk_traffic_signal_overrides[id] = to_color;
    }
  }
}

void CrosswalkTrafficLightEstimatorNode::estimateIntersectionTrafficSignal(
  const std::vector<std::string> & target_intersection_ids,
  const TrafficLightIdMap & traffic_light_id_map,
  std::unordered_map<lanelet::Id, uint8_t> & estimated_intersection_traffic_signal_overrides)
{
  for (const std::string & intersection_id : target_intersection_ids) {
    if (signals_by_intersection_.count(intersection_id) == 0) {
      continue;
    }
    const std::vector<lanelet::Id> & reg_elem_ids = signals_by_intersection_.at(intersection_id);
    const std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> same_signal_head_members =
      buildSameSignalHeadMembers(reg_elem_ids);

    for (const lanelet::Id reg_elem_id : reg_elem_ids) {
      if (!lanelet_map_ptr_->regulatoryElementLayer.exists(reg_elem_id)) {
        continue;
      }
      const lanelet::RegulatoryElementPtr traffic_light =
        lanelet_map_ptr_->regulatoryElementLayer.get(reg_elem_id);
      const SignalHeadPhase phase = classifySignalHeadPhase(
        traffic_light->id(), traffic_light_id_map, same_signal_head_members);

      for (const std::pair<const std::string, lanelet::Attribute> & attribute :
           traffic_light->attributes()) {
        const std::optional<std::pair<uint8_t, uint8_t>> color_mapping =
          parse_signal_estimation_rules(attribute.first);
        if (!color_mapping) {
          continue;
        }
        overwrite_to_color_by_phase(
          phase, color_mapping->first, color_mapping->second, parse_ids(attribute.second.value()),
          estimated_intersection_traffic_signal_overrides);
      }
    }
  }
}

void CrosswalkTrafficLightEstimatorNode::overwrite_to_color_by_phase(
  const SignalHeadPhase & phase, const uint8_t from_color, const uint8_t to_color,
  const lanelet::Ids & target_reg_elem_ids,
  std::unordered_map<lanelet::Id, uint8_t> & estimated_intersection_traffic_signal_overrides) const
{
  const auto overwrite_targets = [&](const uint8_t color) {
    for (const lanelet::Id target_reg_elem_id : target_reg_elem_ids) {
      estimated_intersection_traffic_signal_overrides[target_reg_elem_id] = color;
    }
  };

  switch (phase.movement) {
    case SignalHeadMovement::protected_turn_only:
      // Opposing and cross all stop, which a from->to color cannot express, so force red.
      overwrite_targets(TrafficSignalElement::RED);
      return;
    case SignalHeadMovement::through_active:
      // from color is GREEN here; apply the rule whose from matches.
      if (phase.from_color && from_color == phase.from_color.get()) {
        overwrite_targets(to_color);
      }
      return;
    case SignalHeadMovement::not_proceeding:
      // from color is the circle color; apply the rule whose from matches.
      if (phase.from_color && from_color == phase.from_color.get()) {
        overwrite_targets(to_color);
      }
      return;
  }
}

std::unordered_map<lanelet::Id, std::vector<lanelet::Id>>
CrosswalkTrafficLightEstimatorNode::buildSameSignalHeadMembers(
  const std::vector<lanelet::Id> & traffic_light_reg_elem_ids) const
{
  // A same-signal-head group bundles reg_elems whose physical faces match exactly; matching by
  // exact set (not overlap) keeps independent phases such as green arrows apart.
  std::map<std::vector<lanelet::Id>, std::vector<lanelet::Id>> reg_elem_ids_by_face_set;
  for (const auto reg_elem_id : traffic_light_reg_elem_ids) {
    const auto reg_elem_it = lanelet_map_ptr_->regulatoryElementLayer.find(reg_elem_id);
    if (reg_elem_it == lanelet_map_ptr_->regulatoryElementLayer.end()) {
      continue;
    }
    const auto traffic_light = std::dynamic_pointer_cast<const lanelet::TrafficLight>(*reg_elem_it);
    if (!traffic_light) {
      continue;
    }
    reg_elem_ids_by_face_set[collect_face_ids(*traffic_light)].push_back(reg_elem_id);
  }
  std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> same_signal_head_members;
  for (const auto & [face_linestring_ids, same_head_reg_elem_ids] : reg_elem_ids_by_face_set) {
    for (const auto reg_elem_id : same_head_reg_elem_ids) {
      same_signal_head_members[reg_elem_id] = same_head_reg_elem_ids;
    }
  }
  return same_signal_head_members;
}

CrosswalkTrafficLightEstimatorNode::SignalHeadPhase
CrosswalkTrafficLightEstimatorNode::classifySignalHeadPhase(
  const lanelet::Id & reg_elem_id, const TrafficLightIdMap & traffic_light_id_map,
  const std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> & same_signal_head_members) const
{
  const auto circle_color =
    getSignalHeadCircleColor(reg_elem_id, traffic_light_id_map, same_signal_head_members);

  // Through is active on a green circle, or on a straight arrow (which also lets the
  // opposing through flow). Normalize both to GREEN.
  // 緑丸、または青直進矢印（対向直進も同時に流れる）のとき through。どちらも GREEN に正規化。
  if (
    isSignalHeadShowsStraightArrow(reg_elem_id, traffic_light_id_map, same_signal_head_members) ||
    (circle_color && circle_color.get() == TrafficSignalElement::GREEN)) {
    return {SignalHeadMovement::through_active, TrafficSignalElement::GREEN};
  }

  // Turn-only green arrow stops the opposing and cross traffic, which a from->to color
  // cannot express, so mark it for a forced red instead.
  // 直進でないターン矢印（赤丸＋青右矢印など）。対向・交差を一律 RED に倒す（from→to の色対応では
  // 表せないため別扱い）。右折では物理的に正しく、左折のみのときは安全側の近似。
  if (isSignalHeadShowsGreenArrow(reg_elem_id, traffic_light_id_map, same_signal_head_members)) {
    return {SignalHeadMovement::protected_turn_only, boost::none};
  }

  // Plain red/amber/unknown circle: use the circle color as the rule's from color.
  // 赤丸・黄丸・UNKNOWN の丸。丸の色をそのままルールの from に使う。
  return {SignalHeadMovement::not_proceeding, circle_color};
}

boost::optional<uint8_t> CrosswalkTrafficLightEstimatorNode::getSignalHeadCircleColor(
  const lanelet::Id & reg_elem_id, const TrafficLightIdMap & traffic_light_id_map,
  const std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> & same_signal_head_members) const
{
  // Keep the latest valid color so one occluded (UNKNOWN) reg_elem this frame does
  // not blank the rule for the whole signal-head group.
  boost::optional<uint8_t> latest_valid_color{boost::none};
  for (const auto member_reg_elem_id :
       getSameSignalHeadMemberIds(reg_elem_id, same_signal_head_members)) {
    const auto color = highest_confidence_circle_color(member_reg_elem_id, traffic_light_id_map);
    if (color && color.get() != TrafficSignalElement::UNKNOWN) {
      latest_valid_color = color;
    }
  }
  return latest_valid_color;
}

bool CrosswalkTrafficLightEstimatorNode::isSignalHeadShowsStraightArrow(
  const lanelet::Id & reg_elem_id, const TrafficLightIdMap & traffic_light_id_map,
  const std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> & same_signal_head_members) const
{
  const auto member_reg_elem_ids =
    getSameSignalHeadMemberIds(reg_elem_id, same_signal_head_members);
  return std::any_of(
    member_reg_elem_ids.begin(), member_reg_elem_ids.end(),
    [&](const lanelet::Id member_reg_elem_id) {
      return is_shows_straight_arrow(member_reg_elem_id, traffic_light_id_map);
    });
}

bool CrosswalkTrafficLightEstimatorNode::isSignalHeadShowsGreenArrow(
  const lanelet::Id & reg_elem_id, const TrafficLightIdMap & traffic_light_id_map,
  const std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> & same_signal_head_members) const
{
  const auto member_reg_elem_ids =
    getSameSignalHeadMemberIds(reg_elem_id, same_signal_head_members);
  return std::any_of(
    member_reg_elem_ids.begin(), member_reg_elem_ids.end(),
    [&](const lanelet::Id member_reg_elem_id) {
      return is_shows_green_arrow(member_reg_elem_id, traffic_light_id_map);
    });
}

std::vector<lanelet::Id> CrosswalkTrafficLightEstimatorNode::getSameSignalHeadMemberIds(
  const lanelet::Id & reg_elem_id,
  const std::unordered_map<lanelet::Id, std::vector<lanelet::Id>> & same_signal_head_members) const
{
  const auto it = same_signal_head_members.find(reg_elem_id);
  if (it == same_signal_head_members.end()) {
    return {reg_elem_id};
  }
  return it->second;
}

void CrosswalkTrafficLightEstimatorNode::onTrafficLightArray(
  const TrafficSignalArray::ConstSharedPtr msg)
{
  if (lanelet_map_ptr_ == nullptr) {
    RCLCPP_WARN(get_logger(), "cannot process traffic light array because the map is not received");
    return;
  }

  // example
  // group_id=10323:  [RED/CIRCLE(1.00), GREEN/LEFT_ARROW(1.00), GREEN/UP_ARROW(1.00)]
  // group_id=190426: [RED/CIRCLE(1.00), GREEN/LEFT_ARROW(1.00), GREEN/UP_ARROW(1.00)]
  // group_id=2118985:[RED/CIRCLE(1.00), GREEN/LEFT_ARROW(1.00), GREEN/UP_ARROW(1.00)]
  // group_id=179970: [RED/CIRCLE(1.00)]

  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("Total");

  TrafficSignalArray output = *msg;

  TrafficLightIdMap traffic_light_id_map;

  std::unordered_map<lanelet::Id, uint8_t> crosswalk_traffic_signal_overrides;
  for (const auto & traffic_signal : msg->traffic_light_groups) {
    traffic_light_id_map[traffic_signal.traffic_light_group_id] =
      std::pair<TrafficSignal, rclcpp::Time>(traffic_signal, get_clock()->now());
  }
  // we need the full traffic_light_id_map before calculating overrides from map
  for (const auto & traffic_signal : msg->traffic_light_groups) {
    update_crosswalk_overrides_from_map(
      crosswalk_traffic_signal_overrides, traffic_signal.traffic_light_group_id,
      traffic_light_id_map);
  }

  for (const auto & crosswalk : conflicting_crosswalks_) {
    constexpr int VEHICLE_GRAPH_ID = 0;
    const auto conflict_lls = overall_graphs_ptr_->conflictingInGraph(crosswalk, VEHICLE_GRAPH_ID);
    const auto non_red_lanelets = getNonRedLanelets(conflict_lls, traffic_light_id_map);

    const auto crosswalk_tl_color = estimateCrosswalkTrafficSignal(crosswalk, non_red_lanelets);
    setCrosswalkTrafficSignal(
      crosswalk, crosswalk_tl_color, *msg, output, crosswalk_traffic_signal_overrides);
  }

  // Estimate opposing/cross vehicle signals for the intersections the ego cares about.
  // With a route, use the intersections it traverses; otherwise resolve them from the
  // detected signals so the estimation still runs route-independently (replay/verification).
  std::vector<std::string> target_intersection_ids;
  if (route_received_) {
    target_intersection_ids = target_intersection_ids_;
  } else {
    std::unordered_set<std::string> active_intersections;
    for (const auto & traffic_signal : msg->traffic_light_groups) {
      const auto it = intersection_by_signal_.find(traffic_signal.traffic_light_group_id);
      if (it != intersection_by_signal_.end()) {
        active_intersections.insert(it->second);
      }
    }
    for (const auto & intersection_id : all_intersection_ids_) {
      if (active_intersections.count(intersection_id) > 0) {
        target_intersection_ids.push_back(intersection_id);
      }
    }
  }

  std::unordered_map<lanelet::Id, uint8_t> estimated_intersection_traffic_signal_overrides;
  estimateIntersectionTrafficSignal(
    target_intersection_ids, traffic_light_id_map, estimated_intersection_traffic_signal_overrides);

  // Merge crosswalk (pedestrian) and intersection (vehicle) overrides into the output.
  mergeOverridesIntoTrafficSignals(
    crosswalk_traffic_signal_overrides, estimated_intersection_traffic_signal_overrides, output);

  removeDuplicateIds(output);

  updateLastDetectedSignal(traffic_light_id_map);
  updateLastDetectedSignals(traffic_light_id_map);

  pub_traffic_light_array_->publish(output);
  pub_processing_time_->publish<Float64Stamped>("processing_time_ms", stop_watch.toc("Total"));

  return;
}

void CrosswalkTrafficLightEstimatorNode::updateLastDetectedSignal(
  const TrafficLightIdMap & traffic_light_id_map)
{
  for (const auto & input_traffic_signal : traffic_light_id_map) {
    const auto & elements = input_traffic_signal.second.first.elements;

    if (elements.empty()) {
      continue;
    }

    if (elements.front().color == TrafficSignalElement::UNKNOWN) {
      continue;
    }

    const auto & id = input_traffic_signal.second.first.traffic_light_group_id;

    if (last_detect_color_.count(id) == 0) {
      last_detect_color_.insert(std::make_pair(id, input_traffic_signal.second));
      continue;
    }

    last_detect_color_.at(id) = input_traffic_signal.second;
  }

  std::vector<int32_t> erase_id_list;
  for (const auto & last_traffic_signal : last_detect_color_) {
    const auto & id = last_traffic_signal.second.first.traffic_light_group_id;

    if (traffic_light_id_map.count(id) == 0) {
      // hold signal recognition results for [last_detect_color_hold_time_] seconds.
      const auto time_from_last_detected =
        (get_clock()->now() - last_traffic_signal.second.second).seconds();
      if (time_from_last_detected > last_detect_color_hold_time_) {
        erase_id_list.emplace_back(id);
      }
    }
  }
  for (const auto id : erase_id_list) {
    last_detect_color_.erase(id);
    is_flashing_.erase(id);
    current_color_state_.erase(id);
  }
}

void CrosswalkTrafficLightEstimatorNode::updateLastDetectedSignals(
  const TrafficLightIdMap & traffic_light_id_map)
{
  for (const auto & input_traffic_signal : traffic_light_id_map) {
    const auto & elements = input_traffic_signal.second.first.elements;

    if (elements.empty()) {
      continue;
    }

    if (
      elements.front().color == TrafficSignalElement::UNKNOWN && elements.front().confidence == 1) {
      continue;
    }

    const auto & id = input_traffic_signal.second.first.traffic_light_group_id;

    if (last_colors_.count(id) == 0) {
      std::vector<TrafficSignalAndTime> signal{input_traffic_signal.second};
      last_colors_.insert(std::make_pair(id, signal));
      continue;
    }

    last_colors_.at(id).push_back(input_traffic_signal.second);
  }

  std::vector<int32_t> erase_id_list;
  for (auto & last_traffic_signal : last_colors_) {
    const auto & id = last_traffic_signal.first;
    for (auto it = last_traffic_signal.second.begin(); it != last_traffic_signal.second.end();) {
      auto sig = (*it).first;
      rclcpp::Time t = (*it).second;

      // hold signal recognition results for [last_colors_hold_time_] seconds.
      const auto time_from_last_detected = (get_clock()->now() - t).seconds();
      if (time_from_last_detected > last_colors_hold_time_) {
        it = last_traffic_signal.second.erase(it);
      } else {
        ++it;
      }
    }
    if (last_traffic_signal.second.empty()) {
      erase_id_list.emplace_back(id);
    }
  }
  for (const auto id : erase_id_list) last_colors_.erase(id);
}

void CrosswalkTrafficLightEstimatorNode::setCrosswalkTrafficSignal(
  const lanelet::ConstLanelet & crosswalk, const uint8_t color, const TrafficSignalArray & msg,
  TrafficSignalArray & output,
  const std::unordered_map<lanelet::Id, uint8_t> & crosswalk_traffic_signal_overrides)
{
  const auto tl_reg_elems = crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>();

  std::unordered_map<lanelet::Id, size_t> valid_id2idx_map;  // detected traffic light
  for (size_t i = 0; i < msg.traffic_light_groups.size(); ++i) {
    const auto & signal = msg.traffic_light_groups[i];
    valid_id2idx_map[signal.traffic_light_group_id] = i;
  }

  std::unordered_map<lanelet::Id, size_t> output_id2idx_map;  // to check duplicate
  for (size_t i = 0; i < output.traffic_light_groups.size(); ++i) {
    const auto & signal = output.traffic_light_groups[i];
    output_id2idx_map[signal.traffic_light_group_id] = i;
  }

  TrafficSignalElement base_traffic_signal_element;
  base_traffic_signal_element.color = color;
  base_traffic_signal_element.shape = TrafficSignalElement::CIRCLE;
  base_traffic_signal_element.confidence = 1.0;

  for (const auto & tl_reg_elem : tl_reg_elems) {
    const lanelet::Id id = tl_reg_elem->id();

    // helper lambda to get or create output signal
    auto get_or_create_output_signal = [&](lanelet::Id id) -> TrafficSignal & {
      if (output_id2idx_map.count(id)) {
        return output.traffic_light_groups[output_id2idx_map[id]];
      } else {
        // element need to be added in later
        TrafficSignal new_signal;
        new_signal.traffic_light_group_id = id;
        output.traffic_light_groups.push_back(new_signal);
        output_id2idx_map[id] = output.traffic_light_groups.size() - 1;
        return output.traffic_light_groups.back();
      }
    };

    TrafficSignal & out_signal = get_or_create_output_signal(id);

    auto replace_out_signal_elements = [&](const TrafficSignalElement & element) {
      out_signal.elements.clear();
      out_signal.elements.push_back(element);
    };

    // 1. Map-based override (highest priority)
    if (auto it = crosswalk_traffic_signal_overrides.find(id);
        it != crosswalk_traffic_signal_overrides.end()) {
      replace_out_signal_elements(base_traffic_signal_element);
      out_signal.elements[0].color = it->second;  // override color
      continue;
    }
    // 2. Use detected pedestrian signal if valid
    if (auto it = valid_id2idx_map.find(id); it != valid_id2idx_map.end()) {
      const auto & detected = msg.traffic_light_groups[it->second];

      if (!use_pedestrian_signal_detect_ || isInvalidDetectionStatus(detected)) {
        // Replace detection with estimated base color
        replace_out_signal_elements(base_traffic_signal_element);
        continue;
      }

      // Update flashing state and apply the most recent color
      updateFlashingState(detected);
      if (out_signal.elements
            .empty()) {  // unnecessary check because msg has detection but for safety
        out_signal.elements.push_back(base_traffic_signal_element);
      }
      out_signal.elements[0].color = updateAndGetColorState(
        detected);  // TODO(MasatoSaeki): determine what value is good for confidence
      continue;
    }

    // 3. No detection available → use estimated vehicle-based color
    replace_out_signal_elements(base_traffic_signal_element);
  }
}

void CrosswalkTrafficLightEstimatorNode::mergeOverridesIntoTrafficSignals(
  const std::unordered_map<lanelet::Id, uint8_t> & crosswalk_traffic_signal_overrides,
  const std::unordered_map<lanelet::Id, uint8_t> & estimated_intersection_traffic_signal_overrides,
  TrafficSignalArray & output)
{
  std::unordered_map<lanelet::Id, size_t> group_index_by_id;
  for (size_t i = 0; i < output.traffic_light_groups.size(); ++i) {
    group_index_by_id[output.traffic_light_groups[i].traffic_light_group_id] = i;
  }

  const auto merge_one = [&](const std::unordered_map<lanelet::Id, uint8_t> & overrides) {
    for (const auto & [group_id, color] : overrides) {
      const auto element = make_solid_circle(color);
      if (const auto it = group_index_by_id.find(group_id); it != group_index_by_id.end()) {
        output.traffic_light_groups[it->second].elements.assign(1, element);
      } else {
        TrafficSignal new_group;
        new_group.traffic_light_group_id = group_id;
        new_group.elements.push_back(element);
        output.traffic_light_groups.push_back(new_group);
        group_index_by_id[group_id] = output.traffic_light_groups.size() - 1;
      }
    }
  };
  merge_one(crosswalk_traffic_signal_overrides);
  merge_one(estimated_intersection_traffic_signal_overrides);
}

bool CrosswalkTrafficLightEstimatorNode::isInvalidDetectionStatus(
  const TrafficSignal & signal) const
{
  // invalid if elements is empty
  if (signal.elements.empty()) {
    return true;
  }
  // check occlusion, backlight(shape is unknown) and no detection(shape is circle)
  if (
    signal.elements.front().color == TrafficSignalElement::UNKNOWN &&
    signal.elements.front().confidence == 0.0) {
    return true;
  }

  return false;
}

void CrosswalkTrafficLightEstimatorNode::updateFlashingState(const TrafficSignal & signal)
{
  const auto id = signal.traffic_light_group_id;

  // no record of detected color in last_detect_color_hold_time_
  if (is_flashing_.count(id) == 0) {
    is_flashing_.insert(std::make_pair(id, false));
    return;
  }

  // flashing green
  if (
    !signal.elements.empty() && signal.elements.front().color == TrafficSignalElement::UNKNOWN &&
    signal.elements.front().confidence != 0 &&  // not due to occlusion
    current_color_state_.at(id) != TrafficSignalElement::UNKNOWN) {
    is_flashing_.at(id) = true;
    return;
  }

  // history exists
  if (last_colors_.count(id) > 0) {
    std::vector<TrafficSignalAndTime> history = last_colors_.at(id);
    for (const auto & h : history) {
      if (h.first.elements.front().color != signal.elements.front().color) {
        // keep the current value if not same with input signal
        return;
      }
    }
    // all history is same with input signal
    is_flashing_.at(id) = false;
  }

  // no record of detected color in last_color_hold_time_
  // keep the current value
  return;
}

uint8_t CrosswalkTrafficLightEstimatorNode::updateAndGetColorState(const TrafficSignal & signal)
{
  const auto id = signal.traffic_light_group_id;
  const auto color = signal.elements[0].color;

  if (current_color_state_.count(id) == 0) {
    current_color_state_.insert(std::make_pair(id, color));
  } else if (is_flashing_.at(id) == false) {
    current_color_state_.at(id) = color;
  } else if (is_flashing_.at(id) == true) {
    if (
      current_color_state_.at(id) == TrafficSignalElement::GREEN &&
      color == TrafficSignalElement::RED) {
      current_color_state_.at(id) = TrafficSignalElement::RED;
    } else if (
      current_color_state_.at(id) == TrafficSignalElement::RED &&
      color == TrafficSignalElement::GREEN) {
      current_color_state_.at(id) = TrafficSignalElement::GREEN;
    } else if (current_color_state_.at(id) == TrafficSignalElement::UNKNOWN) {
      if (color == TrafficSignalElement::GREEN || color == TrafficSignalElement::UNKNOWN)
        current_color_state_.at(id) = TrafficSignalElement::GREEN;
      if (color == TrafficSignalElement::RED)
        current_color_state_.at(id) = TrafficSignalElement::RED;
    }
  }

  return current_color_state_.at(id);
}

lanelet::ConstLanelets CrosswalkTrafficLightEstimatorNode::getNonRedLanelets(
  const lanelet::ConstLanelets & lanelets, const TrafficLightIdMap & traffic_light_id_map) const
{
  lanelet::ConstLanelets non_red_lanelets{};

  for (const auto & lanelet : lanelets) {
    const auto tl_reg_elems = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();

    if (tl_reg_elems.empty()) {
      continue;
    }

    const auto tl_reg_elem = tl_reg_elems.front();
    const auto current_detected_signal =
      getHighestConfidenceTrafficSignal(tl_reg_elem->id(), traffic_light_id_map);

    if (!current_detected_signal && !use_last_detect_color_) {
      continue;
    }

    const auto current_is_not_red =
      current_detected_signal ? current_detected_signal.get() == TrafficSignalElement::GREEN ||
                                  current_detected_signal.get() == TrafficSignalElement::AMBER
                              : true;

    const auto current_is_unknown_or_none =
      current_detected_signal ? current_detected_signal.get() == TrafficSignalElement::UNKNOWN
                              : true;

    const auto last_detected_signal =
      getHighestConfidenceTrafficSignal(tl_reg_elem->id(), last_detect_color_);

    if (!last_detected_signal) {
      continue;
    }

    const auto was_not_red = current_is_unknown_or_none &&
                             (last_detected_signal.get() == TrafficSignalElement::GREEN ||
                              last_detected_signal.get() == TrafficSignalElement::AMBER) &&
                             use_last_detect_color_;

    if (!current_is_not_red && !was_not_red) {
      continue;
    }

    non_red_lanelets.push_back(lanelet);
  }

  return non_red_lanelets;
}

uint8_t CrosswalkTrafficLightEstimatorNode::estimateCrosswalkTrafficSignal(
  const lanelet::ConstLanelet & crosswalk, const lanelet::ConstLanelets & non_red_lanelets) const
{
  bool has_left_non_red_lane = false;
  bool has_right_non_red_lane = false;
  bool has_straight_non_red_lane = false;
  bool has_related_non_red_tl = false;

  const std::string related_tl_id = crosswalk.attributeOr("related_traffic_light", "none");

  for (const auto & lanelet : non_red_lanelets) {
    const std::string turn_direction = lanelet.attributeOr("turn_direction", "none");

    if (turn_direction == "left") {
      has_left_non_red_lane = true;
    } else if (turn_direction == "right") {
      has_right_non_red_lane = true;
    } else {
      has_straight_non_red_lane = true;
    }

    const auto tl_reg_elems = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
    if (tl_reg_elems.front()->id() == std::atoi(related_tl_id.c_str())) {
      has_related_non_red_tl = true;
    }
  }

  if (has_straight_non_red_lane || has_related_non_red_tl) {
    return TrafficSignalElement::RED;
  }

  const auto has_merge_lane = hasMergeLane(non_red_lanelets, routing_graph_ptr_);
  return !has_merge_lane && has_left_non_red_lane && has_right_non_red_lane
           ? TrafficSignalElement::RED
           : TrafficSignalElement::UNKNOWN;
}

boost::optional<uint8_t> CrosswalkTrafficLightEstimatorNode::getHighestConfidenceTrafficSignal(
  const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
  const TrafficLightIdMap & traffic_light_id_map) const
{
  boost::optional<uint8_t> ret{boost::none};

  double highest_confidence = 0.0;
  for (const auto & traffic_light : traffic_lights) {
    if (!traffic_light.isLineString()) {
      continue;
    }

    const int id = static_cast<lanelet::ConstLineString3d>(traffic_light).id();
    if (traffic_light_id_map.count(id) == 0) {
      continue;
    }

    const auto & elements = traffic_light_id_map.at(id).first.elements;
    if (elements.empty()) {
      continue;
    }

    const auto & color = elements.front().color;
    const auto & confidence = elements.front().confidence;
    if (confidence < highest_confidence) {
      continue;
    }

    highest_confidence = confidence;
    ret = color;
  }

  return ret;
}

boost::optional<uint8_t> CrosswalkTrafficLightEstimatorNode::getHighestConfidenceTrafficSignal(
  const lanelet::Id & id, const TrafficLightIdMap & traffic_light_id_map) const
{
  boost::optional<uint8_t> ret{boost::none};

  double highest_confidence = 0.0;
  if (traffic_light_id_map.count(id) == 0) {
    return ret;
  }

  for (const auto & element : traffic_light_id_map.at(id).first.elements) {
    if (element.confidence < highest_confidence) {
      continue;
    }

    highest_confidence = element.confidence;
    ret = element.color;
  }

  return ret;
}

void CrosswalkTrafficLightEstimatorNode::removeDuplicateIds(TrafficSignalArray & signal_array) const
{
  auto & signals = signal_array.traffic_light_groups;
  std::stable_sort(signals.begin(), signals.end(), [](const auto & s1, const auto & s2) {
    return s1.traffic_light_group_id < s2.traffic_light_group_id;
  });

  signals.erase(
    std::unique(
      signals.begin(), signals.end(),
      [](const auto & s1, const auto s2) {
        return s1.traffic_light_group_id == s2.traffic_light_group_id;
      }),
    signals.end());
}

}  // namespace autoware::crosswalk_traffic_light_estimator

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorNode)
