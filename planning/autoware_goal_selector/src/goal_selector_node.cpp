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

#include "goal_selector_node.hpp"

#include "debug_marker.hpp"

#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::goal_selector
{
namespace
{

bool is_goal_vacant(
  const geometry_msgs::msg::Pose & goal,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double margin)
{
  const auto goal_footprint = autoware_utils::transform_vector(
    vehicle_info.createFootprint(margin), autoware_utils::pose2transform(goal));

  for (const auto & object : objects.objects) {
    if (boost::geometry::intersects(goal_footprint, autoware_utils::to_polygon2d(object))) {
      return false;
    }
  }
  return true;
}

}  // namespace

GoalSelectorNode::GoalSelectorNode(const rclcpp::NodeOptions & options)
: Node("goal_selector", options)
{
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  goal_footprint_margin_ = declare_parameter<double>("goal_footprint_margin");
  traffic_signal_timeout_ = declare_parameter<double>("traffic_signal_timeout");
  trigger_height_tolerance_ = declare_parameter<double>("trigger_height_tolerance", 2.0);
  load_triggers();
  validate_fallback_goals();

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  sub_route_state_ = create_subscription<RouteState>(
    "~/input/route_state", durable_qos,
    [this](const RouteState::ConstSharedPtr msg) { on_route_state(msg); });
  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", durable_qos, [this](const LaneletRoute::ConstSharedPtr msg) { route_ = msg; });
  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/objects", 1, [this](const PredictedObjects::ConstSharedPtr msg) { on_objects(msg); });
  sub_traffic_signals_ = create_subscription<TrafficLightGroupArray>(
    "~/input/traffic_signals", 1,
    [this](const TrafficLightGroupArray::ConstSharedPtr msg) { on_traffic_signals(msg); });
  pub_debug_marker_ = create_publisher<MarkerArray>("~/debug/marker", 1);

  const auto clear_route_service =
    declare_parameter<std::string>("clear_route_service", "/planning/clear_route");
  const auto set_lanelet_route_service =
    declare_parameter<std::string>("set_lanelet_route_service", "/planning/set_lanelet_route");
  clear_route_client_ = create_client<ClearRoute>(clear_route_service);
  set_lanelet_route_client_ = create_client<SetLaneletRoute>(set_lanelet_route_service);
}

geometry_msgs::msg::Pose GoalSelectorNode::declare_pose(const std::string & name)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = declare_parameter<double>(name + ".x");
  pose.position.y = declare_parameter<double>(name + ".y");
  pose.position.z = declare_parameter<double>(name + ".z");
  pose.orientation =
    autoware_utils::create_quaternion_from_yaw(declare_parameter<double>(name + ".yaw"));
  return pose;
}

void GoalSelectorNode::load_triggers()
{
  std::map<std::string, GoalCandidate> goals;
  for (const auto & name : declare_parameter<std::vector<std::string>>("goals")) {
    GoalCandidate candidate;
    candidate.name = name;
    candidate.pose = declare_pose(name);
    candidate.lane_ids =
      declare_parameter<std::vector<int64_t>>(name + ".lane_ids", std::vector<int64_t>{});
    if (candidate.lane_ids.empty()) {
      RCLCPP_WARN(
        get_logger(), "goal '%s' has no lane_ids; it cannot be sent as a route", name.c_str());
    }
    // A negative value means that the goal has no traffic signal assigned.
    const auto group_id = declare_parameter<int64_t>(name + ".traffic_light_group_id", -1);
    if (group_id >= 0) {
      candidate.traffic_light_group_id = group_id;
    }
    goals.emplace(name, candidate);
  }

  for (const auto & name : declare_parameter<std::vector<std::string>>("triggers")) {
    Trigger trigger;
    trigger.name = name;
    trigger.x = declare_parameter<double>(name + ".x");
    trigger.y = declare_parameter<double>(name + ".y");
    trigger.radius = declare_parameter<double>(name + ".radius");

    const auto candidates = declare_parameter<std::vector<std::string>>(name + ".goal_candidates");
    for (const auto & candidate : candidates) {
      const auto goal = goals.find(candidate);
      if (goal == goals.end()) {
        throw std::runtime_error("Unknown goal '" + candidate + "' in trigger '" + name + "'.");
      }
      trigger.goal_candidates.push_back(goal->second);
    }
    if (!trigger.goal_candidates.empty()) {
      trigger.z = trigger.goal_candidates.front().pose.position.z;
    }

    const auto fallback_name = declare_parameter<std::string>(name + ".fallback_goal", "");
    if (!fallback_name.empty()) {
      const auto goal = goals.find(fallback_name);
      if (goal == goals.end()) {
        throw std::runtime_error(
          "Unknown fallback goal '" + fallback_name + "' in trigger '" + name + "'.");
      }
      trigger.fallback_goal = goal->second;
    }
    triggers_.push_back(std::move(trigger));
  }
}

void GoalSelectorNode::validate_fallback_goals() const
{
  // The fallback goal has to lie inside the area of another trigger. Arriving there then selects
  // the goal candidates of that trigger.
  for (const auto & trigger : triggers_) {
    if (!trigger.fallback_goal) {
      continue;
    }
    const auto * arrival_trigger = find_trigger(trigger.fallback_goal->pose.position);
    if (!arrival_trigger) {
      RCLCPP_WARN(
        get_logger(),
        "fallback goal '%s' of trigger '%s' is outside every trigger area, so no goal is selected "
        "after arriving there",
        trigger.fallback_goal->name.c_str(), trigger.name.c_str());
    } else if (arrival_trigger == &trigger) {
      RCLCPP_WARN(
        get_logger(),
        "fallback goal '%s' of trigger '%s' is inside the area of the same trigger, so the "
        "fallback would be repeated",
        trigger.fallback_goal->name.c_str(), trigger.name.c_str());
    }
  }
}

void GoalSelectorNode::on_route_state(const RouteState::ConstSharedPtr msg)
{
  is_arrived_ = msg->state == RouteState::ARRIVED;
  if (!is_arrived_) {
    is_next_goal_set_ = false;
  }
  set_next_goal();
}

void GoalSelectorNode::on_objects(const PredictedObjects::ConstSharedPtr msg)
{
  objects_ = msg;
  if (pub_debug_marker_->get_subscription_count() > 0) {
    publish_debug_marker();
  }

  // Retry on every objects update while all goal candidates are blocked.
  set_next_goal();
}

void GoalSelectorNode::on_traffic_signals(const TrafficLightGroupArray::ConstSharedPtr msg)
{
  traffic_signals_ = msg;
  traffic_signals_received_time_ = now();

  // Retry while all goal candidates are blocked, since the signal may have changed.
  set_next_goal();
}

bool GoalSelectorNode::is_signal_red(const int64_t traffic_light_group_id) const
{
  using autoware_perception_msgs::msg::TrafficLightElement;

  if (!traffic_signals_) {
    return false;
  }
  // Stale information is treated as not red, so that stopping the publisher turns the signal off.
  if ((now() - traffic_signals_received_time_).seconds() > traffic_signal_timeout_) {
    return false;
  }

  for (const auto & group : traffic_signals_->traffic_light_groups) {
    if (group.traffic_light_group_id != traffic_light_group_id) {
      continue;
    }
    for (const auto & element : group.elements) {
      if (
        element.color == TrafficLightElement::RED &&
        element.status == TrafficLightElement::SOLID_ON) {
        return true;
      }
    }
  }
  return false;
}

GoalState GoalSelectorNode::judge_goal(const GoalCandidate & candidate) const
{
  if (!is_goal_vacant(candidate.pose, *objects_, vehicle_info_, goal_footprint_margin_)) {
    return GoalState::OCCUPIED;
  }
  if (candidate.traffic_light_group_id && is_signal_red(*candidate.traffic_light_group_id)) {
    return GoalState::RED_SIGNAL;
  }
  return GoalState::AVAILABLE;
}

void GoalSelectorNode::publish_debug_marker()
{
  std::map<std::string, GoalState> states;
  for (const auto & trigger : triggers_) {
    for (const auto & candidate : trigger.goal_candidates) {
      states[candidate.name] = judge_goal(candidate);
    }
    if (trigger.fallback_goal) {
      states[trigger.fallback_goal->name] = judge_goal(*trigger.fallback_goal);
    }
  }
  pub_debug_marker_->publish(
    create_debug_marker_array(triggers_, states, vehicle_info_, goal_footprint_margin_, now()));
}

void GoalSelectorNode::set_next_goal()
{
  if (!is_arrived_ || is_next_goal_set_ || route_request_in_flight_ || !route_ || !objects_) {
    return;
  }

  const auto * trigger = find_trigger(route_->goal_pose.position);
  if (!trigger) {
    return;
  }

  const auto * goal = select_goal(*trigger);
  if (!goal) {
    goal = select_fallback_goal(*trigger);
    if (goal) {
      RCLCPP_INFO(
        get_logger(), "All goal candidates of trigger '%s' are blocked. Go to fallback goal '%s'.",
        trigger->name.c_str(), goal->name.c_str());
    }
  }
  if (!goal) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "All goal candidates of trigger '%s' are blocked%s.",
      trigger->name.c_str(), trigger->fallback_goal ? ", and so is the fallback goal" : "");
    return;
  }

  if (goal->lane_ids.empty()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "goal '%s' has no lane_ids, so no route was sent",
      goal->name.c_str());
    return;
  }

  send_route(*goal);
}

const Trigger * GoalSelectorNode::find_trigger(const geometry_msgs::msg::Point & position) const
{
  for (const auto & trigger : triggers_) {
    const bool is_inside_area =
      std::hypot(position.x - trigger.x, position.y - trigger.y) <= trigger.radius;
    // A negative tolerance ignores the elevation. Otherwise a trigger on another floor that
    // shares the same x and y is not matched.
    const bool is_same_floor = trigger_height_tolerance_ < 0.0 ||
                               std::abs(position.z - trigger.z) <= trigger_height_tolerance_;
    if (is_inside_area && is_same_floor) {
      return &trigger;
    }
  }
  return nullptr;
}

const GoalCandidate * GoalSelectorNode::select_goal(const Trigger & trigger) const
{
  for (const auto & candidate : trigger.goal_candidates) {
    if (judge_goal(candidate) == GoalState::AVAILABLE) {
      return &candidate;
    }
  }
  return nullptr;
}

const GoalCandidate * GoalSelectorNode::select_fallback_goal(const Trigger & trigger) const
{
  if (trigger.fallback_goal && judge_goal(*trigger.fallback_goal) == GoalState::AVAILABLE) {
    return &*trigger.fallback_goal;
  }
  return nullptr;
}

void GoalSelectorNode::send_route(const GoalCandidate & goal)
{
  if (!clear_route_client_->service_is_ready() || !set_lanelet_route_client_->service_is_ready()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "route services are not ready");
    return;
  }

  pending_goal_ = goal;
  route_request_in_flight_ = true;
  clear_route_client_->async_send_request(
    std::make_shared<ClearRoute::Request>(),
    [this](rclcpp::Client<ClearRoute>::SharedFuture future) { on_route_cleared(future); });
}

void GoalSelectorNode::on_route_cleared(rclcpp::Client<ClearRoute>::SharedFuture future)
{
  const auto cleared = future.get();
  if (!cleared || !cleared->status.success) {
    route_request_in_flight_ = false;
    RCLCPP_ERROR(get_logger(), "failed to clear the current route");
    return;
  }

  auto request = std::make_shared<SetLaneletRoute::Request>();
  request->header.stamp = now();
  request->header.frame_id = "map";
  request->goal_pose = pending_goal_.pose;
  request->allow_modification = false;
  std::random_device seed;
  std::mt19937 generator(seed());
  std::uniform_int_distribution<uint16_t> byte_value(0, 255);
  for (auto & byte : request->uuid.uuid) {
    byte = static_cast<uint8_t>(byte_value(generator));
  }
  for (const auto lane_id : pending_goal_.lane_ids) {
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = lane_id;
    primitive.primitive_type = "lane";
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive = primitive;
    segment.primitives.push_back(primitive);
    request->segments.push_back(segment);
  }

  set_lanelet_route_client_->async_send_request(
    request, [this](rclcpp::Client<SetLaneletRoute>::SharedFuture set_future) {
      const auto set = set_future.get();
      route_request_in_flight_ = false;
      if (!set || !set->status.success) {
        RCLCPP_ERROR(
          get_logger(), "failed to set the lanelet route for '%s'", pending_goal_.name.c_str());
        return;
      }
      is_next_goal_set_ = true;
      RCLCPP_INFO(
        get_logger(), "Set the lanelet route '%s' (%zu lanes).", pending_goal_.name.c_str(),
        pending_goal_.lane_ids.size());
    });
}

}  // namespace autoware::goal_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::goal_selector::GoalSelectorNode)
