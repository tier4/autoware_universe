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

#include <boost/geometry.hpp>

#include <cmath>
#include <map>
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
  load_triggers();

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  sub_route_state_ = create_subscription<RouteState>(
    "~/input/route_state", durable_qos,
    [this](const RouteState::ConstSharedPtr msg) { on_route_state(msg); });
  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", durable_qos, [this](const LaneletRoute::ConstSharedPtr msg) { route_ = msg; });
  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/objects", 1, [this](const PredictedObjects::ConstSharedPtr msg) { on_objects(msg); });
  pub_goal_ = create_publisher<PoseStamped>("~/output/goal", 1);
  pub_debug_marker_ = create_publisher<MarkerArray>("~/debug/marker", 1);
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
  std::map<std::string, geometry_msgs::msg::Pose> goals;
  for (const auto & name : declare_parameter<std::vector<std::string>>("goals")) {
    goals.emplace(name, declare_pose(name));
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
      trigger.goal_candidates.push_back(GoalCandidate{candidate, goal->second});
    }
    triggers_.push_back(std::move(trigger));
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

void GoalSelectorNode::publish_debug_marker()
{
  std::map<std::string, bool> vacancies;
  for (const auto & trigger : triggers_) {
    for (const auto & candidate : trigger.goal_candidates) {
      vacancies[candidate.name] =
        is_goal_vacant(candidate.pose, *objects_, vehicle_info_, goal_footprint_margin_);
    }
  }
  pub_debug_marker_->publish(
    create_debug_marker_array(triggers_, vacancies, vehicle_info_, goal_footprint_margin_, now()));
}

void GoalSelectorNode::set_next_goal()
{
  if (!is_arrived_ || is_next_goal_set_ || !route_ || !objects_) {
    return;
  }

  const auto * trigger = find_trigger(route_->goal_pose.position);
  if (!trigger) {
    return;
  }

  const auto * goal = select_goal(*trigger);
  if (!goal) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "All goal candidates of trigger '%s' are blocked.",
      trigger->name.c_str());
    return;
  }

  publish_goal(goal->pose);
  RCLCPP_INFO(
    get_logger(), "Set the next goal '%s' for trigger '%s'.", goal->name.c_str(),
    trigger->name.c_str());
  is_next_goal_set_ = true;
}

const Trigger * GoalSelectorNode::find_trigger(const geometry_msgs::msg::Point & position) const
{
  for (const auto & trigger : triggers_) {
    if (std::hypot(position.x - trigger.x, position.y - trigger.y) <= trigger.radius) {
      return &trigger;
    }
  }
  return nullptr;
}

const GoalCandidate * GoalSelectorNode::select_goal(const Trigger & trigger) const
{
  for (const auto & candidate : trigger.goal_candidates) {
    if (is_goal_vacant(candidate.pose, *objects_, vehicle_info_, goal_footprint_margin_)) {
      return &candidate;
    }
  }
  return nullptr;
}

void GoalSelectorNode::publish_goal(const geometry_msgs::msg::Pose & goal)
{
  PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = route_->header.frame_id;
  msg.pose = goal;
  pub_goal_->publish(msg);
}

}  // namespace autoware::goal_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::goal_selector::GoalSelectorNode)
