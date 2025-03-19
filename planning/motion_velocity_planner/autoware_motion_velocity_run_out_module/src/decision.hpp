// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef DECISION_HPP_
#define DECISION_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <rclcpp/duration.hpp>

#include <algorithm>
#include <cstdio>
#include <sstream>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief update a decision based on a priority stop > slowdown > nothing
inline void update_decision(std::optional<Decision> & decision_to_update, const Decision & decision)
{
  if (!decision.collision.has_value()) {
    return;
  }
  const auto not_set_yet = !decision_to_update;
  const auto same_decision_but_earlier =
    decision_to_update && (decision_to_update->type == decision.type &&
                           decision.collision->ego_time_interval.from <
                             decision_to_update->collision->ego_time_interval.from);
  const auto higher_priority_decision =
    decision_to_update && ((decision_to_update->type == slowdown && decision.type == stop) ||
                           (decision_to_update->type == nothing && decision.type != nothing));
  if (not_set_yet || same_decision_but_earlier || higher_priority_decision) {
    decision_to_update = decision;
  }
}
/// @brief return true if the given decision is for a detected collision
inline bool is_collision(const Decision & d)
{
  return d.collision.has_value() && d.collision->type == collision;
}
/// @brief calculate the consecutive time with collisions in the history
inline double calculate_consecutive_time_with_collision(
  const DecisionHistory & history, const rclcpp::Time & current_time)
{
  if (history.times.empty()) {
    return 0.0;
  }
  const auto earliest_not_collision_it =
    std::find_if(history.decisions.rbegin(), history.decisions.rend(), std::not_fn(is_collision));
  return (earliest_not_collision_it == history.decisions.rend())
           ? current_time.seconds() - history.times.front()
           : current_time.seconds() -
               history.times[std::distance(
                 earliest_not_collision_it, std::prev(history.decisions.rend()))];
}
/// @brief return true if the conditions to stop are met
inline bool condition_to_stop(
  const DecisionHistory & history, const CollisionType & current_collision_type,
  const rclcpp::Time & current_time, std::stringstream & explanation, const bool ego_is_stopped,
  const Parameters & params)
{
  // only stop after successively detecting collisions for some time
  const auto calc_consecutive_time_with_collision = [&]() {
    if (history.times.empty()) {
      return 0.0;
    }
    const auto earliest_not_collision_it =
      std::find_if(history.decisions.rbegin(), history.decisions.rend(), std::not_fn(is_collision));
    return (earliest_not_collision_it == history.decisions.rend())
             ? current_time.seconds() - history.times.front()
             : current_time.seconds() -
                 history.times[std::distance(
                   earliest_not_collision_it, std::prev(history.decisions.rend()))];
  };
  if (current_collision_type == collision) {
    const auto consecutive_time_with_collision = calc_consecutive_time_with_collision();
    if (consecutive_time_with_collision >= params.stop_on_time_buffer) {
      explanation << "stopping since finding collisions for " << consecutive_time_with_collision
                  << "s (" << params.stop_on_time_buffer << "s buffer)";
      return true;
    }
    explanation << "not stopping since only finding collisions for "
                << consecutive_time_with_collision << "s (" << params.stop_on_time_buffer
                << "s buffer)";
    return false;
  }
  if (
    params.keep_stop_until_object_is_gone && !history.decisions.empty() &&
    history.decisions.back().type == stop && ego_is_stopped &&
    current_collision_type != no_collision) {
    explanation << "keep stop until object is gone";
    return true;
  }
  // keep stopping for some time after the last detected collision
  if (!history.decisions.empty() && history.decisions.back().type == stop) {
    const auto most_recent_collision_it =
      std::find_if(history.decisions.rbegin(), history.decisions.rend(), is_collision);
    if (most_recent_collision_it == history.decisions.rend()) {
      explanation << "removing stop since no collision in history";
      return false;
    }
    // -1 because the reverse iterator has an offset compared to base iterator
    const auto i = std::distance(history.decisions.begin(), most_recent_collision_it.base()) - 1;
    // TODO(Maxime): may be off by one time step
    const auto time_since_last_collision = current_time.seconds() - history.times[i];
    if (time_since_last_collision < params.stop_off_time_buffer) {
      explanation << "keeping stop since last collision found " << time_since_last_collision
                  << "s ago (" << params.stop_off_time_buffer << "s buffer)"
                  << most_recent_collision_it->type;
      return true;
    }
    explanation << "removing stop since last collision found " << time_since_last_collision
                << "s ago (" << params.stop_off_time_buffer << "s buffer)";
    return false;
  }
  if (current_collision_type == pass_first_collision) {
    explanation << "ignoring passing collision";
  }
  return false;
}
/// @brief return true if the conditions to slowdown are met
inline bool condition_to_slowdown(
  const DecisionHistory & history, const CollisionType & current_collision_type,
  const rclcpp::Time & current_time, std::stringstream & explanation, const Parameters & params)
{
  // only slowdown after successively detecting collisions for some time
  if (current_collision_type == collision) {
    const auto consecutive_time_with_collision =
      calculate_consecutive_time_with_collision(history, current_time);
    if (consecutive_time_with_collision >= params.preventive_slowdown_on_time_buffer) {
      explanation << "preventive slowdown since finding collisions for "
                  << consecutive_time_with_collision << "s ("
                  << params.preventive_slowdown_on_time_buffer << "s buffer)";
      return true;
    }
    explanation << "not slowing down since only finding collisions for "
                << consecutive_time_with_collision << "s (" << params.stop_on_time_buffer
                << "s buffer)";
    return false;
  }
  // keep decision for some time after the last detected collision
  if (!history.decisions.empty() && history.decisions.back().type == slowdown) {
    const auto most_recent_collision_it =
      std::find_if(history.decisions.rbegin(), history.decisions.rend(), is_collision);
    if (most_recent_collision_it == history.decisions.rend()) {
      explanation << "removing stop since no collision in history";
      return false;
    }
    // -1 because the reverse iterator has an offset compared to base iterator
    const auto i = std::distance(history.decisions.begin(), most_recent_collision_it.base()) - 1;
    // TODO(Maxime): may be off by one time step
    const auto time_since_last_collision = current_time.seconds() - history.times[i];
    if (time_since_last_collision < params.stop_off_time_buffer) {
      explanation << "keeping slowdown since last collision found " << time_since_last_collision
                  << "s ago (" << params.stop_off_time_buffer << "s buffer)"
                  << most_recent_collision_it->type;
      return true;
    }
    explanation << "removing slowdown since last collision found " << time_since_last_collision
                << "s ago (" << params.stop_off_time_buffer << "s buffer)";
    return false;
  }
  return false;
}
/// @brief calculate the decision type corresponding to a collision type and a decision history
inline DecisionType calculate_decision_type(
  const CollisionType & collision_type, const DecisionHistory & history, const rclcpp::Time & now,
  std::stringstream & explanation, const double time_to_stop, const Parameters & params)
{
  const auto ego_is_stopped = time_to_stop < 1e-3;
  if (condition_to_stop(history, collision_type, now, explanation, ego_is_stopped, params)) {
    return stop;
  }
  if (condition_to_slowdown(history, collision_type, now, explanation, params)) {
    return slowdown;
  }
  return nothing;
}
/// @brief calculate current decisions for the objects and update the decision tracker accordingly
inline void calculate_decisions(
  ObjectDecisionsTracker & decisions_tracker, const std::vector<Object> & objects,
  const rclcpp::Time & now, const double time_to_stop, const Parameters & params)
{
  for (const auto & object : objects) {
    std::optional<Decision> object_decision;
    auto & decision_history = decisions_tracker.history_per_object[object.uuid];
    std::stringstream explanation;
    for (const auto & collision : object.collisions) {
      Decision d(
        collision, calculate_decision_type(
                     collision.type, decision_history, now, explanation, time_to_stop, params));
      update_decision(object_decision, d);
    }
    if (object_decision) {
      object_decision->explanation = explanation.str();
      decision_history.add_decision(now.seconds(), *object_decision);
    }
  }
  decisions_tracker.remove_outdated(now, params.max_history_duration);
  decisions_tracker.update_objects_without_decisions(now);
}
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // DECISION_HPP_
