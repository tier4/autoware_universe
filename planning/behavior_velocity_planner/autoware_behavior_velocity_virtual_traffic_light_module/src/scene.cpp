// Copyright 2021 Tier IV, Inc.
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

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using virtual_traffic_light::calcCenter;
using virtual_traffic_light::calcHeadPose;
using virtual_traffic_light::createKeyValue;
using virtual_traffic_light::toAutowarePoints;

VirtualTrafficLightModule::VirtualTrafficLightModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::VirtualTrafficLight & reg_elem, lanelet::ConstLanelet lane,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  lane_(lane),
  planner_param_(planner_param),
  base_logger_(logger)
{
  // Get map data
  const auto instrument = reg_elem.getVirtualTrafficLight();
  const auto instrument_bottom_line = toAutowarePoints(instrument);

  // Information from instrument
  {
    map_data_.instrument_type = *instrument.attribute("type").as<std::string>();
    map_data_.instrument_id = std::to_string(instrument.id());
    map_data_.instrument_center = calcCenter(instrument_bottom_line);
  }

  // Information from regulatory_element
  {
    map_data_.stop_line = reg_elem.getStopLine();
    map_data_.start_line = reg_elem.getStartLine();
    map_data_.end_lines = reg_elem.getEndLines();

    // Set stop line ID for logging (safe to use in log messages)
    map_data_.stop_line_id_for_log =
      map_data_.stop_line ? std::to_string(map_data_.stop_line->id()) : "none";
  }

  // Custom tags
  {
    // Map attributes
    for (const auto & attr : instrument.attributes()) {
      const auto & key = attr.first;
      const auto & value = *attr.second.as<std::string>();

      // Ignore mandatory tags
      if (key == "type") {
        continue;
      }

      map_data_.custom_tags.emplace_back(createKeyValue(key, value));
    }

    // Lane ID
    map_data_.custom_tags.emplace_back(createKeyValue("lane_id", std::to_string(lane_.id())));

    // Turn direction
    map_data_.custom_tags.emplace_back(
      createKeyValue("turn_direction", lane_.attributeOr("turn_direction", "straight")));
  }

  // Set command
  command_.type = map_data_.instrument_type;
  command_.id = map_data_.instrument_id;
  command_.custom_tags = map_data_.custom_tags;

  // Set up base logger with instrument information
  base_logger_ =
    base_logger_.get_child((map_data_.instrument_type + "_" + map_data_.instrument_id).c_str());
  updateLoggerWithState();
}

void VirtualTrafficLightModule::setModuleState(
  const State new_state, const std::optional<int64_t> end_line_id)
{
  // +----------+
  // |   NONE   |
  // +----------+
  //      |
  //      | vehicle is before start line
  //      v
  // +----------+
  // |REQUESTING|
  // +----------+
  //      |
  //      | vehicle has passed stop line
  //      v
  // +----------+
  // | PASSING  |
  // +----------+
  //      |
  //      +-------------------------------------------+
  //      | - vehicle is near end line                | vehicle has passed end line
  //      | - finalization isn't completed            |
  //      v                                           v
  // +----------+                                 +-----------+
  // |FINALIZING|                                 | FINALIZED |
  // +----------+                                 +-----------+

  if (state_ == new_state) {
    return;
  }

  // NONE -> REQUESTING
  if (state_ == State::NONE && new_state == State::REQUESTING) {
    logInfo(
      "State transition: NONE -> REQUESTING as vehicle is before start line (ID: %ld)",
      map_data_.start_line.id());
  }

  // REQUESTING -> PASSING
  if (state_ == State::REQUESTING && new_state == State::PASSING) {
    logInfo(
      "State transition: REQUESTING -> PASSING as vehicle has passed stop line (ID: %s)",
      map_data_.stop_line_id_for_log.c_str());
  }

  // PASSING -> FINALIZING
  if (state_ == State::PASSING && new_state == State::FINALIZING) {
    if (end_line_id.has_value()) {
      logInfo(
        "State transition: PASSING -> FINALIZING as vehicle is near end line (ID: %ld)",
        end_line_id.value());
    } else {
      logInfo("State transition: PASSING -> FINALIZING as vehicle is near end line");
    }
  }

  // PASSING -> FINALIZED
  if (state_ == State::PASSING && new_state == State::FINALIZED) {
    if (end_line_id.has_value()) {
      logInfo(
        "State transition: PASSING -> FINALIZED as vehicle has passed end line (ID: %ld)",
        end_line_id.value());
    } else {
      logInfo("State transition: PASSING -> FINALIZED as vehicle has passed end line");
    }
  }

  state_ = new_state;
  updateLoggerWithState();
}

bool VirtualTrafficLightModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  // Copy data
  module_data_ = {};
  module_data_.head_pose = calcHeadPose(
    planner_data.current_odometry->pose, planner_data.vehicle_info_.max_longitudinal_offset_m);
  module_data_.path = path;

  // Initialize
  setInfrastructureCommand({});

  // Calculate path index of end line
  // NOTE: In order to deal with u-turn or self-crossing path, only start/stop lines before the end
  // line are used when whether the ego is before/after the start/stop/end lines is calculated.
  const auto opt_end_line_result = getPathIndexOfFirstEndLine(planner_data);
  if (!opt_end_line_result) {
    return true;
  }
  const auto & [end_line_s, end_line_id] = *opt_end_line_result;

  // Do nothing if vehicle is before start line
  if (isBeforeStartLine(end_line_s, planner_data)) {
    logDebug("before start_line");
    setModuleState<State::NONE>();
    updateInfrastructureCommand();
    return true;
  }

  // Do nothing if state is already FINALIZED
  if (state_ == State::FINALIZED) {
    logInfoThrottle(5000, "state is already FINALIZED");
    updateInfrastructureCommand();
    return true;
  }

  // Do nothing if vehicle is after any end line
  if (isAfterAnyEndLine(end_line_s, planner_data)) {
    setModuleState<State::FINALIZED>(end_line_id);
    updateInfrastructureCommand();
    return true;
  }

  // Don't need to stop if there is no stop_line
  if (!map_data_.stop_line) {
    logWarnThrottle(5000, "no stop line is found, do nothing in this module");
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if no message received
  if (!virtual_traffic_light_state_) {
    setModuleState<State::REQUESTING>();
    logInfoThrottle(
      5000, "no message received for instrument (ID: %ld), stop at stop line (ID: %s)",
      map_data_.instrument_id, map_data_.stop_line_id_for_log.c_str());
    insertStopVelocityAtStopLine(end_line_s, planner_data);
    updateInfrastructureCommand();
    _path->points = module_data_.path->restore();
    return true;
  }

  // Stop at stop_line if no right is given
  if (!hasRightOfWay(*virtual_traffic_light_state_)) {
    setModuleState<State::REQUESTING>();
    logInfoThrottle(
      5000, "no right of way for instrument (ID: %ld) is given, stop at stop line (ID: %s)",
      map_data_.instrument_id, map_data_.stop_line_id_for_log.c_str());
    insertStopVelocityAtStopLine(end_line_s, planner_data);
    updateInfrastructureCommand();
    _path->points = module_data_.path->restore();
    return true;
  }

  // Stop at stop_line if state is timeout before stop_line
  if (isBeforeStopLine(end_line_s, planner_data)) {
    setModuleState<State::REQUESTING>();
    if (isStateTimeout(*virtual_traffic_light_state_)) {
      logWarnThrottle(
        5000, "virtual traffic light state is timeout, stop at stop line (ID: %s)",
        map_data_.stop_line_id_for_log.c_str());
      insertStopVelocityAtStopLine(end_line_s, planner_data);
    }

    updateInfrastructureCommand();
    _path->points = module_data_.path->restore();
    return true;
  }

  // After stop_line
  if (state_ == State::REQUESTING) {
    setModuleState<State::PASSING>();
  }

  // Check timeout after stop_line if the option is on
  if (
    planner_param_.check_timeout_after_stop_line && isStateTimeout(*virtual_traffic_light_state_)) {
    setModuleState<State::PASSING>();
    logWarnThrottle(
      5000,
      "virtual traffic light state is timeout after stop line, insert stop velocity at stop line "
      "(ID: %s)",
      map_data_.stop_line_id_for_log.c_str());
    insertStopVelocityAtStopLine(end_line_s, planner_data);
    updateInfrastructureCommand();
    _path->points = module_data_.path->restore();
    return true;
  }

  // Stop at end_line if finalization isn't completed
  if (!virtual_traffic_light_state_->is_finalized) {
    logInfoThrottle(
      5000, "finalization isn't completed, insert stop velocity at end line (ID: %s)",
      map_data_.stop_line_id_for_log.c_str());
    insertStopVelocityAtEndLine(end_line_s, planner_data);

    if (isNearAnyEndLine(end_line_s, planner_data) && planner_data.isVehicleStopped()) {
      setModuleState<State::FINALIZING>(end_line_id);
    }
  }

  updateInfrastructureCommand();
  return true;
}

void VirtualTrafficLightModule::updateInfrastructureCommand()
{
  command_.stamp = clock_->now();
  command_.state = static_cast<uint8_t>(state_);
  setInfrastructureCommand(command_);
}

std::optional<std::pair<double, int64_t>> VirtualTrafficLightModule::getPathIndexOfFirstEndLine(
  const PlannerData & planner_data) const
{
  std::optional<std::pair<double, int64_t>> min_result;

  const auto connected_lane_ids =
    planning_utils::collectConnectedLaneIds(lane_id_, planner_data.route_handler_);

  for (const auto & end_line : map_data_.end_lines) {
    const auto collision = experimental::trajectory::crossed_with_constraint(
      *module_data_.path, end_line, [&connected_lane_ids](const PathPointWithLaneId & point) {
        return std::any_of(
          point.lane_ids.begin(), point.lane_ids.end(),
          [&connected_lane_ids](const lanelet::Id lane_id) {
            return std::find(connected_lane_ids.begin(), connected_lane_ids.end(), lane_id) !=
                   connected_lane_ids.end();
          });
      });

    if (collision.empty()) {
      continue;
    }
    const auto collision_s = collision.front();

    if (!min_result || collision_s < min_result->first) {
      min_result = std::make_pair(collision_s, end_line.id());
    }
  }

  return min_result;
}

bool VirtualTrafficLightModule::isBeforeStartLine(
  const double end_line_s, const PlannerData & planner_data) const
{
  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  const auto arc_length =
    calcArcLengthFromCollision(end_line_s, map_data_.start_line, planner_data);
  return arc_length && *arc_length > 0;
}

bool VirtualTrafficLightModule::isBeforeStopLine(
  const double end_line_s, const PlannerData & planner_data) const
{
  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  const auto arc_length =
    calcArcLengthFromCollision(end_line_s, *map_data_.stop_line, planner_data);
  return arc_length && *arc_length > -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isAfterAnyEndLine(
  const double end_line_s, const PlannerData & planner_data) const
{
  // Assume stop line is before end lines
  if (isBeforeStopLine(end_line_s, planner_data)) {
    return false;
  }

  // If the goal is set before the end line, collision will not be detected.
  // Therefore if there is no collision, the ego vehicle is assumed to be before the end line.
  const auto arc_length = calcArcLengthFromCollision(end_line_s, map_data_.end_lines, planner_data);
  return arc_length && *arc_length < -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isNearAnyEndLine(
  const double end_line_s, const PlannerData & planner_data) const
{
  const auto arc_length = calcArcLengthFromCollision(end_line_s, map_data_.end_lines, planner_data);
  return arc_length && std::abs(*arc_length) < planner_param_.near_line_distance;
}

bool VirtualTrafficLightModule::isStateTimeout(
  const tier4_v2x_msgs::msg::VirtualTrafficLightState & state) const
{
  const auto delay = (clock_->now() - rclcpp::Time(state.stamp)).seconds();
  if (delay > planner_param_.max_delay_sec) {
    logDebug("delay=%f, max_delay=%f", delay, planner_param_.max_delay_sec);
    return true;
  }

  return false;
}

bool VirtualTrafficLightModule::hasRightOfWay(
  const tier4_v2x_msgs::msg::VirtualTrafficLightState & state) const
{
  return state.approval;
}

void VirtualTrafficLightModule::insertStopVelocityAtStopLine(
  const double end_line_s, const PlannerData & planner_data)
{
  const auto collision =
    findLastCollisionBeforeEndLine(*module_data_.path, *map_data_.stop_line, end_line_s);
  const auto offset = -planner_data.vehicle_info_.max_longitudinal_offset_m;

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision) {
    module_data_.path->longitudinal_velocity_mps() = 0.0;
    stop_pose = planner_data.current_odometry->pose;
  } else {
    const auto ego_s = experimental::trajectory::find_nearest_index(
      *module_data_.path, planner_data.current_odometry->pose.position);

    const auto stop_distance = *collision - ego_s + offset;
    const auto is_stopped = planner_data.isVehicleStopped();

    if (stop_distance < planner_param_.hold_stop_margin_distance && is_stopped) {
      module_data_.path->longitudinal_velocity_mps()
        .range(ego_s, module_data_.path->length())
        .set(0.0);
    } else {
      module_data_.path->longitudinal_velocity_mps()
        .range(std::max(0., *collision + offset), module_data_.path->length())
        .set(0.0);
    }

    // for virtual wall
    stop_pose =
      module_data_.path->compute(std::clamp(*collision + offset, 0.0, module_data_.path->length()))
        .point.pose;
  }

  // Set StopReason
  planning_factor_interface_->add(
    module_data_.path->restore(), planner_data.current_odometry->pose, stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  // Set data for visualization
  module_data_.stop_head_pose_at_stop_line =
    calcHeadPose(stop_pose, planner_data.vehicle_info_.max_longitudinal_offset_m);
}

void VirtualTrafficLightModule::insertStopVelocityAtEndLine(
  const double end_line_s, const PlannerData & planner_data)
{
  const auto collision =
    findLastCollisionBeforeEndLine(*module_data_.path, map_data_.end_lines, end_line_s);

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision) {
    // No enough length
    if (isBeforeStopLine(end_line_s, planner_data)) {
      return;
    }

    module_data_.path->longitudinal_velocity_mps() = 0.0;
    stop_pose = planner_data.current_odometry->pose;
  } else {
    const auto offset = -planner_data.vehicle_info_.max_longitudinal_offset_m;
    module_data_.path->longitudinal_velocity_mps()
      .range(std::max(0., *collision + offset), module_data_.path->length())
      .set(0.0);
    stop_pose =
      module_data_.path->compute(std::clamp(*collision + offset, 0.0, module_data_.path->length()))
        .point.pose;
  }

  // Set StopReason
  planning_factor_interface_->add(
    module_data_.path->restore(), planner_data.current_odometry->pose, stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  // Set data for visualization
  module_data_.stop_head_pose_at_end_line =
    calcHeadPose(stop_pose, planner_data.vehicle_info_.max_longitudinal_offset_m);
}

std::optional<tier4_v2x_msgs::msg::InfrastructureCommand>
VirtualTrafficLightModule::getInfrastructureCommand() const
{
  return infrastructure_command_;
}

void VirtualTrafficLightModule::setInfrastructureCommand(
  const std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> & command)
{
  infrastructure_command_ = command;
}

void VirtualTrafficLightModule::setCorrespondingVirtualTrafficLightState(
  const tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr
    virtual_traffic_light_states)
{
  if (!virtual_traffic_light_states) {
    return;
  }

  for (const auto & state : virtual_traffic_light_states->states) {
    if (state.id != map_data_.instrument_id) {
      continue;
    }

    const bool has_previous_state = virtual_traffic_light_state_.has_value();

    const bool prev_has_right_of_way =
      has_previous_state ? hasRightOfWay(*virtual_traffic_light_state_) : false;
    const bool has_right_of_way = hasRightOfWay(state);
    if (!prev_has_right_of_way && has_right_of_way) {
      logInfo(
        "received message for instrument (ID: %s) is updated : right of way is given, approval: "
        "true, finalized: %s, stamp: %ld.%09ld",
        state.id.c_str(), state.is_finalized ? "true" : "false", state.stamp.sec,
        state.stamp.nanosec);
    }

    const bool prev_finalized =
      has_previous_state ? virtual_traffic_light_state_->is_finalized : false;
    const bool current_finalized = state.is_finalized;
    if (!prev_finalized && current_finalized) {
      logInfo(
        "received message for instrument (ID: %s) is updated : finalization is completed, "
        "approval: %s, finalized: true, stamp: %ld.%09ld",
        state.id.c_str(), state.approval ? "true" : "false", state.stamp.sec, state.stamp.nanosec);
    }

    virtual_traffic_light_state_ = state;
    return;
  }
}

std::string VirtualTrafficLightModule::stateToString(State state) const
{
  switch (state) {
    case State::NONE:
      return "NONE";
    case State::REQUESTING:
      return "REQUESTING";
    case State::PASSING:
      return "PASSING";
    case State::FINALIZING:
      return "FINALIZING";
    case State::FINALIZED:
      return "FINALIZED";
    default:
      return "UNKNOWN";
  }
}

void VirtualTrafficLightModule::updateLoggerWithState()
{
  const std::string state_name = stateToString(state_);
  logger_ = base_logger_.get_child(("state: " + state_name).c_str());
}
}  // namespace autoware::behavior_velocity_planner
