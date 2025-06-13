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

#include "utils.hpp"

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
using virtual_traffic_light::findLastCollisionBeforeEndLine;
using virtual_traffic_light::insertStopVelocityFromStart;
using virtual_traffic_light::SegmentIndexWithOffset;
using virtual_traffic_light::SegmentIndexWithPoint;
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
  reg_elem_(reg_elem),
  lane_(lane),
  planner_param_(planner_param),
  base_logger_(logger)
{
  // Get map data
  const auto instrument = reg_elem_.getVirtualTrafficLight();
  const auto instrument_bottom_line = toAutowarePoints(instrument);

  // Information from instrument
  {
    map_data_.instrument_type = *instrument.attribute("type").as<std::string>();
    map_data_.instrument_id = std::to_string(instrument.id());
    map_data_.instrument_center = calcCenter(instrument_bottom_line);
  }

  // Information from regulatory_element
  {
    map_data_.stop_line = toAutowarePoints(reg_elem_.getStopLine());
    map_data_.start_line = toAutowarePoints(reg_elem_.getStartLine());
    map_data_.end_lines = toAutowarePoints(reg_elem_.getEndLines());
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

void VirtualTrafficLightModule::setState(State new_state)
{
  state_ = new_state;
  updateLoggerWithState();
}

bool VirtualTrafficLightModule::modifyPathVelocity(PathWithLaneId * path)
{
  // Initialize
  setInfrastructureCommand({});

  module_data_ = {};

  // Copy data
  module_data_.head_pose = calcHeadPose(
    planner_data_->current_odometry->pose, planner_data_->vehicle_info_.max_longitudinal_offset_m);
  module_data_.path = *path;

  // Define stop line ID string for logging purposes
  const std::string stop_line_id_for_log =
    reg_elem_.getStopLine() ? std::to_string(reg_elem_.getStopLine()->id()) : "";

  // Calculate path index of end line
  // NOTE: In order to deal with u-turn or self-crossing path, only start/stop lines before the end
  // line are used when whether the ego is before/after the start/stop/end lines is calculated.
  const auto opt_end_line_result = getPathIndexOfFirstEndLine();
  if (!opt_end_line_result) {
    return true;
  }
  const size_t end_line_idx = opt_end_line_result->first;
  const int64_t end_line_id = opt_end_line_result->second;

  // Do nothing if vehicle is before start line
  if (isBeforeStartLine(end_line_idx)) {
    logDebug("before start_line");
    setState(State::NONE);
    updateInfrastructureCommand();
    return true;
  }

  // Do nothing if vehicle is after any end line
  if (isAfterAnyEndLine(end_line_idx) || state_ == State::FINALIZED) {
    if (state_ != State::FINALIZED) {
      logInfo(
        "State transition: %d -> FINALIZED because vehicle pass end line (ID: %ld)",
        static_cast<int>(state_), end_line_id);
    }
    setState(State::FINALIZED);
    updateInfrastructureCommand();
    return true;
  }

  // Set state
  if (state_ != State::REQUESTING) {
    logInfo(
      "State transition: %d -> REQUESTING as vehicle is between start line (ID: %ld) and end line "
      "(ID: %ld)",
      static_cast<int>(state_), reg_elem_.getStartLine().id(), end_line_id);
  }
  setState(State::REQUESTING);

  // Don't need to stop if there is no stop_line
  if (!map_data_.stop_line) {
    updateInfrastructureCommand();
    return true;
  }

  // Find corresponding state
  const auto virtual_traffic_light_state = findCorrespondingState();

  // Stop at stop_line if no message received
  if (!virtual_traffic_light_state) {
    logWarnThrottle(5000, "no message received");
    insertStopVelocityAtStopLine(path, end_line_idx);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if no right is given
  if (!hasRightOfWay(*virtual_traffic_light_state)) {
    logInfoThrottle(
      5000, "Approval for passage has not yet been given, so wait at stop line (ID: %s)",
      stop_line_id_for_log.c_str());
    insertStopVelocityAtStopLine(path, end_line_idx);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if state is timeout before stop_line
  if (isBeforeStopLine(end_line_idx)) {
    if (isStateTimeout(*virtual_traffic_light_state)) {
      logWarnThrottle(
        5000, "state is timeout before stop line, stop at stop line (ID: %s)",
        stop_line_id_for_log.c_str());
      insertStopVelocityAtStopLine(path, end_line_idx);
    }

    updateInfrastructureCommand();
    return true;
  }

  // After stop_line
  if (state_ != State::PASSING) {
    logInfo(
      "State transition: %d -> PASSING as vehicle has passed stop line (ID: %s)",
      static_cast<int>(state_), stop_line_id_for_log.c_str());
  }
  setState(State::PASSING);

  // Check timeout after stop_line if the option is on
  if (
    planner_param_.check_timeout_after_stop_line && isStateTimeout(*virtual_traffic_light_state)) {
    logDebug("state is timeout after stop line");
    insertStopVelocityAtStopLine(path, end_line_idx);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if finalization isn't completed
  if (!virtual_traffic_light_state->is_finalized) {
    logDebug("finalization isn't completed");
    insertStopVelocityAtEndLine(path, end_line_idx);

    if (isNearAnyEndLine(end_line_idx) && planner_data_->isVehicleStopped()) {
      if (state_ != State::FINALIZING) {
        logInfo(
          "State transition: %d -> FINALIZING as vehicle is near end line (ID: %ld) and stopped",
          static_cast<int>(state_), end_line_id);
      }
      setState(State::FINALIZING);
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

std::optional<std::pair<size_t, int64_t>> VirtualTrafficLightModule::getPathIndexOfFirstEndLine()
{
  std::optional<std::pair<size_t, int64_t>> min_result;
  const auto original_end_lines = reg_elem_.getEndLines();

  for (size_t i = 0; i < map_data_.end_lines.size() && i < original_end_lines.size(); ++i) {
    const auto & end_line = map_data_.end_lines[i];
    const auto & original_end_line = original_end_lines[i];

    geometry_msgs::msg::Point end_line_p1;
    end_line_p1.x = end_line.front().x();
    end_line_p1.y = end_line.front().y();

    geometry_msgs::msg::Point end_line_p2;
    end_line_p2.x = end_line.back().x();
    end_line_p2.y = end_line.back().y();

    const auto connected_lane_ids =
      planning_utils::collectConnectedLaneIds(lane_id_, planner_data_->route_handler_);

    const auto collision = arc_lane_utils::findCollisionSegment(
      module_data_.path, end_line_p1, end_line_p2, connected_lane_ids);

    if (!collision) {
      continue;
    }

    const size_t collision_seg_idx = collision->first;

    if (!min_result || collision_seg_idx < min_result->first) {
      min_result = std::make_pair(
        collision_seg_idx + 1,  // NOTE: In order that min_seg_idx will be after the end line
        original_end_line.id());
    }
  }

  return min_result;
}

bool VirtualTrafficLightModule::isBeforeStartLine(const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(module_data_.path.points, map_data_.start_line, end_line_idx);

  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  if (!collision) {
    return false;
  }
  const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    module_data_.path.points, collision->point, collision->index);

  const auto & ego_pose = planner_data_->current_odometry->pose;
  const size_t ego_seg_idx = findEgoSegmentIndex(module_data_.path.points);
  const auto signed_arc_length = autoware::motion_utils::calcSignedArcLength(
                                   module_data_.path.points, ego_pose.position, ego_seg_idx,
                                   collision->point, collision_seg_idx) -
                                 planner_data_->vehicle_info_.max_longitudinal_offset_m;

  return signed_arc_length > 0;
}

bool VirtualTrafficLightModule::isBeforeStopLine(const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(module_data_.path.points, *map_data_.stop_line, end_line_idx);

  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  if (!collision) {
    return false;
  }
  const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    module_data_.path.points, collision->point, collision->index);

  const auto & ego_pose = planner_data_->current_odometry->pose;
  const size_t ego_seg_idx = findEgoSegmentIndex(module_data_.path.points);
  const auto signed_arc_length = autoware::motion_utils::calcSignedArcLength(
                                   module_data_.path.points, ego_pose.position, ego_seg_idx,
                                   collision->point, collision_seg_idx) -
                                 planner_data_->vehicle_info_.max_longitudinal_offset_m;

  return signed_arc_length > -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isAfterAnyEndLine(const size_t end_line_idx)
{
  // Assume stop line is before end lines
  if (isBeforeStopLine(end_line_idx)) {
    return false;
  }

  const auto collision =
    findLastCollisionBeforeEndLine(module_data_.path.points, map_data_.end_lines, end_line_idx);

  // If the goal is set before the end line, collision will not be detected.
  // Therefore if there is no collision, the ego vehicle is assumed to be before the end line.
  if (!collision) {
    return false;
  }
  const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    module_data_.path.points, collision->point, collision->index);

  const auto & ego_pose = planner_data_->current_odometry->pose;
  const size_t ego_seg_idx = findEgoSegmentIndex(module_data_.path.points);
  const auto signed_arc_length = autoware::motion_utils::calcSignedArcLength(
                                   module_data_.path.points, ego_pose.position, ego_seg_idx,
                                   collision->point, collision_seg_idx) -
                                 planner_data_->vehicle_info_.max_longitudinal_offset_m;

  return signed_arc_length < -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isNearAnyEndLine(const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(module_data_.path.points, map_data_.end_lines, end_line_idx);

  if (!collision) {
    return false;
  }
  const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    module_data_.path.points, collision->point, collision->index);

  const auto & ego_pose = planner_data_->current_odometry->pose;
  const size_t ego_seg_idx = findEgoSegmentIndex(module_data_.path.points);
  const auto signed_arc_length = autoware::motion_utils::calcSignedArcLength(
                                   module_data_.path.points, ego_pose.position, ego_seg_idx,
                                   collision->point, collision_seg_idx) -
                                 planner_data_->vehicle_info_.max_longitudinal_offset_m;

  return std::abs(signed_arc_length) < planner_param_.near_line_distance;
}

std::optional<tier4_v2x_msgs::msg::VirtualTrafficLightState>
VirtualTrafficLightModule::findCorrespondingState()
{
  // Note: This variable is set by virtual traffic light's manager.
  if (!virtual_traffic_light_states_) {
    return {};
  }

  for (const auto & state : virtual_traffic_light_states_->states) {
    if (state.id == map_data_.instrument_id) {
      return state;
    }
  }

  return {};
}

bool VirtualTrafficLightModule::isStateTimeout(
  const tier4_v2x_msgs::msg::VirtualTrafficLightState & state)
{
  const auto delay = (clock_->now() - rclcpp::Time(state.stamp)).seconds();
  if (delay > planner_param_.max_delay_sec) {
    logDebug("delay=%f, max_delay=%f", delay, planner_param_.max_delay_sec);
    return true;
  }

  return false;
}

bool VirtualTrafficLightModule::hasRightOfWay(
  const tier4_v2x_msgs::msg::VirtualTrafficLightState & state)
{
  return state.approval;
}

void VirtualTrafficLightModule::insertStopVelocityAtStopLine(
  autoware_internal_planning_msgs::msg::PathWithLaneId * path, const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(path->points, *map_data_.stop_line, end_line_idx);
  const auto offset = -planner_data_->vehicle_info_.max_longitudinal_offset_m;

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision) {
    insertStopVelocityFromStart(path);
    stop_pose = planner_data_->current_odometry->pose;
  } else {
    const auto & ego_pose = planner_data_->current_odometry->pose;
    const size_t ego_seg_idx = findEgoSegmentIndex(path->points);
    const size_t collision_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
      path->points, collision->point, collision->index);

    const auto stop_distance =
      autoware::motion_utils::calcSignedArcLength(
        path->points, ego_pose.position, ego_seg_idx, collision.value().point, collision_seg_idx) +
      offset;
    const auto is_stopped = planner_data_->isVehicleStopped();

    if (stop_distance < planner_param_.hold_stop_margin_distance && is_stopped) {
      SegmentIndexWithPoint new_collision;
      const auto ego_pos_on_path =
        autoware::motion_utils::calcLongitudinalOffsetPoint(path->points, ego_pose.position, 0.0);

      if (ego_pos_on_path) {
        new_collision.point = ego_pos_on_path.value();
        new_collision.index = ego_seg_idx;
        insertStopVelocityAtCollision(new_collision, 0.0, path);
      }

      // for virtual wall
      {
        auto path_tmp = path;
        const auto insert_index = insertStopVelocityAtCollision(*collision, offset, path_tmp);
        if (insert_index) {
          stop_pose = path_tmp->points.at(insert_index.value()).point.pose;
        }
      }

    } else {
      const auto insert_index = insertStopVelocityAtCollision(*collision, offset, path);
      if (insert_index) {
        stop_pose = path->points.at(insert_index.value()).point.pose;
      }
    }
  }

  // Set StopReason
  planning_factor_interface_->add(
    path->points, planner_data_->current_odometry->pose, stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  // Set data for visualization
  module_data_.stop_head_pose_at_stop_line =
    calcHeadPose(stop_pose, planner_data_->vehicle_info_.max_longitudinal_offset_m);
}

void VirtualTrafficLightModule::insertStopVelocityAtEndLine(
  autoware_internal_planning_msgs::msg::PathWithLaneId * path, const size_t end_line_idx)
{
  const auto collision =
    findLastCollisionBeforeEndLine(path->points, map_data_.end_lines, end_line_idx);

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision) {
    // No enough length
    if (isBeforeStopLine(end_line_idx)) {
      return;
    }

    insertStopVelocityFromStart(path);
    stop_pose = planner_data_->current_odometry->pose;
  } else {
    const auto offset = -planner_data_->vehicle_info_.max_longitudinal_offset_m;
    const auto insert_index = insertStopVelocityAtCollision(*collision, offset, path);
    if (insert_index) {
      stop_pose = path->points.at(insert_index.value()).point.pose;
    }
  }

  // Set StopReason
  planning_factor_interface_->add(
    path->points, planner_data_->current_odometry->pose, stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  // Set data for visualization
  module_data_.stop_head_pose_at_end_line =
    calcHeadPose(stop_pose, planner_data_->vehicle_info_.max_longitudinal_offset_m);
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

void VirtualTrafficLightModule::setVirtualTrafficLightStates(
  const tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr
    virtual_traffic_light_states)
{
  virtual_traffic_light_states_ = virtual_traffic_light_states;
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
