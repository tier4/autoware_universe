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
  
  // StateTransitionManagerのコールバック関数を設定
  state_manager_.setStateChangeCallback(
    [this](State old_state, State new_state) {
      this->onStateChanged(old_state, new_state);
    });
  
  updateLoggerWithState();
}

void VirtualTrafficLightModule::onStateChanged(State old_state, State new_state)
{
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
  const auto opt_end_line_result = getPathIndexOfFirstEndLine();
  if (!opt_end_line_result) {
    return true;
  }
  const size_t end_line_idx = opt_end_line_result->first;
  const int64_t end_line_id = opt_end_line_result->second;

  // 特殊な状況の処理
  StateTransitionResult result;

  // Do nothing if vehicle is before start line
  if (isBeforeStartLine(end_line_idx)) {
    logDebug("before start_line");
    result = state_manager_.processBeforeStartLine();
    if (result.state_changed) {
      updateInfrastructureCommand();
    }
    return true;
  }

  // Do nothing if vehicle is after any end line
  if (isAfterAnyEndLine(end_line_idx) || state_manager_.getCurrentState() == State::FINALIZED) {
    result = state_manager_.processAfterEndLine(end_line_id);
    if (result.state_changed) {
      logInfo(result.log_message.c_str());
    }
    updateInfrastructureCommand();
    return true;
  }

  // 現在の状態に基づいた処理
  const State current_state = state_manager_.getCurrentState();

  switch (current_state) {
    case State::NONE: {
      result = state_manager_.processNoneState(reg_elem_.getStartLine().id());
      if (result.state_changed) {
        logInfo(result.log_message.c_str());
      }
      break;
    }

    case State::REQUESTING: {
      // 必要な情報を収集
      const auto virtual_traffic_light_state = findCorrespondingState();
      const bool has_stop_line = static_cast<bool>(map_data_.stop_line);
      const bool has_virtual_state = static_cast<bool>(virtual_traffic_light_state);
      const bool has_right_of_way =
        has_virtual_state ? hasRightOfWay(*virtual_traffic_light_state) : false;
      const bool is_before_stop_line = isBeforeStopLine(end_line_idx);
      const bool is_timeout =
        has_virtual_state ? isStateTimeout(*virtual_traffic_light_state) : false;

      result = state_manager_.processRequestingState(
        stop_line_id_for_log, has_stop_line, has_virtual_state, has_right_of_way,
        is_before_stop_line, is_timeout);

      // 結果に基づいた処理
      if (result.state_changed) {
        logInfo(result.log_message.c_str());
      }

      switch (result.action) {
        case StateTransitionResult::Action::RETURN_TRUE:
          updateInfrastructureCommand();
          return true;
        case StateTransitionResult::Action::STOP_AT_STOP_LINE:
          if (!result.log_message.empty()) {
            logWarnThrottle(5000, result.log_message.c_str());
          }
          insertStopVelocityAtStopLine(path, end_line_idx);
          updateInfrastructureCommand();
          return true;
        case StateTransitionResult::Action::CONTINUE:
          break;
        default:
          break;
      }
      break;
    }

    case State::PASSING: {
      // 必要な情報を収集
      const auto virtual_traffic_light_state = findCorrespondingState();
      const bool has_virtual_state = static_cast<bool>(virtual_traffic_light_state);
      const bool is_timeout_after_stop = planner_param_.check_timeout_after_stop_line &&
                                         has_virtual_state &&
                                         isStateTimeout(*virtual_traffic_light_state);
      const bool is_finalized =
        has_virtual_state ? virtual_traffic_light_state->is_finalized : true;
      const bool is_near_end_line = isNearAnyEndLine(end_line_idx);
      const bool is_stopped = planner_data_->isVehicleStopped();

      result = state_manager_.processPassingState(
        end_line_id, has_virtual_state, is_timeout_after_stop, is_finalized, is_near_end_line,
        is_stopped);

      // 結果に基づいた処理
      if (result.state_changed) {
        logInfo(result.log_message.c_str());
      }

      switch (result.action) {
        case StateTransitionResult::Action::RETURN_TRUE:
          updateInfrastructureCommand();
          return true;
        case StateTransitionResult::Action::STOP_AT_STOP_LINE:
          logDebug(result.log_message.c_str());
          insertStopVelocityAtStopLine(path, end_line_idx);
          updateInfrastructureCommand();
          return true;
        case StateTransitionResult::Action::STOP_AT_END_LINE:
          logDebug(result.log_message.c_str());
          insertStopVelocityAtEndLine(path, end_line_idx);
          break;
        case StateTransitionResult::Action::CONTINUE:
          break;
      }
      break;
    }

    case State::FINALIZING: {
      // 必要な情報を収集
      const auto virtual_traffic_light_state = findCorrespondingState();
      const bool is_finalized =
        virtual_traffic_light_state ? virtual_traffic_light_state->is_finalized : true;

      result = state_manager_.processFinalizingState(is_finalized);

      switch (result.action) {
        case StateTransitionResult::Action::RETURN_TRUE:
          updateInfrastructureCommand();
          return true;
        case StateTransitionResult::Action::STOP_AT_END_LINE:
          logDebug(result.log_message.c_str());
          insertStopVelocityAtEndLine(path, end_line_idx);
          break;
        case StateTransitionResult::Action::CONTINUE:
          break;
        default:
          break;
      }
      break;
    }

    case State::FINALIZED: {
      result = state_manager_.processFinalizedState();
      break;
    }

    default: {
      logWarn("Unknown state: %d", static_cast<int>(current_state));
      break;
    }
  }

  updateInfrastructureCommand();
  return true;
}

void VirtualTrafficLightModule::updateInfrastructureCommand()
{
  command_.stamp = clock_->now();
  command_.state = static_cast<uint8_t>(state_manager_.getCurrentState());
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
  return state_manager_.stateToString(state);
}

VirtualTrafficLightModule::State VirtualTrafficLightModule::getState() const
{
  return state_manager_.getCurrentState();
}

void VirtualTrafficLightModule::updateLoggerWithState()
{
  const std::string state_name = stateToString(state_manager_.getCurrentState());
  logger_ = base_logger_.get_child(("state: " + state_name).c_str());
}

// StateTransitionManagerの実装
StateTransitionManager::StateTransitionManager(State initial_state) : current_state_(initial_state)
{
}

void StateTransitionManager::setStateChangeCallback(StateChangeCallback callback)
{
  state_change_callback_ = callback;
}

void StateTransitionManager::changeState(State new_state)
{
  if (current_state_ != new_state) {
    State old_state = current_state_;
    current_state_ = new_state;
    
    // コールバック関数が設定されていれば呼び出す
    if (state_change_callback_) {
      state_change_callback_(old_state, new_state);
    }
  }
}

StateTransitionResult StateTransitionManager::processBeforeStartLine()
{
  StateTransitionResult result;
  if (current_state_ != State::NONE) {
    changeState(State::NONE);
    result.state_changed = true;
    result.log_message = "State transition: -> NONE (before start line)";
  } else {
    result.state_changed = false;
  }
  result.action = StateTransitionResult::Action::RETURN_TRUE;
  return result;
}

StateTransitionResult StateTransitionManager::processAfterEndLine(int64_t end_line_id)
{
  StateTransitionResult result;
  if (current_state_ != State::FINALIZED) {
    changeState(State::FINALIZED);
    result.state_changed = true;
    result.log_message =
      "State transition: -> FINALIZED (passed end line ID: " + std::to_string(end_line_id) + ")";
  } else {
    result.state_changed = false;
  }
  result.action = StateTransitionResult::Action::RETURN_TRUE;
  return result;
}

StateTransitionResult StateTransitionManager::processNoneState(int64_t start_line_id)
{
  StateTransitionResult result;
  changeState(State::REQUESTING);
  result.state_changed = true;
  result.log_message = "State transition: NONE -> REQUESTING (crossed start line ID: " +
                       std::to_string(start_line_id) + ")";
  result.action = StateTransitionResult::Action::CONTINUE;
  return result;
}

StateTransitionResult StateTransitionManager::processRequestingState(
  const std::string & stop_line_id, bool has_stop_line, bool has_virtual_state,
  bool has_right_of_way, bool is_before_stop_line, bool is_timeout)
{
  StateTransitionResult result;
  result.state_changed = false;

  // stop lineがない場合
  if (!has_stop_line) {
    result.action = StateTransitionResult::Action::RETURN_TRUE;
    return result;
  }

  // メッセージ未受信
  if (!has_virtual_state) {
    result.action = StateTransitionResult::Action::STOP_AT_STOP_LINE;
    result.log_message = "No message received";
    return result;
  }

  // 通行権なし
  if (!has_right_of_way) {
    result.action = StateTransitionResult::Action::STOP_AT_STOP_LINE;
    result.log_message = "No right of way, waiting at stop line ID: " + stop_line_id;
    return result;
  }

  // stop line前でタイムアウト
  if (is_before_stop_line) {
    if (is_timeout) {
      result.action = StateTransitionResult::Action::STOP_AT_STOP_LINE;
      result.log_message = "Timeout before stop line ID: " + stop_line_id;
    } else {
      result.action = StateTransitionResult::Action::RETURN_TRUE;
    }
    return result;
  }

  // stop lineを超えた → PASSING遷移
  changeState(State::PASSING);
  result.state_changed = true;
  result.log_message =
    "State transition: REQUESTING -> PASSING (crossed stop line ID: " + stop_line_id + ")";
  result.action = StateTransitionResult::Action::CONTINUE;
  return result;
}

StateTransitionResult StateTransitionManager::processPassingState(
  int64_t end_line_id, bool has_virtual_state, bool is_timeout_after_stop, bool is_finalized,
  bool is_near_end_line, bool is_stopped)
{
  StateTransitionResult result;
  result.state_changed = false;

  if (!has_virtual_state) {
    result.action = StateTransitionResult::Action::RETURN_TRUE;
    return result;
  }

  // stop line後のタイムアウトチェック
  if (is_timeout_after_stop) {
    result.action = StateTransitionResult::Action::STOP_AT_STOP_LINE;
    result.log_message = "Timeout after stop line";
    return result;
  }

  // finalization未完了の場合
  if (!is_finalized) {
    result.action = StateTransitionResult::Action::STOP_AT_END_LINE;
    result.log_message = "Finalization not completed";

    // end line近くで停止 → FINALIZING遷移
    if (is_near_end_line && is_stopped) {
      changeState(State::FINALIZING);
      result.state_changed = true;
      result.log_message = "State transition: PASSING -> FINALIZING (near end line ID: " +
                           std::to_string(end_line_id) + " and stopped)";
    }
    return result;
  }

  result.action = StateTransitionResult::Action::CONTINUE;
  return result;
}

StateTransitionResult StateTransitionManager::processFinalizingState(bool is_finalized)
{
  StateTransitionResult result;
  result.state_changed = false;

  if (!is_finalized) {
    result.action = StateTransitionResult::Action::STOP_AT_END_LINE;
    result.log_message = "Finalization not completed, waiting...";
  } else {
    result.action = StateTransitionResult::Action::CONTINUE;
  }

  return result;
}

StateTransitionResult StateTransitionManager::processFinalizedState()
{
  StateTransitionResult result;
  result.state_changed = false;
  result.action = StateTransitionResult::Action::CONTINUE;
  return result;
}

std::string StateTransitionManager::stateToString(State state) const
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
}  // namespace autoware::behavior_velocity_planner
