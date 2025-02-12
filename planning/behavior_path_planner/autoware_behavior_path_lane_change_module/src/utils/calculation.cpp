
// Copyright 2024 TIER IV, Inc.
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

#include <autoware/behavior_path_lane_change_module/utils/calculation.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>

namespace autoware::behavior_path_planner::utils::lane_change::calculation
{
double calc_ego_dist_to_terminal_end(const CommonDataPtr & common_data_ptr)
{
  const auto & lanes_ptr = common_data_ptr->lanes_ptr;
  const auto & current_lanes = lanes_ptr->current;
  const auto & current_pose = common_data_ptr->get_ego_pose();

  return calc_dist_from_pose_to_terminal_end(common_data_ptr, current_lanes, current_pose);
}

double calc_dist_from_pose_to_terminal_end(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & lanes,
  const Pose & src_pose)
{
  if (lanes.empty()) {
    return 0.0;
  }

  const auto & lanes_ptr = common_data_ptr->lanes_ptr;
  const auto & goal_pose = common_data_ptr->route_handler_ptr->getGoalPose();

  if (lanes_ptr->current_lane_in_goal_section) {
    return utils::getSignedDistance(src_pose, goal_pose, lanes);
  }
  return utils::getDistanceToEndOfLane(src_pose, lanes);
}

double calc_stopping_distance(const LCParamPtr & lc_param_ptr)
{
  // v^2 = u^2 + 2ad
  const auto min_lc_vel = lc_param_ptr->minimum_lane_changing_velocity;
  const auto min_lon_acc = lc_param_ptr->min_longitudinal_acc;
  const auto min_back_dist = std::abs((min_lc_vel * min_lc_vel) / (2 * min_lon_acc));

  const auto param_back_dist = lc_param_ptr->backward_length_buffer_for_end_of_lane;
  return std::max(min_back_dist, param_back_dist);
}

double calc_maximum_prepare_length(const CommonDataPtr & common_data_ptr)
{
  const auto max_prepare_duration = common_data_ptr->lc_param_ptr->lane_change_prepare_duration;
  const auto ego_max_speed = common_data_ptr->bpp_param_ptr->max_vel;

  return max_prepare_duration * ego_max_speed;
}

double calc_ego_dist_to_lanes_start(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes)
{
  const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;

  if (!route_handler_ptr || target_lanes.empty() || current_lanes.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto & target_bound =
    common_data_ptr->direction == autoware::route_handler::Direction::RIGHT
      ? target_lanes.front().leftBound()
      : target_lanes.front().rightBound();

  if (target_bound.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto path =
    route_handler_ptr->getCenterLinePath(current_lanes, 0.0, std::numeric_limits<double>::max());

  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto target_front_pt = lanelet::utils::conversion::toGeomMsgPt(target_bound.front());
  const auto ego_position = common_data_ptr->get_ego_pose().position;

  return motion_utils::calcSignedArcLength(path.points, ego_position, target_front_pt);
}

double calc_minimum_lane_change_length(
  const LaneChangeParameters & lane_change_parameters, const std::vector<double> & shift_intervals)
{
  if (shift_intervals.empty()) {
    return 0.0;
  }

  const auto min_vel = lane_change_parameters.minimum_lane_changing_velocity;
  const auto min_max_lat_acc = lane_change_parameters.lane_change_lat_acc_map.find(min_vel);
  // const auto min_lat_acc = std::get<0>(min_max_lat_acc);
  const auto max_lat_acc = std::get<1>(min_max_lat_acc);
  const auto lat_jerk = lane_change_parameters.lane_changing_lateral_jerk;
  const auto finish_judge_buffer = lane_change_parameters.lane_change_finish_judge_buffer;

  const auto calc_sum = [&](double sum, double shift_interval) {
    const auto t = PathShifter::calcShiftTimeFromJerk(shift_interval, lat_jerk, max_lat_acc);
    return sum + (min_vel * t + finish_judge_buffer);
  };

  const auto total_length =
    std::accumulate(shift_intervals.begin(), shift_intervals.end(), 0.0, calc_sum);

  const auto backward_buffer = lane_change_parameters.backward_length_buffer_for_end_of_lane;
  return total_length + backward_buffer * (static_cast<double>(shift_intervals.size()) - 1.0);
}

double calc_minimum_lane_change_length(
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelet & lane,
  const LaneChangeParameters & lane_change_parameters, Direction direction)
{
  if (!route_handler) {
    return std::numeric_limits<double>::max();
  }

  const auto shift_intervals = route_handler->getLateralIntervalsToPreferredLane(lane, direction);
  return calc_minimum_lane_change_length(lane_change_parameters, shift_intervals);
}

double calc_maximum_lane_change_length(
  const double current_velocity, const LaneChangeParameters & lane_change_parameters,
  const std::vector<double> & shift_intervals, const double max_acc)
{
  if (shift_intervals.empty()) {
    return 0.0;
  }

  const auto finish_judge_buffer = lane_change_parameters.lane_change_finish_judge_buffer;
  const auto lat_jerk = lane_change_parameters.lane_changing_lateral_jerk;
  const auto t_prepare = lane_change_parameters.lane_change_prepare_duration;

  auto vel = current_velocity;

  const auto calc_sum = [&](double sum, double shift_interval) {
    // prepare section
    const auto prepare_length = vel * t_prepare + 0.5 * max_acc * t_prepare * t_prepare;
    vel = vel + max_acc * t_prepare;

    // lane changing section
    const auto [min_lat_acc, max_lat_acc] =
      lane_change_parameters.lane_change_lat_acc_map.find(vel);
    const auto t_lane_changing =
      PathShifter::calcShiftTimeFromJerk(shift_interval, lat_jerk, max_lat_acc);
    const auto lane_changing_length =
      vel * t_lane_changing + 0.5 * max_acc * t_lane_changing * t_lane_changing;

    vel = vel + max_acc * t_lane_changing;
    return sum + (prepare_length + lane_changing_length + finish_judge_buffer);
  };

  const auto total_length =
    std::accumulate(shift_intervals.begin(), shift_intervals.end(), 0.0, calc_sum);

  const auto backward_buffer = lane_change_parameters.backward_length_buffer_for_end_of_lane;
  return total_length + backward_buffer * (static_cast<double>(shift_intervals.size()) - 1.0);
}

double calc_maximum_lane_change_length(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelet & current_terminal_lanelet,
  const double max_acc)
{
  const auto shift_intervals =
    common_data_ptr->route_handler_ptr->getLateralIntervalsToPreferredLane(
      current_terminal_lanelet);
  const auto vel = std::max(
    common_data_ptr->get_ego_speed(),
    common_data_ptr->lc_param_ptr->minimum_lane_changing_velocity);
  return calc_maximum_lane_change_length(
    vel, *common_data_ptr->lc_param_ptr, shift_intervals, max_acc);
}
}  // namespace autoware::behavior_path_planner::utils::lane_change::calculation
