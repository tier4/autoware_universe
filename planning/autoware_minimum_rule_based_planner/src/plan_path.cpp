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

#include "minimum_rule_based_planner.hpp"
#include "utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

std::optional<PathWithLaneId> MinimumRuleBasedPlannerNode::plan_path(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & current_pose = input_data.odometry_ptr->pose.pose;
  const auto params = param_listener_->get_params();
  const auto path_length_backward = params.path_length.backward;
  const auto path_length_forward = params.path_length.forward;

  if (!update_current_lanelet(current_pose, params)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to update current lanelet");
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{*current_lanelet_};
  const auto s_on_current_lanelet =
    lanelet::utils::getArcCoordinates({*current_lanelet_}, current_pose).length;

  const auto backward_length = std::max(
    0., path_length_backward + vehicle_info_.max_longitudinal_offset_m - s_on_current_lanelet);
  const auto backward_lanelets_within_route =
    utils::get_lanelets_within_route_up_to(*current_lanelet_, planner_data_, backward_length);
  if (!backward_lanelets_within_route) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get backward lanelets within route for current lanelet (id: %ld)",
      current_lanelet_->id());
    return std::nullopt;
  }
  lanelets.insert(
    lanelets.begin(), backward_lanelets_within_route->begin(),
    backward_lanelets_within_route->end());

  //  Extend lanelets by backward_length even outside planned route to ensure
  //  ego footprint is inside lanelets if ego is at the beginning of start lane
  auto backward_lanelets_length =
    lanelet::utils::getLaneletLength2d(*backward_lanelets_within_route);
  while (backward_lanelets_length < backward_length) {
    const auto prev_lanelets = planner_data_.routing_graph_ptr->previous(lanelets.front());
    if (prev_lanelets.empty()) {
      break;
    }
    lanelets.insert(lanelets.begin(), prev_lanelets.front());
    backward_lanelets_length += lanelet::geometry::length2d(prev_lanelets.front());
  }

  const auto forward_length = std::max(
    0., path_length_forward + vehicle_info_.max_longitudinal_offset_m -
          (lanelet::geometry::length2d(*current_lanelet_) - s_on_current_lanelet));
  const auto forward_lanelets_within_route =
    utils::get_lanelets_within_route_after(*current_lanelet_, planner_data_, forward_length);
  if (!forward_lanelets_within_route) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get forward lanelets within route for current lanelet (id: %ld)",
      current_lanelet_->id());
    return std::nullopt;
  }
  lanelets.insert(
    lanelets.end(), forward_lanelets_within_route->begin(), forward_lanelets_within_route->end());

  //  Extend lanelets by forward_length even outside planned route to ensure
  //  ego footprint is inside lanelets if ego is at the end of goal lane
  auto forward_lanelets_length = lanelet::utils::getLaneletLength2d(*forward_lanelets_within_route);
  while (forward_lanelets_length < forward_length) {
    const auto next_lanelets = planner_data_.routing_graph_ptr->following(lanelets.back());
    if (next_lanelets.empty()) {
      break;
    }
    lanelets.insert(lanelets.end(), next_lanelets.front());
    forward_lanelets_length += lanelet::geometry::length2d(next_lanelets.front());
  }

  const auto s = s_on_current_lanelet + backward_lanelets_length;
  const auto s_start = std::max(0., s - path_length_backward);
  const auto s_end = [&]() {
    auto s_end_val = s + path_length_forward;

    if (!utils::get_next_lanelet_within_route(lanelets.back(), planner_data_)) {
      s_end_val = std::min(s_end_val, lanelet::utils::getLaneletLength2d(lanelets));
    }

    for (auto [it, goal_arc_length] = std::make_tuple(lanelets.begin(), 0.); it != lanelets.end();
         ++it) {
      if (std::any_of(
            planner_data_.goal_lanelets.begin(), planner_data_.goal_lanelets.end(),
            [&](const auto & goal_lanelet) { return it->id() == goal_lanelet.id(); })) {
        goal_arc_length += lanelet::utils::getArcCoordinates({*it}, planner_data_.goal_pose).length;
        s_end_val = std::min(s_end_val, goal_arc_length);
        break;
      }
      goal_arc_length += lanelet::geometry::length2d(*it);
    }

    const lanelet::LaneletSequence lanelet_seq(lanelets);
    if (
      const auto s_intersection = utils::get_first_intersection_arc_length(
        lanelet_seq, std::max(0., s_start - vehicle_info_.max_longitudinal_offset_m),
        s_end_val + vehicle_info_.max_longitudinal_offset_m, vehicle_info_.vehicle_length_m)) {
      s_end_val = std::min(
        s_end_val, std::max(0., *s_intersection - vehicle_info_.max_longitudinal_offset_m));
    }

    return s_end_val;
  }();

  return generate_path(lanelets, s_start, s_end, params);
}

std::optional<PathWithLaneId> MinimumRuleBasedPlannerNode::generate_path(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const Params & params)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (lanelet_sequence.empty()) {
    RCLCPP_ERROR(get_logger(), "Lanelet sequence is empty");
    return std::nullopt;
  }

  std::vector<PathPointWithLaneId> path_points_with_lane_id{};

  const auto waypoint_groups = utils::get_waypoint_groups(
    lanelet_sequence, *planner_data_.lanelet_map_ptr, params.waypoint_group.separation_threshold,
    params.waypoint_group.interval_margin_ratio);

  auto extended_lanelets = lanelet_sequence.lanelets();
  auto extended_arc_length = 0.;
  for (const auto & [waypoints, interval] : waypoint_groups) {
    while (interval.start + extended_arc_length < 0.) {
      const auto prev_lanelets =
        planner_data_.routing_graph_ptr->previous(extended_lanelets.front());
      if (prev_lanelets.empty()) {
        break;
      }
      extended_lanelets.insert(extended_lanelets.begin(), prev_lanelets.front());
      extended_arc_length += lanelet::utils::getLaneletLength2d(prev_lanelets.front());
    }
  }

  const auto add_path_point =
    [&](const lanelet::ConstPoint3d & path_point, const lanelet::Id & lane_id) {
      PathPointWithLaneId path_point_with_lane_id{};
      path_point_with_lane_id.lane_ids.push_back(lane_id);
      path_point_with_lane_id.point.pose.position =
        lanelet::utils::conversion::toGeomMsgPt(path_point);
      path_point_with_lane_id.point.longitudinal_velocity_mps =
        planner_data_.traffic_rules_ptr
          ->speedLimit(planner_data_.lanelet_map_ptr->laneletLayer.get(lane_id))
          .speedLimit.value();
      path_points_with_lane_id.push_back(std::move(path_point_with_lane_id));
    };

  const lanelet::LaneletSequence extended_lanelet_sequence(extended_lanelets);
  std::optional<size_t> overlapping_waypoint_group_index = std::nullopt;

  for (auto [lanelet_it, s] = std::make_tuple(extended_lanelet_sequence.begin(), 0.);
       lanelet_it != extended_lanelet_sequence.end(); ++lanelet_it) {
    const auto & centerline = lanelet_it->centerline();

    for (auto point_it = centerline.begin(); point_it != centerline.end(); ++point_it) {
      if (point_it != centerline.begin()) {
        s += lanelet::geometry::distance2d(*std::prev(point_it), *point_it);
      } else if (lanelet_it != extended_lanelet_sequence.begin()) {
        continue;
      }

      if (overlapping_waypoint_group_index) {
        const auto & [waypoints, interval] = waypoint_groups[*overlapping_waypoint_group_index];
        if (s >= interval.start + extended_arc_length && s <= interval.end + extended_arc_length) {
          continue;
        }
        overlapping_waypoint_group_index = std::nullopt;
      }

      for (size_t i = 0; i < waypoint_groups.size(); ++i) {
        const auto & [waypoints, interval] = waypoint_groups[i];
        if (s < interval.start + extended_arc_length || s > interval.end + extended_arc_length) {
          continue;
        }
        for (const auto & waypoint : waypoints) {
          add_path_point(waypoint.point, waypoint.lane_id);
        }
        overlapping_waypoint_group_index = i;
        break;
      }
      if (overlapping_waypoint_group_index) {
        continue;
      }

      add_path_point(*point_it, lanelet_it->id());
      if (
        point_it == std::prev(centerline.end()) &&
        lanelet_it != std::prev(extended_lanelet_sequence.end())) {
        if (
          lanelet_it != extended_lanelet_sequence.begin() ||
          lanelet_it->id() == lanelet_sequence.begin()->id()) {
          path_points_with_lane_id.back().lane_ids.push_back(std::next(lanelet_it)->id());
        } else {
          path_points_with_lane_id.back().lane_ids = {std::next(lanelet_it)->id()};
        }
      }
    }
  }

  if (path_points_with_lane_id.empty()) {
    RCLCPP_ERROR(get_logger(), "No path points generated from lanelet sequence");
    return std::nullopt;
  }

  auto trajectory = autoware::experimental::trajectory::pretty_build(path_points_with_lane_id);
  if (!trajectory) {
    RCLCPP_ERROR(get_logger(), "Failed to build trajectory from path points");
    return std::nullopt;
  }

  // Attach orientation for all the points
  trajectory->align_orientation_with_trajectory_direction();

  const auto s_path_start = utils::get_arc_length_on_path(
    extended_lanelet_sequence, path_points_with_lane_id, extended_arc_length + s_start);
  const auto s_path_end = utils::get_arc_length_on_path(
    extended_lanelet_sequence, path_points_with_lane_id, extended_arc_length + s_end);

  // Refine the trajectory by cropping
  if (trajectory->length() - s_path_end > 0) {
    trajectory->crop(0., s_path_end);
  }

  // Check if the goal point is in the search range
  // Note: We only see if the goal is approaching the tail of the path.
  const auto distance_to_goal = autoware_utils::calc_distance2d(
    trajectory->compute(trajectory->length()), planner_data_.goal_pose);

  if (distance_to_goal < params.smooth_goal_connection.search_radius_range) {
    auto refined_path = utils::modify_path_for_smooth_goal_connection(
      *trajectory, planner_data_, params.smooth_goal_connection.search_radius_range,
      params.smooth_goal_connection.pre_goal_offset);

    if (refined_path) {
      refined_path->align_orientation_with_trajectory_direction();
      *trajectory = *refined_path;
    }
  }

  if (trajectory->length() - s_path_start > 0) {
    trajectory->crop(s_path_start, trajectory->length() - s_path_start);
  }

  // Compose the polished path
  PathWithLaneId finalized_path_with_lane_id{};
  finalized_path_with_lane_id.points = trajectory->restore();

  if (finalized_path_with_lane_id.points.empty()) {
    RCLCPP_ERROR(get_logger(), "Finalized path points are empty after cropping");
    return std::nullopt;
  }

  // Set header which is needed to engage
  finalized_path_with_lane_id.header.frame_id = planner_data_.route_frame_id;
  finalized_path_with_lane_id.header.stamp = now();

  const auto [left_bound, right_bound] = utils::get_path_bounds(
    extended_lanelet_sequence,
    std::max(0., extended_arc_length + s_start - vehicle_info_.max_longitudinal_offset_m),
    extended_arc_length + s_end + vehicle_info_.max_longitudinal_offset_m);
  finalized_path_with_lane_id.left_bound = left_bound;
  finalized_path_with_lane_id.right_bound = right_bound;

  return finalized_path_with_lane_id;
}

}  // namespace autoware::minimum_rule_based_planner
