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

#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

namespace
{
std::vector<PathPointWithLaneId> generate_centerline_points(
  const lanelet::ConstLanelets & lanelets, const RouteContext & route_context)
{
  std::vector<PathPointWithLaneId> points;
  for (auto lanelet_it = lanelets.begin(); lanelet_it != lanelets.end(); ++lanelet_it) {
    const auto & centerline = lanelet_it->centerline();
    for (auto point_it = centerline.begin(); point_it != centerline.end(); ++point_it) {
      // Skip first point of subsequent lanelets (shared with previous lanelet's last point)
      if (point_it == centerline.begin() && lanelet_it != lanelets.begin()) {
        continue;
      }
      PathPointWithLaneId path_point{};
      path_point.lane_ids.push_back(lanelet_it->id());
      path_point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(*point_it);
      path_point.point.longitudinal_velocity_mps =
        route_context.traffic_rules_ptr
          ->speedLimit(route_context.lanelet_map_ptr->laneletLayer.get(lanelet_it->id()))
          .speedLimit.value();
      points.push_back(std::move(path_point));
    }
  }
  return points;
}
}  // namespace

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

  // Check if a lane transition to an adjacent preferred lane is needed
  auto transition_info =
    utils::detect_lane_transition(*current_lanelet_, route_context_, path_length_forward);

  // Maintain transition continuity:
  // - When a new transition has the same destination as stored, keep the original
  //   (prevents path jump when ego crosses intermediate lanes, e.g. 2→4→7)
  // - When ego reaches transition_to, keep using the stored transition
  if (transition_info) {
    if (
      active_transition_ &&
      transition_info->transition_to.id() == active_transition_->transition_to.id()) {
      // Same destination: reuse original transition for path continuity
      transition_info = active_transition_;
    } else {
      active_transition_ = transition_info;
    }
  } else if (
    active_transition_ &&
    current_lanelet_->id() == active_transition_->transition_to.id()) {
    // Ego is on transition_to lanelet - reuse stored transition for continuity
    transition_info = active_transition_;
  } else {
    active_transition_.reset();
  }

  // If a lane transition is detected, use blended path generation
  // Backward lanelets are computed from pre_transition chain (not current_lanelet_)
  // to ensure correct centerline when ego is on an intermediate lane
  if (transition_info) {
    const auto & pre_chain_start = transition_info->pre_transition_lanelets.front();
    const auto s_on_pre_chain =
      lanelet::utils::getArcCoordinates({pre_chain_start}, current_pose).length;
    const auto backward_length = std::max(
      0., path_length_backward + vehicle_info_.max_longitudinal_offset_m - s_on_pre_chain);
    const auto backward_lanelets =
      utils::get_lanelets_within_route_up_to(pre_chain_start, route_context_, backward_length);
    if (!backward_lanelets) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to get backward lanelets for transition (pre_chain_start id: %ld)",
        pre_chain_start.id());
      return std::nullopt;
    }

    RCLCPP_INFO(
      get_logger(), "Lane transition: from lanelet %ld to lanelet %ld (ego on %ld)",
      transition_info->transition_from.id(), transition_info->transition_to.id(),
      current_lanelet_->id());

    return generate_blended_path(
      *transition_info, *backward_lanelets, current_pose, path_length_backward,
      path_length_forward, params);
  }

  // No transition needed: use existing logic with closest preferred lanelet
  const auto & base_lanelet = [&]() -> const lanelet::ConstLanelet & {
    if (route_context_.preferred_lanelets.empty()) {
      return *current_lanelet_;
    }
    lanelet::ConstLanelet closest;
    if (lanelet::utils::query::getClosestLanelet(
          route_context_.preferred_lanelets, current_pose, &closest)) {
      route_context_.closest_preferred_lanelet = closest;
      return *route_context_.closest_preferred_lanelet;
    }
    return *current_lanelet_;
  }();

  lanelet::ConstLanelets lanelets{base_lanelet};
  const auto s_on_current_lanelet =
    lanelet::utils::getArcCoordinates({base_lanelet}, current_pose).length;

  const auto backward_length = std::max(
    0., path_length_backward + vehicle_info_.max_longitudinal_offset_m - s_on_current_lanelet);
  const auto backward_lanelets_within_route =
    utils::get_lanelets_within_route_up_to(base_lanelet, route_context_, backward_length);
  if (!backward_lanelets_within_route) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get backward lanelets within route for current lanelet (id: %ld)",
      base_lanelet.id());
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
    const auto prev_lanelets = route_context_.routing_graph_ptr->previous(lanelets.front());
    if (prev_lanelets.empty()) {
      break;
    }
    lanelets.insert(lanelets.begin(), prev_lanelets.front());
    backward_lanelets_length += lanelet::geometry::length2d(prev_lanelets.front());
  }

  const auto forward_length = std::max(
    0., path_length_forward + vehicle_info_.max_longitudinal_offset_m -
          (lanelet::geometry::length2d(base_lanelet) - s_on_current_lanelet));
  const auto forward_lanelets_within_route =
    utils::get_lanelets_within_route_after(base_lanelet, route_context_, forward_length);
  if (!forward_lanelets_within_route) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get forward lanelets within route for current lanelet (id: %ld)",
      base_lanelet.id());
    return std::nullopt;
  }
  lanelets.insert(
    lanelets.end(), forward_lanelets_within_route->begin(), forward_lanelets_within_route->end());

  //  Extend lanelets by forward_length even outside planned route to ensure
  //  ego footprint is inside lanelets if ego is at the end of goal lane
  auto forward_lanelets_length = lanelet::utils::getLaneletLength2d(*forward_lanelets_within_route);
  while (forward_lanelets_length < forward_length) {
    const auto next_lanelets = route_context_.routing_graph_ptr->following(lanelets.back());
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

    if (!utils::get_next_lanelet_within_route(lanelets.back(), route_context_)) {
      s_end_val = std::min(s_end_val, lanelet::utils::getLaneletLength2d(lanelets));
    }

    auto is_goal_lanelet = [&](const lanelet::ConstLanelet & ll) {
      return std::any_of(
        route_context_.goal_lanelets.begin(), route_context_.goal_lanelets.end(),
        [&](const auto & goal_ll) { return ll.id() == goal_ll.id(); });
    };

    for (auto [it, goal_arc_length] = std::make_tuple(lanelets.begin(), 0.); it != lanelets.end();
         ++it) {
      if (is_goal_lanelet(*it)) {
        goal_arc_length +=
          lanelet::utils::getArcCoordinates({*it}, route_context_.goal_pose).length;
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

  if (s_end <= s_start) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "s_end (%.2f) <= s_start (%.2f), cannot generate path",
      s_end, s_start);
    return std::nullopt;
  }

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

  {
    std::string ids_str;
    for (const auto & lanelet : lanelet_sequence.lanelets()) {
      if (!ids_str.empty()) ids_str += ", ";
      ids_str += std::to_string(lanelet.id());
    }
    RCLCPP_INFO(get_logger(), "lanelet_sequence ids: [%s]", ids_str.c_str());
  }

  std::vector<PathPointWithLaneId> path_points_with_lane_id{};

  const auto waypoint_groups = utils::get_waypoint_groups(
    lanelet_sequence, *route_context_.lanelet_map_ptr, params.waypoint_group.separation_threshold,
    params.waypoint_group.interval_margin_ratio);

  auto extended_lanelets = lanelet_sequence.lanelets();
  auto extended_arc_length = 0.;
  for (const auto & [waypoints, interval] : waypoint_groups) {
    while (interval.start + extended_arc_length < 0.) {
      const auto prev_lanelets =
        route_context_.routing_graph_ptr->previous(extended_lanelets.front());
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
        route_context_.traffic_rules_ptr
          ->speedLimit(route_context_.lanelet_map_ptr->laneletLayer.get(lane_id))
          .speedLimit.value();
      path_points_with_lane_id.push_back(std::move(path_point_with_lane_id));
    };

  const lanelet::LaneletSequence extended_lanelet_sequence(extended_lanelets);
  std::optional<size_t> overlapping_waypoint_group_index;

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
        overlapping_waypoint_group_index.reset();
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
    trajectory->compute(trajectory->length()), route_context_.goal_pose);

  if (distance_to_goal < params.smooth_goal_connection.search_radius_range) {
    auto refined_path = utils::modify_path_for_smooth_goal_connection(
      *trajectory, route_context_, params.smooth_goal_connection.search_radius_range,
      params.smooth_goal_connection.pre_goal_offset);

    if (refined_path) {
      refined_path->align_orientation_with_trajectory_direction();
      *trajectory = *refined_path;
    }
  }

  if (trajectory->length() - s_path_start > 0) {
    trajectory->crop(s_path_start, trajectory->length() - s_path_start);
  }

  if (trajectory->length() < 1e-3) {
    RCLCPP_WARN(
      get_logger(), "Trajectory length too short after cropping: %f", trajectory->length());
    return std::nullopt;
  }

  // Compose the polished path
  PathWithLaneId finalized_path_with_lane_id{};
  finalized_path_with_lane_id.points = trajectory->restore();

  if (finalized_path_with_lane_id.points.empty()) {
    RCLCPP_ERROR(get_logger(), "Finalized path points are empty after cropping");
    return std::nullopt;
  }

  // Set header which is needed to engage
  finalized_path_with_lane_id.header.frame_id = route_context_.route_frame_id;
  finalized_path_with_lane_id.header.stamp = now();

  const auto [left_bound, right_bound] = utils::get_path_bounds(
    extended_lanelet_sequence,
    std::max(0., extended_arc_length + s_start - vehicle_info_.max_longitudinal_offset_m),
    extended_arc_length + s_end + vehicle_info_.max_longitudinal_offset_m);
  finalized_path_with_lane_id.left_bound = left_bound;
  finalized_path_with_lane_id.right_bound = right_bound;

  return finalized_path_with_lane_id;
}

std::optional<PathWithLaneId> MinimumRuleBasedPlannerNode::generate_blended_path(
  const LaneTransitionInfo & transition_info, const lanelet::ConstLanelets & backward_lanelets,
  const geometry_msgs::msg::Pose & current_pose, const double path_length_backward,
  const double path_length_forward, const Params & params)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  // Build pre-transition lanelets: backward + pre_transition_lanelets
  lanelet::ConstLanelets pre_lanelets;
  pre_lanelets.insert(pre_lanelets.end(), backward_lanelets.begin(), backward_lanelets.end());
  for (const auto & ll : transition_info.pre_transition_lanelets) {
    // Avoid duplicates (backward_lanelets may overlap with pre_transition_lanelets)
    if (pre_lanelets.empty() || pre_lanelets.back().id() != ll.id()) {
      pre_lanelets.push_back(ll);
    }
  }

  // Generate centerline points for pre-transition and post-transition sequences
  const auto pre_points = generate_centerline_points(pre_lanelets, route_context_);
  const auto post_points =
    generate_centerline_points(transition_info.post_transition_lanelets, route_context_);

  if (pre_points.empty() || post_points.empty()) {
    RCLCPP_ERROR(get_logger(), "Failed to generate centerline points for lane transition");
    return std::nullopt;
  }

  // Compute blend_start_s: arc length on pre_points where transition_from lanelet starts
  double blend_start_s = 0.0;
  {
    // Sum arc lengths of pre_lanelets up to (but not including) transition_from
    double arc_len = 0.0;
    for (size_t i = 1; i < pre_points.size(); ++i) {
      const double ds = autoware_utils::calc_distance2d(
        pre_points[i - 1].point.pose.position, pre_points[i].point.pose.position);
      arc_len += ds;
      // Check if we've entered the transition_from lanelet
      if (
        !pre_points[i].lane_ids.empty() &&
        pre_points[i].lane_ids.front() == transition_info.transition_from.id() &&
        (pre_points[i - 1].lane_ids.empty() ||
         pre_points[i - 1].lane_ids.front() != transition_info.transition_from.id())) {
        blend_start_s = arc_len;
        break;
      }
    }
  }

  const double blend_length = lanelet::geometry::length2d(transition_info.transition_from);

  {
    std::string pre_ids, post_ids;
    for (const auto & ll : pre_lanelets) {
      if (!pre_ids.empty()) pre_ids += ", ";
      pre_ids += std::to_string(ll.id());
    }
    for (const auto & ll : transition_info.post_transition_lanelets) {
      if (!post_ids.empty()) post_ids += ", ";
      post_ids += std::to_string(ll.id());
    }
    RCLCPP_INFO(
      get_logger(),
      "Blended path: pre=[%s], post=[%s], blend_start=%.1f, blend_length=%.1f",
      pre_ids.c_str(), post_ids.c_str(), blend_start_s, blend_length);
  }

  // Blend centerlines
  auto blended_points =
    utils::blend_centerlines(pre_points, post_points, blend_start_s, blend_length);

  if (blended_points.empty()) {
    RCLCPP_ERROR(get_logger(), "Blended path points are empty");
    return std::nullopt;
  }

  // Build trajectory from blended points
  auto trajectory = autoware::experimental::trajectory::pretty_build(blended_points);
  if (!trajectory) {
    RCLCPP_ERROR(get_logger(), "Failed to build trajectory from blended path points");
    return std::nullopt;
  }

  trajectory->align_orientation_with_trajectory_direction();

  // Find ego's actual position on the blended trajectory by projecting current_pose
  const double ego_s =
    autoware::experimental::trajectory::closest(*trajectory, current_pose);
  const double s_start = std::max(0., ego_s - path_length_backward);
  const double s_end = std::min(ego_s + path_length_forward, trajectory->length());

  RCLCPP_INFO(
    get_logger(),
    "Blended path cropping: ego_s=%.1f, s_start=%.1f, s_end=%.1f, traj_length=%.1f",
    ego_s, s_start, s_end, trajectory->length());

  // Crop rear first, then front
  if (trajectory->length() - s_end > 0) {
    trajectory->crop(0., s_end);
  }

  // Check if goal is in range for smooth connection
  const auto distance_to_goal = autoware_utils::calc_distance2d(
    trajectory->compute(trajectory->length()), route_context_.goal_pose);

  if (distance_to_goal < params.smooth_goal_connection.search_radius_range) {
    auto refined_path = utils::modify_path_for_smooth_goal_connection(
      *trajectory, route_context_, params.smooth_goal_connection.search_radius_range,
      params.smooth_goal_connection.pre_goal_offset);

    if (refined_path) {
      refined_path->align_orientation_with_trajectory_direction();
      *trajectory = *refined_path;
    }
  }

  if (s_start > 0 && trajectory->length() - s_start > 0) {
    trajectory->crop(s_start, trajectory->length() - s_start);
  }

  if (trajectory->length() < 1e-3) {
    RCLCPP_WARN(
      get_logger(), "Blended trajectory length too short after cropping: %f",
      trajectory->length());
    return std::nullopt;
  }

  // Compose finalized path
  PathWithLaneId finalized_path{};
  finalized_path.points = trajectory->restore();

  if (finalized_path.points.empty()) {
    RCLCPP_ERROR(get_logger(), "Finalized blended path points are empty after cropping");
    return std::nullopt;
  }

  finalized_path.header.frame_id = route_context_.route_frame_id;
  finalized_path.header.stamp = now();

  // Compute bounds: use pre-transition bounds + post-transition bounds
  {
    const lanelet::LaneletSequence pre_seq(pre_lanelets);
    const auto pre_total_length = lanelet::utils::getLaneletLength2d(pre_lanelets);
    const auto [pre_left, pre_right] = utils::get_path_bounds(
      pre_seq,
      std::max(0., s_start - vehicle_info_.max_longitudinal_offset_m),
      pre_total_length);

    const lanelet::LaneletSequence post_seq(transition_info.post_transition_lanelets);
    const auto post_total_length =
      lanelet::utils::getLaneletLength2d(transition_info.post_transition_lanelets);
    const auto [post_left, post_right] = utils::get_path_bounds(
      post_seq, 0., std::min(s_end, post_total_length));

    // Concatenate bounds
    finalized_path.left_bound = pre_left;
    finalized_path.left_bound.insert(
      finalized_path.left_bound.end(), post_left.begin(), post_left.end());
    finalized_path.right_bound = pre_right;
    finalized_path.right_bound.insert(
      finalized_path.right_bound.end(), post_right.begin(), post_right.end());
  }

  return finalized_path;
}

}  // namespace autoware::minimum_rule_based_planner
