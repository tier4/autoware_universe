// Copyright 2022 TIER IV, Inc.
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

#include "start_goal_planner.hpp"

#include "../path_planner.hpp"
#include "clothoid_pull_generater.hpp"

#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{
std::vector<PathPointWithLaneId> generate_trajectory_from_points(
  std::vector<geometry_msgs::msg::Point> points, PathPointWithLaneId goal)
{
  std::vector<PathPointWithLaneId> trajectory;
  if (points.size() < 2) {
    return trajectory;
  }
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    const auto & prev = points[i - 1];
    const auto & curr = points[i];
    PathPointWithLaneId point = goal;
    point.point.pose.position = curr;
    point.point.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(std::atan2(curr.y - prev.y, curr.x - prev.x));
    trajectory.push_back(point);
  }
  return trajectory;
}

std::vector<PathPointWithLaneId> calc_goal_planner_start_poses(
  const PathPointTrajectory & trajectory, const Pose & goal_pose, const lanelet::Id & goal_lane_id,
  const StartGoalPlannerParams & params)
{
  auto contain_goal_lane_id = [&](const PathPointWithLaneId & point) {
    const auto & ids = point.lane_ids;
    return std::find(ids.begin(), ids.end(), goal_lane_id) != ids.end();
  };

  std::vector<PathPointWithLaneId> candidates;
  const auto & search_radius_range = params.search_radius_range;
  const auto & pre_goal_offset = params.pre_goal_offset;

  for (double s = search_radius_range; s > 0; s -= 0.1) {
    auto outside_circle = [&](const PathPointWithLaneId & point) {
      return autoware_utils::calc_distance2d(point.point.pose, goal_pose) > s;
    };

    auto closest_to_goal = autoware::experimental::trajectory::closest_with_constraint(
      trajectory, goal_pose, contain_goal_lane_id);

    // If no point with the goal lane ID exists in the trajectory (e.g. goal is on an adjacent
    // lane), fall back to the geometrically closest point so the goal connection still applies.
    if (!closest_to_goal) {
      closest_to_goal = autoware::experimental::trajectory::closest(trajectory, goal_pose);
    }

    auto cropped_path = autoware::experimental::trajectory::crop(trajectory, 0, *closest_to_goal);

    auto intervals =
      autoware::experimental::trajectory::find_intervals(cropped_path, outside_circle, 10);

    std::vector<PathPointWithLaneId> goal_connected_trajectory_points;

    if (!intervals.empty()) {
      auto cropped =
        autoware::experimental::trajectory::crop(cropped_path, 0, intervals.back().end);
      goal_connected_trajectory_points = cropped.restore();
    } else if (cropped_path.length() > pre_goal_offset) {
      // If distance from start to goal is smaller than refine_goal_search_radius_range and start is
      // farther from goal than pre_goal, we just connect start, pre_goal, and goal.
      goal_connected_trajectory_points = {cropped_path.compute(0)};
    }

    candidates.push_back(
      goal_connected_trajectory_points.empty() ? cropped_path.compute(0)
                                               : goal_connected_trajectory_points.back());
  }

  return candidates;
}

bool is_trajectory_inside_lanelets_or_areas(
  const PathPointTrajectory & trajectory, const lanelet::ConstLanelets & lanelets,
  const lanelet::Areas & areas)
{
  auto is_in_lanelets = [&](const geometry_msgs::msg::Pose & pose) {
    return std::any_of(lanelets.begin(), lanelets.end(), [&](const auto & lane) {
      return lanelet::utils::isInLanelet(pose, lane);
    });
  };
  auto is_in_areas = [&](const geometry_msgs::msg::Pose & pose) {
    const lanelet::BasicPoint2d point(pose.position.x, pose.position.y);
    return std::any_of(areas.begin(), areas.end(), [&](const auto & area) {
      return lanelet::geometry::within(
        point, lanelet::traits::to2D(area.outerBoundPolygon()).basicPolygon());
    });
  };

  const auto points = trajectory.restore();
  return std::none_of(points.begin(), points.end(), [&](const auto & point) {
    return !is_in_lanelets(point.point.pose) && !is_in_areas(point.point.pose);
  });
}
}  // namespace

StartGoalPlanner::StartGoalPlanner(
  const rclcpp::Logger & logger, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const StartGoalPlannerParams & params, const VehicleInfo & vehicle_info)
: logger_(logger),
  time_keeper_(std::move(time_keeper)),
  params_(params),
  vehicle_info_(vehicle_info)
{
}

void StartGoalPlanner::set_route_data(const RouteData & route_data)
{
  route_data_ = route_data;
}

void StartGoalPlanner::update_params(const StartGoalPlannerParams & params)
{
  params_ = params;
}

std::optional<PathPointTrajectory> StartGoalPlanner::plan(
  const PathPointTrajectory & trajectory, const double & s_path_end)
{
  judge_goal_planner_act(trajectory, s_path_end);
  judge_start_planner_act();

  if (!start_planner_act && !goal_planner_act) {
    return std::nullopt;  // StartGoalPlanner is not applied. normal termination
  }

  const auto goal_pose_candidates = get_goal_pose(trajectory);
  const auto start_pose_candidates = get_start_pose(trajectory);
  if (!start_pose_candidates || !goal_pose_candidates) {
    return std::nullopt;
  }

  const auto available_area = get_available_area(trajectory);
  if (available_area.lanelets.empty() && available_area.areas.empty()) {
    return std::nullopt;
  }

  const auto candidate_trajectories =
    generate_pull_trajectories(*start_pose_candidates, *goal_pose_candidates);
  if (!candidate_trajectories) {
    return std::nullopt;
  }

  const auto pull_trajectory = evaluate_trajectory(*candidate_trajectories, available_area);
  if (!pull_trajectory) {
    return std::nullopt;
  }

  const auto refined_trajectory = connect_pull_trajectory(trajectory, *pull_trajectory);
  if (!refined_trajectory) {
    return std::nullopt;
  }

  return refined_trajectory;
}

void StartGoalPlanner::judge_start_planner_act()
{
  start_planner_act = false;
}

void StartGoalPlanner::judge_goal_planner_act(
  const PathPointTrajectory & trajectory, const double & s_path_end)
{
  const auto s_path_end_clamped = std::min(trajectory.length(), s_path_end);
  const auto distance_to_goal =
    autoware_utils::calc_distance2d(trajectory.compute(s_path_end_clamped), route_data_.goal_pose);

  goal_planner_act = distance_to_goal < params_.search_radius_range;
}

StartGoalPlanner::AvailableArea StartGoalPlanner::get_available_area(
  const PathPointTrajectory & trajectory)
{
  const auto base_lanelets =
    utils::extract_lanelets_from_trajectory(trajectory, route_data_.lanelet_map_ptr);
  const auto & lanelet_map_ptr = route_data_.lanelet_map_ptr;
  const auto & routing_graph_ptr = route_data_.routing_graph_ptr;

  AvailableArea available;
  available.lanelets = base_lanelets;

  auto try_add_lanelet = [&](const lanelet::ConstLanelet & candidate) {
    if (
      std::find(available.lanelets.begin(), available.lanelets.end(), candidate) ==
      available.lanelets.end()) {
      available.lanelets.push_back(candidate);
    }
  };
  auto try_add_area = [&](const lanelet::Area & candidate) {
    if (
      std::find(available.areas.begin(), available.areas.end(), candidate) ==
      available.areas.end()) {
      available.areas.push_back(candidate);
    }
  };

  // Lanelets reachable from base_lanelets by a lane change.
  for (const auto & ll : base_lanelets) {
    for (const auto & neighbor : routing_graph_ptr->besides(ll)) {
      try_add_lanelet(neighbor);
    }
  }

  // Road-shoulder lanelets sharing a boundary linestring with the lanelets collected so far.
  const lanelet::ConstLanelets lanelets_after_lane_change = available.lanelets;
  for (const auto & ll : lanelets_after_lane_change) {
    for (const auto & other : lanelet_map_ptr->laneletLayer.findUsages(ll.leftBound())) {
      if (
        other.rightBound() == ll.leftBound() &&
        autoware::experimental::lanelet2_utils::is_shoulder_lane(other)) {
        try_add_lanelet(other);
      }
    }
    for (const auto & other : lanelet_map_ptr->laneletLayer.findUsages(ll.rightBound())) {
      if (
        other.leftBound() == ll.rightBound() &&
        autoware::experimental::lanelet2_utils::is_shoulder_lane(other)) {
        try_add_lanelet(other);
      }
    }
  }

  // Freespace areas sharing a boundary linestring with the lanelets collected so far.
  for (const auto & ll : available.lanelets) {
    for (const auto & bound : {ll.leftBound(), ll.rightBound()}) {
      for (const auto & area : lanelet_map_ptr->areaLayer.findUsages(bound)) {
        const auto & outer = area.outerBound();
        const bool shares_bound =
          std::any_of(outer.begin(), outer.end(), [&](const auto & ls) { return ls == bound; });
        if (
          area.attributeOr(lanelet::AttributeName::Subtype, std::string()) == "freespace" &&
          shares_bound) {
          try_add_area(area);
        }
      }
    }
  }

  return available;
}

std::optional<std::vector<PathPointWithLaneId>> StartGoalPlanner::get_start_pose(
  const PathPointTrajectory & trajectory)
{
  if (goal_planner_act) {
    const auto goal_lane_id = route_data_.preferred_lanelets.back().id();
    auto candidates =
      calc_goal_planner_start_poses(trajectory, route_data_.goal_pose, goal_lane_id, params_);
    if (candidates.empty()) {
      return std::nullopt;
    }
    return candidates;
  }
  if (start_planner_act) {
    // TODO(TIER IV): determine pull-out start pose
    return std::nullopt;
  }
  return std::nullopt;
}

std::optional<std::vector<PathPointWithLaneId>> StartGoalPlanner::get_goal_pose(
  const PathPointTrajectory & trajectory)
{
  if (goal_planner_act) {
    const auto pre_goal_pose =
      autoware_utils::calc_offset_pose(route_data_.goal_pose, -params_.pre_goal_offset, 0.0, 0.0);
    auto pre_goal =
      trajectory.compute(autoware::experimental::trajectory::closest(trajectory, pre_goal_pose));
    pre_goal.point.pose = pre_goal_pose;
    return std::vector<PathPointWithLaneId>{pre_goal};
  }
  if (start_planner_act) {
    // TODO(TIER IV): determine pull-out goal pose
    return std::nullopt;
  }
  return std::nullopt;
}

std::optional<std::vector<PathPointTrajectory>> StartGoalPlanner::generate_pull_trajectories(
  const std::vector<PathPointWithLaneId> & start_pose_candidates,
  const std::vector<PathPointWithLaneId> & goal_pose_candidates)
{
  const auto max_steer_angles_rad = generate_candidate_steer_angles_rad(
    vehicle_info_.max_steer_angle_rad, params_.clothoid_steer_angle_trial_count);
  const auto max_steer_angle_rate_rad_per_sec =
    autoware_utils::deg2rad(params_.clothoid_max_steer_angle_rate_deg_per_sec);

  std::vector<PathPointTrajectory> candidate_trajectories;

  for (const auto & start_point : start_pose_candidates) {
    for (const auto & goal_point : goal_pose_candidates) {
      for (const auto max_steering_angle : max_steer_angles_rad) {
        const auto clothoid_paths = plan_clothoid_pull(
          start_point.point.pose, goal_point.point.pose, vehicle_info_.wheel_base_m,
          max_steering_angle, max_steer_angle_rate_rad_per_sec,
          params_.clothoid_reference_velocity);

        if (!clothoid_paths.has_value()) {
          continue;
        }

        for (const auto & clothoid_points : *clothoid_paths) {
          std::vector<PathPointWithLaneId> trajectory = {start_point};
          const auto interior_points = generate_trajectory_from_points(clothoid_points, goal_point);
          trajectory.insert(trajectory.end(), interior_points.begin(), interior_points.end());
          trajectory.push_back(goal_point);

          if (const auto output = autoware::experimental::trajectory::pretty_build(trajectory)) {
            candidate_trajectories.push_back(*output);
          }
        }
      }
    }
  }

  if (candidate_trajectories.empty()) {
    return std::nullopt;
  }
  return candidate_trajectories;
}

std::optional<PathPointTrajectory> StartGoalPlanner::evaluate_trajectory(
  const std::vector<PathPointTrajectory> & candidate_trajectories,
  const AvailableArea & available_area)
{
  // TODO(TIER IV): evaluate candidates with a proper KPI instead of taking the first one that fits.
  for (const auto & candidate : candidate_trajectories) {
    if (is_trajectory_inside_lanelets_or_areas(
          candidate, available_area.lanelets, available_area.areas)) {
      return candidate;
    }
  }
  return std::nullopt;
}

std::optional<PathPointTrajectory> StartGoalPlanner::connect_pull_trajectory(
  const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory)
{
  if (goal_planner_act) {
    return connect_goal_planner_trajectory(trajectory, pull_trajectory);
  }
  if (start_planner_act) {
    return connect_start_planner_trajectory(trajectory, pull_trajectory);
  }
  return std::nullopt;
}

std::optional<PathPointTrajectory> StartGoalPlanner::connect_start_planner_trajectory(
  const PathPointTrajectory & /*trajectory*/, const PathPointTrajectory & /*pull_trajectory*/)
{
  // TODO(TIER IV): connect the pull-out trajectory back to the base path
  return std::nullopt;
}

std::optional<PathPointTrajectory> StartGoalPlanner::connect_goal_planner_trajectory(
  const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory)
{
  auto pull_points = pull_trajectory.restore();

  auto goal = trajectory.compute(
    autoware::experimental::trajectory::closest(trajectory, route_data_.goal_pose));
  goal.point.pose = route_data_.goal_pose;
  goal.point.longitudinal_velocity_mps = 0.0;
  pull_points.push_back(goal);

  const auto pull_start_pose = pull_trajectory.compute(0.0).point.pose;
  const auto s_closest = autoware::experimental::trajectory::closest(trajectory, pull_start_pose);

  auto base_points = autoware::experimental::trajectory::crop(trajectory, 0, s_closest).restore();

  constexpr double kDuplicatePointEpsilon = 1e-3;
  if (
    !base_points.empty() &&
    autoware_utils::calc_distance2d(base_points.back().point.pose, pull_start_pose) <
      kDuplicatePointEpsilon) {
    base_points.pop_back();
  }

  base_points.insert(base_points.end(), pull_points.begin(), pull_points.end());

  auto connected_trajectory = autoware::experimental::trajectory::pretty_build(base_points);
  if (!connected_trajectory) {
    return std::nullopt;
  }
  connected_trajectory->align_orientation_with_trajectory_direction();
  return connected_trajectory;
}

}  // namespace autoware::minimum_rule_based_planner
