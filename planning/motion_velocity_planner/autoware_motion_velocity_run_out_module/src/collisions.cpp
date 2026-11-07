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

#include "footprints.hpp"
#include "parameters.hpp"
#include "types.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/trajectory_time_overlap/overlap.hpp>
#include <autoware/universe_utils/math/normalization.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

namespace
{
bool is_same_direction(const TimeOverlapInterval & ego, const Parameters & params)
{
  return -params.collision_same_direction_angle_threshold < ego.first_intersection.yaw_diff &&
         ego.first_intersection.yaw_diff < params.collision_same_direction_angle_threshold;
}

bool is_opposite_direction(const TimeOverlapInterval & ego, const Parameters & params)
{
  return M_PI - params.collision_opposite_direction_angle_threshold <
           ego.first_intersection.yaw_diff &&
         ego.first_intersection.yaw_diff <
           M_PI + params.collision_opposite_direction_angle_threshold;
}
}  // namespace

void calculate_overlapping_collision(
  Collision & c, const TimeOverlapInterval & ego, const TimeOverlapInterval & object,
  const Parameters & params)
{
  c.type = collision;
  c.ego_collision_time = ego.first_intersection.ego_time;
  // TODO(Maxime): can unify the logic ? (whatever the angle we can refine the collision time
  // calculation within the overlap)
  if (is_same_direction(ego, params)) {
    const auto object_is_first_with_margin =
      ego.first_intersection.object_time <
      ego.first_intersection.ego_time + params.collision_time_margin;
    const auto object_is_faster_than_ego = ego.first_intersection.vel_diff < 0;
    if (object_is_faster_than_ego && object_is_first_with_margin) {
      c.type = no_collision;
      c.explanation = " no collision will happen because object is faster and enter first";
    } else {
      // adjust the collision time based on the time and velocity difference
      const auto time_margin =
        std::abs(ego.first_intersection.object_time - ego.first_intersection.ego_time);
      const auto near_zero_ego_vel = std::abs(ego.first_intersection.ego_vel) < 1e-3;
      const auto catchup_time =
        near_zero_ego_vel
          ? 0.0
          : (time_margin * ego.first_intersection.vel_diff) / ego.first_intersection.ego_vel;
      c.ego_collision_time += std::max(0.0, catchup_time);
      std::stringstream ss;
      ss << std::setprecision(2) << "coll_t = ego_enter_time[" << ego.first_intersection.ego_time
         << "]+enter_t_diff[" << time_margin << "]*v_diff[" << ego.first_intersection.vel_diff
         << "]/ego_vel[" << ego.first_intersection.ego_vel << "]";
      c.explanation = ss.str();
    }
  } else if (is_opposite_direction(ego, params)) {
    // predict time when collision would occur by finding time when arc lengths are equal
    const auto overlap_length =
      ego.last_intersection.arc_length - ego.first_intersection.arc_length;
    const auto ego_overlap_duration =
      ego.last_intersection.ego_time - ego.first_intersection.ego_time;
    const auto object_overlap_duration =
      object.last_intersection.object_time - object.first_intersection.object_time;
    const auto ego_vel = overlap_length / ego_overlap_duration;
    const auto obj_vel = overlap_length / object_overlap_duration;
    const auto lon_buffer = std::min(overlap_length, 4.0);
    const auto collision_time_within_overlap = (overlap_length - lon_buffer) / (ego_vel + obj_vel);
    // TODO(Maxime): we need to correctly account for the agents' longitudinal offsets
    c.ego_collision_time += collision_time_within_overlap;
    std::stringstream ss;
    ss << std::setprecision(2) << "coll_t = ego_enter_time[" << ego.first_intersection.ego_time
       << "]+coll_t_within_overlap[" << collision_time_within_overlap << "]";
    c.explanation = ss.str();
  }
}

Collision calculate_collision(
  const TimeOverlapInterval & ego, const TimeOverlapInterval & object, const Parameters & params)
{
  Collision c(ego, object);
  const auto is_overlapping_at_same_time = ego.overlaps(object, params.collision_time_margin);
  const auto & ignore_params = params.ignore_collision_conditions;
  const auto clamped_ego_enter_time = std::clamp(
    ego.from, ignore_params.if_ego_arrives_first.margin.ego_enter_times.front(),
    ignore_params.if_ego_arrives_first.margin.ego_enter_times.back());
  const auto passing_margin = interpolation::lerp(
    ignore_params.if_ego_arrives_first.margin.ego_enter_times,
    ignore_params.if_ego_arrives_first.margin.time_margins, clamped_ego_enter_time);
  const auto is_ignored_ego_arrives_first =
    !is_opposite_direction(ego, params) && ignore_params.if_ego_arrives_first.enable &&
    is_overlapping_at_same_time && (ego.from + passing_margin) < object.from &&
    ego.to - ego.from <= ignore_params.if_ego_arrives_first.max_overlap_duration;
  const auto is_ignored_ego_arrives_first_and_cannot_stop =
    !is_opposite_direction(ego, params) &&
    ignore_params.if_ego_arrives_first_and_cannot_stop.enable && ego.from < object.from &&
    is_overlapping_at_same_time &&
    ego.from < ignore_params.if_ego_arrives_first_and_cannot_stop.calculated_stop_time_limit;
  if (is_ignored_ego_arrives_first) {
    c.type = ignored_collision;
    std::stringstream ss;
    ss << std::setprecision(2) << "ignore since ego arrives first (" << ego.from << " < "
       << object.from << "), including with margin (" << passing_margin
       << ") and ego overlap bellow max (" << ego.to - ego.from << " < "
       << ignore_params.if_ego_arrives_first.max_overlap_duration << ")";
    c.explanation += ss.str();
  } else if (is_ignored_ego_arrives_first_and_cannot_stop) {
    c.type = ignored_collision;
    std::stringstream ss;
    ss << std::setprecision(2) << "ignore ego arrives first (" << ego.from << " < " << object.from
       << "), and does not have time to stop ("
       << ignore_params.if_ego_arrives_first_and_cannot_stop.calculated_stop_time_limit << ")";
    c.explanation += ss.str();
  } else if (is_overlapping_at_same_time) {
    calculate_overlapping_collision(c, ego, object, params);
  } else if (ego.to < object.from) {
    c.type = pass_first_no_collision;
  } else {
    c.type = no_collision;
  }
  return c;
}

std::vector<Collision> calculate_interval_collisions(
  const std::vector<TimeOverlapIntervalPair> & intervals, const Parameters & params)
{
  std::vector<Collision> collisions;
  collisions.reserve(intervals.size());
  for (const auto & interval : intervals) {
    collisions.push_back(calculate_collision(interval.ego, interval.object, params));
  }
  return collisions;
}

std::vector<TimeOverlapIntervalPair> filter_time_overlap_intervals(
  const std::vector<TimeOverlapIntervalPair> & intervals, const FilteringData & filtering_data,
  const double min_arc_length, const ObjectParameters & params)
{
  std::vector<TimeOverlapIntervalPair> filtered_overlap_intervals;
  for (const auto & interval : intervals) {
    const auto is_before_min_arc_length =
      interval.ego.first_intersection.arc_length <= min_arc_length;
    const auto can_be_ignored =
      interval.ego.first_intersection.ego_time >= params.start_ignore_collisions_time &&
      interval.ego.first_intersection.arc_length >= params.start_ignore_collisions_distance;
    const auto ignored =
      can_be_ignored &&
      !filtering_data.ignore_collisions_rtree.is_geometry_disjoint_from_rtree_polygons(
        interval.ego.first_intersection.intersection, filtering_data.ignore_collisions_polygons);
    if (!is_before_min_arc_length && !ignored) {
      filtered_overlap_intervals.push_back(interval);
    }
  }
  return filtered_overlap_intervals;
}

void calculate_object_collisions(
  Object & object, const TrajectoryCornerFootprint & ego_footprint,
  const FilteringDataPerLabel & filtering_data, const double min_arc_length,
  const Parameters & params)
{
  for (const auto & predicted_path_footprint : object.predicted_path_footprints) {
    // combining overlap intervals over all predicted paths gives bad results
    // the current way to calculate collisions independently for each predicted path is best
    const auto time_overlap_intervals =
      trajectory_time_overlap::calculate_ego_and_object_time_overlap_intervals(
        ego_footprint, predicted_path_footprint);
    const auto combined_time_overlap_intervals =
      trajectory_time_overlap::combine_time_overlap_intervals(
        time_overlap_intervals, params.collision_time_overlap_tolerance);
    const auto filtered_time_overlap_intervals = filter_time_overlap_intervals(
      combined_time_overlap_intervals, filtering_data[object.label], min_arc_length,
      params.object_parameters_per_label[object.label]);
    const auto collisions = calculate_interval_collisions(filtered_time_overlap_intervals, params);
    for (const auto & c : collisions) {
      const auto is_after_overlap = c.ego_collision_time > c.ego_time_interval.to;
      // objects can have multiple predicted paths that overlap and cause the same collisions
      const auto is_duplicate_collision =
        std::find_if(
          object.collisions.begin(), object.collisions.end(), [&](const Collision & collision) {
            return collision.ego_collision_time == c.ego_collision_time &&
                   collision.type == c.type &&
                   collision.object_time_interval.from == c.object_time_interval.from &&
                   collision.object_time_interval.to == c.object_time_interval.to &&
                   collision.ego_time_interval.from == c.ego_time_interval.from &&
                   collision.ego_time_interval.to == c.ego_time_interval.to;
          }) != object.collisions.end();
      if (!is_after_overlap && !is_duplicate_collision) {
        object.collisions.push_back(c);
      }
    }
  }
}

void calculate_collisions(
  std::vector<Object> & objects, const TrajectoryCornerFootprint & ego_footprint,
  const FilteringDataPerLabel & filtering_data, const double min_arc_length,
  const Parameters & params)
{
  for (auto & object : objects) {
    calculate_object_collisions(object, ego_footprint, filtering_data, min_arc_length, params);
  }
}
}  // namespace autoware::motion_velocity_planner::run_out
