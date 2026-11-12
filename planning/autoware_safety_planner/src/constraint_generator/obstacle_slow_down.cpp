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

#include "obstacle_slow_down.hpp"

#include "../utils/frenet_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware::safety_planner::experimental
{

namespace
{

namespace slow_down = obstacle_slow_down;
using autoware_perception_msgs::msg::ObjectClassification;
using slow_down::ObstacleFilteringParam;

//! [m] spacing of the TrajectoryPoints handed to the slow down planner
constexpr double RESAMPLE_INTERVAL_M = 0.5;
//! [m] spacing of the centerline the region of a zone is built along
constexpr double REGION_SAMPLE_INTERVAL_M = 1.0;

std::uint8_t to_label(const std::string & object_type)
{
  static const std::map<std::string, std::uint8_t, std::less<>> labels{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK},     {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN}};
  return labels.at(object_type);
}

//! Each class is its own generated type, hence the template
template <typename ClassParams>
slow_down::ObjectTypeSpecificParams to_object_type_specific_params(const ClassParams & p)
{
  const auto to_interpolation = [](const auto & q) {
    return slow_down::VelocityInterpolationParam{
      q.min_lat_margin, q.max_lat_margin, q.min_ego_velocity, q.max_ego_velocity};
  };
  constexpr auto LEFT = static_cast<std::underlying_type_t<slow_down::Side>>(slow_down::Side::Left);
  constexpr auto RIGHT =
    static_cast<std::underlying_type_t<slow_down::Side>>(slow_down::Side::Right);
  constexpr auto MOVING =
    static_cast<std::underlying_type_t<slow_down::Motion>>(slow_down::Motion::Moving);
  constexpr auto STATIC =
    static_cast<std::underlying_type_t<slow_down::Motion>>(slow_down::Motion::Static);

  slow_down::ObjectTypeSpecificParams out;
  out.velocity_params[LEFT][MOVING] = to_interpolation(p.left.moving);
  out.velocity_params[LEFT][STATIC] = to_interpolation(p.left.stationary);
  out.velocity_params[RIGHT][MOVING] = to_interpolation(p.right.moving);
  out.velocity_params[RIGHT][STATIC] = to_interpolation(p.right.stationary);
  out.wheel_off_track_scale = p.wheel_off_track_scale;
  return out;
}

slow_down::SlowDownParameters to_slow_down_parameters(const Params & params)
{
  const auto & p = params.obstacle_slow_down;
  slow_down::SlowDownParameters out;

  auto & filtering = out.obstacle_filtering;
  for (const auto & object_type : p.obstacle_filtering.object_types) {
    filtering.object_types.push_back(to_label(object_type));
  }
  filtering.min_lat_margin = p.obstacle_filtering.min_lat_margin;
  filtering.max_lat_margin = p.obstacle_filtering.max_lat_margin;
  filtering.lat_hysteresis_margin = p.obstacle_filtering.lat_hysteresis_margin;
  filtering.max_lat_velocity = p.obstacle_filtering.max_lat_velocity;
  filtering.successive_num_to_entry_slow_down_condition =
    static_cast<int>(p.obstacle_filtering.successive_num_to_entry_slow_down_condition);
  filtering.successive_num_to_exit_slow_down_condition =
    static_cast<int>(p.obstacle_filtering.successive_num_to_exit_slow_down_condition);

  auto & planning = out.slow_down_planning;
  const auto & q = p.slow_down_planning;
  planning.slow_down_min_acc = q.slow_down_min_acc;
  planning.slow_down_min_jerk = q.slow_down_min_jerk;
  planning.lpf_gain_slow_down_vel = q.lpf_gain_slow_down_vel;
  planning.lpf_gain_lateral_distance = q.lpf_gain_lateral_distance;
  planning.lpf_gain_dist_to_slow_down = q.lpf_gain_dist_to_slow_down;
  planning.time_margin_on_target_velocity = q.time_margin_on_target_velocity;
  planning.moving_object_speed_threshold = q.moving_object_speed_threshold;
  planning.moving_object_hysteresis_range = q.moving_object_hysteresis_range;
  const auto & classes = q.object_type_specified_params;
  auto & per_class = planning.object_type_specific_params;
  per_class[ObjectClassification::UNKNOWN] = to_object_type_specific_params(classes.unknown);
  per_class[ObjectClassification::CAR] = to_object_type_specific_params(classes.car);
  per_class[ObjectClassification::TRUCK] = to_object_type_specific_params(classes.truck);
  per_class[ObjectClassification::BUS] = to_object_type_specific_params(classes.bus);
  per_class[ObjectClassification::TRAILER] = to_object_type_specific_params(classes.trailer);
  per_class[ObjectClassification::MOTORCYCLE] = to_object_type_specific_params(classes.motorcycle);
  per_class[ObjectClassification::BICYCLE] = to_object_type_specific_params(classes.bicycle);
  per_class[ObjectClassification::PEDESTRIAN] = to_object_type_specific_params(classes.pedestrian);

  auto & polygon = out.trajectory_polygon;
  const auto & r = p.trajectory_polygon;
  polygon.decimate_trajectory_step_length = r.decimate_trajectory_step_length;
  polygon.goal_extended_trajectory_length = r.goal_extended_trajectory_length;
  polygon.ego_nearest_dist_threshold = r.ego_nearest_dist_threshold;
  polygon.ego_nearest_yaw_threshold = r.ego_nearest_yaw_threshold;
  polygon.enable_to_consider_current_pose = r.enable_to_consider_current_pose;
  polygon.time_to_convergence = r.time_to_convergence;
  return out;
}

//! Band along the reference_path over [s0, s1]
Polygon2d make_region(
  const PathPointTrajectory & path, const double s0, const double s1, const double half_width_m)
{
  const auto num_division =
    static_cast<std::size_t>(std::ceil((s1 - s0) / REGION_SAMPLE_INTERVAL_M));
  std::vector<Pose2d> centerline;
  centerline.reserve(num_division + 1);
  for (std::size_t i = 0; i <= num_division; ++i) {
    const double s = std::min(s0 + static_cast<double>(i) * REGION_SAMPLE_INTERVAL_M, s1);
    const auto position = path.compute(s).point.pose.position;
    centerline.push_back(Pose2d{Point2d{position.x, position.y}, path.azimuth(s)});
  }
  return make_band_polygon(centerline, half_width_m);
}

//! Outlines at which an object enters (inner) and leaves (outer) the slow down condition
MarkerArray make_detection_area_markers(
  const PathPointTrajectory & path, const double s_ego, const VehicleInfo & vehicle_info,
  const ObstacleFilteringParam & filtering, const builtin_interfaces::msg::Time & stamp,
  const double z)
{
  using autoware_utils_geometry::create_point;
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  MarkerArray markers;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  const double s0 = std::max(s_ego - vehicle_info.rear_overhang_m, 0.0);
  const double s1 = path.length();
  if (s1 <= s0) {
    return markers;
  }
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  const double hysteresis = filtering.lat_hysteresis_margin / 2.0;
  const auto add =
    [&](const std::string & ns, const double margin, const std_msgs::msg::ColorRGBA & color) {
      const auto region = make_region(path, s0, s1, half_width + margin);
      auto marker = create_default_marker(
        "map", stamp, ns, 0, Marker::LINE_STRIP, create_marker_scale(0.1, 0.0, 0.0), color);
      for (const auto & p : region.outer()) {
        marker.points.push_back(create_point(p.x(), p.y(), z));
      }
      if (marker.points.size() >= 2) {
        markers.markers.push_back(std::move(marker));
      }
    };
  add(
    "detection_area/entry", filtering.max_lat_margin - hysteresis,
    create_marker_color(1.0, 1.0, 0.0, 0.999));
  add(
    "detection_area/exit", filtering.max_lat_margin + hysteresis,
    create_marker_color(1.0, 0.5, 0.0, 0.6));
  return markers;
}

}  // namespace

const ObstacleSlowDownConstraintGenerator::SlowDownParameters &
ObstacleSlowDownConstraintGenerator::slow_down_parameters()
{
  if (!slow_down_parameters_) {
    auto parameters = to_slow_down_parameters(params_);
    slow_down::validate_parameters(parameters);
    slow_down_parameters_ = std::move(parameters);
  }
  return *slow_down_parameters_;
}

ConstraintGeneratorOutput ObstacleSlowDownConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & parameters = slow_down_parameters();
  const auto & path = context.reference_path;
  const auto & ego_pose = context.odometry.pose.pose;
  const double s_ego = compute_ego_frenet_state(context).s;

  // The slow down intervals are measured from s_ego
  const auto traj_points =
    make_trajectory_points(path, s_ego, ego_pose.position, RESAMPLE_INTERVAL_M);
  const PredictedObjects no_objects;
  const auto & objects = context.predicted_objects ? *context.predicted_objects : no_objects;

  const auto intervals = slow_down_planner_.plan(
    traj_points, objects, ego_pose, context.odometry.twist.twist.linear.x,
    context.acceleration.accel.accel.linear.x, context.vehicle_info,
    rclcpp::Time(context.odometry.header.stamp), parameters);

  ConstraintGeneratorOutput output;
  const double half_width_m = context.vehicle_info.vehicle_width_m / 2.0;
  for (const auto & interval : intervals) {
    const double s0 = std::clamp(s_ego + interval.start_s, 0.0, path.length());
    const double s1 = std::clamp(s_ego + interval.end_s, 0.0, path.length());
    if (s1 <= s0) {
      continue;
    }
    auto region = make_region(path, s0, s1, half_width_m);
    if (region.outer().size() < 3) {
      continue;
    }

    Constraint constraint;
    constraint.certainty = Certainty::DEFINITE;
    constraint.hardness = Hardness::HARD;
    constraint.payload = SpeedLimitZone{std::move(region), interval.velocity};
    constraint.source =
      Source{get_name(), autoware_utils_uuid::to_hex_string(interval.uuid), "slow_down"};
    output.constraints.push_back(std::move(constraint));
  }
  output.debug_markers = make_detection_area_markers(
    path, s_ego, context.vehicle_info, parameters.obstacle_filtering, context.odometry.header.stamp,
    ego_pose.position.z);
  return output;
}

}  // namespace autoware::safety_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experimental::ObstacleSlowDownConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
