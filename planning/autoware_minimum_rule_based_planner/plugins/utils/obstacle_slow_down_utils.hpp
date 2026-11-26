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

// NOLINTNEXTLINE
#ifndef PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__UTILS__OBSTACLE_SLOW_DOWN_UTILS_HPP_
// NOLINTNEXTLINE
#define PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__UTILS__OBSTACLE_SLOW_DOWN_UTILS_HPP_

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_minimum_rule_based_planner/minimum_rule_based_planner_parameters.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <array>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::Polygon2d;
using unique_identifier_msgs::msg::UUID;
using EgoTrajectory = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using ObstacleSlowDownParams = ::minimum_rule_based_planner::Params::ObstacleSlowDown;

enum class Side { Left = 0, Right, Count };
enum class Motion { Moving = 0, Static, Count };

struct SlowDownObstacle
{
  UUID uuid{};
  rclcpp::Time stamp{};
  geometry_msgs::msg::Pose pose{};  // interpolated with the current stamp
  double velocity{};                // longitudinal velocity against ego's trajectory
  double lat_velocity{};            // lateral velocity against ego's trajectory

  double dist_to_traj_poly{};
  geometry_msgs::msg::Point front_collision_point{};
  geometry_msgs::msg::Point back_collision_point{};
  ObjectClassification classification{};
  Side side{};  // side of the obstacle relative to the ego trajectory
};

struct SlowdownInterval
{
  double from_s{};
  double to_s{};
  double velocity{};
};

struct SlowDownCarryOver
{
  double target_vel{};
  double feasible_target_vel{};
  double dist_from_obj_poly_to_traj_poly{};
  std::optional<geometry_msgs::msg::Pose> start_point{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> end_point{std::nullopt};
  Motion obstacle_motion{};
};

struct SlowDownTarget
{
  const SlowDownObstacle & obstacle;
  const std::optional<SlowDownCarryOver> & prev;
  Motion obstacle_motion{};
  double stable_dist_to_traj_poly{};
  double slow_down_vel{};
};

// parameters to calculate the slow down velocity by linear interpolation of the lateral distance
struct VelocityInterpolationParam
{
  double min_lat_margin{};
  double max_lat_margin{};
  double min_ego_velocity{};
  double max_ego_velocity{};
};

struct ObjectTypeSpecificParams
{
  std::array<
    std::array<VelocityInterpolationParam, static_cast<size_t>(Motion::Count)>,
    static_cast<size_t>(Side::Count)>
    velocity_params;
  double wheel_off_track_scale{};

  VelocityInterpolationParam & get_velocity_param(const Side side, const Motion motion)
  {
    return velocity_params[static_cast<size_t>(side)][static_cast<size_t>(motion)];
  }

  [[nodiscard]] VelocityInterpolationParam get_velocity_param(
    const Side side, const Motion motion) const
  {
    return velocity_params[static_cast<size_t>(side)][static_cast<size_t>(motion)];
  }
};

struct UuidLess
{
  bool operator()(const UUID & a, const UUID & b) const
  {
    return std::lexicographical_compare(
      std::begin(a.uuid), std::end(a.uuid), std::begin(b.uuid), std::end(b.uuid));
  }
};

struct ObstacleTrackingState
{
  double condition_duration{0.0};
  std::optional<rclcpp::Time> last_update_time{};
  bool was_slow_down{false};
  std::optional<SlowDownCarryOver> prev_slow_down{};
};

// apply the slowdown velocity to [from_s, to_s] of the velocity profile. the existing velocity is
// only lowered, never raised (e.g. a stop point inside the interval is kept).
void insert_slowdown(EgoTrajectory & trajectory, const SlowdownInterval & slowdown_interval);

struct SlowDownInput
{
  PredictedObjects::ConstSharedPtr predicted_objects;
  geometry_msgs::msg::Pose current_pose;
  double ego_vel{};
  double ego_acc{};
  rclcpp::Time current_time;
};

struct PlannedSlowDown
{
  SlowdownInterval interval;
  SlowDownObstacle obstacle;
  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose end_pose;
};

struct PlannedSlowDownWithCarryOver
{
  PlannedSlowDown plan;
  SlowDownCarryOver carry_over;
};

struct SlowDownResult
{
  std::vector<SlowDownObstacle> obstacles;
  std::vector<PlannedSlowDown> plans;
  TrajectoryPoints traj_points;
  bool is_driving_forward{true};
};

class SlowDownPlanner
{
public:
  SlowDownPlanner(const VehicleInfo & vehicle_info, const rclcpp::Logger & logger);

  void update_params(const ObstacleSlowDownParams & params);

  void set_object_type_specific_params(
    std::unordered_map<std::string, ObjectTypeSpecificParams> params);

  SlowDownResult plan(const EgoTrajectory & trajectory, const SlowDownInput & input);

  // Common
private:
  const ObjectTypeSpecificParams & get_object_param(const ObjectClassification & label) const;

  // Filter
private:
  bool is_slow_down_obstacle(const uint8_t label) const;

  bool is_slow_down_candidate(
    const PredictedObject & object, const Polygon2d & obstacle_poly,
    const EgoTrajectory & trajectory, const double dist_from_obj_poly_to_traj_poly) const;

  bool is_slow_down_required(
    ObstacleTrackingState & state, const rclcpp::Time & current_time,
    const double dist_from_obj_poly_to_traj_poly, const double lat_vel_relative_to_traj) const;

  std::vector<SlowDownObstacle> filter_slow_down_obstacle_for_predicted_object(
    const std::vector<Polygon2d> & slow_down_corridor_polys, const EgoTrajectory & trajectory,
    const SlowDownInput & input);

  const std::vector<Polygon2d> & get_ego_swept_polys(
    const EgoTrajectory & trajectory, const geometry_msgs::msg::Pose & current_pose,
    const double wheel_off_track_scale);

  SlowDownObstacle create_slow_down_obstacle_for_predicted_object(
    const EgoTrajectory & trajectory, const PredictedObject & object,
    const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> & collision_points,
    const double lon_vel_relative_to_traj, const double lat_vel_relative_to_traj,
    const rclcpp::Time & predicted_objects_stamp, const rclcpp::Time & current_time,
    const double dist_from_obj_poly_to_traj_poly);

  // Plan
private:
  std::vector<PlannedSlowDown> plan_slow_down(
    const SlowDownInput & input, const EgoTrajectory & trajectory,
    const std::vector<SlowDownObstacle> & obstacles, const double dist_to_ego,
    const bool is_driving_forward);

  SlowDownTarget make_slow_down_target(
    const SlowDownObstacle & obstacle,
    const std::optional<SlowDownCarryOver> & prev_slow_down) const;

  // returns nullopt if the obstacle should be ignored.
  std::optional<PlannedSlowDownWithCarryOver> plan_slow_down_for_obstacle(
    const SlowDownInput & input, const EgoTrajectory & trajectory, const SlowDownTarget & target,
    const double dist_to_ego, const bool is_driving_forward) const;

  // returns the stabilized slow down velocity, or nullopt if the obstacle should be ignored.
  std::optional<double> validate_slow_down_interval(
    const SlowDownTarget & target, const EgoTrajectory & trajectory,
    const SlowdownInterval & slow_down_interval) const;

  Motion determine_obstacle_motion(
    const SlowDownObstacle & obstacle, const std::optional<SlowDownCarryOver> & prev_output) const;

  // returns the slow down interval whose velocity is the feasible slow down velocity (before the
  // stabilization LPF applied in plan_slow_down_for_obstacle)
  std::optional<SlowdownInterval> calculate_distance_to_slow_down_with_constraints(
    const SlowDownInput & input, const EgoTrajectory & trajectory, const SlowDownTarget & target,
    const double dist_to_ego, const bool is_driving_forward) const;

  double calculate_feasible_slow_down_velocity(
    const EgoTrajectory & trajectory, const std::optional<SlowDownCarryOver> & prev_output,
    const double ego_vel, const double ego_acc, const double slow_down_vel,
    const double deceleration_dist, const double dist_to_slow_down_start) const;

private:
  ObstacleSlowDownParams params_;
  std::vector<uint8_t> target_object_labels_;

  std::map<UUID, ObstacleTrackingState, UuidLess> tracking_states_;

  std::unordered_map<double, std::vector<Polygon2d>> ego_swept_polys_per_off_track_scale_;

  std::unordered_map<std::string, ObjectTypeSpecificParams>
    object_type_specific_param_per_object_type_;

  VehicleInfo vehicle_info_;
  rclcpp::Logger logger_;
};

}  // namespace autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils

// NOLINTNEXTLINE
#endif  // PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__UTILS__OBSTACLE_SLOW_DOWN_UTILS_HPP_
