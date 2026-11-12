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

#ifndef CONSTRAINT_GENERATOR__OBSTACLE_SLOW_DOWN_UTILS_HPP_
#define CONSTRAINT_GENERATOR__OBSTACLE_SLOW_DOWN_UTILS_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

namespace autoware::safety_planner::experimental::obstacle_slow_down
{

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using ObjectId = decltype(autoware_perception_msgs::msg::PredictedObject{}.object_id);

enum class Side : std::uint8_t { Left = 0, Right = 1 };
enum class Motion : std::uint8_t { Moving = 0, Static = 1 };

struct VelocityInterpolationParam
{
  double min_lat_margin{0.0};
  double max_lat_margin{0.0};
  double min_ego_velocity{0.0};
  double max_ego_velocity{0.0};
};

struct ObjectTypeSpecificParams
{
  // [Side][Motion]
  std::array<std::array<VelocityInterpolationParam, 2>, 2> velocity_params{};
  double wheel_off_track_scale{0.0};
};

struct ObstacleFilteringParam
{
  std::vector<std::uint8_t> object_types{};
  double min_lat_margin{0.0};
  double max_lat_margin{0.0};
  double lat_hysteresis_margin{0.0};
  double max_lat_velocity{0.0};
  int successive_num_to_entry_slow_down_condition{0};
  int successive_num_to_exit_slow_down_condition{0};
};

struct SlowDownPlanningParam
{
  double slow_down_min_acc{0.0};   // strictly negative
  double slow_down_min_jerk{0.0};  // strictly negative
  double lpf_gain_slow_down_vel{0.0};
  double lpf_gain_lateral_distance{0.0};
  double lpf_gain_dist_to_slow_down{0.0};
  double time_margin_on_target_velocity{0.0};
  double moving_object_speed_threshold{0.0};
  double moving_object_hysteresis_range{0.0};
  // Key is ObjectClassification::label
  std::map<std::uint8_t, ObjectTypeSpecificParams> object_type_specific_params{};
};

struct TrajectoryPolygonParam
{
  double decimate_trajectory_step_length{0.0};
  double goal_extended_trajectory_length{0.0};
  double ego_nearest_dist_threshold{0.0};
  double ego_nearest_yaw_threshold{0.0};
  bool enable_to_consider_current_pose{false};
  double time_to_convergence{0.0};
};

struct SlowDownParameters
{
  ObstacleFilteringParam obstacle_filtering{};
  SlowDownPlanningParam slow_down_planning{};
  TrajectoryPolygonParam trajectory_polygon{};
};

// Arc lengths are measured from the first point of the input trajectory
struct SlowDownInterval
{
  ObjectId uuid{};
  double start_s{0.0};
  double end_s{0.0};
  double velocity{0.0};
  geometry_msgs::msg::Point start_point{};
  geometry_msgs::msg::Point end_point{};
};

// Throws std::invalid_argument on inconsistent parameters
void validate_parameters(const SlowDownParameters & params);

// Keeps the per-UUID state across cycles: construct once
class SlowDownPlanner
{
public:
  SlowDownPlanner();
  ~SlowDownPlanner();
  SlowDownPlanner(SlowDownPlanner &&) noexcept;
  SlowDownPlanner & operator=(SlowDownPlanner &&) noexcept;
  SlowDownPlanner(const SlowDownPlanner &) = delete;
  SlowDownPlanner & operator=(const SlowDownPlanner &) = delete;

  // `params` must have passed validate_parameters()
  std::vector<SlowDownInterval> plan(
    const TrajectoryPoints & trajectory, const PredictedObjects & objects,
    const geometry_msgs::msg::Pose & ego_pose, double ego_velocity, double ego_acceleration,
    const VehicleInfo & vehicle_info, const rclcpp::Time & now, const SlowDownParameters & params);

  void reset();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoware::safety_planner::experimental::obstacle_slow_down

#endif  // CONSTRAINT_GENERATOR__OBSTACLE_SLOW_DOWN_UTILS_HPP_
