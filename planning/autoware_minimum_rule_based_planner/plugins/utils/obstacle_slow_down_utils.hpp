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

// 減速区間(軌道始点からの弧長 [m] で表す)
struct SlowdownInterval
{
  double from_s{};
  double to_s{};
  double velocity{};
};

struct SlowDownOutput
{
  double target_vel{};
  double feasible_target_vel{};
  double dist_from_obj_poly_to_traj_poly{};
  std::optional<geometry_msgs::msg::Pose> start_point{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> end_point{std::nullopt};
  Motion obstacle_motion{};
};

// シュミットトリガー(2値ヒステリシス)。
// このモジュールの「前回状態」は単一量の判定結果ではなくフレーム全体の最終判定
// (フィルタ通過可否・減速出力の有無)で決まるため、インスタンスに状態を持たせて
// 持ち回るのではなく、毎回前回状態を seed して update する
// TODO: これだとフリー関数と変わらない
struct SchmittTrigger
{
  bool is_low{false};

  bool update(const double current_val, const double high_val, const double low_val)
  {
    is_low = is_low ? !(high_val < current_val) : current_val < low_val;
    return is_low;
  }
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

// 障害物 uuid ごとにフレームを跨いで保持する状態。エントリはフィルタの粗ふるいを
// 通過した uuid に対して作られ、追跡対象から外れた uuid は毎フレーム GC される。
// フィールドごとに更新される段が異なる:
//  - condition_counter は filter 段(is_slow_down_required)で更新
//  - was_slow_down は filter 段の最後に最終結果で確定
//  - prev_output は plan 段で更新(減速出力を出したフレームだけ値を持つ)
struct ObstacleTrackingState
{
  // 減速条件の連続成立/不成立フレーム数(正: entry 側, 負: exit 側)
  int condition_counter{0};
  // 前フレームのフィルタを最終的に通過して減速対象だったか(横距離ヒステリシスの状態)
  bool was_slow_down{false};
  // 前フレームで実際に減速出力を出した場合の内容(各種 LPF と motion 判定の基準)
  std::optional<SlowDownOutput> prev_output{};
};

// estimate the lower bound of the lateral distance from the object to the trajectory polygon,
// assuming the worst case for both the object shape and the ego footprint
double calc_possible_min_dist_from_obj_to_traj_poly(
  const PredictedObject & object, const EgoTrajectory & trajectory, const double obj_s,
  const VehicleInfo & vehicle_info);

double calc_dist_to_traj_poly(
  const Polygon2d & obj_poly, const std::vector<Polygon2d> & ego_swept_polys);

// returns {longitudinal, lateral} velocity relative to the trajectory. the lateral velocity is
// positive if the object is approaching the trajectory.
std::pair<double, double> calc_vel_relative_to_traj(
  const PredictedObject & object, const EgoTrajectory & trajectory, const double obj_s);

geometry_msgs::msg::Pose get_predicted_current_pose(
  const PredictedObject & object, const rclcpp::Time & current_stamp,
  const rclcpp::Time & predicted_objects_stamp, const rclcpp::Logger & logger);

// calculate the front/back collision points (the collision vertices with the minimum/maximum arc
// length along the trajectory) between the obstacle polygon and the trajectory polygons.
// returns nullopt if there is no collision.
std::optional<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>>
calc_front_back_collision_points(
  const EgoTrajectory & trajectory, const std::vector<Polygon2d> & slow_down_corridor_polys,
  const Polygon2d & obstacle_poly);

double calc_deceleration_velocity_from_distance_to_target(
  const double max_slowdown_jerk, const double max_slowdown_accel, const double current_accel,
  const double current_velocity, const double distance_to_target);

// apply the slowdown velocity to [from_s, to_s] of the velocity profile. the existing velocity is
// only lowered, never raised (e.g. a stop point inside the interval is kept).
void insert_slowdown(EgoTrajectory & trajectory, const SlowdownInterval & slowdown_interval);

template <class T>
bool contains_uuid(const std::vector<T> & obstacles, const UUID & target_uuid)
{
  return std::any_of(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid.uuid == target_uuid.uuid;
  });
}

// ノード境界で受け取る 1 フレームぶんの入力(ModifierData のノード非依存版)
struct SlowDownInput
{
  PredictedObjects::ConstSharedPtr predicted_objects;
  geometry_msgs::msg::Pose current_pose;
  double ego_vel{};
  double ego_acc{};
  rclcpp::Time current_time;
};

// 障害物 1 件ぶんの減速計画。start/end_pose は planning factor 出力用
struct PlannedSlowDown
{
  SlowdownInterval interval;
  SlowDownObstacle obstacle;
  // 減速開始位置が軌道範囲外で点を挿入できなかった場合は nullopt(planning factor は出さない)
  std::optional<geometry_msgs::msg::Pose> start_pose;
  geometry_msgs::msg::Pose end_pose;
};

struct SlowDownResult
{
  std::vector<SlowDownObstacle> obstacles;  // フィルタを通過した減速対象障害物
  std::vector<PlannedSlowDown> plans;
  // 減速区間の始終点を挿入した点列(planning factor の距離計算用)
  TrajectoryPoints traj_points_with_boundaries;
  bool is_driving_forward{true};
};

/// @brief obstacle_slow_down のノード非依存部分。障害物のフィルタリングと減速区間の計画、
/// およびそれらに使うフレーム間状態(ヒステリシス・LPF)を保持する。
/// パラメータ読み込み・planning factor 出力・軌道への適用はプラグイン側が行う
class SlowDownPlanner
{
public:
  SlowDownPlanner(const VehicleInfo & vehicle_info, const rclcpp::Logger & logger);

  void update_params(const ObstacleSlowDownParams & params);

  void set_object_type_specific_params(
    std::unordered_map<std::string, ObjectTypeSpecificParams> params);

  SlowDownResult plan(const EgoTrajectory & trajectory, const SlowDownInput & input);

private:
  const ObjectTypeSpecificParams & get_object_param(const ObjectClassification & label) const;

  bool is_slow_down_obstacle(const uint8_t label) const;

  bool is_slow_down_candidate(
    const PredictedObject & object, const Polygon2d & obstacle_poly,
    const EgoTrajectory & trajectory, const double dist_from_obj_poly_to_traj_poly) const;

  bool is_slow_down_required(
    ObstacleTrackingState & state, const double dist_from_obj_poly_to_traj_poly,
    const double lat_vel_relative_to_traj) const;

  std::vector<SlowDownObstacle> filter_slow_down_obstacle_for_predicted_object(
    const std::vector<Polygon2d> & slow_down_corridor_polys,
    const std::vector<Polygon2d> & ego_swept_polys, const EgoTrajectory & trajectory,
    const SlowDownInput & input);

  SlowDownObstacle create_slow_down_obstacle_for_predicted_object(
    const EgoTrajectory & trajectory, const PredictedObject & object,
    const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> & collision_points,
    const double lon_vel_relative_to_traj, const double lat_vel_relative_to_traj,
    const rclcpp::Time & predicted_objects_stamp, const rclcpp::Time & current_time,
    const double dist_from_obj_poly_to_traj_poly);

  // slow_down_traj_points is shared across obstacles and gets the interval boundary points
  // inserted.
  std::vector<PlannedSlowDown> plan_slow_down(
    const SlowDownInput & input, const EgoTrajectory & trajectory,
    const std::vector<SlowDownObstacle> & obstacles, TrajectoryPoints & slow_down_traj_points,
    const double dist_to_ego, const bool is_driving_forward);

  // returns the slow down interval and the new prev_output entry, or nullopt if the obstacle
  // should be ignored.
  std::optional<std::pair<SlowdownInterval, SlowDownOutput>> plan_slow_down_for_obstacle(
    const SlowDownInput & input, const EgoTrajectory & trajectory,
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
    TrajectoryPoints & slow_down_traj_points, const double dist_to_ego,
    const bool is_driving_forward);

  // 減速区間の始終点を挿入した結果。start_idx は軌道範囲外で挿入できなかった場合 nullopt
  struct InsertedSlowDownInterval
  {
    std::optional<size_t> start_idx;
    size_t end_idx;
    double stable_slow_down_vel;
  };

  // inserts the slow down interval boundary points into slow_down_traj_points, or returns nullopt
  // if the obstacle should be ignored.
  std::optional<InsertedSlowDownInterval> insert_slow_down_interval_points(
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
    const SlowdownInterval & slow_down_interval, TrajectoryPoints & slow_down_traj_points) const;

  Motion determine_obstacle_motion(
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output) const;

  // returns the slow down interval whose velocity is the feasible slow down velocity (before the
  // stabilization LPF applied in plan_slow_down_for_obstacle)
  std::optional<SlowdownInterval> calculate_distance_to_slow_down_with_constraints(
    const SlowDownInput & input, const EgoTrajectory & trajectory,
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
    const double dist_to_ego, const bool is_driving_forward, const double slow_down_vel) const;

  double calculate_feasible_slow_down_velocity(
    const EgoTrajectory & trajectory, const std::optional<SlowDownOutput> & prev_output,
    const double ego_vel, const double ego_acc, const double slow_down_vel,
    const double deceleration_dist, const double dist_to_slow_down_start) const;

  // returns {slow down velocity, LPF-filtered lateral distance used for the calculation}
  std::pair<double, double> calculate_target_slow_down_velocity(
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
    const Motion obstacle_motion) const;

  ObstacleSlowDownParams params_;
  std::vector<uint8_t> target_object_labels_;

  std::map<UUID, ObstacleTrackingState, UuidLess> tracking_states_;

  std::unordered_map<std::string, ObjectTypeSpecificParams>
    object_type_specific_param_per_object_type_;

  VehicleInfo vehicle_info_;
  rclcpp::Logger logger_;
};

}  // namespace autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils

// NOLINTNEXTLINE
#endif  // PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__UTILS__OBSTACLE_SLOW_DOWN_UTILS_HPP_
