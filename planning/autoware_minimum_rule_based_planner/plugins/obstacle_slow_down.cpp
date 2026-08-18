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

#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace
{
using autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils::Motion;
using autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils::
  ObjectTypeSpecificParams;
using autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils::Side;
using autoware_perception_msgs::msg::ObjectClassification;

// 元実装 SlowDownPlanningParam の object_type_specified_params 読み込みの移植。
// 型別ネスト構造は generate_parameter_library で扱えないため node の param を直接読む。
// 型ごとのパラメータが未定義なら default にフォールバックする
std::unordered_map<std::string, ObjectTypeSpecificParams> load_object_type_specific_params(
  rclcpp::Node & node)
{
  const std::string param_prefix = "obstacle_slow_down.object_type_specified_params.";

  static const std::unordered_map<uint8_t, std::string> object_types_maps = {
    {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
    {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
    {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
    {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"},
    {ObjectClassification::HAZARD, "hazard"},   {ObjectClassification::ANIMAL, "animal"}};
  static const std::unordered_map<Side, std::string> side_to_string_map = {
    {Side::Left, "left"}, {Side::Right, "right"}};
  static const std::unordered_map<Motion, std::string> motion_to_string_map = {
    {Motion::Moving, "moving"}, {Motion::Static, "static"}};

  const auto get_object_parameter =
    [&](const std::string & object_label, const std::string & param) -> double {
    try {
      return autoware_utils::get_or_declare_parameter<double>(
        node, param_prefix + object_label + "." + param);
    } catch (const std::exception &) {
      return autoware_utils::get_or_declare_parameter<double>(
        node, param_prefix + "default." + param);
    }
  };

  const auto to_param_name = [&](const Side s, const Motion m, const std::string & str) {
    return side_to_string_map.at(s) + "." + motion_to_string_map.at(m) + "." + str;
  };

  std::unordered_map<std::string, ObjectTypeSpecificParams> param_per_object_type;
  for (const auto & [_, type_str] : object_types_maps) {
    ObjectTypeSpecificParams param{};
    for (const auto side : {Side::Left, Side::Right}) {
      for (const auto motion : {Motion::Moving, Motion::Static}) {
        auto & p = param.get_velocity_param(side, motion);
        p.min_lat_margin =
          get_object_parameter(type_str, to_param_name(side, motion, "min_lat_margin"));
        p.max_lat_margin =
          get_object_parameter(type_str, to_param_name(side, motion, "max_lat_margin"));
        p.min_ego_velocity =
          get_object_parameter(type_str, to_param_name(side, motion, "min_ego_velocity"));
        p.max_ego_velocity =
          get_object_parameter(type_str, to_param_name(side, motion, "max_ego_velocity"));
      }
    }
    param.wheel_off_track_scale = get_object_parameter(type_str, "wheel_off_track_scale");
    param_per_object_type.emplace(type_str, param);
  }
  return param_per_object_type;
}
}  // namespace

namespace autoware::minimum_rule_based_planner::plugin
{
using obstacle_slow_down_utils::insert_slowdown;
using obstacle_slow_down_utils::SlowDownInput;

void ObstacleSlowDown::on_initialize(const MinimumRuleBasedPlannerParams & params)
{
  planner_ =
    std::make_unique<SlowDownPlanner>(context_->vehicle_info, get_node_ptr()->get_logger());
  update_params(params);
  planner_->set_object_type_specific_params(load_object_type_specific_params(*get_node_ptr()));

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      get_node_ptr(), "backup_planner_obstacle_slow_down");
}

void ObstacleSlowDown::update_params(const MinimumRuleBasedPlannerParams & params)
{
  params_ = params.obstacle_slow_down;
  planner_->update_params(params_);
  // 型別ネストの object_type_specified_params は起動時のみ読み込み(元実装同様、動的更新の対象外)
}

void ObstacleSlowDown::run(TrajectoryPoints & traj_points, const ModifierData & modifier_data)
{
  if (!params_.enable) {
    return;
  }
  if (
    !modifier_data.odometry_ptr || !modifier_data.predicted_objects_ptr ||
    !modifier_data.acceleration_ptr) {
    return;
  }

  // pretty_build validates the input (fails for less than 2 points or too short trajectory)
  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(traj_points);
  if (!trajectory_opt) {
    return;
  }
  auto & trajectory = *trajectory_opt;

  SlowDownInput input;
  input.predicted_objects = modifier_data.predicted_objects_ptr;
  input.current_pose = modifier_data.odometry_ptr->pose.pose;
  input.ego_vel = modifier_data.odometry_ptr->twist.twist.linear.x;
  input.ego_acc = modifier_data.acceleration_ptr->accel.accel.linear.x;
  // 予測物体の姿勢補間の基準時刻。壁時計ではなく入力データ(odometry)の時刻に揃える
  // (bag 再生や処理遅延で now() と乖離するため)
  input.current_time = rclcpp::Time(modifier_data.odometry_ptr->header.stamp);

  // 本家は VelocityPlanningResult を返して motion_velocity_planner ノード側で適用するが、
  // modifier プラグインでは自分で軌道に適用する
  const auto result = planner_->plan(trajectory, input);
  if (result.plans.empty()) {
    return;
  }

  for (const auto & plan : result.plans) {
    insert_slowdown(trajectory, plan.interval);
    // planning factor(減速理由の外部通知)。減速開始位置が軌道範囲外の場合は出力しない
    if (plan.start_pose) {
      add_planning_factor(
        result.traj_points, input.current_pose, *plan.start_pose, plan.end_pose,
        result.is_driving_forward, plan.interval.velocity, plan.obstacle);
    }
  }
  traj_points = trajectory.restore();
}

void ObstacleSlowDown::add_planning_factor(
  const TrajectoryPoints & slow_down_traj_points, const geometry_msgs::msg::Pose & current_pose,
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose,
  const bool is_driving_forward, const double slow_down_vel, const SlowDownObstacle & obstacle)
{
  autoware_internal_planning_msgs::msg::SafetyFactor safety_factor;
  // TODO(Yuki TAKAGI): set correct type after pointcloud slow down feature is improved.
  // (元実装のコメントを踏襲)
  safety_factor.type = autoware_internal_planning_msgs::msg::SafetyFactor::UNKNOWN;
  safety_factor.object_id = obstacle.uuid;
  safety_factor.points = {obstacle.pose.position};
  safety_factor.is_safe = false;

  autoware_internal_planning_msgs::msg::SafetyFactorArray safety_factor_array;
  safety_factor_array.factors = {safety_factor};
  safety_factor_array.is_safe = false;

  planning_factor_interface_->add(
    slow_down_traj_points, current_pose, start_pose, end_pose, PlanningFactor::SLOW_DOWN,
    safety_factor_array, is_driving_forward, slow_down_vel, slow_down_vel);
}

}  // namespace autoware::minimum_rule_based_planner::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::minimum_rule_based_planner::plugin::ObstacleSlowDown,
  autoware::minimum_rule_based_planner::plugin::PluginInterface)
