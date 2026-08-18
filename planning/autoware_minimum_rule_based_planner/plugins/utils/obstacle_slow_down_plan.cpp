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

// SlowDownPlanner の plan 段(plan_slow_down 以下)の実装。
// filter 段と共有するヘルパは obstacle_slow_down_utils.cpp 側にある

#include "obstacle_slow_down_utils.hpp"

#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils
{
namespace
{

// TODO: std::cerrではなくloggerに直す
// TODO(murooka) following two functions are copied from behavior_velocity_planner.
// These should be refactored. (元実装のコメントを踏襲)
double find_reach_time(
  const double jerk, const double accel, const double velocity, const double distance,
  const double t_min, const double t_max)
{
  const double j = jerk;
  const double a = accel;
  const double v = velocity;
  const double d = distance;
  const double min = t_min;
  const double max = t_max;
  auto f = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  if (f(min, j, a, v, d) > 0 || f(max, j, a, v, d) < 0) {
    throw std::logic_error("[obstacle_slow_down](find_reach_time): search range is invalid");
  }
  const double eps = 1e-5;
  const int warn_iter = 100;
  double lower = min;
  double upper = max;
  double t;
  int iter = 0;
  for (int i = 0;; i++) {
    t = 0.5 * (lower + upper);
    const double fx = f(t, j, a, v, d);
    if (std::abs(fx) < eps) {
      break;
    } else if (fx > 0.0) {
      upper = t;
    } else {
      lower = t;
    }
    iter++;
    if (iter > warn_iter)
      std::cerr << "[obstacle_slow_down](find_reach_time): current iter is over warning"
                << std::endl;
  }
  return t;
}
}  // namespace

double calc_deceleration_velocity_from_distance_to_target(
  const double max_slowdown_jerk, const double max_slowdown_accel, const double current_accel,
  const double current_velocity, const double distance_to_target)
{
  if (max_slowdown_jerk > 0 || max_slowdown_accel > 0) {
    throw std::logic_error("max_slowdown_jerk and max_slowdown_accel should be negative");
  }
  // case0: distance to target is behind ego
  if (distance_to_target <= 0) return current_velocity;
  auto ft = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  auto vt = [](const double t, const double j, const double a, const double v) {
    return j * t * t / 2.0 + a * t + v;
  };
  const double j_max = max_slowdown_jerk;
  const double a0 = current_accel;
  const double a_max = max_slowdown_accel;
  const double v0 = current_velocity;
  const double l = distance_to_target;
  const double t_const_jerk = (a_max - a0) / j_max;
  const double d_const_jerk_stop = ft(t_const_jerk, j_max, a0, v0, 0.0);
  const double d_const_acc_stop = l - d_const_jerk_stop;

  if (d_const_acc_stop < 0) {
    // case0: distance to target is within constant jerk deceleration
    // use binary search instead of solving cubic equation
    const double t_jerk = find_reach_time(j_max, a0, v0, l, 0, t_const_jerk);
    const double velocity = vt(t_jerk, j_max, a0, v0);
    return velocity;
  } else {
    const double v1 = vt(t_const_jerk, j_max, a0, v0);
    const double discriminant_of_stop = 2.0 * a_max * d_const_acc_stop + v1 * v1;
    // case3: distance to target is farther than distance to stop
    if (discriminant_of_stop <= 0) {
      return 0.0;
    }
    // case2: distance to target is within constant accel deceleration
    // solve d = 0.5*a^2+v*t by t
    const double t_acc = (-v1 + std::sqrt(discriminant_of_stop)) / a_max;
    return vt(t_acc, 0.0, a_max, v1);
  }
  return current_velocity;
}

const ObjectTypeSpecificParams & SlowDownPlanner::get_object_param(
  const ObjectClassification & label) const
{
  static const std::unordered_map<uint8_t, std::string> object_types_maps = {
    {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
    {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
    {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
    {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"},
    {ObjectClassification::HAZARD, "hazard"},   {ObjectClassification::ANIMAL, "animal"}};
  const auto & type_str = object_types_maps.at(label.label);
  return object_type_specific_param_per_object_type_.at(type_str);
}

// 元実装 plan_slow_down の移植(virtual wall・デバッグ配列の出力は未移植)
std::vector<PlannedSlowDown> SlowDownPlanner::plan_slow_down(
  const SlowDownInput & input, const EgoTrajectory & trajectory,
  const std::vector<SlowDownObstacle> & obstacles, const double dist_to_ego,
  const bool is_driving_forward)
{
  std::vector<PlannedSlowDown> plans;
  for (const auto & obstacle : obstacles) {
    auto & state = tracking_states_.at(obstacle.uuid);  // filter 段で必ず作られている
    const auto target = make_slow_down_target(obstacle, state.prev_slow_down);
    const auto result =
      plan_slow_down_for_obstacle(input, trajectory, target, dist_to_ego, is_driving_forward);
    // prev_slow_down は「前フレームで減速出力を出したか」を表すため、出さなければ消す
    state.prev_slow_down = result ? std::make_optional(result->carry_over) : std::nullopt;
    if (result) {
      plans.push_back(result->plan);
    }
  }

  // plan 対象にならなかった(フィルタで落ちた)障害物の prev_slow_down も同様に消す
  for (auto & [uuid, state] : tracking_states_) {
    if (!contains_uuid(obstacles, uuid)) {
      state.prev_slow_down.reset();
    }
  }

  return plans;
}

// 前フレームからの持ち越しに依存する量(motion 判定・横距離 LPF)をここで一度だけ確定させる
SlowDownTarget SlowDownPlanner::make_slow_down_target(
  const SlowDownObstacle & obstacle, const std::optional<SlowDownCarryOver> & prev_slow_down) const
{
  // 障害物が移動中か静止かを速度ノルムのシュミットトリガーで判定
  const auto obstacle_motion = determine_obstacle_motion(obstacle, prev_slow_down);

  const double stable_dist_to_traj_poly =
    prev_slow_down ? autoware::signal_processing::lowpassFilter(
                       obstacle.dist_to_traj_poly, prev_slow_down->dist_from_obj_poly_to_traj_poly,
                       params_.lpf_gain_lateral_distance)
                   : obstacle.dist_to_traj_poly;

  return {obstacle, prev_slow_down, obstacle_motion, stable_dist_to_traj_poly};
}

std::optional<PlannedSlowDownWithCarryOver> SlowDownPlanner::plan_slow_down_for_obstacle(
  const SlowDownInput & input, const EgoTrajectory & trajectory, const SlowDownTarget & target,
  const double dist_to_ego, const bool is_driving_forward) const
{
  // 障害物との横距離(LPF 済み)から通過時の目標速度を線形補間で決める
  const double slow_down_vel = calculate_target_slow_down_velocity(target);

  // 減速制約(min acc/jerk)と障害物の縦速度を考慮して、減速区間 [from_s, to_s] と
  // そこで実現可能な速度を計算する(遠すぎる場合は nullopt)
  const auto slow_down_interval = calculate_distance_to_slow_down_with_constraints(
    input, trajectory, target, dist_to_ego, is_driving_forward, slow_down_vel);
  if (!slow_down_interval) {
    RCLCPP_DEBUG(
      logger_, "[SlowDown] Ignore obstacle (%s) since distance to slow down is not valid",
      autoware_utils_uuid::to_hex_string(target.obstacle.uuid).c_str());
    return std::nullopt;
  }

  const auto stable_slow_down_vel =
    validate_slow_down_interval(target, trajectory, *slow_down_interval);
  if (!stable_slow_down_vel) {
    return std::nullopt;
  }

  // 軌道に適用する減速区間(弧長は軌道範囲にクランプ)
  const SlowdownInterval slowdown_interval{
    std::clamp(slow_down_interval->from_s, 0.0, trajectory.length()),
    std::clamp(slow_down_interval->to_s, 0.0, trajectory.length()), *stable_slow_down_vel};

  // 次フレームのヒステリシス・LPF の基準として持ち越す量を組み立てる。
  // target_vel は plan.interval.velocity と同値だが、持ち越し側を自己完結させるため重複させる
  SlowDownCarryOver carry_over;
  carry_over.target_vel = *stable_slow_down_vel;
  carry_over.feasible_target_vel = slow_down_interval->velocity;
  carry_over.dist_from_obj_poly_to_traj_poly = target.stable_dist_to_traj_poly;
  // 減速開始位置が軌道範囲外なら planning factor を出さないため start_point は持たせない
  if (0.0 <= slow_down_interval->from_s && slow_down_interval->from_s <= trajectory.length()) {
    carry_over.start_point = trajectory.compute(slow_down_interval->from_s).pose;
  }
  carry_over.end_point = trajectory.compute(slow_down_interval->to_s).pose;
  carry_over.obstacle_motion = target.obstacle_motion;

  // start/end_point は結果側も持ち越し側も要るので、寿命を絡ませないよう値で重複して持つ
  return PlannedSlowDownWithCarryOver{
    {slowdown_interval, target.obstacle, carry_over.start_point, *carry_over.end_point},
    carry_over};
}

// 減速区間の目標速度を前フレームと LPF して安定化し、減速が不要/区間が無効なら nullopt を返す
std::optional<double> SlowDownPlanner::validate_slow_down_interval(
  const SlowDownTarget & target, const EgoTrajectory & trajectory,
  const SlowdownInterval & slow_down_interval) const
{
  // 区間長が 0 以下、または区間終端が軌道範囲外なら棄却
  if (
    slow_down_interval.to_s <= slow_down_interval.from_s || slow_down_interval.to_s < 0.0 ||
    trajectory.length() < slow_down_interval.to_s) {
    return std::nullopt;
  }

  // 前フレームの目標速度と LPF して目標速度のチャタリングを抑える
  const double stable_slow_down_vel =
    target.prev
      ? autoware::signal_processing::lowpassFilter(
          slow_down_interval.velocity, target.prev->target_vel, params_.lpf_gain_slow_down_vel)
      : slow_down_interval.velocity;

  // 区間内の元の軌道速度が既に目標速度以下なら減速は不要なので棄却。
  // NOTE: 速度は stairstep 補間なので base の値だけを見れば区間内の速度は尽くせる
  // TODO(odashima): Trajectory 側に弧長区間の属性走査 utils を整備してから、この index 演算を
  // vel.max_in(from_s, to_s) 相当に置き換える。詳細は my_docs/TASK_trajectory_range_utils.md
  const auto [bases, values] = trajectory.longitudinal_velocity_mps().get_data();
  // bases は昇順。from_s で有効な base(from_s 以下の最後の base)から、
  // to_s 以上の最初の base の手前まで
  const auto upper = std::upper_bound(bases.begin(), bases.end(), slow_down_interval.from_s);
  const size_t begin_idx =
    upper == bases.begin() ? 0 : static_cast<size_t>(std::distance(bases.begin(), upper) - 1);
  const size_t end_idx = std::distance(
    bases.begin(), std::lower_bound(bases.begin(), bases.end(), slow_down_interval.to_s));

  if (std::none_of(values.begin() + begin_idx, values.begin() + end_idx, [&](const double vel) {
        return stable_slow_down_vel < vel;
      })) {
    RCLCPP_DEBUG(
      logger_,
      "[SlowDown] Ignore obstacle (%s) since slow down velocity (%f) is higher than trajectory "
      "velocity.",
      autoware_utils_uuid::to_hex_string(target.obstacle.uuid).c_str(), stable_slow_down_vel);
    return std::nullopt;
  }

  return stable_slow_down_vel;
}

// schmitt trigger on the object speed norm (threshold ± hysteresis_range)
Motion SlowDownPlanner::determine_obstacle_motion(
  const SlowDownObstacle & obstacle, const std::optional<SlowDownCarryOver> & prev_output) const
{
  const double object_vel_norm = std::hypot(obstacle.velocity, obstacle.lat_velocity);
  // 前フレームに減速出力がなければヒステリシスなしの素の閾値判定
  if (!prev_output) {
    return object_vel_norm > params_.moving_object_speed_threshold ? Motion::Moving
                                                                   : Motion::Static;
  }

  // TODO: SchmittTriggerクラスの意味がないので戻す？
  const bool is_static = SchmittTrigger{prev_output->obstacle_motion == Motion::Static}.update(
    object_vel_norm, params_.moving_object_speed_threshold + params_.moving_object_hysteresis_range,
    params_.moving_object_speed_threshold - params_.moving_object_hysteresis_range);
  return is_static ? Motion::Static : Motion::Moving;
}

std::optional<SlowdownInterval> SlowDownPlanner::calculate_distance_to_slow_down_with_constraints(
  const SlowDownInput & input, const EgoTrajectory & trajectory, const SlowDownTarget & target,
  const double dist_to_ego, const bool is_driving_forward, const double slow_down_vel) const
{
  const auto & obstacle = target.obstacle;
  const auto & prev_output = target.prev;
  const double abs_ego_offset = is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);
  const double obstacle_vel = obstacle.velocity;

  // calculate distance to collision points
  const double dist_to_front_collision =
    autoware::experimental::trajectory::closest(trajectory, obstacle.front_collision_point);
  const double dist_to_back_collision =
    autoware::experimental::trajectory::closest(trajectory, obstacle.back_collision_point);

  const double ego_vel = input.ego_vel;
  const double ego_acc = input.ego_acc;

  // calculate offset distance to first collision considering relative velocity
  const double offset_dist_to_collision = [&]() {
    if (dist_to_front_collision < dist_to_ego + abs_ego_offset) {
      return 0.0;
    }

    // This min/max process prevents the slowdown point from moving closer when the vehicle
    // decelerates towards slow_down_vel.
    const double ego_assumed_vel =
      obstacle.velocity > 0.0 ? std::max(ego_vel, slow_down_vel) : std::min(ego_vel, slow_down_vel);
    const double relative_vel = ego_assumed_vel - obstacle.velocity;

    // NOTE: This min_relative_vel forces the relative velocity positive if the ego velocity is
    // lower than the obstacle velocity. Without this, the slow down feature will flicker where
    // the ego velocity is very close to the obstacle velocity.
    constexpr double min_relative_vel = 1.0;
    const double time_to_collision = (dist_to_front_collision - dist_to_ego - abs_ego_offset) /
                                     std::max(min_relative_vel, relative_vel);

    const double cropped_time_to_collision = std::max(0.0, time_to_collision);
    return obstacle_vel * cropped_time_to_collision;
  }();

  // calculate distance during deceleration, slow down preparation, and slow down
  const double min_slow_down_prepare_dist = 3.0;
  const double slow_down_prepare_dist =
    std::max(min_slow_down_prepare_dist, slow_down_vel * params_.time_margin_on_target_velocity);
  const double deceleration_dist = offset_dist_to_collision + dist_to_front_collision -
                                   abs_ego_offset - dist_to_ego - slow_down_prepare_dist;
  const double slow_down_dist =
    dist_to_back_collision - dist_to_front_collision + slow_down_prepare_dist;

  // calculate distance to start/end slow down
  const double dist_to_slow_down_start = dist_to_ego + deceleration_dist;
  const double dist_to_slow_down_end = dist_to_ego + deceleration_dist + slow_down_dist;
  if (100.0 < dist_to_slow_down_start) {
    // NOTE: distance to slow down is too far.
    return std::nullopt;
  }

  // apply low-pass filter to distance to start/end slow down
  const auto apply_lowpass_filter = [&](const double dist_to_slow_down, const auto prev_point) {
    if (prev_output && prev_point) {
      const double prev_dist_to_slow_down =
        autoware::experimental::trajectory::closest(trajectory, prev_point->position);
      return autoware::signal_processing::lowpassFilter(
        dist_to_slow_down, prev_dist_to_slow_down, params_.lpf_gain_dist_to_slow_down);
    }
    return dist_to_slow_down;
  };

  const double filtered_dist_to_slow_down_start = apply_lowpass_filter(
    dist_to_slow_down_start, prev_output ? prev_output->start_point : std::nullopt);
  const double filtered_dist_to_slow_down_end = apply_lowpass_filter(
    dist_to_slow_down_end, prev_output ? prev_output->end_point : std::nullopt);
  const double deceleration_dist_lpf = filtered_dist_to_slow_down_start - dist_to_ego;

  const double feasible_slow_down_vel = calculate_feasible_slow_down_velocity(
    trajectory, prev_output, ego_vel, ego_acc, slow_down_vel, deceleration_dist_lpf,
    filtered_dist_to_slow_down_start);

  return SlowdownInterval{
    filtered_dist_to_slow_down_start, filtered_dist_to_slow_down_end, feasible_slow_down_vel};
}

// calculate the slow down velocity considering the deceleration constraints (min acc/jerk)
double SlowDownPlanner::calculate_feasible_slow_down_velocity(
  const EgoTrajectory & trajectory, const std::optional<SlowDownCarryOver> & prev_output,
  const double ego_vel, const double ego_acc, const double slow_down_vel,
  const double deceleration_dist, const double dist_to_slow_down_start) const
{
  if (deceleration_dist < 0) {
    if (prev_output) {
      return prev_output->target_vel;
    }
    return std::max(ego_vel, slow_down_vel);
  }
  if (ego_vel < slow_down_vel) {
    return slow_down_vel;
  }

  const double one_shot_slow_down_vel = [&]() {
    if (ego_acc < params_.slow_down_min_acc) {
      const double squared_vel =
        std::pow(ego_vel, 2) + 2 * deceleration_dist * params_.slow_down_min_acc;
      if (squared_vel < 0) {
        return slow_down_vel;
      }
      return std::max(std::sqrt(squared_vel), slow_down_vel);
    }
    // TODO(murooka) Calculate more precisely. Final acceleration should be zero.
    const double min_slow_down_vel = calc_deceleration_velocity_from_distance_to_target(
      params_.slow_down_min_jerk, params_.slow_down_min_acc, ego_acc, ego_vel, deceleration_dist);
    return min_slow_down_vel;
  }();

  // start_point は減速開始位置が軌道範囲外だった前フレームでは空
  if (prev_output && prev_output->start_point) {
    // NOTE: If longitudinal controllability is not good, one_shot_slow_down_vel may be getting
    // larger since we use actual ego's velocity and acceleration for its calculation.
    //       Suppress one_shot_slow_down_vel getting larger here.
    const double start_point_diff =
      dist_to_slow_down_start -
      autoware::experimental::trajectory::closest(trajectory, prev_output->start_point->position);
    const double prev_slow_down_vel = std::sqrt(
      std::max(
        0.0, std::pow(prev_output->feasible_target_vel, 2) +
               2 * params_.slow_down_min_acc * start_point_diff));
    const double feasible_slow_down_vel = std::min(one_shot_slow_down_vel, prev_slow_down_vel);
    return std::max(slow_down_vel, feasible_slow_down_vel);
  }
  return std::max(slow_down_vel, one_shot_slow_down_vel);
}

double SlowDownPlanner::calculate_target_slow_down_velocity(const SlowDownTarget & target) const
{
  const auto & p = get_object_param(target.obstacle.classification)
                     .get_velocity_param(target.obstacle.side, target.obstacle_motion);

  const double ratio = std::clamp(
    (std::abs(target.stable_dist_to_traj_poly) - p.min_lat_margin) /
      (p.max_lat_margin - p.min_lat_margin),
    0.0, 1.0);
  return p.min_ego_velocity + ratio * (p.max_ego_velocity - p.min_ego_velocity);
}

}  // namespace autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils
