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

#include "obstacle_slow_down_utils.hpp"

#include "trajectory_polygon_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware/trajectory/utils/lateral_metrics.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils
{
namespace bg = boost::geometry;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_utils_geometry::Point2d;

namespace
{
// TODO(odashima) following two functions are copied from behavior_velocity_planner.
// These should be refactored.
double find_reach_time(
  const double jerk, const double accel, const double velocity, const double distance,
  const double t_min, const double t_max, const rclcpp::Logger & logger)
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
    if (iter > warn_iter) {
      RCLCPP_WARN(logger, "[SlowDown] find_reach_time: current iter is over warning");
    }
  }
  return t;
}

template <class T>
bool contains_uuid(const std::vector<T> & obstacles, const UUID & target_uuid)
{
  return std::any_of(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid.uuid == target_uuid.uuid;
  });
}

bool schmitt_trigger(
  const bool prev_is_low, const double current_val, const double high_val, const double low_val)
{
  return prev_is_low ? !(high_val < current_val) : current_val < low_val;
}

// estimate the lower bound of the lateral distance from the object to the trajectory polygon,
// assuming the worst case for both the object shape and the ego footprint
double calc_possible_min_dist_from_obj_to_traj_poly(
  const PredictedObject & object, const EgoTrajectory & trajectory, const double obj_s,
  const VehicleInfo & vehicle_info)
{
  const double object_possible_max_dist = std::invoke([&object]() {
    const auto & shape = object.shape;
    if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
      return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
    } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
      return shape.dimensions.x / 2.0;
    } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
      double max_length_to_point = 0.0;
      for (const auto rel_point : shape.footprint.points) {
        const double length_to_point = std::hypot(rel_point.x, rel_point.y);
        if (max_length_to_point < length_to_point) {
          max_length_to_point = length_to_point;
        }
      }
      return max_length_to_point;
    }

    throw std::logic_error("The shape type is not supported in obstacle_slow_down.");
  });
  // The minimum lateral distance to the trajectory polygon is estimated by assuming that the
  // ego-vehicle's front right or left corner is the furthest from the trajectory, in the very worst
  // case
  const double ego_possible_max_dist =
    std::hypot(vehicle_info.max_longitudinal_offset_m, vehicle_info.vehicle_width_m / 2.0);
  const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  // lateral_metrics は Trajectory<Pose> 専用のため基底クラス側で呼ぶ
  const double possible_min_dist_to_traj_poly =
    autoware::experimental::trajectory::compute_lateral_distance<geometry_msgs::msg::Pose>(
      trajectory, obj_pos, obj_s) -
    ego_possible_max_dist - object_possible_max_dist;
  return possible_min_dist_to_traj_poly;
}

double calc_dist_to_traj_poly(
  const Polygon2d & obj_poly, const std::vector<Polygon2d> & ego_swept_polys)
{
  double dist_to_traj_poly = std::numeric_limits<double>::max();
  for (const auto & traj_poly : ego_swept_polys) {
    const double current_dist_to_traj_poly = bg::distance(traj_poly, obj_poly);
    dist_to_traj_poly = std::min(dist_to_traj_poly, current_dist_to_traj_poly);
  }
  return dist_to_traj_poly;
}

// returns {longitudinal, lateral} velocity relative to the trajectory. the lateral velocity is
// positive if the object is approaching the trajectory.
std::pair<double, double> calc_vel_relative_to_traj(
  const PredictedObject & object, const EgoTrajectory & trajectory, const double obj_s)
{
  const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto & obj_twist = object.kinematics.initial_twist_with_covariance.twist;

  const auto nearest_traj_point = trajectory.compute(obj_s);

  const double traj_yaw = tf2::getYaw(nearest_traj_point.pose.orientation);
  const double obj_yaw = tf2::getYaw(obj_pose.orientation);
  const Eigen::Rotation2Dd R_ego_to_obstacle(
    autoware_utils_math::normalize_radian(obj_yaw - traj_yaw));

  const Eigen::Vector2d traj_direction(std::cos(traj_yaw), std::sin(traj_yaw));
  const Eigen::Vector2d traj_to_obstacle(
    obj_pose.position.x - nearest_traj_point.pose.position.x,
    obj_pose.position.y - nearest_traj_point.pose.position.y);

  // Determine if the obstacle is to the left or right of the trajectory using the cross product
  const double cross_product =
    traj_direction.x() * traj_to_obstacle.y() - traj_direction.y() * traj_to_obstacle.x();
  const int sign = (cross_product > 0) ? -1 : 1;

  const Eigen::Vector2d obstacle_velocity(obj_twist.linear.x, obj_twist.linear.y);
  const Eigen::Vector2d projected_velocity = R_ego_to_obstacle * obstacle_velocity;

  return {projected_velocity[0], sign * projected_velocity[1]};
}

geometry_msgs::msg::Pose get_predicted_current_pose(
  const PredictedObject & object, const rclcpp::Time & current_stamp,
  const rclcpp::Time & predicted_objects_stamp, const rclcpp::Logger & logger)
{
  const auto & predicted_paths = object.kinematics.predicted_paths;
  const auto predicted_pose_opt = [&]() -> std::optional<geometry_msgs::msg::Pose> {
    if (predicted_paths.empty()) {
      return std::nullopt;
    }

    // Get the most reliable path
    const auto predicted_path = std::max_element(
      predicted_paths.begin(), predicted_paths.end(),
      [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

    const double rel_time = (current_stamp - predicted_objects_stamp).seconds();
    if (rel_time < 0.0) {
      return std::nullopt;
    }

    const auto pose =
      autoware::object_recognition_utils::calcInterpolatedPose(*predicted_path, rel_time);
    if (!pose) {
      return std::nullopt;
    }
    return pose.get();
  }();

  if (!predicted_pose_opt) {
    RCLCPP_WARN(logger, "Failed to calculate the predicted object pose.");
    return object.kinematics.initial_pose_with_covariance.pose;
  }
  return *predicted_pose_opt;
}

// calculate the front/back collision points (the collision vertices with the minimum/maximum arc
// length along the trajectory) between the obstacle polygon and the trajectory polygons.
// returns nullopt if there is no collision.
std::optional<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>>
calc_front_back_collision_points(
  const EgoTrajectory & trajectory, const std::vector<Polygon2d> & slow_down_corridor_polys,
  const Polygon2d & obstacle_poly)
{
  const auto to_geom_point = [](const Point2d & point) {
    geometry_msgs::msg::Point geom_point;
    geom_point.x = point.x();
    geom_point.y = point.y();
    return geom_point;
  };

  bool has_collision = false;
  double front_min_s = std::numeric_limits<double>::max();
  double back_max_s = std::numeric_limits<double>::lowest();
  geometry_msgs::msg::Point front_collision_point;
  geometry_msgs::msg::Point back_collision_point;
  for (const auto & traj_poly : slow_down_corridor_polys) {
    std::vector<Polygon2d> collision_polygons;
    bg::intersection(traj_poly, obstacle_poly, collision_polygons);

    if (collision_polygons.empty()) {
      if (has_collision) {
        break;  // for efficient calculation
      }
      continue;
    }
    has_collision = true;

    for (const auto & collision_poly : collision_polygons) {
      for (const auto & collision_point : collision_poly.outer()) {
        const auto collision_geom_point = to_geom_point(collision_point);
        const double s =
          autoware::experimental::trajectory::closest(trajectory, collision_geom_point);
        if (s < front_min_s) {
          front_min_s = s;
          front_collision_point = collision_geom_point;
        }
        if (back_max_s < s) {
          back_max_s = s;
          back_collision_point = collision_geom_point;
        }
      }
    }
  }

  if (!has_collision) {
    return std::nullopt;
  }
  return std::make_pair(front_collision_point, back_collision_point);
}

double calc_deceleration_velocity_from_distance_to_target(
  const double max_slowdown_jerk, const double max_slowdown_accel, const double current_accel,
  const double current_velocity, const double distance_to_target, const rclcpp::Logger & logger)
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
    const double t_jerk = find_reach_time(j_max, a0, v0, l, 0, t_const_jerk, logger);
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
}  // namespace

void insert_slowdown(EgoTrajectory & trajectory, const SlowdownInterval & slowdown_interval)
{
  auto & vel = trajectory.longitudinal_velocity_mps();

  // 境界 base を「現在の補間値の自己代入」で挿入する(プロファイルは変えず base だけ増やす)。
  // at().set は base が無ければ挿入するので、補間器(stairstep/linear)によらず恒等操作になる
  vel.at(slowdown_interval.from_s).set(vel.compute(slowdown_interval.from_s));
  vel.at(slowdown_interval.to_s).set(vel.compute(slowdown_interval.to_s));

  // clamp the velocity of all bases inside the interval
  const auto [bases, values] = vel.get_data();
  for (size_t i = 0; i < bases.size(); ++i) {
    if (
      slowdown_interval.from_s <= bases.at(i) && bases.at(i) <= slowdown_interval.to_s &&
      slowdown_interval.velocity < values.at(i)) {
      vel.at(bases.at(i)).set(slowdown_interval.velocity);
    }
  }
}

SlowDownPlanner::SlowDownPlanner(const VehicleInfo & vehicle_info, const rclcpp::Logger & logger)
: vehicle_info_(vehicle_info), logger_(logger)
{
}

void SlowDownPlanner::update_params(const ObstacleSlowDownParams & params)
{
  params_ = params;

  const std::unordered_map<std::string, uint8_t> types_map{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK},     {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN},
    {"animal", ObjectClassification::ANIMAL},   {"hazard", ObjectClassification::HAZARD}};
  target_object_labels_.clear();
  for (const auto & type : params_.target_objects) {
    target_object_labels_.push_back(types_map.at(type));
  }
}

void SlowDownPlanner::set_object_type_specific_params(
  std::unordered_map<std::string, ObjectTypeSpecificParams> params)
{
  object_type_specific_param_per_object_type_ = std::move(params);
}

SlowDownResult SlowDownPlanner::plan(const EgoTrajectory & trajectory, const SlowDownInput & input)
{
  const auto & p = params_.obstacle_filtering;
  const auto & tp = params_.trajectory_polygon;

  // 減速をかける縦区間の検出用コリドー(max_lat_margin + hysteresis で拡幅)。
  // exit ヒステリシス帯域(max + hys/2)内の障害物からも衝突点を取るため、
  // hysteresis は半分にせず全量を足す(元実装の NOTE を踏襲)。
  const auto slow_down_corridor_polys = trajectory_polygon_utils::create_one_step_polygons(
    trajectory, vehicle_info_, input.current_pose, p.max_lat_margin + p.lat_hysteresis_margin,
    tp.enable_to_consider_current_pose, tp.time_to_convergence, tp.decimate_trajectory_step_length,
    0.0);

  ego_swept_polys_per_off_track_scale_.clear();

  SlowDownResult result;
  result.obstacles =
    filter_slow_down_obstacle_for_predicted_object(slow_down_corridor_polys, trajectory, input);

  result.traj_points = trajectory.restore();
  const double dist_to_ego =
    autoware::experimental::trajectory::closest(trajectory, input.current_pose);
  const auto is_driving_forward_opt = autoware::motion_utils::isDrivingForward(result.traj_points);
  result.is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  result.plans =
    plan_slow_down(input, trajectory, result.obstacles, dist_to_ego, result.is_driving_forward);
  return result;
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

bool SlowDownPlanner::is_slow_down_obstacle(const uint8_t label) const
{
  return std::find(target_object_labels_.begin(), target_object_labels_.end(), label) !=
         target_object_labels_.end();
}

// static candidate check: object type, lateral distance range, and trajectory center line overlap
bool SlowDownPlanner::is_slow_down_candidate(
  const PredictedObject & object, const Polygon2d & obstacle_poly, const EgoTrajectory & trajectory,
  const double dist_from_obj_poly_to_traj_poly) const
{
  const auto & p = params_.obstacle_filtering;
  const auto obj_uuid_str = autoware_utils_uuid::to_hex_string(object.object_id);

  if (!is_slow_down_obstacle(object.classification.at(0).label)) {
    return false;
  }

  if (dist_from_obj_poly_to_traj_poly <= p.min_lat_margin) {
    RCLCPP_DEBUG(
      logger_,
      "[SlowDown] Ignore obstacle (%s) since the lateral distance to the trajectory is too close.",
      obj_uuid_str.substr(0, 4).c_str());
    return false;
  }

  // NOTE: crossed() detects boundary crossings, so unlike the original bg::intersects it misses
  // the case where the whole trajectory is inside the obstacle polygon, which cannot happen for
  // realistic object sizes.
  // TODO(odashima): crossed() は spline の基準点間隔(元実装は 2m リサンプル)で評価するため
  // 評価セグメント数が増えている。処理時間を計測して必要に応じて点数を間引く処理を入れる。
  if (!autoware::experimental::trajectory::crossed(trajectory, obstacle_poly.outer()).empty()) {
    RCLCPP_DEBUG(
      logger_,
      "[SlowDown] Ignore obstacle (%s) since the obstacle polygon intersects with the trajectory "
      "center line.",
      obj_uuid_str.substr(0, 4).c_str());
    return false;
  }

  return true;
}

// temporal hysteresis check: lateral distance and velocity condition must hold (or fail) for a
// given duration to enter (or exit) the slow down state
bool SlowDownPlanner::is_slow_down_required(
  ObstacleTrackingState & state, const rclcpp::Time & current_time,
  const double dist_from_obj_poly_to_traj_poly, const double lat_vel_relative_to_traj) const
{
  const auto & p = params_.obstacle_filtering;

  // check lateral distance considering hysteresis
  const bool is_lat_dist_low = schmitt_trigger(
    state.was_slow_down, dist_from_obj_poly_to_traj_poly,
    p.max_lat_margin + p.lat_hysteresis_margin / 2.0,
    p.max_lat_margin - p.lat_hysteresis_margin / 2.0);
  const bool is_lat_vel_low = std::abs(lat_vel_relative_to_traj) < p.max_lat_velocity;
  const bool is_slow_down_condition_met = is_lat_dist_low && is_lat_vel_low;

  const double dt =
    state.last_update_time ? (current_time - *state.last_update_time).seconds() : 0.0;

  // NOTE: 出入り方向が切り替わると最初の増減で ±dt に丸められるため、
  // しきい値到達時に累積時間を 0 に戻さなくても挙動は変わらないが、状態としては戻しておく
  if (state.was_slow_down) {
    // check if exiting slow down
    if (!is_slow_down_condition_met) {
      state.condition_duration = std::min(-dt, state.condition_duration - dt);
      if (state.condition_duration <= -p.time_to_exit_slow_down_condition) {
        state.condition_duration = 0.0;
        return false;
      }
    }
    return true;
  }
  // check if entering slow down
  if (is_slow_down_condition_met) {
    state.condition_duration = std::max(dt, state.condition_duration + dt);
    if (p.time_to_entry_slow_down_condition <= state.condition_duration) {
      state.condition_duration = 0.0;
      return true;
    }
  }
  return false;
}

const std::vector<Polygon2d> & SlowDownPlanner::get_ego_swept_polys(
  const EgoTrajectory & trajectory, const geometry_msgs::msg::Pose & current_pose,
  const double wheel_off_track_scale)
{
  const auto [it, inserted] =
    ego_swept_polys_per_off_track_scale_.try_emplace(wheel_off_track_scale);
  if (inserted) {
    const auto & tp = params_.trajectory_polygon;
    // ゴール付近で隙間がポリゴン末端の角までの距離として過大評価されないよう、ゴール後方に延長する
    it->second = trajectory_polygon_utils::create_one_step_polygons(
      trajectory, vehicle_info_, current_pose, 0.0, tp.enable_to_consider_current_pose,
      tp.time_to_convergence, tp.decimate_trajectory_step_length,
      tp.goal_extended_trajectory_length, wheel_off_track_scale);
  }
  return it->second;
}

std::vector<SlowDownObstacle> SlowDownPlanner::filter_slow_down_obstacle_for_predicted_object(
  const std::vector<Polygon2d> & slow_down_corridor_polys, const EgoTrajectory & trajectory,
  const SlowDownInput & input)
{
  const rclcpp::Time predicted_objects_stamp(input.predicted_objects->header.stamp);

  // slow down
  std::vector<UUID> current_uuids;  // 今フレームで追跡対象になった uuid(状態の GC 用)
  std::vector<SlowDownObstacle> slow_down_obstacles;
  const double ego_s = autoware::experimental::trajectory::closest(trajectory, input.current_pose);
  for (const auto & object : input.predicted_objects->objects) {
    const double obj_s = autoware::experimental::trajectory::closest(
      trajectory, object.kinematics.initial_pose_with_covariance.pose.position);

    // 1. rough filtering
    // 1.1. Check if the obstacle is in front of the ego.
    if (obj_s < ego_s) {
      continue;
    }

    // 1.2. Check if the rough lateral distance is smaller than the threshold.
    const double min_lat_dist_to_traj_poly =
      calc_possible_min_dist_from_obj_to_traj_poly(object, trajectory, obj_s, vehicle_info_);
    if (params_.obstacle_filtering.max_lat_margin < min_lat_dist_to_traj_poly) {
      continue;
    }

    // 2. calc lateral distance to trajectory polygon
    const auto obstacle_poly = autoware_utils_geometry::to_polygon2d(
      object.kinematics.initial_pose_with_covariance.pose, object.shape);
    // 横の隙間は「マージン 0 の ego 掃引ポリゴン」との距離。旋回時の前外輪のはみ出しを
    // どれだけ見込むかは物体種別ごとに異なる
    const double dist_from_obj_poly_to_traj_poly = calc_dist_to_traj_poly(
      obstacle_poly, get_ego_swept_polys(
                       trajectory, input.current_pose,
                       get_object_param(object.classification.at(0)).wheel_off_track_scale));

    // 3. check the slow down conditions
    // NOTE: 候補判定で落ちるフレームでも状態は GC しない(ヒステリシスを保持する)ため、
    // 追跡対象への登録は判定より前に行う
    auto & state = tracking_states_[object.object_id];
    current_uuids.push_back(object.object_id);

    if (!is_slow_down_candidate(
          object, obstacle_poly, trajectory, dist_from_obj_poly_to_traj_poly)) {
      continue;
    }

    const auto [lon_vel_relative_to_traj, lat_vel_relative_to_traj] =
      calc_vel_relative_to_traj(object, trajectory, obj_s);

    if (!is_slow_down_required(
          state, input.current_time, dist_from_obj_poly_to_traj_poly, lat_vel_relative_to_traj)) {
      RCLCPP_DEBUG(
        logger_, "[SlowDown] Ignore obstacle (%s) since it's far from trajectory. (%f [m])",
        autoware_utils_uuid::to_hex_string(object.object_id).substr(0, 4).c_str(),
        dist_from_obj_poly_to_traj_poly);
      continue;
    }

    // 4. calc front/back collision points on the slow down corridor
    const auto collision_points =
      calc_front_back_collision_points(trajectory, slow_down_corridor_polys, obstacle_poly);
    if (!collision_points) {
      RCLCPP_DEBUG(
        logger_, "[SlowDown] Ignore obstacle (%s) since there is no collision point",
        autoware_utils_uuid::to_hex_string(object.object_id).substr(0, 4).c_str());
      continue;
    }

    // 5. create the slow down obstacle
    slow_down_obstacles.push_back(create_slow_down_obstacle_for_predicted_object(
      trajectory, object, *collision_points, lon_vel_relative_to_traj, lat_vel_relative_to_traj,
      predicted_objects_stamp, input.current_time, dist_from_obj_poly_to_traj_poly));
  }
  // 今フレームで追跡対象から外れた障害物の状態を破棄する
  for (auto it = tracking_states_.begin(); it != tracking_states_.end();) {
    const bool is_current = std::any_of(
      current_uuids.begin(), current_uuids.end(),
      [&](const UUID & uuid) { return uuid.uuid == it->first.uuid; });
    it = is_current ? std::next(it) : tracking_states_.erase(it);
  }

  // 横距離ヒステリシスの状態は、横距離単体の判定ではなくフィルタ全体の最終結果で確定する
  for (auto & [uuid, state] : tracking_states_) {
    state.was_slow_down = contains_uuid(slow_down_obstacles, uuid);
    // NOTE: is_slow_down_required の中では更新しない。候補判定で落ちたフレームは同関数が
    // 呼ばれないため、そこで更新すると次の評価時に dt が数フレーム分まとまって入り、
    // フレーム数カウント時代には加算されなかった時間まで累積してしまう。ここで生存する全
    // state を毎フレーム更新すれば dt は常に 1 フレーム分になる。state 新規作成の初回だけ
    // dt=0 となりカウント方式(初回 +1)と 1 フレーム分ずれるが、これは許容する。
    state.last_update_time = input.current_time;
  }

  RCLCPP_DEBUG(
    logger_, "The number of output obstacles of filter_slow_down_obstacles is %ld",
    slow_down_obstacles.size());
  return slow_down_obstacles;
}

SlowDownObstacle SlowDownPlanner::create_slow_down_obstacle_for_predicted_object(
  const EgoTrajectory & trajectory, const PredictedObject & object,
  const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> & collision_points,
  const double lon_vel_relative_to_traj, const double lat_vel_relative_to_traj,
  const rclcpp::Time & predicted_objects_stamp, const rclcpp::Time & current_time,
  const double dist_from_obj_poly_to_traj_poly)
{
  const auto predicted_object_pose =
    get_predicted_current_pose(object, current_time, predicted_objects_stamp, logger_);
  const bool is_left = autoware::experimental::trajectory::is_left_side<geometry_msgs::msg::Pose>(
    trajectory, predicted_object_pose.position,
    autoware::experimental::trajectory::closest(trajectory, predicted_object_pose.position));

  SlowDownObstacle obstacle;
  obstacle.uuid = object.object_id;
  obstacle.stamp = predicted_objects_stamp;
  obstacle.pose = predicted_object_pose;
  obstacle.velocity = lon_vel_relative_to_traj;
  obstacle.lat_velocity = lat_vel_relative_to_traj;
  obstacle.dist_to_traj_poly = dist_from_obj_poly_to_traj_poly;
  obstacle.front_collision_point = collision_points.first;
  obstacle.back_collision_point = collision_points.second;
  obstacle.classification = object.classification.at(0);
  obstacle.side = is_left ? Side::Left : Side::Right;
  return obstacle;
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

  // 障害物との横距離から通過時の目標速度を線形補間で決める
  const auto & p =
    get_object_param(obstacle.classification).get_velocity_param(obstacle.side, obstacle_motion);
  const double ratio = std::clamp(
    (std::abs(stable_dist_to_traj_poly) - p.min_lat_margin) / (p.max_lat_margin - p.min_lat_margin),
    0.0, 1.0);
  const double slow_down_vel =
    p.min_ego_velocity + ratio * (p.max_ego_velocity - p.min_ego_velocity);

  return {obstacle, prev_slow_down, obstacle_motion, stable_dist_to_traj_poly, slow_down_vel};
}

std::optional<PlannedSlowDownWithCarryOver> SlowDownPlanner::plan_slow_down_for_obstacle(
  const SlowDownInput & input, const EgoTrajectory & trajectory, const SlowDownTarget & target,
  const double dist_to_ego, const bool is_driving_forward) const
{
  // 減速制約(min acc/jerk)と障害物の縦速度を考慮して、減速区間 [from_s, to_s] と
  // そこで実現可能な速度を計算する(遠すぎる場合は nullopt)
  const auto slow_down_interval = calculate_distance_to_slow_down_with_constraints(
    input, trajectory, target, dist_to_ego, is_driving_forward);
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
  // 減速開始位置が軌道範囲外なら、次フレームの距離 LPF の基準にできないので持たせない
  if (0.0 <= slow_down_interval->from_s && slow_down_interval->from_s <= trajectory.length()) {
    carry_over.start_point = trajectory.compute(slow_down_interval->from_s).pose;
  }
  carry_over.end_point = trajectory.compute(slow_down_interval->to_s).pose;
  carry_over.obstacle_motion = target.obstacle_motion;

  // planning factor の start は幾何的な減速開始位置ではなく、元実装の virtual wall と同じく
  // ego 位置を減速区間 [from_s, to_s] にクランプした点(通過中は ego に追従して end まで残す)
  const double wall_s = std::clamp(dist_to_ego, slowdown_interval.from_s, slowdown_interval.to_s);

  // end_point は結果側も持ち越し側も要るので、寿命を絡ませないよう値で重複して持つ
  return PlannedSlowDownWithCarryOver{
    {slowdown_interval, target.obstacle, trajectory.compute(wall_s).pose, *carry_over.end_point},
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

  const bool is_static = schmitt_trigger(
    prev_output->obstacle_motion == Motion::Static, object_vel_norm,
    params_.moving_object_speed_threshold + params_.moving_object_hysteresis_range,
    params_.moving_object_speed_threshold - params_.moving_object_hysteresis_range);
  return is_static ? Motion::Static : Motion::Moving;
}

std::optional<SlowdownInterval> SlowDownPlanner::calculate_distance_to_slow_down_with_constraints(
  const SlowDownInput & input, const EgoTrajectory & trajectory, const SlowDownTarget & target,
  const double dist_to_ego, const bool is_driving_forward) const
{
  const auto & obstacle = target.obstacle;
  const auto & prev_output = target.prev;
  const double slow_down_vel = target.slow_down_vel;
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
      params_.slow_down_min_jerk, params_.slow_down_min_acc, ego_acc, ego_vel, deceleration_dist,
      logger_);
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

}  // namespace autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils
