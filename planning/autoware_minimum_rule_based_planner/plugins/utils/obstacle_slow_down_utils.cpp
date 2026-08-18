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

// temporal hysteresis check: lateral distance and velocity condition must hold (or fail) for
// successive frames to enter (or exit) the slow down state
bool SlowDownPlanner::is_slow_down_required(
  ObstacleTrackingState & state, const double dist_from_obj_poly_to_traj_poly,
  const double lat_vel_relative_to_traj) const
{
  const auto & p = params_.obstacle_filtering;

  // check lateral distance considering hysteresis
  const bool is_lat_dist_low = SchmittTrigger{state.was_slow_down}.update(
    dist_from_obj_poly_to_traj_poly, p.max_lat_margin + p.lat_hysteresis_margin / 2.0,
    p.max_lat_margin - p.lat_hysteresis_margin / 2.0);
  const bool is_lat_vel_low = std::abs(lat_vel_relative_to_traj) < p.max_lat_velocity;
  const bool is_slow_down_condition_met = is_lat_dist_low && is_lat_vel_low;

  // NOTE: 出入り方向が切り替わると最初の increment/decrement で ±1 に丸められるため、
  // しきい値到達時にカウンタを 0 に戻さなくても挙動は変わらないが、状態としては戻しておく
  if (state.was_slow_down) {
    // check if exiting slow down
    if (!is_slow_down_condition_met) {
      state.condition_counter = std::min(-1, state.condition_counter - 1);
      if (state.condition_counter <= -p.successive_num_to_exit_slow_down_condition) {
        state.condition_counter = 0;
        return false;
      }
    }
    return true;
  }
  // check if entering slow down
  if (is_slow_down_condition_met) {
    state.condition_counter = std::max(1, state.condition_counter + 1);
    if (p.successive_num_to_entry_slow_down_condition <= state.condition_counter) {
      state.condition_counter = 0;
      return true;
    }
  }
  return false;
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

std::vector<SlowDownObstacle> SlowDownPlanner::filter_slow_down_obstacle_for_predicted_object(
  const std::vector<Polygon2d> & slow_down_corridor_polys,
  const std::vector<Polygon2d> & ego_swept_polys, const EgoTrajectory & trajectory,
  const SlowDownInput & input)
{
  const rclcpp::Time predicted_objects_stamp(input.predicted_objects->header.stamp);

  // slow down
  // TODO: condition_counterを廃止し、時間方向のヒステリシスを回数ではなく時間に揃える
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
    const double dist_from_obj_poly_to_traj_poly =
      calc_dist_to_traj_poly(obstacle_poly, ego_swept_polys);

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

    if (!is_slow_down_required(state, dist_from_obj_poly_to_traj_poly, lat_vel_relative_to_traj)) {
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
  }

  RCLCPP_DEBUG(
    logger_, "The number of output obstacles of filter_slow_down_obstacles is %ld",
    slow_down_obstacles.size());
  return slow_down_obstacles;
}

SlowDownResult SlowDownPlanner::plan(const EgoTrajectory & trajectory, const SlowDownInput & input)
{
  const auto & p = params_.obstacle_filtering;
  const auto & tp = params_.trajectory_polygon;

  // 減速をかける縦区間の検出用コリドー(max_lat_margin + hysteresis で拡幅)。
  // exit ヒステリシス帯域(max + hys/2)内の障害物からも衝突点を取るため、
  // hysteresis は半分にせず全量を足す(元実装の NOTE を踏襲)。
  // TODO: wheel_off_track_scaleの意味の確認と実装
  const auto slow_down_corridor_polys = trajectory_polygon_utils::create_one_step_polygons(
    trajectory, vehicle_info_, input.current_pose, p.max_lat_margin + p.lat_hysteresis_margin,
    tp.enable_to_consider_current_pose, tp.time_to_convergence, tp.decimate_trajectory_step_length,
    0.0);

  // 横の隙間の測定用: ego フットプリントの掃引そのもの(マージン 0)。
  // ゴール付近で隙間がポリゴン末端の角までの距離として過大評価されないよう、ゴール後方に延長する
  const auto ego_swept_polys = trajectory_polygon_utils::create_one_step_polygons(
    trajectory, vehicle_info_, input.current_pose, 0.0, tp.enable_to_consider_current_pose,
    tp.time_to_convergence, tp.decimate_trajectory_step_length, tp.goal_extended_trajectory_length);

  SlowDownResult result;
  result.obstacles = filter_slow_down_obstacle_for_predicted_object(
    slow_down_corridor_polys, ego_swept_polys, trajectory, input);

  // insertTargetPoint によるゼロ次ホールド挿入は Trajectory に直訳できないため点列ベースを残す
  result.traj_points_with_boundaries = trajectory.restore();
  const double dist_to_ego =
    autoware::experimental::trajectory::closest(trajectory, input.current_pose);
  const auto is_driving_forward_opt =
    autoware::motion_utils::isDrivingForward(result.traj_points_with_boundaries);
  result.is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  result.plans = plan_slow_down(
    input, trajectory, result.obstacles, result.traj_points_with_boundaries, dist_to_ego,
    result.is_driving_forward);
  return result;
}

}  // namespace autoware::minimum_rule_based_planner::plugin::obstacle_slow_down_utils
