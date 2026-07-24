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

#include "autoware/trajectory_validator/filters/safety/point_cloud_collision_check_filter.hpp"

#include "point_cloud_collision_check/debug_marker.hpp"
#include "point_cloud_collision_check/parameter.hpp"
#include "point_cloud_collision_check/pointcloud_preprocessing.hpp"
#include "point_cloud_collision_check/types.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace pcc = autoware::trajectory_validator::plugin::safety::point_cloud_collision_check;

namespace
{
namespace mvp = autoware::motion_velocity_planner;
using pcc::CollisionPointWithDist;
using pcc::DebugData;
using pcc::DetectionPolygon;
using pcc::Params;
using pcc::PointcloudStopCandidate;
using pcc::Polygon2d;
using pcc::PreprocessedPointcloud;
using pcc::StopObstacle;
using pcc::StopObstacleClassification;
using pcc::TrajectoryPoint;
using pcc::VehicleInfo;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// base_link から進行方向側のバンパー端までの縦オフセット。
double calc_x_offset_to_bumper(const bool is_driving_forward, const VehicleInfo & vehicle_info)
{
  return is_driving_forward ? vehicle_info.max_longitudinal_offset_m
                            : vehicle_info.min_longitudinal_offset_m;
}

// 指定加速度で停止するまでに必要な最小距離。
double calc_minimum_distance_to_stop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }
  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

// RSS 停止で用いる障害物の制動距離（点群は pointcloud_deceleration を使用）。
double calc_braking_dist_along_trajectory(const double lon_vel, const pcc::RSSParam & rss_params)
{
  const double error_considered_vel = std::max(lon_vel + rss_params.velocity_offset, 0.0);
  return error_considered_vel * error_considered_vel * 0.5 / -rss_params.pointcloud_deceleration;
}

// 自車がジャーク・加速度制約下で停止するまでの制動距離（前進時のみ）。
std::optional<double> calc_ego_forwarding_braking_distance(
  const std::vector<TrajectoryPoint> & traj_points, const nav_msgs::msg::Odometry & odometry,
  const pcc::CommonParam & common)
{
  if (
    traj_points.empty() ||
    autoware::motion_utils::isDrivingForward(traj_points).value_or(false) != true) {
    return std::nullopt;
  }
  return autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    odometry.twist.twist.linear.x, 0.0, common.max_accel, common.min_accel, common.max_jerk,
    common.min_jerk);
}

// 検出ポリゴン生成パラメータを構築する（点群は object_velocity=nullopt＝停止扱い）。
pcc::PolygonParam create_polygon_param(
  const pcc::TrimTrajectoryParam & trim, const std::optional<double> ego_braking_distance,
  const pcc::LateralMarginParam & lateral_margin)
{
  pcc::PolygonParam p;
  if (!trim.enable_trimming || !ego_braking_distance.has_value()) {
    p.trimming_length = std::nullopt;
  } else {
    p.trimming_length =
      trim.min_trajectory_length + trim.braking_distance_scale_factor * ego_braking_distance.value();
  }
  p.lateral_margin = lateral_margin.nominal_margin + lateral_margin.additional_is_stop_margin;
  p.off_track_scale = lateral_margin.additional_wheel_off_track_scale;
  return p;
}

// polygon_param に対応する検出ポリゴンを生成する（候補ごとに 1 回のみのためキャッシュ省略）。
DetectionPolygon get_trajectory_polygon(
  const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const pcc::PolygonParam & polygon_param,
  const bool enable_to_consider_current_pose, const double time_to_convergence)
{
  auto cropped_traj_points =
    polygon_param.trimming_length.has_value()
      ? autoware::motion_utils::cropForwardPoints(
          decimated_traj_points, decimated_traj_points.front().pose.position, 0,
          polygon_param.trimming_length.value())
      : decimated_traj_points;

  auto traj_polys = mvp::polygon_utils::create_one_step_polygons(
    cropped_traj_points, vehicle_info, current_ego_pose, polygon_param.lateral_margin,
    enable_to_consider_current_pose, time_to_convergence, 0.0, polygon_param.off_track_scale);
  return DetectionPolygon{std::move(cropped_traj_points), std::move(traj_polys)};
}

// 検出ポリゴン内の点群点のうち、自車バンパーから最も手前で衝突する点を軌道前方から順に探索する。
std::optional<CollisionPointWithDist> get_nearest_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const PreprocessedPointcloud & point_cloud, const double x_offset_to_bumper,
  const VehicleInfo & vehicle_info, const pcc::HeightMarginParam & height_margin)
{
  if (traj_points.size() != traj_polygons.size()) {
    return std::nullopt;
  }
  const auto & pointcloud_ptr = point_cloud.filtered_pointcloud_ptr;
  if (!pointcloud_ptr || pointcloud_ptr->empty()) {
    return std::nullopt;
  }
  const auto & clusters = point_cloud.cluster_indices;

  for (size_t traj_index = 0; traj_index < traj_points.size(); ++traj_index) {
    const double rough_dist_th = boost::geometry::perimeter(traj_polygons.at(traj_index)) * 0.5;
    const double traj_height = traj_points.at(traj_index).pose.position.z;

    std::vector<geometry_msgs::msg::Point> collision_geom_points{};
    for (const auto & cluster : clusters) {
      for (const auto & point_index : cluster.indices) {
        const auto obstacle_point = mvp::utils::to_geometry_point(pointcloud_ptr->at(point_index));
        if (
          obstacle_point.z - traj_height < -height_margin.margin_from_bottom ||
          obstacle_point.z - traj_height >
            vehicle_info.max_height_offset_m + height_margin.margin_from_top) {
          continue;
        }
        const double dist_from_base_link =
          autoware_utils_geometry::calc_distance2d(traj_points.at(traj_index).pose, obstacle_point);
        if (dist_from_base_link > rough_dist_th) {
          continue;
        }
        autoware_utils_geometry::Point2d obstacle_point_2d{obstacle_point.x, obstacle_point.y};
        if (boost::geometry::within(obstacle_point_2d, traj_polygons.at(traj_index))) {
          collision_geom_points.push_back(obstacle_point);
        }
      }
    }
    if (collision_geom_points.empty()) {
      continue;
    }

    const auto bumper_pose = autoware_utils_geometry::calc_offset_pose(
      traj_points.at(traj_index).pose, x_offset_to_bumper, 0.0, 0.0);
    std::optional<double> max_collision_length = std::nullopt;
    std::optional<geometry_msgs::msg::Point> max_collision_point = std::nullopt;
    for (const auto & point : collision_geom_points) {
      const double dist_from_bumper =
        std::abs(autoware_utils_geometry::inverse_transform_point(point, bumper_pose).x);
      if (!max_collision_length.has_value() || dist_from_bumper > *max_collision_length) {
        max_collision_length = dist_from_bumper;
        max_collision_point = point;
      }
    }
    return CollisionPointWithDist{
      *max_collision_point,
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, traj_index) -
        *max_collision_length};
  }
  return std::nullopt;
}

// map 系世界点を候補軌道へ射影し、衝突距離と縦速度を得る（決定10の読み出し時射影）。
// 射影点が横マージン外／自車後方なら nullopt（phantom 防止の横ゲート）。
struct Projection
{
  double dist_to_collide{};
  double lon_velocity{};
};
std::optional<Projection> project_world_point(
  const geometry_msgs::msg::Point & world_point, const Eigen::Vector2d & world_velocity,
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Point & ego_position,
  const double x_offset_to_bumper, const double lateral_margin, const double min_clamp_velocity,
  const double max_clamp_velocity)
{
  if (traj_points.size() < 2) {
    return std::nullopt;
  }
  const double lateral_offset =
    autoware::motion_utils::calcLateralOffset(traj_points, world_point);
  if (std::abs(lateral_offset) > lateral_margin) {
    return std::nullopt;
  }
  const double dist_to_collide =
    autoware::motion_utils::calcSignedArcLength(traj_points, ego_position, world_point) -
    x_offset_to_bumper;
  if (dist_to_collide <= 0.0) {
    return std::nullopt;
  }

  const size_t nearest_index =
    autoware::motion_utils::findNearestIndex(traj_points, world_point);
  const double yaw = yaw_from_quaternion(traj_points.at(nearest_index).pose.orientation);
  const double lon_velocity = world_velocity.x() * std::cos(yaw) + world_velocity.y() * std::sin(yaw);
  return Projection{
    dist_to_collide, std::clamp(lon_velocity, min_clamp_velocity, max_clamp_velocity)};
}

}  // namespace

// pimpl: 状態（時系列 deque・パラメータ）と全ロジックを保持する。
struct PointCloudCollisionCheckFilter::Impl
{
  Params params_;
  std::deque<PointcloudStopCandidate> stop_candidates_;
  int cycle_candidate_counter_{0};

  // 今回の観測点を map 系世界点で時系列 deque へ対応付け、速度ベクトルを推定する（決定10）。
  void upsert_stop_candidate(
    const geometry_msgs::msg::Point & world_point, const rclcpp::Time & obs_stamp)
  {
    const auto & assoc = params_.time_series_association;
    const auto & vel = params_.velocity_estimation;
    const double v_max = std::max(std::abs(assoc.min_velocity), assoc.max_velocity);

    bool gated_exists = false;
    for (auto candidate = stop_candidates_.rbegin(); candidate != stop_candidates_.rend();
         ++candidate) {
      const double dt = (obs_stamp - candidate->latest_collision_pointcloud_time).seconds();
      const double displacement =
        autoware_utils_geometry::calc_distance2d(candidate->latest_world_point, world_point);
      const bool within_gate = displacement < assoc.position_diff + v_max * std::max(dt, 0.0);
      if (within_gate) {
        gated_exists = true;
      }
      // per-entry stamp ガード：この観測時刻で既に更新済みのエントリは飛ばす（決定10）。
      if (dt < 0.05) {
        continue;
      }
      if (dt < assoc.max_time_diff && within_gate) {
        const Eigen::Vector2d world_velocity{
          (world_point.x - candidate->latest_world_point.x) / dt,
          (world_point.y - candidate->latest_world_point.y) / dt};
        if (!candidate->has_velocity()) {
          candidate->initial_velocities.push_back(world_velocity);
          if (candidate->initial_velocities.size() >= vel.required_velocity_count) {
            Eigen::Vector2d sum = Eigen::Vector2d::Zero();
            for (const auto & v : candidate->initial_velocities) {
              sum += v;
            }
            const Eigen::Vector2d mean = sum / candidate->initial_velocities.size();
            candidate->vel_lpf_x.reset(mean.x());
            candidate->vel_lpf_y.reset(mean.y());
          }
        } else {
          candidate->vel_lpf_x.filter(world_velocity.x());
          candidate->vel_lpf_y.filter(world_velocity.y());
        }
        candidate->latest_world_point = world_point;
        candidate->latest_collision_pointcloud_time = obs_stamp;
        std::sort(
          stop_candidates_.begin(), stop_candidates_.end(),
          [](const PointcloudStopCandidate & a, const PointcloudStopCandidate & b) {
            return a.latest_collision_pointcloud_time < b.latest_collision_pointcloud_time;
          });
        return;
      }
    }
    // 距離ゲートを通る既存エントリが在れば重複登録しない（多候補固有の歯止め・決定10）。
    if (gated_exists) {
      return;
    }
    PointcloudStopCandidate new_candidate;
    new_candidate.latest_world_point = world_point;
    new_candidate.latest_collision_pointcloud_time = obs_stamp;
    new_candidate.vel_lpf_x.setGain(vel.lpf_gain);
    new_candidate.vel_lpf_y.setGain(vel.lpf_gain);
    stop_candidates_.push_back(new_candidate);
  }

  std::vector<StopObstacle> calc_obstacle_stop(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context,
    const VehicleInfo & vehicle_info, DebugData * debug = nullptr)
  {
    const auto & raw_trajectory_points = candidate_trajectory.points;
    const auto & odometry = *context.odometry;
    const bool is_driving_forward =
      autoware::motion_utils::isDrivingForward(raw_trajectory_points).value_or(true);
    const double x_offset_to_bumper = calc_x_offset_to_bumper(is_driving_forward, vehicle_info);

    const auto & tp = params_.trajectory_polygon;
    const auto decimated_traj_points = mvp::utils::decimate_trajectory_points_from_ego(
      raw_trajectory_points, odometry.pose.pose, tp.ego_nearest_dist_threshold,
      tp.ego_nearest_yaw_threshold, tp.decimate_trajectory_step_length, params_.stop_margin);
    if (decimated_traj_points.size() < 2) {
      return {};
    }

    const auto polygon_param = create_polygon_param(
      params_.obstacle_filtering.trim_trajectory,
      calc_ego_forwarding_braking_distance(raw_trajectory_points, odometry, params_.common),
      params_.obstacle_filtering.lateral_margin);
    const auto detection_polygon = get_trajectory_polygon(
      decimated_traj_points, vehicle_info, odometry.pose.pose, polygon_param,
      tp.enable_to_consider_current_pose, tp.time_to_convergence);

    // 点群を map 系へ変換し前処理する（決定6・9・7）。
    const auto map_pointcloud = pcc::convert_pointcloud_to_map_frame(
      *context.segmented_pointcloud, odometry.pose.pose,
      params_.obstacle_filtering.excluded_class_ids);
    const auto preprocessed = pcc::filter_and_cluster_point_clouds(
      map_pointcloud, detection_polygon.polygons, detection_polygon.traj_points, vehicle_info,
      params_.preprocess);

    const auto nearest_collision_point = get_nearest_collision_point(
      detection_polygon.traj_points, detection_polygon.polygons, preprocessed, x_offset_to_bumper,
      vehicle_info, params_.height_margin);

    if (debug) {
      debug->detection_polygons = detection_polygon.polygons;
      debug->filtered_pointcloud_ptr = preprocessed.filtered_pointcloud_ptr;
      debug->ego_position = odometry.pose.pose.position;
      if (nearest_collision_point) {
        debug->nearest_collision_point = nearest_collision_point->point;
      }
    }

    const rclcpp::Time obs_stamp{context.segmented_pointcloud->header.stamp};
    const rclcpp::Time now_stamp{context.odometry->header.stamp};
    if (nearest_collision_point) {
      upsert_stop_candidate(nearest_collision_point->point, obs_stamp);
    }

    // 保持時間を超えた古い候補を先頭から削除する。
    while (!stop_candidates_.empty() &&
           (obs_stamp - stop_candidates_.front().latest_collision_pointcloud_time).seconds() >
             params_.obstacle_filtering.stop_obstacle_hold_time_threshold) {
      stop_candidates_.pop_front();
    }

    std::vector<StopObstacle> stop_obstacles;
    for (const auto & candidate : stop_candidates_) {
      if (!candidate.has_velocity()) {
        continue;
      }
      const Eigen::Vector2d world_velocity{
        candidate.vel_lpf_x.getValue().value(), candidate.vel_lpf_y.getValue().value()};
      const auto projection = project_world_point(
        candidate.latest_world_point, world_velocity, detection_polygon.traj_points,
        odometry.pose.pose.position, x_offset_to_bumper, polygon_param.lateral_margin,
        params_.velocity_estimation.min_clamp_velocity,
        params_.velocity_estimation.max_clamp_velocity);
      if (!projection) {
        continue;
      }

      const double time_delay =
        (now_stamp - candidate.latest_collision_pointcloud_time).seconds();
      const double time_compensated_dist =
        projection->dist_to_collide + projection->lon_velocity * time_delay;

      if (
        !params_.velocity_estimation.use_estimated_velocity ||
        projection->lon_velocity < params_.obstacle_velocity_threshold_enter_fixed_stop) {
        stop_obstacles.emplace_back(
          candidate.latest_collision_pointcloud_time, StopObstacleClassification{},
          projection->lon_velocity, candidate.latest_world_point, time_compensated_dist,
          polygon_param);
      } else if (params_.rss_params.use_rss_stop) {
        const auto braking_dist =
          calc_braking_dist_along_trajectory(projection->lon_velocity, params_.rss_params);
        stop_obstacles.emplace_back(
          candidate.latest_collision_pointcloud_time, StopObstacleClassification{},
          projection->lon_velocity, candidate.latest_world_point, time_compensated_dist,
          polygon_param, braking_dist);
      }
    }

    if (debug) {
      for (const auto & candidate : stop_candidates_) {
        DebugData::Track track;
        track.point = candidate.latest_world_point;
        track.settled = candidate.has_velocity();
        if (track.settled) {
          track.velocity =
            Eigen::Vector2d{candidate.vel_lpf_x.getValue().value(), candidate.vel_lpf_y.getValue().value()};
        }
        debug->tracks.push_back(track);
      }
    }
    return stop_obstacles;
  }
};

PointCloudCollisionCheckFilter::PointCloudCollisionCheckFilter()
: ValidatorInterface("point_cloud_collision_check_filter"), impl_(std::make_unique<Impl>())
{
}

PointCloudCollisionCheckFilter::~PointCloudCollisionCheckFilter() = default;

// memo: assume trajectory_selector subscribes "/perception/obstacle_segmentation/pointcloud" or
// "/perception/segmented/pointcloud" that was published from ptv3 node
PointCloudCollisionCheckFilter::result_t PointCloudCollisionCheckFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  const bool debug_enabled = impl_->params_.enable_debug_markers;
  // take_debug_markers() が毎サイクル clear するので、入場時に空なら先頭候補（決定10/可視化）。
  const bool first_candidate = debug_markers_.markers.empty();
  if (debug_enabled) {
    impl_->cycle_candidate_counter_ = first_candidate ? 0 : impl_->cycle_candidate_counter_ + 1;
  }
  const int k = impl_->cycle_candidate_counter_;

  const auto emit_status = [&](const std::string & text, const int level) {
    if (!debug_enabled || !first_candidate || !context.odometry) {
      return;
    }
    DebugData status;
    status.ego_position = context.odometry->pose.pose.position;
    status.status_text = text;
    status.status_level = level;
    add_cycle_debug_markers(debug_markers_, status, rclcpp::Time{context.odometry->header.stamp});
  };

  if (!context.odometry || !vehicle_info_ptr_) {
    return ValidationResult{};
  }
  // 点群 null はテスト要件どおり feasible（決定9）。注意（黄）で明示。
  if (!context.segmented_pointcloud) {
    emit_status("PCC: pointcloud null -> feasible (cannot evaluate)", 1);
    return ValidationResult{};
  }
  // base_link 以外の frame は TF が必要で対象外。安全側に error（DANGER）を返す（決定9）。
  if (context.segmented_pointcloud->header.frame_id != "base_link") {
    emit_status("PCC: frame != base_link -> DANGER", 2);
    return tl::make_unexpected(
      "segmented_pointcloud frame_id is not base_link: " +
      context.segmented_pointcloud->header.frame_id);
  }
  if (candidate_trajectory.points.size() < 2) {
    return ValidationResult{};
  }

  DebugData debug;
  std::vector<pcc::StopObstacle> stop_obstacles;
  try {
    stop_obstacles = impl_->calc_obstacle_stop(
      candidate_trajectory, context, *vehicle_info_ptr_, debug_enabled ? &debug : nullptr);
  } catch (const std::exception & e) {
    // 退化した候補軌道（spline 化に必要なユニーク点が不足する等）は幾何評価不能。
    // validator を巻き込まないよう安全側でスキップし feasible を返す。
    emit_status(std::string{"PCC: skipped un-evaluatable candidate ("} + e.what() + ")", 1);
    return ValidationResult{};
  }

  // feasibility：最も手前の衝突距離が必要制動距離＋stop_margin を下回れば infeasible。
  std::optional<double> nearest_dist_to_collide;
  for (const auto & stop_obstacle : stop_obstacles) {
    if (
      !nearest_dist_to_collide.has_value() ||
      stop_obstacle.dist_to_collide_on_decimated_traj < *nearest_dist_to_collide) {
      nearest_dist_to_collide = stop_obstacle.dist_to_collide_on_decimated_traj;
    }
  }

  const double ego_velocity = context.odometry->twist.twist.linear.x;
  const double required_distance =
    calc_minimum_distance_to_stop(
      ego_velocity, impl_->params_.common.max_accel, impl_->params_.common.min_accel) +
    impl_->params_.stop_margin;

  ValidationResult result{};
  if (nearest_dist_to_collide.has_value() && *nearest_dist_to_collide < required_distance) {
    result.is_feasible = false;
  }

  if (debug_enabled) {
    debug.dist_to_collide = nearest_dist_to_collide;
    debug.required_distance = required_distance;
    debug.is_feasible = result.is_feasible;
    debug.has_stop_obstacle = nearest_dist_to_collide.has_value();
    // generator_name（ranker と同じ前方一致）で検出ポリゴンの色を決める。
    // 名前は context.generator_id_to_name 経由で得る（本番 process() が populate）。
    const auto name_it = context.generator_id_to_name.find(candidate_trajectory.generator_id.uuid);
    if (name_it != context.generator_id_to_name.end()) {
      const std::string & generator_name = name_it->second;
      if (generator_name.rfind("DiffusionPlanner_", 0) == 0) {
        debug.generator_kind = 0;
      } else if (generator_name.rfind("MinimumRuleBasedPlanner", 0) == 0) {
        debug.generator_kind = 1;
      }
    }
    const rclcpp::Time stamp{context.odometry->header.stamp};
    add_candidate_debug_markers(debug_markers_, debug, k, stamp);
    if (first_candidate) {
      // 常時バナー：安全なら緑 SAFE、危険なら赤 STOP。点群 OK と追跡数も表示。
      debug.status_level = result.is_feasible ? 0 : 2;
      debug.status_text = std::string{"PCC: "} +
                          (result.is_feasible ? "SAFE" : "STOP REQUIRED") +
                          " | pointcloud:OK | tracked obstacles:" +
                          std::to_string(debug.tracks.size());
      add_cycle_debug_markers(debug_markers_, debug, stamp);
    }
  }
  return result;
}

void PointCloudCollisionCheckFilter::update_parameters(const validator::Params & params)
{
  impl_->params_ = pcc::Params{params.point_cloud_collision_check};
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::PointCloudCollisionCheckFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
