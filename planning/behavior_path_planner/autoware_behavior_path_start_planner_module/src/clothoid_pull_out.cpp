// Copyright 2022 TIER IV, Inc.
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

#include "autoware/behavior_path_start_planner_module/clothoid_pull_out.hpp"

#include "autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"
#include "autoware/motion_utils/trajectory/path_with_lane_id.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using autoware::motion_utils::findNearestIndex;
using autoware_utils::calc_distance2d;
using autoware_utils::calc_offset_pose;
using lanelet::utils::getArcCoordinates;
namespace autoware::behavior_path_planner
{
using start_planner_utils::getPullOutLanes;

/**
 * @brief 指定されたポーズに対してlane_idsを取得する汎用関数
 * 他のbehavior_path_plannerモジュールの実装を参考にした汎用的なlane_ids取得関数
 * @param pose 対象のポーズ
 * @param road_lanes 検索対象のレーン群
 * @param previous_lane_ids 前の点のlane_ids（継承用、オプション）
 * @return 取得されたlane_ids
 */
std::vector<int64_t> getLaneIdsFromPose(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & road_lanes,
  const std::vector<int64_t> & previous_lane_ids)
{
  std::vector<int64_t> lane_ids;

  // 1. まず、ポーズが含まれるレーンを全て探す
  bool found_containing_lane = false;
  for (const auto & lane : road_lanes) {
    if (lanelet::utils::isInLanelet(pose, lane)) {
      lane_ids.push_back(lane.id());
      found_containing_lane = true;
    }
  }

  // 2. 含まれるレーンが見つからない場合のフォールバック処理
  if (!found_containing_lane) {
    // 2.1 最近接レーンを探す
    lanelet::Lanelet closest_lanelet{};
    if (lanelet::utils::query::getClosestLanelet(road_lanes, pose, &closest_lanelet)) {
      lane_ids = {closest_lanelet.id()};
    } else if (!previous_lane_ids.empty()) {
      // 2.2 最近接レーンも見つからない場合、前の点のlane_idsを継承
      lane_ids = previous_lane_ids;
    } else if (!road_lanes.empty()) {
      // 2.3 最後のフォールバック：最初のレーンを使用
      lane_ids.push_back(road_lanes.front().id());
    }
  }

  return lane_ids;
}

/**
 * @brief PathPointWithLaneIdにlane_idsを設定する関数
 * @param point 設定対象のPathPointWithLaneId
 * @param road_lanes 検索対象のレーン群
 * @param previous_lane_ids 前の点のlane_ids（継承用、オプション）
 */
void setLaneIdsToPathPoint(
  PathPointWithLaneId & point, const lanelet::ConstLanelets & road_lanes,
  const std::vector<int64_t> & previous_lane_ids)
{
  point.lane_ids = getLaneIdsFromPose(point.point.pose, road_lanes, previous_lane_ids);
}

/**
 * @brief 剛体変換（回転・平行移動・スケーリング）のみでクロソイドを補正
 * @param clothoid_points クロソイド変換後の点列
 * @param original_segment 元の円弧セグメント
 * @param start_pose セグメントの開始姿勢
 * @return 補正後の点列
 */
std::vector<geometry_msgs::msg::Point> correctClothoidByRigidTransform(
  const std::vector<geometry_msgs::msg::Point> & clothoid_points,
  const ArcSegment & original_segment, const geometry_msgs::msg::Pose & start_pose)
{
  if (clothoid_points.size() < 2) {
    return clothoid_points;
  }

  auto clothoid_start = clothoid_points.front();
  auto clothoid_end = clothoid_points.back();

  // 目標の開始・終了位置を取得
  auto target_start = start_pose.position;
  auto target_end = original_segment.getPointAtAngle(original_segment.getEndAngle());

  // 2. 方向ベクトルを計算
  double clothoid_dx = clothoid_end.x - clothoid_start.x;
  double clothoid_dy = clothoid_end.y - clothoid_start.y;
  double clothoid_length = std::sqrt(clothoid_dx * clothoid_dx + clothoid_dy * clothoid_dy);

  double target_dx = target_end.x - target_start.x;
  double target_dy = target_end.y - target_start.y;
  double target_length = std::sqrt(target_dx * target_dx + target_dy * target_dy);

  // 3. スケーリング係数を計算
  double scale_factor = (clothoid_length > 1e-10) ? target_length / clothoid_length : 1.0;

  // 4. 回転角度を計算
  double clothoid_angle = std::atan2(clothoid_dy, clothoid_dx);
  double target_angle = std::atan2(target_dy, target_dx);
  double rotation_angle = target_angle - clothoid_angle;

  // 角度を [-π, π] の範囲に正規化
  while (rotation_angle > M_PI) rotation_angle -= 2 * M_PI;
  while (rotation_angle < -M_PI) rotation_angle += 2 * M_PI;

  // 5. 変換行列の要素を計算
  double cos_theta = std::cos(rotation_angle);
  double sin_theta = std::sin(rotation_angle);

  // 6. 剛体変換を適用
  std::vector<geometry_msgs::msg::Point> corrected_points;
  corrected_points.reserve(clothoid_points.size());

  for (size_t i = 0; i < clothoid_points.size(); ++i) {
    geometry_msgs::msg::Point corrected_point;

    // 始点を原点に移動
    double rel_x = clothoid_points[i].x - clothoid_start.x;
    double rel_y = clothoid_points[i].y - clothoid_start.y;

    // スケーリング
    rel_x *= scale_factor;
    rel_y *= scale_factor;

    // 回転
    double rotated_x = cos_theta * rel_x - sin_theta * rel_y;
    double rotated_y = sin_theta * rel_x + cos_theta * rel_y;

    // 目標始点に平行移動
    corrected_point.x = rotated_x + target_start.x;
    corrected_point.y = rotated_y + target_start.y;
    corrected_point.z = clothoid_points[i].z;  // Z座標はそのまま

    corrected_points.push_back(corrected_point);
  }

  return corrected_points;
}

/**
 * @brief エントリクロソイドセグメントを生成（数値積分版）
 */
std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateClothoidEntry(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double A = segment.A;
  double L = segment.L;
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Point> points;

  // Entry Clothoid: 曲率を0から目標曲率まで線形に増加させる
  double target_curvature = (L / (A * A)) * direction_factor;
  double start_curvature = 0.0;

  // 数値積分による正確な計算
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    geometry_msgs::msg::Point point;
    point.x = current_x;
    point.y = current_y;
    point.z = 0.0;
    points.push_back(point);

    double progress = static_cast<double>(i) / (num_points - 1);
    // Entry Clothoid: 曲率を線形に0から目標曲率まで増加させる
    double current_curvature = start_curvature + (target_curvature - start_curvature) * progress;

    if (i < num_points - 1) {
      double ds = L / (num_points - 1);  // 微小区間

      // 数値積分による座標更新
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // 終端状態
  double final_psi = current_psi;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = points.back();
  end_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), final_psi));

  return {points, end_pose};
}

/**
 * @brief 円弧セグメントを生成
 */
std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateCircularSegment(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double radius = segment.radius;
  double angle = segment.angle;
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Point> points;

  // 円弧中心計算
  double center_x = start_pose.position.x - radius * std::sin(start_yaw) * direction_factor;
  double center_y = start_pose.position.y + radius * std::cos(start_yaw) * direction_factor;

  for (int i = 0; i < num_points; ++i) {
    double progress = static_cast<double>(i) / (num_points - 1);
    double angle_progress = angle * progress * direction_factor;
    double current_psi = start_yaw + angle_progress;
    double angle_from_center = current_psi - M_PI / 2.0 * direction_factor;

    geometry_msgs::msg::Point point;
    point.x = center_x + radius * std::cos(angle_from_center);
    point.y = center_y + radius * std::sin(angle_from_center);
    point.z = 0.0;

    points.push_back(point);
  }

  // 終端状態
  double final_psi = start_yaw + angle * direction_factor;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = points.back();
  end_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), final_psi));

  return {points, end_pose};
}

/**
 * @brief エグジットクロソイドセグメントを生成
 */
std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateClothoidExit(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double L = segment.L;
  double start_yaw = tf2::getYaw(start_pose.orientation);
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;

  std::vector<geometry_msgs::msg::Point> points;

  // 前のセグメント（円弧）の曲率を計算（回転方向を考慮）
  double start_curvature = (1.0 / segment.radius) * direction_factor;

  // 数値積分による正確な計算
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    geometry_msgs::msg::Point point;
    point.x = current_x;
    point.y = current_y;
    point.z = 0.0;
    points.push_back(point);

    double progress = static_cast<double>(i) / (num_points - 1);
    // Exit Clothoid: 曲率を線形に0まで減少させる
    double current_curvature = start_curvature * (1.0 - progress);

    if (i < num_points - 1) {
      double ds = L / (num_points - 1);  // 微小区間

      // 数値積分による座標更新
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // 終端状態
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = current_x;
  end_pose.position.y = current_y;
  end_pose.position.z = start_pose.position.z;
  end_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), current_psi));

  return {points, end_pose};
}

/**
 * @brief クロソイド経路の座標点列を生成
 */
std::vector<geometry_msgs::msg::Point> generateClothoidPath(
  const std::vector<ClothoidSegment> & segments, double point_interval,
  const geometry_msgs::msg::Pose & start_pose)
{
  // 各セグメントの理論弧長を計算
  std::vector<double> theoretical_lengths;
  for (const auto & segment : segments) {
    if (
      segment.type == ClothoidSegment::CLOTHOID_ENTRY ||
      segment.type == ClothoidSegment::CLOTHOID_EXIT) {
      theoretical_lengths.push_back(segment.L);
    } else if (segment.type == ClothoidSegment::CIRCULAR_ARC) {
      theoretical_lengths.push_back(segment.radius * segment.angle);
    }
  }

  // セグメント毎の点数を弧長と指定間隔から計算
  std::vector<int> segment_points;
  for (double length : theoretical_lengths) {
    int points = std::max(2, static_cast<int>(std::ceil(length / point_interval)) + 1);
    segment_points.push_back(points);
  }

  // 初期状態の設定
  geometry_msgs::msg::Pose current_pose = start_pose;

  std::vector<geometry_msgs::msg::Point> all_points;

  for (size_t i = 0; i < segments.size(); ++i) {
    std::vector<geometry_msgs::msg::Point> segment_points_vec;
    geometry_msgs::msg::Pose end_pose;

    int num_points = segment_points[i];

    if (segments[i].type == ClothoidSegment::CLOTHOID_ENTRY) {
      auto result = generateClothoidEntry(segments[i], current_pose, num_points);
      segment_points_vec = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CIRCULAR_ARC) {
      auto result = generateCircularSegment(segments[i], current_pose, num_points);
      segment_points_vec = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CLOTHOID_EXIT) {
      auto result = generateClothoidExit(segments[i], current_pose, num_points);
      segment_points_vec = result.first;
      end_pose = result.second;
    }

    // 重複点を避けて結合
    size_t start_idx = (all_points.empty()) ? 0 : 1;
    for (size_t j = start_idx; j < segment_points_vec.size(); ++j) {
      all_points.push_back(segment_points_vec[j]);
    }

    current_pose = end_pose;
  }

  return all_points;
}

/**
 * @brief ArcSegmentをクロソイド曲線に変換する
 */
std::vector<geometry_msgs::msg::Point> convertArcToClothoid(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min,
  double L_min, double point_interval)
{
  // 円弧情報の抽出
  double start_angle = arc_segment.getStartAngle();
  double end_angle = arc_segment.getEndAngle();

  double total_angle = std::abs(end_angle - start_angle);

  if (total_angle > M_PI) {
    total_angle = 2.0 * M_PI - total_angle;
  }

  double radius = arc_segment.radius;
  bool is_clockwise = arc_segment.is_clockwise;

  // クロソイドパラメータ
  double A = A_min;
  double L = L_min;
  double alpha_clothoid = (L * L) / (2.0 * A * A);  // 単一クロソイドの角度変化

  std::vector<ClothoidSegment> segments;

  if (total_angle >= 2.0 * alpha_clothoid) {
    // Case A: CAC(A, L, θ)
    double theta_arc = total_angle - 2.0 * alpha_clothoid;

    // エントリクロソイド
    ClothoidSegment entry(ClothoidSegment::CLOTHOID_ENTRY, A, L);
    entry.radius = radius;
    entry.is_clockwise = is_clockwise;
    entry.description = "Entry clothoid (κ: 0 → 1/R)";
    segments.push_back(entry);

    // 円弧セグメント
    ClothoidSegment circular(ClothoidSegment::CIRCULAR_ARC);
    circular.radius = radius;
    circular.angle = theta_arc;
    circular.is_clockwise = is_clockwise;
    circular.description = "Circular arc (κ = 1/R = " + std::to_string(1.0 / radius) + ")";
    segments.push_back(circular);

    // エグジットクロソイド
    ClothoidSegment exit(ClothoidSegment::CLOTHOID_EXIT, A, L);
    exit.radius = radius;
    exit.is_clockwise = is_clockwise;
    exit.description = "Exit clothoid (κ: 1/R → 0)";
    segments.push_back(exit);
  } else {
    std::cerr << "Case B is not implemented. Please use Case A conditions." << std::endl;
    return {};
  }

  // クロソイド経路生成
  std::vector<geometry_msgs::msg::Point> clothoid_path =
    generateClothoidPath(segments, point_interval, start_pose);

  return clothoid_path;
}

/**
 * @brief 改良版のクロソイド変換関数（終点補正付き）
 */
std::vector<geometry_msgs::msg::Point> convertArcToClothoidWithCorrection(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose,
  double initial_velocity, double wheel_base, double max_steer_angle_rate, double point_interval)
{
  // 最小半径を計算（arc_segmentから）
  const double minimum_radius = arc_segment.radius;

  // 車両パラメータから最適なクロソイドパラメータを計算
  const double circular_steer_angle = std::atan(wheel_base / minimum_radius);
  const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
  const double L_min = initial_velocity * minimum_steer_time;
  const double A_min = std::sqrt(minimum_radius * L_min);

  // 元のクロソイド変換を実行
  auto clothoid_points =
    convertArcToClothoid(arc_segment, start_pose, A_min, L_min, point_interval);

  if (clothoid_points.empty()) {
    std::cerr << "Clothoid conversion failed!" << std::endl;
    return clothoid_points;
  }

  // 終点補正を適用
  auto corrected_points = correctClothoidByRigidTransform(clothoid_points, arc_segment, start_pose);

  return corrected_points;
}

/**
 * @brief クロソイドパスからPathWithLaneIdを生成する関数
 * @param clothoid_paths クロソイドパスの配列
 * @param target_pose 目標姿勢
 * @param velocity 初期速度
 * @param target_velocity 目標速度
 * @param acceleration 加速度
 * @param road_lanes 道路レーン情報
 * @param route_handler ルートハンドラー
 * @param parameters パラメータ
 * @return PathWithLaneId
 */
// std::optional<PathWithLaneId> でよさそう
PathWithLaneId createPathWithLaneIdFromClothoidPaths(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & clothoid_paths,
  const geometry_msgs::msg::Pose & target_pose, double velocity, double target_velocity,
  double acceleration, const lanelet::ConstLanelets & road_lanes,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  (void)target_pose;  // unused parameter警告抑制
  // クロソイドパスが空の場合は空のPathWithLaneIdを返す
  if (clothoid_paths.empty()) {
    PathWithLaneId empty_path;
    empty_path.header = route_handler->getRouteHeader();
    return empty_path;
  }

  // 全てのクロソイドパスを結合
  std::vector<geometry_msgs::msg::Point> all_clothoid_points;
  for (const auto & path : clothoid_paths) {
    // パスが空でない場合のみ処理
    if (path.empty()) {
      continue;
    }

    // 最初のパス以外は最初の点をスキップ（重複回避）
    // スキップしていいの？
    size_t start_idx = (all_clothoid_points.empty()) ? 0 : 1;
    for (size_t j = start_idx; j < path.size(); ++j) {
      all_clothoid_points.push_back(path[j]);
    }
  }

  // 結合後も空の場合は空のPathWithLaneIdを返す
  if (all_clothoid_points.empty()) {
    PathWithLaneId empty_path;
    empty_path.header = route_handler->getRouteHeader();
    return empty_path;
  }

  // PathWithLaneIdを作成
  PathWithLaneId path_with_lane_id;
  path_with_lane_id.header = route_handler->getRouteHeader();

  // 各座標点をPathPointWithLaneIdに変換
  for (size_t i = 0; i < all_clothoid_points.size(); ++i) {
    PathPointWithLaneId path_point;

    // 座標設定
    path_point.point.pose.position = all_clothoid_points[i];

    // // 向きを計算（次の点への方向）
    // // 計算方法怪しい？
    // if (i < all_clothoid_points.size() - 1) {
    //   const double dx = all_clothoid_points[i + 1].x - all_clothoid_points[i].x;
    //   const double dy = all_clothoid_points[i + 1].y - all_clothoid_points[i].y;
    //   const double yaw = std::atan2(dy, dx);
    //   // quaternionを直接設定
    //   path_point.point.pose.orientation.x = 0.0;
    //   path_point.point.pose.orientation.y = 0.0;
    //   path_point.point.pose.orientation.z = std::sin(yaw / 2.0);
    //   path_point.point.pose.orientation.w = std::cos(yaw / 2.0);
    // } else {
    //   // 最後の点も同様に、前の点との方向から計算
    //   if (all_clothoid_points.size() >= 2) {
    //     const double dx = all_clothoid_points[i].x - all_clothoid_points[i - 1].x;
    //     const double dy = all_clothoid_points[i].y - all_clothoid_points[i - 1].y;
    //     const double yaw = std::atan2(dy, dx);
    //     path_point.point.pose.orientation.x = 0.0;
    //     path_point.point.pose.orientation.y = 0.0;
    //     path_point.point.pose.orientation.z = std::sin(yaw / 2.0);
    //     path_point.point.pose.orientation.w = std::cos(yaw / 2.0);
    //   } else {
    //     // 1点しかない場合は0
    //     path_point.point.pose.orientation.x = 0.0;
    //     path_point.point.pose.orientation.y = 0.0;
    //     path_point.point.pose.orientation.z = 0.0;
    //     path_point.point.pose.orientation.w = 1.0;
    //   }
    // }

    // z座標の設定: レーンレット情報から最も近い点のz値を取得
    if (!road_lanes.empty()) {
      // 最も近いレーンレットを見つける
      lanelet::Lanelet closest_lanelet;
      if (lanelet::utils::query::getClosestLanelet(
            road_lanes, path_point.point.pose, &closest_lanelet)) {
        // レーンレットのセンターラインから最も近い点のz値を取得
        const auto centerline = closest_lanelet.centerline();
        if (!centerline.empty()) {
          double min_distance = std::numeric_limits<double>::max();
          double closest_z = all_clothoid_points[i].z;  // デフォルトは元のz値

          for (const auto & point : centerline) {
            const double distance = std::hypot(
              point.x() - all_clothoid_points[i].x, point.y() - all_clothoid_points[i].y);
            if (distance < min_distance) {
              min_distance = distance;
              closest_z = point.z();
            }
          }
          path_point.point.pose.position.z = closest_z;
        }
      }
    }

    // 速度プロファイルの計算（一定加速度で加速）
    double current_velocity;
    if (i == 0) {
      // 最初の点は初期速度
      current_velocity = velocity;
    } else {
      // 累積距離を計算
      double accumulated_distance = 0.0;
      for (size_t j = 0; j < i; ++j) {
        const double dx = all_clothoid_points[j + 1].x - all_clothoid_points[j].x;
        const double dy = all_clothoid_points[j + 1].y - all_clothoid_points[j].y;
        accumulated_distance += std::sqrt(dx * dx + dy * dy);
      }

      // 等加速度運動の公式: v^2 = v0^2 + 2*a*s
      // ただし、目標速度を超えないように制限
      double calculated_velocity =
        std::sqrt(velocity * velocity + 2.0 * acceleration * accumulated_distance);
      current_velocity = std::min(calculated_velocity, target_velocity);
    }

    // 速度設定
    path_point.point.longitudinal_velocity_mps = current_velocity;
    path_point.point.lateral_velocity_mps = 0.0;
    path_point.point.heading_rate_rps = 0.0;
    path_point.point.is_final = false;

    // レーンIDの設定
    std::vector<int64_t> previous_lane_ids;
    if (i > 0) {
      previous_lane_ids = path_with_lane_id.points[i - 1].lane_ids;
    }
    setLaneIdsToPathPoint(path_point, road_lanes, previous_lane_ids);

    path_with_lane_id.points.push_back(path_point);
  }

  return path_with_lane_id;
  // return autoware::behavior_path_planner::utils::resamplePathWithSpline(path_with_lane_id, 1.0);
}

/**
 * @brief 円弧パスのセグメントをクロソイド曲線に変換する関数
 * @param circular_path 円弧パス（2つのセグメントを含む）
 * @param start_pose 開始姿勢
 * @param initial_velocity 初期速度
 * @param wheel_base ホイールベース
 * @param max_steer_angle_rate 最大ステア角速度
 * @param point_interval 点間隔
 * @return クロソイドパスの配列
 */
std::vector<std::vector<geometry_msgs::msg::Point>> convertCircularPathToClothoidPaths(
  const CompositeArcPath & circular_path, const geometry_msgs::msg::Pose & start_pose,
  double initial_velocity, double wheel_base, double max_steer_angle_rate, double point_interval)
{
  std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

  if (circular_path.segments.size() < 2) {
    std::cerr << "Circular path must have at least 2 segments" << std::endl;
    return clothoid_paths;
  }

  geometry_msgs::msg::Pose current_segment_pose = start_pose;

  // 第1セグメント（開始セグメント）の処理
  const auto & first_segment = circular_path.segments[0];
  auto first_clothoid_points = convertArcToClothoidWithCorrection(
    first_segment, current_segment_pose, initial_velocity, wheel_base, max_steer_angle_rate,
    point_interval);

  clothoid_paths.push_back(first_clothoid_points);

  // 第1セグメント終了時の姿勢を計算（第2セグメントの開始姿勢として使用）
  geometry_msgs::msg::Pose second_segment_start_pose;
  const auto & last_point_first = first_clothoid_points.back();
  second_segment_start_pose.position = last_point_first;

  // 終点での進行方向を計算（最後の2点から）
  if (first_clothoid_points.size() >= 2) {
    // TODO(Sugahara): ここでyawの計算方法あってる？
    const auto & second_last_first = first_clothoid_points[first_clothoid_points.size() - 2];
    double dx = last_point_first.x - second_last_first.x;
    double dy = last_point_first.y - second_last_first.y;
    double heading = std::atan2(dy, dx);
    second_segment_start_pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
  } else {
    second_segment_start_pose.orientation = current_segment_pose.orientation;
  }

  // 第2セグメント（終了セグメント）の処理
  const auto & second_segment = circular_path.segments[1];
  auto second_clothoid_points = convertArcToClothoidWithCorrection(
    second_segment, second_segment_start_pose, initial_velocity, wheel_base, max_steer_angle_rate,
    point_interval);

  if (second_clothoid_points.empty()) {
    std::cerr << "Failed to convert second segment to clothoid" << std::endl;
    return clothoid_paths;
  }
  clothoid_paths.push_back(second_clothoid_points);

  return clothoid_paths;
}

/**
 * @brief センターラインパスとクロソイドパスを結合する関数
 */
PathWithLaneId combinePathWithCenterline(
  const PathWithLaneId & clothoid_path, const PathWithLaneId & centerline_path,
  const geometry_msgs::msg::Pose & target_pose)
{
  // センターラインパスが空の場合はクロソイドパスをそのまま返す
  if (centerline_path.points.empty()) {
    return clothoid_path;
  }

  // target_poseの位置でcenterline_pathから接続点を見つける
  const auto target_idx =
    autoware::motion_utils::findNearestIndex(centerline_path.points, target_pose.position);

  // target_poseから先のcenterline pathを取得
  if (target_idx < centerline_path.points.size()) {
    PathWithLaneId centerline_extension;
    centerline_extension.header = centerline_path.header;
    std::cerr << "target_pose: " << target_pose.position.x << ", " << target_pose.position.y
              << std::endl;
    // target_poseから先の点をcenterline_extensionに追加
    // どこまで追加する？？？？
    for (size_t i = target_idx; i < centerline_path.points.size(); ++i) {
      centerline_extension.points.push_back(centerline_path.points[i]);
    }

    // centerline extensionが存在する場合、既存のpathと結合
    if (!centerline_extension.points.empty()) {
      // 重複点を避けて結合
      return utils::combinePath(clothoid_path, centerline_extension);
    }
  }
  return clothoid_path;
}

/**
 * @brief start_poseから前後に直進するposes配列を生成する関数
 * @param start_pose 開始姿勢
 * @param forward_distance 前方直進距離[m]
 * @param backward_distance 後方直進距離[m]
 * @param point_interval 点間隔[m]
 * @return poses配列（後方→start_pose→前方の順）
 */
std::vector<geometry_msgs::msg::Pose> createStraightPathToEndPose(
  const geometry_msgs::msg::Pose & start_pose, double forward_distance, double backward_distance,
  double point_interval)
{
  // 直進方向（yaw角）を計算
  const double start_yaw = tf2::getYaw(start_pose.orientation);

  // poseの配列を格納
  std::vector<geometry_msgs::msg::Pose> poses;

  // 1. 後方経路を生成（最も遠い後退点から順番に生成）
  if (backward_distance > 0.0) {
    const int backward_num_points = static_cast<int>(backward_distance / point_interval);

    for (int i = backward_num_points; i >= 1; --i) {
      geometry_msgs::msg::Pose pose;
      double distance = i * point_interval;
      pose.position.x = start_pose.position.x - distance * std::cos(start_yaw);
      pose.position.y = start_pose.position.y - distance * std::sin(start_yaw);
      pose.position.z = start_pose.position.z;
      pose.orientation = start_pose.orientation;
      poses.push_back(pose);
    }
  }

  // 2. start_poseを追加
  poses.push_back(start_pose);

  // 3. 前方経路を生成
  if (forward_distance > 0.0) {
    // 前方終了姿勢を計算
    geometry_msgs::msg::Pose forward_end_pose = start_pose;
    forward_end_pose.position.x = start_pose.position.x + forward_distance * std::cos(start_yaw);
    forward_end_pose.position.y = start_pose.position.y + forward_distance * std::sin(start_yaw);
    forward_end_pose.orientation = start_pose.orientation;

    // 前方点数を計算（start_poseは既に追加済みなので除外）
    const int forward_num_points =
      std::max(1, static_cast<int>(std::ceil(forward_distance / point_interval)));

    // 実際の間隔を再計算（等間隔にするため）
    const double actual_interval = forward_distance / forward_num_points;

    // 前方各点を生成（start_pose以降）
    for (int i = 1; i <= forward_num_points; ++i) {
      geometry_msgs::msg::Pose pose;

      if (i == forward_num_points) {
        // 最終点（正確にforward_end_poseにする）
        pose = forward_end_pose;
      } else {
        // 中間点
        const double distance = i * actual_interval;
        pose.position.x = start_pose.position.x + distance * std::cos(start_yaw);
        pose.position.y = start_pose.position.y + distance * std::sin(start_yaw);
        pose.position.z = start_pose.position.z;
        pose.orientation = start_pose.orientation;
      }

      poses.push_back(pose);
    }
  }

  return poses;
}

ClothoidPullOut::ClothoidPullOut(
  rclcpp::Node & node, const StartPlannerParameters & parameters,
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
: PullOutPlannerBase{node, parameters, time_keeper}
{
  autoware::boundary_departure_checker::Param boundary_departure_checker_params;
  boundary_departure_checker_params.footprint_extra_margin =
    parameters.lane_departure_check_expansion_margin;
  boundary_departure_checker_ =
    std::make_shared<autoware::boundary_departure_checker::BoundaryDepartureChecker>(
      boundary_departure_checker_params, vehicle_info_, time_keeper_);
}

std::optional<PullOutPath> ClothoidPullOut::plan(
  const Pose & start_pose, const Pose & /*goal_pose*/,
  const std::shared_ptr<const PlannerData> & planner_data,
  PlannerDebugData & /*planner_debug_data*/)
{
  const double initial_velocity = parameters_.clothoid_initial_velocity;
  // 加速度パラメータの設定
  const double acceleration = parameters_.clothoid_acceleration;
  const std::vector<double> max_steer_angle_degs = parameters_.clothoid_max_steer_angle_degs;
  // パラメータから度をラジアンに変換
  std::vector<double> max_steer_angle;
  for (const auto & deg : max_steer_angle_degs) {
    max_steer_angle.push_back(deg * M_PI / 180.0);
  }

  const double max_steer_angle_rate_deg_per_sec =
    parameters_.clothoid_max_steer_angle_rate_deg_per_sec;
  const double max_steer_angle_rate = max_steer_angle_rate_deg_per_sec * M_PI / 180.0;
  constexpr double initial_forward_straight_distance = 3.0;  // [m] 直進区間長さ（仮）

  const auto & route_handler = planner_data->route_handler;
  const auto & common_parameters = planner_data->parameters;

  const double wheel_base = common_parameters.vehicle_info.wheel_base_m;
  const double backward_path_length =
    planner_data->parameters.backward_path_length + parameters_.max_back_distance;
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // pull_out_lanes を取得（shoulder lane を含む）
  const auto pull_out_lanes = getPullOutLanes(planner_data, backward_path_length);

  // road_lanes と pull_out_lanes を結合して全てのレーンを含める
  const auto all_lanes = utils::combineLanelets(road_lanes, pull_out_lanes);

  // Generate centerline path from road_lanes
  const auto row_centerline_path = utils::getCenterLinePath(
    *route_handler, road_lanes, start_pose, backward_path_length,
    std::numeric_limits<double>::max(), common_parameters);

  PathWithLaneId centerline_path =
    utils::resamplePathWithSpline(row_centerline_path, parameters_.center_line_path_interval);

  // =====================================================================
  // 追加: 初期直進距離を設定し，その区間の終端点を start_pose として使用できるよう
  //       直進区間の PathPoint 群を生成しておく．
  //       現状はパラメータ化せず固定長さとする（TODO: パラメータ化）。
  // =====================================================================

  // =====================================================================
  // 前後直進パス生成（全ステア角度共通）
  // =====================================================================
  const double backward_distance = 10.0;  // 後退距離[m]

  // createStraightPathToEndPose関数を使用して前後直進経路を生成
  auto straight_poses = createStraightPathToEndPose(
    start_pose, initial_forward_straight_distance, backward_distance,
    parameters_.center_line_path_interval);

  // straight_posesからPathPointWithLaneIdを生成
  std::vector<PathPointWithLaneId> straight_forward_points;
  for (size_t i = 0; i < straight_poses.size(); ++i) {
    PathPointWithLaneId pt;
    pt.point.pose = straight_poses[i];
    pt.point.longitudinal_velocity_mps = initial_velocity;
    pt.point.lateral_velocity_mps = 0.0;
    pt.point.heading_rate_rps = 0.0;
    pt.point.is_final = false;

    // レーンIDの設定
    std::vector<int64_t> previous_lane_ids;
    if (i > 0) {
      previous_lane_ids = straight_forward_points[i - 1].lane_ids;
    }
    setLaneIdsToPathPoint(pt, all_lanes, previous_lane_ids);

    straight_forward_points.push_back(pt);
  }

  const Pose straight_end_pose = straight_poses.back();

  // lateral_offset: returns positive value when straight_end_pose.position is on the left side of
  // the trajectory segment
  const double lateral_offset = centerline_path.points.empty()
                                  ? 0.0
                                  : autoware::motion_utils::calcLateralOffset(
                                      centerline_path.points, straight_end_pose.position);

  for (const auto & steer_angle : max_steer_angle) {
    // =====================================================================
    // クロソイドパス生成（ステア角度毎に異なる）
    // =====================================================================
    // Calculate minimum radius based on the maximum steer angle
    const double minimum_radius = wheel_base / std::tan(steer_angle);

    const double longitudinal_distance =
      start_planner_utils::calc_necessary_longitudinal_distance(-lateral_offset, minimum_radius);

    const Pose target_pose = start_planner_utils::findTargetPoseAlongPath(
      centerline_path, straight_end_pose, longitudinal_distance);

    // TODO(Sugahara): ここでlateral_offset がプラスな場合は直進経路でよい。
    const auto relative_pose_info =
      start_planner_utils::calculateRelativePoseInVehicleCoordinate(straight_end_pose, target_pose);
    // std::cerr << "target_pose: x=" << target_pose.position.x << ", y=" << target_pose.position.y
    //           << ", yaw=" << tf2::getYaw(target_pose.orientation) * 180.0 / M_PI << " deg"
    //           << std::endl;

    const auto circular_path = start_planner_utils::calc_circular_path(
      straight_end_pose, relative_pose_info.longitudinal_distance_vehicle,
      relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff, minimum_radius);

    if (circular_path.segments.empty()) {
      // TODO(Sugahara): steer_angle, 縦距離、横距離、角度差、最小半径をデバッグ出力
      std::cerr << "No circular path segments found for steer angle " << steer_angle * 180.0 / M_PI
                << " deg." << std::endl;
      continue;
    }

    geometry_msgs::msg::Pose current_segment_pose = straight_end_pose;
    std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

    // 円弧パスをクロソイド曲線に変換
    clothoid_paths = convertCircularPathToClothoidPaths(
      circular_path, current_segment_pose, initial_velocity, wheel_base, max_steer_angle_rate,
      parameters_.center_line_path_interval);

    if (clothoid_paths.empty()) {
      std::cerr << "Failed to convert circular path to clothoid paths for steer angle "
                << steer_angle * 180.0 / M_PI << " deg." << std::endl;
      continue;
    }

    // 目標速度を取得（centerline_pathからtarget_poseに最も近い点の速度を使用）
    // TODO(Sugahara): 関数化
    double target_velocity = initial_velocity;  // デフォルト値
    if (!centerline_path.points.empty()) {
      const auto target_idx =
        autoware::motion_utils::findNearestIndex(centerline_path.points, target_pose.position);
      if (target_idx < centerline_path.points.size()) {
        target_velocity = centerline_path.points[target_idx].point.longitudinal_velocity_mps;
      }
    }

    // =====================================================================
    // クロソイドパスをセンターラインに結合
    // =====================================================================
    PathWithLaneId path_with_lane_id = createPathWithLaneIdFromClothoidPaths(
      clothoid_paths, target_pose, initial_velocity, target_velocity, acceleration, all_lanes,
      route_handler);

    // センターラインパスとの結合
    auto combined_path = combinePathWithCenterline(path_with_lane_id, centerline_path, target_pose);

    // printPathWithLaneIdDetails(combined_path, "combined_path");

    PathWithLaneId resampled_combined_path =
      utils::resamplePathWithSpline(combined_path, parameters_.center_line_path_interval);
    // autoware::interpolation::lerpによる等間隔リサンプリング
    // PathWithLaneId resampled_combined_path = combined_path;
    // if (combined_path.points.size() >= 2) {
    //   // 1. arclength配列
    //   std::vector<double> arclengths(combined_path.points.size(), 0.0);
    //   for (size_t i = 1; i < combined_path.points.size(); ++i) {
    //     const auto & p0 = combined_path.points[i - 1].point.pose.position;
    //     const auto & p1 = combined_path.points[i].point.pose.position;
    //     arclengths[i] = arclengths[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y);
    //   }
    //   // 2. 新しいarclength配列
    //   std::vector<double> query_s;
    //   for (double s = 0.0; s < arclengths.back(); s += parameters_.center_line_path_interval)
    //     query_s.push_back(s);
    //   if (query_s.empty() || query_s.back() < arclengths.back())
    //     query_s.push_back(arclengths.back());
    //   // 3. 各値をlerp補間
    //   std::vector<double> xs, ys, zs, yaws, vels;
    //   for (const auto & pt : combined_path.points) {
    //     xs.push_back(pt.point.pose.position.x);
    //     ys.push_back(pt.point.pose.position.y);
    //     zs.push_back(pt.point.pose.position.z);
    //     vels.push_back(pt.point.longitudinal_velocity_mps);
    //     yaws.push_back(tf2::getYaw(pt.point.pose.orientation));
    //   }
    //   auto lerp_x = autoware::interpolation::lerp(arclengths, xs, query_s);
    //   auto lerp_y = autoware::interpolation::lerp(arclengths, ys, query_s);
    //   auto lerp_z = autoware::interpolation::lerp(arclengths, zs, query_s);
    //   auto lerp_yaw = autoware::interpolation::lerp(arclengths, yaws, query_s);
    //   auto lerp_vel = autoware::interpolation::lerp(arclengths, vels, query_s);
    //   // 4. PathWithLaneId生成
    //   resampled_combined_path = combined_path;
    //   resampled_combined_path.points.clear();
    //   for (size_t i = 0; i < query_s.size(); ++i) {
    //     PathPointWithLaneId pt;
    //     pt.point.pose.position.x = lerp_x[i];
    //     pt.point.pose.position.y = lerp_y[i];
    //     pt.point.pose.position.z = lerp_z[i];
    //     pt.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(lerp_yaw[i]);
    //     pt.point.longitudinal_velocity_mps = lerp_vel[i];
    //     pt.point.is_final = false;
    //     setLaneIdsToPathPoint(pt, all_lanes);
    //     resampled_combined_path.points.push_back(pt);
    //   }
    // }
    // printPathWithLaneIdDetails(resampled_combined_path, "resampled_combined_path");

    // -----------------------------------------------------------------
    // パス結合: 前後直進パス → クロソイドパス → センターライン拡張パス
    // の順序で結合する（前後直進パスは既に統合済み）
    // -----------------------------------------------------------------
    PathWithLaneId final_path;
    final_path.header = resampled_combined_path.header;
    final_path.points = straight_forward_points;  // 前後直進パス（統合済み）

    // クロソイドパス + センターライン拡張パスを追加
    final_path.points.insert(
      final_path.points.end(), resampled_combined_path.points.begin(),
      resampled_combined_path.points.end());
    // 速度と加速度のペア設定
    // PullOutPath pull_out_path;
    // // TODO(Sugahara): set parameter properly
    // pull_out_path.pairs_terminal_velocity_and_accel.push_back(
    //   std::make_pair(initial_velocity, 1.0));
    // pull_out_path.partial_paths.push_back(final_path);
    // // PullOutPathを作成

    // pull_out_path.start_pose =
    //   straight_forward_points.front().point.pose;  // 最も遠い後退点から開始
    // pull_out_path.end_pose = target_pose;
    // デバッグ用：生成されたパスの詳細を出力
    // printPathWithLaneIdDetails(final_path, "Final ClothoidPullOutPath");

    // =====================================================================
    // 車線逸脱判定とパス検証（shift_pull_out.cppを参考に実装）
    // =====================================================================
    const auto lanelet_map_ptr = planner_data->route_handler->getLaneletMapPtr();

    std::vector<lanelet::Id> fused_id_start_to_end{};
    std::optional<autoware_utils::Polygon2d> fused_polygon_start_to_end = std::nullopt;

    std::vector<lanelet::Id> fused_id_crop_points{};
    std::optional<autoware_utils::Polygon2d> fused_polygon_crop_points = std::nullopt;

    // clothoid path is not separate but only one.
    auto & clothoid_path = final_path;

    // check lane_departure with path between pull_out_start to pull_out_end
    PathWithLaneId path_clothoid_start_to_end{};
    {
      const size_t pull_out_start_idx =
        autoware::motion_utils::findNearestIndex(clothoid_path.points, start_pose.position);
      const size_t pull_out_end_idx =
        autoware::motion_utils::findNearestIndex(clothoid_path.points, target_pose.position);

      path_clothoid_start_to_end.points.insert(
        path_clothoid_start_to_end.points.begin(),
        clothoid_path.points.begin() + pull_out_start_idx,
        clothoid_path.points.begin() + pull_out_end_idx + 1);
    }

    // check lane departure
    // The method for lane departure checking verifies if the footprint of each point on the path
    // is contained within a lanelet using `boost::geometry::within`, which incurs a high
    // computational cost.
    if (parameters_.check_clothoid_path_lane_departure &&
        boundary_departure_checker_->checkPathWillLeaveLane(
          lanelet_map_ptr, path_clothoid_start_to_end, fused_id_start_to_end,
          fused_polygon_start_to_end)) {
      std::cerr << "Lane departure detected for steer angle " << steer_angle * 180.0 / M_PI
                << " deg. Continuing to next candidate." << std::endl;
      continue;
    }

    // crop backward path
    // removes points which are out of lanes up to the start pose.
    // this ensures that the backward_path stays within the drivable area when starting from a
    // narrow place.
    const size_t start_segment_idx =
      autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        clothoid_path.points, start_pose, common_parameters.ego_nearest_dist_threshold,
        common_parameters.ego_nearest_yaw_threshold);

    PathWithLaneId cropped_path;
    if (parameters_.check_clothoid_path_lane_departure) {
      cropped_path = boundary_departure_checker_->cropPointsOutsideOfLanes(
        lanelet_map_ptr, clothoid_path, start_segment_idx, fused_id_crop_points,
        fused_polygon_crop_points);
      if (cropped_path.points.empty()) {
        std::cerr << "Cropped path is empty for steer angle " << steer_angle * 180.0 / M_PI
                  << " deg. Continuing to next candidate." << std::endl;
        continue;
      }
    } else {
      // If lane departure check is disabled, use the original path without cropping
      cropped_path = clothoid_path;
    }

    // check that the path is not cropped in excess and there is not excessive longitudinal
    // deviation between the first 2 points
    auto validate_cropped_path = [&](const auto & cropped_path) -> bool {
      if (cropped_path.points.size() < 2) return false;
      const double max_long_offset = parameters_.maximum_longitudinal_deviation;
      const size_t start_segment_idx_after_crop =
        autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
          cropped_path.points, start_pose);

      // if the start segment id after crop is not 0, then the cropping is not excessive
      if (start_segment_idx_after_crop != 0) return true;

      const auto long_offset_to_closest_point =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          cropped_path.points, start_segment_idx_after_crop, start_pose.position);
      const auto long_offset_to_next_point =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          cropped_path.points, start_segment_idx_after_crop + 1, start_pose.position);
      return std::abs(long_offset_to_closest_point - long_offset_to_next_point) < max_long_offset;
    };

    if (parameters_.check_clothoid_path_lane_departure && !validate_cropped_path(cropped_path)) {
      std::cerr << "Cropped path is invalid for steer angle " << steer_angle * 180.0 / M_PI
                << " deg. Continuing to next candidate." << std::endl;
      continue;
    }

    // Update the final path with cropped path
    clothoid_path.points = cropped_path.points;
    clothoid_path.header = planner_data->route_handler->getRouteHeader();

    // Create PullOutPath for collision check
    PullOutPath temp_pull_out_path;
    temp_pull_out_path.partial_paths.push_back(clothoid_path);
    temp_pull_out_path.start_pose =
      clothoid_path.points.empty() ? start_pose : clothoid_path.points.front().point.pose;
    temp_pull_out_path.end_pose = target_pose;

    if (isPullOutPathCollided(
          temp_pull_out_path, planner_data, parameters_.shift_collision_check_distance_from_end)) {
      std::cerr << "Collision detected for steer angle " << steer_angle * 180.0 / M_PI
                << " deg. Continuing to next candidate." << std::endl;
      continue;
    }

    // 検証に成功したら、最終的なPullOutPathを作成して返す
    PullOutPath pull_out_path;
    // TODO(Sugahara): set parameter properly
    pull_out_path.pairs_terminal_velocity_and_accel.push_back(
      std::make_pair(initial_velocity, 1.0));
    pull_out_path.partial_paths.push_back(clothoid_path);  // Use validated and cropped path

    pull_out_path.start_pose =
      clothoid_path.points.empty() ? start_pose : clothoid_path.points.front().point.pose;
    pull_out_path.end_pose = target_pose;

    RCLCPP_ERROR(
      rclcpp::get_logger("clothoid_pull_out"),
      "\n===========================================\n"
      "Successfully generated clothoid pull-out path with steer angle %.2f deg.\n"
      "===========================================",
      steer_angle * 180.0 / M_PI);

    return pull_out_path;
  }

  // 経路が生成できなかった場合
  return std::nullopt;
}

// PathWithLaneIdの各点の詳細情報をプリントする関数
void printPathWithLaneIdDetails(const PathWithLaneId & path, const std::string & path_name)
{
  std::cout << "=== " << path_name << " Details ===" << std::endl;
  std::cout << "Total points: " << path.points.size() << std::endl;

  double cumulative_distance = 0.0;

  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & point = path.points[i];
    const auto & pose = point.point.pose;
    const auto & position = pose.position;
    const auto & orientation = pose.orientation;

    // ヨー角を計算
    const double yaw = tf2::getYaw(orientation);

    // 1つ前の点との距離を計算
    double distance_from_prev = 0.0;
    if (i > 0) {
      const auto & prev_position = path.points[i - 1].point.pose.position;
      distance_from_prev = std::sqrt(
        std::pow(position.x - prev_position.x, 2) + std::pow(position.y - prev_position.y, 2));
      cumulative_distance += distance_from_prev;
    }

    // lane_idsを文字列に変換
    std::string lane_ids_str = "[";
    for (size_t j = 0; j < point.lane_ids.size(); ++j) {
      if (j > 0) lane_ids_str += ", ";
      lane_ids_str += std::to_string(point.lane_ids[j]);
    }
    lane_ids_str += "]";

    // 情報をプリント
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "[" << std::setw(3) << i << "] "
              << "x=" << std::setw(10) << position.x << " "
              << "y=" << std::setw(10) << position.y << " "
              << "z=" << std::setw(10) << position.z << " "
              << "yaw=" << std::setw(8) << yaw << " rad "
              << "(" << std::setw(6) << yaw * 180.0 / M_PI << "°) "
              << "quat[" << std::setw(7) << orientation.x << ", " << std::setw(7) << orientation.y
              << ", " << std::setw(7) << orientation.z << ", " << std::setw(7) << orientation.w
              << "] "
              << "dist_prev=" << std::setw(8) << distance_from_prev << " "
              << "cumul=" << std::setw(8) << cumulative_distance << " "
              << "lane_ids=" << lane_ids_str << std::endl;
  }

  std::cout << "Total path length: " << cumulative_distance << " m" << std::endl;
  std::cout << "=================================" << std::endl;
}

}  // namespace autoware::behavior_path_planner
