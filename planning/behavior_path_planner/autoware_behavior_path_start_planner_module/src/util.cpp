// Copyright 2021 Tier IV, Inc.
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

#include "autoware/behavior_path_start_planner_module/util.hpp"

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::start_planner_utils
{
PathWithLaneId getBackwardPath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & shoulder_lanes,
  const Pose & current_pose, const Pose & backed_pose, const double velocity)
{
  const auto current_pose_arc_coords =
    lanelet::utils::getArcCoordinates(shoulder_lanes, current_pose);
  const auto backed_pose_arc_coords =
    lanelet::utils::getArcCoordinates(shoulder_lanes, backed_pose);

  const double s_start = backed_pose_arc_coords.length;
  const double s_end = current_pose_arc_coords.length;

  PathWithLaneId backward_path;
  {
    // forward center line path
    backward_path = route_handler.getCenterLinePath(shoulder_lanes, s_start, s_end, true);

    // If the returned path is empty, return an empty path
    if (backward_path.points.empty()) {
      return backward_path;
    }

    // backward center line path
    std::reverse(backward_path.points.begin(), backward_path.points.end());
    for (auto & p : backward_path.points) {
      p.point.longitudinal_velocity_mps = velocity;
    }
    backward_path.points.back().point.longitudinal_velocity_mps = 0.0;

    // lateral shift to current_pose
    const double lateral_distance_to_shoulder_center = current_pose_arc_coords.distance;
    for (size_t i = 0; i < backward_path.points.size(); ++i) {
      auto & p = backward_path.points.at(i).point.pose;
      p = autoware_utils::calc_offset_pose(p, 0, lateral_distance_to_shoulder_center, 0);
    }
  }

  return backward_path;
}

lanelet::ConstLanelets getPullOutLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length)
{
  const double & vehicle_width = planner_data->parameters.vehicle_width;
  const auto & route_handler = planner_data->route_handler;
  const auto start_pose = planner_data->route_handler->getOriginalStartPose();

  const auto current_shoulder_lane = route_handler->getPullOutStartLane(start_pose, vehicle_width);
  if (current_shoulder_lane) {
    // pull out from shoulder lane
    return route_handler->getShoulderLaneletSequence(*current_shoulder_lane, start_pose);
  }

  // pull out from road lane
  return utils::getExtendedCurrentLanes(
    planner_data, backward_length,
    /*forward_length*/ std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
}

std::optional<PathWithLaneId> extractCollisionCheckSection(
  const PullOutPath & path, const double collision_check_distance_from_end)
{
  PathWithLaneId full_path;
  for (const auto & partial_path : path.partial_paths) {
    full_path.points.insert(
      full_path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  if (full_path.points.empty()) return std::nullopt;
  // Find the start index for collision check section based on the shift start pose
  const auto shift_start_idx =
    autoware::motion_utils::findNearestIndex(full_path.points, path.start_pose.position);

  // Find the end index for collision check section based on the end pose and collision check
  // distance
  const auto collision_check_end_idx = [&]() -> size_t {
    const auto end_pose_offset = autoware::motion_utils::calcLongitudinalOffsetPose(
      full_path.points, path.end_pose.position, collision_check_distance_from_end);

    return end_pose_offset
             ? autoware::motion_utils::findNearestIndex(full_path.points, end_pose_offset->position)
             : full_path.points.size() - 1;  // Use the last point if offset pose is not calculable
  }();

  // Extract the collision check section from the full path
  PathWithLaneId collision_check_section;
  if (shift_start_idx < collision_check_end_idx) {
    collision_check_section.points.assign(
      full_path.points.begin() + shift_start_idx,
      full_path.points.begin() + collision_check_end_idx + 1);
  }

  return collision_check_section;
}

namespace
{

struct PathEvaluationResult
{
  double distance;
  double arc1_length;
  double lateral_error;
  bool is_valid;

  PathEvaluationResult() : distance(0.0), arc1_length(0.0), lateral_error(0.0), is_valid(false) {}
  PathEvaluationResult(double d, double arc_len, double error, bool valid)
  : distance(d), arc1_length(arc_len), lateral_error(error), is_valid(valid)
  {
  }
};

}  // anonymous namespace

double calc_necessary_longitudinal_distance(
  const double lateral_offset, const double minimum_radius)
{
  // Trial distances based on minimum radius
  const std::vector<double> trial_distances = {
    0.5 * minimum_radius, 0.75 * minimum_radius, 1.0 * minimum_radius, 1.5 * minimum_radius,
    2.0 * minimum_radius, 3.0 * minimum_radius,  4.0 * minimum_radius, 5.0 * minimum_radius,
    6.0 * minimum_radius, 8.0 * minimum_radius,  10.0 * minimum_radius};

  // Starting pose parameters (assumed at origin with 0 yaw)
  constexpr double x_start = 0.0;
  constexpr double y_start = 0.0;
  constexpr double yaw_start = 0.0;

  // Evaluation parameters
  constexpr double error_threshold = 0.5;
  constexpr double tolerance = 0.1;

  // Results tracking
  std::vector<PathEvaluationResult> evaluation_results;
  evaluation_results.reserve(trial_distances.size());

  double best_distance = 0.0;
  double best_score = -1e9;  // Prioritize longer Arc1 length
  bool found_valid = false;
  int valid_results_count = 0;

  for (const double trial_distance : trial_distances) {
    // Calculate goal position considering lateral offset
    const double x_goal =
      x_start + trial_distance * std::cos(yaw_start) + lateral_offset * (-std::sin(yaw_start));
    const double y_goal =
      y_start + trial_distance * std::sin(yaw_start) + lateral_offset * std::cos(yaw_start);

    // Calculate starting arc center (assuming clockwise rotation)
    const double center_rx = x_start + minimum_radius * std::sin(yaw_start);
    const double center_ry = y_start - minimum_radius * std::cos(yaw_start);

    // Calculate target arc radius using Al-Kashi theorem
    const double dx_goal = x_goal - center_rx;
    const double dy_goal = y_goal - center_ry;
    const double distance_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    if (distance_to_goal < 1e-6) {
      std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
                << " m - SKIPPED (goal too close to arc center)" << std::endl;
      continue;
    }

    const double cos_term = std::clamp((y_goal - center_ry) / distance_to_goal, -1.0, 1.0);
    const double alpha = M_PI + std::acos(cos_term);
    const double denominator = 2 * minimum_radius + 2 * distance_to_goal * std::cos(alpha);

    if (std::abs(denominator) < 1e-6) {
      std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
                << " m - SKIPPED (denominator too small)" << std::endl;
      continue;
    }

    const double radius_goal =
      (distance_to_goal * distance_to_goal - minimum_radius * minimum_radius) / denominator;

    // Check physical feasibility
    if (radius_goal < 0 || radius_goal < minimum_radius) {
      std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
                << " m - SKIPPED (radius infeasible: " << std::setprecision(3) << radius_goal << ")"
                << std::endl;
      continue;
    }

    // Calculate target arc center (assuming counter-clockwise rotation)
    const double center_lx = x_goal - radius_goal * std::sin(yaw_start);
    const double center_ly = y_goal + radius_goal * std::cos(yaw_start);

    // Validate arc connection
    const double dx_centers = center_lx - center_rx;
    const double dy_centers = center_ly - center_ry;
    const double distance_between_centers =
      std::sqrt(dx_centers * dx_centers + dy_centers * dy_centers);

    const double external_tangent_distance = minimum_radius + radius_goal;
    const double internal_tangent_distance = std::abs(minimum_radius - radius_goal);

    const bool connection_valid =
      (std::abs(distance_between_centers - external_tangent_distance) <= tolerance ||
       std::abs(distance_between_centers - internal_tangent_distance) <= tolerance ||
       (distance_between_centers > external_tangent_distance + tolerance &&
        distance_between_centers - external_tangent_distance <= 2.0) ||
       (distance_between_centers < internal_tangent_distance - tolerance &&
        internal_tangent_distance - distance_between_centers <=
          std::min(minimum_radius, radius_goal) * 0.8));

    if (!connection_valid) {
      std::cout << "  Trial distance: " << std::fixed << std::setprecision(2) << trial_distance
                << " m - SKIPPED (arc connection invalid)" << std::endl;
      continue;
    }

    // Calculate tangent point between circles
    double tangent_x, tangent_y;
    if (distance_between_centers < 1e-6) {
      tangent_x = (center_rx + center_lx) / 2.0;
      tangent_y = (center_ry + center_ly) / 2.0;
    } else if (std::abs(distance_between_centers - external_tangent_distance) <= tolerance) {
      // External tangent case
      const double ratio = minimum_radius / (minimum_radius + radius_goal);
      tangent_x = center_rx + ratio * dx_centers;
      tangent_y = center_ry + ratio * dy_centers;
    } else {
      // Other cases - use approximation
      const double ratio = 0.5;
      tangent_x = center_rx + ratio * dx_centers;
      tangent_y = center_ry + ratio * dy_centers;
    }

    // Calculate actual lateral offset achieved
    const double dx_actual = x_goal - x_start;
    const double dy_actual = y_goal - y_start;
    const double lateral_x = -std::sin(yaw_start);
    const double lateral_y = std::cos(yaw_start);
    const double actual_lateral_offset = dx_actual * lateral_x + dy_actual * lateral_y;
    const double lateral_error = std::abs(actual_lateral_offset - lateral_offset);

    // Calculate Arc1 length
    const double start_angle = std::atan2(y_start - center_ry, x_start - center_rx);
    const double tangent_angle = std::atan2(tangent_y - center_ry, tangent_x - center_rx);
    double angle_diff = tangent_angle - start_angle;

    // Adjust for clockwise direction
    if (angle_diff > 0) {
      angle_diff -= 2 * M_PI;
    }

    const double arc1_length = minimum_radius * std::abs(angle_diff);

    // Store evaluation result
    evaluation_results.emplace_back(trial_distance, arc1_length, lateral_error, true);
    valid_results_count++;

    // Update best candidate selection
    if (lateral_error <= error_threshold) {
      if (arc1_length > best_score) {
        best_score = arc1_length;
        best_distance = trial_distance;
        found_valid = true;
      }
    } else if (!found_valid && arc1_length > best_score) {
      // If no acceptable solution found yet, select the best available
      best_score = arc1_length;
      best_distance = trial_distance;
    }
  }

  // Output selection results
  std::cout << "\n--- Selection Results ---" << std::endl;
  std::cout << "Valid results: " << valid_results_count << std::endl;

  if (found_valid) {
    std::cout << "Acceptable results (error <= " << std::fixed << std::setprecision(1)
              << error_threshold << "m): found" << std::endl;
    std::cout << "Selected result: Arc1 length = " << std::setprecision(3) << best_score
              << " m, Distance = " << std::setprecision(3) << best_distance << " m" << std::endl;
  } else {
    std::cout << "No acceptable results found" << std::endl;
  }

  // Fallback if no valid solution found
  if (!found_valid && best_distance == 0.0) {
    best_distance = std::max(4.0 * minimum_radius, std::abs(lateral_offset) * 2.0);
    std::cout << "Using geometric estimation: " << std::setprecision(3) << best_distance << " m"
              << std::endl;
  }

  return best_distance;
}

CompositeArcPath calc_circular_path(
  const Pose & start_pose, const double longitudinal_distance, const double lateral_distance,
  const double angle_diff, const double minimum_radius)
{
  const double PI = M_PI;

  std::cout << "\n=== Circular Path Planning (Relative Direct) ===" << std::endl;
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Start: (" << start_pose.position.x << ", " << start_pose.position.y
            << "), yaw=" << tf2::getYaw(start_pose.orientation) * 180.0 / PI << "°" << std::endl;
  std::cout << "Relative target: longitudinal=" << longitudinal_distance
            << "m, lateral=" << lateral_distance << "m, angle_diff=" << angle_diff * 180.0 / PI
            << "°" << std::endl;

  // 相対座標系で計算（原点を開始点、X軸を進行方向とする）
  // 開始点: (0, 0, 0)
  // 目標点: (longitudinal_distance, lateral_distance, angle_diff)

  const double x_start_rel = 0.0;
  const double y_start_rel = 0.0;
  const double yaw_start_rel = 0.0;

  const double x_goal_rel = longitudinal_distance;
  const double y_goal_rel = lateral_distance;
  const double yaw_goal_rel = angle_diff;

  std::cout << "Relative coordinates - Start: (" << x_start_rel << ", " << y_start_rel
            << "), Goal: (" << x_goal_rel << ", " << y_goal_rel << ")" << std::endl;

  // 開始円弧の中心を計算（右回りを想定）
  double C_rx_rel = x_start_rel + minimum_radius * std::sin(yaw_start_rel);
  double C_ry_rel = y_start_rel - minimum_radius * std::cos(yaw_start_rel);

  // 目標点から開始円弧中心までの距離
  double dx_goal_rel = x_goal_rel - C_rx_rel;
  double dy_goal_rel = y_goal_rel - C_ry_rel;
  double d_goal_Cr_rel = std::sqrt(dx_goal_rel * dx_goal_rel + dy_goal_rel * dy_goal_rel);

  if (d_goal_Cr_rel < 1e-6) {
    std::cout << "Warning: Goal is too close to start arc center (distance: "
              << std::setprecision(6) << d_goal_Cr_rel << ")" << std::endl;
    return CompositeArcPath();
  }

  // Al-Kashi定理を使用した半径計算
  double cos_term = dy_goal_rel / d_goal_Cr_rel;
  cos_term = std::max(-1.0, std::min(1.0, cos_term));

  // 目標への進入角度調整（逆方向なのでπを加算）
  double alpha = (yaw_goal_rel + PI) + std::acos(cos_term);

  double denominator = 2 * minimum_radius + 2 * d_goal_Cr_rel * std::cos(alpha);

  if (std::abs(denominator) < 1e-6) {
    std::cout << "Warning: Denominator too small (denominator: " << std::setprecision(6)
              << denominator << ")" << std::endl;
    return CompositeArcPath();
  }

  double R_goal = (d_goal_Cr_rel * d_goal_Cr_rel - minimum_radius * minimum_radius) / denominator;

  // 物理的に接続可能かチェック
  if (R_goal < 0) {
    std::cout << "Warning: Calculated radius is negative (R_goal: " << std::setprecision(3)
              << R_goal << ")" << std::endl;
    return CompositeArcPath();
  }

  if (R_goal < minimum_radius) {
    std::cout << "Warning: Calculated radius is smaller than minimum (R_goal: "
              << std::setprecision(3) << R_goal << " < R_min: " << minimum_radius << ")"
              << std::endl;
    return CompositeArcPath();
  }

  // 目標円弧の中心を計算（左回りを想定）
  double C_lx_rel = x_goal_rel - R_goal * std::sin(yaw_goal_rel);
  double C_ly_rel = y_goal_rel + R_goal * std::cos(yaw_goal_rel);

  // 接線点の計算
  double dx_centers = C_lx_rel - C_rx_rel;
  double dy_centers = C_ly_rel - C_ry_rel;
  double distance_centers = std::sqrt(dx_centers * dx_centers + dy_centers * dy_centers);

  double tangent_x_rel, tangent_y_rel;

  // 外接円の場合の接線点計算
  double external_tangent_distance = minimum_radius + R_goal;
  double tolerance = 0.01;

  if (distance_centers < 1e-6) {
    tangent_x_rel = (C_rx_rel + C_lx_rel) / 2.0;
    tangent_y_rel = (C_ry_rel + C_ly_rel) / 2.0;
  } else if (std::abs(distance_centers - external_tangent_distance) <= tolerance) {
    double ratio = minimum_radius / distance_centers;
    tangent_x_rel = C_rx_rel + ratio * dx_centers;
    tangent_y_rel = C_ry_rel + ratio * dy_centers;
  } else {
    double ratio = minimum_radius / distance_centers;
    tangent_x_rel = C_rx_rel + ratio * dx_centers;
    tangent_y_rel = C_ry_rel + ratio * dy_centers;
  }

  std::cout << "Relative arc centers: C_r=(" << C_rx_rel << ", " << C_ry_rel << "), C_l=("
            << C_lx_rel << ", " << C_ly_rel << ")" << std::endl;
  std::cout << "Radii: R_start=" << minimum_radius << " m, R_goal=" << R_goal << " m" << std::endl;
  std::cout << "Relative tangent point: (" << tangent_x_rel << ", " << tangent_y_rel << ")"
            << std::endl;

  // 第1円弧（開始点から接線点まで、時計回り）
  double start_angle1 = std::atan2(y_start_rel - C_ry_rel, x_start_rel - C_rx_rel);
  double end_angle1 = std::atan2(tangent_y_rel - C_ry_rel, tangent_x_rel - C_rx_rel);
  double angle_diff1 = end_angle1 - start_angle1;

  // 時計回りの角度調整
  if (angle_diff1 > 0) {
    angle_diff1 -= 2 * PI;
  }

  double arc1_length = minimum_radius * std::abs(angle_diff1);

  std::cout << "\n=== Arc Length Analysis ===" << std::endl;
  std::cout << std::setprecision(3);
  std::cout << "Arc 1 length: " << arc1_length << " m" << std::endl;

  // 第2円弧（接線点から目標点まで、反時計回り）
  double start_angle2 = std::atan2(tangent_y_rel - C_ly_rel, tangent_x_rel - C_lx_rel);
  double end_angle2 = std::atan2(y_goal_rel - C_ly_rel, x_goal_rel - C_lx_rel);
  double angle_diff2 = end_angle2 - start_angle2;

  // 反時計回りの角度調整
  if (angle_diff2 < 0) {
    angle_diff2 += 2 * PI;
  }

  double arc2_length = R_goal * std::abs(angle_diff2);
  std::cout << "Arc 2 length: " << arc2_length << " m" << std::endl;
  std::cout << "Total path length: " << arc1_length + arc2_length << " m" << std::endl;

  // グローバル座標系への変換のための準備
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double cos_yaw = std::cos(start_yaw);
  const double sin_yaw = std::sin(start_yaw);

  // CompositeArcPathを作成
  CompositeArcPath composite_path;

  // 第1円弧セグメントを作成
  ArcSegment arc1;
  arc1.radius = minimum_radius;
  arc1.is_clockwise = true;

  // 相対座標系の中心をグローバル座標系に変換
  arc1.center.x = start_pose.position.x + C_rx_rel * cos_yaw - C_ry_rel * sin_yaw;
  arc1.center.y = start_pose.position.y + C_rx_rel * sin_yaw + C_ry_rel * cos_yaw;
  arc1.center.z = start_pose.position.z;

  // 開始姿勢と終了姿勢を設定
  arc1.start_pose = start_pose;

  // 接線点での姿勢を計算（グローバル座標系）
  geometry_msgs::msg::Pose tangent_pose;
  tangent_pose.position.x =
    start_pose.position.x + tangent_x_rel * cos_yaw - tangent_y_rel * sin_yaw;
  tangent_pose.position.y =
    start_pose.position.y + tangent_x_rel * sin_yaw + tangent_y_rel * cos_yaw;
  tangent_pose.position.z = start_pose.position.z;

  // 接線点での向きを計算（円弧の接線方向）
  double tangent_angle_global = end_angle1 + (arc1.is_clockwise ? -PI / 2 : PI / 2) + start_yaw;
  tangent_pose.orientation.x = 0.0;
  tangent_pose.orientation.y = 0.0;
  tangent_pose.orientation.z = std::sin(tangent_angle_global / 2.0);
  tangent_pose.orientation.w = std::cos(tangent_angle_global / 2.0);

  arc1.end_pose = tangent_pose;

  // 第2円弧セグメントを作成
  ArcSegment arc2;
  arc2.radius = R_goal;
  arc2.is_clockwise = false;

  // 相対座標系の中心をグローバル座標系に変換
  arc2.center.x = start_pose.position.x + C_lx_rel * cos_yaw - C_ly_rel * sin_yaw;
  arc2.center.y = start_pose.position.y + C_lx_rel * sin_yaw + C_ly_rel * cos_yaw;
  arc2.center.z = start_pose.position.z;

  // 開始姿勢（接線点）と終了姿勢（目標点）を設定
  arc2.start_pose = tangent_pose;

  // 目標姿勢を計算（グローバル座標系）
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = start_pose.position.x + x_goal_rel * cos_yaw - y_goal_rel * sin_yaw;
  goal_pose.position.y = start_pose.position.y + x_goal_rel * sin_yaw + y_goal_rel * cos_yaw;
  goal_pose.position.z = start_pose.position.z;

  // 目標点での向きを計算
  double goal_yaw_global = start_yaw + yaw_goal_rel;
  goal_pose.orientation.x = 0.0;
  goal_pose.orientation.y = 0.0;
  goal_pose.orientation.z = std::sin(goal_yaw_global / 2.0);
  goal_pose.orientation.w = std::cos(goal_yaw_global / 2.0);

  arc2.end_pose = goal_pose;

  // セグメントを追加
  composite_path.segments.push_back(arc1);
  composite_path.segments.push_back(arc2);

  return composite_path;
}

autoware_planning_msgs::msg::Trajectory convertCircularPathToTrajectory(
  const CompositeArcPath & composite_arc_path, const double velocity, const double z)
{
  using autoware_planning_msgs::msg::Trajectory;
  using autoware_planning_msgs::msg::TrajectoryPoint;

  Trajectory trajectory;
  trajectory.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  trajectory.header.frame_id = "map";

  if (composite_arc_path.segments.empty()) {
    return trajectory;
  }

  // CompositeArcPathから点群を生成
  std::vector<std::pair<double, double>> path_points;
  const int points_per_segment = 50;

  for (const auto & segment : composite_arc_path.segments) {
    // 各セグメントから点を生成
    for (int i = 0; i < points_per_segment; ++i) {
      // 最初のセグメント以外は最初の点をスキップ（重複回避）
      if (!path_points.empty() && i == 0) {
        continue;
      }

      double progress = static_cast<double>(i) / (points_per_segment - 1);

      // 開始角度と終了角度を計算
      double start_angle = segment.getStartAngle();
      double end_angle = segment.getEndAngle();
      double current_angle;

      if (segment.is_clockwise) {
        // 時計回りの場合の角度調整
        double angle_diff = end_angle - start_angle;
        if (angle_diff > 0) {
          angle_diff -= 2 * M_PI;
        }
        current_angle = start_angle + angle_diff * progress;
      } else {
        // 反時計回りの場合の角度調整
        double angle_diff = end_angle - start_angle;
        if (angle_diff < 0) {
          angle_diff += 2 * M_PI;
        }
        current_angle = start_angle + angle_diff * progress;
      }

      auto point = segment.getPointAtAngle(current_angle);
      path_points.push_back(std::make_pair(point.x, point.y));
    }
  }

  if (path_points.empty()) {
    return trajectory;
  }

  trajectory.points.reserve(path_points.size());

  for (size_t i = 0; i < path_points.size(); ++i) {
    TrajectoryPoint point;

    // 位置設定
    point.pose.position.x = path_points[i].first;
    point.pose.position.y = path_points[i].second;
    point.pose.position.z = z;

    // 方向設定（次の点への方向）
    if (i < path_points.size() - 1) {
      const double dx = path_points[i + 1].first - path_points[i].first;
      const double dy = path_points[i + 1].second - path_points[i].second;
      const double yaw = std::atan2(dy, dx);
      point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    } else {
      // 最後の点は前の点と同じ方向
      if (i > 0) {
        const double dx = path_points[i].first - path_points[i - 1].first;
        const double dy = path_points[i].second - path_points[i - 1].second;
        const double yaw = std::atan2(dy, dx);
        point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
      } else {
        point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
      }
    }

    // 速度設定
    point.longitudinal_velocity_mps = velocity;
    point.lateral_velocity_mps = 0.0;
    point.acceleration_mps2 = 0.0;
    point.heading_rate_rps = 0.0;
    point.front_wheel_angle_rad = 0.0;
    point.rear_wheel_angle_rad = 0.0;

    // 時間設定
    if (i == 0) {
      point.time_from_start.sec = 0;
      point.time_from_start.nanosec = 0;
    } else {
      const double distance = std::sqrt(
        std::pow(path_points[i].first - path_points[i - 1].first, 2) +
        std::pow(path_points[i].second - path_points[i - 1].second, 2));
      const double time_diff = distance / velocity;

      // builtin_interfaces::msg::Durationを使用
      const auto prev_time = trajectory.points[i - 1].time_from_start;
      const auto time_diff_sec = static_cast<int32_t>(time_diff);
      const auto time_diff_nanosec = static_cast<uint32_t>((time_diff - time_diff_sec) * 1e9);

      point.time_from_start.sec = prev_time.sec + time_diff_sec;
      point.time_from_start.nanosec = prev_time.nanosec + time_diff_nanosec;

      // ナノ秒のオーバーフロー処理
      if (point.time_from_start.nanosec >= 1000000000) {
        point.time_from_start.sec += 1;
        point.time_from_start.nanosec -= 1000000000;
      }
    }

    trajectory.points.push_back(point);
  }

  return trajectory;
}

std::vector<double> calcCurvatureFromTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  using autoware_utils::calc_curvature;

  std::vector<double> curvatures;

  if (trajectory.points.size() < 3) {
    // 点が3つ未満の場合は曲率を計算できない
    curvatures.resize(trajectory.points.size(), 0.0);
    return curvatures;
  }

  curvatures.reserve(trajectory.points.size());

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    if (i == 0) {
      // 最初の点：次の2点を使用
      const auto & p1 = trajectory.points[0].pose.position;
      const auto & p2 = trajectory.points[1].pose.position;
      const auto & p3 = trajectory.points[2].pose.position;
      curvatures.push_back(calc_curvature(p1, p2, p3));
    } else if (i == trajectory.points.size() - 1) {
      // 最後の点：前の2点を使用
      const auto & p1 = trajectory.points[i - 2].pose.position;
      const auto & p2 = trajectory.points[i - 1].pose.position;
      const auto & p3 = trajectory.points[i].pose.position;
      curvatures.push_back(calc_curvature(p1, p2, p3));
    } else {
      // 中間の点：前後の点を使用
      const auto & p1 = trajectory.points[i - 1].pose.position;
      const auto & p2 = trajectory.points[i].pose.position;
      const auto & p3 = trajectory.points[i + 1].pose.position;
      curvatures.push_back(calc_curvature(p1, p2, p3));
    }
  }

  return curvatures;
}

std::vector<double> calcCurvatureFromPoints(const std::vector<geometry_msgs::msg::Point> & points)
{
  using autoware_utils::calc_curvature;

  std::vector<double> curvatures;

  if (points.size() < 3) {
    // 点が3つ未満の場合は曲率を計算できない
    curvatures.resize(points.size(), 0.0);
    return curvatures;
  }

  curvatures.reserve(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    try {
      if (i == 0) {
        // 最初の点：次の2点を使用
        const auto & p1 = points[0];
        const auto & p2 = points[1];
        const auto & p3 = points[2];
        curvatures.push_back(calc_curvature(p1, p2, p3));
      } else if (i == points.size() - 1) {
        // 最後の点：前の2点を使用
        const auto & p1 = points[i - 2];
        const auto & p2 = points[i - 1];
        const auto & p3 = points[i];
        curvatures.push_back(calc_curvature(p1, p2, p3));
      } else {
        // 中間の点：前後の点を使用
        const auto & p1 = points[i - 1];
        const auto & p2 = points[i];
        const auto & p3 = points[i + 1];
        curvatures.push_back(calc_curvature(p1, p2, p3));
      }
    } catch (const std::runtime_error & e) {
      // 点が近すぎる場合は曲率を0とする
      curvatures.push_back(0.0);
    }
  }

  return curvatures;
}

Pose findTargetPoseAlongPath(
  const PathWithLaneId & centerline_path, const Pose & start_pose,
  const double longitudinal_distance)
{
  Pose target_pose = start_pose;
  if (!centerline_path.points.empty()) {
    // Find the point on centerline path that is longitudinal_distance ahead
    const auto start_idx =
      autoware::motion_utils::findNearestIndex(centerline_path.points, start_pose.position);
    double accumulated_distance = 0.0;
    size_t target_idx = start_idx;

    for (size_t i = start_idx; i < centerline_path.points.size() - 1; ++i) {
      const double segment_distance = autoware_utils::calc_distance2d(
        centerline_path.points[i].point.pose.position,
        centerline_path.points[i + 1].point.pose.position);
      accumulated_distance += segment_distance;

      if (accumulated_distance >= longitudinal_distance) {
        target_idx = i + 1;
        break;
      }
    }

    if (target_idx < centerline_path.points.size()) {
      target_pose = centerline_path.points[target_idx].point.pose;
    }
  }

  return target_pose;
}

RelativePoseInfo calculateRelativePoseInVehicleCoordinate(
  const Pose & start_pose, const Pose & target_pose)
{
  const double dx = target_pose.position.x - start_pose.position.x;
  const double dy = target_pose.position.y - start_pose.position.y;
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double target_yaw = tf2::getYaw(target_pose.orientation);

  // Transform to vehicle coordinate system (x: forward, y: left)
  const double longitudinal_distance_vehicle = dx * std::cos(start_yaw) + dy * std::sin(start_yaw);
  const double lateral_distance_vehicle = -dx * std::sin(start_yaw) + dy * std::cos(start_yaw);

  // Calculate angle difference
  double angle_diff = target_yaw - start_yaw;
  // Normalize angle to [-pi, pi]
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  return {longitudinal_distance_vehicle, lateral_distance_vehicle, angle_diff};
}

}  // namespace autoware::behavior_path_planner::start_planner_utils
