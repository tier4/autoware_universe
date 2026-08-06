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

// cspell:ignore Kashi Al-Kashi
#include "clothoid_pull_generater.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/normalization.hpp"

#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
using autoware::universe_utils::normalizeRadian;

namespace
{

/// Pose-based arc segment (geometry-only, no lane/route dependency).
struct ArcSegment
{
  geometry_msgs::msg::Point center;
  double radius;
  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose end_pose;
  bool is_clockwise;

  ArcSegment() : radius(0.0), is_clockwise(true) { center.x = center.y = center.z = 0.0; }

  double calculateStartAngle() const
  {
    return std::atan2(start_pose.position.y - center.y, start_pose.position.x - center.x);
  }

  double calculateEndAngle() const
  {
    return std::atan2(end_pose.position.y - center.y, end_pose.position.x - center.x);
  }

  double calculateArcLength() const
  {
    double start_angle = calculateStartAngle();
    double end_angle = calculateEndAngle();
    double angle_diff = std::abs(end_angle - start_angle);
    if (angle_diff > 2.0 * M_PI) {
      angle_diff = 2.0 * M_PI - std::fmod(angle_diff, 2.0 * M_PI);
    }
    return radius * angle_diff;
  }

  geometry_msgs::msg::Point getPointAtAngle(double angle) const
  {
    geometry_msgs::msg::Point point;
    point.x = center.x + radius * std::cos(angle);
    point.y = center.y + radius * std::sin(angle);
    point.z = center.z;
    return point;
  }
};

/// Composite arc path consisting of two arc segments (start-side, goal-side).
struct CompositeArcPath
{
  std::vector<ArcSegment> segments;
};

/// Clothoid segment used while assembling entry/circular/exit sub-paths.
struct ClothoidSegment
{
  enum Type { CLOTHOID_ENTRY, CIRCULAR_ARC, CLOTHOID_EXIT };

  Type type;
  double A{0.0};       // Clothoid parameter
  double L{0.0};       // Arc length
  double radius{0.0};  // Radius (for circular arc segment)
  double angle{0.0};   // Angle (for circular arc segment)
  bool is_clockwise{true};
  std::string description;

  explicit ClothoidSegment(Type type_in, double A_in = 0.0, double L_in = 0.0)
  : type(type_in), A(A_in), L(L_in)
  {
  }
};

/// Relative pose of target_pose expressed in start_pose's vehicle coordinate frame.
struct RelativePoseInfo
{
  double longitudinal_distance_vehicle;
  double lateral_distance_vehicle;
  double angle_diff;
};

RelativePoseInfo calculate_relative_pose_in_vehicle_coordinate(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & target_pose)
{
  const double dx = target_pose.position.x - start_pose.position.x;
  const double dy = target_pose.position.y - start_pose.position.y;
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double target_yaw = tf2::getYaw(target_pose.orientation);

  const double longitudinal_distance_vehicle = dx * std::cos(start_yaw) + dy * std::sin(start_yaw);
  const double lateral_distance_vehicle = -dx * std::sin(start_yaw) + dy * std::cos(start_yaw);

  double angle_diff = target_yaw - start_yaw;
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  return {longitudinal_distance_vehicle, lateral_distance_vehicle, angle_diff};
}

}  // namespace

std::vector<geometry_msgs::msg::Point> correct_clothoid_by_rigid_transform(
  const std::vector<geometry_msgs::msg::Point> & clothoid_points,
  const ArcSegment & original_segment, const geometry_msgs::msg::Pose & start_pose)
{
  if (clothoid_points.size() < 2) {
    return clothoid_points;
  }

  const auto clothoid_start = clothoid_points.front();
  const auto clothoid_end = clothoid_points.back();

  // Get target start and end positions
  const auto target_start = start_pose.position;
  const auto target_end = original_segment.getPointAtAngle(original_segment.calculateEndAngle());

  // Calculate direction vectors
  const double clothoid_dx = clothoid_end.x - clothoid_start.x;
  const double clothoid_dy = clothoid_end.y - clothoid_start.y;
  const double clothoid_length = std::sqrt(clothoid_dx * clothoid_dx + clothoid_dy * clothoid_dy);

  const double target_dx = target_end.x - target_start.x;
  const double target_dy = target_end.y - target_start.y;
  const double target_length = std::sqrt(target_dx * target_dx + target_dy * target_dy);

  // Calculate scaling factor
  const double scale_factor = (clothoid_length > 1e-10) ? target_length / clothoid_length : 1.0;

  // Calculate rotation angle
  const double clothoid_angle = std::atan2(clothoid_dy, clothoid_dx);
  const double target_angle = std::atan2(target_dy, target_dx);
  double rotation_angle = target_angle - clothoid_angle;

  rotation_angle = normalizeRadian(rotation_angle);

  // Choose shorter rotation if over 180 degrees
  if (std::abs(rotation_angle) > M_PI) {
    rotation_angle = (rotation_angle > 0) ? rotation_angle - 2 * M_PI : rotation_angle + 2 * M_PI;
  }

  // Calculate transformation matrix elements
  const double cos_theta = std::cos(rotation_angle);
  const double sin_theta = std::sin(rotation_angle);

  // Apply rigid transformation
  std::vector<geometry_msgs::msg::Point> corrected_points;
  corrected_points.reserve(clothoid_points.size());

  for (size_t i = 0; i < clothoid_points.size(); ++i) {
    geometry_msgs::msg::Point corrected_point;

    // Move to origin
    double rel_x = clothoid_points[i].x - clothoid_start.x;
    double rel_y = clothoid_points[i].y - clothoid_start.y;

    // Scale
    rel_x *= scale_factor;
    rel_y *= scale_factor;

    // Rotate
    double rotated_x = cos_theta * rel_x - sin_theta * rel_y;
    double rotated_y = sin_theta * rel_x + cos_theta * rel_y;

    // Translate to target start
    corrected_point.x = rotated_x + target_start.x;
    corrected_point.y = rotated_y + target_start.y;
    corrected_point.z = clothoid_points[i].z;  // Keep Z coordinate as is

    corrected_points.push_back(corrected_point);
  }

  return corrected_points;
}

std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generate_clothoid_entry_with_yaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  const double A = segment.A;
  const double L = segment.L;
  const double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  const double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Pose> poses;

  // Entry Clothoid: linearly increase curvature from 0 to target curvature
  const double target_curvature = (L / (A * A)) * direction_factor;
  const double start_curvature = 0.0;

  // Accurate calculation using numerical integration
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    // Create current point and add to poses
    geometry_msgs::msg::Pose pose;
    pose.position.x = current_x;
    pose.position.y = current_y;
    pose.position.z = 0.0;  // This is temporarily set to 0.0. The z value will be overwritten from
                            // the lanelet when generating the final path.
    pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);
    poses.push_back(pose);

    // If not the last point, perform integration calculation to the next point
    if (i < num_points - 1) {
      // Infinitesimal interval to next point
      double ds = L / (num_points - 1);

      // Calculate curvature at current position
      double progress = static_cast<double>(i) / (num_points - 1);
      double current_curvature = start_curvature + (target_curvature - start_curvature) * progress;

      // Update coordinates using numerical integration
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // Create terminal state
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = current_x;
  end_pose.position.y = current_y;
  end_pose.position.z = 0.0;  // This is temporarily set to 0.0. The z value will be overwritten
                              // from the lanelet when generating the final path.
  end_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);

  return {poses, end_pose};
}

std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generate_circular_segment_with_yaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double radius = segment.radius;
  double angle = segment.angle;
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Pose> poses;

  // Calculate arc center
  double center_x = start_pose.position.x - radius * std::sin(start_yaw) * direction_factor;
  double center_y = start_pose.position.y + radius * std::cos(start_yaw) * direction_factor;

  for (int i = 0; i < num_points; ++i) {
    double progress = static_cast<double>(i) / (num_points - 1);
    double angle_progress = angle * progress * direction_factor;
    double current_psi = start_yaw + angle_progress;

    // Calculate position on arc (angle from center)
    double angle_from_center = current_psi - M_PI / 2.0 * direction_factor;

    geometry_msgs::msg::Pose pose;
    pose.position.x = center_x + radius * std::cos(angle_from_center);
    pose.position.y = center_y + radius * std::sin(angle_from_center);
    pose.position.z = 0.0;
    pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);

    poses.push_back(pose);
  }

  // Terminal state (final yaw angle)
  double final_psi = start_yaw + angle * direction_factor;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = poses.back().position;
  end_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(final_psi);

  return {poses, end_pose};
}

std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generate_clothoid_exit_with_yaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double L = segment.L;
  double start_yaw = tf2::getYaw(start_pose.orientation);
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;

  std::vector<geometry_msgs::msg::Pose> poses;

  // Calculate curvature of previous segment (arc) considering rotation direction
  double start_curvature = (1.0 / segment.radius) * direction_factor;

  // Accurate calculation using numerical integration
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    // Create current point and add to poses
    geometry_msgs::msg::Pose pose;
    pose.position.x = current_x;
    pose.position.y = current_y;
    pose.position.z = 0.0;
    pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);
    poses.push_back(pose);

    // If not the last point, perform integration calculation to the next point
    if (i < num_points - 1) {
      // Infinitesimal interval to next point
      double ds = L / (num_points - 1);

      // Calculate curvature at current position (Exit Clothoid: linearly decrease from start
      // curvature to 0)
      double progress = static_cast<double>(i) / (num_points - 1);
      double current_curvature = start_curvature * (1.0 - progress);

      // Update coordinates using numerical integration
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // Create terminal state
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = current_x;
  end_pose.position.y = current_y;
  end_pose.position.z = 0.0;  // This is temporarily set to 0.0. The z value will be overwritten
                              // from the lanelet when generating the final path.
  end_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);

  return {poses, end_pose};
}

std::vector<geometry_msgs::msg::Point> generate_clothoid_path(
  const std::vector<ClothoidSegment> & segments, double point_interval,
  const geometry_msgs::msg::Pose & start_pose)
{
  // Early return if segments are empty
  if (segments.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("ClothoidPullOut"),
      "No clothoid segments provided to generate_clothoid_path");
    return {};
  }

  // Calculate theoretical arc length for each segment
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

  // Calculate number of points per segment from arc length and specified interval
  std::vector<int> segment_points;
  for (double length : theoretical_lengths) {
    int points = std::max(2, static_cast<int>(std::ceil(length / point_interval)) + 1);
    segment_points.push_back(points);
  }

  // Set initial state
  geometry_msgs::msg::Pose current_pose = start_pose;

  std::vector<geometry_msgs::msg::Point> all_points;

  for (size_t i = 0; i < segments.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> segment_poses_vec;
    geometry_msgs::msg::Pose end_pose;

    int num_points = segment_points[i];

    if (segments[i].type == ClothoidSegment::CLOTHOID_ENTRY) {
      auto result = generate_clothoid_entry_with_yaw(segments[i], current_pose, num_points);
      segment_poses_vec = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CIRCULAR_ARC) {
      auto result = generate_circular_segment_with_yaw(segments[i], current_pose, num_points);
      segment_poses_vec = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CLOTHOID_EXIT) {
      auto result = generate_clothoid_exit_with_yaw(segments[i], current_pose, num_points);
      segment_poses_vec = result.first;
      end_pose = result.second;
    }

    // Combine avoiding duplicate points (extract coordinates only from Pose)
    size_t start_idx = (all_points.empty()) ? 0 : 1;
    for (size_t j = start_idx; j < segment_poses_vec.size(); ++j) {
      all_points.push_back(segment_poses_vec[j].position);
    }

    current_pose = end_pose;
  }

  return all_points;
}

std::optional<std::vector<geometry_msgs::msg::Point>> convert_arc_to_clothoid(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min,
  double L_min, double point_interval)
{
  // Extract arc information
  double start_angle = arc_segment.calculateStartAngle();
  double end_angle = arc_segment.calculateEndAngle();

  double total_angle = std::abs(end_angle - start_angle);

  if (total_angle > M_PI) {
    total_angle = 2.0 * M_PI - total_angle;
  }

  double radius = arc_segment.radius;

  // Clothoid parameters
  double A = A_min;
  double L = L_min;
  double alpha_clothoid = (L * L) / (2.0 * A * A);  // Angle change of single clothoid

  RCLCPP_DEBUG(
    rclcpp::get_logger("ClothoidPullOut"),
    "Clothoid parameters: radius=%.3f, A_min=%.3f, L_min=%.3f, total_angle=%.3f°, "
    "alpha_clothoid=%.3f°",
    radius, A_min, L_min, total_angle * 180.0 / M_PI, alpha_clothoid * 180.0 / M_PI);

  std::vector<ClothoidSegment> segments;

  if (total_angle >= 2.0 * alpha_clothoid) {
    const bool is_clockwise = arc_segment.is_clockwise;

    // Case A: CAC(A, L, θ)
    double theta_arc = total_angle - 2.0 * alpha_clothoid;

    // Entry clothoid
    ClothoidSegment entry(ClothoidSegment::CLOTHOID_ENTRY, A, L);
    entry.radius = radius;
    entry.is_clockwise = is_clockwise;
    entry.description = "Entry clothoid (κ: 0 → 1/R)";
    segments.push_back(entry);

    // Circular arc segment
    ClothoidSegment circular(ClothoidSegment::CIRCULAR_ARC);
    circular.radius = radius;
    circular.angle = theta_arc;
    circular.is_clockwise = is_clockwise;
    circular.description = "Circular arc (κ = 1/R = " + std::to_string(1.0 / radius) + ")";
    segments.push_back(circular);

    // Exit clothoid
    ClothoidSegment exit(ClothoidSegment::CLOTHOID_EXIT, A, L);
    exit.radius = radius;
    exit.is_clockwise = is_clockwise;
    exit.description = "Exit clothoid (κ: 1/R → 0)";
    segments.push_back(exit);
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Case B is not implemented. Please use Case A conditions.");
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Current parameters: total_angle=%.3f°, alpha_clothoid=%.3f°, required: total_angle >= %.3f°",
      total_angle * 180.0 / M_PI, alpha_clothoid * 180.0 / M_PI,
      2.0 * alpha_clothoid * 180.0 / M_PI);

    // Return failure if Case B is not implemented
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"), "Clothoid conversion failed! Case B not implemented.");
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Arc segment: radius=%.3f, center=(%.3f, %.3f), is_clockwise=%s", arc_segment.radius,
      arc_segment.center.x, arc_segment.center.y, arc_segment.is_clockwise ? "true" : "false");
    return std::nullopt;
  }

  // Generate clothoid path
  std::vector<geometry_msgs::msg::Point> clothoid_path =
    generate_clothoid_path(segments, point_interval, start_pose);

  // Return failure if generated path is empty
  if (clothoid_path.empty()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Clothoid conversion failed! Generated path is empty.");
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Arc segment: radius=%.3f, center=(%.3f, %.3f), is_clockwise=%s", arc_segment.radius,
      arc_segment.center.x, arc_segment.center.y, arc_segment.is_clockwise ? "true" : "false");
    return std::nullopt;
  }

  return clothoid_path;
}

std::vector<geometry_msgs::msg::Pose> create_straight_path_to_end_pose(
  const geometry_msgs::msg::Pose & start_pose, double forward_distance, double backward_distance,
  double point_interval)
{
  // Calculate straight direction (yaw angle)
  const double start_yaw = tf2::getYaw(start_pose.orientation);

  // Store pose array
  std::vector<geometry_msgs::msg::Pose> poses;

  // 1. Generate backward path (generate from farthest backward point in order)
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

  // 2. Add start_pose
  poses.push_back(start_pose);

  // 3. Generate forward path
  if (forward_distance > 0.0) {
    // Calculate forward end pose
    geometry_msgs::msg::Pose forward_end_pose = start_pose;
    forward_end_pose.position.x = start_pose.position.x + forward_distance * std::cos(start_yaw);
    forward_end_pose.position.y = start_pose.position.y + forward_distance * std::sin(start_yaw);
    forward_end_pose.orientation = start_pose.orientation;

    // Calculate forward point count (exclude start_pose as it's already added)
    const int forward_num_points =
      std::max(1, static_cast<int>(std::ceil(forward_distance / point_interval)));

    // Recalculate actual interval (for equal spacing)
    const double actual_interval = forward_distance / forward_num_points;

    // Generate each forward point (after start_pose)
    for (int i = 1; i <= forward_num_points; ++i) {
      geometry_msgs::msg::Pose pose;

      if (i == forward_num_points) {
        // Final point (exactly forward_end_pose)
        pose = forward_end_pose;
      } else {
        // Intermediate point
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

std::optional<CompositeArcPath> calc_circular_path(
  const geometry_msgs::msg::Pose & start_pose, const double longitudinal_distance,
  const double lateral_distance, const double angle_diff, const double first_radius,
  const bool shift_direction)
{
  const double PI = M_PI;

  // Calculate in relative coordinate system (origin at start point, X-axis as forward direction)
  // Start point: (0, 0, 0)
  // Goal point: (longitudinal_distance, lateral_distance, angle_diff)

  const double x_start_rel = 0.0;
  const double y_start_rel = 0.0;
  const double yaw_start_rel = 0.0;

  const double x_goal_rel = longitudinal_distance;
  const double y_goal_rel = lateral_distance;
  const double C_r_direction = (shift_direction) ? 1.0 : -1.0;  // left shift : right shift
  const double yaw_goal_rel = angle_diff;

  RCLCPP_DEBUG(
    rclcpp::get_logger("ClothoidPullOut"),
    "Relative coordinates - Start: (%.3f, %.3f), Goal: (%.3f, %.3f)", x_start_rel, y_start_rel,
    x_goal_rel, y_goal_rel);

  // Calculate starting arc center (assuming clockwise rotation)
  double C_rx_rel = x_start_rel + first_radius * std::sin(yaw_start_rel);
  double C_ry_rel = y_start_rel + C_r_direction * first_radius * std::cos(yaw_start_rel);

  // Distance from goal point to starting arc center
  double dx_goal_rel = x_goal_rel - C_rx_rel;
  double dy_goal_rel = y_goal_rel - C_ry_rel;
  double d_goal_Cr_rel = std::sqrt(dx_goal_rel * dx_goal_rel + dy_goal_rel * dy_goal_rel);

  if (d_goal_Cr_rel < 1e-6) {
    RCLCPP_WARN(
      rclcpp::get_logger("ClothoidPullOut"),
      "Goal is too close to start arc center (distance: %.6f)", d_goal_Cr_rel);
    return std::nullopt;
  }

  // Calculate radius using Al-Kashi theorem
  double cos_term = dy_goal_rel / d_goal_Cr_rel;
  cos_term = std::max(-1.0, std::min(1.0, cos_term));

  // Adjust approach angle to goal (add π for reverse direction)
  double alpha = (C_r_direction > 0) ? yaw_goal_rel + std::acos(cos_term)
                                     : yaw_goal_rel + PI + std::acos(cos_term);

  double denominator = 2 * first_radius + 2 * d_goal_Cr_rel * std::cos(alpha);

  if (std::abs(denominator) < 1e-6) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"), "Denominator too small (denominator: %.6f)",
      denominator);
    return std::nullopt;
  }

  double R_goal = (d_goal_Cr_rel * d_goal_Cr_rel - first_radius * first_radius) / denominator;

  // Check physical feasibility
  if (R_goal < 0) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"), "Calculated radius is negative (R_goal: %.3f)",
      R_goal);
    return std::nullopt;
  }

  // Calculate goal arc center (assuming counter-clockwise rotation)
  double C_lx_rel = x_goal_rel + C_r_direction * R_goal * std::sin(yaw_goal_rel);
  double C_ly_rel = y_goal_rel - C_r_direction * R_goal * std::cos(yaw_goal_rel);

  // Calculate tangent point
  double dx_centers = C_lx_rel - C_rx_rel;
  double dy_centers = C_ly_rel - C_ry_rel;
  double distance_centers = std::sqrt(dx_centers * dx_centers + dy_centers * dy_centers);

  double tangent_x_rel, tangent_y_rel;
  if (distance_centers < 1e-6) {
    tangent_x_rel = (C_rx_rel + C_lx_rel) / 2.0;
    tangent_y_rel = (C_ry_rel + C_ly_rel) / 2.0;
  } else {
    double ratio = first_radius / distance_centers;
    tangent_x_rel = C_rx_rel + ratio * dx_centers;
    tangent_y_rel = C_ry_rel + ratio * dy_centers;
  }

  // First arc (from start point to tangent point, clockwise)
  double start_angle1 = std::atan2(y_start_rel - C_ry_rel, x_start_rel - C_rx_rel);
  double end_angle1 = std::atan2(tangent_y_rel - C_ry_rel, tangent_x_rel - C_rx_rel);
  const double total_angle1 = std::abs(end_angle1 - start_angle1);

  // Adjust for clockwise direction
  if (total_angle1 > 0) {
    end_angle1 -= 2 * PI;
  }

  // Prepare for transformation to global coordinate system
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double cos_yaw = std::cos(start_yaw);
  const double sin_yaw = std::sin(start_yaw);

  // Create CompositeArcPath
  CompositeArcPath composite_path;

  // Create first arc segment
  ArcSegment arc1;
  arc1.radius = first_radius;
  arc1.is_clockwise = (C_r_direction > 0.0) ? false : true;

  // Transform relative coordinate center to global coordinate system
  arc1.center.x = start_pose.position.x + C_rx_rel * cos_yaw - C_ry_rel * sin_yaw;
  arc1.center.y = start_pose.position.y + C_rx_rel * sin_yaw + C_ry_rel * cos_yaw;
  arc1.center.z = start_pose.position.z;

  // Set start and end poses
  arc1.start_pose = start_pose;

  // Calculate pose at tangent point (global coordinate system)
  geometry_msgs::msg::Pose tangent_pose;
  tangent_pose.position.x =
    start_pose.position.x + tangent_x_rel * cos_yaw - tangent_y_rel * sin_yaw;
  tangent_pose.position.y =
    start_pose.position.y + tangent_x_rel * sin_yaw + tangent_y_rel * cos_yaw;
  tangent_pose.position.z = start_pose.position.z;

  // Calculate orientation at tangent point (tangent direction of arc)
  double tangent_angle_global = end_angle1 + (arc1.is_clockwise ? -PI / 2 : PI / 2) + start_yaw;
  tangent_pose.orientation.x = 0.0;
  tangent_pose.orientation.y = 0.0;
  tangent_pose.orientation.z = std::sin(tangent_angle_global / 2.0);
  tangent_pose.orientation.w = std::cos(tangent_angle_global / 2.0);

  arc1.end_pose = tangent_pose;

  // Create second arc segment
  ArcSegment arc2;
  arc2.radius = R_goal;
  arc2.is_clockwise = (C_r_direction > 0.0) ? true : false;

  // Transform relative coordinate center to global coordinate system
  arc2.center.x = start_pose.position.x + C_lx_rel * cos_yaw - C_ly_rel * sin_yaw;
  arc2.center.y = start_pose.position.y + C_lx_rel * sin_yaw + C_ly_rel * cos_yaw;
  arc2.center.z = start_pose.position.z;

  // Set start pose (tangent point) and end pose (goal point)
  arc2.start_pose = tangent_pose;

  // Calculate goal pose (global coordinate system)
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = start_pose.position.x + x_goal_rel * cos_yaw - y_goal_rel * sin_yaw;
  goal_pose.position.y = start_pose.position.y + x_goal_rel * sin_yaw + y_goal_rel * cos_yaw;
  goal_pose.position.z = start_pose.position.z;

  // Calculate orientation at goal point
  double goal_yaw_global = start_yaw + yaw_goal_rel;
  goal_pose.orientation.x = 0.0;
  goal_pose.orientation.y = 0.0;
  goal_pose.orientation.z = std::sin(goal_yaw_global / 2.0);
  goal_pose.orientation.w = std::cos(goal_yaw_global / 2.0);

  arc2.end_pose = goal_pose;

  // Add segments
  composite_path.segments.push_back(arc1);
  composite_path.segments.push_back(arc2);

  // Check if circular path generation failed
  if (composite_path.segments.size() < 2) {
    RCLCPP_WARN(
      rclcpp::get_logger("ClothoidPullOut"), "Circular path generation failed (segments: %zu)",
      composite_path.segments.size());
    return std::nullopt;
  }

  return composite_path;
}

std::vector<double> generate_candidate_steer_angles_rad(double max_steer_angle_rad, int trial_count)
{
  std::vector<double> angles_rad;
  if (trial_count <= 1) {
    angles_rad.push_back(max_steer_angle_rad);
    return angles_rad;
  }

  constexpr double min_steer_angle_rad = 1.0 * M_PI / 180.0;
  for (int i = 0; i < trial_count; ++i) {
    const double angle_rad = min_steer_angle_rad + (max_steer_angle_rad - min_steer_angle_rad) * i /
                                                     static_cast<double>(trial_count - 1);
    angles_rad.push_back(angle_rad);
  }
  return angles_rad;
}

std::optional<std::vector<geometry_msgs::msg::Point>> biclothoid_approximation(
  const CompositeArcPath & circular_path, const geometry_msgs::msg::Pose & start_pose,
  double wheel_base_m, double max_steer_angle_rate_rad_per_sec, double reference_velocity_mps)
{
  geometry_msgs::msg::Pose segment_start_pose = start_pose;

  std::vector<std::vector<geometry_msgs::msg::Point>> segment_points;
  for (const auto & segment : circular_path.segments) {
    const double segment_radius = segment.radius;
    const double segment_steer_angle = std::atan(wheel_base_m / segment_radius);
    const double minimum_steer_time = segment_steer_angle / max_steer_angle_rate_rad_per_sec;
    // Arc length needed to steer into the segment's curvature at the vehicle's steering rate
    // limit, given the (assumed constant) reference velocity.
    const double L_min = reference_velocity_mps * minimum_steer_time;
    const double A_min = std::sqrt(segment_radius * L_min);

    auto clothoid_points_opt =
      convert_arc_to_clothoid(segment, segment_start_pose, A_min, L_min, 1.0);
    if (!clothoid_points_opt) {
      return std::nullopt;
    }

    auto corrected_points =
      correct_clothoid_by_rigid_transform(*clothoid_points_opt, segment, segment_start_pose);

    if (corrected_points.size() >= 2) {
      const auto & last = corrected_points.back();
      const auto & second_last = corrected_points[corrected_points.size() - 2];
      const double heading = std::atan2(last.y - second_last.y, last.x - second_last.x);
      segment_start_pose.position = last;
      segment_start_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
    }
    segment_points.push_back(std::move(corrected_points));
  }

  std::vector<geometry_msgs::msg::Point> connected_points;
  for (const auto & path : segment_points) {
    const size_t start_idx = connected_points.empty() ? 0 : 1;
    for (size_t j = start_idx; j < path.size(); ++j) {
      connected_points.push_back(path[j]);
    }
  }

  return connected_points;
}

void interpolate_z_by_cumulative_distance(
  std::vector<geometry_msgs::msg::Point> & points, double start_z, double goal_z)
{
  if (points.empty()) {
    return;
  }
  if (points.size() == 1) {
    points.front().z = start_z;
    return;
  }

  std::vector<double> s(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    s[i] = s[i - 1] + std::hypot(dx, dy);
  }

  const double total_distance = s.back();
  for (size_t i = 0; i < points.size(); ++i) {
    const double ratio = (total_distance > 1e-9) ? s[i] / total_distance : 0.0;
    points[i].z = start_z + (goal_z - start_z) * ratio;
  }
}

std::optional<std::vector<std::vector<geometry_msgs::msg::Point>>> plan_clothoid_pull(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & target_pose,
  double wheel_base_m, const double & max_steer_angle, double max_steer_angle_rate_rad_per_sec,
  double reference_velocity_mps)
{
  const auto relative_pose_info =
    calculate_relative_pose_in_vehicle_coordinate(start_pose, target_pose);

  const double minimum_radius = wheel_base_m / std::tan(max_steer_angle);
  std::vector<std::vector<geometry_msgs::msg::Point>> candidate_paths;
  for (const bool is_leftShift : {true, false}) {
    double first_radius = minimum_radius;
    for (int i = 0; i < 2; ++i) {
      const auto circular_path_opt = calc_circular_path(
        start_pose, relative_pose_info.longitudinal_distance_vehicle,
        relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff, first_radius,
        is_leftShift);
      if (!circular_path_opt || circular_path_opt->segments[1].radius < minimum_radius) {
        break;
      }
      first_radius = circular_path_opt->segments[1].radius;

      auto connected_points_opt = biclothoid_approximation(
        *circular_path_opt, start_pose, wheel_base_m, max_steer_angle_rate_rad_per_sec,
        reference_velocity_mps);
      if (!connected_points_opt) {
        continue;
      }

      if (!connected_points_opt->empty()) {
        interpolate_z_by_cumulative_distance(
          *connected_points_opt, start_pose.position.z, target_pose.position.z);
        candidate_paths.push_back(*connected_points_opt);
      }
    }
  }

  if (!candidate_paths.empty()) {
    return candidate_paths;
  }
  return std::nullopt;
}

}  // namespace autoware::minimum_rule_based_planner
