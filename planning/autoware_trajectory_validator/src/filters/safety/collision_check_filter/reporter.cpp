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

#include "reporter.hpp"

#include <autoware_trajectory_validator/msg/metric_report.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/logging.hpp>

#include <utility>

namespace autoware::trajectory_validator::plugin::safety::reporter
{
namespace
{
struct Color
{
  float r;
  float g;
  float b;
};

int next_marker_id(const visualization_msgs::msg::MarkerArray & debug_markers)
{
  return debug_markers.markers.empty() ? 0 : debug_markers.markers.back().id + 1;
}

Color resolve_trajectory_color(const std::string & trajectory_id)
{
  if (trajectory_id.find("_diffusion_based_trajectory") != std::string::npos) {
    return Color{1.0F, 0.55F, 0.0F};
  }
  if (trajectory_id.find("_constant_curvature_path") != std::string::npos) {
    return Color{0.0F, 0.75F, 1.0F};
  }
  return Color{0.2F, 1.0F, 0.2F};
}
}  // namespace

void ContinuousDetectionTimes::clear()
{
  current_time_.reset();
  detection_start_times_.clear();
}

double ContinuousDetectionTimes::get_time(const std::string & key) const
{
  if (!current_time_) {
    return 0.0;
  }

  const auto it = detection_start_times_.find(key);
  if (it == detection_start_times_.end()) {
    return 0.0;
  }

  return (*current_time_ - it->second).seconds();
}

void add_debug_markers(
  visualization_msgs::msg::MarkerArray & debug_markers, const rclcpp::Time & stamp,
  const std::string & ns, const std::string & trajectory_id, const PoseTrajectory & ego_trajectory,
  const PoseTrajectory & object_trajectory, const Polygon2d & ego_hull,
  const Polygon2d & object_hull)
{
  int id = next_marker_id(debug_markers);
  const auto trajectory_color = resolve_trajectory_color(trajectory_id);

  auto add_poly_marker =
    [&](const Polygon2d & poly, const std::string & local_namespace, float r, float g, float b) {
      if (poly.outer().empty()) {
        return;
      }

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = ns + "/" + local_namespace;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.05;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 0.9;

      for (const auto & p : poly.outer()) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = 0.0;
        m.points.push_back(pt);
      }

      geometry_msgs::msg::Point first_point;
      first_point.x = poly.outer().front().x();
      first_point.y = poly.outer().front().y();
      first_point.z = 0.0;
      m.points.push_back(first_point);

      debug_markers.markers.push_back(std::move(m));
    };

  auto add_trajectory_marker = [&](
                                 const PoseTrajectory & trajectory,
                                 const std::string & local_namespace, float r, float g, float b,
                                 float alpha) {
    if (trajectory.empty()) {
      return;
    }

    for (const auto & pose : trajectory) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = ns + "/" + local_namespace;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = pose;
      m.scale.x = 0.3;
      m.scale.y = 0.18;
      m.scale.z = 0.18;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = alpha;
      debug_markers.markers.push_back(std::move(m));
    }
  };

  add_poly_marker(ego_hull, "ego_worst_pet", 0.0F, 0.0F, 1.0F);
  add_poly_marker(object_hull, "obj_worst_pet", 1.0F, 0.0F, 0.0F);
  add_trajectory_marker(ego_trajectory, "ego_trajectory", 1.0F, 1.0F, 1.0F, 0.9F);
  add_trajectory_marker(
    object_trajectory, "object_trajectory", trajectory_color.r, trajectory_color.g,
    trajectory_color.b, 0.95F);
}

void add_error_text_marker(
  visualization_msgs::msg::MarkerArray & debug_markers, const rclcpp::Time & stamp,
  const geometry_msgs::msg::Pose & ego_pose, const std::string & error_msg)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = stamp;
  m.ns = "collision_check_error";
  m.id = next_marker_id(debug_markers);
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.z = 0.6;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 1.0;
  m.color.a = 0.95;
  m.pose = ego_pose;
  m.pose.position.z += 1.0;
  m.text = error_msg;
  debug_markers.markers.push_back(std::move(m));
}

void append_text_marker_message(std::string & text, const std::string & message)
{
  if (!message.empty()) {
    text += message + "\n";
  }
}

void log_collision_messages(const uint8_t level, const std::string & messages)
{
  using autoware_trajectory_validator::msg::MetricReport;

  if (messages.empty()) {
    return;
  }
  if (level == MetricReport::ERROR) {
    RCLCPP_ERROR(rclcpp::get_logger("CollisionCheckFilter"), "Not feasible: %s", messages.c_str());
    return;
  }
  RCLCPP_WARN(rclcpp::get_logger("CollisionCheckFilter"), "Warning: %s", messages.c_str());
}
}  // namespace autoware::trajectory_validator::plugin::safety::reporter
