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
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/control_point.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <fmt/core.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::reporter
{
namespace
{
using autoware_trajectory_validator::msg::MetricReport;

struct VisualizationData
{
  std::string error_msg{};
  autoware_internal_planning_msgs::msg::PlanningFactorArray planning_factors{};
};

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
  if (trajectory_id.find("_constant_curvature_path") != std::string::npos) {
    return Color{0.0F, 0.75F, 1.0F};
  }
  return Color{0.2F, 1.0F, 0.2F};
}

autoware_internal_planning_msgs::msg::SafetyFactorArray make_safety_factor_array(
  const builtin_interfaces::msg::Time & stamp, const CollisionDetail & collision_detail,
  const std::string & collision_type, double time_resolution)
{
  using autoware_internal_planning_msgs::msg::SafetyFactor;
  using autoware_internal_planning_msgs::msg::SafetyFactorArray;

  SafetyFactor safety_factor;
  safety_factor.type = SafetyFactor::OBJECT;
  safety_factor.object_id = collision_detail.object_identification.uuid;
  safety_factor.ttc_begin = static_cast<float>(collision_detail.first_collision_timing.ttc);
  safety_factor.ttc_end =
    static_cast<float>(collision_detail.first_collision_timing.ttc + time_resolution);
  safety_factor.is_safe = false;
  if (!collision_detail.object_trajectory.empty()) {
    safety_factor.points.push_back(collision_detail.object_trajectory.front().position);
  }

  SafetyFactorArray safety_factors;
  safety_factors.header.stamp = stamp;
  safety_factors.header.frame_id = "map";
  safety_factors.factors.push_back(std::move(safety_factor));
  safety_factors.is_safe = false;
  safety_factors.detail = collision_type;
  return safety_factors;
}

void add_collision_planning_factor(
  const double time_resolution, const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & ego_pose, const CollisionDetail & collision_detail,
  const std::string & collision_type,
  autoware_internal_planning_msgs::msg::PlanningFactorArray & planning_factors)
{
  const auto safety_factors =
    make_safety_factor_array(stamp, collision_detail, collision_type, time_resolution);
  const auto control_point =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::ControlPoint>()
      .pose(ego_pose)
      .velocity(0.0)
      .shift_length(0.0)
      .distance(0.0);
  auto factor =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::PlanningFactor>()
      .module("")
      .is_driving_forward(true)
      .control_points({control_point})
      .behavior(autoware_internal_planning_msgs::msg::PlanningFactor::STOP)
      .detail(collision_type)
      .safety_factors(safety_factors);
  planning_factors.factors.push_back(std::move(factor));
}

std::string format_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return fmt::format("{}.{:09d}", stamp.sec, stamp.nanosec);
}

std::string format_risk_level(const RiskLevel::_level_type level)
{
  switch (level) {
    case RiskLevel::SAFE:
      return "SAFE";
    case RiskLevel::LOW_CAUTION:
      return "LOW_CAUTION";
    case RiskLevel::HIGH_CAUTION:
      return "HIGH_CAUTION";
    case RiskLevel::DANGER:
      return "DANGER";
    case RiskLevel::FATAL:
      return "FATAL";
    default:
      return "UNKNOWN";
  }
}

std::string format_required_acceleration(const std::optional<double> & acceleration)
{
  return acceleration.has_value() ? fmt::format("{:.2f} m/s^2", acceleration.value())
                                  : std::string{"cannot be avoided"};
}

// "1 DRAC collision" / "2 DRAC collisions"
std::string format_finding_count(const size_t count, const std::string & noun)
{
  return fmt::format("{} {}{}", count, noun, count == 1 ? "" : "s");
}

// Findings are printed as indented multi-line blocks, separated by a blank line so that a report
// covering several objects stays readable on a terminal.
std::string join_findings(const std::vector<std::string> & findings)
{
  std::string joined{};
  for (const auto & finding : findings) {
    if (!joined.empty()) {
      joined += "\n\n";
    }
    joined += finding;
  }
  return joined;
}

void process_drac_artifacts(
  const nav_msgs::msg::Odometry & odometry, const rclcpp::Time & current_time,
  const DracArtifact & drac_artifact, VisualizationData & artifacts,
  visualization_msgs::msg::MarkerArray & debug_markers, double time_resolution)
{
  if (drac_artifact.evaluations.empty()) {
    return;
  }

  std::vector<std::string> findings{};
  findings.reserve(drac_artifact.evaluations.size());
  for (const auto & evaluation : drac_artifact.evaluations) {
    if (evaluation.risk == RiskLevel::SAFE) {
      continue;
    }
    const auto & timing = evaluation.detail;
    const auto & obj_id = timing.object_identification;

    findings.push_back(
      fmt::format(
        "  [{}] {} {} | {} (acc {:.2f} m/s^2)\n"
        "      logic : {}\n"
        "      first : TTC {:.3f} s, PET {:6.3f} s | worst PET: TTC {:.3f} s, PET {:6.3f} s\n"
        "      result: {}, ego req. acc = {} | stamp {}",
        findings.size() + 1, obj_id.classification, obj_id.object_id_string(),
        obj_id.trajectory_type, obj_id.acceleration, evaluation.method,
        timing.first_collision_timing.ttc, timing.first_collision_timing.pet,
        timing.worst_pet_timing.ttc, timing.worst_pet_timing.pet,
        format_risk_level(evaluation.risk),
        format_required_acceleration(evaluation.ego_drac_acceleration),
        format_stamp(obj_id.stamp)));

    reporter::add_debug_markers(
      debug_markers, current_time, "drac_collision", obj_id.trajectory_id_string(),
      timing.ego_trajectory, timing.object_trajectory, timing.ego_hull, timing.object_hull);
    if (evaluation.risk >= RiskLevel::DANGER) {
      add_collision_planning_factor(
        time_resolution, odometry.header.stamp, odometry.pose.pose, timing, "DRAC",
        artifacts.planning_factors);
    }
  }

  if (findings.empty()) {
    return;
  }

  const auto findings_text = join_findings(findings);
  reporter::append_text_marker_message(artifacts.error_msg, findings_text);
  reporter::log_collision_messages(
    drac_artifact.risk, format_finding_count(findings.size(), "DRAC collision"), findings_text);
}

void process_rss_artifacts(const RssArtifact & rss_artifact, VisualizationData & artifacts)
{
  if (rss_artifact.risk == RiskLevel::SAFE) {
    return;
  }

  std::vector<std::string> findings{};
  findings.reserve(rss_artifact.object_evaluations.size());
  for (const auto & evaluation : rss_artifact.object_evaluations) {
    if (evaluation.risk == RiskLevel::SAFE) {
      continue;
    }
    const auto & detail = evaluation.detail;

    findings.push_back(
      fmt::format(
        "  [{}] {} {}\n"
        "      result: {}, ego req. acc = {:.2f} m/s^2 | stamp {}",
        findings.size() + 1, detail.object_identification.classification,
        detail.object_identification.object_id_string(), format_risk_level(evaluation.risk),
        detail.rss_acceleration, format_stamp(detail.object_identification.stamp)));
  }

  if (findings.empty()) {
    return;
  }

  const auto findings_text = join_findings(findings);
  reporter::append_text_marker_message(artifacts.error_msg, findings_text);
  reporter::log_collision_messages(
    rss_artifact.risk, format_finding_count(findings.size(), "RSS violation"), findings_text);
}
}  // namespace

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

void log_collision_messages(
  const RiskLevel::_level_type level, const std::string & summary, const std::string & findings)
{
  if (findings.empty()) {
    return;
  }
  if (level >= RiskLevel::DANGER) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CollisionCheckFilter"), "Not feasible: %s\n\n%s", summary.c_str(),
      findings.c_str());
    return;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("CollisionCheckFilter"), "Warning: %s\n\n%s", summary.c_str(),
    findings.c_str());
}

autoware_internal_planning_msgs::msg::PlanningFactorArray process_collision_artifacts(
  const nav_msgs::msg::Odometry & odometry, const DracArtifact & drac_artifact,
  const RssArtifact & rss_artifact, visualization_msgs::msg::MarkerArray & debug_markers,
  double time_resolution)
{
  VisualizationData visualization_data{};
  const auto current_time = rclcpp::Time{odometry.header.stamp};

  process_drac_artifacts(
    odometry, current_time, drac_artifact, visualization_data, debug_markers, time_resolution);
  process_rss_artifacts(rss_artifact, visualization_data);

  if (!visualization_data.error_msg.empty()) {
    add_error_text_marker(
      debug_markers, current_time, odometry.pose.pose, visualization_data.error_msg);
  }

  return std::move(visualization_data.planning_factors);
}
}  // namespace autoware::trajectory_validator::plugin::safety::reporter
