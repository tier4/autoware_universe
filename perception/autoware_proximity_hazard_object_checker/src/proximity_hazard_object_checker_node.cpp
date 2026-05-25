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

#include "autoware/proximity_hazard_object_checker/proximity_hazard_object_checker_node.hpp"

#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::proximity_hazard_object_checker
{
ProximityHazardObjectCheckerNode::ProximityHazardObjectCheckerNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("proximity_hazard_object_checker", options)
{
  param_listener_ =
    std::make_shared<proximity_hazard_object::ParamListener>(get_node_parameters_interface());

  const auto node_ms = 1 / param_listener_->get_params().node_hz;
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(node_ms),
    std::bind(&ProximityHazardObjectCheckerNode::on_timer, this));

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  impl_ = std::make_unique<ProximityHazardObjectChecker>(
    param_listener_->get_params(), vehicle_info.createFootprint());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pub_hazards_ = create_publisher<ProximityHazardObjects>(
    "~/output/proximity_hazards", rclcpp::QoS{1}.reliable());
  pub_debug_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/sector_markers", rclcpp::QoS{1});
}

void ProximityHazardObjectCheckerNode::on_timer()
{
  const auto object_ptr = sub_objects_.take_data();

  if (!object_ptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to take predicted objects data");
    return;
  }

  const auto odometry_ptr = sub_odometry_.take_data();
  if (!odometry_ptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to take predicted odometry data");
    return;
  }

  geometry_msgs::msg::TransformStamped to_base_link;
  try {
    to_base_link = tf_buffer_->lookupTransform(
      odometry_ptr->child_frame_id, object_ptr->header.frame_id, object_ptr->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "TF lookup %s -> %s failed: %s",
      object_ptr->header.frame_id.c_str(), odometry_ptr->child_frame_id.c_str(), ex.what());
    return;
  }

  pub_hazards_->publish(impl_->process(*object_ptr, to_base_link));
  publish_sector_markers(odometry_ptr->child_frame_id);
}

void ProximityHazardObjectCheckerNode::publish_sector_markers(const std::string & frame_id)
{
  using visualization_msgs::msg::Marker;
  using visualization_msgs::msg::MarkerArray;

  const auto params = param_listener_->get_params();
  const double range = params.max_detection_range_m;
  const auto & ego = impl_->ego_center();
  const auto stamp = now();

  // Sectors in the same order as bearing_to_sector (CCW around the vehicle).
  struct SectorVis
  {
    const char * name;
    const std::vector<double> & range_deg;
  };
  const std::array<SectorVis, 8> sectors = {{
    {"FRONT", params.sector_range.front},
    {"FRONT_LEFT", params.sector_range.front_left},
    {"LEFT", params.sector_range.left},
    {"REAR_LEFT", params.sector_range.rear_left},
    {"REAR", params.sector_range.rear},
    {"REAR_RIGHT", params.sector_range.rear_right},
    {"RIGHT", params.sector_range.right},
    {"FRONT_RIGHT", params.sector_range.front_right},
  }};

  std_msgs::msg::ColorRGBA color;
  color.r = 1.0f;
  color.g = 1.0f;
  color.b = 1.0f;
  color.a = 0.7f;

  MarkerArray out;
  out.markers.reserve(2 + sectors.size());

  // Outer circle at max_detection_range_m around ego_center.
  {
    Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.frame_locked = true;
    m.ns = "proximity_hazard_sectors";
    m.id = 0;
    m.type = Marker::LINE_STRIP;
    m.action = Marker::ADD;
    m.scale.x = 0.03;
    m.color = color;
    m.pose.orientation.w = 1.0;
    constexpr int kSegments = 64;
    m.points.reserve(kSegments + 1);
    for (int i = 0; i <= kSegments; ++i) {
      const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kSegments);
      geometry_msgs::msg::Point p;
      p.x = ego.x() + range * std::cos(a);
      p.y = ego.y() + range * std::sin(a);
      p.z = 0.0;
      m.points.push_back(p);
    }
    out.markers.push_back(m);
  }

  // Sector boundary lines: one line per sector start angle, from ego_center
  // outward to the outer circle. End-of-one == start-of-next when ranges tile,
  // so no duplicates.
  {
    Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.frame_locked = true;
    m.ns = "proximity_hazard_sectors";
    m.id = 1;
    m.type = Marker::LINE_LIST;
    m.action = Marker::ADD;
    m.scale.x = 0.03;
    m.color = color;
    m.pose.orientation.w = 1.0;
    m.points.reserve(sectors.size() * 2);
    for (const auto & s : sectors) {
      const double a = autoware_utils_math::deg2rad(s.range_deg.front());
      geometry_msgs::msg::Point p0;
      p0.x = ego.x();
      p0.y = ego.y();
      p0.z = 0.0;
      geometry_msgs::msg::Point p1;
      p1.x = ego.x() + range * std::cos(a);
      p1.y = ego.y() + range * std::sin(a);
      p1.z = 0.0;
      m.points.push_back(p0);
      m.points.push_back(p1);
    }
    out.markers.push_back(m);
  }

  // Sector labels placed at angular midpoint, just outside the circle.
  int label_id = 100;
  for (const auto & s : sectors) {
    const double start = s.range_deg.front();
    const double end = s.range_deg.back();
    // Wraparound: if end < start, the sector crosses +/-180; midpoint shifts by 180.
    double mid_deg = (end > start) ? 0.5 * (start + end) : 0.5 * (start + end + 360.0);
    if (mid_deg >= 180.0) {
      mid_deg -= 360.0;
    }
    const double mid_rad = autoware_utils_math::deg2rad(mid_deg);
    constexpr double kLabelRadiusScale = 1.2;
    const double label_r = range * kLabelRadiusScale;

    Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.frame_locked = true;
    m.ns = "proximity_hazard_sectors";
    m.id = label_id++;
    m.type = Marker::TEXT_VIEW_FACING;
    m.action = Marker::ADD;
    m.pose.position.x = ego.x() + label_r * std::cos(mid_rad);
    m.pose.position.y = ego.y() + label_r * std::sin(mid_rad);
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.z = 0.3;  // text height in meters
    m.color = color;
    m.text = s.name;
    out.markers.push_back(m);
  }

  pub_debug_markers_->publish(out);
}

}  // namespace autoware::proximity_hazard_object_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::proximity_hazard_object_checker::ProximityHazardObjectCheckerNode)
