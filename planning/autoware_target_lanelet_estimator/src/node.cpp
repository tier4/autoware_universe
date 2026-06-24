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

#include "autoware/target_lanelet_estimator/node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

namespace autoware::target_lanelet_estimator
{
namespace
{
// Heatmap color from posterior probability: cold blue -> hot red, more opaque when higher.
std_msgs::msg::ColorRGBA probability_to_marker_color(double probability)
{
  const float t = static_cast<float>(std::clamp(probability, 0.0, 1.0));
  return autoware_utils::create_marker_color(t, 0.0f, 1.0f - t, 0.3f + 0.5f * t);
}

LaneletProbabilityMap create_probability_map(
  const std::vector<LaneletProbability> & lanelet_probabilities)
{
  LaneletProbabilityMap probability_by_id;
  probability_by_id.reserve(lanelet_probabilities.size());
  for (const auto & lanelet : lanelet_probabilities) {
    probability_by_id[lanelet.id] = lanelet.posterior;
  }
  return probability_by_id;
}

const std_msgs::msg::ColorRGBA transparent_marker_color =
  autoware_utils::create_marker_color(0.0f, 0.0f, 0.0f, 0.0f);

geometry_msgs::msg::Point to_msg_point(const lanelet::BasicPoint3d & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  return point;
}

// point along a line by normalized ratio (0 = start, 1 = end), interpolating between vertices
lanelet::BasicPoint3d interpolate_linestring(const lanelet::ConstLineString3d & line, double ratio)
{
  const double fractional_index = ratio * static_cast<double>(line.size() - 1);
  const auto i0 = static_cast<size_t>(std::floor(fractional_index));
  const auto i1 = std::min(i0 + 1, line.size() - 1);
  const double t = fractional_index - static_cast<double>(i0);
  return line[i0].basicPoint() * (1.0 - t) + line[i1].basicPoint() * t;
}

// Fill a lanelet as a triangle strip between its left and right bounds.
// Robust by construction (no ear-clipping), so it never produces invalid triangles.
std::vector<geometry_msgs::msg::Point> create_lanelet_triangle_list(
  const lanelet::ConstLanelet & lanelet)
{
  const auto left = lanelet.leftBound();
  const auto right = lanelet.rightBound();
  std::vector<geometry_msgs::msg::Point> triangles;
  if (left.size() < 2 || right.size() < 2) {
    return triangles;
  }

  const size_t segments = std::max(left.size(), right.size()) - 1;
  for (size_t i = 0; i < segments; ++i) {
    const double r0 = static_cast<double>(i) / static_cast<double>(segments);
    const double r1 = static_cast<double>(i + 1) / static_cast<double>(segments);
    const auto left_start = to_msg_point(interpolate_linestring(left, r0));
    const auto left_end = to_msg_point(interpolate_linestring(left, r1));
    const auto right_start = to_msg_point(interpolate_linestring(right, r0));
    const auto right_end = to_msg_point(interpolate_linestring(right, r1));
    triangles.insert(
      triangles.end(), {left_start, right_start, right_end, left_start, right_end, left_end});
  }
  return triangles;
}
}  // namespace

TargetLaneletEstimatorNode::TargetLaneletEstimatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("target_lanelet_estimator", options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  // map is latched
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { on_map(msg); });

  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletRoute::ConstSharedPtr msg) { on_route(msg); });

  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    [this](const Trajectory::ConstSharedPtr msg) { on_trajectory(msg); });

  pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/route_marker", 1);
  pub_target_lanelet_ids_ =
    create_publisher<Int64MultiArrayStamped>("~/output/target_lanelet_ids", 1);
  pub_target_lanelet_probabilities_ =
    create_publisher<Float64MultiArrayStamped>("~/output/target_lanelet_probabilities", 1);
  pub_out_of_lanelet_ = create_publisher<BoolStamped>("~/output/out_of_lanelet", 1);
  pub_debug_text_ = create_publisher<StringStamped>("~/debug/target_lanelet_text", 1);

  RCLCPP_INFO(
    get_logger(), "vehicle_info: length=%.2fm width=%.2fm wheel_base=%.2fm",
    vehicle_info_.vehicle_length_m, vehicle_info_.vehicle_width_m, vehicle_info_.wheel_base_m);
}

void TargetLaneletEstimatorNode::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  lanelet_map_ = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg);
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  routing_graph_ = lanelet::routing::RoutingGraph::build(*lanelet_map_, *traffic_rules);
  RCLCPP_INFO(
    get_logger(), "Received vector map (%zu lanelets).", lanelet_map_->laneletLayer.size());
}

void TargetLaneletEstimatorNode::on_route(const LaneletRoute::ConstSharedPtr msg)
{
  route_ = msg;
  posterior_probabilities_ = initialize_lanelet_probabilities(*route_);
  covered_lanelet_ids_.clear();
  RCLCPP_INFO(get_logger(), "Received route (%zu segments).", msg->segments.size());
}

void TargetLaneletEstimatorNode::on_trajectory(const Trajectory::ConstSharedPtr msg)
{
  trajectory_ = msg;
  run_estimation();
}

void TargetLaneletEstimatorNode::run_estimation()
{
  if (!lanelet_map_ || !route_ || !trajectory_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for inputs (map: %d, route: %d, trajectory: %d).",
      lanelet_map_ != nullptr, route_ != nullptr, trajectory_ != nullptr);
    return;
  }

  const auto result = get_target_lanelets(
    *route_, *trajectory_, lanelet_map_, vehicle_info_, posterior_probabilities_, routing_graph_);
  posterior_probabilities_.clear();
  for (const auto & lanelet : result.lanelet_probabilities) {
    posterior_probabilities_[lanelet.id] = lanelet.posterior;
    if (lanelet.likelihood > 0.0) {
      covered_lanelet_ids_.insert(
        lanelet.id);  // keep it on the trail once the trajectory reaches it
    }
  }

  // Only the lanelets the trajectory actually touches are interesting: those updated this cycle and
  // those already on the trail. The rest of the route just sits at its initial prior, so collapse
  // it into a count instead of printing every lanelet.
  std::stringstream log_stream;
  log_stream << std::fixed << std::setprecision(2);
  size_t shown_lanelet_count = 0;
  for (const auto & lanelet : result.lanelet_probabilities) {
    const bool on_trail = covered_lanelet_ids_.count(lanelet.id) > 0;
    if (!lanelet.updated && !on_trail) {
      continue;
    }
    log_stream << lanelet.id << ":" << lanelet.posterior;
    if (lanelet.updated) {
      log_stream << "(prior=" << lanelet.prior << ",L=" << lanelet.likelihood << ")";
    }
    log_stream << " ";
    ++shown_lanelet_count;
  }
  if (result.out_of_lanelet) {
    log_stream << "out_of_lanelet";
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000, "trajectory lanelets (%zu of %zu route): %s",
    shown_lanelet_count, result.lanelet_probabilities.size(), log_stream.str().c_str());

  publish_result(result);
  publish_markers(result);
}

void TargetLaneletEstimatorNode::publish_result(const TargetLaneletsResult & result)
{
  const auto stamp = now();
  const auto probability_by_id = create_probability_map(result.lanelet_probabilities);

  Int64MultiArrayStamped ids_msg;
  ids_msg.stamp = stamp;
  ids_msg.layout.dim.resize(1);
  ids_msg.layout.dim.front().label = "target_lanelets";
  ids_msg.layout.dim.front().size = result.target_lanelet_ids.size();
  ids_msg.layout.dim.front().stride = result.target_lanelet_ids.size();
  ids_msg.layout.data_offset = 0;
  ids_msg.data.reserve(result.target_lanelet_ids.size());

  Float64MultiArrayStamped probabilities_msg;
  probabilities_msg.stamp = stamp;
  probabilities_msg.layout = ids_msg.layout;
  probabilities_msg.data.reserve(result.target_lanelet_ids.size());

  std::stringstream text;
  text << std::fixed << std::setprecision(3);
  if (result.out_of_lanelet) {
    text << "out_of_lanelet";
  } else if (result.target_lanelet_ids.empty()) {
    text << "no_route_lanelet_overlap";
  } else {
    text << "target_lanelets:";
  }

  for (const auto id : result.target_lanelet_ids) {
    const auto probability_it = probability_by_id.find(id);
    const auto probability =
      probability_it != probability_by_id.end() ? probability_it->second : 0.0;
    ids_msg.data.push_back(id);
    probabilities_msg.data.push_back(probability);
    text << " " << id << "(" << probability << ")";
  }

  BoolStamped out_of_lanelet_msg;
  out_of_lanelet_msg.stamp = stamp;
  out_of_lanelet_msg.data = result.out_of_lanelet;

  StringStamped text_msg;
  text_msg.stamp = stamp;
  text_msg.data = text.str();

  pub_target_lanelet_ids_->publish(ids_msg);
  pub_target_lanelet_probabilities_->publish(probabilities_msg);
  pub_out_of_lanelet_->publish(out_of_lanelet_msg);
  pub_debug_text_->publish(text_msg);
}

void TargetLaneletEstimatorNode::publish_markers(const TargetLaneletsResult & result)
{
  const bool route_changed = triangles_route_ != route_;
  if (route_changed) {
    route_triangles_.clear();
    for (const auto & segment : route_->segments) {
      for (const auto & primitive : segment.primitives) {
        if (!lanelet_map_->laneletLayer.exists(primitive.id)) {
          continue;
        }
        const auto route_lanelet = lanelet_map_->laneletLayer.get(primitive.id);
        LaneletTriangles triangles;
        triangles.id = primitive.id;
        triangles.points = create_lanelet_triangle_list(route_lanelet);
        route_triangles_.push_back(std::move(triangles));
      }
    }
    triangles_route_ = route_;
  }

  const auto probability_by_id = create_probability_map(result.lanelet_probabilities);

  visualization_msgs::msg::MarkerArray marker_array;
  if (route_changed) {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
  }

  int marker_id = 0;
  for (const auto & triangles : route_triangles_) {
    const auto probability_it = probability_by_id.find(triangles.id);
    const double probability =
      probability_it != probability_by_id.end() ? probability_it->second : 0.0;
    const bool covered = covered_lanelet_ids_.count(triangles.id) > 0;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "lanelet_probability";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.points = triangles.points;
    marker.color = covered ? probability_to_marker_color(probability) : transparent_marker_color;
    marker_array.markers.push_back(marker);
  }

  pub_marker_->publish(marker_array);
}

}  // namespace autoware::target_lanelet_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::target_lanelet_estimator::TargetLaneletEstimatorNode)
