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
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::target_lanelet_estimator
{
namespace
{
// Heatmap color from posterior probability: cold blue -> hot red, more opaque when higher.
std_msgs::msg::ColorRGBA score_to_color(double score)
{
  const float t = static_cast<float>(std::clamp(score, 0.0, 1.0));
  return autoware_utils::create_marker_color(t, 0.0f, 1.0f - t, 0.2f + 0.6f * t);
}

geometry_msgs::msg::Point to_point(const lanelet::BasicPoint3d & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  return point;
}

// point along a line by normalized ratio (0 = start, 1 = end), interpolating between vertices
lanelet::BasicPoint3d interpolate(const lanelet::ConstLineString3d & line, double ratio)
{
  const double fractional_index = ratio * static_cast<double>(line.size() - 1);
  const auto i0 = static_cast<size_t>(std::floor(fractional_index));
  const auto i1 = std::min(i0 + 1, line.size() - 1);
  const double t = fractional_index - static_cast<double>(i0);
  return line[i0].basicPoint() * (1.0 - t) + line[i1].basicPoint() * t;
}

// Fill a lanelet as a triangle strip between its left and right bounds.
// Robust by construction (no ear-clipping), so it never produces invalid triangles.
std::vector<geometry_msgs::msg::Point> lanelet_fill_triangles(const lanelet::ConstLanelet & ll)
{
  const auto left = ll.leftBound();
  const auto right = ll.rightBound();
  std::vector<geometry_msgs::msg::Point> triangles;
  if (left.size() < 2 || right.size() < 2) {
    return triangles;
  }

  const size_t segments = std::max(left.size(), right.size()) - 1;
  for (size_t i = 0; i < segments; ++i) {
    const double r0 = static_cast<double>(i) / static_cast<double>(segments);
    const double r1 = static_cast<double>(i + 1) / static_cast<double>(segments);
    const auto l0 = to_point(interpolate(left, r0));
    const auto l1 = to_point(interpolate(left, r1));
    const auto rg0 = to_point(interpolate(right, r0));
    const auto rg1 = to_point(interpolate(right, r1));
    triangles.insert(triangles.end(), {l0, rg0, rg1, l0, rg1, l1});
  }
  return triangles;
}
}  // namespace

TargetLaneletEstimatorNode::TargetLaneletEstimatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("target_lanelet_estimator", options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  using std::placeholders::_1;

  // map is latched
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TargetLaneletEstimatorNode::on_map, this, _1));

  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&TargetLaneletEstimatorNode::on_route, this, _1));

  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&TargetLaneletEstimatorNode::on_trajectory, this, _1));

  pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/route_marker", 1);

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
  for (const auto & lanelet : result.lanelets) {
    posterior_probabilities_[lanelet.id] = lanelet.score;
  }

  std::stringstream ids;
  ids << std::fixed << std::setprecision(2);
  for (const auto & lanelet : result.lanelets) {
    ids << lanelet.id << ":" << lanelet.score;
    if (lanelet.updated) {
      ids << "(prior=" << lanelet.prior << ",L=" << lanelet.likelihood << ")";
    }
    ids << " ";
  }
  if (result.out_of_lanelet) {
    ids << "out_of_lanelet";
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000, "target lanelets (%zu): %s", result.lanelets.size(),
    ids.str().c_str());

  publish_markers(result);
}

void TargetLaneletEstimatorNode::publish_markers(const TargetLaneletsResult & result)
{
  const bool route_changed = triangles_route_ != route_;
  if (route_changed) {
    route_triangles_.clear();
    for (const auto & segment : route_->segments) {
      for (const auto & primitive : segment.primitives) {
        const auto route_lanelet = lanelet_map_->laneletLayer.get(primitive.id);
        LaneletTriangles triangles;
        triangles.id = primitive.id;
        triangles.points = lanelet_fill_triangles(route_lanelet);
        route_triangles_.push_back(std::move(triangles));
      }
    }
    triangles_route_ = route_;
  }

  std::unordered_map<lanelet::Id, double> score_by_id;
  for (const auto & scored : result.lanelets) {
    score_by_id[scored.id] = scored.score;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  if (route_changed) {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
  }

  int marker_id = 0;
  for (const auto & triangles : route_triangles_) {
    const auto it = score_by_id.find(triangles.id);
    const double score = it != score_by_id.end() ? it->second : 0.0;

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
    marker.color = score_to_color(score);
    marker_array.markers.push_back(marker);
  }

  pub_marker_->publish(marker_array);
}

}  // namespace autoware::target_lanelet_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::target_lanelet_estimator::TargetLaneletEstimatorNode)
