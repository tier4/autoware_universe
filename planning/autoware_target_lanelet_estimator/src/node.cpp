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
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::target_lanelet_estimator
{
namespace
{
// Ego heatmap color from posterior probability: cold blue -> hot red, more opaque when higher.
std_msgs::msg::ColorRGBA probability_to_marker_color(double probability)
{
  const float t = static_cast<float>(std::clamp(probability, 0.0, 1.0));
  return autoware_utils::create_marker_color(t, 0.0f, 1.0f - t, 0.3f + 0.5f * t);
}

// Object heatmap in a different hue (green -> yellow) so surrounding vehicles are told apart from
// the ego, whose lanelets they may share.
std_msgs::msg::ColorRGBA object_probability_to_marker_color(double probability)
{
  const float t = static_cast<float>(std::clamp(probability, 0.0, 1.0));
  return autoware_utils::create_marker_color(t, 1.0f, 0.0f, 0.3f + 0.5f * t);
}

// Draw objects as a narrow centered band lifted above the ego full-width fill, so an object sharing
// a lanelet with the ego shows as a distinct stripe on top instead of overlapping the whole lane.
constexpr double object_marker_z_offset = 0.1;
constexpr double object_band_lateral_from = 0.35;
constexpr double object_band_lateral_to = 0.65;

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

// Fill a lanelet as a triangle strip between two lateral fractions of its width (0 = left bound,
// 1 = right bound). Robust by construction (no ear-clipping), so it never produces invalid
// triangles. (0, 1) fills the whole lanelet; a centered sub-range draws a narrower band.
std::vector<geometry_msgs::msg::Point> create_lanelet_band_triangles(
  const lanelet::ConstLanelet & lanelet, double lateral_from, double lateral_to)
{
  const auto left = lanelet.leftBound();
  const auto right = lanelet.rightBound();
  std::vector<geometry_msgs::msg::Point> triangles;
  if (left.size() < 2 || right.size() < 2) {
    return triangles;
  }

  const auto lateral_point = [&](double longitudinal, double lateral) {
    const auto l = interpolate_linestring(left, longitudinal);
    const auto r = interpolate_linestring(right, longitudinal);
    return to_msg_point(l * (1.0 - lateral) + r * lateral);
  };

  const size_t segments = std::max(left.size(), right.size()) - 1;
  for (size_t i = 0; i < segments; ++i) {
    const double r0 = static_cast<double>(i) / static_cast<double>(segments);
    const double r1 = static_cast<double>(i + 1) / static_cast<double>(segments);
    const auto from_start = lateral_point(r0, lateral_from);
    const auto from_end = lateral_point(r1, lateral_from);
    const auto to_start = lateral_point(r0, lateral_to);
    const auto to_end = lateral_point(r1, lateral_to);
    triangles.insert(triangles.end(), {from_start, to_start, to_end, from_start, to_end, from_end});
  }
  return triangles;
}

// The dominant classification is CAR.
bool is_car(const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto & classification = object.classification;
  if (classification.empty()) {
    return false;
  }
  const auto dominant = std::max_element(
    classification.begin(), classification.end(),
    [](const auto & a, const auto & b) { return a.probability < b.probability; });
  return dominant->label == autoware_perception_msgs::msg::ObjectClassification::CAR;
}

std::string to_uuid_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto byte : uuid.uuid) {
    ss << std::setw(2) << static_cast<int>(byte);
  }
  return ss.str();
}

// Local-frame footprint of an object from its bounding-box dimensions (x = length, y = width).
autoware_utils_geometry::LinearRing2d object_base_footprint(
  const autoware_perception_msgs::msg::Shape & shape)
{
  const double half_length = 0.5 * shape.dimensions.x;
  const double half_width = 0.5 * shape.dimensions.y;
  autoware_utils_geometry::LinearRing2d footprint;
  footprint.push_back(autoware_utils_geometry::Point2d(half_length, half_width));
  footprint.push_back(autoware_utils_geometry::Point2d(half_length, -half_width));
  footprint.push_back(autoware_utils_geometry::Point2d(-half_length, -half_width));
  footprint.push_back(autoware_utils_geometry::Point2d(-half_length, half_width));
  footprint.push_back(autoware_utils_geometry::Point2d(half_length, half_width));  // close the ring
  return footprint;
}

// Poses of the object's most likely predicted path.
std::vector<geometry_msgs::msg::Pose> best_predicted_path_poses(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto & paths = object.kinematics.predicted_paths;
  if (paths.empty()) {
    return {};
  }
  const auto best = std::max_element(
    paths.begin(), paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  return {best->path.begin(), best->path.end()};
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

  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    [this](const PredictedObjects::ConstSharedPtr msg) { on_objects(msg); });

  pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/route_marker", 1);
  pub_object_marker_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/object_marker", 1);
  pub_target_lanelet_ids_ =
    create_publisher<Int64MultiArrayStamped>("~/output/target_lanelet_ids", 1);
  pub_target_lanelet_probabilities_ =
    create_publisher<Float64MultiArrayStamped>("~/output/target_lanelet_probabilities", 1);
  pub_out_of_lanelet_ = create_publisher<BoolStamped>("~/output/out_of_lanelet", 1);
  pub_ego_text_ = create_publisher<StringStamped>("~/debug/ego_target_text", 1);
  pub_object_text_ = create_publisher<StringStamped>("~/debug/object_target_text", 1);

  track_objects_ = declare_parameter<bool>("track_objects");
  target_object_uuid_ = declare_parameter<std::string>("target_object_uuid");
  max_tracked_objects_ = declare_parameter<int64_t>("max_tracked_objects");

  params_.preferred_lanelet_initial_probability =
    declare_parameter<double>("preferred_lanelet_initial_probability");
  params_.other_lanelet_initial_probability =
    declare_parameter<double>("other_lanelet_initial_probability");
  params_.same_segment_lane_change_probability =
    declare_parameter<double>("same_segment_lane_change_probability");
  params_.following_transition_weight = declare_parameter<double>("following_transition_weight");
  params_.non_following_transition_weight =
    declare_parameter<double>("non_following_transition_weight");
  params_.selection_likelihood_threshold =
    declare_parameter<double>("selection_likelihood_threshold");
  params_.out_of_lanelet_search_margin = declare_parameter<double>("out_of_lanelet_search_margin");

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
  ego_tracker_ = {};
  object_trackers_.clear();
  covered_lanelet_ids_.clear();
  RCLCPP_INFO(get_logger(), "Received route (%zu segments).", msg->segments.size());
}

void TargetLaneletEstimatorNode::on_trajectory(const Trajectory::ConstSharedPtr msg)
{
  trajectory_ = msg;
  run_estimation();
}

bool TargetLaneletEstimatorNode::inputs_ready()
{
  const bool ready = lanelet_map_ && route_ && routing_graph_;
  if (!ready) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for inputs (map: %d, route: %d).",
      lanelet_map_ != nullptr, route_ != nullptr);
  }
  return ready;
}

void TargetLaneletEstimatorNode::run_estimation()
{
  if (!inputs_ready() || !trajectory_) {
    return;
  }

  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(trajectory_->points.size());
  for (const auto & point : trajectory_->points) {
    poses.push_back(point.pose);
  }
  ego_tracker_.update(
    *route_, poses, vehicle_info_.createFootprint(), lanelet_map_, routing_graph_, params_);
  const auto & result = ego_tracker_.get_target_lanelets();

  for (const auto & lanelet : result.lanelet_probabilities) {
    // once the trajectory reaches a lanelet it stays on the colored trail
    if (lanelet.likelihood > 0.0) {
      covered_lanelet_ids_.insert(lanelet.id);
    }
  }

  // detailed per-lanelet text goes to a dedicated topic (echo it in its own terminal); the console
  // keeps only a brief count so the ego and object logs do not clutter each other.
  std::stringstream text;
  text << std::fixed << std::setprecision(2) << "ego target lanelets ("
       << result.target_lanelet_ids.size() << "):";
  const auto probability_by_id = create_probability_map(result.lanelet_probabilities);
  for (const auto id : result.target_lanelet_ids) {
    text << " " << id << ":" << probability_by_id.at(id);
  }
  if (result.out_of_lanelet) {
    text << " (out_of_lanelet)";
  }
  StringStamped text_msg;
  text_msg.stamp = now();
  text_msg.data = text.str();
  pub_ego_text_->publish(text_msg);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000, "ego target lanelets: %zu%s",
    result.target_lanelet_ids.size(), result.out_of_lanelet ? " (out_of_lanelet)" : "");

  publish_result(result);
  publish_markers(result);
}

void TargetLaneletEstimatorNode::on_objects(const PredictedObjects::ConstSharedPtr msg)
{
  if (!track_objects_ || !inputs_ready()) {
    return;
  }

  std::unordered_set<std::string> present_uuids;
  for (const auto & object : msg->objects) {
    if (!is_car(object)) {
      continue;
    }
    const auto uuid = to_uuid_string(object.object_id);
    if (!target_object_uuid_.empty() && uuid != target_object_uuid_) {
      continue;
    }
    if (
      max_tracked_objects_ > 0 &&
      static_cast<int64_t>(present_uuids.size()) >= max_tracked_objects_) {
      break;
    }
    const auto poses = best_predicted_path_poses(object);
    if (poses.empty()) {
      continue;
    }
    object_trackers_[uuid].update(
      *route_, poses, object_base_footprint(object.shape), lanelet_map_, routing_graph_, params_);
    present_uuids.insert(uuid);
  }

  // drop trackers of objects no longer seen so the map does not grow unbounded
  for (auto it = object_trackers_.begin(); it != object_trackers_.end();) {
    it = present_uuids.count(it->first) ? std::next(it) : object_trackers_.erase(it);
  }

  // detailed per-object text goes to its own topic (echo it in a separate terminal); the console
  // keeps only a brief count so it does not interleave with the ego log.
  std::stringstream text;
  text << std::fixed << std::setprecision(2) << "objects tracked: " << object_trackers_.size();
  size_t objects_with_target = 0;
  for (const auto & [uuid, tracker] : object_trackers_) {
    const auto & result = tracker.get_target_lanelets();
    if (result.target_lanelet_ids.empty()) {
      continue;
    }
    ++objects_with_target;
    const auto probability_by_id = create_probability_map(result.lanelet_probabilities);
    text << "\n  " << uuid.substr(0, 8) << ":";
    for (const auto id : result.target_lanelet_ids) {
      text << " " << id << ":" << probability_by_id.at(id);
    }
  }
  StringStamped text_msg;
  text_msg.stamp = now();
  text_msg.data = text.str();
  pub_object_text_->publish(text_msg);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000, "objects tracked: %zu (with target lanelets: %zu)",
    object_trackers_.size(), objects_with_target);

  publish_object_markers();
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

  for (const auto id : result.target_lanelet_ids) {
    const auto probability_it = probability_by_id.find(id);
    const auto probability =
      probability_it != probability_by_id.end() ? probability_it->second : 0.0;
    ids_msg.data.push_back(id);
    probabilities_msg.data.push_back(probability);
  }

  BoolStamped out_of_lanelet_msg;
  out_of_lanelet_msg.stamp = stamp;
  out_of_lanelet_msg.data = result.out_of_lanelet;

  pub_target_lanelet_ids_->publish(ids_msg);
  pub_target_lanelet_probabilities_->publish(probabilities_msg);
  pub_out_of_lanelet_->publish(out_of_lanelet_msg);
}

void TargetLaneletEstimatorNode::ensure_route_triangles()
{
  if (triangles_route_ == route_) {
    return;
  }
  route_triangles_.clear();
  for (const auto & segment : route_->segments) {
    for (const auto & primitive : segment.primitives) {
      if (!lanelet_map_->laneletLayer.exists(primitive.id)) {
        continue;
      }
      const auto route_lanelet = lanelet_map_->laneletLayer.get(primitive.id);
      LaneletTriangles triangles;
      triangles.id = primitive.id;
      triangles.points = create_lanelet_band_triangles(route_lanelet, 0.0, 1.0);
      triangles.narrow_points = create_lanelet_band_triangles(
        route_lanelet, object_band_lateral_from, object_band_lateral_to);
      route_triangles_.push_back(std::move(triangles));
    }
  }
  triangles_route_ = route_;
}

void TargetLaneletEstimatorNode::publish_markers(const TargetLaneletsResult & result)
{
  const bool route_changed = triangles_route_ != route_;
  ensure_route_triangles();

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

void TargetLaneletEstimatorNode::publish_object_markers()
{
  ensure_route_triangles();

  std::unordered_map<lanelet::Id, const std::vector<geometry_msgs::msg::Point> *> triangles_by_id;
  triangles_by_id.reserve(route_triangles_.size());
  for (const auto & triangles : route_triangles_) {
    triangles_by_id[triangles.id] = &triangles.narrow_points;  // narrow band so it sits inside ego
  }

  // objects move, so clear the previous markers and redraw the current target lanelets each cycle
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int marker_id = 0;
  for (const auto & [uuid, tracker] : object_trackers_) {
    const auto & result = tracker.get_target_lanelets();
    const auto probability_by_id = create_probability_map(result.lanelet_probabilities);
    for (const auto id : result.target_lanelet_ids) {
      const auto it = triangles_by_id.find(id);
      if (it == triangles_by_id.end()) {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = now();
      marker.ns = "object_" + uuid;
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.pose.orientation.w = 1.0;
      marker.pose.position.z = object_marker_z_offset;
      marker.points = *it->second;
      marker.color = object_probability_to_marker_color(probability_by_id.at(id));
      marker_array.markers.push_back(marker);
    }
  }

  pub_object_marker_->publish(marker_array);
}

}  // namespace autoware::target_lanelet_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::target_lanelet_estimator::TargetLaneletEstimatorNode)
