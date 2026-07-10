// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/priority_predictor/traffic_signal_stop_predictor.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/utils.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Utilities.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction::priority_predictor
{
namespace
{

bool isRoadLanelet(const lanelet::ConstLanelet & lanelet)
{
  if (!lanelet.hasAttribute(lanelet::AttributeName::Subtype)) {
    return true;
  }
  const auto subtype = lanelet.attribute(lanelet::AttributeName::Subtype).value();
  return subtype != lanelet::AttributeValueString::Crosswalk &&
         subtype != lanelet::AttributeValueString::Walkway;
}

}  // namespace

std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> stopLineChord(
  const lanelet::ConstLineString3d & stop_line)
{
  geometry_msgs::msg::Point c1;
  c1.x = stop_line.front().x();
  c1.y = stop_line.front().y();
  c1.z = stop_line.front().z();
  geometry_msgs::msg::Point c2;
  c2.x = stop_line.back().x();
  c2.y = stop_line.back().y();
  c2.z = stop_line.back().z();
  return {c1, c2};
}

void clipPathAtStopLine(PredictedPath & path, const lanelet::ConstLineString3d & stop_line)
{
  if (path.path.size() < 2 || stop_line.size() < 2) {
    return;
  }

  const auto [c1, c2] = stopLineChord(stop_line);
  for (size_t i = 1; i < path.path.size(); ++i) {
    const auto crossing_point =
      autoware_utils::intersect(path.path.at(i - 1).position, path.path.at(i).position, c1, c2);
    if (crossing_point) {
      auto crossing = path.path.at(i);
      crossing.position = *crossing_point;
      path.path.resize(i);
      path.path.push_back(crossing);
      return;
    }
  }
}

bool path_crosses_stop_line(const PosePath & path, const lanelet::ConstLineString3d & stop_line)
{
  if (path.size() < 2 || stop_line.size() < 2) {
    return false;
  }
  const auto [c1, c2] = stopLineChord(stop_line);
  for (size_t i = 1; i < path.size(); ++i) {
    if (autoware_utils::intersect(path.at(i - 1).position, path.at(i).position, c1, c2)) {
      return true;
    }
  }
  return false;
}

std::optional<double> arcLengthToStopLine(
  const PosePath & ref_path, const lanelet::ConstLineString3d & stop_line)
{
  if (ref_path.size() < 2 || stop_line.size() < 2) {
    return std::nullopt;
  }

  const auto [c1, c2] = stopLineChord(stop_line);
  double arc_length = 0.0;
  for (size_t i = 1; i < ref_path.size(); ++i) {
    const auto & a = ref_path.at(i - 1).position;
    const auto & b = ref_path.at(i).position;
    if (const auto crossing = autoware_utils::intersect(a, b, c1, c2)) {
      return std::max(arc_length + std::hypot(crossing->x - a.x, crossing->y - a.y), 0.0);
    }
    arc_length += std::hypot(b.x - a.x, b.y - a.y);
  }

  const double center_x = 0.5 * (c1.x + c2.x);
  const double center_y = 0.5 * (c1.y + c2.y);

  double cum_length = 0.0;
  double nearest_arc_length = 0.0;
  size_t nearest_index = 0;
  double nearest_distance_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < ref_path.size(); ++i) {
    if (i > 0) {
      cum_length += autoware_utils::calc_distance2d(ref_path.at(i - 1), ref_path.at(i));
    }
    const double dx = ref_path.at(i).position.x - center_x;
    const double dy = ref_path.at(i).position.y - center_y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_arc_length = cum_length;
      nearest_index = i;
    }
  }
  if (nearest_index + 1 < ref_path.size()) {
    return std::nullopt;
  }
  return std::max(nearest_arc_length, 0.0);
}

bool hasStopLineAhead(
  const geometry_msgs::msg::Point & position, const PosePath & ref_path,
  const lanelet::ConstLineString3d & stop_line)
{
  const auto arc_length = arcLengthToStopLine(ref_path, stop_line);
  if (!arc_length) {
    return false;
  }
  const auto & r0 = ref_path.front();
  const double h0 = tf2::getYaw(r0.orientation);
  const double s_obj =
    (position.x - r0.position.x) * std::cos(h0) + (position.y - r0.position.y) * std::sin(h0);
  return *arc_length > s_obj;
}

bool hasTrafficLight(const lanelet::ConstLanelet & way_lanelet)
{
  return !way_lanelet.regulatoryElementsAs<lanelet::TrafficLight>().empty();
}

std::optional<lanelet::ConstLineString3d> getStopLine(const lanelet::ConstLanelet & way_lanelet)
{
  for (const auto & traffic_light : way_lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
    if (const auto stop_line = traffic_light->stopLine()) {
      return *stop_line;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLineString3d> getStopLineOrEntryEdge(
  const lanelet::ConstLanelet & way_lanelet)
{
  if (const auto stop_line = getStopLine(way_lanelet)) {
    return stop_line;
  }
  const auto & left = way_lanelet.leftBound();
  const auto & right = way_lanelet.rightBound();
  if (left.empty() || right.empty()) {
    return std::nullopt;
  }
  const auto lp = left.front();
  const auto rp = right.front();
  return lanelet::ConstLineString3d(
    lanelet::LineString3d(
      lanelet::utils::getId(),
      {lanelet::Point3d(lanelet::utils::getId(), lp.x(), lp.y(), lp.z()),
       lanelet::Point3d(lanelet::utils::getId(), rp.x(), rp.y(), rp.z())}));
}

std::optional<lanelet::Id> getTrafficSignalId(const lanelet::ConstLanelet & way_lanelet)
{
  const auto traffic_light_reg_elems =
    way_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
  if (traffic_light_reg_elems.empty()) {
    return std::nullopt;
  }
  if (traffic_light_reg_elems.size() > 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("map_based_prediction"),
      "[Map Based Prediction]: Multiple regulatory elements as TrafficLight are defined to one "
      "lanelet object.");
  }
  return traffic_light_reg_elems.front()->id();
}

std::optional<TrafficLightGroup> getSignalForLanelet(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & signal_id_map,
  const lanelet::ConstLanelet & way_lanelet)
{
  const auto signal_id = getTrafficSignalId(way_lanelet);
  if (!signal_id) {
    return std::nullopt;
  }
  const auto it = signal_id_map.find(*signal_id);
  if (it == signal_id_map.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<SignalizedStopLine> collect_signalized_stop_lines(const lanelet::LaneletMap & lanelet_map)
{
  std::vector<SignalizedStopLine> signalized_stop_lines;
  for (const auto & lanelet : lanelet_map.laneletLayer) {
    if (!isRoadLanelet(lanelet) || !hasTrafficLight(lanelet)) {
      continue;
    }
    const auto stop_line = getStopLineOrEntryEdge(lanelet);
    if (!stop_line || stop_line->size() < 2) {
      continue;
    }
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto & point : *stop_line) {
      min_x = std::min(min_x, point.x());
      max_x = std::max(max_x, point.x());
      min_y = std::min(min_y, point.y());
      max_y = std::max(max_y, point.y());
    }
    signalized_stop_lines.push_back({lanelet, *stop_line, min_x, min_y, max_x, max_y});
  }
  return signalized_stop_lines;
}

bool evaluateSignalStopRequirement(
  const lanelet::ConstLanelet & way_lanelet, const std::optional<TrafficLightGroup> & signal)
{
  if (!signal) {
    return false;
  }
  return autoware::traffic_light_utils::isTrafficSignalStop(way_lanelet, *signal);
}

bool shouldAddStopHypothesis(const bool signal_requires_stop, const bool has_stop_line_ahead)
{
  return signal_requires_stop && has_stop_line_ahead;
}

double weakenConfidenceInLaneChange(const Maneuver & maneuver, const double stop_weight)
{
  constexpr double lane_change_penalty = 0.5;
  return maneuver == Maneuver::LANE_FOLLOW ? stop_weight : stop_weight * lane_change_penalty;
}

namespace
{

bool can_stop_before_stop_line(
  const TrackedObject & object, const PosePath & predicted_path,
  const lanelet::ConstLineString3d & stop_line,
  const path_cut::MaxDecelerationParams & max_decel_params)
{
  const auto distance_to_line = arcLengthToStopLine(predicted_path, stop_line);
  if (!distance_to_line) {
    return false;
  }
  const auto & twist = object.kinematics.twist_with_covariance.twist;
  const double object_speed = std::hypot(twist.linear.x, twist.linear.y);
  const double max_deceleration = path_cut::max_deceleration_for_label(
    max_decel_params,
    autoware::object_recognition_utils::getHighestProbLabel(object.classification));
  return path_cut::can_stop_before_the_line(*distance_to_line, object_speed, max_deceleration);
}

struct StopLineSelection
{
  std::optional<lanelet::ConstLineString3d> stop_line;
  bool signal_requires_stop{false};
  bool stop_line_ahead{false};
};

std::array<double, 4> predicted_path_bounds(const PredictedPath & predicted_path)
{
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & pose : predicted_path.path) {
    min_x = std::min(min_x, pose.position.x);
    max_x = std::max(max_x, pose.position.x);
    min_y = std::min(min_y, pose.position.y);
    max_y = std::max(max_y, pose.position.y);
  }
  return {min_x, min_y, max_x, max_y};
}

// Associate the path with the stop line it actually crosses (geometry), not with the
// lanelet it snaps to: pick the nearest crossed stop line whose signal requires a stop and
// that the object can brake for. A laterally-drifting path that snaps to a parallel lane is
// still clipped at the red stop line it crosses. The stop-line bbox pre-filter keeps the
// map-wide scan cheap (a path only overlaps stop lines in its local intersection).
StopLineSelection select_nearest_stop_line_requiring_stop(
  const PredictedPath & predicted_path, const TrackedObject & object,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & traffic_signal_id_map,
  const std::vector<SignalizedStopLine> & signalized_stop_lines,
  const path_cut::MaxDecelerationParams & max_decel_params)
{
  StopLineSelection selection;
  double nearest_arc_length = std::numeric_limits<double>::infinity();
  const auto [path_min_x, path_min_y, path_max_x, path_max_y] =
    predicted_path_bounds(predicted_path);

  for (const auto & candidate : signalized_stop_lines) {
    if (
      candidate.max_x < path_min_x || candidate.min_x > path_max_x ||
      candidate.max_y < path_min_y || candidate.min_y > path_max_y) {
      continue;
    }
    const std::optional<TrafficLightGroup> signal_status =
      getSignalForLanelet(traffic_signal_id_map, candidate.lanelet);
    if (!evaluateSignalStopRequirement(candidate.lanelet, signal_status)) {
      continue;
    }
    selection.signal_requires_stop = true;

    if (
      !path_crosses_stop_line(predicted_path.path, candidate.stop_line) ||
      !hasStopLineAhead(
        object.kinematics.pose_with_covariance.pose.position, predicted_path.path,
        candidate.stop_line)) {
      continue;
    }
    selection.stop_line_ahead = true;

    // Keep the constant-velocity path when the object cannot brake in time (it runs the light).
    if (!can_stop_before_stop_line(
          object, predicted_path.path, candidate.stop_line, max_decel_params)) {
      continue;
    }

    const auto arc_length = arcLengthToStopLine(predicted_path.path, candidate.stop_line);
    if (arc_length && *arc_length < nearest_arc_length) {
      nearest_arc_length = *arc_length;
      selection.stop_line = candidate.stop_line;
    }
  }
  return selection;
}

std::vector<PredictedPath> addTrafficSignalStopHypotheses(
  const ObjectPrediction & prediction,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & traffic_signal_id_map,
  const std::vector<SignalizedStopLine> & signalized_stop_lines,
  const path_cut::MaxDecelerationParams & max_decel_params, StopHypothesisDebug & debug)
{
  const TrackedObject & object = prediction.object;
  const std::vector<PredictedPath> & predicted_paths = prediction.predicted_paths;

  if (predicted_paths.empty()) {
    return predicted_paths;
  }

  debug.counter.vehicles++;

  std::vector<PredictedPath> result = predicted_paths;

  for (size_t i = 0; i < predicted_paths.size(); ++i) {
    PredictedPath predicted_path = predicted_paths.at(i);

    if (predicted_path.path.size() < 2) {
      continue;
    }

    const auto selection = select_nearest_stop_line_requiring_stop(
      predicted_path, object, traffic_signal_id_map, signalized_stop_lines, max_decel_params);
    debug.counter.signal_stop += selection.signal_requires_stop ? 1 : 0;
    debug.counter.stopline_found += selection.stop_line_ahead ? 1 : 0;

    if (!selection.stop_line) {
      continue;
    }

    clipPathAtStopLine(predicted_path, *selection.stop_line);

    if (predicted_path.path.size() < 2) {
      continue;
    }

    result.at(i) = predicted_path;

    debug.stop_hypothesis_path_indices[autoware_utils::to_hex_string(object.object_id)].insert(i);
    debug.stop_lines.push_back(*selection.stop_line);
    debug.counter.stop_hypothesis_added++;
  }

  return result;
}

}  // namespace

void TrafficSignalStopPredictor::setLaneletMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  lanelet_map_ptr_ = std::move(lanelet_map_ptr);
  if (!lanelet_map_ptr_) {
    signalized_stop_lines_.clear();
    return;
  }
  signalized_stop_lines_ = collect_signalized_stop_lines(*lanelet_map_ptr_);
}

void TrafficSignalStopPredictor::setTrafficSignal(
  const TrafficLightGroupArray & traffic_signals, const rclcpp::Time & now)
{
  traffic_signal_id_map_.clear();
  for (const auto & group : traffic_signals.traffic_light_groups) {
    traffic_signal_id_map_[group.traffic_light_group_id] = group;
  }

  stabilized_traffic_signal_id_map_ = stabilizeTrafficSignalMap(
    traffic_signal_id_map_, signal_stabilize_state_, now, params_.stop_time_hysteresis,
    params_.go_time_hysteresis, params_.signal_retention_timeout);
  latest_traffic_signal_time_ = now;
}

void TrafficSignalStopPredictor::clearFrameDebug()
{
  debug_.stop_lines.clear();
  debug_.stop_hypothesis_path_indices.clear();
  debug_.used_signal_colors.clear();
}

std::vector<PredictedPath> TrafficSignalStopPredictor::addStopHypotheses(
  const ObjectPrediction & prediction, const rclcpp::Time & now)
{
  if (signalized_stop_lines_.empty()) {
    return prediction.predicted_paths;
  }

  const bool signal_observation_stale =
    signal_observation_timeout_ > 0.0 &&
    (!latest_traffic_signal_time_ ||
     (now - *latest_traffic_signal_time_).seconds() > signal_observation_timeout_);
  if (signal_observation_stale) {
    signal_stabilize_state_.clear();
    stabilized_traffic_signal_id_map_.clear();
    debug_.used_signal_colors.clear();
    return prediction.predicted_paths;
  }

  debug::populateUsedSignalColors(stabilized_traffic_signal_id_map_, debug_.used_signal_colors);

  return addTrafficSignalStopHypotheses(
    prediction, stabilized_traffic_signal_id_map_, signalized_stop_lines_, max_decel_params_,
    debug_);
}

}  // namespace autoware::map_based_prediction::priority_predictor
