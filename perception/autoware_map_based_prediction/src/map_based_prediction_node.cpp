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

#include "map_based_prediction/map_based_prediction_node.hpp"

#include "map_based_prediction/data_structure.hpp"
#include "map_based_prediction/utils.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils/autoware_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/constants.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <ratio>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

namespace
{
// Same set of (color, shape) elements, ignoring confidence / ordering.
bool sameSignalState(const TrafficLightGroup & a, const TrafficLightGroup & b)
{
  if (a.elements.size() != b.elements.size()) {
    return false;
  }
  const auto key = [](const TrafficLightGroup & group) {
    std::vector<std::pair<uint8_t, uint8_t>> pairs;
    pairs.reserve(group.elements.size());
    for (const auto & element : group.elements) {
      pairs.emplace_back(element.color, element.shape);
    }
    std::sort(pairs.begin(), pairs.end());
    return pairs;
  };
  return key(a) == key(b);
}

// Truncate the path at its first crossing with the stop line; if it never crosses,
// bridge the last point to the line (bounded) so the path ends exactly on it.
void clipPathAtStopLine(PredictedPath & path, const lanelet::ConstLineString3d & stop_line)
{
  if (path.path.size() < 2 || stop_line.size() < 2) {
    return;
  }

  // The u (lateral) bound is generous so a laterally-offset path still clips at
  // the longitudinal stop position.
  for (size_t i = 1; i < path.path.size(); ++i) {
    const auto & a = path.path[i - 1].position;
    const auto & b = path.path[i].position;
    const double rx = b.x - a.x;
    const double ry = b.y - a.y;
    for (size_t j = 1; j < stop_line.size(); ++j) {
      const double cx = stop_line[j - 1].x();
      const double cy = stop_line[j - 1].y();
      const double sx = stop_line[j].x() - cx;
      const double sy = stop_line[j].y() - cy;
      const double denom = rx * sy - ry * sx;
      if (std::abs(denom) < 1e-9) {
        continue;  // parallel segments
      }
      const double t = ((cx - a.x) * sy - (cy - a.y) * sx) / denom;  // along path segment
      const double u = ((cx - a.x) * ry - (cy - a.y) * rx) / denom;  // along stop-line segment
      if (t >= -1e-6 && t <= 1.0 + 1e-6 && u >= -1.0 && u <= 2.0) {
        auto crossing = path.path[i];  // keep orientation / time fields
        crossing.position.x = a.x + std::clamp(t, 0.0, 1.0) * rx;
        crossing.position.y = a.y + std::clamp(t, 0.0, 1.0) * ry;
        crossing.position.z = a.z;
        path.path.resize(i);
        path.path.push_back(crossing);
        return;
      }
    }
  }

  constexpr double max_bridge = 3.0;  // [m]
  const auto & last = path.path.back().position;
  double best_d_sq = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Point target;
  for (size_t j = 1; j < stop_line.size(); ++j) {
    const double cx = stop_line[j - 1].x();
    const double cy = stop_line[j - 1].y();
    const double dx = stop_line[j].x() - cx;
    const double dy = stop_line[j].y() - cy;
    const double len_sq = dx * dx + dy * dy;
    const double u = len_sq > 1e-12
                       ? std::clamp(((last.x - cx) * dx + (last.y - cy) * dy) / len_sq, 0.0, 1.0)
                       : 0.0;
    const double qx = cx + u * dx;
    const double qy = cy + u * dy;
    const double d_sq = (last.x - qx) * (last.x - qx) + (last.y - qy) * (last.y - qy);
    if (d_sq < best_d_sq) {
      best_d_sq = d_sq;
      target.x = qx;
      target.y = qy;
      target.z = last.z;
    }
  }
  if (std::isfinite(best_d_sq) && std::sqrt(best_d_sq) <= max_bridge) {
    auto pose = path.path.back();
    pose.position = target;
    path.path.push_back(pose);
    return;
  }
}

/**
 * @brief First order Low pass filtering
 *
 * @param prev_y previous filtered value
 * @param prev_x previous input value
 * @param x current input value
 * @param cutoff_freq  cutoff frequency in Hz not rad/s (1/s)
 * @param sampling_time  sampling time of discrete system (s)
 *
 * @return double current filtered value
 */
double FirstOrderLowpassFilter(
  const double prev_y, const double prev_x, const double x, const double sampling_time = 0.1,
  const double cutoff_freq = 0.1)
{
  // Eq:  yn = a yn-1 + b (xn-1 + xn)
  const double wt = 2.0 * M_PI * cutoff_freq * sampling_time;
  const double a = (2.0 - wt) / (2.0 + wt);
  const double b = wt / (2.0 + wt);

  return a * prev_y + b * (prev_x + x);
}

/**
 * @brief calc lateral offset from pose to linestring
 *
 * @param boundary_line 2d line strings
 * @param search_pose search point
 * @return double
 */
double calcAbsLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = autoware_utils::create_point(x, y, 0.0);
  }

  return std::fabs(autoware::motion_utils::calcLateralOffset(boundary_path, search_pose.position));
}

/**
 * @brief init lateral kinematics struct
 *
 * @param lanelet closest lanelet
 * @param pose search pose
 * @return lateral kinematics data struct
 */
LateralKinematicsToLanelet initLateralKinematics(
  const lanelet::ConstLanelet & lanelet, geometry_msgs::msg::Pose pose)
{
  LateralKinematicsToLanelet lateral_kinematics;

  const lanelet::ConstLineString2d left_bound = lanelet.leftBound2d();
  const lanelet::ConstLineString2d right_bound = lanelet.rightBound2d();
  const double left_dist = calcAbsLateralOffset(left_bound, pose);
  const double right_dist = calcAbsLateralOffset(right_bound, pose);

  // calc boundary distance
  lateral_kinematics.dist_from_left_boundary = left_dist;
  lateral_kinematics.dist_from_right_boundary = right_dist;
  // velocities are not init in the first step
  lateral_kinematics.left_lateral_velocity = 0;
  lateral_kinematics.right_lateral_velocity = 0;
  lateral_kinematics.filtered_left_lateral_velocity = 0;
  lateral_kinematics.filtered_right_lateral_velocity = 0;
  return lateral_kinematics;
}

/**
 * @brief calc lateral velocity and filtered velocity of object in a lanelet
 *
 * @param prev_lateral_kinematics previous lateral lanelet kinematics
 * @param current_lateral_kinematics current lateral lanelet kinematics
 * @param dt sampling time [s]
 */
void calcLateralKinematics(
  const LateralKinematicsToLanelet & prev_lateral_kinematics,
  LateralKinematicsToLanelet & current_lateral_kinematics, const double dt, const double cutoff)
{
  // calc velocity via backward difference
  current_lateral_kinematics.left_lateral_velocity =
    (current_lateral_kinematics.dist_from_left_boundary -
     prev_lateral_kinematics.dist_from_left_boundary) /
    dt;
  current_lateral_kinematics.right_lateral_velocity =
    (current_lateral_kinematics.dist_from_right_boundary -
     prev_lateral_kinematics.dist_from_right_boundary) /
    dt;

  // low pass filtering left velocity: default cut_off is 0.6 Hz
  current_lateral_kinematics.filtered_left_lateral_velocity = FirstOrderLowpassFilter(
    prev_lateral_kinematics.filtered_left_lateral_velocity,
    prev_lateral_kinematics.left_lateral_velocity, current_lateral_kinematics.left_lateral_velocity,
    dt, cutoff);
  current_lateral_kinematics.filtered_right_lateral_velocity = FirstOrderLowpassFilter(
    prev_lateral_kinematics.filtered_right_lateral_velocity,
    prev_lateral_kinematics.right_lateral_velocity,
    current_lateral_kinematics.right_lateral_velocity, dt, cutoff);
}

/**
 * @brief look for matching lanelet between current/previous object state and calculate velocity
 *
 * @param prev_obj previous RoadUser
 * @param current_obj current RoadUser to be updated
 * @param routing_graph_ptr_ routing graph pointer
 */
void updateLateralKinematicsVector(
  const RoadUser & prev_obj, RoadUser & current_obj,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr_, const double lowpass_cutoff)
{
  const double dt = (current_obj.header.stamp.sec - prev_obj.header.stamp.sec) +
                    (current_obj.header.stamp.nanosec - prev_obj.header.stamp.nanosec) * 1e-9;
  if (dt < 1e-6) {
    return;  // do not update
  }

  // look for matching lanelet between current and previous kinematics
  for (auto & current_set : current_obj.lateral_kinematics_set) {
    const auto & current_lane = current_set.first;
    auto & current_lateral_kinematics = current_set.second;

    // 1. has same lanelet
    if (prev_obj.lateral_kinematics_set.count(current_lane) != 0) {
      const auto & prev_lateral_kinematics = prev_obj.lateral_kinematics_set.at(current_lane);
      calcLateralKinematics(
        prev_lateral_kinematics, current_lateral_kinematics, dt, lowpass_cutoff);
      break;
    }
    // 2. successive lanelet
    for (auto & prev_set : prev_obj.lateral_kinematics_set) {
      const auto & prev_lane = prev_set.first;
      const auto & prev_lateral_kinematics = prev_set.second;
      const bool successive_lanelet =
        routing_graph_ptr_->routingRelation(prev_lane, current_lane) ==
        lanelet::routing::RelationType::Successor;
      if (successive_lanelet) {  // lanelet can be connected
        calcLateralKinematics(
          prev_lateral_kinematics, current_lateral_kinematics, dt,
          lowpass_cutoff);  // calc velocity
        break;
      }
    }
  }
}

/**
 * @brief Get the Right LineSharing Lanelets object
 *
 * @param current_lanelet
 * @param lanelet_map_ptr
 * @return lanelet::ConstLanelets
 */
lanelet::ConstLanelets getRightLineSharingLanelets(
  const lanelet::ConstLanelet & current_lanelet, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelets
    output_lanelets;  // create an empty container of type lanelet::ConstLanelets

  // step1: look for lane sharing current right bound
  lanelet::Lanelets right_lane_candidates =
    lanelet_map_ptr->laneletLayer.findUsages(current_lanelet.rightBound());
  for (auto & candidate : right_lane_candidates) {
    // exclude self lanelet
    if (candidate == current_lanelet) continue;
    // if candidate has linestring as left bound, assign it to output
    if (candidate.leftBound() == current_lanelet.rightBound()) {
      output_lanelets.push_back(candidate);
    }
  }
  return output_lanelets;  // return empty
}

/**
 * @brief Get the Left LineSharing Lanelets object
 *
 * @param current_lanelet
 * @param lanelet_map_ptr
 * @return lanelet::ConstLanelets
 */
lanelet::ConstLanelets getLeftLineSharingLanelets(
  const lanelet::ConstLanelet & current_lanelet, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelets
    output_lanelets;  // create an empty container of type lanelet::ConstLanelets

  // step1: look for lane sharing current left bound
  lanelet::Lanelets left_lane_candidates =
    lanelet_map_ptr->laneletLayer.findUsages(current_lanelet.leftBound());
  for (auto & candidate : left_lane_candidates) {
    // exclude self lanelet
    if (candidate == current_lanelet) continue;
    // if candidate has linestring as right bound, assign it to output
    if (candidate.rightBound() == current_lanelet.leftBound()) {
      output_lanelets.push_back(candidate);
    }
  }
  return output_lanelets;  // return empty
}

/**
 * @brief Check if the lanelet is isolated in routing graph
 * @param current_lanelet
 * @param lanelet_map_ptr
 */
bool isIsolatedLanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::routing::RoutingGraphPtr & graph)
{
  const auto & following_lanelets = graph->following(lanelet);
  const auto & left_lanelets = graph->lefts(lanelet);
  const auto & right_lanelets = graph->rights(lanelet);
  return left_lanelets.empty() && right_lanelets.empty() && following_lanelets.empty();
}

/**
 * @brief Get the Possible Paths For Isolated Lanelet object
 * @param lanelet
 * @return lanelet::routing::LaneletPaths
 */
lanelet::routing::LaneletPaths getPossiblePathsForIsolatedLanelet(
  const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstLanelets possible_lanelets;
  possible_lanelets.push_back(lanelet);
  lanelet::routing::LaneletPaths possible_paths;
  // need to initialize path with constant lanelets
  lanelet::routing::LaneletPath possible_path(possible_lanelets);
  possible_paths.push_back(possible_path);
  return possible_paths;
}

/**
 * @brief validate isolated lanelet length has enough length for prediction
 * @param lanelet
 * @param object: object information for calc length threshold
 * @param prediction_time: time horizon[s] for calc length threshold
 * @return bool
 */
bool validateIsolatedLaneletLength(
  const lanelet::ConstLanelet & lanelet, const TrackedObject & object, const double prediction_time)
{
  // get closest center line point to object
  const auto & center_line = lanelet.centerline2d();
  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const lanelet::BasicPoint2d obj_point(obj_pos.x, obj_pos.y);
  // get end point of the center line
  const auto & end_point = center_line.back();
  // calc approx distance between closest point and end point
  const double approx_distance = lanelet::geometry::distance2d(obj_point, end_point);
  // calc min length for prediction
  const double abs_speed = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  const double min_length = abs_speed * prediction_time;
  return approx_distance > min_length;
}

lanelet::ConstLanelets getLanelets(const map_based_prediction::LaneletsData & data)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & lanelet_data : data) {
    lanelets.push_back(lanelet_data.lanelet);
  }

  return lanelets;
}

void replaceObjectYawWithLaneletsYaw(
  const LaneletsData & current_lanelets, TrackedObject & transformed_object)
{
  // return if no lanelet is found
  if (current_lanelets.empty()) return;
  auto & pose_with_cov = transformed_object.kinematics.pose_with_covariance;
  // for each lanelet, calc lanelet angle and calculate mean angle
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto & current_lanelet : current_lanelets) {
    const auto lanelet_angle = autoware::experimental::lanelet2_utils::get_lanelet_angle(
      current_lanelet.lanelet,
      autoware::experimental::lanelet2_utils::from_ros(pose_with_cov.pose).basicPoint());
    sum_x += std::cos(lanelet_angle);
    sum_y += std::sin(lanelet_angle);
  }
  const double mean_yaw_angle = std::atan2(sum_y, sum_x);
  double roll, pitch, yaw;
  tf2::Quaternion original_quaternion;
  tf2::fromMsg(pose_with_cov.pose.orientation, original_quaternion);
  tf2::Matrix3x3(original_quaternion).getRPY(roll, pitch, yaw);
  tf2::Quaternion filtered_quaternion;
  filtered_quaternion.setRPY(roll, pitch, mean_yaw_angle);
  pose_with_cov.pose.orientation = tf2::toMsg(filtered_quaternion);
}

}  // namespace

MapBasedPredictionNode::MapBasedPredictionNode(const rclcpp::NodeOptions & node_options)
: Node("map_based_prediction", node_options)
{
  prediction_time_horizon_.vehicle = declare_parameter<double>("prediction_time_horizon.vehicle");
  prediction_time_horizon_.pedestrian =
    declare_parameter<double>("prediction_time_horizon.pedestrian");
  prediction_time_horizon_.unknown = declare_parameter<double>("prediction_time_horizon.unknown");
  lateral_control_time_horizon_ =
    declare_parameter<double>("lateral_control_time_horizon");  // [s] for lateral control point
  prediction_sampling_time_interval_ = declare_parameter<double>("prediction_sampling_delta_time");
  min_velocity_for_map_based_prediction_ =
    declare_parameter<double>("min_velocity_for_map_based_prediction");

  dist_threshold_for_searching_lanelet_ =
    declare_parameter<double>("dist_threshold_for_searching_lanelet");
  delta_yaw_threshold_for_searching_lanelet_ =
    declare_parameter<double>("delta_yaw_threshold_for_searching_lanelet");
  sigma_lateral_offset_ = declare_parameter<double>("sigma_lateral_offset");
  sigma_yaw_angle_deg_ = declare_parameter<double>("sigma_yaw_angle_deg");
  object_buffer_time_length_ = declare_parameter<double>("object_buffer_time_length");
  history_time_length_ = declare_parameter<double>("history_time_length");

  check_lateral_acceleration_constraints_ =
    declare_parameter<bool>("check_lateral_acceleration_constraints");
  max_lateral_accel_ = declare_parameter<double>("max_lateral_accel");
  min_acceleration_before_curve_ = declare_parameter<double>("min_acceleration_before_curve");

  {  // lane change detection
    lane_change_detection_method_ = declare_parameter<std::string>("lane_change_detection.method");

    // lane change detection by time_to_change_lane
    dist_threshold_to_bound_ = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.dist_threshold_for_lane_change_detection");  // 1m
    time_threshold_to_bound_ = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.time_threshold_for_lane_change_detection");
    cutoff_freq_of_velocity_lpf_ = declare_parameter<double>(
      "lane_change_detection.time_to_change_lane.cutoff_freq_of_velocity_for_lane_change_"
      "detection");

    // lane change detection by lat_diff_distance
    dist_ratio_threshold_to_left_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_left_bound");
    dist_ratio_threshold_to_right_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.dist_ratio_threshold_to_right_bound");
    diff_dist_threshold_to_left_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_left_bound");
    diff_dist_threshold_to_right_bound_ = declare_parameter<double>(
      "lane_change_detection.lat_diff_distance.diff_dist_threshold_to_right_bound");

    num_continuous_state_transition_ =
      declare_parameter<int>("lane_change_detection.num_continuous_state_transition");

    consider_only_routable_neighbours_ =
      declare_parameter<bool>("lane_change_detection.consider_only_routable_neighbours");
  }
  reference_path_resolution_ = declare_parameter<double>("reference_path_resolution");
  /* prediction path will disabled when the estimated path length exceeds lanelet length. This
   * parameter control the estimated path length = vx * th * (rate)  */
  prediction_time_horizon_rate_for_validate_lane_length_ =
    declare_parameter<double>("prediction_time_horizon_rate_for_validate_shoulder_lane_length");

  use_vehicle_acceleration_ = declare_parameter<bool>("use_vehicle_acceleration");
  speed_limit_multiplier_ = declare_parameter<double>("speed_limit_multiplier");
  acceleration_exponential_half_life_ =
    declare_parameter<double>("acceleration_exponential_half_life");

  // Traffic-signal-aware stop/creep prediction parameters.
  use_priority_prediction_ = declare_parameter<bool>("priority_prediction.enable");
  priority_calibration_params_.use_signal_priority =
    declare_parameter<bool>("priority_prediction.use_signal_priority");
  priority_calibration_params_.stop_probability_boost =
    declare_parameter<double>("priority_prediction.stop_probability_boost");
  priority_calibration_params_.creep_probability_boost =
    declare_parameter<double>("priority_prediction.creep_probability_boost");
  priority_calibration_params_.go_probability_decay_on_yield =
    declare_parameter<double>("priority_prediction.go_probability_decay_on_yield");
  priority_stop_deceleration_ = declare_parameter<double>("priority_prediction.stop_deceleration");
  priority_yellow_params_.lamp_period =
    declare_parameter<double>("priority_prediction.yellow.lamp_period");
  priority_yellow_params_.max_stop_acceleration =
    declare_parameter<double>("priority_prediction.yellow.max_stop_acceleration");
  priority_yellow_params_.max_stop_jerk =
    declare_parameter<double>("priority_prediction.yellow.max_stop_jerk");
  priority_yellow_params_.delay_response_time =
    declare_parameter<double>("priority_prediction.yellow.delay_response_time");
  priority_yellow_params_.stop_velocity =
    declare_parameter<double>("priority_prediction.yellow.stop_velocity");
  priority_hysteresis_time_ = declare_parameter<double>("priority_prediction.hysteresis_time");
  priority_use_hysteresis_ = declare_parameter<bool>("priority_prediction.use_hysteresis");
  priority_retain_last_valid_signal_ =
    declare_parameter<bool>("priority_prediction.retain_last_valid_signal");
  priority_debug_viz_ = declare_parameter<bool>("priority_prediction.debug_visualization");
  priority_suppress_go_on_conservative_ =
    declare_parameter<bool>("priority_prediction.suppress_go_on_conservative");
  priority_extend_stop_path_to_stopline_ =
    declare_parameter<bool>("priority_prediction.extend_stop_path_to_stopline");
  priority_use_lead_vehicle_ = declare_parameter<bool>("priority_prediction.use_lead_vehicle");
  priority_follow_lateral_threshold_ =
    declare_parameter<double>("priority_prediction.follow_lateral_threshold");
  priority_follow_gap_margin_ = declare_parameter<double>("priority_prediction.follow_gap_margin");

  // initialize VRU predictor
  predictor_vru_ = std::make_unique<PredictorVru>(*this);

  // VRU parameters
  remember_lost_crosswalk_users_ =
    declare_parameter<bool>("use_crosswalk_user_history.remember_lost_users");
  {
    bool match_lost_and_appeared_crosswalk_users =
      declare_parameter<bool>("use_crosswalk_user_history.match_lost_and_appeared_users");
    double min_crosswalk_user_velocity = declare_parameter<double>("min_crosswalk_user_velocity");
    double max_crosswalk_user_delta_yaw_threshold_for_lanelet =
      declare_parameter<double>("max_crosswalk_user_delta_yaw_threshold_for_lanelet");
    double max_crosswalk_user_on_road_distance =
      declare_parameter<double>("max_crosswalk_user_on_road_distance");
    bool use_crosswalk_signal =
      declare_parameter<bool>("crosswalk_with_signal.use_crosswalk_signal");
    double threshold_velocity_assumed_as_stopping =
      declare_parameter<double>("crosswalk_with_signal.threshold_velocity_assumed_as_stopping");
    double crossing_intention_duration = declare_parameter<double>("crossing_intention_duration");
    double no_crossing_intention_duration =
      declare_parameter<double>("no_crossing_intention_duration");
    std::vector<double> distance_set_for_no_intention_to_walk =
      declare_parameter<std::vector<double>>(
        "crosswalk_with_signal.distance_set_for_no_intention_to_walk");
    std::vector<double> timeout_set_for_no_intention_to_walk =
      declare_parameter<std::vector<double>>(
        "crosswalk_with_signal.timeout_set_for_no_intention_to_walk");
    predictor_vru_->setParameters(
      match_lost_and_appeared_crosswalk_users, min_crosswalk_user_velocity,
      max_crosswalk_user_delta_yaw_threshold_for_lanelet, max_crosswalk_user_on_road_distance,
      use_crosswalk_signal, threshold_velocity_assumed_as_stopping,
      distance_set_for_no_intention_to_walk, timeout_set_for_no_intention_to_walk,
      prediction_sampling_time_interval_, prediction_time_horizon_.pedestrian,
      crossing_intention_duration, no_crossing_intention_duration);
  }

  // debug parameter
  bool use_time_publisher = declare_parameter<bool>("publish_processing_time");
  bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
  bool use_debug_marker = declare_parameter<bool>("publish_debug_markers");

  // initialize path generator
  path_generator_ = std::make_shared<PathGenerator>(prediction_sampling_time_interval_);

  path_generator_->setUseVehicleAcceleration(use_vehicle_acceleration_);
  path_generator_->setAccelerationHalfLife(acceleration_exponential_half_life_);

  // subscribers
  sub_objects_ = this->create_subscription<TrackedObjects>(
    "~/input/objects", 1,
    std::bind(&MapBasedPredictionNode::objectsCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<LaneletMapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedPredictionNode::mapCallback, this, std::placeholders::_1));

  // publishers
  pub_objects_ = this->create_publisher<PredictedObjects>("~/output/objects", rclcpp::QoS{1});

  // stopwatch
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  {  // diagnostics
    diagnostics_interface_ptr_ =
      std::make_unique<autoware_utils::DiagnosticsInterface>(this, "map_based_prediction");

    // [s] -> [ms]
    processing_time_tolerance_ms_ = declare_parameter<double>("processing_time_tolerance") * 1e3;
    processing_time_consecutive_excess_tolerance_ms_ =
      declare_parameter<double>("processing_time_consecutive_excess_tolerance") * 1e3;
  }

  // debug publishers
  if (use_time_publisher) {
    processing_time_publisher_ =
      std::make_unique<autoware_utils::DebugPublisher>(this, "map_based_prediction");
    published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
  }

  // debug time keeper
  if (use_time_keeper) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware_utils::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    auto time_keeper = autoware_utils::TimeKeeper(detailed_processing_time_publisher_);
    time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(time_keeper);
    path_generator_->setTimeKeeper(time_keeper_);
    predictor_vru_->setTimeKeeper(time_keeper_);
  }

  // debug marker
  if (use_debug_marker) {
    pub_debug_markers_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("maneuver", rclcpp::QoS{1});
  }

  pub_priority_object_markers_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/priority_objects", 1);
  pub_stabilized_signals_ =
    this->create_publisher<TrafficLightGroupArray>("~/debug/stabilized_traffic_signals", 1);
  // dynamic reconfigure
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MapBasedPredictionNode::onParam, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MapBasedPredictionNode::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  update_param(parameters, "max_lateral_accel", max_lateral_accel_);
  update_param(parameters, "min_acceleration_before_curve", min_acceleration_before_curve_);
  update_param(
    parameters, "check_lateral_acceleration_constraints", check_lateral_acceleration_constraints_);
  update_param(parameters, "use_vehicle_acceleration", use_vehicle_acceleration_);
  update_param(parameters, "speed_limit_multiplier", speed_limit_multiplier_);
  update_param(
    parameters, "acceleration_exponential_half_life", acceleration_exponential_half_life_);

  path_generator_->setUseVehicleAcceleration(use_vehicle_acceleration_);
  path_generator_->setAccelerationHalfLife(acceleration_exponential_half_life_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void MapBasedPredictionNode::updateDiagnostics(
  const rclcpp::Time & timestamp, double processing_time_ms)
{
  diagnostics_interface_ptr_->clear();
  diagnostics_interface_ptr_->add_key_value("timestamp", timestamp.seconds());
  diagnostics_interface_ptr_->add_key_value("processing_time_ms", processing_time_ms);
  // check processing time is in time
  bool is_processing_in_time = processing_time_ms <= processing_time_tolerance_ms_;
  diagnostics_interface_ptr_->add_key_value("is_processing_in_time", is_processing_in_time);
  if (!is_processing_in_time) {
    // publish warning if the current processing time exceeded
    std::ostringstream oss;
    oss << "Processing time exceeded: " << processing_time_tolerance_ms_ << "[ms] < "
        << processing_time_ms << "[ms]";
    diagnostics_interface_ptr_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, oss.str());
  }

  if (is_processing_in_time || !last_in_time_processing_timestamp_) {
    last_in_time_processing_timestamp_ = timestamp;
  }

  // calculate consecutive excess duration
  const double consecutive_excess_duration_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds(
        (timestamp - last_in_time_processing_timestamp_.value()).nanoseconds()))
      .count();

  bool is_consecutive_excess_duration_ok =
    consecutive_excess_duration_ms < processing_time_consecutive_excess_tolerance_ms_;
  diagnostics_interface_ptr_->add_key_value(
    "consecutive_excess_duration_ms", consecutive_excess_duration_ms);
  diagnostics_interface_ptr_->add_key_value(
    "is_consecutive_excess_duration_ok", is_consecutive_excess_duration_ok);
  if (!is_consecutive_excess_duration_ok) {
    // publish error if the processing time exceeded in a long term
    std::ostringstream oss;
    oss << "Processing time exceeded consecutively in a long term: "
        << processing_time_consecutive_excess_tolerance_ms_ << "[ms] < "
        << consecutive_excess_duration_ms << "[ms]";
    diagnostics_interface_ptr_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, oss.str());
  }

  diagnostics_interface_ptr_->publish(timestamp);
}

void MapBasedPredictionNode::mapCallback(const LaneletMapBin::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "[Map Based Prediction]: Start loading lanelet");
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));

  auto routing_graph_and_traffic_rules =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr_);

  routing_graph_ptr_ =
    autoware::experimental::lanelet2_utils::remove_const(routing_graph_and_traffic_rules.first);
  traffic_rules_ptr_ = routing_graph_and_traffic_rules.second;

  lru_cache_of_convert_path_type_.clear();  // clear cache

  RCLCPP_INFO(get_logger(), "[Map Based Prediction]: Map is loaded.");

  predictor_vru_->setLaneletMap(lanelet_map_ptr_);
}

void MapBasedPredictionNode::trafficSignalsCallback(
  const TrafficLightGroupArray::ConstSharedPtr msg)
{
  // load traffic signals to the predictor
  predictor_vru_->setTrafficSignal(*msg);

  // Optionally retain the last non-UNKNOWN observation per group so a transient
  // perception dropout does not erase a red/green state.
  if (!priority_retain_last_valid_signal_) {
    raw_signal_id_map_.clear();
  }
  for (const auto & group : msg->traffic_light_groups) {
    const bool has_valid_color = std::any_of(
      group.elements.begin(), group.elements.end(),
      [](const auto & element) { return element.color != TrafficLightElement::UNKNOWN; });
    if (
      !priority_retain_last_valid_signal_ || has_valid_color ||
      raw_signal_id_map_.find(group.traffic_light_group_id) == raw_signal_id_map_.end()) {
      raw_signal_id_map_[group.traffic_light_group_id] = group;
    }
  }

  // Debounce: a new observation must persist for hysteresis_time before it
  // replaces the held state, so chattering recognition does not flip the decision.
  const double now = this->now().seconds();
  traffic_signal_id_map_.clear();
  for (const auto & [group_id, observed] : raw_signal_id_map_) {
    auto & state = signal_hysteresis_[group_id];
    if (!state.initialized) {
      state.held = observed;
      state.candidate = observed;
      state.candidate_since = now;
      state.initialized = true;
    } else if (!priority_use_hysteresis_) {
      state.held = observed;
    } else if (sameSignalState(observed, state.held)) {
      state.candidate = state.held;
      state.candidate_since = now;
    } else if (sameSignalState(observed, state.candidate)) {
      if (now - state.candidate_since >= priority_hysteresis_time_) {
        state.held = observed;
      }
    } else {
      state.candidate = observed;
      state.candidate_since = now;
    }
    traffic_signal_id_map_[group_id] = state.held;
  }

  TrafficLightGroupArray stabilized;
  stabilized.stamp = msg->stamp;
  stabilized.traffic_light_groups.reserve(traffic_signal_id_map_.size());
  for (const auto & [group_id, group] : traffic_signal_id_map_) {
    stabilized.traffic_light_groups.push_back(group);
  }
  pub_stabilized_signals_->publish(stabilized);
}

void MapBasedPredictionNode::objectsCallback(const TrackedObjects::ConstSharedPtr in_objects)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  stop_watch_ptr_->toc("processing_time", true);

  // take traffic_signal
  {
    const auto msg = sub_traffic_signals_.take_data();
    if (msg) {
      trafficSignalsCallback(msg);
    }
  }

  // Guard for map pointer and frame transformation
  if (!lanelet_map_ptr_) {
    return;
  }

  // get world to map transform
  geometry_msgs::msg::TransformStamped::ConstSharedPtr world2map_transform;
  bool is_object_not_in_map_frame = in_objects->header.frame_id != "map";
  if (is_object_not_in_map_frame) {
    world2map_transform = transform_listener_.get_transform(
      "map",                        // target
      in_objects->header.frame_id,  // src
      in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
    if (!world2map_transform) return;
  }

  // Get objects detected time
  const double objects_detected_time = rclcpp::Time(in_objects->header.stamp).seconds();

  // Remove old objects information in object history
  // road users
  const auto removed_object_ids = utils::removeOldObjectsHistory(
    objects_detected_time, object_buffer_time_length_, road_users_history_);
  // crosswalk users
  predictor_vru_->removeOldKnownMatches(objects_detected_time, object_buffer_time_length_);

  // result output
  PredictedObjects output;
  output.header = in_objects->header;
  output.header.frame_id = "map";

  // result debug
  visualization_msgs::msg::MarkerArray debug_markers;

  // get current crosswalk users for later prediction
  predictor_vru_->loadCurrentCrosswalkUsers(*in_objects);

  priority_object_markers_.markers.clear();
  conservative_path_is_creep_.clear();
  priority_debug_stop_lines_.clear();
  if (priority_debug_viz_) {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = this->now();
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    priority_object_markers_.markers.push_back(clear_marker);
  }

  lead_objects_.clear();
  if (use_priority_prediction_ && priority_use_lead_vehicle_) {
    lead_objects_.reserve(in_objects->objects.size());
    for (const auto & object : in_objects->objects) {
      geometry_msgs::msg::Pose pose_map = object.kinematics.pose_with_covariance.pose;
      if (is_object_not_in_map_frame) {
        geometry_msgs::msg::PoseStamped pose_in_map;
        geometry_msgs::msg::PoseStamped pose_orig;
        pose_orig.pose = pose_map;
        tf2::doTransform(pose_orig, pose_in_map, *world2map_transform);
        pose_map = pose_in_map.pose;
      }
      lead_objects_.push_back(
        {autoware_utils::to_hex_string(object.object_id), pose_map, object.shape.dimensions.x});
    }
  }

  // for each object
  for (const auto & object : in_objects->objects) {
    TrackedObject transformed_object = object;

    // transform object frame if it's based on map frame
    if (is_object_not_in_map_frame) {
      geometry_msgs::msg::PoseStamped pose_in_map;
      geometry_msgs::msg::PoseStamped pose_orig;
      pose_orig.pose = object.kinematics.pose_with_covariance.pose;
      tf2::doTransform(pose_orig, pose_in_map, *world2map_transform);
      transformed_object.kinematics.pose_with_covariance.pose = pose_in_map.pose;
    }

    // get the maximum probability label from the classification array
    const auto & label_ =
      autoware::object_recognition_utils::getHighestProbLabel(transformed_object.classification);
    // overwrite the label for VRU in specific cases
    const auto label = utils::changeVRULabelForPrediction(label_, object, lanelet_map_ptr_);

    switch (label) {
      case ObjectClassification::PEDESTRIAN:
      case ObjectClassification::BICYCLE: {
        // Run pedestrian/bicycle prediction
        const auto predicted_vru =
          getPredictionForNonVehicleObject(output.header, transformed_object);
        output.objects.emplace_back(predicted_vru);
        break;
      }
      case ObjectClassification::CAR:
      case ObjectClassification::BUS:
      case ObjectClassification::TRAILER:
      case ObjectClassification::MOTORCYCLE:
      case ObjectClassification::TRUCK: {
        const auto predicted_object_opt = getPredictionForVehicleObject(
          output.header, transformed_object, objects_detected_time, debug_markers);
        if (predicted_object_opt) {
          output.objects.push_back(predicted_object_opt.value());
        }
        break;
      }
      default: {
        auto predicted_unknown_object = utils::convertToPredictedObject(transformed_object);
        PredictedPath predicted_path = path_generator_->generatePathForNonVehicleObject(
          transformed_object, prediction_time_horizon_.unknown);
        predicted_path.confidence = 1.0;

        predicted_unknown_object.kinematics.predicted_paths.push_back(predicted_path);
        output.objects.push_back(predicted_unknown_object);
        break;
      }
    }
  }

  // process lost crosswalk users to tackle unstable detection
  if (remember_lost_crosswalk_users_) {
    PredictedObjects retrieved_objects = predictor_vru_->retrieveUndetectedObjects();
    output.objects.insert(
      output.objects.end(), retrieved_objects.objects.begin(), retrieved_objects.objects.end());
  }

  // Publish Results
  publish(output, debug_markers);

  if (priority_debug_viz_) {
    publishPriorityDebugMarkers(output, in_objects);
  }

  // Processing time
  const auto processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  const auto cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);

  // Diagnostics
  updateDiagnostics(output.header.stamp, processing_time_ms);

  // Publish Processing Time
  if (processing_time_publisher_) {
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

void MapBasedPredictionNode::publish(
  const PredictedObjects & output, const visualization_msgs::msg::MarkerArray & debug_markers) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  pub_objects_->publish(output);
  if (published_time_publisher_)
    published_time_publisher_->publish_if_subscribed(pub_objects_, output.header.stamp);
  if (pub_debug_markers_) pub_debug_markers_->publish(debug_markers);
}

void MapBasedPredictionNode::updateObjectData(TrackedObject & object)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE) {
    return;
  }

  // assumption: the object vx is much larger than vy
  if (object.kinematics.twist_with_covariance.twist.linear.x >= 0.0) return;

  // calculate absolute velocity and do nothing if it is too slow
  const double abs_object_speed = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  constexpr double min_abs_speed = 1e-1;  // 0.1 m/s
  if (abs_object_speed < min_abs_speed) return;

  // invert yaw to align with tracked movement when state is SIGN_UNKNOWN
  if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN) {
    const auto original_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    // flip the angle
    object.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(autoware_utils::pi + original_yaw);

    // flip the vector
    object.kinematics.twist_with_covariance.twist.linear.x *= -1.0;
    object.kinematics.twist_with_covariance.twist.linear.y *= -1.0;
  }

  return;
}

LaneletsData MapBasedPredictionNode::getCurrentLanelets(const TrackedObject & object)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  return utils::getCurrentLanelets(
    object, lanelet_map_ptr_, road_users_history_, dist_threshold_for_searching_lanelet_,
    delta_yaw_threshold_for_searching_lanelet_, sigma_lateral_offset_, sigma_yaw_angle_deg_);
}

void MapBasedPredictionNode::updateRoadUsersHistory(
  const std_msgs::msg::Header & header, const TrackedObject & object,
  const LaneletsData & current_lanelets_data)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::string object_id = autoware_utils::to_hex_string(object.object_id);
  const auto current_lanelets = getLanelets(current_lanelets_data);

  RoadUser road_user;
  road_user.header = header;
  road_user.current_lanelets = current_lanelets;
  road_user.future_possible_lanelets = current_lanelets;
  road_user.pose = object.kinematics.pose_with_covariance.pose;
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  road_user.pose.orientation = autoware_utils::create_quaternion_from_yaw(object_yaw);
  road_user.time_delay = std::fabs((this->get_clock()->now() - header.stamp).seconds());
  road_user.twist = object.kinematics.twist_with_covariance.twist;

  // Init lateral kinematics
  for (const auto & current_lane : current_lanelets) {
    const LateralKinematicsToLanelet lateral_kinematics =
      initLateralKinematics(current_lane, road_user.pose);
    road_user.lateral_kinematics_set[current_lane] = lateral_kinematics;
  }

  if (road_users_history_.count(object_id) == 0) {
    // New Object(Create a new object in object histories)
    road_users_history_.emplace(object_id, std::deque<RoadUser>({road_user}));
  } else {
    // Object that is already in the object buffer
    std::deque<RoadUser> & road_users = road_users_history_.at(object_id);
    // get previous object data and update
    const auto prev_road_user = road_users.back();
    updateLateralKinematicsVector(
      prev_road_user, road_user, routing_graph_ptr_, cutoff_freq_of_velocity_lpf_);

    road_users.push_back(road_user);
  }
}

std::vector<LaneletPathWithPathInfo> MapBasedPredictionNode::getPredictedReferencePath(
  const TrackedObject & object, const LaneletsData & current_lanelets_data,
  const double object_detected_time, const double time_horizon)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Step 1. Set conditions for the prediction
  const double obj_vel = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);

  // Using a decaying acceleration model. Consult the README for more information about the model.
  const double obj_acc = (use_vehicle_acceleration_)
                           ? std::hypot(
                               object.kinematics.acceleration_with_covariance.accel.linear.x,
                               object.kinematics.acceleration_with_covariance.accel.linear.y)
                           : 0.0;
  const double t_h = time_horizon;
  const double lambda = std::log(2) / acceleration_exponential_half_life_;
  const double validate_time_horizon = t_h * prediction_time_horizon_rate_for_validate_lane_length_;
  const double final_speed_after_acceleration =
    obj_vel + obj_acc * (1.0 / lambda) * (1.0 - std::exp(-lambda * t_h));

  auto get_search_distance_with_decaying_acc = [&]() -> double {
    const double acceleration_distance =
      obj_acc * (1.0 / lambda) * t_h +
      obj_acc * (1.0 / (lambda * lambda)) * std::expm1(-lambda * t_h);
    double search_dist = acceleration_distance + obj_vel * t_h;
    return search_dist;
  };

  auto get_search_distance_with_partial_acc = [&](const double final_speed) -> double {
    constexpr double epsilon = 1E-5;
    if (std::abs(obj_acc) < epsilon) {
      // Assume constant speed
      return obj_vel * t_h;
    }
    // Time to reach final speed
    const double t_f = (-1.0 / lambda) * std::log(1 - ((final_speed - obj_vel) * lambda) / obj_acc);
    // It is assumed the vehicle accelerates until final_speed is reached and
    // then continues at constant speed for the rest of the time horizon
    const double search_dist =
      // Distance covered while accelerating
      obj_acc * (1.0 / lambda) * t_f +
      obj_acc * (1.0 / std::pow(lambda, 2)) * std::expm1(-lambda * t_f) + obj_vel * t_f +
      // Distance covered at constant speed
      final_speed * (t_h - t_f);
    return search_dist;
  };

  std::string object_id = autoware_utils::to_hex_string(object.object_id);
  geometry_msgs::msg::Pose object_pose = object.kinematics.pose_with_covariance.pose;

  // Step 2. Get possible paths for each lanelet
  std::vector<LaneletPathWithPathInfo> lanelet_ref_paths;
  for (const auto & current_lanelet_data : current_lanelets_data) {
    std::vector<LaneletPathWithPathInfo> ref_paths_per_lanelet;

    // Set condition on each lanelet
    lanelet::routing::PossiblePathsParams possible_params{0, {}, 0, false, true};
    double target_speed_limit = 0.0;
    {
      const lanelet::traffic_rules::SpeedLimitInformation limit =
        traffic_rules_ptr_->speedLimit(current_lanelet_data.lanelet);
      const double legal_speed_limit = static_cast<double>(limit.speedLimit.value());
      target_speed_limit = legal_speed_limit * speed_limit_multiplier_;
      const bool final_speed_surpasses_limit = final_speed_after_acceleration > target_speed_limit;
      const bool object_has_surpassed_limit_already = obj_vel > target_speed_limit;

      double search_dist = (final_speed_surpasses_limit && !object_has_surpassed_limit_already)
                             ? get_search_distance_with_partial_acc(target_speed_limit)
                             : get_search_distance_with_decaying_acc();
      search_dist += lanelet::geometry::length3d(current_lanelet_data.lanelet);
      possible_params.routingCostLimit = search_dist;
    }

    // lambda function to get possible paths for isolated lanelet
    // isolated is often caused by lanelet with no connection e.g. shoulder-lane
    auto getPathsForNormalOrIsolatedLanelet = [&](const lanelet::ConstLanelet & lanelet) {
      // if lanelet is not isolated, return normal possible paths
      if (!isIsolatedLanelet(lanelet, routing_graph_ptr_)) {
        return routing_graph_ptr_->possiblePaths(lanelet, possible_params);
      }
      // if lanelet is isolated, check if it has enough length
      if (!validateIsolatedLaneletLength(lanelet, object, validate_time_horizon)) {
        return lanelet::routing::LaneletPaths{};
      } else {
        // if lanelet has enough length, return possible paths
        return getPossiblePathsForIsolatedLanelet(lanelet);
      }
    };

    // lambda function to extract left/right lanelets
    auto getLeftOrRightLanelets = [&](
                                    const lanelet::ConstLanelet & lanelet,
                                    const bool get_left) -> std::optional<lanelet::ConstLanelet> {
      const auto opt =
        get_left ? routing_graph_ptr_->left(lanelet) : routing_graph_ptr_->right(lanelet);
      if (!!opt) {
        return *opt;
      }
      if (!consider_only_routable_neighbours_) {
        const auto adjacent = get_left ? routing_graph_ptr_->adjacentLeft(lanelet)
                                       : routing_graph_ptr_->adjacentRight(lanelet);
        if (!!adjacent) {
          return *adjacent;
        }
        // search for unconnected lanelet
        const auto unconnected_lanelets =
          get_left ? getLeftLineSharingLanelets(lanelet, lanelet_map_ptr_)
                   : getRightLineSharingLanelets(lanelet, lanelet_map_ptr_);
        // just return first candidate of unconnected lanelet for now
        if (!unconnected_lanelets.empty()) {
          return unconnected_lanelets.front();
        }
      }

      // if no candidate lanelet found, return empty
      return std::nullopt;
    };

    bool left_paths_exists = false;
    bool right_paths_exists = false;
    bool center_paths_exists = false;

    // a-1. Get the left lanelet
    {
      PredictedRefPath ref_path_info;
      lanelet::routing::LaneletPaths left_paths;
      const auto left_lanelet = getLeftOrRightLanelets(current_lanelet_data.lanelet, true);
      if (!!left_lanelet) {
        left_paths = getPathsForNormalOrIsolatedLanelet(left_lanelet.value());
        left_paths_exists = !left_paths.empty();
      }
      ref_path_info.speed_limit = target_speed_limit;
      ref_path_info.maneuver = Maneuver::LEFT_LANE_CHANGE;
      for (auto & path : left_paths) {
        ref_paths_per_lanelet.emplace_back(path, ref_path_info);
      }
    }

    // a-2. Get the right lanelet
    {
      PredictedRefPath ref_path_info;
      lanelet::routing::LaneletPaths right_paths;
      const auto right_lanelet = getLeftOrRightLanelets(current_lanelet_data.lanelet, false);
      if (!!right_lanelet) {
        right_paths = getPathsForNormalOrIsolatedLanelet(right_lanelet.value());
        right_paths_exists = !right_paths.empty();
      }
      ref_path_info.speed_limit = target_speed_limit;
      ref_path_info.maneuver = Maneuver::RIGHT_LANE_CHANGE;
      for (auto & path : right_paths) {
        ref_paths_per_lanelet.emplace_back(path, ref_path_info);
      }
    }

    // a-3. Get the center lanelet
    {
      PredictedRefPath ref_path_info;
      lanelet::routing::LaneletPaths center_paths =
        getPathsForNormalOrIsolatedLanelet(current_lanelet_data.lanelet);
      center_paths_exists = !center_paths.empty();
      ref_path_info.speed_limit = target_speed_limit;
      ref_path_info.maneuver = Maneuver::LANE_FOLLOW;
      for (auto & path : center_paths) {
        ref_paths_per_lanelet.emplace_back(path, ref_path_info);
      }
    }

    // Skip calculations if all paths are empty
    if (ref_paths_per_lanelet.empty()) {
      continue;
    }

    // b. Predict Object Maneuver
    const Maneuver predicted_maneuver =
      predictObjectManeuver(object_id, object_pose, current_lanelet_data, object_detected_time);

    // c. Allocate probability for each predicted maneuver
    const float & path_prob = current_lanelet_data.probability;
    const auto maneuver_prob = calculateManeuverProbability(
      predicted_maneuver, left_paths_exists, right_paths_exists, center_paths_exists);
    for (auto & ref_path : ref_paths_per_lanelet) {
      auto & ref_path_info = ref_path.second;
      ref_path_info.probability = maneuver_prob.at(ref_path_info.maneuver) * path_prob;
    }

    // move the calculated ref paths to the lanelet_ref_paths
    lanelet_ref_paths.insert(
      lanelet_ref_paths.end(), ref_paths_per_lanelet.begin(), ref_paths_per_lanelet.end());
  }

  // update future possible lanelets
  if (road_users_history_.count(object_id) != 0) {
    std::vector<lanelet::ConstLanelet> & possible_lanelets =
      road_users_history_.at(object_id).back().future_possible_lanelets;
    for (const auto & ref_path : lanelet_ref_paths) {
      for (const auto & lanelet : ref_path.first) {
        if (
          std::find(possible_lanelets.begin(), possible_lanelets.end(), lanelet) ==
          possible_lanelets.end()) {
          possible_lanelets.push_back(lanelet);
        }
      }
    }
  }

  return lanelet_ref_paths;
}

/**
 * @brief Do lane change prediction
 * @return predicted manuever (lane follow, left/right lane change)
 */
Maneuver MapBasedPredictionNode::predictObjectManeuver(
  const std::string & object_id, const geometry_msgs::msg::Pose & object_pose,
  const LaneletData & current_lanelet_data, const double object_detected_time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // calculate maneuver
  const auto current_maneuver = [&]() {
    if (lane_change_detection_method_ == "time_to_change_lane") {
      return predictObjectManeuverByTimeToLaneChange(
        object_id, current_lanelet_data, object_detected_time);
    } else if (lane_change_detection_method_ == "lat_diff_distance") {
      return predictObjectManeuverByLatDiffDistance(
        object_id, object_pose, current_lanelet_data, object_detected_time);
    }
    throw std::logic_error("Lane change detection method is invalid.");
  }();

  if (road_users_history_.count(object_id) == 0) {
    return current_maneuver;
  }
  auto & object_info = road_users_history_.at(object_id);

  // update maneuver in object history
  if (!object_info.empty()) {
    object_info.back().one_shot_maneuver = current_maneuver;
  }

  // decide maneuver considering previous results
  if (object_info.size() < 2) {
    object_info.back().output_maneuver = current_maneuver;
    return current_maneuver;
  }
  // NOTE: The index of previous maneuver is not object_info.size() - 1
  const auto prev_output_maneuver =
    object_info.at(static_cast<int>(object_info.size()) - 2).output_maneuver;

  for (int i = 0;
       i < std::min(num_continuous_state_transition_, static_cast<int>(object_info.size())); ++i) {
    const auto & tmp_maneuver =
      object_info.at(static_cast<int>(object_info.size()) - 1 - i).one_shot_maneuver;
    if (tmp_maneuver != current_maneuver) {
      object_info.back().output_maneuver = prev_output_maneuver;
      return prev_output_maneuver;
    }
  }

  object_info.back().output_maneuver = current_maneuver;
  return current_maneuver;
}

Maneuver MapBasedPredictionNode::predictObjectManeuverByTimeToLaneChange(
  const std::string & object_id, const LaneletData & current_lanelet_data,
  const double /*object_detected_time*/)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Step1. Check if we have the object in the buffer
  if (road_users_history_.count(object_id) == 0) {
    return Maneuver::LANE_FOLLOW;
  }

  const std::deque<RoadUser> & object_info = road_users_history_.at(object_id);

  // Step2. Check if object history length longer than history_time_length
  const int latest_id = static_cast<int>(object_info.size()) - 1;
  // object history is not long enough
  if (latest_id < 1) {
    return Maneuver::LANE_FOLLOW;
  }

  // Step3. get object lateral kinematics
  const auto & latest_info = object_info.at(static_cast<size_t>(latest_id));

  bool not_found_corresponding_lanelet = true;
  double left_dist, right_dist;
  double v_left_filtered, v_right_filtered;
  if (latest_info.lateral_kinematics_set.count(current_lanelet_data.lanelet) != 0) {
    const auto & lateral_kinematics =
      latest_info.lateral_kinematics_set.at(current_lanelet_data.lanelet);
    left_dist = lateral_kinematics.dist_from_left_boundary;
    right_dist = lateral_kinematics.dist_from_right_boundary;
    v_left_filtered = lateral_kinematics.filtered_left_lateral_velocity;
    v_right_filtered = lateral_kinematics.filtered_right_lateral_velocity;
    not_found_corresponding_lanelet = false;
  }

  // return lane follow when catch exception
  if (not_found_corresponding_lanelet) {
    return Maneuver::LANE_FOLLOW;
  }

  const double latest_lane_width = left_dist + right_dist;
  if (latest_lane_width < 1e-3) {
    RCLCPP_ERROR(get_logger(), "[Map Based Prediction]: Lane Width is too small");
    return Maneuver::LANE_FOLLOW;
  }

  // Step 4. check time to reach left/right bound
  const double epsilon = 1e-9;
  const double margin_to_reach_left_bound = left_dist / (std::fabs(v_left_filtered) + epsilon);
  const double margin_to_reach_right_bound = right_dist / (std::fabs(v_right_filtered) + epsilon);

  // Step 5. detect lane change
  if (
    left_dist < right_dist &&                              // in left side,
    left_dist < dist_threshold_to_bound_ &&                // close to boundary,
    v_left_filtered < 0 &&                                 // approaching,
    margin_to_reach_left_bound < time_threshold_to_bound_  // will soon arrive to left bound
  ) {
    return Maneuver::LEFT_LANE_CHANGE;
  } else if (
    right_dist < left_dist &&                               // in right side,
    right_dist < dist_threshold_to_bound_ &&                // close to boundary,
    v_right_filtered < 0 &&                                 // approaching,
    margin_to_reach_right_bound < time_threshold_to_bound_  // will soon arrive to right bound
  ) {
    return Maneuver::RIGHT_LANE_CHANGE;
  }

  return Maneuver::LANE_FOLLOW;
}

Maneuver MapBasedPredictionNode::predictObjectManeuverByLatDiffDistance(
  const std::string & object_id, const geometry_msgs::msg::Pose & object_pose,
  const LaneletData & current_lanelet_data, const double /*object_detected_time*/)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Step1. Check if we have the object in the buffer
  if (road_users_history_.count(object_id) == 0) {
    return Maneuver::LANE_FOLLOW;
  }

  const std::deque<RoadUser> & object_info = road_users_history_.at(object_id);
  const double current_time = (this->get_clock()->now()).seconds();

  // Step2. Get the previous id
  int prev_id = static_cast<int>(object_info.size()) - 1;
  while (prev_id >= 0) {
    const double prev_time_delay = object_info.at(prev_id).time_delay;
    const double prev_time =
      rclcpp::Time(object_info.at(prev_id).header.stamp).seconds() + prev_time_delay;
    // if (object_detected_time - prev_time > history_time_length_) {
    if (current_time - prev_time > history_time_length_) {
      break;
    }
    --prev_id;
  }

  if (prev_id < 0) {
    return Maneuver::LANE_FOLLOW;
  }

  // Step3. Get closest previous lanelet ID
  const auto & prev_info = object_info.at(static_cast<size_t>(prev_id));
  const auto prev_pose = prev_info.pose;
  const lanelet::ConstLanelets prev_lanelets =
    object_info.at(static_cast<size_t>(prev_id)).current_lanelets;
  if (prev_lanelets.empty()) {
    return Maneuver::LANE_FOLLOW;
  }
  lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
  double closest_prev_yaw = std::numeric_limits<double>::max();
  for (const auto & lanelet : prev_lanelets) {
    const double lane_yaw = autoware::experimental::lanelet2_utils::get_lanelet_angle(
      lanelet, autoware::experimental::lanelet2_utils::from_ros(prev_pose).basicPoint());
    const double delta_yaw = tf2::getYaw(prev_pose.orientation) - lane_yaw;
    const double normalized_delta_yaw = autoware_utils::normalize_radian(delta_yaw);
    if (normalized_delta_yaw < closest_prev_yaw) {
      closest_prev_yaw = normalized_delta_yaw;
      prev_lanelet = lanelet;
    }
  }

  // Step4. Check if the vehicle has changed lane
  const auto current_lanelet = current_lanelet_data.lanelet;
  const auto current_pose = object_pose;
  const double dist = autoware_utils::calc_distance2d(prev_pose, current_pose);
  lanelet::routing::LaneletPaths possible_paths =
    routing_graph_ptr_->possiblePaths(prev_lanelet, dist + 2.0, 0, false);
  bool has_lane_changed = true;
  if (prev_lanelet == current_lanelet) {
    has_lane_changed = false;
  } else {
    for (const auto & path : possible_paths) {
      for (const auto & lanelet : path) {
        if (lanelet == current_lanelet) {
          has_lane_changed = false;
          break;
        }
      }
    }
  }

  if (has_lane_changed) {
    return Maneuver::LANE_FOLLOW;
  }

  // Step5. Lane Change Detection
  const lanelet::ConstLineString2d prev_left_bound = prev_lanelet.leftBound2d();
  const lanelet::ConstLineString2d prev_right_bound = prev_lanelet.rightBound2d();
  const lanelet::ConstLineString2d current_left_bound = current_lanelet.leftBound2d();
  const lanelet::ConstLineString2d current_right_bound = current_lanelet.rightBound2d();
  const double prev_left_dist = calcLeftLateralOffset(prev_left_bound, prev_pose);
  const double prev_right_dist = calcRightLateralOffset(prev_right_bound, prev_pose);
  const double current_left_dist = calcLeftLateralOffset(current_left_bound, current_pose);
  const double current_right_dist = calcRightLateralOffset(current_right_bound, current_pose);
  const double prev_lane_width = std::fabs(prev_left_dist) + std::fabs(prev_right_dist);
  const double current_lane_width = std::fabs(current_left_dist) + std::fabs(current_right_dist);
  if (prev_lane_width < 1e-3 || current_lane_width < 1e-3) {
    RCLCPP_ERROR(get_logger(), "[Map Based Prediction]: Lane Width is too small");
    return Maneuver::LANE_FOLLOW;
  }

  const double current_left_dist_ratio = current_left_dist / current_lane_width;
  const double current_right_dist_ratio = current_right_dist / current_lane_width;
  const double diff_left_current_prev = current_left_dist - prev_left_dist;
  const double diff_right_current_prev = current_right_dist - prev_right_dist;

  if (
    current_left_dist_ratio > dist_ratio_threshold_to_left_bound_ &&
    diff_left_current_prev > diff_dist_threshold_to_left_bound_) {
    return Maneuver::LEFT_LANE_CHANGE;
  } else if (
    current_right_dist_ratio < dist_ratio_threshold_to_right_bound_ &&
    diff_right_current_prev < diff_dist_threshold_to_right_bound_) {
    return Maneuver::RIGHT_LANE_CHANGE;
  }

  return Maneuver::LANE_FOLLOW;
}

double MapBasedPredictionNode::calcRightLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = autoware_utils::create_point(x, y, 0.0);
  }

  return std::fabs(autoware::motion_utils::calcLateralOffset(boundary_path, search_pose.position));
}

double MapBasedPredictionNode::calcLeftLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  return -calcRightLateralOffset(boundary_line, search_pose);
}

ManeuverProbability MapBasedPredictionNode::calculateManeuverProbability(
  const Maneuver & predicted_maneuver, const bool & left_paths_exists,
  const bool & right_paths_exists, const bool & center_paths_exists) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  return utils::calculateManeuverProbability(
    predicted_maneuver, left_paths_exists, right_paths_exists, center_paths_exists);
}

std::optional<lanelet::Id> MapBasedPredictionNode::getTrafficSignalId(
  const lanelet::ConstLanelet & way_lanelet) const
{
  const auto traffic_light_reg_elems =
    way_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
  if (traffic_light_reg_elems.empty()) {
    return std::nullopt;
  }
  if (traffic_light_reg_elems.size() > 1) {
    RCLCPP_ERROR(
      get_logger(),
      "[Map Based Prediction]: Multiple regulatory elements as TrafficLight are defined to one "
      "lanelet object.");
  }
  return traffic_light_reg_elems.front()->id();
}

std::optional<TrafficLightGroup> MapBasedPredictionNode::getSignalForLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  const auto signal_id = getTrafficSignalId(lanelet);
  if (!signal_id) {
    return std::nullopt;
  }
  const auto it = traffic_signal_id_map_.find(*signal_id);
  if (it == traffic_signal_id_map_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void MapBasedPredictionNode::publishPriorityDebugMarkers(
  const PredictedObjects & output, const TrackedObjects::ConstSharedPtr & in_objects)
{
  const auto now = this->now();
  int32_t path_id = 0;
  const auto go_color = autoware_utils::create_marker_color(0.6, 1.0, 0.6, 0.95);
  const auto stop_color = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 1.0);
  const auto creep_color = autoware_utils::create_marker_color(1.0, 0.85, 0.0, 1.0);
  for (const auto & obj : output.objects) {
    const auto label =
      obj.classification.empty()
        ? ObjectClassification::UNKNOWN
        : autoware::object_recognition_utils::getHighestProbLabel(obj.classification);
    if (
      label != ObjectClassification::CAR && label != ObjectClassification::BUS &&
      label != ObjectClassification::TRAILER && label != ObjectClassification::MOTORCYCLE &&
      label != ObjectClassification::TRUCK) {
      continue;
    }
    const auto & paths = obj.kinematics.predicted_paths;
    const std::string oid = autoware_utils::to_hex_string(obj.object_id);
    const auto cons_it = conservative_path_is_creep_.find(oid);
    const bool has_conservative = cons_it != conservative_path_is_creep_.end();

    const auto & dims = obj.shape.dimensions;
    auto box = autoware_utils::create_default_marker(
      "map", now, "vehicle_boxes", path_id++, visualization_msgs::msg::Marker::CUBE,
      autoware_utils::create_marker_scale(
        dims.x > 0.1 ? dims.x : 4.5, dims.y > 0.1 ? dims.y : 2.0, dims.z > 0.1 ? dims.z : 1.7),
      autoware_utils::create_marker_color(0.75, 0.85, 1.0, 0.45));
    box.pose = obj.kinematics.initial_pose_with_covariance.pose;
    box.lifetime = rclcpp::Duration::from_seconds(0.3);
    priority_object_markers_.markers.push_back(box);

    size_t max_conf_pi = paths.size();
    float max_conf = -1.0f;
    for (size_t pi = 0; pi < paths.size(); ++pi) {
      if (paths[pi].path.size() >= 2 && paths[pi].confidence > max_conf) {
        max_conf = paths[pi].confidence;
        max_conf_pi = pi;
      }
    }
    for (size_t pi = 0; pi < paths.size(); ++pi) {
      if (paths[pi].path.size() < 2) {
        continue;
      }
      bool is_conservative = false;
      bool is_creep = false;
      if (has_conservative) {
        const auto idx_it = cons_it->second.find(pi);
        if (idx_it != cons_it->second.end()) {
          is_conservative = true;
          is_creep = idx_it->second;
        }
      }
      auto color = !is_conservative ? go_color : (is_creep ? creep_color : stop_color);
      color.a = (pi == max_conf_pi) ? 0.7f : 0.1f;
      constexpr double width = 0.6;
      constexpr double z_off = 0.8;
      auto line = autoware_utils::create_default_marker(
        "map", now, "step3_paths", path_id++, visualization_msgs::msg::Marker::LINE_STRIP,
        autoware_utils::create_marker_scale(width, 0.0, 0.0), color);
      line.lifetime = rclcpp::Duration::from_seconds(0.3);
      for (const auto & pose : paths[pi].path) {
        auto pt = pose.position;
        pt.z += z_off;
        line.points.push_back(pt);
      }
      priority_object_markers_.markers.push_back(line);
    }
  }

  for (const auto & stop_line : priority_debug_stop_lines_) {
    auto sl = autoware_utils::create_default_marker(
      "map", now, "stop_lines", path_id++, visualization_msgs::msg::Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.4, 0.0, 0.0),
      autoware_utils::create_marker_color(1.0, 0.0, 1.0, 0.9));  // magenta
    sl.lifetime = rclcpp::Duration::from_seconds(0.3);
    for (const auto & p : stop_line) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z() + 0.5;
      sl.points.push_back(pt);
    }
    priority_object_markers_.markers.push_back(sl);
  }

  const auto map_to_base = transform_listener_.get_transform(
    "map", "base_link", in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
  if (map_to_base) {
    geometry_msgs::msg::Pose ego_pose;
    ego_pose.position.x = map_to_base->transform.translation.x;
    ego_pose.position.y = map_to_base->transform.translation.y;
    ego_pose.position.z = map_to_base->transform.translation.z;
    ego_pose.orientation = map_to_base->transform.rotation;
    const auto ego_color = autoware_utils::create_marker_color(0.1, 0.9, 1.0, 0.95);  // cyan
    auto ego_box = autoware_utils::create_default_marker(
      "map", now, "ego", 0, visualization_msgs::msg::Marker::CUBE,
      autoware_utils::create_marker_scale(5.0, 2.2, 1.8), ego_color);
    ego_box.pose = ego_pose;
    ego_box.pose.position.z += 1.5;
    ego_box.lifetime = rclcpp::Duration::from_seconds(0.3);
    priority_object_markers_.markers.push_back(ego_box);

    auto ego_text = autoware_utils::create_default_marker(
      "map", now, "ego_text", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.0, 0.0, 2.5),
      autoware_utils::create_marker_color(0.6, 1.0, 1.0, 1.0));
    ego_text.pose = ego_pose;
    ego_text.pose.position.z += 4.0;
    ego_text.text = "EGO";
    ego_text.lifetime = rclcpp::Duration::from_seconds(0.3);
    priority_object_markers_.markers.push_back(ego_text);
  }
  pub_priority_object_markers_->publish(priority_object_markers_);
}

void MapBasedPredictionNode::applyPriorityCalibration(
  const TrackedObject & object, const std::vector<PredictedRefPath> & ref_paths,
  const std::vector<int> & predicted_path_ref_index, const double time_horizon,
  std::vector<PredictedPath> & predicted_paths)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (predicted_paths.empty() || ref_paths.empty()) {
    return;
  }

  const std::string object_id_str = autoware_utils::to_hex_string(object.object_id);
  priority_debug_.vehicles++;

  const size_t original_count = predicted_paths.size();
  // Conservative hypotheses are appended after the loop so the (object, path
  // index) bookkeeping stays correct when go paths are suppressed.
  std::vector<bool> suppress_go(original_count, false);
  std::vector<PredictedPath> conservative_to_add;
  std::vector<bool> conservative_is_creep;
  bool object_has_stop = false;
  for (size_t i = 0; i < original_count; ++i) {
    const int ref_idx = predicted_path_ref_index[i];
    if (ref_idx < 0 || static_cast<size_t>(ref_idx) >= ref_paths.size()) {
      continue;
    }
    const auto & ref_path = ref_paths[ref_idx];
    if (ref_path.path.size() < 2) {
      continue;
    }

    const auto info = priority::classifyPathAtTrafficLight(ref_path.lanelet_path);

    priority::PriorityContext context;
    if (info.found) {
      context.has_traffic_light = true;
      const auto signal = getSignalForLanelet(info.signal_lanelet);
      context.signal_priority = priority::evaluateSignalPriority(info.signal_lanelet, signal);
    }
    if (context.signal_priority == priority::SignalPriority::FULLY_PRIORITIZED) {
      priority_debug_.signal_stop++;
    }
    if (context.signal_priority == priority::SignalPriority::PARTIALLY_PRIORITIZED) {
      priority_debug_.signal_creep++;
    }

    // ref_path[0] may sit a lanelet ahead of / behind the object, so arc lengths
    // measured from ref_path[0] are converted to object-relative by subtracting
    // s_obj -- the same convention as the path generator.
    const auto & op = object.kinematics.pose_with_covariance.pose.position;
    const auto & r0 = ref_path.path.front();
    const double h0 = tf2::getYaw(r0.orientation);
    const double s_obj =
      (op.x - r0.position.x) * std::cos(h0) + (op.y - r0.position.y) * std::sin(h0);

    if (info.stop_line) {
      if (const auto distance = priority::arcLengthToStopLine(ref_path.path, *info.stop_line)) {
        context.distance_to_stopline = *distance - s_obj;
        priority_debug_.stopline_found++;
      }
    }

    // TEMP[leadcheck]
    const double dbg_stopline_rel = context.distance_to_stopline;
    double dbg_lead_rel = -1.0;
    bool dbg_clamped = false;

    // Lead-vehicle clamp only refines the stop POSITION; whether the object stops
    // at all is still decided by the signal, so on green a leader has no effect.
    if (priority_use_lead_vehicle_) {
      if (
        const auto lead = priority::distanceToLeadObject(
          ref_path.path, object_id_str, object.shape.dimensions.x, lead_objects_,
          priority_follow_lateral_threshold_, priority_follow_gap_margin_, s_obj)) {
        const double lead_rel = *lead - s_obj;
        dbg_lead_rel = lead_rel;
        if (lead_rel < context.distance_to_stopline) {
          context.distance_to_stopline = lead_rel;
          dbg_clamped = true;
          priority_debug_.lead_clamped++;
        }
      }
    }

    // Chattering is already suppressed by the per-signal hysteresis, so no
    // per-object debounce is needed.
    const auto maneuver =
      priority::decideConservativeManeuver(context, priority_calibration_params_);
    const auto calibration = priority::weightsForManeuver(maneuver, priority_calibration_params_);

    if (calibration.maneuver == priority::ConservativeManeuver::NONE) {
      continue;
    }

    const bool is_amber = calibration.maneuver == priority::ConservativeManeuver::CREEP;
    bool keep_go = false;
    if (is_amber) {
      const double velocity = object.kinematics.twist_with_covariance.twist.linear.x;
      const double acceleration = object.kinematics.acceleration_with_covariance.accel.linear.x;
      switch (priority::judgeYellow(
        velocity, acceleration, context.distance_to_stopline, priority_yellow_params_)) {
        case priority::YellowOutcome::PASS:
          continue;
        case priority::YellowOutcome::DILEMMA:
          keep_go = true;
          break;
        case priority::YellowOutcome::STOP:
          break;
      }
    }

    // target_velocity is 0.0 even for CREEP: amber objects that pass through have
    // already been filtered out by judgeYellow.
    PredictedPath conservative_path = path_generator_->generateStoppingPathForOnLaneVehicle(
      object, ref_path.path, time_horizon, priority_stop_deceleration_, 0.0,
      context.distance_to_stopline, ref_path.speed_limit, priority_extend_stop_path_to_stopline_);
    if (conservative_path.path.empty()) {
      continue;
    }
    // Clip regardless of the extend flag: lane curvature / angled stop lines can
    // produce a small overshoot even without extend.
    if (info.stop_line) {
      clipPathAtStopLine(conservative_path, *info.stop_line);
    }
    // TEMP[leadcheck]: verify whether the lead clamp actually moves the stop position.
    {
      double path_arc = 0.0;
      for (size_t k = 1; k < conservative_path.path.size(); ++k) {
        const auto & pa = conservative_path.path[k - 1].position;
        const auto & pb = conservative_path.path[k].position;
        path_arc += std::hypot(pb.x - pa.x, pb.y - pa.y);
      }
      RCLCPP_INFO(
        get_logger(), "[leadcheck] clamped=%d lead_rel=%.2f stopline_rel=%.2f path_end=%.2f",
        dbg_clamped ? 1 : 0, dbg_lead_rel, dbg_stopline_rel, path_arc);
    }
    conservative_path.confidence = static_cast<float>(calibration.conservative_weight);
    if (!keep_go) {
      object_has_stop = true;
      if (priority_suppress_go_on_conservative_) {
        suppress_go[i] = true;
      }
    }
    conservative_to_add.push_back(conservative_path);
    conservative_is_creep.push_back(is_amber);
    if (info.stop_line) {
      priority_debug_stop_lines_.push_back(*info.stop_line);
    }
    priority_debug_.conservative_added++;
  }

  if (object_has_stop) {
    const auto go_scale =
      static_cast<float>(priority_calibration_params_.go_probability_decay_on_yield);
    for (size_t i = 0; i < original_count; ++i) {
      if (!suppress_go[i]) {
        predicted_paths[i].confidence *= go_scale;
      }
    }
  }

  if (!conservative_to_add.empty()) {
    std::vector<PredictedPath> rebuilt;
    rebuilt.reserve(predicted_paths.size() + conservative_to_add.size());
    for (size_t i = 0; i < predicted_paths.size(); ++i) {
      if (i < original_count && suppress_go[i]) {
        continue;
      }
      rebuilt.push_back(predicted_paths[i]);
    }
    for (size_t k = 0; k < conservative_to_add.size(); ++k) {
      conservative_path_is_creep_[object_id_str][rebuilt.size()] = conservative_is_creep[k];
      rebuilt.push_back(conservative_to_add[k]);
    }
    predicted_paths = std::move(rebuilt);
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "[priority] vehicles=%zu sig_stop=%zu sig_creep=%zu stopline=%zu lead=%zu added=%zu",
    priority_debug_.vehicles, priority_debug_.signal_stop, priority_debug_.signal_creep,
    priority_debug_.stopline_found, priority_debug_.lead_clamped,
    priority_debug_.conservative_added);
}

std::vector<PredictedRefPath> MapBasedPredictionNode::convertPredictedReferencePath(
  const TrackedObject & object,
  const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::vector<PredictedRefPath> converted_ref_paths;

  // Step 1. Convert lanelet path to pose path
  for (const auto & ref_path : lanelet_ref_paths) {
    const auto & lanelet_path = ref_path.first;
    const auto & ref_path_info = ref_path.second;
    const auto converted_path = convertLaneletPathToPosePath(lanelet_path);
    PredictedRefPath predicted_path;
    predicted_path.probability = ref_path_info.probability;
    predicted_path.path = converted_path.first;
    predicted_path.width = converted_path.second;
    predicted_path.maneuver = ref_path_info.maneuver;
    predicted_path.speed_limit = ref_path_info.speed_limit;
    predicted_path.lanelet_path = lanelet_path;
    converted_ref_paths.push_back(predicted_path);
  }

  // Step 2. Search starting point for each reference path
  for (auto it = converted_ref_paths.begin(); it != converted_ref_paths.end();) {
    auto & pose_path = it->path;
    if (pose_path.empty()) {
      continue;
    }

    const std::optional<size_t> opt_starting_idx =
      searchProperStartingRefPathIndex(object, pose_path);

    if (opt_starting_idx.has_value()) {
      // Trim the reference path
      pose_path.erase(pose_path.begin(), pose_path.begin() + opt_starting_idx.value());
      ++it;
    } else {
      // Proper starting point is not found, remove the reference path
      it = converted_ref_paths.erase(it);
    }
  }

  return converted_ref_paths;
}

std::pair<PosePath, double> MapBasedPredictionNode::convertLaneletPathToPosePath(
  const lanelet::routing::LaneletPath & path) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (lru_cache_of_convert_path_type_.contains(path)) {
    return *lru_cache_of_convert_path_type_.get(path);
  }

  std::pair<PosePath, double> converted_path_and_width;
  {
    PosePath converted_path;
    double width = 10.0;  // Initialize with a large value

    // Insert Positions. Note that we start inserting points from previous lanelet
    if (!path.empty()) {
      lanelet::ConstLanelets prev_lanelets = routing_graph_ptr_->previous(path.front());
      if (!prev_lanelets.empty()) {
        lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
        bool init_flag = true;
        geometry_msgs::msg::Pose prev_p;
        for (const auto & lanelet_p : prev_lanelet.centerline()) {
          geometry_msgs::msg::Pose current_p;
          current_p.position = experimental::lanelet2_utils::to_ros(lanelet_p);
          if (init_flag) {
            init_flag = false;
            prev_p = current_p;
            continue;
          }

          // only considers yaw of the lanelet
          const double lane_yaw = std::atan2(
            current_p.position.y - prev_p.position.y, current_p.position.x - prev_p.position.x);
          const double sin_yaw_half = std::sin(lane_yaw / 2.0);
          const double cos_yaw_half = std::cos(lane_yaw / 2.0);
          current_p.orientation.x = 0.0;
          current_p.orientation.y = 0.0;
          current_p.orientation.z = sin_yaw_half;
          current_p.orientation.w = cos_yaw_half;

          converted_path.push_back(current_p);
          prev_p = current_p;
        }
      }
    }

    for (const auto & lanelet : path) {
      bool init_flag = true;
      geometry_msgs::msg::Pose prev_p;
      for (const auto & lanelet_p : lanelet.centerline()) {
        geometry_msgs::msg::Pose current_p;
        current_p.position = experimental::lanelet2_utils::to_ros(lanelet_p);
        if (init_flag) {
          init_flag = false;
          prev_p = current_p;
          continue;
        }

        // Prevent from inserting same points
        if (!converted_path.empty()) {
          const auto last_p = converted_path.back();
          const double tmp_dist = autoware_utils::calc_distance2d(last_p, current_p);
          if (tmp_dist < 1e-6) {
            prev_p = current_p;
            continue;
          }
        }

        const double lane_yaw = std::atan2(
          current_p.position.y - prev_p.position.y, current_p.position.x - prev_p.position.x);
        const double sin_yaw_half = std::sin(lane_yaw / 2.0);
        const double cos_yaw_half = std::cos(lane_yaw / 2.0);
        current_p.orientation.x = 0.0;
        current_p.orientation.y = 0.0;
        current_p.orientation.z = sin_yaw_half;
        current_p.orientation.w = cos_yaw_half;

        converted_path.push_back(current_p);
        prev_p = current_p;
      }

      // Update minimum width
      const auto left_bound = lanelet.leftBound2d();
      const auto right_bound = lanelet.rightBound2d();
      const double lanelet_width_front = std::hypot(
        left_bound.front().x() - right_bound.front().x(),
        left_bound.front().y() - right_bound.front().y());
      width = std::min(width, lanelet_width_front);
    }

    // Resample Path
    const bool use_akima_spline_for_xy = true;
    const bool use_lerp_for_z = true;
    // the options use_akima_spline_for_xy and use_lerp_for_z are set to true
    // but the implementation of use_akima_spline_for_xy in resamplePoseVector and
    // resamplePointVector is opposite to the options so the options are set to true to use linear
    // interpolation for xy
    const auto resampled_converted_path = autoware::motion_utils::resamplePoseVector(
      converted_path, reference_path_resolution_, use_akima_spline_for_xy, use_lerp_for_z);
    converted_path_and_width = std::make_pair(resampled_converted_path, width);
  }

  lru_cache_of_convert_path_type_.put(path, converted_path_and_width);
  return converted_path_and_width;
}

PredictedObject MapBasedPredictionNode::getPredictionForNonVehicleObject(
  const std_msgs::msg::Header & header, const TrackedObject & object)
{
  return predictor_vru_->predict(header, object);
}

std::optional<PredictedObject> MapBasedPredictionNode::getPredictionForVehicleObject(
  const std_msgs::msg::Header & header, const TrackedObject & transformed_object,
  const double objects_detected_time, visualization_msgs::msg::MarkerArray & debug_markers)
{
  auto object = transformed_object;

  // Update object yaw and velocity
  updateObjectData(object);

  // Get Closest Lanelet
  const auto current_lanelets = utils::getCurrentLanelets(
    object, lanelet_map_ptr_, road_users_history_, dist_threshold_for_searching_lanelet_,
    delta_yaw_threshold_for_searching_lanelet_, sigma_lateral_offset_, sigma_yaw_angle_deg_);

  // Update Objects History
  updateRoadUsersHistory(header, object, current_lanelets);

  // For off lane obstacles
  if (current_lanelets.empty()) {
    PredictedPath predicted_path =
      path_generator_->generatePathForOffLaneVehicle(object, prediction_time_horizon_.vehicle);
    predicted_path.confidence = 1.0;
    if (predicted_path.path.empty()) {
      return std::nullopt;
    }

    auto predicted_object_vehicle = utils::convertToPredictedObject(object);
    predicted_object_vehicle.kinematics.predicted_paths.push_back(predicted_path);
    return predicted_object_vehicle;
  }

  // For too-slow vehicle
  const double abs_obj_speed = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  if (std::fabs(abs_obj_speed) < min_velocity_for_map_based_prediction_) {
    PredictedPath predicted_path =
      path_generator_->generatePathForLowSpeedVehicle(object, prediction_time_horizon_.vehicle);
    predicted_path.confidence = 1.0;
    if (predicted_path.path.empty()) {
      return std::nullopt;
    }

    auto predicted_slow_object = utils::convertToPredictedObject(object);
    predicted_slow_object.kinematics.predicted_paths.push_back(predicted_path);
    return predicted_slow_object;
  }

  // Get Predicted Reference Path for Each Maneuver and current lanelets
  // return: <probability, paths>
  const auto lanelet_ref_paths = getPredictedReferencePath(
    object, current_lanelets, objects_detected_time, prediction_time_horizon_.vehicle);
  const auto ref_paths = convertPredictedReferencePath(object, lanelet_ref_paths);

  // If predicted reference path is empty, assume this object is out of the lane
  if (ref_paths.empty()) {
    PredictedPath predicted_path =
      path_generator_->generatePathForOffLaneVehicle(object, prediction_time_horizon_.vehicle);
    predicted_path.confidence = 1.0;
    if (predicted_path.path.empty()) {
      return std::nullopt;
    }

    auto predicted_object_out_of_lane = utils::convertToPredictedObject(object);
    predicted_object_out_of_lane.kinematics.predicted_paths.push_back(predicted_path);
    return predicted_object_out_of_lane;
  }

  // Get Debug Marker for On Lane Vehicles
  if (pub_debug_markers_) {
    const auto max_prob_path = std::max_element(
      ref_paths.begin(), ref_paths.end(),
      [](const PredictedRefPath & a, const PredictedRefPath & b) {
        return a.probability < b.probability;
      });
    const auto debug_marker =
      getDebugMarker(object, max_prob_path->maneuver, debug_markers.markers.size());
    debug_markers.markers.push_back(debug_marker);
  }

  // Fix object angle if its orientation unreliable (e.g. far object by radar sensor)
  // This prevent bending predicted path
  TrackedObject yaw_fixed_object = object;
  if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE) {
    replaceObjectYawWithLaneletsYaw(current_lanelets, yaw_fixed_object);
  }
  // Generate Predicted Path
  std::vector<PredictedPath> predicted_paths;
  // Index into ref_paths for each generated predicted_path (-1 = no source ref path).
  std::vector<int> predicted_path_ref_index;
  double min_avg_curvature = std::numeric_limits<double>::max();
  PredictedPath path_with_smallest_avg_curvature;
  int ref_index_of_smallest_avg_curvature = -1;

  for (size_t ref_idx = 0; ref_idx < ref_paths.size(); ++ref_idx) {
    const auto & ref_path = ref_paths[ref_idx];
    PredictedPath predicted_path = path_generator_->generatePathForOnLaneVehicle(
      yaw_fixed_object, ref_path.path, prediction_time_horizon_.vehicle,
      lateral_control_time_horizon_, ref_path.width, ref_path.speed_limit);
    if (predicted_path.path.empty()) continue;

    if (!check_lateral_acceleration_constraints_) {
      predicted_path.confidence = ref_path.probability;
      predicted_paths.push_back(predicted_path);
      predicted_path_ref_index.push_back(static_cast<int>(ref_idx));
      continue;
    }

    // Check lat. acceleration constraints
    const auto trajectory_with_const_velocity = toTrajectoryPoints(predicted_path, abs_obj_speed);

    if (isLateralAccelerationConstraintSatisfied(
          trajectory_with_const_velocity, prediction_sampling_time_interval_)) {
      predicted_path.confidence = ref_path.probability;
      predicted_paths.push_back(predicted_path);
      predicted_path_ref_index.push_back(static_cast<int>(ref_idx));
      continue;
    }

    // Calculate curvature assuming the trajectory points interval is constant
    // In case all paths are deleted, a copy of the straightest path is kept

    constexpr double curvature_calculation_distance = 2.0;
    constexpr double points_interval = 1.0;
    const size_t idx_dist = static_cast<size_t>(
      std::max(static_cast<int>((curvature_calculation_distance) / points_interval), 1));
    const auto curvature_v =
      calcTrajectoryCurvatureFrom3Points(trajectory_with_const_velocity, idx_dist);
    if (curvature_v.empty()) {
      continue;
    }
    const auto curvature_avg =
      std::accumulate(curvature_v.begin(), curvature_v.end(), 0.0) / curvature_v.size();
    if (curvature_avg < min_avg_curvature) {
      min_avg_curvature = curvature_avg;
      path_with_smallest_avg_curvature = predicted_path;
      path_with_smallest_avg_curvature.confidence = ref_path.probability;
      ref_index_of_smallest_avg_curvature = static_cast<int>(ref_idx);
    }
  }

  if (predicted_paths.empty()) {
    predicted_paths.push_back(path_with_smallest_avg_curvature);
    predicted_path_ref_index.push_back(ref_index_of_smallest_avg_curvature);
  }

  if (use_priority_prediction_) {
    applyPriorityCalibration(
      yaw_fixed_object, ref_paths, predicted_path_ref_index, prediction_time_horizon_.vehicle,
      predicted_paths);
  }

  // Normalize Path Confidence and output the predicted object

  float sum_confidence = 0.0;
  for (const auto & predicted_path : predicted_paths) {
    sum_confidence += predicted_path.confidence;
  }
  const float min_sum_confidence_value = 1e-3;
  sum_confidence = std::max(sum_confidence, min_sum_confidence_value);

  auto predicted_object = utils::convertToPredictedObject(transformed_object);

  for (auto & predicted_path : predicted_paths) {
    predicted_path.confidence = predicted_path.confidence / sum_confidence;
    if (predicted_object.kinematics.predicted_paths.size() >= 100) break;
    predicted_object.kinematics.predicted_paths.push_back(predicted_path);
  }
  return predicted_object;
}

std::optional<size_t> MapBasedPredictionNode::searchProperStartingRefPathIndex(
  const TrackedObject & object, const PosePath & pose_path) const
{
  std::unique_ptr<ScopedTimeTrack> st1_ptr;
  if (time_keeper_) st1_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  bool is_position_found = false;
  std::optional<size_t> opt_index{std::nullopt};
  auto & index = opt_index.emplace(0);

  // starting segment index is a segment close enough to the object
  const auto obj_point = object.kinematics.pose_with_covariance.pose.position;
  {
    std::unique_ptr<ScopedTimeTrack> st2_ptr;
    if (time_keeper_)
      st2_ptr = std::make_unique<ScopedTimeTrack>("find_close_segment_index", *time_keeper_);
    double min_dist_sq = std::numeric_limits<double>::max();
    constexpr double acceptable_dist_sq = 1.0;  // [m2]
    for (size_t i = 0; i < pose_path.size(); i++) {
      const double dx = pose_path.at(i).position.x - obj_point.x;
      const double dy = pose_path.at(i).position.y - obj_point.y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        index = i;
      }
      if (dist_sq < acceptable_dist_sq) {
        break;
      }
    }
  }

  // calculate score that object can reach the target path smoothly, and search the
  // starting segment index that have the best score
  size_t idx = 0;
  {  // find target segmentation index
    std::unique_ptr<ScopedTimeTrack> st3_ptr;
    if (time_keeper_)
      st3_ptr = std::make_unique<ScopedTimeTrack>("find_target_seg_index", *time_keeper_);

    constexpr double search_distance = 22.0;       // [m]
    constexpr double yaw_diff_limit = M_PI / 3.0;  // 60 degrees

    const double obj_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    const size_t search_segment_count =
      static_cast<size_t>(std::floor(search_distance / reference_path_resolution_));
    const size_t search_segment_num =
      std::min(search_segment_count, static_cast<size_t>(pose_path.size() - index));

    // search for the best score, which is the smallest
    double best_score = 1e9;  // initial value is large enough
    for (size_t i = 0; i < search_segment_num; ++i) {
      const auto & path_pose = pose_path.at(index + i);
      // yaw difference
      const double path_yaw = tf2::getYaw(path_pose.orientation);
      const double relative_path_yaw = autoware_utils::normalize_radian(path_yaw - obj_yaw);
      if (std::abs(relative_path_yaw) > yaw_diff_limit) {
        continue;
      }

      const double dx = path_pose.position.x - obj_point.x;
      const double dy = path_pose.position.y - obj_point.y;
      const double dx_cp = std::cos(obj_yaw) * dx + std::sin(obj_yaw) * dy;
      const double dy_cp = -std::sin(obj_yaw) * dx + std::cos(obj_yaw) * dy;
      const double neutral_yaw = std::atan2(dy_cp, dx_cp) * 2.0;
      const double delta_yaw = autoware_utils::normalize_radian(path_yaw - obj_yaw - neutral_yaw);
      if (std::abs(delta_yaw) > yaw_diff_limit) {
        continue;
      }

      // objective function score
      constexpr double weight_ratio = 0.01;
      double score = delta_yaw * delta_yaw + weight_ratio * neutral_yaw * neutral_yaw;
      constexpr double acceptable_score = 1e-3;

      if (score < best_score) {
        best_score = score;
        idx = i;
        is_position_found = true;
        if (score < acceptable_score) {
          // if the score is small enough, we can break the loop
          break;
        }
      }
    }
  }

  // update starting segment index
  index += idx;
  index = std::clamp(index, 0ul, pose_path.size() - 1);

  return is_position_found ? opt_index : std::nullopt;
}

}  // namespace autoware::map_based_prediction

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_based_prediction::MapBasedPredictionNode)
