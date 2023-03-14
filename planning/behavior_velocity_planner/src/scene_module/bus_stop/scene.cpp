// Copyright 2020 Tier IV, Inc.
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

#include "tier4_autoware_utils/geometry/pose_deviation.hpp"
#include "utilization/arc_lane_util.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <scene_module/bus_stop/scene.hpp>
#include <utilization/util.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bus_stop
{
namespace bg = boost::geometry;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using tier4_autoware_utils::calcLongitudinalDeviation;

namespace
{
lanelet::BasicPoint3d getCentroidPoint(const lanelet::BasicPolygon3d & poly)
{
  lanelet::BasicPoint3d p_sum{0.0, 0.0, 0.0};
  for (const auto & p : poly) {
    p_sum += p;
  }
  return p_sum / poly.size();
}

geometry_msgs::msg::Point toROSMsg(const lanelet::BasicPoint3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

visualization_msgs::msg::MarkerArray createCorrespondenceMarkerArray(
  const lanelet::autoware::BusStop & bus_stop_reg_elem, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;

  const lanelet::ConstLineString3d stop_line = bus_stop_reg_elem.stopLine();
  const auto stop_line_center_point =
    (stop_line.front().basicPoint() + stop_line.back().basicPoint()) / 2;

  // ID
  {
    auto marker = createDefaultMarker(
      "map", now, "bus_stop_id", bus_stop_reg_elem.id(),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & bus_stop : bus_stop_reg_elem.busStops()) {
      const auto poly = bus_stop.basicPolygon();

      marker.pose.position = toROSMsg(poly.front());
      marker.pose.position.z += 2.0;
      marker.text = std::to_string(bus_stop_reg_elem.id());

      msg.markers.push_back(marker);
    }
  }

  // Polygon
  {
    auto marker = createDefaultMarker(
      "map", now, "bus_stop_polygon", bus_stop_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(0.1, 0.1, 1.0, 0.500));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & bus_stop : bus_stop_reg_elem.busStops()) {
      const auto poly = bus_stop.basicPolygon();

      for (size_t i = 0; i < poly.size(); ++i) {
        const auto idx_front = i;
        const auto idx_back = (i == poly.size() - 1) ? 0 : i + 1;

        const auto & p_front = poly.at(idx_front);
        const auto & p_back = poly.at(idx_back);

        marker.points.push_back(toROSMsg(p_front));
        marker.points.push_back(toROSMsg(p_back));
      }
    }

    msg.markers.push_back(marker);
  }

  // Polygon to StopLine
  {
    auto marker = createDefaultMarker(
      "map", now, "bus_stop_correspondence", bus_stop_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(0.1, 0.1, 1.0, 0.500));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & bus_stop : bus_stop_reg_elem.busStops()) {
      const auto poly = bus_stop.basicPolygon();
      const auto centroid_point = getCentroidPoint(poly);
      for (size_t i = 0; i < poly.size(); ++i) {
        marker.points.push_back(toROSMsg(centroid_point));
        marker.points.push_back(toROSMsg(stop_line_center_point));
      }
    }

    msg.markers.push_back(marker);
  }

  return msg;
}

// calc smallest enclosing circle with average O(N) algorithm
// reference:
// https://erickimphotography.com/blog/wp-content/uploads/2018/09/Computational-Geometry-Algorithms-and-Applications-3rd-Ed.pdf
std::pair<lanelet::BasicPoint2d, double> calcSmallestEnclosingCircle(
  const lanelet::ConstPolygon2d & poly)
{
  // The `eps` is used to avoid precision bugs in circle inclusion checkings.
  // If the value of `eps` is too small, this function doesn't work well. More than 1e-10 is
  // recommended.
  const double eps = 1e-5;
  lanelet::BasicPoint2d center(0.0, 0.0);
  double radius_squared = 0.0;

  auto cross = [](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> double {
    return p1.x() * p2.y() - p1.y() * p2.x();
  };

  auto make_circle_3 = [&](
                         const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2,
                         const lanelet::BasicPoint2d & p3) -> void {
    // reference for circumcenter vector https://en.wikipedia.org/wiki/Circumscribed_circle
    const double A = (p2 - p3).squaredNorm();
    const double B = (p3 - p1).squaredNorm();
    const double C = (p1 - p2).squaredNorm();
    const double S = cross(p2 - p1, p3 - p1);
    if (std::abs(S) < eps) return;
    center = (A * (B + C - A) * p1 + B * (C + A - B) * p2 + C * (A + B - C) * p3) / (4 * S * S);
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto make_circle_2 =
    [&](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> void {
    center = (p1 + p2) * 0.5;
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto in_circle = [&](const lanelet::BasicPoint2d & p) -> bool {
    return (center - p).squaredNorm() <= radius_squared;
  };

  // mini disc
  for (size_t i = 1; i < poly.size(); i++) {
    const auto p1 = poly[i].basicPoint2d();
    if (in_circle(p1)) continue;

    // mini disc with point
    const auto p0 = poly[0].basicPoint2d();
    make_circle_2(p0, p1);
    for (size_t j = 0; j < i; j++) {
      const auto p2 = poly[j].basicPoint2d();
      if (in_circle(p2)) continue;

      // mini disc with two points
      make_circle_2(p1, p2);
      for (size_t k = 0; k < j; k++) {
        const auto p3 = poly[k].basicPoint2d();
        if (in_circle(p3)) continue;

        // mini disc with tree points
        make_circle_3(p1, p2, p3);
      }
    }
  }

  return std::make_pair(center, radius_squared);
}

std::vector<geometry_msgs::msg::Point> findPointsWithinPolygons(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const lanelet::ConstPolygons3d & polys)
{
  std::vector<geometry_msgs::msg::Point> obstacle_points;

  for (const auto & poly : polys) {
    const auto poly_2d = lanelet::utils::to2D(poly);
    const auto circle = calcSmallestEnclosingCircle(poly_2d);
    for (const auto p : input_points) {
      const double squared_dist = (circle.first.x() - p.x) * (circle.first.x() - p.x) +
                                  (circle.first.y() - p.y) * (circle.first.y() - p.y);
      if (squared_dist <= circle.second) {
        if (bg::within(Point2d{p.x, p.y}, poly_2d.basicPolygon())) {
          obstacle_points.push_back(planning_utils::toRosPoint(p));
        }
      }
    }
  }

  return obstacle_points;
}

// find longitudinal nearest point from base_pose
std::pair<geometry_msgs::msg::Point, double> findLongitudinalForwardPoint(
  const std::vector<geometry_msgs::msg::Point> & input_points,
  const geometry_msgs::msg::Pose & base_pose)
{
  double max_dist = -std::numeric_limits<double>::max();
  geometry_msgs::msg::Point forward_point;

  for (const auto & p : input_points) {
    const auto longitudinal_deviation = calcLongitudinalDeviation(base_pose, p);

    if (longitudinal_deviation > max_dist) {
      max_dist = longitudinal_deviation;
      forward_point = p;
    }
  }

  return {forward_point, max_dist};
}

// p1 is older data
double calcPredictedVelocityFromTwoPoint(
  const BusStopModule::PointWithDistStamped & p1, const BusStopModule::PointWithDistStamped & p2)
{
  const double dist_diff = p2.dist - p1.dist;
  const double time_diff = (p2.stamp - p1.stamp).seconds();
  const double vel_mps = dist_diff / time_diff;
  return vel_mps;
}

BusStopModule::PointWithDistStamped createPointWithDist(
  const geometry_msgs::msg::Point & point, const double dist, std::uint64_t stamp_pcl)
{
  BusStopModule::PointWithDistStamped point_with_dist;
  point_with_dist.point = point;
  point_with_dist.dist = dist;
  pcl_conversions::fromPCL(stamp_pcl, point_with_dist.stamp);

  return point_with_dist;
}

std::string toStringState(const StateMachine::State & state)
{
  switch (state) {
    case StateMachine::State::GO:
      return "GO";

    case StateMachine::State::STOP:
      return "STOP";

    case StateMachine::State::READY:
      return "READY";

    default:
      return "UNKNOWN";
  }
}

template <class T>
void pushDataToBuffer(const T & data, const size_t max_size, std::deque<T> & buffer)
{
  buffer.emplace_back(data);
  if (buffer.size() > max_size) {
    buffer.pop_front();
  }
}
}  // namespace

BusStopModule::BusStopModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::BusStop & bus_stop_reg_elem, const PlannerParam & planner_param,
  rclcpp::Node & node, const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  bus_stop_reg_elem_(bus_stop_reg_elem),
  planner_param_(planner_param)
{
  turn_indicator_ = std::make_shared<TurnIndicator>(node);
  state_machine_ = std::make_shared<StateMachine>(node, planner_param.state_param);
  lpf_ = std::make_shared<LowpassFilter1d>(planner_param.lpf_gain);
  debug_data_ = std::make_shared<DebugData>(node);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("debug"), "lpf_gain: " << planner_param_.lpf_gain);

  //! debug
  // change log level for debugging
  const auto result = rcutils_logging_set_logger_level("debug", RCUTILS_LOG_SEVERITY_DEBUG);
  if (result == RCUTILS_RET_ERROR) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "failed to set logger level.");
  }
}

bool BusStopModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  // Store original path
  const auto original_path = *path;

  // Reset debug data
  debug_data_->clearDebugData();
  if (!debug_data_->base_link2front) {
    debug_data_->base_link2front =
      std::make_shared<double>(planner_data_->vehicle_info_.max_longitudinal_offset_m);
  }

  // Find obstacles in the bus stop area
  const auto obstacle_points =
    findPointsWithinPolygons(*planner_data_->no_ground_pointcloud, bus_stop_reg_elem_.busStops());

  // find longitudinal nearest point behind the ego vehicle
  // use the first path point as base pose
  const auto nearest_point_with_dist =
    findLongitudinalForwardPoint(obstacle_points, path->points.at(0).point.pose);

  // update buffer and calculate predicted velocity from nearest obstacle point
  PointWithDistStamped point_with_dist = createPointWithDist(
    nearest_point_with_dist.first, nearest_point_with_dist.second,
    planner_data_->no_ground_pointcloud->header.stamp);
  updatePointsBuffer(point_with_dist);

  // judge the condition to decide the state
  const bool is_safe_velocity = judgeSafetyFromObstacleVelocity(velocity_buffer_lpf_);
  const bool is_obstacle_on_the_side = judgeIfObstacleOnTheSide(points_buffer_);

  // update and current state
  state_machine_->updateState({is_safe_velocity, is_obstacle_on_the_side}, *clock_);
  const auto current_state = state_machine_->getCurrentState();

  //! debug
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("debug"), "obstacle points size: " << obstacle_points.size());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug"), "longitudinal dist: " << point_with_dist.dist);
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("debug"), "current_state: " << toStringState(current_state));
  if (!velocity_buffer_.empty() && !velocity_buffer_lpf_.empty()) {
    debug_data_->pushPredictedVelKmph(velocity_buffer_.back() * 3.6);
    debug_data_->pushPredictedVelLpfKmph(velocity_buffer_lpf_.back() * 3.6);
    debug_data_->publishDebugValue();
  }
  debug_data_->nearest_point = nearest_point_with_dist.first;
  debug_data_->is_safe_velocity = is_safe_velocity;
  debug_data_->is_obstacle_on_the_side = is_obstacle_on_the_side;
  if (is_obstacle_on_the_side) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug"), "obstacle is on the side of the vehicle");
  }

  // calculate stop point for the stop line
  const auto path_idx_with_pose = calcStopPoint(*path);
  if (!path_idx_with_pose) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "failed to calculate stop point.");
    return true;
  }
  const auto stop_dist = calcStopDistance(*path, *path_idx_with_pose);
  const auto & stop_pose = path_idx_with_pose->second;
  const auto & base_link_pose = planner_data_->current_pose.pose;

  if (current_state == State::STOP) {
    // if RTC is activated, do not insert zero velocity and go
    if (isRTCActivated(stop_dist, false)) {
      return true;
    }

    planning_utils::insertStopPoint(stop_pose.position, *path);

    // create virtual wall at the head of the ego vehicle
    debug_data_->stop_poses.push_back(base_link_pose);

    return true;
  }

  if (current_state == State::READY) {
    // if RTC is activated, do not insert zero velocity and go
    if (isRTCActivated(stop_dist, false)) {
      return true;
    }

    planning_utils::insertStopPoint(stop_pose.position, *path);

    // create virtual wall at the head of the ego vehicle
    debug_data_->stop_poses.push_back(base_link_pose);

    // Set Turn Indicator
    turn_indicator_->setTurnSignal(TurnIndicatorsCommand::ENABLE_RIGHT, clock_->now());
    turn_indicator_->publish();

    return true;
  }

  if (current_state == State::GO) {
    // if RTC is activated, do not insert zero velocity and go
    if (isRTCActivated(stop_dist, true)) {
      return true;
    }

    planning_utils::insertStopPoint(stop_pose.position, *path);

    // create virtual wall at the head of the ego vehicle
    debug_data_->stop_poses.push_back(base_link_pose);

    return true;
  }

  return true;
}

LineString2d BusStopModule::getStopLineGeometry2d() const
{
  const lanelet::ConstLineString3d stop_line = bus_stop_reg_elem_.stopLine();
  return planning_utils::extendLine(
    stop_line[0], stop_line[1], planner_data_->stop_line_extend_length);
}

boost::optional<PathIndexWithPose> BusStopModule::calcStopPoint(const PathWithLaneId & path) const
{
  // Get stop line geometry
  const auto stop_line = getStopLineGeometry2d();

  // Get stop point
  // In this module, the ego vehicle want to keep stopping at the first stop position
  // set the stop_margin so that the ego vehicle don't approach the stop line
  const double stop_margin = 3.0;
  const auto stop_point = arc_lane_utils::createTargetPoint(
    path, stop_line, lane_id_, stop_margin, planner_data_->vehicle_info_.max_longitudinal_offset_m);

  return stop_point;
}

double BusStopModule::calcStopDistance(
  const PathWithLaneId & path, const PathIndexWithPose & stop_point)
{
  const auto & stop_point_idx = stop_point.first;
  const auto & stop_pose = stop_point.second;
  const size_t stop_line_seg_idx =
    planning_utils::calcSegmentIndexFromPointIndex(path.points, stop_pose.position, stop_point_idx);

  const size_t current_seg_idx = findEgoSegmentIndex(path.points);
  const auto stop_dist = calcSignedArcLength(
    path.points, planner_data_->current_pose.pose.position, current_seg_idx, stop_pose.position,
    stop_line_seg_idx);

  return stop_dist;
}

void BusStopModule::updatePointsBuffer(const BusStopModule::PointWithDistStamped & point_with_dist)
{
  // if there are no obstacles in detection area, clear buffer
  // push 0.0 to velocity buffer so we know it is safe
  const double dist_thresh = 10000;
  if (std::abs(point_with_dist.dist) > dist_thresh) {
    points_buffer_.clear();
    pushDataToBuffer(0.0, planner_param_.buffer_size, velocity_buffer_);
    pushDataToBuffer(0.0, planner_param_.buffer_size, velocity_buffer_lpf_);
    return;
  }

  // if previously there are no obstacles and there are the obstacle at current cycle,
  // clear buffer to predict velocity precisely
  if (points_buffer_.empty()) {
    velocity_buffer_.clear();
    velocity_buffer_lpf_.clear();
  }

  points_buffer_.emplace_back(point_with_dist);
  if (points_buffer_.size() > planner_param_.buffer_size) {
    points_buffer_.pop_front();
  }

  // not enough data
  if (points_buffer_.size() < 2) {
    return;
  }

  const auto points_buffer_size = points_buffer_.size();
  const auto & p1 = points_buffer_.at(points_buffer_size - 2);
  const auto & p2 = points_buffer_.at(points_buffer_size - 1);

  // if subscribed points are the same data as previous one,
  // skip calculating predicted velocity and use previous one
  if (p1.stamp == p2.stamp) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug"), "same points are subscribed");
    return;
  }

  const double predicted_vel = calcPredictedVelocityFromTwoPoint(p1, p2);
  pushDataToBuffer(predicted_vel, planner_param_.buffer_size, velocity_buffer_);

  const auto predicted_vel_lpf = lpf_->filter(predicted_vel);
  pushDataToBuffer(predicted_vel_lpf, planner_param_.buffer_size, velocity_buffer_lpf_);
}

bool BusStopModule::judgeSafetyFromObstacleVelocity(const std::deque<double> & velocity_buffer)
{
  if (velocity_buffer.empty()) {
    return false;
  }

  // count the number of velocity which is less than threshold
  const auto safe_vel_mps = planner_param_.safe_obstacle_vel_threshold_kmph / 3.6;
  const size_t safe_vel_count = std::count_if(
    velocity_buffer.cbegin(), velocity_buffer.cend(),
    [safe_vel_mps](const double velocity) -> bool { return velocity < safe_vel_mps; });
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug"), "safe_vel_count: " << safe_vel_count);
  debug_data_->pushSafeVelCount(safe_vel_count);

  if (safe_vel_count >= planner_param_.num_safe_vel_threshold) {
    return true;
  }

  return false;
}

bool BusStopModule::judgeIfObstacleOnTheSide(const std::deque<PointWithDistStamped> & points_buffer)
{
  // there are no obstacles
  if (points_buffer_.empty()) {
    return false;
  }

  const auto & latest_detected_point = points_buffer.back().point;
  const auto & base_link_pose = planner_data_->current_pose.pose;
  const auto ego_rear_pose = tier4_autoware_utils::calcOffsetPose(
    base_link_pose, -planner_data_->vehicle_info_.rear_overhang_m, 0, 0);
  const auto longitudinal_deviation_from_ego_rear =
    tier4_autoware_utils::calcLongitudinalDeviation(ego_rear_pose, latest_detected_point);

  // if the value is positive, that means latest_detected_point is ahead of the ego_rear_pose
  const bool target_is_on_the_side = longitudinal_deviation_from_ego_rear > 0;
  return target_is_on_the_side;
}

bool BusStopModule::isRTCActivated(const double stop_distance, const bool safe)
{
  setDistance(stop_distance);
  setSafe(safe);
  return isActivated();
}

visualization_msgs::msg::MarkerArray BusStopModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker;
  const rclcpp::Time now = clock_->now();

  if (!debug_data_->stop_poses.empty()) {
    appendMarkerArray(createCorrespondenceMarkerArray(bus_stop_reg_elem_, now), &debug_marker, now);

    const auto marker_color = [&] {
      if (!debug_data_->is_safe_velocity || debug_data_->is_obstacle_on_the_side) {
        // RED
        return createMarkerColor(1.0, 0, 0, 0.999);
      } else {
        // YELLOW
        return createMarkerColor(1.0, 1.0, 0, 0.999);
      }
    };

    auto nearest_obstacle_marker = createDefaultMarker(
      "map", now, "nearest_obstacle", module_id_, visualization_msgs::msg::Marker::SPHERE,
      createMarkerScale(0.6, 0.6, 0.6), marker_color());
    nearest_obstacle_marker.pose.position = debug_data_->nearest_point;
    nearest_obstacle_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    debug_marker.markers.emplace_back(nearest_obstacle_marker);
  }

  return debug_marker;
}

visualization_msgs::msg::MarkerArray BusStopModule::createVirtualWallMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;

  const rclcpp::Time now = clock_->now();

  auto id = getModuleId();
  for (const auto & p : debug_data_->stop_poses) {
    const auto p_front =
      tier4_autoware_utils::calcOffsetPose(p, *debug_data_->base_link2front, 0.0, 0.0);
    appendMarkerArray(
      motion_utils::createStopVirtualWallMarker(p_front, "bus_stop", now, id++), &wall_marker, now);
  }

  return wall_marker;
}

}  // namespace bus_stop
}  // namespace behavior_velocity_planner
