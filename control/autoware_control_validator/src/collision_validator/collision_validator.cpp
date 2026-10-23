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

#include "autoware/control_validator/collision_validator/collision_validator.hpp"

#include "autoware/control_validator/utils.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_validator
{
namespace
{
using autoware_control_validator::msg::HazardCandidate;
using autoware_control_validator::msg::HazardCandidateArray;
using collision_validator::CollisionCandidate;
using collision_validator::CollisionEvaluatorParam;
using collision_validator::ObjectMargin;
using collision_validator::Point2d;
using collision_validator::PointCloudParam;
using collision_validator::PointCluster;
using collision_validator::Polygon2d;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std::optional<uint8_t> label_from_name(const std::string & name)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  std::string upper(name);
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
  if (upper == "UNKNOWN") return ObjectClassification::UNKNOWN;
  if (upper == "CAR") return ObjectClassification::CAR;
  if (upper == "TRUCK") return ObjectClassification::TRUCK;
  if (upper == "BUS") return ObjectClassification::BUS;
  if (upper == "TRAILER") return ObjectClassification::TRAILER;
  if (upper == "MOTORCYCLE") return ObjectClassification::MOTORCYCLE;
  if (upper == "BICYCLE") return ObjectClassification::BICYCLE;
  if (upper == "PEDESTRIAN") return ObjectClassification::PEDESTRIAN;
  if (upper == "ANIMAL") return ObjectClassification::ANIMAL;
  if (upper == "HAZARD") return ObjectClassification::HAZARD;
  return std::nullopt;
}

PointCloudParam to_pointcloud_param(const ::control_validator::Params::CollisionValidator & params)
{
  const auto & p = params.pointcloud;
  PointCloudParam out;
  out.roi_rear = p.roi.rear;
  out.roi_front_margin = p.roi.front_margin;
  out.roi_lateral = p.roi.lateral;
  out.z_margin_low = p.roi.z_margin_low;
  out.z_margin_high = p.roi.z_margin_high;
  out.max_voxels = static_cast<size_t>(std::max<int64_t>(p.max_voxels, 0));
  out.voxel_size = p.voxel_size;
  out.cluster_tolerance = p.cluster_tolerance;
  out.min_cluster_size = static_cast<size_t>(std::max<int64_t>(p.min_cluster_size, 0));
  return out;
}

CollisionEvaluatorParam to_collision_evaluator_param(
  const ::control_validator::Params::CollisionValidator & params)
{
  const auto & p = params;
  CollisionEvaluatorParam out;
  out.max_time = p.prediction_horizon;
  out.time_resolution = p.time_resolution;
  out.n_eval_object = static_cast<size_t>(std::max<int64_t>(p.max_evaluations.object, 0));
  out.n_eval_pointcloud = static_cast<size_t>(std::max<int64_t>(p.max_evaluations.pointcloud, 0));
  out.enable_pointcloud = p.enable_pointcloud;
  out.max_objects_stamp_gap_s = p.max_objects_stamp_gap_s;
  out.stop_velocity = p.stop_velocity;
  for (size_t i = 0; i < out.reaction_times.size() && i < p.braking.reaction_times.size(); ++i) {
    out.reaction_times[i] = p.braking.reaction_times[i];
  }
  out.braking_deceleration = p.braking.deceleration;
  out.cos_threshold = p.collision_cos_threshold;
  out.rear_axle_ratio = p.vehicle_yaw.rear_axle_ratio;
  for (const auto & name : p.enabled_object_classes) {
    if (const auto label = label_from_name(name)) {
      out.enabled_labels.push_back(*label);
    }
  }
  const auto & m = p.pointcloud.object_margin;
  out.object_margin = ObjectMargin{m.front, m.rear, m.left, m.right};
  return out;
}

std_msgs::msg::ColorRGBA make_color(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::msg::ColorRGBA level_color(const uint8_t level)
{
  switch (level) {
    case 0:
      return make_color(0.3F, 0.8F, 0.3F, 0.8F);
    case 1:
      return make_color(1.0F, 1.0F, 0.0F, 0.9F);
    case 2:
      return make_color(1.0F, 0.5F, 0.0F, 0.9F);
    default:
      return make_color(1.0F, 0.0F, 0.0F, 0.9F);
  }
}

Marker make_polygon_marker(
  const Polygon2d & polygon, const std::string & ns, const int32_t id, const double z,
  const std_msgs::msg::ColorRGBA & color, const builtin_interfaces::msg::Time & stamp)
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.color = color;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  for (const auto & pt : polygon.outer()) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = z;
    marker.points.push_back(p);
  }
  return marker;
}
}  // namespace

CollisionValidator::CollisionValidator(
  const rclcpp::Logger & logger, const ::control_validator::Params & params, rclcpp::Node * node,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: logger_(logger),
  time_keeper_(time_keeper),
  params_(params.collision_validator),
  pointcloud_preprocessor_(logger, to_pointcloud_param(params.collision_validator), time_keeper),
  collision_evaluator_(to_collision_evaluator_param(params.collision_validator))
{
  update_parameters(params);
  if (node) {
    pub_collision_markers_ =
      node->create_publisher<MarkerArray>("~/debug/hazard/collision/markers", 1);
    pub_hazard_candidates_ =
      node->create_publisher<HazardCandidateArray>("~/debug/hazard/candidates", 1);
  }
}

void CollisionValidator::update_parameters(const ::control_validator::Params & params)
{
  params_ = params.collision_validator;
  pointcloud_preprocessor_.update_parameters(to_pointcloud_param(params_));
  collision_evaluator_.update_parameters(to_collision_evaluator_param(params_));
}

bool CollisionValidator::is_valid_vehicle_info(const vehicle_info_utils::VehicleInfo & vehicle_info)
{
  return vehicle_info.vehicle_height_m > 0.0 && vehicle_info.vehicle_width_m > 0.0 &&
         vehicle_info.vehicle_length_m > 0.0;
}

void CollisionValidator::update_history(
  const nav_msgs::msg::Odometry & odom,
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects)
{
  if (
    odometry_history_.empty() ||
    rclcpp::Time(odometry_history_.back().header.stamp) != rclcpp::Time(odom.header.stamp)) {
    odometry_history_.push_back(odom);
    while (odometry_history_.size() > kOdometryHistorySize) {
      odometry_history_.pop_front();
    }
  }
  if (
    objects && (objects_history_.empty() || rclcpp::Time(objects_history_.back()->header.stamp) !=
                                              rclcpp::Time(objects->header.stamp))) {
    objects_history_.push_back(objects);
    while (objects_history_.size() > kObjectsHistorySize) {
      objects_history_.pop_front();
    }
  }
}

std::optional<geometry_msgs::msg::Pose> CollisionValidator::select_ego_pose_at(
  const rclcpp::Time & stamp) const
{
  const nav_msgs::msg::Odometry * best = nullptr;
  double best_gap = std::numeric_limits<double>::infinity();
  for (const auto & odom : odometry_history_) {
    const double gap = std::abs((rclcpp::Time(odom.header.stamp) - stamp).seconds());
    if (gap < best_gap) {
      best_gap = gap;
      best = &odom;
    }
  }
  if (!best || best_gap > params_.pointcloud.max_stamp_gap_s) {
    return std::nullopt;
  }
  return best->pose.pose;
}

autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr
CollisionValidator::select_objects_at(const rclcpp::Time & stamp) const
{
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr best;
  double best_gap = std::numeric_limits<double>::infinity();
  for (const auto & objects : objects_history_) {
    const double gap = std::abs((rclcpp::Time(objects->header.stamp) - stamp).seconds());
    if (gap < best_gap) {
      best_gap = gap;
      best = objects;
    }
  }
  if (!best || best_gap > params_.objects.max_stamp_gap_s) {
    return nullptr;
  }
  return best;
}

std::vector<HazardSource> CollisionValidator::to_hazard_sources(
  const std::vector<CollisionCandidate> & candidates, const nav_msgs::msg::Odometry & odom)
{
  std::vector<HazardSource> sources;
  for (const auto & candidate : candidates) {
    if (candidate.brake_level < 1) {
      continue;
    }
    HazardSource source;
    source.shape = transform_polygon_to_base_link(candidate.polygon_now, odom.pose.pose);
    if (candidate.kind == CollisionCandidate::Kind::POINTCLOUD && !source.shape.outer().empty()) {
      source.shape.outer().resize(1);
    }
    source.risk_level = candidate.brake_level;
    sources.push_back(std::move(source));
  }
  return sources;
}

void CollisionValidator::validate(
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
  const nav_msgs::msg::Odometry & odom,
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud, const rclcpp::Time & now,
  HazardWarningSources & hazard_sources)
{
  update_history(odom, objects);
  if (
    predicted_trajectory.points.size() < 2 || objects_history_.empty() ||
    (params_.enable_pointcloud && !pointcloud)) {
    return;
  }
  if (!is_valid_vehicle_info(vehicle_info)) {
    RCLCPP_ERROR_THROTTLE(
      logger_, steady_clock_, 5000,
      "hazard warning: invalid vehicle_info (height, width or length <= 0). Collision warning "
      "skipped.");
    return;
  }

  const double horizon = std::min(
    params_.prediction_horizon,
    rclcpp::Duration(predicted_trajectory.points.back().time_from_start).seconds());

  const std::vector<PointCluster> * clusters = nullptr;
  builtin_interfaces::msg::Time pointcloud_stamp;
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_at_pointcloud_stamp;
  if (params_.enable_pointcloud) {
    const auto ego_pose_at_pointcloud_stamp =
      select_ego_pose_at(rclcpp::Time(pointcloud->header.stamp));
    const auto & result = pointcloud_preprocessor_.process(
      *pointcloud, ego_pose_at_pointcloud_stamp, odom.twist.twist.linear.x, horizon, vehicle_info);
    if (result) {
      clusters = &result->clusters;
      pointcloud_stamp = result->stamp;
      objects_at_pointcloud_stamp = select_objects_at(result->stamp);
    }
  }

  const std::vector<PointCluster> no_clusters;
  std::vector<CollisionCandidate> candidates;
  try {
    autoware_utils_debug::ScopedTimeTrack st("collision_evaluator_evaluate", *time_keeper_);
    candidates = collision_evaluator_.evaluate(
      predicted_trajectory, odom, vehicle_info, *objects_history_.back(),
      objects_at_pointcloud_stamp.get(), clusters ? *clusters : no_clusters, pointcloud_stamp);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000, "hazard warning: collision evaluation skipped: %s", e.what());
    return;
  }

  hazard_sources.collision = to_hazard_sources(candidates, odom);
  time_keeper_->start_track("collision_validator_publish_debug");
  publish_debug(candidates, odom, now);
  time_keeper_->end_track("collision_validator_publish_debug");
}

void CollisionValidator::publish_debug(
  const std::vector<CollisionCandidate> & candidates, const nav_msgs::msg::Odometry & odom,
  const rclcpp::Time & now) const
{
  if (pub_hazard_candidates_ && pub_hazard_candidates_->get_subscription_count() > 0) {
    HazardCandidateArray array;
    array.header.stamp = now;
    array.header.frame_id = "map";
    array.candidates.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      HazardCandidate out;
      out.source_stamp = candidate.source_stamp;
      out.kind = candidate.kind == CollisionCandidate::Kind::OBJECT
                   ? HazardCandidate::KIND_OBJECT
                   : HazardCandidate::KIND_POINTCLOUD;
      out.label = candidate.label;
      out.ttc_s = candidate.ttc ? static_cast<float>(*candidate.ttc)
                                : std::numeric_limits<float>::quiet_NaN();
      out.cos_collision = candidate.cos_collision ? static_cast<float>(*candidate.cos_collision)
                                                  : std::numeric_limits<float>::quiet_NaN();
      out.level = candidate.brake_level;
      array.candidates.push_back(std::move(out));
    }
    pub_hazard_candidates_->publish(array);
  }

  if (
    !params_.debug.publish_markers || !pub_collision_markers_ ||
    pub_collision_markers_->get_subscription_count() == 0) {
    return;
  }

  std::map<size_t, size_t> min_ttc_point_of_cluster;
  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates[i];
    if (
      candidate.kind != CollisionCandidate::Kind::POINTCLOUD || !candidate.ttc ||
      !candidate.cluster_index) {
      continue;
    }
    const auto [it, inserted] = min_ttc_point_of_cluster.try_emplace(*candidate.cluster_index, i);
    if (!inserted && *candidate.ttc < *candidates[it->second].ttc) {
      it->second = i;
    }
  }
  std::vector<bool> draw_point(candidates.size(), false);
  for (const auto & [cluster_index, i] : min_ttc_point_of_cluster) {
    draw_point[i] = true;
  }

  MarkerArray markers;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  markers.markers.push_back(delete_all);
  const double z = odom.pose.pose.position.z;
  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates[i];
    const auto color = level_color(candidate.brake_level);
    const auto & stamp = candidate.source_stamp;
    const bool is_point = candidate.kind == CollisionCandidate::Kind::POINTCLOUD;
    const auto id = static_cast<int32_t>(i);
    if (is_point && !draw_point[i]) {
      continue;
    }

    if (is_point) {
      Marker sphere;
      sphere.header.frame_id = "map";
      sphere.header.stamp = stamp;
      sphere.ns = "hazard_hit_points";
      sphere.id = id;
      sphere.type = Marker::SPHERE;
      sphere.action = Marker::ADD;
      sphere.pose.position.x = candidate.polygon_now.outer().front().x();
      sphere.pose.position.y = candidate.polygon_now.outer().front().y();
      sphere.pose.position.z = z + 0.1;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.3;
      sphere.scale.y = 0.3;
      sphere.scale.z = 0.3;
      sphere.color = color;
      sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
      markers.markers.push_back(sphere);
    } else {
      markers.markers.push_back(
        make_polygon_marker(candidate.polygon_now, "hazard_objects", id, z + 0.1, color, stamp));
    }

    if (candidate.ttc.has_value()) {
      Marker text;
      text.header.frame_id = "map";
      text.header.stamp = stamp;
      text.ns = "hazard_ttc";
      text.id = id;
      text.type = Marker::TEXT_VIEW_FACING;
      text.action = Marker::ADD;
      Point2d centroid = candidate.polygon_now.outer().front();
      if (!is_point) {
        boost::geometry::centroid(candidate.polygon_now, centroid);
      }
      text.pose.position.x = centroid.x();
      text.pose.position.y = centroid.y();
      text.pose.position.z = z + 1.0;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.5;
      text.color = color;
      text.lifetime = rclcpp::Duration::from_seconds(0.5);
      char buffer[48];
      std::snprintf(
        buffer, sizeof(buffer), "ttc %.1f cos %.2f", *candidate.ttc,
        candidate.cos_collision.value_or(std::numeric_limits<double>::quiet_NaN()));
      text.text = buffer;
      markers.markers.push_back(text);
    }
  }
  pub_collision_markers_->publish(markers);
}
}  // namespace autoware::control_validator
