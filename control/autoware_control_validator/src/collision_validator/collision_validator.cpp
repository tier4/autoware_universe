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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_validator
{
namespace
{
using collision_validator::CollisionCandidate;
using collision_validator::CollisionEvaluatorParam;
using collision_validator::PointCloudClusters;
using collision_validator::PointCloudParam;

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
  const auto & pointcloud = params.pointcloud;
  PointCloudParam out;
  out.roi_rear = pointcloud.roi.rear;
  out.roi_front_margin = pointcloud.roi.front_margin;
  out.roi_lateral = pointcloud.roi.lateral;
  out.z_margin_low = pointcloud.roi.z_margin_low;
  out.z_margin_high = pointcloud.roi.z_margin_high;
  out.max_voxels = static_cast<size_t>(std::max<int64_t>(pointcloud.max_voxels, 0));
  out.voxel_size = pointcloud.voxel_size;
  out.cluster_tolerance = pointcloud.cluster_tolerance;
  out.min_cluster_size = static_cast<size_t>(std::max<int64_t>(pointcloud.min_cluster_size, 0));
  return out;
}

CollisionEvaluatorParam to_collision_evaluator_param(
  const ::control_validator::Params::CollisionValidator & params)
{
  CollisionEvaluatorParam out;
  out.max_time = params.prediction_horizon;
  out.time_resolution = params.time_resolution;
  out.n_eval_object = static_cast<size_t>(std::max<int64_t>(params.max_evaluations.object, 0));
  out.n_eval_pointcloud =
    static_cast<size_t>(std::max<int64_t>(params.max_evaluations.pointcloud, 0));
  out.max_objects_stamp_gap_s = params.max_objects_stamp_gap_s;
  out.stop_velocity = params.stop_velocity;
  out.proximity_distance = params.proximity_distance;
  std::copy_n(params.braking.reaction_times.begin(), 3, out.reaction_times.begin());
  out.braking_deceleration = params.braking.deceleration;
  out.cos_threshold = params.collision_cos_threshold;
  out.rear_axle_ratio = params.vehicle_yaw.rear_axle_ratio;
  for (const auto & name : params.enabled_object_classes) {
    if (const auto label = label_from_name(name)) {
      out.enabled_labels.push_back(*label);
    }
  }
  return out;
}

uint8_t risk_level_of(const CollisionCandidate & candidate)
{
  const uint8_t near_level = candidate.near ? 1 : 0;
  const uint8_t collision_level =
    candidate.brake_level >= 1 ? static_cast<uint8_t>(candidate.brake_level + 1) : 0;
  return std::max(near_level, collision_level);
}
}  // namespace

CollisionValidator::CollisionValidator(
  const rclcpp::Logger & logger, const ::control_validator::Params & params,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: logger_(logger),
  time_keeper_(time_keeper),
  params_(params.collision_validator),
  pointcloud_preprocessor_(logger, to_pointcloud_param(params.collision_validator), time_keeper),
  collision_evaluator_(to_collision_evaluator_param(params.collision_validator))
{
}

void CollisionValidator::update_parameters(const ::control_validator::Params & params)
{
  params_ = params.collision_validator;
  pointcloud_preprocessor_.update_parameters(to_pointcloud_param(params_));
  collision_evaluator_.update_parameters(to_collision_evaluator_param(params_));
}

void CollisionValidator::update_history(const nav_msgs::msg::Odometry & odom)
{
  if (
    odometry_history_.empty() ||
    rclcpp::Time(odometry_history_.back().header.stamp) != rclcpp::Time(odom.header.stamp)) {
    odometry_history_.push_back(odom);
    while (odometry_history_.size() > kOdometryHistorySize) {
      odometry_history_.pop_front();
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

std::vector<HazardSource> CollisionValidator::to_hazard_sources(
  const std::vector<CollisionCandidate> & candidates, const nav_msgs::msg::Odometry & odom)
{
  std::vector<HazardSource> sources;
  for (const auto & candidate : candidates) {
    HazardSource source;
    source.shape = transform_polygon_to_base_link(candidate.polygon_now, odom.pose.pose);
    source.risk_level = risk_level_of(candidate);
    source.stamp = odom.header.stamp;
    sources.push_back(std::move(source));
  }
  return sources;
}

void CollisionValidator::validate(
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
  const nav_msgs::msg::Odometry & odom,
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud,
  HazardWarningSources & hazard_sources)
{
  if (!params_.enable_warning) {
    return;
  }
  update_history(odom);
  if (predicted_trajectory.points.size() < 2) {
    return;
  }

  const double horizon = std::min(
    params_.prediction_horizon,
    rclcpp::Duration(predicted_trajectory.points.back().time_from_start).seconds());

  const PointCloudClusters * clusters = nullptr;
  if (params_.enable_pointcloud && pointcloud) {
    const auto ego_pose_at_pointcloud_stamp =
      select_ego_pose_at(rclcpp::Time(pointcloud->header.stamp));
    try {
      const auto & result = pointcloud_preprocessor_.process(
        *pointcloud, ego_pose_at_pointcloud_stamp, odom.twist.twist.linear.x, horizon,
        vehicle_info);
      if (result) {
        clusters = &*result;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 5000, "hazard warning: pointcloud preprocessing skipped: %s",
        e.what());
    }
  }
  if (!objects && !clusters) {
    return;
  }

  std::vector<CollisionCandidate> candidates;
  try {
    autoware_utils_debug::ScopedTimeTrack st("collision_evaluator_evaluate", *time_keeper_);
    candidates = collision_evaluator_.evaluate(
      predicted_trajectory, odom, vehicle_info, objects.get(), clusters);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000, "hazard warning: collision evaluation skipped: %s", e.what());
    return;
  }

  hazard_sources.collision = to_hazard_sources(candidates, odom);
}
}  // namespace autoware::control_validator
