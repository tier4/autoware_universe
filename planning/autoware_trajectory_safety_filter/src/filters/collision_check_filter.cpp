// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory_safety_filter/filters/collision_check_filter.hpp"

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <any>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{

namespace motion
{
MotionProfile compute_motion_profile_1d(
  const geometry_msgs::msg::Twist & initial_twist, double assumed_lag, double assumed_acceleration,
  double max_time)
{
  MotionProfile profile;
  const double initial_velocity = std::hypot(initial_twist.linear.x, initial_twist.linear.y);

  if (initial_velocity <= 0.0) {
    profile.times.push_back(0.0);
    profile.distances.push_back(0.0);
    return profile;
  }

  const size_t estimated_size = static_cast<size_t>(max_time / TIME_RESOLUTION) + 1;
  profile.times.reserve(estimated_size);
  profile.distances.reserve(estimated_size);

  for (double t = 0.0; t < max_time; t += TIME_RESOLUTION) {
    profile.times.push_back(t);

    if (t < assumed_lag) {
      profile.distances.push_back(initial_velocity * t);
    } else {
      const double lag_distance = initial_velocity * assumed_lag;
      const double time_after_lag = t - assumed_lag;
      const double current_velocity = initial_velocity + assumed_acceleration * time_after_lag;

      if (current_velocity <= 0.0) {
        const double time_to_stop = initial_velocity / -assumed_acceleration;
        const double stop_distance = lag_distance + (initial_velocity * time_to_stop) +
                                     (0.5 * assumed_acceleration * time_to_stop * time_to_stop);
        profile.distances.push_back(stop_distance);
        break;
      }

      const double distance = lag_distance + (initial_velocity * time_after_lag) +
                              (0.5 * assumed_acceleration * time_after_lag * time_after_lag);
      profile.distances.push_back(distance);
    }
  }

  return profile;
}

}  // namespace motion

namespace constant_curvature_predictor
{
struct TwistPerDistance
{
  Eigen::Vector2d linear;
  double angular;
};

namespace
{

// --- ヘルパー関数群 ---
// ① 初期PoseからIsometry(SE2)への変換
Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
  iso.translation() << pose.position.x, pose.position.y;
  iso.linear() = Eigen::Rotation2Dd(tf2::getYaw(pose.orientation)).toRotationMatrix();
  return iso;
}

// ② 時間あたりのTwistを、単位道のりあたりのTwist(TwistPerDistance)へ変換
TwistPerDistance compute_twist_per_distance(const geometry_msgs::msg::Twist & twist)
{
  const Eigen::Vector2d vel(twist.linear.x, twist.linear.y);
  const double linear_speed = vel.norm();
  const double inv_speed = (linear_speed > 1e-6) ? (1.0 / linear_speed) : 0.0;

  return {vel * inv_speed, twist.angular.z * inv_speed};
}

// ③ 単位道のりあたりのTwistと移動距離から、変分Isometry(SE2)を生成（指数写像）
Eigen::Isometry2d compute_delta_isometry(const TwistPerDistance & twist_per_dist, double distance)
{
  Eigen::Isometry2d delta_iso = Eigen::Isometry2d::Identity();
  const double theta = twist_per_dist.angular * distance;

  delta_iso.rotate(Eigen::Rotation2Dd(theta));

  double a, b;
  if (std::abs(theta) < 1e-4) {
    const double theta_sq = theta * theta;
    a = 1.0 - theta_sq / 6.0;
    b = theta / 2.0 - (theta_sq * theta) / 24.0;
  } else {
    a = std::sin(theta) / theta;
    b = (1.0 - std::cos(theta)) / theta;
  }

  Eigen::Matrix2d V;
  V << a, -b, b, a;
  delta_iso.translation() = V * (twist_per_dist.linear * distance);

  return delta_iso;
}

// ④ Isometry(SE2)からPoseへの変換（Z座標は初期値を維持）
geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry2d & iso, double initial_z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = iso.translation().x();
  pose.position.y = iso.translation().y();
  pose.position.z = initial_z;

  const double yaw = Eigen::Rotation2Dd(iso.linear()).angle();
  pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  return pose;
}

}  // namespace

PoseTrajectory compute(
  const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Twist & initial_twist,
  const TravelDistanceTrajectory & distance_trajectory)
{
  const Eigen::Isometry2d start_iso = pose_to_isometry(initial_pose);
  const TwistPerDistance twist_per_dist = compute_twist_per_distance(initial_twist);

  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(distance_trajectory.size());

  for (const auto & distance : distance_trajectory) {
    const Eigen::Isometry2d delta_iso = compute_delta_isometry(twist_per_dist, distance);
    const Eigen::Isometry2d target_iso = start_iso * delta_iso;
    pose_trajectory.push_back(isometry_to_pose(target_iso, initial_pose.position.z));
  }

  return pose_trajectory;
}

}  // namespace constant_curvature_predictor

namespace polygon
{
FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const autoware_perception_msgs::msg::Shape & object_shape)
{
  FootprintTrajectory footprint_trajectory;
  footprint_trajectory.reserve(pose_trajectory.size());

  for (const auto & pose : pose_trajectory) {
    const auto footprint = autoware::universe_utils::toPolygon2d(pose, object_shape);
    footprint_trajectory.push_back(footprint);
  }
  return footprint_trajectory;
}

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const VehicleInfo & vehicle_info)
{
  FootprintTrajectory footprint_trajectory;
  footprint_trajectory.reserve(pose_trajectory.size());

  for (const auto & pose : pose_trajectory) {
    const auto footprint = autoware::universe_utils::toFootprint(
      pose, vehicle_info.max_longitudinal_offset_m, vehicle_info.min_longitudinal_offset_m,
      vehicle_info.vehicle_width_m);
    footprint_trajectory.push_back(footprint);
  }
  return footprint_trajectory;
}

Box2d compute_overall_envelope(const FootprintTrajectory & polygons)
{
  Box2d overall_box;
  boost::geometry::assign_inverse(overall_box);

  for (const auto & poly : polygons) {
    for (const auto & p : poly.outer()) {
      boost::geometry::expand(overall_box, p);
    }
  }

  return overall_box;
}

Polygon2d compute_overall_convex_hull(const FootprintTrajectory & polygons)
{
  MultiPoint2d all_points;

  for (const auto & poly : polygons) {
    for (const auto & pt : poly.outer()) {
      all_points.push_back(pt);
    }
  }

  Polygon2d hull;
  boost::geometry::convex_hull(all_points, hull);

  return hull;
}

bool check_path_polygon_convex_collision(
  const FootprintTrajectory & footprints1, const FootprintTrajectory & footprints2)
{
  const auto overall_box1 = compute_overall_envelope(footprints1);
  const auto overall_box2 = compute_overall_envelope(footprints2);
  if (!boost::geometry::intersects(overall_box1, overall_box2)) {
    return false;
  }

  const auto overall_convex1 = compute_overall_convex_hull(footprints1);
  const auto overall_convex2 = compute_overall_convex_hull(footprints2);
  if (!boost::geometry::intersects(overall_convex1, overall_convex2)) {
    return false;
  }

  return true;
}

}  // namespace polygon

TrajectoryData generate_ego_trajectory(
  const geometry_msgs::msg::Twist & initial_twist, double assumed_lag, double assumed_acceleration,
  double max_time, const TrajectoryPoints & traj_points, VehicleInfo & vehicle_info)
{
  auto profile =
    motion::compute_motion_profile_1d(initial_twist, assumed_lag, assumed_acceleration, max_time);

  auto poses = compute_pose_trajectory(traj_points, profile.distances);
  auto footprints = polygon::compute_footprint_trajectory(poses, vehicle_info);

  return TrajectoryData(
    std::string("ego"), std::move(profile.times), std::move(profile.distances), std::move(poses),
    std::move(footprints));
}

// todo: consider time delay
// todo: use all predicted paths
TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double assumed_lag,
  double assumed_acceleration, double max_time)
{
  const auto & max_confidence_path = std::max_element(
    predicted_object.kinematics.predicted_paths.begin(),
    predicted_object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  auto profile = motion::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, assumed_lag,
    assumed_acceleration,
    std::min(
      max_time, max_confidence_path->path.size() *
                  rclcpp::Duration(max_confidence_path->time_step).seconds()));

  auto poses = compute_pose_trajectory(max_confidence_path->path, profile.distances);
  auto footprints = polygon::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    autoware_utils_uuid::to_hex_string(predicted_object.object_id) + "_predicted_path",
    std::move(profile.times), std::move(profile.distances), std::move(poses),
    std::move(footprints));
}

TrajectoryData generate_constant_curvature_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double assumed_lag,
  double assumed_acceleration, double max_time)
{
  auto profile = motion::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, assumed_lag,
    assumed_acceleration, max_time);

  auto poses = constant_curvature_predictor::compute(
    predicted_object.kinematics.initial_pose_with_covariance.pose,
    predicted_object.kinematics.initial_twist_with_covariance.twist, profile.distances);
  auto footprints = polygon::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    autoware_utils_uuid::to_hex_string(predicted_object.object_id) + "_constant_curvature_path",
    std::move(profile.times), std::move(profile.distances), std::move(poses),
    std::move(footprints));
}

bool check_pet_collision(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  double pet_threshold)
{
  if (!polygon::check_path_polygon_convex_collision(
        ref_trajectory.getFootprints(), test_trajectory.getFootprintsInTimeRange(
                                          0.0, ref_trajectory.getTimes().back() + pet_threshold))) {
    return false;
  }

  for (size_t i = 0; i < ref_trajectory.size(); ++i) {
    double ref_start_time = ref_trajectory.getTimes().at(i);
    double ref_end_time = ref_start_time + TIME_RESOLUTION;

    double test_start_time = ref_start_time - pet_threshold;
    double test_end_time = ref_end_time + pet_threshold;

    const auto & ref_poly = ref_trajectory.getFootprintsInTimeRange(ref_start_time, ref_end_time);
    const auto & test_poly =
      test_trajectory.getFootprintsInTimeRange(test_start_time, test_end_time);

    if (polygon::check_path_polygon_convex_collision(ref_poly, test_poly)) {
      return true;
    }
  }

  return false;
}

void CollisionCheckFilter::set_parameters(const std::unordered_map<std::string, std::any> & params)
{
  return;
}

bool CollisionCheckFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    return true;  // No objects to check collision with
  }

  constexpr double pet_th = 1.0;

  const auto ego_trajectory_data = generate_ego_trajectory(
    context.odometry->twist.twist, 0.5, -5.0, 10.0, traj_points, *vehicle_info_ptr_);

  std::vector<TrajectoryData> object_trajectory_data_list{};
  for (const auto & object : context.predicted_objects->objects) {
    object_trajectory_data_list.push_back(
      generate_predicted_path_trajectory(object, 0.0, 0.0, 10.0));

    object_trajectory_data_list.push_back(
      generate_constant_curvature_path_trajectory(object, 0.0, 0.0, 10.0));
  }

  for (const auto & object_trajectory_data : object_trajectory_data_list) {
    // check PET collision between ego and object
    if (check_pet_collision(ego_trajectory_data, object_trajectory_data, pet_th)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("safety_filter::collision_check_filter"),
        "Collision detected by CollisionCheckFilter. Object ID: "
          << object_trajectory_data.getId());
      return false;  // Collision detected
    }
  }

  return true;
}
}  // namespace autoware::trajectory_safety_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_safety_filter::plugin::CollisionCheckFilter,
  autoware::trajectory_safety_filter::plugin::SafetyFilterInterface)
