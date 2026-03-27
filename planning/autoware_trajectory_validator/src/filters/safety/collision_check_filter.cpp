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

#include "autoware/trajectory_validator/filters/safety/collision_check_filter.hpp"

#include <autoware/universe_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <any>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{

namespace trajectory
{
namespace time_distance
{
std::pair<TimeTrajectory, TravelDistanceTrajectory> compute_motion_profile_1d(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double start_time, double max_end_time)
{
  struct MotionProfile
  {
    double time;
    double distance;
  };

  const double initial_velocity = std::hypot(initial_twist.linear.x, initial_twist.linear.y);

  if (initial_velocity <= 0.0 || start_time >= max_end_time) {
    return {{start_time}, {0.0}};
  }

  std::vector<MotionProfile> profile;

  for (double t = start_time; t < max_end_time;
       t = std::floor((t + TIME_RESOLUTION + 1e-6) / TIME_RESOLUTION) * TIME_RESOLUTION) {
    if (t < braking_lag) {
      profile.emplace_back(MotionProfile{t, initial_velocity * t});
    } else {
      const double lag_distance = initial_velocity * braking_lag;
      const double time_after_lag = t - braking_lag;
      const double current_velocity = initial_velocity + assumed_acceleration * time_after_lag;

      if (current_velocity <= 0.0) {
        const double time_to_stop = initial_velocity / -assumed_acceleration;
        const double stop_distance = lag_distance + (initial_velocity * time_to_stop) +
                                     (0.5 * assumed_acceleration * time_to_stop * time_to_stop);
        profile.emplace_back(MotionProfile{braking_lag + time_to_stop, stop_distance});
        break;
      }

      const double distance = lag_distance + (initial_velocity * time_after_lag) +
                              (0.5 * assumed_acceleration * time_after_lag * time_after_lag);
      profile.emplace_back(MotionProfile{t, distance});
    }
  }

  TimeTrajectory return_times;
  TravelDistanceTrajectory return_distances;
  for (const auto & p : profile) {
    return_times.push_back(p.time);
    return_distances.push_back(p.distance);
  }

  return {return_times, return_distances};
}
}  // namespace time_distance

namespace pose
{
namespace constant_curvature_predictor
{
struct TwistPerDistance
{
  Eigen::Vector2d linear;
  double angular;
};

namespace
{

Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
  iso.translation() << pose.position.x, pose.position.y;
  iso.linear() = Eigen::Rotation2Dd(tf2::getYaw(pose.orientation)).toRotationMatrix();
  return iso;
}

TwistPerDistance compute_twist_per_distance(const geometry_msgs::msg::Twist & twist)
{
  const Eigen::Vector2d vel(twist.linear.x, twist.linear.y);
  const double linear_speed = vel.norm();
  const double inv_speed = (linear_speed > 1e-6) ? (1.0 / linear_speed) : 0.0;

  return {vel * inv_speed, twist.angular.z * inv_speed};
}

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

template <class T>
PoseTrajectory compute_pose_trajectory(
  const T & traj_points, const TravelDistanceTrajectory & distance_trajectory)
{
  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(distance_trajectory.size());
  for (const auto & distance : distance_trajectory) {
    const auto pose = autoware::motion_utils::calcInterpolatedPose(traj_points, distance, false);
    pose_trajectory.push_back(pose);
  }
  return pose_trajectory;
}

}  // namespace pose

namespace footprint
{
FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const autoware_perception_msgs::msg::Shape & object_shape)
{
  FootprintTrajectory footprint_trajectory;
  footprint_trajectory.reserve(pose_trajectory.size());

  for (const auto & pose : pose_trajectory) {
    footprint_trajectory.push_back(autoware_utils_geometry::to_polygon2d(pose, object_shape));
  }
  return footprint_trajectory;
}

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const VehicleInfo & vehicle_info)
{
  FootprintTrajectory footprint_trajectory;
  footprint_trajectory.reserve(pose_trajectory.size());

  for (const auto & pose : pose_trajectory) {
    footprint_trajectory.push_back(autoware_utils_geometry::to_footprint(
      pose, vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
      vehicle_info.vehicle_width_m));
  }
  return footprint_trajectory;
}
}  // namespace footprint

TrajectoryData generate_ego_trajectory(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double max_time, const TrajectoryPoints & traj_points, VehicleInfo & vehicle_info)
{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    initial_twist, braking_lag, assumed_acceleration, 0.0, max_time);

  double distance_offset =
    autoware::motion_utils::calcSignedArcLength(traj_points, traj_points.front().pose.position, 0);
  for (double & val : distances) {
    val += distance_offset;
  }
  auto poses = pose::compute_pose_trajectory(traj_points, distances);

  auto footprints = footprint::compute_footprint_trajectory(poses, vehicle_info);

  return TrajectoryData(
    std::string("ego"), std::move(times), std::move(distances), std::move(poses),
    std::move(footprints));
}

// todo: use all predicted paths
TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time)
{
  const auto & max_confidence_path = std::max_element(
    predicted_object.kinematics.predicted_paths.begin(),
    predicted_object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, braking_lag,
    assumed_acceleration, start_time.seconds(),
    std::min(
      max_time, max_confidence_path->path.size() *
                  rclcpp::Duration(max_confidence_path->time_step).seconds()));

  auto poses = pose::compute_pose_trajectory(max_confidence_path->path, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    autoware_utils_uuid::to_hex_string(predicted_object.object_id) + "_predicted_path",
    std::move(times), std::move(distances), std::move(poses), std::move(footprints));
}

TrajectoryData generate_constant_curvature_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time)
{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, braking_lag,
    assumed_acceleration, start_time.seconds(), max_time);

  auto poses = pose::constant_curvature_predictor::compute(
    predicted_object.kinematics.initial_pose_with_covariance.pose,
    predicted_object.kinematics.initial_twist_with_covariance.twist, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    autoware_utils_uuid::to_hex_string(predicted_object.object_id) + "_constant_curvature_path",
    std::move(times), std::move(distances), std::move(poses), std::move(footprints));
}
}  // namespace trajectory

namespace geometry
{
template <typename Range>
Box2d compute_overall_envelope(const Range & polygons)
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

template <typename Range>
Polygon2d compute_overall_convex_hull(const Range & polygons)
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

template <typename Range1, typename Range2>
bool check_path_polygon_convex_collision(const Range1 & footprints1, const Range2 & footprints2)
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
}  // namespace geometry

namespace collision_assessment
{
struct PetCollisionResult
{
  std::optional<double> pet;
  std::optional<double> ttc;
  std::optional<DebugData> debug_data;
};

bool check_pet_collision(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  double pet_threshold)
{
  if (!geometry::check_path_polygon_convex_collision(
        ref_trajectory.getFootprints(), test_trajectory.getFootprintsInTimeRange(
                                          0.0, ref_trajectory.getTimes().back() + pet_threshold))) {
    return false;
  }

  for (size_t i = 0; i < ref_trajectory.size(); ++i) {
    double ref_start_time = ref_trajectory.getTimes().at(i);
    double ref_end_time = ref_start_time + TIME_RESOLUTION;

    double test_start_time = ref_start_time - pet_threshold;
    double test_end_time = ref_end_time + pet_threshold;

    const auto ref_poly = ref_trajectory.getFootprintsInTimeRange(ref_start_time, ref_end_time);
    const auto test_poly = test_trajectory.getFootprintsInTimeRange(test_start_time, test_end_time);

    if (geometry::check_path_polygon_convex_collision(ref_poly, test_poly)) {
      return true;
    }
  }

  return false;
}

PetCollisionResult compute_pet_and_ttc(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  double pet_threshold)
{
  DebugData debug_data;
  if (!geometry::check_path_polygon_convex_collision(
        ref_trajectory.getFootprints(), test_trajectory.getFootprintsInTimeRange(
                                          0.0, ref_trajectory.getTimes().back() + pet_threshold))) {
    return PetCollisionResult{std::nullopt, std::nullopt, std::nullopt};
  }

  std::optional<double> candidate_pet{};
  std::optional<double> candidate_ttc{};
  for (size_t i = 0; i < ref_trajectory.size(); ++i) {
    const double ref_start_time = ref_trajectory.getTimes().at(i);
    const double ref_end_time = ref_start_time + TIME_RESOLUTION;
    const auto ref_poly = ref_trajectory.getFootprintsInTimeRange(ref_start_time, ref_end_time);
    auto check_slice_collision = [&](double start, double end) {
      const auto slice_poly = test_trajectory.getFootprintsInTimeRange(start, end);
      return geometry::check_path_polygon_convex_collision(ref_poly, slice_poly);
    };

    const double current_pet_limit = candidate_pet.value_or(pet_threshold);
    const double test_start_time = ref_start_time - current_pet_limit;
    const double test_end_time = ref_end_time + current_pet_limit;
    if (!check_slice_collision(test_start_time, test_end_time)) {
      continue;
    }
    for (double pet_range = 0.0; pet_range <= current_pet_limit; pet_range += TIME_RESOLUTION) {
      const double test_start_time_before = ref_start_time - pet_range;
      const double test_end_time_before = test_start_time_before + TIME_RESOLUTION;

      const double test_start_time_after = ref_end_time + pet_range - TIME_RESOLUTION;
      const double test_end_time_after = ref_end_time + pet_range;

      if (
        check_slice_collision(test_start_time_before, test_end_time_before) ||
        check_slice_collision(test_start_time_after, test_end_time_after)) {
        candidate_pet = pet_range;
        candidate_ttc = ref_start_time;

        debug_data.pet = pet_range;
        debug_data.ego_polygons = geometry::compute_overall_convex_hull(
          ref_trajectory.getFootprintsInTimeRange(ref_start_time, ref_end_time));
        debug_data.object_polygons = geometry::compute_overall_convex_hull(
          test_trajectory.getFootprintsInTimeRange(test_start_time_before, test_end_time_after));
        debug_data.object_id = test_trajectory.getId();

        break;
      }
    }
    if (candidate_pet.has_value() && candidate_pet.value() == 0.0) {
      return PetCollisionResult{candidate_pet, candidate_ttc, debug_data};
    }
  }

  return PetCollisionResult{candidate_pet, candidate_ttc, debug_data};
}

template <typename PosePoints, typename Object>
double calc_longitudinal_velocity(const PosePoints & points, const Object & object)
{
  if (points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  constexpr double min_path_end_to_end_distance = 1e-3;

  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const bool use_path_yaw =
    points.size() >= 2 && autoware_utils_geometry::calc_distance2d(points.front(), points.back()) >=
                            min_path_end_to_end_distance;
  const double object_yaw_relative_to_points =
    use_path_yaw ? autoware::motion_utils::calcYawDeviation(points, object_pose, true)
                 : autoware::universe_utils::calcYawDeviation(points.front(), object_pose);
  const Eigen::Rotation2Dd object_to_points_rotation(object_yaw_relative_to_points);

  const auto & object_twist = object.kinematics.initial_twist_with_covariance.twist;
  const Eigen::Vector2d object_velocity_in_object_frame(
    object_twist.linear.x, object_twist.linear.y);
  const Eigen::Vector2d object_velocity_in_points_frame =
    object_to_points_rotation * object_velocity_in_object_frame;

  return object_velocity_in_points_frame.x();
}

std::optional<double> calc_dist_to_collide(
  const TrajectoryData & ego_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto obj_footprint = autoware_utils_geometry::to_polygon2d(
    object.kinematics.initial_pose_with_covariance.pose, object.shape);
  const auto obj_foot_print_as_range =
    boost::make_iterator_range(&obj_footprint, &obj_footprint + 1);

  if (!geometry::check_path_polygon_convex_collision(
        ego_trajectory.getFootprints(), obj_foot_print_as_range)) {
    return std::nullopt;
  }

  for (size_t i = 0; i < ego_trajectory.size(); ++i) {
    const auto ego_footprint = ego_trajectory.getFootprints().at(i);
    const auto ego_foot_print_as_range =
      boost::make_iterator_range(&ego_footprint, &ego_footprint + 1);
    if (geometry::check_path_polygon_convex_collision(
          ego_foot_print_as_range, obj_foot_print_as_range)) {
      // todo(takagi): for precise calculation, intersection length should be considered.
      return ego_trajectory.getDistances().at(i);
    }
  }

  return std::nullopt;
}

}  // namespace collision_assessment

void CollisionCheckFilter::update_parameters(const validator::Params & params)
{
  pet_collision_params_ = params.collision_check.pet_collision;
  rss_params_ = params.collision_check.rss;
}

void CollisionCheckFilter::add_debug_markers(
  const DebugData & debug_data, const rclcpp::Time & stamp)
{
  int id = debug_markers_.markers.empty() ? 0 : debug_markers_.markers.back().id + 1;

  auto add_poly_marker =
    [&](const Polygon2d & poly, const std::string & ns, float r, float g, float b) {
      if (poly.outer().empty()) return;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = ns;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.05;  // line width
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 0.9;

      for (const auto & p : poly.outer()) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = 0.0;
        m.points.push_back(pt);
      }
      // close the polygon by adding the first point at the end
      geometry_msgs::msg::Point pt_first;
      pt_first.x = poly.outer().front().x();
      pt_first.y = poly.outer().front().y();
      pt_first.z = 0.0;
      m.points.push_back(pt_first);

      debug_markers_.markers.push_back(std::move(m));
    };

  add_poly_marker(debug_data.ego_polygons, "ego_worst_pet_" + debug_data.object_id, 0.0, 0.0, 1.0);
  add_poly_marker(
    debug_data.object_polygons, "obj_worst_pet_" + debug_data.object_id, 1.0, 0.0, 0.0);
}

double CollisionCheckFilter::compute_rss_deceleration(
  const TrajectoryData & ego_trajectory, const geometry_msgs::msg::Twist & ego_twist,
  const autoware_perception_msgs::msg::PredictedObject & object) const
{
  const auto ego_long_vel = ego_twist.linear.x;
  if (ego_long_vel <= 0.0) {
    return 0.0;
  }

  // calc current distance
  const auto dist_to_collide = collision_assessment::calc_dist_to_collide(ego_trajectory, object);
  if (!dist_to_collide.has_value()) {
    return 0.0;
  }

  // calc safe distance
  const double obj_long_vel = std::clamp(
    collision_assessment::calc_longitudinal_velocity(ego_trajectory.getPoses(), object), 0.0, 30.0);
  const double safe_distance =
    dist_to_collide.value() + obj_long_vel * obj_long_vel * 0.5 / -rss_params_.object_acceleration -
    ego_long_vel * rss_params_.ego_reaction_time;

  // calc required deceleration
  if (safe_distance <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return ego_long_vel * ego_long_vel * 0.5 / safe_distance;
}

// todo(takagi): separate the core logic which returns detailed collision information.
tl::expected<void, std::string> CollisionCheckFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  // autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  // stopwatch.tic();

  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    return {};  // No objects to check collision with
  }

  if (traj_points.empty()) {
    return {};  // No trajectory to check
  }

  std::string error_msg{};

  // todo takagi: refactor to separate functions and add more detailed collision information in the
  // error message. calc PET and TTC metrics
  rclcpp::Duration objects_reference_time = rclcpp::Time(context.predicted_objects->header.stamp) -
                                            rclcpp::Time(context.odometry->header.stamp);

  const double ego_consider_time_for_pet = std::abs(context.odometry->twist.twist.linear.x) * 0.5 /
                                             -pet_collision_params_.ego_assumed_acceleration +
                                           pet_collision_params_.ego_braking_delay;
  const auto ego_trajectory_data = trajectory::generate_ego_trajectory(
    context.odometry->twist.twist, 0.0, 0.0, ego_consider_time_for_pet, traj_points,
    *vehicle_info_ptr_);

  std::vector<TrajectoryData> object_trajectory_data_list{};
  for (const auto & object : context.predicted_objects->objects) {
    object_trajectory_data_list.push_back(trajectory::generate_predicted_path_trajectory(
      object, 0.0, 0.0, objects_reference_time,
      ego_consider_time_for_pet + pet_collision_params_.collision_time_threshold));

    object_trajectory_data_list.push_back(trajectory::generate_constant_curvature_path_trajectory(
      object, 0.0, 0.0, objects_reference_time,
      ego_consider_time_for_pet + pet_collision_params_.collision_time_threshold));
  }

  for (const auto & object_trajectory_data : object_trajectory_data_list) {
    auto collision_result = collision_assessment::compute_pet_and_ttc(
      ego_trajectory_data, object_trajectory_data, pet_collision_params_.collision_time_threshold);
    auto pet = collision_result.pet;
    auto ttc = collision_result.ttc;
    if (pet.has_value()) {
      // todo(takagi): should be refactored for optional access.
      error_msg += fmt::format(
        "PET collision, ID: {}, PET: {}, TTC: {}, stamp: {}.{}; ", object_trajectory_data.getId(),
        pet.value(), ttc.has_value() ? std::to_string(ttc.value()) : "N/A",
        context.predicted_objects->header.stamp.sec,
        context.predicted_objects->header.stamp.nanosec);
      add_debug_markers(collision_result.debug_data.value(), context.odometry->header.stamp);
    }
  }

  // calc RSS metrics
  {
    const double ego_consider_time_for_rss =
      rclcpp::Duration(traj_points.back().time_from_start).seconds();
    const auto ego_trajectory_data = trajectory::generate_ego_trajectory(
      context.odometry->twist.twist, 0.0, 0.0, ego_consider_time_for_rss, traj_points,
      *vehicle_info_ptr_);

    for (const auto & object : context.predicted_objects->objects) {
      const auto required_deceleration =
        compute_rss_deceleration(ego_trajectory_data, context.odometry->twist.twist, object);

      if (required_deceleration > rss_params_.ego_deceleration_threshold) {
        error_msg += std::string("RSS collision, ID: ") +
                     autoware_utils_uuid::to_hex_string(object.object_id) +
                     std::string(", required deceleration: ") +
                     std::to_string(required_deceleration);
      }
    }
  }

  if (!error_msg.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("CollisionCheckFilter"), "Not feasible: %s", error_msg.c_str());
    return tl::make_unexpected(error_msg);
  }

  return {};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::CollisionCheckFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
