#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__TRAJECTORY_UTILS_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__TRAJECTORY_UTILS_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"
#include "parameter.hpp"
#include "types.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <rclcpp/duration.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <string>

namespace autoware::trajectory_validator::plugin::safety::trajectory::time_distance
{
std::pair<TimeTrajectory, TravelDistanceTrajectory> compute_motion_profile_1d(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double start_time, double end_time, double time_resolution);
}  // namespace autoware::trajectory_validator::plugin::safety::trajectory::time_distance

namespace autoware::trajectory_validator::plugin::safety::trajectory::pose
{
geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose,
  double ratio);

namespace constant_curvature_predictor
{
struct TwistPerDistance
{
  Eigen::Vector2d linear;
  double angular;
};

namespace detail
{
Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose);
TwistPerDistance compute_twist_per_distance(const geometry_msgs::msg::Twist & twist);
Eigen::Isometry2d compute_delta_isometry(const TwistPerDistance & twist_per_dist, double distance);
geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry2d & iso, double initial_z);
}  // namespace detail

PoseTrajectory compute(
  const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Twist & initial_twist,
  const TravelDistanceTrajectory & distance_trajectory);
}  // namespace constant_curvature_predictor

template <class T>
PoseTrajectory compute_pose_trajectory(
  const T & traj_points, const TravelDistanceTrajectory & distance_trajectory)
{
  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(distance_trajectory.size());
  for (const auto & distance : distance_trajectory) {
    pose_trajectory.push_back(
      autoware::motion_utils::calcInterpolatedPose(traj_points, distance, false));
  }
  return pose_trajectory;
}

PoseTrajectory compute_pose_trajectory_from_time(
  const TrajectoryPoints & traj_points, const TimeTrajectory & time_trajectory);
}  // namespace autoware::trajectory_validator::plugin::safety::trajectory::pose

namespace autoware::trajectory_validator::plugin::safety::geometry
{
namespace detail
{
template <typename Points>
std::pair<double, double> project_points(const Points & ring, double axis_x, double axis_y)
{
  if (ring.empty()) {
    return {0.0, 0.0};
  }

  const auto project = [&](const auto & point) {
    return boost::geometry::get<0>(point) * axis_x + boost::geometry::get<1>(point) * axis_y;
  };

  auto point_it = ring.begin();
  double min_projection = project(*point_it);
  double max_projection = min_projection;
  for (++point_it; point_it != ring.end(); ++point_it) {
    const double projection = project(*point_it);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
  }

  return {min_projection, max_projection};
}

template <typename ClosedRing>
bool has_separating_axis(
  const ClosedRing & candidate_axes, const ClosedRing & ring_a, const ClosedRing & ring_b)
{
  auto previous_it = candidate_axes.begin();
  for (auto current_it = std::next(candidate_axes.begin()); current_it != candidate_axes.end();
       ++current_it) {
    const double edge_x =
      boost::geometry::get<0>(*current_it) - boost::geometry::get<0>(*previous_it);
    const double edge_y =
      boost::geometry::get<1>(*current_it) - boost::geometry::get<1>(*previous_it);
    const auto [min_a, max_a] = project_points(ring_a, -edge_y, edge_x);
    const auto [min_b, max_b] = project_points(ring_b, -edge_y, edge_x);
    if (max_a < min_b || max_b < min_a) {
      return true;
    }
    previous_it = current_it;
  }

  return false;
}
}  // namespace detail

template <typename ConvexPolygon>
bool intersects_sat(const ConvexPolygon & poly_a, const ConvexPolygon & poly_b)
{
  const auto & ring_a = poly_a.outer();
  const auto & ring_b = poly_b.outer();

  constexpr size_t minimum_closed_convex_ring_size = 3U;
  if (
    ring_a.size() < minimum_closed_convex_ring_size ||
    ring_b.size() < minimum_closed_convex_ring_size) {
    return false;
  }

  return !detail::has_separating_axis(ring_a, ring_a, ring_b) &&
         !detail::has_separating_axis(ring_b, ring_a, ring_b);
}

Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape);
}  // namespace autoware::trajectory_validator::plugin::safety::geometry

namespace autoware::trajectory_validator::plugin::safety::trajectory
{
namespace footprint
{
FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory,
  const autoware_perception_msgs::msg::Shape & object_shape);

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const VehicleInfo & vehicle_info);
}  // namespace footprint

namespace detail
{
double to_seconds(const builtin_interfaces::msg::Duration & duration);

double project_current_pose_on_trajectory(
  const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & current_pose);

TravelDistanceTrajectory compute_cumulative_distances(const PoseTrajectory & pose_trajectory);

TimeTrajectory compute_sample_times(double start_time, double end_time, double time_resolution);

geometry_msgs::msg::Pose interpolate_predicted_path_pose(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path, double query_time,
  double path_start_time);
}  // namespace detail

TrajectoryData generate_ego_trajectory(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double max_time, double time_resolution, const TrajectoryPoints & traj_points,
  VehicleInfo & vehicle_info);

TrajectoryData generate_ego_trajectory(
  const TrajectoryPoints & traj_points, const FilterContext & context, double max_time,
  double time_resolution, VehicleInfo & vehicle_info);

TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time,
  const builtin_interfaces::msg::Time & stamp, double time_resolution);

TrajectoryData generate_diffusion_based_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  rclcpp::Duration start_time, double max_time, const builtin_interfaces::msg::Time & stamp,
  double time_resolution);

TrajectoryData generate_constant_curvature_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time,
  const builtin_interfaces::msg::Time & stamp, double time_resolution);

TrajectoryData generate_object_trajectory(
  const FilterContext & context, unique_identifier_msgs::msg::UUID object_id,
  const std::string & traj_type_str, double acc, double time_resolution, double time_horizon);
}  // namespace autoware::trajectory_validator::plugin::safety::trajectory

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__TRAJECTORY_UTILS_HPP_
