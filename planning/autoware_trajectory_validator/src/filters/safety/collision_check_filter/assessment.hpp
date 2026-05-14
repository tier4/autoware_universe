#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__ASSESSMENT_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__ASSESSMENT_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"
#include "parameter.hpp"
#include "trajectory_utils.hpp"
#include "types.hpp"

#include <Eigen/Geometry>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment
{
struct ObjectTrajectoryGenerationOptions
{
  bool predicted_path_trajectory{false};
  bool constant_curvature_trajectory{false};
  bool diffusion_based_trajectory{false};

  ObjectTrajectoryGenerationOptions() = default;

  template <typename ParamsT>
  explicit ObjectTrajectoryGenerationOptions(const ParamsT & params)
  {
    predicted_path_trajectory = params.assessment_trajectories.map_based;
    constant_curvature_trajectory = params.assessment_trajectories.constant_curvature;
    diffusion_based_trajectory = params.assessment_trajectories.diffusion_based;
  }

  void merge_with(const ObjectTrajectoryGenerationOptions & other)
  {
    predicted_path_trajectory = predicted_path_trajectory || other.predicted_path_trajectory;
    constant_curvature_trajectory =
      constant_curvature_trajectory || other.constant_curvature_trajectory;
    diffusion_based_trajectory = diffusion_based_trajectory || other.diffusion_based_trajectory;
  }
};

std::vector<TrajectoryData> generate_object_trajectories(
  const FilterContext & context, double required_time_horizon, double object_assumed_acceleration,
  double time_resolution, const ObjectTrajectoryGenerationOptions & options);

std::pair<PetArtifact, DracArtifact> assess(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const PetCollisionParams & pet_collision_params, const DracParams & drac_params,
  const GlobalParams & global_params, VehicleInfo & vehicle_info);
}  // namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment

namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
{
template <typename PosePoints, typename Object>
double compute_longitudinal_velocity(const PosePoints & points, const Object & object)
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

std::optional<double> compute_distance_to_collision(
  const TrajectoryData & ego_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & object);

RssArtifact assess(
  const TrajectoryPoints & traj_points, const FilterContext & context, const RssParams & rss_params,
  double time_resolution, VehicleInfo & vehicle_info);
}  // namespace autoware::trajectory_validator::plugin::safety::rss_deceleration

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__ASSESSMENT_HPP_
