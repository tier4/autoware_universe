#ifndef AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_TRAJECTORY_CONVERTER_HPP_
#define AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_TRAJECTORY_CONVERTER_HPP_

#include "autoware/tensorrt_vad/converter.hpp"
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/time.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <Eigen/Dense>
#include <vector>
#include <map>

namespace autoware::tensorrt_vad::vad_interface {

/**
 * @brief OutputTrajectoryConverter handles trajectory data conversion from VAD to ROS format
 * 
 * This class converts VAD trajectory predictions to Autoware trajectory messages:
 * - Single trajectory conversion to autoware_planning_msgs::msg::Trajectory
 * - Multiple candidate trajectories conversion to CandidateTrajectories
 * - Coordinate system transformation (VAD → Autoware → Map)
 * - Trajectory point generation with proper timing and velocity calculation
 */
class OutputTrajectoryConverter : public Converter {
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer for coordinate conversions
   * @param config Reference to configuration containing trajectory parameters
   */
  OutputTrajectoryConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config);

  /**
   * @brief Convert VAD predicted trajectory to ROS Trajectory message
   * @param predicted_trajectory VAD trajectory data as flattened [x,y] pairs
   * @param stamp Timestamp for the trajectory
   * @param trajectory_timestep Time interval between trajectory points (seconds)
   * @param base2map_transform Transformation matrix from base_link to map coordinate
   * @return autoware_planning_msgs::msg::Trajectory ROS trajectory message
   */
  autoware_planning_msgs::msg::Trajectory process_trajectory(
    const std::vector<float>& predicted_trajectory,
    const rclcpp::Time& stamp,
    const double trajectory_timestep,
    const Eigen::Matrix4d& base2map_transform) const;

  /**
   * @brief Convert VAD candidate trajectories to ROS CandidateTrajectories message
   * @param predicted_trajectories Map of command indices to trajectory data
   * @param stamp Timestamp for the trajectories
   * @param trajectory_timestep Time interval between trajectory points (seconds)
   * @param base2map_transform Transformation matrix from base_link to map coordinate
   * @return autoware_internal_planning_msgs::msg::CandidateTrajectories ROS candidate trajectories message
   */
  autoware_internal_planning_msgs::msg::CandidateTrajectories process_candidate_trajectories(
    const std::map<int32_t, std::vector<float>>& predicted_trajectories,
    const rclcpp::Time& stamp,
    const double trajectory_timestep,
    const Eigen::Matrix4d& base2map_transform) const;

private:
  /**
   * @brief Generate trajectory points from VAD trajectory data
   * @param predicted_trajectory VAD trajectory data as flattened [x,y] pairs
   * @param trajectory_timestep Time interval between trajectory points (seconds)
   * @param base2map_transform Transformation matrix from base_link to map coordinate
   * @return std::vector<autoware_planning_msgs::msg::TrajectoryPoint> Vector of trajectory points
   */
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> create_trajectory_points(
    const std::vector<float>& predicted_trajectory,
    const double trajectory_timestep,
    const Eigen::Matrix4d& base2map_transform) const;
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_TRAJECTORY_CONVERTER_HPP_
