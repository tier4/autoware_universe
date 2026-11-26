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

#ifndef AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__PREDICTED_TRAJECTORY_VALIDATOR_HPP_
#define AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__PREDICTED_TRAJECTORY_VALIDATOR_HPP_

#include "autoware/predicted_trajectory_validator/collision_validator/collision_validator.hpp"
#include "autoware/predicted_trajectory_validator/debug_marker.hpp"
#include "autoware/predicted_trajectory_validator/hazard_warning/hazard_warning.hpp"
#include "autoware/predicted_trajectory_validator/uncrossable_bound_departure_validator.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "autoware_vehicle_info_utils/vehicle_info.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include <autoware_predicted_trajectory_validator/msg/predicted_trajectory_validator_status.hpp>
#include <autoware_predicted_trajectory_validator/msg/validation_result.hpp>
#include <autoware_predicted_trajectory_validator/predicted_trajectory_validator_parameters.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_control_msgs/msg/hazard_alert.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace autoware::predicted_trajectory_validator
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_internal_control_msgs::msg::HazardAlert;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_predicted_trajectory_validator::msg::PredictedTrajectoryValidatorStatus;
using autoware_predicted_trajectory_validator::msg::ValidationResult;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

/**
 * @class PredictedTrajectoryValidator
 * @brief Evaluates the effect of the control predicted trajectory on the surroundings: uncrossable
 * boundary departure, collision and the hazard alert.
 */
class PredictedTrajectoryValidator : public rclcpp::Node
{
public:
  explicit PredictedTrajectoryValidator(const rclcpp::NodeOptions & options);

  /**
   * @brief Callback running one validation cycle per predicted trajectory.
   * @param msg Predicted trajectory of the controller
   */
  void on_predicted_trajectory(const Trajectory::ConstSharedPtr msg);

  /**
   * @brief Callback storing the lanelet map used by the boundary departure check.
   * @param msg Lanelet map binary message
   */
  void on_lanelet_map(const LaneletMapBin::ConstSharedPtr msg);

private:
  enum Judgment : std::size_t { UNCROSSABLE_BOUND_DEPARTURE, NUM_JUDGMENTS };

  void setup_diag();

  void setup_parameters();

  void publish_debug_info(const geometry_msgs::msg::Pose & ego_pose);

  /**
   * @brief Publish the boundary departure debug markers, clearing stale markers first.
   * @param markers Markers produced by the boundary departure validator this cycle
   */
  void publish_boundary_departure_markers(visualization_msgs::msg::MarkerArray markers);

  void display_status();

  /**
   * @brief Set the diagnostic status from one validation result
   * @param stat Diagnostic status wrapper
   * @param result Validation result whose invalid_count decides WARN or ERROR
   * @param msg Status message
   */
  void set_status(
    DiagnosticStatusWrapper & stat, const ValidationResult & result, const std::string & msg) const;

  static bool infer_autonomous_control_state(const OperationModeState::ConstSharedPtr);

  bool is_all_valid() const;

  PredictedTrajectoryValidatorStatus to_msg() const;

  // Subscribers and publishers
  rclcpp::Subscription<Trajectory>::SharedPtr sub_predicted_traj_;
  autoware_utils::InterProcessPollingSubscriber<OperationModeState>::SharedPtr
    sub_operational_state_;
  autoware_utils::InterProcessPollingSubscriber<Odometry>::SharedPtr sub_kinematics_;
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>::SharedPtr
    sub_measured_acc_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_lanelet_map_;
  autoware_utils::InterProcessPollingSubscriber<PredictedObjects>::SharedPtr sub_objects_;
  autoware_utils::InterProcessPollingSubscriber<PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Publisher<PredictedTrajectoryValidatorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;
  rclcpp::Publisher<HazardAlert>::SharedPtr pub_hazard_alert_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_hazard_processing_time_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    pub_hazard_processing_time_detail_;

  autoware_utils::StopWatch<std::chrono::milliseconds> hazard_stop_watch_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> hazard_time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>();

  // parameters (generated by generate_parameter_library)
  ::predicted_trajectory_validator::ParamListener param_listener_;
  ::predicted_trajectory_validator::Params params_;

  int64_t diag_error_count_threshold_ = 0;
  Updater diag_updater_{this};
  std::array<ValidationResult, NUM_JUDGMENTS> results_;
  vehicle_info_utils::VehicleInfo vehicle_info_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  bool flag_autonomous_control_enabled_ = false;

  // debug
  std::shared_ptr<PredictedTrajectoryValidatorDebugMarkerPublisher> debug_pose_publisher_;
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;

  // individual validators
  UncrossableBoundDepartureValidator uncrossable_bound_departure_validator_;
  CollisionValidator collision_validator_;
  HazardWarning hazard_warning_;
};
}  // namespace autoware::predicted_trajectory_validator

#endif  // AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__PREDICTED_TRAJECTORY_VALIDATOR_HPP_
