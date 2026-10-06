// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_

#include "autoware/trajectory_follower_base/control_horizon.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"
#include "autoware/trajectory_follower_node/visibility_control.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/agnocast_wrapper/timer.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <tf2/utils.hpp>

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_control_msgs/msg/control_horizon.hpp"
#include "autoware_control_msgs/msg/longitudinal.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <autoware_control_msgs/msg/detail/control_horizon__struct.hpp>
#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control
{
using trajectory_follower::LateralHorizon;
using trajectory_follower::LateralOutput;
using trajectory_follower::LongitudinalHorizon;
using trajectory_follower::LongitudinalOutput;
namespace trajectory_follower_node
{

using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_control_msgs::msg::ControlHorizon;
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_utils::StopWatch;

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
namespace agnocast_polling = autoware::agnocast_wrapper::polling;

/// \classController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class TRAJECTORY_FOLLOWER_PUBLIC Controller : public autoware::agnocast_wrapper::Node
{
public:
  explicit Controller(const rclcpp::NodeOptions & node_options);
  virtual ~Controller() {}

private:
  AUTOWARE_TIMER_PTR timer_control_;
  double timeout_thr_sec_;
  double cyclic_message_timeout_thr_sec_;
  bool enable_control_cmd_horizon_pub_{false};
  boost::optional<LongitudinalOutput> longitudinal_output_{boost::none};

  std::shared_ptr<autoware::agnocast_wrapper::diagnostic_updater::Updater>
    diag_updater_;  // Diagnostic updater for publishing diagnostic data.

  std::shared_ptr<trajectory_follower::LongitudinalControllerBase> longitudinal_controller_;
  std::shared_ptr<trajectory_follower::LateralControllerBase> lateral_controller_;

  // Subscribers
  agnocast_polling::PollingSubscriber<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_ref_path_ =
      agnocast_polling::create_polling_subscriber<autoware_planning_msgs::msg::Trajectory>(
        this, "~/input/reference_trajectory");

  agnocast_polling::PollingSubscriber<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_ =
    agnocast_polling::create_polling_subscriber<nav_msgs::msg::Odometry>(
      this, "~/input/current_odometry");

  agnocast_polling::PollingSubscriber<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    sub_steering_ =
      agnocast_polling::create_polling_subscriber<autoware_vehicle_msgs::msg::SteeringReport>(
        this, "~/input/current_steering");

  agnocast_polling::PollingSubscriber<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    sub_accel_ =
      agnocast_polling::create_polling_subscriber<geometry_msgs::msg::AccelWithCovarianceStamped>(
        this, "~/input/current_accel");

  agnocast_polling::PollingSubscriber<OperationModeState>::SharedPtr sub_operation_mode_ =
    agnocast_polling::create_polling_subscriber<OperationModeState>(
      this, "~/input/current_operation_mode", rclcpp::QoS{1}.transient_local());

  AUTOWARE_SUBSCRIPTION_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  sub_steering_offset_update_;

  // Publishers
  AUTOWARE_PUBLISHER_PTR(autoware_control_msgs::msg::Control) control_cmd_pub_;
  AUTOWARE_PUBLISHER_PTR(Float64Stamped) pub_processing_time_lat_ms_;
  AUTOWARE_PUBLISHER_PTR(Float64Stamped) pub_processing_time_lon_ms_;
  AUTOWARE_PUBLISHER_PTR(visualization_msgs::msg::MarkerArray) debug_marker_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_control_msgs::msg::ControlHorizon) control_cmd_horizon_pub_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr current_trajectory_ptr_;
  // agnocast_polling::PollingSubscriber exposes no last_taken_data_timestamp().
  std::optional<rclcpp::Time> last_trajectory_taken_time_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry_ptr_;
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr current_steering_ptr_;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr current_accel_ptr_;
  OperationModeState::ConstSharedPtr current_operation_mode_ptr_;

  enum class LateralControllerMode {
    INVALID = 0,
    MPC = 1,
    PURE_PURSUIT = 2,
  };
  enum class LongitudinalControllerMode {
    INVALID = 0,
    PID = 1,
  };

  /**
   * @brief compute control command, and publish periodically
   */
  boost::optional<trajectory_follower::InputData> createInputData(rclcpp::Clock & clock);
  void callbackTimerControl();
  bool processData(rclcpp::Clock & clock);
  bool isTimeOut(const LongitudinalOutput & lon_out, const LateralOutput & lat_out);
  void check_cyclic_message_timeout(diagnostic_updater::DiagnosticStatusWrapper & stat);
  LateralControllerMode getLateralControllerMode(const std::string & algorithm_name) const;
  LongitudinalControllerMode getLongitudinalControllerMode(
    const std::string & algorithm_name) const;
  void publishDebugMarker(
    const trajectory_follower::InputData & input_data,
    const trajectory_follower::LateralOutput & lat_out) const;
  /**
   * @brief merge lateral and longitudinal horizons
   * @details If one of the commands has only one control, repeat the control to match the other
   *          horizon. If each horizon has different time intervals, resample them to match the size
   *          with the greatest common divisor.
   * @param lateral_horizon lateral horizon
   * @param longitudinal_horizon longitudinal horizon
   * @param stamp stamp
   * @return merged control horizon
   */
  static std::optional<ControlHorizon> mergeLatLonHorizon(
    const LateralHorizon & lateral_horizon, const LongitudinalHorizon & longitudinal_horizon,
    const rclcpp::Time & stamp);

  std::unique_ptr<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>
    logger_configure_;

  std::unique_ptr<
    autoware_utils_debug::BasicPublishedTimePublisher<autoware::agnocast_wrapper::Node>>
    published_time_publisher_;

  void publishProcessingTime(const double t_ms, const AUTOWARE_PUBLISHER_PTR(Float64Stamped) & pub);
  StopWatch<std::chrono::milliseconds> stop_watch_;

  static constexpr double logger_throttle_interval = 5000;
};
}  // namespace trajectory_follower_node
}  // namespace autoware::motion::control

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
