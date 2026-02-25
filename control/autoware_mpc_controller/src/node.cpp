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

/**
 * MPC controller node — combined longitudinal + lateral.
 * Drop-in replacement for trajectory_follower Controller: same subscriptions and
 * ~/output/control_cmd (autoware_control_msgs/Control).
 */

#include "acados_interface.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>

#include <cmath>
#include <memory>
#include <optional>

namespace
{
constexpr double kPi = 3.141592653589793;
double normalizeAngle(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}
}  // namespace

class MpcControllerNode : public rclcpp::Node
{
public:
  explicit MpcControllerNode(const rclcpp::NodeOptions & options) : Node("mpc_controller_node", options)
  {
    enable_controller_ = declare_parameter<bool>("enable_controller", true);
    ctrl_period_ = declare_parameter<double>("ctrl_period", 0.03);
    delay_compensation_time_ = declare_parameter<double>("delay_compensation_time", 0.17);
    tau_equiv_ = declare_parameter<double>("tau_equiv", 1.5);
    wheelbase_lf_ = declare_parameter<double>("wheelbase_lf", 1.0);
    wheelbase_lr_ = declare_parameter<double>("wheelbase_lr", 1.0);
    max_steer_rad_ = declare_parameter<double>("max_steer_rad", 0.7);
    timeout_trajectory_sec_ = declare_parameter<double>("timeout_trajectory_sec", 1.0);

    const int max_iter = declare_parameter<int>("mpc_max_iter", 20);
    const double tol = declare_parameter<double>("mpc_tol", 1e-4);

    interface_ = std::make_unique<AcadosInterface>(max_iter, tol);

    sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "~/input/reference_trajectory", 1, [this](const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
        trajectory_ = msg;
      });
    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/current_odometry", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odometry_ = msg;
      });
    sub_steering_ = create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
      "~/input/current_steering", 1, [this](const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg) {
        steering_ = msg;
      });
    sub_accel_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "~/input/current_accel", 1, [this](const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg) {
        accel_ = msg;
      });

    pub_control_ = create_publisher<autoware_control_msgs::msg::Control>(
      "~/output/control_cmd", rclcpp::QoS(1).transient_local());

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(ctrl_period_)),
      std::bind(&MpcControllerNode::onTimer, this));
  }

private:
  void onTimer()
  {
    if (!enable_controller_) {
      return;
    }
    if (!trajectory_ || !odometry_ || !accel_) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for trajectory, odometry, accel.");
      return;
    }
    if (trajectory_->points.size() < 2) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Trajectory too short.");
      return;
    }
    if ((this->now() - rclcpp::Time(trajectory_->header.stamp)).seconds() > timeout_trajectory_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Trajectory too old (%.1f s), skipping.", timeout_trajectory_sec_);
      return;
    }

    const geometry_msgs::msg::Pose ego_pose = odometry_->pose.pose;
    const double v = odometry_->twist.twist.linear.x;
    const double a = accel_->accel.accel.linear.x;

    std::optional<double> s_opt;
    std::optional<double> eY_opt;
    std::optional<double> ePsi_opt;
    std::optional<double> kappa_ref_opt;
    std::optional<double> ref_velocity_opt;

    try {
      const double s = autoware::motion_utils::calcSignedArcLength(
        trajectory_->points, static_cast<size_t>(0), ego_pose.position);
      s_opt = s;
    } catch (const std::exception & e) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "calcSignedArcLength: %s", e.what());
      return;
    }

    try {
      const double eY = autoware::motion_utils::calcLateralOffset(trajectory_->points, ego_pose.position);
      eY_opt = eY;
    } catch (const std::exception & e) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "calcLateralOffset: %s", e.what());
      return;
    }

    autoware_planning_msgs::msg::TrajectoryPoint ref_point;
    try {
      ref_point = autoware::motion_utils::calcInterpolatedPoint(*trajectory_, ego_pose, false);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "calcInterpolatedPoint: %s", e.what());
      return;
    }

    const double ref_yaw = tf2::getYaw(ref_point.pose.orientation);
    const double ego_yaw = tf2::getYaw(ego_pose.orientation);
    ePsi_opt = normalizeAngle(ego_yaw - ref_yaw);

    const double wheelbase = wheelbase_lf_ + wheelbase_lr_;
    if (std::fabs(ref_point.front_wheel_angle_rad) > 1e-6) {
      kappa_ref_opt = std::tan(ref_point.front_wheel_angle_rad) / wheelbase;
    } else {
      const auto curvatures = autoware::motion_utils::calcCurvature(trajectory_->points);
      const auto nearest_idx = autoware::motion_utils::findNearestIndex(
        trajectory_->points, ego_pose.position);
      if (nearest_idx && !curvatures.empty()) {
        const size_t idx = std::min(nearest_idx, curvatures.size() - 1);
        kappa_ref_opt = curvatures[idx];
      } else {
        kappa_ref_opt = 0.0;
      }
    }
    ref_velocity_opt = static_cast<double>(ref_point.longitudinal_velocity_mps);

    if (!s_opt || !eY_opt || !ePsi_opt || !kappa_ref_opt || !ref_velocity_opt) {
      return;
    }

    std::array<double, NX> x0 = {
      *s_opt, v, a, *eY_opt, *ePsi_opt
    };

    AcadosSolution sol = interface_->getControlWithDelayCompensation(
      x0, delay_compensation_time_, *kappa_ref_opt, tau_equiv_, wheelbase_lf_, wheelbase_lr_);

    if (sol.status != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "MPC solve status=%d iter=%d", sol.status, sol.sqp_iter);
      return;
    }

    double u_cmd = sol.utraj[0][0];
    double delta = sol.utraj[0][1];
    delta = std::clamp(delta, -max_steer_rad_, max_steer_rad_);

    autoware_control_msgs::msg::Control out;
    out.stamp = this->now();
    out.control_time = out.stamp;

    out.lateral.stamp = out.stamp;
    out.lateral.control_time = out.stamp;
    out.lateral.steering_tire_angle = static_cast<float>(delta);
    out.lateral.steering_tire_rotation_rate = 0.0f;
    out.lateral.is_defined_steering_tire_rotation_rate = false;

    out.longitudinal.stamp = out.stamp;
    out.longitudinal.control_time = out.stamp;
    out.longitudinal.velocity = static_cast<float>(*ref_velocity_opt);
    out.longitudinal.acceleration = static_cast<float>(u_cmd);
    out.longitudinal.jerk = 0.0f;
    out.longitudinal.is_defined_acceleration = true;
    out.longitudinal.is_defined_jerk = false;

    pub_control_->publish(out);

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 200,
      "MPC: u_cmd=%.2f delta=%.3f v_ref=%.2f status=%d time=%.1f ms",
      u_cmd, delta, *ref_velocity_opt, sol.status, sol.elapsed_time * 1000.0);
  }

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr sub_accel_;
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_control_;
  rclcpp::TimerBase::SharedPtr timer_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr steering_;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr accel_;

  std::unique_ptr<AcadosInterface> interface_;
  bool enable_controller_;
  double ctrl_period_;
  double delay_compensation_time_;
  double tau_equiv_;
  double wheelbase_lf_;
  double wheelbase_lr_;
  double max_steer_rad_;
  double timeout_trajectory_sec_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<MpcControllerNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
