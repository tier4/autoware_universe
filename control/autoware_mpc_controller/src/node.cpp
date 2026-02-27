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
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

namespace
{
constexpr double kPi = 3.141592653589793;
constexpr double kHorizonTf = 10.0;  // Must match OCP horizon in generators/mpc.py.
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
    steer_tau_ = declare_parameter<double>("steer_tau", 0.27);
    wheelbase_lf_ = declare_parameter<double>("wheelbase_lf", 1.395);
    wheelbase_lr_ = declare_parameter<double>("wheelbase_lr", 1.395);
    max_steer_rad_ = declare_parameter<double>("max_steer_rad", M_PI/3);
    timeout_trajectory_sec_ = declare_parameter<double>("timeout_trajectory_sec", 1.0);
    accel_state_lpf_alpha_ = declare_parameter<double>("accel_state_lpf_alpha", 0.8);

    const int max_iter = declare_parameter<int>("mpc_max_iter", 20);
    const double tol = declare_parameter<double>("mpc_tol", 1e-4);
    solver_verbose_ = declare_parameter<bool>("solver_verbose", false);

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
    pub_predicted_trajectory_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
      "~/output/predicted_trajectory", rclcpp::QoS(1).transient_local());
    pub_current_steering_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
      "~/output/current_steering", rclcpp::QoS(1).transient_local());

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(ctrl_period_)),
      std::bind(&MpcControllerNode::onTimer, this));
  }

private:
  double sampleCurvatureAtArcLength(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
    const std::vector<double> & curvatures, const double s) const
  {
    if (points.size() < 2 || curvatures.empty()) {
      return 0.0;
    }
    try {
      const auto ref_pose = autoware::motion_utils::calcInterpolatedPose(points, std::max(0.0, s));
      const size_t idx = autoware::motion_utils::findNearestIndex(points, ref_pose.position);
      return curvatures.at(std::min(idx, curvatures.size() - 1));
    } catch (const std::exception &) {
      return curvatures.back();
    }
  }

  void onTimer()
  {
    if (!enable_controller_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[MPC] Controller disabled.");
      return;
    }
    if (!trajectory_ || !odometry_ || !steering_ || !accel_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[MPC] Skip: waiting for data (traj=%d odom=%d steer=%d accel=%d)",
        trajectory_ ? 1 : 0, odometry_ ? 1 : 0, steering_ ? 1 : 0, accel_ ? 1 : 0);
      return;
    }
    if (trajectory_->points.size() < 2) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[MPC] Skip: trajectory too short (size=%zu)", trajectory_->points.size());
      return;
    }
    const double traj_age = (this->now() - rclcpp::Time(trajectory_->header.stamp)).seconds();
    if (traj_age > timeout_trajectory_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[MPC] Skip: trajectory too old (%.1f s > %.1f s)", traj_age, timeout_trajectory_sec_);
      return;
    }

    const geometry_msgs::msg::Pose ego_pose = odometry_->pose.pose;
    const double v = odometry_->twist.twist.linear.x;
    const double steer_meas =
      std::clamp(static_cast<double>(steering_->steering_tire_angle), -max_steer_rad_, max_steer_rad_);
    const double a_meas = accel_->accel.accel.linear.x;
    if (!a_state_initialized_) {
      a_state_filtered_ = a_meas;
      a_state_initialized_ = true;
      // Use measured acceleration as initial hold command to avoid startup delay-predict mismatch.
      last_u_cmd_ = a_meas;
    } else {
      a_state_filtered_ =
        accel_state_lpf_alpha_ * a_state_filtered_ + (1.0 - accel_state_lpf_alpha_) * a_meas;
    }
    const double a = a_state_filtered_;

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
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "[MPC] Skip: calcSignedArcLength failed: %s", e.what());
      return;
    }

    try {
      const double eY = autoware::motion_utils::calcLateralOffset(trajectory_->points, ego_pose.position);
      eY_opt = eY;
    } catch (const std::exception & e) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "[MPC] Skip: calcLateralOffset failed: %s", e.what());
      return;
    }

    autoware_planning_msgs::msg::TrajectoryPoint ref_point;
    try {
      ref_point = autoware::motion_utils::calcInterpolatedPoint(*trajectory_, ego_pose, false);
    } catch (const std::exception & e) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "[MPC] Skip: calcInterpolatedPoint failed: %s", e.what());
      return;
    }

    const double ref_yaw = tf2::getYaw(ref_point.pose.orientation);
    const double ego_yaw = tf2::getYaw(ego_pose.orientation);
    ePsi_opt = normalizeAngle(ego_yaw - ref_yaw);

    const auto curvatures = autoware::motion_utils::calcCurvature(trajectory_->points);
    if (!curvatures.empty()) {
      const size_t nearest_idx =
        autoware::motion_utils::findNearestIndex(trajectory_->points, ego_pose.position);
      const size_t idx = std::min(nearest_idx, curvatures.size() - 1);
      kappa_ref_opt = curvatures[idx];
    } else {
      kappa_ref_opt = 0.0;
    }
    ref_velocity_opt = static_cast<double>(ref_point.longitudinal_velocity_mps);

    if (!s_opt || !eY_opt || !ePsi_opt || !kappa_ref_opt || !ref_velocity_opt) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[MPC] Skip: missing state (s=%d eY=%d ePsi=%d kappa=%d v_ref=%d)",
        s_opt ? 1 : 0, eY_opt ? 1 : 0, ePsi_opt ? 1 : 0,
        kappa_ref_opt ? 1 : 0, ref_velocity_opt ? 1 : 0);
      return;
    }

    std::array<double, NX> x0 = {
      *s_opt, v, a, *eY_opt, *ePsi_opt, steer_meas
    };

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "[MPC] state s=%.1f v=%.2f a=%.2f eY=%.3f ePsi=%.3f kappa_ref=%.4f v_ref=%.2f",
      x0[0], x0[1], x0[2], x0[3], x0[4], *kappa_ref_opt, *ref_velocity_opt);

    const std::array<double, NU> u_hold = {last_u_cmd_, last_delta_};
    std::array<double, NX> x_for_mpc = x0;
    if (delay_compensation_time_ > 0.0) {
      const std::array<double, NP> p_now = {
        tau_equiv_, *kappa_ref_opt, wheelbase_lf_, wheelbase_lr_, steer_tau_};
      x_for_mpc = interface_->predictStateAfterDelay(x0, delay_compensation_time_, p_now, u_hold);
    }
    const double s_base = x_for_mpc[0];
    interface_->setCostReference(s_base, *ref_velocity_opt);
    const double dt_h = kHorizonTf / static_cast<double>(N);
    for (size_t stage = 0; stage <= N; ++stage) {
      const double s_stage = s_base + *ref_velocity_opt * static_cast<double>(stage) * dt_h;
      const double kappa_stage = sampleCurvatureAtArcLength(trajectory_->points, curvatures, s_stage);
      interface_->setParameters(
        static_cast<int>(stage), {tau_equiv_, kappa_stage, wheelbase_lf_, wheelbase_lr_, steer_tau_});
    }
    AcadosSolution sol = interface_->getControl(x_for_mpc);

    if (sol.status != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[MPC] Solve failed: status=%d sqp_iter=%d time=%.1f ms (not publishing control)",
        sol.status, sol.sqp_iter, sol.elapsed_time * 1000.0);
      return;
    }

    double u_cmd = sol.utraj[0][0];
    double delta = sol.utraj[0][1];
    delta = std::clamp(delta, -max_steer_rad_, max_steer_rad_);
    last_u_cmd_ = u_cmd;
    last_delta_ = delta;

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
    if (steering_) {
      pub_current_steering_->publish(*steering_);
    }

    // Publish predicted trajectory (for AEB, lane_departure_checker, control_validator)
    autoware_planning_msgs::msg::Trajectory pred_traj;
    pred_traj.header.stamp = this->now();
    pred_traj.header.frame_id = trajectory_->header.frame_id;
    pred_traj.points.reserve(sol.xtraj.size());
    for (size_t i = 0; i < sol.xtraj.size(); ++i) {
      const double s = sol.xtraj[i][0];
      const double v = sol.xtraj[i][1];
      const double a = sol.xtraj[i][2];
      const double eY = sol.xtraj[i][3];
      const double ePsi = sol.xtraj[i][4];
      const double steer = sol.xtraj[i][5];
      geometry_msgs::msg::Pose ref_pose;
      if (trajectory_->points.size() < 2 || s < 0.0) {
        ref_pose = trajectory_->points.front().pose;
      } else {
        ref_pose = autoware::motion_utils::calcInterpolatedPose(trajectory_->points, s);
      }
      const double ref_yaw = tf2::getYaw(ref_pose.orientation);
      const double nx = -std::sin(ref_yaw);
      const double ny = std::cos(ref_yaw);
      geometry_msgs::msg::Pose world_pose;
      world_pose.position.x = ref_pose.position.x + nx * eY;
      world_pose.position.y = ref_pose.position.y + ny * eY;
      world_pose.position.z = ref_pose.position.z;
      world_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), ref_yaw + ePsi));
      autoware_planning_msgs::msg::TrajectoryPoint pt;
      pt.pose = world_pose;
      pt.longitudinal_velocity_mps = static_cast<float>(v);
      pt.acceleration_mps2 = static_cast<float>(a);
      pt.front_wheel_angle_rad = static_cast<float>(steer);
      pred_traj.points.push_back(pt);
    }
    pub_predicted_trajectory_->publish(pred_traj);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "[MPC] Published: accel=%.2f steering=%.3f v_ref=%.2f (solve %.1f ms)",
      u_cmd, delta, *ref_velocity_opt, sol.elapsed_time * 1000.0);

    if (solver_verbose_ && !sol.solver_stats.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "[MPC] SQP stats:\n%s", sol.solver_stats.c_str());
    }
    if (solver_verbose_ && !sol.lam_summary.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "[MPC] Inequality lambdas (active if non-zero):\n%s",
        sol.lam_summary.c_str());
    }
  }

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr sub_accel_;
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_control_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_predicted_trajectory_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_current_steering_;
  rclcpp::TimerBase::SharedPtr timer_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr steering_;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr accel_;

  std::unique_ptr<AcadosInterface> interface_;
  bool enable_controller_;
  bool solver_verbose_;
  double ctrl_period_;
  double delay_compensation_time_;
  double tau_equiv_;
  double steer_tau_;
  double wheelbase_lf_;
  double wheelbase_lr_;
  double max_steer_rad_;
  double timeout_trajectory_sec_;
  double accel_state_lpf_alpha_;
  double a_state_filtered_{0.0};
  bool a_state_initialized_{false};
  double last_u_cmd_{0.0};
  double last_delta_{0.0};
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
