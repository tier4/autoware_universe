// Copyright 2026 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTOWARE__DATA_DRIVEN_PLANNING_SIMULATOR__DATA_DRIVEN_PLANNING_SIMULATOR_NODE_HPP_
#define AUTOWARE__DATA_DRIVEN_PLANNING_SIMULATOR__DATA_DRIVEN_PLANNING_SIMULATOR_NODE_HPP_

#include "autoware/data_driven_planning_simulator/models/baseline_model.hpp"
#include "autoware/data_driven_planning_simulator/visibility_control.hpp"

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::simulator::data_driven_planning_simulator
{

class DATA_DRIVEN_PLANNING_SIMULATOR_PUBLIC DataDrivenPlanningSimulator : public rclcpp::Node
{
public:
  explicit DataDrivenPlanningSimulator(const rclcpp::NodeOptions & options);

private:
  void on_timer();
  void on_control_command(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
  void on_twist_command(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void on_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void publish_state(const rclcpp::Time & stamp);
  VehicleState pose_to_state(const geometry_msgs::msg::PoseWithCovarianceStamped & msg) const;
  bool load_linear_weights(const std::string & path);
  bool load_matrix_csv(const std::string & path, std::vector<std::vector<double>> & matrix);
  bool load_mlp_weights();
  bool apply_linear_inference(double dt);
  bool apply_mlp_inference(double dt);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_control_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<ModelInterface> model_;
  VehicleCommand command_{};
  VehicleState initial_state_{};

  std::string origin_frame_id_{"odom"};
  std::string simulated_frame_id_{"base_link"};
  std::string command_type_{"ackermann"};
  std::string learned_backend_{"none"};
  std::string learned_model_type_{"direct_next_state"};
  std::string learned_weights_csv_{};
  std::vector<std::vector<double>> linear_weights_{};
  std::string mlp_w1_csv_{};
  std::string mlp_b1_csv_{};
  std::string mlp_w2_csv_{};
  std::string mlp_b2_csv_{};
  std::string mlp_x_mean_csv_{};
  std::string mlp_x_std_csv_{};
  std::string mlp_y_mean_csv_{};
  std::string mlp_y_std_csv_{};
  std::vector<std::vector<double>> mlp_w1_{};
  std::vector<std::vector<double>> mlp_b1_{};
  std::vector<std::vector<double>> mlp_w2_{};
  std::vector<std::vector<double>> mlp_b2_{};
  std::vector<std::vector<double>> mlp_x_mean_{};
  std::vector<std::vector<double>> mlp_x_std_{};
  std::vector<std::vector<double>> mlp_y_mean_{};
  std::vector<std::vector<double>> mlp_y_std_{};
  double timer_period_s_{0.03};
  rclcpp::Time previous_update_time_{0, 0, RCL_ROS_TIME};
  bool initialized_time_{false};
};

}  // namespace autoware::simulator::data_driven_planning_simulator

#endif  // AUTOWARE__DATA_DRIVEN_PLANNING_SIMULATOR__DATA_DRIVEN_PLANNING_SIMULATOR_NODE_HPP_
