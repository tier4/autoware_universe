// Copyright 2026 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autoware/data_driven_planning_simulator/data_driven_planning_simulator_node.hpp"

#include "autoware_utils_geometry/geometry.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::simulator::data_driven_planning_simulator
{
namespace
{
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

DataDrivenPlanningSimulator::DataDrivenPlanningSimulator(const rclcpp::NodeOptions & options)
: Node("data_driven_planning_simulator", options)
{
  origin_frame_id_ = declare_parameter<std::string>("origin_frame_id", "odom");
  simulated_frame_id_ = declare_parameter<std::string>("simulated_frame_id", "base_link");
  command_type_ = declare_parameter<std::string>("command_type", "ackermann");
  learned_backend_ = declare_parameter<std::string>("learned_backend", "none");
  learned_model_type_ = declare_parameter<std::string>("learned_model_type", "direct_next_state");
  learned_weights_csv_ = declare_parameter<std::string>("learned_weights_csv", "");
  mlp_w1_csv_ = declare_parameter<std::string>("mlp_w1_csv", "");
  mlp_b1_csv_ = declare_parameter<std::string>("mlp_b1_csv", "");
  mlp_w2_csv_ = declare_parameter<std::string>("mlp_w2_csv", "");
  mlp_b2_csv_ = declare_parameter<std::string>("mlp_b2_csv", "");
  mlp_x_mean_csv_ = declare_parameter<std::string>("mlp_x_mean_csv", "");
  mlp_x_std_csv_ = declare_parameter<std::string>("mlp_x_std_csv", "");
  mlp_y_mean_csv_ = declare_parameter<std::string>("mlp_y_mean_csv", "");
  mlp_y_std_csv_ = declare_parameter<std::string>("mlp_y_std_csv", "");
  const auto baseline_type = declare_parameter<std::string>("baseline_model_type", "ackermann");
  timer_period_s_ = declare_parameter<double>("timer_period_s", 0.03);

  BaselineModelParams params;
  params.type = parse_baseline_model_type(baseline_type);
  params.wheelbase = declare_parameter<double>("wheelbase", 2.7);
  params.track_width = declare_parameter<double>("track_width", 1.5);
  params.wheel_radius = declare_parameter<double>("wheel_radius", 0.3);
  params.steer_time_constant = declare_parameter<double>("steer_time_constant", 0.0);
  params.use_wheel_command = declare_parameter<bool>("use_wheel_command", false);

  model_ = std::make_unique<BaselineModel>(params);
  model_->reset(initial_state_);
  if (learned_backend_ == "linear_cpu" && !learned_weights_csv_.empty() && load_linear_weights(learned_weights_csv_)) {
    RCLCPP_INFO(get_logger(), "Loaded deterministic linear CPU weights from %s", learned_weights_csv_.c_str());
  } else if (learned_backend_ == "numpy_mlp_cpu" && load_mlp_weights()) {
    RCLCPP_INFO(get_logger(), "Loaded deterministic MLP CPU weights");
  } else if (!learned_weights_csv_.empty()) {
    RCLCPP_WARN(get_logger(), "Failed to load learned weights; falling back to baseline model");
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  using namespace std::chrono_literals;

  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("output/odometry", rclcpp::QoS{1});
  pub_velocity_ =
    create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("output/twist", rclcpp::QoS{1});
  pub_steering_ =
    create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("output/steering", rclcpp::QoS{1});
  pub_tf_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS{1});

  sub_initial_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input/initialpose", rclcpp::QoS{1}, std::bind(&DataDrivenPlanningSimulator::on_initial_pose, this, _1));

  if (command_type_ == "cmd_vel" || command_type_ == "twist") {
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
      "input/cmd_vel", rclcpp::QoS{1}, std::bind(&DataDrivenPlanningSimulator::on_twist_command, this, _1));
  } else {
    sub_control_ = create_subscription<autoware_control_msgs::msg::Control>(
      "input/ackermann_control_command", rclcpp::QoS{1},
      std::bind(&DataDrivenPlanningSimulator::on_control_command, this, _1));
  }

  srv_reset_ = create_service<std_srvs::srv::Trigger>(
    "reset", std::bind(&DataDrivenPlanningSimulator::on_reset, this, _1, _2));

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(timer_period_s_),
    std::bind(&DataDrivenPlanningSimulator::on_timer, this));
}

void DataDrivenPlanningSimulator::on_control_command(
  const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
  command_.velocity = msg->longitudinal.velocity;
  command_.acceleration = msg->longitudinal.acceleration;
  command_.steer = msg->lateral.steering_tire_angle;
  command_.vx = msg->longitudinal.velocity;
  command_.wz = 0.0;
  model_->set_command(command_);
}

void DataDrivenPlanningSimulator::on_twist_command(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  command_.vx = msg->linear.x;
  command_.vy = msg->linear.y;
  command_.wz = msg->angular.z;
  command_.velocity = msg->linear.x;
  model_->set_command(command_);
}

void DataDrivenPlanningSimulator::on_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  initial_state_ = pose_to_state(*msg);
  model_->reset(initial_state_);
  initialized_time_ = false;
}

void DataDrivenPlanningSimulator::on_reset(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  model_->reset(initial_state_);
  initialized_time_ = false;
  response->success = true;
  response->message = "data-driven planning simulator state reset";
}

VehicleState DataDrivenPlanningSimulator::pose_to_state(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg) const
{
  VehicleState state;
  state.x = msg.pose.pose.position.x;
  state.y = msg.pose.pose.position.y;
  state.yaw = yaw_from_quaternion(msg.pose.pose.orientation);
  return state;
}

void DataDrivenPlanningSimulator::on_timer()
{
  const auto now = get_clock()->now();
  if (!initialized_time_) {
    previous_update_time_ = now;
    initialized_time_ = true;
    publish_state(now);
    return;
  }

  const double dt = std::clamp((now - previous_update_time_).seconds(), 0.0, 1.0);
  previous_update_time_ = now;
  const bool learned_applied =
    (learned_backend_ == "linear_cpu" && apply_linear_inference(dt)) ||
    (learned_backend_ == "numpy_mlp_cpu" && apply_mlp_inference(dt));
  if (!learned_applied) {
    model_->update(dt);
  }
  publish_state(now);
}

bool DataDrivenPlanningSimulator::load_linear_weights(const std::string & path)
{
  return load_matrix_csv(path, linear_weights_) && linear_weights_.size() == 13 &&
         linear_weights_.front().size() == 6;
}

bool DataDrivenPlanningSimulator::load_matrix_csv(
  const std::string & path, std::vector<std::vector<double>> & matrix)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return false;
  }
  matrix.clear();
  std::string line;
  while (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> row;
    while (std::getline(ss, cell, ',')) {
      row.push_back(std::stod(cell));
    }
    if (!row.empty()) {
      matrix.push_back(row);
    }
  }
  return !matrix.empty();
}

bool DataDrivenPlanningSimulator::load_mlp_weights()
{
  const bool loaded =
    load_matrix_csv(mlp_w1_csv_, mlp_w1_) &&
    load_matrix_csv(mlp_b1_csv_, mlp_b1_) &&
    load_matrix_csv(mlp_w2_csv_, mlp_w2_) &&
    load_matrix_csv(mlp_b2_csv_, mlp_b2_) &&
    load_matrix_csv(mlp_x_mean_csv_, mlp_x_mean_) &&
    load_matrix_csv(mlp_x_std_csv_, mlp_x_std_) &&
    load_matrix_csv(mlp_y_mean_csv_, mlp_y_mean_) &&
    load_matrix_csv(mlp_y_std_csv_, mlp_y_std_);
  if (!loaded || mlp_w1_.size() != 12 || mlp_w2_.empty() || mlp_w2_.front().size() != 6) {
    return false;
  }
  const auto hidden_size = mlp_w1_.front().size();
  return mlp_b1_.front().size() == hidden_size && mlp_w2_.size() == hidden_size &&
         mlp_b2_.front().size() == 6 && mlp_x_mean_.front().size() == 12 &&
         mlp_x_std_.front().size() == 12 && mlp_y_mean_.front().size() == 6 &&
         mlp_y_std_.front().size() == 6;
}

bool DataDrivenPlanningSimulator::apply_linear_inference([[maybe_unused]] const double dt)
{
  if (linear_weights_.empty()) {
    return false;
  }
  const auto state = model_->state();
  const std::vector<double> features{
    state.x, state.y, state.yaw, state.vx, state.vy, state.wz,
    command_.velocity, command_.acceleration, command_.steer,
    command_.vx, command_.vy, command_.wz, 1.0};
  if (features.size() != linear_weights_.size()) {
    return false;
  }
  VehicleState predicted = state;
  double outputs[6] = {};
  for (size_t i = 0; i < features.size(); ++i) {
    for (size_t j = 0; j < 6; ++j) {
      outputs[j] += features[i] * linear_weights_[i][j];
    }
  }
  predicted.x = outputs[0];
  predicted.y = outputs[1];
  predicted.yaw = std::atan2(std::sin(outputs[2]), std::cos(outputs[2]));
  predicted.vx = outputs[3];
  predicted.vy = outputs[4];
  predicted.wz = outputs[5];
  predicted.steer = command_.steer;
  model_->reset(predicted);
  return true;
}

bool DataDrivenPlanningSimulator::apply_mlp_inference([[maybe_unused]] const double dt)
{
  if (mlp_w1_.empty()) {
    return false;
  }
  const auto state = model_->state();
  const std::vector<double> raw_features{
    state.x, state.y, state.yaw, state.vx, state.vy, state.wz,
    command_.velocity, command_.acceleration, command_.steer,
    command_.vx, command_.vy, command_.wz};
  std::vector<double> x(raw_features.size(), 0.0);
  for (size_t i = 0; i < raw_features.size(); ++i) {
    x[i] = (raw_features[i] - mlp_x_mean_.front()[i]) / mlp_x_std_.front()[i];
  }
  const auto hidden_size = mlp_w1_.front().size();
  std::vector<double> hidden(hidden_size, 0.0);
  for (size_t h = 0; h < hidden_size; ++h) {
    double value = mlp_b1_.front()[h];
    for (size_t i = 0; i < x.size(); ++i) {
      value += x[i] * mlp_w1_[i][h];
    }
    hidden[h] = std::tanh(value);
  }
  double outputs[6] = {};
  for (size_t j = 0; j < 6; ++j) {
    double value = mlp_b2_.front()[j];
    for (size_t h = 0; h < hidden_size; ++h) {
      value += hidden[h] * mlp_w2_[h][j];
    }
    outputs[j] = value * mlp_y_std_.front()[j] + mlp_y_mean_.front()[j];
  }

  VehicleState predicted = state;
  if (learned_model_type_ == "mlp_residual") {
    predicted.x = state.x + outputs[0];
    predicted.y = state.y + outputs[1];
    predicted.yaw = std::atan2(std::sin(state.yaw + outputs[2]), std::cos(state.yaw + outputs[2]));
    predicted.vx = state.vx + outputs[3];
    predicted.vy = state.vy + outputs[4];
    predicted.wz = state.wz + outputs[5];
  } else {
    predicted.x = outputs[0];
    predicted.y = outputs[1];
    predicted.yaw = std::atan2(std::sin(outputs[2]), std::cos(outputs[2]));
    predicted.vx = outputs[3];
    predicted.vy = outputs[4];
    predicted.wz = outputs[5];
  }
  predicted.steer = command_.steer;
  model_->reset(predicted);
  return true;
}

void DataDrivenPlanningSimulator::publish_state(const rclcpp::Time & stamp)
{
  const auto state = model_->state();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = origin_frame_id_;
  odom.child_frame_id = simulated_frame_id_;
  odom.pose.pose.position.x = state.x;
  odom.pose.pose.position.y = state.y;
  odom.pose.pose.orientation = autoware_utils_geometry::create_quaternion_from_rpy(0.0, 0.0, state.yaw);
  odom.twist.twist.linear.x = state.vx;
  odom.twist.twist.linear.y = state.vy;
  odom.twist.twist.angular.z = state.wz;
  pub_odom_->publish(odom);

  autoware_vehicle_msgs::msg::VelocityReport velocity;
  velocity.header.stamp = stamp;
  velocity.longitudinal_velocity = static_cast<float>(state.vx);
  velocity.lateral_velocity = static_cast<float>(state.vy);
  velocity.heading_rate = static_cast<float>(state.wz);
  pub_velocity_->publish(velocity);

  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp = stamp;
  steering.steering_tire_angle = static_cast<float>(state.steer);
  pub_steering_->publish(steering);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = origin_frame_id_;
  tf.child_frame_id = simulated_frame_id_;
  tf.transform.translation.x = state.x;
  tf.transform.translation.y = state.y;
  tf.transform.rotation = odom.pose.pose.orientation;
  tf2_msgs::msg::TFMessage tf_msg;
  tf_msg.transforms.push_back(tf);
  pub_tf_->publish(tf_msg);
}

}  // namespace autoware::simulator::data_driven_planning_simulator

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::simulator::data_driven_planning_simulator::DataDrivenPlanningSimulator)
