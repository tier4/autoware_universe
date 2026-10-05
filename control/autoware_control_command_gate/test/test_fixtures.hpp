// Copyright 2026 The Autoware Contributors
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

#ifndef TEST_FIXTURES_HPP_
#define TEST_FIXTURES_HPP_

#include "command/filter.hpp"
#include "command/interface.hpp"
#include "test_utils.hpp"

#include <autoware_command_mode_types/sources.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_command_gate::test
{

constexpr uint16_t builtin_id = autoware::command_mode_types::sources::builtin;
constexpr uint16_t main_id = 12;
constexpr uint16_t in_lane_stop_id = 31;

struct VehicleState
{
  double speed = 0.0;
  double steer = 0.0;
  uint8_t mode = ControlModeReport::AUTONOMOUS;
};

inline rosgraph_msgs::msg::Clock make_clock(const double t)
{
  rosgraph_msgs::msg::Clock msg;
  msg.clock = rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)), RCL_ROS_TIME);
  return msg;
}

inline bool spin_until(
  rclcpp::Executor & executor, const std::function<bool()> & condition,
  const std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    executor.spin_some(std::chrono::milliseconds(10));
  }
  return true;
}

inline bool reached_time(const rclcpp::Node & node, const double t)
{
  return node.now().nanoseconds() == static_cast<int64_t>(std::llround(t * 1e9));
}

inline bool publish_clock_until_reached(
  rclcpp::Executor & executor, rclcpp::Publisher<rosgraph_msgs::msg::Clock> & publisher,
  const rclcpp::Node & node, const double t)
{
  for (int i = 0; i < 100; ++i) {
    publisher.publish(make_clock(t));
    if (spin_until(
          executor, [&node, t]() { return reached_time(node, t); },
          std::chrono::milliseconds(50))) {
      return true;
    }
  }
  return false;
}

class RecordingOutput : public CommandOutput
{
public:
  void on_control(uint16_t, const Control & msg) override { controls.push_back(msg); }
  void on_gear(const GearCommand &) override {}
  void on_turn_indicators(const TurnIndicatorsCommand &) override {}
  void on_hazard_lights(const HazardLightsCommand &) override {}

  std::vector<Control> controls;
};

class FilterFixture
{
public:
  FilterFixture(const VehicleCmdFilterParam & nominal, const VehicleCmdFilterParam & transition)
  {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.use_clock_thread(false);
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("enable_command_limit_filter", true),
      rclcpp::Parameter("stop_check_duration", 1.0),
    });
    node_ = std::make_shared<rclcpp::Node>("command_filter_test", options);
    pub_clock_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    pub_kinematics_ = node_->create_publisher<Odometry>("/localization/kinematic_state", 1);
    pub_acceleration_ =
      node_->create_publisher<AccelWithCovarianceStamped>("/localization/acceleration", 1);
    pub_steering_ = node_->create_publisher<SteeringReport>("/vehicle/status/steering_status", 1);
    pub_control_mode_ =
      node_->create_publisher<ControlModeReport>("/vehicle/status/control_mode", 1);

    auto output = std::make_unique<RecordingOutput>();
    output_ = output.get();
    filter_ = std::make_unique<CommandFilter>(std::move(output), *node_);
    filter_->set_nominal_filter_params(nominal);
    filter_->set_transition_filter_params(transition);
    executor_.add_node(node_);
  }

  ~FilterFixture() { executor_.remove_node(node_); }

  bool set_time(const double t)
  {
    return publish_clock_until_reached(executor_, *pub_clock_, *node_, t);
  }

  void set_state(const VehicleState & state)
  {
    Odometry kinematics;
    kinematics.twist.twist.linear.x = state.speed;
    SteeringReport steering;
    steering.steering_tire_angle = static_cast<float>(state.steer);
    ControlModeReport control_mode;
    control_mode.mode = state.mode;
    pub_kinematics_->publish(kinematics);
    pub_acceleration_->publish(AccelWithCovarianceStamped());
    pub_steering_->publish(steering);
    pub_control_mode_->publish(control_mode);
    for (int i = 0; i < 3; ++i) {
      executor_.spin_some(std::chrono::milliseconds(10));
    }
  }

  Control step(const uint16_t source_id, const double t, const Control & cmd)
  {
    if (!set_time(t)) {
      throw std::runtime_error("sim time did not reach " + std::to_string(t));
    }
    filter_->on_control(source_id, cmd);
    return output_->controls.back();
  }

  CommandFilter & filter() { return *filter_; }
  const RecordingOutput & output() const { return *output_; }

  void enable_diag() { diag_ = filter_->create_diag_task(); }

  uint8_t run_diag()
  {
    diagnostic_updater::DiagnosticStatusWrapper stat;
    static_cast<diagnostic_updater::DiagnosticTask &>(*diag_).run(stat);
    return stat.level;
  }

private:
  ClipDiag * diag_ = nullptr;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_kinematics_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acceleration_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr pub_control_mode_;
  std::unique_ptr<CommandFilter> filter_;
  RecordingOutput * output_;
};

}  // namespace autoware::control_command_gate::test

#endif  // TEST_FIXTURES_HPP_
