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

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tier4_system_msgs/msg/command_source_status.hpp>
#include <tier4_system_msgs/srv/select_command_source.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_command_gate::test
{

constexpr uint16_t builtin_id = autoware::command_mode_types::sources::builtin;
constexpr uint16_t stop_id = 11;
constexpr uint16_t main_id = 12;
constexpr uint16_t local_id = 13;
constexpr uint16_t remote_id = 14;
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
  void on_control(uint16_t source_id, const Control & msg) override
  {
    source_ids.push_back(source_id);
    controls.push_back(msg);
  }
  void on_gear(const GearCommand &) override {}
  void on_turn_indicators(const TurnIndicatorsCommand &) override {}
  void on_hazard_lights(const HazardLightsCommand &) override {}

  std::vector<uint16_t> source_ids;
  std::vector<Control> controls;
};

class FilterFixture
{
public:
  FilterFixture(
    const VehicleCmdFilterParam & nominal, const VehicleCmdFilterParam & transition,
    const bool enable_command_limit_filter = true)
  {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.use_clock_thread(false);
    options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("enable_command_limit_filter", enable_command_limit_filter),
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

struct TimedControl
{
  double t;
  Control control;
};

class GateFixture
{
public:
  using SelectCommandSource = tier4_system_msgs::srv::SelectCommandSource;
  using CommandSourceStatus = tier4_system_msgs::msg::CommandSourceStatus;

  explicit GateFixture(const std::vector<rclcpp::Parameter> & overrides)
  {
    auto params = load_default_parameters();
    for (const auto & p : overrides) {
      params = remove_parameters(params, {p.get_name()});
      params.push_back(p);
    }
    params.push_back(rclcpp::Parameter("use_sim_time", true));
    gate_ = create_gate(params);

    driver_ = std::make_shared<rclcpp::Node>("control_command_gate_test_driver");
    pub_clock_ = driver_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    pub_kinematics_ = driver_->create_publisher<Odometry>("/localization/kinematic_state", 1);
    pub_acceleration_ =
      driver_->create_publisher<AccelWithCovarianceStamped>("/localization/acceleration", 1);
    pub_steering_ = driver_->create_publisher<SteeringReport>("/vehicle/status/steering_status", 1);
    pub_control_mode_ =
      driver_->create_publisher<ControlModeReport>("/vehicle/status/control_mode", 1);
    sub_control_mode_ = driver_->create_subscription<ControlModeReport>(
      "/vehicle/status/control_mode", 1,
      [this](const ControlModeReport::SharedPtr) { ++received_states_; });

    const auto inputs = find_parameter(params, "inputs").as_integer_array();
    for (const auto input : inputs) {
      const auto name = find_parameter(params, "inputs_names." + std::to_string(input)).as_string();
      pub_inputs_[static_cast<uint16_t>(input)] = driver_->create_publisher<Control>(
        "/control_command_gate/inputs/" + name + "/control", rclcpp::QoS(5));
    }
    sub_output_ = driver_->create_subscription<Control>(
      "/control_command_gate/output/control", rclcpp::QoS(5),
      [this](const Control::SharedPtr msg) { outputs_.push_back({gate_->now().seconds(), *msg}); });
    sub_status_ = driver_->create_subscription<CommandSourceStatus>(
      "/control_command_gate/source/status", rclcpp::QoS(1).transient_local(),
      [this](const CommandSourceStatus::SharedPtr msg) { source_ = msg->source; });
    client_select_ =
      driver_->create_client<SelectCommandSource>("/control_command_gate/source/select");
    sub_diagnostics_ = driver_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        for (const auto & status : msg->status) {
          diagnostic_levels_[status.name].push_back(status.level);
        }
      });

    executor_.add_node(gate_);
    executor_.add_node(driver_);

    const auto matched = [this]() {
      for (const auto & [id, pub] : pub_inputs_) {
        if (pub->get_subscription_count() == 0) return false;
      }
      return pub_clock_->get_subscription_count() > 0 &&
             pub_kinematics_->get_subscription_count() > 0 &&
             pub_steering_->get_subscription_count() > 0 &&
             pub_control_mode_->get_subscription_count() > 1 &&
             sub_output_->get_publisher_count() > 0 && client_select_->service_is_ready();
    };
    if (!spin_until(executor_, matched, std::chrono::milliseconds(10000))) {
      throw std::runtime_error("gate test driver could not connect");
    }
  }

  ~GateFixture()
  {
    executor_.remove_node(driver_);
    executor_.remove_node(gate_);
  }

  bool set_time(const double t)
  {
    const bool ok = publish_clock_until_reached(executor_, *pub_clock_, *gate_, t);
    for (int i = 0; i < 3; ++i) {
      executor_.spin_some(std::chrono::milliseconds(5));
    }
    return ok;
  }

  bool set_state(const VehicleState & state)
  {
    Odometry kinematics;
    kinematics.twist.twist.linear.x = state.speed;
    SteeringReport steering;
    steering.steering_tire_angle = static_cast<float>(state.steer);
    ControlModeReport control_mode;
    control_mode.mode = state.mode;
    const auto expected = received_states_ + 1;
    pub_kinematics_->publish(kinematics);
    pub_acceleration_->publish(AccelWithCovarianceStamped());
    pub_steering_->publish(steering);
    pub_control_mode_->publish(control_mode);
    const bool ok =
      spin_until(executor_, [this, expected]() { return received_states_ >= expected; });
    for (int i = 0; i < 5; ++i) {
      executor_.spin_some(std::chrono::milliseconds(10));
    }
    return ok;
  }

  bool select(const uint16_t source, const bool transition)
  {
    auto request = std::make_shared<SelectCommandSource::Request>();
    request->source = source;
    request->transition = transition;
    auto future = client_select_->async_send_request(request);
    if (!spin_until(executor_, [&future]() {
          return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        })) {
      return false;
    }
    return future.get()->status.success;
  }

  void publish(const uint16_t source, const Control & cmd) { pub_inputs_.at(source)->publish(cmd); }

  bool wait_outputs(const size_t count)
  {
    return spin_until(executor_, [this, count]() { return outputs_.size() >= count; });
  }

  void spin_for_a_while()
  {
    for (int i = 0; i < 5; ++i) {
      executor_.spin_some(std::chrono::milliseconds(10));
    }
  }

  const std::vector<TimedControl> & outputs() const { return outputs_; }
  uint16_t source() const { return source_; }

  bool wait_diagnostic_level(
    const std::string & name, const uint8_t level, const std::chrono::milliseconds timeout)
  {
    return spin_until(
      executor_,
      [this, &name, level]() {
        const auto it = diagnostic_levels_.find(name);
        return it != diagnostic_levels_.end() &&
               std::find(it->second.begin(), it->second.end(), level) != it->second.end();
      },
      timeout);
  }

private:
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diagnostics_;
  std::map<std::string, std::vector<uint8_t>> diagnostic_levels_;
  std::shared_ptr<ControlCmdGate> gate_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_kinematics_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acceleration_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr pub_control_mode_;
  rclcpp::Subscription<ControlModeReport>::SharedPtr sub_control_mode_;
  std::map<uint16_t, rclcpp::Publisher<Control>::SharedPtr> pub_inputs_;
  rclcpp::Subscription<Control>::SharedPtr sub_output_;
  rclcpp::Subscription<CommandSourceStatus>::SharedPtr sub_status_;
  rclcpp::Client<SelectCommandSource>::SharedPtr client_select_;
  std::vector<TimedControl> outputs_;
  size_t received_states_ = 0;
  uint16_t source_ = 0;
};

}  // namespace autoware::control_command_gate::test

#endif  // TEST_FIXTURES_HPP_
