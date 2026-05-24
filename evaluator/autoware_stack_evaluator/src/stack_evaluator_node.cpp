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

#include "autoware/stack_evaluator/stack_evaluator_node.hpp"

#include <autoware_utils/ros/parameter.hpp>

#include <functional>
#include <string>
#include <vector>

namespace autoware::stack_evaluator
{

StackEvaluatorNode::StackEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("stack_evaluator", node_options),
  plugin_loader_(
    "autoware_stack_evaluator",
    "autoware::stack_evaluator::plugin::EvaluatorPluginBase")
{
  processing_time_pub_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);

  set_up_params();
  initialize_plugins();

  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&StackEvaluatorNode::on_parameter, this, std::placeholders::_1));

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&StackEvaluatorNode::onTimer, this));
}

void StackEvaluatorNode::set_up_params()
{
  using autoware_utils_rclcpp::get_or_declare_parameter;

  const std::vector<std::string> default_plugins = {
    "autoware::stack_evaluator::plugin::ControlMetricsPlugin",
    "autoware::stack_evaluator::plugin::PlanningMetricsPlugin",
  };
  declare_parameter("plugin_names", default_plugins);

  use_control_evaluator_ =
    get_or_declare_parameter<bool>(*this, "use_control_evaluator");
  use_planning_evaluator_ =
    get_or_declare_parameter<bool>(*this, "use_planning_evaluator");
}

void StackEvaluatorNode::initialize_plugins()
{
  if (initialized_plugins_) {
    return;
  }

  const auto plugin_names = get_parameter("plugin_names").as_string_array();

  for (const auto & plugin_name : plugin_names) {
    load_plugin(plugin_name);
  }
  initialized_plugins_ = true;
}

void StackEvaluatorNode::load_plugin(const std::string & plugin_name)
{
  try {
    auto plugin = plugin_loader_.createSharedInstance(plugin_name);

    for (const auto & p : plugins_) {
      if (plugin->get_name() == p->get_name()) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Plugin '" << plugin_name << "' is already loaded.");
        return;
      }
    }

    plugin->initialize(plugin_name, this);
    plugins_.push_back(plugin);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded plugin: " << plugin_name);
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to load plugin '" << plugin_name << "': " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Unexpected error loading plugin '" << plugin_name << "': " << e.what());
  }
}

rcl_interfaces::msg::SetParametersResult StackEvaluatorNode::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param<bool>(parameters, "use_control_evaluator", use_control_evaluator_);
  update_param<bool>(parameters, "use_planning_evaluator", use_planning_evaluator_);

  for (auto & plugin : plugins_) {
    plugin->on_parameter(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void StackEvaluatorNode::fetchSharedData(plugin::EvaluatorData & data)
{
  data.odometry = odometry_sub_.take_data();
  data.acceleration = accel_sub_.take_data();
  data.trajectory = traj_sub_.take_data();
  data.objects = objects_sub_.take_data();
  data.route = route_sub_.take_data();
  data.lanelet_map_bin = lanelet_map_bin_sub_.take_data();
  data.behavior_path = behavior_path_sub_.take_data();
  data.steering = steering_sub_.take_data();
  data.blinker = blinker_sub_.take_data();
}

void StackEvaluatorNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  plugin::EvaluatorData data;
  fetchSharedData(data);

  for (auto & plugin : plugins_) {
    const auto & name = plugin->get_name();
    if (name.find("ControlMetrics") != std::string::npos && !use_control_evaluator_) {
      continue;
    }
    if (name.find("PlanningMetrics") != std::string::npos && !use_planning_evaluator_) {
      continue;
    }
    plugin->evaluate(data);
  }

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}

}  // namespace autoware::stack_evaluator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::stack_evaluator::StackEvaluatorNode)
