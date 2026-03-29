// Copyright 2025 The Autoware Contributors
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

#include "pipeline_latency_monitor_node.hpp"

#include <autoware_planning_validator/msg/planning_validator_status.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <deque>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::system::pipeline_latency_monitor
{

namespace
{

bool has_valid_data(const std::deque<ProcessData> & history)
{
  return !history.empty();
}

double get_latest_value(const std::deque<ProcessData> & history)
{
  if (!has_valid_data(history)) {
    return 0.0;
  }
  const double value = history.back().latency_ms;
  // Return 0.0 for negative values
  return value < 0.0 ? 0.0 : value;
}

rclcpp::Time get_latest_timestamp(const std::deque<ProcessData> & history)
{
  if (history.empty()) {
    return rclcpp::Time(0);
  }
  return history.back().timestamp;
}

};  // namespace

PipelineLatencyMonitorNode::PipelineLatencyMonitorNode(const rclcpp::NodeOptions & options)
: agnocast::Node("pipeline_latency_monitor", options)
{
  update_rate_ = declare_parameter<double>("update_rate");
  latency_threshold_ms_ = declare_parameter<double>("latency_threshold_ms");
  window_size_ = declare_parameter<int>("window_size");

  const auto processing_steps =
    declare_parameter<std::vector<std::string>>("processing_steps.sequence");
  for (const auto & step : processing_steps) {
    const auto topic = declare_parameter<std::string>("processing_steps." + step + ".topic");
    const auto topic_type =
      declare_parameter<std::string>("processing_steps." + step + ".topic_type");
    const auto timestamp_meaning =
      declare_parameter<std::string>("processing_steps." + step + ".timestamp_meaning");
    const auto latency_multiplier =
      declare_parameter<double>("processing_steps." + step + ".latency_multiplier");
    const auto index = input_sequence_.size();
    ProcessInput & input = input_sequence_.emplace_back();
    input.name = step;
    input.topic = topic;
    input.topic_type = topic_type;
    input.timestamp_meaning =
      timestamp_meaning == "start" ? TimestampMeaning::start : TimestampMeaning::end;
    input.latency_multiplier = latency_multiplier;

    // WORKAROUND: agnocast has no GenericSubscription. Only 2 types used.
    // TODO(agnocast): remove workaround when agnocast adds GenericSubscription
    if (topic_type == "autoware_internal_debug_msgs/msg/Float64Stamped") {
      auto sub =
        create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
          topic, rclcpp::QoS(1),
          [this, index](
            const agnocast::ipc_shared_ptr<
              const autoware_internal_debug_msgs::msg::Float64Stamped> & msg) {
            auto & input = this->input_sequence_[index];
            this->update_history(
              input.latency_history, msg->stamp, msg->data * input.latency_multiplier);
            RCLCPP_DEBUG(
              get_logger(), "Received %s: %.2f", input.name.c_str(),
              msg->data * input.latency_multiplier);
          });
      typed_subscribers_.push_back(sub);
    } else if (topic_type == "autoware_planning_validator/msg/PlanningValidatorStatus") {
      auto sub =
        create_subscription<autoware_planning_validator::msg::PlanningValidatorStatus>(
          topic, rclcpp::QoS(1),
          [this, index](
            const agnocast::ipc_shared_ptr<
              const autoware_planning_validator::msg::PlanningValidatorStatus> & msg) {
            auto & input = this->input_sequence_[index];
            this->update_history(
              input.latency_history, msg->stamp, msg->latency * input.latency_multiplier);
            RCLCPP_DEBUG(
              get_logger(), "Received %s: %.2f", input.name.c_str(),
              msg->latency * input.latency_multiplier);
          });
      typed_subscribers_.push_back(sub);
    } else {
      throw std::runtime_error("unsupported message type " + topic_type);
    }

    // Create debug publisher for this step
    debug_step_pubs_.push_back(
      create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "~/debug/" + step + "_latency_ms", 10));
  }
  latency_offsets_ = declare_parameter<std::vector<double>>("latency_offsets_ms");

  total_latency_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/output/total_latency_ms", 10);

  debug_total_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/pipeline_total_latency_ms", 10);

  const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));
  timer_ = create_wall_timer(period, std::bind(&PipelineLatencyMonitorNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "PipelineLatencyMonitorNode initialized");
}

void PipelineLatencyMonitorNode::on_timer()
{
  calculate_total_latency();

  publish_total_latency();
}

void PipelineLatencyMonitorNode::calculate_total_latency()
{
  if (input_sequence_.empty()) {
    return;
  }
  const auto to_duration = [](const double latency_ms) {
    return rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(latency_ms * 1e6));
  };
  std::stringstream debug_ss;
  total_latency_ms_ = 0.0;
  rclcpp::Time start_of_next_step;
  // we go through the steps in reverse
  auto step_it = input_sequence_.rbegin();
  // use the latest latency of the last step in the sequence without any constraint
  const auto & input = *step_it;
  if (has_valid_data(input.latency_history)) {
    const auto latency = get_latest_value(input.latency_history);
    total_latency_ms_ += latency;
    debug_ss << step_it->name << "=" << latency;
    start_of_next_step = get_latest_timestamp(input.latency_history);
    if (input.timestamp_meaning == TimestampMeaning::end) {
      start_of_next_step -= to_duration(latency);
    }
  } else {
    // skip step if no data available
    debug_ss << step_it->name << "=skipped";
    start_of_next_step = now();
  }
  // we go through the rest of the sequence in reverse order with the following constraint:
  // end of current step < start of next step
  for (step_it++; step_it != input_sequence_.rend(); ++step_it) {
    const auto & step_input = *step_it;
    bool found_valid_data = false;
    if (has_valid_data(step_input.latency_history)) {
      for (auto it = step_input.latency_history.rbegin(); it != step_input.latency_history.rend();
           ++it) {
        const rclcpp::Time end_of_current_step =
          it->timestamp + (step_input.timestamp_meaning == TimestampMeaning::start
                             ? to_duration(it->latency_ms)
                             : to_duration(0.0));
        if (is_timestamp_older(end_of_current_step, start_of_next_step)) {
          total_latency_ms_ += it->latency_ms;
          start_of_next_step = it->timestamp;
          debug_ss << " + " << step_it->name << "=" << it->latency_ms;
          if (step_input.timestamp_meaning == TimestampMeaning::end) {
            start_of_next_step -= to_duration(it->latency_ms);
          }
          found_valid_data = true;
          break;
        }
      }
    }
    // skip step if no valid data found
    if (!found_valid_data) {
      debug_ss << " + " << step_it->name << "=skipped";
    }
  }
  RCLCPP_DEBUG(
    get_logger(), "Total latency calculation (cumulative time-ordered): %s = %2.2fms",
    debug_ss.str().c_str(), total_latency_ms_);

  std::stringstream ss;
  ss << "offsets added to the total: [ ";
  for (const auto offset : latency_offsets_) {
    ss << offset << " ";
    total_latency_ms_ += offset;
  }
  ss << "]";

  RCLCPP_DEBUG(
    get_logger(), "Total latency with offsets: %.2f ms (%s)", total_latency_ms_, ss.str().c_str());
}

void PipelineLatencyMonitorNode::publish_total_latency()
{
  auto total_latency_msg = total_latency_pub_->borrow_loaned_message();
  total_latency_msg->stamp = now();
  total_latency_msg->data = total_latency_ms_;
  total_latency_pub_->publish(std::move(total_latency_msg));

  // Publish latest latency values from each topic as debug information
  for (size_t i = 0; i < input_sequence_.size(); ++i) {
    const auto latest_latency = get_latest_value(input_sequence_[i].latency_history);
    auto debug_msg = debug_step_pubs_[i]->borrow_loaned_message();
    debug_msg->stamp = now();
    debug_msg->data = latest_latency;
    debug_step_pubs_[i]->publish(std::move(debug_msg));
  }

  {
    auto debug_total_msg = debug_total_pub_->borrow_loaned_message();
    debug_total_msg->stamp = now();
    debug_total_msg->data = total_latency_ms_;
    debug_total_pub_->publish(std::move(debug_total_msg));
  }

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000, "Total latency: %.2f ms (threshold: %.2f ms)",
    total_latency_ms_, latency_threshold_ms_);
}

void PipelineLatencyMonitorNode::update_history(
  std::deque<ProcessData> & history, const rclcpp::Time & timestamp, double value)
{
  if (value < 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 30000,
      "Negative latency value detected: %.6f, treating as 0.0", value);
    value = 0.0;
  }

  // Add new value to history
  history.emplace_back(timestamp, value);

  // Remove old data if window size is exceeded
  while (history.size() > window_size_) {
    history.pop_front();
  }
}

bool PipelineLatencyMonitorNode::is_timestamp_older(
  const rclcpp::Time & timestamp1, const rclcpp::Time & timestamp2) const
{
  try {
    return timestamp1 < timestamp2;
  } catch (const std::runtime_error & e) {
    // If timestamps have different time sources, compare nanoseconds directly
    RCLCPP_DEBUG(get_logger(), "Timestamp comparison failed, using nanoseconds: %s", e.what());
    return timestamp1.nanoseconds() < timestamp2.nanoseconds();
  }
}

}  // namespace autoware::system::pipeline_latency_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::system::pipeline_latency_monitor::PipelineLatencyMonitorNode)
