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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__TIMESTAMPED_POLLING_SUBSCRIBER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__TIMESTAMPED_POLLING_SUBSCRIBER_HPP_

#include "autoware/diffusion_planner/utils/timestamp_selection.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cassert>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

/**
 * @brief Poll a ROS subscription while retaining the transport timestamp from MessageInfo.
 *
 * autoware_utils' polling subscriber intentionally exposes only the deserialized message. The
 * fixed-rate training contract, however, selects by the recorder/arrival clock, not by the message
 * header. `received_timestamp` is the closest runtime equivalent. When it is in a different clock
 * domain, a matching `source_timestamp` is preferred because some middleware/player combinations
 * preserve a source-clock timestamp even when the local receive timestamp is wall-clock time.
 * Rosbag playback still may not preserve the original bag timestamp in either DDS field, so the
 * current planner clock is used to detect a transport clock-domain mismatch and select the header
 * in that case. This small Node-local adapter preserves that timestamp without changing the shared
 * utility API. It uses the same non-executing callback-group pattern as
 * InterProcessPollingSubscriber and drains the DDS queue synchronously from the planner timer.
 */
template <typename MessageT>
class TimestampedPollingSubscriber
{
public:
  using MessagePtr = typename MessageT::ConstSharedPtr;
  using HeaderTimestampFunction = std::function<int64_t(const MessageT &)>;
  static constexpr int64_t CLOCK_DOMAIN_MISMATCH_NS = 10'000'000'000LL;

  struct TimedMessage
  {
    int64_t selection_time_ns{0};
    MessagePtr message;
    bool used_header_fallback{false};
  };

  TimestampedPollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    HeaderTimestampFunction header_timestamp)
  : header_timestamp_(std::move(header_timestamp))
  {
    auto noexec_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto noexec_subscription_options = rclcpp::SubscriptionOptions();
    noexec_subscription_options.callback_group = noexec_callback_group;

    subscriber_ = node->create_subscription<MessageT>(
      topic_name, qos, [node]([[maybe_unused]] const MessagePtr msg) { assert(false); },
      noexec_subscription_options);
  }

  [[nodiscard]] std::vector<TimedMessage> take_data(const int64_t reference_time_ns)
  {
    std::vector<TimedMessage> data;
    rclcpp::MessageInfo message_info;
    for (;;) {
      auto message = std::make_shared<MessageT>();
      if (!subscriber_->take(*message, message_info)) {
        break;
      }

      const auto & rmw_message_info = message_info.get_rmw_message_info();
      const int64_t received_timestamp = rmw_message_info.received_timestamp;
      const int64_t source_timestamp = rmw_message_info.source_timestamp;
      const int64_t header_timestamp = header_timestamp_ ? header_timestamp_(*message) : 0;
      const auto selection = utils::select_timestamp(
        received_timestamp, source_timestamp, header_timestamp, reference_time_ns,
        CLOCK_DOMAIN_MISMATCH_NS);
      data.push_back(TimedMessage{selection.time_ns, message, selection.used_header_fallback});
    }
    return data;
  }

private:
  typename rclcpp::Subscription<MessageT>::SharedPtr subscriber_;
  HeaderTimestampFunction header_timestamp_;
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__TIMESTAMPED_POLLING_SUBSCRIBER_HPP_
