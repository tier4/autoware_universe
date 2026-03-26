// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_

#include <agnocast/agnocast.hpp>
#include <rclcpp/publisher.hpp>

namespace autoware::component_interface_utils
{

/// The wrapper class of agnocast::Publisher for Agnocast pub/sub migration.
template <class SpecT>
class Publisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Publisher)
  using SpecType = SpecT;
  using Message = typename SpecT::Message;
  using WrapType = agnocast::Publisher<Message>;

  /// Constructor.
  explicit Publisher(typename WrapType::SharedPtr publisher)
  {
    publisher_ = publisher;  // to keep the reference count
  }

  /// Publish a message using the loaned message pattern.
  void publish(const Message & msg)
  {
    auto loaned_msg = publisher_->borrow_loaned_message();
    *loaned_msg = msg;
    publisher_->publish(std::move(loaned_msg));
  }

private:
  RCLCPP_DISABLE_COPY(Publisher)
  typename WrapType::SharedPtr publisher_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_
