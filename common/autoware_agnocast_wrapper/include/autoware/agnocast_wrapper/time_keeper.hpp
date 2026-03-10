#pragma once

#include "autoware/agnocast_wrapper/node.hpp"

#include <autoware_utils_debug/time_keeper.hpp>

#include <memory>
#include <string>

namespace autoware::agnocast_wrapper
{

/// @brief Create a TimeKeeper that uses the appropriate publisher backend
///        (agnocast or rclcpp) based on the wrapper Node's mode.
/// @param node Reference to the wrapper Node
/// @param topic_name Topic name for processing time detail
/// @param qos QoS settings for the publisher
/// @return Shared pointer to autoware_utils_debug::TimeKeeper
inline std::shared_ptr<autoware_utils_debug::TimeKeeper> create_time_keeper(
  Node & node, const std::string & topic_name, const rclcpp::QoS & qos)
{
#ifdef USE_AGNOCAST_ENABLED
  if (node.is_using_agnocast()) {
    auto pub = agnocast::create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      node.get_agnocast_node().get(), topic_name, qos);
    return std::make_shared<autoware_utils_debug::TimeKeeper>(pub);
  }
#endif
  auto pub =
    node.get_rclcpp_node()->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      topic_name, qos);
  return std::make_shared<autoware_utils_debug::TimeKeeper>(pub);
}

}  // namespace autoware::agnocast_wrapper
