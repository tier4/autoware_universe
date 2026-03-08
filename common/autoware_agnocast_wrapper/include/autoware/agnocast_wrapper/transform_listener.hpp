#pragma once

#include "autoware/agnocast_wrapper/node.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils_tf/transform_listener.hpp>

#include <memory>
#include <string>

namespace autoware::agnocast_wrapper
{

/// @brief Create a TransformListener that uses the appropriate backend
///        (rclcpp or agnocast) based on the wrapper Node's mode.
/// @param node Pointer to the wrapper Node
/// @return Shared pointer to autoware_utils_tf::TransformListener
inline std::shared_ptr<autoware_utils_tf::TransformListener> make_transform_listener(Node * node)
{
  if (node->is_using_agnocast()) {
    return std::make_shared<autoware_utils_tf::TransformListener>(node->get_agnocast_node().get());
  } else {
    return std::make_shared<autoware_utils_tf::TransformListener>(node->get_rclcpp_node().get());
  }
}

/// @brief Call autoware::object_recognition_utils::transformObjects using the
///        appropriate buffer from the TransformListener.
template <class T>
bool transformObjects(
  const T & input_msg, const std::string & target_frame_id,
  autoware_utils_tf::TransformListener & tf_listener, T & output_msg)
{
  if (auto * buf = tf_listener.get_tf2_buffer()) {
    return autoware::object_recognition_utils::transformObjects(
      input_msg, target_frame_id, *buf, output_msg);
  }
  if (auto * buf = tf_listener.get_agnocast_buffer()) {
    return autoware::object_recognition_utils::transformObjects(
      input_msg, target_frame_id, *buf, output_msg);
  }
  return false;
}

}  // namespace autoware::agnocast_wrapper
