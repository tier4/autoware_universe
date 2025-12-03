// Copyright 2025 TIER IV, Inc.
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

#include "autoware/cuda_pointcloud_preprocessor/cuda_filter/cuda_crop_box_filter_node.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_filter/cuda_crop_box_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

namespace autoware::cuda_pointcloud_preprocessor
{
CudaCropBoxFilterNode::CudaCropBoxFilterNode(const rclcpp::NodeOptions & node_options)
: Node("cuda_crop_box_filter", node_options)
{
  // Set initial parameters
  const auto min_x_vector = declare_parameter<std::vector<double>>("min_x");
  const auto min_y_vector = declare_parameter<std::vector<double>>("min_y");
  const auto min_z_vector = declare_parameter<std::vector<double>>("min_z");
  const auto max_x_vector = declare_parameter<std::vector<double>>("max_x");
  const auto max_y_vector = declare_parameter<std::vector<double>>("max_y");
  const auto max_z_vector = declare_parameter<std::vector<double>>("max_z");
  const auto negative_vector = declare_parameter<std::vector<bool>>("negative", std::vector<bool>());
  int64_t max_mem_pool_size_in_byte =
    declare_parameter<int64_t>("max_mem_pool_size_in_byte", 1e9);

  if (max_mem_pool_size_in_byte < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid pool size was specified. The value should be positive");
    return;
  }

  // Validate parameter sizes
  const size_t num_boxes = min_x_vector.size();
  if (
    num_boxes != min_y_vector.size() || num_boxes != min_z_vector.size() ||
    num_boxes != max_x_vector.size() || num_boxes != max_y_vector.size() ||
    num_boxes != max_z_vector.size()) {
    throw std::runtime_error("Crop box parameters must have the same size");
  }

  // Handle negative parameter: if not provided, default to false for all boxes
  std::vector<bool> negative_values;
  if (negative_vector.empty()) {
    negative_values.resize(num_boxes, false);
  } else if (negative_vector.size() != num_boxes) {
    throw std::runtime_error(
      "negative parameter size must match the number of crop boxes or be empty");
  } else {
    negative_values = negative_vector;
  }

  // Build crop box parameters
  std::vector<CropBoxParameters> crop_box_parameters;
  for (size_t i = 0; i < num_boxes; i++) {
    CropBoxParameters params;
    params.min_x = static_cast<float>(min_x_vector[i]);
    params.min_y = static_cast<float>(min_y_vector[i]);
    params.min_z = static_cast<float>(min_z_vector[i]);
    params.max_x = static_cast<float>(max_x_vector[i]);
    params.max_y = static_cast<float>(max_y_vector[i]);
    params.max_z = static_cast<float>(max_z_vector[i]);
    params.negative = negative_values[i] ? 1 : 0;
    crop_box_parameters.push_back(params);
  }

  sub_ = std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
    *this, "~/input/pointcloud",
    std::bind(
      &CudaCropBoxFilterNode::cudaPointcloudCallback, this, std::placeholders::_1));

  pub_ = std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
    *this, "~/output/pointcloud");

  cuda_crop_box_filter_ = std::make_unique<CudaCropBoxFilter>(
    crop_box_parameters, max_mem_pool_size_in_byte);
}

void CudaCropBoxFilterNode::cudaPointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // The following only checks compatibility with xyzi
  if (!pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzi(msg->fields)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Input pointcloud data layout is not compatible with PointXYZI. "
      "The output result may not be correct");
  }

  auto output_pointcloud_ptr = cuda_crop_box_filter_->filter(msg);
  pub_->publish(std::move(output_pointcloud_ptr));
}
}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaCropBoxFilterNode)

