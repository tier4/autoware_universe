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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_outlier_filter/cuda_polar_voxel_outlier_filter.hpp"

#include <agnocast/agnocast.hpp>
#include <agnocast/cuda/types.hpp>
// diagnostic_updater is commented out — agnocast::Node incompatible
// #include <diagnostic_updater/diagnostic_updater.hpp>
// #include <diagnostic_updater/publisher.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
class CudaPolarVoxelOutlierFilterNode : public agnocast::Node
{
  static constexpr double diagnostics_update_period_sec = 0.1;

public:
  explicit CudaPolarVoxelOutlierFilterNode(const rclcpp::NodeOptions & node_options);

protected:
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

  /** \brief main callback for pointcloud processing */
  void pointcloud_callback(
    agnocast::ipc_shared_ptr<const agnocast::cuda::PointCloud2> msg);

  // Utility functions to validate inputs
  void validate_filter_inputs(const agnocast::cuda::PointCloud2 & input_cloud);
  void validate_return_type_field(const agnocast::cuda::PointCloud2 & input_cloud);
  void validate_intensity_field(const agnocast::cuda::PointCloud2 & input_cloud);
  bool has_field(const agnocast::cuda::PointCloud2 & input, const std::string & field_name);

  // Parameter validation helpers (static, private)
  static bool validate_positive_double(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_non_negative_double(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_positive_int(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_non_negative_int(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_intensity_threshold(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_primary_return_types(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_normalized(const rclcpp::Parameter & param, std::string & reason);

private:
  CudaPolarVoxelOutlierFilterParameters filter_params_;
  std::vector<int> primary_return_types_;
  std::mutex param_mutex_;

  // Diagnostics members
  std::optional<double> visibility_;
  std::optional<double> filter_ratio_;
  // diagnostic_updater::Updater commented out — agnocast::Node incompatible
  // diagnostic_updater::Updater updater_;

  // CUDA sub
  agnocast::Subscription<agnocast::cuda::PointCloud2>::SharedPtr pointcloud_sub_;

  // CUDA pub
  agnocast::Publisher<agnocast::cuda::PointCloud2>::SharedPtr filtered_cloud_pub_;
  agnocast::Publisher<agnocast::cuda::PointCloud2>::SharedPtr noise_cloud_pub_;
  agnocast::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  agnocast::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr ratio_pub_;

  std::unique_ptr<CudaPolarVoxelOutlierFilter> cuda_polar_voxel_outlier_filter_{};
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
