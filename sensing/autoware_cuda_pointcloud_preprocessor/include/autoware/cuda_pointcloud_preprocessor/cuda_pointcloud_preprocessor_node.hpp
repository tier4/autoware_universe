// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <agnocast/agnocast.hpp>
#include <agnocast/cuda/types.hpp>
#include <agnocast/node/tf2/buffer.hpp>
#include <agnocast/node/tf2/transform_listener.hpp>
#include <autoware/point_types/types.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/transform_datatypes.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>

#define CHECK_OFFSET(structure1, structure2, field)             \
  static_assert(                                                \
    offsetof(structure1, field) == offsetof(structure2, field), \
    "Offset of " #field " in " #structure1 " does not match expected offset.")

namespace autoware::cuda_pointcloud_preprocessor
{

static_assert(sizeof(InputPointType) == sizeof(autoware::point_types::PointXYZIRCAEDT));
static_assert(sizeof(OutputPointType) == sizeof(autoware::point_types::PointXYZIRC));

CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, x);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, y);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, z);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, intensity);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, return_type);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, channel);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, azimuth);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, elevation);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, distance);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, time_stamp);

CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, x);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, y);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, z);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, intensity);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, return_type);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, channel);

class CudaPointcloudPreprocessorNode : public agnocast::Node
{
public:
  explicit CudaPointcloudPreprocessorNode(const rclcpp::NodeOptions & node_options);

private:
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    tf2::Transform * tf2_transform_ptr);

  // Callbacks
  void pointcloudCallback(
    agnocast::ipc_shared_ptr<const sensor_msgs::msg::PointCloud2> input_pointcloud_msg_ptr);
  void twistCallback(
    agnocast::ipc_shared_ptr<const geometry_msgs::msg::TwistWithCovarianceStamped> twist_msg);
  void imuCallback(agnocast::ipc_shared_ptr<const sensor_msgs::msg::Imu> imu_msg);

  // Helper Functions
  [[nodiscard]] bool validatePointcloudLayout(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg) const;
  std::pair<std::uint64_t, std::uint32_t> getFirstPointTimeInfo(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg);

  std::optional<geometry_msgs::msg::TransformStamped> lookupTransformToBase(
    const std::string & source_frame);

  void publishDiagnostics(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
    const agnocast::cuda::PointCloud2 & output_pointcloud);

  agnocast::Buffer tf2_buffer_;
  agnocast::TransformListener tf2_listener_;

  // Diagnostic
  std::unique_ptr<autoware_utils_diagnostics::BasicDiagnosticsInterface<agnocast::Node>>
    diagnostics_interface_;

  std::string base_frame_;
  bool use_3d_undistortion_;
  bool use_imu_;
  double processing_time_threshold_sec_;
  double timestamp_mismatch_fraction_threshold_;

  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  // Subscribers
  agnocast::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  agnocast::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  agnocast::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  // CUDA publisher
  agnocast::Publisher<agnocast::cuda::PointCloud2>::SharedPtr pub_;

  std::unique_ptr<CudaPointcloudPreprocessor> cuda_pointcloud_preprocessor_;

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils_debug::BasicDebugPublisher<agnocast::Node>> debug_publisher_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_NODE_HPP_
